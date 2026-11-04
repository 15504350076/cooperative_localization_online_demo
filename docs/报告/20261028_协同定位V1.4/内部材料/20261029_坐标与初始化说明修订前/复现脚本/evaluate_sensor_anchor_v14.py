#!/usr/bin/env python3
"""V1.4平面传感器/锚点对照。输出级VIO/LIO模型，不运行图像/点云前端。

各条件共享轨迹、起点测量样本和同编号测距噪声；所有评价在固定场地坐标中
计算，纯车间UWB另用规范相对队形坐标。不会修改已有算法库和录包。
"""
import argparse
import csv
import hashlib
import json
import subprocess
import tempfile
from pathlib import Path

import numpy as np

from evaluate_rte import EDGES, HEADINGS, HEADER, ROOT, summarize
from evaluate_rte_expanded import motion, world_position
from evaluate_uwb_formation_sim import library, solve

OUT = ROOT / 'data/sensor_anchor_v14_20261028'
SIGMAS = (0.02, 0.05, 0.10)
ANCHORS = np.array(((0., 0.), (110., 0.), (0., 130.), (110., 130.)))
SHIFT = np.array((15., 65.))
PROFILES = {
    'imu': dict(name='IMU', accel_white_mps2=0.02, accel_bias_std_mps2=0.001,
                gyro_white_rps=0.0005, gyro_bias_std_rps=0.000015),
    'vio': dict(name='VIO输出模型', scale_std=0.002, yaw_drift_std_rps=0.00003,
                translation_noise_m_sqrt_m=0.020),
    'lio': dict(name='LIO输出模型', scale_std=0.0005, yaw_drift_std_rps=0.000008,
                translation_noise_m_sqrt_m=0.005),
}


def rotate(a, theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.stack((c*a[..., 0]-s*a[..., 1], s*a[..., 0]+c*a[..., 1]), axis=-1)


def tangent_heading(velocity):
    heading=np.arctan2(velocity[:,:,1],velocity[:,:,0])
    for n in range(velocity.shape[1]):
        moving=np.flatnonzero(np.linalg.norm(velocity[:,n],axis=-1)>1e-7)
        if len(moving):heading[:moving[0],n]=heading[moving[0],n]
    return np.unwrap(heading,axis=0)


def scene():
    t = np.arange(15001)*0.02
    p = np.array([[world_position(n, x)+SHIFT for n in range(3)] for x in t])
    v = np.array([[rotate(motion(n,x)[1],HEADINGS[n]) for n in range(3)] for x in t])
    yaw=tangent_heading(v)
    lateral=-v[:,:,0]*np.sin(yaw)+v[:,:,1]*np.cos(yaw)
    assert np.max(np.abs(lateral))<1e-7
    return t, p, yaw, v


def inputs(t, truth, yaw, velocity, trials, *, position_std_m=0.05, heading_std_deg=0.2, anchor_survey_std_m=0.02):
    """模拟器可使用真值生成传感器；估计器只接收本函数的测量输出。"""
    k, nt = trials, len(t)
    rng = np.random.default_rng(14001)
    initial = truth[0]+rng.normal(0,position_std_m,(k,3,2))
    angle_error = rng.normal(0,np.deg2rad(heading_std_deg),(k,3))
    surveyed = ANCHORS+rng.normal(0,anchor_survey_std_m,(k,4,2))
    d = np.diff(truth,axis=0,prepend=truth[:1]); distance=np.linalg.norm(d,axis=-1)
    data = {}
    for profile in ('vio','lio'):
        cfg=PROFILES[profile]; r=np.random.default_rng(14002)
        scale=r.normal(0,cfg['scale_std'],(k,1,3,1))
        rate=r.normal(0,cfg['yaw_drift_std_rps'],(k,1,3))
        drift=rate*np.maximum(t-30,0)[None,:,None]
        noisy=rotate(d[None,:,:,:],drift)*(1+scale)
        noisy+=r.normal(size=(k,nt,3,2))*cfg['translation_noise_m_sqrt_m']*np.sqrt(distance)[None,:,:,None]
        noisy[:,0]=0
        # local_delta是各车自身起点坐标中的传感器输出，不包含起点测量误差。
        local_delta=rotate(noisy,-HEADINGS[None,None,:])
        local=np.cumsum(local_delta,axis=1)
        local_yaw=yaw[None,:,:]-HEADINGS[None,None,:]+drift
        common_delta=rotate(local_delta,HEADINGS[None,None,:]+angle_error[:,None,:])
        data[profile]=dict(delta=common_delta.reshape(k,nt,6),local=local,local_yaw=local_yaw)
    r=np.random.default_rng(14003)
    # 100Hz白噪声按10Hz时间段平均，残余零偏模拟静止初始化后的误差。
    coarse_t=t[::5];dt=0.1
    acceleration=np.diff(velocity[::5],axis=0,prepend=velocity[:1])/dt
    body=rotate(acceleration,-yaw[::5])
    accel_bias=r.normal(0,PROFILES['imu']['accel_bias_std_mps2'],(k,1,3,2))
    gyro_bias=r.normal(0,PROFILES['imu']['gyro_bias_std_rps'],(k,1,3))
    gyro_noise=r.normal(0,PROFILES['imu']['gyro_white_rps']/np.sqrt(10),(k,len(coarse_t),3))
    heading_error=np.cumsum(gyro_noise*dt,axis=1)+gyro_bias*np.maximum(coarse_t-30,0)[None,:,None]
    measured=body[None]+accel_bias+r.normal(0,PROFILES['imu']['accel_white_mps2']/np.sqrt(10),(k,len(coarse_t),3,2))
    acc_world=rotate(measured,yaw[None,::5,:]+angle_error[:,None,:]+heading_error)
    acc_world[:,coarse_t<=30]=0  # 已知静止启动阶段不传播速度，运动后不再施加零速约束。
    data['imu']=dict(acceleration=acc_world.reshape(k,len(coarse_t),6))
    r=np.random.default_rng(14004)
    peer_true=np.stack([np.linalg.norm(truth[::5,i]-truth[::5,j],axis=-1) for i,j in EDGES],axis=-1)
    anchor_true=np.linalg.norm(truth[::5,:,None,:]-ANCHORS[None,None,:,:],axis=-1)
    data['peer_standard']=r.normal(size=(k,len(coarse_t),3))
    data['anchor_standard']=r.normal(size=(k,len(coarse_t),3,4))
    return data,initial,angle_error,surveyed,peer_true,anchor_true


def update(x, covariance, h, innovation, variance):
    ph=np.einsum('kij,kj->ki',covariance,h)
    s=np.einsum('ki,ki->k',h,ph)+variance
    gain=ph/s[:,None]
    x+=gain*innovation[:,None]
    covariance-=np.einsum('ki,kj->kij',gain,ph)


def fusion(profile, anchors, peer, sigma, data, initial, surveyed, peer_ranges, anchor_ranges, times, *, anchor_survey_std_m=0.02, anchor_rate_hz=10):
    trials=len(initial);imu=profile=='imu';size=12 if imu else 6
    x=np.zeros((trials,size));x[:,:6]=initial.reshape(trials,6)
    covariance=np.broadcast_to(np.eye(size)[None]*0.05**2,(trials,size,size)).copy()
    if imu: covariance[:,6:,6:]=np.eye(6)[None]*0.005**2
    result=np.empty((trials,len(times),3,2));correction=np.zeros((trials,len(times)))
    # 稀疏测距更新不使用真值，只使用相应传感器增量/加速度和有误差的锚点测量。
    for step,t in enumerate(times):
        if step:
            if imu:
                a=data['imu']['acceleration'][:,step]
                x[:,:6]+=x[:,6:]*0.1+0.5*a*0.1**2;x[:,6:]+=a*0.1
                pp=covariance[:,:6,:6];pv=covariance[:,:6,6:];vp=covariance[:,6:,:6];vv=covariance[:,6:,6:]
                pp+=0.1*(pv+vp)+0.01*vv;pv+=0.1*vv;vp+=0.1*vv
                q=0.02**2
                pp+=np.eye(6)[None]*q*0.1**4/4
                pv+=np.eye(6)[None]*q*0.1**3/2;vp+=np.eye(6)[None]*q*0.1**3/2
                vv+=np.eye(6)[None]*q*0.1**2
            else:
                delta=data[profile]['delta'][:,step*5-4:step*5+1].sum(axis=1)
                x+=delta
                cfg=PROFILES[profile]
                process=cfg['translation_noise_m_sqrt_m']**2*np.repeat(np.linalg.norm(delta.reshape(trials,3,2),axis=-1),2,axis=-1)
                process+=1e-7
                covariance[:,np.arange(6),np.arange(6)]+=process
        predicted=x[:,:6].copy()
        if peer:
            for edge,(i,j) in enumerate(EDGES):
                d=x[:,2*i:2*i+2]-x[:,2*j:2*j+2];dist=np.maximum(np.linalg.norm(d,axis=-1),1e-8)
                h=np.zeros((trials,size));h[:,2*i:2*i+2]=d/dist[:,None];h[:,2*j:2*j+2]=-d/dist[:,None]
                update(x,covariance,h,peer_ranges[:,step,edge]-dist,sigma**2)
        for anchor in range(anchors if step%round(10/anchor_rate_hz)==0 else 0):
            for i in range(3):
                d=x[:,2*i:2*i+2]-surveyed[:,anchor];dist=np.maximum(np.linalg.norm(d,axis=-1),1e-8)
                h=np.zeros((trials,size));h[:,2*i:2*i+2]=d/dist[:,None]
                update(x,covariance,h,anchor_ranges[:,step,i,anchor]-dist,sigma**2+anchor_survey_std_m**2)
        result[:,step]=x[:,:6].reshape(trials,3,2)
        correction[:,step]=np.linalg.norm(x[:,:6]-predicted,axis=-1)
    assert np.isfinite(result).all()
    assert np.linalg.eigvalsh(covariance).min()>-1e-7,'滤波协方差失去半正定性'
    return result,correction


def anchor_only(count, surveyed, ranges, initial):
    # 每时刻独立多边定位，不使用里程计；前一帧仅作为数值迭代初值。
    k,nt,_,_=ranges.shape;p=initial.copy();out=np.empty((k,nt,3,2))
    for step in range(nt):
        for _ in range(5):
            delta=p[:,:,None,:]-surveyed[:,None,:count,:]
            dist=np.maximum(np.linalg.norm(delta,axis=-1),1e-8);j=delta/dist[...,None]
            h=np.einsum('knai,knaj->knij',j,j)+np.eye(2)*1e-10
            g=np.einsum('knai,kna->kni',j,dist-ranges[:,step,:,:count])
            p-=np.linalg.solve(h,g[...,None])[...,0]
        out[:,step]=p
    return out


def metrics(estimate,truth):
    relative=estimate[:,:,1:]-estimate[:,:,:1]
    target=truth[None,:,1:]-truth[None,:,:1]
    error=np.linalg.norm(relative-target,axis=-1)
    absolute=np.linalg.norm(estimate-truth[None],axis=-1)
    return [dict(position_rmse_m=float(np.sqrt(np.mean(e**2))),position_p95_m=float(np.percentile(e,95)),
                 position_max_m=float(e.max()),field_position_rmse_m=float(np.sqrt(np.mean(a**2)))) for e,a in zip(error,absolute)]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=OUT);parser.add_argument('--trials',type=int,default=20)
    parser.add_argument('--rte-only',action='store_true');parser.add_argument('--skip-rte',action='store_true')
    parser.add_argument('--runner',type=Path,default=ROOT/'build_rte_verify/evaluate_rte')
    args=parser.parse_args();out=args.output.resolve();out.mkdir(parents=True,exist_ok=True)
    assert 1<=args.trials<=100
    t,p,yaw,velocity=scene();data,initial,angle,surveyed,peer_true,anchor_true=inputs(t,p,yaw,velocity,args.trials)
    coarse=t[::5];truth=p[::5]
    np.savez_compressed(out/'sensor_inputs.npz',time_s=t,truth=p,yaw=yaw,velocity=velocity,initial=initial,
        surveyed_anchors=surveyed,initial_heading_error_rad=angle,peer_standard=data['peer_standard'],anchor_standard=data['anchor_standard'],
        vio_local=data['vio']['local'],vio_yaw=data['vio']['local_yaw'],lio_local=data['lio']['local'],lio_yaw=data['lio']['local_yaw'],imu_acceleration=data['imu']['acceleration'])
    summary={'date':'2026-10-28','duration_s':300,'motion_start_s':30,'trials_per_condition':args.trials,
        'scope':'平面运动传感器输出级仿真；VIO/LIO为位姿误差模型，非图像/点云前端；原程序未修改',
        'sensor_profiles':PROFILES,'initial_position_std_m':0.05,'initial_heading_std_deg':0.2,'anchor_survey_std_m':0.02,
        'anchors_m':ANCHORS.tolist(),'anchor_sampling_hz':10,'position_frame':'固定场地测量标记O为原点，场地方向标记为正X，Z向上',
        'relative_metric':'车2/3相对车1位置误差合并RMSE；共同场地轴，不做事后轨迹拟合',
        'scene':{'x_extent_m':float(np.ptp(p[:,:,0])),'y_extent_m':float(np.ptp(p[:,:,1])),
                 'maximum_speed_mps':float(np.linalg.norm(velocity,axis=-1).max()),
                 'heading_model':'车体前向沿速度切向；静止阶段保持起步方向',
                 'maximum_yaw_rate_rps':float(np.max(np.abs(np.gradient(yaw,t,axis=0)))),
                 'maximum_body_lateral_speed_mps':float(np.max(np.abs(-velocity[:,:,0]*np.sin(yaw)+velocity[:,:,1]*np.cos(yaw)))),
                 'maximum_anchor_range_m':float(anchor_true.max())},'conditions':[],'rte':[]}
    def record(name,profile,count,peer,sigma,estimate,correction=None,frame='field'):
        result=metrics(estimate,truth) if frame=='field' else None
        case=dict(name=name,profile=profile,anchors=count,peer_ranges=peer,range_sigma_m=sigma,frame=frame,trials=result)
        summary['conditions'].append(case)
        arrays=dict(time_s=coarse,position=estimate[0],truth=truth)
        if correction is not None:arrays['correction_m']=correction[0]
        np.savez_compressed(out/(name+'.npz'),**arrays)
        (out/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
        values=[x['position_rmse_m'] for x in result]
        print(f'[完成] {name}: 相对位置RMSE {min(values):.3f}～{max(values):.3f}米',flush=True)
    if not args.rte_only:
        for profile in PROFILES:
            est,c=fusion(profile,0,False,.05,data,initial,surveyed,None,None,coarse)
            record(f'{profile}_initial_only',profile,0,False,None,est,c)
        for sigma in SIGMAS:
            peers=peer_true[None]+sigma*data['peer_standard'];ars=anchor_true[None]+sigma*data['anchor_standard']
            for profile in PROFILES:
                for count in (0,1,3,4):
                    est,c=fusion(profile,count,True,sigma,data,initial,surveyed,peers,ars,coarse)
                    record(f'{profile}_peer_a{count}_s{sigma:.2f}',profile,count,True,sigma,est,c)
            for count in (3,4):
                est=anchor_only(count,surveyed,ars,initial)
                record(f'uwb_anchors_a{count}_s{sigma:.2f}','uwb_anchors',count,False,sigma,est)
            lib=library(ROOT/'build_rte_verify/libzju_coop.so');errors=[];example=[]
            for k in range(args.trials):
                trial=[]
                for step,distances in enumerate(peers[k]):
                    f=solve(lib,distances);assert f.disposition==0
                    q2=truth[step,1]-truth[step,0];q3=truth[step,2]-truth[step,0];norm=np.linalg.norm(q2)
                    gt=np.array(((0.,0.),(norm,0.),(np.dot(q2,q3)/norm,abs(np.cross(q2,q3))/norm)))
                    xy=np.stack((f.x_m[:],f.y_m[:]),axis=-1)
                    trial.extend(np.linalg.norm(xy[1:]-gt[1:],axis=-1))
                    if k==0:example.append((xy,gt))
                e=np.asarray(trial);errors.append(dict(position_rmse_m=float(np.sqrt(np.mean(e**2))),position_p95_m=float(np.percentile(e,95)),position_max_m=float(e.max())))
            summary['conditions'].append(dict(name=f'uwb_triangle_s{sigma:.2f}',profile='uwb_triangle',anchors=0,peer_ranges=True,range_sigma_m=sigma,frame='triangle',trials=errors))
            np.savez_compressed(out/f'uwb_triangle_s{sigma:.2f}.npz',time_s=coarse,position=np.array([r[0] for r in example]),truth=np.array([r[1] for r in example]))
            (out/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    elif (out/'summary.json').exists():
        summary['conditions']=json.loads((out/'summary.json').read_text())['conditions']
    if not args.skip_rte:
        target=out/'rte';target.mkdir(exist_ok=True)
        for profile in ('vio','lio'):
            for sigma in SIGMAS:
                ranges=peer_true[None]+sigma*data['peer_standard']
                for k in range(args.trials):
                    name=f'rte_{profile}_s{sigma:.2f}_trial{k+1:02d}';output=target/(name+'_output.csv')
                    with tempfile.TemporaryDirectory(prefix='v14_rte_') as work:
                        source=Path(work)/'input.csv';tr=Path(work)/'truth.csv'
                        with source.open('w',newline='') as stream:
                            w=csv.writer(stream);w.writerow(HEADER)
                            for step,seconds in enumerate(t):
                                stamp=10_000_000_000+step*20_000_000
                                for n in range(3):
                                    pos=data[profile]['local'][k,step,n];last=max(0,step-1)
                                    vel=(pos-data[profile]['local'][k,last,n])/.02
                                    angle=data[profile]['local_yaw'][k,step,n]
                                    w.writerow(('O',stamp,n+1,0,*pos,0.,*vel,0.,0.,0.,np.sin(angle/2),np.cos(angle/2)))
                                if step%5==0:
                                    for e,(i,j) in enumerate(EDGES):w.writerow(('R',stamp,i+1,j+1,ranges[k,step//5,e],sigma,*([0.]*8)))
                        relative=p-p[:,:1]
                        np.savetxt(tr,np.column_stack((10_000_000_000+np.arange(len(t))*20_000_000,
                            np.stack((relative[:,:,0],relative[:,:,1],yaw),axis=-1).reshape(len(t),9))),delimiter=',',comments='',
                            header='timestamp_ns,'+','.join(f'{a}_{n}' for n in (1,2,3) for a in ('x','y','yaw')),fmt='%.17g')
                        subprocess.run([str(args.runner.resolve()),str(source),str(output)],check=True)
                        r=summarize(output,tr,duration_s=300,motion_start_s=30)
                        r.update(name=name,profile=profile,range_sigma_m=sigma,trial=k+1,input_sha256=hashlib.sha256(source.read_bytes()).hexdigest())
                    summary['rte'].append(r)
                    (out/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
                    print(f'[RTE完成] {name}: RMSE={r["position_rmse_m"]}',flush=True)
    summary['condition_count']=len(summary['conditions'])+len({(r['profile'],r['range_sigma_m']) for r in summary['rte']})
    summary['trial_count']=sum(len(c['trials']) for c in summary['conditions'])+len(summary['rte'])
    (out/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    print(f'[全部完成] {summary["condition_count"]}种条件，{summary["trial_count"]}次仿真',flush=True)


if __name__=='__main__':main()
