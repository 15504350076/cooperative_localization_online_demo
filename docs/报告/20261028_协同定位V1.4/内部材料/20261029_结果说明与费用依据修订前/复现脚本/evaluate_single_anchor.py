#!/usr/bin/env python3
"""三车、一个已知位置UWB点；从传感器测量求解，不用真值初始化或配准轨迹。

VIO/LIO为局部位移输出模型。初始化用六条测距和局部运动最小二乘，
在线使用独立EKF。生成器、估计器和事后误差计算分别保存输入输出。
"""
import argparse
import csv
import hashlib
import json
import platform
import subprocess
import time
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares

from evaluate_rte import ROOT, EDGES
from evaluate_sensor_anchor_v14 import scene, rotate
from evaluate_uwb_formation_sim import library, solve

OUT = ROOT / 'data/single_anchor_outdoor_20261029'
INIT_SECONDS = 60.0
DT = .1
SIGMAS = (.20, .10, .05)
BASE_SIGMA = .10
RANGE_BIAS_STD_M = .03
MODES = ('start', 'peer', 'single')
PROFILES = {}
for grade,bias,gyro_bias,white,acc_walk,gyro_walk,tilt in [
        ('基础',.03,.00015,.05,.001,.0001,.2),
        ('增强',.01,.00005,.02,.0003,.000035640318696367613,.1),
        ('精细',.003,.000015,.01,.0001,.00001,.05)]:
    PROFILES[f'imu_{grade}'] = dict(kind='imu',grade=grade,accel_bias=bias,gyro_bias=gyro_bias,
        accel_white=white,gyro_white=.0005,accel_bias_walk=acc_walk,gyro_bias_walk=gyro_walk,tilt_std_deg=tilt,
        provenance='残余偏置为校准后敏感性假设；随机游走模型参考Kalibr，增强档陀螺游走参考LIO-SAM；非器件等级')
for kind, rows in {
    'vio': [('基础', .010, .00010, .050), ('增强', .005, .00005, .030), ('精细', .002, .00002, .015)],
    'lio': [('基础', .005, .00005, .030), ('增强', .002, .00002, .015), ('精细', .001, .00001, .008)],
}.items():
    for grade, scale, rate, noise in rows:
        PROFILES[f'{kind}_{grade}'] = dict(kind=kind,grade=grade,scale_std=scale,yaw_drift_std=rate,
            displacement_noise=noise,provenance='明确的输出级敏感性设定；不等于VINS-Fusion或FAST-LIO实测参数')


def wrap(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))


def make_measurements(times, truth, yaw, velocity, profile, trials):
    """只在模拟传感器的这一步读取真值；后续估计函数不接收真值。"""
    cfg=PROFILES[profile]; n=len(times); rng=np.random.default_rng(291001)
    yaw0=yaw[0]; delta=np.diff(truth,axis=0,prepend=truth[:1])
    if cfg['kind']=='imu':
        rate=np.diff(yaw,axis=0,prepend=yaw[:1])/DT
        gyro_bias=rng.normal(0,cfg['gyro_bias'],(trials,1,3))+np.cumsum(rng.normal(0,cfg['gyro_bias_walk']*np.sqrt(DT),(trials,n,3)),axis=1)
        measured_gyro=rate[None]+gyro_bias+rng.normal(0,cfg['gyro_white']/np.sqrt(10),(trials,n,3))
        local_yaw=np.cumsum(measured_gyro*DT,axis=1)
        acc=np.diff(velocity,axis=0,prepend=velocity[:1])/DT
        accel_bias=rng.normal(0,cfg['accel_bias'],(trials,1,3,2))+np.cumsum(rng.normal(0,cfg['accel_bias_walk']*np.sqrt(DT),(trials,n,3,2)),axis=1)
        tilt=rng.normal(0,np.deg2rad(cfg['tilt_std_deg']),(trials,1,3,2))
        gravity_leak=9.80665*tilt
        measured_acc=rotate(acc,-yaw)[None]+accel_bias+gravity_leak+rng.normal(0,cfg['accel_white']/np.sqrt(10),(trials,n,3,2))
        acceleration=rotate(measured_acc,local_yaw)
        acceleration[:,times<=30]=0  # 起步前已知静止，运动后不添加零速约束。
        v=np.zeros((trials,3,2)); local=np.zeros((trials,n,3,2))
        for step in range(1,n):
            local[:,step]=local[:,step-1]+v*DT+.5*acceleration[:,step]*DT**2
            v+=acceleration[:,step]*DT
        return dict(local=local,acceleration=acceleration,gyro=measured_gyro,accel_body=measured_acc,
                    simulated_gyro_bias=gyro_bias,simulated_accel_bias=accel_bias,gravity_leak_mps2=gravity_leak)
    distance=np.linalg.norm(delta,axis=-1)
    scale=rng.normal(0,cfg['scale_std'],(trials,1,3,1))
    drift=rng.normal(0,cfg['yaw_drift_std'],(trials,1,3))*np.maximum(times-30,0)[None,:,None]
    increments=rotate(delta[None],drift-yaw0[None,None])*(1+scale)
    increments+=rng.normal(size=(trials,n,3,2))*cfg['displacement_noise']*np.sqrt(distance)[None,:,:,None]
    increments[:,0]=0
    return dict(local=np.cumsum(increments,axis=1), increments=increments)


def initial_shape(ranges, sign):
    """从锚点到三车和三条车间距离恢复一个启动形状；两种镜像均尝试。"""
    r1,r2,r3=ranges[3:]; d12,d13,d23=ranges[:3]
    r1=max(r1,.1);x2=(r1*r1+r2*r2-d12*d12)/(2*r1)
    y2=sign*np.sqrt(max(r2*r2-x2*x2,.01))
    x3=(r1*r1+r3*r3-d13*d13)/(2*r1)
    y3=np.sqrt(max(r3*r3-x3*x3,.01))
    if abs(np.hypot(x3-x2,-y3-y2)-d23)<abs(np.hypot(x3-x2,y3-y2)-d23):y3=-y3
    return np.array(((r1,0.),(x2,y2),(x3,y3)))


def initialize(local, ranges, times, sigma):
    """8个未知数：初始三车位置5自由度及3个局部坐标转角。

    唯一坐标原点是锚点，初始参考车位于正X轴。只以测距残差选解，
    不读取真值、实际初始航向，不对输出作事后平移/旋转拟合。
    """
    sample=np.flatnonzero(times<=INIT_SECONDS)[::10]
    loc=local[sample]; measured=ranges[sample]; selection=[0,2,3,4,5]
    def evaluate(z, jacobian=False):
        p0=np.array(((z[0],0.),(z[1],z[2]),(z[3],z[4])))
        moved=rotate(loc,z[None,5:]); p=p0[None]+moved
        # 每车对自身位置参数、局部坐标转角的导数。
        jp=np.zeros((len(sample),3,2,8))
        for n in range(3):
            for axis in range(2):
                if 2*n+axis in selection:jp[:,n,axis,selection.index(2*n+axis)]=1
            jp[:,n,0,5+n]=-moved[:,n,1];jp[:,n,1,5+n]=moved[:,n,0]
        residual=[];jac=[]
        for edge,(i,j) in enumerate(EDGES):
            d=p[:,i]-p[:,j];dist=np.maximum(np.linalg.norm(d,axis=-1),1e-8)
            residual.append((dist-measured[:,edge])/sigma)
            if jacobian:jac.append(np.einsum('ti,tij->tj',d/dist[:,None],jp[:,i]-jp[:,j])/sigma)
        for i in range(3):
            dist=np.maximum(np.linalg.norm(p[:,i],axis=-1),1e-8)
            residual.append((dist-measured[:,3+i])/sigma)
            if jacobian:jac.append(np.einsum('ti,tij->tj',p[:,i]/dist[:,None],jp[:,i])/sigma)
        return np.concatenate(jac,axis=0) if jacobian else np.concatenate(residual)
    candidates=[]
    for sign in (-1,1):
        pos=initial_shape(np.mean(ranges[times<30],axis=0),sign).ravel()[selection]
        for angle in (-np.pi/2,0.,np.pi/2):
            x=np.r_[pos,[angle]*3]
            result=least_squares(evaluate,x,jac=lambda z:evaluate(z,True),bounds=(np.r_[.01,[-np.inf]*7],np.full(8,np.inf)),max_nfev=100,ftol=1e-8,xtol=1e-8,gtol=1e-8)
            candidates.append(result)
    result=min(candidates,key=lambda r:float(r.fun@r.fun));z=result.x
    p0=np.array(((z[0],0.),(z[1],z[2]),(z[3],z[4])))
    theta=wrap(z[5:]);pos=p0+rotate(local[int(round(INIT_SECONDS/DT))],theta)
    singular=np.linalg.svd(result.jac,compute_uv=False)
    rank=int(np.count_nonzero(singular>singular[0]*max(result.jac.shape)*np.finfo(float).eps))
    return pos,theta,dict(range_fit_rmse_m=float(np.sqrt(np.mean(result.fun**2))*sigma),iterations=int(result.nfev),
        selected_by='测距残差最小，无真值参与',jacobian_rank=rank,unknown_count=8,
        singular_values=singular.tolist(),optimizer_success=bool(result.success))


def scalar_update(x,P,h,innovation,variance):
    ph=np.einsum('kij,kj->ki',P,h);s=np.einsum('ki,ki->k',h,ph)+variance
    gain=ph/s[:,None];x+=gain*innovation[:,None];P-=np.einsum('ki,kj->kij',gain,ph)


def estimate(sensor, measured_ranges, cfg, initial_position, initial_angle, times, sigma, mode):
    """测量驱动的在线滤波；参数中不含任何真值。"""
    trials=len(initial_position);imu=cfg['kind']=='imu';size=15 if imu else 9;ai=12 if imu else 6
    start=int(round(INIT_SECONDS/DT));x=np.zeros((trials,size));x[:,:6]=initial_position.reshape(trials,6);x[:,ai:ai+3]=initial_angle
    P=np.broadcast_to(np.eye(size)[None],(trials,size,size)).copy()
    P[:,:6,:6]*=.25**2;P[:,ai:,ai:]*=np.deg2rad(3)**2
    if imu:
        local_velocity=(sensor['local'][:,start]-sensor['local'][:,start-1])/DT+.5*sensor['acceleration'][:,start]*DT
        x[:,6:12]=rotate(local_velocity,initial_angle).reshape(trials,6);P[:,6:12,6:12]*=.05**2
    positions=np.full((trials,len(times),3,2),np.nan);angles=np.full((trials,len(times),3),np.nan);corrections=np.zeros((trials,len(times)))
    positions[:,start]=x[:,:6].reshape(trials,3,2);angles[:,start]=x[:,ai:ai+3]
    identity=np.eye(size)
    for step in range(start+1,len(times)):
        theta=x[:,ai:ai+3];F=np.broadcast_to(identity,(trials,size,size)).copy()
        if imu:
            acceleration=rotate(sensor['acceleration'][:,step],theta);perp=np.stack((-acceleration[:,:,1],acceleration[:,:,0]),axis=-1)
            x[:,:6]+=x[:,6:12]*DT+.5*acceleration.reshape(trials,6)*DT**2;x[:,6:12]+=acceleration.reshape(trials,6)*DT
            F[:,:6,6:12]=np.eye(6)[None]*DT
            for n in range(3):F[:,2*n:2*n+2,ai+n]=.5*perp[:,n]*DT**2;F[:,6+2*n:8+2*n,ai+n]=perp[:,n]*DT
            q=cfg['accel_white']**2/10+cfg['accel_bias']**2
        else:
            delta=rotate(sensor['increments'][:,step],theta);x[:,:6]+=delta.reshape(trials,6)
            for n in range(3):F[:,2*n,ai+n]=-delta[:,n,1];F[:,2*n+1,ai+n]=delta[:,n,0]
        P=F@P@np.transpose(F,(0,2,1))
        if imu:
            P[:,:6,:6]+=np.eye(6)[None]*q*DT**4/4;P[:,6:12,6:12]+=np.eye(6)[None]*q*DT**2
            P[:,:6,6:12]+=np.eye(6)[None]*q*DT**3/2;P[:,6:12,:6]+=np.eye(6)[None]*q*DT**3/2
            angle_noise=cfg['gyro_bias']**2*DT
        else:
            process=cfg['displacement_noise']**2*np.repeat(np.linalg.norm(delta,axis=-1),2,axis=-1)+(cfg['scale_std']*np.linalg.norm(delta,axis=-1).max(axis=-1))[:,None]**2
            P[:,np.arange(6),np.arange(6)]+=process+1e-9
            angle_noise=cfg['yaw_drift_std']**2*DT
        P[:,np.arange(ai,ai+3),np.arange(ai,ai+3)]+=angle_noise
        predicted=x[:,:6].copy()
        if mode in ('peer','single'):
            for edge,(i,j) in enumerate(EDGES):
                d=x[:,2*i:2*i+2]-x[:,2*j:2*j+2];dist=np.maximum(np.linalg.norm(d,axis=-1),1e-8)
                h=np.zeros((trials,size));h[:,2*i:2*i+2]=d/dist[:,None];h[:,2*j:2*j+2]=-d/dist[:,None]
                scalar_update(x,P,h,measured_ranges[:,step,edge]-dist,sigma*sigma)
        if mode=='single':
            for n in range(3):
                d=x[:,2*n:2*n+2];dist=np.maximum(np.linalg.norm(d,axis=-1),1e-8);h=np.zeros((trials,size));h[:,2*n:2*n+2]=d/dist[:,None]
                scalar_update(x,P,h,measured_ranges[:,step,3+n]-dist,sigma*sigma)
        x[:,ai:ai+3]=wrap(x[:,ai:ai+3]);positions[:,step]=x[:,:6].reshape(trials,3,2);angles[:,step]=x[:,ai:ai+3]
        corrections[:,step]=np.linalg.norm(x[:,:6]-predicted,axis=-1)
    assert np.isfinite(positions[:,start:]).all()
    assert np.linalg.eigvalsh(P).min()>-1e-7
    return positions,angles,corrections


def score(position, truth, start):
    p=position[:,start:];gt=truth[start:]
    relative=np.linalg.norm((p[:,:,1:]-p[:,:,:1])-(gt[None,:,1:]-gt[None,:,:1]),axis=-1)
    absolute=np.linalg.norm(p-gt[None],axis=-1)
    return [dict(position_rmse_m=float(np.sqrt(np.mean(e**2))),position_p95_m=float(np.percentile(e,95)),position_max_m=float(e.max()),field_position_rmse_m=float(np.sqrt(np.mean(a*a)))) for e,a in zip(relative,absolute)]


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,default=OUT);parser.add_argument('--trials',type=int,default=20);parser.add_argument('--profiles',nargs='*',choices=list(PROFILES));args=parser.parse_args()
    if not 1<=args.trials<=100:parser.error('重复次数需为1～100')
    out=args.output.resolve();out.mkdir(parents=True,exist_ok=True);(out/'inputs').mkdir(exist_ok=True);(out/'outputs').mkdir(exist_ok=True)
    began=time.time();fine,p,yaw,v=scene();times=fine[::5];p=p[::5];yaw=yaw[::5];v=v[::5]
    # 单点原点；仅评价器知道坐标轴的真实方向。估计器以初始参考车方向定义X轴。
    anchor=np.array((55.,75.));offset=p[0,0]-anchor;axis=np.arctan2(offset[1],offset[0]);truth=rotate(p-anchor,-axis)
    actual_ranges=np.column_stack([np.linalg.norm(p[:,i]-p[:,j],axis=-1) for i,j in EDGES]+[np.linalg.norm(p[:,i]-anchor,axis=-1) for i in range(3)])
    standard=np.random.default_rng(291002).normal(size=(args.trials,len(times),6))
    range_bias=np.random.default_rng(291003).normal(0,RANGE_BIAS_STD_M,(args.trials,1,6))
    np.savez_compressed(out/'truth_and_range_inputs.npz',time_s=times,truth=truth,world_truth=p,world_anchor=anchor,
        range_standard=standard,range_bias=range_bias,range_true=actual_ranges,yaw=yaw,velocity=v)
    start=int(round(INIT_SECONDS/DT));summary=dict(date='2026-10-29',anchor_count=1,anchor_world_position_m=anchor.tolist(),anchor_role='一个已知位置、配备UWB的地标或落地无人机',trials_per_condition=args.trials,duration_s=300,initialization_end_s=INIT_SECONDS,evaluation_interval_s=[INIT_SECONDS,300],profiles=PROFILES,range_sigmas_m=SIGMAS,range_frequency_hz=10,link_count=6,total_ranges_per_second=60,position_frame='原点为单锚点，X轴沿初始参考车径向，Y按右手规则；不提供地图北向',sensor_model='IMU测量级、VIO/LIO位移输出级；不运行相机或点云前端',truth_used_by_estimator=False,post_alignment=False,conditions=[],scene=dict(max_speed_mps=float(np.linalg.norm(v,axis=-1).max()),max_separation_m=float(actual_ranges[:,:3].max()),max_anchor_distance_m=float(actual_ranges[:,3:].max()),x_extent_m=float(np.ptp(p[:,:,0])),y_extent_m=float(np.ptp(p[:,:,1]))))
    def persist():
        summary['range_bias_std_m']=RANGE_BIAS_STD_M
        summary['main_range_sigma_m']=BASE_SIGMA
        summary['parameter_basis']='室外MEMS误差项审查后重跑；模型参数的来源/假设分别列明，不冒充开源实机复测'
        summary['elapsed_seconds']=round(time.time()-began,2);(out/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    for profile in (args.profiles or list(PROFILES)):
        sensor=make_measurements(times,p,yaw,v,profile,args.trials);np.savez_compressed(out/'inputs'/f'{profile}.npz',**sensor)
        for sigma in SIGMAS:
            measured=actual_ranges[None]+sigma*standard+range_bias
            init=[initialize(sensor['local'][k],measured[k],times,sigma) for k in range(args.trials)]
            initial_position=np.array([r[0] for r in init]);initial_angle=np.array([r[1] for r in init]);details=[r[2] for r in init]
            np.savez_compressed(out/'inputs'/f'{profile}_s{sigma:.2f}_initialization.npz',position=initial_position,angle=initial_angle)
            for mode in MODES:
                position,angle,correction=estimate(sensor,measured,PROFILES[profile],initial_position,initial_angle,times,sigma,mode)
                name=f'{profile}_{mode}_s{sigma:.2f}';metrics=score(position,truth,start)
                np.savez_compressed(out/'outputs'/f'{name}.npz',time_s=times,position=position,alignment_angle=angle,correction_m=correction)
                summary['conditions'].append(dict(name=name,profile=profile,mode=mode,range_sigma_m=sigma,anchor_count=1,initialization=details,trials=metrics))
                vals=[r['position_rmse_m'] for r in metrics];print(f'[完成] {name}: {args.trials}次，相对位置RMSE {min(vals):.3f}～{max(vals):.3f}米',flush=True);persist()
    # 调用已经跑通的C++默认UWB库，未复制公式冒充库输出。
    lib=library(ROOT/'build_rte_verify/libzju_coop.so');baseline=[]
    for sigma in SIGMAS:
        positions=np.empty((args.trials,len(times),3,2));target=np.zeros_like(truth)
        q2=truth[:,1]-truth[:,0];q3=truth[:,2]-truth[:,0];r=np.linalg.norm(q2,axis=-1);target[:,1,0]=r;target[:,2,0]=np.sum(q2*q3,axis=-1)/r;target[:,2,1]=np.abs(q2[:,0]*q3[:,1]-q2[:,1]*q3[:,0])/r
        for k in range(args.trials):
            for step in range(len(times)):
                f=solve(lib,actual_ranges[step,:3]+sigma*standard[k,step,:3]+range_bias[k,0,:3]);assert f.disposition==0;positions[k,step]=np.stack((f.x_m[:],f.y_m[:]),axis=-1)
        name=f'uwb_s{sigma:.2f}';np.savez_compressed(out/'outputs'/f'{name}.npz',time_s=times,position=positions)
        baseline.append(dict(name=name,profile='uwb',mode='triangle',anchor_count=0,range_sigma_m=sigma,trials=score(positions,target,start)))
        print(f'[完成] 默认UWB库 {sigma*100:.0f}厘米测距',flush=True)
    summary['conditions']+=baseline;persist()
    with (out/'全部仿真结果.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=['condition','profile','mode','range_sigma_m','repeat','position_rmse_m','position_p95_m','position_max_m','field_position_rmse_m']);w.writeheader()
        for row in summary['conditions']:
            for k,m in enumerate(row['trials']):w.writerow(dict(condition=row['name'],profile=row['profile'],mode=row['mode'],range_sigma_m=row['range_sigma_m'],repeat=k+1,**m))
    paths=[Path(__file__),out/'truth_and_range_inputs.npz',ROOT/'build_rte_verify/libzju_coop.so']+list((out/'inputs').glob('*.npz'))+list((out/'outputs').glob('*.npz'))
    manifest=dict(command=' '.join(__import__('sys').argv),python=platform.python_version(),numpy=np.__version__,elapsed_seconds=summary['elapsed_seconds'],conditions=len(summary['conditions']),total_trials=sum(len(c['trials']) for c in summary['conditions']),hashes={str(p.relative_to(ROOT)) if p.is_relative_to(ROOT) else str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths})
    (out/'运行清单.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
    print(f'全部完成：{manifest["conditions"]}种条件、{manifest["total_trials"]}次，耗时{summary["elapsed_seconds"]}秒',flush=True)


if __name__=='__main__':main()
