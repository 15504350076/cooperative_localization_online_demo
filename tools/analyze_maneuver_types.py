#!/usr/bin/env python3
"""配对比较三车平行、加减速、转弯和非平行运动的单锚点定位。"""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from evaluate_single_anchor import ROOT, DT, PROFILES, make_measurements, initialize, estimate, score, rotate
from evaluate_sensor_anchor_v14 import tangent_heading
import plot_sensor_anchor_v14 as shared

OUT=ROOT/'data/maneuver_types_20261102'
REPORT=ROOT/'docs/报告/20261028_协同定位V1.4'
NAMES=('同向平行','相对加减速','转弯变队形','非平行运动')
COLORS=('#24557b','#ad653a','#4f806b','#805f88')
TRIALS=20


def smoothstep(x):
    x=np.clip(x,0,1)
    return x*x*(3-2*x)


def scenarios(raw):
    t=raw['time_s'];world=raw['world_truth'];center=world[:,0];offset=world-world[:,:1]
    i60=np.argmin(abs(t-60));base=offset.copy();blend=smoothstep((t-60)/20)
    base[:,1:]=offset[:,1:]*(1-blend[:,None,None])+offset[i60,1:][None]*blend[:,None,None]
    tau=np.maximum(t-80,0);gate=smoothstep(tau/12);wave=np.sin(2*np.pi*tau/90)*gate
    values={name:base.copy() for name in NAMES}
    values['相对加减速'][:,1,0]+=4*wave;values['相对加减速'][:,2,0]-=3*wave
    values['非平行运动'][:,1,1]+=4*wave;values['非平行运动'][:,2,1]-=3*wave
    # 旋转队形时跟随201转向，限制队形转动速度与总转角。
    yaw=raw['yaw'][:,0];delta=yaw-yaw[i60];target=np.clip(delta,-.35,.35)*gate
    theta=np.zeros_like(t)
    for j in range(1,len(t)):theta[j]=theta[j-1]+np.clip((target[j]-theta[j-1])/8,-.015,.015)*DT
    values['转弯变队形'][:,1:]=rotate(base[:,1:],theta[:,None])
    return {name:center[:,None,:]+offsets for name,offsets in values.items()}


def compute(name,world,t,anchor,standard,bias):
    velocity=np.gradient(world,DT,axis=0)
    speed=np.linalg.norm(velocity,axis=-1)
    assert speed.max()<3.0,(name,speed.max())
    yaw=tangent_heading(velocity)
    distances=np.stack([np.linalg.norm(world[:,i]-world[:,j],axis=-1) for i,j in ((0,1),(0,2),(1,2))]+[np.linalg.norm(world[:,i]-anchor,axis=-1) for i in range(3)],axis=-1)
    measured=distances[None]+.1*standard+bias
    sensor=make_measurements(t,world,yaw,velocity,'vio_增强',TRIALS)
    init=[initialize(sensor['local'][i],measured[i],t,.1) for i in range(TRIALS)]
    p0=np.array([x[0] for x in init]);angle0=np.array([x[1] for x in init])
    position,_,_=estimate(sensor,measured,PROFILES['vio_增强'],p0,angle0,t,.1,'single')
    estimate_ref=rotate(position-position[:,:,:1],-angle0[:,0,None,None])
    truth_ref=rotate(world-world[:,:1],-yaw[0,0])
    errors=np.sqrt(np.mean(np.sum((estimate_ref[:,:,1:]-truth_ref[None,:,1:])**2,axis=-1),axis=-1))
    metrics=score(estimate_ref,truth_ref,600);rmse=np.array([m['position_rmse_m'] for m in metrics])
    rel_speed=np.stack([np.linalg.norm(velocity[:,i]-velocity[:,j],axis=-1) for i,j in ((0,1),(0,2),(1,2))],axis=-1).max(axis=-1)
    rel_acc=np.gradient(velocity,DT,axis=0);rel_acc=np.stack([np.linalg.norm(rel_acc[:,i]-rel_acc[:,j],axis=-1) for i,j in ((0,1),(0,2),(1,2))],axis=-1).max(axis=-1)
    diff=np.arctan2(np.sin(yaw[:,1:]-yaw[:,:1]),np.cos(yaw[:,1:]-yaw[:,:1]))
    moving=(speed[:,1:]>=.3)&(speed[:,:1]>=.3)
    heading=np.where(np.any(moving,axis=1),np.rad2deg(np.max(np.where(moving,abs(diff),0.),axis=1)),np.nan)
    return dict(time_s=t,world=world,truth_ref=truth_ref,estimate_ref=estimate_ref,velocity=velocity,yaw=yaw,errors=errors,rmse=rmse,
        rel_speed=rel_speed,rel_acc=rel_acc,heading_diff_deg=heading,measured=measured,local=sensor['local'],initial_position=p0,initial_angle=angle0,
        metrics=dict(max_speed_mps=float(speed.max()),max_peer_distance_m=float(distances[:,:3].max()),max_anchor_distance_m=float(distances[:,3:].max()),
            rmse_mean=float(rmse.mean()),rmse_std=float(rmse.std(ddof=1)),rmse_min=float(rmse.min()),rmse_max=float(rmse.max()),
            peak_mean=float(np.mean(np.max(errors[:,600:],axis=1))),max_relative_speed=float(rel_speed[800:].max()),
            max_relative_acceleration=float(rel_acc[800:].max()),max_velocity_angle_deg=float(np.nanmax(heading[800:])),
            init_converged=sum(int(x[2]['optimizer_success']) for x in init)))


def draw(results):
    shared.DEST=REPORT/'figures'
    # 四种场景的三车轨迹：由计算中保存的真实轨迹绘制。
    anchor=np.load(ROOT/'data/single_anchor_outdoor_20261029/truth_and_range_inputs.npz')['world_anchor']
    fig,axes=plt.subplots(2,2,figsize=(9.5,9.5))
    for ax,name in zip(axes.flat,NAMES):
        a=results[name];world=a['world']
        for i,color in enumerate(shared.C):
            ax.plot(world[:,i,0],world[:,i,1],color=color,label=f'{201+i}车')
            ax.scatter(world[0,i,0],world[0,i,1],color=color,s=20)
        ax.scatter(*anchor,marker='*',s=130,color='#333',label='UWB锚点')
        ax.set(xlabel='场地X（米）',ylabel='场地Y（米）',title=name,aspect='equal',xlim=(0,110),ylim=(0,150))
        ax.legend(loc='upper right')
    shared.save(fig,'review_不同类型机动轨迹')
    # 相同初始条件下的误差、相对速度、加速度和运动方向差。
    fig,axes=plt.subplots(2,2,figsize=(9.5,9.0))
    for name,color in zip(NAMES,COLORS):
        a=results[name];t=a['time_s'];m=t>=60
        axes[0,0].plot(t[m],a['errors'][:,m].mean(axis=0),color=color,label=name)
        axes[0,1].plot(t[m],a['rel_speed'][m],color=color,label=name)
        axes[1,0].plot(t[m],a['rel_acc'][m],color=color,label=name)
        axes[1,1].plot(t[m],a['heading_diff_deg'][m],color=color,label=name)
    axes[0,0].set(xlabel='仿真时间（秒）',ylabel='两车合并相对位置误差（米）',title='20次重复的平均误差')
    axes[0,1].set(xlabel='仿真时间（秒）',ylabel='最大车间相对速度（米/秒）',title='相对速度变化')
    axes[1,0].set(xlabel='仿真时间（秒）',ylabel='最大车间相对加速度（米/秒²）',title='相对加速度变化')
    axes[1,1].set(xlabel='仿真时间（秒）',ylabel='速度方向差（度）',title='车速≥0.3米/秒时的方向差')
    for ax in axes.flat:
        ax.legend(loc='upper right');lo,hi=ax.get_ylim();ax.set_ylim(lo,hi+(hi-lo)*.4)
    shared.save(fig,'review_不同类型机动对照')


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--plot-only',action='store_true');args=parser.parse_args()
    OUT.mkdir(parents=True,exist_ok=True);(OUT/'outputs').mkdir(exist_ok=True)
    results={}
    if args.plot_only:
        for name in NAMES:
            with np.load(OUT/'outputs'/f'{name}.npz') as z:results[name]={k:z[k] for k in z.files}
    else:
        raw=np.load(ROOT/'data/single_anchor_outdoor_20261029/truth_and_range_inputs.npz');t=raw['time_s'];anchor=raw['world_anchor']
        paths=scenarios(raw);standard=raw['range_standard'];bias=raw['range_bias'];common_prefix=None;rows=[]
        for name in NAMES:
            r=compute(name,paths[name],t,anchor,standard,bias)
            prefix=(r['measured'][:,:601],r['local'][:,:601],r['initial_position'],r['initial_angle'])
            if common_prefix is None:common_prefix=prefix
            else:
                for a,b in zip(prefix,common_prefix):np.testing.assert_allclose(a,b,atol=1e-9,rtol=1e-9)
            np.savez_compressed(OUT/'outputs'/f'{name}.npz',**{k:v for k,v in r.items() if k!='metrics'})
            rows.append(dict(name=name,**r['metrics']));results[name]=r
            print(name,json.dumps(r['metrics'],ensure_ascii=False),flush=True)
        summary=dict(date='2026-11-02',trials=TRIALS,profile='vio_增强',anchor=anchor.tolist(),range_std_m=.1,range_bias_std_m=.03,
            evaluation_interval_s=[60,300],common_initialization_s=[0,60],scenario_transition_s=[60,80],active_maneuver_from_s=80,
            conditions=rows,scope='中部单锚点、VIO增强输出模型、配对测距及运动随机样本；起步和初始化输入一致。')
        (OUT/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
    draw(results)
    print('图表绘制完成',flush=True)

if __name__=='__main__':main()
