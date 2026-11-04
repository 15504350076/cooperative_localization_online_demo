#!/usr/bin/env python3
"""受控改变编队相对机动幅度，评估几何观测量和最终定位误差。"""
import argparse, csv, json, hashlib
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.linalg import svdvals
from evaluate_single_anchor import ROOT, DT, PROFILES, make_measurements, initialize, estimate, score, rotate
from evaluate_sensor_anchor_v14 import scene
from plot_sensor_anchor_v14 import save
import plot_sensor_anchor_v14 as shared

OUT=ROOT/'data/maneuver_observability_20261101'
REPORT=ROOT/'docs/报告/20261028_协同定位V1.4'
PROFILE='vio_增强'; SIGMA=.10; TRIALS=20
CASES={'低相对机动':.6,'当前相对机动':1.0,'高相对机动':1.2}
COLORS=['#24557b','#ad653a','#4f806b']


def measurement_jacobian(points, anchor):
    rows=[]
    for i,j in ((0,1),(0,2),(1,2)):
        d=points[i]-points[j]; r=max(np.linalg.norm(d),1e-8);h=np.zeros(6);h[2*i:2*i+2]=d/r;h[2*j:2*j+2]=-d/r;rows.append(h)
    for i in range(3):
        d=points[i]-anchor; r=max(np.linalg.norm(d),1e-8);h=np.zeros(6);h[2*i:2*i+2]=d/r;rows.append(h)
    return np.stack(rows)


def metrics_for_case(name,alpha,world_base,yaw,anchor,times):
    ref=world_base[:,0].copy();world=ref[:,None,:]+alpha*(world_base-ref[:,None,:])
    velocity=np.gradient(world,DT,axis=0)
    axis=np.arctan2((world[0,0]-anchor)[1],(world[0,0]-anchor)[0])
    truth=rotate(world-anchor,-axis)
    ranges=np.concatenate((np.stack([np.linalg.norm(world[:,i]-world[:,j],axis=-1) for i,j in ((0,1),(0,2),(1,2))],axis=-1),np.stack([np.linalg.norm(world[:,i]-anchor,axis=-1) for i in range(3)],axis=-1)),axis=-1)
    rng=np.random.default_rng(291002);standard=rng.normal(size=(TRIALS,len(times),6))
    bias=np.random.default_rng(291003).normal(0,.03,(TRIALS,1,6));measured=ranges[None]+SIGMA*standard+bias
    sensor=make_measurements(times,world,yaw,velocity,PROFILE,TRIALS)
    init=[initialize(sensor['local'][k],measured[k],times,SIGMA) for k in range(TRIALS)]
    p0=np.array([x[0] for x in init]);a0=np.array([x[1] for x in init])
    position,angles,correction=estimate(sensor,measured,PROFILES[PROFILE],p0,a0,times,SIGMA,'single')
    # 与分级、布点结果同一口径：201同刻位置作原点，轴向取201自身起点局部轴。
    position_ref=rotate(position-position[:,:,:1],-a0[:,0,None,None])
    truth_ref=rotate(world-world[:,0,None,:],-yaw[0,0])
    trials=score(position_ref,truth_ref,int(60/DT))
    vals=np.array([x['position_rmse_m'] for x in trials])
    # 5-second rolling geometry and relative-motion indicators.
    singular=np.array([svdvals(measurement_jacobian(truth[k],np.zeros(2))) for k in range(len(times))])
    min_sv=singular[:,-2]
    rel_speed=np.zeros(len(times));
    for i,j in ((0,1),(0,2),(1,2)): rel_speed=np.maximum(rel_speed,np.linalg.norm(velocity[:,i]-velocity[:,j],axis=-1))
    yaw_rate=np.max(np.abs(np.gradient(np.unwrap(yaw,axis=0),DT,axis=0)),axis=-1)
    w=max(1,int(5/DT));
    min_sv_roll=np.array([np.min(min_sv[max(0,k-w):k+1]) for k in range(len(times))])
    rel_speed_roll=np.array([np.sqrt(np.mean(rel_speed[max(0,k-w):k+1]**2)) for k in range(len(times))])
    est_ref=position_ref[0]
    error=np.sqrt(np.mean(np.sum((position_ref[:,:,1:]-truth_ref[None,:,1:])**2,axis=-1),axis=-1))
    return dict(name=name,alpha=alpha,world=world,truth=truth,truth_ref=truth_ref,est_ref=est_ref,time_s=times,position=position,errors=error,rmse=vals,
        min_sv=min_sv,min_sv_roll=min_sv_roll,rel_speed=rel_speed,rel_speed_roll=rel_speed_roll,yaw_rate=yaw_rate,
        metrics=dict(rmse_mean=float(vals.mean()),rmse_std=float(vals.std(ddof=1)),rmse_min=float(vals.min()),rmse_max=float(vals.max()),
            speed_max=float(np.linalg.norm(velocity,axis=-1).max()),relative_speed_max=float(rel_speed.max()),
            min_sv_median=float(np.median(min_sv[600:])),min_sv_min=float(np.min(min_sv[600:])),
            relative_speed_rms=float(np.sqrt(np.mean(rel_speed[600:]**2))),yaw_rate_max=float(yaw_rate.max())))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plot-only',action='store_true',help='只读取已保存的仿真结果重新绘图')
    args=parser.parse_args()
    OUT.mkdir(exist_ok=True);(OUT/'outputs').mkdir(exist_ok=True)
    if not args.plot_only:
        times,world_base,yaw,velocity=scene();times=times[::5];world_base=world_base[::5];yaw=yaw[::5]
        anchor=np.array((55.,75.))
    results={}
    for name,alpha in CASES.items():
        if args.plot_only:
            with np.load(OUT/'outputs'/f'{name}.npz') as saved:r=dict(saved)
        else:
            r=metrics_for_case(name,alpha,world_base,yaw,anchor,times)
            np.savez_compressed(OUT/'outputs'/f'{name}.npz',time_s=r['time_s'],truth_ref=r['truth_ref'],est_ref=r['est_ref'],errors=r['errors'],min_sv=r['min_sv'],min_sv_roll=r['min_sv_roll'],rel_speed=r['rel_speed'],rel_speed_roll=r['rel_speed_roll'],yaw_rate=r['yaw_rate'],rmse=r['rmse'])
        results[name]=r
    # Figure: maneuver geometry, observability, motion excitation, and final error.
    fig,axes=plt.subplots(2,2,figsize=(12,8.2))
    for color,(name,r) in zip(COLORS,results.items()):
        axes[0,0].plot(r['truth_ref'][:,1,0],r['truth_ref'][:,1,1],color=color,label=name)
        axes[0,0].plot(r['truth_ref'][:,2,0],r['truth_ref'][:,2,1],color=color,ls='--')
        axes[0,1].plot(r['time_s'],r['min_sv_roll'],color=color,label=name)
        axes[1,0].plot(r['time_s'],r['rel_speed_roll'],color=color,label=name)
    axes[0,0].set(xlabel='相对201的固定X（米）',ylabel='相对201的固定Y（米）',title='不同相对机动下的编队轨迹');axes[0,0].legend()
    axes[0,1].set(xlabel='仿真时间（秒）',ylabel='距离观测矩阵最小非零奇异值',title='距离观测几何的局部敏感度');axes[0,1].legend()
    axes[1,0].set(xlabel='仿真时间（秒）',ylabel='车间相对速度（米/秒）',title='相对机动强度');axes[1,0].legend()
    for ax in (axes[0,1],axes[1,0]):
        lo,hi=ax.get_ylim();ax.set_ylim(lo,hi+(hi-lo)*.4)
    groups=[r['rmse'] for r in results.values()];axes[1,1].boxplot(groups,labels=list(results),showfliers=True)
    axes[1,1].set(xlabel='机动条件',ylabel='相对位置RMSE（米）',title='评价时段定位误差（20次重复）')
    for ax in axes.flat:ax.grid(alpha=.2)
    shared.DEST=REPORT/'figures';save(fig,'review_相对机动与可观性')
    # Error process figure for direct output comparison.
    fig,ax=plt.subplots(figsize=(11,5.4))
    for color,(name,r) in zip(COLORS,results.items()):ax.plot(r['time_s'],r['errors'].mean(axis=0),color=color,label=name)
    ax.set(xlabel='仿真时间（秒）',ylabel='两车合并相对位置误差（米）',title='相对机动对逐时定位误差的影响');ax.legend();ax.grid(alpha=.2)
    save(fig,'review_相对机动误差过程')
    if args.plot_only:
        print('已从保存结果重绘两幅相对机动图。')
        return
    summary=dict(date='2026-11-01',profile=PROFILE,range_sigma_m=SIGMA,trials=TRIALS,environment='滩涂、沙地和训练场退化输入条件',evaluation_frame='201当前位置为原点、平行201自身起点局部轴；估计以初始化所得201转角转换，真值仅用于评价',scene='保持201轨迹，按比例缩放202/203相对201的轨迹，形成低/当前/高相对机动三组；其余条件配对',conditions=[])
    for name,r in results.items():summary['conditions'].append(dict(name=name,alpha=r['alpha'],metrics=r['metrics']))
    (OUT/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
    with (OUT/'机动结果.csv').open('w',newline='') as f:
        w=csv.writer(f);w.writerow(['机动条件','相对轨迹比例','速度最大值_mps','车间相对速度最大值_mps','观测矩阵最小奇异值中位数','RMSE均值_m','RMSE标准差_m','RMSE范围_m'])
        for c in summary['conditions']:
            m=c['metrics'];w.writerow([c['name'],c['alpha'],m['speed_max'],m['relative_speed_max'],m['min_sv_median'],m['rmse_mean'],m['rmse_std'],f'{m["rmse_min"]}～{m["rmse_max"]}'])
    source=dict(input='data/single_anchor_outdoor_20261029/truth_and_range_inputs.npz',profile=PROFILE,range_sigma_m=SIGMA,trials=TRIALS,
        conditions=[dict(name=n,output=f'data/maneuver_observability_20261101/outputs/{n}.npz',alpha=r['alpha']) for n,r in results.items()],
        figures=['review_相对机动与可观性','review_相对机动误差过程'],drawing='由实际受控仿真输出绘制，未手工调整曲线')
    (OUT/'图线来源.json').write_text(json.dumps(source,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(summary,ensure_ascii=False,indent=2))

if __name__=='__main__':main()
