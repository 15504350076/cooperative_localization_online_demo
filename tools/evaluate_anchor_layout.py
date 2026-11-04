#!/usr/bin/env python3
"""真正无锚点的传感器分级，及同一个已知UWB点的五种布置；输入配对。"""
import csv
import hashlib
import json
import time
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares

from evaluate_single_anchor import OUT as INPUTS, ROOT, PROFILES, DT, INIT_SECONDS, initialize, estimate, score, rotate, wrap
from evaluate_rte import EDGES

OUT=ROOT/'data/single_anchor_layout_20261029'
PLACEMENTS={'区域中部':(55.,75.),'起点侧方':(15.,45.),'区域左侧':(0.,75.),'区域右侧':(115.,75.),'区域角部':(0.,0.)}


def initialize_peer_only(local, ranges, times, sigma):
    """只接收三条车间距离，不接收锚点位置或车锚距离。

    参考车起点原点、起点参考边为正X，未知起点位置3自由度与局部转角3自由度。
    """
    assert ranges.shape[1]==3
    selected=np.flatnonzero(times<=INIT_SECONDS)[::10];loc=local[selected];measured=ranges[selected]
    d12,d13,d23=np.mean(ranges[times<30],axis=0);x3=(d12*d12+d13*d13-d23*d23)/(2*d12);y3=np.sqrt(max(d13*d13-x3*x3,.01))
    def residual(z,jacobian=False):
        p0=np.array(((0.,0.),(z[0],0.),(z[1],z[2])));moved=rotate(loc,z[None,3:]);p=p0[None]+moved
        jp=np.zeros((len(selected),3,2,6));jp[:,1,0,0]=1;jp[:,2,0,1]=1;jp[:,2,1,2]=1
        for n in range(3):jp[:,n,0,3+n]=-moved[:,n,1];jp[:,n,1,3+n]=moved[:,n,0]
        rows=[]
        for edge,(i,j) in enumerate(EDGES):
            delta=p[:,i]-p[:,j];distance=np.maximum(np.linalg.norm(delta,axis=-1),1e-8)
            rows.append(np.einsum('ti,tij->tj',delta/distance[:,None],jp[:,i]-jp[:,j])/sigma if jacobian else (distance-measured[:,edge])/sigma)
        return np.concatenate(rows,axis=0)
    candidates=[]
    for sign in (-1,1):
        # 完整覆盖共同转角，并增加相对转角初值；所有候选只按测量残差选取。
        for angle in (-np.pi/2,0.,np.pi/2,np.pi):
          for offsets in ((0.,0.,0.),(0.,-.5,.5),(0.,.5,-.5)):
            initial=np.r_[d12,x3,sign*y3,np.array([angle]*3)+offsets]
            candidates.append(least_squares(residual,initial,jac=lambda z:residual(z,True),bounds=(np.r_[.01,[-np.inf]*5],np.full(6,np.inf)),max_nfev=100))
    result=min(candidates,key=lambda r:float(r.fun@r.fun));z=result.x;theta=wrap(z[3:]);p0=np.array(((0.,0.),(z[0],0.),(z[1],z[2])))
    singular=np.linalg.svd(result.jac,compute_uv=False);rank=int(np.count_nonzero(singular>singular[0]*max(result.jac.shape)*np.finfo(float).eps))
    return p0+rotate(local[int(round(INIT_SECONDS/DT))],theta),theta,dict(range_fit_rmse_m=float(np.sqrt(np.mean(result.fun**2))*sigma),jacobian_rank=rank,unknown_count=6,optimizer_success=bool(result.success),anchor_measurements_used=0)


def geometry(world,anchor,times):
    delta=world-anchor;angles=np.unwrap(np.arctan2(delta[:,:,1],delta[:,:,0]),axis=0);spans=[]
    for step in range(int((INIT_SECONDS+5)/DT),len(times)):
        spans.extend(np.rad2deg(np.ptp(angles[step-50:step+1],axis=0)))
    return dict(maximum_range_m=float(np.linalg.norm(delta,axis=-1).max()),minimum_range_m=float(np.linalg.norm(delta,axis=-1).min()),median_five_second_bearing_span_deg=float(np.median(spans)))


def main():
    out=OUT;out.mkdir(parents=True,exist_ok=True);(out/'outputs').mkdir(exist_ok=True);began=time.time()
    raw=np.load(INPUTS/'truth_and_range_inputs.npz');times=raw['time_s'];world=raw['world_truth'];start=int(INIT_SECONDS/DT);sigma=.1
    peer=raw['range_true'][None,:,:3]+sigma*raw['range_standard'][:,:,:3]+raw['range_bias'][:,:,:3]
    origin=world[0,0];axis=np.arctan2(*(world[0,1]-origin)[::-1]);peer_truth=rotate(world-origin,-axis)
    summary=dict(date='2026-10-29',input_directory=str(INPUTS.relative_to(ROOT)),range_sigma_m=sigma,range_bias_std_m=.03,trials_per_condition=20,
        evaluation_interval_s=[60,300],anchor_count_per_run='0或1，绝不同时布置多个点',conditions=[],placement_geometry={name:geometry(world,np.array(point),times) for name,point in PLACEMENTS.items()})
    def record(name,profile,mode,point,position,angles,correction,truth,details):
        target=out/'outputs'/f'{name}.npz';np.savez_compressed(target,time_s=times,position=position,truth=truth,alignment_angle=angles,correction_m=correction)
        metrics=score(position,truth,start);summary['conditions'].append(dict(name=name,profile=profile,mode=mode,placement=point,anchor_count=0 if point is None else 1,
            trials=metrics,initialization=details,frame='起点参考车/参考边固定局部系' if point is None else '唯一点原点/初始点至参考车固定局部系'))
        summary['elapsed_seconds']=round(time.time()-began,2);(out/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
        v=[r['position_rmse_m'] for r in metrics];print(f'[完成] {name}: 20次，RMSE {min(v):.3f}～{max(v):.3f}米',flush=True)
    for profile,cfg in PROFILES.items():
        sensor=dict(np.load(INPUTS/'inputs'/f'{profile}.npz'))
        init=[initialize_peer_only(sensor['local'][k],peer[k],times,sigma) for k in range(20)];p0=np.array([r[0] for r in init]);a0=np.array([r[1] for r in init]);details=[r[2] for r in init]
        np.savez_compressed(out/'outputs'/f'{profile}_no_anchor_initialization.npz',position=p0,angle=a0)
        for mode in ('start','peer'):
            p,a,c=estimate(sensor,peer,cfg,p0,a0,times,sigma,mode);record(f'{profile}_no_anchor_{mode}',profile,mode,None,p,a,c,peer_truth,details)
        for label,coordinates in PLACEMENTS.items():
            point=np.array(coordinates);ranges=np.linalg.norm(world-point,axis=-1)
            measured=np.concatenate((peer,ranges[None]+sigma*raw['range_standard'][:,:,3:]+raw['range_bias'][:,:,3:]),axis=-1)
            direction=world[0,0]-point;theta=np.arctan2(direction[1],direction[0]);truth=rotate(world-point,-theta)
            init=[initialize(sensor['local'][k],measured[k],times,sigma) for k in range(20)];p0=np.array([r[0] for r in init]);a0=np.array([r[1] for r in init]);details=[r[2] for r in init]
            np.savez_compressed(out/'outputs'/f'{profile}_{label}_initialization.npz',position=p0,angle=a0)
            p,a,c=estimate(sensor,measured,cfg,p0,a0,times,sigma,'single');record(f'{profile}_anchor_{label}',profile,'single',label,p,a,c,truth,details)
    with (out/'蒙特卡洛分组统计.csv').open('w',newline='') as f:
        w=csv.writer(f);w.writerow(['条件','运动输入','后续处理','锚点布置','次数','RMSE均值_米','RMSE样本标准差_米','最小RMSE_米','最大RMSE_米','小于1米次数'])
        for c in summary['conditions']:
            a=np.array([r['position_rmse_m'] for r in c['trials']]);w.writerow([c['name'],c['profile'],c['mode'],c['placement'],len(a),a.mean(),a.std(ddof=1),a.min(),a.max(),np.count_nonzero(a<1)])
    paths=[Path(__file__),ROOT/'tools/evaluate_single_anchor.py',INPUTS/'truth_and_range_inputs.npz']+list((INPUTS/'inputs').glob('*.npz'))+list((out/'outputs').glob('*.npz'))
    manifest=dict(conditions=len(summary['conditions']),total_trials=sum(len(c['trials']) for c in summary['conditions']),elapsed_seconds=summary['elapsed_seconds'],
        hashes={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths})
    (out/'运行清单.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
    print(f'全部完成：{manifest["conditions"]}条件，{manifest["total_trials"]}次，{manifest["elapsed_seconds"]}秒',flush=True)


if __name__=='__main__':main()
