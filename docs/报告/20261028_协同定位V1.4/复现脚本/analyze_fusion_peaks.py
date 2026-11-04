#!/usr/bin/env python3
"""用真实EKF重放、方向/测距噪声对照及几何分解分析峰值，不拟合输出。"""
import json
import argparse
import csv
from pathlib import Path
import numpy as np
from scipy.signal import find_peaks
import evaluate_single_anchor as base
import evaluate_anchor_layout as layout
from evaluate_common_reference import reference_relative, OUT as COMMON
from evaluate_rte import ROOT

OUT=ROOT/'data/fusion_peak_analysis_20261030'
INPUTS=ROOT/'data/single_anchor_outdoor_20261029'
LAYOUT=ROOT/'data/single_anchor_layout_20261029'


def main():
    global OUT
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profiles', nargs='+', default=['vio_基础','vio_增强','lio_基础','lio_增强'], choices=[p for p in base.PROFILES if not p.startswith('imu_')])
    parser.add_argument('--output', type=Path, default=OUT)
    args=parser.parse_args()
    OUT=args.output
    OUT.mkdir(parents=True,exist_ok=True);raw=np.load(INPUTS/'truth_and_range_inputs.npz');t=raw['time_s'];world=raw['world_truth'];truth=base.rotate(world-world[:,:1],-raw['yaw'][0,0])
    peer=raw['range_true'][None,:,:3]+.1*raw['range_standard'][:1,:,:3]+raw['range_bias'][:1,:,:3]
    lengths=np.stack([np.linalg.norm(world[:,i]-world[:,j],axis=-1) for i,j in ((0,1),(0,2),(1,2))],axis=-1)
    cross=np.abs(np.cross(world[:,1]-world[:,0],world[:,2]-world[:,0]));minimum_altitude=cross/lengths.max(axis=-1)
    yaw_rate=np.max(np.abs(np.gradient(np.unwrap(raw['yaw'],axis=0),.1,axis=0)),axis=-1);records=[]
    def finite_number(value):return float(value) if np.isfinite(value) else None
    def window_max(value):return float(np.nanmax(value)) if np.isfinite(value).any() else None
    for profile in args.profiles:
        cfg=dict(base.PROFILES[profile]);sensor={k:v[:1].copy() for k,v in dict(np.load(INPUTS/'inputs'/f'{profile}.npz')).items()};initial=np.load(LAYOUT/'outputs'/f'{profile}_no_anchor_initialization.npz');p0=initial['position'][:1];a0=initial['angle'][:1]
        original=base.scalar_update;trace=[];calls=0
        def observed(x,P,h,innovation,variance):
            nonlocal calls
            original(x,P,h,innovation,variance);calls+=1
            if calls%3==0:
                blocks=[]
                for n in (1,2):blocks.append(P[:,2*n:2*n+2,2*n:2*n+2]+P[:,:2,:2]-P[:,2*n:2*n+2,:2]-P[:,:2,2*n:2*n+2])
                trace.append((x.copy(),np.sqrt(np.maximum(np.mean([np.trace(b,axis1=1,axis2=2) for b in blocks],axis=0),0)),float(np.abs(innovation).max())))
        base.scalar_update=observed
        try:p,angle,correction=base.estimate(sensor,peer,cfg,p0,a0,t,.1,'peer')
        finally:base.scalar_update=original
        saved=np.load(LAYOUT/'outputs'/f'{profile}_no_anchor_peer.npz')['position'][:1];np.testing.assert_allclose(p,saved,equal_nan=True,atol=1e-11,rtol=1e-11)
        variants={'原始输入':reference_relative(p,a0[:,0])[0]};extra={}
        # 同初始状态、同随机样本，去掉60秒以后增量中的方向慢漂移。
        previous=base.PROFILES[profile];base.PROFILES[profile]=dict(cfg,yaw_drift_std=0.)
        try:clean=base.make_measurements(t,world,raw['yaw'],raw['velocity'],profile,20)
        finally:base.PROFILES[profile]=previous
        changed={k:v.copy() for k,v in sensor.items()};changed['increments'][:,601:]=clean['increments'][:1,601:]
        got,_,_=base.estimate(changed,peer,cfg,p0,a0,t,.1,'peer');variants['去除后续方向漂移']=reference_relative(got,a0[:,0])[0]
        clean_ranges=raw['range_true'][None,:,:3]+raw['range_bias'][:1,:,:3]
        got,_,_=base.estimate(sensor,clean_ranges,cfg,p0,a0,t,.1,'peer');variants['去除后续测距随机项']=reference_relative(got,a0[:,0])[0]
        got,_,_=base.estimate(sensor,peer,dict(cfg,yaw_drift_std=cfg['yaw_drift_std']*10),p0,a0,t,.1,'peer');variants['方向过程方差扩大100倍']=reference_relative(got,a0[:,0])[0]
        # 测量驱动的90秒建系对照：延长数据积累，不向估计器传入真值。
        old_base,old_layout=base.INIT_SECONDS,layout.INIT_SECONDS
        base.INIT_SECONDS=layout.INIT_SECONDS=90.
        try:
            p90,a90,details90=layout.initialize_peer_only(sensor['local'][0],peer[0],t,.1)
            got,_,_=base.estimate(sensor,peer,cfg,p90[None],a90[None],t,.1,'peer')
            variants['90秒测量建系']=reference_relative(got,a90[:1])[0]
        finally:base.INIT_SECONDS,layout.INIT_SECONDS=old_base,old_layout
        error={name:np.sqrt(np.mean(np.sum((v[:,1:]-truth[:,1:])**2,axis=-1),axis=-1)) for name,v in variants.items()}
        q=truth[:,1:];estimated=variants['原始输入'][:,1:]
        common_angle=np.arctan2(np.sum(q[:,:,0]*estimated[:,:,1]-q[:,:,1]*estimated[:,:,0],axis=1),np.sum(q*estimated,axis=(1,2)))
        rotated_truth=base.rotate(q,common_angle[:,None]);shape_error=np.sqrt(np.mean(np.sum((estimated-rotated_truth)**2,axis=-1),axis=-1))
        e=variants['原始输入'][:,1:]-truth[:,1:];u=truth[:,1:]/np.maximum(np.linalg.norm(truth[:,1:],axis=-1,keepdims=True),1e-9)
        radial=np.sum(e*u,axis=-1);tangent=e[:,:,1]*u[:,:,0]-e[:,:,0]*u[:,:,1]
        peaks,_=find_peaks(np.nan_to_num(error['原始输入'],nan=0),distance=150,prominence=.04)
        peaks=sorted([i for i in peaks if i>650],key=lambda i:error['原始输入'][i],reverse=True)[:3]
        peak_rows=[]
        for i in peaks:
            row=dict(time_s=float(t[i]),error_m=float(error['原始输入'][i]),tangential_energy_fraction=float(np.sum(tangent[i]**2)/max(np.sum(e[i]**2),1e-20)),maximum_peer_range_m=float(lengths[i].max()),minimum_triangle_altitude_m=float(minimum_altitude[i]),maximum_yaw_rate_rad_s=float(yaw_rate[i]),
                controlled_errors_m={k:finite_number(v[i]) for k,v in error.items()},window_max_controlled_errors_m={k:window_max(v[max(600,i-30):min(len(t),i+31)]) for k,v in error.items()})
            row.update(common_rotation_error_deg=float(np.rad2deg(common_angle[i])),remaining_shape_error_m=float(shape_error[i]))
            peak_rows.append(row)
        std=np.r_[np.full(601,np.nan),[v[1][0] for v in trace]];assert len(std)==len(t)
        values=dict(time_s=t,truth=truth,original_position=variants['原始输入'],internal_relative_std_m=std,maximum_peer_range_m=lengths.max(axis=-1),minimum_triangle_altitude_m=minimum_altitude,maximum_yaw_rate_rad_s=yaw_rate,radial_error_m=radial,tangential_error_m=tangent)
        for k,(name,val) in enumerate(error.items()):values[f'error_{k}']=val
        values.update(common_rotation_error_deg=np.rad2deg(common_angle),remaining_shape_error_m=shape_error)
        np.savez_compressed(OUT/f'{profile}.npz',**values)
        records.append(dict(profile=profile,peaks=peak_rows,variant_names=list(error),rmse_m={name:float(np.sqrt(np.nanmean(v[600:]**2))) for name,v in error.items()},
            same_interval_90_300_rmse_m={name:float(np.sqrt(np.mean(v[900:]**2))) for name,v in error.items()},initialization_90_s=details90,replay_positions_match_saved_output=True))
        print(profile,records[-1]['rmse_m'],'peaks',[(r['time_s'],r['tangential_energy_fraction']) for r in peak_rows],flush=True)
    # 静止加速度观测无法同时分离倾角与三轴偏置；航向不进入重力观测。
    J=np.zeros((6,9));J[:3,:2]=np.array([[0,-9.80665],[9.80665,0],[0,0]]);J[:3,2:5]=np.eye(3);J[3:,5:8]=np.eye(3)
    rank=int(np.linalg.matrix_rank(J));assert rank==6 and np.linalg.norm(J[:,-1])==0
    np.savez_compressed(OUT/'静止可观性.npz',jacobian=J,singular_values=np.linalg.svd(J,compute_uv=False))
    (OUT/'summary.json').write_text(json.dumps(dict(conditions=records,static_observability=dict(unknowns=9,rank=rank,unobservable_directions=3,meaning='两个倾角与加计偏置的耦合方向，以及航向；不以静止均值代替完整标定'),controls='重复试验1，前三项诊断同一初态；另以90秒测量重新建系，90～300秒同区间比较。方向/形状分解仅用于归因，不修饰算法输出，不作为校正后定位精度'),ensure_ascii=False,indent=2)+'\n')


if __name__=='__main__':main()
