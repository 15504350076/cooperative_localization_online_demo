#!/usr/bin/env python3
"""用输入里程计和SDK输出重建变换，区分真实运动与RTE修正跳变。"""
import csv
import json
from pathlib import Path
import numpy as np

from evaluate_rte import ROOT

OUT=ROOT/'data/sensor_anchor_v14_20261028/jump_analysis'


def rotate(p,angle):
    return np.stack((np.cos(angle)*p[...,0]-np.sin(angle)*p[...,1],
                     np.sin(angle)*p[...,0]+np.cos(angle)*p[...,1]),axis=-1)


def analyze(name):
    base=ROOT/'data/rte_expanded_20261025'
    a=np.genfromtxt(base/f'{name}_output.csv',delimiter=',',names=True)
    a=a[a['elapsed_s']<=300]
    tr=np.genfromtxt(base/f'{name}.truth.csv',delimiter=',',names=True)
    odom=[[],[],[]];ranges=[]
    with (base/f'{name}.csv').open() as stream:
        for row in csv.DictReader(stream):
            if row['kind']=='O':
                odom[int(row['node_id'])-1].append((int(row['timestamp_ns']),float(row['v0']),float(row['v1']),2*np.arctan2(float(row['v8']),float(row['v9']))))
            elif row['kind']=='R':
                ranges.append((int(row['timestamp_ns']),int(row['node_id'])-1,int(row['target_id'])-1,float(row['v0']),float(row['v1'])))
    odom=[np.asarray(rows) for rows in odom]
    stamp=a['state_timestamp_ns'];local=[];angles=[];origin=[]
    for n in (1,2,3):
        o=np.array(odom[n-1]);p=np.stack([np.interp(stamp,o[:,0],o[:,i]) for i in (1,2)],axis=-1)
        local.append(p)
        angle=a[f'yaw_{n}']-np.interp(stamp,o[:,0],np.unwrap(o[:,3]))
        angle=np.arctan2(np.sin(angle),np.cos(angle));angles.append(angle)
    for n in (1,2,3):
        relative=np.stack([a[f'x_{n}'],a[f'y_{n}']],axis=-1)
        origin.append(relative+local[0]-rotate(local[n-1],angles[n-1]))
    valid=(a['status'][1:]==1)&(a['status'][:-1]==1)
    events=[];jump=np.full((len(a),2),np.nan)
    error=np.full((len(a),2),np.nan)
    for n in (2,3):
        xy=np.stack([a[f'x_{n}'],a[f'y_{n}']],axis=-1)
        expected=origin[n-1][:-1]+rotate(local[n-1][1:],angles[n-1][:-1])-local[0][1:]
        shift=np.linalg.norm(xy[1:]-expected,axis=-1)
        jump[1:,n-2]=np.where(valid,shift,np.nan)
        truth=np.stack([np.interp(stamp,tr['timestamp_ns'],tr[f'{axis}_{n}']) for axis in ('x','y')],axis=-1)
        error[:,n-2]=np.where(a['status']==1,np.linalg.norm(xy-truth,axis=-1),np.nan)
        # 未修正时，原变换传播应与SDK输出相符。
        same=valid&(np.diff(a['corrections'])==0)
        assert shift[same].max(initial=0)<1e-7
        for k in np.where(valid&(np.diff(a['corrections'])>0))[0]:
            i=k+1
            events.append(dict(source=name,time_s=float(a['elapsed_s'][i]),vehicle=n,
                correction_displacement_m=float(shift[k]),previous_error_m=float(error[k,n-2]),error_m=float(error[i,n-2]),
                residual_before_m=float(a['rms_m'][k]),residual_after_m=float(a['rms_m'][i]),
                corrections_before=int(a['corrections'][k]),corrections_after=int(a['corrections'][i]),
                origin_yaw_change_deg=float(np.degrees(np.arctan2(np.sin(angles[n-1][i]-angles[n-1][k]),np.cos(angles[n-1][i]-angles[n-1][k]))))))
    # 对本次最大修正重建滑窗的测距雅可比，核对残差并量化弱约束。
    largest=max(events,key=lambda e:e['correction_displacement_m'])
    index=int(np.argmin(abs(a['elapsed_s']-largest['time_s'])))
    end=a['timestamp_ns'][index];jac=[];residual=[]
    for time,i,j,distance,sigma in ranges:
        if not end-5e9<=time<=end:continue
        local_at=[];world_at=[]
        for node in range(3):
            o=odom[node];q=np.array([np.interp(time,o[:,0],o[:,axis]) for axis in (1,2)])
            local_at.append(q);world_at.append(origin[node][index]+rotate(q,angles[node][index]))
        delta=world_at[j]-world_at[i];norm=np.linalg.norm(delta);direction=delta/norm;h=np.zeros(6)
        for node,sign in ((i,-1),(j,1)):
            if not node:continue
            start=(node-1)*3;h[start:start+2]=sign*direction
            q=local_at[node];h[start+2]=sign*np.dot(direction,rotate(np.array([-q[1],q[0]]),angles[node][index]))
        jac.append(h/sigma);residual.append(norm-distance)
    fim=np.array(jac).T@np.array(jac);eigen=np.linalg.eigvalsh(fim);cov=np.linalg.pinv(fim)
    largest['window_recomputed_rms_m']=float(np.sqrt(np.mean(np.square(residual))))
    largest['window_condition_number']=float(eigen[-1]/eigen[0])
    largest['translation_std_m']=np.sqrt(np.diag(cov)[[0,1,3,4]]).tolist()
    assert abs(largest['window_recomputed_rms_m']-largest['residual_after_m'])<1e-5
    np.savez_compressed(OUT/(name+'.npz'),time_s=a['elapsed_s'],valid=a['status']==1,error_m=error,
        residual_m=np.where(a['status']==1,a['rms_m'],np.nan),correction_displacement_m=jump,corrections=a['corrections'])
    return events


def main():
    OUT.mkdir(parents=True,exist_ok=True);events=[]
    for seed in range(3):events.extend(analyze(f'exp_odometry_0.02_seed{seed}'))
    events.sort(key=lambda r:r['correction_displacement_m'],reverse=True)
    with (OUT/'rte_correction_events.csv').open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(events[0]));writer.writeheader();writer.writerows(events)
    (OUT/'summary.json').write_text(json.dumps({'definition':'同一当前里程计位置，比较修正前后相对变换得到的位置差；不使用真值选取修正值',
        'top_events':events[:12],'mechanism':'src/rte/three_vehicle_rte.cpp的correct()直接替换滑窗估计的平移与航向；初次初始化后不再应用初始位姿标准差门限；无位姿增量先验或修正步长约束',
        'source_file':'src/rte/three_vehicle_rte.cpp','source_lines':[212,233,254,255,263]},ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(events[:3],ensure_ascii=False,indent=2))


if __name__=='__main__':main()
