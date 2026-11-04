#!/usr/bin/env python3
"""独立复算全部单锚点结果，重放已保存测量，核对真值未进入估计器。"""
import csv
import hashlib
import inspect
import json
from pathlib import Path

import numpy as np
from evaluate_single_anchor import OUT, ROOT, PROFILES, INIT_SECONDS, estimate


def main():
    summary=json.loads((OUT/'summary.json').read_text());manifest=json.loads((OUT/'运行清单.json').read_text())
    assert summary['anchor_count']==1 and len(summary['conditions'])==84
    assert sum(len(c['trials']) for c in summary['conditions'])==1680
    for filename,sha in manifest['hashes'].items():assert hashlib.sha256((ROOT/filename).read_bytes()).hexdigest()==sha,filename
    raw=np.load(OUT/'truth_and_range_inputs.npz');truth=raw['truth'];times=raw['time_s'];start=int(round(INIT_SECONDS/.1));checked=0
    assert raw['range_true'].shape==(len(times),6)
    assert raw['world_anchor'].shape==(2,)
    for case in summary['conditions']:
        output=np.load(OUT/'outputs'/f'{case["name"]}.npz');position=output['position'][:,start:]
        if case['profile']=='uwb':
            q2=truth[:,1]-truth[:,0];q3=truth[:,2]-truth[:,0];r=np.linalg.norm(q2,axis=-1);target=np.zeros_like(truth)
            target[:,1,0]=r;target[:,2,0]=np.sum(q2*q3,axis=-1)/r;target[:,2,1]=np.abs(q2[:,0]*q3[:,1]-q2[:,1]*q3[:,0])/r
        else:
            assert case['anchor_count']==1
            target=truth
        e=(position[:,:,1:]-position[:,:,:1])-(target[None,start:,1:]-target[None,start:,:1])
        rmse=np.sqrt(np.mean(np.sum(e*e,axis=-1),axis=(1,2)))
        np.testing.assert_allclose(rmse,[r['position_rmse_m'] for r in case['trials']],rtol=1e-12,atol=1e-12)
        checked+=len(rmse)
    replay=[]
    for profile in ('imu_精细','vio_增强','lio_增强'):
        sensor={k:v[:1] for k,v in dict(np.load(OUT/'inputs'/f'{profile}.npz')).items()};initial=np.load(OUT/'inputs'/f'{profile}_s0.10_initialization.npz')
        measured=raw['range_true'][None]+.10*raw['range_standard'][:1]+raw['range_bias'][:1]
        got,_,_=estimate(sensor,measured,PROFILES[profile],initial['position'][:1],initial['angle'][:1],times,.10,'single')
        expected=np.load(OUT/'outputs'/f'{profile}_single_s0.10.npz')['position'][:1]
        np.testing.assert_allclose(got,expected,equal_nan=True,rtol=1e-11,atol=1e-11)
        replay.append(profile)
    # 将一组实际输出转为接口可读的相对/锚点位置，航向未作为本轮输出。
    a=np.load(OUT/'outputs/lio_增强_single_s0.10.npz')['position'][0]
    with (OUT/'单锚点位置输出.csv').open('w',newline='') as f:
        writer=csv.writer(f);writer.writerow(['time_s','node_id','frame_id','x_m','y_m','anchor_frame_id','anchor_x_m','anchor_y_m','position_valid','yaw_valid'])
        for step in range(start,len(times)):
            for n in range(3):
                rel=a[step,n]-a[step,0];writer.writerow([times[step],n+1,'coop_single_anchor_relative',*rel,'coop_single_anchor',*a[step,n],True,False])
    baseline=ROOT/'docs/报告/20261028_协同定位V1.4/内部材料/V1.3输入与算法基线校验.json'
    for filename,sha in json.loads(baseline.read_text()).items():assert hashlib.sha256((ROOT/filename).read_bytes()).hexdigest()==sha,filename
    with (OUT/'蒙特卡洛分组统计.csv').open('w',newline='') as f:
        w=csv.writer(f);w.writerow(['条件','输入','后续处理','测距标准差_米','重复次数','RMSE均值_米','RMSE样本标准差_米','最小RMSE_米','最大RMSE_米','RMSE小于1米次数'])
        for c in summary['conditions']:
            v=np.array([t['position_rmse_m'] for t in c['trials']])
            w.writerow([c['name'],c['profile'],c['mode'],c['range_sigma_m'],len(v),v.mean(),v.std(ddof=1),v.min(),v.max(),np.count_nonzero(v<1)])
    result={'conditions':84,'trials':checked,'anchor_count':1,'range_edges':6,'all_saved_output_rmse_recomputed':True,'replay_profiles':replay,'estimator_parameters_exclude_truth':'truth' not in inspect.signature(estimate).parameters,'post_trajectory_alignment':False,'original_algorithms_and_V13_unchanged':True,'output_rows':(len(times)-start)*3}
    (OUT/'独立结果核对.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n');print(json.dumps(result,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
