#!/usr/bin/env python3
"""独立重算逐次RMSE、核对分组计数、运动学和历史算法基线。"""
import csv
from collections import Counter
import hashlib
import json
import numpy as np
from evaluate_sensor_anchor_v14 import OUT, ROOT


def relative_rmse(estimate,truth):
    difference=(estimate[:,1:]-estimate[:,:1])-(truth[:,1:]-truth[:,:1])
    return float(np.sqrt(np.mean(np.sum(difference**2,axis=-1))))


def main():
    primary=json.loads((OUT/'summary.json').read_text());sens=json.loads((OUT/'sensitivity/summary.json').read_text())
    rte=json.loads((OUT/'rte_evaluation/summary.json').read_text())
    with (OUT/'全部仿真结果.csv').open() as stream:rows=list(csv.DictReader(stream))
    counts=Counter(r['condition'] for r in rows)
    assert len(rows)==1400 and len(counts)==70 and set(counts.values())=={20}
    raw=np.load(OUT/'sensor_inputs.npz');rte_raw=np.load(OUT/'rte_evaluation/sensor_inputs.npz')
    for key in ('time_s','truth','yaw','velocity','vio_local','lio_local','peer_standard','anchor_standard'):
        np.testing.assert_array_equal(raw[key],rte_raw[key],err_msg=f'RTE与场地组未使用相同输入：{key}')
    velocity,yaw,truth,time=(raw[k] for k in ('velocity','yaw','truth','time_s'))
    lateral=-velocity[:,:,0]*np.sin(yaw)+velocity[:,:,1]*np.cos(yaw)
    assert abs(lateral).max()<1e-7 and np.linalg.norm(velocity,axis=-1).max()<3
    distances=np.stack([np.linalg.norm(truth[:,i]-truth[:,j],axis=-1) for i,j in ((0,1),(0,2),(1,2))])
    assert distances.min()>5 and distances.max()<30
    assert np.max(np.abs(np.gradient(yaw,time,axis=0)))<.5
    checked=0
    for folder,summary in [(OUT,primary),(OUT/'sensitivity',sens)]:
        for case in summary['conditions']:
            a=np.load(folder/(case['name']+'.npz'))
            value=relative_rmse(a['position'],a['truth'])
            assert abs(value-case['trials'][0]['position_rmse_m'])<1e-10,(case['name'],value)
            assert len(case['trials'])==20
            checked+=1
    truth_relative=truth-truth[:,:1]
    stamps=10_000_000_000+np.arange(len(time))*20_000_000
    for case in rte['rte']:
        a=np.genfromtxt(OUT/'rte_evaluation/rte'/f'{case["name"]}_output.csv',delimiter=',',names=True)
        a=a[a['elapsed_s']<=300+1e-6];good=a['status']==1;errors=[]
        for n in (2,3):
            gt=np.stack([np.interp(a['state_timestamp_ns'],stamps,truth_relative[:,n-1,axis]) for axis in (0,1)],axis=-1)
            estimate=np.stack([a[f'x_{n}'],a[f'y_{n}']],axis=-1)
            errors.extend(np.sum((estimate[good]-gt[good])**2,axis=-1).tolist())
        value=float(np.sqrt(np.mean(errors))) if errors else None
        assert (value is None and case['position_rmse_m'] is None) or abs(value-case['position_rmse_m'])<1e-9,case['name']
        checked+=1
    report=ROOT/'docs/报告/20261028_协同定位V1.4'
    baseline=json.loads((report/'内部材料/V1.3输入与算法基线校验.json').read_text())
    for file,sha in baseline.items():assert hashlib.sha256((ROOT/file).read_bytes()).hexdigest()==sha,file
    audit=json.loads((OUT/'jump_analysis/summary.json').read_text())['top_events'][0]
    assert abs(audit['window_recomputed_rms_m']-audit['residual_after_m'])<1e-5
    labels=json.loads((report/'图线坐标轴清单.json').read_text())
    assert all(ax['横轴'] and ax['纵轴'] for axes in labels.values() for ax in axes)
    result=dict(conditions=70,trial_rows=1400,trials_per_condition=20,recomputed_saved_examples_and_rte_runs=checked,
        maximum_speed_mps=float(np.linalg.norm(velocity,axis=-1).max()),maximum_yaw_rate_rps=float(np.abs(np.gradient(yaw,time,axis=0)).max()),
        maximum_lateral_speed_mps=float(abs(lateral).max()),min_vehicle_spacing_m=float(distances.min()),max_vehicle_spacing_m=float(distances.max()),
        paired_inputs_identical=True,old_algorithms_and_v13_unchanged=True,all_generated_axes_labeled=True,
        jump_window_residual_independently_reproduced=True)
    (OUT/'独立结果核对.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(result,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
