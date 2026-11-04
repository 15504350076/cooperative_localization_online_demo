#!/usr/bin/env python3
"""逐次复算布点实验，重放无锚点和单锚点输入，检查旧算法及图1。"""
import hashlib
import csv
import inspect
import json
import numpy as np
from evaluate_anchor_layout import OUT, INPUTS, ROOT, PLACEMENTS, PROFILES, initialize_peer_only
from evaluate_single_anchor import estimate


def main():
    summary=json.loads((OUT/'summary.json').read_text());manifest=json.loads((OUT/'运行清单.json').read_text())
    assert len(summary['conditions'])==63 and manifest['total_trials']==1260
    for name,digest in manifest['hashes'].items():
        assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==digest,name
    checked=0;center_error=0.
    for case in summary['conditions']:
        a=np.load(OUT/'outputs'/f'{case["name"]}.npz');p=a['position'][:,600:];q=a['truth'][None,600:]
        error=(p[:,:,1:]-p[:,:,:1])-(q[:,:,1:]-q[:,:,:1])
        got=np.sqrt(np.mean(np.sum(error**2,axis=-1),axis=(1,2)))
        np.testing.assert_allclose(got,[r['position_rmse_m'] for r in case['trials']],atol=1e-12,rtol=1e-12)
        assert len(got)==20 and case['anchor_count']==(0 if case['placement'] is None else 1)
        if case['anchor_count']==0:assert all(d['anchor_measurements_used']==0 for d in case['initialization'])
        if case['placement']=='区域中部':
            old=np.load(INPUTS/'outputs'/f'{case["profile"]}_single_s0.10.npz')['position']
            center_error=max(center_error,float(np.nanmax(np.abs(old-a['position']))))
        checked+=len(got)
    raw=np.load(INPUTS/'truth_and_range_inputs.npz');peer=raw['range_true'][None,:,:3]+.1*raw['range_standard'][:1,:,:3]+raw['range_bias'][:1,:,:3]
    replay=[]
    for placement in (None,'区域右侧'):
        profile='lio_增强';sensor={k:v[:1] for k,v in dict(np.load(INPUTS/'inputs'/f'{profile}.npz')).items()}
        stem=f'{profile}_no_anchor_peer' if placement is None else f'{profile}_anchor_{placement}'
        initstem=f'{profile}_no_anchor_initialization' if placement is None else f'{profile}_{placement}_initialization'
        init=np.load(OUT/'outputs'/f'{initstem}.npz');measured=peer
        if placement:
            ranges=np.linalg.norm(raw['world_truth']-np.array(PLACEMENTS[placement]),axis=-1)
            measured=np.concatenate((peer,ranges[None]+.1*raw['range_standard'][:1,:,3:]+raw['range_bias'][:1,:,3:]),axis=-1)
        got,_,_=estimate(sensor,measured,PROFILES[profile],init['position'][:1],init['angle'][:1],raw['time_s'],.1,'peer' if placement is None else 'single')
        np.testing.assert_allclose(got,np.load(OUT/'outputs'/f'{stem}.npz')['position'][:1],equal_nan=True,atol=1e-11,rtol=1e-11);replay.append(stem)
    baseline=ROOT/'docs/报告/20261028_协同定位V1.4/内部材料/V1.3输入与算法基线校验.json'
    for name,digest in json.loads(baseline.read_text()).items():assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==digest,name
    image=ROOT/'docs/报告/20261028_协同定位V1.4/figures/图1_协同定位系统组成.png'
    assert hashlib.sha256(image.read_bytes()).hexdigest()=='a4b8725191f5958ed51468c702bedf98b80e18912f0ed66d4f02577353e0a4cc'
    assert 'truth' not in inspect.signature(initialize_peer_only).parameters
    assert center_error<1e-10
    with (OUT/'全部仿真结果.csv').open('w',newline='') as stream:
        writer=csv.writer(stream)
        writer.writerow(['条件','运动输入','后续处理','锚点位置','重复试验序号','相对位置RMSE_米','位置误差95分位_米','最大位置误差_米','任务局部位置RMSE_米'])
        for case in summary['conditions']:
            for index,row in enumerate(case['trials'],1):
                writer.writerow([case['name'],case['profile'],case['mode'],case['placement'],index,row['position_rmse_m'],row['position_p95_m'],row['position_max_m'],row['field_position_rmse_m']])
    result=dict(conditions=63,trials_recomputed=checked,maximum_anchor_count=1,no_anchor_initialization_uses_only_three_peer_ranges=True,
        replay=replay,center_placement_matches_previous_outputs_max_error_m=center_error,default_algorithms_and_V13_unchanged=True,user_figure1_sha256_verified=True)
    (OUT/'独立结果核对.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n');print(json.dumps(result,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
