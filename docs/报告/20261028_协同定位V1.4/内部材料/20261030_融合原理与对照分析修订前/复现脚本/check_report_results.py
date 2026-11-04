#!/usr/bin/env python3
"""按本版条件清单复算正文全部1680次位置RMSE与分组统计。"""
import json
import csv
from pathlib import Path
import numpy as np
from evaluate_rte import ROOT

REPORT=ROOT/'docs/报告/20261028_协同定位V1.4'
OUT=ROOT/'data/report_replay_20261029'


def main():
    index=json.loads((REPORT/'本版实验条件清单.json').read_text());count=0;rows=[];seen=set()
    assert len(index['conditions_index'])==84
    for case in index['conditions_index']:
        assert case['output_file'] not in seen;seen.add(case['output_file'])
        output=np.load(ROOT/case['output_file']);p=output['position'][:,600:]
        if 'truth' in output:q=output['truth']
        else:q=np.load(ROOT/case['source_directory']/'truth_and_range_inputs.npz')['truth']
        if case['profile']=='uwb':
            q2=q[:,1]-q[:,0];q3=q[:,2]-q[:,0];length=np.linalg.norm(q2,axis=-1);canonical=np.zeros_like(q)
            canonical[:,1,0]=length;canonical[:,2,0]=np.sum(q2*q3,axis=-1)/length
            canonical[:,2,1]=np.abs(q2[:,0]*q3[:,1]-q2[:,1]*q3[:,0])/length;q=canonical
        e=(p[:,:,1:]-p[:,:,:1])-(q[None,600:,1:]-q[None,600:,:1]);value=np.sqrt(np.mean(np.sum(e**2,axis=-1),axis=(1,2)))
        assert len(value)==20
        got=(value.mean(),value.std(ddof=1),value.min(),value.max());want=tuple(case[k] for k in ('mean_rmse_m','sample_std_rmse_m','minimum_rmse_m','maximum_rmse_m'))
        np.testing.assert_allclose(got,want,atol=1e-10,rtol=1e-10)
        source=json.loads((ROOT/case['source_directory']/'summary.json').read_text())
        original=next(c for c in source['conditions'] if c['name']==case['name'])
        np.testing.assert_allclose(value,[r['position_rmse_m'] for r in original['trials']],atol=1e-10,rtol=1e-10)
        for n,v in enumerate(value,1):rows.append([case['name'],n,float(v),case['output_file']])
        count+=len(value)
    assert count==index['trials']==1680
    replay=json.loads((OUT/'复现核对.json').read_text())
    assert all(c['replay_all_fields_except_elapsed_compute_time_identical'] for c in replay['rte'].values())
    np.testing.assert_allclose([c['first_valid_s'] for c in replay['rte'].values()],[127.2,215.52,216.24],atol=1e-9)
    assert replay['lidar']['selected_messages']==225 and replay['lidar']['plotted_points']==120000
    budget=json.loads((REPORT/'硬件配置概算.json').read_text());a,b,c=budget['configurations']
    assert a['complete_box_count']==3 and b['complete_box_count']==4 and c['imu_count']==3
    assert round(c['reference_unit_price_including_vat_yuan']*3,2)==c['reference_extended_price_yuan']
    with (OUT/'本版1680次误差复算.csv').open('w',newline='') as stream:
        writer=csv.writer(stream);writer.writerow(['条件','重复试验序号','复算RMSE_米','逐时输出文件']);writer.writerows(rows)
    result=dict(conditions_recomputed=84,trials_recomputed=count,trials_per_condition=20,all_reported_group_statistics_match_saved_outputs=True,
        actual_cpp_replays=3,cpp_all_fields_except_computation_duration_match=True,rte_first_valid_seconds=[127.2,215.52,216.24],
        lidar_messages=225,lidar_points_drawn=120000,budget_counts_and_reference_arithmetic_verified=True,simulation_outputs_modified=False)
    (OUT/'本版结果复算.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n');print(json.dumps(result,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
