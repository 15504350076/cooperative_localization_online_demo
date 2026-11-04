#!/usr/bin/env python3
"""将已保存融合输出统一为201局部初始轴；只用估计初始转角转换输出。"""
import csv
import hashlib
import json
from pathlib import Path
import numpy as np
from evaluate_rte import ROOT
from evaluate_single_anchor import rotate, score, wrap

OUT=ROOT/'data/common_reference_20261030'
REPORT=ROOT/'docs/报告/20261028_协同定位V1.4'
INDEX=REPORT/'内部材料/20261030_融合原理与对照分析修订前/本版实验条件清单.json'


def reference_relative(position, initial_reference_angle):
    """不接收真值；201同刻位置作原点，201自身局部输入轴作固定轴。"""
    return rotate(position-position[:,:,:1],-initial_reference_angle[:,None,None])


def main():
    OUT.mkdir(exist_ok=True);(OUT/'outputs').mkdir(exist_ok=True)
    selection=OUT/'原始条件选择.json'
    if not selection.exists():selection.write_bytes(INDEX.read_bytes())
    index=json.loads(selection.read_text());rawpath=ROOT/'data/single_anchor_outdoor_20261029/truth_and_range_inputs.npz';raw=np.load(rawpath)
    world=raw['world_truth'];truth=rotate(world-world[:,:1],-raw['yaw'][0,0]);conditions=[];hashes={}
    for item in index['conditions_index']:
        source=ROOT/item['output_file'];a=np.load(source);profile=item['profile'];mode=item['mode'];angles=None
        if profile=='uwb':
            position=a['position'];q2=world[:,1]-world[:,0];q3=world[:,2]-world[:,0];length=np.linalg.norm(q2,axis=-1);target=np.zeros_like(world)
            target[:,1,0]=length;target[:,2,0]=np.sum(q2*q3,axis=-1)/length;target[:,2,1]=np.abs(q2[:,0]*q3[:,1]-q2[:,1]*q3[:,0])/length;initial=np.zeros(20)
        else:
            initial=a['alignment_angle'][:,600,0];assert np.isfinite(initial).all()
            position=reference_relative(a['position'],initial);target=truth;angles=wrap(a['alignment_angle']-initial[:,None,None])
            for i,j in ((0,1),(0,2),(1,2)):
                np.testing.assert_allclose(np.linalg.norm(position[:,600:,i]-position[:,600:,j],axis=-1),np.linalg.norm(a['position'][:,600:,i]-a['position'][:,600:,j],axis=-1),rtol=1e-11,atol=1e-11)
            np.testing.assert_allclose(position[:,600:,0],0.,atol=1e-12);np.testing.assert_allclose(angles[:,600,0],0.,atol=1e-12)
        values=dict(time_s=a['time_s'],position=position,truth=target,initial_reference_alignment_angle=initial)
        if angles is not None:values.update(alignment_angle=angles,correction_m=a['correction_m'])
        np.savez_compressed(OUT/'outputs'/f'{item["name"]}.npz',**values)
        source_summary=json.loads((ROOT/item['source_directory']/'summary.json').read_text());old=next(c for c in source_summary['conditions'] if c['name']==item['name'])
        trials=score(position,target,600)
        for row in trials:row.pop('field_position_rmse_m')
        conditions.append(dict(name=item['name'],profile=profile,mode=mode,placement=old.get('placement'),range_sigma_m=old.get('range_sigma_m',.1),anchor_count=old['anchor_count'],
            source_output=item['output_file'],frame='原默认UWB瞬时队形' if profile=='uwb' else '201当前位置原点/201自身初始局部里程计固定轴',trials=trials))
        hashes[item['output_file']]=hashlib.sha256(source.read_bytes()).hexdigest()
    summary=dict(date='2026-10-30',conditions=conditions,condition_count=len(conditions),total_trials=sum(len(c['trials']) for c in conditions),
        conversion='R(-估计的201初始局部转角)乘各车减201同刻位置；固定转角仅取初始化结果，不使用真值配准',
        truth_role='仅评价器用真实201初始方向生成相应真值，未传入输出转换函数',default_uwb_unchanged=True,original_algorithm_outputs_unchanged=True)
    assert summary['condition_count']==84 and summary['total_trials']==1680
    (OUT/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
    with (OUT/'全部仿真结果.csv').open('w',newline='') as stream:
        w=csv.writer(stream);w.writerow(['条件','运动输入','模式','锚点位置','测距标准差_米','重复试验','相对位置RMSE_米'])
        for c in conditions:
            for k,r in enumerate(c['trials'],1):w.writerow([c['name'],c['profile'],c['mode'],c['placement'],c['range_sigma_m'],k,r['position_rmse_m']])
    hashes[str(rawpath.relative_to(ROOT))]=hashlib.sha256(rawpath.read_bytes()).hexdigest();hashes[str(Path(__file__).relative_to(ROOT))]=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    exported=OUT/'pose_exports';exported.mkdir(exist_ok=True)
    for profile in ('imu_增强','vio_增强','lio_增强'):
        for suffix in ('no_anchor_peer','anchor_区域中部'):
            name=f'{profile}_{suffix}';a=np.load(OUT/'outputs'/f'{name}.npz')
            with (exported/f'{name}_trial1.csv').open('w',newline='') as f:
                w=csv.writer(f);w.writerow(['simulation_time_s','frame_id','reference_node_id','node_id','x_m','y_m','position_valid','yaw_valid'])
                for time,positions in zip(a['time_s'][600:],a['position'][0,600:]):
                    for n,xy in enumerate(positions,1):w.writerow([time,'coop_rte_1_local',1,n,*xy,1,0])
    (OUT/'运行清单.json').write_text(json.dumps(dict(conditions=84,trials=1680,input_and_code_sha256=hashes),ensure_ascii=False,indent=2)+'\n')
    print('统一201参考坐标：84条件1680次完成，所有车间距离保持，估计转换未使用真值')


if __name__=='__main__':main()
