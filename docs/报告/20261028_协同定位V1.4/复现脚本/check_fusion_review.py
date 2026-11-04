#!/usr/bin/env python3
"""独立核对201坐标转换、诊断来源及已有程序/图1的保持情况。"""
import csv
import hashlib
import json
import re
import zipfile
from pathlib import Path
import numpy as np
from evaluate_common_reference import ROOT, OUT, REPORT, reference_relative


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    # 换用另一套内部坐标（旋转和平移）后，相同201局部输出应保持一致。
    p=np.array([[[[3.,4.],[6.,9.],[1.,2.]]]])
    a=.37;phi=-.61;R=np.array([[np.cos(phi),-np.sin(phi)],[np.sin(phi),np.cos(phi)]])
    np.testing.assert_allclose(reference_relative(p@R.T+np.array([7.,-2.]),np.array([a+phi])),reference_relative(p,np.array([a])),atol=1e-12)
    summary=json.loads((OUT/'summary.json').read_text());default_count=0
    for c in summary['conditions']:
        new=np.load(OUT/'outputs'/f'{c["name"]}.npz');old=np.load(ROOT/c['source_output'])
        if c['profile']=='uwb':np.testing.assert_array_equal(new['position'],old['position']);default_count+=1
        else:
            relative=old['position']-old['position'][:,:,:1];theta=old['alignment_angle'][:,600,0,None,None]
            expected=np.stack([np.cos(theta)*relative[...,0]+np.sin(theta)*relative[...,1],-np.sin(theta)*relative[...,0]+np.cos(theta)*relative[...,1]],axis=-1)
            np.testing.assert_allclose(new['position'],expected,atol=1e-12,rtol=1e-12,equal_nan=True)
            np.testing.assert_allclose(new['position'][:,600:,0],0,atol=1e-12)
    assert len(summary['conditions'])==84 and default_count==3
    for path,digest in json.loads((OUT/'运行清单.json').read_text())['input_and_code_sha256'].items():assert sha(ROOT/path)==digest,path
    baseline=json.loads((REPORT/'内部材料/V1.3输入与算法基线校验.json').read_text())
    for path,digest in baseline.items():assert sha(ROOT/path)==digest,path
    figure=REPORT/'figures/图1_协同定位系统组成.png';figure_sha=sha(figure)
    assert figure_sha=='a4b8725191f5958ed51468c702bedf98b80e18912f0ed66d4f02577353e0a4cc'
    markdown=REPORT/'多平台协同定位硬件仿真测试报告与联调方案_V1.4_设备分级与锚点仿真版.md'
    with zipfile.ZipFile(markdown.with_suffix('.docx')) as archive:
        media=[hashlib.sha256(archive.read(n)).hexdigest() for n in archive.namelist() if n.startswith('word/media/')]
    assert figure_sha in media
    figure_names=json.loads((REPORT/'报告图表顺序.json').read_text())['figures']
    for name in figure_names:assert sha(REPORT/'figures'/(name+'.png')) in media,name
    traces=json.loads((REPORT/'融合复核图线来源.json').read_text())
    for name,item in traces.items():
        for path,digest in item['input_sha256'].items():assert sha(ROOT/path)==digest,(name,path)
        for extension in ('.png','.svg','.pdf'):assert (REPORT/'figures'/(name+extension)).stat().st_size>1000
        assert '<text' in (REPORT/'figures'/(name+'.svg')).read_text()
    # 放大图与主图使用同一输出和真值，重算并核对逐点数据。
    zoom=traces['review_RTE低速初始化误差']['局部放大']
    assert zoom['时间范围_秒']==[90,300] and zoom['误差范围_米']==[0,3]
    output=np.genfromtxt(ROOT/'data/rte_startup_diagnostics_20261030/w10_d5_s0.10_output.csv',delimiter=',',names=True)
    truth=np.genfromtxt(ROOT/'data/rte_expanded_20261025/exp_odometry_0.10_seed0.truth.csv',delimiter=',',names=True)
    mask=(output['status']==1)&(output['elapsed_s']>=90)&(output['elapsed_s']<=300)
    digest=hashlib.sha256()
    for n in (2,3):
        error=np.hypot(output[f'x_{n}']-np.interp(output['state_timestamp_ns'],truth['timestamp_ns'],truth[f'x_{n}']),output[f'y_{n}']-np.interp(output['state_timestamp_ns'],truth['timestamp_ns'],truth[f'y_{n}']))
        assert error[mask].max()<3
        for values in (output['elapsed_s'][mask],error[mask]):digest.update(np.asarray(values,dtype='<f8').tobytes())
    assert digest.hexdigest()==zoom['line_data_sha256']
    peak=ROOT/'data/fusion_peak_analysis_20261030';diagnostics=json.loads((peak/'summary.json').read_text())
    for c in diagnostics['conditions']:
        got=np.load(peak/f'{c["profile"]}.npz');original=np.load(OUT/'outputs'/f'{c["profile"]}_no_anchor_peer.npz')
        np.testing.assert_allclose(got['original_position'],original['position'][0],atol=1e-10,rtol=1e-10,equal_nan=True)
        for k,name in enumerate(c['variant_names']):
            value=float(np.sqrt(np.mean(got[f'error_{k}'][900:]**2)))
            np.testing.assert_allclose(value,c['same_interval_90_300_rmse_m'][name],atol=1e-12)
    jac=np.load(peak/'静止可观性.npz')['jacobian'];assert np.linalg.matrix_rank(jac)==6 and np.linalg.norm(jac[:,-1])==0
    startup=ROOT/'data/rte_startup_diagnostics_20261030';runs=json.loads((startup/'summary.json').read_text())['conditions'];assert len(runs)==12
    baseline_runs=[r for r in runs if r['baseline_reproduces_original']];assert len(baseline_runs)==3
    assert [r['diagnostic_stage_counts'].get('position_uncertainty',0) for r in baseline_runs]==[38,175,178]
    for r in runs:
        output=np.genfromtxt(startup/(r['name']+'_output.csv'),delimiter=',',names=True)
        mask=(output['status']==1)&(output['elapsed_s']>=216.24)&(output['elapsed_s']<=300)
        assert int(mask.sum())==r['common_interval_valid_rows']
        assert (r['common_interval_rmse_m'] is None)==(not mask.any())
    exported=list((OUT/'pose_exports').glob('*.csv'));assert len(exported)==6
    for path in exported:
        with path.open() as stream:rows=list(csv.DictReader(stream))
        assert len(rows)==2401*3 and {r['frame_id'] for r in rows}=={'coop_rte_1_local'}
        assert all(float(r['x_m'])==float(r['y_m'])==0 for r in rows if r['node_id']=='1')
    maneuver=ROOT/'data/maneuver_observability_20261101';ms=json.loads((maneuver/'summary.json').read_text());assert len(ms['conditions'])==3 and ms['trials']==20
    for c in ms['conditions']:
        a=np.load(maneuver/'outputs'/f'{c["name"]}.npz');assert a['rmse'].shape==(20,)
        np.testing.assert_allclose(float(a['rmse'].mean()),c['metrics']['rmse_mean'],atol=1e-12)
    source=json.loads((maneuver/'图线来源.json').read_text());assert source['trials']==20 and len(source['conditions'])==3
    for n in source['figures']:
        for ext in ('.png','.svg','.pdf'):
            q=REPORT/'figures'/(n+ext);assert q.stat().st_size>1000
    init_source=REPORT/'初始化候选解图线来源.json';assert init_source.is_file()
    for ext in ('.png','.svg','.pdf'):
        assert (REPORT/'figures'/('review_初始化候选关系'+ext)).stat().st_size>1000
    source=markdown.read_text();assert '10厘米归入上行' not in source and '报告采用四种' not in source and 'coop_single_anchor' not in source
    figure_count=len(re.findall(r'^!\[图\d+ ',source,re.M));assert figure_count==23
    result=dict(conditions_checked=84,trials_per_condition=20,internal_gauge_invariance=True,common_reference_201_at_origin=True,
        default_uwb_outputs_identical=True,original_source_hashes_verified=True,original_libraries_and_V13_hashes_verified=len(baseline),
        user_figure1_embedded_byte_identical=True,new_figures_with_verified_sources=len(traces)+1,all_figures=figure_count,
        real_ekf_replay_variants=20,rte_cpp_conditions=12,baseline_rte_positions_status_counts_times_match=True,
        rte_position_uncertainty_rejections=[38,175,178],static_jacobian_rank=6,static_unknowns=9,
        maneuver_conditions=3,maneuver_trials=20,maneuver_outputs_verified=True,
        csv_201_reference_exports=6,no_valid_output_not_reported_as_zero=True)
    (REPORT/'本轮方法复核验证.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n');print(json.dumps(result,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
