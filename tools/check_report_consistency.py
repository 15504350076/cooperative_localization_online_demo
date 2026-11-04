#!/usr/bin/env python3
"""核对报告的章节、图表编号、主要统计和真实接口口径。"""
import json
import re
from pathlib import Path
import numpy as np
from evaluate_single_anchor import PROFILES

ROOT = Path(__file__).resolve().parents[1]
REPORT = ROOT / 'docs/报告/20261028_协同定位V1.4'
SOURCE = REPORT / '多平台协同定位硬件仿真测试报告与联调方案_V1.4_设备分级与锚点仿真版.md'


def table(text, number):
    match = re.search(rf'^表{number} [^\n]+\n\n((?:\|[^\n]*\n)+)', text, re.M)
    assert match, f'表{number}不存在'
    rows = [[x.strip() for x in line.strip('|').split('|')] for line in match[1].strip().splitlines()]
    return rows[0], rows[2:]


def main():
    text = SOURCE.read_text()
    figures = re.findall(r'^!\[图(\d+) ([^\]]+)\]\(([^)]+)\)', text, re.M)
    tables = re.findall(r'^表(\d+) ([^\n]+)$', text, re.M)
    assert [int(n) for n, *_ in figures] == list(range(1, len(figures)+1))
    assert [int(n) for n, *_ in tables] == list(range(1, len(tables)+1))
    assert all((REPORT / path).is_file() for _, _, path in figures)
    manifest=json.loads((REPORT/'报告图表顺序.json').read_text())
    assert [Path(path).stem for _, _, path in figures]==manifest['figures']
    assert [title for _,title in tables]==manifest['table_titles']
    assert all(1 <= int(n) <= len(figures) for n in re.findall(r'图(\d+)', text))
    assert all(1 <= int(n) <= len(tables) for n in re.findall(r'表(\d+)', text))
    sections = re.findall(r'^### (\d+\.\d+) ([^\n]+)', text, re.M)
    for c in range(1, 7):
        nums = [int(n.split('.')[1]) for n, _ in sections if n.startswith(f'{c}.')]
        assert nums == list(range(1, len(nums)+1)), (c, nums)
    assert [(n,t) for n,t in sections if n.startswith('3.')] == [
        ('3.1','单锚点测量与初始关系'),('3.2','增加一个锚点后的精度变化'),
        ('3.3','机动对相对定位的影响'),('3.4','锚点位置的影响'),('3.5','测距精度与RTE对照')]
    for start,end,figure_names in [
        ('### 3.3 ','### 3.4 ',('平行与非平行直线轨迹','不同机动方式的三车轨迹')),
        ('### 3.4 ','### 3.5 ',('三车轨迹与候选布点','单锚点辅助估计与真值轨迹')),
        ('### 3.5 ','## 4 ',('测距与运动输入质量的影响','RTE低速初始化后的逐车位置误差'))]:
        block=text.split(start,1)[1].split(end,1)[0]
        assert all(name in block for name in figure_names),(start,figure_names)
    vio = text.split('### 2.3 ', 1)[1].split('### 2.4 ', 1)[0]
    lio = text.split('### 2.4 ', 1)[1].split('### 2.5 ', 1)[0]
    joint = text.split('### 2.5 ', 1)[1].split('## 3 ', 1)[0]
    assert 'VIO' in vio and 'LIO' not in vio and 'LIO' in lio and 'VIO' not in lio
    assert 'VIO' in joint and 'LIO' in joint and '表13 VIO与LIO' in joint

    data = json.loads((ROOT/'data/common_reference_20261030/summary.json').read_text())
    cases = {x['name']: np.array([t['position_rmse_m'] for t in x['trials']]) for x in data['conditions']}
    span = lambda x: f'{x.min():.3f}～{x.max():.3f}'
    avg = lambda x: f'{x.mean():.3f}±{x.std(ddof=1):.3f}'
    _, rows = table(text, 3)
    for row, sigma in zip(rows, (0.20, 0.10, 0.05)):
        v=cases[f'uwb_s{sigma:.2f}']
        assert row == [str(round(sigma*100)),avg(v),span(v)]
    _, rows = table(text, 4)
    for row,g in zip(rows,('基础','增强','精细')):
        cfg=PROFILES['imu_'+g]
        assert row == [g, f'{cfg["accel_bias"]:g}', f'{cfg["gyro_bias"]*180/np.pi*3600:.2f}', f'{cfg["tilt_std_deg"]:g}']
    _, rows=table(text,5)
    parameters=('accel_bias_walk','gyro_bias_walk','accel_white','gyro_white')
    for row,key in zip(rows,parameters):
        for val,g in zip(row[1:],('基础','增强','精细')):
            assert np.isclose(float(val),PROFILES['imu_'+g][key],rtol=1e-5), (5,key,val,g)
    for number,kind in ((8,'vio'),(11,'lio')):
        _,rows=table(text,number)
        for row,g in zip(rows,('基础','增强','精细')):
            cfg=PROFILES[kind+'_'+g]
            assert row==[g,f'{cfg["scale_std"]*100:g}',f'{cfg["yaw_drift_std"]:g}',f'{cfg["displacement_noise"]*1000:g}']
    _, rows=table(text,7)
    for row,g in zip(rows,('基础','增强','精细')):
        vals=[]
        for mode in ('start','peer'):
            a=np.load(ROOT/f'data/common_reference_20261030/outputs/imu_{g}_no_anchor_{mode}.npz')
            p,q,t=a['position'],a['truth'],a['time_s']; d=(p[:,:,1:]-p[:,:,:1])-(q[None,:,1:]-q[None,:,:1])
            e=np.sqrt(np.mean(np.sum(d*d,axis=-1),axis=-1)).mean(axis=0); active=(t>=60)&(t<=300)
            vals.append((np.polyfit(t[active],e[active],1)[0],e[-1]))
        assert row==[g,f'{vals[0][0]:.3f}',f'{vals[1][0]:.3f}',f'{vals[0][1]:.2f}',f'{vals[1][1]:.2f}']
    for number, kind in ((6,'imu'), (9,'vio'), (12,'lio')):
        _, rows = table(text, number)
        for row, grade in zip(rows, ('基础','增强','精细')):
            start, peer = cases[f'{kind}_{grade}_no_anchor_start'], cases[f'{kind}_{grade}_no_anchor_peer']
            assert row == [grade, span(start), span(peer), avg(peer)], (number, row)
    _, rows = table(text, 16)
    for row, profile in zip(rows, (f'{kind}_{g}' for kind in ('imu','vio','lio') for g in ('基础','增强','精细'))):
        peer, anchor = cases[profile+'_no_anchor_peer'], cases[profile+'_anchor_区域中部']
        assert row[1:] == [span(peer), span(anchor), avg(anchor)], (16, row)
    _,rows=table(text,17)
    options=[('vio_增强','no_anchor_peer'),('vio_精细','no_anchor_peer'),('vio_增强','anchor_区域中部'),('lio_增强','anchor_区域中部')]
    for row,(profile,mode) in zip(rows,options):assert row[1]==span(cases[profile+'_'+mode])
    geometry=json.loads((ROOT/'data/single_anchor_layout_20261029/summary.json').read_text())['placement_geometry']
    _,rows=table(text,21)
    for row in rows:
        g=geometry[row[0]]
        assert row[2:]==[f'{g["minimum_range_m"]:.2f}～{g["maximum_range_m"]:.2f}',f'{g["median_five_second_bearing_span_deg"]:.2f}']
    _, rows = table(text,22)
    for row in rows:
        assert row[1:] == [span(cases[f'{kind}_增强_anchor_{row[0]}']) for kind in ('imu','vio','lio')]
    _, rows = table(text,23)
    for row, profile in zip(rows, (f'{kind}_{g}' for kind in ('imu','vio','lio') for g in ('基础','增强','精细'))):
        names = [profile+'_single_s0.20', profile+'_anchor_区域中部', profile+'_single_s0.05']
        assert row[1:] == [span(cases[name]) for name in names], (23, row)
    diagnostics=json.loads((ROOT/'data/fusion_peak_analysis_20261030/summary.json').read_text())
    diag={r['profile']:r['same_interval_90_300_rmse_m'] for r in diagnostics['conditions']}
    variants=['原始输入','去除后续方向漂移','去除后续测距随机项','方向过程方差扩大100倍','90秒测量建系']
    header, rows=table(text,13)
    assert header==['对照条件','VIO基础（米）','VIO增强／LIO基础（米）','VIO精细／LIO增强（米）']
    precise=json.loads((ROOT/'data/fusion_peak_vio_precise_20261102/summary.json').read_text())['conditions'][0]
    assert precise['profile']=='vio_精细'
    for row, variant in zip(rows,variants):
        assert row[1:]==[f'{diag[p][variant]:.3f}' for p in ('vio_基础','vio_增强','lio_增强')]
        np.testing.assert_allclose(diag['vio_增强'][variant],diag['lio_基础'][variant],atol=1e-10)
        np.testing.assert_allclose(precise['same_interval_90_300_rmse_m'][variant],diag['lio_增强'][variant],atol=1e-10)
    _,rows=table(text,10)
    peaks=next(x for x in diagnostics['conditions'] if x['profile']=='vio_增强')['peaks'][:2]
    for row,x in zip(rows,peaks):
        assert row==[f'{x["time_s"]:.1f}',f'{x["error_m"]:.3f}',f'{x["common_rotation_error_deg"]:.2f}',f'{x["remaining_shape_error_m"]:.3f}',f'{x["maximum_peer_range_m"]:.2f}',f'{x["minimum_triangle_altitude_m"]:.2f}']
    startup=json.loads((ROOT/'data/rte_startup_diagnostics_20261030/summary.json').read_text())['conditions']
    _,rows=table(text,24)
    for row,sigma in zip(rows,(.02,.05,.10)):
        c=next(x for x in startup if x['name']==f'w10_d5_s{sigma:.2f}')
        assert row==[f'{sigma*100:.0f}',f'{c["first_valid_s"]:.2f}',f'{c["metrics"]["position_rmse_m"]:.3f}']
    _,rows=table(text,25)
    assert sum(int(r[2].replace(',','')) for r in rows)==3229145
    assert sum(int(r[3].replace(',','')) for r in rows)==48985
    maneuver=json.loads((ROOT/'data/maneuver_observability_20261101/summary.json').read_text())
    _, rows=table(text,19)
    for row, case in zip(rows,maneuver['conditions']):
        a=np.load(ROOT/f'data/maneuver_observability_20261101/outputs/{case["name"]}.npz')
        np.testing.assert_allclose(np.sqrt(np.mean(a['errors'][:,600:]**2,axis=1)),a['rmse'],atol=1e-10)
        if case['name']=='当前相对机动':np.testing.assert_allclose(a['rmse'],cases['vio_增强_anchor_区域中部'],atol=1e-8)
        m=case['metrics']; expected=[case['name'],f'{m["relative_speed_max"]:.3f}',f'{m["min_sv_median"]:.3f}',f'{m["rmse_mean"]:.3f}±{m["rmse_std"]:.3f}',f'{m["rmse_min"]:.3f}～{m["rmse_max"]:.3f}']
        assert row==expected,(19,row,expected)
    types=json.loads((ROOT/'data/maneuver_types_20261102/summary.json').read_text())
    _,rows=table(text,20)
    assert len(rows)==len(types['conditions'])==4 and types['trials']==20
    common=None
    for row,case in zip(rows,types['conditions']):
        a=np.load(ROOT/f'data/maneuver_types_20261102/outputs/{case["name"]}.npz')
        delta=a['estimate_ref'][:,:,1:]-a['truth_ref'][None,:,1:]
        np.testing.assert_allclose(np.sqrt(np.mean(np.sum(delta**2,axis=-1),axis=-1)),a['errors'],atol=1e-10)
        np.testing.assert_allclose(np.sqrt(np.mean(a['errors'][:,600:]**2,axis=1)),a['rmse'],atol=1e-10)
        assert a['rmse'].shape==(20,) and np.linalg.norm(a['velocity'],axis=-1).max()<3
        velocity=a['velocity'];acceleration=np.gradient(velocity,0.1,axis=0)
        pairs=((0,1),(0,2),(1,2))
        relspeed=np.stack([np.linalg.norm(velocity[:,i]-velocity[:,j],axis=-1) for i,j in pairs],axis=-1).max(axis=-1)
        relacc=np.stack([np.linalg.norm(acceleration[:,i]-acceleration[:,j],axis=-1) for i,j in pairs],axis=-1).max(axis=-1)
        np.testing.assert_allclose(relspeed,a['rel_speed'],atol=1e-12)
        np.testing.assert_allclose(relacc,a['rel_acc'],atol=1e-12)
        np.testing.assert_allclose([relspeed[800:].max(),relacc[800:].max(),np.nanmax(a['heading_diff_deg'][800:])],
            [case['max_relative_speed'],case['max_relative_acceleration'],case['max_velocity_angle_deg']],atol=1e-12)
        prefix=[a['measured'][:,:601],a['local'][:,:601],a['initial_position'],a['initial_angle']]
        if common is None: common=prefix
        else:
            for x,y in zip(common,prefix): np.testing.assert_allclose(x,y,atol=1e-9,rtol=1e-9)
        expected=[case['name'],f'{case["max_relative_speed"]:.3f}',f'{case["max_relative_acceleration"]:.3f}',f'{case["max_velocity_angle_deg"]:.1f}',f'{case["rmse_min"]:.3f}～{case["rmse_max"]:.3f}']
        assert row==expected,(20,row,expected)
        np.testing.assert_allclose([a['rmse'].mean(),a['rmse'].min(),a['rmse'].max()], [case['rmse_mean'],case['rmse_min'],case['rmse_max']],atol=1e-12)
    parallel=np.load(ROOT/'data/maneuver_types_20261102/outputs/同向平行.npz')
    relative=parallel['world'][:,1:]-parallel['world'][:,:1]
    bearing=np.unwrap(np.arctan2(relative[:,:,1],relative[:,:,0]),axis=0)
    assert np.all(np.ptp(bearing[300:601],axis=0)>0.1)
    np.testing.assert_allclose(relative[800:],np.broadcast_to(relative[800],relative[800:].shape),atol=1e-10)
    assert f'{parallel["rmse"].min():.3f}～{parallel["rmse"].max():.3f}米' in text
    comparisons=[]
    for name in ('低相对机动','当前相对机动','高相对机动'):
        x=np.load(ROOT/f'data/maneuver_observability_20261101/outputs/{name}.npz')
        index=int(np.argmin(abs(x['time_s']-146.5)))
        assert abs(x['time_s'][index]-146.5)<1e-9
        comparisons.append(float(x['errors'][:,index].mean()))
    assert '0.278、0.428和0.511米' in text
    assert [f'{v:.3f}' for v in comparisons]==['0.278','0.428','0.511']
    anchor=json.loads((ROOT/'data/single_anchor_outdoor_20261029/summary.json').read_text())
    assert anchor['anchor_world_position_m']==[55.,75.] and anchor['range_frequency_hz']==10 and anchor['link_count']==6
    assert anchor['main_range_sigma_m']==.1 and anchor['range_bias_std_m']==.03
    assert '中部单锚点' in text.split('### 3.3 ',1)[1].split('### 3.4 ',1)[0]
    assert '201当前位置为原点、201自身起点局部轴' in text.split('### 3.3 ',1)[1].split('### 3.4 ',1)[0]
    assert '201当前位置为原点' in maneuver['evaluation_frame']
    assert 'coop_rte_1_local' in text and 'coop_rte_1_relative_fixed' in text
    viewer=(ROOT/'tools/rte_coordinate_viewer.py').read_text()
    assert "beta=math.atan2(second['y'], second['x'])" in viewer
    assert "relative_frame=f'coop_rte_{self.ids[0]}_relative_fixed'" in viewer
    assert all(x in text for x in ('图3、图14、图16和图18展示20次结果的平均过程','无锚点时比较车间距离；增加锚点后，同时比较车间和车锚距离'))
    from analyze_straight_maneuvers import sensitivity
    straight=json.loads((ROOT/'data/straight_maneuvers_20261102/summary.json').read_text())
    _,errors=table(text,18)
    assert len(straight['conditions'])==len(errors)==3 and straight['trials']==20
    for c,error_row,expected_rank in zip(straight['conditions'],errors,((5,8),(5,8),(6,8))):
        a=np.load(ROOT/f'data/straight_maneuvers_20261102/outputs/{c["name"]}.npz')
        np.testing.assert_allclose(a['velocity'][402:-1],np.broadcast_to(a['velocity'][500],a['velocity'][402:-1].shape),atol=1e-9)
        assert np.linalg.norm(a['velocity'],axis=-1).max()<3 and a['range_truth'][:,:3].max()<30
        np.testing.assert_allclose(a['peer_measured'],a['single_measured'][:,:,:3],atol=0)
        for mode in ('peer','single'):
            d=a[mode+'_estimate_ref'][:,:,1:]-a['truth_ref'][None,:,1:]
            e=np.sqrt(np.mean(np.sum(d*d,axis=-1),axis=-1))
            np.testing.assert_allclose(e,a[mode+'_errors'],atol=1e-10)
            rmse=np.sqrt(np.mean(e[:,600:]**2,axis=1));np.testing.assert_allclose(rmse,a[mode+'_rmse'],atol=1e-10)
            np.testing.assert_allclose([rmse.mean(),rmse.min(),rmse.max()],[c[mode]['rmse_mean'],c[mode]['rmse_min'],c[mode]['rmse_max']],atol=1e-10)
            obs=sensitivity(a['world'],a['yaw'],np.array(straight['anchor']),mode)
            assert obs==c[mode]['sensitivity']
            gt=a['truth_ref'][600,1:];pred=a[mode+'_estimate_ref'][:,600,1:]
            assert np.count_nonzero(np.cross(pred[:,0],pred[:,1])*np.cross(gt[0],gt[1])<0)==c[mode]['mirrored_at_60_count']
        assert (c['peer']['sensitivity']['rank'],c['single']['sensitivity']['rank'])==expected_rank
        assert error_row==[c['name'],f'{c["peer"]["rmse_mean"]:.3f}',f'{c["peer"]["rmse_min"]:.3f}～{c["peer"]["rmse_max"]:.3f}',f'{c["single"]["rmse_mean"]:.3f}',f'{c["single"]["rmse_min"]:.3f}～{c["single"]["rmse_max"]:.3f}']
    result={'章节与图表编号':'通过','图表文件':len(figures),'编号表格':len(tables),
            'VIO与LIO章节归属':'通过','已按来源逐行核对的表格':[3,4,5,6,7,8,9,10,11,12,13,16,17,18,19,20,21,22,23,24,25],
            '坐标转换代码与报告说明':'通过','机动场景与统计口径':'通过'}
    (REPORT/'本轮全文一致性检查.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(result,ensure_ascii=False,indent=2))

if __name__=='__main__': main()
