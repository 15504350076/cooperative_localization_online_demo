#!/usr/bin/env python3
"""只读复核RTE建系等待、逐次接口耗时及报告四类坐标轴。"""
import csv
import hashlib
import json
from pathlib import Path
import numpy as np
from evaluate_rte import ROOT
from evaluate_single_anchor import rotate
from evaluate_anchor_layout import PLACEMENTS

OUT=ROOT/'data/report_replay_20261029/初始化与坐标核对'
DATA=ROOT/'data/rte_expanded_20261025'


def main():
    OUT.mkdir(exist_ok=True);records=[];paths=[]
    for sigma in (.02,.05,.1):
        name=f'exp_odometry_{sigma:.2f}_seed0';path=OUT.parent/f'{name}_output.csv';paths.append(path)
        a=np.genfromtxt(path,delimiter=',',names=True);a=a[a['elapsed_s']<=300];first=float(a['elapsed_s'][a['status']==1][0]);before=a[a['elapsed_s']<first]
        records.append(dict(range_sigma_cm=int(sigma*100),first_valid_s=first,step_calls_before_valid=len(before),
            cumulative_step_wall_seconds=float(before['solve_ms'].sum()/1000),maximum_step_wall_ms=float(before['solve_ms'].max()),
            status_counts={int(n):int(np.count_nonzero(before['status']==n)) for n in np.unique(before['status'])}))
    source=DATA/'exp_odometry_0.02_seed0.csv';paths.append(source)
    data=np.genfromtxt(source,delimiter=',',names=True,dtype=None,encoding='utf-8');odom=[data[(data['kind']=='O')&(data['node_id']==n)] for n in (1,2,3)]
    a=np.genfromtxt(OUT.parent/'exp_odometry_0.02_seed0_output.csv',delimiter=',',names=True);a=a[a['elapsed_s']<=300];spans=[]
    for now in a['timestamp_ns']:
        span=0.
        for vehicle in odom:
            lo=np.searchsorted(vehicle['timestamp_ns'],now-5e9);hi=np.searchsorted(vehicle['timestamp_ns'],now,side='right');window=vehicle[lo:hi]
            if len(window)>1:span=max(span,float(np.hypot(np.ptp(window['v0']),np.ptp(window['v1']))))
        spans.append(span)
    spans=np.array(spans);good=spans>=10;changes=np.flatnonzero(np.diff(np.r_[False,good,False])).reshape(-1,2)
    intervals=[(float(a['elapsed_s'][i]),float(a['elapsed_s'][j-1])) for i,j in changes]
    np.testing.assert_allclose([r['first_valid_s'] for r in records],[127.2,215.52,216.24],atol=1e-9)
    assert records[0]['first_valid_s']>a['elapsed_s'][good][0]
    with (OUT/'五秒运动范围.csv').open('w',newline='') as stream:
        w=csv.writer(stream);w.writerow(['仿真时间_秒','三车最大五秒位置包络对角线_米','达到10米条件'])
        w.writerows(zip(a['elapsed_s'],spans,good.astype(int)))
    # 直接检查保存的仿真真值使用哪一原点和轴向，不对估计输出做配准。
    rawpath=ROOT/'data/single_anchor_outdoor_20261029/truth_and_range_inputs.npz';raw=np.load(rawpath);world=raw['world_truth'];paths.append(rawpath)
    origin=world[0,0];beta=np.arctan2(*(world[0,1]-origin)[::-1]);peer=np.load(ROOT/'data/single_anchor_layout_20261029/outputs/lio_增强_no_anchor_peer.npz')['truth']
    np.testing.assert_allclose(peer,rotate(world-origin,-beta),atol=1e-12)
    for name,point in PLACEMENTS.items():
        p=np.array(point);beta=np.arctan2(*(world[0,0]-p)[::-1]);expected=rotate(world-p,-beta)
        saved=np.load(ROOT/f'data/single_anchor_layout_20261029/outputs/lio_增强_anchor_{name}.npz')['truth']
        np.testing.assert_allclose(saved,expected,atol=1e-12)
    cpp=ROOT/'src/rte/three_vehicle_rte.cpp';runner=ROOT/'tools/evaluate_rte.cpp';paths.extend([cpp,runner])
    assert 'minimum_initialization_motion_span_m=10.0' in runner.read_text()
    assert 'h.transforms[i].point(aligned[i].position) - reference' in cpp.read_text()
    frames=dict(default_uwb='201当前位置为原点，201至202当刻连线为X，随队形改变',
        rte_raw_and_figure12='201当前位置为原点，轴平行201本车局部里程计系L1；图12的L1对应201初始局部轴',
        no_anchor_motion_model='相对位置以201当前位置为原点，轴沿仿真起点201至202连线并保持固定',
        single_anchor_motion_model='任务位置以锚点为原点，轴沿锚点至起点201方向固定；相对位置仅减去201当前位置，轴不变',
        optional_rte_adapter='首次有效历元201至202连线固定轴；不参与图12；任务表达另把原点留在首次有效201位置')
    result=dict(date='2026-10-29',window_seconds=5,minimum_recent_motion_span_m=10,
        motion_span_definition='对各车最近5秒局部位置取X/Y极差组成包络对角线，再取三车最大值；不是累计路程',
        earliest_motion_condition_s=float(a['elapsed_s'][good][0]),motion_condition_intervals_s=intervals,
        runs=records,time_definition='first_valid_s为数据的仿真时间；step耗时为开发机实测接口墙钟耗时，不含读文件和绘图',
        quality_check='运动范围满足后，继续检查收敛、几何条件、残差及初始平移0.25米/转角0.1弧度的不确定度；现有状态码不能区分各项约束不足的具体分支',
        coordinate_definitions=frames,saved_truth_frames_verified=True,algorithms_or_configuration_modified=False,
        source_sha256={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths})
    (OUT/'核对结果.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n');print(json.dumps(result,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
