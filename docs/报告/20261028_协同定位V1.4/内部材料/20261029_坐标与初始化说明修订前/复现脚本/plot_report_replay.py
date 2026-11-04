#!/usr/bin/env python3
"""重放真实C++输出、标出首次有效时刻，并从原始雷达包绘图。"""
import argparse
import hashlib
import json
import subprocess
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from evaluate_rte import ROOT
import plot_sensor_anchor_v14 as drawing

OUT=ROOT/'data/report_replay_20261029'
DEST=ROOT/'docs/报告/20261028_协同定位V1.4/figures'
DATA=ROOT/'data/rte_expanded_20261025'


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--replay-rte',action='store_true');args=parser.parse_args()
    OUT.mkdir(exist_ok=True);drawing.DEST=DEST;records={};hashes={};starts=[]
    fig,axes=plt.subplots(3,1,figsize=(11,9),sharex=True)
    for ax,sigma in zip(axes,(.02,.05,.1)):
        name=f'exp_odometry_{sigma:.2f}_seed0';source=DATA/f'{name}.csv';old=DATA/f'{name}_output.csv';output=OUT/f'{name}_output.csv'
        if args.replay_rte:
            subprocess.run([str(ROOT/'build_rte_verify/evaluate_rte'),str(source),str(output)],check=True)
        used_replay=output.exists()
        if used_replay:
            a=np.genfromtxt(output,delimiter=',',names=True);previous=np.genfromtxt(old,delimiter=',',names=True)
            assert a.shape==previous.shape
            for field in a.dtype.names:
                if field!='solve_ms':np.testing.assert_allclose(a[field],previous[field],rtol=1e-11,atol=1e-10,equal_nan=True,err_msg=field)
        else:a=np.genfromtxt(old,delimiter=',',names=True)
        a=a[a['elapsed_s']<=300];truth=np.genfromtxt(DATA/f'{name}.truth.csv',delimiter=',',names=True)
        valid=a['status']==1;first=float(a['elapsed_s'][valid][0]);starts.append(first);errors=[]
        for n,color in zip((2,3),drawing.C):
            error=np.hypot(a[f'x_{n}']-np.interp(a['state_timestamp_ns'],truth['timestamp_ns'],truth[f'x_{n}']),a[f'y_{n}']-np.interp(a['state_timestamp_ns'],truth['timestamp_ns'],truth[f'y_{n}']))
            ax.plot(a['elapsed_s'],np.where(valid,error,np.nan),color=color,lw=.9,label=f'{200+n}车相对位置误差');errors.append(error[valid])
        ax.axhline(1,color='#a3483c',ls='--',lw=.8,label='1米参考线');ax.axvspan(0,first,color='#ddd',alpha=.35)
        ax.axvline(first,color='#777',ls=':',lw=.8);ax.text(.015,.71,f'灰色：建立初始坐标关系\n首次有效输出：{first:.2f}秒',transform=ax.transAxes,fontsize=11)
        ax.set(ylabel='位置误差（米）',title=f'测距误差标准差{sigma*100:g}厘米',xlim=(0,300));ax.legend(ncol=3,loc='upper right')
        records[name]=dict(first_valid_s=first,position_rmse_m=float(np.sqrt(np.mean(np.square(errors)))),source=str(source.relative_to(ROOT)),saved_output=str(old.relative_to(ROOT)),replayed_output=str(output.relative_to(ROOT)) if args.replay_rte else None,
            replay_all_fields_except_elapsed_compute_time_identical=used_replay,blank_interval='算法尚未给出有效位置，未补零、插值或平移时间轴')
        for f in (source,old,DATA/f'{name}.truth.csv'):
            hashes[str(f.relative_to(ROOT))]=hashlib.sha256(f.read_bytes()).hexdigest()
    np.testing.assert_allclose(starts,[127.2,215.52,216.24],atol=1e-9)
    fig.suptitle('RTE逐车相对位置误差：仿真无误差局部里程计，重复试验1',fontsize=14)
    drawing.save(fig,'replay_RTE逐车定位误差',shared_xlabel='仿真时间（秒，三组均从0秒开始）')
    # 复用现有只读CDR解码与窗口选择，直接读取原始/livox/lidar消息。
    from visualize_livox_bags import load_box
    cloud_root=ROOT/'data/three_boxes_20260811_012046';cloud=load_box(cloud_root,'201',True);points=cloud['points']
    np.savez_compressed(OUT/'主盒雷达绘图点.npz',xyz_intensity=points)
    fig,axes=plt.subplots(1,2,figsize=(11,5))
    for ax,column,title in zip(axes,(1,2),('俯视图','侧视图')):
        ax.scatter(points[:,0],points[:,column],s=.2,c=points[:,2],cmap='viridis',rasterized=True)
        ax.set(xlabel='雷达局部X（米）',ylabel='雷达局部'+('Y' if column==1 else 'Z')+'（米）',title=title,aspect='equal')
    drawing.save(fig,'replay_主盒实测点云')
    bag=next(cloud_root.glob('uwb_imu_livox_v201_*/livox/livox_0.db3'));hashes[str(bag.relative_to(ROOT))]=hashlib.sha256(bag.read_bytes()).hexdigest()
    result=dict(date='2026-10-29',rte=records,lidar=dict(topic='/livox/lidar',coordinate='201盒本机雷达坐标，未做跨盒配准或地图重建',input=str(bag.relative_to(ROOT)),message_window=[cloud['lo'],cloud['hi']-1],selected_messages=cloud['selected_frames'],plotted_points=len(points),filter='仅去除非有限及全零点，超过120000点时按固定编号抽样',point_array_sha256=hashlib.sha256(points.tobytes()).hexdigest()),input_hashes=hashes)
    (OUT/'复现核对.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    (OUT/'图线坐标说明.json').write_text(json.dumps(drawing.LABELS,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(result,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
