#!/usr/bin/env python3
"""统一201坐标下重新绘图，并绘制实际受控重放与建系诊断。"""
import hashlib
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import plot_anchor_layout as layout
import plot_single_anchor as single
import plot_sensor_anchor_v14 as shared
from evaluate_common_reference import OUT as COMMON
from evaluate_single_anchor import ROOT, PROFILES, SIGMAS, rotate

REPORT=ROOT/'docs/报告/20261028_协同定位V1.4'
PEAKS=ROOT/'data/fusion_peak_analysis_20261030'
STARTUP=ROOT/'data/rte_startup_diagnostics_20261030'


def main():
    shared.DEST=REPORT/'figures';layout.OUT=COMMON
    original=layout.finish
    layout.finish=lambda fig,name,sources: original(fig,'review_'+name,sources)
    layout.main()
    def finish(fig,name,sources):single.finish(fig,'review_'+name,[str(p.relative_to(ROOT)) for p in sources])
    cases={c['name']:c for c in json.loads((COMMON/'summary.json').read_text())['conditions']}
    fig,axes=plt.subplots(1,3,figsize=(13,4.8),sharex=True,sharey=False)
    for ax,grade in zip(axes,('基础','增强','精细')):
        for mode,color,label in zip(('start','peer'),shared.C[:2],('仅IMU传播','加入车间测距')):
            a=np.load(COMMON/'outputs'/f'imu_{grade}_no_anchor_{mode}.npz');truth=a['truth']
            d=(a['position'][:,:,1:]-a['position'][:,:,:1])-(truth[None,:,1:]-truth[None,:,:1])
            error=np.sqrt(np.mean(np.sum(d*d,axis=-1),axis=-1)).mean(axis=0)
            ax.plot(a['time_s'],error,color=color,label=label)
        ax.set(xlabel='仿真时间（秒）',ylabel='两车合并相对位置误差（米）',title='IMU'+grade+'输入')
        lo,hi=ax.get_ylim();ax.set_ylim(lo,hi+(hi-lo)*.3)
        ax.legend()
    finish(fig,'imu纯传播与测距',[COMMON/'outputs/imu_基础_no_anchor_start.npz',COMMON/'outputs/imu_基础_no_anchor_peer.npz',COMMON/'outputs/imu_增强_no_anchor_start.npz',COMMON/'outputs/imu_增强_no_anchor_peer.npz',COMMON/'outputs/imu_精细_no_anchor_start.npz',COMMON/'outputs/imu_精细_no_anchor_peer.npz'])
    fig,axes=plt.subplots(1,2,figsize=(12,4.8))
    paths=[]
    for ax,grade in zip(axes,('基础','增强')):
        path=COMMON/'outputs'/f'vio_{grade}_no_anchor_peer.npz';paths.append(path)
        a=np.load(path);position=a['position'][0];truth=a['truth']
        delta=(position[:,1:]-position[:,:1])-(truth[:,1:]-truth[:,:1])
        error=np.sqrt(np.mean(np.sum(delta*delta,axis=-1),axis=-1))
        ax.plot(a['time_s'][600:],error[600:],color=shared.C[0],label='无锚点相对位置误差')
        ax.set(xlabel='仿真时间（秒）',ylabel='两车合并相对位置误差（米）',title=f'VIO{grade}，重复试验1',xlim=(60,300))
        lo,hi=ax.get_ylim();ax.set_ylim(lo,hi+(hi-lo)*.2);ax.legend(loc='upper right')
    finish(fig,'VIO无锚点误差过程',paths)
    fig,axes=plt.subplots(1,3,figsize=(13,5.3))
    for ax,kind in zip(axes,('imu','vio','lio')):
        for sigma,color in zip(SIGMAS,shared.C):
            groups=[[r['position_rmse_m'] for r in cases[f'{kind}_{g}_single_s{sigma:.2f}' if sigma!=.1 else f'{kind}_{g}_anchor_区域中部']['trials']] for g in ('基础','增强','精细')]
            med=np.median(groups,axis=1);ax.errorbar([0,1,2],med,yerr=[med-np.min(groups,axis=1),np.max(groups,axis=1)-med],marker='o',capsize=3,color=color,label=f'测距{sigma*100:.0f}厘米')
        ax.set_xticks([0,1,2],['基础','增强','精细']);ax.set(xlabel='运动输入质量档位',ylabel='相对位置RMSE（米）',title=kind.upper()+'＋车间测距＋单锚点');lo,hi=ax.get_ylim();ax.set_ylim(lo,hi+(hi-lo)*.3);ax.legend()
    finish(fig,'测距精度对照',[COMMON/'summary.json'])
    raw=np.load(ROOT/'data/single_anchor_outdoor_20261029/truth_and_range_inputs.npz');world=raw['world_truth'];angle=-raw['yaw'][0,0]
    fig,axes=plt.subplots(1,2,figsize=(12,5.4))
    for ax,step in zip(axes,(0,1200)):
        relative=rotate(world[step]-world[step,0],angle);anchor=rotate(raw['world_anchor']-world[step,0],angle)
        for n,color in enumerate(shared.C):
            ax.scatter(*relative[n],s=70,color=color);ax.annotate(f'{201+n}车',relative[n],xytext=(-42,8) if n==0 else (5,-14),textcoords='offset points')
            ax.plot([anchor[0],relative[n,0]],[anchor[1],relative[n,1]],ls=':',lw=.8,color=color)
        ax.plot(relative[[0,1,2,0],0],relative[[0,1,2,0],1],color='#777',lw=1)
        ax.scatter(*anchor,marker='*',s=140,color='#333',label='同一个已知UWB锚点')
        ax.annotate('锚点',anchor,xytext=(5,5),textcoords='offset points')
        ax.annotate('',(12,0),(0,0),arrowprops=dict(arrowstyle='->',color='#333'));ax.annotate('',(0,12),(0,0),arrowprops=dict(arrowstyle='->',color='#333'))
        ax.set(xlabel='201参考X（米）',ylabel='201参考Y（米）',aspect='equal',title=f'仿真{step/10:.0f}秒，轴向保持固定',xlim=(-10,35),ylim=(-47,22) if step==0 else (-18,42))
    finish(fig,'201参考坐标与锚点',[ROOT/'data/single_anchor_outdoor_20261029/truth_and_range_inputs.npz'])
    a=np.load(PEAKS/'vio_增强.npz');t=a['time_s'];fig,axes=plt.subplots(3,1,figsize=(11,9),sharex=True)
    axes[0].plot(t,a['error_0'],label='原始相对位置误差',color=shared.C[0]);axes[0].plot(t,a['remaining_shape_error_m'],label='去除共同转角后的形状分量（仅归因）',color=shared.C[1]);axes[0].set(ylabel='误差（米）',title='VIO增强、无锚点、重复试验1');axes[0].legend()
    axes[1].plot(t,a['common_rotation_error_deg'],color=shared.C[0]);axes[1].set_ylabel('队形共同转角误差（度）')
    axes[2].plot(t,a['maximum_peer_range_m'],label='最大车间距',color=shared.C[0]);axes[2].plot(t,a['minimum_triangle_altitude_m'],label='队形三角形最小高度',color=shared.C[1]);axes[2].set_ylabel('距离（米）');axes[2].legend()
    lo,hi=axes[2].get_ylim();axes[2].set_ylim(lo,hi+(hi-lo)*.3)
    for ax in axes:
        for sec in (72.3,89.3):ax.axvline(sec,color='#888',ls=':',lw=.8)
        ax.set(xlabel='仿真时间（秒）',xlim=(60,300))
    finish(fig,'峰值与队形方向',[PEAKS/'vio_增强.npz'])
    fig,axes=plt.subplots(1,2,figsize=(12,4.8))
    for ax,profile in zip(axes,('vio_增强','lio_增强')):
        a=np.load(PEAKS/f'{profile}.npz')
        for k,title,color in zip((0,1,2),('原始输入','后续方向漂移设为零','后续测距随机项设为零'),shared.C):ax.plot(a['time_s'],a[f'error_{k}'],color=color,lw=1,label=title)
        ax.set(xlabel='仿真时间（秒）',ylabel='两车合并相对位置误差（米）',xlim=(60,300),title=profile.replace('_','').upper()+'，重复试验1');ax.legend()
    finish(fig,'同输入峰值对照',[PEAKS/'vio_增强.npz',PEAKS/'lio_增强.npz'])
    fig,axes=plt.subplots(3,1,figsize=(11,10.5),sharex=True,gridspec_kw={'height_ratios':[1,1,1.5]})
    rte_sources=[]
    zoom_line_hash=hashlib.sha256()
    for ax,sigma in zip(axes,(.02,.05,.10)):
        output=STARTUP/f'w10_d5_s{sigma:.2f}_output.csv'
        truth=ROOT/'data/rte_expanded_20261025'/f'exp_odometry_{sigma:.2f}_seed0.truth.csv'
        a=np.genfromtxt(output,delimiter=',',names=True);a=a[a['elapsed_s']<=300]
        tr=np.genfromtxt(truth,delimiter=',',names=True);good=a['status']==1
        errors=[]
        inset=None
        if sigma==.10:
            inset=ax.inset_axes([.49,.18,.48,.52])
            inset.set(xlim=(90,300),ylim=(0,3),title='90～300秒局部放大',ylabel='误差（米）')
            inset.set_xticks([100,200,300]);inset.set_yticks([0,1,2,3]);inset.tick_params(labelsize=10)
            for label in [*inset.get_xticklabels(),*inset.get_yticklabels()]:label.set_fontfamily(['Times New Roman','Liberation Serif'])
            inset.xaxis.label.set_size(10);inset.yaxis.label.set_size(10);inset.title.set_size(11);inset.grid(alpha=.2)
        for n,color in zip((2,3),shared.C):
            e=np.hypot(a[f'x_{n}']-np.interp(a['state_timestamp_ns'],tr['timestamp_ns'],tr[f'x_{n}']),
                       a[f'y_{n}']-np.interp(a['state_timestamp_ns'],tr['timestamp_ns'],tr[f'y_{n}']))
            errors.extend(e[good].tolist());ax.plot(a['elapsed_s'],np.where(good,e,np.nan),color=color,lw=1,label=f'{200+n}车相对位置误差')
            if inset is not None:
                mask=good & (a['elapsed_s']>=90) & (a['elapsed_s']<=300)
                zoom_t=a['elapsed_s'][mask];zoom_e=e[mask]
                assert len(zoom_e)>0 and np.max(zoom_e)<3
                line,=inset.plot(zoom_t,zoom_e,color=color,lw=1)
                np.testing.assert_array_equal(line.get_xdata(),zoom_t);np.testing.assert_array_equal(line.get_ydata(),zoom_e)
                for values in (zoom_t,zoom_e):zoom_line_hash.update(np.asarray(values,dtype='<f8').tobytes())
        row=next(r for r in json.loads((STARTUP/'summary.json').read_text())['conditions'] if r['name']==f'w10_d5_s{sigma:.2f}')
        np.testing.assert_allclose(np.sqrt(np.mean(np.square(errors))),row['metrics']['position_rmse_m'],rtol=1e-10)
        ax.axvspan(0,float(a['elapsed_s'][good][0]),color='#dddddd',alpha=.5,label='尚无有效结果')
        ax.set(xlabel=' ',ylabel='误差（米）',title=f'测距误差标准差 {sigma*100:.0f} 厘米',xlim=(0,300));ax.legend(loc='upper right')
        rte_sources.extend((output,truth))
    axes[-1].set_xlabel('仿真时间（秒）')
    finish(fig,'RTE低速初始化误差',rte_sources)
    single.TRACES['review_RTE低速初始化误差']['局部放大']={'时间范围_秒':[90,300],'误差范围_米':[0,3],'数据':'相同输出与真值逐点计算','line_data_sha256':zoom_line_hash.hexdigest()}
    records=json.loads((STARTUP/'summary.json').read_text())['conditions'];fig,axes=plt.subplots(1,2,figsize=(12,5.4))
    choices=((5,10),(5,5),(5,0),(10,5));names=['5秒窗\n当前10米判据','5秒窗\n5米判据对照','5秒窗\n取消运动下限','10秒窗\n5米判据对照']
    for sigma,color in zip((.02,.05,.1),shared.C):
        rows=[next(r for r in records if r['window_s']==w and r['minimum_motion_span_m']==d and r['range_sigma_m']==sigma) for w,d in choices]
        axes[0].plot(range(4),[r['first_valid_s'] for r in rows],marker='o',color=color,label=f'测距{100*sigma:.0f}厘米')
        axes[1].plot(range(4),[r['metrics']['position_rmse_m'] for r in rows],marker='o',color=color,label=f'测距{100*sigma:.0f}厘米')
    for ax in axes:ax.set_xticks(range(4),names);ax.set_xlabel('建系窗口与运动条件');ax.legend()
    axes[0].set(ylabel='首次有效仿真时间（秒）',title='同一轨迹的建系时间')
    axes[1].set(ylabel='有效输出相对位置RMSE（米）',title='同一配置的实际定位误差')
    finish(fig,'RTE建系参数对照',[STARTUP/'summary.json'])
    for item in single.TRACES.values():item['input_sha256']={p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in item['input_files']}
    (REPORT/'融合复核图线来源.json').write_text(json.dumps(single.TRACES,ensure_ascii=False,indent=2)+'\n')
    (REPORT/'融合复核图坐标轴清单.json').write_text(json.dumps(shared.LABELS,ensure_ascii=False,indent=2)+'\n')
    print('统一坐标与对照分析图完成：',len(single.TRACES))


if __name__=='__main__':main()
