#!/usr/bin/env python3
"""V1.4图线：显式坐标、单位、设备条件、锚点数量与RTE跳变。"""
import json
import csv
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

import plot_expanded_simulation as legacy
from evaluate_rte import ROOT
from evaluate_sensor_anchor_v14 import OUT, ANCHORS
from report_plot_fonts import numeric_ticks

DEST=ROOT/'docs/报告/20261028_协同定位V1.4/figures'
C=['#24557b','#ad653a','#4f806b']
NAMES={'imu':'IMU','vio':'VIO输出模型','lio':'LIO输出模型'}
plt.rcParams.update({'font.family':legacy.FONT_FAMILY,'font.size':14,'axes.labelsize':14,'axes.titlesize':15,
    'xtick.labelsize':12,'ytick.labelsize':12,'legend.fontsize':11,'axes.unicode_minus':False,
    'svg.fonttype':'none','pdf.fonttype':42})
LABELS={}


def save(fig,name,shared_xlabel=None):
    DEST.mkdir(parents=True,exist_ok=True)
    for ax in fig.axes:
        if not ax.axison:
            continue
        if not ax.get_xlabel() and not shared_xlabel:ax.set_xlabel('仿真时间（秒）')
        if not ax.get_ylabel():
            shared={other.get_ylabel() for other in fig.axes if other.get_ylabel()}
            if len(shared)==1:ax.set_ylabel(next(iter(shared)))
        assert (ax.get_xlabel() or shared_xlabel) and ax.get_ylabel(),f'{name}存在未标注坐标轴'
        ax.xaxis.label.set_size(14);ax.yaxis.label.set_size(14);ax.title.set_fontsize(14)
        ax.tick_params(labelsize=12)
        if ax.get_legend():
            legend=ax.get_legend()
            legend.set_bbox_to_anchor(None)
            legend._set_loc(1)  # Matplotlib 3.5: 1 = upper right.
            assert legend._loc==1
            for item in legend.get_texts():item.set_fontsize(11)
        ax.grid(alpha=.2)
    LABELS[name]=[{'横轴':ax.get_xlabel() or shared_xlabel,'纵轴':ax.get_ylabel(),'标题':ax.get_title()} for ax in fig.axes]
    if shared_xlabel:fig.supxlabel(shared_xlabel,fontsize=14)
    numeric_ticks(fig)
    fig.tight_layout()
    fig.savefig(DEST/(name+'.png'),dpi=360,bbox_inches='tight')
    fig.savefig(DEST/(name+'.svg'),bbox_inches='tight')
    fig.savefig(DEST/(name+'.pdf'),bbox_inches='tight')
    plt.close(fig)


def values(case):return np.array([r['position_rmse_m'] for r in case['trials']])


def old_baselines():
    summary=json.loads((ROOT/'data/rte_accuracy_20261023/summary.json').read_text())
    fig,ax=plt.subplots(figsize=(11,5.8))
    groups=[('odometry',0),('imu_ideal',0),('odometry',.02),('odometry',.1),('imu_noisy',.02),('imu_noisy',.1)]
    labels=['无误差里程计\n零测距噪声','理想IMU\n零测距噪声','无误差里程计\n2厘米','无误差里程计\n10厘米','含噪IMU\n2厘米','含噪IMU\n10厘米']
    for index,(mode,sigma) in enumerate(groups):
        rows=[r for r in summary['simulation'] if r['mode']==mode and r['range_sigma_m']==sigma]
        for j,row in enumerate(rows):ax.scatter(index+(j-(len(rows)-1)/2)*.09,row['position_rmse_m'],color=C[index%3],s=42)
    ax.set_xticks(range(6),labels);ax.set(xlabel='运动输入与测距噪声标准差',ylabel='相对位置均方根误差（米）',title='18组既有仿真的逐次结果')
    ax.axhline(1,color='#888',ls='--',label='1米参考线');ax.legend()
    save(fig,'07_RTE全部随机用例精度')
    fig,axes=plt.subplots(2,1,figsize=(10,8.5))
    for seed,color in zip((0,4),C):
        name=f'sim_odometry_0.02_seed{seed}';base=ROOT/'data/rte_accuracy_20261023'
        a=np.genfromtxt(base/(name+'_output.csv'),delimiter=',',names=True);truth=np.genfromtxt(base/(name+'.truth.csv'),delimiter=',',names=True)
        good=(a['status']==1)&(a['elapsed_s']<=40)
        e=np.hypot(a['x_3']-np.interp(a['state_timestamp_ns'],truth['timestamp_ns'],truth['x_3']),a['y_3']-np.interp(a['state_timestamp_ns'],truth['timestamp_ns'],truth['y_3']))
        axes[0].plot(a['elapsed_s'],np.where(good,e,np.nan),color=color,label=f'重复试验{seed+1}')
        axes[1].plot(a['elapsed_s'],np.where(good,a['rms_m'],np.nan),color=color,label=f'重复试验{seed+1}')
    axes[0].set(ylabel='203车相对位置误差（米）',title='2厘米测距噪声、无误差里程计：位置误差')
    axes[1].set(ylabel='测距拟合均方根残差（米）',title='同一次计算的测距拟合结果')
    for ax in axes:ax.set_xlim(0,40);ax.legend()
    save(fig,'08_位置误差与测距残差对照')
    name='joint_time_20261017_151037';a=np.genfromtxt(ROOT/f'data/rte_accuracy_20261023/{name}_output.csv',delimiter=',',names=True)
    fig,axes=plt.subplots(2,1,figsize=(10,8.2))
    for n,color in zip((1,2,3),C):axes[0].plot(a['elapsed_s'],np.hypot(a[f'local_x_{n}'],a[f'local_y_{n}']),color=color,label=f'{200+n}盒')
    axes[0].set(ylabel='本地惯性积分水平位移（米）',title='三盒静止实测回放');axes[0].legend(ncol=3)
    axes[1].step(a['elapsed_s'],a['status'],where='post',color=C[0]);axes[1].set_ylabel('计算状态')
    states={0:'初始化中',1:'有效',2:'约束不足',3:'未收敛',4:'多解',6:'输入过期',7:'残差超限'}
    seen=sorted(set(a['status'].astype(int)));axes[1].set_yticks(seen,[states[v] for v in seen])
    for ax in axes:ax.set_xlabel('消息头相对时间（秒）')
    save(fig,'09_静止实测惯导漂移与RTE状态')


def measured_plots():
    base=ROOT/'data/rte_accuracy_20261023/uwb_baseline_20261017'
    with (base/'uwb_records.csv').open() as f:ranges=list(csv.DictReader(f))
    with (base/'pose_records.csv').open() as f:poses=list(csv.DictReader(f))
    start=min(int(r['header_ns']) for r in ranges)
    fig,axes=plt.subplots(2,1,figsize=(10,8))
    for edge,color in zip(((1,2),(1,3),(2,3)),C):
        rows=[r for r in ranges if (int(r['src_id']),int(r['target_id']))==edge]
        axes[0].plot([(int(r['header_ns'])-start)/1e9 for r in rows],[float(r['distance_m']) for r in rows],color=color,label=f'{200+edge[0]}—{200+edge[1]}')
    for axis,color in zip(('x_m','y_m'),C):
        rows=[r for r in poses if r['node_id']=='3' and r['position_valid']=='True']
        axes[1].plot([(int(r['header_ns'])-start)/1e9 for r in rows],[float(r[axis]) for r in rows],color=color,label=f'203车相对{axis[0].upper()}')
    axes[0].set(ylabel='UWB车间测距（米）',title='静止实测：三条车间距离')
    axes[1].set(ylabel='规范队形相对坐标（米）',title='静止实测：203车相对201车的位置')
    for ax in axes:
        ax.set_xlabel('消息头相对时间（秒）');ax.legend(ncol=3)
        lo,hi=ax.get_ylim();ax.set_ylim(lo,hi+(hi-lo)*.2)
    save(fig,'05_实测在线输入输出')
    fig,ax=plt.subplots(figsize=(8,6));middle={}
    for n,color in zip((1,2,3),C):
        rows=[r for r in poses if r['node_id']==str(n) and r['position_valid']=='True']
        xy=np.array([[float(r['x_m']),float(r['y_m'])] for r in rows]);middle[n]=np.median(xy,axis=0)
        ax.scatter(xy[::4,0],xy[::4,1],s=8,alpha=.2,color=color)
        ax.scatter(*middle[n],s=50,color=color,label=f'{200+n}车位置中位数')
    for i,j in ((1,2),(1,3),(2,3)):ax.plot([middle[i][0],middle[j][0]],[middle[i][1],middle[j][1]],color='#555')
    ax.set(xlabel='规范队形相对X（米）',ylabel='规范队形相对Y（米）',title='三盒静止实测相对队形',aspect='equal');ax.legend()
    save(fig,'06_实测三车相对三角形')


def jump_figure():
    a=np.load(OUT/'jump_analysis/exp_odometry_0.02_seed0.npz')
    fig,axes=plt.subplots(3,1,figsize=(10.5,10))
    for index,color in enumerate(C[:2]):
        axes[0].plot(a['time_s'],a['error_m'][:,index],color=color,label=f'{202+index}车')
        axes[2].plot(a['time_s'],a['correction_displacement_m'][:,index],color=color,label=f'{202+index}车')
    axes[1].plot(a['time_s'],a['residual_m'],color=C[2],label='测距拟合均方根残差')
    axes[0].set(ylabel='相对位置误差（米）',title='上图：与同一时刻位置真值比较')
    axes[1].set(ylabel='测距拟合残差（米）',title='中图：距离模型对输入测距的拟合误差')
    axes[2].set(ylabel='位置修正幅度（米）',title='下图：在同一当前里程计位置比较修正前后变换')
    for ax in axes:
        ax.set_xlim(0,300);ax.set_xlabel('仿真时间（秒）');ax.legend(loc='upper left',ncol=2)
        for sec in (147.6,294.48):ax.axvline(sec,color='#777',ls=':',lw=1)
    axes[0].annotate('294.48秒',xy=(294.48,2.1167),xytext=(214,1.95),arrowprops={'arrowstyle':'->'},fontsize=12)
    fig.suptitle('现有RTE：无误差里程计、2厘米测距噪声、重复试验1',fontsize=16)
    save(fig,'13_扩展仿真误差与状态')


def main():
    summary=json.loads((OUT/'summary.json').read_text());cases={r['name']:r for r in summary['conditions']}
    data=np.load(OUT/'sensor_inputs.npz');time=data['time_s'][::5];truth=data['truth'][::5]
    old_baselines();jump_figure();measured_plots()
    # 既有运动与定位图也写入V1.4独立目录，绝不覆盖V1.3配图。
    original=legacy.save;legacy.save=save;legacy.DEST=DEST
    t,w=legacy.build_world();legacy.trajectory(t,w);legacy.range_and_displacement(t,w)
    legacy.localization_error_curves();legacy.algorithm_trajectories();legacy.save=original
    fig,ax=plt.subplots(figsize=(10.5,7.5))
    for n,color in enumerate(C):
        ax.plot(truth[:,n,0],truth[:,n,1],color=color,label=f'{201+n}车轨迹');ax.scatter(*truth[0,n],color=color,s=45)
    ax.scatter(ANCHORS[:,0],ANCHORS[:,1],marker='^',s=110,color='#555',label='固定锚点')
    for n,a in enumerate(ANCHORS):ax.annotate(('场地O／锚点1' if n==0 else f'锚点{n+1}'),a,xytext=(6,7),textcoords='offset points')
    ax.set(xlabel='场地X（米）',ylabel='场地Y（米）',title='300秒三车轨迹与固定锚点布置',aspect='equal');ax.legend(loc='upper center',ncol=4)
    save(fig,'v14_16_场地坐标与锚点布置')
    fig,axes=plt.subplots(3,1,figsize=(11,11))
    for ax,profile,color in zip(axes,NAMES,C):
        names=[f'{profile}_initial_only',f'{profile}_peer_a0_s0.05',f'{profile}_peer_a1_s0.05',f'{profile}_peer_a3_s0.05',f'{profile}_peer_a4_s0.05']
        arrays=[values(cases[n]) for n in names]
        ax.boxplot(arrays,labels=['起点标定\n运动传播','增加\n车间UWB','再增\n1个锚点','改用\n3个锚点','改用\n4个锚点'],showfliers=True)
        ax.set_yscale('log')
        ax.set(ylabel='相对位置RMSE（米，对数刻度）',xlabel='输入与约束逐项增加',title=f'{NAMES[profile]}：5厘米测距噪声，每组20次')
    save(fig,'v14_17_逐项增加条件的误差变化')
    fig,ax=plt.subplots(figsize=(10,6))
    for profile,color in zip(NAMES,C):
        arrays=[values(cases[f'{profile}_peer_a{n}_s0.05']) for n in (0,1,3,4)]
        med=np.array([np.median(a) for a in arrays]);lo=med-np.array([a.min() for a in arrays]);hi=np.array([a.max() for a in arrays])-med
        ax.errorbar([0,1,3,4],med,yerr=[lo,hi],marker='o',capsize=4,color=color,label=NAMES[profile])
    ax.set(xlabel='新增固定UWB锚点数量（个）',ylabel='相对位置RMSE（米）',title='相同5厘米测距条件下的锚点数量与定位效果');ax.set_xticks([0,1,3,4]);ax.legend()
    save(fig,'v14_18_锚点数量与误差')
    fig,axes=plt.subplots(1,2,figsize=(12,6));ax=axes[1]
    for profile,color in zip(NAMES,C):
        arrays=[values(cases[f'{profile}_peer_a3_s{s:.2f}']) for s in (.02,.05,.10)]
        ax.plot([2,5,10],[np.median(a) for a in arrays],marker='o',color=color,label=NAMES[profile]+'＋3锚点')
    arrays=[values(cases[f'uwb_triangle_s{s:.2f}']) for s in (.02,.05,.10)]
    axes[0].plot([2,5,10],[np.median(a) for a in arrays],marker='s',color='#777',label='纯UWB规范队形坐标')
    axes[0].set(title='规范相对队形坐标');axes[1].set(title='固定场地轴向相对坐标')
    for ax in axes:
        ax.set(xlabel='测距噪声标准差（厘米）',ylabel='相对位置RMSE（米）');ax.legend()
    save(fig,'v14_19_测距精度与定位结果')
    fig,axes=plt.subplots(3,1,figsize=(10.5,10.5))
    for ax,profile in zip(axes,NAMES):
        for count,color in zip((0,1,3),C):
            a=np.load(OUT/f'{profile}_peer_a{count}_s0.05.npz')
            e=np.linalg.norm((a['position'][:,1:]-a['position'][:,:1])-(a['truth'][:,1:]-a['truth'][:,:1]),axis=-1)
            ax.plot(a['time_s'],np.sqrt(np.mean(e**2,axis=-1)),lw=1.1,color=color,label=f'车间UWB＋{count}个固定锚点')
        ax.set(xlabel='仿真时间（秒）',ylabel='两车合并误差（米）',title=NAMES[profile]+'，5厘米测距噪声，重复试验1');ax.legend(ncol=1)
    save(fig,'v14_20_锚点辅助逐时误差')
    fig,axes=plt.subplots(1,2,figsize=(12,6))
    for ax,count in zip(axes,(0,3)):
        a=np.load(OUT/f'lio_peer_a{count}_s0.05.npz')
        for n,color in enumerate(C):
            ax.plot(a['truth'][:,n,0],a['truth'][:,n,1],color=color,lw=2,label=f'{201+n}车真值')
            ax.plot(a['position'][:,n,0],a['position'][:,n,1],color=color,ls='--',label=f'{201+n}车估计')
        ax.set(xlabel='场地X（米）',ylabel='场地Y（米）',aspect='equal',title=f'LIO输出模型＋车间UWB＋{count}个固定锚点');ax.legend(fontsize=10)
    save(fig,'v14_21_场地坐标轨迹对照')
    sensitivity=json.loads((OUT/'sensitivity/summary.json').read_text())['conditions']
    lookup={r['name']:r for r in sensitivity}
    fig,axes=plt.subplots(1,2,figsize=(12,5.8))
    for count,color in zip((0,3),C):
        arrays=[values(cases[f'imu_peer_a{count}_s0.05'])]+[values(lookup[f'imu_bias{b}_a{count}']) for b in (.005,.01)]
        axes[0].plot([.001,.005,.01],[np.median(a) for a in arrays],marker='o',color=color,label=f'{count}个固定锚点')
    axes[0].set(xlabel='IMU残余加速度零偏标准差（米/秒²）',ylabel='相对位置RMSE（米）',title='IMU精度敏感性');axes[0].legend()
    arrays=[values(cases['lio_peer_a0_s0.05'])]+[values(lookup[f'initial_heading{d}']) for d in (1.,3.)]
    axes[1].plot([.2,1,3],[np.median(a) for a in arrays],marker='o',color=C[2]);axes[1].set(xlabel='起点航向标定标准差（度）',ylabel='相对位置RMSE（米）',title='起点定向精度：LIO＋车间UWB，无固定锚点')
    save(fig,'v14_22_输入与标定精度敏感性')
    rte=json.loads((OUT/'rte_evaluation/summary.json').read_text())['rte']
    fig,ax=plt.subplots(figsize=(11,6))
    groups=[];labels=[]
    for profile in ('vio','lio'):
        for sigma in (.02,.05,.10):
            group=[r['position_rmse_m'] for r in rte if r['profile']==profile and r['range_sigma_m']==sigma and r['position_rmse_m'] is not None]
            groups.append(group);labels.append(f'{profile.upper()}输出\n{sigma*100:.0f}厘米测距')
    ax.boxplot(groups,labels=labels);ax.set_yscale('log')
    ax.set(xlabel='局部里程计输入与测距噪声标准差',ylabel='RTE相对位置RMSE（米，对数刻度）',title='未知初始坐标关系：现有RTE库的120次重复试验')
    save(fig,'v14_23_RTE里程计精度对比')
    fig,ax=plt.subplots(figsize=(10,6))
    for profile,color in zip(('imu','lio'),C):
        arrays=[values(lookup[f'{profile}_anchor_rate{rate}']) for rate in (1,5)]+[values(cases[f'{profile}_peer_a3_s0.05'])]
        median=np.array([np.median(v) for v in arrays])
        ax.errorbar([1,5,10],median,yerr=[median-np.array([v.min() for v in arrays]),np.array([v.max() for v in arrays])-median],marker='o',capsize=4,color=color,label=NAMES[profile]+'＋车间UWB＋3锚点')
    ax.set(xlabel='每条锚点测距链路频率（赫兹）',ylabel='相对位置RMSE（米）',title='5厘米测距：锚点更新频率与位置误差');ax.set_xticks([1,5,10]);ax.legend()
    save(fig,'v14_24_锚点测距频率与误差')
    # 同步更新场景布置图和实拍拼图中的排版文字，原始实拍与页面截图不改动。
    import build_stage_report as stage
    stage.save=save;stage.ASSETS=DEST
    stage.scene_plan();stage.hardware()
    (DEST.parent/'图线坐标轴清单.json').write_text(json.dumps(LABELS,ensure_ascii=False,indent=2)+'\n')
    print('V1.4配图已生成，所有坐标轴均写出测量对象及适用单位。')


if __name__=='__main__':main()
