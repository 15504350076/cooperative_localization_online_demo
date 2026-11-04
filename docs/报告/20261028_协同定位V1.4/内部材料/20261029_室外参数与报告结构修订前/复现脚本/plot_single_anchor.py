#!/usr/bin/env python3
"""仅从保存的单锚点仿真结果绘图；不调用图像生成服务，不修改或拟合轨迹。"""
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from evaluate_single_anchor import OUT, ROOT, PROFILES, SIGMAS, INIT_SECONDS
from plot_sensor_anchor_v14 import save, C
import plot_sensor_anchor_v14 as shared

DEST=ROOT/'docs/报告/20261028_协同定位V1.4/figures'
MODE_NAMES=['起点结果＋运动传播','继续使用车间UWB','继续使用车间UWB＋单锚点']
TRACES={}


def output(name):
    return np.load(OUT/'outputs'/f'{name}.npz')


def relative_error(position, truth):
    return np.linalg.norm((position[...,1:,:]-position[...,:1,:])-(truth[...,1:,:]-truth[...,:1,:]),axis=-1)


def finish(fig, name, sources):
    # 记录实际交给绘图器的折线数据哈希，可同输入NPZ逐点复核。
    h=hashlib.sha256()
    for ax in fig.axes:
        for line in ax.lines:
            for a in (line.get_xdata(),line.get_ydata()):
                try:h.update(np.asarray(a,dtype='<f8').tobytes())
                except ValueError:h.update(str(a).encode())
    TRACES[name]={'input_files':sources,'line_data_sha256':h.hexdigest(),'drawing':'Matplotlib字体排字与数据折线；未使用生成式图片，未对轨迹事后拟合'}
    save(fig,name)


def main():
    shared.DEST=DEST
    summary=json.loads((OUT/'summary.json').read_text());cases={c['name']:c for c in summary['conditions']}
    raw=np.load(OUT/'truth_and_range_inputs.npz');time=raw['time_s'];truth=raw['truth'];start=int(INIT_SECONDS/.1)
    plt.rcParams.update({'font.size':14,'svg.fonttype':'none','pdf.fonttype':42})
    fig,ax=plt.subplots(figsize=(10.5,7))
    for n,color in enumerate(C):
        ax.plot(truth[:,n,0],truth[:,n,1],color=color,label=f'{201+n}车轨迹');ax.scatter(*truth[0,n],color=color,s=45)
    ax.scatter(0,0,marker='*',s=180,color='#333',label='唯一已知位置UWB点')
    for n in range(3):ax.plot([0,truth[0,n,0]],[0,truth[0,n,1]],ls=':',color=C[n],alpha=.6)
    ax.set(xlabel='锚点坐标X（米）',ylabel='锚点坐标Y（米）',aspect='equal',title='三车运动与一个UWB锚点');ax.legend(ncol=2)
    finish(fig,'single_16_三车与唯一锚点',['truth_and_range_inputs.npz'])
    for index,kind in enumerate(('imu','vio','lio'),17):
        fig,axes=plt.subplots(3,1,figsize=(11,10))
        sources=[]
        for ax,grade in zip(axes,('基础','增强','精细')):
            groups=[]
            for mode in ('start','peer','single'):
                name=f'{kind}_{grade}_{mode}_s0.05';sources.append('summary.json:'+name)
                groups.append([r['position_rmse_m'] for r in cases[name]['trials']])
            ax.boxplot(groups,labels=MODE_NAMES,showfliers=True)
            ax.set_yscale('log');ax.axhline(1,color='#888',ls='--',lw=.8)
            ax.set(xlabel='同一传感器输入下逐项增加条件',ylabel='相对位置RMSE（米，对数刻度）',title=f'{kind.upper()} {grade}精度，5厘米测距，每组20次')
        finish(fig,f'single_{index}_{kind.upper()}逐级对比',sources)
    fig,axes=plt.subplots(1,3,figsize=(13,6.2))
    for ax,kind in zip(axes,('imu','vio','lio')):
        for sigma,color in zip(SIGMAS,C):
            groups=[[r['position_rmse_m'] for r in cases[f'{kind}_{grade}_single_s{sigma:.2f}']['trials']] for grade in ('基础','增强','精细')]
            median=np.median(groups,axis=1);ax.errorbar([0,1,2],median,yerr=[median-np.min(groups,axis=1),np.max(groups,axis=1)-median],marker='o',capsize=3,color=color,label=f'测距{sigma*100:.0f}厘米')
        ax.set_xticks([0,1,2],['基础','增强','精细']);ax.set(xlabel='运动输入精度档位',ylabel='相对位置RMSE（米）',title=kind.upper()+'＋车间UWB＋单锚点');ax.legend()
    finish(fig,'single_20_传感器与测距精度',['summary.json'])
    fig,axes=plt.subplots(3,1,figsize=(11,10))
    for ax,kind in zip(axes,('imu','vio','lio')):
        for mode,color,label in zip(('start','peer','single'),C,MODE_NAMES):
            a=output(f'{kind}_增强_{mode}_s0.05');e=relative_error(a['position'][0],truth)
            ax.plot(time,np.sqrt(np.mean(e*e,axis=-1)),color=color,lw=1,label=label)
        ax.axhline(1,color='#888',ls='--',lw=.8);ax.set(xlabel='仿真时间（秒）',ylabel='两车合并位置误差（米）',xlim=(0,300),title=f'{kind.upper()}增强精度，5厘米测距，重复试验1');ax.legend(fontsize=10)
    finish(fig,'single_21_逐时定位误差',['outputs/*_增强_*_s0.05.npz','truth_and_range_inputs.npz'])
    fig,axes=plt.subplots(3,1,figsize=(11,10))
    for ax,kind in zip(axes,('imu','vio','lio')):
        a=output(f'{kind}_增强_single_s0.05');e=relative_error(a['position'][0],truth)
        for n,color in enumerate(C[:2]):ax.plot(time,e[:,n],color=color,lw=.9,label=f'{202+n}车相对参考车的位置误差')
        ax.axhline(1,color='#888',ls='--',lw=.8,label='1米参考线');ax.set(xlabel='仿真时间（秒）',ylabel='相对位置误差（米）',xlim=(0,300),title=f'{kind.upper()}增强精度＋车间UWB＋单锚点');ax.legend(ncol=1)
    finish(fig,'single_22_单锚点逐车误差',['outputs/*_增强_single_s0.05.npz','truth_and_range_inputs.npz'])
    fig,axes=plt.subplots(1,2,figsize=(12,6))
    for ax,kind in zip(axes,('vio','lio')):
        a=output(f'{kind}_增强_single_s0.05');p=a['position'][0]
        for n,color in enumerate(C):
            ax.plot(truth[start:,n,0],truth[start:,n,1],color=color,label=f'{201+n}车真值',lw=1.5)
            ax.plot(p[:,n,0],p[:,n,1],color=color,ls='--',label=f'{201+n}车估计',lw=1)
        ax.scatter(0,0,marker='*',color='#333',s=90);ax.set(xlabel='锚点坐标X（米）',ylabel='锚点坐标Y（米）',aspect='equal',title=kind.upper()+'增强精度，5厘米测距');ax.legend(ncol=2)
    finish(fig,'single_23_算法输出与真值轨迹',['outputs/vio_增强_single_s0.05.npz','outputs/lio_增强_single_s0.05.npz','truth_and_range_inputs.npz'])
    fig,ax=plt.subplots(figsize=(10,6))
    for kind,color in zip(('imu','vio','lio'),C):
        a=output(f'{kind}_增强_single_s0.05');e=relative_error(a['position'][:,start:],truth[start:]).ravel();sorted_errors=np.sort(e)
        sample=np.linspace(0,len(e)-1,1000).astype(int)
        ax.plot(sorted_errors[sample],(sample+1)/len(e)*100,color=color,label=kind.upper()+'增强精度')
    ax.set(xlabel='逐车相对位置误差（米）',ylabel='累计样本比例（%）',title='5厘米测距，单锚点融合，20次全部历元');ax.legend()
    finish(fig,'single_24_位置误差累计分布',['outputs/*_增强_single_s0.05.npz','truth_and_range_inputs.npz'])
    fig,axes=plt.subplots(3,1,figsize=(11,10));a=output('lio_增强_single_s0.05');p=a['position'][0];e=relative_error(p,truth)
    axes[0].plot(time,e[:,0],color=C[0],label='202车');axes[0].plot(time,e[:,1],color=C[1],label='203车');axes[0].set_ylabel('相对位置误差（米）');axes[0].legend()
    measured=raw['range_true']+.05*raw['range_standard'][0]
    predicted=np.column_stack([np.linalg.norm(p[:,i]-p[:,j],axis=-1) for i,j in ((0,1),(0,2),(1,2))]+[np.linalg.norm(p[:,i],axis=-1) for i in range(3)])
    residual=np.sqrt(np.mean((predicted-measured)**2,axis=-1));axes[1].plot(time,residual,color=C[2]);axes[1].set_ylabel('六条测距拟合残差（米）')
    axes[2].plot(time,a['correction_m'][0],color=C[0]);axes[2].set_ylabel('三车位置更新总幅度（米）')
    for ax in axes:ax.set(xlabel='仿真时间（秒）',xlim=(0,300))
    fig.suptitle('LIO增强精度＋单锚点：位置、测距残差与更新幅度',fontsize=15)
    finish(fig,'single_25_位置与测量更新',['outputs/lio_增强_single_s0.05.npz','truth_and_range_inputs.npz'])
    budget=json.loads((OUT/'成本预算.json').read_text());fig,ax=plt.subplots(figsize=(11,6.5))
    for row in budget['combinations']:
        if row['profile']=='uwb':continue
        values=[r['position_rmse_m'] for r in cases[f'{row["profile"]}_single_s0.05']['trials']];cost=np.array(row['total_wan']);median=np.median(values);color=C[('imu','vio','lio').index(PROFILES[row['profile']]['kind'])]
        ax.errorbar(cost.mean(),median,xerr=[[cost.mean()-cost[0]],[cost[1]-cost.mean()]],yerr=[[median-min(values)],[max(values)-median]],fmt='o',color=color,capsize=3)
        ax.annotate(row['profile'].replace('_',' '),(cost.mean(),median),xytext=(5,5),textcoords='offset points',fontsize=10)
    ax.set(xlabel='三车硬件、算法与联调总预算（万元）',ylabel='相对位置RMSE（米）',title='预算与仿真条件对应关系：5厘米测距、单锚点');ax.set_yscale('log')
    finish(fig,'single_26_预算与仿真精度',['成本预算.json','summary.json'])
    (DEST.parent/'单锚点图线来源.json').write_text(json.dumps(TRACES,ensure_ascii=False,indent=2)+'\n')
    axis_file=DEST.parent/'图线坐标轴清单.json'
    previous=json.loads(axis_file.read_text()) if axis_file.exists() else {}
    previous={k:v for k,v in previous.items() if not k.startswith('v14_')}
    previous.update(shared.LABELS)
    axis_file.write_text(json.dumps(previous,ensure_ascii=False,indent=2)+'\n')
    print('单锚点11幅结果图已由保存输出绘制，PNG/SVG/矢量PDF及来源清单已写入。')


if __name__=='__main__':main()
