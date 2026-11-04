#!/usr/bin/env python3
"""保存结果直接绘图：无锚点分级、唯一点的布置及误差。"""
import json
import hashlib
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from evaluate_anchor_layout import OUT, INPUTS, PLACEMENTS, ROOT
from plot_single_anchor import finish, TRACES, relative_error
import plot_sensor_anchor_v14 as shared

DEST=ROOT/'docs/报告/20261028_协同定位V1.4/figures'


def main():
    shared.DEST=DEST;s=json.loads((OUT/'summary.json').read_text());cases={c['name']:c for c in s['conditions']}
    def values(name):return [r['position_rmse_m'] for r in cases[name]['trials']]
    colors=['#24557b','#ad653a','#4f806b']
    for kind in ('imu','vio','lio'):
        fig,axes=plt.subplots(1,2,figsize=(12,4.8))
        for ax,mode,title in zip(axes,('start','peer'),('仅运动传播','运动传播＋车间测距')):
            ax.boxplot([values(f'{kind}_{g}_no_anchor_{mode}') for g in ('基础','增强','精细')],labels=['基础','增强','精细'],showfliers=True)
            ax.axhline(1,ls='--',color='#777',lw=.8);ax.set_yscale('log')
            ax.set(xlabel='输入质量档位',ylabel='相对位置RMSE（米，对数刻度）',title=title)
        finish(fig,'layout_'+kind+'_无锚点分级',['data/single_anchor_layout_20261029/summary.json'])
    world=np.load(INPUTS/'truth_and_range_inputs.npz')['world_truth']
    fig,ax=plt.subplots(figsize=(9.5,6))
    for i,color in enumerate(colors):ax.plot(world[:,i,0],world[:,i,1],color=color,label=f'{201+i}车轨迹')
    for name,p in PLACEMENTS.items():
        ax.scatter(*p,marker='*',s=110,color='#333');ax.annotate(name,p,xytext=(5,7),textcoords='offset points',fontsize=12)
    ax.set(xlabel='场景X（米）',ylabel='场景Y（米）',aspect='equal',title='候选位置对比：每次仅选一个已知位置UWB点');ax.legend(loc='upper right');ax.margins(.15)
    finish(fig,'layout_候选布点',['data/single_anchor_outdoor_20261029/truth_and_range_inputs.npz'])
    fig,axes=plt.subplots(1,3,figsize=(13,5.2))
    for ax,kind in zip(axes,('imu','vio','lio')):
        groups=[values(f'{kind}_{g}_anchor_{p}') for p in PLACEMENTS for g in ('增强',)]
        ax.boxplot(groups,labels=['中部','起点侧方','左侧','右侧','角部'],showfliers=True)
        ax.tick_params(axis='x',rotation=25);ax.set(xlabel='唯一点的候选位置',ylabel='相对位置RMSE（米）',title=kind.upper()+'增强输入')
    finish(fig,'layout_布点误差分布',['data/single_anchor_layout_20261029/summary.json'])
    for kind in ('vio','lio'):
        fig,axes=plt.subplots(1,2,figsize=(12,4.8))
        sources=[]
        for ax,grade in zip(axes,('基础','增强')):
            names=[f'{kind}_{grade}_no_anchor_peer',f'{kind}_{grade}_anchor_区域中部',f'{kind}_{grade}_anchor_区域右侧']
            sources.extend(names)
            for name,label,color in zip(names,('无锚点','中部单点','右侧单点'),colors):
                a=np.load(OUT/'outputs'/f'{name}.npz');e=relative_error(a['position'][0],a['truth'])
                ax.plot(a['time_s'][600:],np.sqrt(np.mean(e[600:]**2,axis=-1)),label=label,color=color,lw=1)
            ax.set(xlabel='仿真时间（秒）',ylabel='两车相对位置误差合成值（米）',title=kind.upper()+grade+'，重复试验1');ax.legend()
        finish(fig,'layout_'+kind+'_逐时定位误差',['data/single_anchor_layout_20261029/outputs/'+n+'.npz' for n in sources])
    fig,axes=plt.subplots(1,2,figsize=(12,5))
    for ax,point in zip(axes,('区域中部','区域右侧')):
        a=np.load(OUT/'outputs'/f'lio_增强_anchor_{point}.npz')
        for n,color in enumerate(colors):
            ax.plot(a['truth'][600:,n,0],a['truth'][600:,n,1],color=color,ls='--',lw=1,label=f'{201+n}车真值')
            ax.plot(a['position'][0,600:,n,0],a['position'][0,600:,n,1],color=color,lw=.8,label=f'{201+n}车估计')
        ax.set(xlabel='任务局部X（米）',ylabel='任务局部Y（米）',aspect='equal',title=point+'，LIO增强，重复试验1');ax.legend(ncol=2,fontsize=9)
    finish(fig,'layout_单点估计轨迹',['data/single_anchor_layout_20261029/outputs/lio_增强_anchor_'+p+'.npz' for p in ('区域中部','区域右侧')])
    # 每幅图同时保留原始数据引用和实际绘图折线的哈希。
    for record in TRACES.values():
        record['input_sha256']={p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in record['input_files']}
    (DEST/'布点与无锚点图线来源.json').write_text(json.dumps(TRACES,ensure_ascii=False,indent=2)+'\n')
    (DEST/'布点图线坐标说明.json').write_text(json.dumps(shared.LABELS,ensure_ascii=False,indent=2)+'\n')
    print('完成',len(TRACES),'幅数据图，均提供PNG、SVG、PDF')


if __name__=='__main__':main()
