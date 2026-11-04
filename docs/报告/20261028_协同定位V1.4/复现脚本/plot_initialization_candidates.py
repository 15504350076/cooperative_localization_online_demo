#!/usr/bin/env python3
"""使用实际初始化函数的仿真输入及候选初值绘制说明图。"""
import json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import evaluate_anchor_layout as peer
import evaluate_single_anchor as single
from plot_sensor_anchor_v14 import save
import plot_sensor_anchor_v14 as shared

ROOT=single.ROOT
INPUTS=ROOT/'data/single_anchor_outdoor_20261029'
REPORT=ROOT/'docs/报告/20261028_协同定位V1.4'

def capture(module, fn, local, ranges, times):
    original=module.least_squares
    records=[]
    def record(fun, x0, *args, **kwargs):
        result=original(fun,x0,*args,**kwargs)
        records.append({'initial':np.asarray(x0).tolist(),'solution':result.x.tolist(),
                        'cost':float(result.fun@result.fun),'converged':bool(result.success)})
        return result
    module.least_squares=record
    try:
        position, angle, detail=fn(local,ranges,times,.1)
    finally:
        module.least_squares=original
    best=int(np.argmin([r['cost'] for r in records]));z=np.asarray(records[best]['solution'])
    if module is peer:
        p=np.array([[0,0],[z[0],0],[z[1],z[2]]]);a=single.wrap(z[3:])
    else:
        p=np.array([[z[0],0],[z[1],z[2]],[z[3],z[4]]]);a=single.wrap(z[5:])
    np.testing.assert_allclose(p+single.rotate(local[600],a),position,atol=1e-9)
    np.testing.assert_allclose(a,angle,atol=1e-9)
    return records,best,detail

def draw_shape(ax, candidates, is_anchor):
    for index,color,label in [(0,'#24557b','镜像起点一'),(1,'#ad653a','镜像起点二')]:
        z=np.array(candidates[index]['initial'])
        p=np.array([[z[0],0],[z[1],z[2]],[z[3],z[4]]]) if is_anchor else np.array([[0,0],[z[0],0],[z[1],z[2]]])
        ax.plot(p[[0,1,2,0],0],p[[0,1,2,0],1],'-o',color=color,label=label)
        for n,(x,y) in enumerate(p):
            ax.annotate(str(201+n),(x,y),xytext=(4,5 if index==0 else -12),textcoords='offset points',fontsize=9,color=color)
    if is_anchor:
        ax.scatter([0],[0],marker='*',s=120,color='#333',label='UWB锚点',zorder=5)
        ax.update_datalim(np.array([[0.,0.]]));ax.autoscale_view()
    lo,hi=ax.get_ylim();ax.set_ylim(lo,hi+(hi-lo)*.5)
    ax.set(xlabel='初始坐标 X（米）',ylabel='初始坐标 Y（米）',aspect='equal');ax.legend(fontsize=9,loc='lower left' if not is_anchor else 'upper left');ax.grid(alpha=.2)

def main():
    raw=np.load(INPUTS/'truth_and_range_inputs.npz');local=np.load(INPUTS/'inputs/vio_增强.npz')['local'][0]
    times=raw['time_s'];ranges=(raw['range_true'][None]+.1*raw['range_standard']+raw['range_bias'])[0]
    pr,pbest,_=capture(peer,peer.initialize_peer_only,local,ranges[:,:3],times)
    ar,abest,_=capture(single,single.initialize,local,ranges,times)
    assert len(pr)==24 and len(ar)==6
    for records,best,filename,is_anchor in (
        (pr,pbest,'vio_增强_no_anchor_initialization.npz',False),
        (ar,abest,'vio_增强_区域中部_initialization.npz',True)):
        saved=np.load(peer.OUT/'outputs'/filename);z=np.asarray(records[best]['solution'])
        p=np.array([[z[0],0],[z[1],z[2]],[z[3],z[4]]]) if is_anchor else np.array([[0,0],[z[0],0],[z[1],z[2]]])
        angle=single.wrap(z[5:] if is_anchor else z[3:])
        np.testing.assert_allclose(p+single.rotate(local[600],angle),saved['position'][0],atol=1e-8)
        np.testing.assert_allclose(angle,saved['angle'][0],atol=1e-8)
    fig,axes=plt.subplots(2,2,figsize=(12.5,8.7))
    draw_shape(axes[0,0],[pr[0],pr[12]],False);axes[0,0].set_title('无锚点：三车起点的两种镜像')
    draw_shape(axes[1,0],[ar[0],ar[3]],True);axes[1,0].set_title('单锚点：三车与锚点的两种镜像')
    ax=axes[0,1]; angles=np.array([pr[i]['initial'][3] for i in (0,3,6,9)])
    for angle,color in zip(angles,('#24557b','#ad653a','#4f806b','#765786')):
        ax.arrow(0,0,np.cos(angle),np.sin(angle),head_width=.09,length_includes_head=True,color=color)
        ax.text(1.2*np.cos(angle),1.2*np.sin(angle),f'{np.rad2deg(angle):.0f}°',ha='center',va='center',color=color)
    ax.set(xlim=(-1.5,1.5),ylim=(-1.5,1.5),xlabel='水平分量',ylabel='竖直分量',aspect='equal',title='无锚点：四种共同转角初值');ax.grid(alpha=.2)
    ax=axes[1,1]; offsets=np.array([pr[i]['initial'][3:6] for i in (0,1,2)])-angles[0]
    x=np.arange(3);w=.22
    for n,(color,label) in enumerate(zip(('#24557b','#ad653a','#4f806b'),('201车','202车','203车'))):
        ax.bar(x+(n-1)*w,np.rad2deg(offsets[:,n]),w,label=label,color=color)
    ax.set_xticks(x,['相对转角起点一','相对转角起点二','相对转角起点三'])
    ax.set(xlabel='无锚点相对转角起点',ylabel='相对共同转角偏移（度）',title='三种车辆间相对转角初值',ylim=(-32,60));ax.legend(fontsize=9);ax.grid(alpha=.2,axis='y')
    fig.suptitle('初始候选关系：位置镜像与局部坐标转角',fontsize=16)
    shared.DEST=REPORT/'figures';save(fig,'review_初始化候选关系')
    output={'说明':'输入为带噪仿真数据；图中展示初始化前的候选起点，优化及选解调用实际初始化函数。',
            'profile':'vio_增强','trial':1,'range_sigma_m':.1,'sample_time_s':[0,60],
            'peer_count':len(pr),'anchor_count':len(ar),'peer_selected':pbest+1,'anchor_selected':abest+1,
            'peer_candidates':pr,'anchor_candidates':ar,'input':'data/single_anchor_outdoor_20261029/truth_and_range_inputs.npz',
            'local_input':'data/single_anchor_outdoor_20261029/inputs/vio_增强.npz'}
    for path in (INPUTS/'初始化候选解.json',REPORT/'初始化候选解图线来源.json'):
        path.write_text(json.dumps(output,ensure_ascii=False,indent=2)+'\n')
    print('无锚点24组、单锚点6组；实际初始化选解一致',pbest+1,abest+1)
if __name__=='__main__':main()
