#!/usr/bin/env python3
"""平行与非平行直行：真实重复计算与有限时间测距敏感度。"""
import argparse,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from evaluate_single_anchor import ROOT, DT, PROFILES, make_measurements, initialize, estimate, rotate
from evaluate_anchor_layout import initialize_peer_only
from evaluate_sensor_anchor_v14 import tangent_heading
import plot_sensor_anchor_v14 as shared

OUT=ROOT/'data/straight_maneuvers_20261102'
REPORT=ROOT/'docs/报告/20261028_协同定位V1.4'
NAMES=('同速平行直行','不同速平行直行','非平行直行')
COLORS=('#24557b','#ad653a','#4f806b')
EDGES=((0,1),(0,2),(1,2));TRIALS=20


def make_paths(raw):
    t=raw['time_s'];p0=raw['world_truth'][0];e=np.array([1.,.3]);e=e/np.linalg.norm(e);n=np.array([-e[1],e[0]])
    speeds=(np.array([e*.6,e*.6,e*.6]),np.array([e*.6,e*.575,e*.625]),np.array([e*.6,e*.6+n*.035,e*.6-n*.035]))
    # 30～40秒平滑起步，40秒后各车均为严格恒速直线。
    u=np.clip((t-30)/10,0,1);integral=np.where(t<30,0,np.where(t<40,10*(u**3-.5*u**4),t-35))
    return {name:p0[None]+integral[:,None,None]*v[None] for name,v in zip(NAMES,speeds)}


def ranges_from_z(z,displacement,anchor_mode):
    """局部灵敏度模型：201固定轴，状态为初始相对位置与202/203局部位移转角。"""
    r=z[:4].reshape(2,2);theta=z[4:6];du=np.stack([rotate(displacement[:,i],theta[i-1]) for i in (1,2)],axis=1)
    rel=r[None]+du-displacement[:,:1]
    rows=[np.linalg.norm(rel[:,0],axis=-1),np.linalg.norm(rel[:,1],axis=-1),np.linalg.norm(rel[:,1]-rel[:,0],axis=-1)]
    if anchor_mode:
        p1=z[6:8]+displacement[:,0];p=np.concatenate((p1[:,None],p1[:,None]+rel),axis=1)
        rows.extend(np.linalg.norm(p[:,i],axis=-1) for i in range(3))
    return np.stack(rows,axis=-1)


def sensitivity(world,yaw,anchor,mode):
    """对40～300秒的理想局部位移和距离线性化；不把结论泛化到其他状态。"""
    ix=np.arange(400,len(world),10);origin=world[400];heading=yaw[0];ref=heading[0]
    d=world[ix]-origin[None];local=rotate(d,-heading[None]);local[:,0]=rotate(d[:,0],-ref)
    rel=rotate(origin[1:]-origin[:1],-ref);angles=heading[1:]-ref
    z=np.r_[rel.ravel(),angles]
    if mode=='single':z=np.r_[z,rotate(origin[0]-anchor,-ref)]
    expected=np.column_stack([np.linalg.norm(world[ix,i]-world[ix,j],axis=-1) for i,j in EDGES]+([np.linalg.norm(world[ix,i]-anchor,axis=-1) for i in range(3)] if mode=='single' else []))
    np.testing.assert_allclose(ranges_from_z(z,local,mode=='single'),expected,atol=1e-10)
    scales=np.r_[np.ones(4),[.1,.1],np.ones(2) if mode=='single' else []]
    # 用中心差分直接对整个距离序列求导，采样值本身来自相同的运动模型。
    h=1e-5;j=[]
    for i in range(len(z)):
        dz=np.zeros_like(z);dz[i]=h
        j.append(((ranges_from_z(z+dz,local,mode=='single')-ranges_from_z(z-dz,local,mode=='single'))/(2*h)*scales[i]).ravel())
    jac=np.stack(j,axis=1)/np.sqrt(len(ix));singular=np.linalg.svd(jac,compute_uv=False)
    rank=int(np.count_nonzero(singular>singular[0]*1e-7))
    return dict(rank=rank,unknowns=len(z),normalized_min=float(singular[-1]),singular_values=singular.tolist(),time_s=[40,300],position_scale_m=1.,angle_scale_rad=.1)


def run(name,world,raw):
    t=raw['time_s'];anchor=raw['world_anchor'];velocity=np.gradient(world,DT,axis=0);yaw=tangent_heading(velocity)
    truth_ref=rotate(world-world[:,:1],-yaw[0,0]);sensor=make_measurements(t,world,yaw,velocity,'vio_增强',TRIALS)
    truth=np.column_stack([np.linalg.norm(world[:,i]-world[:,j],axis=-1) for i,j in EDGES]+[np.linalg.norm(world[:,i]-anchor,axis=-1) for i in range(3)])
    measured=truth[None]+.1*raw['range_standard']+raw['range_bias'];results={}
    assert np.linalg.norm(velocity,axis=-1).max()<3 and truth[:,:3].max()<30
    for mode in ('peer','single'):
        mr=measured[:,:,:3] if mode=='peer' else measured
        fun=initialize_peer_only if mode=='peer' else initialize
        init=[fun(sensor['local'][k],mr[k],t,.1) for k in range(TRIALS)]
        p0=np.array([x[0] for x in init]);a0=np.array([x[1] for x in init])
        pos,_,_=estimate(sensor,mr,PROFILES['vio_增强'],p0,a0,t,.1,mode)
        ref=rotate(pos-pos[:,:,:1],-a0[:,0,None,None])
        error=np.sqrt(np.mean(np.sum((ref[:,:,1:]-truth_ref[None,:,1:])**2,axis=-1),axis=-1))
        rmse=np.sqrt(np.mean(error[:,600:]**2,axis=1));obs=sensitivity(world,yaw,anchor,mode)
        gt=truth_ref[600,1:];pred=ref[:,600,1:]
        mirrored=int(np.count_nonzero(np.cross(pred[:,0],pred[:,1])*np.cross(gt[0],gt[1])<0))
        results[mode]=dict(estimate_ref=ref,errors=error,rmse=rmse,measured=mr,initial_position=p0,initial_angle=a0)
        print(name,mode,'RMSE',rmse.min(),rmse.mean(),rmse.max(),'秩',obs['rank'],'/',obs['unknowns'],'最小灵敏度',obs['normalized_min'],flush=True)
        results[mode]['summary']=dict(rmse_mean=float(rmse.mean()),rmse_std=float(rmse.std(ddof=1)),rmse_min=float(rmse.min()),rmse_max=float(rmse.max()),mirrored_at_60_count=mirrored,sensitivity=obs)
    return dict(time_s=t,world=world,yaw=yaw,velocity=velocity,truth_ref=truth_ref,range_truth=truth,local=sensor['local'],results=results,
        max_speed=float(np.linalg.norm(velocity,axis=-1).max()),max_peer_distance=float(truth[:,:3].max()))


def draw(data,anchor):
    shared.DEST=REPORT/'figures'
    fig,axes=plt.subplots(3,1,figsize=(9,9))
    for ax,name in zip(axes,NAMES):
        a=data[name];w=a['world'];t=a['time_s'];mask=t>=40
        for i,color in enumerate(shared.C):ax.plot(w[mask,i,0],w[mask,i,1],color=color,label=f'{201+i}车');ax.scatter(*w[400,i],s=16,color=color)
        ax.scatter(*anchor,marker='*',s=80,color='#333',label='UWB锚点')
        ax.set(xlabel='场地X（米）',ylabel='场地Y（米）',title=name,aspect='equal');ax.legend(loc='upper right')
    shared.save(fig,'review_平行与非平行直线轨迹')
    fig,axes=plt.subplots(2,1,figsize=(9,7.8))
    for ax,mode,title in zip(axes,('peer','single'),('无锚点：仅车间测距','单锚点：车间和车锚测距')):
        for name,color in zip(NAMES,COLORS):
            a=data[name];t=a['time_s'];m=t>=60;ax.plot(t[m],a['results'][mode]['errors'][:,m].mean(axis=0),color=color,label=name)
        ax.set(xlabel='仿真时间（秒）',ylabel='两车合并相对位置误差（米）',title=title);ax.legend(loc='upper right')
        _,hi=ax.get_ylim();ax.set_ylim(0,hi*1.35)
    shared.save(fig,'review_直线机动定位误差')


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--plot-only',action='store_true');args=parser.parse_args()
    OUT.mkdir(parents=True,exist_ok=True);(OUT/'outputs').mkdir(exist_ok=True)
    raw=np.load(ROOT/'data/single_anchor_outdoor_20261029/truth_and_range_inputs.npz');paths=make_paths(raw);data={};conditions=[]
    for name in NAMES:
        f=OUT/'outputs'/f'{name}.npz'
        if args.plot_only:
            with np.load(f) as a:
                d={k:a[k] for k in a.files if not k.startswith(('peer_','single_'))}
                d['results']={mode:{k[len(mode)+1:]:a[k] for k in a.files if k.startswith(mode+'_')} for mode in ('peer','single')}
        else:
            d=run(name,paths[name],raw)
            flat={k:v for k,v in d.items() if k not in ('results','max_speed','max_peer_distance')}
            for mode,r in d['results'].items():
                for k,v in r.items():
                    if k!='summary':flat[mode+'_'+k]=v
            np.savez_compressed(f,**flat)
            conditions.append(dict(name=name,max_speed=d['max_speed'],max_peer_distance=d['max_peer_distance'],**{m:d['results'][m]['summary'] for m in ('peer','single')}))
        data[name]=d
    if not args.plot_only:
        (OUT/'summary.json').write_text(json.dumps(dict(trials=TRIALS,profile='vio_增强',range_sigma_m=.1,range_bias_std_m=.03,anchor=raw['world_anchor'].tolist(),evaluation_interval_s=[60,300],straight_interval_s=[40,300],conditions=conditions),ensure_ascii=False,indent=2)+'\n')
    draw(data,raw['world_anchor']);print('仿真与图表完成',flush=True)

if __name__=='__main__':main()
