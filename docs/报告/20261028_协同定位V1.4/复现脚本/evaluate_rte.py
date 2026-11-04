#!/usr/bin/env python3
"""独立RTE精度评估：合成真值试验＋原始Header实测回放；不修改原始bag。

先编译 tools/evaluate_rte.cpp，运行本文件需加载ROS和正式cooperative_interfaces。
默认结果：data/rte_accuracy_20261023。原始IMU按FRD→FLU转换，未用AHRS姿态。
"""
import argparse
import csv
import json
from pathlib import Path
import sqlite3
import struct
import subprocess

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
EDGES = ((0, 1), (0, 2), (1, 2))
ORIGINS = np.array(((0., 0.), (4., -1.), (-2., 3.)))
HEADINGS = np.array((0., .8, -1.2))
HEADER = ['kind', 'timestamp_ns', 'node_id', 'target_id'] + [f'v{i}' for i in range(10)]


def save_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n')


def motion(n, t):
    t = max(0., t-12.)
    wx, wy, ax, ay = .6+.13*n, .43+.17*n, 2+.5*n, 1.5+.3*n
    p = np.array((ax*(1-np.cos(wx*t)), ay*(np.sin(wy*t)-wy*t)))
    v = np.array((ax*wx*np.sin(wx*t), ay*wy*(np.cos(wy*t)-1)))
    a = np.array((ax*wx*wx*np.cos(wx*t), -ay*wy*wy*np.sin(wy*t))) if t>0 else np.zeros(2)
    return p, v, a, .04*(n+1)*t, .04*(n+1) if t>0 else 0.


def rotate(p, angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array((c*p[0]-s*p[1], s*p[0]+c*p[1]))


def synthetic(path, mode, sigma, seed):
    rng = np.random.default_rng(seed)
    rows, truth = [], []
    for tick in range(4001):
        t, ns = tick*.01, 10_000_000_000+tick*10_000_000
        world, yaws = [], []
        for n in range(3):
            p, v, a, yaw, rate = motion(n, t)
            world.append(ORIGINS[n]+rotate(p, HEADINGS[n])); yaws.append(HEADINGS[n]+yaw)
            if mode == 'odometry':
                if tick % 2 == 0:
                    values = [*p, 0., *v, 0., 0., 0., np.sin(yaw/2), np.cos(yaw/2)]
                    rows.append(('O', ns, n+1, 0, *values))
            else:
                gyro = np.array((0., 0., rate))
                accel = np.array((*rotate(a, -yaw), 9.80665))
                if mode == 'imu_noisy':
                    gyro += rng.normal(0., .0005, 3)
                    accel += rng.normal(0., .02, 3)
                    if n == 1: accel[0] += .002  # 固定加速度零偏，静止窗口不消除它。
                rows.append(('I', ns, n+1, 0, *gyro, *accel, 0., 0., 0., 0.))
        truth.append([ns, *[x for n in range(3) for x in (*((world[n]-world[0]).tolist()), yaws[n])]])
        # 仅按设定的测距噪声生成完整测距序列，不人为插入中断。
        if tick % 10 == 0:
            for i, j in EDGES:
                d = np.linalg.norm(world[i]-world[j])+rng.normal(0., sigma)
                rows.append(('R', ns, i+1, j+1, d, max(sigma,.001), *([0.]*8)))
    rows.sort(key=lambda r:(r[1], r[0]=='R', r[2], r[3]))
    with path.open('w', newline='') as f:
        w=csv.writer(f); w.writerow(HEADER); w.writerows(rows)
    np.savetxt(path.with_suffix('.truth.csv'), truth, delimiter=',',
               header='timestamp_ns,'+','.join(f'{axis}_{n}' for n in range(1,4) for axis in ['x','y','yaw']), comments='', fmt='%.17g')


def summarize(output, truth=None, *, duration_s=None, motion_start_s=12., dropout_window=None):
    a = np.genfromtxt(output, delimiter=',', names=True)
    limit = (40. if truth is not None else a['elapsed_s'].max()-1.) if duration_s is None else duration_s
    a = a[a['elapsed_s'] <= limit+1e-6]
    good = a['status']==1
    result = {'evaluated_rows':len(a), 'ready_rows':int(good.sum()),
              'status_counts':{str(int(s)):int((a['status']==s).sum()) for s in np.unique(a['status'])},
              'first_ready_s':float(a['elapsed_s'][good][0]) if good.any() else None,
              'final_corrections':int(a['corrections'][-1]),
              'accepted_ranges':int(a['accepted_ranges'][-1]), 'rejected_ranges':int(a['rejected_ranges'][-1]),
              'solve_p95_ms':float(np.percentile(a['solve_ms'],95)),
              'solve_max_ms':float(np.max(a['solve_ms'])),
              'local_final_displacement_m':[float(np.hypot(a[f'local_x_{n}'][-1],a[f'local_y_{n}'][-1])) for n in range(1,4)],
              'local_ready':[bool(a[f'local_ready_{n}'][-1]) for n in range(1,4)],
              'local_fault':[bool(a[f'fault_{n}'][-1]) for n in range(1,4)]}
    if good.any(): result['range_fit_rms_m'] = float(np.sqrt(np.mean(a['rms_m'][good]**2)))
    if truth is not None:
        tr = np.genfromtxt(truth, delimiter=',', names=True)
        # 相同参考局部轴、相同状态Header时刻，直接比较；不做事后刚体拟合。
        # 使用SDK返回的共同状态历元，而不是step调用时刻，避免引入20ms离散延迟误差。
        result['vehicles']={}
        all_errors=[]
        for n in (2,3):
            pos=np.hypot(a[f'x_{n}']-np.interp(a['state_timestamp_ns'],tr['timestamp_ns'],tr[f'x_{n}']),
                         a[f'y_{n}']-np.interp(a['state_timestamp_ns'],tr['timestamp_ns'],tr[f'y_{n}']))
            dy=a[f'yaw_{n}']-np.interp(a['state_timestamp_ns'],tr['timestamp_ns'],tr[f'yaw_{n}'])
            yaw=np.degrees(np.arctan2(np.sin(dy),np.cos(dy)))
            valid=pos[good]
            if valid.size:
                all_errors.extend(valid.tolist())
                result['vehicles'][str(n)]={'position_rmse_m':float(np.sqrt(np.mean(valid**2))),
                  'position_p95_m':float(np.percentile(valid,95)), 'position_max_m':float(valid.max()),
                  'yaw_rmse_deg':float(np.sqrt(np.mean(yaw[good]**2)))}
        result['position_rmse_m']=float(np.sqrt(np.mean(np.array(all_errors)**2))) if all_errors else None
        active=a['elapsed_s']>=motion_start_s
        result['ready_fraction_after_motion']=float(np.mean(good[active]))
        if dropout_window is None:
            result['dropout_stale_seen'] = False
            result['recovered_after_dropout'] = False
        else:
            dropout_start, dropout_end = dropout_window
            result['dropout_stale_seen']=bool(np.any((a['status']==6)&(a['elapsed_s']>dropout_start+0.6)&(a['elapsed_s']<dropout_end)))
            result['recovered_after_dropout']=bool(np.any(good&(a['elapsed_s']>dropout_end)))
    return result


def read_ros_bag(bag, topic):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    import yaml
    meta=yaml.safe_load((bag/'metadata.yaml').read_text())['rosbag2_bagfile_information']
    classes={}
    checked={}
    for filename in meta['relative_file_paths']:
        with sqlite3.connect((bag/filename).resolve().as_uri()+'?mode=ro',uri=True) as db:
            query='SELECT t.type,m.data FROM messages m JOIN topics t ON t.id=m.topic_id WHERE t.name=? ORDER BY m.id'
            for typename,data in db.execute(query,(topic,)):
                if typename not in classes: classes[typename]=get_message(typename)
                # 已知固定字段的CDR1快速读取；逐帧检查布局，每包前16帧与ROS解码交叉核验。
                # 避免数百万帧逐个构造Python ROS消息导致离线评估耗时过长。
                if data[:4]!=b'\x00\x01\x00\x00': raise ValueError('仅支持小端CDR1，拒绝猜测其他序列化布局')
                sec,nano,length=struct.unpack_from('<iII',data,4)
                frame=data[16:16+length-1].decode('utf-8')
                if sec<0 or nano>=10**9 or length<1 or data[15+length]!=0: raise ValueError('非法CDR Header')
                align=lambda offset,n:4+((offset-4+n-1)//n)*n
                if topic=='/imu/raw' and typename=='sensor_msgs/msg/Imu':
                    offset=align(16+length,8)
                    if len(data)!=offset+37*8: raise ValueError('Imu CDR长度不匹配')
                    value=struct.unpack_from('<37d',data,offset)
                    decoded=(*value[13:16],*value[25:28])
                elif topic=='/uwb/range' and typename=='cooperative_interfaces/msg/UwbRange':
                    if list(classes[typename].get_fields_and_field_types())!=['header','src_id','target_id','rolling','distance']:
                        raise ValueError('UWB正式消息定义不匹配')
                    offset=align(16+length,4); end=align(offset+10,4)
                    if len(data)!=end+4: raise ValueError('UwbRange CDR长度不匹配')
                    src,target=struct.unpack_from('<II',data,offset)
                    decoded=(src,target,struct.unpack_from('<f',data,end)[0])
                else: raise ValueError('不支持的消息布局')
                if checked.get(typename,0)<16:
                    m=deserialize_message(data,classes[typename])
                    assert (sec,nano,frame)==(m.header.stamp.sec,m.header.stamp.nanosec,m.header.frame_id)
                    expected=(m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z,m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z) if topic=='/imu/raw' else (m.src_id,m.target_id,m.distance)
                    assert decoded==expected, 'CDR快速读取与ROS标准解码不一致'
                    checked[typename]=checked.get(typename,0)+1
                yield sec*10**9+nano,frame,decoded


def header_stats(times):
    a=np.asarray(times,dtype=np.int64); d=np.diff(a)
    return {'count':len(a),'first_ns':int(a[0]),'last_ns':int(a[-1]),
            'zero':int((a<=0).sum()),'rollback':int((d<0).sum()),'duplicate':int((d==0).sum()),
            'gaps_over_100ms':int((d>100_000_000).sum()),
            'median_dt_ms':float(np.median(d)/1e6),'max_dt_ms':float(d.max()/1e6)}


def real_input(directory, destination):
    imu_bags=[]
    for n in range(1,4):
        matches=list(directory.glob(f'*_imu_rec_v{n}')) or list(directory.glob(f'uwb_imu_livox_v{200+n}_*/imu'))
        if len(matches)!=1: raise ValueError(f'{directory}: 未唯一找到盒{200+n}车载IMU')
        imu_bags.append(matches[0])
    uwb_bags=list(directory.glob('*_uwb_rec_v1')) or list(directory.glob('uwb_imu_livox_v201_*/uwb'))
    if len(uwb_bags)!=1: raise ValueError('未唯一找到201共享UWB')
    rows=[]; stats={}; sources=[]
    for n,bag in enumerate(imu_bags,1):
        times=[]; gyro=[]; accel=[]; frames=set()
        for t,frame,values in read_ros_bag(bag,'/imu/raw'):
            # FRD转FLU，采用ROS SI单位；不使用上游四元数或录包接收时间。
            g=[values[0],-values[1],-values[2]]
            a=[values[3],-values[4],-values[5]]
            times.append(t); gyro.append(g); accel.append(a); frames.add(frame)
            rows.append(('I',t,n,0,*g,*a,0.,0.,0.,0.))
        h=header_stats(times)
        h.update(gyro_std_rad_s=np.std(gyro,axis=0).tolist(),accel_std_m_s2=np.std(accel,axis=0).tolist(),frame_ids=sorted(frames))
        stats[f'IMU_{n}']=h
        sources.append(str(bag.relative_to(ROOT)))
        print(f'  [读取完成] 盒{200+n}车载IMU：{len(times)}帧',flush=True)
    directions={}; ranges={}
    for t,frame,(src,target,distance) in read_ros_bag(uwb_bags[0],'/uwb/range'):
        key=f'{src}-{target}'
        directions.setdefault(key,[]).append(t); ranges.setdefault(key,[]).append(distance)
        if not 1<=src<=3 or not 1<=target<=3: raise ValueError('UWB逻辑ID不是1/2/3')
        if np.isfinite(distance) and distance>0:
            rows.append(('R',t,src,target,distance,.1,*([0.]*8)))
    for key,times in directions.items():
        h=header_stats(times); d=np.array(ranges[key])
        h.update(mean_m=float(d.mean()),std_m=float(d.std()),median_m=float(np.median(d)))
        stats['UWB_'+key]=h
    sources.append(str(uwb_bags[0].relative_to(ROOT)))
    # 异常时间的包不排序“修复”后强行融合；保留为数据门禁失败。
    blockers=[]
    if any(h['zero'] or h['rollback'] for h in stats.values()): blockers.append('原始Header存在零值或回退')
    common_start=max(h['first_ns'] for h in stats.values())
    common_end=min(h['last_ns'] for h in stats.values())
    if common_end-common_start<15*10**9: blockers.append('各流原始Header没有至少15秒共同区间，不能直接配对')
    if any(not np.isfinite(r[4:]).all() for r in rows): blockers.append('输入含非有限值')
    info={'dataset':directory.name,'sources':sources,'header_only':True,'truth_available':False,
          'confirmed_stationary':True,'streams':stats,'blockers':blockers,
          'common_header_duration_s':max(0.,(common_end-common_start)/1e9)}
    if not blockers:
        rows.sort(key=lambda r:(r[1],r[0]=='R',r[2],r[3]))
        with destination.open('w',newline='') as f:
            w=csv.writer(f); w.writerow(HEADER); w.writerows(rows)
    return info


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runner',type=Path,default=ROOT/'build_rte_verify/evaluate_rte')
    parser.add_argument('--output',type=Path,default=ROOT/'data/rte_accuracy_20261023')
    parser.add_argument('--simulation-only',action='store_true')
    parser.add_argument('--real-only',action='store_true')
    args=parser.parse_args(); out=args.output.resolve(); out.mkdir(parents=True,exist_ok=True)
    summary={'date':'2026-10-23','simulation':[],'real':[],
      'scope':'离线SDK评估，无网络/调度延迟模型；不以bag接收时间改写测量时标。',
      'configuration':{'local_static_window_s':10,'local_state_hz':50,'rte_step_hz':20,
        'rte_window_s':5,'rte_max_samples_per_edge':128,'rte_solve_interval_s':.2,
        'minimum_initialization_motion_span_m':10.0,
        'range_dropout_s':None,'sim_duration_s':40,'sim_motion_start_s':12}}
    summary_path=out/'summary.json'
    if summary_path.exists() and (args.real_only or args.simulation_only):
        previous=json.loads(summary_path.read_text())
        key='simulation' if args.real_only else 'real'
        summary[key]=previous.get(key,[])
    def run(source):
        dest=source.with_name(source.stem+'_output.csv')
        subprocess.run([str(args.runner.resolve()),str(source),str(dest)],check=True)
        return dest
    if not args.real_only:
        experiments=[('odometry',0.,0),('imu_ideal',0.,0)]
        experiments += [('odometry',sigma,seed) for sigma in [.02,.1] for seed in range(5)]
        experiments += [('imu_noisy',sigma,seed) for sigma in [.02,.1] for seed in range(3)]
        for mode,sigma,seed in experiments:
            name=f'sim_{mode}_{sigma:.2f}_seed{seed}'; print(f'[开始] {name}',flush=True)
            source=out/(name+'.csv'); synthetic(source,mode,sigma,seed)
            result=summarize(run(source),source.with_suffix('.truth.csv'))
            result.update(name=name,mode=mode,range_sigma_m=sigma,seed=seed)
            summary['simulation'].append(result); save_json(summary_path,summary)
            print(f'[完成] READY={result["ready_rows"]} RMSE={result["position_rmse_m"]}',flush=True)
        zero=summary['simulation'][0]
        assert zero['ready_rows']>100 and zero['position_rmse_m']<.001, zero
        assert zero['ready_rows'] > 100 and zero['position_rmse_m'] < .001
    if not args.simulation_only:
        datasets=['joint_time_20261016_125027','joint_time_20261016_134950',
                  'joint_time_20261016_150956','joint_time_20261017_151037',
                  'three_boxes_20260811_054640','three_boxes_20260811_060745']
        for name in datasets:
            print(f'[开始] 实测 {name}',flush=True)
            source=out/(name+'.csv'); info=real_input(ROOT/'data'/name,source)
            if not info['blockers']: info['replay']=summarize(run(source))
            summary['real'].append(info); save_json(summary_path,summary)
            print(f'[完成] 门禁={info["blockers"]} 回放={info.get("replay",{}).get("ready_rows")}',flush=True)
    save_json(summary_path,summary)
    print(f'[全部完成] {summary_path}',flush=True)


if __name__=='__main__':
    main()
