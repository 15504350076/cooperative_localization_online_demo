#!/usr/bin/env python3
"""保留参数来源、静止统计和单点整体旋转不变性验证；不运行前端或改动原库。"""
import hashlib
import json
from pathlib import Path
import urllib.request

import numpy as np
from evaluate_single_anchor import OUT, ROOT, PROFILES, rotate

SOURCES = [
 ('Kalibr_IMU误差模型','https://raw.githubusercontent.com/wiki/ethz-asl/kalibr/IMU-Noise-Model.md',
  'https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model','模型与连续/离散噪声换算，不提供本项目残余零偏'),
 ('VINS_Fusion_车载双目IMU','https://raw.githubusercontent.com/HKUST-Aerial-Robotics/VINS-Fusion/master/config/vi_car/vi_car.yaml',
  'https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/master/config/vi_car/vi_car.yaml','车载实例的原始IMU/视觉配置，不是输出轨迹误差模型'),
 ('LIO_SAM_参数','https://raw.githubusercontent.com/TixiaoShan/LIO-SAM/master/config/params.yaml',
  'https://github.com/TixiaoShan/LIO-SAM/blob/master/config/params.yaml','含室外降采样配置、IMU噪声及偏置游走；不等于扣偏后恒定偏置'),
 ('LIO_SAM_预积分','https://raw.githubusercontent.com/TixiaoShan/LIO-SAM/master/src/imuPreintegration.cpp',
  'https://github.com/TixiaoShan/LIO-SAM/blob/master/src/imuPreintegration.cpp','核对协方差平方及偏置随机游走的sqrt(dt)用法'),
 ('FAST_LIO_MID360','https://raw.githubusercontent.com/hku-mars/FAST_LIO/main/config/mid360.yaml',
  'https://github.com/hku-mars/FAST_LIO/blob/main/config/mid360.yaml','滤波协方差参数，不能当作传感器残余零偏或轨迹误差'),
 ('Omni_swarm_实机','https://raw.githubusercontent.com/HKUST-Aerial-Robotics/Omni-swarm/main/swarm_localization/launch/loop-5-drone.launch',
  'https://github.com/HKUST-Aerial-Robotics/Omni-swarm','实机距离协方差与多帧初始化；多视觉观测系统，不等于本项目六轴IMU单测距系统'),
]


def main():
    folder=OUT/'参数与可观性核对';folder.mkdir(parents=True,exist_ok=True);sources=[]
    for name,url,link,meaning in SOURCES:
        target=folder/(name+'.txt')
        if not target.exists():
            try:
                content=urllib.request.urlopen(url,timeout=20).read()
            except Exception:
                # 部分仓库使用master分支，保留实际下载地址。
                url=url.replace('/main/','/master/');content=urllib.request.urlopen(url,timeout=20).read()
            target.write_bytes(content)
        sources.append(dict(name=name,url=url,reference=link,usage=meaning,sha256=hashlib.sha256(target.read_bytes()).hexdigest()))
    static=[]
    for p in (ROOT/'data').glob('**/静止初始化统计.json'):
        a=json.loads(p.read_text());g=a.get('post_corrected_mean_rad_s')
        if g:static.append(dict(file=str(p.relative_to(ROOT)),seconds=a['evaluation_span_seconds'],post_bias_rad_s=g,
            equivalent_deg_per_hour=[v*180/np.pi*3600 for v in g],gyro_std_rad_s=a.get('gyro_std_rad_s'),accel_std_mps2=a.get('accel_std_m_s2')))
    (folder/'静止片段统计核对.json').write_text(json.dumps(static,ensure_ascii=False,indent=2)+'\n')
    raw=np.load(OUT/'truth_and_range_inputs.npz');p=raw['world_truth'];anchor=raw['world_anchor'];yaw=raw['yaw']
    angle=np.deg2rad(45);alternative=anchor+rotate(p-anchor,angle)
    edges=((0,1),(0,2),(1,2))
    def distances(track):return np.column_stack([np.linalg.norm(track[:,i]-track[:,j],axis=-1) for i,j in edges]+[np.linalg.norm(track[:,i]-anchor,axis=-1) for i in range(3)])
    maximum_range_change=float(np.max(np.abs(distances(p)-distances(alternative))))
    local=rotate(np.diff(p,axis=0),-yaw[0]);local_rotated=rotate(np.diff(alternative,axis=0),-yaw[0]-angle)
    maximum_local_change=float(np.max(np.abs(local-local_rotated)))
    assert maximum_range_change<1e-10 and maximum_local_change<1e-10
    # 静态和动态都不能凭此观测区别整体转动；局部转角仍可在运动充分时求解。
    np.savez_compressed(folder/'整体旋转不变性.npz',time_s=raw['time_s'],original=p,rotated=alternative,anchor=anchor,
        range_difference=distances(alternative)-distances(p))
    proof=dict(rotation_deg=45,maximum_range_difference_m=maximum_range_change,maximum_local_increment_difference_m=maximum_local_change,
        known_anchor_count=1,magnetic_heading=False,earth_rotation_heading=False,
        conclusion='一个已知点与六条距离固定形状/原点参考，但整体绕点旋转仍不可观；局部轴约定不是地图方位观测')
    profiles={name:dict(cfg,gyro_bias_deg_per_hour=cfg.get('gyro_bias',0)*180/np.pi*3600,
        gravity_leak_std_mps2=9.80665*np.deg2rad(cfg.get('tilt_std_deg',0))) for name,cfg in PROFILES.items()}
    audit=dict(date='2026-10-29',sources=sources,profiles=profiles,static_record_count=len(static),observability=proof,
        parameter_classification={'IMU残余偏置':'工程敏感性假设；结合短时静止统计说明量级，未作为器件稳定性指标',
         'IMU偏置游走':'形式来自Kalibr；增强档陀螺游走数值采用LIO-SAM配置，其他档为敏感性设定',
         '重力泄漏':'倾角残余工程假设，按9.80665乘弧度计入水平加速度',
         'UWB':'10厘米主档参考公开产品量级、Omni-swarm实机权重；5/20厘米是校准/保守对照；3厘米链路偏置为假设',
         'VIO/LIO':'输出增量误差敏感性模型；未从室内ATE倒推，也不冒称开源室外前端实跑结果'},
        papers=[{'title':'Omni-swarm','url':'https://arxiv.org/html/2103.04131v5','use':'室外实机、多观测及多起点运动初始化'},
         {'title':'Accurate position tracking with a single UWB anchor','url':'https://arxiv.org/html/2005.10648','use':'观测模型含磁场航向，不可用于证明六轴IMU的地图航向可观'},
         {'title':'VIR-SLAM','url':'https://arxiv.org/abs/2006.00420','use':'单锚点与视觉惯性组合的相关研究，不把其室内验证当作本项目室外精度依据'},
         {'title':'NTU VIRAL dataset','url':'https://ntu-aris.github.io/ntu_viral_dataset/','use':'室外停车场/广场序列识别，仅使用场景说明，不直接套其三锚点结果'},
         {'title':'Relative Transformation Estimation','url':'https://arxiv.org/abs/2202.00279','use':'相对坐标变换、几何构型与可观性分析'},
         {'title':'LinkTrack官方资料','url':'https://www.nooploop.com/linktrack/','use':'10厘米是厂商典型测距准确度，仿真标准差另为显式假设'}])
    (folder/'参数来源与可观性核对.json').write_text(json.dumps(audit,ensure_ascii=False,indent=2)+'\n')
    print('参数来源、',len(static),'组静止统计与45度整体旋转不变性已核对：',proof)


if __name__=='__main__':main()
