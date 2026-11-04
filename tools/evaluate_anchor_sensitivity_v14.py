#!/usr/bin/env python3
"""补充IMU残余零偏、里程计精度、起点定向和锚点测量误差敏感性。"""
from copy import deepcopy
import json
import numpy as np
import evaluate_sensor_anchor_v14 as sim


def main():
    out=sim.OUT/'sensitivity';out.mkdir(exist_ok=True)
    a=np.load(sim.OUT/'sensor_inputs.npz');t,p,yaw,v=(a[k] for k in ('time_s','truth','yaw','velocity'))
    original=deepcopy(sim.PROFILES);records=[]
    cases=[]
    for bias in (.005,.01):
        for anchors in (0,3):cases.append(dict(name=f'imu_bias{bias}_a{anchors}',profile='imu',anchors=anchors,imu_bias=bias))
    for profile in ('vio','lio'):
        for anchors in (0,3):cases.append(dict(name=f'{profile}_double_error_a{anchors}',profile=profile,anchors=anchors,scale_factor=2.))
    for sigma in (.10,.30):cases.append(dict(name=f'anchor_survey{sigma}',profile='lio',anchors=4,anchor_survey_std_m=sigma))
    for deg in (1.,3.):cases.append(dict(name=f'initial_heading{deg}',profile='lio',anchors=0,heading_std_deg=deg))
    for profile in ('imu','lio'):
        for rate in (1,5):cases.append(dict(name=f'{profile}_anchor_rate{rate}',profile=profile,anchors=3,anchor_rate_hz=rate))
    for case in cases:
        sim.PROFILES=deepcopy(original)
        if 'imu_bias' in case:sim.PROFILES['imu']['accel_bias_std_mps2']=case['imu_bias']
        if 'scale_factor' in case:
            for key in ('scale_std','yaw_drift_std_rps','translation_noise_m_sqrt_m'):
                sim.PROFILES[case['profile']][key]*=case['scale_factor']
        anchor_std=case.get('anchor_survey_std_m',.02)
        data,initial,angle,surveyed,peer,ars=sim.inputs(t,p,yaw,v,20,heading_std_deg=case.get('heading_std_deg',.2),anchor_survey_std_m=anchor_std)
        sigma=.05
        estimate,correction=sim.fusion(case['profile'],case['anchors'],True,sigma,data,initial,surveyed,
            peer[None]+sigma*data['peer_standard'],ars[None]+sigma*data['anchor_standard'],t[::5],anchor_survey_std_m=anchor_std,anchor_rate_hz=case.get('anchor_rate_hz',10))
        metrics=sim.metrics(estimate,p[::5]);r=dict(case,range_sigma_m=sigma,trials=metrics)
        records.append(r)
        np.savez_compressed(out/(case['name']+'.npz'),time_s=t[::5],position=estimate[0],truth=p[::5],correction_m=correction[0])
        (out/'summary.json').write_text(json.dumps({'conditions':records,'trials_per_condition':20},ensure_ascii=False,indent=2)+'\n')
        vals=[r['position_rmse_m'] for r in metrics]
        print(f'[敏感性完成] {case["name"]}: {min(vals):.3f}～{max(vals):.3f}米',flush=True)


if __name__=='__main__':main()
