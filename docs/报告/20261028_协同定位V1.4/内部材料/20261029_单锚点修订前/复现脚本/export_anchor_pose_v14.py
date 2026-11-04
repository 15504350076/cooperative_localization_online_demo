#!/usr/bin/env python3
"""导出锚点仿真的位置消息字段，统一坐标但不覆盖现有ROS话题。"""
import csv
import numpy as np
from evaluate_sensor_anchor_v14 import OUT


def main():
    target=OUT/'pose_exports';target.mkdir(exist_ok=True)
    for profile in ('imu','vio','lio'):
        name=f'{profile}_peer_a3_s0.05';a=np.load(OUT/(name+'.npz'))
        with (target/(name+'_poses.csv')).open('w',newline='') as stream:
            writer=csv.writer(stream)
            writer.writerow(['timestamp_ns','frame_id','reference_node_id','node_id','x_m','y_m','yaw_rad','position_valid','yaw_valid','field_frame_id','field_x_m','field_y_m'])
            for t,position in zip(a['time_s'],a['position']):
                for n in range(3):
                    relative=position[n]-position[0]
                    writer.writerow([10_000_000_000+round(t*1e9),'coop_anchor_field_relative',1,n+1,*relative,0,1,0,'coop_anchor_field',*position[n]])
        print(target/(name+'_poses.csv'))


if __name__=='__main__':main()
