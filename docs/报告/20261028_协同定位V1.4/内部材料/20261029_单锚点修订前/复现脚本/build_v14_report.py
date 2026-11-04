#!/usr/bin/env python3
"""V1.4源稿与配套说明。默认保留已经存在的手工源稿，--refresh-content才整理正文。"""
import argparse
from collections import defaultdict
import hashlib
import json
from pathlib import Path
import re
import shutil

import numpy as np
from build_stage_report import render_word
from evaluate_sensor_anchor_v14 import OUT, ROOT

DEST=ROOT/'docs/报告/20261028_协同定位V1.4'
SOURCE=ROOT/'docs/报告/20261023_协同定位阶段报告/多平台协同定位硬件仿真测试报告与联调方案_V1.3_正式报告版.md'
CURRENT=DEST/'多平台协同定位硬件仿真测试报告与联调方案_V1.4_设备分级与锚点仿真版.md'


def table(title,head,rows):
    return title+'\n\n| '+' | '.join(head)+' |\n|'+'|'.join(['---']*len(head))+'|\n'+''.join('| '+' | '.join(map(str,row))+' |\n' for row in rows)+'\n'


def span(case):
    values=[r['position_rmse_m'] for r in case['trials'] if r['position_rmse_m'] is not None]
    return f'{min(values):.3f}～{max(values):.3f}' if values else '无有效输出'


def prepare_content():
    main=json.loads((OUT/'summary.json').read_text());rte=json.loads((OUT/'rte_evaluation/summary.json').read_text())
    sensitivity=json.loads((OUT/'sensitivity/summary.json').read_text())
    cases={r['name']:r for r in main['conditions']};sens={r['name']:r for r in sensitivity['conditions']}
    total=sum(len(r['trials']) for r in main['conditions'])+len(rte['rte'])+sum(len(r['trials']) for r in sensitivity['conditions'])
    assert total==1400 and len(main['conditions'])==48 and len(rte['rte'])==120 and len(sensitivity['conditions'])==16
    s=SOURCE.read_text()
    s=s.replace('18组带真值仿真完成首轮精度与错误分支评估','保留18组既有基线，新增70种条件、1400次传感器与锚点仿真')
    s=s.replace('独立试验、初始化改进和动态真值验证','比较惯性与视觉/雷达运动输入及固定锚点组合，开展动态实车验证')
    s=s.replace('现有消息接口、三盒数据、在线输出和离线回放程序可用于通信、显示和决策模块的接口联调。',
        '现有消息接口、三盒数据、在线输出和离线回放程序可用于通信、显示和决策模块联调。本轮进一步按设备组合、输入精度和场地布设条件开展仿真，比较增加哪类输入能够带来多大的位置误差改善。')
    definition='本轮还引入固定场地锚点。场地坐标以固定测量标记O为原点、场地方向标记为X轴正向，Z轴向上，Y轴由右手规则确定。三车位置在这套固定坐标中表达，再减去201车同一时刻的位置得到相对位置；这种表达可同时观察编队整体运动和车间相对关系。场景和锚点位置见图16。'
    s=s.replace('图4对照三种坐标的作用，实际输出入口见第3.3节。','图4对照现有三种坐标的作用，实际输出入口见第3.3节。\n\n'+definition)
    s=s.replace('### 2.2 十八组带真值仿真试验','### 2.3 既有RTE基线仿真')
    s=s.replace('### 2.3 已有Gazebo物理仿真载体','### 2.4 已有Gazebo物理仿真载体')
    s=s.replace('### 2.4 大运动范围三车仿真','### 2.5 既有大运动范围对照')
    s=s.replace('### 2.5 已完成工作与后续试验安排','### 2.8 已完成工作与后续试验安排')
    overview='''### 2.2 设备组合、仿真条件与比较方法

本轮采用同一段300秒低速轨迹运动学仿真：前30秒静止，随后三车在约85×115米范围内运动，最高速度约2.39米/秒，最大车间距约24.6米。场地为平面，按视距条件连续生成测距；各流使用统一仿真时间，位置以车载UWB天线点表示，IMU与里程计参考点在模型中与其重合。比较时依次增加运动估计、起点标定和固定UWB锚点，观察相对位置误差及轨迹变化。

视觉惯性里程计（Visual-Inertial Odometry，VIO）由相机和IMU估计本车运动；激光惯性里程计（LiDAR-Inertial Odometry，LIO）由激光雷达和IMU估计本车运动。本轮在它们的位置增量、航向输出上设置尺度、慢漂移和随机误差，用于比较这类输入接入后的定位效果。IMU组则在静止初始化确定重力方向后，由含噪声和残余偏置的水平加速度、角速度传播平面位置。三类组合都包含IMU，VIO/LIO输出模型对应上游局部位姿接口。

'''
    overview+=f'车头方向沿行驶轨迹切线，静止时保持起步方向；新场景最大航向角速度约{main["scene"]["maximum_yaw_rate_rps"]:.3f}弧度/秒，并检查了车体无侧向滑行。\n\n'
    overview+=table('表101 设备组合与投入构成',['组合','设备和现场条件','新增投入与工作','所解决的问题'],[
        ['纯UWB队形','三车UWB与计算单元','基础测距配置','恢复车间距离和相对三角形'],
        ['IMU＋UWB','增加车载IMU、起点位置及方向标定','IMU采集、安装方向与起点测量','用运动传播衔接测距，维持共同轴向'],
        ['VIO＋UWB','每车相机组＋IMU＋UWB','相机处理、内外参和起点标定','利用视觉与惯性估计局部运动'],
        ['LIO＋UWB','每车激光雷达＋IMU＋UWB','雷达处理、安装标定和起点测量','利用雷达与惯性估计局部运动'],
        ['上述组合＋固定锚点','在场地增加1、3或4个UWB固定锚点','增加对应锚点硬件、供电及位置测量','持续补充场地位置约束'],
        ['UWB＋3或4个固定锚点','三车UWB标签与已知位置基站','布设基站并测量坐标','直接在场地坐标内求三车位置'],
    ])
    overview+='现有三盒已配置IMU、UWB、相机和雷达，可在同一硬件基础上比较不同处理组合；固定锚点属于新增场地设备。费用按所需设备、计算处理和布设工作区分，具体金额随型号和报价确定。三锚点组包含9条车锚链路和3条车间链路，逐链路10赫兹对应每秒120条测距；四锚点组对应每秒150条。另将锚点链路降至1、5赫兹作补充试验，见图24。基站配置按约152米的最大场景距离及所需链路频率核对。\n\n'
    overview+=table('表102 本轮仿真输入精度设定',['输入或测量','误差设定','使用方式'],[
        ['车间及锚点测距','独立零均值噪声，标准差2、5、10厘米；10赫兹','同序号方案使用相同测距样本'],
        ['IMU','噪声按100赫兹设定；加速度0.02米/秒²、角速度0.0005弧度/秒，按10赫兹等效平均量融合','运动后自由传播，不持续施加零速约束'],
        ['IMU残余偏置','加速度标准差0.001米/秒²，另测0.005和0.01；角速度0.000015弧度/秒','模拟静止初始化后仍保留的误差'],
        ['VIO输出模型','尺度标准差0.2%；航向漂移0.00003弧度/秒；位置随机项0.020米/√米','50赫兹局部位姿输出；另测误差加倍'],
        ['LIO输出模型','尺度标准差0.05%；航向漂移0.000008弧度/秒；位置随机项0.005米/√米','50赫兹局部位姿输出；另测误差加倍'],
        ['起点标定','各车初始位置标准差5厘米，初始方向标准差0.2度；另测1度和3度','仅起步前使用一次，之后无持续位置输入'],
        ['固定锚点','坐标测量标准差2厘米，另测10厘米和30厘米；持续10赫兹测距','使用1、3、4个锚点分别比较'],
    ])
    overview+='''IMU噪声幅值参考既有静止数据的量级，并加大白噪声及残余偏置作对照。VIO/LIO为本轮设定的输出误差档位，不与具体设备型号绑定。尺度误差作用于运动增量；航向误差随运动时间积累；位置随机项按行驶距离的平方根增长，例如行驶100米时，该项每轴标准差为VIO模型0.20米、LIO模型0.05米。上述参数用于将增加输入的作用算清楚，实车组合由后续采集结果确定。

起点标定与固定锚点分开比较。起点标定提供一次位置和方向；固定锚点在运动中持续提供距离。一个已知位置点提供位置参考，场地方向由另一方向标记确定；三个不共线锚点可在给定场地坐标中约束二维位置，第四个锚点增加冗余并改善场地覆盖。

本轮共70种条件、1400次仿真运行，每种条件重复20次，属于蒙特卡洛重复分析。每种条件的20次重复用于观察同条件波动，完整保留20次结果，不以单次最好值作结论。同一条件内独立生成噪声；不同方案间使用同序号配对样本，便于判断改变设备或约束后的影响。其中，48种传感器与锚点组合运行960次，6种RTE输入条件运行120次，16种精度敏感性条件运行320次。

位置误差为估计相对位置向量与同一时刻真值向量之差的模长；位置均方根误差（Root Mean Square Error，RMSE）按202、203车合并统计，单位为米。表中范围表示20次运行的最小和最大RMSE，图中同时给出逐次分布或时间曲线。固定场地坐标的绝对位置误差另外保存。纯UWB队形使用其规范相对坐标；其他固定场地方案使用共同场地轴向，均与各自坐标中的真值比较。

RTE组读取各车自己的局部里程计和车间距离，用于估计未知的局部坐标关系。起点已经标定或场地布有锚点的组，在共同场地坐标中直接更新位置：IMU采用位置、速度传播，VIO/LIO采用局部位移增量传播，再用车间和锚点距离进行扩展卡尔曼更新。该对照程序在开发机独立运行，默认UWB和RTE程序保持原样。

'''
    s=s.replace('### 2.3 既有RTE基线仿真',overview+'### 2.3 既有RTE基线仿真')
    compare='''### 2.6 起点标定与固定锚点对照

先固定测距噪声标准差为5厘米，依次比较仅起点标定、增加车间UWB，再增加固定锚点。这样每一步只在同一类运动输入上增加位置约束，见表103及图17、图18。三车运动和起点测量样本保持一致，以下误差均为固定场地轴向下的相对位置RMSE。

'''
    rows=[]
    for profile,label in [('imu','IMU'),('vio','VIO输出模型'),('lio','LIO输出模型')]:
        rows.append([label,span(cases[f'{profile}_initial_only'])]+[span(cases[f'{profile}_peer_a{n}_s0.05']) for n in (0,1,3,4)])
    compare+=table('表103 5厘米测距下逐项增加约束的结果',['运动输入','仅起点标定（米）','加车间UWB（米）','再加1锚点（米）','采用3锚点（米）','采用4锚点（米）'],rows)
    absolute=[]
    for profile,label in [('imu','IMU'),('vio','VIO输出模型'),('lio','LIO输出模型')]:
        case=cases[f'{profile}_peer_a3_s0.05'];v=[r['field_position_rmse_m'] for r in case['trials']]
        absolute.append([label,span(case),f'{min(v):.3f}～{max(v):.3f}'])
    compare+=table('表108 3锚点条件下相对位置与场地位置误差',['运动输入','相对位置RMSE范围（米）','场地位置RMSE范围（米）'],absolute)
    compare+='相对位置误差由202、203车减去201车后计算；场地位置误差则使用三车在固定场地坐标中的位置。前者对应编队相对关系，后者对应整体轨迹。\n\n'
    for profile,label in [('imu','IMU'),('vio','VIO输出模型'),('lio','LIO输出模型')]:
        compare+=f'{label}从仅起点标定的{span(cases[f"{profile}_initial_only"])}米，加入车间UWB后为{span(cases[f"{profile}_peer_a0_s0.05"])}米，加入3个固定锚点后为{span(cases[f"{profile}_peer_a3_s0.05"])}米。\n\n'
    compare+='三锚点组的结果同时受测距、输入误差和传播方式影响：IMU组估计位置与速度，VIO/LIO组按带噪位移增量更新。选配时综合位置精度、输入连续性、场地布设和任务需求，按表中条件选择。\n\n'
    compare+='单个固定锚点只提供一条径向距离，效果取决于运动方向、已有起点信息和车间几何；3个不共线锚点提供二维位置约束。4锚点相较3锚点的改善与设备增加同时列示，实际布设可结合覆盖和冗余需要选择。图20给出三类输入在同一次重复试验中的误差曲线，图21给出固定场地坐标的三车轨迹。\n\n'
    rows=[]
    for sigma in (.02,.05,.10):rows.append([int(sigma*100),span(cases[f'uwb_triangle_s{sigma:.2f}'])]+[span(cases[f'{p}_peer_a3_s{sigma:.2f}']) for p in ('imu','vio','lio')])
    compare+=table('表104 不同测距精度下的结果',['测距标准差（厘米）','纯UWB规范队形（米）','IMU＋3锚点（米）','VIO＋3锚点（米）','LIO＋3锚点（米）'],rows)
    compare+=f'直接使用固定锚点进行UWB多边定位时，5厘米测距组采用3锚点的相对位置RMSE为{span(cases["uwb_anchors_a3_s0.05"])}米，4锚点为{span(cases["uwb_anchors_a4_s0.05"])}米。结合运动输入后，滤波可利用连续运动信息抑制逐次测距波动，效果与单次多边定位分别保存。图19分别表示规范队形与固定场地轴向下的测距精度变化。\n\n'
    groups=defaultdict(list)
    for row in rte['rte']:groups[(row['profile'],row['range_sigma_m'])].append(row)
    rows=[]
    for profile in ('vio','lio'):
        for sigma in (.02,.05,.10):
            rr=groups[(profile,sigma)];valid=[r['position_rmse_m'] for r in rr if r['position_rmse_m'] is not None]
            rows.append([profile.upper()+'输出模型',int(sigma*100),len(rr),f'{min(valid):.3f}～{max(valid):.3f}'])
    compare+=table('表106 未知初始坐标关系的RTE结果',['局部里程计输入','测距标准差（厘米）','重复次数','相对位置RMSE范围（米）'],rows)
    compare+='RTE组使用同一现有算法库，输入为带不同误差的局部里程计位姿。RTE误差统计初始化后的有效位置输出，起点标定组统计300秒整段；两类结果各自保留完整时间曲线。表106和图23显示，较精细的局部运动输入能够降低典型误差，但未知坐标关系的初始化和滑动窗口修正仍会影响结果。起点标定和固定锚点组增加了位置与方向信息，采用共同坐标下的位置更新；两组的输入条件与处理方式在表中分别说明。\n\n'
    compare+='''### 2.7 输入精度与布设精度的影响

补充试验保持5厘米测距噪声，分别增大IMU残余加速度零偏、VIO/LIO输出误差、锚点测量误差及起点方向误差。图22给出IMU和起点定向的变化，表105列出全部补充结果。

'''
    labels={'imu_bias0.005_a0':'IMU偏置0.005米/秒²，无固定锚点','imu_bias0.005_a3':'IMU偏置0.005米/秒²，3锚点',
        'imu_bias0.01_a0':'IMU偏置0.01米/秒²，无固定锚点','imu_bias0.01_a3':'IMU偏置0.01米/秒²，3锚点',
        'vio_double_error_a0':'VIO输出误差加倍，无固定锚点','vio_double_error_a3':'VIO输出误差加倍，3锚点',
        'lio_double_error_a0':'LIO输出误差加倍，无固定锚点','lio_double_error_a3':'LIO输出误差加倍，3锚点',
        'anchor_survey0.1':'LIO＋4锚点，锚点测量标准差10厘米','anchor_survey0.3':'LIO＋4锚点，锚点测量标准差30厘米',
        'initial_heading1.0':'LIO＋车间UWB，起点方向标准差1度','initial_heading3.0':'LIO＋车间UWB，起点方向标准差3度',
        'imu_anchor_rate1':'IMU＋3锚点，锚点测距1赫兹','imu_anchor_rate5':'IMU＋3锚点，锚点测距5赫兹',
        'lio_anchor_rate1':'LIO＋3锚点，锚点测距1赫兹','lio_anchor_rate5':'LIO＋3锚点，锚点测距5赫兹'}
    compare+=table('表105 精度敏感性结果',['改变的条件','重复次数','相对位置RMSE范围（米）'],[[labels[r['name']],len(r['trials']),span(r)] for r in sensitivity['conditions']])
    compare+=f'IMU残余加速度零偏增大到0.01米/秒²时，只有起点标定和车间UWB的结果为{span(sens["imu_bias0.01_a0"])}米；保持相同输入、增加3个固定锚点后为{span(sens["imu_bias0.01_a3"])}米。起点方向标准差由0.2度增大到3度时，LIO＋车间UWB组由{span(cases["lio_peer_a0_s0.05"])}米变为{span(sens["initial_heading3.0"])}米。结果说明，运动输入精度与统一坐标的建立都需在实车部署中落实。\n\n'
    s=s.replace('### 2.8 已完成工作与后续试验安排',compare+'### 2.8 已完成工作与后续试验安排')
    s=s.replace('| 大运动范围仿真 | 完成300秒、约85×115米大运动范围三车运动、测距连续输入和逐历元误差统计 |',
        '| 设备与锚点仿真 | 完成70种条件、1400次运行，比较IMU、VIO/LIO输入、起点标定及1/3/4锚点，另保留既有大运动对照 |')
    s=s.replace('表5、相关仿真图示、data/rte_expanded_20261025、docs/visualization/rte_coordinates.html',
        '表103、表104、表105、图16～24、data/sensor_anchor_v14_20261028')
    s=s.replace('### 6.2 实车运动采集\n', '### 6.2 实车运动采集\n\n首轮按三锚点方案布点，测量固定站坐标、起点位置和车头方向，记录UWB天线安装高度，并将固定站与车载天线布置在近似同高平面。先复测1、5、10赫兹锚点链路的更新，再用同一段运动数据比较IMU和视觉/雷达运动输入。\n')
    # 保留既有坐标及消息，新锚点结果作为独立开发机输出。
    marker='| 任务坐标 | 原点为首次有效定位时201盒的位置；轴向与RTE固定相对坐标相同 | 查看三车整体运动轨迹，供任务轨迹对照使用 |'
    s=s.replace(marker,marker+'\n| 锚点场地坐标 | 场地测量标记O为原点，沿场地方向标记为正X，Z向上；三车均可移动 | 输出场地位置，并按同一轴向求各车相对201的位置 |')
    marker='各车局部里程计状态分别使用rte_odom_1、rte_odom_2、rte_odom_3。'
    s=s.replace(marker,'本轮锚点仿真结果写入独立CSV和NPZ。CSV中的x_m、y_m为减去201车后的相对位置，field_x_m、field_y_m为场地位置；frame_id为coop_anchor_field_relative，场地坐标名为coop_anchor_field。位置有效位与原有消息同义，航向有效位按位置试验置为false。\n\n'+marker)
    s=s.replace('### 5.2 录包分析与运行排查','### 5.3 录包分析与运行排查')
    new_commands='''新增设备、锚点与精度敏感性仿真使用以下入口。完整命令和编译环境见配套《阅读与复现说明》；逐次结果汇总为data/sensor_anchor_v14_20261028/全部仿真结果.csv。

```bash
PYTHONPATH=tools python3 tools/evaluate_sensor_anchor_v14.py --trials 20 --skip-rte
PYTHONPATH=tools python3 tools/evaluate_sensor_anchor_v14.py --rte-only --trials 20 --output data/sensor_anchor_v14_20261028/rte_evaluation
PYTHONPATH=tools python3 tools/evaluate_anchor_sensitivity_v14.py
PYTHONPATH=tools python3 tools/collect_v14_results.py
PYTHONPATH=tools python3 tools/plot_sensor_anchor_v14.py
PYTHONPATH=tools python3 tools/export_anchor_pose_v14.py
```

'''
    s=s.replace('第2章给出分组结果。重新计算18组RTE仿真时，在工程根目录执行：',
        new_commands+'第2章给出分组结果。重新计算18组既有RTE仿真时，在工程根目录执行：')
    jump=json.loads((OUT/'jump_analysis/summary.json').read_text())
    row=jump['top_events'][0]
    explanation=f'''### 5.2 RTE跳变与图14的含义

图14固定展示既有300秒场景中“无误差里程计、2厘米测距噪声、重复试验1”的计算过程。上图为202、203车相对位置误差，中图为滑动窗口的测距拟合均方根残差，下图为同一当前里程计位置经过修正前、后坐标变换所得的位置差。三幅图横轴均为仿真时间，纵轴均以米计；竖线对应147.60秒和294.48秒的修正时刻。初始化之前和无有效输出时段留空。

294.48秒时，203车坐标变换修正引起约{row['correction_displacement_m']:.2f}米的位置变化；位置误差由{row['previous_error_m']:.3f}米变为{row['error_m']:.3f}米，而测距拟合残差由{row['residual_before_m']:.4f}米变为{row['residual_after_m']:.4f}米。修正次数同时由697变为698。重建该5秒窗口后，计算得到相同的测距残差，部分平移分量的估计标准差达到约{max(row['translation_std_m']):.2f}米，说明这一窗口对坐标变换的约束较弱。

程序每次滑窗求解后直接更新相对平移和转角。初次建系之后，输出会随新窗口解变化；当前实现对后续修正没有增加连续位置先验或单次位移约束。因此，某些窗口虽然仍能较好拟合车间距离，更新后的相对位置却会发生突变。跳变来自该次坐标变换更新，不能由测距残差的大小单独判断。

图14下图的修正变化由输入里程计和输出变换重建，不通过滤波平滑曲线。固定锚点对照增加场地位置约束，在连续运动传播的基础上更新位置，其逐时效果见图20。图15仍保留既有RTE轨迹与真值的对照。

'''
    s=s.replace('图11给出18组RTE仿真的误差分布；图12将位置误差与测距拟合残差放在一起，便于查找测距拟合较好、位置却偏差较大的试验。图14、图15分别给出大运动范围仿真的误差变化和轨迹对照。',
        '图11给出18组RTE仿真误差分布；图12同时给出位置误差与测距拟合残差。测距拟合残差是窗口内预测车间距离与输入UWB测距差值的均方根，单位为米。图14、图15分别给出既有大运动范围仿真的修正过程与轨迹对照，第5.2节解释其中的位置跳变。')
    s=s.replace('![图14 扩展仿真误差与解算状态]','![图14 RTE位置误差、测距残差与修正变化]')
    newfigs='''
### 5.4 设备组合与锚点试验图线

图16中三个起点分别为（15，65）、（30，65）、（20，78）米；四个锚点位于（0，0）、（110，0）、（0，130）、（110，130）米。起点位置和方向按表102加入测量误差，锚点坐标另按设定误差生成。最大仿真锚点测距约152米，测距精度按表102给定的视距工况生成。

![图16 三车场地轨迹与锚点布置](figures/v14_16_场地坐标与锚点布置.png)

图17按同一输入逐项增加条件，每个箱线图包含20次运行。为同时看清米级与厘米级结果，纵轴采用对数刻度；它仍表示以米计的相对位置RMSE。图18的点为20次RMSE的中位数，误差棒为最小至最大范围，横轴直接对应新增固定锚点数量。

![图17 逐项增加输入与位置约束的效果](figures/v14_17_逐项增加条件的误差变化.png)

![图18 固定锚点数量与定位误差](figures/v14_18_锚点数量与误差.png)

图19分开展示纯UWB规范队形坐标与固定场地轴向相对坐标。每条曲线内比较不同测距精度的影响，条件和坐标口径与表104一致。

![图19 测距精度与相对定位误差](figures/v14_19_测距精度与定位结果.png)

![图20 运动过程中锚点辅助的定位误差](figures/v14_20_锚点辅助逐时误差.png)

![图21 场地坐标中的估计轨迹与真值](figures/v14_21_场地坐标轨迹对照.png)

![图22 IMU与起点定向精度的影响](figures/v14_22_输入与标定精度敏感性.png)

图23对应表106，使用RTE原有求解方式，分别输入VIO和LIO输出模型。每种测距噪声下运行20次，图中保留全部结果；纵轴采用对数刻度。

![图23 RTE局部里程计精度对照](figures/v14_23_RTE里程计精度对比.png)

图24在保留10赫兹车间测距的条件下，仅改变每条锚点链路的频率。三锚点方案采用1、5、10赫兹时，全部测距吞吐分别为每秒39、75、120条，可结合设备调度能力比较精度与处理量。

![图24 锚点测距频率与定位误差](figures/v14_24_锚点测距频率与误差.png)

'''
    s=s.replace('### 5.3 录包分析与运行排查',explanation+'### 5.3 录包分析与运行排查')
    s=s.replace('## 6 计算盒接入与实车试验',newfigs+'## 6 阶段结论与实车试验')
    s=s.replace('[计算盒接入与实车试验](#chapter_6)','[阶段结论与实车试验](#chapter_6)')
    start=s.index('表13 复现资料清单')
    s=s[:start]+'全部新增数据位于data/sensor_anchor_v14_20261028，具体命令及文件对应关系见配套《阅读与复现说明》。\n\n'+table('表13 复现资料清单',['内容','打开位置','用途'],[
        ['仿真分组结果','全部仿真结果.csv、评估总清单.json','查看1400次运行和70种条件'],
        ['输入、位置与真值','sensor_inputs.npz、各条件NPZ、summary.json','复算传感器输入及位置误差'],
        ['RTE与跳变','rte_evaluation/、jump_analysis/','逐时输出与坐标变换修正重建'],
        ['精度与频率变化','sensitivity/','核对16种补充条件'],
        ['位置结果及重算','pose_exports/；本报告配套复现说明与脚本','读取场地和相对位置，重新生成图表和报告'],
    ])
    # 章节内使用图号与有用文件入口，避免另设版本页。
    s+='''
### 6.5 阶段结论

现阶段已形成三盒数据采集、默认UWB队形定位、独立RTE库、消息接口和开发机仿真工具。六组静止数据用于采集及回放分析；本轮新增70种条件、1400次仿真，将传感器输入、测距精度、起点标定、锚点数量及测量精度分别比较。

纯UWB方案继续作为现有相对队形输出。对于需要固定场地轴向或整体运动轨迹的场景，本轮采用起点标定建立共同坐标，再比较持续增加固定锚点的效果；3个不共线锚点配合连续运动输入能够抑制累计漂移，4锚点可提供覆盖和冗余。设备投入可按所需能力选择：只看队形采用原有UWB配置，需要统一场地位置时增加锚点，再按实际数据选用IMU、VIO或LIO运动输入。

RTE在未知初始坐标关系时承担局部坐标对齐。新一轮输入精度对照和既有窗口重建表明，上游里程计质量、窗口运动几何和后续坐标变换修正共同影响误差；图14已给出可核对的跳变实例。增加起点和场地约束后，定位问题可转为共同坐标中的连续位置更新，本轮仿真已给出相应输入、计算程序、误差曲线和导出结果。

后续实车试验沿用本轮低速、20～30米编队间距及共同坐标定义，先测量起点和锚点，使用同一段运动数据比较不同输入组合，再将位置、坐标系和结果时间接入通信与规划控制。亚米级相对定位目标依据实车真值复测，并以本轮配对仿真结果选择首轮配置。
'''
    # 在结论中直接列出代表性结果及实际条件。
    conclusion=f'以5厘米测距、3个固定锚点、锚点坐标测量标准差2厘米的条件为例，IMU、VIO、LIO组合20次相对位置RMSE分别为{span(cases["imu_peer_a3_s0.05"])}、{span(cases["vio_peer_a3_s0.05"])}、{span(cases["lio_peer_a3_s0.05"])}米。该结果对应本轮输出级平面仿真，VIO/LIO的尺度和漂移设定见表102。\n\n'
    s=s.replace('纯UWB方案继续作为现有相对队形输出。',conclusion+'纯UWB方案继续作为现有相对队形输出。')
    # 所有表号按正文出现顺序统一，引用同步替换。
    old_numbers=re.findall(r'^表(\d+) ',s,re.M);assert len(old_numbers)==len(set(old_numbers))
    mapping={old:str(i+1) for i,old in enumerate(old_numbers)}
    s=re.sub(r'表(\d+)(?!\d)',lambda m:'表'+mapping.get(m[1],m[1]),s)
    return s,dict(total_trials=total,conditions=70,numbered_tables=len(old_numbers),figures=24,table_mapping=mapping)


def supplements(info):
    readme='''# V1.4阅读与复现说明

当前正式稿为《多平台协同定位硬件仿真测试报告与联调方案_V1.4_设备分级与锚点仿真版》。V1.3保留在原目录，版本和逐次改动另存工程内部文档，不放入报告正文。

## 手工修改与导出

编辑本目录同名Markdown后，在工程根目录执行：

```bash
PYTHONPATH=/tmp/cooperative_report_deps python3 tools/render_formal_report.py
python3 tools/export_formal_pdf.py
```

`tools/build_v14_report.py`默认保留已存在的Markdown并重新排版，只有显式指定`--refresh-content`时才从V1.3与统计结果重新整理源稿。完整重建正文前请保存手工修改。旧tools/build_formal_report.py用于V1.3，V1.4使用tools/build_v14_report.py。

## 仿真复现

```bash
PYTHONPATH=tools OPENBLAS_NUM_THREADS=1 python3 tools/evaluate_sensor_anchor_v14.py --trials 20 --skip-rte
PYTHONPATH=tools OPENBLAS_NUM_THREADS=1 python3 tools/evaluate_sensor_anchor_v14.py --rte-only --trials 20 --output data/sensor_anchor_v14_20261028/rte_evaluation
PYTHONPATH=tools OPENBLAS_NUM_THREADS=1 python3 tools/evaluate_anchor_sensitivity_v14.py
PYTHONPATH=tools python3 tools/analyze_rte_jumps_v14.py
PYTHONPATH=tools python3 tools/plot_sensor_anchor_v14.py
PYTHONPATH=tools python3 tools/export_anchor_pose_v14.py
PYTHONPATH=tools python3 tools/collect_v14_results.py
```

仿真使用已有build_rte_verify中的两个库与evaluate_rte入口，构建方法见data/rte_expanded_20261025/README.md。新结果全部在data/sensor_anchor_v14_20261028；全部仿真结果.csv逐行列出1400次运行，评估总清单.json给出70种条件与次数；传感器输入保存为sensor_inputs.npz，场地位置与真值示例保存为各条件NPZ，20次逐组结果保存为summary.json。RTE的120份逐时输出保存在rte_evaluation/rte。原始实测录包与V1.3结果不改动。

本轮为平面输出级模型：IMU采用水平加速度传播，VIO/LIO采用带尺度、航向漂移和随机位移误差的局部位姿输出。所有组在相同轨迹上比较，起点位置/航向测量、锚点位置测量均有独立误差。不同输入之间共享相同序号的扰动样本。没有人为加入丢包、测距中断、动态重定位或原点复位。

## 图线来源与字体

报告曲线由Matplotlib读取算法输出和仿真真值绘制，未使用生成式图像工具。图9对应`figures/16_RTE逐车定位误差`，读取`data/rte_expanded_20261025/exp_odometry_0.02_seed0_output.csv`及同名`.truth.csv`；另外两幅子图将`0.02`分别换为`0.05`、`0.10`。每个时刻的误差为估计相对位置与插值到该时刻的真值之差的二维长度，非有效状态留空。

只重新绘图、沿用已保存的仿真输出时，在工程根目录执行：

```bash
PYTHONPATH=tools python3 tools/plot_sensor_anchor_v14.py
```

本命令不重跑仿真；随后用前述两个导出命令更新Word/PDF。曲线图同时提供360 dpi PNG、保留文字的SVG和嵌入字体的矢量PDF，可打开`figures/16_RTE逐车定位误差.pdf`放大或复制图中文字。

绘图统一使用简体中文字形。开发机未安装中易黑体及Times New Roman时，图中文字采用文泉驿微米黑，纯数字刻度采用Liberation Serif；Word中原有字体设置不变。旧图误用了Noto CJK字体集中的JP字形，现已更正；报告PDF导出关闭JPEG有损压缩和图片降采样。2026年10月29日此次修改只调整绘制和导出，输入、误差计算和报告正文未变。

## 图14与坐标说明

图14使用既有exp_odometry_0.02_seed0的原始输入与输出。下图在相同当前里程计位置重建修正前后的坐标变换，不用真值计算修正幅度；位置误差仍与独立真值比较。jump_analysis/summary.json保存294.48秒事件的重建残差、平移不确定度和程序对应位置。

本轮场地坐标以固定测量标记O为原点，沿场地方向标记为正X，Z向上；相对位置为各车场地位置减去201车场地位置。无连续锚点的组在起点由场地标记测量位置和方向，之后只使用运动输入与所选测距。一个位置点不单独提供航向。默认UWB几何坐标及原RTE坐标不修改。

## 图1与图4

图1可编辑PPTX和SVG仍位于figures/，文字与连线为独立对象。微调后导出PNG替换01_系统与联调边界.png，再重新排版即可。

图4为真实坐标示意页面截图，来自docs/visualization/rte_coordinates.html。在1600×1000窗口中选择第一种情景、暂停至12秒可复现该截图。详细操作属于配套复现资料，报告正文集中说明坐标定义与用途。

## 自查

```bash
PYTHONPATH=tools python3 -m unittest tools/test_sensor_anchor_v14.py
PYTHONPATH=tools python3 tools/check_v14_results.py
```

算法类型可参考官方实现说明：VIO的相机与IMU组合见[VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)，LIO的雷达与IMU组合见[FAST-LIO](https://github.com/hku-mars/FAST_LIO)。上述链接用于核对前端类型，本轮误差参数采用报告表中显式设定值。
'''
    (DEST/'阅读与复现说明.md').write_text(readme)
    questions=['本项目现有默认输出是什么，使用哪种传感器，坐标轴如何定义？',
        '本轮70种条件和1400次运行如何组成？为什么每种条件重复20次？',
        'VIO和LIO各自用了什么设备？本轮输入是图像/点云还是局部位姿输出？',
        'IMU白噪声、残余零偏分别怎样进入位置传播，为什么二者影响不同？',
        '起点标定与运动中持续测距的固定锚点有什么区别？仅给一个位置点能否确定方向？',
        '场地坐标的原点、X/Y/Z轴是什么？相对位置由哪个向量减哪个向量？',
        '5厘米测距下，同一IMU输入从起点传播、加入车间UWB、加入3锚点后误差各是多少？',
        '增加相机、雷达或固定锚点分别增加哪些硬件或处理工作？没有报价时如何比较投入？',
        'RTE组与起点已经标定的融合组，已知条件和计算方法有什么不同？',
        '图14的三幅图各表示什么、单位是什么？294.48秒发生了什么？',
        '为什么1.7厘米的测距拟合残差会与2米级位置误差同时出现？',
        '表中的误差范围是逐时误差还是20次试验RMSE的范围？RMSE合并了哪些车？',
        '纯UWB队形误差、固定场地相对误差和场地绝对误差，三者是否同一指标？',
        '3锚点与4锚点怎样选择？本轮测量噪声、锚点坐标误差和最大基站距离是多少？',
        '起点方向从0.2度变为3度、IMU偏置从0.001变为0.01米/秒²，各改变了哪组结果？',
        '本轮哪些是仿真结果、哪些来自静止实测，亚米级目标下一步如何用实车真值验证？',
        '三锚点逐链路10赫兹为什么对应每秒120条测距？降至1或5赫兹后如何比较？',
        '本轮车头方向和行驶方向如何约束，最高车速及航向角速度是多少？']
    text='# V1.4报告理解核对问题\n\n建议先不看正文，逐项用自己的话回答，并指出对应表格或图片。对不上数据的地方记录下来。\n\n'
    text+='\n'.join(f'{i+1}. {q}' for i,q in enumerate(questions))+'\n\n'
    text+='核对入口：设备与参数在第2.2节，分组结果在第2.6、2.7节，跳变与图14在第5.2节，坐标及导出字段在第3.3节，结论在第6.5节。具体数值以报告和summary.json为准。\n'
    (DEST/'V1.4_报告理解核对问题.md').write_text(text)
    shutil.copy2(ROOT/'tools/evaluate_sensor_anchor_v14.py',DEST/'内部材料/evaluate_sensor_anchor_v14.py')
    scripts=['build_v14_report.py','evaluate_sensor_anchor_v14.py','evaluate_anchor_sensitivity_v14.py','analyze_rte_jumps_v14.py','plot_sensor_anchor_v14.py','collect_v14_results.py','test_sensor_anchor_v14.py','check_v14_results.py','render_formal_report.py','export_formal_pdf.py','export_anchor_pose_v14.py','build_stage_report.py','evaluate_rte.py','evaluate_rte.cpp','evaluate_rte_expanded.py','evaluate_uwb_formation_sim.py','plot_expanded_simulation.py','report_plot_fonts.py']
    (DEST/'复现脚本').mkdir(exist_ok=True)
    manifest={'version':'1.4','date':'2026-10-28',**info,'hashes':{}}
    for name in scripts:
        p=ROOT/'tools'/name
        if p.exists():shutil.copy2(p,DEST/'复现脚本'/name)
        if p.exists():manifest['hashes'][str(p.relative_to(ROOT))]=hashlib.sha256(p.read_bytes()).hexdigest()
    for p in (CURRENT,OUT/'summary.json',OUT/'rte_evaluation/summary.json',OUT/'sensitivity/summary.json'):
        manifest['hashes'][str(p.relative_to(ROOT))]=hashlib.sha256(p.read_bytes()).hexdigest()
    (DEST/'V1.4_版本与复现清单.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--refresh-content',action='store_true');args=parser.parse_args()
    DEST.mkdir(exist_ok=True)
    source,info=prepare_content()
    if not CURRENT.exists() or args.refresh_content:CURRENT.write_text(source)
    supplements(info)
    render_word(CURRENT.read_text(),CURRENT.with_suffix('.docx'),title='多平台协同定位硬件仿真测试报告与联调方案',header='')
    print(f'已生成：{CURRENT}；{info}')


if __name__=='__main__':main()
