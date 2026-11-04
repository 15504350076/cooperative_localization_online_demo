#!/usr/bin/env python3
"""V1.4室外参数审查版；条件、方法、统计与曲线逐项相邻，默认保留手工源稿。"""
import argparse
import json
import re
import shutil
from pathlib import Path
import numpy as np
from build_stage_report import render_word
from evaluate_single_anchor import OUT, ROOT, PROFILES, SIGMAS

DEST=ROOT/'docs/报告/20261028_协同定位V1.4'
STEM='多平台协同定位硬件仿真测试报告与联调方案_V1.4_设备分级与锚点仿真版'
CURRENT=DEST/(STEM+'.md')
BEFORE=DEST/'内部材料/20261029_室外参数与报告结构修订前'/CURRENT.name


def generate():
    old=BEFORE.read_text();summary=json.loads((OUT/'summary.json').read_text());audit=json.loads((OUT/'参数与可观性核对/参数来源与可观性核对.json').read_text())
    static=json.loads((OUT/'参数与可观性核对/静止片段统计核对.json').read_text());budget=json.loads((OUT/'成本预算.json').read_text());cases={c['name']:c for c in summary['conditions']}
    figures=[];tables=[]
    def tab(title,head,rows):
        tables.append(title)
        return f'表{len(tables)} {title}\n\n| '+' | '.join(head)+' |\n|'+'|'.join(['---']*len(head))+'|\n'+''.join('| '+' | '.join(map(str,row))+' |\n' for row in rows)+'\n'
    def fig(title,filename):
        figures.append(filename);return f'![图{len(figures)} {title}](figures/{filename}.png)\n\n'
    def label(p):return p.replace('imu','IMU').replace('vio','VIO').replace('lio','LIO').replace('_',' ')
    def span(c):
        a=[r['position_rmse_m'] for r in c['trials']];return f'{min(a):.3f}～{max(a):.3f}'
    def old_table(number):
        block=re.search(r'^表'+str(number)+r' [^\n]+\n\n((?:\|[^\n]*\n)+)',old,re.M)[1]
        rows=[[v.strip() for v in line.strip().strip('|').split('|')] for line in block.splitlines()]
        return rows[0],rows[2:]
    def results(kind):
        rows=[]
        for grade in ('基础','增强','精细'):
            for mode,mode_name in [('start','起点后传播'),('peer','加车间测距'),('single','再加单锚点')]:
                c=cases[f'{kind}_{grade}_{mode}_s0.10'];a=np.array([r['position_rmse_m'] for r in c['trials']])
                rows.append([grade,mode_name,f'{a.mean():.3f}±{a.std(ddof=1):.3f}',f'{a.min():.3f}～{a.max():.3f}',f'{np.count_nonzero(a<1)}/20'])
        return tab(kind.upper()+'输入的20次蒙特卡洛结果',['档位','后续处理','RMSE均值±标准差（米）','RMSE范围（米）','RMSE小于1米次数'],rows)
    toc=['项目概况与系统方案','单锚点坐标与初始化','传感器分级仿真与结果','实测数据与接口应用','设备预算与实车试验','复现与阶段结论']
    s='# 多平台协同定位硬件仿真测试报告与联调方案\n\n浙江大学\n\n<!-- pagebreak -->\n\n## 目录\n\n章节目录\n\n| 章节 | 内容 |\n|---|---|\n'
    s+=''.join(f'| [{i}](#chapter_{i}) | [{title}](#chapter_{i}) |\n' for i,title in enumerate(toc,1))
    s+='\n<!-- pagebreak -->\n\n## 1 项目概况与系统方案\n\n'
    s+='''本项目面向卫星导航拒止条件下多平台协同任务，已形成三盒传感器采集、在线相对队形输出、联合录包和仿真复现程序。协同定位系统主盒（系统中称为201盒）接收三车信息，对外发送相对位置；202、203盒对应另外两车。系统由浙江大学开展定位算法及接口开发，定位结果可供通信、规划、控制与显示使用。

现有默认程序通过超宽带测距（Ultra-Wideband，UWB）恢复三车相对三角形。惯性测量单元（Inertial Measurement Unit，IMU）和相对变换估计（Relative Transformation Estimation，RTE）采用独立试验入口。本轮进一步比较一个已知位置UWB点与不同运动输入的组合，用相同轨迹和配对误差样本说明增加哪些条件、得到怎样的相对位置结果。

'''
    s+=fig('协同定位系统组成','01_系统与联调边界')
    s+='''本轮在开发电脑实际执行84种条件、1680次仿真，保存测量输入、初始化结果、20次完整输出和逐次误差。报告按每类传感器依次说明参数、依据、处理方式、统计结果和曲线，实测数据另按实际采集结果列示。

## 2 单锚点坐标与初始化

### 2.1 一个已知点提供的测量

锚点只有一个，位置已知并配备UWB，可以是地标上的设备，也可以是已落地并保持静止的无人机。三车互相测距形成三条车间链路，三车到同一点另有三条链路，共六条距离；没有增加其他固定点。

'''
    s+=fig('三车运动与唯一UWB点','single_16_三车与唯一锚点')
    s+=tab('测距与方位信息',['输入条件','可确定的信息','坐标表达'],[
        ['仅三车到同一点的距离','每辆车到点的半径','各车位于以该点为圆心的圆上'],
        ['再加入三条车间距离','三车与该点的整体几何形状及夹角','绕点整体旋转保持六条距离不变'],
        ['再加入各车局部运动','运动充分时估计各局部系之间的相对转角和位置，维持共同局部轴','整体地图方向仍由共同坐标的方向约定或外部方向参考确定'],
    ])
    s+='''本轮将唯一UWB点设为局部原点，X轴取起点时由该点指向201车的方向，Y轴在平面内逆时针90度，Z轴向上，运动后轴向保持固定。三车位置减去201车同一时刻的位置得到固定轴向相对位置。该约定定义了任务局部坐标；若接入地图方向，结合已测方向变换换算。

用a表示已知点，p表示车辆位置，对所有车辆及局部姿态同时施加p′=a+Rφ(p−a)的同一个旋转，车间距离、车锚距离和各车局部运动增量都保持不变。因此，单点测距不能判别这个共同旋转角。下图把同一组运动整体旋转45度，六条距离的最大差值仅为数值舍入量。

'''
    s+=fig('整体方位变化与相同测距','outdoor_整体旋转不变性')
    s+='''本轮验证的最大距离差为2.84×10⁻¹⁴米，局部运动增量差为2.98×10⁻¹⁴米。机体系相对方位和队形在地图中的整体朝向是不同的量：前者可在运动与测距约束充分时估计，后者需独立方向参考。六轴MEMS在本轮用于运动传播，不采用地球自转或磁场寻北。

相关的[单锚点定位研究](https://arxiv.org/html/2005.10648)在观测中加入了磁场辅助的航向；[Omni-swarm](https://github.com/HKUST-Aerial-Robotics/Omni-swarm)采用VIO、视觉检测、共同地图及UWB。组合观测提供的方向信息各有来源，本系统按六轴配置保留独立的地图方向变换。

### 2.2 初始化的输入与计算

初始化是把各车自己的局部运动坐标变为可共同使用的初始位置和相对坐标关系。盒端本地惯导先通过静止窗口估计重力方向和角速度均值；本轮仿真从校准后的残余误差建模，再用多时刻的六条距离和各车局部运动共同求起点关系。前30秒为静止段，30～60秒积累起步及转弯运动，60秒后进入统一评价。60秒是本轮固定的数据窗口，不是传感器启动时间或通用实车要求。

'''
    s+=tab('本轮初始化步骤',['步骤','实际使用的输入','计算输出'],[
        ['形成候选形状','静止段六条测距均值','由距离几何形成两种镜像候选'],
        ['累计运动约束','0～60秒局部运动与六条测距，按1秒抽样','五个起点位置自由度、三个局部坐标转角'],
        ['多初值数值求解','两种镜像，各尝试三个转角初值','按测距残差选择一个最小二乘解'],
        ['核对数值条件','求解是否收敛、规定局部轴后的雅可比矩阵','记录局部秩及奇异值，随后保存连续位置输出'],
    ])
    init=[d for c in summary['conditions'] if c['mode']=='start' for d in c['initialization']]
    s+=f'本轮27种输入与测距组合各初始化20次，共{len(init)}次独立初始化；{sum(d["optimizer_success"] for d in init)}次数值求解收敛，规定局部轴后{sum(d["jacobian_rank"]==8 for d in init)}次雅可比矩阵为8秩。这是该轨迹下的数值结果，整体地图转角仍按第2.1节处理。静止输入的独立检查只得到5秩，三个局部转角不能从静止距离中确定。\n\n'
    s+='''[Omni-swarm的UWB–里程计初始化](https://arxiv.org/html/2103.04131v5)同样累计多帧运动、检查可观条件，并使用多个初值降低局部极小的影响。其飞行平台使用的运动幅度和视觉辅助由该系统确定。本工程实现的是上述思路下的独立平面测距—运动求解程序，输入、候选、求解及后续位置均可复算；它与默认UWB节点及原RTE库分别运行。

## 3 传感器分级仿真与结果

### 3.1 共同场景与统计方法

三车在300秒内完成静止、起步、转弯和编队整体运动，范围约85×115米，最高速度2.39米/秒，最大车间距24.56米。唯一UWB点位于场景（55，75）米，最大车锚距离66.16米。采用平面视距工况，天线参考点同高；六条链路各10赫兹，共每秒60条距离。

IMU、视觉惯性里程计（Visual-Inertial Odometry，VIO）、激光惯性里程计（LiDAR-Inertial Odometry，LIO）各设三种输入质量，测距标准差取20、10、5厘米。每一组合分别比较起点后仅运动传播、继续车间测距、再继续单锚点测距三种处理，构成81种条件；另有三档默认纯UWB队形，共84种条件，每种20次。10厘米为主对照，5厘米为改善测距后的敏感性对照，20厘米为较大测距误差对照。

本轮属于固定运动轨迹上的噪声蒙特卡洛分析。同一条件内20次独立生成噪声，不同处理分支使用相同编号的输入及初始化结果。统计60～300秒的202、203车相对201车位置向量误差，先在每次运行内求位置均方根误差（Root Mean Square Error，RMSE），再列20次RMSE的均值、样本标准差及最小至最大范围。表中小于1米的次数用于直观比较本轮亚米级结果，不作为另定的合同指标。

真值用于生成传感器测量及事后误差计算；估计程序只接收带误差测量。位置输出没有经真值轨迹配准或事后平滑。所有20次完整输出保留，过程图固定展示重复试验1。

### 3.2 六轴MEMS输入

静止校准后的残余偏置、厂家零偏稳定性、白噪声和偏置随机游走分别建模。现有15份静止统计文件的验证段约30～270秒，可用于说明短时波动与扣偏后残余量，不能把它们等同于长时间温漂或器件稳定性标定。加计偏置需结合多姿态标定，单姿态统计用于量化随机波动。

'''
    gs=[v for r in static for v in r['gyro_std_rad_s']];ac=[v for r in static for v in r['accel_std_mps2']];post=[max(abs(v) for v in r['equivalent_deg_per_hour']) for r in static]
    s+=tab('本项目静止片段的统计量',['统计量','已保存片段的范围','含义'],[
        ['陀螺逐样本标准差',f'{min(gs):.2g}～{max(gs):.2g}弧度/秒','含实际采样和内部滤波影响的短时波动'],
        ['加计逐样本标准差',f'{min(ac):.3f}～{max(ac):.3f}米/秒²','短时随机波动'],
        ['扣偏后后段均值的最大轴',f'{min(post):.2f}～{max(post):.2f}度/小时','短时间均值折算，不代表能够测出地球自转'],
    ])
    s+='''地球自转角速度约15度/小时。扣除某个静止窗口均值后，短时后段均值可以小于这一数值，因为地球自转的近似常量也被包含在启动均值中；这不提供北向测量。表中的三档初始陀螺残余是校准后输入假设，同时加入后续偏置游走；初始残余与器件零偏稳定性分别定义。

'''
    rows=[]
    for grade in ('基础','增强','精细'):
        c=PROFILES['imu_'+grade];rows.append([grade,f'{c["accel_bias"]:g}',f'{c["gyro_bias"]*180/np.pi*3600:.2f}',f'{c["tilt_std_deg"]:g}',f'{9.80665*np.deg2rad(c["tilt_std_deg"]):.4f}'])
    s+=tab('MEMS初始残余误差设定',['档位','加计偏置标准差（米/秒²）','陀螺偏置标准差（度/小时）','倾角残余标准差（度）','水平重力泄漏标准差（米/秒²）'],rows)
    s+=tab('MEMS随时间变化的误差',['噪声项','基础','增强','精细','离散化'],[
        ['加计偏置游走（米/秒²/√秒）','0.001','0.0003','0.0001','每步乘√时间间隔'],
        ['陀螺偏置游走（弧度/秒/√秒）','0.0001','0.00003564','0.00001','每步乘√时间间隔'],
        ['100赫兹加计白噪声（米/秒²）','0.05','0.02','0.01','取10样本均值后进入10赫兹计算'],
        ['100赫兹陀螺白噪声（弧度/秒）','0.0005','0.0005','0.0005','同上'],
    ])
    s+='''偏置游走采用[Kalibr误差模型](https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model)的形式；增强档陀螺游走数值参考[LIO-SAM原始配置](https://github.com/TixiaoShan/LIO-SAM/blob/master/config/params.yaml)，其余档位为明确的敏感性设定。倾角残余按重力加速度乘小角度计入水平加速度，0.1度对应约0.0171米/秒²。IMU分支估计位置、速度和局部坐标转角，残余偏置作为输入扰动；结果反映这一传播与测距更新实现。

'''
    s+=results('imu')+fig('MEMS输入的逐项对比','single_17_IMU逐级对比')+fig('MEMS输入的误差过程','outdoor_imu_过程')
    s+='''### 3.3 相机与IMU运动输入

[VINS-Fusion车载示例](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/tree/master/config/vi_car)提供双目与IMU配置，并有车载运行资料。其acc_n、gyr_n、acc_w、gyr_w等是前端的IMU测量与偏置噪声参数；本轮以下三项描述的是前端已经输出的局部位移误差，二者不能直接替换。

本节用尺度误差、随时间积累的航向偏移和按行驶距离平方根增长的位置随机误差扰动局部增量，评估相同位置融合接口对输入质量的敏感性。三档数值为本轮位姿接口的显式设定，设备选型在实车室外轨迹上按同一口径统计输出质量后对应这些条件。

'''
    for kind,title,distribution in [('vio','VIO','single_18_VIO逐级对比'),('lio','LIO','single_19_LIO逐级对比')]:
        if kind=='lio':
            s+='''### 3.4 雷达与IMU运动输入

[FAST-LIO](https://github.com/hku-mars/FAST_LIO)和[LIO-SAM](https://github.com/TixiaoShan/LIO-SAM)均有室外实机资料；[NTU VIRAL](https://ntu-aris.github.io/ntu_viral_dataset/)中的停车场、广场序列可用于室外复测。数据集同时包含室内序列，后续选择时按场景逐一标记。

FAST-LIO中的acc_cov、gyr_cov和偏置协方差用于设置滤波权重；以下参数描述局部位移输出误差，模型层次分别列示。以下仍采用局部位移输出假设，用同一初始化和六条距离计算结果。它检验运动输入质量对本工程融合程序的影响，真实点云前端按实车场景和外参接入。

'''
        rows=[]
        for grade in ('基础','增强','精细'):
            c=PROFILES[kind+'_'+grade];rows.append([grade,f'{c["scale_std"]*100:g}',f'{c["yaw_drift_std"]:g}',f'{c["displacement_noise"]:g}'])
        s+=tab(title+'局部位移误差设定',['档位','尺度标准差（%）','航向漂移标准差（弧度/秒）','位置随机项（米/√米）'],rows)
        if kind=='lio':
            s+='VIO增强与LIO基础、VIO精细与LIO增强采用相同输出误差参数，并共享同编号扰动，因此相应统计结果相同；实际前端结合视场、算力和室外输出质量选配。\n\n'
        s+=results(kind)+fig(title+'输入的逐项对比',distribution)+fig(title+'输入的误差过程','outdoor_'+kind+'_过程')+fig(title+'估计轨迹与真值','outdoor_'+kind+'_轨迹')
        c=cases[kind+'_增强_single_s0.10'];s+=f'{title}增强输入配合单锚点的20次相对位置RMSE为{span(c)}米。表格、分布与过程图均读取同一批已保存输出。\n\n'
    s+='''### 3.5 测距精度与默认UWB对照

[LinkTrack官方资料](https://www.nooploop.com/linktrack/)列出的典型测距准确度为10厘米；Omni-swarm实机配置的距离协方差为0.02平方米，对应约14厘米标准差。厂商典型准确度、滤波协方差及随机噪声标准差含义不同，本轮选择20、10、5厘米作为显式对照，分别说明数值口径。

每条链路还加入每次运行固定的零均值偏差，标准差3厘米，用于表示校准后仍可能保留的链路偏差；该值是工程假设。它与逐时白噪声分开生成，同编号方案使用完全相同的偏差。原静止包中的距离波动标准差约1.7～7.9厘米可说明短时波动，绝对测距误差仍以独立距离真值确定。

'''
    s+=tab('不同测距输入下的20次结果',['组合','20厘米测距RMSE（米）','10厘米测距RMSE（米）','5厘米测距RMSE（米）'],[[label(p)]+[span(cases[f'{p}_single_s{x:.2f}']) for x in SIGMAS] for p in PROFILES])
    s+=tab('默认纯UWB队形结果',['测距标准差（厘米）','RMSE均值±标准差（米）','RMSE范围（米）'],[[int(x*100),f'{np.mean([r["position_rmse_m"] for r in cases[f"uwb_s{x:.2f}"]["trials"]]):.3f}±{np.std([r["position_rmse_m"] for r in cases[f"uwb_s{x:.2f}"]["trials"]],ddof=1):.3f}',span(cases[f'uwb_s{x:.2f}'])] for x in SIGMAS])
    s+='默认纯UWB调用原C++求解库，误差在原规范队形坐标中计算；其他组使用规定的局部固定轴。两种坐标分别与各自的真值比较。\n\n'
    s+=fig('测距与运动输入精度的影响','single_20_传感器与测距精度')+fig('相对位置误差累计分布','single_24_位置误差累计分布')
    s+='''### 3.6 原RTE库的既有对照

原RTE库在局部里程计与车间测距序列上估计相对坐标变换。以下保留既有300秒、无误差局部里程计、重复试验1的实际C++输出，分别使用2、5、10厘米测距噪声，与本轮室外误差模型分开标注。

'''
    s+=fig('原RTE库的逐车误差','16_RTE逐车定位误差')
    s+='''294.48秒时，原RTE窗口更新使203车位置变化约2.76米，位置误差由0.646米变为2.117米，而测距拟合残差约为1.7厘米。代码及重建记录表明，这是新窗口对相对变换约束较弱时直接更新变换产生的变化。测距残差与位置误差分别评价，本轮保留原输出，不用平滑隐藏跳变。

## 4 实测数据与接口应用

### 4.1 六组静止数据与在线输出

六组静止采集共记录3,229,145帧车载IMU及48,985帧UWB测距，已用于传感器分析、默认UWB位置回放及输入检查。数据按编号1～6对外列示。消息头时间戳（Header）指消息携带的测量时间；曲线按其相对起点表示。

'''
    h,rows=old_table(2);s+=tab('六组静止实测数据',h,rows)
    s+=fig('实测测距与在线位置','05_实测在线输入输出')+fig('实测三车相对队形','06_实测三车相对三角形')
    s+='''两批在线输出频率约9.24～9.42赫兹，原有队形坐标和话题继续作为默认接口。三盒点云与三目图像采集已经完成，后续按实车视场进行特征提取与安装标定。

'''
    s+=fig('主盒三目视觉采集画面','03_三路相机实拍')
    s+='''### 4.2 位置接口与坐标选择

默认UWB输出话题为/cooperative_localization/poses_2d，坐标系coop_ref_1_relative：201车为原点，X轴沿201至202连线，203取Y轴正侧。原RTE使用/cooperative_localization/rte/poses_2d；固定相对坐标由现有转换程序输出。本轮单锚点仿真另导出CSV，使用coop_single_anchor及coop_single_anchor_relative，位置有效位与航向有效位分别给出。

'''
    h,rows=old_table(12);s+=tab('定位输入与输出',h,rows)
    s+='''接收程序按车辆编号读取相对位置、坐标系、测量时间和有效性。单锚点CSV中的anchor_x_m、anchor_y_m为固定锚点局部位置，x_m、y_m为减去201车后的相对位置；本轮导出位置，航向有效位为false。该离线对照入口与默认UWB、原RTE库分别保留。

## 5 设备预算与实车试验

### 5.1 三车配置预算

按三车整套新建配置估算硬件、一次性算法开发、安装标定与联调，单位人民币万元。各运动组合包含一个UWB点，点的供电防护和安装测量预算约0.3～0.8万元计入硬件；复用落地无人机时不另计飞行平台。默认纯UWB一行为三车基础配置。算法预算按三车项目计一次，既有设备与功能可复用时按新增部分重新核算。

'''
    s+=tab('三车方案预算',['组合','硬件（万元）','算法（万元）','联调（万元）','总计（万元）'],[[label(b['profile']) if b['profile']!='uwb' else '默认纯UWB']+[f'{b[k][0]:g}～{b[k][1]:g}' for k in ('hardware_wan','algorithm_wan','integration_wan','total_wan')] for b in budget['combinations']])
    s+='''费用为项目预算测算，按约1500～2000元/人日综合研发成本组织算法、前端接入、消息适配、测试及复现工作，采购价格按最终型号和报价细化。设备档位不自动等同于输出精度；下图将预算与相应假设输入条件下的结果对应，实际选型结合实车前端输出确定。

'''
    s+=fig('预算与假设输入条件的结果','single_26_预算与仿真精度')
    s+='''### 5.2 实车采集与联调

先使用同一个已知位置UWB点，记录天线位置和安装高度，同时采集三条车间测距及三条车锚测距。车速控制在2～3米/秒以内，车间距不超过20～30米，轨迹包含起步、转弯及相对位置变化。点云与图像在车辆实际视场中开展标定和前端处理，分别输出可核对的本车运动。

IMU按静止窗口估计启动角速度均值和重力方向，再检查运动后的偏置变化；加计采用多姿态标定，必要时以长静止数据计算阿伦方差（Allan variance）区分白噪声与偏置游走。VIO/LIO按室外轨迹和独立位置真值量化增量误差、尺度及方向漂移，再代入本轮相同测距和评价程序。

用独立位置测量设备形成动态真值，例如载波相位差分定位（Real-Time Kinematic，RTK）或全站仪。定位侧输出统一局部坐标下的相对位置和测量时间，通信、规划与控制按同一车辆编号接入，地图方向则通过方向变换统一。

## 6 复现与阶段结论

### 6.1 仿真与图表重算

在工程根目录执行，数据保存于data/single_anchor_outdoor_20261029。若只查看已有结果，直接打开summary.json和全部仿真结果.csv；各条件20次完整位置在outputs中。参数引用、原始配置快照、静止统计和旋转不变性记录在参数与可观性核对目录。

```bash
PYTHONPATH=tools OPENBLAS_NUM_THREADS=1 python3 -u tools/evaluate_single_anchor.py --trials 20
PYTHONPATH=tools python3 tools/audit_outdoor_assumptions.py
PYTHONPATH=tools python3 tools/check_single_anchor.py
PYTHONPATH=tools python3 tools/plot_single_anchor.py
```

只修改本报告Markdown后，使用以下命令生成Word和PDF；默认不覆盖手工源稿：

```bash
PYTHONPATH=/tmp/cooperative_report_deps python3 tools/render_formal_report.py
python3 tools/export_formal_pdf.py
```

默认在线程序和原RTE保持现有独立入口，运行环境及盒端步骤见21号操作文档，完整调用和复现命令见本报告配套说明。新图由实际保存数据绘制，中文是字体文字，附带可选择文字的矢量PDF和SVG。

### 6.2 阶段结论

本阶段形成了三盒采集、默认UWB相对队形、独立RTE库和消息接口，本轮以一个已知位置UWB点完成84种条件、1680次误差模型修订后的仿真。方法、参数依据、蒙特卡洛统计、逐时误差和轨迹已按输入类别连贯列示。

'''
    s+=f'10厘米测距并含3厘米链路偏差标准差时，VIO增强输入配合单锚点的相对位置RMSE为{span(cases["vio_增强_single_s0.10"])}米，LIO增强输入为{span(cases["lio_增强_single_s0.10"])}米；默认纯UWB队形为{span(cases["uwb_s0.10"])}米。上述结果对应第3章列出的传感器输入模型和共同轨迹，完整20次结果均保留。\n\n'
    s+='''单个已知点提供位置参考，测距与运动可在规定局部轴下维持三车相对关系；地图整体转角通过独立方向变换确定。后续按同一UWB点和低速编队条件采集实车数据，以实测的传感器误差和独立动态真值复测，继续接入通信、规划和控制。
'''
    (DEST/'报告图表顺序.json').write_text(json.dumps({'figures':figures,'table_titles':tables},ensure_ascii=False,indent=2)+'\n')
    return s


def supporting():
    text='''# V1.4室外参数与结构修订：阅读与复现

正式源稿为本目录同名Markdown。手工修改后，只运行tools/render_formal_report.py和tools/export_formal_pdf.py；tools/build_v14_report.py默认保留Markdown，--refresh-content才重新组织正文。逐次前后对照和意见记录在工程docs目录。

最新数据位于data/single_anchor_outdoor_20261029：84种条件×20次=1680次。旧的data/single_anchor_20261029完整保留，不把旧模型结果覆盖为新结果。

```bash
PYTHONPATH=tools OPENBLAS_NUM_THREADS=1 python3 -u tools/evaluate_single_anchor.py --trials 20
PYTHONPATH=tools python3 tools/audit_outdoor_assumptions.py
PYTHONPATH=tools python3 tools/check_single_anchor.py
PYTHONPATH=tools python3 tools/plot_single_anchor.py
PYTHONPATH=/tmp/cooperative_report_deps python3 tools/render_formal_report.py
python3 tools/export_formal_pdf.py
```

参数与可观性核对目录保存Kalibr、VINS-Fusion车载配置、LIO-SAM、FAST-LIO及Omni-swarm原始配置快照和SHA；参数来源与可观性核对.json区分实际项目短静止统计、开源配置及敏感性假设。静止扣偏后的短时均值不是器件零偏稳定性。IMU新增偏置随机游走和重力泄漏；主档10厘米测距，5/20厘米为改善/保守对照，固定链路偏差标准差3厘米为假设。VIO/LIO仍是局部位移输出模型，没有伪称运行原始图像/点云前端；其输出误差参数不冒称照搬开源室外实测结果。

单个点与局部运动的整体共同旋转不变性已通过45度反例验证。单锚点局部坐标约定为点原点、起点点至201车为正X，不代表观测到地图方向。初始化使用60秒数据窗口，由六条测距与本车运动共同求初始位置/相对转角；六个初值只按测量残差选取，真值不入估计。真实前端的重力、偏置及尺度初始化需按各开源实现和实车安装处理，报告中的60秒不是商业部署要求。

全部仿真结果.csv与summary.json保存全部重复结果，outputs保存20次完整位置，inputs保存实际使用的测量，运行日志.txt和运行清单.json记录代码与输入输出，独立结果核对.json复算全部RMSE并重放三类输入。位置CSV为LIO增强、10厘米、重复试验1。IMU对照为平面位置/速度/局部转角估计，偏置作为扰动，不能把该组误差当成MEMS器件的性能上限。

图表与参数说明、统计在正文同一节，不另列“已完成工作”或图片汇编章节。每幅新图有360 dpi PNG、保留文字节点的SVG和嵌入字体的矢量PDF；单锚点图线来源.json记录源数据及折线哈希。原始RTE图仍读取旧C++库输出，明确标作既有对照。框架图为原生文字和连线，实拍来自实际图像，未使用生成式图片。

测试：PYTHONPATH=tools python3 -m unittest tools/test_single_anchor.py。参数与模型的详细审查说明见同目录参数来源_可观性_初始化核对.md。
'''
    (DEST/'阅读与复现说明.md').write_text(text)
    questions=['残余偏置、零偏稳定性、噪声密度、逐样本噪声和偏置游走分别是什么？','为什么静止扣偏后的后段均值小于地球自转，也不能说明能够寻北？','VINS/FAST-LIO配置参数和本轮位移输出误差参数有何区别？','一个已知点为何不能确定整个队形相对地图的转角，45度反例如何复算？','相对方位、机体朝向、地图方向分别由什么确定？','初始化使用哪些测量、未知数、初值与选择条件，60秒的含义是什么？','为何采用84种条件、每组20次，蒙特卡洛统计量怎样从输出算出？','每类输入的参数、均值/标准差、范围和曲线在报告什么位置？','单锚点论文的九轴磁场航向条件能否直接用于本项目六轴MEMS？','预算对应什么新增工作，哪些参数仍需实际设备动态标定？']
    (DEST/'V1.4_报告理解核对问题.md').write_text('# 报告理解核对问题\n\n'+'\n'.join(f'{i+1}. {q}' for i,q in enumerate(questions))+'\n')
    for name in ['build_single_anchor_report.py','build_v14_report.py','evaluate_single_anchor.py','plot_single_anchor.py','check_single_anchor.py','test_single_anchor.py','audit_outdoor_assumptions.py','build_stage_report.py','render_formal_report.py','export_formal_pdf.py','report_plot_fonts.py']:
        shutil.copy2(ROOT/'tools'/name,DEST/'复现脚本'/name)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--refresh-content',action='store_true');args=p.parse_args()
    if args.refresh_content or not CURRENT.exists():CURRENT.write_text(generate())
    supporting();render_word(CURRENT.read_text(),CURRENT.with_suffix('.docx'),title='多平台协同定位硬件仿真测试报告与联调方案',header='')
    print('已排版：',CURRENT)


if __name__=='__main__':main()
