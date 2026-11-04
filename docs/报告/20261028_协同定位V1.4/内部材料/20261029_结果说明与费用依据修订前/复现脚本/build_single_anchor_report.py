#!/usr/bin/env python3
"""V1.4传感器分级、单点布置与硬件概算；默认保留手工Markdown。"""
import argparse
import hashlib
import json
import re
import shutil
from pathlib import Path
import numpy as np
from build_stage_report import render_word
from evaluate_single_anchor import OUT as INPUTS, ROOT, PROFILES, SIGMAS
from evaluate_anchor_layout import OUT, PLACEMENTS

DEST=ROOT/'docs/报告/20261028_协同定位V1.4'
STEM='多平台协同定位硬件仿真测试报告与联调方案_V1.4_设备分级与锚点仿真版'
CURRENT=DEST/(STEM+'.md')
BEFORE=DEST/'内部材料/20261029_布点对比与叙述调整前'/CURRENT.name
FIGURE1_SHA='a4b8725191f5958ed51468c702bedf98b80e18912f0ed66d4f02577353e0a4cc'
GRADES=('基础','增强','精细')
PACKAGES={'uwb':(2.4,3.8),'imu_基础':(3.2,5.5),'imu_增强':(4.2,7.),'imu_精细':(7.,12.),
          'vio_基础':(5.5,9.5),'vio_增强':(7.5,13.),'vio_精细':(11.5,19.),
          'lio_基础':(7.5,12.),'lio_增强':(9.5,16.5),'lio_精细':(14.5,24.)}
ANCHOR_PACKAGE=(.5,1.2)


def label(p):return p.replace('imu','IMU').replace('vio','VIO').replace('lio','LIO').replace('_',' ')


def generate():
    original=DEST/'figures/图1_协同定位系统组成.png'
    assert hashlib.sha256(original.read_bytes()).hexdigest()==FIGURE1_SHA,'图1原图发生变化，停止生成'
    old=BEFORE.read_text();summary=json.loads((OUT/'summary.json').read_text());cases={c['name']:c for c in summary['conditions']}
    previous=json.loads((INPUTS/'summary.json').read_text());previous_cases={c['name']:c for c in previous['conditions']}
    figures=[];tables=[]
    def tab(title,head,rows):
        tables.append(title)
        return f'表{len(tables)} {title}\n\n| '+' | '.join(head)+' |\n|'+'|'.join(['---']*len(head))+'|\n'+''.join('| '+' | '.join(map(str,row))+' |\n' for row in rows)+'\n'
    def fig(title,filename):
        figures.append(filename);return f'![图{len(figures)} {title}](figures/{filename}.png)\n\n'
    def arr(c):return np.array([r['position_rmse_m'] for r in c['trials']])
    def span(c):a=arr(c);return f'{a.min():.3f}～{a.max():.3f}'
    def mean(c):a=arr(c);return f'{a.mean():.3f}±{a.std(ddof=1):.3f}'
    def old_table(number):
        block=re.search(r'^表'+str(number)+r' [^\n]+\n\n((?:\|[^\n]*\n)+)',old,re.M)[1]
        rows=[[v.strip() for v in line.strip().strip('|').split('|')] for line in block.splitlines()]
        return rows[0],rows[2:]
    def no_anchor_results(kind):
        return tab(kind.upper()+'输入的无锚点结果',['档位','仅运动传播RMSE（米）','加入车间测距RMSE（米）','加入测距后均值±标准差（米）'],[
            [g,span(cases[f'{kind}_{g}_no_anchor_start']),span(cases[f'{kind}_{g}_no_anchor_peer']),mean(cases[f'{kind}_{g}_no_anchor_peer'])] for g in GRADES])
    toc=['系统方案与试验安排','传感器分级与配置投入','单锚点辅助与布置对比','实测数据与接口应用','实车采集与联调','复现与阶段结论']
    s='# 多平台协同定位硬件仿真测试报告与联调方案\n\n浙江大学\n\n<!-- pagebreak -->\n\n## 目录\n\n章节目录\n\n| 章节 | 内容 |\n|---|---|\n'
    s+=''.join(f'| [{i}](#chapter_{i}) | [{title}](#chapter_{i}) |\n' for i,title in enumerate(toc,1))
    s+='\n<!-- pagebreak -->\n\n## 1 系统方案与试验安排\n\n'
    s+='''本项目面向卫星导航拒止条件下多平台协同任务，已形成三盒传感器采集、在线相对队形输出、联合录包和仿真复现程序。协同定位系统主盒（系统中称为201盒）接收三车信息，对外发送相对位置；202、203盒对应另外两车。浙江大学开展定位算法及接口开发，定位结果可供通信、规划、控制与显示使用。

默认程序通过超宽带测距（Ultra-Wideband，UWB）恢复三车相对三角形。惯性测量单元（Inertial Measurement Unit，IMU）运动传播和相对变换估计（Relative Transformation Estimation，RTE）采用独立入口。本轮围绕三车低速编队，比较运动输入质量、测距精度及增加一个已知位置UWB点后的结果，为后续实车配置和联调提供依据。

'''
    s+=fig('协同定位系统组成','图1_协同定位系统组成')
    s+='''### 1.1 运动条件与评价口径

仿真时长300秒，前30秒静止，30～60秒积累起步及转弯数据用于求初始坐标关系，60～300秒统计连续定位结果。三车整体运动范围约85×115米，最高速度2.39米/秒，最大车间距24.56米。位置、测距及运动输入按10赫兹处理，保持连续视距测距，不添加人为中断。平面模型采用同高天线参考点。

位置误差评价202、203车相对201车的位置向量：分别将估计相对向量减去同一坐标定义下的真值向量，取二维长度。每次运行的位置均方根误差（Root Mean Square Error，RMSE）为两车全部评价时刻的误差平方均值再开平方，单位为米；随后统计20次运行RMSE的均值、样本标准差及最小至最大范围。它直接量化相对位置偏差，测距拟合残差另行说明。

### 1.2 对比试验的组成

IMU、视觉惯性里程计（Visual-Inertial Odometry，VIO）、激光惯性里程计（LiDAR-Inertial Odometry，LIO）各设三档输入质量。VIO/LIO在本轮用局部位移输出误差模型生成数据，包含尺度、方向漂移和随机位移项。传感器原始图像、点云处理在实车安装后按实际视场接入，仿真数值对应本报告列出的输入条件。

本次新增9档输入的无锚点及单锚点布置对比，共63种条件、1260次运行。无锚点组在初始化及后续处理中均只使用车间距离；单锚点组增加到同一个已知点的距离。各组使用同一轨迹、同编号运动扰动和测距噪声，初始化按各自实际具有的测量求解。另保留此前84种条件、1680次测距精度对照；区域中部单点的9组、180次结果与本次重复核对一致。因此两批合计为2940次执行记录、2760个不重复的条件—重复试验组合。

20次重复用于观察固定轨迹下随机误差带来的离散程度，属于噪声蒙特卡洛分析。所有重复均进入统计，过程图固定采用重复试验1。仿真真值用于生成测量及事后评价，估计器接收带误差的运动和距离输入，输出不做真值配准或事后平滑。

## 2 传感器分级与配置投入

### 2.1 默认UWB方案

默认方案根据三条车间距离求解三角形，以201车为原点、201至202连线为正X轴，203位于Y轴正侧。该方案直接给出相对队形，适合车间距离、相对布局和编队显示。下表调用已有C++库，在各时刻规范队形坐标内与真值比较。

'''
    s+=tab('默认纯UWB队形仿真',['测距标准差（厘米）','RMSE均值±标准差（米）','RMSE范围（米）'],[
        [int(x*100),mean(previous_cases[f'uwb_s{x:.2f}']),span(previous_cases[f'uwb_s{x:.2f}'])] for x in SIGMAS])
    s+='''测距标准差为10厘米、每条链路固定偏差标准差为3厘米时，默认队形结果已达到本轮亚米级相对位置目标。以下运动融合组进一步保持起点建立的固定轴向，用于连续相对轨迹及本车运动接入；其误差在各自固定局部坐标内计算。两类结果分别对应相对形状和连续固定轴向位置。

### 2.2 UWB与六轴MEMS

六轴MEMS输入包含加速度、角速度及启动校准后的残余误差。三档参数用于比较残余偏置、短时噪声、倾角误差和随时间变化的偏置影响，未将设备划为导航级或寻北配置。本轮不使用地球自转和磁场方向。

'''
    rows=[]
    for grade in GRADES:
        c=PROFILES['imu_'+grade];rows.append([grade,f'{c["accel_bias"]:g}',f'{c["gyro_bias"]*180/np.pi*3600:.2f}',f'{c["tilt_std_deg"]:g}'])
    s+=tab('MEMS初始残余误差',['输入档位','加计偏置标准差（米/秒²）','陀螺偏置标准差（度/小时）','倾角标准差（度）'],rows)
    s+=tab('MEMS动态误差设定',['误差项','基础','增强','精细'],[
        ['加计偏置游走（米/秒²/√秒）','0.001','0.0003','0.0001'],
        ['陀螺偏置游走（弧度/秒/√秒）','0.0001','0.00003564','0.00001'],
        ['100赫兹加计白噪声（米/秒²）','0.05','0.02','0.01'],
        ['100赫兹陀螺白噪声（弧度/秒）','0.0005','0.0005','0.0005']])
    s+='''白噪声按100赫兹逐样本标准差设置，取10样本均值进入10赫兹计算；偏置游走每步按系数乘以时间间隔平方根增加。倾角残余按重力加速度乘小角度计入水平加速度，0.1度约对应0.0171米/秒²。前30秒按已知静止处理，运动后连续积分。融合器估计位置、速度和局部坐标转角，本轮将残余偏置作为输入扰动。

'''
    s+=no_anchor_results('imu')+fig('MEMS输入的无锚点对比','layout_imu_无锚点分级')
    s+='分布图的箱体表示20次RMSE的中间50%，箱内横线为中位数；须线按1.5倍四分位距确定，超出的重复结果单独画点。对数纵轴用于同时显示不同误差量级，所有结果均保留。\n\n'
    s+='''车间测距明显约束了纯惯性积分的增长；持续固定轴向定位还受到残余加速度、方向漂移与初始化误差共同影响。下一类配置以相机或雷达与IMU联合形成的本车运动输入替代直接惯性积分，继续使用相同测距与位置融合接口。

### 2.3 UWB、相机与IMU

VIO输入对逐步位移施加固定尺度扰动、随运动时间增加的方向漂移，以及按行驶距离平方根增加的位置随机项。三档数值是本轮输入质量条件，实车通过相机、IMU安装标定及室外轨迹测量对应到同一口径。

'''
    for kind,title in [('vio','VIO'),('lio','LIO')]:
        if kind=='lio':s+='''### 2.4 UWB、雷达与IMU

LIO输入采用同样的位移接口，以更细的尺度、方向与随机位移误差设定对比雷达与IMU组合。VIO增强与LIO基础、VIO精细与LIO增强的输入参数相同，并使用同编号扰动，因此相应结果一致；设备类别本身不代替前端输出质量。

'''
        s+=tab(title+'局部位移误差',['输入档位','尺度标准差（%）','方向漂移标准差（弧度/秒）','位置随机项（米/√米）'],[
            [g,f'{PROFILES[kind+"_"+g]["scale_std"]*100:g}',f'{PROFILES[kind+"_"+g]["yaw_drift_std"]:g}',f'{PROFILES[kind+"_"+g]["displacement_noise"]:g}'] for g in GRADES])
        s+=no_anchor_results(kind)+fig(title+'输入的无锚点对比','layout_'+kind+'_无锚点分级')
    s+='''在固定轴向目标下，改善本车运动输入能够降低相对位置误差；具体收益同时取决于起步运动和初始坐标求解。仿真中的输出档位用于提出设备与前端要求，硬件配置概算按设备构成列示。

### 2.5 三车硬件配置概算

下表按三车整套配置估算，包含UWB、相应运动传感器、计算与通信配套、电源防护及安装件，并计入配套选型和备件余量。金额以人民币万元计，用于配置方案比较；型号和采购价确定后按实际配置调整。各档不以器件价格直接保证相应仿真输出精度。

'''
    s+=tab('三车硬件配置概算',['配置类别','硬件组成','配置概算（万元）'],[
        ['默认UWB','三车UWB及计算、通信与供电配套','2.4～3.8']]+[
        [label(p),'UWB＋'+{'imu':'六轴IMU','vio':'相机＋IMU','lio':'雷达＋IMU'}[p[:3]],f'{v[0]:g}～{v[1]:g}'] for p,v in PACKAGES.items() if p!='uwb'])
    s+='''相机、雷达及配套计算配置提高了硬件投入，同时为本车运动估计提供更丰富的观测。除提高本车输入质量外，还可增加一个已知位置UWB点，以较小的增量配置为三车共同提供距离参考。单点配套概算为0.5～1.2万元，包含UWB、供电防护及安装件；若装在已落地无人机上，沿用其平台，不另计飞行平台价格。该方案与无锚点方案按同一批运动输入直接对比。

## 3 单锚点辅助与布置对比

### 3.1 测量与坐标定义

锚点只有一个，位置已知且配备UWB，可以是地标上的设备，也可以是已落地并保持静止的无人机。三条车间距离加三车到该点的三条距离，共六条链路，各为10赫兹，总计每秒60条距离。新增观测同时参与起点求解和运动中的更新。

单点方案以该点为局部原点，X轴取起点由该点指向201车的方向，Y轴在平面内逆时针90度，Z轴向上，建立后保持轴向固定。将三车局部位置减去201车同一时刻的位置，得到固定轴向相对位置。无锚点运动方案以201起点为原点，X轴取起点201至202方向，建立后同样固定；默认纯UWB继续采用第2.1节的瞬时队形轴。

单个点的距离确定半径，结合三条车间距离可恢复三车与点的形状及夹角。整体绕点共同旋转时，这些距离保持相同；本车局部运动也随其局部坐标关系共同旋转。因此地图方向通过独立方向参考与局部坐标变换接入。局部轴的明确定义使三车能够在同一约定下连续输出和使用相对位置。

'''
    s+=fig('唯一点与整体方位关系','outdoor_整体旋转不变性')
    s+='''### 3.2 初始关系求解

初始化把各车自己的局部运动变成同一个局部坐标内的初始位置与转角。先由静止段平均测距形成几何候选，再累计0～60秒运动与测距、按1秒抽样求解。60秒是本轮统一数据窗口，起步与转弯提供了多时刻约束。数值优化仅以测距残差选解，静止状态用于恢复形状，运动信息用于确定局部坐标之间的关系。

'''
    s+=tab('初始化输入与求解',['方案','使用的测量','坐标约定后的未知量','候选与选择'],[
        ['无锚点','三条车间距离与三车局部运动','起点位置3自由度、局部转角3自由度','两种镜像、四种共同转角、三种相对转角初值，按测距残差选解'],
        ['单锚点','六条距离与三车局部运动','起点位置5自由度、局部转角3自由度','两种镜像、三种转角初值，按测距残差选解']])
    s+='''本轮检查求解是否收敛及规定坐标后雅可比矩阵的局部秩，同时保存所有初始解和连续输出。静止输入下，局部运动坐标转角尚无运动约束；测试轨迹包含相对运动后再求解。初始化结果直接进入后续估计，误差统计保留起点偏差及后续累积影响。

### 3.3 增加一个点后的精度变化

先把同一个点放在场景中部（55，75）米。测距白噪声标准差均为10厘米，链路固定偏差标准差为3厘米，同编号运动输入和车间测距逐点相同。无锚点组只使用三条距离建系及更新，单点组增加三条车锚距离；对比包含新增测量对初始化和连续更新的共同作用。

'''
    s+=tab('同输入条件下增加单点的结果',['运动输入','无锚点RMSE范围（米）','中部单点RMSE范围（米）','中部单点均值±标准差（米）'],[
        [label(p),span(cases[p+'_no_anchor_peer']),span(cases[p+'_anchor_区域中部']),mean(cases[p+'_anchor_区域中部'])] for p in PROFILES])
    options=[('vio_增强',False),('vio_精细',False),('vio_增强',True),('lio_增强',True)]
    s+=tab('输入条件、精度与配置投入',['配置方案','相对位置RMSE（米）','硬件概算（万元）'],[
        [label(p)+('＋中部单点' if use_point else '，无锚点'),span(cases[p+('_anchor_区域中部' if use_point else '_no_anchor_peer')]),
         f'{PACKAGES[p][0]+(ANCHOR_PACKAGE[0] if use_point else 0):g}～{PACKAGES[p][1]+(ANCHOR_PACKAGE[1] if use_point else 0):g}'] for p,use_point in options])
    s+='VIO增强加单点与VIO精细无锚点在本轮得到接近的误差范围，前者可采用较低一档本车运动输入，并增加共同距离参考。配置表给出两种实现路线的投入比较，实车以安装后的运动输入和测距结果确定选型。\n\n'
    s+=fig('VIO输入的定位误差过程','layout_vio_逐时定位误差')+fig('LIO输入的定位误差过程','layout_lio_逐时定位误差')
    s+='''过程图分别给出基础和增强输入下的定位误差，纵轴为202、203两车位置误差平方均值的平方根，保留每次距离更新后的波动。相对运动改变测量方向，本车运动漂移与测距更新在各时刻共同作用；图中的修正直接来自估计器计算，没有人工加入跳变。

单锚点提供了相对于固定点的距离约束，较低误差的运动输入可进一步改善相对位置。默认纯UWB以较少设备满足本轮相对形状目标；连续固定轴向位置可按本节结果选择运动输入及是否增加单点，不要求为相对形状目标配置高价惯导。

### 3.4 同一个点放在哪里

选取区域中部、起点侧方、区域左侧、区域右侧及区域角部五个候选位置，每次仅选其中一个。三车轨迹、噪声样本及输入参数保持一致，每个位置按六条测距重新求初始关系并连续更新，9档输入各重复20次，共900次布点运行。

'''
    s+=fig('三车轨迹与候选布点','layout_候选布点')
    s+=tab('候选位置与测距几何',['候选位置','场景坐标（米）','车锚距离范围（米）','5秒方向变化中位数（度）'],[
        [p,f'（{v[0]:g}，{v[1]:g}）',f'{summary["placement_geometry"][p]["minimum_range_m"]:.2f}～{summary["placement_geometry"][p]["maximum_range_m"]:.2f}',f'{summary["placement_geometry"][p]["median_five_second_bearing_span_deg"]:.2f}'] for p,v in PLACEMENTS.items()])
    s+='''表中方向变化指车辆相对点的视线角在连续5秒内的跨度，对评价时段及三车取中位数，用于描述测距方向随运动的变化。布点比较同时包含距离、方向变化、起点几何与整段运动，不以某一个几何量单独判定最优位置。超过100米的车锚距离是候选位置比较中的几何条件，车间距始终保持在24.56米以内；这些候选均沿用相同测距误差模型，未模拟随距离变化的无线传播或天线遮挡。

'''
    s+=tab('不同布点下增强输入的结果',['候选位置','IMU增强RMSE（米）','VIO增强RMSE（米）','LIO增强RMSE（米）'],[
        [p]+[span(cases[f'{kind}_增强_anchor_{p}']) for kind in ('imu','vio','lio')] for p in PLACEMENTS])
    s+=fig('不同布点的20次误差分布','layout_布点误差分布')+fig('单点辅助估计与真值轨迹','layout_单点估计轨迹')
    s+='''在本轮轨迹及等测距误差条件下，区域右侧的VIO/LIO增强结果优于区域中部；IMU输入下各位置的结果仍受惯性传播误差显著影响。起点侧方的轨迹最近处仅约0.24米，实车布设需按安全距离和天线安装位置调整。选点时结合行驶路线检查车锚距离、遮挡和起步几何，再按同一位置误差口径复测，仿真位置排序不替代现场测距结果。

### 3.5 测距精度变化与既有RTE对照

此前单点对照进一步改变逐时测距标准差，固定链路偏差仍为3厘米。下表均为区域中部单点，保留全部20次结果。5厘米表示改善后的输入条件，10厘米为主对照，20厘米为较大误差条件。

'''
    s+=tab('单点方案的测距精度对照',['运动输入','20厘米测距RMSE（米）','10厘米测距RMSE（米）','5厘米测距RMSE（米）'],[
        [label(p)]+[span(previous_cases[f'{p}_single_s{x:.2f}']) for x in SIGMAS] for p in PROFILES])
    s+=fig('测距与运动输入质量的影响','single_20_传感器与测距精度')
    s+='''原RTE库保留既有300秒无误差局部里程计对照，以下为C++库实际输出，按2、5、10厘米测距噪声分别绘制，与本轮带误差运动模型区分。

'''
    s+=fig('既有RTE逐车定位误差','16_RTE逐车定位误差')
    s+='''294.48秒时，原RTE窗口更新使203车位置变化约2.76米，位置误差由0.646米变为2.117米，而测距拟合残差约1.7厘米。代码重建表明，新窗口对相对变换约束较弱时直接更新变换，造成位置跳变。这里的厘米级残差描述距离拟合，位置误差仍由独立真值评价；本轮保留原输出和默认程序。

## 4 实测数据与接口应用

### 4.1 静止采集与位置回放

六组静止采集共记录3,229,145帧车载IMU及48,985帧UWB测距，已用于传感器分析、默认UWB位置回放及输入检查。下表用数据编号1～6列示采集规模。消息头时间戳（Header）为消息携带的测量时间，实测曲线按其相对起点表示。

'''
    h,rows=old_table(13);s+=tab('六组静止实测数据',h,rows)
    s+=fig('实测测距与在线位置','05_实测在线输入输出')+fig('实测三车相对队形','06_实测三车相对三角形')
    s+='''两批在线输出频率约9.24～9.42赫兹。三盒均已完成三目视觉采集，雷达点云采集已完成；点云与图像后续需按特征提取、标定和甲方实车调试流程接入。已有静止数据用于输入质量分析和回放，动态位置误差按独立运动真值评价。

'''
    s+=fig('主盒三目视觉采集画面','03_三路相机实拍')
    s+='''### 4.2 对外位置接口

默认UWB输出话题为/cooperative_localization/poses_2d，坐标系为coop_ref_1_relative，继续使用瞬时三角形定义。原RTE使用/cooperative_localization/rte/poses_2d，固定轴向相对坐标由已有转换程序输出。下表保留定位输入及输出，接收方按车辆编号、坐标系、测量时间与有效性使用位置。

'''
    h,rows=old_table(14);s+=tab('定位输入与输出',h,rows)
    s+='''单点仿真另导出位置CSV，任务局部系为coop_single_anchor，相对系为coop_single_anchor_relative。anchor_x_m、anchor_y_m表示以点为原点的固定轴向位置，x_m、y_m表示减去201车同刻位置后的相对向量。该离线文件给出位置结果，航向有效位为false；默认在线UWB话题保持现状。

## 5 实车采集与联调

实车使用同一个已知位置UWB点，记录点的天线坐标及安装高度，同时采集三条车间测距和三条车锚测距。车速保持2～3米/秒以内，最大车间距20～30米。沿起步、转弯及编队整体移动路线采集，分别记录无锚点和有单点的输入，复用本轮位置评价程序比较配置收益。

IMU通过静止窗口估计启动角速度均值和重力方向，加计开展多姿态标定；在安装后的温度与振动条件下检查偏置变化。相机、雷达与IMU完成时间和安装位姿标定后，在实际室外视场中输出本车运动，并按尺度、方向漂移和随机位移统计对应本报告的输入条件。

运动试验配套独立位置测量，例如载波相位差分定位（Real-Time Kinematic，RTK）或全站仪。定位侧按统一车辆编号输出相对位置和测量时间，通信、规划与控制沿用现有消息接口接入；需要地图坐标时，将实测方向关系与任务局部系结合换算。采集命令、启动顺序和时间检查继续使用21号操作文档。

## 6 复现与阶段结论

### 6.1 仿真与图表重算

前一批测距精度数据位于data/single_anchor_outdoor_20261029，本次无锚点和布点结果位于data/single_anchor_layout_20261029。各目录summary.json给出逐条件统计，蒙特卡洛分组统计.csv便于查表，outputs保存全部重复的逐时位置。新数据独立结果核对.json记录1260次RMSE复算及输入重放结果。

```bash
PYTHONPATH=tools OPENBLAS_NUM_THREADS=1 python3 tools/evaluate_single_anchor.py --trials 20
PYTHONPATH=tools OPENBLAS_NUM_THREADS=1 python3 tools/evaluate_anchor_layout.py
PYTHONPATH=tools python3 tools/check_anchor_layout.py
PYTHONPATH=tools python3 tools/plot_single_anchor.py
PYTHONPATH=tools python3 tools/plot_anchor_layout.py
```

手工修改本报告Markdown后，用以下命令更新其他格式；它们保留手工源稿：

```bash
PYTHONPATH=/tmp/cooperative_report_deps python3 tools/render_formal_report.py
python3 tools/export_formal_pdf.py
```

图1直接使用提供的原图；新结果图由保存的位置与真值通过绘图脚本生成，提供PNG、SVG及矢量PDF。SVG保留文字和图形对象，可用Inkscape修改，操作及原图保留说明见配套文件。

### 6.2 阶段结论

三盒采集、默认UWB相对队形、独立RTE库和消息接口已形成可运行链路。六组静止数据支持输入分析及位置回放，本轮仿真补充了三类运动输入、三档质量、三档测距误差与五个单点候选位置的量化对比。参数、统计、误差过程及轨迹按同一条件相邻列示。

'''
    s+=f'在10厘米测距白噪声和3厘米链路固定偏差标准差下，默认纯UWB队形的20次RMSE为{span(previous_cases["uwb_s0.10"])}米。固定轴向定位采用LIO增强输入、无锚点时为{span(cases["lio_增强_no_anchor_peer"])}米；增加中部单点后为{span(cases["lio_增强_anchor_区域中部"])}米，改为区域右侧单点后为{span(cases["lio_增强_anchor_区域右侧"])}米。上述结果分别按规定坐标和明确的仿真输入计算。\n\n'
    s+='''配置上可保留默认UWB满足相对队形使用，再根据连续运动与共同位置参考的需要选用相机/雷达与IMU组合，以及一个已知位置UWB点。三车硬件概算及单点增量配置已列示。后续按低速编队和同一评价口径采集实车动态数据，完成位置输出与通信、规划和控制的联调。
'''
    (DEST/'报告图表顺序.json').write_text(json.dumps({'figures':figures,'table_titles':tables},ensure_ascii=False,indent=2)+'\n')
    budget=dict(unit='人民币万元',scope='三车硬件配置，含必要配套、防护安装件及余量；不列人力开发成本',basis='配置级概算，非厂商报价；输入档位不等同于器件采购等级',packages=PACKAGES,single_point_increment_wan=ANCHOR_PACKAGE)
    (DEST/'硬件配置概算.json').write_text(json.dumps(budget,ensure_ascii=False,indent=2)+'\n')
    return s


def supporting():
    text='''# V1.4阅读与复现

正式源稿为本目录“多平台协同定位硬件仿真测试报告与联调方案_V1.4_设备分级与锚点仿真版.md”。手工修改后运行render_formal_report.py与export_formal_pdf.py，保留手工Markdown。build_v14_report.py默认也保留源稿，只有--refresh-content才按生成器重新组织正文。

## 数据与运行

- data/single_anchor_outdoor_20261029：84种条件×20次，原有测距精度对照。
- data/single_anchor_layout_20261029：63种条件×20次，新无锚点与5种单点布置对照；每次最多一个点。中部单点180次与前一批一致，不重复算作新增独立样本。
- 新批次首轮三转角初值结果保留在同目录子文件夹，当前正式统计采用扩大初值覆盖后的结果；输入测量保持相同。旧结果不作为新增独立噪声样本。
- summary.json、蒙特卡洛分组统计.csv及outputs分别用于查统计、查表和复核逐时位置；独立结果核对.json记录全部1260次复算和无点/单点输入重放。

```bash
PYTHONPATH=tools OPENBLAS_NUM_THREADS=1 python3 tools/evaluate_single_anchor.py --trials 20
PYTHONPATH=tools OPENBLAS_NUM_THREADS=1 python3 tools/evaluate_anchor_layout.py
PYTHONPATH=tools python3 tools/check_anchor_layout.py
PYTHONPATH=tools python3 -m unittest tools/test_single_anchor.py tools/test_anchor_layout.py
PYTHONPATH=tools python3 tools/plot_single_anchor.py
PYTHONPATH=tools python3 tools/plot_anchor_layout.py
PYTHONPATH=/tmp/cooperative_report_deps python3 tools/render_formal_report.py
python3 tools/export_formal_pdf.py
```

## 图1与可编辑图

图1使用figures/图1_协同定位系统组成.png原文件，原样备份在figures/原始素材；SHA256记录在图1原图保留清单.json。生成器校验原图哈希，Word内嵌同一PNG字节，不重新绘制。用户原图外的透明区域按文档白底显示。

新数据曲线提供同名PNG、SVG、PDF；SVG保留文字节点，可用Inkscape打开，按T修改文字，选择工具移动和缩放对象。详细说明见SVG修改说明.md。旧图1的六框SVG/PPTX不对应本次用户提供的五框PNG，不应当作该原图的可编辑版本。

## 参数、费用与内部资料

参数来源核对与参考实现保留在参数来源_可观性_初始化核对.md、锚点布置参考与比较说明.md及data/single_anchor_outdoor_20261029/参数与可观性核对，不在客户正式正文列出外部项目。VIO/LIO采用显式位移输出模型；没有把模型结果写成已运行原始图像/点云前端的精度。

硬件配置概算.json仅保留三车配置概算和单点增量，包含配套、防护安装件和余量，不再列人日、算法研发或联调人工费用。它是配置级估计，不冒充已有型号的厂商报价。

每次修改前后对照、意见与版本记录分别保存在docs/报告改动前后对照记录.md、docs/报告编制与对外表述要求_持续记录.md和docs/报告内部版本记录.md。默认UWB、原RTE库和V1.3均按既有哈希核对保留。
'''
    (DEST/'阅读与复现说明.md').write_text(text)
    questions=['三类输入的误差参数分别是什么，哪部分来自假设、哪部分对应实际采集？','默认UWB的瞬时队形轴和固定轴向运动定位有何用途区别？','无锚点初始化是否使用了任何车锚距离？','相同输入下加一个点，误差分别从多少变成多少？','为什么改善初始化初值覆盖，不能说成传感器精度提高？','五个候选点是否同时存在？每种位置的车锚距离是否都满足同样传播条件？','单点为什么不能直接给出地图朝向，采用的局部轴怎么定义？','为什么右侧在这条轨迹下优于中部，现场还需要检查什么？','1260与1680次有哪些重叠，20次重复具体改变了什么？','硬件概算包括哪些实物配置，是否对应已确定型号或报价？','VIO/LIO模型试验如何在安装后的实车输入上复测？','SVG能修改哪些对象，PNG是否能直接变成逐对象可编辑原稿？']
    (DEST/'V1.4_报告理解核对问题.md').write_text('# 报告理解核对问题\n\n'+'\n'.join(f'{i+1}. {q}' for i,q in enumerate(questions))+'\n')
    for name in ['build_single_anchor_report.py','build_v14_report.py','evaluate_single_anchor.py','evaluate_anchor_layout.py','plot_anchor_layout.py','check_anchor_layout.py','test_anchor_layout.py','plot_single_anchor.py','plot_sensor_anchor_v14.py','check_single_anchor.py','test_single_anchor.py','audit_outdoor_assumptions.py','build_stage_report.py','render_formal_report.py','export_formal_pdf.py','report_plot_fonts.py']:
        shutil.copy2(ROOT/'tools'/name,DEST/'复现脚本'/name)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--refresh-content',action='store_true');args=p.parse_args()
    if args.refresh_content or not CURRENT.exists():CURRENT.write_text(generate())
    supporting();render_word(CURRENT.read_text(),CURRENT.with_suffix('.docx'),title='多平台协同定位硬件仿真测试报告与联调方案',header='')
    print('已排版：',CURRENT)


if __name__=='__main__':main()
