#!/usr/bin/env python3
"""V1.4传感器分级、单锚点布置与硬件概算；默认保留手工Markdown。"""
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
IMU_REFERENCE_PRICE_YUAN=47224.54
IMU_BUDGET_PER_UNIT_WAN=5.


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
    toc=['系统方案与试验安排','传感器分级与精度分析','单锚点辅助与布置对比','实测数据与接口应用','实车采集与联调','复现与阶段结论']
    s='# 多平台协同定位硬件仿真测试报告与联调方案\n\n浙江大学\n\n<!-- pagebreak -->\n\n## 目录\n\n章节目录\n\n| 章节 | 内容 |\n|---|---|\n'
    s+=''.join(f'| [{i}](#chapter_{i}) | [{title}](#chapter_{i}) |\n' for i,title in enumerate(toc,1))
    s+='\n<!-- pagebreak -->\n\n## 1 系统方案与试验安排\n\n'
    s+='''本项目面向卫星导航拒止条件下多平台协同任务，已形成三盒传感器采集、在线相对队形输出、联合录包和仿真复现程序。协同定位系统主盒（系统中称为201盒）接收三车信息，对外发送相对位置；202、203盒对应另外两车。浙江大学开展定位算法及接口开发，定位结果可供通信、规划、控制与显示使用。

默认程序通过超宽带测距（Ultra-Wideband，UWB）恢复三车相对三角形。惯性测量单元（Inertial Measurement Unit，IMU）运动传播和相对变换估计（Relative Transformation Estimation，RTE）采用独立入口。本轮围绕三车低速编队，比较运动输入质量、测距精度及增加一个已知位置UWB锚点后的结果，为后续实车配置和联调提供依据。

'''
    s+=fig('协同定位系统组成','图1_协同定位系统组成')
    s+='''### 1.1 运动条件与评价口径

运动融合分组的仿真时长300秒，前30秒静止，30～60秒积累起步及转弯数据用于求初始坐标关系，60～300秒统计连续定位结果。图12的RTE库测试按其自身的首次有效时刻开始输出，第3.5节说明其建系配置。三车整体运动范围约85×115米，最高速度2.39米/秒，最大车间距24.56米。位置、测距及运动输入按10赫兹处理，保持连续视距测距。平面模型采用同高天线参考点。

UWB依据无线信号的传播时间计算距离，天线及收发电路本身也会引入延迟。延迟补偿后的残余可能使同一对设备的平均测距持续偏大或偏小，在短时、工作条件稳定时可用近似固定的测距偏差表示。这类偏移会随重复测量保留，进而影响三车距离关系和相对位置；逐次测量的随机波动则另行建模。因此，传感器分级、锚点布置及默认UWB对照保留了固定偏差项，用于考察测距标定残余对定位的影响。其标准差设为3厘米，作为本轮仿真条件。图12的RTE库专门对照采用的测距模型在第3.5节单独说明。

仿真程序保存每辆车的真实位置作为对照，称为真值。位置误差评价202、203车相对201车的位置向量：将估计相对向量减去同一坐标定义下的真值向量，取二维长度。每次运行的位置均方根误差（Root Mean Square Error，RMSE）为两车全部评价时刻的误差平方均值再开平方，单位为米；随后统计20次运行RMSE的均值、样本标准差及最小至最大范围。它直接量化相对位置偏差，测距拟合残差另行说明。

### 1.2 对比试验的组成

IMU、视觉惯性里程计（Visual-Inertial Odometry，VIO）、激光惯性里程计（LiDAR-Inertial Odometry，LIO）各设三档输入质量。VIO/LIO在本轮用局部位移输出误差模型生成数据，包含尺度、方向漂移和随机位移项。传感器原始图像、点云处理在实车安装后按实际视场接入，仿真数值对应本报告列出的输入条件。

试验比较三件事：提高本车运动输入质量能降低多少误差，增加一个已知位置UWB锚点有多大作用，以及锚点的位置和测距精度怎样影响结果。三类输入各设三档，共9种运动输入。无锚点组使用车间距离和本车运动，单锚点组再加入三车到同一个锚点的距离；各组按实际可用测量求解初始关系，同编号对比使用相同运动与测距噪声样本。

20次重复用于观察固定轨迹下随机误差带来的离散程度，属于噪声蒙特卡洛分析。所有重复均进入统计，过程图固定采用重复试验1。仿真真值用于生成测量及事后评价，估计器接收带误差的运动和距离输入，输出不做真值配准或事后平滑。
'''
    s+=tab('仿真项目与分组',['试验目的','条件设置','条件数','重复次数'],[
        ['比较运动输入质量','9档输入×仅传播/加入车间测距','18','每条件20次'],
        ['比较单锚点位置','9档输入×5个候选位置，每次只有一个锚点','45','每条件20次'],
        ['比较测距精度','中部单锚点下，9档输入×5/20厘米测距；10厘米归入上行','18','每条件20次'],
        ['检验默认UWB队形','5/10/20厘米三档测距','3','每条件20次']])
    s+='''以上共84种条件、1680次重复运行。另用三组无误差局部里程计输入检查RTE求解过程，单独说明首次有效时刻及位置跳变。输入质量、测距精度和布点的作用分别用表格及实际误差曲线对照。

### 1.3 本报告使用的坐标系

相对位置的原点取201当前位置，201移动时原点随之平移；坐标轴是否旋转另按下表定义。报告采用四种轴向约定，分别对应不同的输出或仿真分组。

'''
    s+=tab('各类输出的原点与轴向',['输出或试验','原点','X轴与变化方式','对应结果'],[
        ['默认纯UWB','201当前位置','当刻201→202连线，随队形改变','默认UWB位置及队形结果'],
        ['RTE库原始输出','201当前位置','平行201本车初始局部里程计X轴，运动后轴向固定','图12'],
        ['无锚点运动融合仿真','相对位置原点为201当前位置','仿真起点201→202方向，之后固定','无锚点IMU/VIO/LIO分组'],
        ['单锚点运动融合仿真','任务位置以锚点为原点；相对位置以201当前位置为原点','锚点→起点201方向，之后固定','单锚点IMU/VIO/LIO及布置对比']])
    s+='''各组均以Z轴向上、Y轴在平面内相对X轴逆时针90度定义方向。任务位置保留建立时的固定原点，用于表示整体运动；相对位置减去201同一时刻的位置，用于表示车间关系，二者使用同一轴向。

**图12的RTE坐标：原点在201当前位置，轴向保持201本车初始局部里程计方向。** 这里固定的是轴向，201当前车头转动时坐标轴不随车头转动。它与每时刻沿201→202重新定轴的默认UWB坐标分开使用。现有坐标适配工具还保留首次有效历元201→202方向固定的可选表达，使用方法见第4.2节；图12使用上表RTE原始输出。

## 2 传感器分级与精度分析

### 2.1 默认UWB方案

默认方案根据三条车间距离求解三角形，以201车为原点、201至202连线为正X轴，203位于Y轴正侧。该方案直接给出相对队形，适合车间距离、相对布局和编队显示。下表调用项目C++库，在各时刻规范队形坐标内与真值比较。

'''
    s+=tab('默认纯UWB队形仿真',['测距标准差（厘米）','RMSE均值±标准差（米）','RMSE范围（米）'],[
        [int(x*100),mean(previous_cases[f'uwb_s{x:.2f}']),span(previous_cases[f'uwb_s{x:.2f}'])] for x in SIGMAS])
    s+='''测距随机误差标准差为10厘米、标定残余固定偏差标准差为3厘米时，默认队形结果已达到本轮亚米级相对位置目标。以下运动融合组进一步保持起点建立的固定轴向，用于连续相对轨迹及本车运动接入；其误差在各自固定局部坐标内计算。两类结果分别对应相对形状和连续固定轴向位置。

### 2.2 UWB与六轴MEMS

六轴MEMS输入包含加速度、角速度及启动校准后的残余误差。三档参数用于比较残余偏置、短时噪声、倾角误差和随时间变化的偏置影响，按这些输入进行运动传播和车间测距更新。

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
        s+=f'{title}输入由基础改善到增强时，加入车间测距后的相对位置RMSE由{span(cases[kind+"_基础_no_anchor_peer"])}米变为{span(cases[kind+"_增强_no_anchor_peer"])}米；精细输入为{span(cases[kind+"_精细_no_anchor_peer"])}米。这组对比用于明确本车运动输出质量对连续相对定位的影响，为实车前端标定和选配提供参考。\n\n'
    s+='''### 2.5 传感器配置与方案选择

本章对比说明，改善本车运动输入能够降低连续相对位置误差，起步运动和初始坐标求解也会影响结果。相机或雷达与IMU联合形成的位移输入，可沿用相同的车间测距和位置融合接口；实车前端按本章的尺度、方向漂移和位移误差口径核对输出质量。

配置选择上，优先复用现有三只完整传感器盒，每车一只，单盒3万余元。逐车增配高配惯性器件的可选方案，三只器件采购预算约15万元；若在现有配置上增加一只同型盒作为UWB锚点，则增加约3万余元，并为三车共同提供距离参考。综合设备复用和投入，选用现有传感器融合与单锚点相结合的方案，通过共同测距约束改善连续相对定位。

## 3 单锚点辅助与布置对比

### 3.1 测量与坐标定义

锚点只有一个，位置已知且配备UWB，可以是地标上的设备，也可以是已落地并保持静止的无人机。三条车间距离加三车到该锚点的三条距离，共六条链路，各为10赫兹，总计每秒60条距离。新增观测同时参与起点求解和运动中的更新。

本节单锚点融合仿真采用锚点局部坐标：以锚点为固定原点，X轴取锚点指向仿真起点201的方向，建立后保持固定。三车任务位置减去201同刻位置，得到轴向不变、原点随201平移的相对位置。无锚点运动融合仿真的轴向固定在起点201→202方向。各组误差均与第1.3节相应坐标下的真值比较；图12的RTE原始坐标另采用201本车局部里程计轴。

单个锚点的距离确定半径，结合三条车间距离可恢复三车与锚点的形状及夹角。整体绕锚点共同旋转时，这些距离保持相同；本车局部运动也随其局部坐标关系共同旋转。因此地图方向通过独立方向参考与局部坐标变换接入。局部轴的明确定义使三车能够在同一约定下连续输出和使用相对位置。

'''
    s+=fig('唯一锚点与整体方位关系','outdoor_整体旋转不变性')
    s+='''### 3.2 初始关系求解

初始化把各车自己的局部运动变成同一个局部坐标内的初始位置与转角。先由静止段平均测距形成几何候选，再累计0～60秒运动与测距、按1秒抽样求解。60秒是本轮统一数据窗口，起步与转弯提供了多时刻约束。数值优化仅以测距残差选解，静止状态用于恢复形状，运动信息用于确定局部坐标之间的关系。

'''
    s+=tab('初始化输入与求解',['方案','使用的测量','坐标约定后的未知量','候选与选择'],[
        ['无锚点','三条车间距离与三车局部运动','起点位置3自由度、局部转角3自由度','两种镜像、四种共同转角、三种相对转角初值，按测距残差选解'],
        ['单锚点','六条距离与三车局部运动','起点位置5自由度、局部转角3自由度','两种镜像、三种转角初值，按测距残差选解']])
    s+='''本轮检查求解是否收敛及规定坐标后雅可比矩阵的局部秩，同时保存所有初始解和连续输出。静止输入下，局部运动坐标转角尚无运动约束；测试轨迹包含相对运动后再求解。初始化结果直接进入后续估计，误差统计保留起点偏差及后续累积影响。

### 3.3 增加一个锚点后的精度变化

先把同一个锚点放在场景中部（55，75）米。测距白噪声标准差均为10厘米，标定残余固定偏差标准差为3厘米，同编号运动输入和车间测距逐点相同。无锚点组只使用三条距离建系及更新，单锚点组增加三条车锚距离；对比包含新增测量对初始化和连续更新的共同作用。

'''
    s+=tab('同输入条件下增加单锚点的结果',['运动输入','无锚点RMSE范围（米）','中部单锚点RMSE范围（米）','中部单锚点均值±标准差（米）'],[
        [label(p),span(cases[p+'_no_anchor_peer']),span(cases[p+'_anchor_区域中部']),mean(cases[p+'_anchor_区域中部'])] for p in PROFILES])
    options=[('vio_增强',False),('vio_精细',False),('vio_增强',True),('lio_增强',True)]
    s+=tab('输入条件、精度与设备数量',['仿真输入与方案','相对位置RMSE（米）','完整盒数量'],[
        [label(p)+('＋中部单锚点' if use_point else '，无锚点'),span(cases[p+('_anchor_区域中部' if use_point else '_no_anchor_peer')]),
         '三只车载盒＋一只单锚点盒' if use_point else '三只车载盒'] for p,use_point in options])
    s+='VIO增强加单锚点与VIO精细无锚点得到接近的误差范围，说明增加共同距离参考能够降低对本车运动输入质量的要求。无锚点组均复用三只现有盒，单锚点方案增加一只同型盒；输入档位的变化通过实车标定和前端输出质量核对。\n\n'
    s+=fig('VIO输入的定位误差过程','layout_vio_逐时定位误差')+fig('LIO输入的定位误差过程','layout_lio_逐时定位误差')
    s+='''过程图分别给出基础和增强输入下的定位误差，纵轴为202、203两车位置误差平方均值的平方根，保留每次距离更新后的波动。相对运动改变测量方向，本车运动漂移与测距更新在各时刻共同作用；图中的修正直接来自估计器计算，没有人工加入跳变。

单锚点提供了相对于固定锚点的距离约束，较低误差的运动输入可进一步改善相对位置。默认纯UWB使用现有盒内设备完成相对形状输出；连续固定轴向位置按本节结果选择运动输入及单锚点配置，结合设备数量和费用安排实车验证。

### 3.4 锚点位置的影响

选取区域中部、起点侧方、区域左侧、区域右侧及区域角部五个候选位置，每次仅选其中一个。三车轨迹、噪声样本及输入参数保持一致，每个位置按六条测距重新求初始关系并连续更新，9档输入各重复20次，共900次布点运行。

'''
    s+=fig('三车轨迹与候选布点','layout_候选布点')
    s+=tab('候选位置与测距几何',['候选位置','场景坐标（米）','车锚距离范围（米）','5秒方向变化中位数（度）'],[
        [p,f'（{v[0]:g}，{v[1]:g}）',f'{summary["placement_geometry"][p]["minimum_range_m"]:.2f}～{summary["placement_geometry"][p]["maximum_range_m"]:.2f}',f'{summary["placement_geometry"][p]["median_five_second_bearing_span_deg"]:.2f}'] for p,v in PLACEMENTS.items()])
    s+='''表中方向变化指车辆相对锚点的视线角在连续5秒内的跨度，对评价时段及三车取中位数，用于描述测距方向随运动的变化。布点比较同时包含距离、方向变化、起点几何与整段运动，不以某一个几何量单独判定最优位置。超过100米的车锚距离是候选位置比较中的几何条件，车间距始终保持在24.56米以内；这些候选均沿用相同测距误差模型，未模拟随距离变化的无线传播或天线遮挡。

'''
    s+=tab('不同布点下增强输入的结果',['候选位置','IMU增强RMSE（米）','VIO增强RMSE（米）','LIO增强RMSE（米）'],[
        [p]+[span(cases[f'{kind}_增强_anchor_{p}']) for kind in ('imu','vio','lio')] for p in PLACEMENTS])
    s+=fig('不同布点的20次误差分布','layout_布点误差分布')+fig('单锚点辅助估计与真值轨迹','layout_单点估计轨迹')
    s+='''在本轮轨迹及等测距误差条件下，区域右侧的VIO/LIO增强结果优于区域中部；IMU输入下各位置的结果仍受惯性传播误差显著影响。起点侧方的轨迹最近处仅约0.24米，实车布设需按安全距离和天线安装位置调整。选点时结合行驶路线检查车锚距离、遮挡和起步几何，再按同一位置误差口径复测，仿真位置排序不替代现场测距结果。

### 3.5 测距精度与RTE求解结果

测距精度对比保持中部单锚点和运动输入不变，改变逐时测距标准差，标定残余固定偏差的标准差仍为3厘米。下表保留每个条件的全部20次结果，直接观察改善测距后位置误差的变化。

'''
    s+=tab('单锚点方案的测距精度对照',['运动输入','20厘米测距RMSE（米）','10厘米测距RMSE（米）','5厘米测距RMSE（米）'],[
        [label(p)]+[span(previous_cases[f'{p}_single_s{x:.2f}']) for x in SIGMAS] for p in PROFILES])
    s+=fig('测距与运动输入质量的影响','single_20_传感器与测距精度')
    s+='''RTE求解测试采用300秒运动轨迹及仿真无误差局部里程计，测距误差按零均值、标准差分别为2、5、10厘米逐次生成，用于观察测距误差对初次建系和后续位置修正的影响。下图直接读取C++算法输出，横轴均从仿真0秒开始，纵轴为逐车相对位置误差。 **本图使用201局部固定轴：原点随201当前位置平移，X/Y轴保持201起点局部里程计方向。**

'''
    s+=fig('RTE逐车相对位置误差','replay_RTE逐车定位误差')
    s+='''三组首次有效输出分别为127.20、215.52、216.24秒，均为输入数据的仿真时间。图中灰色区间表示等待运动条件及初始坐标求解检查，横轴始终采用同一0～300秒。

图12使用5秒滑动窗口，配置要求至少一辆车在最近5秒内的局部位置包络对角线达到10米，之后才尝试初次建系。这是窗口内的运动范围，不是从起点累计行驶10米。在当前低速、含转弯的轨迹中，该要求到约118秒才首次满足；本次配置对低速运动较严格，是首次输出较晚的主要原因。初次解算还检查收敛、测距几何和位置方向不确定度，平移标准差配置为0.25米、方向标准差为0.1弧度。2厘米组随后在127.20秒形成有效结果；5、10厘米组在前面的运动窗口未形成有效解，待约215.46秒再次满足运动条件后，分别在215.52、216.24秒出线。

算法随数据推进间隔尝试解算。三组在首次有效输出之前，定位接口累计调用耗时分别约0.81、5.90、10.28秒，最长单次调用约178毫秒；这些是本次开发电脑记录的耗时，区别于127～216秒的仿真时间。结果说明本次较长的建系等待主要由轨迹、窗口及初始解检查条件共同造成，后续实车配置需结合低速起步运动重新核对。

2厘米组在294.48秒时，窗口更新使203车位置变化约2.76米，位置误差由0.646米变为2.117米，而测距拟合残差约1.7厘米。代码重建表明，新窗口对相对变换约束较弱时直接更新变换，造成位置跳变。厘米级残差描述距离拟合，位置误差由仿真真值直接计算。这组测试帮助确定初始关系检查及后续变换更新对位置输出的影响。

## 4 实测数据与接口应用

### 4.1 静止采集与位置回放

六组静止采集共记录3,229,145帧车载IMU及48,985帧UWB测距，已用于传感器分析、默认UWB位置回放及输入检查。下表用数据编号1～6列示采集规模。消息头时间戳（Header）为消息携带的测量时间，实测曲线按其相对起点表示。

'''
    h,rows=old_table(13);s+=tab('六组静止实测数据',h,rows)
    s+=fig('实测测距与在线位置','05_实测在线输入输出')+fig('实测三车相对队形','06_实测三车相对三角形')
    s+='''两批在线输出频率约9.24～9.42赫兹。三盒均已完成三目视觉采集，雷达点云采集已完成；点云与图像后续需按特征提取、标定和甲方实车调试流程接入。已有静止数据用于输入质量分析和回放，动态位置误差按独立运动真值评价。

'''
    s+=fig('主盒三目视觉采集画面','03_三路相机实拍')
    s+=fig('主盒实测雷达点云','replay_主盒实测点云')
    s+='点云图直接读取主盒/livox/lidar消息，累积225条消息，去除全零和非有限点后固定抽样绘制12万个点，分别给出俯视及侧视投影，颜色辅助区分点高。它展示采集场景的空间结构，与三目图像一起为后续实车视场检查、特征提取和安装标定提供数据。图中使用本盒雷达局部坐标。\n\n'
    s+='''### 4.2 对外位置接口

默认UWB输出话题为/cooperative_localization/poses_2d，坐标系为coop_ref_1_relative，采用瞬时三角形定义。RTE使用/cooperative_localization/rte/poses_2d，固定轴向相对坐标由坐标转换程序输出。下表给出定位输入及输出，接收方按车辆编号、坐标系、测量时间与有效性使用位置。 RTE原始话题的坐标标识为coop_rte_1_local，对应第1.3节的201局部固定轴。

'''
    h,rows=old_table(14);s+=tab('定位输入与输出',h,rows)
    s+='''单锚点仿真另导出位置CSV，任务局部系为coop_single_anchor，相对系为coop_single_anchor_relative。anchor_x_m、anchor_y_m表示以锚点为原点的固定轴向位置，x_m、y_m表示减去201车同刻位置后的相对向量。该离线文件给出位置结果，航向有效位为false；默认在线UWB话题保持现状。

RTE可选坐标适配输出为/cooperative_localization/rte/coordinates/poses_2d，坐标标识coop_rte_1_relative_fixed：在首次有效历元锁定201→202为正X，之后轴向固定，原点仍随201移动。它与原始RTE轴向可有一个固定转角。适配工具的任务表达coop_rte_1_task使用相同轴向，但把原点固定在首次有效历元的201位置；这是原点的另一种表达。图12读取原始RTE结果，不经过该适配转换。

## 5 实车采集与联调

实车使用同一个已知位置UWB锚点，记录锚点的天线坐标及安装高度，同时采集三条车间测距和三条车锚测距。车速保持2～3米/秒以内，最大车间距20～30米。沿起步、转弯及编队整体移动路线采集，分别记录无锚点和有单锚点的输入，复用本轮位置评价程序比较配置收益。

IMU通过静止窗口估计启动角速度均值和重力方向，加计开展多姿态标定；在安装后的温度与振动条件下检查偏置变化。相机、雷达与IMU完成时间和安装位姿标定后，在实际室外视场中输出本车运动，并按尺度、方向漂移和随机位移统计对应本报告的输入条件。

运动试验配套独立位置测量，例如载波相位差分定位（Real-Time Kinematic，RTK）或全站仪。定位侧按统一车辆编号输出相对位置和测量时间，通信、规划与控制沿用现有消息接口接入；需要地图坐标时，将实测方向关系与任务局部系结合换算。采集命令、启动顺序和时间检查继续使用21号操作文档。

## 6 复现与阶段结论

### 6.1 仿真与图表重算

测距精度数据位于data/single_anchor_outdoor_20261029，无锚点和布点数据位于data/single_anchor_layout_20261029。各目录summary.json给出逐条件统计，蒙特卡洛分组统计.csv便于查表，outputs保存全部重复的逐时位置。本版实验条件清单.json逐项列出报告采用的84个条件及输出文件；按对应输出能够复算全部1680次统计。

```bash
PYTHONPATH=tools OPENBLAS_NUM_THREADS=1 python3 tools/evaluate_single_anchor.py --trials 20
PYTHONPATH=tools OPENBLAS_NUM_THREADS=1 python3 tools/evaluate_anchor_layout.py
PYTHONPATH=tools python3 tools/check_anchor_layout.py
PYTHONPATH=tools python3 tools/plot_single_anchor.py
PYTHONPATH=tools python3 tools/plot_anchor_layout.py
source /opt/ros/humble/setup.bash
python3 tools/plot_report_replay.py --replay-rte
python3 tools/check_report_results.py
python3 tools/analyze_rte_initialization.py
```

手工修改本报告Markdown后，用以下命令更新其他格式；它们保留手工源稿：

```bash
PYTHONPATH=/tmp/cooperative_report_deps python3 tools/render_formal_report.py
python3 tools/export_formal_pdf.py
```

图1直接使用提供的原图。误差图及统计表读取保存的算法输出与真值，点云读取原始消息，三目图像使用实际采集画面。图中文字由绘图程序按明确文字与字体排版；结果图提供PNG、SVG及矢量PDF。逐图数据和重算命令见配套复现说明，SVG可用Inkscape修改。

### 6.2 阶段结论

三盒采集、默认UWB相对队形、独立RTE库和消息接口已形成可运行链路。六组静止数据支持输入分析及位置回放，本轮仿真补充了三类运动输入、三档质量、三档测距误差与五个单锚点候选位置的量化对比。参数、统计、误差过程及轨迹按同一条件相邻列示。

'''
    s+=f'在测距白噪声标准差10厘米、标定残余固定偏差标准差3厘米的条件下，默认纯UWB队形的20次RMSE为{span(previous_cases["uwb_s0.10"])}米。固定轴向定位采用LIO增强输入、无锚点时为{span(cases["lio_增强_no_anchor_peer"])}米；增加中部单锚点后为{span(cases["lio_增强_anchor_区域中部"])}米，改为区域右侧单锚点后为{span(cases["lio_增强_anchor_区域右侧"])}米。上述结果分别按规定坐标和明确的仿真输入计算。\n\n'
    s+='''配置上保留默认UWB满足相对队形使用，连续运动定位优先采用现有相机/雷达与IMU组合，并以一个已知位置UWB锚点提供共同距离参考，兼顾设备复用与配置投入。后续按低速编队和同一评价口径采集实车动态数据，完成位置输出与通信、规划和控制的联调。
'''
    initialization=json.loads((ROOT/'data/report_replay_20261029/初始化与坐标核对/核对结果.json').read_text())
    durations='、'.join(f'{r["cumulative_step_wall_seconds"]:.2f}' for r in initialization['runs'])
    maximum=max(r['maximum_step_wall_ms'] for r in initialization['runs'])
    s=s.replace('0.81、5.90、10.28秒',durations+'秒').replace('178毫秒',f'{maximum:.0f}毫秒')
    (DEST/'报告图表顺序.json').write_text(json.dumps({'figures':figures,'table_titles':tables},ensure_ascii=False,indent=2)+'\n')
    budget=dict(unit='人民币元',scope='一套=一只完整传感器盒；三车=三只；增加单锚点按另配一只同型完整盒；不列人力开发成本',
        complete_box_unit_price=dict(value=None,known_description='3万余元/只',basis='用户2026-10-29确认：一套是一只完整传感器盒',exact_transaction_price_pending=True),
        configurations=[dict(name='三车现有配置',complete_box_count=3,total='实际单盒成交价×3'),dict(name='三车加一个已知位置UWB锚点',complete_box_count=4,increment='1只同型完整盒，3万余元',total='实际单盒成交价×4'),dict(name='三车增加高配IMU',complete_box_count=3,imu_model='ADIS16495-2BMLZ',imu_count=3,reference_unit_price_including_vat_yuan=IMU_REFERENCE_PRICE_YUAN,reference_extended_price_yuan=round(IMU_REFERENCE_PRICE_YUAN*3,2),reserved_unit_price_yuan=IMU_BUDGET_PER_UNIT_WAN*10000,reserved_extended_price_yuan=IMU_BUDGET_PER_UNIT_WAN*30000)],
        reference_date='2026-10-29',public_price_source='https://www.digikey.cn/zh/products/detail/analog-devices-inc/ADIS16495-2BMLZ/7931993',manufacturer_model_source='https://www.analog.com/en/products/adis16495.html',sensor_model_scope='九档输入是误差模型条件，不声称该IMU已参与仿真或与某档逐项等同')
    (DEST/'硬件配置概算.json').write_text(json.dumps(budget,ensure_ascii=False,indent=2)+'\n')
    chosen=[dict(case,source_directory=str(OUT.relative_to(ROOT))) for case in summary['conditions']]
    chosen.extend(dict(c,source_directory=str(INPUTS.relative_to(ROOT))) for c in previous['conditions']
                  if c['profile']=='uwb' or (c['mode']=='single' and abs(c['range_sigma_m']-.1)>1e-8))
    assert len(chosen)==84 and sum(len(c['trials']) for c in chosen)==1680
    index=[]
    for c in chosen:
        a=arr(c);index.append(dict(name=c['name'],profile=c['profile'],mode=c['mode'],source_directory=c['source_directory'],
            output_file=c['source_directory']+'/outputs/'+c['name']+'.npz',trials=len(a),mean_rmse_m=float(a.mean()),sample_std_rmse_m=float(a.std(ddof=1)),minimum_rmse_m=float(a.min()),maximum_rmse_m=float(a.max())))
    (DEST/'本版实验条件清单.json').write_text(json.dumps(dict(conditions=84,trials=1680,rte_diagnostic_runs=3,conditions_index=index),ensure_ascii=False,indent=2)+'\n')
    return s


def supporting():
    text='''# V1.4阅读与复现

正式源稿为本目录“多平台协同定位硬件仿真测试报告与联调方案_V1.4_设备分级与锚点仿真版.md”。手工修改后运行render_formal_report.py与export_formal_pdf.py，保留手工Markdown。build_v14_report.py默认也保留源稿，只有--refresh-content才按生成器重新组织正文。

## 数据与运行

- 本版实验条件清单.json给出正文所用的84个条件、1680次输出文件和分组统计，不要求读者先了解历史版本。试验目的及条件构成见正文表1。
- data/single_anchor_layout_20261029提供无锚点及五种单锚点位置的63个条件；data/single_anchor_outdoor_20261029提供另外18个5/20厘米测距条件及3个默认UWB条件。全部位置结果都保留在outputs中。
- data/report_replay_20261029保存三组C++重新回放输出、首次有效时刻、点云绘图点及复现核对.json。C++输出逐字段比较时只排除随电脑负载变化的计算耗时列，位置、状态、时间及修正计数都核对。
- 本次费用与图线说明修订不改变传感器仿真输出。图12的灰色区间为建系阶段，首次有效时刻127.20、215.52、216.24秒，横轴只在整图底部标注一次；没有补零或移动出线时刻。
- 坐标集中说明在正文1.3节：图12用201本车局部固定轴，相对原点随201移动；可选首次有效201→202定轴适配器另行说明。
- 固定测距偏差在正文按天线/收发电路延迟补偿残余解释，说明它对平均测距及相对位置的持续影响；具体抽样实现留在源码和内部核对文件。物理依据见测距固定偏差_实际意义核对.md，3厘米仍为仿真假设。
- 建系等待核对见RTE坐标与建系等待说明.md。analyze_rte_initialization.py复算5秒窗内10米范围的满足时段和实际接口耗时，区分仿真时间与运算时间。核对记录保存在data/report_replay_20261029/初始化与坐标核对，不修改算法或参数。

```bash
PYTHONPATH=tools OPENBLAS_NUM_THREADS=1 python3 tools/evaluate_single_anchor.py --trials 20
PYTHONPATH=tools OPENBLAS_NUM_THREADS=1 python3 tools/evaluate_anchor_layout.py
PYTHONPATH=tools python3 tools/check_anchor_layout.py
PYTHONPATH=tools python3 -m unittest tools/test_single_anchor.py tools/test_anchor_layout.py
PYTHONPATH=tools python3 tools/plot_single_anchor.py
PYTHONPATH=tools python3 tools/plot_anchor_layout.py
source /opt/ros/humble/setup.bash
python3 tools/plot_report_replay.py --replay-rte
python3 tools/check_report_results.py
python3 tools/analyze_rte_initialization.py
PYTHONPATH=/tmp/cooperative_report_deps python3 tools/render_formal_report.py
python3 tools/export_formal_pdf.py
```

## 图1与可编辑图

图1使用figures/图1_协同定位系统组成.png原文件，原样备份在figures/原始素材；SHA256记录在图1原图保留清单.json。生成器校验原图哈希，Word内嵌同一PNG字节，不重新绘制。用户原图外的透明区域按文档白底显示。

新数据曲线提供同名PNG、SVG、PDF；SVG保留文字节点，可用Inkscape打开，按T修改文字，选择工具移动和缩放对象。详细说明见SVG修改说明.md。旧图1的六框SVG/PPTX不对应本次用户提供的五框PNG，不应当作该原图的可编辑版本。

## 参数、费用与内部资料

参数来源核对与参考实现保留在参数来源_可观性_初始化核对.md、锚点布置参考与比较说明.md及data/single_anchor_outdoor_20261029/参数与可观性核对，不在客户正式正文列出外部项目。VIO/LIO采用显式位移输出模型；没有把模型结果写成已运行原始图像/点云前端的精度。

正文第2.5节围绕传感器配置与方案选择说明设备复用、配置投入与选择单锚点的理由，删除独立费用比较表及报价来源。数量、型号与价格依据继续保存在硬件配置概算.json和硬件费用依据核对.md，供内部复核；仿真输入档位与采购价格分别说明。

## 图线与数据对应

| 图线或表格 | 输入与输出 | 重算命令（工程根目录） |
|---|---|---|
| 传感器分级、单锚点及布点统计 | 本版实验条件清单.json列出的84个输出；对应summary.json和outputs | python3 tools/check_report_results.py |
| 无锚点、布点及定位误差图 | data/single_anchor_layout_20261029/outputs | python3 tools/plot_anchor_layout.py |
| 测距精度影响图 | data/single_anchor_outdoor_20261029/outputs | python3 tools/plot_single_anchor.py |
| RTE逐车误差图 | data/rte_expanded_20261025/exp_odometry_*_seed0.csv及.truth.csv，C++回放输出另存report_replay目录 | source /opt/ros/humble/setup.bash 后 python3 tools/plot_report_replay.py --replay-rte |
| 主盒实测点云 | data/three_boxes_20260811_012046中201盒/livox/lidar原始消息 | 同上；225条消息，过滤全零/非有限点后按固定编号抽样绘制 |
| 实测测距、位置及相对队形 | data/rte_accuracy_20261023/uwb_baseline_20261017的uwb_records.csv、pose_records.csv | python3 -c 'import tools.plot_sensor_anchor_v14 as p; p.measured_plots()'；使用PYTHONPATH=tools |
| 三目采集画面 | data/camera_samples_analysis/video33_preview.png、video42_preview.png、video51_preview.png | 原始采集画面，未以仿真或生成图替代 |

RTE的三组轨迹和带噪运动输入的融合仿真均由程序输出计算误差；VIO/LIO使用位移输出误差模型，不声称已经运行原始相机/雷达前端。点云图表示本机采集结构，不将它写成共同地图或定位融合结果。图中文字是代码中写明的文字，由字体排版，非生成式图片。

每次修改前后对照、意见与版本记录分别保存在docs/报告改动前后对照记录.md、docs/报告编制与对外表述要求_持续记录.md和docs/报告内部版本记录.md。默认UWB、原RTE库和V1.3均按既有哈希核对保留。
'''
    (DEST/'阅读与复现说明.md').write_text(text)
    questions=['三类输入的误差参数分别是什么，哪部分来自假设、哪部分对应实际采集？','默认UWB的瞬时队形轴和固定轴向运动定位有何用途区别？','无锚点初始化是否使用了任何车锚距离？','相同输入下加一个点，误差分别从多少变成多少？','为什么改善初始化初值覆盖，不能说成传感器精度提高？','五个候选点是否同时存在？每种位置的车锚距离是否都满足同样传播条件？','单锚点为什么不能直接给出地图朝向，采用的局部轴怎么定义？','为什么右侧在这条轨迹下优于中部，现场还需要检查什么？','84种条件怎样组成，每种20次实际改变什么？','一套是几只盒，三车及加单锚点各多少只，费用依据是什么？','RTE三组曲线为什么从不同时刻出线，灰色区间表示什么？','实测点云从哪个话题读取，累积和抽样方式是什么？','VIO/LIO模型试验如何在安装后的实车输入上复测？','SVG能修改哪些对象，PNG是否能直接变成逐对象可编辑原稿？']
    (DEST/'V1.4_报告理解核对问题.md').write_text('# 报告理解核对问题\n\n'+'\n'.join(f'{i+1}. {q}' for i,q in enumerate(questions))+'\n')
    for name in ['build_single_anchor_report.py','build_v14_report.py','evaluate_single_anchor.py','evaluate_anchor_layout.py','plot_anchor_layout.py','plot_report_replay.py','analyze_rte_initialization.py','check_report_results.py','check_anchor_layout.py','test_anchor_layout.py','plot_single_anchor.py','plot_sensor_anchor_v14.py','check_single_anchor.py','test_single_anchor.py','audit_outdoor_assumptions.py','build_stage_report.py','render_formal_report.py','export_formal_pdf.py','report_plot_fonts.py']:
        shutil.copy2(ROOT/'tools'/name,DEST/'复现脚本'/name)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--refresh-content',action='store_true');args=p.parse_args()
    if args.refresh_content or not CURRENT.exists():CURRENT.write_text(generate())
    supporting();render_word(CURRENT.read_text(),CURRENT.with_suffix('.docx'),title='多平台协同定位硬件仿真测试报告与联调方案',header='')
    print('已排版：',CURRENT)


if __name__=='__main__':main()
