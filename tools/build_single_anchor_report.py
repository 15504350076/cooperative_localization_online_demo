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
COMMON=ROOT/'data/common_reference_20261030'
PEAKS=ROOT/'data/fusion_peak_analysis_20261030'
STARTUP=ROOT/'data/rte_startup_diagnostics_20261030'
GRADES=('基础','增强','精细')
IMU_REFERENCE_PRICE_YUAN=47224.54
IMU_BUDGET_PER_UNIT_WAN=5.


def label(p):return p.replace('imu','IMU').replace('vio','VIO').replace('lio','LIO').replace('_',' ')


def generate():
    original=DEST/'figures/图1_协同定位系统组成.png'
    assert hashlib.sha256(original.read_bytes()).hexdigest()==FIGURE1_SHA,'图1原图发生变化，停止生成'
    old=BEFORE.read_text();summary=json.loads((OUT/'summary.json').read_text())
    current=json.loads((COMMON/'summary.json').read_text());cases={c['name']:c for c in current['conditions']}
    previous_cases=dict(cases)
    for profile in PROFILES:previous_cases[profile+'_single_s0.10']=cases[profile+'_anchor_区域中部']
    peak_results=json.loads((PEAKS/'summary.json').read_text())
    startup=json.loads((STARTUP/'summary.json').read_text())['conditions']
    maneuver=json.loads((ROOT/'data/maneuver_observability_20261101/summary.json').read_text())
    maneuver_types=json.loads((ROOT/'data/maneuver_types_20261102/summary.json').read_text())['conditions']
    straight=json.loads((ROOT/'data/straight_maneuvers_20261102/summary.json').read_text())['conditions']
    figures=[];tables=[]
    def tab(title,head,rows):
        tables.append(title)
        return f'表{len(tables)} {title}\n\n| '+' | '.join(head)+' |\n|'+'|'.join(['---']*len(head))+'|\n'+''.join('| '+' | '.join(map(str,row))+' |\n' for row in rows)+'\n'
    def fig(title,filename):
        figures.append(filename);return f'![图{len(figures)} {title}](figures/{filename}.png)\n\n'
    def arr(c):return np.array([r['position_rmse_m'] for r in c['trials']])
    def span(c):a=arr(c);return f'{a.min():.3f}～{a.max():.3f}'
    def mean(c):a=arr(c);return f'{a.mean():.3f}±{a.std(ddof=1):.3f}'
    def imu_curve_metrics(profile,mode):
        a=np.load(COMMON/'outputs'/f'{profile}_no_anchor_{mode}.npz');truth=a['truth']
        d=(a['position'][:,:,1:]-a['position'][:,:,:1])-(truth[None,:,1:]-truth[None,:,:1])
        error=np.sqrt(np.mean(np.sum(d*d,axis=-1),axis=-1));t=a['time_s'];avg=error.mean(axis=0)
        active=(t>=60)&(t<=300);slope=float(np.polyfit(t[active],avg[active],1)[0])
        return dict(slope=slope,final=float(avg[-1]),rmse=float(np.sqrt(np.mean(error[:,600:]**2))))
    def old_table(number):
        block=re.search(r'^表'+str(number)+r' [^\n]+\n\n((?:\|[^\n]*\n)+)',old,re.M)[1]
        rows=[[v.strip() for v in line.strip().strip('|').split('|')] for line in block.splitlines()]
        return rows[0],rows[2:]
    def no_anchor_results(kind):
        return '最后一列是20次重复仿真中，每次完整评价段RMSE的均值和样本标准差；每次RMSE合并统计202、203两车相对201车的60～300秒误差。\n\n'+tab(kind.upper()+'输入的无锚点结果',['档位','仅运动传播RMSE（米）','加入车间测距RMSE（米）','加入测距后均值±标准差（米）'],[
            [g,span(cases[f'{kind}_{g}_no_anchor_start']),span(cases[f'{kind}_{g}_no_anchor_peer']),mean(cases[f'{kind}_{g}_no_anchor_peer'])] for g in GRADES])
    toc=['系统方案与试验安排','传感器分级与精度分析','单锚点辅助与布置对比','实测数据与接口应用','实车采集与联调','结论']
    s='# 多平台协同定位硬件仿真测试报告与联调方案\n\n浙江大学\n\n<!-- pagebreak -->\n\n## 目录\n\n章节目录\n\n| 章节 | 内容 |\n|---|---|\n'
    s+=''.join(f'| [{i}](#chapter_{i}) | [{title}](#chapter_{i}) |\n' for i,title in enumerate(toc,1))
    s+='\n<!-- pagebreak -->\n\n## 1 系统方案与试验安排\n\n'
    s+='''本项目面向卫星导航拒止条件下多平台协同任务，已形成三盒传感器采集、在线相对队形输出、联合录包和仿真复现流程。协同定位系统主盒（系统中称为201盒）接收三车信息，对外发送相对位置；202、203盒对应另外两车。浙江大学开展定位算法及接口开发，定位结果可供通信、规划、控制与显示使用。

默认方案以多平台为节点、超宽带测距（Ultra-Wideband，UWB）链路为连接，维护测距拓扑并恢复动态相对队形；本次采用三车配置验证。惯性测量单元（Inertial Measurement Unit，IMU）运动推算和相对变换估计（Relative Transformation Estimation，RTE）分别进行。本轮围绕三车低速编队，比较运动输入质量、测距精度及增加一个已知位置UWB锚点后的结果，为后续实车配置和联调提供依据。

'''
    s+=fig('协同定位系统组成','图1_协同定位系统组成')
    s+='''### 1.1 运动条件与评价口径

传感器分级和锚点布置仿真的时长为300秒，前30秒静止，30～60秒积累起步及转弯数据用于求初始坐标关系，60～300秒统计连续定位结果。RTE低速初始化对照及评价口径见第3.5节。三车整体运动范围约85×115米，最高速度2.39米/秒，最大车间距24.56米。位置、测距及运动输入按10赫兹处理，保持连续视距测距。平面模型采用同高天线参考点。主计算周期为0.1秒，即10赫兹；UWB距离和视觉惯性里程计（Visual-Inertial Odometry，VIO）或激光惯性里程计（LiDAR-Inertial Odometry，LIO）的局部位移按10赫兹进入位置计算。IMU白噪声用100赫兹原始采样水平折算到10赫兹，表示噪声强度的参考采样率。

UWB依据无线信号的传播时间计算距离，天线及收发电路本身也会引入延迟。延迟补偿后的残余可能使同一对设备的平均测距持续偏大或偏小，在短时、工作条件稳定时可用近似固定的测距偏差表示。这类偏移会随重复测量保留，进而影响三车距离关系和相对位置；逐次测量的随机波动则另行建模。因此，传感器分级、锚点布置及默认UWB对照保留了固定偏差项，用于考察测距标定残余对定位的影响。其标准差设为3厘米，作为本轮仿真条件。RTE低速初始化对照采用的测距模型在第3.5节单独说明。

仿真过程中同时保留每辆车的真实位置作为对照。位置误差评价202、203车相对201车的位置向量：将估计相对向量减去同一坐标定义下的真值向量，取二维长度。每次运行的位置均方根误差（Root Mean Square Error，RMSE）为两车全部评价时刻的误差平方均值再开平方，单位为米；随后统计20次运行RMSE的均值、样本标准差及最小至最大范围。它直接量化相对位置偏差，测距拟合残差另行说明。

### 1.2 对比试验的组成

IMU、VIO、LIO各设三档输入质量。VIO/LIO仿真按局部位移输出建立误差模型，将滩涂、沙地和训练场中可能出现的纹理不足、强反光、扬尘、稀疏点面特征及局部遮挡等影响，折算为尺度误差、方向漂移和随机位移条件。第2.3节和第2.4节的曲线和表格按上述位移输入条件计算；第2.2节按IMU测量误差模型计算。输入档位表示本车运动输出质量从基础到精细的变化。

试验比较三件事：提高本车运动输入质量能降低多少误差，增加一个已知位置UWB锚点有多大作用，以及锚点的位置和测距精度怎样影响结果。三类输入各设三档，共9种运动输入。无锚点组使用车间距离和本车运动，单锚点组再加入三车到同一个锚点的距离；各组按实际可用测量求解初始关系，同编号对比使用相同运动与测距噪声样本。

20次重复用于观察固定轨迹下随机误差带来的离散程度，属于噪声蒙特卡洛分析。所有重复均进入统计；单次过程图采用重复试验1，图3、图14、图16和图18展示20次结果的平均过程。仿真真值用于生成测量和评价，算法直接使用带误差的运动和距离输入输出定位结果。
'''
    s+=tab('仿真项目与分组',['试验目的','条件设置','条件数','重复次数'],[
        ['比较运动输入质量','9档输入×仅传播/加入车间测距','18','每条件20次'],
        ['比较单锚点位置','9档输入×5个候选位置，每次只有一个锚点','45','每条件20次'],
        ['比较测距精度','中部单锚点下，9档输入×5/20厘米测距；10厘米条件已包含在单锚点位置对比中','18','每条件20次'],
        ['检验默认UWB队形','5/10/20厘米三档测距','3','每条件20次']])
    s+='''以上共84种条件、1680次重复运行。另以三档测距误差、四种建系配置开展12次RTE库对照，并对四档VIO/LIO输入进行原始重放及四项受控检查，分析误差峰值和初始化条件。这些诊断复用已有输入，作为独立结果列示；主统计仍为1680次独立噪声重复。

### 1.3 本报告使用的坐标系

**本报告的分级、布点仿真与RTE原始输出采用201局部固定轴：原点为201当前位置，X轴平行201自身起点局部里程计X轴，Y轴在水平面内逆时针转90度，Z轴向上。** 201移动时原点随之平移，坐标轴保持起点方向。无锚点、单锚点仿真、机动对照以及RTE原始输出使用这一约定。

'''
    s+=tab('定位结果的两种坐标约定',['输出','原点','轴向','用途'],[
        ['默认UWB队形','201当前位置','正X沿当刻201→202，随队形变化','相对布局、距离和编队形状'],
        ['运动融合与RTE','201当前位置','平行201自身起点局部里程计轴，保持固定','连续相对位置和运动轨迹']])
    s+='''增加锚点只增加三车到同一个已知位置的距离观测，输出仍以201为原点。系统内部在便于求解的坐标中计算，输出时减去201同刻位置，再按初始化估计的201局部转角转换至固定轴；真值按同一定义用于评价，所有分级和布点图表采用同一口径。

两种坐标分别服务队形形状与连续运动；默认UWB方案和坐标约定保持原样。RTE坐标转换话题另外以首次有效结果中201指向202的方向为固定X轴，原点仍为201当前位置。使用时按话题选择对应轴向；需要地图坐标时再结合201的地图位置与方向关系换算。

'''
    s+='''## 2 传感器分级与精度分析

### 2.1 默认UWB方案

默认方案把车间距离组织为测距拓扑：节点对应平台，链路对应有效测距，随测量更新相对几何关系。定位接口按节点编号保存和输出位置，面向多平台编队接入；本次三车验证由三条车间链路构成闭合队形，以201为原点、当刻201至202为正X，203置于Y正侧。三车几何求解是当前默认验证配置，扩展节点时按实际连接和几何约束组织求解。下表按三车距离关系计算队形坐标，并在同一定义下与真实位置比较。

'''
    s+=tab('默认纯UWB队形仿真',['测距标准差（厘米）','RMSE均值±标准差（米）','RMSE范围（米）'],[
        [int(x*100),mean(previous_cases[f'uwb_s{x:.2f}']),span(previous_cases[f'uwb_s{x:.2f}'])] for x in SIGMAS])
    s+='''在测距随机误差标准差为10厘米、标定残余固定偏差标准差为3厘米的条件下，默认纯UWB队形20次运行的RMSE范围为0.187～0.225米。默认UWB结果用于评价瞬时队形坐标下的相对位置；以下运动融合组采用201固定轴，用于评价连续相对位置，两者按各自的坐标定义使用。

### 2.2 UWB与六轴MEMS

各运动分组使用扩展卡尔曼滤波（Extended Kalman Filter，EKF）融合运动与距离，按10赫兹依次进行运动预测和距离更新。IMU分支联合估计三车的平面位置、速度与局部坐标转角；先按加速度积分预测位置和速度，再根据当前预测位置计算车间距离。测得距离减去预测距离得到测距残差，算法依据运动和测距的不确定度，沿两车连线方向修正两端状态。状态之间的相关性使速度、局部转角也能随多时刻运动得到调整。

六轴微机电系统（Micro-Electro-Mechanical Systems，MEMS）惯性器件输入包含加速度、角速度及残余误差。静止阶段计算角速度均值、建立重力方向，初始速度设为零；运动阶段按加速度和角速度进行位置、速度和方向推算。三档残余参数对应不同的运动输入质量，结果见后面的误差曲线和统计表。

视觉/雷达惯性运动估计把加计和陀螺偏置加入状态，用连续图像或点云观测修正偏置。本轮IMU分支按位置、速度和局部转角进行推算，三档输入结果用于比较不同运动输入质量。


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
    s+='''白噪声按100赫兹逐样本标准差设置，按10样本平均的噪声水平折算到10赫兹；偏置变化和倾角误差随运动输入进入位置推算。前30秒按已知静止处理，运动后连续推算。纯IMU传播和加入车间测距的误差变化见图3和表7。

'''
    s+=no_anchor_results('imu')+fig('MEMS输入的无锚点对比','review_layout_imu_无锚点分级')
    s+=tab('纯IMU传播与车间测距的误差变化',['输入档位','仅IMU传播误差增长率（米/秒）','加入车间测距误差增长率（米/秒）','仅IMU传播300秒误差（米）','加入车间测距300秒误差（米）'],[
        [g,f'{imu_curve_metrics("imu_"+g,"start")["slope"]:.3f}',f'{imu_curve_metrics("imu_"+g,"peer")["slope"]:.3f}',f'{imu_curve_metrics("imu_"+g,"start")["final"]:.2f}',f'{imu_curve_metrics("imu_"+g,"peer")["final"]:.2f}'] for g in GRADES])
    s+=fig('纯IMU传播与车间测距的误差过程','review_imu纯传播与测距')
    s+='分布图的箱体表示20次RMSE的中间50%，箱内横线为中位数；须线按1.5倍四分位距确定，超出的重复结果单独画点。对数纵轴用于同时显示不同误差量级，所有结果均保留。\n\n'
    s+=f'在这条300秒轨迹中，加入车间测距后，三档IMU的误差增长趋势均明显减缓。以增强档为例，60～300秒平均误差曲线的线性拟合斜率由{imu_curve_metrics("imu_增强","start")["slope"]:.3f}米/秒变为{imu_curve_metrics("imu_增强","peer")["slope"]:.3f}米/秒，300秒的20次平均相对位置误差由{imu_curve_metrics("imu_增强","start")["final"]:.2f}米降至{imu_curve_metrics("imu_增强","peer")["final"]:.2f}米。表7和图3采用相同的初始化、IMU输入和测距噪声，体现了本次仿真中车间测距对相对位置误差的修正效果。\n\n'
    s+='''

### 2.3 UWB、相机与IMU

VIO由相机特征运动和IMU提供连续局部位移。算法先把各车局部位移转换到共同轴向，预测下一时刻位置，再用车间测距修正三车位置及转角。该分支联合状态为三车二维位置和三个局部转角，共9维；测距改变两车的相对几何，多时刻位移使这些修正传递到局部方向。结果仍按201固定轴输出。

本轮VIO把滩涂、沙地和训练场中的特征减少、反光和短时遮挡折算为更大的尺度误差、方向漂移和随机位移。基础、增强、精细三档分别对应100米累计随机位移约100、60、30厘米；图4和表9给出这组室外退化输入下的连续定位结果。三档数值用于比较不同本车运动输出质量对相对定位的影响，实车接入后按同一三项误差口径核对。

'''
    for kind,title in [('vio','VIO'),('lio','LIO')]:
        if kind=='lio':s+='''### 2.4 UWB、雷达与IMU

LIO通过点云几何匹配与IMU运动传播形成局部位移，按同一9维状态模型与UWB共同计算：位移预测位置，车间测距更新相对状态。本轮把滩涂、沙地和训练场中平坦地表、稀疏回波、扬尘和局部遮挡造成的点面约束减弱，折算为表中尺度、方向与随机位移误差。

'''
        s+=tab(title+'室外退化输入条件',['输入档位','尺度标准差（%）','方向变化标准差（弧度/秒）','100米累计随机位移误差（厘米）'],[
            [g,f'{PROFILES[kind+"_"+g]["scale_std"]*100:g}',f'{PROFILES[kind+"_"+g]["yaw_drift_std"]:g}',f'{PROFILES[kind+"_"+g]["displacement_noise"]*1000:g}'] for g in GRADES])
        s+=no_anchor_results(kind)+fig(title+'输入的无锚点对比','review_layout_'+kind+'_无锚点分级')
        s+=f'{title}输入由基础改善到增强时，加入车间测距后的相对位置RMSE由{span(cases[kind+"_基础_no_anchor_peer"])}米变为{span(cases[kind+"_增强_no_anchor_peer"])}米；精细输入为{span(cases[kind+"_精细_no_anchor_peer"])}米。这组对比用于明确本车运动输出质量对连续相对定位的影响，为实车前端标定和选配提供参考。\n\n'
        if kind=='vio':
            selected=next(r for r in peak_results['conditions'] if r['profile']=='vio_增强')
            first_peak,second_peak=selected['peaks'][:2]
            s+='<!-- pagebreak -->\n\n'
            s+=fig('无锚点时VIO输入的相对位置误差','review_VIO无锚点误差过程')
            s+=f'''图5展示一组无锚点仿真中VIO基础和增强输入的误差变化。横轴为仿真时间，纵轴为202、203两车相对201位置误差的合并值。VIO增强组在{first_peak["time_s"]:.1f}秒出现{first_peak["error_m"]:.3f}米的误差峰值。比较估计队形与仿真真值，此时整个队形的方向偏差约为{first_peak["common_rotation_error_deg"]:.2f}度，去除这一方向偏差后的形状分量为{first_peak["remaining_shape_error_m"]:.3f}米，表明这次峰值主要由队形方向偏差造成。

下表和图6进一步比较峰值时的方向与队形形状。方向、形状分解用于分析峰值成因，位置误差仍按原始定位结果与真值之差计算。

'''
            rows=[]
            for r in selected['peaks'][:2]:
                rows.append([f'{r["time_s"]:.1f}',f'{r["error_m"]:.3f}',f'{r["common_rotation_error_deg"]:.2f}',f'{r["remaining_shape_error_m"]:.3f}',f'{r["maximum_peer_range_m"]:.2f}',f'{r["minimum_triangle_altitude_m"]:.2f}'])
            s+=tab('VIO增强组的典型峰值',['仿真时刻（秒）','位置误差（米）','共同转角误差（度）','形状分量（米）','最大车间距（米）','队形最小高度（米）'],rows)
            s+=fig('峰值与队形方向','review_峰值与队形方向')
            s+=f'''在{first_peak["time_s"]:.1f}秒，切向误差占平方误差约{first_peak["tangential_energy_fraction"]*100:.1f}%；当时最大车间距{first_peak["maximum_peer_range_m"]:.2f}米、队形最小高度{first_peak["minimum_triangle_altitude_m"]:.2f}米，三车间距仍在设定范围内，队形也未接近共线。{second_peak["time_s"]:.1f}秒也出现相近现象。纯车间测距直接约束连线长度，对同一时刻整支队形共同转动的约束较弱；固定轴向中的方向误差可在形状保持较好时形成明显位置误差。

'''
        if kind=='lio':
            s+='''### 2.5 UWB与不同运动输入的组合比较及配置选择

前述试验分别比较了UWB与IMU航位推算、VIO和LIO运动输入的组合。本节结合定位结果分析运动输入与测距变化的影响，并说明传感器配置。

本次仿真中，VIO增强与LIO基础、VIO精细与LIO增强分别采用相同的位移误差参数和同编号扰动，表13按相同输入条件合并列示。表中数值为相对位置均方根误差；两类传感器的实际运动输出质量由具体设备和环境决定。针对前述VIO和LIO无锚点结果，选择第1次重复试验，进一步保持60秒初始化、相同随机样本及其余参数不变，分别将60秒后的运动输入方向漂移设为零、将测距随机项设为零（固定偏差保留）、将滤波方向过程方差扩大100倍。另用0～90秒测量重新建系，各对照均由测量输入求解。下表统一在90～300秒比较，避免不同起始时刻影响统计。

'''
            variants=['原始输入','去除后续方向漂移','去除后续测距随机项','方向过程方差扩大100倍','90秒测量建系']
            controls={'原始输入':'原始条件','去除后续方向漂移':'无后续方向漂移','去除后续测距随机项':'无后续测距随机项','方向过程方差扩大100倍':'方向方差扩大100倍','90秒测量建系':'90秒数据建系'}
            pr={r['profile']:r for r in peak_results['conditions']}
            precise=json.loads((ROOT/'data/fusion_peak_vio_precise_20261102/summary.json').read_text())['conditions'][0]
            assert precise['profile']=='vio_精细'
            for v in variants:
                np.testing.assert_allclose(pr['vio_增强']['same_interval_90_300_rmse_m'][v],pr['lio_基础']['same_interval_90_300_rmse_m'][v],atol=1e-10)
                np.testing.assert_allclose(precise['same_interval_90_300_rmse_m'][v],pr['lio_增强']['same_interval_90_300_rmse_m'][v],atol=1e-10)
            s+=tab('VIO与LIO误差对照',['对照条件','VIO基础（米）','VIO增强／LIO基础（米）','VIO精细／LIO增强（米）'],[
                [controls[v]]+[f'{pr[p]["same_interval_90_300_rmse_m"][v]:.3f}' for p in ('vio_基础','vio_增强','lio_增强')] for v in variants])
            s+='图8分别展示VIO增强和LIO增强的误差过程；相同输入条件下，结果也分别对应LIO基础和VIO精细。\n\n'
            s+=fig('VIO与LIO峰值对照','review_同输入峰值对照')
            vio_diag=pr['vio_增强']['same_interval_90_300_rmse_m']
            s+=f'''VIO增强组在90～300秒的原始RMSE为{vio_diag["原始输入"]:.3f}米；去掉后续方向漂移、去掉后续测距随机项、扩大方向过程方差和改用90秒测量建系后分别为{vio_diag["去除后续方向漂移"]:.3f}、{vio_diag["去除后续测距随机项"]:.3f}、{vio_diag["方向过程方差扩大100倍"]:.3f}和{vio_diag["90秒测量建系"]:.3f}米。LIO增强组相同条件下的原始RMSE为{pr["lio_增强"]["same_interval_90_300_rmse_m"]["原始输入"]:.3f}米，四种对照结果依次为{pr["lio_增强"]["same_interval_90_300_rmse_m"]["去除后续方向漂移"]:.3f}、{pr["lio_增强"]["same_interval_90_300_rmse_m"]["去除后续测距随机项"]:.3f}、{pr["lio_增强"]["same_interval_90_300_rmse_m"]["方向过程方差扩大100倍"]:.3f}和{pr["lio_增强"]["same_interval_90_300_rmse_m"]["90秒测量建系"]:.3f}米。这组对照有助于分析初始相对方向、运动输入和测距变化对定位结果的影响。

'''
    s+='''本章对比说明，改善本车运动输入能够降低连续相对位置误差，起步运动和初始坐标求解也会影响结果。相机或雷达与IMU联合形成的位移输入，可采用相同的车间测距和位置更新方法；实车前端按本章的尺度、方向漂移和位移误差口径核对输出质量。

前面的输入质量比较表明，连续相对定位的结果取决于本车运动输入质量。综合IMU、相机和雷达三类输入的误差范围，三车采用每车一只完整传感器盒，盒内保留IMU、相机和雷达接口；单盒费用为3万余元。若进一步提高本车运动输入，可比较逐车升级高配惯性器件的方案。

'''
    s+=tab('传感器配置与方案选择',['方案','设备配置','三车设备费用','技术作用'],[
        ['当前三车盒端方案','三只完整传感器盒，每车一只','三车合计为单盒费用×3','IMU、相机和雷达提供本车运动输入'],
        ['逐车升级惯性器件','三只高配IMU，每车一只','在三盒基础上增加约15万元（三只合计）','改善惯性运动输入，按实车标定核对性能']])
    s+='''前面的比较显示，继续提高每车运动输入需要增加较高等级器件。基于三车盒端现有配置，下一节在编队外设置一个已知位置的UWB锚点，比较增加共同距离参考后的定位结果。

## 3 单锚点辅助与布置对比

### 3.1 单锚点测量与初始关系

前面的传感器分级比较表明，继续提升每车运动输入需要增加较高等级器件。本节考察增加一个外部共同参考的方案：在编队外设置一个已知位置的UWB锚点，可安装在地标或已落地并保持静止的无人机上。

单锚点仿真的基准设置为：三辆车和一个位置已知、固定不动的锚点，天线按同一高度处理；三条车间测距和三条车锚测距均按10赫兹生成，共六条链路，每秒合计60条距离。基准测距随机误差标准差为10厘米，每条链路的标定残余固定偏差按标准差3厘米设置。第3.5节另比较5厘米、20厘米测距误差，固定偏差条件保持一致。

运动和评价设置为：三车整体运动范围约85×115米，最高速度2.39米/秒，最大车间距24.56米；仿真共300秒，前30秒静止，使用0～60秒的测距和运动信息求初始关系，以60～300秒评价相对位置。每种条件重复20次，无锚点与单锚点的同编号试验使用对应的运动及车间测距误差样本。第3.2节采用场景中部（55，75）米的锚点，第3.4节另比较其他四个位置；各位置的车锚距离见表21。模型采用连续视距测距，各位置使用相同的测距误差参数。

本车运动分别使用三档IMU、VIO或LIO仿真输入，具体误差参数见第2.2～2.4节；第3.3节机动对照采用VIO增强档，尺度误差标准差为1%，方向漂移标准差为0.0001弧度/秒，100米累计随机位移误差设为60厘米。输出沿用第1.3节的201局部固定轴，原点仍在201当前位置。新增距离的预测值是车辆到已知锚点的距离，EKF用车锚测距残差修正位置；锚点约束参与起步关系求解和后续更新。图9取实际仿真轨迹的两个时刻说明：201一直在原点，锚点在该相对坐标中的位置随201移动而变化。

单个锚点提供距离参考，结合车间测距和各车运动约束局部相对位置。所有平台及其局部方向绕锚点共同转动时，距离仍保持不变；本系统以201局部初始轴约定共同方向。地图方向由201的地图位置和方向关系换算，锚点提供共同距离参考。

'''
    s+=fig('201参考坐标与锚点观测','review_201参考坐标与锚点')
    s+='''初始化先看车辆刚开始静止时的距离，得到三车的大致位置关系；再看0～60秒内车辆怎样运动，每隔1秒取一次数据。对几组可能的起点位置和朝向分别计算，哪一组计算出的距离最接近实际测距，就采用哪一组作为初始相对位置和方向。无锚点时比较车间距离；增加锚点后，同时比较车间和车锚距离。

'''
    s+=tab('初始化输入与求解',['方案','使用的测量','坐标约定后的未知量','候选与选择'],[
        ['无锚点','三条车间距离与三车局部运动','起点位置3自由度、局部转角3自由度','24个候选关系，选测距残差最小者'],
        ['单锚点','六条距离与三车局部运动','起点位置5自由度、局部转角3自由度','6个候选关系，选测距残差最小者']])
    s+=fig('初始化候选关系','review_初始化候选关系')
    s+='''图10用一组带噪仿真输入说明候选初值。左侧分别为无锚点和单锚点时的两种镜像位置；右上为无锚点的四种共同转角初值，右下为三种车辆间相对转角初值。无锚点时组合得到24组初值；单锚点时把两种镜像与三个共同转角初值组合，得到6组初值。每组结合运动和测距分别求解，选择计算距离与测量距离差异最小的结果，用于后续位置计算。

本轮保存初始化求解、连续输出和规定坐标后的局部秩。初始化结果直接进入后续估计，误差统计包含起点偏差和后续累积影响。

### 3.2 增加一个锚点后的精度变化

本节采用第3.1节的基准条件，锚点位于场景中部。无锚点组只使用三条车间距离建系和更新，单锚点组增加三条车锚距离；对比包含新增测量对初始化和连续更新的共同作用。

'''
    s+=tab('同输入条件下增加单锚点的结果',['运动输入','无锚点RMSE范围（米）','中部单锚点RMSE范围（米）','中部单锚点均值±标准差（米）'],[
        [label(p),span(cases[p+'_no_anchor_peer']),span(cases[p+'_anchor_区域中部']),mean(cases[p+'_anchor_区域中部'])] for p in PROFILES])
    options=[('vio_增强',False),('vio_精细',False),('vio_增强',True),('lio_增强',True)]
    s+=tab('输入条件、精度与设备数量',['仿真输入与方案','相对位置RMSE（米）','完整盒数量'],[
        [label(p)+('＋中部单锚点' if use_point else '，无锚点'),span(cases[p+('_anchor_区域中部' if use_point else '_no_anchor_peer')]),
         '三只车载盒＋一只单锚点盒' if use_point else '三只车载盒'] for p,use_point in options])
    s+='VIO增强加单锚点与VIO精细无锚点得到接近的误差范围，说明增加共同距离参考能够降低对本车运动输入质量的要求。无锚点组均复用三只现有盒，单锚点方案增加一只同型盒；输入档位的变化通过实车标定和前端输出质量核对。\n\n'
    peak_errors=[];peak_angles=[]
    for mode in ('no_anchor_peer','anchor_区域中部','anchor_区域右侧'):
        data=np.load(COMMON/'outputs'/f'vio_增强_{mode}.npz')
        index=int(np.argmin(np.abs(data['time_s']-first_peak['time_s'])))
        pos=data['position'][0,index];truth=data['truth'][index]
        difference=(pos[1:]-pos[0])-(truth[1:]-truth[0])
        peak_errors.append(float(np.sqrt(np.mean(np.sum(difference*difference,axis=-1)))))
        estimate=pos[1:]-pos[0];reference=truth[1:]-truth[0]
        cross=np.sum(reference[:,0]*estimate[:,1]-reference[:,1]*estimate[:,0])
        dot=np.sum(reference*estimate)
        peak_angles.append(float(np.rad2deg(np.arctan2(cross,dot))))
    s+=fig('VIO输入的定位误差过程','review_layout_vio_逐时定位误差')
    s+=f'''图11比较相同运动输入和车间测距下，有无锚点时的相对位置误差。针对第2.3节所述{first_peak["time_s"]:.1f}秒的无锚点峰值，增加中部或右侧锚点后，同一时刻误差分别从{peak_errors[0]:.3f}米降至{peak_errors[1]:.3f}米和{peak_errors[2]:.3f}米，分别降低约{(1-peak_errors[1]/peak_errors[0])*100:.1f}%和{(1-peak_errors[2]/peak_errors[0])*100:.1f}%。图中对应的是VIO增强输入的第1次重复试验；表中的20次统计反映各组结果的变化范围。

峰值减弱的原因可以从队形方向看出来：在上述时刻，将估计队形与仿真真值比较，无锚点时整个队形的共同方向偏差约为{peak_angles[0]:.2f}度；加入中部或右侧锚点后分别为{peak_angles[1]:.2f}度和{peak_angles[2]:.2f}度。车间测距主要约束车辆之间的间隔，增加锚点后，三车到锚点的距离也参与初始关系求解和后续位置修正；这些测距与各车运动信息共同作用，使本次试验的队形方向偏差减小，位置误差峰值随之降低。上述对照同时包含初始化和运动过程的影响。单个锚点提供的是距离参考，地图方向仍需结合独立方向信息确定。

'''
    s+=fig('LIO输入的定位误差过程','review_layout_lio_逐时定位误差')
    s+='''图12使用相同口径展示LIO输入的误差过程。

单锚点提供了相对于固定锚点的距离约束，较低误差的运动输入可进一步改善相对位置。默认纯UWB使用现有盒内设备完成相对形状输出；连续固定轴向位置按本节结果选择运动输入及单锚点配置，结合设备数量和费用安排实车验证。

'''
    s+='''### 3.3 机动对相对定位的影响

'''
    s+='''直线运动对照包括同速平行、不同速平行和非平行直行，如图13所示。三组试验采用相同的三车起始位置，40～300秒保持直线运动，最高车速约0.63米/秒，最大车间距不超过18.7米；不同速平行组中202、203车的速度分别为0.575和0.625米/秒，非平行组中两车的行驶方向相对201车分别偏转约3.34度。

各组分别比较无锚点和单锚点情况，均使用VIO增强档和车间测距，单锚点组增加车锚测距；测距误差标准差为10厘米，每种条件重复20次，使用对应的误差样本。各组按实际可用的测距和运动信息求初始关系，以60～300秒的相对位置误差计算RMSE。这里的无锚点结果采用运动信息与车间测距联合计算，第2.1节默认纯UWB队形采用另一种坐标定义。

'''
    s+=fig('平行与非平行直线轨迹','review_平行与非平行直线轨迹')
    s+='''同速平行直行时，车间距离基本不变；非平行直行使车间位置关系发生变化；在假设局部位移准确的分析中，本组非平行直行对位置关系的局部约束有所改善。表18和图14给出20次试验的定位误差：表中列出各次RMSE的均值和范围，曲线为逐时误差的平均值。

'''
    s+=tab('直线运动下的相对位置误差',['运动方式', '无锚点均值（米）', '无锚点范围（米）', '单锚点均值（米）', '单锚点范围（米）'],[[c['name'],f'{c["peer"]["rmse_mean"]:.3f}',f'{c["peer"]["rmse_min"]:.3f}～{c["peer"]["rmse_max"]:.3f}',f'{c["single"]["rmse_mean"]:.3f}',f'{c["single"]["rmse_min"]:.3f}～{c["single"]["rmse_max"]:.3f}'] for c in straight])
    s+=fig('直线运动下的定位误差','review_直线机动定位误差')
    s+='''三种直线运动加入锚点后，20次试验的平均误差均有所降低。起步阶段持续直行时，仅凭距离和运动信息可能将左右相反的队形判断为同一种位置关系。单锚点的同速平行、不同速平行和非平行组分别有7、6和7次在60秒出现镜像队形，造成较大位置误差；表中保留了全部试验结果。后续可结合持续运动及相机或雷达提供的相对方位，进一步区分候选位置。

起步时适当改变车间相对方位，即改变201车指向其他车辆的方向，可能使不同初始位置关系产生更容易区分的测距变化。后面的同向平行运动组在0～60秒已有车间相对方位变化，80秒后保持车间相对位置不变，其20次试验在60～300秒的位置RMSE为0.116～0.589米，见表20。因此，可建议规划模块在条件允许时考虑起步阶段适当改变车间相对方位，并结合可用的初始位置和方位信息选择运动方式。

'''
    s+='''在上述直线对照之外，进一步比较不同相对机动幅度下的距离观测变化和位置误差。三组均使用中部单锚点、室外退化VIO增强输入、10厘米测距、20次重复和同编号随机误差样本；把202、203相对201的运动幅度分别设置为当前轨迹的0.6、1.0和1.2倍，由此形成三组距离和运动输入。图15左上轨迹以及表19、图16的误差，均以201当前位置为原点、201自身起点局部轴为固定轴。各组按各自的起步测量建立初始关系，统计60～300秒的定位结果。

'''
    s+=tab('机动条件与定位结果',['机动条件','车间相对速度最大值（米/秒）','距离观测最小非零敏感度中位数','RMSE均值±标准差（米）','RMSE范围（米）'],[
        [c['name'],f'{c["metrics"]["relative_speed_max"]:.3f}',f'{c["metrics"]["min_sv_median"]:.3f}',f'{c["metrics"]["rmse_mean"]:.3f}±{c["metrics"]["rmse_std"]:.3f}',f'{c["metrics"]["rmse_min"]:.3f}～{c["metrics"]["rmse_max"]:.3f}'] for c in maneuver['conditions']])
    s+=fig('相对机动与可观性','review_相对机动与可观性')
    s+=fig('相对机动误差过程','review_相对机动误差过程')
    s+='''图15包含四个子图：左上是三种机动下202、203相对201的轨迹；右上是最近5秒内距离观测最小非零敏感度的最小值，左下是最近5秒内最大车间相对速度的均方根；右下是20次重复试验在60～300秒的位置RMSE分布。表19的敏感度为60～300秒逐时值的中位数。图16为20次重复的平均误差过程。距离本身对整队共同转动仍有一个不敏感方向，因此右上采用最小非零值表示其余位置变化的可区分程度；数值越大，距离变化对位置变化越敏感。

从图16可见，本次单锚点对照中，较大相对机动对应的平均误差及部分峰值更高。例如在146.5秒，低、当前和高相对机动组的20次平均误差分别为0.278、0.428和0.511米。三组同时改变了相对轨迹、车间和车锚测距几何及本车运动输入，并分别求解初始关系，因此这一结果反映了本次条件下机动幅度变化与误差的关系。为进一步分辨峰值由哪些因素造成，还需分别控制测距几何、起点关系和运动输入开展对照。

'''
    mlow,mcurrent,mhigh=[next(c for c in maneuver['conditions'] if c['name']==name) for name in ('低相对机动','当前相对机动','高相对机动')]
    s+=f'''当前三组中，车间相对速度最大值由{mlow["metrics"]["relative_speed_max"]:.3f}米/秒增加到{mcurrent["metrics"]["relative_speed_max"]:.3f}和{mhigh["metrics"]["relative_speed_max"]:.3f}米/秒，距离观测几何的最小非零敏感度中位数由{mlow["metrics"]["min_sv_median"]:.3f}增加到{mcurrent["metrics"]["min_sv_median"]:.3f}和{mhigh["metrics"]["min_sv_median"]:.3f}；同时60～300秒位置RMSE均值由{mlow["metrics"]["rmse_mean"]:.3f}米增加到{mcurrent["metrics"]["rmse_mean"]:.3f}和{mhigh["metrics"]["rmse_mean"]:.3f}米。该对照中，相对机动改变了测距几何与各车运动输入，较高机动对应的位置RMSE没有降低。后续实车可按低速直行、转弯和编队变速分别采集，在相同测距条件下比较位置误差。

'''
    s+='''机动方式还包括车辆之间的加减速、转弯和行驶方向变化。为比较这些因素，在同一条201车轨迹上设置同向平行、相对加减速、转弯变队形和非平行运动四种情况，各重复20次。四组在0～60秒采用相同的三车运动、起步测量和初始关系，60～80秒平滑过渡，80秒后进入各自的运动方式；同编号试验使用相同的测距和运动误差样本。运动输入采用VIO增强档，使用同一个中部锚点，测距误差及评价坐标与本节前述条件一致。

图17给出四组完整轨迹。同向平行组保持各车间的相对位置不变，三车随201车共同转弯；相对加减速组在场地X方向增加往复相对运动，非平行组在场地Y方向增加往复相对运动，转弯变队形组在201车转弯时调整队形方向。这些运动方式可能同时改变相对速度、加速度和方向，不将它们视为彼此完全独立的因素。四组最高车速为2.52米/秒，最大车间距为27.72米。

'''
    s+=fig('不同机动方式的三车轨迹','review_不同类型机动轨迹')
    s+=tab('不同机动方式的定位结果',['机动方式','最大相对速度（米/秒）','最大相对加速度（米/秒²）','最大方向差（度）','位置RMSE范围（米）'],[
        [c['name'],f'{c["max_relative_speed"]:.3f}',f'{c["max_relative_acceleration"]:.3f}',f'{c["max_velocity_angle_deg"]:.1f}',f'{c["rmse_min"]:.3f}～{c["rmse_max"]:.3f}'] for c in maneuver_types])
    s+=fig('不同机动方式的运动量与位置误差','review_不同类型机动对照')
    s+='''图18上方为20次重复的平均位置误差和最大车间相对速度，下方为最大车间相对加速度和201车与其他车辆的最大运动方向差。方向差只在对应两车速度均不低于0.3米/秒时统计，以免车辆接近静止时方向角失去意义。表20的运动量取80～300秒最大值；位置RMSE按60～300秒计算，范围包括各组全部20次结果。

四组位置RMSE均值依次为0.301、0.308、0.302和0.302米，全部80次运行的RMSE为0.115～0.610米。相同起步过程使四组在前段的误差过程一致，之后曲线有交叉；在本次设定下，不同运动方式的平均RMSE接近，不能据此认为某一种机动总能提高精度，也不能仅凭图中峰值判断由哪一种机动引起。两组往复运动的位移幅度和周期相同，因此对应的最大相对速度和加速度曲线重合，方向差则随运动方向不同而变化。

本次按既定VIO位移误差模型生成运动输入；实车比较还需结合地面纹理、转弯振动和图像模糊对运动估计的影响，在实际场地分别采集平行行驶、变速、转弯和方向变化的数据。

'''
    s+='''### 3.4 锚点位置的影响

车辆运动路线确定后，锚点位置也会改变车锚距离及其变化。选取区域中部、起点侧方、区域左侧、区域右侧及区域角部五个候选位置，每次仅选其中一个。三车轨迹、噪声样本及输入参数保持一致，每个位置按六条测距重新求初始关系并连续更新，9档输入各重复20次，共900次布点运行。

'''
    s+=fig('三车轨迹与候选布点','review_layout_候选布点')
    s+=tab('候选位置与测距几何',['候选位置','场景坐标（米）','车锚距离范围（米）','5秒方向变化中位数（度）'],[
        [p,f'（{v[0]:g}，{v[1]:g}）',f'{summary["placement_geometry"][p]["minimum_range_m"]:.2f}～{summary["placement_geometry"][p]["maximum_range_m"]:.2f}',f'{summary["placement_geometry"][p]["median_five_second_bearing_span_deg"]:.2f}'] for p,v in PLACEMENTS.items()])
    s+='''表中方向变化指车辆相对锚点的视线角在连续5秒内的跨度，对评价时段及三车取中位数。布点比较同时看距离、方向变化、起点几何和整段运动；候选车间距保持在24.56米以内，车锚距离按表中条件计算。

'''
    s+=tab('不同布点下增强输入的结果',['候选位置','IMU增强RMSE（米）','VIO增强RMSE（米）','LIO增强RMSE（米）'],[
        [p]+[span(cases[f'{kind}_增强_anchor_{p}']) for kind in ('imu','vio','lio')] for p in PLACEMENTS])
    s+=fig('不同布点的20次误差分布','review_layout_布点误差分布')+fig('单锚点辅助估计与真值轨迹','review_layout_单点估计轨迹')
    s+='''在本轮轨迹及等测距误差条件下，区域中部与右侧的VIO增强组20次RMSE均值分别为0.321米和0.240米，LIO增强组分别为0.163米和0.132米；各次结果范围见表22。IMU输入下各位置的结果仍受惯性传播误差影响。起点侧方的轨迹最近处仅约0.24米，实车布设需按安全距离和天线安装位置调整，并结合行驶路线检查车锚距离和遮挡。

'''
    s+='''### 3.5 测距精度与RTE对照

测距精度对比保持中部单锚点和运动输入不变，改变逐时测距标准差，标定残余固定偏差的标准差仍为3厘米。下表保留每个条件的全部20次结果，直接观察改善测距后位置误差的变化。

'''
    s+=tab('单锚点方案的测距精度对照',['运动输入','20厘米测距RMSE（米）','10厘米测距RMSE（米）','5厘米测距RMSE（米）'],[
        [label(p)]+[span(previous_cases[f'{p}_single_s{x:.2f}']) for x in SIGMAS] for p in PROFILES])
    s+=fig('测距与运动输入质量的影响','review_测距精度对照')
    s+='''图22按条件汇总20次重复仿真结果。横轴是本车运动输入质量，左、中、右三个图分别对应IMU、VIO和LIO；三条颜色线分别表示测距误差标准差20、10、5厘米。圆点是每组20次结果的中位数，竖线表示20次结果的最小值到最大值。它用来同时比较两件事：提高本车运动输入质量能降低多少误差，以及改善测距精度能降低多少误差。图中的数值属于表中设定条件下的仿真结果。

原RTE方法将本车运动与车间测距结合，计算车辆之间的相对位置和方向。下面选取低速初始化的一组无锚点独立仿真对照，使用三条车间测距和本车运动信息；使用10秒窗口、5米窗口内运动范围参数、无误差局部运动输入及2、5、10厘米测距随机误差，每档展示一次试验。仿真前30秒车辆静止，之后按同一轨迹运动。输出按201局部固定轴，与同一时刻的仿真真值比较。

'''
    s+=fig('RTE低速初始化后的逐车位置误差','review_RTE低速初始化误差')
    s+='''图23中，蓝线和橙线分别表示202、203车相对201的位置误差；灰色区域表示尚未输出有效结果。三个子图的纵轴范围不同；第三幅中的局部放大图展示90～300秒、0～3米范围内的逐车误差，颜色与主图一致。三组使用同一条运动轨迹，车辆起步后的运动变化达到本次仿真设定条件后，算法开始检查初始相对位置。2厘米和5厘米组在36.00秒通过检查并开始输出，首次输出时两车误差均小于0.4米；整段有效结果的RMSE分别为0.068米和0.173米。10厘米组的首次候选结果有一项位置不确定度检查未通过，下一次检查通过后于36.24秒开始输出。

10厘米组在36.24秒首次输出时，202车误差为28.90米，203车为1.35米。此时202车的估计相对位置约为（−12.86，7.67）米，真值约为（15.04，0.14）米；三车估计队形与真值的方向相反，出现了错误的镜像位置关系。虽然选错了位置，首次输出时的测距拟合残差仍约为0.088米。位置不确定度检查主要反映当前候选解附近的稳定程度，距离拟合较好且局部检查通过时，仍可能存在另一种与测距接近的位置关系。

随后两车的位置解随运动和测距更新发生变化：203车误差在53.22秒升至23.23米，在61.92秒因位置解跳变降至0.63米；202车在84秒前后经过多次大幅调整，到85.02秒两车误差分别降至0.67米和0.74米。10厘米组恢复过程中累计有312条测距未被采用。图中前段的大幅起落反映了该次试验的初始选解错误和后续位置跳变；后段是位置接近真值后的误差变化。10厘米组整段有效结果的RMSE为7.901米，其中包含前段大误差。后续可保留并比较多组可能的初始位置关系，结合后续测距、运动信息以及相机或雷达对其他车辆的相对方位观测，研究初始解选择与错误解恢复方法。

'''
    rte_rows=[next(r for r in startup if r['name']==f'w10_d5_s{x:.2f}') for x in (.02,.05,.10)]
    s+=tab('RTE低速初始化对照',['测距误差标准差（厘米）','首次有效输出（秒）','有效输出相对位置RMSE（米）'],[
        [int(r['range_sigma_m']*100),f'{r["first_valid_s"]:.2f}',f'{r["metrics"]["position_rmse_m"]:.3f}'] for r in rte_rows])
    s+='''## 4 实测数据与接口应用

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

默认UWB输出话题为/cooperative_localization/poses_2d，坐标系为coop_ref_1_relative，采用瞬时队形坐标定义。RTE原始话题/cooperative_localization/rte/poses_2d的坐标标识为coop_rte_1_local，以201当前位置为原点、201本车初始局部轴为固定轴。启用坐标转换后，/cooperative_localization/rte/coordinates/poses_2d以201当前位置为原点，X轴沿首次有效结果中201指向202的方向并保持固定，坐标标识为coop_rte_1_relative_fixed。下表给出定位输入及输出，接收方按车辆编号、对应坐标系、测量时间与有效性使用位置。

'''
    h,rows=old_table(14);s+=tab('定位输入与输出',h,rows)
    s+='''运动融合仿真另提供201固定轴位置CSV，字段包括仿真时间、参考节点编号、车辆编号、相对位置x/y及位置有效性；原点随201平移，轴向与原RTE的coop_rte_1_local一致，输出继续采用201固定轴。配套文件给出重复试验1的代表性离线位置，全部重复逐时结果保留在NPZ文件中。该CSV轴向与RTE原始话题一致，转换话题采用上文的初始201至202方向轴；默认UWB及原RTE在线话题保持现状。

## 5 实车采集与联调

实车使用同一个已知位置UWB锚点，记录锚点的天线坐标及安装高度，同时采集三条车间测距和三条车锚测距。车速保持2～3米/秒以内，最大车间距20～30米。沿起步、转弯及编队整体移动路线采集，分别记录无锚点和有单锚点的输入，按本轮位置评价方法比较配置收益。

IMU通过静止窗口估计启动角速度均值和重力方向，加计开展多姿态标定；在安装后的温度与振动条件下检查偏置变化。相机、雷达与IMU完成时间和安装位姿标定后，在实际室外视场中输出本车运动，并按尺度、方向漂移和随机位移统计对应本报告的输入条件。

运动试验配套独立位置测量，例如载波相位差分定位（Real-Time Kinematic，RTK）或全站仪。定位侧按统一车辆编号输出相对位置和测量时间，通信、规划与控制沿用现有消息接口接入；需要地图坐标时，将实测方向关系与任务局部系结合换算。采集命令、启动顺序和时间检查继续使用21号操作文档。

## 6 结论

三盒采集、默认UWB相对队形、独立RTE库和消息接口已形成可运行链路。六组静止数据支持输入分析及位置回放，本轮仿真补充了三类运动输入、三档质量、三档测距误差与五个单锚点候选位置的量化对比。参数、统计、误差过程及轨迹按同一条件相邻列示。

'''
    s+=f'在测距白噪声标准差10厘米、标定残余固定偏差标准差3厘米的条件下，默认纯UWB队形的20次RMSE为{span(previous_cases["uwb_s0.10"])}米。固定轴向定位采用LIO增强输入、无锚点时为{span(cases["lio_增强_no_anchor_peer"])}米；增加中部单锚点后为{span(cases["lio_增强_anchor_区域中部"])}米，改为区域右侧单锚点后为{span(cases["lio_增强_anchor_区域右侧"])}米。上述结果分别按规定坐标和明确的仿真输入计算。\n\n'
    s+='''配置上保留默认UWB满足相对队形使用，连续运动定位优先采用现有相机/雷达与IMU组合，并以一个已知位置UWB锚点提供共同距离参考，兼顾设备复用与配置投入。后续按低速编队和同一评价口径采集实车动态数据，完成位置输出与通信、规划和控制的联调。
'''
    s=s.replace('RTE固定相对位置输出','RTE转换后相对位置输出')
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
    chosen=[dict(case,source_directory=str(COMMON.relative_to(ROOT))) for case in current['conditions']]
    assert len(chosen)==84 and sum(len(c['trials']) for c in chosen)==1680
    index=[]
    for c in chosen:
        a=arr(c);index.append(dict(name=c['name'],profile=c['profile'],mode=c['mode'],source_directory=c['source_directory'],
            output_file=c['source_directory']+'/outputs/'+c['name']+'.npz',trials=len(a),mean_rmse_m=float(a.mean()),sample_std_rmse_m=float(a.std(ddof=1)),minimum_rmse_m=float(a.min()),maximum_rmse_m=float(a.max())))
    (DEST/'本版实验条件清单.json').write_text(json.dumps(dict(conditions=84,trials=1680,rte_diagnostic_runs=12,fusion_diagnostic_runs=20,conditions_index=index),ensure_ascii=False,indent=2)+'\n')
    return s


def supporting():
    text='# V1.4阅读与复现\n\n本目录同名Markdown是正式报告源稿，Word和PDF由其排版。手改源稿后执行下方“只更新格式”命令；build_v14_report.py默认也保留源稿，只有--refresh-content会用生成器重建正文。\n\n## 当前数据与方法\n\n- 正文84个条件、每条件20次，共1680次噪声重复；本版实验条件清单.json列出逐时输出和统计。三类输入各三档，无锚点18条件、五个单锚点位置45条件、额外5/20厘米测距18条件、默认UWB三条件。\n- 默认UWB保持原有瞬时队形坐标。无锚点、单锚点运动融合及RTE均按201当前位置为原点，轴平行201自身起点局部里程计轴，保持固定。锚点提供额外距离，输出继续采用201固定轴。\n- data/single_anchor_outdoor_20261029保存运动与测距输入；data/single_anchor_layout_20261029保存测量驱动初始化和原始融合输出；两目录全部保留。\n- data/common_reference_20261030保存统一201轴的84条件输出、summary.json及全部仿真结果.csv。转换使用初始化估计的201局部转角和201同刻估计位置，真值按同一坐标用于评价；原始条件选择.json固定输入清单，避免后续编辑报告影响复算。pose_exports给出六组代表性重复试验1的离线位置CSV。\n- 分级运动仿真运行独立Python扩展卡尔曼滤波：IMU状态15维（位置、速度、转角），VIO/LIO状态9维（位置、转角）。本轮IMU输入采用三档残余和游走参数，算法状态为位置、速度和局部转角；VIO/LIO采用位移输出层模型。\n- data/fusion_peak_analysis_20261030保存四档输入的原始重放、去方向漂移、去测距随机项、方向过程方差扩大和90秒建系共20次诊断，复用重复试验1。方向/形状分解用于给出峰值来源，主曲线和精度统计按原始输出计算。\n- data/rte_startup_diagnostics_20261030保存三档测距与四种建系配置的12组结果和拒绝原因。建系参数对照.csv同时列有效段RMSE及后段RMSE，后段没有有效输出时留空。data/maneuver_observability_20261101保存低、当前、高三种相对机动条件的20次重复结果、几何敏感度和最终RMSE。\n\n## 从输入重算\n\n在工程根目录执行；已保存原始输出时可从第三条命令开始。完整重跑会更新对应数据目录，请保留原始归档后进行。\n\n```bash\nOPENBLAS_NUM_THREADS=1 python3 tools/evaluate_single_anchor.py --trials 20\nOPENBLAS_NUM_THREADS=1 python3 tools/evaluate_anchor_layout.py\nOPENBLAS_NUM_THREADS=1 python3 tools/evaluate_common_reference.py\nOPENBLAS_NUM_THREADS=1 python3 tools/analyze_fusion_peaks.py\nOPENBLAS_NUM_THREADS=1 python3 tools/analyze_fusion_peaks.py --profiles vio_精细 --output data/fusion_peak_vio_precise_20261102\nOPENBLAS_NUM_THREADS=1 python3 tools/evaluate_rte_startup_diagnostics.py\nOPENBLAS_NUM_THREADS=1 python3 tools/analyze_maneuver_observability.py\nOPENBLAS_NUM_THREADS=1 python3 tools/plot_fusion_review.py\npython3 tools/check_report_consistency.py\npython3 tools/check_report_results.py\npython3 tools/check_fusion_review.py\n```\n\nRTE诊断重编译需要g++、Eigen和既有build_rte_verify库；--reuse-outputs只重算已经完成的12次输出。旧RTE图13与主盒点云由plot_report_replay.py生成，运行前加载ROS环境；图13保持原始三个出线时刻127.20、215.52、216.24秒。\n\n## 只更新Word和PDF\n\n```bash\nPYTHONPATH=/tmp/cooperative_report_deps python3 tools/render_formal_report.py\npython3 tools/export_formal_pdf.py\n```\n\n若需要按生成器重新组织全文，再使用build_v14_report.py --refresh-content；该命令会重建Markdown，不用于保护手改内容的排版操作。\n\n## 图表来源与编辑\n\n| 图表 | 真实输入、计算和文件 | 重算入口 |\n|---|---|---|\n| 图1 | figures/图1_协同定位系统组成.png原样嵌入，SHA256记录在原图保留清单 | 不重画、不重新编码 |\n| 分级、单锚点、布点及测距精度图 | common_reference_20261030的逐时输出与真值、summary.json | plot_fusion_review.py |\n| 201参考坐标图 | 已保存仿真轨迹0秒和120秒、同一个锚点 | plot_fusion_review.py，坐标示意用于说明原点、轴向和锚点位置 |\n| 峰值分析和受控重放 | fusion_peak_analysis_20261030的实际EKF重放，输出减真值；方向/形状分解只用于归因 | analyze_fusion_peaks.py及plot_fusion_review.py |\n| RTE参数对照 | rte_startup_diagnostics_20261030中12组结果和逐次状态 | evaluate_rte_startup_diagnostics.py及plot_fusion_review.py |\n| 相对机动对照 | maneuver_observability_20261101中三种相对机动、几何敏感度和最终RMSE | analyze_maneuver_observability.py |\n| 原RTE逐车误差 | rte_expanded_20261025的.csv、.truth.csv与原C++输出 | source /opt/ros/humble/setup.bash后python3 tools/plot_report_replay.py --replay-rte |\n| 实测测距和相对队形 | rte_accuracy_20261023/uwb_baseline_20261017中的测距及位置记录 | plot_sensor_anchor_v14.py的measured_plots |\n| 三目图像和点云 | 三目实际采集画面；201盒/livox/lidar的225条原始消息，固定抽样12万点 | plot_report_replay.py及复现核对.json |\n\n新增数据图均为Matplotlib读取实际运行结果绘制，文字使用真实字体，保存高清PNG、保留文字节点的SVG、矢量PDF。来源和折线哈希见融合复核图线来源.json。SVG可用Inkscape选择对象，按T编辑文字，移动和缩放后另存；当前用户PNG图1不作为可逐对象编辑的SVG提供，旧六框SVG不对应该原图。\n\n参数依据、参考实现与可观性核对保存在参数来源_可观性_初始化核对.md和融合方法与现象分析_意见落实核对.md；外部项目与采购核价资料保留内部。第2.5节的表格比较三车传感器盒与逐车升级高配IMU的配置及费用；单锚点在第3章介绍，按另增一只同型完整盒计算。仿真输入档位与硬件配置分别说明。\n\n历次意见、版本和逐次具体改动分别见docs/报告编制与对外表述要求_持续记录.md、docs/报告内部版本记录.md、docs/报告改动前后对照记录.md。\n'
    text=text.replace('分级运动仿真运行独立Python扩展卡尔曼滤波：IMU状态15维（位置、速度、转角），VIO/LIO状态9维（位置、转角）。本轮IMU输入采用三档残余和游走参数，算法状态为位置、速度和局部转角；VIO/LIO采用位移输出层模型。','分级运动仿真按扩展卡尔曼滤波的状态和测量更新过程计算：IMU状态包括位置、速度和局部转角，VIO/LIO状态包括位置和局部转角。本轮IMU输入采用三档残余和游走参数；VIO/LIO采用包含滩涂、沙地和训练场退化因素的位移输出条件。')
    text=text.replace('data/rte_startup_diagnostics_20261030保存三档测距与四种建系配置的12组结果和拒绝原因。建系参数对照.csv同时列有效段RMSE及后段RMSE，后段没有有效输出时留空。','data/rte_startup_diagnostics_20261030保存三档测距与四种建系配置的12组结果和拒绝原因；其中10米仅作为当前RTE实现的初始化判据，不作为规划运动要求。建系参数对照.csv同时列有效段RMSE及后段RMSE，后段没有有效输出时留空。')
    text=text.replace('旧RTE图13与主盒点云由plot_report_replay.py生成，运行前加载ROS环境；图13保持原始三个出线时刻127.20、215.52、216.24秒。','正式报告图23采用10秒窗口、5米运动判据的三组RTE对照，读取data/rte_startup_diagnostics_20261030/w10_d5_s*_output.csv及对应真值，由plot_fusion_review.py绘图。较早一组10米判据的原始回放和全部配置对照仍保留在该数据目录。')
    text=text.replace('| 原RTE逐车误差 | rte_expanded_20261025的.csv、.truth.csv与原C++输出 | source /opt/ros/humble/setup.bash后python3 tools/plot_report_replay.py --replay-rte |','| RTE低速逐车误差（图23） | rte_startup_diagnostics_20261030/w10_d5_s*_output.csv及rte_expanded_20261025对应真值 | evaluate_rte_startup_diagnostics.py及plot_fusion_review.py |')
    text=text.replace('OPENBLAS_NUM_THREADS=1 python3 tools/analyze_maneuver_observability.py\n', 'OPENBLAS_NUM_THREADS=1 python3 tools/analyze_maneuver_observability.py\nOPENBLAS_NUM_THREADS=1 python3 tools/analyze_maneuver_types.py\n')
    text=text.replace('OPENBLAS_NUM_THREADS=1 python3 tools/analyze_maneuver_types.py\n', 'OPENBLAS_NUM_THREADS=1 python3 tools/analyze_maneuver_types.py\nOPENBLAS_NUM_THREADS=1 python3 tools/analyze_straight_maneuvers.py\n')
    text=text.replace('| 原RTE逐车误差 |', '| 直线运动对照 | straight_maneuvers_20261102中无锚点及单锚点的逐时输出和局部测距敏感度；6项和8项的定义见内部材料/直线运动局部测距分析补充说明.md | analyze_straight_maneuvers.py |\n| 原RTE逐车误差 |')
    text=text.replace('| 原RTE逐车误差 |', '| 不同机动方式对照 | maneuver_types_20261102中四种运动的20次逐时输出、真值和统计 | analyze_maneuver_types.py |\n| 原RTE逐车误差 |')
    text=text.replace('## 只更新Word和PDF', '## 仅重绘图例与图表\n\n使用已保存的结果重绘，不重新执行仿真：\n\n```bash\nPYTHONPATH=tools OPENBLAS_NUM_THREADS=1 python3 tools/plot_fusion_review.py\nPYTHONPATH=tools OPENBLAS_NUM_THREADS=1 python3 tools/analyze_maneuver_observability.py --plot-only\nPYTHONPATH=tools OPENBLAS_NUM_THREADS=1 python3 tools/analyze_maneuver_types.py --plot-only\nPYTHONPATH=tools OPENBLAS_NUM_THREADS=1 python3 tools/analyze_straight_maneuvers.py --plot-only\nPYTHONPATH=tools OPENBLAS_NUM_THREADS=1 python3 tools/plot_initialization_candidates.py\nPYTHONPATH=tools OPENBLAS_NUM_THREADS=1 python3 -c "import plot_sensor_anchor_v14 as p; p.measured_plots()"\n```\n\n## 只更新Word和PDF')
    text=text.replace('| 201参考坐标图 | 已保存仿真轨迹0秒和120秒、同一个锚点 | plot_fusion_review.py，坐标示意用于说明原点、轴向和锚点位置 |','| 201参考坐标图 | 已保存仿真轨迹0秒和120秒、同一个锚点 | plot_fusion_review.py，坐标示意用于说明原点、轴向和锚点位置 |\n| 初始化候选关系图 | 同一组带噪仿真测距及局部运动输入，展示实际初始化函数使用的候选初值 | plot_initialization_candidates.py |')
    text=text.replace('无锚点、单锚点运动融合及RTE均按201当前位置为原点，轴平行201自身起点局部里程计轴，保持固定。锚点提供额外距离，输出继续采用201固定轴。','传感器分级、锚点布置及RTE原始输出按201当前位置为原点、201本车初始局部轴为固定轴。RTE坐标转换话题以首次有效结果中的201至202方向为固定X轴。第3.3节机动试验也采用201固定轴评价。')
    text=text.replace('几何敏感度和最终RMSE','几何敏感度和60～300秒位置RMSE')
    (DEST/'阅读与复现说明.md').write_text(text)
    questions=['三类输入的误差参数分别是什么，哪部分来自假设、哪部分对应实际采集？','默认UWB的瞬时队形轴和固定轴向运动定位有何用途区别？','无锚点初始化是否使用了任何车锚距离？','相同输入下增加一个锚点，误差分别从多少变成多少？','为什么改善初始化初值覆盖，不能说成传感器精度提高？','五个候选点是否同时存在？每种位置的车锚距离是否都满足同样传播条件？','单锚点为什么不能直接给出地图朝向，采用的局部轴怎么定义？','为什么右侧在这条轨迹下优于中部，现场还需要检查什么？','84种条件怎样组成，每种20次实际改变什么？','一套是几只盒，三车及加单锚点各多少只，费用依据是什么？','RTE三组曲线为什么从不同时刻出线，灰色区间表示什么？','实测点云从哪个话题读取，累积和抽样方式是什么？','VIO/LIO模型试验如何在安装后的实车输入上复测？','SVG能修改哪些对象，PNG是否能直接变成逐对象可编辑原稿？','分级试验的EKF与原RTE滑动窗口分别估计什么，测距怎样参与？','静止初始化能确定哪些量，哪些残余参数是仿真假设？','位移随机误差为何按路程平方根积累，与IMU时间噪声有何区别？','VIO峰值的方向与形状分量分别是多少，去掉后续方向漂移是否解决峰值？','延长建系到90秒、放宽RTE运动条件是否改善结果，为什么不能只看出线快慢？']
    (DEST/'V1.4_报告理解核对问题.md').write_text('# 报告理解核对问题\n\n'+'\n'.join(f'{i+1}. {q}' for i,q in enumerate(questions))+'\n')
    for name in ['build_single_anchor_report.py','build_v14_report.py','evaluate_single_anchor.py','evaluate_anchor_layout.py','plot_anchor_layout.py','plot_report_replay.py','plot_initialization_candidates.py','analyze_rte_initialization.py','evaluate_common_reference.py','analyze_fusion_peaks.py','evaluate_rte_startup_diagnostics.py','plot_fusion_review.py','analyze_maneuver_observability.py','analyze_maneuver_types.py','analyze_straight_maneuvers.py','check_report_consistency.py','check_fusion_review.py','check_report_results.py','check_anchor_layout.py','test_anchor_layout.py','plot_single_anchor.py','plot_sensor_anchor_v14.py','check_single_anchor.py','test_single_anchor.py','audit_outdoor_assumptions.py','build_stage_report.py','render_formal_report.py','export_formal_pdf.py','report_plot_fonts.py']:
        shutil.copy2(ROOT/'tools'/name,DEST/'复现脚本'/name)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--refresh-content',action='store_true');args=p.parse_args()
    if args.refresh_content or not CURRENT.exists():CURRENT.write_text(generate())
    supporting();render_word(CURRENT.read_text(),CURRENT.with_suffix('.docx'),title='多平台协同定位硬件仿真测试报告与联调方案',header='')
    print('已排版：',CURRENT)


if __name__=='__main__':main()
