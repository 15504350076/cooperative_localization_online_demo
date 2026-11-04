#!/usr/bin/env python3
"""V1.4单锚点修订：按已运行的输出成文，默认保留手工Markdown。"""
import argparse
import hashlib
import json
import re
import shutil
from pathlib import Path

import numpy as np
from build_stage_report import render_word
from evaluate_single_anchor import OUT, ROOT, PROFILES

DEST=ROOT/'docs/报告/20261028_协同定位V1.4'
STEM='多平台协同定位硬件仿真测试报告与联调方案_V1.4_设备分级与锚点仿真版'
CURRENT=DEST/(STEM+'.md')
BEFORE=DEST/'内部材料/20261029_单锚点修订前'/CURRENT.name


def table(title,headers,rows):
    return title+'\n\n| '+' | '.join(headers)+' |\n|'+'|'.join(['---']*len(headers))+'|\n'+''.join('| '+' | '.join(map(str,row))+' |\n' for row in rows)+'\n'


def span(case):
    a=[r['position_rmse_m'] for r in case['trials']]
    return f'{min(a):.3f}～{max(a):.3f}'


def name(profile):
    return profile.replace('imu','IMU').replace('vio','VIO').replace('lio','LIO').replace('_',' ')


def generate():
    old=BEFORE.read_text();summary=json.loads((OUT/'summary.json').read_text());budget=json.loads((OUT/'成本预算.json').read_text());cases={c['name']:c for c in summary['conditions']}
    assert len(cases)==84 and all(len(c['trials'])==20 for c in cases.values())
    assert summary['anchor_count']==1 and not summary['truth_used_by_estimator'] and not summary['post_alignment']
    def section(begin,end):return old.split(begin,1)[1].split(end,1)[0]
    s=old.split('### 2.2 ',1)[0]
    s=s.replace('保留18组既有基线，新增70种条件、1400次传感器与锚点仿真','保留既有RTE基线，完成84种条件、1680次单锚点与传感器分级仿真')
    s=s.replace('比较惯性与视觉/雷达运动输入及固定锚点组合','比较惯性、视觉/雷达运动输入与唯一UWB锚点')
    s=re.sub(r'本轮还引入固定场地锚点。.*?场景和锚点位置见图16。',
        '本轮只使用一个位置已知、配备UWB的锚点，可以是地标上的UWB设备，也可以是已落地并保持静止的无人机。三车之间有三条测距，三车到同一锚点另有三条测距，共六条链路。单锚点坐标定义和布置见第3.3节及图16。',s)
    s+='''### 2.2 单锚点与传感器分级仿真

本轮把设备类别、运动输入精度、测距精度和约束增加过程分别比较。传感器包括IMU、视觉惯性里程计（Visual-Inertial Odometry，VIO）和激光惯性里程计（LiDAR-Inertial Odometry，LIO）。VIO由相机与IMU提供局部运动，LIO由雷达与IMU提供局部运动，因此两类方案均包含IMU。

试验在开发电脑实际执行仿真程序。模拟器生成含误差的加速度、角速度、局部位移和UWB距离；初始化及位置估计程序读取这些测量计算结果。VIO/LIO在本轮采用位移输出模型，参数直接作用于局部运动增量；相机图像、雷达点云的特征提取与前端处理在实车接入时开展。

每类运动输入设基础、增强、精细三档；各档分别搭配10、5、2厘米测距噪声，并比较三种后续处理。九种运动输入、三档测距、三种处理共81种条件；另用默认UWB库计算三档纯测距队形，共84种条件。每种条件20次，合计1680次。各方案共享相同编号的测距扰动；同一档位的三种处理使用完全相同的传感器输入与初始化结果。

'''
    rows=[]
    for profile,cfg in PROFILES.items():
        if cfg['kind']=='imu':params=f"残余加速度零偏{cfg['accel_bias']:.3f}米/秒²；角速度零偏{cfg['gyro_bias']:.6f}弧度/秒；加速度白噪声{cfg['accel_white']:.3f}米/秒²"
        else:params=f"尺度标准差{cfg['scale_std']*100:g}%；航向漂移标准差{cfg['yaw_drift_std']:g}弧度/秒；位移随机项{cfg['displacement_noise']:g}米/√米"
        rows.append([name(profile),params])
    s+=table('表101 运动输入精度设定',['运动输入及档位','仿真误差参数'],rows)
    s+='''IMU参数对应静止初始化后的残余误差，加速度与角速度白噪声按100赫兹采集、10赫兹平均量设置；角速度白噪声标准差统一为0.0005弧度/秒。VIO/LIO的尺度与航向慢漂移在每次运行中独立生成，位移随机项按单步行驶距离的平方根增长。档位用于比较输入质量，不与某一硬件型号直接等同。VIO精细档与LIO增强档采用相同位移输出误差参数，因此对应结果相同；同等输入质量时，可结合视场、算力和成本选择前端。

三车采用300秒轨迹，前30秒静止，随后在约85×115米范围内运动；最高车速约2.39米/秒，最大车间距约24.56米。唯一锚点放在运动区域中部，场景坐标为（55，75）米，最大车锚距离约66.16米。全部UWB链路连续生成，每条10赫兹，共每秒60条测距，测距误差为零均值独立噪声。仿真采用平面视距工况，三车车载参考点与UWB天线在模型中重合，锚点和车载天线按同高平面处理。

初始化使用0～60秒的六条测距及局部运动，不预先给定各车位置与朝向。程序同时求三车起点位置和各自局部坐标转角，用测距残差选择初始解；真值不参与这一过程。60秒之后才计入本轮位置误差。三个对照分支从同一初始状态继续运行，见表102。

'''
    s+=table('表102 逐项增加约束的比较',['处理方式','0～60秒','60～300秒'],[
        ['起点结果＋运动传播','六条测距与本车运动共同初始化','仅使用本车运动输入'],
        ['继续使用车间UWB','使用相同初始化结果','本车运动＋三条车间测距'],
        ['继续使用车间UWB＋单锚点','使用相同初始化结果','本车运动＋三条车间测距＋三条到同一锚点的测距'],
        ['默认纯UWB队形','读取三条车间测距','逐时调用现有C++三角形求解库'],
    ])
    s+='''位置均方根误差（Root Mean Square Error，RMSE）按202、203车相对201车的位置向量与同一时刻真值之差合并计算。表中范围为20次运行RMSE的最小值至最大值。单锚点组使用固定局部坐标轴，纯UWB使用原规范队形坐标，分别与各自坐标的真值比较。真值只用于模拟传感器与事后评价，输出没有经过真值轨迹配准或事后平滑。

'''
    s+='### 2.3 既有RTE基线仿真'+section('### 2.3 既有RTE基线仿真','### 2.4 已有Gazebo物理仿真载体')
    s+='### 2.4 已有Gazebo物理仿真载体'+section('### 2.4 已有Gazebo物理仿真载体','### 2.5 既有大运动范围对照')
    s+='### 2.5 既有大运动范围对照'+section('### 2.5 既有大运动范围对照','### 2.6 起点标定与固定锚点对照')
    s+='''### 2.6 单锚点逐级对比结果

先固定测距噪声标准差为5厘米，分别观察同一输入在起点传播、加入车间UWB、继续加入唯一锚点后的结果。表103逐项列出，图17～图19给出20次重复试验的分布。

'''
    s+=table('表103 5厘米测距下的逐项比较',['运动输入','起点结果传播（米）','加车间UWB（米）','再加唯一锚点（米）'],[[name(p)]+[span(cases[f'{p}_{m}_s0.05']) for m in ('start','peer','single')] for p in PROFILES])
    for kind in ('imu','vio','lio'):
        p=kind+'_增强';s+=f'{kind.upper()}增强档的相对位置RMSE由仅运动传播的{span(cases[p+"_start_s0.05"])}米，加入车间UWB后为{span(cases[p+"_peer_s0.05"])}米，再持续使用单锚点后为{span(cases[p+"_single_s0.05"])}米。\n\n'
    s+='''在保持唯一锚点和三条车间测距的条件下，再单独改变测距精度，见表104及图20。表中的设备档位沿用表101；横向比较测距精度，纵向比较运动输入条件。

'''
    s+=table('表104 单锚点下不同输入与测距精度的结果',['运动输入','10厘米测距（米）','5厘米测距（米）','2厘米测距（米）'],[[name(p)]+[span(cases[f'{p}_single_s{v:.2f}']) for v in (.10,.05,.02)] for p in PROFILES])
    s+=table('表105 默认纯UWB队形对照',['测距标准差（厘米）','重复次数','相对位置RMSE范围（米）'],[[int(v*100),20,span(cases[f'uwb_s{v:.2f}'])] for v in (.10,.05,.02)])
    s+='''IMU由加速度二次积分得到位置，残余偏置会持续积累；车间测距主要维持队形形状，增加单锚点后还可利用编队相对已知点的距离变化修正状态。VIO/LIO直接提供局部位移，在同一融合方式下比较不同运动输入精度。图21、图22展示逐时与逐车位置误差，图23展示实际算法输出与真值轨迹，图24汇总全部重复运行的误差分布。

### 2.7 设备组合与费用测算

费用按三车整套新建配置测算，分别列出硬件、一次性算法开发、标定部署与联调，单位为人民币万元。每种运动组合均包含三车UWB及一个UWB锚点；VIO包含相机、IMU和计算单元，LIO包含雷达、IMU和计算单元。默认纯UWB一行采用三车基础设备。单个锚点的供电、防护、安装与测量预算约0.3～0.8万元，已计入相应硬件预算；采用已有落地无人机承载时不另计飞行平台采购。

'''
    rows=[]
    for b in budget['combinations']:
        cells=[name(b['profile']) if b['profile']!='uwb' else '默认纯UWB']
        cells += [f'{b[k][0]:g}～{b[k][1]:g}' for k in ('hardware_wan','algorithm_wan','integration_wan','total_wan')]
        rows.append(cells)
    s+=table('表106 三车方案预算',['组合','硬件（万元）','算法（万元）','联调（万元）','总计（万元）'],rows)
    s+='''算法费按三车项目一次性计取，包括定位初始化、运动融合、VIO/LIO前端接入、消息适配、测试与复现工具；联调费包括安装标定、数据处理和实车调试。预算按约1500～2000元/人日的综合研发成本估计，不按单车重复收取整套研发费。现有设备可复用时，按新增硬件和新增功能重新核算。

上述金额是项目预算测算，采购型号及正式报价确定后再细化。相机量级可参考[Stereolabs ZED 2i官方商店](https://www.stereolabs.com/store/products/zed-2i)的499美元起价；雷达类别可参考[Livox MID-360官方资料](https://www.livoxtech.com/mid-360)。单件公开售价只用于核对硬件量级，三车总预算还包含计算、供电防护、算法及联调。图26把预算区间与相应仿真输入条件放在同一图中，便于按任务需要选配。

'''
    s+='## 3 接口与数据说明'+section('## 3 接口与数据说明','## 4 启动与复现')
    s=s.replace('| 锚点场地坐标 | 场地测量标记O为原点，沿场地方向标记为正X，Z向上；三车均可移动 | 输出场地位置，并按同一轴向求各车相对201的位置 |',
        '| 单锚点局部坐标 | 唯一锚点为原点，正X沿起点锚点至201车方向，Y按右手规则，运动后轴向固定 | 输出相对锚点位置，并减去201车位置获得固定轴向相对位置 |')
    s=re.sub(r'本轮锚点仿真结果写入独立CSV和NPZ。.*?航向有效位按位置试验置为false。',
        '本轮单锚点结果由独立离线程序导出。原点是唯一UWB锚点，X轴取初始化起点时锚点指向201车的方向，Y轴为平面内逆时针90度方向，Z轴向上。运动中的局部里程计和测距共同维持这套固定坐标轴；单点的已知位置提供原点参考，地图方向需结合方向测量换算。CSV中的x_m、y_m为各车减去201车后的相对位置，anchor_x_m、anchor_y_m为相对固定锚点的位置；坐标系分别标为coop_single_anchor_relative和coop_single_anchor。CSV仅导出位置，航向有效标志为false，默认UWB与RTE运行话题继续采用原配置。',s)
    s+='## 4 启动与复现'+section('## 4 启动与复现','## 5 仿真复算与数据检查')
    s+='''## 5 仿真复算与数据检查

### 5.1 单锚点仿真复现

在工程根目录执行以下命令，先生成测量，再运行初始化和位置滤波，最后从保存的输出绘图。运行日志逐项输出完成的条件和20次误差范围。原始输入、每次完整位置输出和统计表位于data/single_anchor_20261029。

```bash
PYTHONPATH=tools OPENBLAS_NUM_THREADS=1 python3 -u tools/evaluate_single_anchor.py --trials 20
PYTHONPATH=tools python3 tools/check_single_anchor.py
PYTHONPATH=tools python3 tools/plot_single_anchor.py
```

查看已有结果时直接打开summary.json与全部仿真结果.csv。inputs保存实际使用的IMU或局部位移输出及初始化结果，outputs保存每种条件20次完整位置输出，truth_and_range_inputs.npz保存独立真值与六条测距的生成样本。绘图程序只读取这些文件，图中横轴、纵轴和单位由代码中的文字对象排版，配套SVG和矢量PDF可选择、复制文字。

'''
    for number in (11,12,13):
        s+=re.search(r'^!\[图'+str(number)+r' .+\]\(.+\)$',old,re.M)[0]+'\n\n'
    s+='### 5.2 RTE跳变与图14的含义'+section('### 5.2 RTE跳变与图14的含义','### 5.3 录包分析与运行排查')
    s=s.replace('固定锚点对照增加场地位置约束，在连续运动传播的基础上更新位置，其逐时效果见图20。','本轮单锚点对照采用运动传播和逐次测距更新，逐时位置误差见图21，位置更新幅度与测距残差见图25。')
    for number in (14,15):
        s+=re.search(r'^!\[图'+str(number)+r' .+\]\(.+\)$',old,re.M)[0]+'\n\n'
    s+='### 5.3 录包分析与运行排查'+section('### 5.3 录包分析与运行排查','### 5.4 设备组合与锚点试验图线')
    s+='''### 5.4 单锚点运行图线

图16由本轮轨迹输入绘制，只标出一个锚点；虚线是初始时刻三车至同一点的三条测距关系。

![图16 三车运动与唯一UWB锚点](figures/single_16_三车与唯一锚点.png)

图17～图19逐类比较三档运动输入。每个箱线图包含20次运行，箱体为中间50%的结果、中线为中位数，须线范围外的点单独绘出；纵轴采用对数刻度，以便同图读取米级和厘米级误差。

![图17 IMU逐级对比](figures/single_17_IMU逐级对比.png)

![图18 VIO逐级对比](figures/single_18_VIO逐级对比.png)

![图19 LIO逐级对比](figures/single_19_LIO逐级对比.png)

图20每一点表示20次RMSE的中位数，误差棒为最小至最大范围。各曲线都使用唯一锚点和相同六条链路，改变的只是运动输入档位与测距噪声。

![图20 运动输入与测距精度](figures/single_20_传感器与测距精度.png)

图21～图23固定使用重复试验1。0～60秒用于初始化，位置曲线从60秒开始；没有按效果选择试验、裁去跳变或对估计轨迹进行真值配准。图24使用20次全部有效历元，横轴是逐车相对位置误差，纵轴是累计样本比例。

![图21 逐项增加条件后的逐时误差](figures/single_21_逐时定位误差.png)

![图22 单锚点逐车位置误差](figures/single_22_单锚点逐车误差.png)

![图23 算法输出与真值轨迹](figures/single_23_算法输出与真值轨迹.png)

![图24 位置误差累计分布](figures/single_24_位置误差累计分布.png)

图25的测距残差为三条车间距离和三条车锚距离的均方根；位置更新幅度为本历元六个位置分量更新前后差值的长度。曲线局部波动可与测量更新时刻对照，位置误差仍由同一时刻真值独立计算。图26横向预算区间对应表106，纵向范围对应20次RMSE，不把价格与精度作线性拟合。

![图25 位置误差与测量更新](figures/single_25_位置与测量更新.png)

![图26 三车预算与仿真条件](figures/single_26_预算与仿真精度.png)

'''
    s+='## 6 阶段结论与实车试验'+section('## 6 阶段结论与实车试验','### 6.4 复现资料清单')
    s=re.sub(r'首轮按三锚点方案布点，.*?再用同一段运动数据比较IMU和视觉/雷达运动输入。',
        '首轮只布置一个已知位置的UWB点，使用固定地标或已落地并保持静止的无人机承载。记录该点坐标及天线高度，三车分别与它测距，同时保留原有三条车间测距。按同一段低速数据比较IMU、VIO、LIO输入和六条测距的融合结果。',s)
    s+='''### 6.4 复现资料清单

表107 本轮运行资料

| 内容 | 文件 | 用途 |
|---|---|---|
| 全部试验 | data/single_anchor_20261029/summary.json、全部仿真结果.csv | 84种条件、1680次逐次结果 |
| 实际输入 | truth_and_range_inputs.npz、inputs/ | 真值、六条距离及本车测量输入 |
| 实际输出 | outputs/ | 每组20次完整位置、坐标转角和更新幅度 |
| 运行与核对 | 运行日志.txt、运行清单.json、独立结果核对.json | 命令、版本、哈希、逐组RMSE复算 |
| 费用测算 | 成本预算.json | 硬件、算法和联调预算及来源 |
| 位置导出与配图 | 单锚点位置输出.csv、单锚点图线来源.json、figures/ | 读取坐标、核对数据与矢量文字 |

### 6.5 阶段结论

现阶段已形成三盒采集、默认UWB队形定位、独立RTE库、消息接口和仿真复现工具。六组静止实测用于采集、回放和输入分析；本轮按一个已知位置UWB点完成84种条件、1680次分级仿真，输入、初始化、完整输出和逐次误差均已保存。

'''
    for kind in ('imu','vio','lio'):
        s+=f'5厘米测距下，{kind.upper()}增强档配合三条车间测距及单锚点的相对位置RMSE为{span(cases[kind+"_增强_single_s0.05"])}米。'
    s+='\n\n'
    s+=f'默认纯UWB在相同评价时段的三档测距结果为{span(cases["uwb_s0.02"])}、{span(cases["uwb_s0.05"])}、{span(cases["uwb_s0.10"])}米，继续用于现有相对队形输出。VIO/LIO输入在本轮对应的参数条件下给出亚米级相对位置结果；完整档位范围见表104。按所需坐标稳定性、运动输入条件和预算选择组合，实际设备复测后再确定实施配置。\n\n'
    s+='''单锚点仿真从测距与运动输入共同求起点关系，输出固定局部坐标中的连续位置；原默认UWB坐标及原RTE程序保持不变。后续以同一个UWB点开展低速实车采集，结合安装标定、前端处理和独立位置真值验证，将相对位置、坐标系和测量时间接入通信、规划与控制模块。
'''
    # 图1～图26保留连续编号；表格按正文首次出现统一重排，同时修改引用。
    ids=re.findall(r'^表(\d+) ',s,re.M);assert len(ids)==len(set(ids)),ids
    mapping={old:str(i+1) for i,old in enumerate(ids)}
    s=re.sub(r'表(\d+)',lambda m:'表'+mapping.get(m[1],m[1]),s)
    assert '3锚点' not in s and '4锚点' not in s and '三个固定锚点' not in s
    return s


def supporting():
    readme='''# V1.4单锚点修订：阅读与复现

正式稿仍为本目录同名Markdown、Word与PDF。锚点只指一个已知位置、配备UWB的地标或落地无人机；三车间三条距离，三车到同一点另三条距离。旧多基站设定留存在内部材料/20261029_单锚点修订前，不作为当前方案。

## 编辑与导出

手工改本目录正式报告Markdown后，在工程根目录执行：

```bash
PYTHONPATH=/tmp/cooperative_report_deps python3 tools/render_formal_report.py
python3 tools/export_formal_pdf.py
```

tools/build_v14_report.py和tools/build_single_anchor_report.py默认保留现有Markdown。只有显式使用--refresh-content才从内部留存稿与新结果重新组织正文。

## 实际运行与重绘

```bash
PYTHONPATH=tools OPENBLAS_NUM_THREADS=1 python3 -u tools/evaluate_single_anchor.py --trials 20
PYTHONPATH=tools python3 tools/check_single_anchor.py
PYTHONPATH=tools python3 tools/plot_single_anchor.py
```

第一条实际运行84种条件、1680次。第二条独立复算各组误差、重放估计与导出位置；第三条只读取已有输出绘制11幅新图，不重跑算法。数据在data/single_anchor_20261029。只改文字用前述两个导出命令；重新绘图后也用它们更新Word/PDF。

模拟器生成IMU测量与VIO/LIO局部位移输出；估计器只读含误差测量，先使用0～60秒求初始关系，再独立运行60～300秒，真值不进入估计器。默认UWB对照调用build_rte_verify/libzju_coop.so原库。VIO/LIO是位移输出级仿真，不是运行图像/点云前端；模型、参数和比较方式在第2.2节逐项说明。

启动初始化中的最小二乘依据实际测距与运动求状态。绘图直接使用算法输出与同一时刻真值计算误差，不按真值修饰输出或平滑曲线。每个条件的20次完整位置均在outputs/，共1680行汇总在全部仿真结果.csv。成本预算为三车项目分项估算，不是厂家报价；其来源和公式在成本预算.json。

## 图片中文字与来源

全部新图由tools/plot_single_anchor.py中的Matplotlib文字对象排版，中文是字体文字。每张图都有360 dpi PNG、保留文字节点的SVG、嵌入TrueType字体的矢量PDF。可打开figures/single_22_单锚点逐车误差.pdf，选择、复制文字；单锚点图线来源.json列出输入文件和送入绘图器的折线数据哈希。当前开发机中文为文泉驿微米黑、数字为Liberation Serif；报告正文字体设置仍为宋体、黑体和Times New Roman。没有调用生成式图像工具。

原图9仍来自data/rte_expanded_20261025中的三组exp_odometry_*_seed0_output.csv与同名.truth.csv。实际C++RTE输出坐标减去同时刻真值后求二维误差，其矢量文件为figures/16_RTE逐车定位误差.pdf。图1为可编辑框架，图4为坐标页面截图，图5由Gazebo SDF模型位置绘制；它们各自说明系统或场景，不冒充运行误差结果。

## 核对命令

```bash
PYTHONPATH=tools python3 -m unittest tools/test_single_anchor.py
PYTHONPATH=tools python3 tools/check_single_anchor.py
```

完整改动前后位置与原文在工程docs/报告改动前后对照记录.md；意见及原因持续记入docs/报告编制与对外表述要求_持续记录.md。
'''
    (DEST/'阅读与复现说明.md').write_text(readme)
    questions=['一个锚点装在哪里，六条测距分别是哪六条？','九种运动输入、三档测距、三种后续处理如何组成81种条件？','为什么纯UWB另有三种条件，1680次如何计数？','IMU、VIO、LIO的三档误差分别加在哪里？','初始化如何在不读取真值和预设车辆朝向的情况下求解？','单个已知点确定原点后，X轴如何约定？与地图方向怎样换算？','表中误差是相对位置还是固定锚点位置？统计时段是什么？','同一传感器从起点传播、增加车间UWB到增加单锚点改善多少？','图中的跳变对应算法什么更新，能否从输出复查？','VIO/LIO输入模型与运行真实前端有什么区别？','硬件费、算法费和联调费各包含什么，哪些设备可以复用？','怎样从日志、输入、输出与矢量文件复现一张曲线？']
    (DEST/'V1.4_报告理解核对问题.md').write_text('# V1.4报告理解核对问题\n\n'+'\n'.join(f'{i+1}. {q}' for i,q in enumerate(questions))+'\n')
    for p in [Path(__file__),ROOT/'tools/build_v14_report.py',ROOT/'tools/evaluate_single_anchor.py',ROOT/'tools/plot_single_anchor.py',ROOT/'tools/check_single_anchor.py',ROOT/'tools/test_single_anchor.py',ROOT/'tools/report_plot_fonts.py']:
        shutil.copy2(p,DEST/'复现脚本'/p.name)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--refresh-content',action='store_true');args=p.parse_args()
    if args.refresh_content or not CURRENT.exists():CURRENT.write_text(generate())
    supporting();render_word(CURRENT.read_text(),CURRENT.with_suffix('.docx'),title='多平台协同定位硬件仿真测试报告与联调方案',header='')
    print('V1.4单锚点报告已排版，源稿：',CURRENT)


if __name__=='__main__':main()
