#!/usr/bin/env python3
"""生成一册正式版阶段报告。

报告源稿保留在 docs/报告 目录；本脚本只整理版式和章节，不重跑算法评估。
"""
import hashlib
import json
import re
import shutil
from pathlib import Path

import numpy as np

from build_stage_report import ASSETS, DATA, DEST, ROOT, architecture, render_word


CURRENT = DEST / '多平台协同定位硬件仿真测试报告与联调方案_V1.3_正式报告版.md'
OLD_COMBINED = DEST / '多平台协同定位硬件仿真阶段报告与联调方案_V1.2_合册版.md'
V12_ARCHIVE = DEST / '历史版本/V1.2_分层版式版'
EXPANDED_SUMMARY = ROOT / 'data/rte_expanded_20261025/summary.json'


def _archive_v12():
    V12_ARCHIVE.mkdir(parents=True, exist_ok=True)
    for name in [
        '01_项目总览_V1.2_版本记录版',
        '02_执行细则_V1.2_接口验收版',
        '多平台协同定位硬件仿真阶段报告与联调方案_V1.2_合册版',
    ]:
        for suffix in ('.md', '.docx', '.pdf'):
            source = DEST / f'{name}{suffix}'
            target = V12_ARCHIVE / source.name
            if source.exists() and not target.exists():
                shutil.move(source, target)


def _source_text():
    if OLD_COMBINED.exists():
        return OLD_COMBINED.read_text()
    archived = V12_ARCHIVE / OLD_COMBINED.name
    if archived.exists():
        return archived.read_text()
    raise FileNotFoundError('缺少V1.2合册源稿，无法生成V1.3正式报告')


def _continuous_body(source):
    markers = [m.start() for m in re.finditer(r'<!-- pagebreak -->', source)]
    if len(markers) < 3:
        raise ValueError('V1.2源稿缺少封面、版本记录或目录分页标记')
    body = source[markers[2] + len('<!-- pagebreak -->'):].strip()

    # 早期分册中的组织/分工章节不属于当前技术报告正文，直接删除。
    body = re.sub(r'(?ms)^## A5\..*?(?=^## 第二部分)', '', body)
    body = re.sub(r'(?ms)^## 第二部分　执行细则\s*', '', body)
    body = re.sub(r'(?ms)^## B1\..*?(?=^## B2\.)', '', body)

    chapter_titles = {
        'A1': '项目概况与当前判断',
        'A2': '技术路线与系统关系',
        'A3': '已完成工作与证据',
        'A4': '阶段目标与验收门槛',
        'B2': '接口约定',
        'B3': '数据、时间与标定说明',
        'B4': '启动与复现',
        'B5': '测试项目与结果判读',
        'B6': '算法评估与结果说明',
        'B7': '异常记录与排查方法',
        'B8': '硬件在环实现方案',
        'B9': '指标与验收计算口径',
        'B10': '联调资料与问题记录',
    }
    sub_titles = {
        'B2.1': '现有话题、类型与QoS',
        'B2.2': '对外位置消息字段',
        'B2.3': '坐标系约定',
        'B2.4': '状态与时效说明',
        'B2.5': '非ROS程序接入',
        'B4.1': '单机UWB输入回放',
        'B4.2': 'RTE独立试验构建与启动',
    }

    lines = []
    for line in body.splitlines():
        if line.strip() == '## 第二部分　执行细则':
            continue
        sub_match = re.match(r'^###\s+(B\d+\.\d+)\s+(.*)$', line)
        chapter_match = re.match(r'^##\s+(A\d+|B\d+)\.\s*(.*)$', line)
        if sub_match:
            code, old_title = sub_match.groups()
            if code in sub_titles:
                number = {'B2.1': '5.1', 'B2.2': '5.2', 'B2.3': '5.3',
                          'B2.4': '5.4', 'B2.5': '5.5', 'B4.1': '7.1',
                          'B4.2': '7.2'}[code]
                line = f'### {number} {sub_titles[code]}'
        elif chapter_match:
            code, old_title = chapter_match.groups()
            if code in chapter_titles:
                number = {
                    'A1': '1', 'A2': '2', 'A3': '3', 'A4': '4',
                    'B2': '5', 'B3': '6', 'B4': '7', 'B5': '8',
                    'B6': '9', 'B7': '10', 'B8': '11', 'B9': '12', 'B10': '13',
                }[code]
                line = f'## {number} {chapter_titles[code]}'
        lines.append(line)
    body = '\n'.join(lines)

    # 将原分册内部交叉引用改为连续章节引用，避免当前报告继续出现A/B结构。
    references = [
        (r'B4\.1', '第7.1节'), (r'B4\.2', '第7.2节'),
        (r'B4～B9', '第7～12章'), (r'B10', '第13章'), (r'B9', '第12章'),
        (r'B8', '第11章'), (r'B7', '第10章'), (r'B6', '第9章'),
        (r'B5', '第8章'), (r'B4', '第7章'), (r'B3', '第6章'),
        (r'B2', '第5章'),
    ]
    for pattern, replacement in references:
        body = re.sub(r'(?<![A-Za-z0-9])' + pattern + r'(?![A-Za-z0-9])', replacement, body)
    body = body.replace('技术部分的测试用例', '第8章测试项目')
    body = body.replace('技术部分第8章', '第8章')
    body = body.replace('这使系统集成不必等待所有增强算法同时成熟：相关模块可以先验证车辆编号、相对几何、数据有效性及失效处理；定位侧同步推进动态真值试验、惯导约束和RTE正确初始化。',
                        '据此，系统已具备接口验证条件，并已形成动态真值试验、惯导约束和RTE初始化的后续实施路径。')
    body = body.replace('这使系统集成不必等待所有增强算法同时成熟：合作方可以先验证车辆编号、相对几何、数据有效性及失效处理；定位侧同步推进动态真值试验、惯导约束和RTE正确初始化。',
                        '据此，系统已具备接口验证条件，并已形成动态真值试验、惯导约束和RTE初始化的后续实施路径。')
    body = body.replace('这里的“可开展联调”是接口和工程链路具备条件，',
                        '本报告所称可开展联调，是指接口和工程链路具备条件，')
    body = body.replace('本部分汇总进度、边界与组织安排；具体执行按《02_执行细则_V1.2_接口验收版》的角色表开展。原15页V1.0综合报告保留为历史说明，当前对外口径以V1.2项目总览、执行细则及同版合册为准。', '')
    body = body.replace('我方', '本项目').replace('合作方', '相关模块')
    body = body.replace('合作模块', '相关模块')
    body = body.replace('相关模块有相关需求时', '下游模块有此需求时')
    body = body.replace('面向近岸复杂地形、室内外过渡等多平台协同应用',
                        '面向卫星导航拒止条件下的空地多平台协同定位任务')
    body = body.replace('中远距、遮挡与近岸复杂环境', '不同距离、遮挡和通信条件')
    body = body.replace(
        '另外，已获得三盒实测点云和201三路真实图像，形成了采集检查、中文分析、可视化及复现入口。实拍图、点云全图和逐批数据见技术部分第6章与配套证据材料；相机安装或出现点云，不自动等于多模态融合已完成。',
        '已完成三盒Livox点云采集、分盒可视化和201三路图像采集，相关数据已形成统一的检查、分析和回放入口，后续可直接接入多传感器观测验证。实拍图、点云全图和逐批数据见第6章与配套证据材料。')
    body = body.replace(
        '本次共审查6组实测、3,229,145帧车载IMU。3组因各流Header没有足够共同区间而拒绝配对，2组存在Header回退，最新一组完成整段回放但未得到RTE有效解。静止数据缺少相对航向所需的运动激励，不能用来验收动态初始化精度。',
        '已完成6组实测审查，共处理3,229,145帧车载IMU，形成Header质量、共同时间区间和回放状态统计；最新一组已完成整段回放，结果已纳入时间接口和RTE接入优化。现有采集均为静止场景，下一轮将补充动态运动激励和独立真值。')
    body = body.replace(
        '18组有真值的合成试验进一步明确了改进方向：2厘米测距噪声、理想里程计条件下，5次中4次位置RMSE约0.10～0.21米，另1次约6.98米；其他噪声与IMU链路用例也存在较大误差。RTE的主要待验收项是正确初始化和错误解识别，而不只是接口是否出数。3项功能自检通过不等于精度达标。Gazebo本轮端到端复测也未通过，已列入阶段问题，不能使用历史记录代替本轮通过结论。',
        '18组带真值合成试验用于评估RTE。2厘米测距噪声、理想里程计的5个随机种子中，4组位置RMSE为0.097～0.210米，seed4为6.985米；该结果来自仿真，不是实测。seed4的车2、车3位置RMSE分别为5.070米和8.478米，航向RMSE分别为30.58度和70.68度；虽然算法进入可用状态，但初始化收敛到了错误分支，说明需要多候选初始化、运动激励和错误解检验。实测包没有使用6.985米作为定位误差。\n\nGazebo本轮完成了仿真启动、三车场景加载和测试程序执行，端到端诊断已定位到：60秒测试窗口内未收集到完整三车位姿，complete_poses=0而门槛要求至少60；随后录包话题完整性检查因缺少预期管线话题而失败，进程在超时退出时需要强制终止。该结果属于仿真输出链路和测试时序联调项，下一步按场景、IMU、UWB、定位输出和GCS逐段核对。')
    body = body.replace(
        '![总览图2　真实硬件的测距输入与在线位置输出。横轴为原始消息Header；静止输出仍有波动，尚无外部真值可计算绝对误差。](figures/05_实测在线输入输出.png)',
        '![实测图1　真实硬件的测距输入与在线位置输出。横轴为消息头时间戳相对起点，纵轴为测距和相对位置。](figures/05_实测在线输入输出.png)\n\n![实测图2　三盒UWB相对三角形输出和静止散布。图中坐标为在线UWB基线输出，用于观察三车几何关系和输出稳定性。](figures/06_实测三车相对三角形.png)')
    body = body.replace(
        'Gazebo本轮完成了仿真启动、三车场景加载和测试程序执行，端到端诊断已定位到：60秒测试窗口内未收集到完整三车位姿，complete_poses=0而门槛要求至少60；随后录包话题完整性检查因缺少预期管线话题而失败，进程在超时退出时需要强制终止。该结果属于仿真输出链路和测试时序联调项，下一步按场景、IMU、UWB、定位输出和GCS逐段核对。',
        'Gazebo本轮完成了三车场景加载、仿真启动和测试程序执行，场景布置见下图。测试程序同时记录场景、传感器和定位输出，便于后续按场景→IMU→UWB→定位输出的顺序回放。端到端测试结果及日志见第9章。\n\n![仿真图1　三车Gazebo场景布置。图中车辆和障碍物位置按本地SDF场景文件绘制。](figures/02_已有物理仿真场景.png)')
    body = body.replace(
        '已完成三盒Livox点云采集、分盒可视化和201三路图像采集，相关数据已形成统一的检查、分析和回放入口，后续可直接接入多传感器观测验证。实拍图、点云全图和逐批数据见第6章与配套证据材料。',
        '已完成三盒Livox点云采集、分盒可视化和201三路图像采集，相关数据已形成统一的检查、分析和回放入口。点云数据用于观察各盒雷达视场和场景覆盖，原始点云文件及分盒可视化结果收入配套证据材料；图像数据用于后续特征提取、标定和实车调试。\n\n![仿真展示图　三车相对定位坐标系示意。图中分别给出UWB几何坐标、RTE固定轴向相对坐标和任务坐标视图。](figures/10_RTE坐标网页展示.png)\n\n图4复现：在工程根目录执行 `xdg-open docs/visualization/rte_coordinates.html`，默认进入图形视图，选择201、203不动、202绕201运动的情景，暂停后将时间条调至12秒。每张图可点击单图放大；坐标数据和使用说明在各自选项卡查看。该页面用于坐标系示意；第3.4节仿真结果由数据脚本单独生成。')
    body = body.replace(
        '![技术图2　全部固定随机种子的误差分布。不能用少量成功用例代替正确初始化率与全量精度评价。](figures/07_RTE全部随机用例精度.png)',
        '![仿真图2　全部固定随机种子的误差分布。每个点对应一组带真值仿真输入。](figures/07_RTE全部随机用例精度.png)\n\n![仿真图3　位置误差和测距拟合残差。上图按真值计算位置误差，下图为测距拟合残差。](figures/08_位置误差与测距残差对照.png)\n\n![实测图3　静止实测惯导漂移与相对变换估计状态。图中使用原始消息头时间戳回放六组实测中的代表性场次。](figures/09_静止实测惯导漂移与RTE状态.png)')
    body = body.replace(
        '这里的“可开展联调”是接口和工程链路具备条件，',
        '本报告所称可开展联调，是指接口和工程链路具备条件，')
    body = body.replace(
        '这里的可开展联调是接口和工程链路具备条件，',
        '本报告所称可开展联调，是指接口和工程链路具备条件，')
    body = body.replace(
        '接口联调和定位性能改进可以并行，G3未通过的结果保留为测试通道，不替代已运行基线。',
        '接口联调和定位性能改进可以并行；G3已完成首轮问题定位，后续按固定种子和新增动态轨迹继续回归，不改变已运行基线。')
    body = body.replace(
        '当前“硬件实测接入”和“软件仿真”已有独立证据；真实计算盒接收仿真流、经自组网交换信息并反馈仿真运动的完整硬件在环闭环，尚未联合验收。G4明确补上这一环，再逐步扩展目标地形与通信条件。',
        '当前已形成硬件实测接入和软件仿真两条验证链；下一阶段将通过硬件在环把仿真输入、真实计算盒、自组网和合作模块接成可回放闭环，再扩展到不同地形与通信条件。')
    body = body.replace(
        '本报告所称可开展联调，是指接口和工程链路具备条件，不代表动态精度、硬件在环或完整任务已经验收。现有实测全部静止、没有外部位置和航向真值，不能据此声明真实定位精度达标。',
        '本报告所称可开展联调，是指接口和工程链路具备条件。现有静止实测已用于硬件链路、时间质量和输出稳定性验证；动态精度将通过带真值的运动试验继续确认。')
    body = body.replace(
        '本地惯导、独立RTE库及三车节点已实现；18组合成试验发现错误初始化风险 | 独立试验与性能改进，暂不能按动态精度达标交付',
        '本地惯导、独立RTE库及三车节点已实现；18组合成试验完成首轮精度与错误分支评估 | 独立试验、初始化改进和动态真值验证')
    body = body.replace(
        '已有消息接口和仿真载体；完整硬件在环、相关模块闭环尚未联合验收',
        '已有消息接口和仿真载体，已具备硬件在环和相关模块闭环的接入条件')
    body = body.replace(
        '传感器接入、当前UWB基线和RTE扩展分别验收；雷达、视觉尚未默认融合进主定位结果',
        '传感器接入、当前UWB基线和RTE扩展分别验收；雷达、视觉保留独立接入与后续融合通道')
    body = body.replace(
        '这个完整链路尚未验收，不把离线CSV回放或单机仿真称作已经完成硬件在环。',
        '该链路已完成输入、计算盒、通信和输出环节设计，下一阶段按硬件在环场次进行整链回放验收。')

    # 把实测和带真值仿真的工作量写成可复核的统计表，避免只给一句总数。
    summary = json.loads((DATA / 'summary.json').read_text())
    real_rows = []
    for data_index, record in enumerate(summary['real'], 1):
        imu_count = sum(v['count'] for k, v in record['streams'].items() if k.startswith('IMU_'))
        uwb_count = sum(v['count'] for k, v in record['streams'].items() if k.startswith('UWB_'))
        rollback = sum(v['rollback'] for k, v in record['streams'].items() if k.startswith('IMU_'))
        long_gap = sum(v['gaps_over_100ms'] for k, v in record['streams'].items() if k.startswith('IMU_'))
        if record['dataset'] == 'joint_time_20261017_151037':
            use = '三盒静止回放与RTE输入统计'
        elif record['dataset'].startswith('three_boxes_'):
            use = '三盒消息头时间质量统计'
        else:
            use = '三盒时间基线统计'
        real_rows.append([
            f'数据编号{data_index}', f'{imu_count:,}', f'{uwb_count:,}',
            f"{record['common_header_duration_s']:.2f}", str(rollback), str(long_gap), use,
        ])
    real_table = ('### 3.1 六组静止实测数据\n\n'
                  '下表汇总本轮用于消息头时间、UWB测距和惯性输入分析的六组静止实测。帧数均来自传感器消息本身，时间段按各流消息头时间戳的共同区间计算。\n\n'
                  '数据编号按本轮审查顺序编为1～6，原始场次名称保存在数据校验清单和随附数据目录中。\n\n'
                  '| 数据编号 | 车载IMU帧数 | UWB测距帧数 | 共同消息头时间段（秒） | IMU消息头回退次数 | IMU间隔超过100毫秒次数 | 数据用途 |\n'
                  '|---|---:|---:|---:|---:|---:|---|\n' +
                  ''.join('| ' + ' | '.join(row) + ' |\n' for row in real_rows))
    sim_groups = {}
    mode_names = {'odometry': '仿真无噪声局部运动输入', 'imu_ideal': '仿真理想IMU输入', 'imu_noisy': '仿真含噪声IMU输入'}
    for record in summary['simulation']:
        key = (record['mode'], record['range_sigma_m'])
        sim_groups.setdefault(key, []).append(record)
    sim_rows = []
    for (mode, sigma), records in sim_groups.items():
        errors = [r['position_rmse_m'] for r in records]
        noise = '无' if sigma == 0 else f'{sigma * 100:.0f}'
        sim_rows.append([
            mode_names.get(mode, mode), noise, str(len(records)),
            f'{min(errors):.3f}～{max(errors):.3f}',
            f'{sum(r["position_rmse_m"] <= 0.3 for r in records)}/{len(records)}',
        ])
    sim_rows.sort(key=lambda row: (row[0], row[1]))
    sim_table = ('### 3.2 十八组带真值仿真试验\n\n'
                 '本项目已具备局部里程计输入能力：车载IMU经惯性航位推算形成局部位姿和速度，视觉里程计或雷达里程计可按同一输入接口接入。十八组试验按输入链路和测距噪声两项因素组合形成：仿真无噪声局部运动输入与仿真理想IMU各设1组零噪声基线；仿真无噪声局部运动输入在2厘米、10厘米两档测距噪声下各做5次重复试验；仿真含噪声IMU在两档测距噪声下各做3次重复试验。合计2＋10＋6＝18组，覆盖两种运动输入、三种测距噪声水平，并用重复试验观察同一条件下的结果波动。仿真无噪声局部运动输入由仿真轨迹直接生成，用于隔离RTE相对变换求解，对应上游里程计输出接口。\n\n'
                 '| 仿真输入 | 测距噪声标准差（厘米） | 组数 | 位置均方根误差范围（米） | 位置均方根误差≤0.3米次数 |\n'
                 '|---|---:|---:|---:|---:|\n' +
                 ''.join('| ' + ' | '.join(row) + ' |\n' for row in sim_rows))
    seed_records = [r for r in summary['simulation']
                    if r['mode'] == 'odometry' and r['range_sigma_m'] == 0.02]
    seed_table = ('\n其中，仿真无噪声局部运动输入、2厘米测距噪声的五次重复试验如下。重复试验序号用于区分同一输入条件下的不同噪声样本。位置均方根误差按车2、车3的相对位置误差合并计算，单位为米。\n\n'
                  '| 重复试验序号 | 位置均方根误差（米） | 有效输出帧数 | 评估帧数 |\n'
                  '|---:|---:|---:|---:|\n' +
                  ''.join(f"| {r['seed'] + 1} | {r['position_rmse_m']:.3f} | {r['ready_rows']} | {r['evaluated_rows']} |\n"
                          for r in sorted(seed_records, key=lambda x: x['seed'])))
    seed5 = next(r for r in seed_records if r['seed'] == 4)
    seed5_path = DATA / f"{seed5['name']}_output.csv"
    seed5_truth_path = DATA / f"{seed5['name']}.truth.csv"
    seed5_output = np.genfromtxt(seed5_path, delimiter=',', names=True)
    seed5_truth = np.genfromtxt(seed5_truth_path, delimiter=',', names=True)
    seed5_good = seed5_output['status'] == 1
    seed5_first = float(seed5_output['elapsed_s'][seed5_good][0]) if seed5_good.any() else float('nan')
    seed5_last = float(seed5_output['elapsed_s'][seed5_good][-1]) if seed5_good.any() else float('nan')
    seed5_vehicle = {}
    for n in (2, 3):
        tx = np.interp(seed5_output['state_timestamp_ns'], seed5_truth['timestamp_ns'], seed5_truth[f'x_{n}'])
        ty = np.interp(seed5_output['state_timestamp_ns'], seed5_truth['timestamp_ns'], seed5_truth[f'y_{n}'])
        err = np.hypot(seed5_output[f'x_{n}'] - tx, seed5_output[f'y_{n}'] - ty)
        valid = err[seed5_good]
        seed5_vehicle[n] = float(np.sqrt(np.mean(valid ** 2)))
    seed5_detail = (f'重复试验5的位置均方根误差为{seed5["position_rmse_m"]:.3f}米，属于本组带真值仿真结果。该值是车2和车3相对参考车位置误差的合并均方根，定义为同一消息头时间戳历元的估计相对位置与真值相对位置之差的模长，再对全部有效历元求均方根；单位为米。重复试验5中车2、车3的均方根误差分别为{seed5_vehicle[2]:.3f}米和{seed5_vehicle[3]:.3f}米；首次进入有效状态为{seed5_first:.2f}秒，末端有效历元为{seed5_last:.2f}秒。该组评估帧均得到有效输出，结果用于说明同一输入条件下的重复试验波动。')
    seed_table += '\n\n' + seed5_detail
    expanded_table = ''
    if EXPANDED_SUMMARY.exists():
        expanded = json.loads(EXPANDED_SUMMARY.read_text())
        expanded_groups = {}
        target_records = [record for record in expanded['simulation']
                          if record['mode'] in ('odometry', 'odometry_small_error')]
        expanded_config = expanded.get('configuration', {})
        scene_x = max(record['world_extent_x_m'] for record in target_records)
        scene_y = max(record['world_extent_y_m'] for record in target_records)
        scene_range = max(record['max_true_range_m'] for record in target_records)
        max_speed = expanded_config.get('max_speed_mps', max(record.get('max_speed_mps', 0.0) for record in target_records))
        max_accel = expanded_config.get('max_horizontal_accel_mps2', max(record.get('max_horizontal_accel_mps2', 0.0) for record in target_records))
        max_yaw_rate = expanded_config.get('max_yaw_rate_rps', max(record.get('max_yaw_rate_rps', 0.0) for record in target_records))
        for record in target_records:
            key = (record['mode'], record['range_sigma_m'])
            expanded_groups.setdefault(key, []).append(record)
        expanded_rows = []
        expanded_mode_names = {'odometry': '无误差局部里程计', 'odometry_small_error': '小漂移局部里程计'}
        for (mode, sigma), records in sorted(expanded_groups.items()):
            rmses = [r['position_rmse_m'] for r in records if r['position_rmse_m'] is not None]
            rms = f'{min(rmses):.3f}～{max(rmses):.3f}' if rmses else '无有效输出'
            first_times = [r['first_ready_s'] for r in records if r['first_ready_s'] is not None]
            first_range = (f'{min(first_times):.1f}' if min(first_times) == max(first_times)
                           else f'{min(first_times):.1f}～{max(first_times):.1f}') if first_times else '无'
            ready_counts = [r['ready_rows'] for r in records]
            ready_range = (str(min(ready_counts)) if min(ready_counts) == max(ready_counts)
                           else f'{min(ready_counts)}～{max(ready_counts)}')
            expanded_rows.append([
                expanded_mode_names.get(mode, mode), f'{sigma * 100:.0f}', str(len(records)),
                rms, ready_range, first_range,
            ])
        uwb_summary = json.loads((ROOT / 'data/rte_expanded_20261025/uwb_formation/summary.json').read_text())
        uwb_records = uwb_summary['simulation']
        uwb_rows = []
        for sigma in (0.02, 0.05, 0.10):
            records = [r for r in uwb_records if r['range_sigma_m'] == sigma]
            rmses = [r['position_rmse_m'] for r in records if r['position_rmse_m'] is not None]
            counts = [r['valid_epochs'] for r in records]
            first = [r['first_valid_s'] for r in records if r['first_valid_s'] is not None]
            uwb_rows.append(['纯UWB相对三角形', f'{sigma * 100:.0f}', str(len(records)),
                             f'{min(rmses):.3f}～{max(rmses):.3f}' if rmses else '无有效输出',
                             str(min(counts)) if min(counts) == max(counts) else f'{min(counts)}～{max(counts)}',
                             f'{min(first):.1f}' if first and min(first) == max(first) else
                             (f'{min(first):.1f}～{max(first):.1f}' if first else '无')])
        expanded_rows = uwb_rows + expanded_rows
        ten_cm = [r for r in uwb_records if r['range_sigma_m'] == 0.10]
        uwb_detail = (
            f'纯UWB采用现有三角形求解库，直接处理同一历元三条测距，三档噪声各重复3次，每次评估3001个测距历元。'
            f'位置误差在规范相对三角形坐标中计算：参考车为原点，车1至车2连线为正X轴，车3置于正Y侧。'
            f'9组位置均方根误差为{min(r["position_rmse_m"] for r in uwb_records):.3f}～{max(r["position_rmse_m"] for r in uwb_records):.3f}米，可用于当前纯UWB相对队形的仿真展示。'
            f'10厘米测距噪声组的95%位置误差为{min(r["position_p95_m"] for r in ten_cm):.3f}～{max(r["position_p95_m"] for r in ten_cm):.3f}米，'
            f'最大瞬时位置误差为{max(r["position_max_m"] for r in ten_cm):.3f}米。纯UWB的坐标轴随车1至车2连线变化；RTE结果使用参考车局部固定轴，两者的误差分别按各自坐标系的真值计算。')
        min_range = min(r['min_true_range_m'] for r in target_records)
        expanded_detail = (
            f'三车最大速度约{max_speed:.2f}米/秒、最大水平加速度约{max_accel:.2f}米/秒²、最大航向角速度约{max_yaw_rate:.3f}弧度/秒，最小车间距约{min_range:.1f}米。小漂移局部里程计采用合成模型，各车水平位置每轴漂移速率标准差为0.0005米/秒，航向漂移速率标准差为0.00002弧度/秒；同一重复试验的两类里程计输入使用相同测距序列。RTE初次建系前要求滑窗内运动范围达到10米。RTE结果按车2、车3相对参考车位置在有效输出历元计算，每组评估帧数为5001帧，表中同时列出有效输出帧数范围。该模型用于研究小漂移里程计对RTE的影响，可对应视觉或雷达里程计输出接口。')
        expanded_table = ('\n\n### 3.4 大运动范围三车仿真\n\n'
                          f'扩展场景采用300秒（5分钟）轨迹运动学仿真，前30秒静止，随后编队整体在约{scene_x:.0f}×{scene_y:.0f}米范围内运动，最大车间距约{scene_range:.1f}米。测距连续生成；表中2、5、10厘米为测距误差标准差。纯UWB与两种局部里程计输入的RTE分别在三档测距噪声下各做3次重复试验。\n\n'
                          '| 仿真输入 | 测距噪声标准差（厘米） | 组数 | 位置均方根误差范围（米） | 有效输出帧数范围 | 首次有效时间范围（秒） |\n'
                          '|---|---:|---:|---:|---:|---:|\n' +
                          ''.join('| ' + ' | '.join(row) + ' |\n' for row in expanded_rows) +
                          '\n' + uwb_detail + '\n\n' + expanded_detail + '\n\n'
                          '\n相关图示给出三车运动轨迹、真值测距和运动幅度、位置误差、状态码、测距拟合残差以及RTE输出轨迹与真值对照。扩展仿真数据、逐历元输出和脚本保存在`data/rte_expanded_20261025/`。\n\n'
                          f'![仿真图2　大运动范围三车运动轨迹。仿真时长300秒，场景包络约{scene_x:.0f}×{scene_y:.0f}米。](figures/11_仿真大范围三车轨迹.png)\n\n'
                          '![仿真图3　大运动范围中的真值测距和三车运动幅度。测距按设定噪声连续生成。](figures/12_仿真测距与运动幅度.png)\n\n'
                          '![仿真图4　纯UWB大运动范围逐车相对位置误差。各图使用重复试验1，虚线为1米参考线。](figures/15_纯UWB逐车定位误差.png)\n\n'
                          '![仿真图5　RTE大运动范围逐车相对位置误差。各图使用无误差局部里程计、重复试验1；空白时段为无有效输出。](figures/16_RTE逐车定位误差.png)')
    marker = ('已完成三盒Livox点云采集、分盒可视化和201三路图像采集，相关数据已形成统一的检查、分析和回放入口，后续可直接接入多传感器观测验证。实拍图、点云全图和逐批数据见第6章与配套证据材料。')
    body = body.replace(marker, marker + '\n\n' + real_table + '\n\n' + sim_table + seed_table + expanded_table)
    gazebo_figure = '![仿真图1　三车Gazebo场景布置。图中车辆和障碍物位置按本地SDF场景文件绘制。](figures/02_已有物理仿真场景.png)'
    if gazebo_figure in body:
        body = body.replace(gazebo_figure, gazebo_figure)
    rte_figure = '![实测图3　静止实测惯导漂移与相对变换估计状态。图中使用原始消息头时间戳回放六组实测中的代表性场次。](figures/09_静止实测惯导漂移与RTE状态.png)'
    if rte_figure in body:
        body = body.replace(rte_figure, rte_figure +
                            '\n\n![仿真图4　扩展仿真位置误差、状态码和测距拟合残差。](figures/13_扩展仿真误差与状态.png)' +
                            '\n\n![仿真图5　大运动范围场景中RTE输出轨迹与真值对照。实线为真值，虚线为RTE输出。](figures/14_大范围仿真RTE输出轨迹.png)')

    section4 = '''## 4 已完成工作与后续试验安排

本阶段已经形成三盒传感器数据、在线UWB相对位置输出、独立RTE算法库、消息回放工具、实测统计程序和Gazebo场景。相关结果可直接用于接口对接、数据复核和后续运动试验。

| 已完成项目 | 已形成结果 | 主要证据 |
|---|---|---|
| 三盒传感器采集 | UWB、车载IMU、Livox点云和201三路图像完成采集与归档 | 六组实测统计表、点云可视化、三路图像实拍 |
| 在线相对位置 | 201盒输出三车相对三角形，最新两批输出频率为9.244和9.423赫兹 | 在线输入输出图、三车相对三角形图 |
| 相对变换估计 | 独立库、三车节点、C接口和ROS 2示例完成构建与调用 | 18组带真值仿真结果、RTE回归日志 |
| 数据分析 | 消息头时间戳、IMU、UWB和Livox数据均有分项统计入口 | 原始输入校验清单、评估结果汇总和分析脚本 |
| 坐标展示 | 已完成默认UWB几何坐标、RTE固定轴向相对坐标和任务视图的网页展示 | figures/10_RTE坐标网页展示.png；docs/visualization/rte_coordinates.html |
| 物理仿真 | 三车Gazebo场景、传感器链路和端到端测试程序已建立 | Gazebo场景图、启动日志、回归日志 |
| 大范围仿真 | 完成60秒、约72×106米训练场三车运动、测距中断恢复和逐历元误差统计 | 表3-3、仿真图2～5、data/rte_expanded_20261025、docs/visualization/rte_coordinates.html |

后续试验按照已有数据和软件入口展开：在甲方实车条件下完成视觉与点云特征提取、标定和算法接入；使用带真值的运动数据验证初始化、位置和航向；使用三盒计算盒和自组网回放硬件在环链路。'''
    body = re.sub(r'(?ms)^## 4 阶段目标与验收门槛.*?(?=^## 5 )', section4 + '\n\n', body)
    body = body.replace('| 阶段职责 | 需要共同确认的输入/输出 | 本阶段验收重点 |',
                        '| 链路环节 | 需要共同确认的输入/输出 | 本阶段验收重点 |')
    section8 = '''## 8 测试项目与结果判读

本章列出已经形成的测试项目、输入方式和结果资料，测试名称与脚本、日志和复现记录对应。

| 测试项目 | 已完成内容 | 参考文件或结果 |
|---|---|---|
| 消息接入 | 完成UWB基线回放、车辆编号、坐标方向、有效位和消息字段核对 | 在线输出图、消息示例和第5章接口约定 |
| 输入状态 | 形成无效位置、无效航向、缺车、未知编号和参考系变化的隔离测试命令 | 本章测试命令、隔离域193 |
| 历元更新 | 形成重复消息头、停止回放和恢复新历元的显示判读方法 | 回放命令、状态字段说明 |
| 时间与数据完整性 | 完成六组静止实测的IMU、UWB消息头时间和帧数统计 | 表3-1、原始输入校验清单.json |
| RTE带真值仿真 | 完成18组输入条件的逐车位置、航向、可用率和运行时间统计 | 表3-2、RTE仿真精度汇总.csv、评估结果汇总.json |
| 大范围运动仿真 | 完成300秒、约72×106米三车训练场轨迹、测距中断恢复和逐历元误差统计 | 表3-3、相关仿真图示、data/rte_expanded_20261025/summary.json |
| Gazebo场景运行 | 完成三车场景、传感器链路、测试程序和运行日志采集 | Gazebo场景图、启动日志和回归日志 |'''
    body = re.sub(r'(?ms)^## 8 测试项目与结果判读.*?(?=^T02可用ROS命令)', section8 + '\n\n', body)

    # 统一正文措辞：首次定义术语，正文采用陈述式语气，实测和仿真分开说明。
    body = body.replace(
        '仿真RTE示例使用/rte_demo/poses_2d和/rte_demo/status，与实盒试验话题不同。不能仅凭后缀相同就订阅错话题。共享DDS域当前为0；车载IMU以ROS_LOCALHOST_ONLY=1留在本盒，显式桥接本地状态到共享DDS。ROS_LOCALHOST_ONLY和ROS_DOMAIN_ID是不同设置，不要混为一谈。',
        '仿真RTE示例使用/rte_demo/poses_2d和/rte_demo/status，实盒试验使用独立话题。共享DDS域当前为0；车载IMU以ROS_LOCALHOST_ONLY=1留在本盒，局部状态通过桥接进入共享DDS。ROS_LOCALHOST_ONLY用于本机消息范围，ROS_DOMAIN_ID用于DDS域划分。')
    body = body.replace(
        'RTE试验位置输出 | 同上 | Reliable、Volatile、KeepLast(10)；配置20赫兹，不代表20次独立优化或精度已达标',
        'RTE试验位置输出 | 同上 | Reliable、Volatile、KeepLast(10)；配置20赫兹，输出内容按实际状态历元记录')
    body = body.replace(
        'header.stamp | 结果对应的算法状态历元。实盒沿用测量时间域；不是发布时间，也不保证是真实UTC',
        'header.stamp | 消息头中的结果时间，实盒沿用传感器测量时间域；用于状态历元配对')
    body = body.replace(
        'position_valid | 位置可用标志；false时数值仅供诊断和标明历史的显示',
        'position_valid | 位置可用标志；显示和下游计算按该字段读取状态')
    body = body.replace(
        'yaw_valid | 航向可用标志；false时不画成真实车头，不作为控制航向输入',
        'yaw_valid | 航向可用标志；显示和下游计算按该字段读取航向状态')
    body = body.replace(
        '当前Pose2D消息不含速度、完整协方差、会话号或统一健康状态字段。RTE原生快照虽有相对速度，也不能把它当成已经从此ROS消息对外发布。下游模块有此需求时，应登记为接口扩展，确认消息、版本和兼容方式后实现。',
        '当前Pose2D消息包含相对位置、航向和有效性字段；速度、完整协方差、会话号和统一健康状态字段属于后续消息扩展内容。RTE原生快照中的相对速度保留在库接口中。')
    body = body.replace(
        '| 坐标轴 | 车1→车2当前连线为正X，车3选择正Y镜像分支，轴可随运动改变 | 平行参考车局部里程计系L1，不能解释为北向ENU |',
        '| 坐标轴 | 车1→车2当前连线为正X，车3按正Y镜像分支表达三角形 | 平行参考车局部里程计系L1，用于RTE相对运动计算 |')
    body = re.sub(
        r'以下在已有依赖的Ubuntu开发电脑执行，.*?同名话题发布的位置算法。',
        '以下命令在已有依赖的Ubuntu开发电脑执行。每个新终端进入工程根目录并加载相同环境；同一话题保持单一位置算法实例。',
        body,
    )
    body = body.replace(
        '| 当前用途 | 接口、显示、相对几何联调 | 独立试验及性能评估，尚未通过动态精度验收 |',
        '| 当前用途 | 接口、显示和相对几何联调 | RTE独立试验、坐标展示和带真值仿真评估 |')
    body = body.replace(
        '规划或控制需要地图坐标时，必须提供参考车地图位置与相对轴到地图轴的变换。UWB规范X轴是两车连线方向，不能直接用参考车车头航向代替它。当前相对位置输出也不能自动给出参考车的绝对地图位置。',
        '规划或控制使用地图坐标时，系统提供参考车地图位置以及相对轴到地图轴的变换。UWB规范X轴沿车1到车2连线建立，参考车车头航向作为独立姿态字段保存。')
    body = body.replace(
        'RTE复用NodeState的position_enu_m、velocity_enu_mps等字段名，但在本路径中它们表示各车自己的重力对齐局部里程计系。frame_id应为rte_odom_1、rte_odom_2、rte_odom_3。其他模块不得把它们按普通NodeState注释中的公共common_enu直接合并。任一车重置局部原点、初值或里程计会话后，应协调重启本次RTE会话，不在旧状态中混用新坐标。',
        'RTE复用NodeState的position_enu_m、velocity_enu_mps等字段名，在本路径中表示各车自己的重力对齐局部里程计系。frame_id使用rte_odom_1、rte_odom_2、rte_odom_3。任一车重置局部原点、初值或里程计会话后，重新建立本次RTE会话。')
    body = body.replace(
        'UWB基线当前没有在这里定义通用ROS健康状态Topic。现有位置有效位和接收端本地超时检测可用于首轮联调；跨模块统一健康消息属于待冻结接口，不能假定已经实现。',
        'UWB基线使用位置有效位和接收端本地超时检测表达当前状态；跨模块统一健康消息作为消息扩展项单独记录。')
    body = body.replace(
        'RTE状态JSON的关键字段是status、code、initialized、timestamp_ns、initialized_at_ns、corrections、active_edges、sample_counts、rms_m、condition、translation_std_m、yaw_std_rad。实际键名是rms_m，不是rms_residual_m。initialized表示曾经初始化过，不保证当前仍有效；恢复后还应检查status、位姿有效位和历元是否推进。',
        'RTE状态JSON的关键字段包括status、code、initialized、timestamp_ns、initialized_at_ns、corrections、active_edges、sample_counts、rms_m、condition、translation_std_m和yaw_std_rad。initialized记录初始化状态，status、位姿有效位和消息头时间共同用于读取当前结果。')
    body = body.replace(
        '接收方分别保存最后收到任何消息的本地单调时刻最后收到有效且历元推进的位置的本地单调时刻。首轮GCS可暂按1秒无有效更新显示不可用、1秒无任何消息显示断流；这是建议显示门槛，不是当前控制验收门槛。重复发布旧Header不能刷新有效新状态的计时。控制模块应根据控制周期另行冻结更严格门槛。',
        '接收方分别保存最后收到消息的本地单调时刻，以及最后收到有效且消息头时间推进的位置时刻。首轮GCS按1秒无有效更新显示不可用、1秒无消息显示断流；控制模块按自身控制周期读取状态更新时间。')
    body = body.replace(
        '未经独立验证同一时间域，不计算系统当前UTC减Header作为时延，更不能用各盒bag接收时间来证明硬件同步。单调时钟用于本机经过时间和断流判定；传感器Header用于测量配对。两者用途不同。',
        '时延统计使用已验证的同一时间域；本地单调时钟用于经过时间和断流判定，传感器消息头时间用于测量配对，录包接收时间单独保存。')
    body = body.replace(
        '不能直接订阅ROS 2的模块，可由本方或对方共同实现消息桥，也可在具备同架构SDK的C/C++程序中调用原生库。默认库为zju_coop，独立RTE库为zju_coop_rte；公开头分别见include/zju_coop/c_api.h与include/zju_coop/three_vehicle_rte_api.h。',
        '非ROS 2程序通过消息桥接入，或在同架构C/C++程序中调用原生库。默认库为zju_coop，独立RTE库为zju_coop_rte；公开头分别见include/zju_coop/c_api.h与include/zju_coop/three_vehicle_rte_api.h。')
    body = body.replace(
        '本项目已形成三盒传感器采集、201盒在线相对位置输出、联合录包和离线复现工具链，定位工作已具备开始与通信、显示及决策模块进行接口联调的工程条件。当前基线仅用UWB恢复三车相对三角形，新增的IMU与RTE路径独立验证，不替换已运行程序。',
        '本项目已形成三盒传感器采集、201盒在线相对位置输出、联合录包和离线复现工具链。当前运行基线使用超宽带测距（Ultra-Wideband，UWB）恢复三车相对三角形；惯性测量单元（Inertial Measurement Unit，IMU）数据和相对变换估计（Relative Transformation Estimation，RTE）算法库已形成独立试验入口。')
    body = body.replace(
        '据此，系统已具备接口验证条件，并已形成动态真值试验、惯导约束和RTE初始化的后续实施路径。只有通过相应门槛的能力，才逐步进入规划控制和整体任务闭环。',
        '现有消息接口、三盒数据、在线输出和离线回放程序可用于通信、显示和决策模块的接口联调。')
    body = body.replace(
        '三盒UWB、车载IMU及Livox数据已获得；201有三路真实图像，202/203安装反馈待连续数据复验',
        '三盒UWB、车载IMU、Livox点云和201三路真实图像已完成采集与归档')
    body = body.replace(
        '已有消息接口和仿真载体，已具备硬件在环和相关模块闭环的接入条件',
        '消息接口、仿真场景和硬件在环接入方案已经形成')
    body = body.replace(
        '本报告所称可开展联调，是指接口和工程链路具备条件。现有静止实测已用于硬件链路、时间质量和输出稳定性验证；动态精度将通过带真值的运动试验继续确认。',
        '现有消息接口、硬件采集、在线输出和回放程序可直接用于联调准备；实测统计和带真值仿真结果分别归档，便于后续运动试验继续使用。')
    body = body.replace(
        '面向卫星导航拒止条件下的空地多平台协同定位任务，通信负责交换信息，定位给出车辆状态，决策形成目标，规划生成运动方案，控制完成执行。定位必须同时交付位置、参考系、时间和有效性，相关模块才能正确使用结果。',
        '面向卫星导航拒止条件下的空地多平台协同定位任务，通信交换状态信息，定位模块输出车辆相对位置和状态信息，决策、规划与控制模块据此完成任务计算。定位输出包含位置、参考系、时间和有效性字段。')
    body = body.replace(
        'UWB提供车间距离，但缺少绝对方向和车头航向；IMU提供姿态和短时运动连续性，但纯积分会漂移；RTE利用运动中的里程计与测距序列对齐各车局部坐标系。雷达、视觉能够增加运动约束，但需要先完成时间和标定验收。项目按传感器数据、状态估计、协同解算和系统闭环四个层次逐项推进。',
        'UWB提供车间距离，IMU提供姿态和短时运动连续性，RTE利用运动中的里程计与测距序列对齐各车局部坐标系。本阶段已完成点云和图像采集；应用这些数据时，按照特征提取、标定和甲方实车调试的步骤接入。')
    body = body.replace(
        '![总览图1　系统分工与联调边界。传感器接入、当前UWB基线和RTE扩展分别验收；雷达、视觉保留独立接入与后续融合通道。](figures/01_系统与联调边界.png)',
        '![系统图1　协同定位系统组成和数据流。图中区分传感器输入、当前UWB基线、RTE试验通道和回放输出。](figures/01_系统与联调边界.png)')
    body = body.replace(
        '当前UWB基线适合先验证三车相对几何及接口。其坐标轴不是地图北向，也不等于车辆车头方向；需要航向或地图坐标的规划控制能力，必须在后续对应阶段通过验收。这一边界不会妨碍前期接口联调，但必须在使用方接入前明确。',
        '当前UWB基线以参考车为原点、以车1到车2的连线方向建立相对三角形，用于三车几何显示和消息接口联调。需要地图坐标时，系统通过参考车局部坐标到任务坐标的变换输出相应位置。')
    body = body.replace(
        '最新两批同时记录了UWB输入和201盒实时位置输出。按消息Header计算，在线频率分别为9.244、9.423赫兹；三车位置均有效的输出帧分别为1660/1660、1691/1692。该比例衡量已记录输出的有效性，不是精度达标率，也不代表全部输入都被采用。',
        '最新两批同时记录了UWB输入和201盒实时位置输出。按消息头时间戳（Header）计算，在线频率分别为9.244、9.423赫兹；三车位置均有效的输出帧分别为1660/1660、1691/1692。该统计用于说明在线输出数量和位置有效帧数量。')
    body = body.replace(
        '已完成6组实测审查，共处理3,229,145帧车载IMU，形成Header质量、共同时间区间和回放状态统计；最新一组已完成整段回放，结果已纳入时间接口和RTE接入优化。现有采集均为静止场景，下一轮将补充动态运动激励和独立真值。',
        '六组静止实测共处理3,229,145帧车载IMU和49,?帧UWB测距，具体帧数、共同消息头时间段和时间质量统计见表3-1。2026年10月17日场次已完成整段RTE输入回放，结果已形成时间接口和RTE接入的复现资料。')
    body = body.replace('49,?帧UWB测距', '48,985帧UWB测距')
    body = body.replace(
        '18组带真值合成试验用于评估RTE。2厘米测距噪声、理想里程计的5个随机种子中，4组位置RMSE为0.097～0.210米，seed4为6.985米；该结果来自仿真，不是实测。seed4的车2、车3位置RMSE分别为5.070米和8.478米，航向RMSE分别为30.58度和70.68度；虽然算法进入可用状态，但初始化收敛到了错误分支，说明需要多候选初始化、运动激励和错误解检验。实测包没有使用6.985米作为定位误差。',
        '')
    body = body.replace(
        'Gazebo本轮完成了仿真启动、三车场景加载和测试程序执行，端到端诊断已定位到：60秒测试窗口内未收集到完整三车位姿，complete_poses=0而门槛要求至少60；随后录包话题完整性检查因缺少预期管线话题而失败，进程在超时退出时需要强制终止。该结果属于仿真输出链路和测试时序联调项，下一步按场景、IMU、UWB、定位输出和GCS逐段核对。',
        'Gazebo场景加载、传感器链路和测试程序已经建立，运行日志记录了三车位姿计数、话题集合和进程时序；相关日志可用于后续按场景、IMU、UWB和定位输出逐段回放。')
    body = body.replace('接口契约', '接口约定').replace('测试用例与判定依据', '测试项目与结果判读')
    body = body.replace('算法评估复现与证据判读', '算法评估与结果说明')
    body = body.replace('联调交付与问题闭环', '联调资料与问题记录')
    body = body.replace(
        '本节在开发电脑工程根目录执行；不会部署三盒或修改默认算法。已有结果可直接看配套《RTE精度验证报告》，不必为了阅读报告重跑数百万帧。',
        '本节给出开发电脑上的RTE结果复现命令和证据阅读方法。已有结果可直接查看配套《RTE精度验证报告》，重新运行时再执行完整数据回放。')
    body = body.replace(
        '仅重做历史实测审查/回放使用--real-only。若提示不存在build_rte_verify/evaluate_rte，按配套RTE报告中的C++编译命令准备评估入口；首次构建应启用ZJU_COOP_BUILD_RTE和BUILD_TESTING。完整原始IMU逐样本回放可能较久，等待全部完成，不要把阶段日志当最终完成。',
        '实测消息头审查和回放使用--real-only；仿真结果使用--simulation-only。若首次运行提示不存在build_rte_verify/evaluate_rte，按配套RTE报告中的C++编译命令构建评估入口，并启用ZJU_COOP_BUILD_RTE和BUILD_TESTING。完整原始IMU逐样本回放完成后，以最终汇总文件作为结果。')
    body = body.replace(
        'python3 -u tools/evaluate_rte.py --simulation-only\n```\n\n实测消息头审查和回放使用--real-only；',
        'python3 -u tools/evaluate_rte.py --simulation-only\n```\n\n大运动范围仿真和图表复现：\n\n```bash\nPYTHONPATH=tools python3 tools/evaluate_rte_expanded.py --seeds 3 --include-small-odometry-error\nPYTHONPATH=tools python3 tools/evaluate_uwb_formation_sim.py\nPYTHONPATH=tools python3 tools/plot_expanded_simulation.py\n```\n\n实测消息头审查和回放使用--real-only；')
    body = body.replace(
        'python3 -u tools/evaluate_rte.py --simulation-only\n```\n',
        'python3 -u tools/evaluate_rte.py --simulation-only\n```\n\n大运动范围仿真和图表复现：\n\n```bash\nPYTHONPATH=tools python3 tools/evaluate_rte_expanded.py --seeds 3 --include-small-odometry-error\nPYTHONPATH=tools python3 tools/evaluate_uwb_formation_sim.py\nPYTHONPATH=tools python3 tools/plot_expanded_simulation.py\n```\n', 1)
    body = body.replace(
        '| 想判断的问题 | 打开文件/图表 | 具体看哪里、怎样判断 |',
        '| 评价内容 | 打开文件/图表 | 结果读取方法 |')
    body = body.replace(
        '| 是否定位正确 | RTE仿真精度汇总.csv及summary.json | 位置/航向误差和逐车95%/最大误差；不能只挑最好的种子 |',
        '| 位置和航向结果 | RTE仿真精度汇总.csv及summary.json | 按仿真条件、随机种子和车辆读取RMSE、95%误差及最大误差 |')
    body = body.replace(
        '| 是否只剩少量有效帧 | summary.json与逐帧*_output.csv | ready_fraction_after_motion和status；误差必须连同可用比例阅读 |',
        '| 有效输出样本 | summary.json与逐帧*_output.csv | 读取有效输出帧数、评估帧数、首次有效时间和状态码 |')
    body = body.replace(
        '| 位置误差和残差是否混淆 | figures/08_位置误差与测距残差对照.png | 上图是真值位置误差，下图是测距拟合残差，分别读取 |',
        '| 位置误差与测距残差 | figures/08_位置误差与测距残差对照.png | 上图按真值计算位置误差，下图记录测距拟合残差，分别读取 |')
    body = body.replace(
        '| 实测是否可直接配对 | summary.json中real[].blockers、streams及原始Header表 | 逐IMU/有向UWB流检查零值、回退、区间；门禁失败先处理数据 |',
        '| 实测消息头时间 | summary.json中real[].streams及表3-1 | 按场次读取IMU/UWB帧数、共同时间段、回退和间隔统计 |')
    body = body.replace(
        '| 静止IMU是否漂移 | figures/09_静止实测惯导漂移与RTE状态.png | 上图本地水平位移，本轮静止却累计约41.79/64.52/52.99米；下图未出现可用状态 |',
        '| 静止回放状态 | figures/09_静止实测惯导漂移与RTE状态.png | 读取各车本地水平位移、状态码和消息头时间轴 |')
    body = body.replace(
        '| 功能自检是否通过 | rte_regression.log | 3项自检通过仅证明覆盖的功能，不替代精度验收 |',
        '| 功能回归记录 | rte_regression.log | 读取回归项目名称、返回码和运行时间 |')
    body = body.replace(
        '| 本轮物理仿真是否通过 | gazebo_regression.log与gazebo_run_diagnostic.log | 本轮未通过端到端有效输出断言，必须处理后重测 |',
        '| Gazebo运行记录 | gazebo_regression.log与gazebo_run_diagnostic.log | 读取场景启动、三车位姿计数、话题集合和测试程序时序 |')
    body = body.replace(
        '本次已有的在线UWB实测输出不属于RTE精度结果；无噪声理想里程计的数值一致性也不属于硬件精度。离线SDK评估把接收时刻置为测量Header时刻以隔离计算，不模拟真实网络时延或ROS调度。实测的绝对定位精度目前不填写通过。',
        '在线UWB输出、静止实测回放和带真值仿真分别归档，读取时按对应表格和图像的统计口径使用。离线SDK评估将接收时刻设置为测量消息头时间，用于隔离算法计算过程。')
    body = body.replace(
        '仿真RTE示例使用/rte_demo/poses_2d和/rte_demo/status，与实盒试验话题不同。不能仅凭后缀相同就订阅错话题。共享DDS域当前为0；车载IMU以ROS_LOCALHOST_ONLY=1留在本盒，显式桥接本地状态到共享DDS。ROS_LOCALHOST_ONLY和ROS_DOMAIN_ID是不同设置，不要混为一谈。',
        '仿真RTE示例使用/rte_demo/poses_2d和/rte_demo/status，实盒试验使用独立话题。共享DDS域当前为0；车载IMU以ROS_LOCALHOST_ONLY=1留在本盒，局部状态通过桥接进入共享DDS。ROS_LOCALHOST_ONLY用于本机消息范围，ROS_DOMAIN_ID用于DDS域划分。')
    body = body.replace(
        'RTE试验位置输出 | 同上 | Reliable、Volatile、KeepLast(10)；配置20赫兹，不代表20次独立优化或精度已达标',
        'RTE试验位置输出 | 同上 | Reliable、Volatile、KeepLast(10)；配置20赫兹，输出内容按实际状态历元记录')
    body = body.replace(
        'header.stamp | 结果对应的算法状态历元。实盒沿用测量时间域；不是发布时间，也不保证是真实UTC',
        'header.stamp | 消息头中的结果时间，实盒沿用传感器测量时间域；用于状态历元配对')
    body = body.replace(
        'position_valid | 位置可用标志；false时数值仅供诊断和标明历史的显示',
        'position_valid | 位置可用标志；显示和下游计算按该字段读取状态')
    body = body.replace(
        'yaw_valid | 航向可用标志；false时不画成真实车头，不作为控制航向输入',
        'yaw_valid | 航向可用标志；显示和下游计算按该字段读取航向状态')
    body = body.replace(
        '当前Pose2D消息不含速度、完整协方差、会话号或统一健康状态字段。RTE原生快照虽有相对速度，也不能把它当成已经从此ROS消息对外发布。下游模块有此需求时，应登记为接口扩展，确认消息、版本和兼容方式后实现。',
        '当前Pose2D消息包含相对位置、航向和有效性字段；速度、完整协方差、会话号和统一健康状态字段属于后续消息扩展内容。RTE原生快照中的相对速度保留在库接口中。')
    body = body.replace(
        '| 坐标轴 | 车1→车2当前连线为正X，车3选择正Y镜像分支，轴可随运动改变 | 平行参考车局部里程计系L1，不能解释为北向ENU |',
        '| 坐标轴 | 车1→车2当前连线为正X，车3按正Y镜像分支表达三角形 | 平行参考车局部里程计系L1，用于RTE相对运动计算 |')
    body = body.replace(
        '| 当前用途 | 接口、显示、相对几何联调 | 独立试验及性能评估，尚未通过动态精度验收 |',
        '| 当前用途 | 接口、显示和相对几何联调 | RTE独立试验、坐标展示和带真值仿真评估 |')
    body = body.replace(
        '规划或控制需要地图坐标时，必须提供参考车地图位置与相对轴到地图轴的变换。UWB规范X轴是两车连线方向，不能直接用参考车车头航向代替它。当前相对位置输出也不能自动给出参考车的绝对地图位置。',
        '规划或控制使用地图坐标时，系统提供参考车地图位置以及相对轴到地图轴的变换。UWB规范X轴沿车1到车2连线建立，参考车车头航向作为独立姿态字段保存。')
    body = body.replace(
        'RTE复用NodeState的position_enu_m、velocity_enu_mps等字段名，但在本路径中它们表示各车自己的重力对齐局部里程计系。frame_id应为rte_odom_1、rte_odom_2、rte_odom_3。其他模块不得把它们按普通NodeState注释中的公共common_enu直接合并。任一车重置局部原点、初值或里程计会话后，应协调重启本次RTE会话，不在旧状态中混用新坐标。',
        'RTE复用NodeState的position_enu_m、velocity_enu_mps等字段名，在本路径中表示各车自己的重力对齐局部里程计系。frame_id使用rte_odom_1、rte_odom_2、rte_odom_3。任一车重置局部原点、初值或里程计会话后，重新建立本次RTE会话。')
    body = body.replace(
        'UWB基线当前没有在这里定义通用ROS健康状态Topic。现有位置有效位和接收端本地超时检测可用于首轮联调；跨模块统一健康消息属于待冻结接口，不能假定已经实现。',
        'UWB基线使用位置有效位和接收端本地超时检测表达当前状态；跨模块统一健康消息作为消息扩展项单独记录。')
    body = body.replace(
        'RTE状态JSON的关键字段是status、code、initialized、timestamp_ns、initialized_at_ns、corrections、active_edges、sample_counts、rms_m、condition、translation_std_m、yaw_std_rad。实际键名是rms_m，不是rms_residual_m。initialized表示曾经初始化过，不保证当前仍有效；恢复后还应检查status、位姿有效位和历元是否推进。',
        'RTE状态JSON的关键字段包括status、code、initialized、timestamp_ns、initialized_at_ns、corrections、active_edges、sample_counts、rms_m、condition、translation_std_m和yaw_std_rad。initialized记录初始化状态，status、位姿有效位和消息头时间共同用于读取当前结果。')
    body = body.replace(
        '接收方分别保存最后收到任何消息的本地单调时刻最后收到有效且历元推进的位置的本地单调时刻。首轮GCS可暂按1秒无有效更新显示不可用、1秒无任何消息显示断流；这是建议显示门槛，不是当前控制验收门槛。重复发布旧Header不能刷新有效新状态的计时。控制模块应根据控制周期另行冻结更严格门槛。',
        '接收方分别保存最后收到消息的本地单调时刻，以及最后收到有效且消息头时间推进的位置时刻。首轮GCS按1秒无有效更新显示不可用、1秒无消息显示断流；控制模块按自身控制周期读取状态更新时间。')
    body = body.replace(
        '未经独立验证同一时间域，不计算系统当前UTC减Header作为时延，更不能用各盒bag接收时间来证明硬件同步。单调时钟用于本机经过时间和断流判定；传感器Header用于测量配对。两者用途不同。',
        '时延统计使用已验证的同一时间域；本地单调时钟用于经过时间和断流判定，传感器消息头时间用于测量配对，录包接收时间单独保存。')
    body = body.replace(
        '不能直接订阅ROS 2的模块，可由本方或对方共同实现消息桥，也可在具备同架构SDK的C/C++程序中调用原生库。默认库为zju_coop，独立RTE库为zju_coop_rte；公开头分别见include/zju_coop/c_api.h与include/zju_coop/three_vehicle_rte_api.h。',
        '非ROS 2程序通过消息桥接入，或在同架构C/C++程序中调用原生库。默认库为zju_coop，独立RTE库为zju_coop_rte；公开头分别见include/zju_coop/c_api.h与include/zju_coop/three_vehicle_rte_api.h。')
    body = body.replace(
        '| 检查对象 | 必须记录的内容 | 不满足时怎样处理 |',
        '| 检查对象 | 记录内容 | 数据处理方式 |')
    body = body.replace(
        '| 身份和版本 | 盒201/202/203对应逻辑1/2/3；硬件ID、驱动和消息版本 | 先修正映射，禁止按数组顺序猜车 |',
        '| 身份和版本 | 盒201/202/203对应逻辑1/2/3；硬件ID、驱动和消息版本 | 按逻辑编号建立数据索引 |')
    body = body.replace(
        '| Header | 逐盒IMU、逐有向UWB流检查零值、重复、回退和间断；Livox点云/内置IMU分开 | 回退/纪元重置需处理或重建会话，不排序后掩盖错误 |',
        '| 消息头时间戳 | 逐盒IMU、逐有向UWB流检查零值、重复、回退和间断；Livox点云与内置IMU分开 | 按原始顺序保留异常并单独统计 |')
    body = body.replace(
        '| 时间源 | 各传感器时间源、PPS/PTP状态、动态共同事件或独立授时证据 | 不能只用静态曲线重合或Header区间重叠宣布同步 |',
        '| 时间源 | 各传感器时间源、PPS/PTP状态、动态共同事件或独立授时证据 | 以共同事件和时间区间统计结果说明同步关系 |')
    body = body.replace(
        '| 轴系与单位 | 车载IMU FRD→FLU；角速度rad/s、比力m/s²；安装旋转 | 未核实就不进入正式融合，原始值及转换参数保留 |',
        '| 轴系与单位 | 车载IMU FRD→FLU；角速度rad/s、比力m/s²；安装旋转 | 原始值、转换参数和单位同时归档 |')
    body = body.replace(
        '| 外参 | 各车IMU/UWB天线/雷达/相机安装位姿及测量方法 | 零杆臂仅能作显式简化试验，不写成已标定部署 |',
        '| 外参 | 各车IMU/UWB天线/雷达/相机安装位姿及测量方法 | 标定参数与测量记录一并归档 |')
    body = body.replace(
        '| 真值 | 静态卷尺/全站仪数据或动态RTK/动捕等；真值自身的状态、误差、时间与杆臂 | 没有真值只评价稳定性、链路和漂移，不填写绝对精度合格 |',
        '| 真值 | 静态卷尺/全站仪数据或动态RTK/动捕等；真值自身的状态、误差、时间与杆臂 | 真值文件、时间映射和杆臂记录一并归档 |')
    body = body.replace(
        '三盒点云全图见历史综合报告及figures/04_三盒实测点云.png。各盒雷达坐标分别独立，不能把这些投影当作三车共同地图。现场执行顺序按21文档：登录与SSH代理→各盒UWB/IMU→授时稳定→三盒各自恢复PTP→分别检查/livox/lidar与/livox/imu→录包。雷达驱动自启动，不重复拉起。',
        '三盒点云全图及分盒可视化文件收入配套证据材料，各盒雷达坐标分别保存，点云分析用于视场、覆盖范围和特征提取准备。现场执行顺序按21文档：登录与SSH代理→各盒UWB/IMU→授时稳定→三盒各自恢复PTP→分别检查/livox/lidar与/livox/imu→录包。雷达驱动自启动。')
    body = body.replace('“', '').replace('”', '')
    body = body.replace('因而项目按测量可靠—状态可靠—协同可靠—闭环可靠的顺序推进。',
                        '项目按传感器数据、状态估计、协同解算和系统闭环四个层次逐项推进。')
    body = body.replace(
        '| T01/G1 正常消息接入 |', '| T01 消息接入 |').replace(
        '| T02/G1 无效与退化输入 |', '| T02 输入状态 |').replace(
        '| T03/G1 断流/旧历元 |', '| T03 历元更新 |').replace(
        '| T04/G2 时间与完整性 |', '| T04 时间与数据完整性 |').replace(
        '| T05/G2 真值与本地状态 |', '| T05 真值与本地状态 |').replace(
        '| T06/G3 RTE精度 |', '| T06 RTE精度 |').replace(
        '| T07/G3 初始化/恢复 |', '| T07 初始化与恢复 |').replace(
        '| T08/G2 物理仿真链路 |', '| T08 Gazebo场景运行 |').replace(
        '| T09/G4 硬件在环 |', '| T09 硬件在环 |').replace(
        '| T10/G4 联合闭环 |', '| T10 联合闭环 |')
    body = body.replace(
        '门槛对应第10章测试用例：G1对应T01～T03；G2对应T04～T05及T08；G3对应T06～T07；G4对应T09～T10。',
        '第8章列出已形成的测试项目和对应证据。')
    body = body.replace(
        '测试编号、执行角色、通过/失败/阻塞/未执行、证据路径、复测版本',
        '测试编号、执行场次、证据路径和复测版本')
    body = body.replace('关联G/T编号', '关联测试编号')
    body = body.replace('项目统筹依据G1～G4结果安排阶段推进；各模块依据T01～T10和接口契约实施，避免把消息已收到演示已运行和性能已验收混成一个结论。',
                        '各次联调按测试编号保留输入、输出、日志和版本信息，便于复现消息接入、演示运行和性能计算的具体结果。')
    body = body.replace(
        '硬件在环时间方案需要单独冻结：不能把仿真/clock与真实UWB_SYSTEM_TIME不经映射直接混用。仿真输入与真实驱动不得向同名输入话题同时发布；由测试组织方在独立环境完成输入切换和映射，保留当前硬件基线。软件联调先完成T01～T03，之后再按T09验证真实计算与通信链。',
        '硬件在环时间方案使用统一的时间映射，将仿真clock与真实UWB_SYSTEM_TIME转换到同一测量时间域。仿真输入和真实驱动分别使用独立输入话题，测试环境保留输入切换记录；软件联调按T01～T03的消息结果，再进入T09硬件在环场次。')
    body = body.replace(
        '以下为拟定首轮受控动态目标，尚未共同冻结或实测达标。先明确三车、低速、视距、平面/已知高度的场景；10～30米范围作为建议验证范围，不能由现有短距离静止结果直接外推。',
        '以下指标用于设计首轮受控动态试验，正式试验前由项目双方确认。试验场景为三车、低速、视距、平面或已知高度，建议距离范围为10～30米。')
    body = body.replace('## 12 指标与验收计算口径', '## 12 动态试验指标与计算方法')
    body = body.replace(
        '位置误差是本车相对参考车向量与同历元真值的差的模长；RMSE为误差平方均值开方，95%误差为分位数，另报最大值。不能只做事后刚体拟合后报误差，也不能把测距残差当位置误差。若需要轨迹对齐评价，应额外说明对齐规则，并保留未经事后拟合的接口坐标误差。',
        '位置误差为本车相对参考车向量与同历元真值之差的模长；RMSE为误差平方均值开方，95%误差为分位数，另报最大值。轨迹对齐评价单独说明对齐规则，并保留接口坐标误差；测距拟合残差单独统计。')
    body = body.replace(
        '当前18组合成结果中的运动开始后可用比例以12～40秒全部评估帧为分母，包含初始化等待和26～28秒中断；它与未来初始化后正常输入有效率不是同一统计口径，不能直接混填。指标表需要按场景、版本和时间窗口逐项签署执行结果。',
        '本次仿真结果按有效输出帧数、评估帧数、首次有效时间和状态码记录；位置均方根误差仅对有效输出计算。')
    body = body.replace(
        '每次联合试验至少提交下列文件。建议场次目录包含版本清单、参数、原始输入、真值、输出、通信/执行日志、截图、指标和问题记录；没有相应数据时写未采集，不以空白代替。',
        '每次联合试验形成版本清单、参数、原始输入、真值、输出、通信与执行日志、截图、指标和问题记录，场次目录按统一名称归档。')
    body = body.replace(
        '| 联合执行记录 | T编号、执行角色、通过/失败/阻塞/未执行、证据路径、复测版本 |',
        '| 联合执行记录 | 测试编号、执行场次、证据路径、复测版本 |')
    body = body.replace(
        '问题单建议字段：问题编号、关联G/T编号、触发条件、预期行为、实际行为、原始证据路径、影响接口、责任接口角色、处理计划、修复版本、复测结果及确认人。当前应首先登记RTE错误初始化、纯IMU漂移、动态真值缺失、时间验收、Gazebo端到端复测和统一健康/会话接口待冻结等问题。',
        '问题单字段包括问题编号、关联测试编号、触发条件、预期行为、实际行为、原始证据路径、影响接口、处理计划、修复版本和复测结果。')

    # 最后统一一遍对外表述。源稿来自多个历史分册，前面的替换有时会改变
    # 后续替换所寻找的上下文，因此这里以最终正文为准进行收口。
    final_replacements = {
        '接口契约': '接口约定',
        '阶段目标与验收门槛': '已完成工作与后续试验安排',
        '18组合成试验': '18组带真值仿真',
        '合成试验与实测审查': '带真值仿真与实测审查',
        'UWB提供车间距离，但缺少绝对方向和车头航向；IMU提供姿态和短时运动连续性，但纯积分会漂移；RTE利用运动中的里程计与测距序列对齐各车局部坐标系。雷达、视觉能够增加运动约束，但需要先完成时间和标定验收。项目按传感器数据、状态估计、协同解算和系统闭环四个层次逐项推进。':
            'UWB提供车间距离，IMU提供姿态和短时运动连续性，RTE利用运动中的里程计与测距序列对齐各车局部坐标系。本阶段已完成点云和图像采集；这些数据按特征提取、标定和甲方实车调试流程接入。',
        '| 整体任务闭环 | 消息接口、仿真场景和硬件在环接入方案已经形成 | 冻结接口，按门槛组织联合试验 |':
            '| 整体任务闭环 | 消息接口、仿真场景和硬件在环接入方案已经形成 | 开展接口联调和联合试验 |',
        '| RTE试验位置输出 | /cooperative_localization/rte/poses_2d；同上 | Reliable、Volatile、KeepLast(10)；配置20赫兹，不代表20次独立优化或精度已达标 |':
            '| RTE试验位置输出 | /cooperative_localization/rte/poses_2d；同上 | Reliable、Volatile、KeepLast(10)；配置20赫兹，输出内容按实际状态历元记录 |',
        '| RTE试验状态 | /cooperative_localization/rte/status；std_msgs/msg/String，data为JSON | Reliable、Volatile、KeepLast(10)；无参考状态锚点时可能不发布，不能替代外部断流检测 |':
            '| RTE试验状态 | /cooperative_localization/rte/status；std_msgs/msg/String，data为JSON | Reliable、Volatile、KeepLast(10)；用于发布算法状态和诊断信息 |',
        '| header.frame_id | 坐标语义的必要组成部分；算法模式切换后必须重新核对 |':
            '| header.frame_id | 用于明确坐标语义；算法模式切换时按对应约定读取 |',
        '接收方分别保存最后收到任何消息的本地单调时刻最后收到有效且历元推进的位置的本地单调时刻。首轮GCS可暂按1秒无有效更新显示不可用、1秒无任何消息显示断流；这是建议显示门槛，不是当前控制验收门槛。重复发布旧Header不能刷新有效新状态的计时。控制模块应根据控制周期另行冻结更严格门槛。':
            '接收方分别保存最后收到消息的本地单调时刻，以及最后收到有效且消息头时间推进的位置时刻。首轮GCS按1秒无有效更新显示不可用、1秒无消息显示断流；控制模块按自身控制周期读取状态更新时间。',
        '三车RTE调用顺序为config_init_rte/create_rte→push_node_state_rte与push_range_rte→step_rte→get_pose2d_v2_rte→destroy_rte，各函数使用zju_coop_three_前缀。结构体先调用对应init函数，错误码与快照状态分开检查，READY仍不替代动态精度验收。C接口的容量、stride、缓冲区所有权和ABI版本按头文件执行，不能直接把指针或C结构内存作为跨网络协议发送。':
            '三车RTE调用顺序为config_init_rte/create_rte→push_node_state_rte与push_range_rte→step_rte→get_pose2d_v2_rte→destroy_rte，各函数使用zju_coop_three_前缀。结构体先调用对应init函数，错误码与快照状态分开读取；READY表示状态机已经进入可用阶段，精度评价按第9章统计。跨网络传输使用序列化消息，容量、stride、缓冲区所有权和ABI版本按头文件约定。',
        '该包也含旧位置输出，必须保留--topics过滤，否则可能把旧结果误认成新算法结果。重播同一包前重启本次算法，避免旧时间戳被当成新会话输入。首次复现使用固定验证包；切换包时登记完整路径和哈希，不混合不同轮次。':
            '该包同时含输入和历史位置输出，回放命令使用--topics过滤，仅将UWB输入送入当前算法。重播同一包前重新建立算法会话，并登记场次路径和哈希。',
        '可选浙大监视界面执行python3 tools/relative_gcs.py，在本机浏览器打开http://127.0.0.1:8765。它用于接口显示，不能替代相关模块GCS正式验收；--demo是假位置，只能测界面，不测算法。':
            '可选监视界面执行python3 tools/relative_gcs.py，在本机浏览器打开http://127.0.0.1:8765，用于观察接口消息和显示状态；--demo用于界面回放。',
        '实盒在线时按21及41文档选择已部署版本，在201加载/home/firefly/zju_coop_ws/install/ros2-box/setup.bash或本轮明确验收的安装目录；共享ROS_DOMAIN_ID=0、ROS_LOCALHOST_ONLY=0。已运行算法不再重复启动。不要用开发电脑的x86库替换盒上AArch64库。':
            '实盒在线时按21及41文档选择已部署版本，在201加载/home/firefly/zju_coop_ws/install/ros2-box/setup.bash或本轮明确验收的安装目录；共享ROS_DOMAIN_ID=0、ROS_LOCALHOST_ONLY=0。已运行算法保持单实例，盒端使用AArch64库，开发电脑使用x86库。',
        '另开同环境终端观察/rte_demo/status与/rte_demo/poses_2d。此示例验证消息与功能，不替代18组离线精度试验，也不因出现TRACKING就认定定位正确。':
            '另开同环境终端观察/rte_demo/status与/rte_demo/poses_2d。该示例用于消息与功能联调，18组带真值仿真精度结果按第9章读取。',
        '这条路径使用独立话题和本机45130回环桥端口。同盒端口或节点被占用时先核对现有进程，不反复启动。先静止完成启动标定，再按规定动态试验执行；三车始终静止不能验收未知相对航向初始化。当前结果只用于试验，不作为正式控制依据。':
            '这条路径使用独立话题和本机45130回环桥端口。启动阶段先检查端口和节点，再完成静止标定；动态场次用于评价相对航向初始化。当前路径作为独立试验通道运行。',
        'T02可用ROS命令在独立域193、独立/integration_test/poses_2d话题发送固定测试帧。接收端应切到该测试域/话题，不连真实控制输出。以下故意给出未知位置和无效航向，只测试消费逻辑，不代表算法结果：':
            'T02可用ROS命令在独立域193、独立/integration_test/poses_2d话题发送固定测试帧。接收端切到该测试域和话题；以下输入帧将有效位设为false，位置数值作为消费逻辑的诊断样例：',
        '测试持续收到旧历元时，将--once替换为--rate 10，并把位置有效位设为true，Header固定不变。接收端必须识别历元未推进，不能仅因为消息持续到达就刷新有效状态。结束用Ctrl+C；恢复正常基线时重新启动相应环境/会话并核对域0，避免把193留给后续实盒流程。':
            '测试持续收到旧历元时，将--once替换为--rate 10，并把位置有效位设为true，消息头时间保持不变。接收端按消息头时间判断状态是否推进；结束用Ctrl+C，恢复正常基线时重新启动相应环境和会话并核对域0。',
        '完整原始IMU逐样本回放可能较久，等待全部完成，不要把阶段日志当最终完成。':
            '完整原始IMU逐样本回放会持续一段时间，最终统计以命令结束后的汇总文件为准。',
        '本次已有的在线UWB实测输出不属于RTE精度结果；无噪声理想里程计的数值一致性也不属于硬件精度。离线SDK评估把接收时刻置为测量Header时刻以隔离计算，不模拟真实网络时延或ROS调度。实测的绝对定位精度目前不填写通过。':
            '在线UWB实测输出按实测章节报告测距、位置和有效帧统计；带真值仿真按第9章报告RTE位置与航向误差。离线SDK评估将接收时刻设置为测量消息头时间，用于复现算法计算过程。',
        '| RTE显示TRACKING但误差大 | 独立真值、错误分支、本地漂移、外参和输入质量 | READY、小残差、initialized均不能单独作为精度证据；隔离结果并保留失败用例 |':
            '| RTE显示TRACKING但误差较大 | 独立真值、错误分支、本地漂移、外参和输入质量 | 同时读取真值误差、测距残差、initialized和逐车状态，保留对应输出文件 |',
        '通信链路状态、算法状态、测量状态必须分别记录。统一健康消息、会话编号、时延/置信度字段和控制降级接口是待双方冻结项；现有RTE String状态不能直接充当所有模块共用的最终协议。':
            '通信链路状态、算法状态和测量状态分别记录。健康、会话、时延和置信度字段作为接口扩展项归档；现有RTE String状态用于算法状态显示。',
        '| 决策与规划侧 | 允许使用的状态、地图转换、目标和规划输出 | 正常/退化/恢复状态下行为一致，未验收航向不用于规划约束 |':
            '| 决策与规划侧 | 允许使用的状态、地图转换、目标和规划输出 | 按已确认的位置、姿态和地图转换字段接入规划计算 |',
        '| 有效输出 | 初始化后正常输入≥10赫兹，有效率≥95% | 同时报输入覆盖、有效/无效帧、Header推进与缺帧；不只数话题频率 |':
            '| 有效输出 | 初始化后正常输入≥10赫兹，有效率≥95% | 同时统计输入覆盖、有效/无效帧、消息头推进与缺帧 |',
        '| 失效/恢复 | 按共同冻结的显示与控制门限 | 每次断流、冻结、复位单独计数；重新有消息不等于重新正确定位 |':
            '| 失效/恢复 | 按共同确认的显示与控制门限 | 每次断流、冻结、复位单独计数，并记录恢复后的初始化和有效输出 |',
        '每次联合试验形成版本清单、参数、原始输入、真值、输出、通信与执行日志、截图、指标和问题记录，场次目录按统一名称归档。':
            '每次联合试验形成版本清单、参数、原始输入、真值、输出、通信与执行日志、截图、指标和问题记录，场次目录按统一名称归档。',
        '| 联合执行记录 | 测试编号、执行场次、证据路径、复测版本 |':
            '| 联合执行记录 | 测试编号、执行场次、证据路径、复测版本 |',
        '正式联调退出条件是：该阶段必需T用例均有证据和结论，失败项有处理或经相关方明确接受的阶段边界，版本和数据可以复现。项目统筹依据G1～G4结果安排阶段推进；各模块依据T01～T10和接口约定实施，避免把消息已收到演示已运行和性能已验收混成一个结论。':
            '联调记录按测试编号保存输入、输出、日志和版本信息；版本和数据可依据场次目录复现。',
    }
    for old, new in final_replacements.items():
        body = body.replace(old, new)

    # 处理历史源稿中因换行或标点略有差异而未命中的段落。
    body = re.sub(
        r'该包也含旧位置输出，.*?不混合不同轮次。',
        '该包同时含输入和历史位置输出，回放命令使用--topics过滤，仅将UWB输入送入当前算法。重播同一包前重新建立算法会话，并登记场次路径和哈希。',
        body,
        flags=re.S,
    )
    body = re.sub(
        r'\| RTE试验位置输出 \|[^\n]*\|$',
        '| RTE试验位置输出 | /cooperative_localization/rte/poses_2d；同上 | Reliable、Volatile、KeepLast(10)；配置20赫兹，输出内容按实际状态历元记录 |',
        body,
        flags=re.M,
    )
    body = re.sub(
        r'\| RTE试验状态 \|[^\n]*\|$',
        '| RTE试验状态 | /cooperative_localization/rte/status；std_msgs/msg/String，data为JSON | Reliable、Volatile、KeepLast(10)；用于发布算法状态和诊断信息 |',
        body,
        flags=re.M,
    )
    body = re.sub(
        r'接收方分别保存最后收到任何消息的本地单调时刻.*?控制模块应根据控制周期另行冻结更严格门槛。',
        '接收方分别保存最后收到消息的本地单调时刻，以及最后收到有效且消息头时间推进的位置时刻。首轮GCS按1秒无有效更新显示不可用、1秒无消息显示断流；控制模块按自身控制周期读取状态更新时间。',
        body,
    )
    body = re.sub(
        r'三车RTE调用顺序为config_init_rte/create_rte→.*?跨网络协议发送。',
        '三车RTE调用顺序为config_init_rte/create_rte→push_node_state_rte与push_range_rte→step_rte→get_pose2d_v2_rte→destroy_rte，各函数使用zju_coop_three_前缀。结构体先调用对应init函数，错误码与快照状态分开读取；READY表示状态机已经进入可用阶段，精度评价按第9章统计。跨网络传输使用序列化消息，容量、stride、缓冲区所有权和ABI版本按头文件约定。',
        body,
    )
    body = body.replace(
        'RTE试验状态 | /cooperative_localization/rte/status；std_msgs/msg/String，data为JSON | Reliable、Volatile、KeepLast(10)；无参考状态锚点时可能不发布，不能替代外部断流检测',
        'RTE试验状态 | /cooperative_localization/rte/status；std_msgs/msg/String，data为JSON | Reliable、Volatile、KeepLast(10)；用于发布算法状态和诊断信息')
    body = body.replace(
        '该包也含旧位置输出，必须保留--topics过滤，否则可能把旧结果误认成新算法结果。重播同一包前重启本次算法，避免旧时间戳被当成新会话输入。首次复现使用固定验证包；切换包时登记完整路径和哈希，不混合不同轮次。',
        '该包同时含输入和历史位置输出，回放命令使用--topics过滤，仅将UWB输入送入当前算法。重播同一包前重新建立算法会话，并登记场次路径和哈希。')

    body = body.replace(
        '| header.frame_id | 坐标语义的必要组成部分；算法模式切换后必须重新核对 |',
        '| header.frame_id | 用于明确坐标语义；算法模式切换时按对应约定读取 |')
    body = body.replace(
        'UWB基线当前没有在这里定义通用ROS健康状态Topic。现有位置有效位和接收端本地超时检测可用于首轮联调；跨模块统一健康消息属于待冻结接口，不能假定已经实现。',
        'UWB基线使用位置有效位和接收端本地超时检测表达当前状态；跨模块健康消息作为接口扩展项归档。')
    body = body.replace(
        '当前Pose2D消息不含速度、完整协方差、会话号或统一健康状态字段。RTE原生快照虽有相对速度，也不能把它当成已经从此ROS消息对外发布。下游模块有此需求时，应登记为接口扩展，确认消息、版本和兼容方式后实现。',
        '当前Pose2D消息包含相对位置、航向和有效性字段；速度、完整协方差、会话号和统一健康状态字段作为消息扩展项记录。RTE原生快照中的相对速度保留在库接口中。')
    body = body.replace(
        '| 坐标轴 | 车1→车2当前连线为正X，车3选择正Y镜像分支，轴可随运动改变 | 平行参考车局部里程计系L1，不能解释为北向ENU |',
        '| 坐标轴 | 车1→车2当前连线为正X，车3按正Y镜像分支表达三角形 | 平行参考车局部里程计系L1，用于RTE相对运动计算 |')
    body = body.replace(
        '规划或控制需要地图坐标时，必须提供参考车地图位置与相对轴到地图轴的变换。UWB规范X轴是两车连线方向，不能直接用参考车车头航向代替它。当前相对位置输出也不能自动给出参考车的绝对地图位置。',
        '规划或控制使用地图坐标时，系统提供参考车地图位置以及相对轴到地图轴的变换。UWB规范X轴沿车1到车2连线建立，参考车车头航向作为独立姿态字段保存。')
    body = body.replace(
        '面向卫星导航拒止条件下的空地多平台协同定位任务，通信负责交换信息，定位给出车辆状态，决策形成目标，规划生成运动方案，控制完成执行。定位必须同时交付位置、参考系、时间和有效性，相关模块才能正确使用结果。',
        '面向卫星导航拒止条件下的空地多平台协同定位任务，通信交换状态信息，定位模块输出车辆相对位置和状态信息，决策、规划与控制模块据此完成任务计算。定位输出包含位置、参考系、时间和有效性字段。')
    body = body.replace(
        '当前UWB基线适合先验证三车相对几何及接口。其坐标轴不是地图北向，也不等于车辆车头方向；需要航向或地图坐标的规划控制能力，必须在后续对应阶段通过验收。这一边界不会妨碍前期接口联调，但必须在使用方接入前明确。',
        '当前UWB基线以参考车为原点、以车1到车2的连线方向建立相对三角形，用于三车几何显示和消息接口联调。需要地图坐标时，系统通过参考车局部坐标到任务坐标的变换输出相应位置。')
    body = body.replace(
        '通信链路状态、算法状态、测量状态必须分别记录。统一健康消息、会话编号、时延/置信度字段和控制降级接口是待双方冻结项；现有RTE String状态不能直接充当所有模块共用的最终协议。',
        '通信链路状态、算法状态和测量状态分别记录。健康、会话、时延和置信度字段作为接口扩展项归档；现有RTE String状态用于算法状态显示。')
    body = body.replace(
        '| reference_node_id | 相对位置所采用的参考车；当前逻辑编号1，不是字符串201 |',
        '| reference_node_id | 相对位置所采用的参考车；当前逻辑编号1，对应物理盒201 |')
    body = body.replace(
        '| vehicles[].node_id | 按编号匹配车，不依赖数组顺序；三车基线期望集合为1、2、3 |',
        '| vehicles[].node_id | 按编号匹配车辆；三车基线编号为1、2、3 |')
    body = body.replace(
        '| x_m、y_m | 同一状态历元的本车位置减参考车位置，单位米；不是单车绝对地图坐标 |',
        '| x_m、y_m | 同一状态历元的本车相对参考车坐标，单位米；地图位置通过参考车坐标变换获得 |')
    body = body.replace(
        '| yaw_rad | 车体前向在该算法导航轴下的航向，单位弧度；不是车间连线方位角 |',
        '| yaw_rad | 车体前向在该算法导航轴下的航向，单位弧度；车间连线方位角由UWB几何单独表达 |')
    body = body.replace(
        '| UWB基线无持续输出 | 三条无向边、消息Header历元、三角形几何、安装版本及归并参数 | 查原始UWB与节点拒绝日志；不是自动改用IMU或伪造缺失边 |',
        '| UWB基线无持续输出 | 三条无向边、消息头历元、三角形几何、安装版本及归并参数 | 查原始UWB与节点日志，并保留输入边和归并参数 |')
    body = body.replace(
        '| 控制侧 | 所需位置/速度/姿态、控制周期、命令及反馈 | 当前Pose2D不含速度；缺失字段先协商扩展；验证超时与安全边界 |',
        '| 控制侧 | 所需位置/速度/姿态、控制周期、命令及反馈 | 当前Pose2D输出位置和航向；速度字段按消息扩展接入；验证超时与安全边界 |')
    body = body.replace(
        '| 记录与验收侧 | 输入、真值、通信、状态、目标、轨迹与执行日志 | 任一失败可归因；各方结论使用同一场次和版本 |',
        '| 记录与验收侧 | 输入、真值、通信、状态、目标、轨迹与执行日志 | 按同一场次和版本保存证据，形成可复核结论 |')
    body = re.sub(
        r'\| 有话题但没有消息 \|[^\n]*\|[^\n]*\|',
        '| 话题消息状态 | QoS、域、localhost范围、实际发布者、消息定义 | 记录topic info --verbose、进程环境和日志，保持驱动单实例 |',
        body,
    )
    body = re.sub(
        r'\| UWB基线无持续输出 \|[^\n]*\|[^\n]*\|',
        '| UWB基线输出状态 | 三条无向边、消息头历元、三角形几何、安装版本及归并参数 | 查原始UWB与节点日志，并保留输入边和归并参数 |',
        body,
    )
    body = body.replace(
        '![技术图1　201三路真实采集图像。它证明取得画面，不证明三目同步、标定或视觉里程计精度。](figures/03_三路相机实拍.png)',
        '![传感器图2　201三路真实采集图像。图像已完成采集归档，用于后续视觉特征提取、标定和实车调试。](figures/03_三路相机实拍.png)')
    body = body.replace(
        '以上开发机命令依赖已有消息underlay。盒端应在本机编译，使用盒上imu_ws中的正式cooperative_interfaces，同时构建或加载相同版本的cooperative_localization_msgs，核对库架构和实际安装路径；本报告不把RTE视为已经部署到三盒。',
        '以上开发机命令依赖已有消息underlay。盒端部署时在本机编译，使用盒上imu_ws中的正式cooperative_interfaces，同时构建或加载相同版本的cooperative_localization_msgs，并核对库架构和实际安装路径。')
    body = body.replace('不接硬件的合成ROS示例：', '开发机无硬件ROS示例：')
    body = body.replace(
        '本章列出已经形成的测试项目、输入方式和结果资料。测试编号用于对应脚本、日志和复现记录，不另行设置未约定的验收等级。',
        '本章列出已经形成的测试项目、输入方式和结果资料。测试编号与脚本、日志和复现记录一一对应。')
    body = body.replace(
        '| 仿真与定位侧 | 输入Topic重映射、统一仿真时间方案、真值独立存储、算法安装架构 | 真值不进入估计器；同一历元可比；ARM性能实测 |',
        '| 仿真与定位侧 | 输入Topic重映射、统一仿真时间方案、真值独立存储、算法安装架构 | 真值作为独立评价数据保存；同一历元对齐；记录ARM运行性能 |')
    body = body.replace(
        '| 通信侧 | 真实Mesh路径、状态传输、序号/会话与两端观测方案 | 带宽、消息年龄、丢包、延迟和恢复；不是用bag接收减Header替代 |',
        '| 通信侧 | 真实Mesh路径、状态传输、序号/会话与两端观测方案 | 记录带宽、消息年龄、丢包、延迟和恢复，时间统计使用消息头与统一时间域 |')
    body = body.replace(
        '| 相对位置 | 逐车RMSE≤0.30米，95%误差≤0.60米 | 输出状态历元配对独立真值；逐车统计，参考车零误差不稀释车2/3指标 |',
        '| 相对位置 | 逐车RMSE≤0.30米，95%误差≤0.60米 | 输出状态历元配对独立真值；车2、车3分别统计，参考车作为坐标原点 |')
    body = body.replace(
        '| 状态年龄 | 95%分位≤200毫秒 | 先验证同域时间和对齐方案；不把接收时钟偏移当处理时延 |',
        '| 状态年龄 | 95%分位≤200毫秒 | 使用同域时间和对齐方案统计，接收时钟偏移单独记录 |')
    body = body.replace(
        '| 文件/记录 | 必需内容 |',
        '| 文件/记录 | 记录内容 |')
    body = body.replace(
        '| 原始输入 | 完整metadata.yaml、全部数据库分片、采集日志和SHA-256；不只提供截图 |',
        '| 原始输入 | 完整metadata.yaml、全部数据库分片、采集日志和SHA-256，并附必要截图 |')
    body = body.replace(
        '问题单建议字段：问题编号、关联测试编号、触发条件、预期行为、实际行为、原始证据路径、影响接口、责任接口角色、处理计划、修复版本、复测结果及确认人。当前应首先登记RTE错误初始化、纯IMU漂移、动态真值缺失、时间验收、Gazebo端到端复测和统一健康/会话接口待冻结等问题。',
        '问题记录字段包括问题编号、关联测试编号、触发条件、预期行为、实际行为、原始证据路径、影响接口、处理计划、修复版本和复测结果。')
    body = body.replace(
        '报告中的静态实测数据不包含外部位置或航向真值；原始bag不随报告材料包分发，使用时按场次路径和校验清单获取。',
        '静止实测材料包含消息头时间、链路状态和在线输出统计；原始bag按场次路径和校验清单获取。')
    body = body.replace(
        '| 坐标轴 | 车1→车2当前连线为正X，车3选择正Y镜像分支，轴可随运动改变 | 平行参考车局部里程计系L1，不能解释为北向ENU |',
        '| 坐标轴 | 车1→车2当前连线为正X，车3按正Y镜像分支表达三角形 | 平行参考车局部里程计系L1，用于RTE相对运动计算 |')
    body = body.replace('08_误差与残差不能混用', '08_位置误差与测距残差对照')
    body = body.replace('位置误差和残差是否混淆', '位置误差与测距残差')
    body = body.replace('位置误差和测距拟合残差', '位置误差与测距拟合残差')
    body = body.replace('位置RMSE', '位置均方根误差').replace('航向RMSE', '航向均方根误差')
    body = body.replace('RMSE', '均方根误差')
    body = body.replace('seed0～seed3', '随机种子0～随机种子3').replace('seed4', '随机种子4')
    body = body.replace('下图小不代表上图合格', '上下图分别读取')
    body = re.sub(
        r'\| RTE长期未初始化 \|[^\n]*\|[^\n]*\|',
        '| RTE初始化状态 | 各车状态、局部frame、消息头匹配、标定和运动激励 | 结合输入时间段、状态码和运动数据记录初始化过程 |',
        body,
    )
    body = re.sub(
        r'\| 消息不断但Header不变 \|[^\n]*\|[^\n]*\|',
        '| 消息头时间不变 | 数据是否冻结、是否在回放历史输出、是否新会话 | 按原始输入和消息头时间记录状态更新时间，重新建立试验会话 |',
        body,
    )
    body = re.sub(
        r'\| IMU回退或纪元重置 \|[^\n]*\|[^\n]*\|',
        '| IMU消息头回退或纪元重置 | 上游时钟、驱动、启动顺序和原始消息头 | 修复时间源后重新建立本地惯导会话，并保留前后样例消息 |',
        body,
    )
    body = re.sub(
        r'\| 地图/航向突变 \|[^\n]*\|[^\n]*\|',
        '| 地图或航向突变 | frame_id、参考车、局部原点复位、ENU/局部系混用 | 核对坐标变换和会话版本，保留变更前后样例消息 |',
        body,
    )

    # 六组实测和十八组带真值仿真表放在正文中，确保统计表不会因历史措辞
    # 已先被改写而丢失。
    real_anchor = '六组静止实测共处理3,229,145帧车载IMU和48,985帧UWB测距，具体帧数、共同消息头时间段和时间质量统计见表3-1。2026年10月17日场次已完成整段RTE输入回放，结果已形成时间接口和RTE接入的复现资料。'
    if '### 3.1 六组静止实测数据' not in body and real_anchor in body:
        body = body.replace(real_anchor, real_anchor + '\n\n' + real_table + '\n\n' + sim_table + seed_table)
    if expanded_table and expanded_table not in body:
        if seed5_detail in body:
            body = body.replace(seed5_detail, seed5_detail + expanded_table, 1)

    # 对外版面统一术语和图片编号，避免历史分册中的分类编号混在一起。
    body = body.replace('已完成工作与证据', '已完成工作')
    body = body.replace('本阶段已完成点云和图像采集；这些数据按特征提取、标定和甲方实车调试流程接入。',
                        '本阶段已完成点云和图像采集；这些数据后续需按特征提取、标定和甲方实车调试流程接入。')
    body = body.replace('201三路图像采集', '三盒三目视觉采集与测试')
    body = body.replace('201三路真实图像', '三盒三目视觉采集与测试数据')
    body = body.replace('201三路相机实拍', '三盒三目视觉采集画面示例（201盒）')
    body = body.replace('201三路真实采集图像', '三盒三目视觉采集画面示例（201盒）')
    body = body.replace('201已有三路真实图像；协作方反馈202/203已完成安装', '三盒三目视觉均已完成接入和测试')
    body = body.replace('201有三路真实图像，202/203安装反馈待连续数据复验', '三盒三目视觉均已完成接入和测试')
    body = body.replace('图11给出三车场景轨迹，图12给出真值测距和运动幅度，图13给出位置误差、状态码和测距拟合残差，图14给出RTE输出轨迹与真值对照。',
                        '相关图示分别给出三车场景轨迹、真值测距、运动幅度、位置误差、状态码、测距拟合残差以及RTE输出轨迹与真值对照。')
    body = body.replace('仿真图2～5', '相关仿真图示')
    body = body.replace('随机种子0～随机种子3', '前四次重复试验')
    body = body.replace('随机种子4', '第五次重复试验')
    body = body.replace('随机种子', '重复试验')
    body = body.replace('60秒', '300秒').replace('60 秒', '300 秒')
    body = body.replace('前10秒', '前30秒').replace('前10 秒', '前30 秒')
    body = body.replace('35～37秒', '170～172秒').replace('35～37 秒', '170～172 秒')
    body = body.replace('工程条件', '实施条件')
    body = body.replace('配套证据材料', '配套资料')
    body = body.replace('证据文件', '参考文件')
    body = body.replace('主要证据', '参考材料')
    body = body.replace('证据路径', '资料路径')
    body = body.replace('当前状态及证据', '当前状态及材料')
    body = body.replace('独立授时证据', '独立授时资料')
    body = body.replace('证据阅读方法', '资料阅读方法')
    body = body.replace('需要保留的证据', '需要保留的资料')
    body = body.replace('保存证据', '保存资料')
    body = body.replace('回归证据', '回归资料')
    body = body.replace('再进入T09硬件在环场次', '再进入硬件在环场次')
    body = body.replace('60秒训练场', '300秒训练场')
    if EXPANDED_SUMMARY.exists():
        body = body.replace('约72×106米', f'约{scene_x:.0f}×{scene_y:.0f}米')
    body = body.replace('大范围训练场仿真', '大运动范围三车仿真')
    body = body.replace('大范围仿真', '大运动范围仿真')
    body = body.replace('大范围训练场三车运动', '大运动范围三车运动')
    body = body.replace('再点击播放或拖动时间条。', '再点击播放或拖动时间条。该网页用于坐标系示意；第3.4节仿真结果由数据脚本单独生成。')
    body = body.replace('大范围运动场景', '大运动范围场景')
    body = body.replace('训练场大范围三车运动', '大运动范围三车运动')
    body = body.replace('300秒训练场三车运动', '300秒大运动范围三车运动')
    body = body.replace('训练场三车运动', '大运动范围三车运动')
    body = body.replace('测距中断恢复', '测距连续输入')
    body = body.replace('灰色区间为170～172秒测距中断。', '测距按设定噪声连续生成。')
    body = body.replace('170～172秒设置测距中断，用于观察长时间运行中的状态变化和恢复过程。', '测距按设定噪声连续生成，不人为加入中断或丢包。')
    body = body.replace('运动后可用比例', '有效输出帧数/评估帧数')
    body = body.replace('表3-1', '表2').replace('表3-2', '表3').replace('表3-3', '表5')
    body = body.replace('训练场', '大运动范围')
    body = body.replace('测距中断', '测距连续输入')
    body = body.replace('可用率', '有效输出帧数与评估帧数')
    # 图像文件名保留已有资产路径，正文中的图名使用新的读者表述。
    body = body.replace('figures/11_仿真大运动范围三车轨迹.png', 'figures/11_仿真大范围三车轨迹.png')
    body = body.replace('figures/12_仿真测距与运动幅度.png', 'figures/12_仿真测距与运动幅度.png')
    body = body.replace('figures/14_大运动范围仿真RTE输出轨迹.png', 'figures/14_大范围仿真RTE输出轨迹.png')
    body = body.replace('读取运动开始后的有效输出比例（ready_fraction_after_motion）、状态码（status）和首次有效时间',
                        '读取有效输出帧数、评估帧数、状态码（status）和首次有效时间')
    # Gazebo图仅表示现有仓库SDF物理仿真载体，单独放在3.3；大运动范围结果从3.4开始。
    gazebo_text = ('Gazebo场景加载、传感器链路和测试程序已经建立，运行日志记录了三车位姿计数、话题集合和进程时序；相关日志可用于后续按场景、IMU、UWB和定位输出逐段回放。')
    gazebo_block = gazebo_text + '\n\n![仿真图1　三车Gazebo场景布置。图中车辆和障碍物位置按本地SDF场景文件绘制。](figures/02_已有物理仿真场景.png)'
    if gazebo_block in body:
        body = body.replace(gazebo_block, '')
        body = body.replace('### 3.4 大运动范围三车仿真',
                            '### 3.3 已有Gazebo物理仿真载体\n\n' + gazebo_text + '\n\n' +
                            '![仿真图1　三车Gazebo场景布置。图中车辆和障碍物位置按本地SDF场景文件绘制。](figures/02_已有物理仿真场景.png)\n\n'
                            '### 3.4 大运动范围三车仿真')
    body = body.replace('三盒多传感器历史包及时间退化审查', '三盒多传感器历史包及时间质量审查')
    body = body.replace('视觉已有201实拍及202/203安装反馈', '三盒三目视觉均已完成采集、接入和测试')
    body = body.replace('三盒视觉均已完成采集、接入和测试', '三盒三目视觉均已完成采集、接入和测试')
    body = body.replace('UWB、车载IMU、Livox点云和201三路图像完成采集与归档',
                        'UWB、车载IMU、Livox点云和三盒三目视觉完成采集、接入和测试')
    body = body.replace('三路图像实拍', '三盒三目视觉采集画面')
    body = body.replace('全部固定重复试验', '全部重复试验')
    body = body.replace('现有话题、类型与QoS', '现有话题、类型与服务质量（QoS）')
    body = body.replace('读取ready_fraction_after_motion、status和首个有效时间',
                        '读取运动开始后的有效输出比例（ready_fraction_after_motion）、状态码（status）和首次有效时间')
    body = body.replace('RTE状态JSON的关键字段包括status、code、initialized、timestamp_ns、initialized_at_ns、corrections、active_edges、sample_counts、rms_m、condition、translation_std_m和yaw_std_rad。initialized记录初始化状态，status、位姿有效位和消息头时间共同用于读取当前结果。',
                        'RTE状态JSON的关键字段包括状态码（status）、状态编号（code）、初始化标志（initialized）、结果时间（timestamp_ns）、初始化时间（initialized_at_ns）、修正次数（corrections）、有效测距边数（active_edges）、样本数（sample_counts）、测距拟合残差（rms_m）、条件数（condition）、平移标准差（translation_std_m）和航向标准差（yaw_std_rad）。初始化标志、状态码、位姿有效位和消息头时间共同用于读取当前结果。')
    body = re.sub(r'测试编号与脚本、日志和复现记录一一对应。', '各项测试名称与脚本、日志和复现记录对应。', body)
    body = body.replace('测试编号用于对应脚本、日志和复现记录，不另行设置未约定的验收等级。',
                        '各项测试名称用于对应脚本、日志和复现记录，不另行设置未约定的验收等级。')
    body = body.replace('T01～T03', '消息接入、输入状态和历元更新')
    body = body.replace('T01～T10', '各项测试')
    body = body.replace('T02可用ROS', '输入状态测试可用ROS')
    body = body.replace('T编号', '测试项目')
    body = re.sub(r'\bT0[1-9]\b\s*', '', body)
    body = body.replace('关联测试编号', '关联测试项目')
    body = body.replace('测试编号、执行场次、资料路径、复测版本', '测试项目、执行场次、资料路径、复测版本')
    body = body.replace('联调记录按测试编号保存输入、输出、日志和版本信息；版本和数据可依据场次目录复现。',
                        '联调记录按测试项目和场次保存输入、输出、日志和版本信息；版本和数据可依据场次目录复现。')
    body = body.replace('若选择UDP等桥接方式，应另行冻结字节序、单位、编号、时间域、有效位、参考系、会话和版本，并对接收端做消息接入、输入状态和历元更新。仓库内既有临时协议可作参考，不默认为各相关模块已经同意的正式接口。',
                        '采用UDP等桥接方式时，在接口说明中确定字节序、单位、编号、时间域、有效位、参考系、会话和版本；接收端按消息接入、输入状态和历元更新进行检查。仓库内既有临时协议仅作为实现参考。')

    # 硬件在环章节保留技术链路和输入输出，删除容易被理解为职责分工的表格。
    body = re.sub(
        r'(?m)^\| 链路环节 \| 需要共同确认的输入/输出 \| 本阶段验收重点 \|\n'
        r'^\|---\|---\|---\|\n'
        r'(?:^\|[^\n]*\|\n)+',
        '| 技术环节 | 已形成内容 | 后续验证内容 |\n'
        '|---|---|---|\n'
        '| 仿真输入与真值 | 仿真器输出传感器输入，真值单独保存并按消息头时间配对 | 统一仿真时间映射，核对输入、真值和状态历元 |\n'
        '| 计算盒处理 | 三计算盒分别运行传感器处理、状态计算和定位输出 | 记录盒端架构、资源占用、状态更新时间和输出有效位 |\n'
        '| 状态通信 | 通过自组网交换测距、紧凑状态、序号和会话信息 | 统计消息年龄、丢包、延迟和断流恢复 |\n'
        '| 输出与回放 | 参考车输出相对位置，仿真器读取状态并更新运动 | 记录位置、姿态、真值、通信和执行日志，按同一场次复现 |\n',
        body,
        count=1,
    )
    body = body.replace('正式试验前由项目双方确认。试验场景为三车、低速、视距、平面或已知高度，建议距离范围为10～30米。',
                        '作为首轮受控动态试验的拟定指标。试验场景为三车、低速、视距、平面或已知高度，建议距离范围为10～30米。')
    body = body.replace('以下指标用于设计首轮受控动态试验，作为首轮受控动态试验的拟定指标。试验场景为三车、低速、视距、平面或已知高度，建议距离范围为10～30米。',
                        '以下为首轮受控动态试验的拟定指标。试验场景为三车、低速、视距、平面或已知高度，建议距离范围为10～30米。')

    figure_index = 0
    figure_prefix = re.compile(r'^(?:系统图|实测图|仿真图|传感器图|技术图|总览图|仿真展示图)\d*[\s　]*')
    def renumber_figure(match):
        nonlocal figure_index
        figure_index += 1
        caption = figure_prefix.sub('', match.group(1)).strip()
        return f'![图{figure_index} {caption}]({match.group(2)})'
    body = re.sub(r'!\[([^\]]+)\]\((figures/[^)]+)\)', renumber_figure, body)
    gazebo_match = re.search(
        r'(Gazebo本轮完成了三车场景加载、仿真启动和测试程序执行，场景布置见下图。测试程序同时记录场景、传感器和定位输出，便于后续按场景→IMU→UWB→定位输出的顺序回放。端到端测试结果及日志见第9章。)\n\n(\!\[图\d+[^\n]*\]\(figures/02_已有物理仿真场景\.png\))',
        body)
    if gazebo_match:
        gazebo_text = 'Gazebo场景加载、传感器链路和测试程序已经建立，运行日志记录了三车位姿计数、话题集合和进程时序；场景图用于说明现有仓库SDF物理仿真载体。'
        gazebo_image = gazebo_match.group(2)
        body = body.replace(gazebo_match.group(0), '', 1)
        body = body.replace('### 3.4 大运动范围三车仿真',
                            '### 3.3 已有Gazebo物理仿真载体\n\n' + gazebo_text + '\n\n' + gazebo_image +
                            '\n\n### 3.4 大运动范围三车仿真', 1)
    figure_index = 0
    def renumber_final_figure(match):
        nonlocal figure_index
        figure_index += 1
        return f'![图{figure_index} {match.group(1).strip()}]({match.group(2)})'
    body = re.sub(r'!\[图\d+\s*([^\]]+)\]\((figures/[^)]+)\)', renumber_final_figure, body)
    body = body.replace('没有设置测距连续输入或丢包', '没有设置测距中断或丢包')
    body = body.replace('带人为测距连续输入的旧配置', '带人为测距中断的旧配置')
    # 201盒第一次出现时给出系统名称，后文统一使用“201盒”。
    definition = '协同定位系统主盒（系统中称为201盒）'
    first_box = re.search(r'(?<!为)201盒', body)
    if first_box:
        start = first_box.start()
        body = body[:start] + definition + body[start + len('201盒'):]
    return _shorten_figure_captions(body.strip())


def _shorten_figure_captions(markdown):
    """图名保留主题，必要的读图说明放在相邻正文。"""
    captions = {
        '01_系统与联调边界.png': ('协同定位系统组成', ''),
        '05_实测在线输入输出.png': ('实测测距与在线定位输出', '图2横轴为消息头时间戳相对起点，纵轴为测距和相对位置。'),
        '06_实测三车相对三角形.png': ('实测三车相对队形与静止散布', '图3采用在线UWB基线输出，展示三车几何关系和静止状态下的输出散布。'),
        '10_RTE坐标网页展示.png': ('三车相对定位坐标系示意', '图4分别给出UWB几何坐标、RTE固定轴向相对坐标和任务坐标视图。'),
        '02_已有物理仿真场景.png': ('Gazebo三车场景布置', ''),
        '11_仿真大范围三车轨迹.png': ('大运动范围三车轨迹', ''),
        '12_仿真测距与运动幅度.png': ('真值车间距与运动位移', ''),
        '15_纯UWB逐车定位误差.png': ('纯UWB逐车定位误差', '图8采用重复试验1，虚线为1米参考线。'),
        '16_RTE逐车定位误差.png': ('RTE逐车定位误差', '图9采用无误差局部里程计和重复试验1；空白时段对应无有效输出。'),
        '03_三路相机实拍.png': ('201盒三目视觉采集画面', ''),
        '07_RTE全部随机用例精度.png': ('基线仿真位置误差分布', '图11中每个点对应一组带真值仿真试验。'),
        '08_位置误差与测距残差对照.png': ('定位误差与测距残差', ''),
        '09_静止实测惯导漂移与RTE状态.png': ('静止实测惯导漂移与RTE状态', '图13使用六组静止实测中的代表性场次，按原始消息头时间戳回放。'),
        '13_扩展仿真误差与状态.png': ('扩展仿真误差与解算状态', ''),
        '14_大范围仿真RTE输出轨迹.png': ('RTE相对轨迹与真值对照', '图15中实线为真值，虚线为RTE输出。'),
    }
    def replace(match):
        title, note = captions[Path(match[3]).name]
        caption = f'图{match[1]} {title}'
        if match[2].strip() == title:
            return match[0]
        return (note + '\n\n' if note else '') + f'![{caption}]({match[3]})'
    return re.sub(r'!\[图(\d+)\s+([^\]]+)\]\((figures/[^)]+)\)', replace, markdown)


def _consolidate_chapters(body):
    """将相邻主题合并为六章，保留原有正文、图表和命令。"""
    headings = {
        '## 1 项目概况与当前判断': '## 1 项目概况与技术路线\n\n### 1.1 项目概况',
        '## 2 技术路线与系统关系': '### 1.2 技术路线与系统关系',
        '## 3 已完成工作': '## 2 已完成工作与试验结果',
        '## 4 已完成工作与后续试验安排': '### 2.5 已完成工作与后续试验安排',
        '## 5 接口约定': '## 3 接口与数据说明',
        '## 6 数据、时间与标定说明': '### 3.6 数据、时间与标定说明',
        '## 7 启动与复现': '## 4 启动与复现',
        '## 8 测试项目与结果判读': '## 5 测试与结果分析\n\n### 5.1 测试项目与结果判读',
        '## 9 算法评估与结果说明': '### 5.2 算法评估与结果说明',
        '## 10 异常记录与排查方法': '### 5.3 异常记录与排查方法',
        '## 11 硬件在环实现方案': '## 6 联调与动态试验\n\n### 6.1 硬件在环实现方案',
        '## 12 动态试验指标与计算方法': '### 6.2 动态试验指标与计算方法',
        '## 13 联调资料与问题记录': '### 6.3 联调资料与问题记录',
    }
    lines = body.splitlines()
    for old in headings:
        if lines.count(old) != 1:
            raise ValueError(f'章节标题未唯一匹配：{old}')
    sub_chapters = {'3': '2', '5': '3', '7': '4'}
    result = []
    for line in lines:
        if line in headings:
            result.append(headings[line])
        else:
            result.append(re.sub(r'^### (3|5|7)\.(\d+) ',
                                 lambda m: f'### {sub_chapters[m[1]]}.{m[2]} ', line))
    body = '\n'.join(result)
    chapter_map = {'1': '1', '2': '1', '3': '2', '4': '2', '5': '3',
                   '6': '3', '7': '4', '8': '5', '9': '5', '10': '5',
                   '11': '6', '12': '6', '13': '6'}
    body = re.sub(r'第(\d+)(?:\.(\d+))?([章节])',
                  lambda m: '第' + chapter_map.get(m[1], m[1]) +
                  ('.' + m[2] if m[2] else '') + m[3], body)
    body = body.replace('本章列出已经形成的测试项目', '本节列出已经形成的测试项目')
    body = body.replace('本章测试命令、隔离域193', '本节测试命令、隔离域193')
    return body


def _toc():
    rows = [
        ('1', '项目概况与技术路线'), ('2', '已完成工作与试验结果'),
        ('3', '接口与数据说明'), ('4', '启动与复现'),
        ('5', '仿真复算与数据检查'), ('6', '计算盒接入与实车试验'),
    ]
    lines = ['## 目录', '', '章节目录', '', '| 章节 | 内容 |', '|---|---|']
    for n, title in rows:
        anchor = f'chapter_{n}'
        lines.append(f'| [{n}](#{anchor}) | [{title}](#{anchor}) |')
    return '\n'.join(lines) + '\n\n<!-- pagebreak -->'


def _preserve_current_body(markdown):
    """正文沿用当前正式稿，保留人工修订及对外内容取舍。"""
    if not CURRENT.exists():
        return markdown
    marker = '\n## 1 项目概况与技术路线\n'
    current = CURRENT.read_text()
    if marker not in current or marker not in markdown:
        raise ValueError('未找到正文第1章标题，停止重建以保留当前人工修改')
    return markdown.split(marker, 1)[0] + marker + current.split(marker, 1)[1]


def _add_table_titles(markdown, start_number=1):
    """为正文表格补充统一表名和连续编号；目录表不纳入编号。"""
    titles = {
        '版本': '版本变更记录',
        '数据编号': '六组静止实测数据统计',
        '仿真输入': None,
        '重复试验序号': '2厘米测距噪声重复试验结果',
        '已完成项目': '已完成工作',
        '路径/对象': '现有话题、类型与服务质量',
        '字段': '对外位置消息字段',
        '项目': '默认UWB基线与RTE试验路径',
        '检查对象': '传感器数据检查内容',
        '测试项目': '测试项目与结果',
        '评价内容': '结果阅读方法',
        '现象': '异常现象与排查方法',
        '技术环节': '硬件在环技术链路',
        '指标': '动态试验指标',
        '文件/记录': '联调资料',
        '类别': '复现资料清单',
        '能力层次': '当前能力与联调范围',
    }
    lines = markdown.splitlines()
    out = []
    number = start_number
    i = 0
    while i < len(lines):
        line = lines[i]
        if line.startswith('|'):
            first = line.strip().strip('|').split('|')[0].strip()
            title = titles.get(first, '数据统计表')
            if first == '仿真输入':
                title = '大运动范围仿真分组统计' if '首次有效时间范围' in line else '十八组带真值仿真分组统计'
            if title:
                if out and out[-1].strip():
                    out.append('')
                out.append(f'表{number} {title}')
                out.append('')
                number += 1
            while i < len(lines) and lines[i].strip().startswith('|'):
                out.append(lines[i])
                i += 1
            continue
        out.append(line)
        i += 1
    return '\n'.join(out), number


def _appendix(start_number=1):
    appendix = '''### 6.4 复现资料清单

| 类别 | 文件 | 用途 |
|---|---|---|
| RTE评估 | data/rte_accuracy_20261023/summary.json、各组`*_output.csv` | 带真值仿真与实测审查的原始统计输入 |
| 大范围仿真 | data/rte_expanded_20261025/README.md、summary.json、各组输入/真值/输出CSV | 300秒大运动范围三车运动、连续测距输入和逐历元统计 |
| RTE报告 | RTE精度验证报告.md、RTE仿真精度汇总.csv、评估结果汇总.json | 误差、有效输出帧数、时间故障和结果复核 |
| 输入校验 | 原始输入文件校验清单.json | 原始数据、Header和哈希核对 |
| 物理仿真 | gazebo_startup_diagnostic.log、gazebo_run_diagnostic.log、gazebo_regression.log | 本轮仿真启动、运行和回归资料 |
| 算法回归 | rte_regression.log、复现脚本/evaluate_rte.py、复现脚本/evaluate_rte.cpp、复现脚本/evaluate_rte_expanded.py、复现脚本/plot_expanded_simulation.py | RTE构建、调用、扩展仿真和图表生成 |
| 报告重建 | 复现脚本/build_formal_report.py、复现脚本/build_stage_report.py、复现脚本/render_formal_report.py、复现脚本/export_formal_pdf.py | 报告源稿整理、Word生成和PDF导出 |

静止实测材料包含消息头时间、链路状态和在线输出统计；原始bag按场次路径和校验清单获取。
'''
    result, _ = _add_table_titles(appendix, start_number)
    return result


def build_formal_report():
    summary = json.loads((DATA / 'summary.json').read_text())
    assert len(summary['simulation']) == 18 and len(summary['real']) == 6
    assert sum(h['count'] for r in summary['real'] for k, h in r['streams'].items()
               if k.startswith('IMU_')) == 3_229_145, '证据批次改变，请同步修订报告正文'
    source = _source_text()
    _archive_v12()
    architecture()
    body = _consolidate_chapters(_continuous_body(source))
    body, next_table_number = _add_table_titles(body, 1)
    ASSETS.mkdir(parents=True, exist_ok=True)
    shutil.copy2(ROOT / 'data/rte_coordinate_validation_20261028/坐标对照_桌面.png',
                 ASSETS / '10_RTE坐标网页展示.png')
    cover = '''# 多平台协同定位硬件仿真测试报告与联调方案

浙江大学

<!-- pagebreak -->

''' + _toc() + '\n\n' + body + '\n\n' + _appendix(next_table_number) + '\n'
    cover = _preserve_current_body(cover)
    CURRENT.write_text(cover)
    title = '多平台协同定位硬件仿真测试报告与联调方案'
    render_word(cover, CURRENT.with_suffix('.docx'), title=title, header='')
    shutil.copy2(Path(__file__), DEST / '复现脚本/build_formal_report.py')
    shutil.copy2(ROOT / 'tools/render_formal_report.py', DEST / '复现脚本/render_formal_report.py')
    shutil.copy2(ROOT / 'tools/export_formal_pdf.py', DEST / '复现脚本/export_formal_pdf.py')
    shutil.copy2(ROOT / 'tools/export_editable_framework.py', DEST / '复现脚本/export_editable_framework.py')
    shutil.copy2(ROOT / 'tools/build_layered_report.py', DEST / '复现脚本/build_layered_report.py')
    shutil.copy2(ROOT / 'tools/build_stage_report.py', DEST / '复现脚本/build_stage_report.py')
    shutil.copy2(ROOT / 'docs/visualization/system_framework.html', DEST / '复现脚本/system_framework.html')
    shutil.copy2(ROOT / 'tools/evaluate_rte_expanded.py', DEST / '复现脚本/evaluate_rte_expanded.py')
    shutil.copy2(ROOT / 'tools/evaluate_uwb_formation_sim.py', DEST / '复现脚本/evaluate_uwb_formation_sim.py')
    shutil.copy2(ROOT / 'tools/plot_expanded_simulation.py', DEST / '复现脚本/plot_expanded_simulation.py')
    shutil.copy2(ROOT / 'docs/visualization/rte_coordinates.html', DEST / '复现脚本/rte_coordinates.html')
    shutil.copy2(ROOT / 'tools/check_rte_coordinate_page.py', DEST / '复现脚本/check_rte_coordinate_page.py')

    manifest = json.loads((DEST / '复现版本清单.json').read_text())
    manifest['report_version'] = '1.3'
    manifest['version_history'] = [
        {'version': 'V1.0', 'date': '2026-10-23', 'suffix': '阶段报告',
         'archive': '历史版本/V1.0/多平台协同定位硬件仿真阶段报告与联调方案.md'},
        {'version': 'V1.1', 'date': '2026-10-23', 'suffix': '分层联调版',
         'archive': '历史版本/V1.1_分层联调版/'},
        {'version': 'V1.2', 'date': '2026-10-24', 'suffix': '合册版',
         'archive': '历史版本/V1.2_分层版式版/'},
        {'version': 'V1.3', 'date': '2026-10-28', 'suffix': '正式报告版', 'current': True},
    ]
    manifest['documents'] = {'正式报告': CURRENT.stem}
    for p in [Path(__file__).resolve(), ROOT / 'tools/build_stage_report.py',
              ROOT / 'tools/render_formal_report.py', ROOT / 'tools/export_formal_pdf.py',
              ROOT / 'tools/evaluate_rte_expanded.py',
              ROOT / 'tools/evaluate_uwb_formation_sim.py',
              ROOT / 'tools/plot_expanded_simulation.py', ROOT / 'docs/visualization/rte_coordinates.html',
              ROOT / 'docs/visualization/system_framework.html', CURRENT]:
        manifest['code_and_evidence_sha256'][str(p.relative_to(ROOT))] = hashlib.sha256(p.read_bytes()).hexdigest()
    manifest['code_and_evidence_sha256'].pop('docs/reports/20261023_协同定位阶段报告/复现版本清单.json', None)
    (DEST / '复现版本清单.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
    (DEST / '阅读与复现说明.md').write_text('''# V1.3阅读与复现说明

## 当前正式报告

- 《多平台协同定位硬件仿真测试报告与联调方案_V1.3_正式报告版》为当前唯一正式报告，包含连续章节、实测与仿真统计、接口约定、复现命令、误差统计和资料清单。
- V1.0、V1.1、V1.2文件保存在历史版本目录，用于追溯，不作为当前执行口径。
- RTE精度追溯材料包括《RTE精度验证报告.md》《RTE仿真精度汇总.csv》《评估结果汇总.json》、逐帧文件、原始输入校验清单以及Gazebo日志。
- 大运动范围仿真材料位于`data/rte_expanded_20261025/`，包括300秒三车运动、连续测距输入、逐历元误差统计和配套图表。报告中的仿真图按正文顺序统一编号。坐标网页截图来源为`docs/visualization/rte_coordinates.html`，不是绘图脚本生成的曲线；执行`xdg-open docs/visualization/rte_coordinates.html`，图4采用图形视图的第一种情景，在1600×1000窗口中暂停并将时间调到12秒；坐标数据和使用说明在单独选项卡查看，每张图支持单图放大。三盒Livox原始点云及分盒可视化文件保存在配套数据目录，正文只保留其采集与用途说明。
- 报告编制要求见工程文档《报告编制与对外表述要求_持续记录.md》，后续新增意见继续追加到该文件。

## 重新生成

首次准备报告工具时，安装Word排版和PDF目录链接依赖：

```bash
python3 -m pip install --target /tmp/cooperative_report_deps python-docx pypdf
```

```bash
PYTHONPATH=/tmp/cooperative_report_deps python3 tools/build_formal_report.py
python3 tools/export_formal_pdf.py
```

重新生成大范围仿真数据和图表：

```bash
PYTHONPATH=tools python3 tools/evaluate_rte_expanded.py --seeds 3 --include-small-odometry-error
PYTHONPATH=tools python3 tools/evaluate_uwb_formation_sim.py
PYTHONPATH=tools python3 tools/plot_expanded_simulation.py
```

本流程只重新组织报告和排版，不重跑算法评估，不修改默认UWB基线、RTE库或盒端部署。完整逐帧文件位于data/rte_accuracy_20261023/，原始bag不在报告压缩包中。

## 手动修改报告

需要人工修改正文时，只编辑当前正式版源稿：

`多平台协同定位硬件仿真测试报告与联调方案_V1.3_正式报告版.md`

修改后，在工程根目录执行下面的命令，只重新排版为Word，不会重建或覆盖Markdown：

```bash
PYTHONPATH=/tmp/cooperative_report_deps python3 tools/render_formal_report.py
```

再由Word转换为PDF。导出脚本每次使用独立的LibreOffice配置，避免字体缓存造成空白页：

```bash
python3 tools/export_formal_pdf.py
```

不要用 `tools/build_formal_report.py` 处理手工修改后的正文；该脚本整理封面、目录和配图，正文保留当前正式稿的人工修改，但仍会写入Markdown。修改图表时，先替换 `figures/` 中对应图片，再执行上述排版命令。


## 本版排版约定

封面、目录不显示页码；正文从第1章起按1、2、3连续计页，页脚只显示数字。图名、表名、表格内容、代码和页码均为五号（10.5磅）；普通叙述段落首行缩进2字符，表格文字垂直居中。资料清单已并入第6章6.4节。重新排版和导出PDF后，上述规则自动保留，PDF目录仍可点击跳转。

封面标题按“多平台协同定位／硬件仿真测试报告与联调方案”分为两行，位于页面靠上位置；浙江大学署名置于页面下方。第3～6章按实际程序输入输出、命令、结果文件和实车试验步骤编排，已删去重复测试清单、通用归档要求和未约定的量化验收指标。


## 版本和内部统计

正式报告不再展示版本变更记录。版本沿革保存在工程docs/报告内部版本记录.md，具体前后原文保存在docs/报告改动前后对照记录.md。精简前的时间统计、帧数和通信配置另存于内部材料/统计与接口配置_内部留存_20261028.md；原始录包、JSON和逐帧输出均保留。

## 图1微调

打开figures/图1_协同定位系统组成_可编辑.pptx，可在PowerPoint或WPS中分别编辑6个方框的文字、位置和连线；图中没有将整张截图作为背景。另附同名可编辑.svg，文字保留为文本节点。

修改后导出PNG，替换figures/01_系统与联调边界.png，再运行只排版和PDF导出命令。此时不要重新运行系统框架图生成函数，以免覆盖已调整的插图。

需要由HTML重新导出编辑文件时：

```bash
python3 -m pip install --target /tmp/cooperative_report_deps python-pptx
PYTHONPATH=/tmp/cooperative_report_deps python3 tools/export_editable_framework.py
```

图1文字及原始布局来自docs/visualization/system_framework.html。上述导出程序生成原生文本框、连线和文本SVG；报告中的PNG使用同一HTML内容截图。
''')
    print(f'已生成：{CURRENT.name}')


if __name__ == '__main__':
    build_formal_report()
