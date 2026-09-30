#!/usr/bin/env python3
"""联合录包转为 Foxglove 本地 MCAP + 中文布局；全量保留，不校时、不滤波。

先 source ROS 和 cooperative_interfaces 的安装环境，再运行：
    python3 tools/export_joint_foxglove.py data/joint_time_实验编号
额外依赖：mcap。只写实验目录下的 Foxglove 派生文件，不改原包。
"""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import re
import struct


def schema(value):
    """为 JSON 消息嵌入字段定义，查看时不需要 ROS 或自定义消息安装包。"""
    if isinstance(value, dict):
        return {"type": "object", "properties": {k: schema(v) for k, v in value.items()}}
    if isinstance(value, list):
        return {"type": "array", "items": schema(value[0]) if value else {}}
    if isinstance(value, str):
        return {"type": "string"}
    if isinstance(value, bool):
        return {"type": "boolean"}
    # 首帧没有时间差/姿态差，用 null 而不是编造 0；后续帧是数值。
    return {"type": ["number", "null"]}


def finite_json(value):
    """JSON 不支持 NaN/Inf；将这些值置 null，而不是假装成零。"""
    if isinstance(value, dict):
        return {k: finite_json(v) for k, v in value.items()}
    if isinstance(value, list):
        return [finite_json(v) for v in value]
    return None if isinstance(value, float) and not math.isfinite(value) else value


def diagnostics(message, receive_ns, previous):
    """H为消息header时间，R为原bag记录时间，并非下载电脑的接收时间。

    previous仅来自同一分流：UWB为同录包盒、同src_id、同target_id。
    先用整数纳秒作差；R-H含时间基准差，不能直接解释成网络延迟。
    """
    stamp = message["header"]["stamp"]
    header_ns = stamp["sec"] * 10**9 + stamp["nanosec"]
    result = {
        "receive_ns": str(receive_ns), "header_ns": str(header_ns),
        "header_time_s": header_ns / 1e9,
        "receive_minus_header_s": (receive_ns - header_ns) / 1e9,
        "header_dt_ms": None, "receive_dt_ms": None,
        "attitude_step_deg": None,
    }
    if previous is not None:
        result["header_dt_ms"] = (header_ns - previous[0]) / 1e6
        result["receive_dt_ms"] = (receive_ns - previous[1]) / 1e6
    q = None
    if "orientation" in message:
        values = [message["orientation"][k] for k in ("x", "y", "z", "w")]
        norm = math.sqrt(sum(v * v for v in values))
        if math.isfinite(norm) and norm > 1e-12:
            q = [v / norm for v in values]
            if previous is not None and previous[2] is not None:
                # q 和 -q 是同一旋转；abs(dot) 排除仅符号翻转造成的假突变。
                dot = abs(sum(a * b for a, b in zip(q, previous[2])))
                result["attitude_step_deg"] = math.degrees(2 * math.acos(min(1., dot)))
    return result, (header_ns, receive_ns, q)


def layout(imu_topics, range_topics):
    """使用原生 Plot/Tab 面板；不用扩展、在线桥接或额外服务。"""
    configs = {}
    colors = ["#4E9AFF", "#FFB347", "#54D6A1", "#D78EFF", "#FF6B82", "#43CFDF"]

    def plot(title, series):
        panel = f"Plot!joint{len(configs)}"
        configs[panel] = {
            "foxglovePanelTitle": title, "xAxisVal": "timestamp", "xAxisDisplayMethod": "relative",
            "isSynced": True, "showLegend": True, "legendDisplay": "left",
            "showXAxisLabels": True, "showYAxisLabels": True, "showYAxisGridLines": True,
            "paths": [{"value": path, "label": label, "enabled": True,
                       "timestampMethod": "receiveTime", "showLine": True,
                       "color": colors[i % len(colors)]} for i, (path, label) in enumerate(series)],
        }
        return panel

    def grid(panels):
        rows = [{"direction": "row", "first": panels[i], "second": panels[i+1]}
                for i in range(0, len(panels), 2)]
        return {"direction": "column", "first": rows[0], "splitPercentage": 33.3,
                "second": {"direction": "column", "first": rows[1], "second": rows[2]}}

    def imu_series(field):
        return [(f"{t}.{field}", f"盒{box}") for box, t in sorted(imu_topics.items())]

    def uwb_series(field):
        return [(f"{t}.{field}", f"{src}→{dst}（录包盒{rec}）")
                for (rec, src, dst), t in sorted(range_topics.items())]

    def all_series(field):
        return imu_series(field) + uwb_series(field)

    def same_box_series(box, field):
        series = [(f"{imu_topics[box]}.{field}", f"盒{box} IMU")]
        series.extend((f"{topic}.{field}", f"盒{box} UWB {src}→{dst}")
                      for (_, src, dst), topic in sorted(range_topics.items())
                      if src == int(box))
        return series

    overview = [
        plot("六向 UWB 测距（m）", uwb_series("distance")),
        plot("IMU 消息时间间隔（ms）：负值为回退", imu_series("analysis.header_dt_ms")),
        plot("相邻四元数旋转角（°）：已排除 q/-q 变号", imu_series("analysis.attitude_step_deg")),
        plot("IMU 录得接收间隔（ms）：不等于硬件采样周期", imu_series("analysis.receive_dt_ms")),
        plot("UWB 相邻消息自带时刻之差 ΔH（ms，同录包盒/同方向）", uwb_series("analysis.header_dt_ms")),
        plot("IMU/UWB 录包时刻R − 消息自带时刻H（s，非纯延迟）",
             all_series("analysis.receive_minus_header_s")),
    ]
    tabs = [{"title": "异常总览", "layout": grid(overview)}]
    time_panels = [
        plot("① IMU/UWB 的 R−H（s）：同图只作时间基准检查，非纯延迟",
             all_series("analysis.receive_minus_header_s")),
        plot("② IMU/UWB完整测量时间 H（s）：sec+nanosec，异常纪元会形成尖峰",
             all_series("analysis.header_time_s")),
        plot("③ 三盒IMU相邻H之差 ΔH（ms；负值为回退）", imu_series("analysis.header_dt_ms")),
        plot("④ 六向UWB相邻H之差 ΔH（ms；负值为回退）", uwb_series("analysis.header_dt_ms")),
        plot("⑤ 三盒IMU相邻录包时刻之差 ΔR（ms）", imu_series("analysis.receive_dt_ms")),
        plot("⑥ 六向UWB相邻录包时刻之差 ΔR（ms）", uwb_series("analysis.receive_dt_ms")),
    ]
    tabs.append({"title": "IMU与UWB时间戳对比", "layout": grid(time_panels)})
    if len(imu_topics) == 3:
        same_box_panels = []
        for box in sorted(imu_topics):
            same_box_panels.extend([
                plot(f"盒{box}：IMU与本盒发出UWB的完整测量时间H（s）",
                     same_box_series(box, "analysis.header_time_s")),
                plot(f"盒{box}：IMU与本盒发出UWB的相邻测量时间差ΔH（ms）",
                     same_box_series(box, "analysis.header_dt_ms")),
            ])
        tabs.append({"title": "同盒IMU与UWB时间戳", "layout": grid(same_box_panels)})
    # 页面按任务组织，不由消息类型决定；六向测距叠在一起便于比较三盒。
    if range_topics:
        panels = [
            plot("① UWB 距离（m）；横轴＝录包时刻R减整个MCAP起点（s）", uwb_series("distance")),
            plot("② R−H（s）：录包盒记录时刻减消息自带时刻，非纯延迟",
                 uwb_series("analysis.receive_minus_header_s")),
            plot("③ ΔH（ms）：本条H−上一条H，同录包盒/同方向；负值为回退", uwb_series("analysis.header_dt_ms")),
            plot("④ ΔR（ms）：本条R−上一条R，同录包盒/同方向", uwb_series("analysis.receive_dt_ms")),
            plot("⑤ H的整数秒字段 header.stamp.sec（s，未减起点）", uwb_series("header.stamp.sec")),
            plot("⑥ H的秒内余数 header.stamp.nanosec（ns，须与⑤合看）", uwb_series("header.stamp.nanosec")),
        ]
        tabs.append({"title": "三盒 UWB 测距与时间", "layout": grid(panels)})
    for box, topic in sorted(imu_topics.items()):
        panels = [
            plot(f"盒{box} 陀螺角速度（rad/s，按 ROS 字段约定）",
                 [(f"{topic}.angular_velocity.{k}", k) for k in "xyz"]),
            plot(f"盒{box} 加速度（m/s²）", [(f"{topic}.linear_acceleration.{k}", k) for k in "xyz"]),
            plot(f"盒{box} 原始四元数（无量纲，未做坐标变换）",
                 [(f"{topic}.orientation.{k}", k) for k in "xyzw"]),
            plot(f"盒{box} 原始消息时间戳 H（sec / nanosec）", [(f"{topic}.header.stamp.sec", "H.sec"),
                 (f"{topic}.header.stamp.nanosec", "H.nanosec（须与sec合看）")]),
            plot(f"盒{box} 消息/接收间隔（ms）", [(f"{topic}.analysis.{k}_dt_ms", k) for k in ("header", "receive")]),
            plot(f"盒{box} 接收减消息时间（s）", [(f"{topic}.analysis.receive_minus_header_s", "receive - header")]),
        ]
        tabs.append({"title": f"盒{box} IMU 原始量", "layout": grid(panels)})
    configs["Tab!joint"] = {"activeTabIdx": 0, "tabs": tabs}
    return {"configById": configs, "layout": "Tab!joint", "globalVariables": {},
            "userNodes": {}, "playbackConfig": {"speed": 1}}


def export(directory):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.convert import message_to_ordereddict
    from rosidl_runtime_py.utilities import get_message
    from mcap.writer import Writer

    bags = sorted(p for p in directory.iterdir() if (p / "metadata.yaml").is_file())
    if not bags:
        raise ValueError("实验目录中没有已解压、带 metadata.yaml 的录包子目录")
    output = directory / "Foxglove"
    output.mkdir(exist_ok=True)
    channels, states, counts, imu_topics, range_topics = {}, {}, Counter(), {}, {}
    summaries = {}
    mcap_path = output / "三盒IMU与UWB全量分析.mcap"
    # 同名派生文件可重新生成；rosbag 仅以 reader 打开，不重写 header。
    with mcap_path.open("wb") as f:
        writer = Writer(f)
        writer.start()
        writer.add_metadata("时间轴说明", {
            "log_time": "各原包接收时间；跨盒时钟未校准，不代表已同步",
            "publish_time": "原包未记录真实发布时间，使用接收时间占位",
            "header": "原始字段保留；整数纳秒另存 analysis.header_ns 字符串",
            "sampling": "全量导出，无离群剔除、降采样、滤波或坐标转换",
            "invalid_numbers": "JSON不支持的NaN/Inf显示为null；原始CDR请查db3",
        })
        for bag in bags:
            match = re.search(r"_(imu|uwb)_rec_v(\d+)$", bag.name)
            if not match:
                raise ValueError(f"无法从录包目录确定盒号：{bag.name}")
            kind, recorder = match.groups()
            reader = rosbag2_py.SequentialReader()
            reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
                        rosbag2_py.ConverterOptions("", ""))
            types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
            topic_indices = Counter()
            while reader.has_next():
                topic, raw, receive = reader.read_next()
                if topic not in ("/imu/raw", "/uwb/range", "/node/id"):
                    continue
                message = message_to_ordereddict(deserialize_message(raw, types[topic]))
                # 44字节定义中rolling只作为结构占位跳过，不参与任何时间分析；
                # 必须知道它和2字节CDR对齐，才能从末尾正确读取distance。
                if topic == "/uwb/range" and len(raw) == 44:
                    endian = "<" if raw[1] == 1 else ">"
                    rolling = struct.unpack_from(endian + "H", raw, 36)[0]
                    distance = struct.unpack_from(endian + "f", raw, 40)[0]
                    message["wire_layout"] = {
                        "cdr_size_bytes": 44,
                        "alignment_padding_hex": raw[38:40].hex(),
                        "distance_redecoded_m": distance,
                        "definition": "Header,uint32 src_id,uint32 target_id,uint16 rolling,float32 distance",
                    }
                    message["rolling"] = rolling
                    message["distance"] = distance
                if topic == "/imu/raw":
                    target = f"/box{recorder}/imu_raw"
                    imu_topics[recorder] = target
                elif topic == "/uwb/range":
                    src, dst = message["src_id"], message["target_id"]
                    target = f"/recorder{recorder}/uwb/{src}_to_{dst}"
                    range_topics[(recorder, src, dst)] = target
                else:
                    target = f"/recorder{recorder}/node_id"
                if target in summaries and summaries[target]["bag"] != bag.name:
                    raise ValueError(f"重复录包流：{target}，请每盒每类型只保留一包")
                if topic != "/node/id":
                    derived, states[target] = diagnostics(message, receive, states.get(target))
                    derived.update({"source_bag": bag.name, "source_topic": topic,
                                    "topic_index": topic_indices[topic], "stream_index": counts[target]})
                    message["analysis"] = derived
                else:
                    derived = {}
                topic_indices[topic] += 1
                message = finite_json(message)
                if target not in channels:
                    sid = writer.register_schema(target, "jsonschema", json.dumps(schema(message)).encode())
                    channels[target] = writer.register_channel(target, "json", sid)
                    summaries[target] = {"bag": bag.name, "count": 0, "first_receive_ns": str(receive),
                                         "last_receive_ns": str(receive), "header_rollbacks": 0,
                                         "attitude_jumps_gt_10deg": 0}
                # 不按原始 header 排序，否则时间回退会被隐藏。
                writer.add_message(channels[target], receive, json.dumps(message, allow_nan=False).encode(),
                                   receive, sequence=counts[target])
                counts[target] += 1
                summary = summaries[target]
                summary["count"] += 1
                summary["last_receive_ns"] = str(receive)
                summary["header_rollbacks"] += (derived.get("header_dt_ms") or 0) < 0
                summary["attitude_jumps_gt_10deg"] += (derived.get("attitude_step_deg") or 0) > 10
            print(f"已全量导出：{bag.name}", flush=True)
        writer.finish()
    (output / "联合静止分析布局.json").write_text(
        json.dumps(layout(imu_topics, range_topics), ensure_ascii=False, indent=2), encoding="utf-8")
    (output / "导出核对.json").write_text(
        json.dumps(summaries, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"共 {sum(counts.values())} 条；{mcap_path}")


def self_check():
    """无 ROS 也能运行的小检查：正负四元数、180°突变、整秒回退。"""
    def msg(sec, ns, q):
        return {"header": {"stamp": {"sec": sec, "nanosec": ns}},
                "orientation": dict(zip("xyzw", q))}
    a, prev = diagnostics(msg(100, 0, [0, 0, 0, 1]), 100_000_000_000, None)
    assert a["header_dt_ms"] is None
    b, prev = diagnostics(msg(100, 1_000_000, [0, 0, 0, -1]), 100_001_000_000, prev)
    assert b["attitude_step_deg"] == 0 and b["header_dt_ms"] == 1
    c, _ = diagnostics(msg(99, 2_000_000, [1, 0, 0, 0]), 100_002_000_000, prev)
    assert c["attitude_step_deg"] == 180 and c["header_dt_ms"] == -999
    assert c["receive_dt_ms"] == 1
    assert math.isclose(c["header_time_s"], 99.002)
    # 例：设备报100ms间隔，录包记录105ms间隔；R-H增加5ms，不代表时钟差5ms。
    _, previous = diagnostics(msg(100, 0, [0, 0, 0, 1]), 200_000_000_000, None)
    timing, _ = diagnostics(msg(100, 100_000_000, [0, 0, 0, 1]), 200_105_000_000, previous)
    assert timing["header_dt_ms"] == 100 and timing["receive_dt_ms"] == 105
    assert math.isclose(timing["receive_minus_header_s"], 100.005)
    assert finite_json({"x": [math.nan, math.inf]}) == {"x": [None, None]}
    config = layout({"1": "/box1/imu_raw"}, {("3", 1, 2): "/recorder3/uwb/1_to_2"})
    tabs = config["configById"]["Tab!joint"]["tabs"]
    assert [t["title"] for t in tabs] == ["异常总览", "IMU与UWB时间戳对比",
                                           "三盒 UWB 测距与时间", "盒1 IMU 原始量"]
    assert len(config["configById"]) == 25  # 四页各六个 Plot，加一个 Tab。
    uwb_plots = list(config["configById"].values())[12:18]
    assert all(p["foxglovePanelTitle"].startswith(label) for label, p in zip("①②③④⑤⑥", uwb_plots))
    assert all(path["timestampMethod"] == "receiveTime" for p in uwb_plots for path in p["paths"])
    three_boxes = layout({str(i): f"/box{i}/imu_raw" for i in (1, 2, 3)},
                         {("3", s, d): f"/recorder3/uwb/{s}_to_{d}"
                          for s, d in ((1, 2), (1, 3), (2, 1), (2, 3), (3, 1), (3, 2))})
    three_tabs = three_boxes["configById"]["Tab!joint"]["tabs"]
    assert "同盒IMU与UWB时间戳" in [tab["title"] for tab in three_tabs]
    assert len(three_boxes["configById"]) == 43  # 六页共42个Plot，加一个Tab。
    print("自检通过：四元数等价变号/180°差分/时间回退/非有限值")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", nargs="?", type=Path)
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        self_check()
    elif args.directory:
        export(args.directory.resolve())
    else:
        parser.error("请指定已解压的 joint_time 实验目录")
