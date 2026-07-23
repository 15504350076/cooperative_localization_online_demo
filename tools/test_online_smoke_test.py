"""在线冒烟测试辅助函数、输出证据聚合和临时配置生成的单元测试。"""

import configparser
from pathlib import Path
import sys
import tempfile
import time
import unittest

import online_smoke_test
import zjcl_protocol as zjcl


class OnlineSmokeHelpersTests(unittest.TestCase):
    """验证冒烟测试不会因计数误匹配或证据不足而产生假通过。"""
    def test_range_only_smoke_uses_explicit_fallback_config_by_default(self):
        # 仅提供必需可执行路径时的解析结果，防止默认配置误切到IMU融合模式。
        args = online_smoke_test.build_argument_parser().parse_args(
            ["--online-exe", "online", "--replay-exe", "replay"]
        )
        self.assertEqual(args.config, "config/range_only_demo.ini")

    def test_summary_counter_extracts_exact_named_decimal(self):
        # 同时包含名称前缀相近计数的SUMMARY，专测完整键边界。
        summary = (
            "SUMMARY status=OK input_ranges=12 "
            "output_records_skipped=34 accepted_ranges=11"
        )
        self.assertEqual(
            online_smoke_test._summary_counter(summary, "input_ranges"), 12
        )
        self.assertEqual(
            online_smoke_test._summary_counter(
                summary, "output_records_skipped"
            ),
            34,
        )
        with self.assertRaises(AssertionError):
            online_smoke_test._summary_counter(summary, "missing")

    def test_output_evidence_requires_geometry_status_and_post_normal_alert(self):
        # 从空状态逐步组装完整在线证据的聚合器。
        evidence = online_smoke_test.OutputEvidence()
        # node_id覆盖三节点有效定位，证明集合必须完整而非任意单节点。
        for node_id in (1, 2, 3):
            # 当前节点合法且不宣称yaw/z能力的定位payload。
            payload = zjcl.encode_localization_payload(
                zjcl.LocalizationPayload(
                    float(node_id), 0.0, 0.0, 0.0,
                    0.1, 0.0, 0.1,
                    zjcl.LOCALIZATION_NORMAL,
                    True, False, False,
                    zjcl.CAPABILITY_UWB_RANGE
                    | zjcl.CAPABILITY_PLANAR_POSITION
                    | zjcl.CAPABILITY_VELOCITY,
                )
            )
            evidence.ingest(
                zjcl.encode_frame(
                    zjcl.Frame(
                        zjcl.MSG_LOCALIZATION, 0, node_id, 100,
                        node_id, 1, payload,
                    ),
                    udp=True,
                )
            )

        # 三节点三边连通可观网络payload，建立“曾正常”时序锚点。
        network = zjcl.encode_network_payload(
            zjcl.NetworkPayload(3, 3, 3, True, True,
                                zjcl.LOCALIZATION_NORMAL, 0)
        )
        evidence.ingest(
            zjcl.encode_frame(
                zjcl.Frame(zjcl.MSG_NETWORK, 0, 1, 100, 1, 0, network),
                udp=True,
            )
        )
        # ABI/版本/仅测距模式正确且accepted_ranges非零的状态payload。
        status = zjcl.encode_algorithm_status_payload(
            zjcl.AlgorithmStatusPayload(
                zjcl.ALGORITHM_STATUS_ABI_VERSION,
                zjcl.SOFTWARE_VERSION_PACKED,
                zjcl.ALGORITHM_MODE_UWB_ONLY_PLANAR,
                zjcl.ALGORITHM_RUN_RUNNING,
                60,
                2,
                1,
                500_000_000,
            )
        )
        evidence.ingest(
            zjcl.encode_frame(
                zjcl.Frame(
                    zjcl.MSG_ALGORITHM_STATUS, 0, 2, 100, 1, 0, status
                ),
                udp=True,
            )
        )
        # sequence/edge覆盖三条规范无向边，防止几何输出缺边假通过。
        for sequence, edge in enumerate(((1, 2), (1, 3), (2, 3)), 1):
            # 当前边无退化的观测统计payload。
            observation = zjcl.encode_observation_payload(
                zjcl.ObservationPayload(
                    1, 100, 2, 2, 2, 0, 0, 0,
                    0.0, 1.0, 20.0, 1.0,
                    zjcl.OBSERVATION_NORMAL,
                    zjcl.FUSION_USE_NORMAL,
                    False, 0,
                )
            )
            evidence.ingest(
                zjcl.encode_frame(
                    zjcl.Frame(
                        zjcl.MSG_OBSERVATION, 0, sequence, 100,
                        edge[0], edge[1], observation,
                    ),
                    udp=True,
                )
            )

        evidence.mark_inputs_stopped()
        # 明确位于停输标记之后的LINK_TIMEOUT活动告警payload。
        alert = zjcl.encode_alert_payload(
            zjcl.AlertPayload(
                zjcl.ALERT_CODE_NETWORK_STATE,
                zjcl.ALERT_LEVEL_WARNING,
                zjcl.ALERT_LIFECYCLE_ACTIVE,
                zjcl.ALERT_SOURCE_ALGORITHM,
                1 << 5,
                0,
                0,
                0,
                100,
                200,
            )
        )
        evidence.ingest(
            zjcl.encode_frame(
                zjcl.Frame(zjcl.MSG_ALERT, 0, 3, 200, 1, 0, alert),
                udp=True,
            )
        )

        evidence.assert_complete()
        self.assertEqual(evidence.localization_nodes, {1, 2, 3})
        self.assertEqual(evidence.observation_edges,
                         {(1, 2), (1, 3), (2, 3)})
        self.assertTrue(evidence.status_seen)
        self.assertTrue(evidence.active_alert_after_good_network)
        self.assertTrue(evidence.link_timeout_alert_after_stop)

    def test_alert_before_normal_network_does_not_prove_timeout_recovery_cycle(self):
        # 专测错误时序不能满足“正常→停输→超时”的证据聚合器。
        evidence = online_smoke_test.OutputEvidence()
        # 先于正常网络出现的LINK_TIMEOUT告警，只能证明启动退化。
        alert = zjcl.encode_alert_payload(
            zjcl.AlertPayload(
                zjcl.ALERT_CODE_NETWORK_STATE,
                zjcl.ALERT_LEVEL_WARNING,
                zjcl.ALERT_LIFECYCLE_ACTIVE,
                zjcl.ALERT_SOURCE_ALGORITHM,
                1 << 5,
                0,
                0,
                0,
                100,
                100,
            )
        )
        evidence.ingest(
            zjcl.encode_frame(
                zjcl.Frame(zjcl.MSG_ALERT, 0, 1, 100, 1, 0, alert),
                udp=True,
            )
        )
        # 告警之后才出现的正常网络，不能反向赋予旧告警有效时序。
        network = zjcl.encode_network_payload(
            zjcl.NetworkPayload(
                3, 3, 3, True, True, zjcl.LOCALIZATION_NORMAL, 0
            )
        )
        evidence.ingest(
            zjcl.encode_frame(
                zjcl.Frame(zjcl.MSG_NETWORK, 0, 2, 200, 1, 0, network),
                udp=True,
            )
        )

        self.assertFalse(evidence.active_alert_after_good_network)

    def test_output_evidence_rejects_claimed_yaw_or_altitude(self):
        # 仅测距能力边界的证据聚合器。
        evidence = online_smoke_test.OutputEvidence()
        # 故意把yaw_valid置真的越界定位payload。
        payload = zjcl.encode_localization_payload(
            zjcl.LocalizationPayload(
                0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.1,
                zjcl.LOCALIZATION_NORMAL, True, True, False,
                zjcl.CAPABILITY_UWB_RANGE,
            )
        )
        with self.assertRaises(AssertionError):
            evidence.ingest(
                zjcl.encode_frame(
                    zjcl.Frame(
                        zjcl.MSG_LOCALIZATION, 0, 1, 100, 1, 1, payload
                    ),
                    udp=True,
                )
            )

    def test_temporary_config_changes_only_online_integration_values(self):
        # 同时含算法段和在线段的最小基线INI，验证算法参数不被临时化逻辑改写。
        source_text = """\
[engine]
max_nodes = 64
[online]
input_bind_address = 0.0.0.0
input_port = 39001
output_address = 127.0.0.1
output_port = 39002
event_log_enabled = false
event_log_path = old.zjlg
"""
        # directory是TemporaryDirectory生成并在退出时清理的临时目录名称字符串。
        with tempfile.TemporaryDirectory() as directory:
            # 临时基线配置、生成目标配置与嵌套事件日志路径。
            source = Path(directory) / "source.ini"
            target = Path(directory) / "target.ini"
            log = Path(directory) / "records" / "smoke.zjlg"
            source.write_text(source_text, encoding="utf-8")

            online_smoke_test.write_temporary_config(
                source, target, 41001, 41002, log
            )

            # 回读生成文件的解析器，用于逐字段审计差异范围。
            parsed = configparser.ConfigParser(interpolation=None)
            parsed.read(target, encoding="utf-8")
            self.assertEqual(parsed.getint("engine", "max_nodes"), 64)
            self.assertEqual(parsed.getint("online", "input_port"), 41001)
            self.assertEqual(parsed.getint("online", "output_port"), 41002)
            self.assertTrue(parsed.getboolean("online", "event_log_enabled"))
            self.assertEqual(
                Path(parsed.get("online", "event_log_path")), log
            )

    def test_cleanup_stops_a_running_child_process(self):
        # 故意长睡眠的真实子进程，验证清理函数不会等待自然退出。
        process = online_smoke_test._start_process(
            (sys.executable, "-c", "import time; time.sleep(30)")
        )
        # 清理耗时的单调起点，用于限制terminate/kill截止时间。
        started = time.monotonic()
        online_smoke_test._stop_process(process)
        self.assertIsNotNone(process.poll())
        self.assertLess(time.monotonic() - started, 3.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
