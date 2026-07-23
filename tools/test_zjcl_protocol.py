"""Python ZJCL编解码器的固定长度、黄金字节、CRC和异常字段回归测试。

黄金向量与C++测试共享协议口径，用于发现两种实现间的字节级不一致。
"""

import math
import struct
import unittest

import zjcl_protocol as zjcl


class ZjclProtocolTests(unittest.TestCase):
    """覆盖公共帧、全部固定载荷以及数值/枚举/保留字段边界。"""
    def test_imu_payload_is_332_bytes_and_round_trips(self):
        # 含非零角速度/加速度的IMU契约样本，防止字段顺序相同零值掩盖错位。
        value = zjcl.ImuPayload(
            (0.0, 0.0, 0.0, 1.0), (0.0,) * 9,
            (0.1, 0.2, 0.3), (0.0,) * 9,
            (1.0, 2.0, 9.80665), (0.0,) * 9,
            "imu_link", False, True, zjcl.RANGE_STATUS_OK,
        )
        # 待验证的固定332字节线缆表示。
        encoded = zjcl.encode_imu_payload(value)
        self.assertEqual(len(encoded), 332)
        self.assertEqual(zjcl.decode_imu_payload(encoded), value)

    def test_cpp_frame_golden_matches_exactly(self):
        # 与C++黄金向量共用的40字节告警payload。
        alert_payload = zjcl.encode_alert_payload(
            zjcl.AlertPayload(
                zjcl.ALERT_CODE_NETWORK_STATE,
                zjcl.ALERT_LEVEL_WARNING,
                zjcl.ALERT_LIFECYCLE_ACTIVE,
                zjcl.ALERT_SOURCE_ALGORITHM,
                0x10203040,
                0,
                1,
                2,
                0x0102030405060708,
                0x1112131415161718,
            )
        )
        # 各多字节字段均采用非对称十六进制值，以暴露端序或槽位错误。
        frame = zjcl.Frame(
            message_type=zjcl.MSG_ALERT,
            flags=0x1234,
            sequence=0x0102030405060708,
            timestamp_ns=0x1112131415161718,
            source_node=0x2233,
            target_node=0x4455,
            payload=alert_payload,
        )
        # C++实现冻结的完整80字节线缆黄金值（含公共头CRC）。
        expected = bytes.fromhex(
            "5a4a434c010067002800341228000000"
            "08070605040302011817161514131211"
            "33225544d79088a0"
            "01000000010000004030201000000100"
            "02000000080706050403020118171615"
            "1413121100000000"
        )
        # Python编码器针对同一业务帧产生的候选字节。
        encoded = zjcl.encode_frame(frame)
        self.assertEqual(encoded, expected)
        self.assertEqual(zjcl.decode_frame(expected), frame)

    def test_six_fixed_payloads_round_trip(self):
        # range_value覆盖非默认状态、布尔位和概率字段；range_bytes是其24字节黄金候选。
        range_value = zjcl.RangePayload(
            3.0, 0.25, 0.5, True, True, True, zjcl.RANGE_STATUS_INVALID
        )
        range_bytes = zjcl.encode_range_payload(range_value)
        self.assertEqual(len(range_bytes), 24)
        self.assertEqual(
            range_bytes,
            bytes.fromhex(
                "0000000000000840000000000000d03f0000003f01010102"
            ),
        )
        self.assertEqual(zjcl.decode_range_payload(range_bytes), range_value)

        # localization含负交叉协方差与能力位；localization_bytes承载64字节往返风险。
        localization = zjcl.LocalizationPayload(
            1.0, 2.0, 3.0, 4.0, 5.0, -0.5, 0.25,
            zjcl.LOCALIZATION_DEGRADED, True, False, False, 0x01020304,
        )
        localization_bytes = zjcl.encode_localization_payload(localization)
        self.assertEqual(len(localization_bytes), 64)
        self.assertEqual(zjcl.decode_localization_payload(localization_bytes),
                         localization)

        # network是三节点全连通合法边界；network_bytes覆盖20字节固定布局。
        network = zjcl.NetworkPayload(
            3, 3, 3, True, True, zjcl.LOCALIZATION_NORMAL, 0x31,
        )
        network_bytes = zjcl.encode_network_payload(network)
        self.assertEqual(len(network_bytes), 20)
        self.assertEqual(zjcl.decode_network_payload(network_bytes), network)

        # observation覆盖计数/比例/动作/溢出位，observation_bytes覆盖80字节布局。
        observation = zjcl.ObservationPayload(
            1, 2, 40, 39, 38, 3, 2, 1, 0.25, 0.95, 19.5, 4.0,
            zjcl.OBSERVATION_DEGRADED, zjcl.FUSION_USE_DOWNWEIGHTED,
            True, 0x10203040,
        )
        observation_bytes = zjcl.encode_observation_payload(observation)
        self.assertEqual(len(observation_bytes), 80)
        self.assertEqual(zjcl.decode_observation_payload(observation_bytes),
                         observation)

        # algorithm_status使用明显64位计数，status_bytes用于布局与端序黄金校验。
        algorithm_status = zjcl.AlgorithmStatusPayload(
            0x00010000,
            0x00000100,
            zjcl.ALGORITHM_MODE_UWB_ONLY_PLANAR,
            zjcl.ALGORITHM_RUN_DEGRADED,
            0x0102030405060708,
            0x1112131415161718,
            0x2122232425262728,
            0x3132333435363738,
        )
        status_bytes = zjcl.encode_algorithm_status_payload(algorithm_status)
        self.assertEqual(
            status_bytes,
            bytes.fromhex(
                "00000100000100000102000000000000"
                "08070605040302011817161514131211"
                "28272625242322213837363534333231"
            ),
        )
        self.assertEqual(
            zjcl.decode_algorithm_status_payload(status_bytes),
            algorithm_status,
        )

        # alert覆盖原因位/边端点/双时间戳，alert_bytes用于40字节黄金校验。
        alert = zjcl.AlertPayload(
            zjcl.ALERT_CODE_NETWORK_STATE,
            zjcl.ALERT_LEVEL_WARNING,
            zjcl.ALERT_LIFECYCLE_ACTIVE,
            zjcl.ALERT_SOURCE_ALGORITHM,
            0x10203040,
            0,
            1,
            2,
            0x0102030405060708,
            0x1112131415161718,
        )
        alert_bytes = zjcl.encode_alert_payload(alert)
        self.assertEqual(
            alert_bytes,
            bytes.fromhex(
                "01000000010000004030201000000100"
                "02000000080706050403020118171615"
                "1413121100000000"
            ),
        )
        self.assertEqual(zjcl.decode_alert_payload(alert_bytes), alert)

    def test_status_and_alert_reject_invalid_enums_reserved_and_lifecycle(self):
        # 可逐字节破坏的合法状态基线，先测未知模式再测非零保留字节。
        status = bytearray(
            zjcl.encode_algorithm_status_payload(
                zjcl.AlgorithmStatusPayload(
                    0x00010000, 0x00000100,
                    zjcl.ALGORITHM_MODE_UWB_ONLY_PLANAR,
                    zjcl.ALGORITHM_RUN_RUNNING,
                    1, 2, 3, 4,
                )
            )
        )
        status[8] = 0
        with self.assertRaises(zjcl.ProtocolError):
            zjcl.decode_algorithm_status_payload(status)
        status[8] = zjcl.ALGORITHM_MODE_UWB_ONLY_PLANAR
        status[10] = 1
        with self.assertRaises(zjcl.ProtocolError):
            zjcl.decode_algorithm_status_payload(status)

        # 活动告警基线，用于构造生命周期/原因位和时间顺序冲突。
        active = zjcl.AlertPayload(
            zjcl.ALERT_CODE_NETWORK_STATE,
            zjcl.ALERT_LEVEL_WARNING,
            zjcl.ALERT_LIFECYCLE_ACTIVE,
            zjcl.ALERT_SOURCE_ALGORITHM,
            1, 0, 0, 0, 10, 20,
        )
        with self.assertRaises(zjcl.ProtocolError):
            zjcl.encode_alert_payload(
                zjcl.AlertPayload(
                    active.alert_code, active.level, active.lifecycle,
                    active.source, 0, active.node_id, active.from_node,
                    active.to_node, active.first_timestamp_ns,
                    active.last_timestamp_ns,
                )
            )
        with self.assertRaises(zjcl.ProtocolError):
            zjcl.encode_alert_payload(
                zjcl.AlertPayload(
                    active.alert_code, active.level,
                    zjcl.ALERT_LIFECYCLE_CLEARED,
                    active.source, 1, active.node_id, active.from_node,
                    active.to_node, active.first_timestamp_ns,
                    active.last_timestamp_ns,
                )
            )
        # 将first_timestamp改晚于last_timestamp的恶意线缆样本。
        reversed_time = bytearray(zjcl.encode_alert_payload(active))
        reversed_time[20:28] = struct.pack("<Q", 21)
        with self.assertRaises(zjcl.ProtocolError):
            zjcl.decode_alert_payload(reversed_time)
        # 在告警首个保留字节中注入非零值的前向兼容风险样本。
        bad_reserved = bytearray(zjcl.encode_alert_payload(active))
        bad_reserved[7] = 1
        with self.assertRaises(zjcl.ProtocolError):
            zjcl.decode_alert_payload(bad_reserved)

    def test_decode_is_strict_about_crc_sizes_booleans_and_numbers(self):
        # 合法测距payload及完整UDP帧作为各类单点破坏的共同基线。
        payload = zjcl.encode_range_payload(
            zjcl.RangePayload(3.0, 0.1, 0.0, False, False, True, 0)
        )
        # 带CRC的完整合法帧。
        encoded = zjcl.encode_frame(
            zjcl.Frame(zjcl.MSG_RANGE, 0, 1, 2, 1, 2, payload), udp=True
        )
        # 翻转最后一个payload位但保留原CRC，验证完整性检测。
        corrupted = bytearray(encoded)
        corrupted[-1] ^= 1
        with self.assertRaises(zjcl.ProtocolError):
            zjcl.decode_frame(bytes(corrupted), udp=True)
        with self.assertRaises(zjcl.ProtocolError):
            zjcl.decode_frame(encoded + b"\0", udp=True)
        # 将偏移20的nlos_flag/NLOS硬判决布尔槽改为2，验证不得按“非零即真”宽松解码。
        invalid_boolean = bytearray(payload)
        invalid_boolean[20] = 2
        with self.assertRaises(zjcl.ProtocolError):
            zjcl.decode_range_payload(bytes(invalid_boolean))
        # 将range_m注入+inf，验证线缆浮点有限性约束。
        nonfinite = bytearray(payload)
        nonfinite[:8] = struct.pack("<d", math.inf)
        with self.assertRaises(zjcl.ProtocolError):
            zjcl.decode_range_payload(bytes(nonfinite))

    def test_range_status_is_the_normalized_v1_enum(self):
        with self.assertRaises(zjcl.ProtocolError):
            zjcl.encode_range_payload(
                zjcl.RangePayload(3.0, 0.1, 0.0, False, False, True, 3)
            )
        # 合法测距payload基线，随后把尾部status篡改为v1未知枚举3。
        encoded = bytearray(
            zjcl.encode_range_payload(
                zjcl.RangePayload(3.0, 0.1, 0.0, False, False, True, 0)
            )
        )
        encoded[23] = 3
        with self.assertRaises(zjcl.ProtocolError):
            zjcl.decode_range_payload(encoded)

    def test_one_mib_and_udp_limits_are_hard(self):
        self.assertEqual(zjcl.MAX_PAYLOAD_SIZE, 1024 * 1024)
        self.assertEqual(zjcl.MAX_UDP_DATAGRAM, 65507)
        # 固定48字节状态payload，用于验证公共帧总长与配置硬上限。
        payload = zjcl.encode_algorithm_status_payload(
            zjcl.AlgorithmStatusPayload(
                0x00010000, 0x00000100,
                zjcl.ALGORITHM_MODE_UWB_ONLY_PLANAR,
                zjcl.ALGORITHM_RUN_RUNNING, 0, 0, 0, 1,
            )
        )
        # UDP内合法的算法状态公共帧。
        frame = zjcl.Frame(zjcl.MSG_ALGORITHM_STATUS, 0, 1, 2, 1, 0, payload)
        # 头加状态payload的完整编码，用于接收侧上限测试。
        encoded = zjcl.encode_frame(frame, udp=True)
        self.assertEqual(len(encoded), zjcl.HEADER_SIZE + 48)
        with self.assertRaises(zjcl.ProtocolError):
            zjcl.encode_frame(
                zjcl.Frame(zjcl.MSG_ALERT, 0, 1, 2, 1, 2,
                           b"x" * (zjcl.MAX_UDP_DATAGRAM + 1)),
                udp=True,
            )
        with self.assertRaises(zjcl.ProtocolError):
            zjcl.encode_frame(
                zjcl.Frame(zjcl.MSG_ALERT, 0, 1, 2, 1, 2,
                           b"x" * (zjcl.MAX_PAYLOAD_SIZE + 1))
            )
        with self.assertRaises(zjcl.ProtocolError):
            zjcl.encode_frame(
                frame, max_payload_size=zjcl.MAX_PAYLOAD_SIZE + 1
            )
        with self.assertRaises(zjcl.ProtocolError):
            zjcl.decode_frame(
                encoded, max_payload_size=zjcl.MAX_PAYLOAD_SIZE + 1
            )

    def test_cross_field_invariants_and_extreme_covariance_are_strict(self):
        # covariance逐项覆盖负对角、负行列式、上溢和下溢尺度下的非PSD风险。
        for covariance in (
            (-1.0, 0.0, 1.0),
            (1.0, 2.0, 1.0),
            (float.fromhex("0x1.fffffffffffffp+1023"),
             float.fromhex("0x1.fffffffffffffp+1023"), 1.0),
            (float.fromhex("0x0.0000000000001p-1022"), 1.0,
             float.fromhex("0x1.fffffffffffffp+1023")),
        ):
            with self.subTest(covariance=covariance):
                with self.assertRaises(zjcl.ProtocolError):
                    zjcl.encode_localization_payload(
                        zjcl.LocalizationPayload(
                            0.0, 0.0, 0.0, 0.0,
                            covariance[0], covariance[1], covariance[2],
                            zjcl.LOCALIZATION_NORMAL,
                            True, False, False,
                            zjcl.CAPABILITY_PLANAR_POSITION,
                        )
                    )

        # 最大有限double，用于证明等元素秩一PSD矩阵不会因乘法溢出被误拒。
        maximum = float.fromhex("0x1.fffffffffffffp+1023")
        # 极端但合法的半正定定位载荷。
        extreme_psd = zjcl.LocalizationPayload(
            0.0, 0.0, 0.0, 0.0,
            maximum, maximum, maximum,
            zjcl.LOCALIZATION_NORMAL,
            True, False, False,
            zjcl.CAPABILITY_PLANAR_POSITION,
        )
        self.assertEqual(
            zjcl.decode_localization_payload(
                zjcl.encode_localization_payload(extreme_psd)
            ),
            extreme_psd,
        )

        # 依次违反可达数、最多边、连通最少边、connected一致性和observable蕴含。
        invalid_networks = (
            zjcl.NetworkPayload(
                3, 4, 2, False, False,
                zjcl.LOCALIZATION_UNOBSERVABLE, 0,
            ),
            zjcl.NetworkPayload(
                3, 3, 4, True, True,
                zjcl.LOCALIZATION_NORMAL, 0,
            ),
            zjcl.NetworkPayload(
                3, 3, 1, True, True,
                zjcl.LOCALIZATION_NORMAL, 0,
            ),
            zjcl.NetworkPayload(
                3, 2, 2, True, False,
                zjcl.LOCALIZATION_DEGRADED, 0,
            ),
            zjcl.NetworkPayload(
                3, 2, 2, False, True,
                zjcl.LOCALIZATION_UNOBSERVABLE, 0,
            ),
        )
        # value为单个违反网络交叉字段契约的样本。
        for value in invalid_networks:
            with self.assertRaises(zjcl.ProtocolError):
                zjcl.encode_network_payload(value)

        # 依次违反窗口顺序、valid/nlos/rejected相对received及rejected相对valid。
        invalid_observations = (
            zjcl.ObservationPayload(
                200, 100, 20, 18, 15, 0, 0, 2,
                0.0, 0.75, 18.0, 1.0,
                zjcl.OBSERVATION_NORMAL, zjcl.FUSION_USE_NORMAL, False, 0,
            ),
            zjcl.ObservationPayload(
                100, 200, 20, 18, 19, 0, 0, 2,
                0.0, 0.75, 18.0, 1.0,
                zjcl.OBSERVATION_NORMAL, zjcl.FUSION_USE_NORMAL, False, 0,
            ),
            zjcl.ObservationPayload(
                100, 200, 20, 18, 15, 19, 0, 2,
                0.0, 0.75, 18.0, 1.0,
                zjcl.OBSERVATION_NORMAL, zjcl.FUSION_USE_NORMAL, False, 0,
            ),
            zjcl.ObservationPayload(
                100, 200, 20, 18, 15, 0, 19, 2,
                0.0, 0.75, 18.0, 1.0,
                zjcl.OBSERVATION_NORMAL, zjcl.FUSION_USE_NORMAL, False, 0,
            ),
            zjcl.ObservationPayload(
                100, 200, 20, 18, 15, 0, 16, 2,
                0.0, 0.75, 18.0, 1.0,
                zjcl.OBSERVATION_NORMAL, zjcl.FUSION_USE_NORMAL, False, 0,
            ),
        )
        # value为单个违反观测窗口计数契约的样本。
        for value in invalid_observations:
            with self.assertRaises(zjcl.ProtocolError):
                zjcl.encode_observation_payload(value)


if __name__ == "__main__":
    unittest.main(verbosity=2)
