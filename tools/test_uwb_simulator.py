"""三车测距模拟器的几何、随机种子、NLOS、噪声和丢包确定性测试。"""

import unittest

import zjcl_protocol as zjcl
import uwb_simulator


class UwbSimulatorTests(unittest.TestCase):
    """确认生成器既能给出无噪声黄金数据，也能复现随机故障序列。"""
    def test_nominal_tick_emits_three_edges_with_one_timestamp(self):
        # simulator隔离零故障几何；datagrams/decoded/ranges依次承载原始批次、
        # 公共头和按边payload三层证据，避免单层断言掩盖布局风险。
        simulator = uwb_simulator.UwbSimulator(
            seed=7,
            noise_std_m=0.0,
            nlos_probability=0.0,
            drop_probability=0.0,
            range_std_m=0.1,
        )

        datagrams = simulator.generate_tick(123456789)

        self.assertEqual(len(datagrams), 3)
        decoded = [zjcl.decode_frame(item, udp=True) for item in datagrams]
        self.assertEqual({item.timestamp_ns for item in decoded}, {123456789})
        self.assertEqual(
            {(item.source_node, item.target_node) for item in decoded},
            {(1, 2), (1, 3), (2, 3)},
        )
        ranges = {
            (item.source_node, item.target_node):
            zjcl.decode_range_payload(item.payload)
            for item in decoded
        }
        self.assertEqual(ranges[(1, 2)].range_m, 3.0)
        self.assertEqual(ranges[(1, 3)].range_m, 4.0)
        self.assertEqual(ranges[(2, 3)].range_m, 5.0)
        # 两个生成式中的value均为按边解码后的RangePayload，分别审计有效位与NLOS硬判决。
        self.assertTrue(all(value.valid for value in ranges.values()))
        self.assertTrue(all(not value.nlos_flag for value in ranges.values()))

    def test_seed_makes_noise_nlos_and_drop_reproducible(self):
        # 同一seed下包含噪声、NLOS与丢包的高风险随机配置。
        kwargs = dict(
            seed=2026,
            noise_std_m=0.08,
            nlos_probability=0.4,
            drop_probability=0.3,
            range_std_m=0.1,
            nlos_bias_m=0.6,
        )
        # 两个独立实例用于证明随机状态不依赖全局random或创建顺序。
        first = uwb_simulator.UwbSimulator(**kwargs)
        second = uwb_simulator.UwbSimulator(**kwargs)

        # tick为十个递增采样时刻偏移；两条流应逐字节一致。
        first_stream = [first.generate_tick(1000 + tick) for tick in range(10)]
        second_stream = [second.generate_tick(1000 + tick) for tick in range(10)]

        self.assertEqual(first_stream, second_stream)
        # 此处tick是单个采样tick产生的数据报批次，不是上方range(10)的时间偏移整数。
        self.assertTrue(any(len(tick) < 3 for tick in first_stream))
        # 此处tick仍是单tick数据报批次，item是该批次中一条未丢弃边的数据报。
        payloads = [
            zjcl.decode_range_payload(zjcl.decode_frame(item).payload)
            for tick in first_stream for item in tick
        ]
        # value是解码后的RangePayload，用于证明可复现流确实进入NLOS分支。
        self.assertTrue(any(value.nlos_flag for value in payloads))

    def test_dropped_measurements_still_advance_edge_sequences(self):
        # 100%丢包模拟器，专测“不可见数据仍消耗序号”的缺口语义。
        simulator = uwb_simulator.UwbSimulator(
            seed=1,
            noise_std_m=0.0,
            nlos_probability=0.0,
            drop_probability=1.0,
            range_std_m=0.1,
        )
        self.assertEqual(simulator.generate_tick(1), [])
        self.assertEqual(simulator.sequence_by_edge[(1, 2)], 1)
        self.assertEqual(simulator.sequence_by_edge[(1, 3)], 1)
        self.assertEqual(simulator.sequence_by_edge[(2, 3)], 1)

    def test_rejects_invalid_probabilities_and_physical_parameters(self):
        # 各字段均合法的构造参数基线。
        defaults = dict(
            seed=1,
            noise_std_m=0.0,
            nlos_probability=0.0,
            drop_probability=0.0,
            range_std_m=0.1,
        )
        # name/value逐项覆盖越界概率、负物理量和零测量标准差。
        for name, value in (
            ("nlos_probability", -0.1),
            ("nlos_probability", 1.1),
            ("drop_probability", -0.1),
            ("drop_probability", 1.1),
            ("noise_std_m", -0.1),
            ("range_std_m", 0.0),
            ("nlos_bias_m", -0.1),
        ):
            # 仅替换一个字段的隔离风险样本。
            kwargs = dict(defaults)
            kwargs[name] = value
            with self.subTest(name=name, value=value):
                with self.assertRaises(ValueError):
                    uwb_simulator.UwbSimulator(**kwargs)


if __name__ == "__main__":
    unittest.main(verbosity=2)
