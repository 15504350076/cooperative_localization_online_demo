import unittest

import zjcl_protocol as zjcl
import uwb_simulator


class UwbSimulatorTests(unittest.TestCase):
    def test_nominal_tick_emits_three_edges_with_one_timestamp(self):
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
        self.assertTrue(all(value.valid for value in ranges.values()))
        self.assertTrue(all(not value.nlos_flag for value in ranges.values()))

    def test_seed_makes_noise_nlos_and_drop_reproducible(self):
        kwargs = dict(
            seed=2026,
            noise_std_m=0.08,
            nlos_probability=0.4,
            drop_probability=0.3,
            range_std_m=0.1,
            nlos_bias_m=0.6,
        )
        first = uwb_simulator.UwbSimulator(**kwargs)
        second = uwb_simulator.UwbSimulator(**kwargs)

        first_stream = [first.generate_tick(1000 + tick) for tick in range(10)]
        second_stream = [second.generate_tick(1000 + tick) for tick in range(10)]

        self.assertEqual(first_stream, second_stream)
        self.assertTrue(any(len(tick) < 3 for tick in first_stream))
        payloads = [
            zjcl.decode_range_payload(zjcl.decode_frame(item).payload)
            for tick in first_stream for item in tick
        ]
        self.assertTrue(any(value.nlos_flag for value in payloads))

    def test_dropped_measurements_still_advance_edge_sequences(self):
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
        defaults = dict(
            seed=1,
            noise_std_m=0.0,
            nlos_probability=0.0,
            drop_probability=0.0,
            range_std_m=0.1,
        )
        for name, value in (
            ("nlos_probability", -0.1),
            ("nlos_probability", 1.1),
            ("drop_probability", -0.1),
            ("drop_probability", 1.1),
            ("noise_std_m", -0.1),
            ("range_std_m", 0.0),
            ("nlos_bias_m", -0.1),
        ):
            kwargs = dict(defaults)
            kwargs[name] = value
            with self.subTest(name=name, value=value):
                with self.assertRaises(ValueError):
                    uwb_simulator.UwbSimulator(**kwargs)


if __name__ == "__main__":
    unittest.main(verbosity=2)
