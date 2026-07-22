"""三车IMU+测距模拟器的帧数量、节点编号、frame_id和静止比力回归测试。"""

import unittest

import imu_uwb_simulator
import zjcl_protocol as zjcl


class ImuUwbSimulatorTests(unittest.TestCase):
    """确认一次采样为每个节点产生一帧可解码的标准瞬时IMU。"""
    def test_one_tick_contains_three_valid_imu_frames(self):
        simulator = imu_uwb_simulator.ImuUwbSimulator()
        frames = simulator.generate_imu_tick(123)
        self.assertEqual(len(frames), 3)
        for expected_node, encoded in enumerate(frames, 1):
            frame = zjcl.decode_frame(encoded, udp=True)
            self.assertEqual(frame.message_type, zjcl.MSG_IMU)
            self.assertEqual(frame.source_node, expected_node)
            payload = zjcl.decode_imu_payload(frame.payload)
            self.assertEqual(payload.frame_id, "imu_link")
            self.assertAlmostEqual(payload.linear_acceleration_m_s2[2],
                                   9.80665)


if __name__ == "__main__":
    unittest.main()
