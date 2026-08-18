"""三车IMU+测距模拟器的帧数量、节点编号、frame_id和静止比力回归测试。"""

import unittest

import imu_uwb_simulator
import zjcl_protocol as zjcl


class ImuUwbSimulatorTests(unittest.TestCase):
    """确认一次采样为每个节点产生一帧可解码的标准瞬时IMU。"""
    def test_one_tick_contains_three_valid_imu_frames(self):
        # simulator/frames覆盖三节点批次；循环内frame/payload再分层核对公共头与IMU语义。
        simulator = imu_uwb_simulator.ImuUwbSimulator()
        frames = simulator.generate_imu_tick(123)
        self.assertEqual(len(frames), 3)
        # expected_node/encoded把列表顺序与协议源节点逐一绑定。
        for expected_node, encoded in enumerate(frames, 1):
            frame = zjcl.decode_frame(encoded, udp=True)
            self.assertEqual(frame.message_type, zjcl.MSG_IMU)
            self.assertEqual(frame.source_node, expected_node)
            payload = zjcl.decode_imu_payload(frame.payload)
            self.assertEqual(payload.frame_id, "imu_link")
            self.assertAlmostEqual(payload.linear_acceleration_m_s2[2],
                                   9.80665)

    def test_configured_z_angular_rate_remains_an_instantaneous_measurement(self):
        # 模拟器只填写瞬时z轴角速度；orientation仍无效，避免伪造连续姿态真值。
        simulator = imu_uwb_simulator.ImuUwbSimulator(
            gyro_z_rad_s=0.25
        )
        frame = zjcl.decode_frame(simulator.generate_imu_tick(123)[0], udp=True)
        payload = zjcl.decode_imu_payload(frame.payload)
        self.assertEqual(payload.angular_velocity_rad_s, (0.0, 0.0, 0.25))
        self.assertFalse(payload.orientation_valid)
        self.assertEqual(payload.orientation_xyzw, (0.0, 0.0, 0.0, 1.0))


if __name__ == "__main__":
    unittest.main()
