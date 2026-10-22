#!/usr/bin/env python3
"""只发布合成原始IMU和UWB；不向算法发布位置、航向或坐标变换真值。"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from cooperative_interfaces.msg import UwbRange

ORIGINS = ((0.0, 0.0), (4.0, -1.0), (-2.0, 3.0))
HEADINGS = (0.0, 0.8, -1.2)
BASE_NS = 10_000_000_000


def motion(n, t):
    if t <= 2.0:
        return (0.0, 0.0), (0.0, 0.0), 0.0, 0.0
    t -= 2.0
    wx, wy, ax, ay = .6+.13*n, .43+.17*n, 2+.5*n, 1.5+.3*n
    return ((ax*(1-math.cos(wx*t)), ay*(math.sin(wy*t)-wy*t)),
            (ax*wx*wx*math.cos(wx*t), -ay*wy*wy*math.sin(wy*t)),
            .04*(n+1)*t, .04*(n+1))


def world(n, p):
    c, s = math.cos(HEADINGS[n]), math.sin(HEADINGS[n])
    return ORIGINS[n][0]+c*p[0]-s*p[1], ORIGINS[n][1]+s*p[0]+c*p[1]


class Simulator(Node):
    def __init__(self):
        super().__init__("three_vehicle_rte_simulator")
        self.duration = float(self.declare_parameter("duration_s", 30.0).value)
        self.dropout_start = float(self.declare_parameter("dropout_start_s", 16.0).value)
        self.dropout_end = float(self.declare_parameter("dropout_end_s", 18.0).value)
        self.imu = [self.create_publisher(Imu, f"imu_{n+1}", qos_profile_sensor_data) for n in range(3)]
        self.uwb = self.create_publisher(UwbRange, "uwb", qos_profile_sensor_data)
        self.tick = 0
        self.timer = self.create_timer(.01, self.publish)

    def publish(self):
        t = self.tick*.01
        if t > self.duration:
            self.timer.cancel()
            self.get_logger().info("Synthetic stream ended; RTE should transition to STALE.")
            return
        stamp = BASE_NS+self.tick*10_000_000
        positions = []
        for n in range(3):
            p, acceleration, yaw, rate = motion(n, t)
            positions.append(world(n, p))
            imu = Imu()
            imu.header.stamp.sec, imu.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
            imu.header.frame_id = "imu_link"
            imu.orientation_covariance[0] = -1.0
            imu.angular_velocity.z = rate
            imu.linear_acceleration.x = math.cos(yaw)*acceleration[0]+math.sin(yaw)*acceleration[1]
            imu.linear_acceleration.y = -math.sin(yaw)*acceleration[0]+math.cos(yaw)*acceleration[1]
            imu.linear_acceleration.z = 9.80665
            self.imu[n].publish(imu)
        if self.tick % 10 == 0 and not self.dropout_start < t < self.dropout_end:
            for edge, (i, j) in enumerate(((0, 1), (0, 2), (1, 2))):
                message = UwbRange()
                message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
                message.src_id, message.target_id = i+1, j+1
                message.distance = math.dist(positions[i], positions[j])+.003*math.sin(t+edge)
                self.uwb.publish(message)
        self.tick += 1


def main():
    rclpy.init()
    node = Simulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
