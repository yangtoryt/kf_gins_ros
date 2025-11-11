#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from builtin_interfaces.msg import Time as TimeMsg

from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from geometry_msgs.msg import Vector3
from std_msgs.msg import Header

import math

def parse_imu_line(line):
    # 假设 Leador-A15.txt 每行格式: t ax ay az gx gy gz
    # 其中角速度单位[deg/s]或[rad/s]你原始解析已处理；这里沿用你现有逻辑
    parts = line.strip().split()
    t = float(parts[0])
    ax, ay, az = map(float, parts[1:4])
    gx, gy, gz = map(float, parts[4:7])
    return t, ax, ay, az, gx, gy, gz

def parse_gnss_line(line):
    # 假设 GNSS-RTK.txt 每行格式: t lat lon h stdN stdE stdD
    parts = line.strip().split()
    t = float(parts[0])
    lat, lon, h = map(float, parts[1:4])
    stdN, stdE, stdD = map(float, parts[4:7])
    return t, lat, lon, h, stdN, stdE, stdD

class DatasetPlayer(Node):
    def __init__(self):
        super().__init__('dataset_player')

        self.declare_parameter('imu_file', 'imu.txt')
        self.declare_parameter('gnss_file', 'gnss.txt')
        self.declare_parameter('rate_scale', 1.0)

        imu_file = self.get_parameter('imu_file').get_parameter_value().string_value
        gnss_file = self.get_parameter('gnss_file').get_parameter_value().string_value
        self.rate_scale = float(self.get_parameter('rate_scale').value)

        with open(imu_file, 'r') as f:
            self.imu_data = [parse_imu_line(l) for l in f if l.strip() and not l.startswith('#')]
        with open(gnss_file, 'r') as f:
            self.gnss_data = [parse_gnss_line(l) for l in f if l.strip() and not l.startswith('#')]

        self.get_logger().info(f'Loaded IMU samples: {len(self.imu_data)}, GNSS samples: {len(self.gnss_data)}')

        # 发布者
        self.pub_imu  = self.create_publisher(Imu, '/imu/data', 10)
        self.pub_fix  = self.create_publisher(NavSatFix, '/gps/fix', 10)

        # 播放进度
        self.i_imu = 0
        self.i_gnss = 0

        # 以 IMU 时间步进，定时器频率取决于数据间隔与 rate_scale
        self.start_wall = self.get_clock().now()
        self.start_time = self.imu_data[0][0] if self.imu_data else 0.0

        # 使用较快的 tick，内部用 while 补足（兼容非均匀采样）
        self.timer = self.create_timer(0.002, self._tick)

    def _to_stamp(self, t_sec: float) -> TimeMsg:
        sec = int(t_sec)
        nanosec = int((t_sec - sec) * 1e9)
        return TimeMsg(sec=sec, nanosec=nanosec)

    def _tick(self):
        if self.i_imu >= len(self.imu_data) and self.i_gnss >= len(self.gnss_data):
            self.get_logger().info('Playback finished.')
            rclpy.shutdown()
            return

        # 当前“播放时间”
        elapsed_wall = (self.get_clock().now() - self.start_wall).nanoseconds * 1e-9
        t_play = self.start_time + elapsed_wall * self.rate_scale

        # 1) 发布所有 IMU 样本（时间 <= t_play）
        while self.i_imu < len(self.imu_data) and self.imu_data[self.i_imu][0] <= t_play:
            t, ax, ay, az, gx, gy, gz = self.imu_data[self.i_imu]
            msg = Imu()
            msg.header = Header()
            msg.header.stamp = self._to_stamp(t)
            msg.header.frame_id = 'base_link'

            msg.linear_acceleration.x = ax
            msg.linear_acceleration.y = ay
            msg.linear_acceleration.z = az

            msg.angular_velocity.x = gx
            msg.angular_velocity.y = gy
            msg.angular_velocity.z = gz

            # 简单给个最小协方差，防止下游报错（可按需要调整）
            msg.linear_acceleration_covariance[0] = 1e-4
            msg.linear_acceleration_covariance[4] = 1e-4
            msg.linear_acceleration_covariance[8] = 1e-4
            msg.angular_velocity_covariance[0] = 1e-4
            msg.angular_velocity_covariance[4] = 1e-4
            msg.angular_velocity_covariance[8] = 1e-4

            self.pub_imu.publish(msg)
            self.i_imu += 1

        # 2) 发布所有 GNSS 样本（时间 <= t_play）
        while self.i_gnss < len(self.gnss_data) and self.gnss_data[self.i_gnss][0] <= t_play:
            t, lat, lon, h, stdN, stdE, stdD = self.gnss_data[self.i_gnss]
            fix = NavSatFix()
            fix.header = Header()
            fix.header.stamp = self._to_stamp(t)
            fix.header.frame_id = 'map'

            fix.latitude = lat
            fix.longitude = lon
            fix.altitude = h

            # 修正点：NavSatStatus 和 9 维对角协方差
            fix.status.status = NavSatStatus.STATUS_FIX
            fix.status.service = NavSatStatus.SERVICE_GPS
            fix.position_covariance = [
                stdE*stdE, 0.0,       0.0,
                0.0,       stdN*stdN, 0.0,
                0.0,       0.0,       stdD*stdD
            ]
            fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

            self.pub_fix.publish(fix)
            self.i_gnss += 1

def main():
    rclpy.init()
    node = DatasetPlayer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
