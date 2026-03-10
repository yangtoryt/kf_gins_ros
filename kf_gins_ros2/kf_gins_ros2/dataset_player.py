#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from builtin_interfaces.msg import Time as TimeMsg
from std_msgs.msg import Header
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from rosgraph_msgs.msg import Clock   # <<< 新增

def parse_imu_line(line: str):
    p = line.strip().split()
    t = float(p[0])
    ax, ay, az = map(float, p[1:4])
    gx, gy, gz = map(float, p[4:7])
    return t, ax, ay, az, gx, gy, gz

def parse_gnss_line(line: str):
    p = line.strip().split()
    t = float(p[0])
    lat, lon, h = map(float, p[1:4])
    stdN, stdE, stdD = map(float, p[4:7])
    return t, lat, lon, h, stdN, stdE, stdD

class DatasetPlayer(Node):
    def __init__(self):
        super().__init__('dataset_player')

        self.declare_parameter('imu_file', 'imu.txt')
        self.declare_parameter('gnss_file', 'gnss.txt')
        self.declare_parameter('rate_scale', 1.0)
        self.declare_parameter('imu_units', 'deg')  # 'deg' or 'rad'

        imu_file = self.get_parameter('imu_file').get_parameter_value().string_value
        gnss_file = self.get_parameter('gnss_file').get_parameter_value().string_value
        self.rate_scale = float(self.get_parameter('rate_scale').value)
        self.imu_units = self.get_parameter('imu_units').get_parameter_value().string_value

        with open(imu_file, 'r') as f:
            self.imu_data = [parse_imu_line(l) for l in f
                             if l.strip() and not l.lstrip().startswith('#')]
        with open(gnss_file, 'r') as f:
            self.gnss_data = [parse_gnss_line(l) for l in f
                              if l.strip() and not l.lstrip().startswith('#')]

        if not self.imu_data:
            raise RuntimeError('IMU 数据为空')

        self.get_logger().info(f'Loaded IMU samples: {len(self.imu_data)}, GNSS samples: {len(self.gnss_data)}')

        # QoS
        imu_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        fix_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.pub_imu   = self.create_publisher(Imu, '/imu/data', imu_qos)
        self.pub_fix   = self.create_publisher(NavSatFix, '/gps/fix', fix_qos)
        self.pub_clock = self.create_publisher(Clock, '/clock', 10)  # <<< 新增

        # 数据时间起点（数据系）
        gnss_t0 = self.gnss_data[0][0] if self.gnss_data else float('inf')
        self.t0 = min(float(self.imu_data[0][0]), gnss_t0)

        self.i_imu = 0
        self.i_gnss = 0

        # 播放墙钟起点（只用来计算播放进度）
        self.wall_start = self.get_clock().now()
        # 5ms tick（墙钟），我们用它推进“仿真时间”
        self.timer = self.create_timer(0.005, self._tick)

    def _to_stamp(self, t_rel: float) -> TimeMsg:
        # t_rel >= 0, 单位秒；直接生成以0为起点的仿真时间戳
        if t_rel <= 0.0:
            return TimeMsg(sec=0, nanosec=0)
        sec = int(t_rel)
        nanosec = int(round((t_rel - sec) * 1e9))
        if nanosec >= 1_000_000_000:
            sec += 1
            nanosec -= 1_000_000_000
        return TimeMsg(sec=sec, nanosec=nanosec)

    def _tick(self):
        # 以墙钟推进“仿真时间”
        elapsed_wall = (self.get_clock().now() - self.wall_start).nanoseconds * 1e-9
        t_play = elapsed_wall * self.rate_scale  # 相对数据起点

        # 发布 /clock（所有 use_sim_time 的节点都会用它当 now()）
        clk = Clock()
        clk.clock = self._to_stamp(t_play)
        self.pub_clock.publish(clk)

        # 全部播放完就停
        if self.i_imu >= len(self.imu_data) and self.i_gnss >= len(self.gnss_data):
            self.get_logger().info('Playback finished.')
            self.timer.cancel()
            return

        # 1) 发送 IMU（用仿真时间戳）
        while self.i_imu < len(self.imu_data) and (self.imu_data[self.i_imu][0] - self.t0) <= t_play:
            t, ax, ay, az, gx, gy, gz = self.imu_data[self.i_imu]
            if self.imu_units.lower().startswith('deg'):
                k = math.pi / 180.0
                gx, gy, gz = gx*k, gy*k, gz*k

            msg = Imu()
            msg.header.stamp = self._to_stamp(t - self.t0)
            msg.header.frame_id = 'imu_link'
            msg.linear_acceleration.x = float(ax)
            msg.linear_acceleration.y = float(ay)
            msg.linear_acceleration.z = float(az)
            msg.angular_velocity.x = float(gx)
            msg.angular_velocity.y = float(gy)
            msg.angular_velocity.z = float(gz)
            msg.orientation_covariance[0] = -1.0
            msg.angular_velocity_covariance[0] = 1e-4
            msg.angular_velocity_covariance[4] = 1e-4
            msg.angular_velocity_covariance[8] = 1e-4
            msg.linear_acceleration_covariance[0] = 1e-4
            msg.linear_acceleration_covariance[4] = 1e-4
            msg.linear_acceleration_covariance[8] = 1e-4

            self.pub_imu.publish(msg)
            self.i_imu += 1

        # 2) 发送 GNSS（用仿真时间戳）
        while self.i_gnss < len(self.gnss_data) and (self.gnss_data[self.i_gnss][0] - self.t0) <= t_play:
            t, lat, lon, h, stdN, stdE, stdD = self.gnss_data[self.i_gnss]

            fix = NavSatFix()
            fix.header.stamp = self._to_stamp(t - self.t0)
            fix.header.frame_id = 'map'
            fix.status.status = NavSatStatus.STATUS_FIX
            fix.status.service = int(NavSatStatus.SERVICE_GPS)
            fix.latitude  = float(lat)
            fix.longitude = float(lon)
            fix.altitude  = float(h)
            fix.position_covariance = [
                float(stdE*stdE), 0.0, 0.0,
                0.0, float(stdN*stdN), 0.0,
                0.0, 0.0, float(stdD*stdD)
            ]
            fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

            self.pub_fix.publish(fix)
            self.i_gnss += 1

def main():
    rclpy.init()
    node = DatasetPlayer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
