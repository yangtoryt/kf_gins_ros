#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Imu, NavSatFix
import argparse, threading, time, math, os

IMU_COLS = 7
GNSS_COLS = 7

def parse_floats(line):
    tokens = [t for t in line.replace(',', ' ').split() if t]
    vals = []
    for t in tokens:
        try:
            vals.append(float(t))
        except Exception:
            return None
    return vals if vals else None

class KFGINSDatasetPlayer(Node):
    def __init__(self, imu_path, gnss_path, time_scale, imu_topic, gnss_topic):
        super().__init__('kf_gins_dataset_player')
        qos = QoSProfile(depth=50)
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos.history = QoSHistoryPolicy.KEEP_LAST

        self.imu_pub = self.create_publisher(Imu, imu_topic, qos)
        self.gnss_pub = self.create_publisher(NavSatFix, gnss_topic, 10)

        self.imu_path = imu_path
        self.gnss_path = gnss_path
        self.time_scale = max(1e-6, time_scale)

        self._stop = threading.Event()
        self.imu_thread = threading.Thread(target=self._imu_loop, daemon=True)
        self.gnss_thread = threading.Thread(target=self._gnss_loop, daemon=True)

        self.get_logger().info(f"Playing IMU: {imu_path}")
        self.get_logger().info(f"Playing GNSS: {gnss_path}")
        self.get_logger().info(f"time_scale: {self.time_scale}")

        self.start_time_wall = None
        self.sow0 = None

        self.imu_thread.start()
        self.gnss_thread.start()

    def destroy_node(self):
        self._stop.set()
        try:
            if self.imu_thread.is_alive():
                self.imu_thread.join(timeout=1.0)
        except Exception:
            pass
        try:
            if self.gnss_thread.is_alive():
                self.gnss_thread.join(timeout=1.0)
        except Exception:
            pass
        super().destroy_node()

    def _imu_loop(self):
        if not self.imu_path or not os.path.exists(self.imu_path):
            self.get_logger().warn("IMU file missing; skipping IMU.")
            return
        try:
            with open(self.imu_path, 'r') as f:
                for line in f:
                    if self._stop.is_set():
                        return
                    vals = parse_floats(line)
                    if not vals or len(vals) < IMU_COLS:
                        continue
                    sow, dthx, dthy, dthz, dvx, dvy, dvz = vals[:IMU_COLS]
                    if self.sow0 is None:
                        self.sow0 = sow
                        self.start_time_wall = time.time()
                    while not self._stop.is_set():
                        elapsed = time.time() - self.start_time_wall
                        sim_elapsed = (sow - self.sow0) / self.time_scale
                        if elapsed >= sim_elapsed:
                            break
                        time.sleep(0.001)
                    msg = Imu()
                    now = self.get_clock().now().to_msg()
                    msg.header.stamp = now
                    msg.header.frame_id = 'imu_link'
                    msg.angular_velocity.x = dthx
                    msg.angular_velocity.y = dthy
                    msg.angular_velocity.z = dthz
                    msg.linear_acceleration.x = dvx
                    msg.linear_acceleration.y = dvy
                    msg.linear_acceleration.z = dvz
                    self.imu_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"IMU loop error: {e}")

    def _gnss_loop(self):
        if not self.gnss_path or not os.path.exists(self.gnss_path):
            self.get_logger().warn("GNSS file missing; skipping GNSS.")
            return
        try:
            with open(self.gnss_path, 'r') as f:
                for line in f:
                    if self._stop.is_set():
                        return
                    vals = parse_floats(line)
                    if not vals or len(vals) < GNSS_COLS:
                        continue
                    sow, lat, lon, h, stdN, stdE, stdD = vals[:GNSS_COLS]
                    if self.sow0 is None:
                        self.sow0 = sow
                        self.start_time_wall = time.time()
                    while not self._stop.is_set():
                        elapsed = time.time() - self.start_time_wall
                        sim_elapsed = (sow - self.sow0) / self.time_scale
                        if elapsed >= sim_elapsed:
                            break
                        time.sleep(0.002)
                    msg = NavSatFix()
                    now = self.get_clock().now().to_msg()
                    msg.header.stamp = now
                    msg.header.frame_id = 'gps'
                    msg.latitude = lat
                    msg.longitude = lon
                    msg.altitude = h
                    msg.position_covariance[0] = float(stdE*stdE)
                    msg.position_covariance[4] = float(stdN*stdN)
                    msg.position_covariance[8] = float(stdD*stdD)
                    msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
                    self.gnss_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"GNSS loop error: {e}")

def main():
    parser = argparse.ArgumentParser(description="Play KF-GINS text-format IMU/GNSS to ROS 2 topics.")
    parser.add_argument('--imu', required=True, help='Path to IMU text file (SOW, dtheta(rad)*3, dvel(m/s)*3)')
    parser.add_argument('--gnss', required=True, help='Path to GNSS position text file (SOW, lat, lon, h, stdN, stdE, stdD)')
    parser.add_argument('--time-scale', type=float, default=1.0, help='Time scaling (1.0 = real, 2.0 = 2x faster, 0.5 = slower)')
    parser.add_argument('--imu-topic', default='/imu/data')
    parser.add_argument('--gnss-topic', default='/gps/fix_cov')
    args = parser.parse_args()

    rclpy.init()
    node = KFGINSDatasetPlayer(args.imu, args.gnss, args.time_scale, args.imu_topic, args.gnss_topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
