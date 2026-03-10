#!/usr/bin/env python3
"""
PX4 基准性能数据采集器

功能：
- 订阅 PX4 原生导航算法的输出
- 订阅 Gazebo 真值数据
- 记录两者进行离线对比分析
- 支持实时监控和统计信息

使用：
    ros2 run kf_gins_ros2_native record_px4_baseline.py \
      --ros-args \
      -p output_file:=/tmp/px4_baseline_results.csv
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import csv
import os
from datetime import datetime
from pathlib import Path

# ROS 2 消息类型
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import Imu, NavSatFix
from gazebo_msgs.msg import ModelStates
from tf2_geometry_msgs import do_transform_point
import tf2_ros

import numpy as np
from scipy.spatial.transform import Rotation


class PX4BaselineRecorder(Node):
    """记录 PX4 导航性能的数据采集节点"""
    
    def __init__(self):
        super().__init__('px4_baseline_recorder')
        
        # 参数
        self.declare_parameter('output_file', '/tmp/px4_baseline_results.csv')
        self.output_file = self.get_parameter('output_file').value
        
        # 确保输出目录存在
        output_dir = os.path.dirname(self.output_file)
        if output_dir and not os.path.exists(output_dir):
            os.makedirs(output_dir)
        
        # 初始化数据存储
        self.data = []
        self.start_time = None
        
        # 创建 CSV 文件并写入表头
        self.csv_file = open(self.output_file, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self._write_csv_header()
        
        # 订阅 QoS 配置（使用最佳实践）
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # 当前数据容器
        self.current_imu = None
        self.current_gps = None
        self.current_px4_pose = None
        self.current_px4_twist = None
        self.gazebo_true_pose = None
        
        # 订阅 PX4 输出
        self.sub_imu = self.create_subscription(
            Imu, '/imu/data',
            self._on_imu, qos_profile)
        
        self.sub_gps = self.create_subscription(
            NavSatFix, '/gps/fix',
            self._on_gps, qos_profile)
        
        self.sub_px4_pose = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose',
            self._on_px4_pose, qos_profile)
        
        self.sub_px4_twist = self.create_subscription(
            TwistStamped, '/mavros/local_position/velocity_local',
            self._on_px4_twist, qos_profile)
        
        # 订阅 Gazebo 真值
        self.sub_gazebo = self.create_subscription(
            ModelStates, '/gazebo/model_states',
            self._on_gazebo_states, qos_profile)
        
        # 定时器：每 100ms 记录一次（10Hz 采样率）
        self.timer = self.create_timer(0.1, self._record_data)
        
        # 统计信息
        self.record_count = 0
        self.imu_count = 0
        self.gps_count = 0
        self.pose_count = 0
        
        self.get_logger().info(
            f'PX4 Baseline Recorder 启动\n'
            f'  输出文件：{self.output_file}\n'
            f'  采样频率：10 Hz\n'
            f'  等待数据...')
    
    def _write_csv_header(self):
        """写入 CSV 表头"""
        header = [
            # 时间戳
            'timestamp',
            
            # IMU 数据
            'imu_accel_x', 'imu_accel_y', 'imu_accel_z',
            'imu_gyro_x', 'imu_gyro_y', 'imu_gyro_z',
            
            # GPS 数据
            'gps_lat', 'gps_lon', 'gps_alt',
            
            # PX4 估计值
            'px4_pos_x', 'px4_pos_y', 'px4_pos_z',
            'px4_vel_x', 'px4_vel_y', 'px4_vel_z',
            'px4_roll', 'px4_pitch', 'px4_yaw',
            
            # Gazebo 真值
            'gazebo_pos_x', 'gazebo_pos_y', 'gazebo_pos_z',
            'gazebo_vel_x', 'gazebo_vel_y', 'gazebo_vel_z',
            'gazebo_roll', 'gazebo_pitch', 'gazebo_yaw',
            
            # 误差 (PX4 - 真值)
            'error_pos_x', 'error_pos_y', 'error_pos_z',
            'error_vel_x', 'error_vel_y', 'error_vel_z',
            'error_roll', 'error_pitch', 'error_yaw',
            
            # 误差指标
            'pos_error_norm', 'vel_error_norm', 'attitude_error_norm'
        ]
        self.csv_writer.writerow(header)
        self.csv_file.flush()
    
    def _on_imu(self, msg: Imu):
        """处理 IMU 数据"""
        self.current_imu = msg
        self.imu_count += 1
    
    def _on_gps(self, msg: NavSatFix):
        """处理 GPS 数据"""
        self.current_gps = msg
        self.gps_count += 1
    
    def _on_px4_pose(self, msg: PoseStamped):
        """处理 PX4 位置数据"""
        self.current_px4_pose = msg
        self.pose_count += 1
    
    def _on_px4_twist(self, msg: TwistStamped):
        """处理 PX4 速度数据"""
        self.current_px4_twist = msg
    
    def _on_gazebo_states(self, msg: ModelStates):
        """处理 Gazebo 状态数据"""
        # 查找 iris 无人机的索引（通常是 "iris" 或带编号的名称）
        try:
            # 在名称列表中查找包含 "iris" 的模型
            iris_idx = None
            for idx, name in enumerate(msg.name):
                if 'iris' in name.lower():
                    iris_idx = idx
                    break
            
            if iris_idx is not None:
                pose = msg.pose[iris_idx]
                twist = msg.twist[iris_idx]
                
                # 记录真值位置、速度和姿态
                self.gazebo_true_pose = {
                    'position': [pose.position.x, pose.position.y, pose.position.z],
                    'orientation': [pose.orientation.x, pose.orientation.y, 
                                   pose.orientation.z, pose.orientation.w],
                    'linear_velocity': [twist.linear.x, twist.linear.y, twist.linear.z],
                    'angular_velocity': [twist.angular.x, twist.angular.y, twist.angular.z]
                }
        except Exception as e:
            self.get_logger().warn(f'处理 Gazebo 状态失败：{e}')
    
    def _record_data(self):
        """定时记录数据"""
        # 检查是否所有必要的数据都已收到
        if not all([self.current_imu, self.current_px4_pose, self.gazebo_true_pose]):
            return
        
        # 初始化开始时间
        if self.start_time is None:
            self.start_time = self.get_clock().now().nanoseconds
        
        # 计算相对时间（秒）
        current_time_ns = self.get_clock().now().nanoseconds
        elapsed_time = (current_time_ns - self.start_time) / 1e9
        
        # 构建数据行
        row = [elapsed_time]
        
        # IMU 数据
        row.extend([
            self.current_imu.linear_acceleration.x,
            self.current_imu.linear_acceleration.y,
            self.current_imu.linear_acceleration.z,
            self.current_imu.angular_velocity.x,
            self.current_imu.angular_velocity.y,
            self.current_imu.angular_velocity.z
        ])
        
        # GPS 数据（如果可用）
        if self.current_gps:
            row.extend([
                self.current_gps.latitude,
                self.current_gps.longitude,
                self.current_gps.altitude
            ])
        else:
            row.extend([0.0, 0.0, 0.0])
        
        # PX4 估计值
        px4_pos = [
            self.current_px4_pose.pose.position.x,
            self.current_px4_pose.pose.position.y,
            self.current_px4_pose.pose.position.z
        ]
        
        px4_vel = [0.0, 0.0, 0.0]
        if self.current_px4_twist:
            px4_vel = [
                self.current_px4_twist.twist.linear.x,
                self.current_px4_twist.twist.linear.y,
                self.current_px4_twist.twist.linear.z
            ]
        
        # 从四元数转换为欧拉角 (PX4)
        px4_quat = self.current_px4_pose.pose.orientation
        px4_rot = Rotation.from_quat([px4_quat.x, px4_quat.y, px4_quat.z, px4_quat.w])
        px4_euler = px4_rot.as_euler('xyz', degrees=False)  # radians
        px4_euler_deg = np.degrees(px4_euler)  # convert to degrees
        
        row.extend(px4_pos)
        row.extend(px4_vel)
        row.extend(px4_euler_deg.tolist())
        
        # Gazebo 真值
        gazebo_pos = self.gazebo_true_pose['position']
        gazebo_vel = self.gazebo_true_pose['linear_velocity']
        
        # 从四元数转换为欧拉角 (Gazebo)
        quat = self.gazebo_true_pose['orientation']
        gazebo_rot = Rotation.from_quat(quat)
        gazebo_euler = gazebo_rot.as_euler('xyz', degrees=False)
        gazebo_euler_deg = np.degrees(gazebo_euler)
        
        row.extend(gazebo_pos)
        row.extend(gazebo_vel)
        row.extend(gazebo_euler_deg.tolist())
        
        # 计算误差
        pos_error = np.array(px4_pos) - np.array(gazebo_pos)
        vel_error = np.array(px4_vel) - np.array(gazebo_vel)
        attitude_error = px4_euler_deg - gazebo_euler_deg
        
        row.extend(pos_error.tolist())
        row.extend(vel_error.tolist())
        row.extend(attitude_error.tolist())
        
        # 误差范数
        pos_error_norm = np.linalg.norm(pos_error)
        vel_error_norm = np.linalg.norm(vel_error)
        attitude_error_norm = np.linalg.norm(attitude_error)
        
        row.extend([pos_error_norm, vel_error_norm, attitude_error_norm])
        
        # 写入 CSV
        self.csv_writer.writerow(row)
        self.csv_file.flush()
        
        self.record_count += 1
        
        # 每 10 条记录输出一次统计
        if self.record_count % 10 == 0:
            self.get_logger().info(
                f'已记录 {self.record_count} 条数据 | '
                f'时间: {elapsed_time:.1f}s | '
                f'位置误差: {pos_error_norm:.3f}m | '
                f'速度误差: {vel_error_norm:.3f}m/s')
    
    def destroy_node(self):
        """节点销毁时的清理工作"""
        self.csv_file.close()
        self.get_logger().info(
            f'数据记录完成\n'
            f'  记录条数：{self.record_count}\n'
            f'  输出文件：{self.output_file}\n'
            f'  IMU 数据：{self.imu_count}\n'
            f'  GPS 数据：{self.gps_count}\n'
            f'  PX4 位置：{self.pose_count}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        recorder = PX4BaselineRecorder()
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
