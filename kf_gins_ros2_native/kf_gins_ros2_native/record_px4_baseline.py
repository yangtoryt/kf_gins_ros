#!/usr/bin/env python3
"""
PX4 基准性能数据采集器。

功能：
- 订阅 PX4 原生导航算法输出
- 订阅仿真真值数据（支持多种来源）
- 记录两者用于离线对比分析
- 在输入不完整时输出明确诊断信息

使用：
    python3 record_px4_baseline.py \
      --ros-args \
      -p output_file:=/tmp/px4_baseline_results.csv \
      -p imu_topic:=/mavros/imu/data \
      -p ground_truth_source:=auto
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import csv
import os

# ROS 2 消息类型
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
from gazebo_msgs.msg import ModelStates, LinkStates

import numpy as np
from scipy.spatial.transform import Rotation


class PX4BaselineRecorder(Node):
    """记录 PX4 导航性能的数据采集节点"""
    
    def __init__(self):
        super().__init__('px4_baseline_recorder')

        # 参数
        self.declare_parameter('output_file', '/tmp/px4_baseline_results.csv')
        self.declare_parameter('imu_topic', '/mavros/imu/data')
        self.declare_parameter('gps_topic', '/gps/fix')
        self.declare_parameter('px4_pose_topic', '/mavros/local_position/pose')
        self.declare_parameter('px4_velocity_topic', '/mavros/local_position/velocity_local')
        self.declare_parameter('ground_truth_source', 'auto')
        self.declare_parameter('ground_truth_model_states_topic', '/model_states')
        self.declare_parameter('ground_truth_link_states_topic', '/link_states')
        self.declare_parameter('ground_truth_odom_topic', '')
        self.declare_parameter('ground_truth_pose_topic', '')
        self.declare_parameter('ground_truth_twist_topic', '')
        self.declare_parameter('ground_truth_model_name_contains', 'iris')
        self.declare_parameter('require_ground_truth', True)
        self.declare_parameter('write_rate_hz', 10.0)

        self.output_file = self.get_parameter('output_file').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.gps_topic = self.get_parameter('gps_topic').value
        self.px4_pose_topic = self.get_parameter('px4_pose_topic').value
        self.px4_velocity_topic = self.get_parameter('px4_velocity_topic').value
        self.ground_truth_source = self.get_parameter('ground_truth_source').value
        self.ground_truth_model_states_topic = self.get_parameter('ground_truth_model_states_topic').value
        self.ground_truth_link_states_topic = self.get_parameter('ground_truth_link_states_topic').value
        self.ground_truth_odom_topic = self.get_parameter('ground_truth_odom_topic').value
        self.ground_truth_pose_topic = self.get_parameter('ground_truth_pose_topic').value
        self.ground_truth_twist_topic = self.get_parameter('ground_truth_twist_topic').value
        self.ground_truth_model_name_contains = self.get_parameter('ground_truth_model_name_contains').value.lower()
        self.require_ground_truth = bool(self.get_parameter('require_ground_truth').value)
        self.write_rate_hz = float(self.get_parameter('write_rate_hz').value)

        # 确保输出目录存在
        output_dir = os.path.dirname(self.output_file)
        if output_dir and not os.path.exists(output_dir):
            os.makedirs(output_dir)

        # 初始化数据存储
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
        self.current_gt_pose = None
        self.current_gt_twist = None
        self.ground_truth_state = None
        self.active_gt_source = None

        # 订阅 PX4 输出
        self.sub_imu = self.create_subscription(
            Imu, self.imu_topic,
            self._on_imu, qos_profile)
        self.sub_gps = self.create_subscription(
            NavSatFix, self.gps_topic,
            self._on_gps, qos_profile)
        self.sub_px4_pose = self.create_subscription(
            PoseStamped, self.px4_pose_topic,
            self._on_px4_pose, qos_profile)
        self.sub_px4_twist = self.create_subscription(
            TwistStamped, self.px4_velocity_topic,
            self._on_px4_twist, qos_profile)

        # 订阅真值源
        self.sub_gt_model_states = None
        self.sub_gt_link_states = None
        self.sub_gt_odom = None
        self.sub_gt_pose = None
        self.sub_gt_twist = None
        self._configure_ground_truth_subscriptions(qos_profile)

        # 定时器：默认每 100ms 记录一次（10Hz）
        timer_period = 0.1 if self.write_rate_hz <= 0.0 else 1.0 / self.write_rate_hz
        self.timer = self.create_timer(timer_period, self._record_data)
        self.diag_timer = self.create_timer(5.0, self._report_input_status)

        # 统计信息
        self.record_count = 0
        self.imu_count = 0
        self.gps_count = 0
        self.pose_count = 0
        self.gt_count = 0

        self.get_logger().info(
            f'PX4 Baseline Recorder 启动\n'
            f'  输出文件：{self.output_file}\n'
            f'  IMU 话题：{self.imu_topic}\n'
            f'  PX4 位姿话题：{self.px4_pose_topic}\n'
            f'  PX4 速度话题：{self.px4_velocity_topic}\n'
            f'  真值模式：{self.ground_truth_source}\n'
            f'  采样频率：{self.write_rate_hz:.2f} Hz\n'
            f'  等待数据...')

    def _configure_ground_truth_subscriptions(self, qos_profile: QoSProfile):
        """根据配置订阅真值源。"""
        source = str(self.ground_truth_source).strip().lower()

        if source in ('auto', 'model_states') and self.ground_truth_model_states_topic:
            self.sub_gt_model_states = self.create_subscription(
                ModelStates, self.ground_truth_model_states_topic,
                self._on_model_states, qos_profile)

        if source in ('auto', 'link_states') and self.ground_truth_link_states_topic:
            self.sub_gt_link_states = self.create_subscription(
                LinkStates, self.ground_truth_link_states_topic,
                self._on_link_states, qos_profile)

        if source in ('auto', 'odom') and self.ground_truth_odom_topic:
            self.sub_gt_odom = self.create_subscription(
                Odometry, self.ground_truth_odom_topic,
                self._on_ground_truth_odom, qos_profile)

        if source in ('auto', 'pose') and self.ground_truth_pose_topic:
            self.sub_gt_pose = self.create_subscription(
                PoseStamped, self.ground_truth_pose_topic,
                self._on_ground_truth_pose, qos_profile)

        if source in ('auto', 'pose') and self.ground_truth_twist_topic:
            self.sub_gt_twist = self.create_subscription(
                TwistStamped, self.ground_truth_twist_topic,
                self._on_ground_truth_twist, qos_profile)
    
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

    def _match_name_index(self, names):
        """按模型名关键词匹配实体索引。"""
        for idx, name in enumerate(names):
            if self.ground_truth_model_name_contains and self.ground_truth_model_name_contains in name.lower():
                return idx
        return None

    def _set_ground_truth_state(self, position, orientation, linear_velocity=None, angular_velocity=None, source='unknown'):
        """统一缓存真值状态。"""
        if linear_velocity is None:
            linear_velocity = [float('nan')] * 3
        if angular_velocity is None:
            angular_velocity = [float('nan')] * 3

        self.ground_truth_state = {
            'position': list(position),
            'orientation': list(orientation),
            'linear_velocity': list(linear_velocity),
            'angular_velocity': list(angular_velocity),
        }
        self.active_gt_source = source
        self.gt_count += 1

    def _on_model_states(self, msg: ModelStates):
        """处理 Gazebo ModelStates 真值。"""
        try:
            iris_idx = self._match_name_index(msg.name)
            if iris_idx is None:
                return
            pose = msg.pose[iris_idx]
            twist = msg.twist[iris_idx]
            self._set_ground_truth_state(
                [pose.position.x, pose.position.y, pose.position.z],
                [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w],
                [twist.linear.x, twist.linear.y, twist.linear.z],
                [twist.angular.x, twist.angular.y, twist.angular.z],
                source=f'model_states:{self.ground_truth_model_states_topic}',
            )
        except Exception as e:
            self.get_logger().warn(f'处理 Gazebo ModelStates 失败：{e}')

    def _on_link_states(self, msg: LinkStates):
        """处理 Gazebo LinkStates 真值。"""
        try:
            iris_idx = self._match_name_index(msg.name)
            if iris_idx is None:
                return
            pose = msg.pose[iris_idx]
            twist = msg.twist[iris_idx]
            self._set_ground_truth_state(
                [pose.position.x, pose.position.y, pose.position.z],
                [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w],
                [twist.linear.x, twist.linear.y, twist.linear.z],
                [twist.angular.x, twist.angular.y, twist.angular.z],
                source=f'link_states:{self.ground_truth_link_states_topic}',
            )
        except Exception as e:
            self.get_logger().warn(f'处理 Gazebo LinkStates 失败：{e}')

    def _on_ground_truth_odom(self, msg: Odometry):
        """处理 Odometry 真值。"""
        pose = msg.pose.pose
        twist = msg.twist.twist
        self._set_ground_truth_state(
            [pose.position.x, pose.position.y, pose.position.z],
            [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w],
            [twist.linear.x, twist.linear.y, twist.linear.z],
            [twist.angular.x, twist.angular.y, twist.angular.z],
            source=f'odom:{self.ground_truth_odom_topic}',
        )

    def _on_ground_truth_pose(self, msg: PoseStamped):
        """处理 PoseStamped 真值。"""
        self.current_gt_pose = msg
        self._update_ground_truth_from_pose_twist()

    def _on_ground_truth_twist(self, msg: TwistStamped):
        """处理 TwistStamped 真值。"""
        self.current_gt_twist = msg
        self._update_ground_truth_from_pose_twist()

    def _update_ground_truth_from_pose_twist(self):
        """用 pose/twist 组合更新真值。"""
        if self.current_gt_pose is None:
            return

        pose = self.current_gt_pose.pose
        if self.current_gt_twist is not None:
            twist = self.current_gt_twist.twist
            linear_velocity = [twist.linear.x, twist.linear.y, twist.linear.z]
            angular_velocity = [twist.angular.x, twist.angular.y, twist.angular.z]
        else:
            linear_velocity = [float('nan')] * 3
            angular_velocity = [float('nan')] * 3

        self._set_ground_truth_state(
            [pose.position.x, pose.position.y, pose.position.z],
            [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w],
            linear_velocity,
            angular_velocity,
            source=f'pose:{self.ground_truth_pose_topic}',
        )

    def _current_missing_inputs(self):
        """返回当前缺失的输入。"""
        missing = []
        if self.current_imu is None:
            missing.append(f'IMU({self.imu_topic})')
        if self.current_px4_pose is None:
            missing.append(f'PX4 pose({self.px4_pose_topic})')
        if self.require_ground_truth and self.ground_truth_state is None:
            gt_desc = []
            if self.ground_truth_model_states_topic:
                gt_desc.append(self.ground_truth_model_states_topic)
            if self.ground_truth_link_states_topic:
                gt_desc.append(self.ground_truth_link_states_topic)
            if self.ground_truth_odom_topic:
                gt_desc.append(self.ground_truth_odom_topic)
            if self.ground_truth_pose_topic:
                gt_desc.append(self.ground_truth_pose_topic)
            missing.append('GT(' + ', '.join(gt_desc or ['none']) + ')')
        return missing

    def _report_input_status(self):
        """周期性输出输入状态，便于诊断为什么没有写 CSV。"""
        missing = self._current_missing_inputs()
        if not missing:
            return
        self.get_logger().warn(
            '尚未开始写入 CSV，缺失输入: '
            + ', '.join(missing)
            + f' | IMU={self.imu_count}, GPS={self.gps_count}, PX4 pose={self.pose_count}, GT={self.gt_count}'
        )

    @staticmethod
    def _quat_to_euler_deg(quat):
        rot = Rotation.from_quat(quat)
        return np.degrees(rot.as_euler('xyz', degrees=False))

    def _record_data(self):
        """定时记录数据"""
        missing = self._current_missing_inputs()
        if missing:
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
        px4_euler_deg = self._quat_to_euler_deg([px4_quat.x, px4_quat.y, px4_quat.z, px4_quat.w])

        row.extend(px4_pos)
        row.extend(px4_vel)
        row.extend(px4_euler_deg.tolist())

        # 真值
        if self.ground_truth_state is not None:
            gazebo_pos = self.ground_truth_state['position']
            gazebo_vel = self.ground_truth_state['linear_velocity']
            gazebo_euler_deg = self._quat_to_euler_deg(self.ground_truth_state['orientation'])
        else:
            gazebo_pos = [float('nan')] * 3
            gazebo_vel = [float('nan')] * 3
            gazebo_euler_deg = np.array([float('nan')] * 3)

        row.extend(gazebo_pos)
        row.extend(gazebo_vel)
        row.extend(gazebo_euler_deg.tolist())

        # 计算误差
        pos_error = np.array(px4_pos, dtype=float) - np.array(gazebo_pos, dtype=float)
        vel_error = np.array(px4_vel, dtype=float) - np.array(gazebo_vel, dtype=float)
        attitude_error = np.array(px4_euler_deg, dtype=float) - np.array(gazebo_euler_deg, dtype=float)

        row.extend(pos_error.tolist())
        row.extend(vel_error.tolist())
        row.extend(attitude_error.tolist())

        # 误差范数
        pos_error_norm = np.linalg.norm(pos_error) if np.isfinite(pos_error).all() else float('nan')
        vel_error_norm = np.linalg.norm(vel_error) if np.isfinite(vel_error).all() else float('nan')
        attitude_error_norm = np.linalg.norm(attitude_error) if np.isfinite(attitude_error).all() else float('nan')

        row.extend([pos_error_norm, vel_error_norm, attitude_error_norm])

        # 写入 CSV
        self.csv_writer.writerow(row)
        self.csv_file.flush()

        self.record_count += 1

        # 每 10 条记录输出一次统计
        if self.record_count % 10 == 0:
            gt_source = self.active_gt_source or 'none'
            self.get_logger().info(
                f'已记录 {self.record_count} 条数据 | '
                f'时间: {elapsed_time:.1f}s | '
                f'GT源: {gt_source} | '
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
            f'  PX4 位置：{self.pose_count}\n'
            f'  真值数据：{self.gt_count}\n'
            f'  活动真值源：{self.active_gt_source}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    recorder = None
    try:
        recorder = PX4BaselineRecorder()
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        pass
    finally:
        if recorder is not None:
            recorder.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
