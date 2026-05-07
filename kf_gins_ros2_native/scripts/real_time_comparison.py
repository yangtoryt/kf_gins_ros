#!/usr/bin/env python3
"""
实时对比分析器 - 计算 EKF2 和 IEKF 的实时误差指标

功能:
- 订阅两个算法的输出
- 实时计算以下指标:
  * 位置误差 (RMSE, MAE)
  * 姿态误差 (Roll/Pitch/Yaw)
  * 速度误差
  * 初始化时间
  * 收敛速度
- 发布对比结果
"""

import csv
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray, Float32, Bool, UInt32
import numpy as np
from collections import deque
from typing import Optional
from dataclasses import dataclass, field
import math


def parse_bool_param(value, default=False):
    """Parse ROS parameters that may arrive as bools, numbers, or strings."""
    if isinstance(value, bool):
        return value
    if value is None:
        return default
    if isinstance(value, (int, float)):
        return bool(value)
    text = str(value).strip().lower()
    if text in ('1', 'true', 'yes', 'on', 'y'):
        return True
    if text in ('0', 'false', 'no', 'off', 'n', 'none', 'disabled', '__disabled__', ''):
        return False
    return default


@dataclass
class PoseData:
    """位姿数据"""
    timestamp: float
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float
    qx: float
    qy: float
    qz: float
    qw: float
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0


@dataclass
class ComparisonMetrics:
    """对比指标"""
    timestamp: float = 0.0
    
    # 位置误差
    position_error_xyz: list = field(default_factory=lambda: [0.0, 0.0, 0.0])
    position_error_norm: float = 0.0
    position_rmse: float = 0.0
    position_mae: float = 0.0
    
    # 姿态误差 (弧度)
    attitude_error_rpy: list = field(default_factory=lambda: [0.0, 0.0, 0.0])
    attitude_error_norm: float = 0.0
    
    # 速度误差
    velocity_error: list = field(default_factory=lambda: [0.0, 0.0, 0.0])
    velocity_error_norm: float = 0.0
    
    # 统计信息
    ekf2_initialized: bool = False
    iekf_initialized: bool = False
    initialization_time_sec: float = 0.0
    convergence_time_sec: float = 0.0
    fallback_active: bool = False
    raw_available: bool = False
    raw_position_error_xyz: list = field(default_factory=lambda: [float('nan'), float('nan'), float('nan')])
    raw_position_error_norm: float = float('nan')
    raw_attitude_error_rpy: list = field(default_factory=lambda: [float('nan'), float('nan'), float('nan')])
    raw_attitude_error_norm: float = float('nan')
    raw_velocity_error: list = field(default_factory=lambda: [float('nan'), float('nan'), float('nan')])
    raw_velocity_error_norm: float = float('nan')
    raw_pair_dt_ms: float = float('nan')


class RealTimeComparison(Node):
    """实时对比 EKF2 和 IEKF"""
    
    def __init__(self):
        super().__init__('real_time_comparison')
        
        # 参数
        legacy_ekf2_topic = self.declare_parameter('ekf2_topic', '/ekf2/pose').value
        self.ekf2_pose_topic = self.declare_parameter('ekf2_pose_topic', legacy_ekf2_topic).value
        self.ekf2_odom_topic = self.declare_parameter('ekf2_odom_topic', '/ekf2/pose_odom').value
        self.subscribe_ekf2_pose = bool(self.declare_parameter('subscribe_ekf2_pose', True).value)
        self.iekf_topic = self.declare_parameter('iekf_topic', '/kf_gins/odom').value
        self.iekf_raw_topic = self.declare_parameter('iekf_raw_topic', '').value
        if str(self.iekf_raw_topic).strip().lower() in ('', 'none', 'false', 'off', 'disabled', '__disabled__'):
            self.iekf_raw_topic = ''
        self.compute_raw_metrics = parse_bool_param(
            self.declare_parameter('compute_raw_metrics', False).value,
            default=False,
        )
        self.raw_callback_mode = str(self.declare_parameter('raw_callback_mode', 'store').value).strip().lower()
        if self.raw_callback_mode not in ('store', 'count_only', 'drop'):
            self.raw_callback_mode = 'store'
        self.iekf_fallback_topic = self.declare_parameter('iekf_fallback_topic', '/kf_gins/fallback_active').value
        self.comparison_output = self.declare_parameter('comparison_output', '/comparison/metrics').value
        self.metrics_csv_path = self.declare_parameter('metrics_csv_path', '').value
        self.iekf_reset_topic = self.declare_parameter('iekf_reset_topic', '/kf_gins/reset_event').value
        self.metrics_publish_rate = self.declare_parameter('metrics_publish_rate', 50).value
        self.metrics_log_period_sec = float(self.declare_parameter('metrics_log_period_sec', 10.0).value)
        self.sync_tolerance_ms = self.declare_parameter('sync_tolerance_ms', 20).value
        self.align_initial = self.declare_parameter('align_initial', True).value
        self.buffer_len = self.declare_parameter('buffer_len', 200).value
        self.publish_live_metrics = bool(self.declare_parameter('publish_live_metrics', True).value)
        self.publish_named_metrics = bool(self.declare_parameter('publish_named_metrics', True).value)
        self.max_abs_position_m = float(self.declare_parameter('max_abs_position_m', 10000.0).value)
        self.max_abs_velocity_mps = float(self.declare_parameter('max_abs_velocity_mps', 100.0).value)
        self.max_pose_jump_m = float(self.declare_parameter('max_pose_jump_m', 50.0).value)
        self.max_velocity_jump_mps = float(self.declare_parameter('max_velocity_jump_mps', 100.0).value)
        self.recompute_alignment_on_jump = bool(self.declare_parameter('recompute_alignment_on_jump', True).value)
        self.clear_error_history_on_realign = bool(self.declare_parameter('clear_error_history_on_realign', True).value)
        
        # PlotJuggler 友好的“单独状态”输出（分别看到 EKF2/IEKF）
        # 说明：
        # - /comparison/metrics 是“差值/误差”指标（不是两套状态）
        # - 下面这些话题会发布各自的 position/velocity/rpy，便于 PlotJuggler 直接叠加对比
        legacy_publish_state = bool(self.declare_parameter('publish_state', True).value)
        self.publish_ekf2_state = bool(self.declare_parameter('publish_ekf2_state', legacy_publish_state).value)
        self.publish_iekf_state = bool(self.declare_parameter('publish_iekf_state', legacy_publish_state).value)
        self.publish_aligned_iekf_state = bool(self.declare_parameter('publish_aligned_iekf_state', True).value)
        self.aligned_fallback_raw = bool(self.declare_parameter('aligned_fallback_raw', True).value)
        self.state_frame_id = self.declare_parameter('state_frame_id', 'map').value
        self.ekf2_state_prefix = self.declare_parameter('ekf2_state_prefix', '/ekf2/state').value
        self.iekf_state_prefix = self.declare_parameter('iekf_state_prefix', '/iekf/state').value
        self.iekf_state_aligned_prefix = self.declare_parameter('iekf_state_aligned_prefix', '/iekf/state_aligned').value
        
        # 状态
        self.ekf2_data: Optional[PoseData] = None  # latest (for debug)
        self.iekf_data: Optional[PoseData] = None  # latest (for debug)
        self.iekf_raw_data: Optional[PoseData] = None  # latest raw core output
        self.iekf_raw_callback_count = 0
        self.start_time = self.get_clock().now()
        self.ekf2_start_time: Optional[float] = None
        self.iekf_start_time: Optional[float] = None

        # 缓冲区（用于时间同步）
        self.ekf2_buf = deque(maxlen=max(10, int(self.buffer_len)))
        self.iekf_buf = deque(maxlen=max(10, int(self.buffer_len)))
        self.iekf_raw_buf = deque(maxlen=max(10, int(self.buffer_len)))
        self.last_pair_key: Optional[tuple] = None

        # 初始对齐（消除常量偏移，避免 EKF2/home 与 KF-GINS/origin 不一致导致“常量误差”）
        self._offset_ready = not self.align_initial
        self._pos_offset = np.zeros(3, dtype=float)
        self._yaw_offset = 0.0
        
        # 历史数据 (用于 RMSE 计算)
        self.position_error_history = deque(maxlen=5000)  # 100 秒 @ 50Hz
        self._last_valid_ekf2: Optional[PoseData] = None
        self._last_valid_iekf: Optional[PoseData] = None
        self._last_valid_iekf_raw: Optional[PoseData] = None
        self.fallback_active = False
        self._csv_file = None
        self._csv_writer = None
        self._csv_rows_since_flush = 0
        self._csv_flush_interval = max(1, int(self.metrics_publish_rate))
        
        # 初始化标志
        self.ekf2_initialized = False
        self.iekf_initialized = False
        self.convergence_threshold = 0.5  # 米，用于判断是否收敛
        
        # QoS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 订阅
        self.ekf2_pose_sub = None
        if self.subscribe_ekf2_pose and self.ekf2_pose_topic:
            self.ekf2_pose_sub = self.create_subscription(
                PoseStamped,
                self.ekf2_pose_topic,
                self.ekf2_pose_callback,
                qos
            )

        self.ekf2_odom_sub = self.create_subscription(
            Odometry,
            self.ekf2_odom_topic,
            self.ekf2_odom_callback,
            qos
        )
        
        self.iekf_sub = self.create_subscription(
            Odometry,
            self.iekf_topic,
            self.iekf_callback,
            qos
        )
        self.iekf_raw_sub = None
        if self.iekf_raw_topic:
            self.iekf_raw_sub = self.create_subscription(
                Odometry,
                self.iekf_raw_topic,
                self.iekf_raw_callback,
                qos
            )

        self.iekf_reset_sub = self.create_subscription(
            UInt32,
            self.iekf_reset_topic,
            self.iekf_reset_callback,
            qos
        )
        self.fallback_sub = None
        if self.iekf_fallback_topic:
            self.fallback_sub = self.create_subscription(
                Bool,
                self.iekf_fallback_topic,
                self.fallback_callback,
                qos
            )
        
        # 发布 - 发布对比指标
        self.metrics_pub = None
        if self.publish_live_metrics:
            self.metrics_pub = self.create_publisher(
                Float32MultiArray,
                self.comparison_output,
                QoSProfile(
                    reliability=ReliabilityPolicy.RELIABLE,
                    history=HistoryPolicy.KEEP_LAST,
                    depth=10
                )
            )

        # 发布 - EKF2/IEKF 单独状态（position/velocity/rpy）
        # Use TRANSIENT_LOCAL so late subscribers (PlotJuggler) get the latest state immediately.
        self._state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # 发布 - 命名对比指标（避免 PlotJuggler 中 data[0..7] 不直观）
        # 这些 topic 描述的是 “IEKF 相对 EKF2 的差值/误差”，因此每个字段只有一条曲线（误差曲线）。
        self._named_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.pos_err_pub = None
        self.pos_rmse_pub = None
        self.pos_mae_pub = None
        self.vel_err_pub = None
        self.att_err_pub = None
        self.yaw_err_pub = None
        self.pos_err_xyz_pub = None
        self.vel_err_xyz_pub = None
        self.att_err_rpy_pub = None
        self.ekf2_init_pub = None
        self.iekf_init_pub = None
        self.sync_dt_pub = None
        self.fallback_active_pub = None

        self.ekf2_pos_pub = None
        self.ekf2_vel_pub = None
        self.ekf2_rpy_pub = None
        self.iekf_pos_pub = None
        self.iekf_vel_pub = None
        self.iekf_rpy_pub = None
        self.iekf_pos_aligned_pub = None
        self.iekf_vel_aligned_pub = None
        self.iekf_rpy_aligned_pub = None
        self._aligned_fallback_warned = False

        if self.publish_ekf2_state:
            self.ekf2_pos_pub = self.create_publisher(Vector3Stamped, f"{self.ekf2_state_prefix}/position", self._state_qos)
            self.ekf2_vel_pub = self.create_publisher(Vector3Stamped, f"{self.ekf2_state_prefix}/velocity", self._state_qos)
            self.ekf2_rpy_pub = self.create_publisher(Vector3Stamped, f"{self.ekf2_state_prefix}/rpy", self._state_qos)

        if self.publish_iekf_state:
            self.iekf_pos_pub = self.create_publisher(Vector3Stamped, f"{self.iekf_state_prefix}/position", self._state_qos)
            self.iekf_vel_pub = self.create_publisher(Vector3Stamped, f"{self.iekf_state_prefix}/velocity", self._state_qos)
            self.iekf_rpy_pub = self.create_publisher(Vector3Stamped, f"{self.iekf_state_prefix}/rpy", self._state_qos)

        if self.publish_aligned_iekf_state:
            self.iekf_pos_aligned_pub = self.create_publisher(
                Vector3Stamped, f"{self.iekf_state_aligned_prefix}/position", self._state_qos
            )
            self.iekf_vel_aligned_pub = self.create_publisher(
                Vector3Stamped, f"{self.iekf_state_aligned_prefix}/velocity", self._state_qos
            )
            self.iekf_rpy_aligned_pub = self.create_publisher(
                Vector3Stamped, f"{self.iekf_state_aligned_prefix}/rpy", self._state_qos
            )

        if self.publish_named_metrics:
            self.pos_err_pub = self.create_publisher(Float32, "/comparison/pos_err", self._named_qos)
            self.pos_rmse_pub = self.create_publisher(Float32, "/comparison/pos_rmse", self._named_qos)
            self.pos_mae_pub = self.create_publisher(Float32, "/comparison/pos_mae", self._named_qos)
            self.vel_err_pub = self.create_publisher(Float32, "/comparison/vel_err", self._named_qos)
            self.att_err_pub = self.create_publisher(Float32, "/comparison/att_err", self._named_qos)
            self.yaw_err_pub = self.create_publisher(Float32, "/comparison/yaw_err", self._named_qos)
            self.sync_dt_pub = self.create_publisher(Float32, "/comparison/sync_dt", self._named_qos)
            self.ekf2_init_pub = self.create_publisher(Bool, "/comparison/ekf2_initialized", self._named_qos)
            self.iekf_init_pub = self.create_publisher(Bool, "/comparison/iekf_initialized", self._named_qos)
            self.pos_err_xyz_pub = self.create_publisher(Vector3Stamped, "/comparison/pos_err_xyz", self._named_qos)
            self.vel_err_xyz_pub = self.create_publisher(Vector3Stamped, "/comparison/vel_err_xyz", self._named_qos)
            self.att_err_rpy_pub = self.create_publisher(Vector3Stamped, "/comparison/att_err_rpy", self._named_qos)
            self.fallback_active_pub = self.create_publisher(Bool, "/comparison/fallback_active", self._named_qos)
        
        # 定时器 - 定期计算和发布指标
        period = 1.0 / max(1, self.metrics_publish_rate)
        self.timer = self.create_timer(period, self.compute_and_publish_metrics)
        self._metrics_log_interval = max(
            1,
            int(round(max(1.0, float(self.metrics_publish_rate)) * max(0.1, self.metrics_log_period_sec))),
        )
        
        self.metric_count = 0
        self._open_metrics_csv()
        
        self.get_logger().info(
            f"RealTimeComparison 已启动\n"
            f"  EKF2 pose: {self.ekf2_pose_topic if self.ekf2_pose_sub is not None else '(disabled)'}\n"
            f"  EKF2 odom: {self.ekf2_odom_topic}\n"
            f"  IEKF: {self.iekf_topic}\n"
            f"  IEKF raw: {self.iekf_raw_topic if self.iekf_raw_topic else '(disabled)'}\n"
            f"  Raw callback mode: {self.raw_callback_mode}\n"
            f"  Raw metrics: {'开启' if self.compute_raw_metrics else '关闭'}\n"
            f"  Fallback topic: {self.iekf_fallback_topic if self.iekf_fallback_topic else '(disabled)'}\n"
            f"  IEKF reset topic: {self.iekf_reset_topic}\n"
            f"  CSV 输出: {self.metrics_csv_path if self.metrics_csv_path else '(disabled)'}\n"
            f"  发布频率: {self.metrics_publish_rate} Hz\n"
            f"  指标日志周期: {self.metrics_log_period_sec:.1f} s\n"
            f"  同步容差: ±{self.sync_tolerance_ms} ms\n"
            f"  初始对齐: {'开启' if self.align_initial else '关闭'}\n"
            f"  异常过滤: max_abs_position={self.max_abs_position_m:.1f} m, "
            f"max_pose_jump={self.max_pose_jump_m:.1f} m\n"
            f"  Legacy /comparison/metrics 输出: {'开启' if self.publish_live_metrics else '关闭'}\n"
            f"  PlotJuggler 状态输出: "
            f"EKF2={'开启' if self.publish_ekf2_state else '关闭'}, "
            f"IEKF={'开启' if self.publish_iekf_state else '关闭'}, "
            f"IEKF(aligned)={'开启' if self.publish_aligned_iekf_state else '关闭'}\n"
            f"  命名误差话题输出: {'开启' if self.publish_named_metrics else '关闭'}"
        )
        if self.publish_ekf2_state or self.publish_iekf_state or self.publish_aligned_iekf_state:
            lines = ["  状态话题:"]
            if self.publish_ekf2_state:
                lines.append(f"    - EKF2: {self.ekf2_state_prefix}/(position|velocity|rpy)")
            if self.publish_iekf_state:
                lines.append(f"    - IEKF: {self.iekf_state_prefix}/(position|velocity|rpy)")
            if self.publish_aligned_iekf_state:
                lines.append(f"    - IEKF(aligned): {self.iekf_state_aligned_prefix}/(position|velocity|rpy)")
            self.get_logger().info("\n".join(lines))
        if self.publish_named_metrics:
            self.get_logger().info(
                "  误差话题:\n"
                "    - /comparison/pos_err (m), /comparison/pos_rmse (m), /comparison/pos_mae (m)\n"
                "    - /comparison/vel_err (m/s)\n"
                "    - /comparison/att_err (rad), /comparison/yaw_err (rad)\n"
                "    - /comparison/pos_err_xyz, /comparison/vel_err_xyz, /comparison/att_err_rpy (Vector3Stamped)\n"
                "    - /comparison/sync_dt (s), /comparison/fallback_active, /comparison/(ekf2_initialized|iekf_initialized) (Bool)"
            )
        
    def ekf2_pose_callback(self, msg: PoseStamped):
        """处理 EKF2 PoseStamped（无速度）"""
        try:
            ts = self.msg_to_sec(msg.header.stamp)
            
            pos = msg.pose.position
            quat = msg.pose.orientation
            roll, pitch, yaw = self.quat_to_rpy(quat.x, quat.y, quat.z, quat.w)
            
            data = PoseData(
                timestamp=ts,
                x=pos.x, y=pos.y, z=pos.z,
                roll=roll, pitch=pitch, yaw=yaw,
                qx=quat.x, qy=quat.y, qz=quat.z, qw=quat.w
            )
            if not self._accept_pose_sample("EKF2", data, self._last_valid_ekf2):
                return
            self.ekf2_data = data
            self.ekf2_buf.append(data)
            self._last_valid_ekf2 = data

            # 发布 EKF2 状态（无速度时 vx/vy/vz=0），便于 PlotJuggler 立即看到曲线
            if self.publish_ekf2_state:
                self._publish_ekf2_state(msg.header.stamp, data)
            
            if not self.ekf2_initialized:
                self.ekf2_initialized = True
                self.ekf2_start_time = ts
                self.get_logger().info("✓ EKF2 已初始化")
            
        except Exception as e:
            self.get_logger().error(f"EKF2 处理错误: {e}", throttle_duration_sec=5)

    def ekf2_odom_callback(self, msg: Odometry):
        """处理 EKF2 Odometry（包含速度，优先用于对比）"""
        try:
            ts = self.msg_to_sec(msg.header.stamp)

            pos = msg.pose.pose.position
            quat = msg.pose.pose.orientation
            roll, pitch, yaw = self.quat_to_rpy(quat.x, quat.y, quat.z, quat.w)

            twist = msg.twist.twist
            data = PoseData(
                timestamp=ts,
                x=pos.x, y=pos.y, z=pos.z,
                roll=roll, pitch=pitch, yaw=yaw,
                qx=quat.x, qy=quat.y, qz=quat.z, qw=quat.w,
                vx=twist.linear.x, vy=twist.linear.y, vz=twist.linear.z
            )
            if not self._accept_pose_sample("EKF2", data, self._last_valid_ekf2):
                return
            self.ekf2_data = data
            self.ekf2_buf.append(data)
            self._last_valid_ekf2 = data

            if self.publish_ekf2_state:
                self._publish_ekf2_state(msg.header.stamp, data)

            if not self.ekf2_initialized:
                self.ekf2_initialized = True
                self.ekf2_start_time = ts
                self.get_logger().info("✓ EKF2 已初始化 (odom)")

        except Exception as e:
            self.get_logger().error(f"EKF2 odom 处理错误: {e}", throttle_duration_sec=5)

    def _odom_to_pose_data(self, msg: Odometry) -> PoseData:
        ts = self.msg_to_sec(msg.header.stamp)
        pos = msg.pose.pose.position
        quat = msg.pose.pose.orientation
        roll, pitch, yaw = self.quat_to_rpy(quat.x, quat.y, quat.z, quat.w)
        twist = msg.twist.twist
        return PoseData(
            timestamp=ts,
            x=pos.x, y=pos.y, z=pos.z,
            roll=roll, pitch=pitch, yaw=yaw,
            qx=quat.x, qy=quat.y, qz=quat.z, qw=quat.w,
            vx=twist.linear.x, vy=twist.linear.y, vz=twist.linear.z
        )

    def iekf_callback(self, msg: Odometry):
        """处理 IEKF 消息"""
        try:
            data = self._odom_to_pose_data(msg)
            if not self._accept_pose_sample("IEKF", data, self._last_valid_iekf):
                return
            self.iekf_data = data
            self.iekf_buf.append(data)
            self._last_valid_iekf = data

            if self.publish_iekf_state:
                self._publish_iekf_state(msg.header.stamp, data)
            
            if not self.iekf_initialized:
                self.iekf_initialized = True
                self.iekf_start_time = data.timestamp
                self.get_logger().info("✓ IEKF 已初始化")
            
        except Exception as e:
            self.get_logger().error(f"IEKF 处理错误: {e}", throttle_duration_sec=5)

    def iekf_raw_callback(self, msg: Odometry):
        """处理 raw IEKF 消息（不带业务 fallback）"""
        try:
            self.iekf_raw_callback_count += 1
            if self.raw_callback_mode in ('count_only', 'drop'):
                return
            data = self._odom_to_pose_data(msg)
            if not self._accept_pose_sample("IEKF(raw)", data, self._last_valid_iekf_raw):
                return
            self.iekf_raw_data = data
            self.iekf_raw_buf.append(data)
            self._last_valid_iekf_raw = data
        except Exception as e:
            self.get_logger().error(f"IEKF raw 处理错误: {e}", throttle_duration_sec=5)

    def fallback_callback(self, msg: Bool):
        self.fallback_active = bool(msg.data)

    def iekf_reset_callback(self, msg: UInt32):
        self._handle_iekf_reset_event(msg.data)

    def compute_and_publish_metrics(self):
        """计算并发布对比指标"""
        try:
            synced = self._get_synced_pair()
            if synced is None:
                return
            ekf2, iekf, sync_dt = synced

            if not self._pose_is_finite(ekf2) or not self._pose_is_finite(iekf):
                self.get_logger().warn("Synced data contains NaN/Inf; skipping metrics.", throttle_duration_sec=2)
                return

            raw_iekf = None
            raw_dt_sec = float('nan')
            if self.compute_raw_metrics:
                raw_iekf, raw_dt_sec = self._find_closest_pose_with_dt(self.iekf_raw_buf, iekf.timestamp)
            metrics = self._compute_metrics(ekf2, iekf, raw_iekf)
            if raw_iekf is not None and np.isfinite(raw_dt_sec):
                metrics.raw_pair_dt_ms = float(raw_dt_sec) * 1000.0
            if not np.isfinite(metrics.position_error_norm) or not np.isfinite(metrics.attitude_error_norm) or not np.isfinite(metrics.velocity_error_norm):
                self.get_logger().warn("Metrics contains NaN/Inf; skipping publish.", throttle_duration_sec=2)
                return

            elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
            self._write_metrics_csv(elapsed, sync_dt, metrics)

            if self.publish_named_metrics:
                stamp_msg = self.get_clock().now().to_msg()
                self._publish_named_metrics(stamp_msg, metrics, sync_dt)
            
            # 发布为 Float32MultiArray
            if self.metrics_pub is not None:
                array_msg = Float32MultiArray()
                array_msg.data = [
                    metrics.position_error_norm,
                    metrics.attitude_error_norm,
                    metrics.velocity_error_norm,
                    metrics.position_rmse,
                    float(self.ekf2_initialized),
                    float(self.iekf_initialized),
                    metrics.initialization_time_sec,
                    float(sync_dt),
                ]
                self.metrics_pub.publish(array_msg)
            
            self.metric_count += 1
            if self.metric_count % self._metrics_log_interval == 0:
                self._log_metrics(metrics, sync_dt, elapsed)
            
        except Exception as e:
            self.get_logger().error(f"计算指标时出错: {e}", throttle_duration_sec=5)

    def _publish_named_metrics(self, stamp_msg, metrics: ComparisonMetrics, sync_dt_sec: float) -> None:
        if self.pos_err_pub is not None:
            self.pos_err_pub.publish(self._make_f32(metrics.position_error_norm))
        if self.pos_rmse_pub is not None:
            self.pos_rmse_pub.publish(self._make_f32(metrics.position_rmse))
        if self.pos_mae_pub is not None:
            self.pos_mae_pub.publish(self._make_f32(metrics.position_mae))
        if self.vel_err_pub is not None:
            self.vel_err_pub.publish(self._make_f32(metrics.velocity_error_norm))
        if self.att_err_pub is not None:
            self.att_err_pub.publish(self._make_f32(metrics.attitude_error_norm))
        if self.yaw_err_pub is not None and metrics.attitude_error_rpy:
            self.yaw_err_pub.publish(self._make_f32(metrics.attitude_error_rpy[2]))
        if self.sync_dt_pub is not None:
            self.sync_dt_pub.publish(self._make_f32(float(sync_dt_sec)))
        if self.fallback_active_pub is not None:
            msg = Bool()
            msg.data = bool(metrics.fallback_active)
            self.fallback_active_pub.publish(msg)

        if self.ekf2_init_pub is not None:
            msg = Bool()
            msg.data = bool(metrics.ekf2_initialized)
            self.ekf2_init_pub.publish(msg)
        if self.iekf_init_pub is not None:
            msg = Bool()
            msg.data = bool(metrics.iekf_initialized)
            self.iekf_init_pub.publish(msg)

        if self.pos_err_xyz_pub is not None and metrics.position_error_xyz:
            self.pos_err_xyz_pub.publish(
                self._make_vec(stamp_msg, metrics.position_error_xyz[0], metrics.position_error_xyz[1], metrics.position_error_xyz[2])
            )
        if self.vel_err_xyz_pub is not None and metrics.velocity_error:
            self.vel_err_xyz_pub.publish(
                self._make_vec(stamp_msg, metrics.velocity_error[0], metrics.velocity_error[1], metrics.velocity_error[2])
            )
        if self.att_err_rpy_pub is not None and metrics.attitude_error_rpy:
            self.att_err_rpy_pub.publish(
                self._make_vec(stamp_msg, metrics.attitude_error_rpy[0], metrics.attitude_error_rpy[1], metrics.attitude_error_rpy[2])
            )

    @staticmethod
    def _make_f32(v: float) -> Float32:
        msg = Float32()
        msg.data = float(v)
        return msg

    def _publish_state_vectors(self, stamp_msg, ekf2: PoseData, iekf: PoseData) -> None:
        if not self._pose_is_reasonable(ekf2) or not self._pose_is_reasonable(iekf):
            return
        if self.ekf2_pos_pub is not None:
            self.ekf2_pos_pub.publish(self._make_vec(stamp_msg, ekf2.x, ekf2.y, ekf2.z))
        if self.ekf2_vel_pub is not None:
            self.ekf2_vel_pub.publish(self._make_vec(stamp_msg, ekf2.vx, ekf2.vy, ekf2.vz))
        if self.ekf2_rpy_pub is not None:
            self.ekf2_rpy_pub.publish(self._make_vec(stamp_msg, ekf2.roll, ekf2.pitch, ekf2.yaw))

        if self.iekf_pos_pub is not None:
            self.iekf_pos_pub.publish(self._make_vec(stamp_msg, iekf.x, iekf.y, iekf.z))
        if self.iekf_vel_pub is not None:
            self.iekf_vel_pub.publish(self._make_vec(stamp_msg, iekf.vx, iekf.vy, iekf.vz))
        if self.iekf_rpy_pub is not None:
            self.iekf_rpy_pub.publish(self._make_vec(stamp_msg, iekf.roll, iekf.pitch, iekf.yaw))

        if not self.publish_aligned_iekf_state:
            return
        if self.iekf_pos_aligned_pub is None and self.iekf_vel_aligned_pub is None and self.iekf_rpy_aligned_pub is None:
            return
        use_aligned = self._offset_ready
        if not use_aligned and not self.aligned_fallback_raw:
            return
        if not use_aligned and self.aligned_fallback_raw and not self._aligned_fallback_warned:
            self.get_logger().warn(
                "Aligned offset not ready; publishing raw values on /iekf/state_aligned until sync.",
                throttle_duration_sec=5,
            )
            self._aligned_fallback_warned = True

        if use_aligned:
            iekf_x = iekf.x + float(self._pos_offset[0])
            iekf_y = iekf.y + float(self._pos_offset[1])
            iekf_z = iekf.z + float(self._pos_offset[2])
            iekf_yaw = self._normalize_angle(iekf.yaw + self._yaw_offset)
        else:
            iekf_x = iekf.x
            iekf_y = iekf.y
            iekf_z = iekf.z
            iekf_yaw = self._normalize_angle(iekf.yaw)

        if self.iekf_pos_aligned_pub is not None:
            self.iekf_pos_aligned_pub.publish(self._make_vec(stamp_msg, iekf_x, iekf_y, iekf_z))
        if self.iekf_vel_aligned_pub is not None:
            self.iekf_vel_aligned_pub.publish(self._make_vec(stamp_msg, iekf.vx, iekf.vy, iekf.vz))
        if self.iekf_rpy_aligned_pub is not None:
            self.iekf_rpy_aligned_pub.publish(self._make_vec(stamp_msg, iekf.roll, iekf.pitch, iekf_yaw))

    def _make_vec(self, stamp_msg, x: float, y: float, z: float) -> Vector3Stamped:
        msg = Vector3Stamped()
        msg.header.stamp = stamp_msg
        msg.header.frame_id = self.state_frame_id
        msg.vector.x = float(x)
        msg.vector.y = float(y)
        msg.vector.z = float(z)
        return msg

    def _publish_ekf2_state(self, stamp_msg, ekf2: PoseData) -> None:
        if not self._pose_is_reasonable(ekf2):
            return
        if self.ekf2_pos_pub is not None:
            self.ekf2_pos_pub.publish(self._make_vec(stamp_msg, ekf2.x, ekf2.y, ekf2.z))
        if self.ekf2_vel_pub is not None:
            self.ekf2_vel_pub.publish(self._make_vec(stamp_msg, ekf2.vx, ekf2.vy, ekf2.vz))
        if self.ekf2_rpy_pub is not None:
            self.ekf2_rpy_pub.publish(self._make_vec(stamp_msg, ekf2.roll, ekf2.pitch, ekf2.yaw))

    def _publish_iekf_state(self, stamp_msg, iekf: PoseData) -> None:
        if not self._pose_is_reasonable(iekf):
            return
        if self.iekf_pos_pub is not None:
            self.iekf_pos_pub.publish(self._make_vec(stamp_msg, iekf.x, iekf.y, iekf.z))
        if self.iekf_vel_pub is not None:
            self.iekf_vel_pub.publish(self._make_vec(stamp_msg, iekf.vx, iekf.vy, iekf.vz))
        if self.iekf_rpy_pub is not None:
            self.iekf_rpy_pub.publish(self._make_vec(stamp_msg, iekf.roll, iekf.pitch, iekf.yaw))

        if not self.publish_aligned_iekf_state:
            return
        use_aligned = self._offset_ready
        if not use_aligned and not self.aligned_fallback_raw:
            return
        if not use_aligned and self.aligned_fallback_raw and not self._aligned_fallback_warned:
            self.get_logger().warn(
                "Aligned offset not ready; publishing raw values on /iekf/state_aligned until sync.",
                throttle_duration_sec=5,
            )
            self._aligned_fallback_warned = True

        if use_aligned:
            iekf_x = iekf.x + float(self._pos_offset[0])
            iekf_y = iekf.y + float(self._pos_offset[1])
            iekf_z = iekf.z + float(self._pos_offset[2])
            iekf_yaw = self._normalize_angle(iekf.yaw + self._yaw_offset)
        else:
            iekf_x = iekf.x
            iekf_y = iekf.y
            iekf_z = iekf.z
            iekf_yaw = self._normalize_angle(iekf.yaw)

        if self.iekf_pos_aligned_pub is not None:
            self.iekf_pos_aligned_pub.publish(self._make_vec(stamp_msg, iekf_x, iekf_y, iekf_z))
        if self.iekf_vel_aligned_pub is not None:
            self.iekf_vel_aligned_pub.publish(self._make_vec(stamp_msg, iekf.vx, iekf.vy, iekf.vz))
        if self.iekf_rpy_aligned_pub is not None:
            self.iekf_rpy_aligned_pub.publish(self._make_vec(stamp_msg, iekf.roll, iekf.pitch, iekf_yaw))

    @staticmethod
    def _pose_is_finite(p: PoseData) -> bool:
        vals = np.array([p.x, p.y, p.z, p.roll, p.pitch, p.yaw, p.vx, p.vy, p.vz], dtype=float)
        return bool(np.all(np.isfinite(vals)))

    def _pose_is_reasonable(self, p: PoseData) -> bool:
        if not self._pose_is_finite(p):
            return False
        if max(abs(p.x), abs(p.y), abs(p.z)) > self.max_abs_position_m:
            return False
        speed = math.sqrt(p.vx * p.vx + p.vy * p.vy + p.vz * p.vz)
        if speed > self.max_abs_velocity_mps:
            return False
        return True

    def _pose_jump_too_large(self, prev: PoseData, cur: PoseData) -> bool:
        pos_jump = math.sqrt((cur.x - prev.x) ** 2 + (cur.y - prev.y) ** 2 + (cur.z - prev.z) ** 2)
        vel_jump = math.sqrt((cur.vx - prev.vx) ** 2 + (cur.vy - prev.vy) ** 2 + (cur.vz - prev.vz) ** 2)
        return pos_jump > self.max_pose_jump_m or vel_jump > self.max_velocity_jump_mps

    def _invalidate_alignment(self, reason: str) -> None:
        if self.align_initial and self.recompute_alignment_on_jump:
            self._offset_ready = False
            self._pos_offset = np.zeros(3, dtype=float)
            self._yaw_offset = 0.0
            self._aligned_fallback_warned = False
            self.last_pair_key = None
            if self.clear_error_history_on_realign:
                self.position_error_history.clear()
            self.get_logger().warn(f"Alignment invalidated: {reason}", throttle_duration_sec=2.0)

    def _handle_iekf_reset_event(self, seq: int) -> None:
        self._invalidate_alignment(f"IEKF reset event #{seq}")
        self.iekf_buf.clear()
        self.iekf_raw_buf.clear()
        self._last_valid_iekf = None
        self._last_valid_iekf_raw = None
        self.iekf_data = None
        self.iekf_raw_data = None
        self.last_pair_key = None
        self.get_logger().warn(
            f"IEKF reset event #{seq}: cleared IEKF buffer and forcing realignment on next sync.",
            throttle_duration_sec=1.0,
        )

    def _accept_pose_sample(self, source: str, cur: PoseData, prev: Optional[PoseData]) -> bool:
        if not self._pose_is_reasonable(cur):
            self.get_logger().warn(
                f"Drop {source} sample: absurd or non-finite state "
                f"pos=({cur.x:.3f},{cur.y:.3f},{cur.z:.3f}) "
                f"vel=({cur.vx:.3f},{cur.vy:.3f},{cur.vz:.3f})",
                throttle_duration_sec=2.0,
            )
            if source == "IEKF":
                self._invalidate_alignment("invalid IEKF sample")
            return False
        if prev is not None and self._pose_jump_too_large(prev, cur):
            pos_jump = math.sqrt((cur.x - prev.x) ** 2 + (cur.y - prev.y) ** 2 + (cur.z - prev.z) ** 2)
            vel_jump = math.sqrt((cur.vx - prev.vx) ** 2 + (cur.vy - prev.vy) ** 2 + (cur.vz - prev.vz) ** 2)
            self.get_logger().warn(
                f"Drop {source} jump sample: pos_jump={pos_jump:.3f} m, vel_jump={vel_jump:.3f} m/s",
                throttle_duration_sec=2.0,
            )
            if source == "IEKF":
                self._invalidate_alignment("IEKF jump sample")
            return False
        return True

    def _apply_alignment(self, pose: PoseData) -> tuple:
        pose_x = pose.x
        pose_y = pose.y
        pose_z = pose.z
        pose_yaw = pose.yaw
        if self._offset_ready:
            pose_x += float(self._pos_offset[0])
            pose_y += float(self._pos_offset[1])
            pose_z += float(self._pos_offset[2])
            pose_yaw = self._normalize_angle(pose_yaw + self._yaw_offset)
        return pose_x, pose_y, pose_z, pose_yaw

    @staticmethod
    def _quat_multiply(q1: tuple, q2: tuple) -> tuple:
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        return (
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        )

    @staticmethod
    def _quat_conjugate(q: tuple) -> tuple:
        x, y, z, w = q
        return (-x, -y, -z, w)

    @staticmethod
    def _quat_normalize(q: tuple) -> tuple:
        x, y, z, w = q
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm <= 1e-12:
            return (0.0, 0.0, 0.0, 1.0)
        inv = 1.0 / norm
        return (x * inv, y * inv, z * inv, w * inv)

    @staticmethod
    def _yaw_quaternion(yaw_rad: float) -> tuple:
        half = 0.5 * yaw_rad
        return (0.0, 0.0, math.sin(half), math.cos(half))

    def _attitude_geodesic_error(self, reference: PoseData, candidate: PoseData) -> float:
        candidate_q = (candidate.qx, candidate.qy, candidate.qz, candidate.qw)
        if self.align_initial and self._offset_ready:
            candidate_q = self._quat_multiply(self._yaw_quaternion(self._yaw_offset), candidate_q)
        ref_q = self._quat_normalize((reference.qx, reference.qy, reference.qz, reference.qw))
        cand_q = self._quat_normalize(candidate_q)
        q_rel = self._quat_multiply(ref_q, self._quat_conjugate(cand_q))
        q_rel = self._quat_normalize(q_rel)
        w = max(-1.0, min(1.0, q_rel[3]))
        return 2.0 * math.acos(abs(w))

    def _compute_error_terms(self, ekf2: PoseData, candidate: PoseData) -> tuple:
        cand_x, cand_y, cand_z, cand_yaw = self._apply_alignment(candidate)

        pos_error_x = ekf2.x - cand_x
        pos_error_y = ekf2.y - cand_y
        pos_error_z = ekf2.z - cand_z
        pos_error_norm = math.sqrt(pos_error_x**2 + pos_error_y**2 + pos_error_z**2)

        roll_error = self._normalize_angle(ekf2.roll - candidate.roll)
        pitch_error = self._normalize_angle(ekf2.pitch - candidate.pitch)
        yaw_error = self._normalize_angle(ekf2.yaw - cand_yaw)
        att_error_norm = self._attitude_geodesic_error(ekf2, candidate)

        vel_error_x = ekf2.vx - candidate.vx
        vel_error_y = ekf2.vy - candidate.vy
        vel_error_z = ekf2.vz - candidate.vz
        vel_error_norm = math.sqrt(vel_error_x**2 + vel_error_y**2 + vel_error_z**2)

        return (
            [pos_error_x, pos_error_y, pos_error_z],
            pos_error_norm,
            [roll_error, pitch_error, yaw_error],
            att_error_norm,
            [vel_error_x, vel_error_y, vel_error_z],
            vel_error_norm,
        )

    def _compute_metrics(self, ekf2: PoseData, iekf: PoseData, iekf_raw: Optional[PoseData]) -> ComparisonMetrics:
        """计算对比指标"""
        metrics = ComparisonMetrics()
        metrics.timestamp = self.get_clock().now().nanoseconds * 1e-9
        metrics.fallback_active = bool(self.fallback_active)

        if self.align_initial and not self._offset_ready:
            self._pos_offset = np.array([ekf2.x - iekf.x, ekf2.y - iekf.y, ekf2.z - iekf.z], dtype=float)
            self._yaw_offset = self._normalize_angle(ekf2.yaw - iekf.yaw)
            self._offset_ready = True
            self.get_logger().info(
                f"✓ 初始对齐完成: pos_offset={self._pos_offset.tolist()}, yaw_offset={math.degrees(self._yaw_offset):.2f}°"
            )

        (
            metrics.position_error_xyz,
            metrics.position_error_norm,
            metrics.attitude_error_rpy,
            metrics.attitude_error_norm,
            metrics.velocity_error,
            metrics.velocity_error_norm,
        ) = self._compute_error_terms(ekf2, iekf)
        
        # 保存到历史 (用于 RMSE)
        if np.isfinite(metrics.position_error_norm):
            self.position_error_history.append(metrics.position_error_norm)
        
        if len(self.position_error_history) > 0:
            metrics.position_rmse = math.sqrt(
                np.mean(np.array(list(self.position_error_history))**2)
            )
            metrics.position_mae = np.mean(np.abs(list(self.position_error_history)))

        if iekf_raw is not None and self._pose_is_finite(iekf_raw):
            metrics.raw_available = True
            (
                metrics.raw_position_error_xyz,
                metrics.raw_position_error_norm,
                metrics.raw_attitude_error_rpy,
                metrics.raw_attitude_error_norm,
                metrics.raw_velocity_error,
                metrics.raw_velocity_error_norm,
            ) = self._compute_error_terms(ekf2, iekf_raw)
        
        # 初始化状态
        metrics.ekf2_initialized = self.ekf2_initialized
        metrics.iekf_initialized = self.iekf_initialized
        
        # 初始化时间
        if self.ekf2_start_time is not None and self.iekf_start_time is not None:
            metrics.initialization_time_sec = abs(
                self.ekf2_start_time - self.iekf_start_time
            )
        
        return metrics
    
    def _log_metrics(self, metrics: ComparisonMetrics, sync_dt_sec: float, elapsed: float):
        """打印指标摘要"""
        raw_block = ""
        if metrics.raw_available:
            raw_block = (
                f"  Raw core 位置误差: {metrics.raw_position_error_norm:.4f} m "
                f"(XYZ={metrics.raw_position_error_xyz})\n"
                f"  Raw core 姿态误差: {math.degrees(metrics.raw_attitude_error_norm):.2f}° "
                f"(RPY={[math.degrees(x) for x in metrics.raw_attitude_error_rpy]})\n"
                f"  Raw core 速度误差: {metrics.raw_velocity_error_norm:.4f} m/s "
                f"(XYZ={metrics.raw_velocity_error})\n"
            )
        self.get_logger().info(
            f"\n{'='*60}\n"
            f"对比指标 (t={elapsed:.1f}s):\n"
            f"  位置误差: {metrics.position_error_norm:.4f} m "
            f"(RMSE={metrics.position_rmse:.4f} m, MAE={metrics.position_mae:.4f} m, XYZ={metrics.position_error_xyz})\n"
            f"  姿态误差: {math.degrees(metrics.attitude_error_norm):.2f}° "
            f"(RPY={[math.degrees(x) for x in metrics.attitude_error_rpy]})\n"
            f"  速度误差: {metrics.velocity_error_norm:.4f} m/s (XYZ={metrics.velocity_error})\n"
            f"{raw_block}"
            f"  fallback_active: {'true' if metrics.fallback_active else 'false'}\n"
            f"  同步残差: {sync_dt_sec*1000.0:+.1f} ms\n"
            f"  EKF2 初始化: {'✓' if metrics.ekf2_initialized else '✗'}\n"
            f"  IEKF 初始化: {'✓' if metrics.iekf_initialized else '✗'}\n"
            f"{'='*60}"
        )

    def _get_synced_pair(self) -> Optional[tuple]:
        """从缓冲区中取一对时间戳最接近的 EKF2/IEKF 数据。"""
        if not self.ekf2_buf or not self.iekf_buf:
            return None

        tol_sec = max(0.0, float(self.sync_tolerance_ms)) * 1e-3

        ekf2_latest = self.ekf2_buf[-1]
        iekf_match = self._find_closest_pose(self.iekf_buf, ekf2_latest.timestamp)
        if iekf_match is not None:
            dt = ekf2_latest.timestamp - iekf_match.timestamp
            if abs(dt) <= tol_sec:
                key = (ekf2_latest.timestamp, iekf_match.timestamp)
                if key != self.last_pair_key:
                    self.last_pair_key = key
                    return ekf2_latest, iekf_match, dt

        iekf_latest = self.iekf_buf[-1]
        ekf2_match = self._find_closest_pose(self.ekf2_buf, iekf_latest.timestamp)
        if ekf2_match is not None:
            dt = ekf2_match.timestamp - iekf_latest.timestamp
            if abs(dt) <= tol_sec:
                key = (ekf2_match.timestamp, iekf_latest.timestamp)
                if key != self.last_pair_key:
                    self.last_pair_key = key
                    return ekf2_match, iekf_latest, dt

        return None

    def _find_closest_pose(self, buf: deque, target_ts: float) -> Optional[PoseData]:
        best, _ = self._find_closest_pose_with_dt(buf, target_ts)
        return best

    def _find_closest_pose_with_dt(self, buf: deque, target_ts: float) -> tuple[Optional[PoseData], float]:
        if not buf:
            return None, float('inf')
        tol_sec = max(0.0, float(self.sync_tolerance_ms)) * 1e-3
        best = None
        best_dt = float('inf')
        for item in buf:
            dt = item.timestamp - target_ts
            if abs(dt) < abs(best_dt):
                best = item
                best_dt = dt
        if best is None or abs(best_dt) > tol_sec:
            return None, best_dt
        return best, best_dt

    def _open_metrics_csv(self) -> None:
        if not self.metrics_csv_path:
            return
        csv_path = os.path.abspath(str(self.metrics_csv_path))
        parent = os.path.dirname(csv_path)
        if parent:
            os.makedirs(parent, exist_ok=True)
        self._csv_file = open(csv_path, 'w', newline='')
        fieldnames = [
            'elapsed_sec',
            'ros_time_sec',
            'sync_dt_ms',
            'fallback_active',
            'raw_available',
            'position_error_x_m',
            'position_error_y_m',
            'position_error_z_m',
            'position_error_norm_m',
            'position_rmse_m',
            'position_mae_m',
            'velocity_error_x_mps',
            'velocity_error_y_mps',
            'velocity_error_z_mps',
            'velocity_error_norm_mps',
            'attitude_error_roll_deg',
            'attitude_error_pitch_deg',
            'attitude_error_yaw_deg',
            'attitude_error_norm_deg',
            'raw_position_error_x_m',
            'raw_position_error_y_m',
            'raw_position_error_z_m',
            'raw_position_error_norm_m',
            'raw_pair_dt_ms',
            'raw_velocity_error_x_mps',
            'raw_velocity_error_y_mps',
            'raw_velocity_error_z_mps',
            'raw_velocity_error_norm_mps',
            'raw_attitude_error_roll_deg',
            'raw_attitude_error_pitch_deg',
            'raw_attitude_error_yaw_deg',
            'raw_attitude_error_norm_deg',
            'ekf2_initialized',
            'iekf_initialized',
        ]
        self._csv_writer = csv.DictWriter(self._csv_file, fieldnames=fieldnames)
        self._csv_writer.writeheader()

    def _write_metrics_csv(self, elapsed: float, sync_dt_sec: float, metrics: ComparisonMetrics) -> None:
        if self._csv_writer is None:
            return
        row = {
            'elapsed_sec': float(elapsed),
            'ros_time_sec': float(metrics.timestamp),
            'sync_dt_ms': float(sync_dt_sec) * 1000.0,
            'fallback_active': int(bool(metrics.fallback_active)),
            'raw_available': int(bool(metrics.raw_available)),
            'position_error_x_m': float(metrics.position_error_xyz[0]),
            'position_error_y_m': float(metrics.position_error_xyz[1]),
            'position_error_z_m': float(metrics.position_error_xyz[2]),
            'position_error_norm_m': float(metrics.position_error_norm),
            'position_rmse_m': float(metrics.position_rmse),
            'position_mae_m': float(metrics.position_mae),
            'velocity_error_x_mps': float(metrics.velocity_error[0]),
            'velocity_error_y_mps': float(metrics.velocity_error[1]),
            'velocity_error_z_mps': float(metrics.velocity_error[2]),
            'velocity_error_norm_mps': float(metrics.velocity_error_norm),
            'attitude_error_roll_deg': math.degrees(metrics.attitude_error_rpy[0]),
            'attitude_error_pitch_deg': math.degrees(metrics.attitude_error_rpy[1]),
            'attitude_error_yaw_deg': math.degrees(metrics.attitude_error_rpy[2]),
            'attitude_error_norm_deg': math.degrees(metrics.attitude_error_norm),
            'raw_position_error_x_m': float(metrics.raw_position_error_xyz[0]),
            'raw_position_error_y_m': float(metrics.raw_position_error_xyz[1]),
            'raw_position_error_z_m': float(metrics.raw_position_error_xyz[2]),
            'raw_position_error_norm_m': float(metrics.raw_position_error_norm),
            'raw_pair_dt_ms': float(metrics.raw_pair_dt_ms),
            'raw_velocity_error_x_mps': float(metrics.raw_velocity_error[0]),
            'raw_velocity_error_y_mps': float(metrics.raw_velocity_error[1]),
            'raw_velocity_error_z_mps': float(metrics.raw_velocity_error[2]),
            'raw_velocity_error_norm_mps': float(metrics.raw_velocity_error_norm),
            'raw_attitude_error_roll_deg': math.degrees(metrics.raw_attitude_error_rpy[0]),
            'raw_attitude_error_pitch_deg': math.degrees(metrics.raw_attitude_error_rpy[1]),
            'raw_attitude_error_yaw_deg': math.degrees(metrics.raw_attitude_error_rpy[2]),
            'raw_attitude_error_norm_deg': math.degrees(metrics.raw_attitude_error_norm),
            'ekf2_initialized': int(bool(metrics.ekf2_initialized)),
            'iekf_initialized': int(bool(metrics.iekf_initialized)),
        }
        self._csv_writer.writerow(row)
        self._csv_rows_since_flush += 1
        if self._csv_rows_since_flush >= self._csv_flush_interval:
            self._csv_file.flush()
            self._csv_rows_since_flush = 0

    def _close_metrics_csv(self) -> None:
        if self._csv_file is not None:
            self._csv_file.flush()
            self._csv_file.close()
            self._csv_file = None
            self._csv_writer = None

    @staticmethod
    def quat_to_rpy(x, y, z, w) -> tuple:
        """四元数转欧拉角 (RPY)"""
        # Roll (x轴旋转)
        roll = math.atan2(2*(w*x + y*z), 1 - 2*(x**2 + y**2))
        
        # Pitch (y轴旋转)
        sinp = 2*(w*y - z*x)
        sinp = max(-1.0, min(1.0, sinp))  # 夹紧到 [-1, 1]
        pitch = math.asin(sinp)
        
        # Yaw (z轴旋转)
        yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y**2 + z**2))
        
        return roll, pitch, yaw

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """将角度规范化到 [-π, π]"""
        while angle > math.pi:
            angle -= 2*math.pi
        while angle < -math.pi:
            angle += 2*math.pi
        return angle

    def msg_to_sec(self, stamp) -> float:
        """将 ROS 时间戳转换为秒"""
        return stamp.sec + stamp.nanosec * 1e-9

    def destroy_node(self):
        self._close_metrics_csv()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RealTimeComparison()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
