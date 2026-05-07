#!/usr/bin/env python3
"""
EKF2 状态中继器 - 将 MAVROS 发布的位姿转换为标准格式

功能:
- 订阅 MAVROS /mavros/local_position/pose
- 或直接订阅 PX4 DDS /fmu/out/vehicle_odometry
- 优先订阅 MAVROS 原生速度
- 重新发布为 /ekf2/pose (标准 PoseStamped 格式)
- 同时发布为 /ekf2/odom (Odometry 格式，便于对比)
- 当原生速度不可用时，退化为位姿差分速度
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from px4_msgs.msg import VehicleOdometry
import numpy as np
from collections import deque
from typing import Optional
from scipy.spatial.transform import Rotation


class EKF2StateRelay(Node):
    def __init__(self):
        super().__init__('ekf2_state_relay')
        
        # 参数
        self.input_mode = self.declare_parameter('input_mode', 'mavros_pose').value
        self.input_topic = self.declare_parameter('input_topic', '/mavros/local_position/pose').value
        self.vehicle_odometry_topic = self.declare_parameter(
            'vehicle_odometry_topic', '/fmu/out/vehicle_odometry').value
        self.velocity_topic = self.declare_parameter(
            'velocity_topic', '/mavros/local_position/velocity_local').value
        self.output_topic = self.declare_parameter('output_topic', '/ekf2/pose').value
        self.output_odom_topic = self.declare_parameter('output_odom_topic', self.output_topic + '_odom').value
        self.publish_pose = bool(self.declare_parameter('publish_pose', True).value)
        self.use_input_stamp = self.declare_parameter('use_input_stamp', True).value
        self.use_covariance = self.declare_parameter('use_covariance', False).value
        self.frame_id = self.declare_parameter('frame_id', 'map').value
        self.child_frame_id = self.declare_parameter('child_frame_id', 'base_link').value
        self.prefer_native_velocity = self.declare_parameter('prefer_native_velocity', True).value
        self.max_native_velocity_age_sec = float(
            self.declare_parameter('max_native_velocity_age_sec', 0.5).value)
        
        # 发布者 - 使用可靠的 QoS 匹配后处理需求
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.pose_pub = None
        if self.publish_pose:
            self.pose_pub = self.create_publisher(
                PoseStamped, 
                self.output_topic,
                qos
            )
        
        self.odom_pub = self.create_publisher(
            Odometry,
            self.output_odom_topic,  # default: /ekf2/pose_odom
            qos
        )
        
        # 订阅 - 使用传感器 QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.pose_sub = None
        self.vehicle_odometry_sub = None
        self.velocity_sub = None
        self.warned_velocity_frame = False

        if self.input_mode == 'mavros_pose':
            self.pose_sub = self.create_subscription(
                PoseStamped,
                self.input_topic,
                self.pose_callback,
                sensor_qos
            )

            self.velocity_sub = self.create_subscription(
                TwistStamped,
                self.velocity_topic,
                self.velocity_callback,
                sensor_qos
            )
        elif self.input_mode == 'px4_vehicle_odometry':
            self.vehicle_odometry_sub = self.create_subscription(
                VehicleOdometry,
                self.vehicle_odometry_topic,
                self.vehicle_odometry_callback,
                sensor_qos
            )
        else:
            raise RuntimeError(f'Unsupported input_mode: {self.input_mode}')
        
        # 速度估计 / 缓存
        self.last_pose: Optional[PoseStamped] = None
        self.last_time: Optional[float] = None
        self.velocity_history = deque(maxlen=5)
        self.last_native_velocity: Optional[np.ndarray] = None
        self.last_native_velocity_time: Optional[float] = None
        self.logged_native_velocity = False
        self.logged_fallback_velocity = False
        
        self.get_logger().info(
            f"EKF2StateRelay 已启动\n"
            f"  输入模式: {self.input_mode}\n"
            f"  输入: {self.input_topic if self.input_mode == 'mavros_pose' else self.vehicle_odometry_topic}\n"
            f"  使用输入时间戳: {self.use_input_stamp}\n"
            f"  速度: {self.velocity_topic if self.input_mode == 'mavros_pose' else 'embedded in vehicle_odometry'}\n"
            f"  输出 pose: {self.output_topic if self.publish_pose else '(disabled)'}\n"
            f"  输出 odom: {self.output_odom_topic}"
        )

    def velocity_callback(self, msg: TwistStamped):
        """缓存 MAVROS 原生速度"""
        try:
            self.last_native_velocity = np.array([
                msg.twist.linear.x,
                msg.twist.linear.y,
                msg.twist.linear.z,
            ], dtype=float)
            self.last_native_velocity_time = self.msg_to_sec(msg.header.stamp)
        except Exception as e:
            self.get_logger().error(f"处理速度消息时出错: {e}")

    def pose_callback(self, msg: PoseStamped):
        """处理输入的位姿消息"""
        try:
            stamp = msg.header.stamp if self.use_input_stamp else self.get_clock().now().to_msg()

            # 1. 直接发布位姿 (仅调整 frame_id)
            pose_out = PoseStamped()
            pose_out.header = Header()
            pose_out.header.stamp = stamp
            pose_out.header.frame_id = self.frame_id
            pose_out.pose = msg.pose
            
            if self.pose_pub is not None:
                self.pose_pub.publish(pose_out)
            
            # 2. 生成 Odometry（优先使用原生速度）
            current_time_sec = self.msg_to_sec(stamp)

            odom = Odometry()
            odom.header = Header()
            odom.header.stamp = stamp
            odom.header.frame_id = self.frame_id
            odom.child_frame_id = self.child_frame_id
            odom.pose.pose = msg.pose

            velocity = self.resolve_velocity(current_time_sec, msg)
            if velocity is not None:
                odom.twist.twist.linear.x = float(velocity[0])
                odom.twist.twist.linear.y = float(velocity[1])
                odom.twist.twist.linear.z = float(velocity[2])

            if self.use_covariance:
                # TODO: 从 MAVROS 获取协方差
                odom.pose.covariance = [0.0] * 36
                odom.twist.covariance = [0.0] * 36

            self.odom_pub.publish(odom)
            
            self.last_pose = pose_out
            self.last_time = current_time_sec
            
        except Exception as e:
            self.get_logger().error(f"处理消息时出错: {e}")

    def vehicle_odometry_callback(self, msg: VehicleOdometry):
        """处理 PX4 VehicleOdometry 并转成 ENU Pose/Odometry"""
        try:
            stamp = self.use_px4_stamp(msg.timestamp) if self.use_input_stamp else self.get_clock().now().to_msg()
            current_time_sec = self.msg_to_sec(stamp)

            pose_out = PoseStamped()
            pose_out.header = Header()
            pose_out.header.stamp = stamp
            pose_out.header.frame_id = self.frame_id

            # PX4 uses local NED position; convert to ROS ENU.
            pose_out.pose.position.x = float(msg.position[1])
            pose_out.pose.position.y = float(msg.position[0])
            pose_out.pose.position.z = float(-msg.position[2])

            quat_ned = np.array([msg.q[1], msg.q[2], msg.q[3], msg.q[0]], dtype=float)
            r_ned = Rotation.from_quat(quat_ned)
            t_enu_from_ned = np.array([
                [0.0, 1.0, 0.0],
                [1.0, 0.0, 0.0],
                [0.0, 0.0, -1.0],
            ], dtype=float)
            t_frd_from_flu = np.array([
                [1.0, 0.0, 0.0],
                [0.0, -1.0, 0.0],
                [0.0, 0.0, -1.0],
            ], dtype=float)
            quat_enu = Rotation.from_matrix(
                t_enu_from_ned @ r_ned.as_matrix() @ t_frd_from_flu
            ).as_quat()
            pose_out.pose.orientation.x = float(quat_enu[0])
            pose_out.pose.orientation.y = float(quat_enu[1])
            pose_out.pose.orientation.z = float(quat_enu[2])
            pose_out.pose.orientation.w = float(quat_enu[3])

            if self.pose_pub is not None:
                self.pose_pub.publish(pose_out)

            odom = Odometry()
            odom.header = Header()
            odom.header.stamp = stamp
            odom.header.frame_id = self.frame_id
            odom.child_frame_id = self.child_frame_id
            odom.pose.pose = pose_out.pose

            velocity = self.resolve_vehicle_odometry_velocity(msg, r_ned)
            if velocity is not None:
                self.last_native_velocity = velocity.copy()
                self.last_native_velocity_time = current_time_sec
                odom.twist.twist.linear.x = float(velocity[0])
                odom.twist.twist.linear.y = float(velocity[1])
                odom.twist.twist.linear.z = float(velocity[2])

            if self.use_covariance:
                odom.pose.covariance = [0.0] * 36
                odom.twist.covariance = [0.0] * 36

            self.odom_pub.publish(odom)
            self.last_pose = pose_out
            self.last_time = current_time_sec
        except Exception as e:
            self.get_logger().error(f"处理 vehicle_odometry 时出错: {e}")

    def msg_to_sec(self, stamp) -> float:
        """将 ROS 时间戳转换为秒"""
        return stamp.sec + stamp.nanosec * 1e-9

    def use_px4_stamp(self, timestamp_us: int):
        """将 PX4 微秒时间戳转换为 ROS Time 消息"""
        stamp_sec = float(timestamp_us) * 1e-6
        sec = int(stamp_sec)
        nanosec = int(round((stamp_sec - sec) * 1e9))
        if nanosec >= 1_000_000_000:
            sec += 1
            nanosec -= 1_000_000_000
        stamp = self.get_clock().now().to_msg()
        stamp.sec = sec
        stamp.nanosec = nanosec
        return stamp

    def resolve_vehicle_odometry_velocity(
        self, msg: VehicleOdometry, r_ned: Rotation
    ) -> Optional[np.ndarray]:
        """将 VehicleOdometry 速度转换到 ENU"""
        velocity = np.array(msg.velocity, dtype=float)
        if not np.all(np.isfinite(velocity)):
            return None

        if msg.velocity_frame == VehicleOdometry.VELOCITY_FRAME_NED:
            return np.array([velocity[1], velocity[0], -velocity[2]], dtype=float)

        if msg.velocity_frame == VehicleOdometry.VELOCITY_FRAME_BODY_FRD:
            vel_ned = r_ned.apply(velocity)
            return np.array([vel_ned[1], vel_ned[0], -vel_ned[2]], dtype=float)

        if not self.warned_velocity_frame:
            self.get_logger().warn(
                f'Unsupported vehicle_odometry velocity_frame={msg.velocity_frame}, fallback may use pose diff')
            self.warned_velocity_frame = True
        return None

    def resolve_velocity(self, current_time_sec: float, msg: PoseStamped) -> Optional[np.ndarray]:
        """优先返回原生速度，缺失时退化为位姿差分"""
        if self.prefer_native_velocity and self.last_native_velocity is not None and self.last_native_velocity_time is not None:
            age = abs(current_time_sec - self.last_native_velocity_time)
            if age <= max(0.01, self.max_native_velocity_age_sec):
                if not self.logged_native_velocity:
                    self.get_logger().info(
                        f"使用 MAVROS 原生速度: {self.velocity_topic} (max_age={self.max_native_velocity_age_sec:.2f}s)")
                    self.logged_native_velocity = True
                return self.last_native_velocity.copy()

        if self.last_pose is None or self.last_time is None:
            return None

        dt = current_time_sec - self.last_time
        if dt <= 0.001:
            return None

        dx = msg.pose.position.x - self.last_pose.pose.position.x
        dy = msg.pose.position.y - self.last_pose.pose.position.y
        dz = msg.pose.position.z - self.last_pose.pose.position.z

        velocity = np.array([dx / dt, dy / dt, dz / dt], dtype=float)
        self.velocity_history.append(velocity)
        velocities = np.array(list(self.velocity_history))

        if not self.logged_fallback_velocity:
            self.get_logger().warn(
                f"未收到新鲜原生速度，退化为 pose 差分速度: {self.velocity_topic}")
            self.logged_fallback_velocity = True

        return np.mean(velocities, axis=0)


def main(args=None):
    rclpy.init(args=args)
    node = EKF2StateRelay()
    
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
