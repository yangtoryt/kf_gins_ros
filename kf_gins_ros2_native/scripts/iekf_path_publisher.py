#!/usr/bin/env python3
"""
IEKF 轨迹发布器
将 /kf_gins/odom (Odometry) 转换为 /kf_gins/path (Path)
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from std_msgs.msg import UInt32
import math

class IEKFPathPublisher(Node):
    def __init__(self):
        super().__init__('iekf_path_publisher')
        
        # 参数
        self.input_topic = self.declare_parameter('input_topic', '/kf_gins/odom').value
        # 避免和 kf_gins_node 自带的 /kf_gins/path 冲突
        self.output_topic = self.declare_parameter('output_topic', '/kf_gins/path_iekf').value
        self.frame_id = self.declare_parameter('frame_id', 'map').value
        self.max_path_length = int(self.declare_parameter('max_path_length', 1000).value)
        self.min_distance = float(self.declare_parameter('min_distance', 0.05).value)  # 最小距离阈值（米）
        self.require_armed = bool(self.declare_parameter('require_armed', True).value)
        self.clear_on_arm_transition = bool(self.declare_parameter('clear_on_arm_transition', False).value)
        self.clear_on_reset_event = bool(self.declare_parameter('clear_on_reset_event', False).value)
        self.mavros_state_topic = self.declare_parameter('mavros_state_topic', '/mavros/state').value
        self.reset_event_topic = self.declare_parameter('reset_event_topic', '/kf_gins/reset_event').value
        self.startup_ignore_sec = float(self.declare_parameter('startup_ignore_sec', 3.0).value)
        self.max_abs_position_m = float(self.declare_parameter('max_abs_position_m', 10000.0).value)
        self.max_abs_velocity_mps = float(self.declare_parameter('max_abs_velocity_mps', 100.0).value)
        self.max_jump_distance_m = float(self.declare_parameter('max_jump_distance_m', 50.0).value)
        
        # QoS 配置
        qos_sub = QoSProfile(
            depth=100,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )
        qos_pub = QoSProfile(
            depth=10,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        qos_state = QoSProfile(
            depth=10,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )
        
        # 订阅 IEKF odom
        self.sub = self.create_subscription(
            Odometry,
            self.input_topic,
            self.odom_callback,
            qos_sub
        )

        self.state_sub = self.create_subscription(
            State,
            self.mavros_state_topic,
            self.state_callback,
            qos_state
        )
        self.reset_sub = self.create_subscription(
            UInt32,
            self.reset_event_topic,
            self.reset_event_callback,
            qos_state
        )
        
        # 发布 IEKF path
        self.pub = self.create_publisher(Path, self.output_topic, qos_pub)
        
        # 初始化路径
        self.path = Path()
        self.path.header.frame_id = self.frame_id
        self.last_pos = None
        self.mavros_armed = False
        self._start_time_sec = self.get_clock().now().nanoseconds * 1e-9
        
        self.get_logger().info('IEKF Path Publisher 已启动')
        self.get_logger().info(f'订阅话题: {self.input_topic}')
        self.get_logger().info(f'发布话题: {self.output_topic}')
        self.get_logger().info(
            f'门控: require_armed={self.require_armed}, '
            f'startup_ignore_sec={self.startup_ignore_sec:.1f}, '
            f'max_jump_distance_m={self.max_jump_distance_m:.1f}, '
            f'clear_on_reset_event={self.clear_on_reset_event}'
        )

    def clear_path(self):
        self.path = Path()
        self.path.header.frame_id = self.frame_id
        self.last_pos = None
        self.pub.publish(self.path)

    def restart_segment(self):
        # Preserve existing history but let the next odom sample start a new segment.
        self.last_pos = None

    def reset_event_callback(self, msg: UInt32):
        if self.clear_on_reset_event:
            self.get_logger().warn(
                f'IEKF reset event #{msg.data}: clearing IEKF path due to clear_on_reset_event=true.',
                throttle_duration_sec=1.0,
            )
            self.clear_path()
            return
        self.get_logger().warn(
            f'IEKF reset event #{msg.data}: preserving IEKF path history and restarting segment accumulation.',
            throttle_duration_sec=1.0,
        )
        self.restart_segment()

    def state_callback(self, msg: State):
        prev = self.mavros_armed
        self.mavros_armed = bool(msg.armed)
        if prev != self.mavros_armed:
            self.get_logger().info(f'Path armed transition: {prev} -> {self.mavros_armed}')
            if self.clear_on_arm_transition:
                self.clear_path()

    def odom_callback(self, msg: Odometry):
        """处理接收到的 IEKF odometry"""
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if now_sec - self._start_time_sec < self.startup_ignore_sec:
            return
        if self.require_armed and not self.mavros_armed:
            return

        pos = msg.pose.pose.position
        twist = msg.twist.twist.linear

        vals = [pos.x, pos.y, pos.z, twist.x, twist.y, twist.z]
        if not all(math.isfinite(v) for v in vals):
            self.get_logger().warn('Skip IEKF path point: non-finite pose/twist.', throttle_duration_sec=2.0)
            return

        speed = math.sqrt(twist.x * twist.x + twist.y * twist.y + twist.z * twist.z)
        if max(abs(pos.x), abs(pos.y), abs(pos.z)) > self.max_abs_position_m:
            self.get_logger().warn(
                f'Skip IEKF path point: absurd position=({pos.x:.3f},{pos.y:.3f},{pos.z:.3f}).',
                throttle_duration_sec=2.0,
            )
            return
        if speed > self.max_abs_velocity_mps:
            self.get_logger().warn(
                f'Skip IEKF path point: absurd speed={speed:.3f} m/s.',
                throttle_duration_sec=2.0,
            )
            return

        if self.last_pos:
            dx = pos.x - self.last_pos[0]
            dy = pos.y - self.last_pos[1]
            dz = pos.z - self.last_pos[2]
            dist = math.sqrt(dx*dx + dy*dy + dz*dz)

            if dist > self.max_jump_distance_m:
                self.get_logger().warn(
                    f'Skip IEKF path jump: dist={dist:.3f} m exceeds {self.max_jump_distance_m:.3f} m.',
                    throttle_duration_sec=2.0,
                )
                return
            if dist < self.min_distance:
                return
        
        # 保存当前位置
        self.last_pos = (pos.x, pos.y, pos.z)
        
        # 转换为 PoseStamped
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.header.frame_id = self.frame_id
        pose_stamped.pose = msg.pose.pose
        
        # 添加到路径
        self.path.header.stamp = msg.header.stamp
        self.path.header.frame_id = self.frame_id
        self.path.poses.append(pose_stamped)
        
        # 限制路径长度
        if len(self.path.poses) > self.max_path_length:
            self.path.poses.pop(0)
        
        # 发布路径
        self.pub.publish(self.path)


def main():
    rclpy.init()
    node = IEKFPathPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
