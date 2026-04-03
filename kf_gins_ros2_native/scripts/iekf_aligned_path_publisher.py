#!/usr/bin/env python3
"""
IEKF 对齐轨迹发布器
将 /iekf/state_aligned/position (Vector3Stamped) 转换为 /iekf/path_aligned (Path)
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from geometry_msgs.msg import Vector3Stamped, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import UInt32
import math
from mavros_msgs.msg import State


class IEKFAlignedPathPublisher(Node):
    def __init__(self):
        super().__init__("iekf_aligned_path_publisher")

        self.input_topic = self.declare_parameter(
            "input_topic", "/iekf/state_aligned/position"
        ).value
        self.output_topic = self.declare_parameter(
            "output_topic", "/iekf/path_aligned"
        ).value
        self.frame_id = self.declare_parameter("frame_id", "map").value
        self.max_path_length = int(
            self.declare_parameter("max_path_length", 20000).value
        )
        self.min_distance = float(
            self.declare_parameter("min_distance", 0.05).value
        )
        self.require_armed = bool(
            self.declare_parameter("require_armed", True).value
        )
        self.clear_on_arm_transition = bool(
            self.declare_parameter("clear_on_arm_transition", True).value
        )
        self.clear_on_reset_event = bool(
            self.declare_parameter("clear_on_reset_event", False).value
        )
        self.max_abs_position_m = float(
            self.declare_parameter("max_abs_position_m", 10000.0).value
        )
        self.max_jump_distance_m = float(
            self.declare_parameter("max_jump_distance_m", 50.0).value
        )
        self.mavros_state_topic = str(
            self.declare_parameter("mavros_state_topic", "/mavros/state").value
        )
        self.reset_event_topic = str(
            self.declare_parameter("reset_event_topic", "/kf_gins/reset_event").value
        )

        qos_sub = QoSProfile(
            depth=100,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        qos_pub = QoSProfile(
            depth=10,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )

        self.sub = self.create_subscription(
            Vector3Stamped,
            self.input_topic,
            self.vec_callback,
            qos_sub,
        )
        self.pub = self.create_publisher(Path, self.output_topic, qos_pub)

        self.path = Path()
        self.path.header.frame_id = self.frame_id
        self.last_pos = None
        self.mavros_armed = False
        self.have_mavros_state = False

        if self.require_armed:
            qos_state = QoSProfile(
                depth=10,
                history=QoSHistoryPolicy.KEEP_LAST,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
            )
            self.state_sub = self.create_subscription(
                State,
                self.mavros_state_topic,
                self._on_state,
                qos_state,
            )
        else:
            qos_state = QoSProfile(
                depth=10,
                history=QoSHistoryPolicy.KEEP_LAST,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
            )
        self.reset_sub = self.create_subscription(
            UInt32,
            self.reset_event_topic,
            self._on_reset_event,
            qos_state,
        )

        self.get_logger().info("IEKF Aligned Path Publisher 已启动")
        self.get_logger().info(f"订阅话题: {self.input_topic}")
        self.get_logger().info(f"发布话题: {self.output_topic}")
        self.get_logger().info(
            f"门控: require_armed={self.require_armed}, "
            f"max_abs_position_m={self.max_abs_position_m:.1f}, "
            f"max_jump_distance_m={self.max_jump_distance_m:.1f}, "
            f"clear_on_reset_event={self.clear_on_reset_event}"
        )

    def _clear_path(self):
        self.path = Path()
        self.path.header.frame_id = self.frame_id
        self.last_pos = None
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.path)

    def _restart_segment(self):
        # Preserve existing history but let the next aligned point start a new segment.
        self.last_pos = None

    def _on_state(self, msg: State):
        prev = self.mavros_armed
        self.mavros_armed = bool(msg.armed)
        self.have_mavros_state = True
        if self.clear_on_arm_transition and prev != self.mavros_armed:
            self._clear_path()

    def _on_reset_event(self, msg: UInt32):
        if self.clear_on_reset_event:
            self.get_logger().warn(
                f"IEKF reset event #{msg.data}: clearing aligned path due to clear_on_reset_event=true.",
                throttle_duration_sec=1.0,
            )
            self._clear_path()
            return
        self.get_logger().warn(
            f"IEKF reset event #{msg.data}: preserving aligned path history and restarting segment accumulation.",
            throttle_duration_sec=1.0,
        )
        self._restart_segment()

    def vec_callback(self, msg: Vector3Stamped):
        if self.require_armed:
            if not self.have_mavros_state:
                return
            if not self.mavros_armed:
                return
        pos = msg.vector
        vals = [pos.x, pos.y, pos.z]
        if not all(math.isfinite(v) for v in vals):
            self.get_logger().warn(
                "Skip aligned path point: non-finite position.",
                throttle_duration_sec=2.0,
            )
            return
        if max(abs(pos.x), abs(pos.y), abs(pos.z)) > self.max_abs_position_m:
            self.get_logger().warn(
                f"Skip aligned path point: absurd position=({pos.x:.3f},{pos.y:.3f},{pos.z:.3f}).",
                throttle_duration_sec=2.0,
            )
            return
        if self.last_pos:
            dx = pos.x - self.last_pos[0]
            dy = pos.y - self.last_pos[1]
            dz = pos.z - self.last_pos[2]
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)
            if dist > self.max_jump_distance_m:
                self.get_logger().warn(
                    f"Skip aligned path jump: dist={dist:.3f} m exceeds {self.max_jump_distance_m:.3f} m.",
                    throttle_duration_sec=2.0,
                )
                return
            if dist < self.min_distance:
                return

        self.last_pos = (pos.x, pos.y, pos.z)

        pose = PoseStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = float(pos.x)
        pose.pose.position.y = float(pos.y)
        pose.pose.position.z = float(pos.z)
        pose.pose.orientation.w = 1.0

        self.path.header.stamp = msg.header.stamp
        self.path.header.frame_id = self.frame_id
        self.path.poses.append(pose)

        if len(self.path.poses) > self.max_path_length:
            self.path.poses.pop(0)

        self.pub.publish(self.path)


def main():
    rclpy.init()
    node = IEKFAlignedPathPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
