#!/usr/bin/env python3
"""
EKF2 轨迹发布器
将 /ekf2/pose (PoseStamped) 转换为 /ekf2/path (Path)
"""
import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

class EKF2PathPublisher(Node):
    def __init__(self):
        super().__init__("ekf2_path_publisher")

        # 参数
        self.input_topic = self.declare_parameter("input_topic", "/ekf2/pose").value
        self.output_topic = self.declare_parameter("output_topic", "/ekf2/path").value
        self.frame_id = self.declare_parameter("frame_id", "map").value
        self.max_path_length = int(
            self.declare_parameter("max_path_length", 1000).value
        )
        self.min_distance = float(
            self.declare_parameter("min_distance", 0.05).value
        )
        self.require_armed = bool(
            self.declare_parameter("require_armed", True).value
        )
        self.clear_on_arm_transition = bool(
            self.declare_parameter("clear_on_arm_transition", False).value
        )
        self.mavros_state_topic = str(
            self.declare_parameter("mavros_state_topic", "/mavros/state").value
        )
        self.startup_ignore_sec = float(
            self.declare_parameter("startup_ignore_sec", 3.0).value
        )
        self.max_abs_position_m = float(
            self.declare_parameter("max_abs_position_m", 10000.0).value
        )
        self.max_jump_distance_m = float(
            self.declare_parameter("max_jump_distance_m", 50.0).value
        )
        self.mavros_state_stale_sec = max(
            0.1,
            float(self.declare_parameter("mavros_state_stale_sec", 1.5).value),
        )
        self.armed_false_hold_sec = max(
            0.0,
            float(self.declare_parameter("armed_false_hold_sec", 6.0).value),
        )

        # QoS：订阅用 best_effort，发布 Path 用 reliable（RViz 更容易显示）
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

        # 订阅 EKF2 pose
        self.sub = self.create_subscription(
            PoseStamped,
            self.input_topic,
            self.pose_callback,
            qos_sub,
        )

        # 发布 EKF2 path
        self.pub = self.create_publisher(Path, self.output_topic, qos_pub)

        # 初始化路径
        self.path = Path()
        self.path.header.frame_id = self.frame_id
        self.last_pos = None
        self.mavros_armed = False
        self.have_mavros_state = False
        self.mavros_state_rx_sec: Optional[float] = None
        self.path_armed = False
        self._pending_disarm_since_sec: Optional[float] = None
        self._start_time_sec = self.get_clock().now().nanoseconds * 1e-9

        if self.require_armed or self.clear_on_arm_transition:
            qos_state = QoSProfile(
                depth=10,
                history=QoSHistoryPolicy.KEEP_LAST,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
            )
            self.state_sub = self.create_subscription(
                State,
                self.mavros_state_topic,
                self._state_callback,
                qos_state,
            )

        self.get_logger().info("EKF2 Path Publisher 已启动")
        self.get_logger().info(f"订阅话题: {self.input_topic}")
        self.get_logger().info(f"发布话题: {self.output_topic}")
        self.get_logger().info(
            f"门控: require_armed={self.require_armed}, "
            f"startup_ignore_sec={self.startup_ignore_sec:.1f}, "
            f"max_abs_position_m={self.max_abs_position_m:.1f}, "
            f"max_jump_distance_m={self.max_jump_distance_m:.1f}, "
            f"clear_on_arm_transition={self.clear_on_arm_transition}, "
            f"mavros_state_stale_sec={self.mavros_state_stale_sec:.1f}, "
            f"armed_false_hold_sec={self.armed_false_hold_sec:.1f}"
        )

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _mavros_state_age_sec(self, now_sec: Optional[float] = None) -> float:
        if now_sec is None:
            now_sec = self._now_sec()
        if self.mavros_state_rx_sec is None:
            return math.inf
        return max(0.0, now_sec - self.mavros_state_rx_sec)

    def _mavros_state_fresh(self, now_sec: Optional[float] = None) -> bool:
        return self._mavros_state_age_sec(now_sec) <= self.mavros_state_stale_sec

    def _update_path_armed(self, now_sec: Optional[float] = None, reason: str = "pose") -> bool:
        if now_sec is None:
            now_sec = self._now_sec()

        state_fresh = self.have_mavros_state and self._mavros_state_fresh(now_sec)
        candidate_armed = state_fresh and self.mavros_armed
        next_armed = self.path_armed

        if candidate_armed:
            self._pending_disarm_since_sec = None
            next_armed = True
        elif self.path_armed and self.armed_false_hold_sec > 0.0:
            if self._pending_disarm_since_sec is None:
                self._pending_disarm_since_sec = now_sec
            next_armed = (now_sec - self._pending_disarm_since_sec) < self.armed_false_hold_sec
        else:
            self._pending_disarm_since_sec = None
            next_armed = False

        if next_armed != self.path_armed:
            state_age_sec = self._mavros_state_age_sec(now_sec)
            state_age_str = "n/a" if not math.isfinite(state_age_sec) else f"{state_age_sec:.2f}s"
            self.get_logger().info(
                f"Path armed transition: {self.path_armed} -> {next_armed} "
                f"(reason={reason}, raw_armed={self.mavros_armed}, "
                f"state_fresh={state_fresh}, state_age={state_age_str})"
            )
            self.path_armed = next_armed
            if self.clear_on_arm_transition:
                self.get_logger().info("Clearing EKF2 path after armed-state transition.")
                self._clear_path()

        return self.path_armed

    def _clear_path(self):
        self.path = Path()
        self.path.header.frame_id = self.frame_id
        self.last_pos = None
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.path)

    def _state_callback(self, msg: State):
        self.mavros_armed = bool(msg.armed)
        self.have_mavros_state = True
        self.mavros_state_rx_sec = self._now_sec()
        self._update_path_armed(self.mavros_state_rx_sec, reason="state")

    def pose_callback(self, msg: PoseStamped):
        """处理接收到的 EKF2 pose"""

        now_sec = self._now_sec()
        if now_sec - self._start_time_sec < self.startup_ignore_sec:
            return
        if self.require_armed:
            if not self._update_path_armed(now_sec, reason="pose"):
                return

        # 检查距离阈值，避免点太密集
        pos = msg.pose.position
        vals = [pos.x, pos.y, pos.z]
        if not all(math.isfinite(v) for v in vals):
            self.get_logger().warn(
                "Skip EKF2 path point: non-finite position.",
                throttle_duration_sec=2.0,
            )
            return
        if max(abs(pos.x), abs(pos.y), abs(pos.z)) > self.max_abs_position_m:
            self.get_logger().warn(
                f"Skip EKF2 path point: absurd position=({pos.x:.3f},{pos.y:.3f},{pos.z:.3f}).",
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
                    f"Skip EKF2 path jump: dist={dist:.3f} m exceeds {self.max_jump_distance_m:.3f} m.",
                    throttle_duration_sec=2.0,
                )
                return
            if dist < self.min_distance:
                return

        # 保存当前位置
        self.last_pos = (pos.x, pos.y, pos.z)

        # 添加到路径
        self.path.header.stamp = msg.header.stamp
        self.path.header.frame_id = self.frame_id
        self.path.poses.append(msg)

        # 限制路径长度
        if len(self.path.poses) > self.max_path_length:
            self.path.poses.pop(0)

        # 发布路径
        self.pub.publish(self.path)


def main():
    rclpy.init()
    node = EKF2PathPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
