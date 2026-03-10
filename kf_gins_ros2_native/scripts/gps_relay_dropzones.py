#!/usr/bin/env python3
import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import rclpy
from rclpy.exceptions import ParameterAlreadyDeclaredException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import NavSatFix, NavSatStatus
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from mavros_msgs.msg import State


@dataclass
class BoxZone:
    xmin: float
    xmax: float
    ymin: float
    ymax: float
    zmin: float
    zmax: float

    def contains(self, x: float, y: float, z: float) -> bool:
        return (self.xmin <= x <= self.xmax and
                self.ymin <= y <= self.ymax and
                self.zmin <= z <= self.zmax)

    def contains_with_margin(self, x: float, y: float, z: float, margin: float) -> bool:
        # margin > 0 expands the box; margin < 0 shrinks it
        return (self.xmin - margin <= x <= self.xmax + margin and
                self.ymin - margin <= y <= self.ymax + margin and
                self.zmin - margin <= z <= self.zmax + margin)


def parse_zone(s: str) -> BoxZone:
    parts = [p.strip() for p in s.split(",")]
    if len(parts) != 6:
        raise ValueError(f"Zone must have 6 comma-separated numbers, got: {s}")
    vals = list(map(float, parts))
    xmin, xmax, ymin, ymax, zmin, zmax = vals
    if xmin > xmax: xmin, xmax = xmax, xmin
    if ymin > ymax: ymin, ymax = ymax, ymin
    if zmin > zmax: zmin, zmax = zmax, zmin
    return BoxZone(xmin, xmax, ymin, ymax, zmin, zmax)


class GpsRelayDropZones(Node):
    def __init__(self):
        super().__init__("gps_relay_dropzones")

        # ROS 2 may pre-declare `use_sim_time` internally; avoid re-declare crash.
        try:
            self.declare_parameter("use_sim_time", False)
        except ParameterAlreadyDeclaredException:
            pass
        self.declare_parameter("in_fix_topic", "/mavros/global_position/raw/fix")
        self.declare_parameter("pose_topic", "/mavros/local_position/pose")
        self.declare_parameter("out_fix_topic", "/gps/fix")
        self.declare_parameter("mavros_state_topic", "/mavros/state")
        self.declare_parameter("require_armed_for_drop", True)

        # zones: list of strings "xmin,xmax,ymin,ymax,zmin,zmax"
        # NOTE: declaring with an empty Python list defaults to BYTE_ARRAY in rclpy.
        # We must explicitly declare this parameter as STRING_ARRAY for YAML override.
        self.declare_parameter("zones", rclpy.Parameter.Type.STRING_ARRAY)
        # drop_mode: "status" (publish NO_FIX), "stop" (publish nothing), "nan" (publish NaN lat/lon)
        self.declare_parameter("drop_mode", "status")
        # hysteresis meters: avoid chattering on boundary; 0 = off
        self.declare_parameter("hysteresis", 1.0)
        # min_drop_duration seconds: once dropped, keep dropped for at least this long
        self.declare_parameter("min_drop_duration", 0.5)
        # covariance diag value when dropped (for status/nan modes)
        self.declare_parameter("covariance_on_drop", 1e6)

        # RViz 可视化
        self.declare_parameter("publish_markers", True)
        self.declare_parameter("marker_topic", "/gps_dropzones/markers")
        self.declare_parameter("marker_frame_id", "map")
        self.declare_parameter("marker_publish_rate_hz", 1.0)
        self.declare_parameter("marker_alpha", 0.18)
        self.declare_parameter("marker_alpha_active", 0.35)
        self.declare_parameter("publish_current_pose_marker", True)

        in_fix_topic = self.get_parameter("in_fix_topic").value
        pose_topic = self.get_parameter("pose_topic").value
        out_fix_topic = self.get_parameter("out_fix_topic").value
        mavros_state_topic = self.get_parameter("mavros_state_topic").value
        self.require_armed_for_drop = bool(self.get_parameter("require_armed_for_drop").value)

        zones_raw = self.get_parameter("zones").value
        self.zones: List[BoxZone] = []
        for zs in zones_raw:
            try:
                self.zones.append(parse_zone(str(zs)))
            except Exception as e:
                self.get_logger().error(f"Failed to parse zone '{zs}': {e}")

        self.drop_mode = str(self.get_parameter("drop_mode").value).strip().lower()
        self.hysteresis = float(self.get_parameter("hysteresis").value)
        self.min_drop_duration = float(self.get_parameter("min_drop_duration").value)
        self.cov_on_drop = float(self.get_parameter("covariance_on_drop").value)

        self.publish_markers = bool(self.get_parameter("publish_markers").value)
        self.marker_topic = str(self.get_parameter("marker_topic").value)
        self.marker_frame_id = str(self.get_parameter("marker_frame_id").value)
        self.marker_publish_rate_hz = float(self.get_parameter("marker_publish_rate_hz").value)
        self.marker_alpha = float(self.get_parameter("marker_alpha").value)
        self.marker_alpha_active = float(self.get_parameter("marker_alpha_active").value)
        self.publish_current_pose_marker = bool(self.get_parameter("publish_current_pose_marker").value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.sub_pose = self.create_subscription(PoseStamped, pose_topic, self._on_pose, qos)
        self.sub_fix = self.create_subscription(NavSatFix, in_fix_topic, self._on_fix, qos)
        self.sub_state = self.create_subscription(State, mavros_state_topic, self._on_state, qos)
        self.pub_fix = self.create_publisher(NavSatFix, out_fix_topic, 10)

        self.last_pose: Optional[Tuple[float, float, float]] = None
        self.last_good_fix: Optional[NavSatFix] = None
        self._dropped = False
        self._drop_until_time = 0.0
        self._armed = False

        self.marker_pub = None
        self.marker_timer = None
        if self.publish_markers:
            marker_qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            )
            self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, marker_qos)
            hz = max(0.1, self.marker_publish_rate_hz)
            self.marker_timer = self.create_timer(1.0 / hz, self._publish_markers)

        self.get_logger().info(f"in_fix_topic: {in_fix_topic}")
        self.get_logger().info(f"pose_topic  : {pose_topic}")
        self.get_logger().info(f"mavros_state_topic: {mavros_state_topic}")
        self.get_logger().info(f"out_fix_topic: {out_fix_topic}")
        self.get_logger().info(f"drop_mode: {self.drop_mode}, hysteresis: {self.hysteresis} m, min_drop_duration: {self.min_drop_duration} s")
        self.get_logger().info(f"require_armed_for_drop: {self.require_armed_for_drop}")
        self.get_logger().info(f"zones({len(self.zones)}): " + " | ".join([f"[{z.xmin},{z.xmax}]x[{z.ymin},{z.ymax}]x[{z.zmin},{z.zmax}]" for z in self.zones]))
        if self.publish_markers:
            self.get_logger().info(f"markers: topic={self.marker_topic}, frame_id={self.marker_frame_id}, rate={self.marker_publish_rate_hz} Hz")

    def _on_state(self, msg: State):
        self._armed = bool(msg.armed)

    def _on_pose(self, msg: PoseStamped):
        p = msg.pose.position
        self.last_pose = (float(p.x), float(p.y), float(p.z))

    def _inside_any_zone(self, x: float, y: float, z: float) -> bool:
        # With hysteresis:
        # - when currently NOT dropped: use standard zone (no expansion) for first entry
        # - when currently dropped: use shrunken boxes to exit harder (hysteresis resistance)
        # 【修复】避免初始化时hysteresis导致zone过度扩大，防止误判
        if self.hysteresis <= 0.0:
            return any(zone.contains(x, y, z) for zone in self.zones)

        if not self._dropped:
            # First entry: use exact zone bounds, no margin (strict entry condition)
            # 【修复】改为不用margin的contains，确保初始化安全
            return any(zone.contains(x, y, z) for zone in self.zones)
        else:
            # Already dropped: use negative margin to make zone smaller (harder to exit)
            return any(zone.contains_with_margin(x, y, z, -self.hysteresis) for zone in self.zones)

    def _on_fix(self, msg: NavSatFix):
        # Ensure we pass through at least one valid fix to allow downstream nodes
        # (e.g. KF-GINS) to initialize even if the vehicle starts inside a zone.
        # 【修复】记录第一个好的fix，无论何时都需要它
        is_first_fix = self.last_good_fix is None
        if is_first_fix and msg.status.status != NavSatStatus.STATUS_NO_FIX:
            self.last_good_fix = msg
            self.get_logger().warn(f"[GNSS RELAY] FIRST GOOD FIX received: lat={msg.latitude:.6f}, lon={msg.longitude:.6f}, alt={msg.altitude:.2f}m")

        # 【修复】如果还没有pose，无条件通过所有GNSS，不检查zone
        # 这确保IEKF能正常初始化，而不是在坐标系不确定时就被zone过滤
        if self.last_pose is None:
            self.pub_fix.publish(msg)
            self.get_logger().info(f"[GNSS RELAY] Waiting for first pose, relaying GNSS: lat={msg.latitude:.6f}, lon={msg.longitude:.6f}")
            return
        
        if len(self.zones) == 0:
            self.pub_fix.publish(msg)
            self.get_logger().warn("[GNSS RELAY] No zones configured, relaying all GNSS")
            return

        if self.require_armed_for_drop and not self._armed:
            self.pub_fix.publish(msg)
            self.get_logger().debug("[GNSS RELAY] Disarmed, relaying GNSS (dropzone disabled).")
            return

        x, y, z = self.last_pose
        now = self.get_clock().now().nanoseconds * 1e-9

        inside = self._inside_any_zone(x, y, z)
        self.get_logger().debug(f"[GNSS RELAY] Pos({x:.1f},{y:.1f},{z:.1f}) inside_zone={inside}, dropped={self._dropped}")

        # enforce minimum drop duration to avoid flickering
        if self._dropped and now < self._drop_until_time:
            inside = True
            self.get_logger().debug(f"[GNSS RELAY] Min drop duration active until {self._drop_until_time:.2f}s (now={now:.2f}s)")

        if inside:
            if not self._dropped:
                self._dropped = True
                self._drop_until_time = now + self.min_drop_duration
                self.get_logger().warn(f"[GNSS RELAY] DROP ENTER zone at pos=({x:.2f},{y:.2f},{z:.2f}), will stay dropped until {self._drop_until_time:.2f}s")
            else:
                self.get_logger().debug(f"[GNSS RELAY] Already dropped, publishing NO_FIX at pos=({x:.2f},{y:.2f},{z:.2f})")
            self._publish_dropped(msg)
        else:
            if self._dropped:
                self._dropped = False
                self.get_logger().info(f"[GNSS RELAY] DROP EXIT zone at pos=({x:.2f},{y:.2f},{z:.2f}), resuming normal GNSS relay")
            self.last_good_fix = msg
            self.pub_fix.publish(msg)
            self.get_logger().debug(f"[GNSS RELAY] Outside zone, relaying GNSS at pos=({x:.2f},{y:.2f},{z:.2f})")

    def _publish_dropped(self, incoming: NavSatFix):
        if self.drop_mode == "stop":
            return

        out = NavSatFix()
        out.header = incoming.header

        if self.drop_mode == "nan":
            out.latitude = float("nan")
            out.longitude = float("nan")
            out.altitude = float("nan")
        else:
            # "status": keep last good value if available, else keep incoming value
            src = self.last_good_fix if self.last_good_fix is not None else incoming
            out.latitude = src.latitude
            out.longitude = src.longitude
            out.altitude = src.altitude

        out.status.status = NavSatStatus.STATUS_NO_FIX
        out.status.service = 0

        # covariance: set very large to indicate unusable
        out.position_covariance = [
            self.cov_on_drop, 0.0, 0.0,
            0.0, self.cov_on_drop, 0.0,
            0.0, 0.0, self.cov_on_drop,
        ]
        out.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        self.pub_fix.publish(out)

    def _publish_markers(self):
        if self.marker_pub is None:
            return

        now = self.get_clock().now().to_msg()

        active_idx = None
        if self.last_pose is not None:
            x, y, z = self.last_pose
            for i, zone in enumerate(self.zones):
                if zone.contains(x, y, z):
                    active_idx = i
                    break

        arr = MarkerArray()

        for i, zone in enumerate(self.zones):
            m = Marker()
            m.header.stamp = now
            m.header.frame_id = self.marker_frame_id
            m.ns = "gps_dropzones"
            m.id = int(i)
            m.type = Marker.CUBE
            m.action = Marker.ADD

            cx = 0.5 * (zone.xmin + zone.xmax)
            cy = 0.5 * (zone.ymin + zone.ymax)
            cz = 0.5 * (zone.zmin + zone.zmax)
            sx = abs(zone.xmax - zone.xmin)
            sy = abs(zone.ymax - zone.ymin)
            sz = abs(zone.zmax - zone.zmin)

            m.pose.position.x = float(cx)
            m.pose.position.y = float(cy)
            m.pose.position.z = float(cz)
            m.pose.orientation.w = 1.0

            m.scale.x = float(max(0.01, sx))
            m.scale.y = float(max(0.01, sy))
            m.scale.z = float(max(0.01, sz))

            is_active = (active_idx == i)
            m.color.r = 1.0
            m.color.g = 0.2 if is_active else 0.1
            m.color.b = 0.1
            m.color.a = float(self.marker_alpha_active if is_active else self.marker_alpha)

            arr.markers.append(m)

        if self.publish_current_pose_marker and self.last_pose is not None:
            x, y, z = self.last_pose
            p = Marker()
            p.header.stamp = now
            p.header.frame_id = self.marker_frame_id
            p.ns = "gps_dropzones"
            p.id = 1000
            p.type = Marker.SPHERE
            p.action = Marker.ADD
            p.pose.position.x = float(x)
            p.pose.position.y = float(y)
            p.pose.position.z = float(z)
            p.pose.orientation.w = 1.0
            p.scale.x = 0.6
            p.scale.y = 0.6
            p.scale.z = 0.6
            p.color.r = 0.2
            p.color.g = 0.8
            p.color.b = 1.0
            p.color.a = 0.9
            arr.markers.append(p)

        self.marker_pub.publish(arr)


def main():
    rclpy.init()
    node = GpsRelayDropZones()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
