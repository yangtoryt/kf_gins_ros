#!/usr/bin/env python3
"""
Lightweight PX4 flight status monitor for SITL/QGC debugging.

Prints a compact line with armed/nav_state/landed/altitude and a few
cross-check fields so it is easy to see whether QGC, MAVROS and PX4 DDS agree.
"""

import math
from typing import Dict, Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from mavros_msgs.msg import ExtendedState, State, WaypointList, WaypointReached
from px4_msgs.msg import VehicleLandDetected, VehicleLocalPosition, VehicleStatus
from sensor_msgs.msg import BatteryState


def build_enum_map(cls, prefix: str) -> Dict[int, str]:
    mapping: Dict[int, str] = {}
    for name in dir(cls):
        if not name.startswith(prefix):
            continue
        value = getattr(cls, name)
        if isinstance(value, int):
            mapping[value] = name[len(prefix):]
    return mapping


class PX4FlightStatusMonitor(Node):
    def __init__(self) -> None:
        super().__init__("px4_flight_status_monitor")

        self.vehicle_status_topic = self.declare_parameter(
            "vehicle_status_topic", "/fmu/out/vehicle_status").value
        self.vehicle_local_position_topic = self.declare_parameter(
            "vehicle_local_position_topic", "/fmu/out/vehicle_local_position").value
        self.vehicle_land_detected_topic = self.declare_parameter(
            "vehicle_land_detected_topic", "/fmu/out/vehicle_land_detected").value
        self.mavros_state_topic = self.declare_parameter(
            "mavros_state_topic", "/mavros/state").value
        self.mavros_extended_state_topic = self.declare_parameter(
            "mavros_extended_state_topic", "/mavros/extended_state").value
        self.mavros_mission_waypoints_topic = self.declare_parameter(
            "mavros_mission_waypoints_topic", "/mavros/mission/waypoints").value
        self.mavros_mission_reached_topic = self.declare_parameter(
            "mavros_mission_reached_topic", "/mavros/mission/reached").value
        self.mavros_battery_topic = self.declare_parameter(
            "mavros_battery_topic", "/mavros/battery").value
        self.print_hz = max(0.2, float(self.declare_parameter("print_hz", 2.0).value))
        self.stale_sec = max(0.1, float(self.declare_parameter("stale_sec", 1.5).value))
        self.show_xy = bool(self.declare_parameter("show_xy", True).value)
        self.show_mission = bool(self.declare_parameter("show_mission", True).value)
        self.show_battery = bool(self.declare_parameter("show_battery", True).value)

        self.nav_state_map = build_enum_map(VehicleStatus, "NAVIGATION_STATE_")
        self.arming_state_map = build_enum_map(VehicleStatus, "ARMING_STATE_")
        self.landed_state_map = build_enum_map(ExtendedState, "LANDED_STATE_")
        self.battery_status_map = build_enum_map(BatteryState, "POWER_SUPPLY_STATUS_")
        self.battery_health_map = build_enum_map(BatteryState, "POWER_SUPPLY_HEALTH_")

        self.vehicle_status: Optional[VehicleStatus] = None
        self.vehicle_local_position: Optional[VehicleLocalPosition] = None
        self.vehicle_land_detected: Optional[VehicleLandDetected] = None
        self.mavros_state: Optional[State] = None
        self.mavros_extended_state: Optional[ExtendedState] = None
        self.mission_waypoints: Optional[WaypointList] = None
        self.last_reached_seq: Optional[int] = None
        self.mavros_battery: Optional[BatteryState] = None

        self.vehicle_status_rx_sec: Optional[float] = None
        self.vehicle_local_position_rx_sec: Optional[float] = None
        self.vehicle_land_detected_rx_sec: Optional[float] = None
        self.mavros_state_rx_sec: Optional[float] = None
        self.mavros_extended_state_rx_sec: Optional[float] = None
        self.mission_waypoints_rx_sec: Optional[float] = None
        self.last_reached_rx_sec: Optional[float] = None
        self.mavros_battery_rx_sec: Optional[float] = None

        self.last_line = ""

        qos_px4 = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )
        qos_mavros = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(
            VehicleStatus, self.vehicle_status_topic, self.vehicle_status_cb, qos_px4)
        self.create_subscription(
            VehicleLocalPosition, self.vehicle_local_position_topic, self.vehicle_local_position_cb, qos_px4)
        self.create_subscription(
            VehicleLandDetected, self.vehicle_land_detected_topic, self.vehicle_land_detected_cb, qos_px4)
        self.create_subscription(
            State, self.mavros_state_topic, self.mavros_state_cb, qos_mavros)
        self.create_subscription(
            ExtendedState, self.mavros_extended_state_topic, self.mavros_extended_state_cb, qos_mavros)
        self.create_subscription(
            WaypointList, self.mavros_mission_waypoints_topic, self.mission_waypoints_cb, qos_mavros)
        self.create_subscription(
            WaypointReached, self.mavros_mission_reached_topic, self.mission_reached_cb, qos_mavros)
        self.create_subscription(
            BatteryState, self.mavros_battery_topic, self.mavros_battery_cb, qos_mavros)

        self.create_timer(1.0 / self.print_hz, self.report_status)

        self.get_logger().info(
            "PX4 flight status monitor started\n"
            f"  PX4 status: {self.vehicle_status_topic}\n"
            f"  PX4 local position: {self.vehicle_local_position_topic}\n"
            f"  PX4 land detected: {self.vehicle_land_detected_topic}\n"
            f"  MAVROS state: {self.mavros_state_topic}\n"
            f"  MAVROS extended state: {self.mavros_extended_state_topic}\n"
            f"  MAVROS mission: {self.mavros_mission_waypoints_topic}\n"
            f"  MAVROS mission reached: {self.mavros_mission_reached_topic}\n"
            f"  MAVROS battery: {self.mavros_battery_topic}\n"
            f"  print_hz={self.print_hz:.1f}, stale_sec={self.stale_sec:.1f}"
        )

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def vehicle_status_cb(self, msg: VehicleStatus) -> None:
        self.vehicle_status = msg
        self.vehicle_status_rx_sec = self.now_sec()

    def vehicle_local_position_cb(self, msg: VehicleLocalPosition) -> None:
        self.vehicle_local_position = msg
        self.vehicle_local_position_rx_sec = self.now_sec()

    def vehicle_land_detected_cb(self, msg: VehicleLandDetected) -> None:
        self.vehicle_land_detected = msg
        self.vehicle_land_detected_rx_sec = self.now_sec()

    def mavros_state_cb(self, msg: State) -> None:
        self.mavros_state = msg
        self.mavros_state_rx_sec = self.now_sec()

    def mavros_extended_state_cb(self, msg: ExtendedState) -> None:
        self.mavros_extended_state = msg
        self.mavros_extended_state_rx_sec = self.now_sec()

    def mission_waypoints_cb(self, msg: WaypointList) -> None:
        self.mission_waypoints = msg
        self.mission_waypoints_rx_sec = self.now_sec()

    def mission_reached_cb(self, msg: WaypointReached) -> None:
        self.last_reached_seq = int(msg.wp_seq)
        self.last_reached_rx_sec = self.now_sec()

    def mavros_battery_cb(self, msg: BatteryState) -> None:
        self.mavros_battery = msg
        self.mavros_battery_rx_sec = self.now_sec()

    def is_fresh(self, rx_sec: Optional[float]) -> bool:
        return rx_sec is not None and (self.now_sec() - rx_sec) <= self.stale_sec

    def format_age(self, rx_sec: Optional[float]) -> str:
        if rx_sec is None:
            return "n/a"
        return f"{self.now_sec() - rx_sec:.2f}s"

    def enum_name(self, value: int, mapping: Dict[int, str]) -> str:
        return mapping.get(value, f"UNKNOWN_{value}")

    def resolve_px4_armed(self) -> str:
        if not self.is_fresh(self.vehicle_status_rx_sec) or self.vehicle_status is None:
            return "?"
        armed = self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED
        return "1" if armed else "0"

    def resolve_nav_state(self) -> str:
        if not self.is_fresh(self.vehicle_status_rx_sec) or self.vehicle_status is None:
            return "?"
        nav_state = int(self.vehicle_status.nav_state)
        return f"{self.enum_name(nav_state, self.nav_state_map)}({nav_state})"

    def resolve_landed(self) -> str:
        if self.is_fresh(self.vehicle_land_detected_rx_sec) and self.vehicle_land_detected is not None:
            if self.vehicle_land_detected.landed:
                return "ON_GROUND(px4)"
            if self.vehicle_land_detected.maybe_landed:
                return "MAYBE_LANDED(px4)"
            if self.vehicle_land_detected.ground_contact:
                return "GROUND_CONTACT(px4)"
            return "IN_AIR(px4)"

        if self.is_fresh(self.mavros_extended_state_rx_sec) and self.mavros_extended_state is not None:
            landed_state = int(self.mavros_extended_state.landed_state)
            return f"{self.enum_name(landed_state, self.landed_state_map)}(mavros)"

        return "?"

    def resolve_altitude(self) -> str:
        if not self.is_fresh(self.vehicle_local_position_rx_sec) or self.vehicle_local_position is None:
            return "?"

        z = float(self.vehicle_local_position.z)
        if not math.isfinite(z):
            return "?"
        return f"{-z:.2f}m"

    def resolve_xy(self) -> str:
        if not self.show_xy or not self.is_fresh(self.vehicle_local_position_rx_sec) or self.vehicle_local_position is None:
            return ""

        x = float(self.vehicle_local_position.x)
        y = float(self.vehicle_local_position.y)
        if not (math.isfinite(x) and math.isfinite(y)):
            return ""
        return f" xy=({x:.2f},{y:.2f})"

    def resolve_source_summary(self) -> str:
        parts = []

        if self.is_fresh(self.vehicle_status_rx_sec) and self.vehicle_status is not None:
            arming_state = int(self.vehicle_status.arming_state)
            parts.append(f"px4_arm={self.enum_name(arming_state, self.arming_state_map)}")

        if self.is_fresh(self.mavros_state_rx_sec) and self.mavros_state is not None:
            parts.append(
                f"mavros_arm={'1' if self.mavros_state.armed else '0'} "
                f"mode={self.mavros_state.mode or '?'}")

        ages = (
            f"age(px4_status={self.format_age(self.vehicle_status_rx_sec)}, "
            f"local_pos={self.format_age(self.vehicle_local_position_rx_sec)}, "
            f"landed={self.format_age(self.vehicle_land_detected_rx_sec)}, "
            f"mavros_state={self.format_age(self.mavros_state_rx_sec)}, "
            f"ext={self.format_age(self.mavros_extended_state_rx_sec)})"
        )
        parts.append(ages)

        if (
            self.is_fresh(self.vehicle_status_rx_sec)
            and self.vehicle_status is not None
            and self.is_fresh(self.mavros_state_rx_sec)
            and self.mavros_state is not None
        ):
            px4_armed = self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED
            if px4_armed != bool(self.mavros_state.armed):
                parts.append("ARM_DISAGREE")

        return " | ".join(parts)

    def resolve_mission_summary(self) -> str:
        if not self.show_mission:
            return ""

        parts = []
        if self.mission_waypoints is not None:
            current_seq = int(self.mission_waypoints.current_seq)
            count = len(self.mission_waypoints.waypoints)
            current_cmd = "?"
            if 0 <= current_seq < count:
                current_cmd = str(int(self.mission_waypoints.waypoints[current_seq].command))
            parts.append(f"mission={current_seq}/{count}(cmd={current_cmd})")
            parts.append(f"mission_age={self.format_age(self.mission_waypoints_rx_sec)}")

        if self.last_reached_seq is not None:
            parts.append(f"reached={self.last_reached_seq}")
            parts.append(f"reached_age={self.format_age(self.last_reached_rx_sec)}")

        return " ".join(parts)

    def resolve_battery_summary(self) -> str:
        if not self.show_battery or self.mavros_battery is None:
            return ""

        msg = self.mavros_battery
        parts = []

        if math.isfinite(float(msg.percentage)) and msg.percentage >= 0.0:
            parts.append(f"bat={100.0 * float(msg.percentage):.0f}%")
        else:
            parts.append("bat=?")

        if math.isfinite(float(msg.voltage)) and msg.voltage > 0.0:
            parts.append(f"{float(msg.voltage):.2f}V")

        if math.isfinite(float(msg.current)):
            parts.append(f"{float(msg.current):.2f}A")

        parts.append(self.enum_name(int(msg.power_supply_status), self.battery_status_map))
        parts.append(self.enum_name(int(msg.power_supply_health), self.battery_health_map))
        parts.append(f"bat_age={self.format_age(self.mavros_battery_rx_sec)}")
        return " ".join(parts)

    def report_status(self) -> None:
        line = (
            f"armed={self.resolve_px4_armed()} "
            f"nav={self.resolve_nav_state()} "
            f"landed={self.resolve_landed()} "
            f"alt={self.resolve_altitude()}"
            f"{self.resolve_xy()} | {self.resolve_source_summary()}"
        )

        mission_summary = self.resolve_mission_summary()
        battery_summary = self.resolve_battery_summary()
        if mission_summary:
            line += f" | {mission_summary}"
        if battery_summary:
            line += f" | {battery_summary}"

        if line != self.last_line:
            self.get_logger().info(line)
            self.last_line = line
        else:
            self.get_logger().debug(line)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PX4FlightStatusMonitor()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
