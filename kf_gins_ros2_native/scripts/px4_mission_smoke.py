#!/usr/bin/env python3

import argparse
import json
from pathlib import Path
import sys
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State, Waypoint, WaypointList, WaypointReached
from mavros_msgs.srv import CommandBool, SetMode, WaypointClear, WaypointPull, WaypointPush, WaypointSetCurrent
from nav_msgs.msg import Odometry
from px4_msgs.msg import SensorCombined, VehicleAttitude, VehicleLocalPosition, VehicleOdometry
from sensor_msgs.msg import Imu, NavSatFix


@dataclass(frozen=True)
class MissionItem:
    command: int
    frame: int
    lat: float
    lon: float
    alt: float
    param1: float = 0.0
    param2: float = 0.0
    param3: float = 0.0
    param4: float = 0.0


@dataclass(frozen=True)
class GapProbeTopic:
    topic: str
    msg_type: type
    stamp_kind: str = "header"
    stamp_attr: Optional[str] = None
    stamp_scale: float = 1.0


@dataclass
class GapProbeSampleState:
    samples: int = 0
    count: int = 0
    last_wall: Optional[float] = None
    last_stamp: Optional[float] = None
    events: List[str] = None

    def __post_init__(self) -> None:
        if self.events is None:
            self.events = []


QGC_ROUTE_APR15_50M: List[MissionItem] = [
    MissionItem(command=22, frame=Waypoint.FRAME_GLOBAL_RELATIVE_ALT_INT, lat=47.39775, lon=8.545607, alt=50.0),
    MissionItem(command=16, frame=Waypoint.FRAME_GLOBAL_RELATIVE_ALT_INT, lat=47.39774, lon=8.547511, alt=50.0),
    MissionItem(command=16, frame=Waypoint.FRAME_GLOBAL_RELATIVE_ALT_INT, lat=47.39768, lon=8.550354, alt=50.0),
    MissionItem(command=16, frame=Waypoint.FRAME_GLOBAL_RELATIVE_ALT_INT, lat=47.39774, lon=8.546535, alt=50.0),
    MissionItem(command=16, frame=Waypoint.FRAME_GLOBAL_RELATIVE_ALT_INT, lat=47.39780, lon=8.544003, alt=50.0),
    MissionItem(command=16, frame=Waypoint.FRAME_GLOBAL_RELATIVE_ALT_INT, lat=47.39772, lon=8.548058, alt=50.0),
    MissionItem(command=20, frame=Waypoint.FRAME_MISSION, lat=0.0, lon=0.0, alt=0.0),
]


GAP_PROBE_PROFILES: Dict[str, List[GapProbeTopic]] = {
    "none": [],
    "imu_raw": [
        GapProbeTopic("imu/data_raw", Imu),
    ],
    "mavros_highrate": [
        GapProbeTopic("imu/data_raw", Imu),
        GapProbeTopic("imu/data", Imu),
        GapProbeTopic("local_position/velocity_local", TwistStamped),
        GapProbeTopic("local_position/pose", PoseStamped),
    ],
    "runtime_crosscheck": [
        GapProbeTopic("/fmu/out/sensor_combined", SensorCombined, stamp_kind="attr", stamp_attr="timestamp", stamp_scale=1e-6),
        GapProbeTopic(
            "/fmu/out/vehicle_local_position",
            VehicleLocalPosition,
            stamp_kind="attr",
            stamp_attr="timestamp",
            stamp_scale=1e-6,
        ),
        GapProbeTopic("imu/data_raw", Imu),
        GapProbeTopic("imu/data", Imu),
        GapProbeTopic("local_position/velocity_local", TwistStamped),
        GapProbeTopic("local_position/pose", PoseStamped),
    ],
    "directdds_compare_chain": [
        GapProbeTopic("/fmu/out/sensor_combined", SensorCombined, stamp_kind="attr", stamp_attr="timestamp", stamp_scale=1e-6),
        GapProbeTopic(
            "/fmu/out/vehicle_local_position",
            VehicleLocalPosition,
            stamp_kind="attr",
            stamp_attr="timestamp",
            stamp_scale=1e-6,
        ),
        GapProbeTopic(
            "/fmu/out/vehicle_attitude",
            VehicleAttitude,
            stamp_kind="attr",
            stamp_attr="timestamp",
            stamp_scale=1e-6,
        ),
        GapProbeTopic(
            "/fmu/out/vehicle_odometry",
            VehicleOdometry,
            stamp_kind="attr",
            stamp_attr="timestamp",
            stamp_scale=1e-6,
        ),
        GapProbeTopic("/mavros/global_position/raw/fix", NavSatFix),
        GapProbeTopic("/gps/fix", NavSatFix),
        GapProbeTopic("/ekf2/pose_odom", Odometry),
        GapProbeTopic("/kf_gins/odom", Odometry),
    ],
    "directdds_relay_chain": [
        GapProbeTopic("/fmu/out/sensor_combined", SensorCombined, stamp_kind="attr", stamp_attr="timestamp", stamp_scale=1e-6),
        GapProbeTopic(
            "/fmu/out/vehicle_local_position",
            VehicleLocalPosition,
            stamp_kind="attr",
            stamp_attr="timestamp",
            stamp_scale=1e-6,
        ),
        GapProbeTopic(
            "/fmu/out/vehicle_attitude",
            VehicleAttitude,
            stamp_kind="attr",
            stamp_attr="timestamp",
            stamp_scale=1e-6,
        ),
        GapProbeTopic(
            "/fmu/out/vehicle_odometry",
            VehicleOdometry,
            stamp_kind="attr",
            stamp_attr="timestamp",
            stamp_scale=1e-6,
        ),
        GapProbeTopic("/gps/fix", NavSatFix),
        GapProbeTopic("/px4_aux/imu/data", Imu),
        GapProbeTopic("/px4_aux/local_position/velocity_local", TwistStamped),
        GapProbeTopic("/ekf2/pose_odom", Odometry),
        GapProbeTopic("/kf_gins/odom", Odometry),
        GapProbeTopic("/kf_gins/odom_raw", Odometry),
    ],
}


class MissionSmoke(Node):
    def __init__(self, mavros_ns: str, timeout_sec: float) -> None:
        super().__init__("px4_mission_smoke")
        self.timeout_sec = timeout_sec
        self.state: Optional[State] = None
        self.waypoints: Optional[WaypointList] = None
        self.last_reached: Optional[int] = None

        ns = mavros_ns.rstrip("/")
        self.state_topic = f"{ns}/state"
        self.waypoints_topic = f"{ns}/mission/waypoints"
        self.reached_topic = f"{ns}/mission/reached"

        self.clear_service = f"{ns}/mission/clear"
        self.pull_service = f"{ns}/mission/pull"
        self.push_service = f"{ns}/mission/push"
        self.set_current_service = f"{ns}/mission/set_current"
        self.arm_service = f"{ns}/cmd/arming"
        self.set_mode_service = f"{ns}/set_mode"

        self.create_subscription(State, self.state_topic, self._state_cb, 10)
        self.create_subscription(WaypointList, self.waypoints_topic, self._waypoints_cb, 10)
        self.create_subscription(WaypointReached, self.reached_topic, self._reached_cb, 10)

        self.clear_client = self.create_client(WaypointClear, self.clear_service)
        self.pull_client = self.create_client(WaypointPull, self.pull_service)
        self.push_client = self.create_client(WaypointPush, self.push_service)
        self.set_current_client = self.create_client(WaypointSetCurrent, self.set_current_service)
        self.arm_client = self.create_client(CommandBool, self.arm_service)
        self.mode_client = self.create_client(SetMode, self.set_mode_service)

        self.gap_probe_profile = "none"
        self.gap_probe_threshold_sec = 0.9
        self.gap_probe_window_sec = 0.0
        self.gap_probe_active = False
        self.gap_probe_deadline: Optional[float] = None
        self.gap_probe_start_delay_sec = 0.0
        self.gap_probe_start_after_reached: Optional[int] = None
        self.gap_probe_start_at_mono: Optional[float] = None
        self.gap_probe_start_wait_logged = False
        self.gap_probe_states: Dict[str, GapProbeSampleState] = {}
        self.gap_probe_subscriptions = []
        self.gap_probe_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )

    def _state_cb(self, msg: State) -> None:
        self.state = msg

    def _waypoints_cb(self, msg: WaypointList) -> None:
        self.waypoints = msg

    def _reached_cb(self, msg: WaypointReached) -> None:
        self.last_reached = int(msg.wp_seq)

    def spin_until(self, predicate, timeout_sec: Optional[float] = None, label: str = "condition") -> None:
        timeout = self.timeout_sec if timeout_sec is None else timeout_sec
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if predicate():
                return
        raise RuntimeError(f"timeout waiting for {label}")

    def wait_for_service(self, client, name: str) -> None:
        if not client.wait_for_service(timeout_sec=self.timeout_sec):
            raise RuntimeError(f"timeout waiting for service {name}")

    def call(self, client, request, name: str):
        future = client.call_async(request)
        self.spin_until(lambda: future.done(), label=name)
        result = future.result()
        if result is None:
            raise RuntimeError(f"service {name} returned no result")
        return result

    def wait_for_connected(self) -> None:
        self.spin_until(lambda: self.state is not None and self.state.connected, label=self.state_topic)

    def clear_mission(self) -> WaypointClear.Response:
        self.wait_for_service(self.clear_client, self.clear_service)
        return self.call(self.clear_client, WaypointClear.Request(), self.clear_service)

    def pull_mission(self) -> WaypointPull.Response:
        self.wait_for_service(self.pull_client, self.pull_service)
        self.waypoints = None
        result = self.call(self.pull_client, WaypointPull.Request(), self.pull_service)
        self.spin_until(lambda: self.waypoints is not None, timeout_sec=min(self.timeout_sec, 5.0), label=self.waypoints_topic)
        return result

    def push_mission(self, waypoints: List[Waypoint]) -> WaypointPush.Response:
        self.wait_for_service(self.push_client, self.push_service)
        req = WaypointPush.Request()
        req.start_index = 0
        req.waypoints = waypoints
        return self.call(self.push_client, req, self.push_service)

    def set_current(self, seq: int) -> WaypointSetCurrent.Response:
        self.wait_for_service(self.set_current_client, self.set_current_service)
        req = WaypointSetCurrent.Request()
        req.wp_seq = int(seq)
        return self.call(self.set_current_client, req, self.set_current_service)

    def arm(self, value: bool) -> CommandBool.Response:
        self.wait_for_service(self.arm_client, self.arm_service)
        req = CommandBool.Request()
        req.value = bool(value)
        return self.call(self.arm_client, req, self.arm_service)

    def set_mode(self, custom_mode: str) -> SetMode.Response:
        self.wait_for_service(self.mode_client, self.set_mode_service)
        req = SetMode.Request()
        req.base_mode = 0
        req.custom_mode = custom_mode
        return self.call(self.mode_client, req, self.set_mode_service)

    def print_state(self, prefix: str = "[px4-smoke]") -> None:
        if self.state is None:
            print(f"{prefix} state: unavailable")
            return
        print(
            f"{prefix} state: connected={self.state.connected} armed={self.state.armed} "
            f"mode={self.state.mode} system_status={self.state.system_status}"
        )

    def print_mission(self, prefix: str = "[px4-smoke]") -> None:
        if self.waypoints is None:
            print(f"{prefix} mission: unavailable")
            return
        print(f"{prefix} mission: current_seq={self.waypoints.current_seq} count={len(self.waypoints.waypoints)}")
        if self.waypoints.current_seq < len(self.waypoints.waypoints):
            wp = self.waypoints.waypoints[self.waypoints.current_seq]
            print(
                f"{prefix} current waypoint: seq={self.waypoints.current_seq} "
                f"command={wp.command} frame={wp.frame} alt={wp.z_alt:.2f}"
            )

    def wait_for_mode(self, expected_mode: str, timeout_sec: float) -> None:
        self.spin_until(
            lambda: self.state is not None and self.state.mode == expected_mode,
            timeout_sec=timeout_sec,
            label=f"mode={expected_mode}",
        )

    def wait_for_armed(self, expected_value: bool, timeout_sec: float) -> None:
        self.spin_until(
            lambda: self.state is not None and self.state.armed == expected_value,
            timeout_sec=timeout_sec,
            label=f"armed={expected_value}",
        )

    def sleep_with_spin(self, duration_sec: float) -> None:
        deadline = time.monotonic() + duration_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=min(0.1, max(0.0, deadline - time.monotonic())))

    def configure_gap_probe(
        self,
        profile: str,
        threshold_sec: float,
        window_sec: float,
        start_delay_sec: float,
        start_after_reached: Optional[int],
    ) -> None:
        if profile not in GAP_PROBE_PROFILES:
            raise RuntimeError(f"unsupported gap probe profile: {profile}")
        if start_delay_sec < 0.0:
            raise RuntimeError("gap probe start delay must be >= 0")

        self.gap_probe_profile = profile
        self.gap_probe_threshold_sec = threshold_sec
        self.gap_probe_window_sec = window_sec
        self.gap_probe_start_delay_sec = start_delay_sec
        self.gap_probe_start_after_reached = start_after_reached

        if profile == "none" or window_sec <= 0.0:
            return

        for topic in GAP_PROBE_PROFILES[profile]:
            full_topic = topic.topic
            if not full_topic.startswith("/"):
                full_topic = f"{self.state_topic.rsplit('/', 1)[0]}/{full_topic}"
            self.gap_probe_states[full_topic] = GapProbeSampleState()
            self.gap_probe_subscriptions.append(
                self.create_subscription(
                    topic.msg_type,
                    full_topic,
                    self._make_gap_probe_cb(full_topic, topic),
                    self.gap_probe_qos,
                )
            )

    def _extract_gap_probe_stamp(self, msg, topic: GapProbeTopic) -> Optional[float]:
        if topic.stamp_kind == "header":
            header = getattr(msg, "header", None)
            if header is None:
                return None
            return header.stamp.sec + header.stamp.nanosec * 1e-9

        if topic.stamp_kind == "attr":
            if topic.stamp_attr is None:
                return None
            value = getattr(msg, topic.stamp_attr, None)
            if value is None:
                return None
            return float(value) * topic.stamp_scale

        raise RuntimeError(f"unsupported gap probe stamp kind: {topic.stamp_kind}")

    def _make_gap_probe_cb(self, topic_name: str, topic: GapProbeTopic):
        def cb(msg) -> None:
            if not self.gap_probe_active:
                return

            now = time.monotonic()
            if self.gap_probe_deadline is not None and now > self.gap_probe_deadline:
                self.gap_probe_active = False
                return

            stamp = self._extract_gap_probe_stamp(msg, topic)
            state = self.gap_probe_states[topic_name]
            state.samples += 1
            if state.last_wall is not None:
                wall_dt = now - state.last_wall
                stamp_dt: Optional[float] = None
                if stamp is not None and state.last_stamp is not None:
                    stamp_dt = stamp - state.last_stamp
                if wall_dt >= self.gap_probe_threshold_sec or (
                    stamp_dt is not None and stamp_dt >= self.gap_probe_threshold_sec
                ):
                    state.count += 1
                    stamp_dt_str = f"{stamp_dt:.6f}s" if stamp_dt is not None else "n/a"
                    state.events.append(
                        f"[px4-smoke] gap-probe: topic={topic_name} wall_dt={wall_dt:.6f}s "
                        f"stamp_dt={stamp_dt_str} mono_time={now:.6f}"
                    )
            state.last_wall = now
            state.last_stamp = stamp

        return cb

    def _reset_gap_probe_samples(self) -> None:
        for state in self.gap_probe_states.values():
            state.samples = 0
            state.count = 0
            state.last_wall = None
            state.last_stamp = None
            state.events.clear()

    def arm_gap_probe(self) -> None:
        if self.gap_probe_profile == "none" or self.gap_probe_window_sec <= 0.0:
            return

        self._reset_gap_probe_samples()
        self.gap_probe_active = False
        self.gap_probe_deadline = None
        self.gap_probe_start_at_mono = None
        self.gap_probe_start_wait_logged = False

        self.maybe_start_gap_probe(force_log=True)

    def maybe_start_gap_probe(self, force_log: bool = False) -> None:
        if self.gap_probe_profile == "none" or self.gap_probe_window_sec <= 0.0:
            return
        if self.gap_probe_active:
            return
        if self.gap_probe_deadline is not None:
            return

        now = time.monotonic()
        if self.gap_probe_start_after_reached is not None and (
            self.last_reached is None or self.last_reached < self.gap_probe_start_after_reached
        ):
            if force_log or not self.gap_probe_start_wait_logged:
                print(
                    f"[px4-smoke] gap-probe waiting: profile={self.gap_probe_profile} "
                    f"start_after_reached={self.gap_probe_start_after_reached} "
                    f"delay={self.gap_probe_start_delay_sec:.1f}s "
                    f"window={self.gap_probe_window_sec:.1f}s threshold={self.gap_probe_threshold_sec:.3f}s"
                )
                self.gap_probe_start_wait_logged = True
            return

        if self.gap_probe_start_at_mono is None:
            self.gap_probe_start_at_mono = now + self.gap_probe_start_delay_sec
            if self.gap_probe_start_delay_sec > 0.0 and (force_log or not self.gap_probe_start_wait_logged):
                trigger_desc = (
                    f"reached seq={self.gap_probe_start_after_reached}"
                    if self.gap_probe_start_after_reached is not None
                    else "AUTO.MISSION"
                )
                print(
                    f"[px4-smoke] gap-probe scheduled: trigger={trigger_desc} "
                    f"delay={self.gap_probe_start_delay_sec:.1f}s "
                    f"window={self.gap_probe_window_sec:.1f}s threshold={self.gap_probe_threshold_sec:.3f}s"
                )
                self.gap_probe_start_wait_logged = True

        if now < self.gap_probe_start_at_mono:
            return

        self.gap_probe_deadline = time.monotonic() + self.gap_probe_window_sec
        self.gap_probe_active = True
        trigger_desc = (
            f"reached seq={self.gap_probe_start_after_reached}"
            if self.gap_probe_start_after_reached is not None
            else "AUTO.MISSION"
        )
        print(
            f"[px4-smoke] gap-probe start: profile={self.gap_probe_profile} "
            f"trigger={trigger_desc} delay={self.gap_probe_start_delay_sec:.1f}s "
            f"window={self.gap_probe_window_sec:.1f}s threshold={self.gap_probe_threshold_sec:.3f}s"
        )

    def gap_probe_done(self) -> bool:
        if self.gap_probe_profile == "none" or self.gap_probe_window_sec <= 0.0:
            return True
        if self.gap_probe_deadline is None:
            return False
        return self.gap_probe_deadline is not None and time.monotonic() >= self.gap_probe_deadline

    def print_gap_probe_summary(self) -> None:
        if self.gap_probe_profile == "none" or self.gap_probe_window_sec <= 0.0:
            return

        for topic_name, state in self.gap_probe_states.items():
            print(
                f"[px4-smoke] gap-probe summary: topic={topic_name} "
                f"samples={state.samples} gaps={state.count}"
            )
            for event in state.events:
                print(event)

    def monitor_mission(
        self,
        success_seq: Optional[int],
        timeout_sec: float,
        print_period_sec: float,
        exit_after_gap_probe: bool,
        exit_after_reached_seq: Optional[int] = None,
        require_disarm_on_success: bool = False,
    ) -> bool:
        deadline = time.monotonic() + timeout_sec
        last_print = 0.0
        prev_reached = self.last_reached
        mission_success = False
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            now = time.monotonic()
            if self.last_reached != prev_reached:
                print(f"[px4-smoke] waypoint reached: seq={self.last_reached}")
                prev_reached = self.last_reached
                self.maybe_start_gap_probe(force_log=True)
            else:
                self.maybe_start_gap_probe()
            if now - last_print >= print_period_sec:
                mission_seq = self.waypoints.current_seq if self.waypoints is not None else None
                mode = self.state.mode if self.state is not None else "?"
                armed = self.state.armed if self.state is not None else "?"
                print(
                    f"[px4-smoke] progress: mode={mode} armed={armed} "
                    f"current_seq={mission_seq} last_reached={self.last_reached}"
                )
                last_print = now
            if exit_after_reached_seq is not None and self.last_reached is not None:
                if self.last_reached >= exit_after_reached_seq:
                    return True
            if success_seq is not None and self.last_reached is not None and self.last_reached >= success_seq:
                mission_success = True
            success_ready = mission_success
            if require_disarm_on_success:
                success_ready = mission_success and self.state is not None and not self.state.armed
            if exit_after_gap_probe and self.gap_probe_done():
                return True
            if success_ready and self.gap_probe_done():
                return True
        return mission_success


def build_waypoint(item: MissionItem, is_current: bool) -> Waypoint:
    wp = Waypoint()
    wp.frame = item.frame
    wp.command = item.command
    wp.is_current = is_current
    wp.autocontinue = True
    wp.param1 = item.param1
    wp.param2 = item.param2
    wp.param3 = item.param3
    wp.param4 = item.param4
    wp.x_lat = item.lat
    wp.y_long = item.lon
    wp.z_alt = item.alt
    return wp


def load_qgc_plan(path: str) -> List[MissionItem]:
    plan_path = Path(path)
    data = json.loads(plan_path.read_text())
    mission_items = data.get("mission", {}).get("items", [])
    if not mission_items:
        raise RuntimeError(f"QGC plan has no mission items: {plan_path}")

    loaded: List[MissionItem] = []
    for idx, item in enumerate(mission_items, start=1):
        if item.get("type", "SimpleItem") != "SimpleItem":
            raise RuntimeError(f"unsupported non-SimpleItem in QGC plan at item {idx}")
        params = item.get("params") or []
        if len(params) < 7:
            raise RuntimeError(f"QGC plan item {idx} has fewer than 7 params")
        loaded.append(
            MissionItem(
                command=int(item["command"]),
                frame=int(item["frame"]),
                param1=float(params[0] or 0.0),
                param2=float(params[1] or 0.0),
                param3=float(params[2] or 0.0),
                param4=float(params[3] or 0.0),
                lat=float(params[4] or 0.0),
                lon=float(params[5] or 0.0),
                alt=float(params[6] or 0.0),
            )
        )

    if loaded[0].command != 22:
        raise RuntimeError(f"QGC plan first item must be MAV_CMD_NAV_TAKEOFF, got {loaded[0].command}")
    return loaded


def default_success_seq(mission: List[Waypoint]) -> Optional[int]:
    for idx in range(len(mission) - 1, -1, -1):
        if mission[idx].command != 20:
            return idx
    return None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Push a MAVROS mission, arm, switch to AUTO.MISSION, and monitor execution."
    )
    parser.add_argument("--plan-file", default=None, help="optional QGroundControl .plan file to push instead of the built-in route")
    parser.add_argument("--mavros-ns", default="/mavros", help="MAVROS namespace, default: /mavros")
    parser.add_argument("--timeout", type=float, default=15.0, help="service/topic timeout in seconds")
    parser.add_argument("--arm-timeout", type=float, default=20.0, help="arming/mode switch timeout in seconds")
    parser.add_argument("--pre-arm-settle", type=float, default=8.0, help="settle time before first arm attempt")
    parser.add_argument("--arm-retries", type=int, default=6, help="number of arm retries before failing")
    parser.add_argument("--arm-retry-delay", type=float, default=5.0, help="delay between arm retries in seconds")
    parser.add_argument("--mission-timeout", type=float, default=300.0, help="mission monitor timeout in seconds")
    parser.add_argument(
        "--post-mission-settle",
        type=float,
        default=0.0,
        help="after mission success, keep spinning for this many seconds before exiting",
    )
    parser.add_argument("--print-period", type=float, default=5.0, help="status print period in seconds")
    parser.add_argument(
        "--skip-mission-setup",
        action="store_true",
        help="skip mission clear/push/arm/mode setup and only attach to an already-running MAVROS/PX4 session",
    )
    parser.add_argument(
        "--wait-for-mode",
        default=None,
        help="when skipping mission setup, wait for this mode before starting monitoring/probing",
    )
    parser.add_argument(
        "--gap-probe-profile",
        choices=sorted(GAP_PROBE_PROFILES.keys()),
        default="none",
        help="optional topic set to probe for wall/stamp gaps during AUTO.MISSION",
    )
    parser.add_argument(
        "--gap-probe-window",
        type=float,
        default=0.0,
        help="gap probe window in seconds after entering AUTO.MISSION; 0 disables probing",
    )
    parser.add_argument(
        "--gap-probe-threshold",
        type=float,
        default=0.9,
        help="report a gap when wall_dt or stamp_dt reaches this threshold in seconds",
    )
    parser.add_argument(
        "--gap-probe-start-delay",
        type=float,
        default=0.0,
        help="delay gap probe start by this many seconds after its trigger condition fires",
    )
    parser.add_argument(
        "--gap-probe-start-after-reached",
        type=int,
        default=None,
        help="start gap probe only after /mission/reached reports this waypoint seq",
    )
    parser.add_argument(
        "--exit-after-gap-probe",
        action="store_true",
        help="exit successfully once the configured gap probe window finishes, without waiting for mission completion",
    )
    parser.add_argument(
        "--exit-after-auto-mission",
        action="store_true",
        help="exit successfully immediately after AUTO.MISSION is observed",
    )
    parser.add_argument(
        "--exit-after-reached",
        type=int,
        default=None,
        help="exit successfully once /mission/reached reports this waypoint seq",
    )
    parser.add_argument(
        "--require-disarm-on-success",
        action="store_true",
        help="after the success waypoint is reached, keep monitoring until the vehicle is disarmed",
    )
    parser.add_argument(
        "--skip-pull-mission-on-exit",
        action="store_true",
        help="skip mission pull/print during shutdown; useful when runtime MAVROS does not load the waypoint plugin",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    mission_items = load_qgc_plan(args.plan_file) if args.plan_file else QGC_ROUTE_APR15_50M
    mission = [build_waypoint(item, is_current=(idx == 0)) for idx, item in enumerate(mission_items)]
    success_seq: Optional[int] = default_success_seq(mission)

    rclpy.init()
    node = MissionSmoke(args.mavros_ns, args.timeout)
    try:
        if args.skip_mission_setup and args.gap_probe_profile != "none":
            if args.gap_probe_start_after_reached is None and args.wait_for_mode is None:
                raise RuntimeError(
                    "skip-mission-setup with an immediate gap probe requires --wait-for-mode "
                    "(typically AUTO.MISSION)"
                )
        if args.exit_after_auto_mission and args.skip_mission_setup and args.wait_for_mode not in (None, "AUTO.MISSION"):
            raise RuntimeError("--exit-after-auto-mission only supports --wait-for-mode AUTO.MISSION in skip-mission-setup mode")

        node.configure_gap_probe(
            profile=args.gap_probe_profile,
            threshold_sec=args.gap_probe_threshold,
            window_sec=args.gap_probe_window,
            start_delay_sec=args.gap_probe_start_delay,
            start_after_reached=args.gap_probe_start_after_reached,
        )
        node.wait_for_connected()
        node.print_state()

        if args.skip_mission_setup:
            success_seq = None
            if args.wait_for_mode is not None:
                node.wait_for_mode(args.wait_for_mode, timeout_sec=args.arm_timeout)
                node.print_state()
        else:
            clear_result = node.clear_mission()
            print(f"[px4-smoke] clear result: success={clear_result.success}")

            push_result = node.push_mission(mission)
            print(
                f"[px4-smoke] push result: success={push_result.success} "
                f"wp_transfered={push_result.wp_transfered}"
            )
            if not push_result.success or push_result.wp_transfered != len(mission):
                raise RuntimeError("mission push failed")

            pull_result = node.pull_mission()
            print(f"[px4-smoke] pull result: success={pull_result.success} wp_received={pull_result.wp_received}")
            node.print_mission()

            set_current_result = node.set_current(0)
            print(f"[px4-smoke] set-current result: seq=0 success={set_current_result.success}")
            if not set_current_result.success:
                raise RuntimeError("failed to set current waypoint to takeoff item")

            if args.pre_arm_settle > 0.0:
                print(f"[px4-smoke] settling before arm for {args.pre_arm_settle:.1f}s")
                node.sleep_with_spin(args.pre_arm_settle)

            arm_succeeded = False
            last_arm_result = None
            for attempt in range(1, args.arm_retries + 1):
                arm_result = node.arm(True)
                last_arm_result = arm_result
                print(
                    f"[px4-smoke] arm attempt {attempt}/{args.arm_retries}: "
                    f"success={arm_result.success} result={arm_result.result}"
                )
                if arm_result.success:
                    try:
                        node.wait_for_armed(True, timeout_sec=args.arm_timeout)
                        arm_succeeded = True
                        break
                    except Exception:
                        if node.state is not None and node.state.armed:
                            arm_succeeded = True
                            break
                if attempt < args.arm_retries:
                    node.print_state()
                    print(f"[px4-smoke] arm retry delay: {args.arm_retry_delay:.1f}s")
                    node.sleep_with_spin(args.arm_retry_delay)

            if not arm_succeeded:
                result_code = getattr(last_arm_result, "result", None)
                raise RuntimeError(f"arming command failed after retries (last_result={result_code})")

            mode_result = node.set_mode("AUTO.MISSION")
            print(f"[px4-smoke] set-mode result: mode_sent={mode_result.mode_sent}")
            if not mode_result.mode_sent:
                raise RuntimeError("failed to send AUTO.MISSION mode")
            node.wait_for_mode("AUTO.MISSION", timeout_sec=args.arm_timeout)
            node.print_state()

        if args.exit_after_auto_mission:
            print("[px4-smoke] exit: AUTO.MISSION observed")
            return 0

        node.arm_gap_probe()

        success = node.monitor_mission(
            success_seq=success_seq,
            timeout_sec=args.mission_timeout,
            print_period_sec=args.print_period,
            exit_after_gap_probe=args.exit_after_gap_probe,
            exit_after_reached_seq=args.exit_after_reached,
            require_disarm_on_success=args.require_disarm_on_success,
        )
        if success and args.post_mission_settle > 0.0:
            print(f"[px4-smoke] post-mission settle: {args.post_mission_settle:.1f}s")
            node.sleep_with_spin(args.post_mission_settle)
        node.print_gap_probe_summary()
        node.print_state()
        if not args.skip_pull_mission_on_exit:
            try:
                pull_result = node.pull_mission()
                print(f"[px4-smoke] pull result: success={pull_result.success} wp_received={pull_result.wp_received}")
                node.print_mission()
            except Exception as exc:
                print(f"[px4-smoke] pull-on-exit skipped: {exc}")
        print(f"[px4-smoke] final last_reached={node.last_reached} success_seq={success_seq}")
        return 0 if success else 2
    except Exception as exc:
        print(f"[px4-smoke] ERROR: {exc}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
