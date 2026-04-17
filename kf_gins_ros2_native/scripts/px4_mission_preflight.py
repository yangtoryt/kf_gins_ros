#!/usr/bin/env python3

import argparse
import sys
import time
from typing import Optional

import rclpy
from rclpy.node import Node

from mavros_msgs.msg import State, WaypointList
from mavros_msgs.srv import WaypointClear, WaypointPull, WaypointSetCurrent


class MissionPreflight(Node):
    def __init__(self, mavros_ns: str, timeout_sec: float) -> None:
        super().__init__("px4_mission_preflight")
        self.timeout_sec = timeout_sec
        self.state: Optional[State] = None
        self.waypoints: Optional[WaypointList] = None

        ns = mavros_ns.rstrip("/")
        self.state_topic = f"{ns}/state"
        self.waypoints_topic = f"{ns}/mission/waypoints"
        self.clear_service = f"{ns}/mission/clear"
        self.pull_service = f"{ns}/mission/pull"
        self.set_current_service = f"{ns}/mission/set_current"

        self.create_subscription(State, self.state_topic, self._state_cb, 10)
        self.create_subscription(WaypointList, self.waypoints_topic, self._waypoints_cb, 10)

        self.clear_client = self.create_client(WaypointClear, self.clear_service)
        self.pull_client = self.create_client(WaypointPull, self.pull_service)
        self.set_current_client = self.create_client(WaypointSetCurrent, self.set_current_service)

    def _state_cb(self, msg: State) -> None:
        self.state = msg

    def _waypoints_cb(self, msg: WaypointList) -> None:
        self.waypoints = msg

    def wait_for_topics(self) -> None:
        deadline = time.monotonic() + self.timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.state is not None:
                return
        raise RuntimeError(f"timeout waiting for {self.state_topic}")

    def wait_for_service(self, client, name: str) -> None:
        if not client.wait_for_service(timeout_sec=self.timeout_sec):
            raise RuntimeError(f"timeout waiting for service {name}")

    def call(self, client, request, name: str):
        future = client.call_async(request)
        deadline = time.monotonic() + self.timeout_sec
        while time.monotonic() < deadline and not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        if not future.done():
            raise RuntimeError(f"timeout calling service {name}")
        result = future.result()
        if result is None:
            raise RuntimeError(f"service {name} returned no result")
        return result

    def pull_mission(self) -> WaypointPull.Response:
        self.wait_for_service(self.pull_client, self.pull_service)
        result = self.call(self.pull_client, WaypointPull.Request(), self.pull_service)

        deadline = time.monotonic() + min(self.timeout_sec, 3.0)
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.waypoints is not None:
                break
        return result

    def clear_mission(self) -> WaypointClear.Response:
        self.wait_for_service(self.clear_client, self.clear_service)
        return self.call(self.clear_client, WaypointClear.Request(), self.clear_service)

    def set_current(self, seq: int) -> WaypointSetCurrent.Response:
        self.wait_for_service(self.set_current_client, self.set_current_service)
        req = WaypointSetCurrent.Request()
        req.wp_seq = int(seq)
        return self.call(self.set_current_client, req, self.set_current_service)

    def print_state(self) -> None:
        if self.state is None:
            print("[px4-mission] state: unavailable")
            return
        print(
            "[px4-mission] state: "
            f"connected={self.state.connected} armed={self.state.armed} "
            f"mode={self.state.mode} system_status={self.state.system_status}"
        )

    def print_mission(self) -> None:
        msg = self.waypoints
        if msg is None:
            print("[px4-mission] mission: unavailable")
            return
        print(f"[px4-mission] mission: current_seq={msg.current_seq} count={len(msg.waypoints)}")
        if not msg.waypoints:
            print("[px4-mission] mission is empty")
            return

        if msg.current_seq < len(msg.waypoints):
            wp = msg.waypoints[msg.current_seq]
            print(
                "[px4-mission] current waypoint: "
                f"seq={msg.current_seq} command={wp.command} frame={wp.frame} "
                f"is_current={wp.is_current} alt={wp.z_alt:.2f}"
            )
            if wp.command == 20:
                print("[px4-mission] WARNING: current waypoint is RTL (command 20). Do not start mission from this state.")
        else:
            print("[px4-mission] WARNING: current_seq is outside waypoint list")

        if msg.current_seq > 1:
            print("[px4-mission] WARNING: current_seq > 1. Re-upload mission or set current to the takeoff item before flight.")

    def require_disarmed(self, allow_armed: bool) -> None:
        if self.state is not None and self.state.armed and not allow_armed:
            raise RuntimeError("refusing to change mission while armed; pass --allow-armed only if you really intend this")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Check and reset PX4 mission state through MAVROS before QGC mission tests."
    )
    parser.add_argument(
        "command",
        choices=["check", "clear", "reset", "set-current"],
        help="check mission state, clear mission, reset mission state, or set current waypoint",
    )
    parser.add_argument("--seq", type=int, default=1, help="waypoint sequence for set-current, default: 1")
    parser.add_argument("--mavros-ns", default="/mavros", help="MAVROS namespace, default: /mavros")
    parser.add_argument("--timeout", type=float, default=10.0, help="service/topic timeout in seconds")
    parser.add_argument("--allow-armed", action="store_true", help="allow mission mutation while armed")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = MissionPreflight(args.mavros_ns, args.timeout)

    try:
        node.wait_for_topics()
        node.pull_mission()
        node.print_state()
        node.print_mission()

        if args.command == "check":
            return 0

        if args.command in ("clear", "reset"):
            node.require_disarmed(args.allow_armed)
            result = node.clear_mission()
            print(f"[px4-mission] clear result: success={result.success}")
            node.waypoints = None
            node.pull_mission()
            node.print_mission()
            print("[px4-mission] next: upload the mission again from QGC, then run set-current --seq 1 before Start Mission.")
            return 0 if result.success else 2

        if args.command == "set-current":
            node.require_disarmed(args.allow_armed)
            result = node.set_current(args.seq)
            print(f"[px4-mission] set-current result: seq={args.seq} success={result.success}")
            node.waypoints = None
            node.pull_mission()
            node.print_mission()
            return 0 if result.success else 2

        return 2
    except Exception as exc:
        print(f"[px4-mission] ERROR: {exc}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
