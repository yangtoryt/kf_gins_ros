#!/usr/bin/env python3
"""Lightweight EKF2/IEKF paired odometry CSV logger."""

from __future__ import annotations

import csv
import math
import os
from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile


@dataclass
class OdomSample:
    stamp: float
    recv_time: float
    x: float
    y: float
    z: float
    vx: float
    vy: float
    vz: float
    roll: float
    pitch: float
    yaw: float


def stamp_to_sec(msg) -> float:
    stamp = msg.header.stamp
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def quat_to_rpy(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def angle_diff_rad(a: float, b: float) -> float:
    diff = a - b
    while diff > math.pi:
        diff -= 2.0 * math.pi
    while diff < -math.pi:
        diff += 2.0 * math.pi
    return diff


def finite(*values: float) -> bool:
    return all(math.isfinite(v) for v in values)


def dynamic_descriptor() -> ParameterDescriptor:
    descriptor = ParameterDescriptor()
    descriptor.dynamic_typing = True
    return descriptor


class EkfIekfPairLogger(Node):
    def __init__(self) -> None:
        super().__init__("ekf_iekf_pair_logger")

        descriptor = dynamic_descriptor()
        self.ekf2_topic = str(self.declare_parameter("ekf2_topic", "/ekf2/pose_odom", descriptor).value)
        self.iekf_topic = str(self.declare_parameter("iekf_topic", "/kf_gins/odom", descriptor).value)
        self.mavros_state_topic = str(
            self.declare_parameter("mavros_state_topic", "/mavros/state", descriptor).value
        )
        self.csv_path = str(self.declare_parameter("csv_path", "", descriptor).value)
        self.write_rate_hz = max(1.0, float(self.declare_parameter("write_rate_hz", 10.0, descriptor).value))
        self.sync_tolerance_ms = max(
            0.0,
            float(self.declare_parameter("sync_tolerance_ms", 50.0, descriptor).value),
        )
        self.buffer_len = max(10, int(self.declare_parameter("buffer_len", 200, descriptor).value))
        self.align_initial = bool(self.declare_parameter("align_initial", True, descriptor).value)
        self.align_min_pairs = max(1, int(self.declare_parameter("align_min_pairs", 10, descriptor).value))
        self.flush_interval_rows = max(
            1,
            int(self.declare_parameter("flush_interval_rows", 50, descriptor).value),
        )

        if not self.csv_path:
            raise RuntimeError("csv_path parameter is required")

        parent = os.path.dirname(os.path.abspath(self.csv_path))
        if parent:
            os.makedirs(parent, exist_ok=True)

        self.ekf2_buf: Deque[OdomSample] = deque(maxlen=self.buffer_len)
        self.iekf_buf: Deque[OdomSample] = deque(maxlen=self.buffer_len)
        self.last_pair_key: Optional[tuple[float, float]] = None
        self.mavros_armed = False
        self.mavros_mode = ""

        self.align_samples: list[tuple[float, float, float]] = []
        self.align_offset: Optional[tuple[float, float, float]] = None
        self.rows_since_flush = 0

        self.csv_file = open(self.csv_path, "w", newline="")
        self.writer = csv.DictWriter(
            self.csv_file,
            fieldnames=[
                "ros_time_sec",
                "ekf2_stamp_sec",
                "iekf_stamp_sec",
                "sync_dt_ms",
                "mavros_armed",
                "mavros_mode",
                "alignment_ready",
                "align_offset_x_m",
                "align_offset_y_m",
                "align_offset_z_m",
                "ekf2_x_m",
                "ekf2_y_m",
                "ekf2_z_m",
                "iekf_x_m",
                "iekf_y_m",
                "iekf_z_m",
                "iekf_aligned_x_m",
                "iekf_aligned_y_m",
                "iekf_aligned_z_m",
                "position_error_x_m",
                "position_error_y_m",
                "position_error_z_m",
                "position_error_xy_m",
                "position_error_norm_m",
                "velocity_error_x_mps",
                "velocity_error_y_mps",
                "velocity_error_z_mps",
                "velocity_error_norm_mps",
                "yaw_error_deg",
            ],
        )
        self.writer.writeheader()

        qos = QoSProfile(depth=self.buffer_len)
        self.create_subscription(Odometry, self.ekf2_topic, self._ekf2_cb, qos)
        self.create_subscription(Odometry, self.iekf_topic, self._iekf_cb, qos)
        self.create_subscription(State, self.mavros_state_topic, self._state_cb, QoSProfile(depth=10))
        self.create_timer(1.0 / self.write_rate_hz, self._timer_cb)

        self.get_logger().info(
            "EKF/IEKF pair logger started: "
            f"ekf2={self.ekf2_topic} iekf={self.iekf_topic} "
            f"csv={self.csv_path} rate={self.write_rate_hz:.1f}Hz "
            f"tol={self.sync_tolerance_ms:.1f}ms"
        )

    def close(self) -> None:
        if not self.csv_file.closed:
            self.csv_file.flush()
            self.csv_file.close()

    def _state_cb(self, msg: State) -> None:
        self.mavros_armed = bool(msg.armed)
        self.mavros_mode = str(msg.mode)

    def _ekf2_cb(self, msg: Odometry) -> None:
        sample = self._sample_from_odom(msg)
        if sample is not None:
            self.ekf2_buf.append(sample)

    def _iekf_cb(self, msg: Odometry) -> None:
        sample = self._sample_from_odom(msg)
        if sample is not None:
            self.iekf_buf.append(sample)

    def _sample_from_odom(self, msg: Odometry) -> Optional[OdomSample]:
        stamp = stamp_to_sec(msg)
        if stamp <= 0.0:
            stamp = self.get_clock().now().nanoseconds * 1e-9
        recv_time = self.get_clock().now().nanoseconds * 1e-9
        pos = msg.pose.pose.position
        vel = msg.twist.twist.linear
        quat = msg.pose.pose.orientation
        roll, pitch, yaw = quat_to_rpy(quat.x, quat.y, quat.z, quat.w)
        values = (pos.x, pos.y, pos.z, vel.x, vel.y, vel.z, roll, pitch, yaw)
        if not finite(*values):
            return None
        return OdomSample(
            stamp=stamp,
            recv_time=recv_time,
            x=float(pos.x),
            y=float(pos.y),
            z=float(pos.z),
            vx=float(vel.x),
            vy=float(vel.y),
            vz=float(vel.z),
            roll=roll,
            pitch=pitch,
            yaw=yaw,
        )

    def _timer_cb(self) -> None:
        pair = self._get_pair()
        if pair is None:
            return
        ekf2, iekf, sync_dt = pair
        pair_key = (ekf2.stamp, iekf.stamp)
        if pair_key == self.last_pair_key:
            return
        self.last_pair_key = pair_key

        offset_x = offset_y = offset_z = float("nan")
        if self.align_initial:
            if self.align_offset is None:
                self.align_samples.append((ekf2.x - iekf.x, ekf2.y - iekf.y, ekf2.z - iekf.z))
                if len(self.align_samples) >= self.align_min_pairs:
                    n = float(len(self.align_samples))
                    self.align_offset = (
                        sum(v[0] for v in self.align_samples) / n,
                        sum(v[1] for v in self.align_samples) / n,
                        sum(v[2] for v in self.align_samples) / n,
                    )
            if self.align_offset is not None:
                offset_x, offset_y, offset_z = self.align_offset
        else:
            offset_x = offset_y = offset_z = 0.0

        alignment_ready = finite(offset_x, offset_y, offset_z)
        if alignment_ready:
            iekf_ax = iekf.x + offset_x
            iekf_ay = iekf.y + offset_y
            iekf_az = iekf.z + offset_z
            err_x = iekf_ax - ekf2.x
            err_y = iekf_ay - ekf2.y
            err_z = iekf_az - ekf2.z
            err_xy = math.hypot(err_x, err_y)
            err_norm = math.sqrt(err_x * err_x + err_y * err_y + err_z * err_z)
        else:
            iekf_ax = iekf_ay = iekf_az = float("nan")
            err_x = err_y = err_z = err_xy = err_norm = float("nan")

        vel_x = iekf.vx - ekf2.vx
        vel_y = iekf.vy - ekf2.vy
        vel_z = iekf.vz - ekf2.vz
        vel_norm = math.sqrt(vel_x * vel_x + vel_y * vel_y + vel_z * vel_z)
        yaw_err = math.degrees(angle_diff_rad(iekf.yaw, ekf2.yaw))

        self.writer.writerow(
            {
                "ros_time_sec": self.get_clock().now().nanoseconds * 1e-9,
                "ekf2_stamp_sec": ekf2.stamp,
                "iekf_stamp_sec": iekf.stamp,
                "sync_dt_ms": sync_dt * 1000.0,
                "mavros_armed": int(self.mavros_armed),
                "mavros_mode": self.mavros_mode,
                "alignment_ready": int(alignment_ready),
                "align_offset_x_m": offset_x,
                "align_offset_y_m": offset_y,
                "align_offset_z_m": offset_z,
                "ekf2_x_m": ekf2.x,
                "ekf2_y_m": ekf2.y,
                "ekf2_z_m": ekf2.z,
                "iekf_x_m": iekf.x,
                "iekf_y_m": iekf.y,
                "iekf_z_m": iekf.z,
                "iekf_aligned_x_m": iekf_ax,
                "iekf_aligned_y_m": iekf_ay,
                "iekf_aligned_z_m": iekf_az,
                "position_error_x_m": err_x,
                "position_error_y_m": err_y,
                "position_error_z_m": err_z,
                "position_error_xy_m": err_xy,
                "position_error_norm_m": err_norm,
                "velocity_error_x_mps": vel_x,
                "velocity_error_y_mps": vel_y,
                "velocity_error_z_mps": vel_z,
                "velocity_error_norm_mps": vel_norm,
                "yaw_error_deg": yaw_err,
            }
        )
        self.rows_since_flush += 1
        if self.rows_since_flush >= self.flush_interval_rows:
            self.csv_file.flush()
            self.rows_since_flush = 0

    def _get_pair(self) -> Optional[tuple[OdomSample, OdomSample, float]]:
        if not self.ekf2_buf or not self.iekf_buf:
            return None
        tolerance = self.sync_tolerance_ms * 1e-3
        candidates = []
        ekf2_latest = self.ekf2_buf[-1]
        iekf_match = self._closest(self.iekf_buf, ekf2_latest.stamp)
        if iekf_match is not None:
            dt = ekf2_latest.stamp - iekf_match.stamp
            candidates.append((abs(dt), ekf2_latest, iekf_match, dt))
        iekf_latest = self.iekf_buf[-1]
        ekf2_match = self._closest(self.ekf2_buf, iekf_latest.stamp)
        if ekf2_match is not None:
            dt = ekf2_match.stamp - iekf_latest.stamp
            candidates.append((abs(dt), ekf2_match, iekf_latest, dt))
        if not candidates:
            return None
        best_abs_dt, ekf2, iekf, dt = min(candidates, key=lambda item: item[0])
        if best_abs_dt > tolerance:
            return None
        return ekf2, iekf, dt

    @staticmethod
    def _closest(buf: Deque[OdomSample], stamp: float) -> Optional[OdomSample]:
        best = None
        best_abs_dt = float("inf")
        for sample in buf:
            abs_dt = abs(sample.stamp - stamp)
            if abs_dt < best_abs_dt:
                best = sample
                best_abs_dt = abs_dt
        return best


def main() -> None:
    rclpy.init()
    node = EkfIekfPairLogger()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
