#!/usr/bin/env python3

import csv
import math
import os
from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional, Tuple

import rclpy
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix, NavSatStatus


WGS84_A_M = 6378137.0
WGS84_E2 = 6.69437999014e-3


@dataclass
class FixSample:
    stamp_sec: float
    lat_deg: float
    lon_deg: float
    alt_m: float
    east_m: float
    north_m: float
    up_m: float


@dataclass
class PoseSample:
    stamp_sec: float
    x_m: float
    y_m: float
    z_m: float


@dataclass
class ErrorSample:
    sample_index: int
    ros_time_sec: float
    error_x_m: float
    error_y_m: float
    error_z_m: float


class GpsVsPoseProbe(Node):
    def __init__(self) -> None:
        super().__init__("gps_vs_pose_probe")

        self.use_sim_time = bool(self.get_parameter("use_sim_time").value)
        self.gps_topic = str(self.declare_parameter("gps_topic", "/gps/fix").value)
        self.odom_topic = str(self.declare_parameter("odom_topic", "/ekf2/pose_odom").value)
        self.csv_path = str(self.declare_parameter("csv_path", "").value)
        self.sync_tolerance_sec = float(self.declare_parameter("sync_tolerance_sec", 0.10).value)
        self.buffer_len = int(self.declare_parameter("buffer_len", 400).value)
        self.status_print_period_sec = float(self.declare_parameter("status_print_period_sec", 5.0).value)
        self.csv_flush_interval = max(1, int(self.declare_parameter("csv_flush_interval", 20).value))
        dynamic_param = ParameterDescriptor(dynamic_typing=True)
        self.global_alignment_holdoff_sec = max(
            0.0,
            float(
                self.declare_parameter(
                    "global_alignment_holdoff_sec", 0.0, descriptor=dynamic_param
                ).value
            ),
        )
        self.global_alignment_min_pairs = max(
            1, int(self.declare_parameter("global_alignment_min_pairs", 1).value)
        )
        self.local_reanchor_window_sec = float(
            self.declare_parameter("local_reanchor_window_sec", 25.0, descriptor=dynamic_param).value
        )

        self.fix_buf: Deque[FixSample] = deque(maxlen=max(10, self.buffer_len))
        self.pose_buf: Deque[PoseSample] = deque(maxlen=max(10, self.buffer_len))
        self.error_buf: Deque[ErrorSample] = deque(maxlen=max(10, self.buffer_len))
        self.last_pair_key: Optional[Tuple[float, float]] = None

        self.anchor_lat_rad: Optional[float] = None
        self.anchor_lon_rad: Optional[float] = None
        self.anchor_alt_m: Optional[float] = None
        self.anchor_ecef_m: Optional[Tuple[float, float, float]] = None

        self.offset_x_m: Optional[float] = None
        self.offset_y_m: Optional[float] = None
        self.offset_z_m: Optional[float] = None
        self.global_alignment_start_ros_time_sec: Optional[float] = None
        self.global_alignment_pair_count = 0
        self.global_alignment_sum_x_m = 0.0
        self.global_alignment_sum_y_m = 0.0
        self.global_alignment_sum_z_m = 0.0
        self.global_alignment_anchor_sample_index: Optional[int] = None
        self.global_alignment_anchor_ros_time_sec: Optional[float] = None

        self.samples_written = 0
        self.last_status_sec = 0.0
        self.rows_since_flush = 0

        self.csv_file = None
        self.csv_writer = None
        self._open_csv_if_needed()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )

        self.create_subscription(NavSatFix, self.gps_topic, self._gps_callback, qos)
        self.create_subscription(Odometry, self.odom_topic, self._odom_callback, qos)

        self.get_logger().info(
            "gps_vs_pose_probe started\n"
            f"  gps_topic: {self.gps_topic}\n"
            f"  odom_topic: {self.odom_topic}\n"
            f"  csv_path: {self.csv_path or '(disabled)'}\n"
            f"  sync_tolerance_sec: {self.sync_tolerance_sec:.3f}\n"
            f"  global_alignment_holdoff_sec: {self.global_alignment_holdoff_sec:.3f}\n"
            f"  global_alignment_min_pairs: {self.global_alignment_min_pairs}\n"
            f"  local_reanchor_window_sec: {self.local_reanchor_window_sec:.3f}"
        )

    def _open_csv_if_needed(self) -> None:
        if not self.csv_path:
            return
        csv_path = os.path.abspath(self.csv_path)
        parent = os.path.dirname(csv_path)
        if parent:
            os.makedirs(parent, exist_ok=True)
        self.csv_file = open(csv_path, "w", newline="")
        fieldnames = [
            "sample_index",
            "ros_time_sec",
            "gps_stamp_sec",
            "pose_stamp_sec",
            "pair_dt_ms",
            "gps_lat_deg",
            "gps_lon_deg",
            "gps_alt_m",
            "gps_east_m",
            "gps_north_m",
            "gps_up_m",
            "pose_x_m",
            "pose_y_m",
            "pose_z_m",
            "global_alignment_ready",
            "global_alignment_anchor_sample_index",
            "global_alignment_anchor_ros_time_sec",
            "global_alignment_anchor_pair_count",
            "global_alignment_holdoff_sec",
            "global_alignment_min_pairs",
            "pose_offset_x_m",
            "pose_offset_y_m",
            "pose_offset_z_m",
            "aligned_pose_x_m",
            "aligned_pose_y_m",
            "aligned_pose_z_m",
            "error_x_m",
            "error_y_m",
            "error_z_m",
            "xy_error_m",
            "xyz_error_m",
            "local_reanchor_window_sec",
            "local_anchor_sample_index",
            "local_anchor_ros_time_sec",
            "local_anchor_age_sec",
            "local_error_x_m",
            "local_error_y_m",
            "local_error_z_m",
            "local_xy_error_m",
            "local_xyz_error_m",
        ]
        self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=fieldnames)
        self.csv_writer.writeheader()
        self.csv_file.flush()

    def _stamp_sec(self, stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _ros_time_sec(self) -> float:
        return float(self.get_clock().now().nanoseconds) * 1e-9

    def _valid_fix(self, msg: NavSatFix) -> bool:
        if not math.isfinite(msg.latitude) or not math.isfinite(msg.longitude) or not math.isfinite(msg.altitude):
            return False
        return msg.status.status != NavSatStatus.STATUS_NO_FIX

    def _llh_to_ecef(self, lat_deg: float, lon_deg: float, alt_m: float) -> Tuple[float, float, float]:
        lat_rad = math.radians(lat_deg)
        lon_rad = math.radians(lon_deg)
        sin_lat = math.sin(lat_rad)
        cos_lat = math.cos(lat_rad)
        sin_lon = math.sin(lon_rad)
        cos_lon = math.cos(lon_rad)
        prime_vertical_radius_m = WGS84_A_M / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
        x_m = (prime_vertical_radius_m + alt_m) * cos_lat * cos_lon
        y_m = (prime_vertical_radius_m + alt_m) * cos_lat * sin_lon
        z_m = (prime_vertical_radius_m * (1.0 - WGS84_E2) + alt_m) * sin_lat
        return x_m, y_m, z_m

    def _lla_to_local_enu(self, lat_deg: float, lon_deg: float, alt_m: float) -> Tuple[float, float, float]:
        assert self.anchor_lat_rad is not None
        assert self.anchor_lon_rad is not None
        assert self.anchor_alt_m is not None
        assert self.anchor_ecef_m is not None

        x_m, y_m, z_m = self._llh_to_ecef(lat_deg, lon_deg, alt_m)
        anchor_x_m, anchor_y_m, anchor_z_m = self.anchor_ecef_m
        dx_m = x_m - anchor_x_m
        dy_m = y_m - anchor_y_m
        dz_m = z_m - anchor_z_m

        sin_lat0 = math.sin(self.anchor_lat_rad)
        cos_lat0 = math.cos(self.anchor_lat_rad)
        sin_lon0 = math.sin(self.anchor_lon_rad)
        cos_lon0 = math.cos(self.anchor_lon_rad)

        east_m = -sin_lon0 * dx_m + cos_lon0 * dy_m
        north_m = (
            -sin_lat0 * cos_lon0 * dx_m
            - sin_lat0 * sin_lon0 * dy_m
            + cos_lat0 * dz_m
        )
        up_m = (
            cos_lat0 * cos_lon0 * dx_m
            + cos_lat0 * sin_lon0 * dy_m
            + sin_lat0 * dz_m
        )
        return east_m, north_m, up_m

    def _find_closest_fix(self, target_sec: float) -> Tuple[Optional[FixSample], float]:
        if not self.fix_buf:
            return None, float("inf")
        best = None
        best_dt = float("inf")
        for item in self.fix_buf:
            dt = item.stamp_sec - target_sec
            if abs(dt) < abs(best_dt):
                best = item
                best_dt = dt
        if best is None or abs(best_dt) > self.sync_tolerance_sec:
            return None, best_dt
        return best, best_dt

    def _find_closest_pose(self, target_sec: float) -> Tuple[Optional[PoseSample], float]:
        if not self.pose_buf:
            return None, float("inf")
        best = None
        best_dt = float("inf")
        for item in self.pose_buf:
            dt = item.stamp_sec - target_sec
            if abs(dt) < abs(best_dt):
                best = item
                best_dt = dt
        if best is None or abs(best_dt) > self.sync_tolerance_sec:
            return None, best_dt
        return best, best_dt

    def _update_global_alignment(
        self,
        sample_index: int,
        ros_time_sec: float,
        pose: PoseSample,
        fix: FixSample,
    ) -> bool:
        if self.offset_x_m is not None:
            return True

        raw_offset_x_m = pose.x_m - fix.east_m
        raw_offset_y_m = pose.y_m - fix.north_m
        raw_offset_z_m = pose.z_m - fix.up_m

        if self.global_alignment_start_ros_time_sec is None:
            self.global_alignment_start_ros_time_sec = ros_time_sec
        self.global_alignment_pair_count += 1
        self.global_alignment_sum_x_m += raw_offset_x_m
        self.global_alignment_sum_y_m += raw_offset_y_m
        self.global_alignment_sum_z_m += raw_offset_z_m

        elapsed_sec = ros_time_sec - self.global_alignment_start_ros_time_sec
        if (
            self.global_alignment_pair_count < self.global_alignment_min_pairs
            or elapsed_sec < self.global_alignment_holdoff_sec
        ):
            return False

        pair_count = float(self.global_alignment_pair_count)
        self.offset_x_m = self.global_alignment_sum_x_m / pair_count
        self.offset_y_m = self.global_alignment_sum_y_m / pair_count
        self.offset_z_m = self.global_alignment_sum_z_m / pair_count
        self.global_alignment_anchor_sample_index = sample_index
        self.global_alignment_anchor_ros_time_sec = ros_time_sec
        self.get_logger().info(
            "Pose alignment fixed: dx=%.3f dy=%.3f dz=%.3f pairs=%d holdoff=%.3fs"
            % (
                self.offset_x_m,
                self.offset_y_m,
                self.offset_z_m,
                self.global_alignment_pair_count,
                self.global_alignment_holdoff_sec,
            )
        )
        return True

    def _compute_local_metrics(
        self,
        sample_index: int,
        ros_time_sec: float,
        error_x_m: float,
        error_y_m: float,
        error_z_m: float,
    ) -> dict:
        out = {
            "local_anchor_sample_index": None,
            "local_anchor_ros_time_sec": None,
            "local_anchor_age_sec": None,
            "local_error_x_m": float("nan"),
            "local_error_y_m": float("nan"),
            "local_error_z_m": float("nan"),
            "local_xy_error_m": float("nan"),
            "local_xyz_error_m": float("nan"),
        }
        if self.local_reanchor_window_sec <= 0.0:
            return out

        current = ErrorSample(
            sample_index=sample_index,
            ros_time_sec=ros_time_sec,
            error_x_m=error_x_m,
            error_y_m=error_y_m,
            error_z_m=error_z_m,
        )
        self.error_buf.append(current)
        while (
            len(self.error_buf) > 1
            and (ros_time_sec - self.error_buf[0].ros_time_sec) > self.local_reanchor_window_sec
        ):
            self.error_buf.popleft()

        anchor = self.error_buf[0]
        local_error_x_m = error_x_m - anchor.error_x_m
        local_error_y_m = error_y_m - anchor.error_y_m
        local_error_z_m = error_z_m - anchor.error_z_m
        out.update(
            {
                "local_anchor_sample_index": anchor.sample_index,
                "local_anchor_ros_time_sec": anchor.ros_time_sec,
                "local_anchor_age_sec": ros_time_sec - anchor.ros_time_sec,
                "local_error_x_m": local_error_x_m,
                "local_error_y_m": local_error_y_m,
                "local_error_z_m": local_error_z_m,
                "local_xy_error_m": math.hypot(local_error_x_m, local_error_y_m),
                "local_xyz_error_m": math.sqrt(
                    local_error_x_m * local_error_x_m
                    + local_error_y_m * local_error_y_m
                    + local_error_z_m * local_error_z_m
                ),
            }
        )
        return out

    def _gps_callback(self, msg: NavSatFix) -> None:
        if not self._valid_fix(msg):
            return

        if self.anchor_lat_rad is None:
            self.anchor_lat_rad = math.radians(msg.latitude)
            self.anchor_lon_rad = math.radians(msg.longitude)
            self.anchor_alt_m = float(msg.altitude)
            self.anchor_ecef_m = self._llh_to_ecef(msg.latitude, msg.longitude, msg.altitude)
            self.get_logger().info(
                f"GPS anchor fixed at lat={msg.latitude:.8f} lon={msg.longitude:.8f} alt={msg.altitude:.3f}"
            )

        east_m, north_m, up_m = self._lla_to_local_enu(msg.latitude, msg.longitude, msg.altitude)
        sample = FixSample(
            stamp_sec=self._stamp_sec(msg.header.stamp),
            lat_deg=float(msg.latitude),
            lon_deg=float(msg.longitude),
            alt_m=float(msg.altitude),
            east_m=float(east_m),
            north_m=float(north_m),
            up_m=float(up_m),
        )
        self.fix_buf.append(sample)
        self._maybe_write_pair_from_fix(sample)

    def _odom_callback(self, msg: Odometry) -> None:
        sample = PoseSample(
            stamp_sec=self._stamp_sec(msg.header.stamp),
            x_m=float(msg.pose.pose.position.x),
            y_m=float(msg.pose.pose.position.y),
            z_m=float(msg.pose.pose.position.z),
        )
        self.pose_buf.append(sample)

    def _maybe_write_pair_from_fix(self, fix: FixSample) -> None:
        pose, dt_sec = self._find_closest_pose(fix.stamp_sec)
        if pose is None:
            return
        self._write_pair(fix, pose, dt_sec)

    def _write_pair(self, fix: FixSample, pose: PoseSample, dt_sec: float) -> None:
        pair_key = (fix.stamp_sec, pose.stamp_sec)
        if self.last_pair_key == pair_key:
            return

        sample_index = self.samples_written + 1
        ros_time_sec = self._ros_time_sec()
        global_alignment_ready = self._update_global_alignment(sample_index, ros_time_sec, pose, fix)

        aligned_pose_x_m = float("nan")
        aligned_pose_y_m = float("nan")
        aligned_pose_z_m = float("nan")
        error_x_m = float("nan")
        error_y_m = float("nan")
        error_z_m = float("nan")
        xy_error_m = float("nan")
        xyz_error_m = float("nan")
        local_metrics = {
            "local_anchor_sample_index": None,
            "local_anchor_ros_time_sec": None,
            "local_anchor_age_sec": None,
            "local_error_x_m": float("nan"),
            "local_error_y_m": float("nan"),
            "local_error_z_m": float("nan"),
            "local_xy_error_m": float("nan"),
            "local_xyz_error_m": float("nan"),
        }

        if global_alignment_ready:
            assert self.offset_x_m is not None
            assert self.offset_y_m is not None
            assert self.offset_z_m is not None
            aligned_pose_x_m = pose.x_m - self.offset_x_m
            aligned_pose_y_m = pose.y_m - self.offset_y_m
            aligned_pose_z_m = pose.z_m - self.offset_z_m

            error_x_m = aligned_pose_x_m - fix.east_m
            error_y_m = aligned_pose_y_m - fix.north_m
            error_z_m = aligned_pose_z_m - fix.up_m
            xy_error_m = math.hypot(error_x_m, error_y_m)
            xyz_error_m = math.sqrt(
                error_x_m * error_x_m + error_y_m * error_y_m + error_z_m * error_z_m
            )
            local_metrics = self._compute_local_metrics(
                sample_index, ros_time_sec, error_x_m, error_y_m, error_z_m
            )

        self.last_pair_key = pair_key
        self.samples_written = sample_index

        if self.csv_writer is not None:
            self.csv_writer.writerow(
                {
                    "sample_index": sample_index,
                    "ros_time_sec": ros_time_sec,
                    "gps_stamp_sec": fix.stamp_sec,
                    "pose_stamp_sec": pose.stamp_sec,
                    "pair_dt_ms": dt_sec * 1000.0,
                    "gps_lat_deg": fix.lat_deg,
                    "gps_lon_deg": fix.lon_deg,
                    "gps_alt_m": fix.alt_m,
                    "gps_east_m": fix.east_m,
                    "gps_north_m": fix.north_m,
                    "gps_up_m": fix.up_m,
                    "pose_x_m": pose.x_m,
                    "pose_y_m": pose.y_m,
                    "pose_z_m": pose.z_m,
                    "global_alignment_ready": int(global_alignment_ready),
                    "global_alignment_anchor_sample_index": self.global_alignment_anchor_sample_index,
                    "global_alignment_anchor_ros_time_sec": self.global_alignment_anchor_ros_time_sec,
                    "global_alignment_anchor_pair_count": self.global_alignment_pair_count,
                    "global_alignment_holdoff_sec": self.global_alignment_holdoff_sec,
                    "global_alignment_min_pairs": self.global_alignment_min_pairs,
                    "pose_offset_x_m": self.offset_x_m,
                    "pose_offset_y_m": self.offset_y_m,
                    "pose_offset_z_m": self.offset_z_m,
                    "aligned_pose_x_m": aligned_pose_x_m,
                    "aligned_pose_y_m": aligned_pose_y_m,
                    "aligned_pose_z_m": aligned_pose_z_m,
                    "error_x_m": error_x_m,
                    "error_y_m": error_y_m,
                    "error_z_m": error_z_m,
                    "xy_error_m": xy_error_m,
                    "xyz_error_m": xyz_error_m,
                    "local_reanchor_window_sec": self.local_reanchor_window_sec,
                    "local_anchor_sample_index": local_metrics["local_anchor_sample_index"],
                    "local_anchor_ros_time_sec": local_metrics["local_anchor_ros_time_sec"],
                    "local_anchor_age_sec": local_metrics["local_anchor_age_sec"],
                    "local_error_x_m": local_metrics["local_error_x_m"],
                    "local_error_y_m": local_metrics["local_error_y_m"],
                    "local_error_z_m": local_metrics["local_error_z_m"],
                    "local_xy_error_m": local_metrics["local_xy_error_m"],
                    "local_xyz_error_m": local_metrics["local_xyz_error_m"],
                }
            )
            self.rows_since_flush += 1
            if self.rows_since_flush >= self.csv_flush_interval:
                self.csv_file.flush()
                self.rows_since_flush = 0

        now_sec = ros_time_sec
        if now_sec - self.last_status_sec >= self.status_print_period_sec:
            self.last_status_sec = now_sec
            local_xy_error_m = local_metrics["local_xy_error_m"]
            xy_text = f"{xy_error_m:.3f}" if math.isfinite(xy_error_m) else "nan"
            local_text = f"{local_xy_error_m:.3f}" if math.isfinite(local_xy_error_m) else "nan"
            self.get_logger().info(
                f"samples={self.samples_written} topic={self.odom_topic} "
                f"xy_error_m={xy_text} local_xy_error_m={local_text} "
                f"xyz_error_m={xyz_error_m:.3f} pair_dt_ms={dt_sec * 1000.0:.1f}"
            )

    def destroy_node(self) -> bool:
        try:
            if self.csv_file is not None:
                self.csv_file.flush()
                self.csv_file.close()
        finally:
            return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = GpsVsPoseProbe()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
