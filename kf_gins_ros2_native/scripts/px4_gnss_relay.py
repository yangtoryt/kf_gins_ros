#!/usr/bin/env python3

import csv
import math
from dataclasses import dataclass
from pathlib import Path

import rclpy
from px4_msgs.msg import SensorGps, VehicleGlobalPosition
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import NavSatFix, NavSatStatus


@dataclass(frozen=True)
class TraceSample:
    t_sec: float
    dn_m: float
    de_m: float
    du_m: float
    std_scale_h: float
    std_scale_v: float
    valid: bool
    tag: str


class GnssTraceDisturbance:
    """Replay deterministic GNSS measurement disturbances from a CSV trace."""

    def __init__(self, node: Node) -> None:
        self._node = node
        node.declare_parameter("disturbance_enable", False)
        node.declare_parameter("disturbance_trace_csv", "")
        node.declare_parameter("disturbance_log_csv", "")
        node.declare_parameter("disturbance_time_offset_sec", 0.0)

        self.enabled = node.get_parameter("disturbance_enable").get_parameter_value().bool_value
        self.trace_csv = node.get_parameter("disturbance_trace_csv").get_parameter_value().string_value
        self.log_csv = node.get_parameter("disturbance_log_csv").get_parameter_value().string_value
        self.time_offset_sec = (
            node.get_parameter("disturbance_time_offset_sec").get_parameter_value().double_value
        )

        self._samples: list[TraceSample] = []
        self._sample_index = 0
        self._start_time_sec: float | None = None
        self._seq = 0
        self._log_file = None
        self._log_writer: csv.DictWriter | None = None

        if not self.enabled:
            return
        if not self.trace_csv:
            raise ValueError("disturbance_enable=true requires disturbance_trace_csv")
        self._samples = self._load_trace(Path(self.trace_csv))
        if not self._samples:
            raise ValueError(f"empty GNSS disturbance trace: {self.trace_csv}")
        if self.log_csv:
            log_path = Path(self.log_csv)
            log_path.parent.mkdir(parents=True, exist_ok=True)
            self._log_file = log_path.open("w", newline="", encoding="utf-8")
            self._log_writer = csv.DictWriter(
                self._log_file,
                fieldnames=[
                    "sequence",
                    "elapsed_sec",
                    "trace_t_sec",
                    "tag",
                    "valid",
                    "dn_m",
                    "de_m",
                    "du_m",
                    "std_scale_h",
                    "std_scale_v",
                    "raw_lat_deg",
                    "raw_lon_deg",
                    "raw_alt_m",
                    "out_lat_deg",
                    "out_lon_deg",
                    "out_alt_m",
                    "cov_nn",
                    "cov_ee",
                    "cov_uu",
                ],
            )
            self._log_writer.writeheader()
            self._log_file.flush()
        node.get_logger().info(
            "GNSS trace disturbance enabled: "
            f"trace={self.trace_csv}, samples={len(self._samples)}, "
            f"time_offset_sec={self.time_offset_sec:.3f}, log={self.log_csv or 'disabled'}"
        )

    def close(self) -> None:
        if self._log_file is not None:
            self._log_file.close()
            self._log_file = None

    def apply(self, msg: NavSatFix) -> NavSatFix | None:
        if not self.enabled:
            return msg
        elapsed = self._elapsed_sec()
        sample = self._sample_for_time(elapsed)
        raw_lat = float(msg.latitude)
        raw_lon = float(msg.longitude)
        raw_alt = float(msg.altitude)
        out = NavSatFix()
        out.header = msg.header
        out.status = msg.status
        out.latitude = msg.latitude
        out.longitude = msg.longitude
        out.altitude = msg.altitude
        out.position_covariance = list(msg.position_covariance)
        out.position_covariance_type = msg.position_covariance_type

        if sample.valid:
            out.latitude, out.longitude, out.altitude = self._apply_ned_offset(
                raw_lat, raw_lon, raw_alt, sample.dn_m, sample.de_m, sample.du_m
            )
            self._scale_covariance(out, sample.std_scale_h, sample.std_scale_v)
        else:
            out.status.status = NavSatStatus.STATUS_NO_FIX

        self._write_log(elapsed, sample, raw_lat, raw_lon, raw_alt, out)
        return out if sample.valid else None

    def _elapsed_sec(self) -> float:
        now = self._node.get_clock().now().nanoseconds * 1e-9
        if self._start_time_sec is None:
            self._start_time_sec = now
        return max(0.0, now - self._start_time_sec - self.time_offset_sec)

    def _sample_for_time(self, elapsed_sec: float) -> TraceSample:
        while (
            self._sample_index + 1 < len(self._samples)
            and self._samples[self._sample_index + 1].t_sec <= elapsed_sec
        ):
            self._sample_index += 1
        return self._samples[self._sample_index]

    def _write_log(
        self,
        elapsed_sec: float,
        sample: TraceSample,
        raw_lat: float,
        raw_lon: float,
        raw_alt: float,
        out: NavSatFix,
    ) -> None:
        self._seq += 1
        if self._log_writer is None:
            return
        self._log_writer.writerow(
            {
                "sequence": self._seq,
                "elapsed_sec": f"{elapsed_sec:.6f}",
                "trace_t_sec": f"{sample.t_sec:.6f}",
                "tag": sample.tag,
                "valid": int(sample.valid),
                "dn_m": f"{sample.dn_m:.6f}",
                "de_m": f"{sample.de_m:.6f}",
                "du_m": f"{sample.du_m:.6f}",
                "std_scale_h": f"{sample.std_scale_h:.6f}",
                "std_scale_v": f"{sample.std_scale_v:.6f}",
                "raw_lat_deg": f"{raw_lat:.10f}",
                "raw_lon_deg": f"{raw_lon:.10f}",
                "raw_alt_m": f"{raw_alt:.6f}",
                "out_lat_deg": f"{out.latitude:.10f}",
                "out_lon_deg": f"{out.longitude:.10f}",
                "out_alt_m": f"{out.altitude:.6f}",
                "cov_nn": f"{out.position_covariance[0]:.9g}",
                "cov_ee": f"{out.position_covariance[4]:.9g}",
                "cov_uu": f"{out.position_covariance[8]:.9g}",
            }
        )
        if self._seq % 20 == 0 and self._log_file is not None:
            self._log_file.flush()

    @staticmethod
    def _load_trace(path: Path) -> list[TraceSample]:
        if not path.exists():
            raise FileNotFoundError(path)
        samples: list[TraceSample] = []
        with path.open(newline="", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            required = {"t_sec", "dn_m", "de_m", "du_m", "std_scale_h", "std_scale_v", "valid", "tag"}
            missing = required - set(reader.fieldnames or [])
            if missing:
                raise ValueError(f"GNSS disturbance trace missing columns: {sorted(missing)}")
            for row in reader:
                samples.append(
                    TraceSample(
                        t_sec=float(row["t_sec"]),
                        dn_m=float(row["dn_m"]),
                        de_m=float(row["de_m"]),
                        du_m=float(row["du_m"]),
                        std_scale_h=max(0.0, float(row["std_scale_h"])),
                        std_scale_v=max(0.0, float(row["std_scale_v"])),
                        valid=row["valid"].strip().lower() not in {"0", "false", "no"},
                        tag=row["tag"],
                    )
                )
        samples.sort(key=lambda sample: sample.t_sec)
        return samples

    @staticmethod
    def _apply_ned_offset(
        lat_deg: float,
        lon_deg: float,
        alt_m: float,
        dn_m: float,
        de_m: float,
        du_m: float,
    ) -> tuple[float, float, float]:
        # WGS-84 local tangent approximation; valid for the meter-scale perturbations used here.
        a = 6378137.0
        f = 1.0 / 298.257223563
        e2 = f * (2.0 - f)
        lat = math.radians(lat_deg)
        sin_lat = math.sin(lat)
        cos_lat = max(1e-9, math.cos(lat))
        denom = math.sqrt(1.0 - e2 * sin_lat * sin_lat)
        rn = a / denom
        rm = a * (1.0 - e2) / (denom * denom * denom)
        disturbed_lat = lat + dn_m / rm
        disturbed_lon = math.radians(lon_deg) + de_m / (rn * cos_lat)
        disturbed_alt = alt_m - du_m
        return math.degrees(disturbed_lat), math.degrees(disturbed_lon), disturbed_alt

    @staticmethod
    def _scale_covariance(msg: NavSatFix, std_scale_h: float, std_scale_v: float) -> None:
        var_scale_h = std_scale_h * std_scale_h
        var_scale_v = std_scale_v * std_scale_v
        msg.position_covariance[0] *= var_scale_h
        msg.position_covariance[4] *= var_scale_h
        msg.position_covariance[8] *= var_scale_v


class Px4GnssRelay(Node):
    def __init__(self) -> None:
        super().__init__("px4_gnss_relay")

        self.declare_parameter("input_mode", "sensor_gps")
        self.declare_parameter("sensor_gps_topic", "/fmu/out/vehicle_gps_position")
        self.declare_parameter("vehicle_global_position_topic", "/fmu/out/vehicle_global_position")
        self.declare_parameter("output_topic", "/gps/fix")
        self.declare_parameter("frame_id", "gps")
        self.declare_parameter("publish_min_interval_sec", 0.0)

        self._input_mode = self.get_parameter("input_mode").get_parameter_value().string_value
        self._sensor_gps_topic = (
            self.get_parameter("sensor_gps_topic").get_parameter_value().string_value
        )
        self._vehicle_global_position_topic = (
            self.get_parameter("vehicle_global_position_topic").get_parameter_value().string_value
        )
        self._frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self._publish_min_interval_sec = max(
            0.0,
            self.get_parameter("publish_min_interval_sec").get_parameter_value().double_value,
        )
        self._last_publish_time_sec: float | None = None
        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        self._disturbance = GnssTraceDisturbance(self)

        pub_qos = QoSProfile(depth=10)
        pub_qos.reliability = ReliabilityPolicy.RELIABLE
        pub_qos.history = HistoryPolicy.KEEP_LAST
        pub_qos.durability = DurabilityPolicy.VOLATILE
        self._pub = self.create_publisher(NavSatFix, output_topic, pub_qos)

        self._received_samples = 0

        if self._input_mode == "sensor_gps":
            self._sub = self.create_subscription(
                SensorGps,
                self._sensor_gps_topic,
                self._sensor_gps_cb,
                qos_profile_sensor_data,
            )
            self.get_logger().info(
                f"PX4 GNSS relay: {self._sensor_gps_topic} (SensorGps) -> {output_topic} (NavSatFix)"
            )
        elif self._input_mode == "vehicle_global_position":
            self._sub = self.create_subscription(
                VehicleGlobalPosition,
                self._vehicle_global_position_topic,
                self._vehicle_global_position_cb,
                qos_profile_sensor_data,
            )
            self.get_logger().info(
                "PX4 GNSS relay: "
                f"{self._vehicle_global_position_topic} (VehicleGlobalPosition) -> {output_topic} (NavSatFix)"
            )
        else:
            raise ValueError(
                "Unsupported input_mode. Expected one of: sensor_gps, vehicle_global_position"
            )
        if self._publish_min_interval_sec > 0.0:
            self.get_logger().warn(
                "PX4 GNSS relay publish downsampling enabled: "
                f"minimum interval={self._publish_min_interval_sec:.3f} s"
            )

    def _publish_first_sample_log(self, lat: float, lon: float, alt: float, detail: str) -> None:
        self._received_samples += 1
        if self._received_samples == 1:
            self.get_logger().info(
                "Received first PX4 GNSS sample: "
                f"mode={self._input_mode} lat={lat:.8f} lon={lon:.8f} alt={alt:.3f} {detail}"
            )

    def _base_msg(self) -> NavSatFix:
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        msg.status.service = NavSatStatus.SERVICE_GPS
        return msg

    def _should_publish_now(self) -> bool:
        if self._publish_min_interval_sec <= 0.0:
            return True
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if self._last_publish_time_sec is None:
            self._last_publish_time_sec = now_sec
            return True
        if now_sec - self._last_publish_time_sec + 1e-9 < self._publish_min_interval_sec:
            return False
        self._last_publish_time_sec = now_sec
        return True

    def _publish_with_optional_disturbance(self, msg: NavSatFix) -> None:
        if not self._should_publish_now():
            return
        disturbed = self._disturbance.apply(msg)
        if disturbed is not None:
            self._pub.publish(disturbed)

    def _sensor_gps_cb(self, msg: SensorGps) -> None:
        out = self._base_msg()
        out.latitude = msg.lat * 1e-7
        out.longitude = msg.lon * 1e-7
        alt_msl = msg.alt * 1e-3
        alt_ellipsoid = msg.alt_ellipsoid * 1e-3
        out.altitude = alt_ellipsoid if (msg.alt_ellipsoid != 0 or msg.alt == 0) else alt_msl

        if msg.fix_type >= 2:
            out.status.status = NavSatStatus.STATUS_FIX
        else:
            out.status.status = NavSatStatus.STATUS_NO_FIX

        if math.isfinite(msg.eph) and math.isfinite(msg.epv) and msg.eph > 0.0 and msg.epv > 0.0:
            out.position_covariance[0] = float(msg.eph * msg.eph)
            out.position_covariance[4] = float(msg.eph * msg.eph)
            out.position_covariance[8] = float(msg.epv * msg.epv)
            out.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        self._publish_first_sample_log(
            out.latitude,
            out.longitude,
            out.altitude,
            f"(fix_type={msg.fix_type}, alt_msl={alt_msl:.3f}, alt_ellipsoid={alt_ellipsoid:.3f}, eph={msg.eph:.3f}, epv={msg.epv:.3f})",
        )
        self._publish_with_optional_disturbance(out)

    def _vehicle_global_position_cb(self, msg: VehicleGlobalPosition) -> None:
        out = self._base_msg()
        out.latitude = msg.lat
        out.longitude = msg.lon
        alt_msl = float(msg.alt)
        alt_ellipsoid = float(msg.alt_ellipsoid)
        out.altitude = alt_ellipsoid if (math.isfinite(alt_ellipsoid) and abs(alt_ellipsoid) > 1e-3) or not math.isfinite(alt_msl) else alt_msl
        out.status.status = (
            NavSatStatus.STATUS_NO_FIX if msg.dead_reckoning else NavSatStatus.STATUS_FIX
        )

        if math.isfinite(msg.eph) and math.isfinite(msg.epv) and msg.eph > 0.0 and msg.epv > 0.0:
            out.position_covariance[0] = float(msg.eph * msg.eph)
            out.position_covariance[4] = float(msg.eph * msg.eph)
            out.position_covariance[8] = float(msg.epv * msg.epv)
            out.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        self._publish_first_sample_log(
            out.latitude,
            out.longitude,
            out.altitude,
            f"(dead_reckoning={str(msg.dead_reckoning).lower()}, alt_msl={alt_msl:.3f}, alt_ellipsoid={alt_ellipsoid:.3f}, eph={msg.eph:.3f}, epv={msg.epv:.3f})",
        )
        self._publish_with_optional_disturbance(out)

    def destroy_node(self) -> None:
        self._disturbance.close()
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node = Px4GnssRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
