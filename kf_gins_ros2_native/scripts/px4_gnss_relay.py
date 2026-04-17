#!/usr/bin/env python3

import math

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


class Px4GnssRelay(Node):
    def __init__(self) -> None:
        super().__init__("px4_gnss_relay")

        self.declare_parameter("input_mode", "sensor_gps")
        self.declare_parameter("sensor_gps_topic", "/fmu/out/vehicle_gps_position")
        self.declare_parameter("vehicle_global_position_topic", "/fmu/out/vehicle_global_position")
        self.declare_parameter("output_topic", "/gps/fix")
        self.declare_parameter("frame_id", "gps")

        self._input_mode = self.get_parameter("input_mode").get_parameter_value().string_value
        self._sensor_gps_topic = (
            self.get_parameter("sensor_gps_topic").get_parameter_value().string_value
        )
        self._vehicle_global_position_topic = (
            self.get_parameter("vehicle_global_position_topic").get_parameter_value().string_value
        )
        self._frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value

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
        self._pub.publish(out)

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
        self._pub.publish(out)


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
