#!/usr/bin/env python3
"""
NavSatFix -> MAVROS GPS_INPUT relay
将 /gps/fix (含遮挡) 注入到 PX4 (mavros gps_input)
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import NavSatFix, NavSatStatus
from mavros_msgs.msg import GPSINPUT


WEEK_SEC = 7 * 24 * 3600

# MAVLink GPS_INPUT ignore_flags bits
IGNORE_FLAG_ALT = 1
IGNORE_FLAG_HDOP = 2
IGNORE_FLAG_VDOP = 4
IGNORE_FLAG_VN = 8
IGNORE_FLAG_VE = 16
IGNORE_FLAG_VD = 32
IGNORE_FLAG_SPEED_ACCURACY = 64
IGNORE_FLAG_HORIZ_ACCURACY = 128
IGNORE_FLAG_VERT_ACCURACY = 256
IGNORE_FLAG_YAW = 512


class GpsFixToMavrosGpsInput(Node):
    def __init__(self):
        super().__init__("gps_fix_to_mavros_gps_input")

        self.declare_parameter("in_fix_topic", "/gps/fix")
        self.declare_parameter("out_gps_input_topic", "/mavros/gps_input")
        self.declare_parameter("use_input_stamp", True)
        self.declare_parameter("gps_id", 0)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("use_covariance", True)

        self.in_fix_topic = self.get_parameter("in_fix_topic").value
        self.out_gps_input_topic = self.get_parameter("out_gps_input_topic").value
        self.use_input_stamp = bool(self.get_parameter("use_input_stamp").value)
        self.gps_id = int(self.get_parameter("gps_id").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.use_covariance = bool(self.get_parameter("use_covariance").value)

        qos_sub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        qos_pub = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.sub = self.create_subscription(
            NavSatFix, self.in_fix_topic, self._on_fix, qos_sub
        )
        self.pub = self.create_publisher(GPSINPUT, self.out_gps_input_topic, qos_pub)

        self.get_logger().info(
            f"GPS_INPUT relay: {self.in_fix_topic} -> {self.out_gps_input_topic} (gps_id={self.gps_id})"
        )

    def _on_fix(self, msg: NavSatFix):
        if not math.isfinite(msg.latitude) or not math.isfinite(msg.longitude) or not math.isfinite(msg.altitude):
            self.get_logger().warn("GNSS fix contains NaN/Inf, skipping.")
            return

        stamp = msg.header.stamp if self.use_input_stamp else self.get_clock().now().to_msg()
        stamp_sec = stamp.sec + stamp.nanosec * 1e-9

        gps = GPSINPUT()
        gps.header.stamp = stamp
        gps.header.frame_id = self.frame_id

        gps.gps_id = self.gps_id
        gps.fix_type = (
            GPSINPUT.GPS_FIX_TYPE_NO_FIX
            if msg.status.status <= NavSatStatus.STATUS_NO_FIX
            else GPSINPUT.GPS_FIX_TYPE_3D_FIX
        )

        gps.lat = int(round(msg.latitude * 1e7))
        gps.lon = int(round(msg.longitude * 1e7))
        gps.alt = float(msg.altitude)

        ignore = 0

        # No velocity from NavSatFix
        ignore |= IGNORE_FLAG_VN | IGNORE_FLAG_VE | IGNORE_FLAG_VD
        ignore |= IGNORE_FLAG_SPEED_ACCURACY | IGNORE_FLAG_YAW

        # Covariance -> accuracy
        horiz_acc = 0.0
        vert_acc = 0.0
        if self.use_covariance and msg.position_covariance_type in (
            NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN,
            NavSatFix.COVARIANCE_TYPE_APPROXIMATED,
        ):
            var_x = msg.position_covariance[0]
            var_y = msg.position_covariance[4]
            var_z = msg.position_covariance[8]
            if var_x >= 0.0 and var_y >= 0.0 and var_z >= 0.0:
                horiz_acc = math.sqrt(max(var_x, var_y))
                vert_acc = math.sqrt(var_z)
            else:
                ignore |= IGNORE_FLAG_HORIZ_ACCURACY | IGNORE_FLAG_VERT_ACCURACY
        else:
            ignore |= IGNORE_FLAG_HORIZ_ACCURACY | IGNORE_FLAG_VERT_ACCURACY

        # No DOP info in NavSatFix
        ignore |= IGNORE_FLAG_HDOP | IGNORE_FLAG_VDOP

        gps.hdop = 0.0
        gps.vdop = 0.0
        gps.vn = 0.0
        gps.ve = 0.0
        gps.vd = 0.0
        gps.speed_accuracy = 0.0
        gps.horiz_accuracy = float(horiz_acc)
        gps.vert_accuracy = float(vert_acc)
        gps.satellites_visible = 255
        gps.yaw = 0

        # GPS week/time from stamp (sim time is OK)
        week = int(stamp_sec // WEEK_SEC) if stamp_sec > 0 else 0
        tow_ms = int((stamp_sec - week * WEEK_SEC) * 1000.0) if stamp_sec > 0 else 0
        gps.time_week = int(week)
        gps.time_week_ms = int(tow_ms)

        gps.ignore_flags = int(ignore)
        self.pub.publish(gps)


def main():
    rclpy.init()
    node = GpsFixToMavrosGpsInput()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
