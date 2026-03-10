#!/usr/bin/env python3
"""
NavSatFix -> MAVROS HIL_GPS relay
将 /gps/fix (含遮挡) 注入到 PX4 (mavros hil gps)
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import NavSatFix, NavSatStatus
from mavros_msgs.msg import HilGPS

EARTH_RADIUS_M = 6378137.0
CM_PER_M = 100.0

GPS_FIX_NO_FIX = 0
GPS_FIX_2D = 2
GPS_FIX_3D = 3
GPS_FIX_DGPS = 4
GPS_FIX_RTK = 5


def _clamp_int16(value: float) -> int:
    return max(-32768, min(32767, int(round(value))))


def _clamp_u16(value: float) -> int:
    return max(0, min(65535, int(round(value))))


class GpsFixToMavrosHilGps(Node):
    def __init__(self):
        super().__init__("gps_fix_to_mavros_hil_gps")

        self.declare_parameter("in_fix_topic", "/gps/fix")
        self.declare_parameter("out_hil_gps_topic", "/mavros/hil/gps")
        self.declare_parameter("use_input_stamp", True)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("use_covariance", True)
        self.declare_parameter("satellites_visible", 10)

        self.in_fix_topic = self.get_parameter("in_fix_topic").value
        self.out_hil_gps_topic = self.get_parameter("out_hil_gps_topic").value
        self.use_input_stamp = bool(self.get_parameter("use_input_stamp").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.use_covariance = bool(self.get_parameter("use_covariance").value)
        self.satellites_visible = int(self.get_parameter("satellites_visible").value)

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
        self.pub = self.create_publisher(HilGPS, self.out_hil_gps_topic, qos_pub)

        self._last_valid_fix = None
        self._last_valid_stamp_sec = None

        self.get_logger().info(
            f"HIL_GPS relay: {self.in_fix_topic} -> {self.out_hil_gps_topic}"
        )

    def _on_fix(self, msg: NavSatFix):
        if not math.isfinite(msg.latitude) or not math.isfinite(msg.longitude) or not math.isfinite(msg.altitude):
            self.get_logger().warn("GNSS fix contains NaN/Inf, skipping.")
            return

        stamp = msg.header.stamp if self.use_input_stamp else self.get_clock().now().to_msg()
        stamp_sec = stamp.sec + stamp.nanosec * 1e-9

        gps = HilGPS()
        gps.header.stamp = stamp
        gps.header.frame_id = self.frame_id

        if msg.status.status <= NavSatStatus.STATUS_NO_FIX:
            fix_type = GPS_FIX_NO_FIX
        elif msg.status.status == NavSatStatus.STATUS_SBAS_FIX:
            fix_type = GPS_FIX_DGPS
        elif msg.status.status == NavSatStatus.STATUS_GBAS_FIX:
            fix_type = GPS_FIX_RTK
        else:
            fix_type = GPS_FIX_3D
        gps.fix_type = int(fix_type)

        gps.geo.latitude = float(msg.latitude)
        gps.geo.longitude = float(msg.longitude)
        gps.geo.altitude = float(msg.altitude)

        eph = 65535
        epv = 65535
        if self.use_covariance and msg.position_covariance_type in (
            NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN,
            NavSatFix.COVARIANCE_TYPE_APPROXIMATED,
        ):
            var_x = msg.position_covariance[0]
            var_y = msg.position_covariance[4]
            var_z = msg.position_covariance[8]
            if var_x >= 0.0 and var_y >= 0.0 and var_z >= 0.0:
                horiz_std = math.sqrt(max(var_x, var_y))
                vert_std = math.sqrt(var_z)
                eph = _clamp_u16(horiz_std * CM_PER_M)
                epv = _clamp_u16(vert_std * CM_PER_M)
        if fix_type == GPS_FIX_NO_FIX:
            eph = 65535
            epv = 65535

        gps.eph = int(eph)
        gps.epv = int(epv)

        vn = ve = vd = vel = cog = 0
        if fix_type != GPS_FIX_NO_FIX and self._last_valid_fix is not None and self._last_valid_stamp_sec is not None:
            dt = stamp_sec - self._last_valid_stamp_sec
            if dt > 1e-3:
                lat0 = math.radians(self._last_valid_fix.latitude)
                lat1 = math.radians(msg.latitude)
                lon0 = math.radians(self._last_valid_fix.longitude)
                lon1 = math.radians(msg.longitude)
                dlat = lat1 - lat0
                dlon = lon1 - lon0
                lat_avg = 0.5 * (lat0 + lat1)
                dn = dlat * EARTH_RADIUS_M
                de = dlon * EARTH_RADIUS_M * math.cos(lat_avg)
                du = msg.altitude - self._last_valid_fix.altitude

                vn_m_s = dn / dt
                ve_m_s = de / dt
                vd_m_s = -du / dt

                vn = _clamp_int16(vn_m_s * CM_PER_M)
                ve = _clamp_int16(ve_m_s * CM_PER_M)
                vd = _clamp_int16(vd_m_s * CM_PER_M)
                vel = _clamp_u16(math.hypot(vn_m_s, ve_m_s) * CM_PER_M)
                if vel > 0:
                    cog_deg = (math.degrees(math.atan2(ve_m_s, vn_m_s)) + 360.0) % 360.0
                    cog = _clamp_u16(cog_deg * 100.0)

        gps.vn = int(vn)
        gps.ve = int(ve)
        gps.vd = int(vd)
        gps.vel = int(vel)
        gps.cog = int(cog)
        gps.satellites_visible = 0 if fix_type == GPS_FIX_NO_FIX else max(0, min(255, self.satellites_visible))

        if fix_type != GPS_FIX_NO_FIX:
            self._last_valid_fix = msg
            self._last_valid_stamp_sec = stamp_sec

        self.pub.publish(gps)


def main():
    rclpy.init()
    node = GpsFixToMavrosHilGps()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
