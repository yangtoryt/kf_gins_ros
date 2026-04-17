#!/usr/bin/env python3

import math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import TwistStamped
from px4_msgs.msg import VehicleOdometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Header


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def quaternion_wxyz_to_rpy(w: float, x: float, y: float, z: float) -> Tuple[float, float, float]:
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    pitch = math.asin(clamp(sinp, -1.0, 1.0))

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def rpy_to_quaternion_xyzw(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return x, y, z, w


def rotate_body_frd_to_ned(
    w: float, x: float, y: float, z: float, vx: float, vy: float, vz: float
) -> Tuple[float, float, float]:
    r00 = 1.0 - 2.0 * (y * y + z * z)
    r01 = 2.0 * (x * y - z * w)
    r02 = 2.0 * (x * z + y * w)
    r10 = 2.0 * (x * y + z * w)
    r11 = 1.0 - 2.0 * (x * x + z * z)
    r12 = 2.0 * (y * z - x * w)
    r20 = 2.0 * (x * z - y * w)
    r21 = 2.0 * (y * z + x * w)
    r22 = 1.0 - 2.0 * (x * x + y * y)

    return (
        r00 * vx + r01 * vy + r02 * vz,
        r10 * vx + r11 * vy + r12 * vz,
        r20 * vx + r21 * vy + r22 * vz,
    )


class PX4AuxStateRelay(Node):
    def __init__(self) -> None:
        super().__init__("px4_aux_state_relay")

        self.vehicle_odometry_topic = self.declare_parameter(
            "vehicle_odometry_topic", "/fmu/out/vehicle_odometry"
        ).value
        self.imu_output_topic = self.declare_parameter(
            "imu_output_topic", "/px4_aux/imu/data"
        ).value
        self.velocity_output_topic = self.declare_parameter(
            "velocity_output_topic", "/px4_aux/local_position/velocity_local"
        ).value
        self.use_px4_stamp = bool(self.declare_parameter("use_px4_stamp", True).value)
        self.imu_frame_id = self.declare_parameter("imu_frame_id", "base_link").value
        self.velocity_frame_id = self.declare_parameter("velocity_frame_id", "map").value

        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.imu_pub = self.create_publisher(Imu, self.imu_output_topic, pub_qos)
        self.velocity_pub = self.create_publisher(TwistStamped, self.velocity_output_topic, pub_qos)
        self.sub = self.create_subscription(
            VehicleOdometry, self.vehicle_odometry_topic, self.vehicle_odometry_cb, sub_qos
        )

        self.logged_first_heading = False
        self.logged_first_velocity = False
        self.logged_first_input = False
        self.warned_velocity_frame = False

        self.get_logger().info(
            "PX4AuxStateRelay started\n"
            f"  input: {self.vehicle_odometry_topic}\n"
            f"  imu_output: {self.imu_output_topic}\n"
            f"  velocity_output: {self.velocity_output_topic}"
        )

    def use_px4_timestamp(self, timestamp_us: int):
        stamp_sec = float(timestamp_us) * 1e-6
        sec = int(stamp_sec)
        nanosec = int(round((stamp_sec - sec) * 1e9))
        if nanosec >= 1_000_000_000:
            sec += 1
            nanosec -= 1_000_000_000
        stamp = self.get_clock().now().to_msg()
        stamp.sec = sec
        stamp.nanosec = nanosec
        return stamp

    def resolve_velocity_enu(
        self, msg: VehicleOdometry, q_w: float, q_x: float, q_y: float, q_z: float
    ) -> Optional[Tuple[float, float, float]]:
        vx = float(msg.velocity[0])
        vy = float(msg.velocity[1])
        vz = float(msg.velocity[2])
        if not (math.isfinite(vx) and math.isfinite(vy) and math.isfinite(vz)):
            return None

        if msg.velocity_frame == VehicleOdometry.VELOCITY_FRAME_NED:
            return vy, vx, -vz

        if msg.velocity_frame == VehicleOdometry.VELOCITY_FRAME_BODY_FRD:
            vel_ned = rotate_body_frd_to_ned(q_w, q_x, q_y, q_z, vx, vy, vz)
            return vel_ned[1], vel_ned[0], -vel_ned[2]

        if not self.warned_velocity_frame:
            self.get_logger().warn(
                f"Unsupported VehicleOdometry velocity_frame={msg.velocity_frame} on {self.vehicle_odometry_topic}"
            )
            self.warned_velocity_frame = True
        return None

    def vehicle_odometry_cb(self, msg: VehicleOdometry) -> None:
        if not self.logged_first_input:
            self.get_logger().info(
                "Received first VehicleOdometry sample: "
                f"timestamp={int(msg.timestamp)} "
                f"position_frame={int(msg.pose_frame)} velocity_frame={int(msg.velocity_frame)} "
                f"q0={float(msg.q[0]):.6f} q1={float(msg.q[1]):.6f} "
                f"q2={float(msg.q[2]):.6f} q3={float(msg.q[3]):.6f}"
            )
            self.logged_first_input = True

        q_w = float(msg.q[0])
        q_x = float(msg.q[1])
        q_y = float(msg.q[2])
        q_z = float(msg.q[3])
        if not (math.isfinite(q_w) and math.isfinite(q_x) and math.isfinite(q_y) and math.isfinite(q_z)):
            return

        stamp = self.use_px4_timestamp(msg.timestamp) if self.use_px4_stamp else self.get_clock().now().to_msg()

        roll_ned, pitch_ned, yaw_ned = quaternion_wxyz_to_rpy(q_w, q_x, q_y, q_z)
        roll_enu = roll_ned
        pitch_enu = -pitch_ned
        yaw_enu = math.pi * 0.5 - yaw_ned
        quat_enu = rpy_to_quaternion_xyzw(roll_enu, pitch_enu, yaw_enu)

        imu_msg = Imu()
        imu_msg.header = Header()
        imu_msg.header.stamp = stamp
        imu_msg.header.frame_id = self.imu_frame_id
        imu_msg.orientation.x = float(quat_enu[0])
        imu_msg.orientation.y = float(quat_enu[1])
        imu_msg.orientation.z = float(quat_enu[2])
        imu_msg.orientation.w = float(quat_enu[3])
        imu_msg.angular_velocity.x = float(msg.angular_velocity[0]) if math.isfinite(msg.angular_velocity[0]) else 0.0
        imu_msg.angular_velocity.y = float(-msg.angular_velocity[1]) if math.isfinite(msg.angular_velocity[1]) else 0.0
        imu_msg.angular_velocity.z = float(-msg.angular_velocity[2]) if math.isfinite(msg.angular_velocity[2]) else 0.0
        self.imu_pub.publish(imu_msg)

        if not self.logged_first_heading:
            yaw_ned_deg = math.degrees(yaw_ned)
            self.get_logger().info(
                f"Published first heading relay sample: yaw_ned={yaw_ned_deg:.2f} deg topic={self.imu_output_topic}"
            )
            self.logged_first_heading = True

        velocity_enu = self.resolve_velocity_enu(msg, q_w, q_x, q_y, q_z)
        if velocity_enu is None:
            return

        vel_msg = TwistStamped()
        vel_msg.header = Header()
        vel_msg.header.stamp = stamp
        vel_msg.header.frame_id = self.velocity_frame_id
        vel_msg.twist.linear.x = float(velocity_enu[0])
        vel_msg.twist.linear.y = float(velocity_enu[1])
        vel_msg.twist.linear.z = float(velocity_enu[2])
        self.velocity_pub.publish(vel_msg)

        if not self.logged_first_velocity:
            speed = math.sqrt(
                velocity_enu[0] * velocity_enu[0]
                + velocity_enu[1] * velocity_enu[1]
                + velocity_enu[2] * velocity_enu[2]
            )
            self.get_logger().info(
                "Published first velocity relay sample: "
                f"vx={velocity_enu[0]:.3f} vy={velocity_enu[1]:.3f} vz={velocity_enu[2]:.3f} speed={speed:.3f} "
                f"topic={self.velocity_output_topic}"
            )
            self.logged_first_velocity = True


def main(args=None):
    rclpy.init(args=args)
    node = PX4AuxStateRelay()
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
