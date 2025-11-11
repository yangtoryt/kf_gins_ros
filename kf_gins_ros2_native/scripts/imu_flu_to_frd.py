#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

def quat_multiply(a, b):
    return Quaternion(
        x=a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
        y=a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
        z=a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w,
        w=a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z
    )

def quat_from_axis_angle(ax, ay, az, angle):
    s = math.sin(angle/2.0)
    c = math.cos(angle/2.0)
    return Quaternion(x=ax*s, y=ay*s, z=az*s, w=c)

Q_FLU_TO_FRD = quat_from_axis_angle(1.0, 0.0, 0.0, math.pi)  # 绕 X 轴 180°

class ImuFluToFrd(Node):
    def __init__(self):
        super().__init__('imu_flu_to_frd')

        self.declare_parameter('input_topic', '/mavros/imu/data')
        self.declare_parameter('output_topic', '/imu/data')
        self.declare_parameter('flip_gravity', True)

        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.flip_gravity = self.get_parameter('flip_gravity').get_parameter_value().bool_value

        # 订阅：传感器QoS（best_effort）
        self.sub = self.create_subscription(Imu, self.input_topic, self.cb, qos_profile_sensor_data)

        # 发布：reliable，给后端滤波（默认 reliable，但这里显式写清 QoS）
        pub_qos = QoSProfile(depth=10)
        pub_qos.reliability = ReliabilityPolicy.RELIABLE
        pub_qos.history = HistoryPolicy.KEEP_LAST
        pub_qos.durability = DurabilityPolicy.VOLATILE
        self.pub = self.create_publisher(Imu, self.output_topic, pub_qos)

        self.add_on_set_parameters_callback(self._on_set_params)

        self.get_logger().info(f'IMU conv: {self.input_topic} (FLU) -> {self.output_topic} (FRD), flipG={self.flip_gravity}')

    def _on_set_params(self, params):
        ok = True
        for p in params:
            if p.name == 'flip_gravity':
                self.flip_gravity = p.value
        return SetParametersResult(successful=ok)

    def cb(self, msg: Imu):
        out = Imu()
        out.header = msg.header

        out.orientation = quat_multiply(msg.orientation, Q_FLU_TO_FRD)
        out.orientation_covariance = msg.orientation_covariance

        out.angular_velocity.x = msg.angular_velocity.x
        out.angular_velocity.y = -msg.angular_velocity.y
        out.angular_velocity.z = -msg.angular_velocity.z
        out.angular_velocity_covariance = msg.angular_velocity_covariance

        out.linear_acceleration.x = msg.linear_acceleration.x
        if self.flip_gravity:
            out.linear_acceleration.y = -msg.linear_acceleration.y
            out.linear_acceleration.z = -msg.linear_acceleration.z
        else:
            out.linear_acceleration.y = msg.linear_acceleration.y
            out.linear_acceleration.z = msg.linear_acceleration.z
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance

        self.pub.publish(out)

def main():
    rclpy.init()
    node = ImuFluToFrd()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
