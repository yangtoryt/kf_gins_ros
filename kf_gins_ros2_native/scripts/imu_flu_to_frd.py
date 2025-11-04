#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Imu

class ImuFluToFrd(Node):
    def __init__(self):
        super().__init__('imu_flu_to_frd')
        in_topic  = self.declare_parameter('in_topic',  '/imu/data_flu').get_parameter_value().string_value
        out_topic = self.declare_parameter('out_topic', '/imu/data').get_parameter_value().string_value
        self.flip_g = self.declare_parameter('flip_gravity_sign_to_frd', True).get_parameter_value().bool_value

        qos = QoSProfile(depth=50)
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos.history = QoSHistoryPolicy.KEEP_LAST

        self.sub = self.create_subscription(Imu, in_topic, self.cb, qos)
        self.pub = self.create_publisher(Imu, out_topic, qos)
        self.get_logger().info(f'IMU conv: {in_topic} (FLU) -> {out_topic} (FRD), flipG={self.flip_g}')

    def cb(self, m: Imu):
        out = Imu()
        out.header = m.header
        out.header.frame_id = 'base_link'

        # 姿态直接透传（避免额外不确定旋转）
        out.orientation = m.orientation
        out.orientation_covariance = m.orientation_covariance

        # R_x(pi): (x, y, z) -> (x, -y, -z)
        out.angular_velocity.x =  m.angular_velocity.x
        out.angular_velocity.y = -m.angular_velocity.y
        out.angular_velocity.z = -m.angular_velocity.z
        out.angular_velocity_covariance = m.angular_velocity_covariance

        ax =  m.linear_acceleration.x
        ay = -m.linear_acceleration.y
        az = -m.linear_acceleration.z

        if self.flip_g and abs(ax) < 0.5 and abs(ay) < 0.5 and abs(az) > 5.0:
            az = abs(az)

        out.linear_acceleration.x = ax
        out.linear_acceleration.y = ay
        out.linear_acceleration.z = az
        out.linear_acceleration_covariance = m.linear_acceleration_covariance

        self.pub.publish(out)

def main():
    rclpy.init()
    node = ImuFluToFrd()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
