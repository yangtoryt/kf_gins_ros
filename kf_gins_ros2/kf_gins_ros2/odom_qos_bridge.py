#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry

class OdomQosBridge(Node):
    def __init__(self):
        super().__init__('kf_odom_qos_bridge')
        in_topic  = self.declare_parameter('input_odom',  '/kf_gins/odom').get_parameter_value().string_value
        out_topic = self.declare_parameter('output_odom', '/kf_gins/odom_reliable').get_parameter_value().string_value

        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=50
        )
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=50
        )

        self.pub = self.create_publisher(Odometry, out_topic, pub_qos)
        self.sub = self.create_subscription(Odometry, in_topic, self.cb, sub_qos)
        self.get_logger().info(f'Bridging {in_topic} -> {out_topic}')

    def cb(self, msg: Odometry):
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = OdomQosBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
