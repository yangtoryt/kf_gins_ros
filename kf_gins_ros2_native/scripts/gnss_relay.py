#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class GnssRelay(Node):
    def __init__(self):
        super().__init__('gnss_relay')
        self.declare_parameter('input_topic', '/mavros/global_position/raw/fix')
        self.declare_parameter('output_topic', '/gps/fix')

        in_topic  = self.get_parameter('input_topic').get_parameter_value().string_value
        out_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        self.sub = self.create_subscription(NavSatFix, in_topic, self.cb, qos_profile_sensor_data)

        pub_qos = QoSProfile(depth=10)
        pub_qos.reliability = ReliabilityPolicy.RELIABLE
        pub_qos.history = HistoryPolicy.KEEP_LAST
        pub_qos.durability = DurabilityPolicy.VOLATILE
        self.pub = self.create_publisher(NavSatFix, out_topic, pub_qos)

        self.get_logger().info(f'GNSS relay: {in_topic} (best_effort) -> {out_topic} (reliable)')

    def cb(self, msg: NavSatFix):
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = GnssRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
