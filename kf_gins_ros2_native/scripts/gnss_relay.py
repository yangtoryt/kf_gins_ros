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
        self.declare_parameter('subscribe_enable', True)
        self.declare_parameter('publish_enable', True)
        # 兼容旧参数名（部分 launch 使用 in_topic/out_topic）
        self.declare_parameter('in_topic', '')
        self.declare_parameter('out_topic', '')

        in_topic = self.get_parameter('in_topic').get_parameter_value().string_value
        out_topic = self.get_parameter('out_topic').get_parameter_value().string_value
        if not in_topic:
            in_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        if not out_topic:
            out_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        subscribe_enable = self.get_parameter('subscribe_enable').get_parameter_value().bool_value
        publish_enable = self.get_parameter('publish_enable').get_parameter_value().bool_value

        self.sub = None
        if subscribe_enable:
            self.sub = self.create_subscription(NavSatFix, in_topic, self.cb, qos_profile_sensor_data)

        self.pub = None
        if subscribe_enable and publish_enable:
            pub_qos = QoSProfile(depth=10)
            pub_qos.reliability = ReliabilityPolicy.RELIABLE
            pub_qos.history = HistoryPolicy.KEEP_LAST
            pub_qos.durability = DurabilityPolicy.VOLATILE
            self.pub = self.create_publisher(NavSatFix, out_topic, pub_qos)

        if not subscribe_enable:
            self.get_logger().info(f'GNSS relay process-only: subscription disabled, publishing disabled')
        elif publish_enable:
            self.get_logger().info(f'GNSS relay: {in_topic} (best_effort) -> {out_topic} (reliable)')
        else:
            self.get_logger().info(f'GNSS relay subscribe-only: {in_topic} (best_effort), publishing disabled')

    def cb(self, msg: NavSatFix):
        if self.pub is not None:
            self.pub.publish(msg)

def main():
    rclpy.init()
    node = GnssRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
