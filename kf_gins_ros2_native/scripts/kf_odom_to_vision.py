#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import math


class KFOdomToVision(Node):
    def __init__(self):
        super().__init__('kf_odom_to_vision')

        in_topic = self.declare_parameter(
            'input_odom', '/kf_gins/odom'
        ).get_parameter_value().string_value

        out_topic = self.declare_parameter(
            'output_pose', '/mavros/vision_pose/pose'
        ).get_parameter_value().string_value

        frame_id = self.declare_parameter(
            'frame_id', 'map'
        ).get_parameter_value().string_value

        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._sub = self.create_subscription(
            Odometry, in_topic, self._cb_odom, sub_qos
        )
        self._pub = self.create_publisher(
            PoseStamped, out_topic, pub_qos
        )

        self._frame_id = frame_id
        self.get_logger().info(
            f'KFOdomToVision: {in_topic} -> {out_topic}, frame_id={frame_id}'
        )

   
    def _cb_odom(self, msg: Odometry):
        # 简单 NaN 过滤，发现有非有限数就直接丢掉这帧
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        vals = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
        if not all(math.isfinite(v) for v in vals):
            return

        out = PoseStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self._frame_id or msg.header.frame_id
        out.pose = msg.pose.pose
        self._pub.publish(out)


def main():
    rclpy.init()
    node = KFOdomToVision()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
