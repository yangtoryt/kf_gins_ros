#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
import math

class OdomToPath(Node):
    def __init__(self):
        super().__init__('odom_to_path')
        odom_topic = self.declare_parameter('odom_topic', '/quad1/ground_truth').get_parameter_value().string_value
        path_topic = self.declare_parameter('path_topic', '/quad1/ground_truth_path').get_parameter_value().string_value
        self.frame_id = self.declare_parameter('frame_id', 'map').get_parameter_value().string_value
        self.decimation = self.declare_parameter('decimation', 5).get_parameter_value().integer_value
        self.min_dist = float(self.declare_parameter('min_dist_m', 0.05).get_parameter_value().double_value)

        qos = QoSProfile(depth=100)
        self.sub = self.create_subscription(Odometry, odom_topic, self.cb, qos)
        self.pub = self.create_publisher(Path, path_topic, qos)

        self.path = Path()
        self.path.header.frame_id = self.frame_id
        self.counter = 0
        self.last = None
        self.get_logger().info(f'Ground truth path: {odom_topic} -> {path_topic}')

    def cb(self, m: Odometry):
        self.counter += 1
        if self.counter % self.decimation != 0:
            return
        p = m.pose.pose.position
        if self.last:
            dx = p.x - self.last[0]; dy = p.y - self.last[1]; dz = p.z - self.last[2]
            if math.sqrt(dx*dx + dy*dy + dz*dz) < self.min_dist:
                return
        self.last = (p.x, p.y, p.z)

        ps = PoseStamped()
        ps.header = m.header
        ps.header.frame_id = self.frame_id
        ps.pose = m.pose.pose

        self.path.header.stamp = m.header.stamp
        self.path.poses.append(ps)
        self.pub.publish(self.path)

def main():
    rclpy.init()
    node = OdomToPath()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
