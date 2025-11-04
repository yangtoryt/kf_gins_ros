#!/usr/bin/env python3
import rclpy, math, time
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Wrench

class HoverMove(Node):
    def __init__(self):
        super().__init__('hover_move')

        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos.history = QoSHistoryPolicy.KEEP_LAST

        self.pub = self.create_publisher(Wrench, '/quad1/gazebo_ros_force', qos)
        self.t0 = self.get_clock().now().nanoseconds * 1e-9

        # 质量 1.5 kg
        self.mass = 1.5
        self.g    = 9.81
        self.hover_fz = self.mass * self.g  # 14.715 N

        self.timer = self.create_timer(0.01, self.step)

    def step(self):
        t = self.get_clock().now().nanoseconds * 1e-9 - self.t0
        m = Wrench()

        if t < 2.0:
            # 2s 准备阶段：不推力，稳一稳
            fz = 0.0
        elif t < 4.0:
            # 2-4s 起升：给 hover + 6N 的富余
            fz = self.hover_fz + 6.0
        else:
            # 4s 后悬停：精确 hover
            fz = self.hover_fz
            # 可选：加一点体轴 x 向前推力试试水平移动
            # m.force.x = 3.0

        m.force.z = fz
        self.pub.publish(m)
        if abs(t - round(t, 2)) < 1e-3:
            self.get_logger().info(f't={t:.2f}s force=({m.force.x:.1f},{m.force.y:.1f},{m.force.z:.1f})')

def main():
    rclpy.init()
    node = HoverMove()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
