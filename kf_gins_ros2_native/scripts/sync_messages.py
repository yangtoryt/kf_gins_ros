#!/usr/bin/env python3
"""
消息同步器占位符 - 简化版本

注意: 实际的消息同步功能已集成到 real_time_comparison.py 中
此脚本仅用于兼容 launch 文件配置
"""

import rclpy
from rclpy.node import Node


class SimpleMessageSynchronizer(Node):
    """简化的消息同步器"""
    
    def __init__(self):
        super().__init__('message_synchronizer')
        self.get_logger().info("MessageSynchronizer (简化版) 已启动")
        self.get_logger().info("消息同步功能已集成到 real_time_comparison.py")


def main(args=None):
    rclpy.init(args=args)
    node = SimpleMessageSynchronizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

