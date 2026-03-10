#!/usr/bin/env python3
"""
EKF2 轨迹发布器
将 /ekf2/pose (PoseStamped) 转换为 /ekf2/path (Path)
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import math

class EKF2PathPublisher(Node):
    def __init__(self):
        super().__init__('ekf2_path_publisher')
        
        # 参数
        self.input_topic = self.declare_parameter('input_topic', '/ekf2/pose').value
        self.output_topic = self.declare_parameter('output_topic', '/ekf2/path').value
        self.frame_id = self.declare_parameter('frame_id', 'map').value
        self.max_path_length = int(self.declare_parameter('max_path_length', 1000).value)
        self.min_distance = float(self.declare_parameter('min_distance', 0.05).value)  # 最小距离阈值（米）
        
        # QoS：订阅用 best_effort，发布 Path 用 reliable（RViz 更容易显示）
        qos_sub = QoSProfile(
            depth=100,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )
        qos_pub = QoSProfile(
            depth=10,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        
        # 订阅 EKF2 pose
        self.sub = self.create_subscription(
            PoseStamped,
            self.input_topic,
            self.pose_callback,
            qos_sub
        )
        
        # 发布 EKF2 path
        self.pub = self.create_publisher(Path, self.output_topic, qos_pub)
        
        # 初始化路径
        self.path = Path()
        self.path.header.frame_id = self.frame_id
        self.last_pos = None
        
        self.get_logger().info('EKF2 Path Publisher 已启动')
        self.get_logger().info(f'订阅话题: {self.input_topic}')
        self.get_logger().info(f'发布话题: {self.output_topic}')

    def pose_callback(self, msg: PoseStamped):
        """处理接收到的 EKF2 pose"""
        
        # 检查距离阈值，避免点太密集
        pos = msg.pose.position
        if self.last_pos:
            dx = pos.x - self.last_pos[0]
            dy = pos.y - self.last_pos[1]
            dz = pos.z - self.last_pos[2]
            dist = math.sqrt(dx*dx + dy*dy + dz*dz)
            
            if dist < self.min_distance:
                return
        
        # 保存当前位置
        self.last_pos = (pos.x, pos.y, pos.z)
        
        # 添加到路径
        self.path.header.stamp = msg.header.stamp
        self.path.header.frame_id = self.frame_id
        self.path.poses.append(msg)
        
        # 限制路径长度
        if len(self.path.poses) > self.max_path_length:
            self.path.poses.pop(0)
        
        # 发布路径
        self.pub.publish(self.path)


def main():
    rclpy.init()
    node = EKF2PathPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
