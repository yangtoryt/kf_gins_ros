#!/usr/bin/env python3
"""
IEKF 轨迹发布器
将 /kf_gins/odom (Odometry) 转换为 /kf_gins/path (Path)
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
import math

class IEKFPathPublisher(Node):
    def __init__(self):
        super().__init__('iekf_path_publisher')
        
        # 参数
        self.input_topic = self.declare_parameter('input_topic', '/kf_gins/odom').value
        # 避免和 kf_gins_node 自带的 /kf_gins/path 冲突
        self.output_topic = self.declare_parameter('output_topic', '/kf_gins/path_iekf').value
        self.frame_id = self.declare_parameter('frame_id', 'map').value
        self.max_path_length = int(self.declare_parameter('max_path_length', 1000).value)
        self.min_distance = float(self.declare_parameter('min_distance', 0.05).value)  # 最小距离阈值（米）
        
        # QoS 配置
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
        
        # 订阅 IEKF odom
        self.sub = self.create_subscription(
            Odometry,
            self.input_topic,
            self.odom_callback,
            qos_sub
        )
        
        # 发布 IEKF path
        self.pub = self.create_publisher(Path, self.output_topic, qos_pub)
        
        # 初始化路径
        self.path = Path()
        self.path.header.frame_id = self.frame_id
        self.last_pos = None
        
        self.get_logger().info('IEKF Path Publisher 已启动')
        self.get_logger().info(f'订阅话题: {self.input_topic}')
        self.get_logger().info(f'发布话题: {self.output_topic}')

    def odom_callback(self, msg: Odometry):
        """处理接收到的 IEKF odometry"""
        
        # 检查距离阈值，避免点太密集
        pos = msg.pose.pose.position
        if self.last_pos:
            dx = pos.x - self.last_pos[0]
            dy = pos.y - self.last_pos[1]
            dz = pos.z - self.last_pos[2]
            dist = math.sqrt(dx*dx + dy*dy + dz*dz)
            
            if dist < self.min_distance:
                return
        
        # 保存当前位置
        self.last_pos = (pos.x, pos.y, pos.z)
        
        # 转换为 PoseStamped
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.header.frame_id = self.frame_id
        pose_stamped.pose = msg.pose.pose
        
        # 添加到路径
        self.path.header.stamp = msg.header.stamp
        self.path.header.frame_id = self.frame_id
        self.path.poses.append(pose_stamped)
        
        # 限制路径长度
        if len(self.path.poses) > self.max_path_length:
            self.path.poses.pop(0)
        
        # 发布路径
        self.pub.publish(self.path)


def main():
    rclpy.init()
    node = IEKFPathPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
