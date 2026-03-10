#!/usr/bin/env python3
"""
EKF2 状态中继器 - 将 MAVROS 发布的位姿转换为标准格式

功能:
- 订阅 MAVROS /mavros/local_position/pose
- 重新发布为 /ekf2/pose (标准 PoseStamped 格式)
- 同时发布为 /ekf2/odom (Odometry 格式，便于对比)
- 计算和发布速度信息
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import numpy as np
from collections import deque
from typing import Optional


class EKF2StateRelay(Node):
    def __init__(self):
        super().__init__('ekf2_state_relay')
        
        # 参数
        self.input_topic = self.declare_parameter('input_topic', '/mavros/local_position/pose').value
        self.output_topic = self.declare_parameter('output_topic', '/ekf2/pose').value
        self.output_odom_topic = self.declare_parameter('output_odom_topic', self.output_topic + '_odom').value
        self.use_input_stamp = self.declare_parameter('use_input_stamp', True).value
        self.use_covariance = self.declare_parameter('use_covariance', False).value
        self.frame_id = self.declare_parameter('frame_id', 'map').value
        self.child_frame_id = self.declare_parameter('child_frame_id', 'base_link').value
        
        # 发布者 - 使用可靠的 QoS 匹配后处理需求
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.pose_pub = self.create_publisher(
            PoseStamped, 
            self.output_topic,
            qos
        )
        
        self.odom_pub = self.create_publisher(
            Odometry,
            self.output_odom_topic,  # default: /ekf2/pose_odom
            qos
        )
        
        # 订阅 - 使用传感器 QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.pose_sub = self.create_subscription(
            PoseStamped,
            self.input_topic,
            self.pose_callback,
            sensor_qos
        )
        
        # 速度估计 (通过差分)
        self.last_pose: Optional[PoseStamped] = None
        self.last_time: Optional[float] = None
        self.velocity_history = deque(maxlen=5)  # 移动平均
        
        self.get_logger().info(
            f"EKF2StateRelay 已启动\n"
            f"  输入: {self.input_topic}\n"
            f"  输出: {self.output_topic}"
        )

    def pose_callback(self, msg: PoseStamped):
        """处理输入的位姿消息"""
        try:
            stamp = msg.header.stamp if self.use_input_stamp else self.get_clock().now().to_msg()

            # 1. 直接发布位姿 (仅调整 frame_id)
            pose_out = PoseStamped()
            pose_out.header = Header()
            pose_out.header.stamp = stamp
            pose_out.header.frame_id = self.frame_id
            pose_out.pose = msg.pose
            
            self.pose_pub.publish(pose_out)
            
            # 2. 估计速度并发布 Odometry
            current_time_sec = self.msg_to_sec(stamp)
            
            if self.last_pose is not None and self.last_time is not None:
                dt = current_time_sec - self.last_time
                
                if dt > 0.001:  # 避免除以零
                    # 位置差分
                    dx = msg.pose.position.x - self.last_pose.pose.position.x
                    dy = msg.pose.position.y - self.last_pose.pose.position.y
                    dz = msg.pose.position.z - self.last_pose.pose.position.z
                    
                    # 速度
                    vx = dx / dt
                    vy = dy / dt
                    vz = dz / dt
                    
                    # 保存速度历史用于平滑
                    self.velocity_history.append([vx, vy, vz])
                    
                    # 计算平均速度
                    if len(self.velocity_history) > 0:
                        velocities = np.array(list(self.velocity_history))
                        vx_avg = np.mean(velocities[:, 0])
                        vy_avg = np.mean(velocities[:, 1])
                        vz_avg = np.mean(velocities[:, 2])
                    else:
                        vx_avg = vx
                        vy_avg = vy
                        vz_avg = vz
                    
                    # 发布 Odometry
                    odom = Odometry()
                    odom.header = Header()
                    odom.header.stamp = stamp
                    odom.header.frame_id = self.frame_id
                    odom.child_frame_id = self.child_frame_id
                    
                    odom.pose.pose = msg.pose
                    odom.twist.twist.linear.x = vx_avg
                    odom.twist.twist.linear.y = vy_avg
                    odom.twist.twist.linear.z = vz_avg
                    
                    # 协方差 (如果可用)
                    if self.use_covariance:
                        # TODO: 从 MAVROS 获取协方差
                        odom.pose.covariance = [0.0] * 36
                        odom.twist.covariance = [0.0] * 36
                    
                    self.odom_pub.publish(odom)
            
            self.last_pose = pose_out
            self.last_time = current_time_sec
            
        except Exception as e:
            self.get_logger().error(f"处理消息时出错: {e}")

    def msg_to_sec(self, stamp) -> float:
        """将 ROS 时间戳转换为秒"""
        return stamp.sec + stamp.nanosec * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = EKF2StateRelay()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
