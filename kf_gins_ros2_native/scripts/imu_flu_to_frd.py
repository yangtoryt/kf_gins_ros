#!/usr/bin/env python3
"""
改进的IMU转换脚本 - 强制处理时间戳

关键改进：
1. 强制检查并处理dt=0消息
2. 添加详细的时间戳调试日志
3. 实现消息缓冲确保时间单调性
4. 添加统计信息
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from collections import deque

def quat_multiply(a, b):
    return Quaternion(
        x=a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
        y=a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
        z=a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w,
        w=a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z
    )

def quat_from_axis_angle(ax, ay, az, angle):
    s = math.sin(angle/2.0)
    c = math.cos(angle/2.0)
    return Quaternion(x=ax*s, y=ay*s, z=az*s, w=c)

Q_FLU_TO_FRD = quat_from_axis_angle(1.0, 0.0, 0.0, math.pi)

class ImuFluToFrdImproved(Node):
    def __init__(self):
        super().__init__('imu_flu_to_frd')

        self.declare_parameter('input_topic', '/mavros/imu/data')
        self.declare_parameter('output_topic', '/imu/data')
        self.declare_parameter('flip_gravity', True)
        self.declare_parameter('use_node_stamp', False)
        self.declare_parameter('force_dt_ms', -1)  # -1=auto, >=0=强制时间间隔(ms)
        self.declare_parameter('force_monotonic_stamp', True)
        self.declare_parameter('fallback_to_input_stamp', True)
        self.declare_parameter('debug_log', True)

        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.flip_gravity = self.get_parameter('flip_gravity').get_parameter_value().bool_value
        self.use_node_stamp = self.get_parameter('use_node_stamp').get_parameter_value().bool_value
        self.force_dt_ms = self.get_parameter('force_dt_ms').get_parameter_value().integer_value
        self.force_monotonic_stamp = self.get_parameter('force_monotonic_stamp').get_parameter_value().bool_value
        self.fallback_to_input_stamp = self.get_parameter('fallback_to_input_stamp').get_parameter_value().bool_value
        self.debug_log = self.get_parameter('debug_log').get_parameter_value().bool_value

        self.sub = self.create_subscription(Imu, self.input_topic, self.cb, qos_profile_sensor_data)

        pub_qos = QoSProfile(depth=10)
        pub_qos.reliability = ReliabilityPolicy.RELIABLE
        pub_qos.history = HistoryPolicy.KEEP_LAST
        pub_qos.durability = DurabilityPolicy.VOLATILE
        self.pub = self.create_publisher(Imu, self.output_topic, pub_qos)

        self.add_on_set_parameters_callback(self._on_set_params)

        # 统计信息
        self.msg_count = 0
        self.zero_dt_count = 0
        self.backward_dt_count = 0
        self.adjusted_stamp_count = 0
        self.last_stamp = None
        self.estimated_dt_ms = 0
        self.dt_history = deque(maxlen=100)

        self.get_logger().info(
            f'IMU conv: {self.input_topic} (FLU) -> {self.output_topic} (FRD), '
            f'flipG={self.flip_gravity}, use_node_stamp={self.use_node_stamp}, '
            f'force_dt_ms={self.force_dt_ms}, force_monotonic_stamp={self.force_monotonic_stamp}'
        )

    def _on_set_params(self, params):
        ok = True
        for p in params:
            if p.name == 'flip_gravity':
                self.flip_gravity = p.value
            elif p.name == 'use_node_stamp':
                self.use_node_stamp = p.value
                self.get_logger().info(f'Updated use_node_stamp={self.use_node_stamp}')
            elif p.name == 'force_dt_ms':
                self.force_dt_ms = p.value
            elif p.name == 'force_monotonic_stamp':
                self.force_monotonic_stamp = p.value
            elif p.name == 'fallback_to_input_stamp':
                self.fallback_to_input_stamp = p.value
            elif p.name == 'debug_log':
                self.debug_log = p.value
        return SetParametersResult(successful=ok)

    def cb(self, msg: Imu):
        self.msg_count += 1
        
        # 获取原始时间戳
        input_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        # ========== 时间戳处理 ==========
        final_stamp_sec = input_stamp
        if self.use_node_stamp:
            # 使用ROS /clock时间戳（应该来自Gazebo）
            final_stamp = self.get_clock().now().to_msg()
            final_stamp_sec = final_stamp.sec + final_stamp.nanosec * 1e-9
        else:
            # 直接使用输入时间戳
            final_stamp = msg.header.stamp
            final_stamp_sec = input_stamp
        
        # ========== 时间戳一致性检查 ==========
        dt_sec = None
        dt_ms = None
        dt_sec_adj = None
        dt_ms_adj = None
        if self.last_stamp is not None:
            dt_sec = final_stamp_sec - self.last_stamp
            dt_ms = dt_sec * 1000
            
            # 记录统计
            if dt_ms == 0:
                self.zero_dt_count += 1
            if dt_sec < 0:
                self.backward_dt_count += 1
            if dt_ms > 0:  # 忽略负的时间差
                self.dt_history.append(dt_ms)
                self.estimated_dt_ms = sum(self.dt_history) / len(self.dt_history)

            # 当 /clock 不前进时可回退到输入时间戳
            if (self.use_node_stamp and self.fallback_to_input_stamp and dt_sec <= 0.0
                    and input_stamp > self.last_stamp):
                final_stamp_sec = input_stamp
                dt_sec = final_stamp_sec - self.last_stamp
                dt_ms = dt_sec * 1000
                self.adjusted_stamp_count += 1
            
            # 强制时间间隔（优先级最高）
            if self.force_dt_ms >= 0:
                final_stamp_sec = self.last_stamp + self.force_dt_ms / 1000.0
                dt_sec = final_stamp_sec - self.last_stamp
                dt_ms = dt_sec * 1000
                self.adjusted_stamp_count += 1

            # 最后一层保护：强制单调递增
            if self.force_monotonic_stamp and dt_sec is not None and dt_sec <= 0.0:
                fallback_dt = self.estimated_dt_ms if self.estimated_dt_ms > 0 else 10.0
                final_stamp_sec = self.last_stamp + fallback_dt / 1000.0
                dt_sec = final_stamp_sec - self.last_stamp
                dt_ms = dt_sec * 1000
                self.adjusted_stamp_count += 1

            # 更新“调整后”的统计
            dt_sec_adj = final_stamp_sec - self.last_stamp
            dt_ms_adj = dt_sec_adj * 1000.0
            if dt_ms_adj > 0:
                self.dt_history.append(dt_ms_adj)
                self.estimated_dt_ms = sum(self.dt_history) / len(self.dt_history)
            
            # 调试输出
            if self.debug_log and self.msg_count % 100 == 0:
                zero_pct = 100 * self.zero_dt_count / self.msg_count if self.msg_count > 0 else 0
                back_pct = 100 * self.backward_dt_count / self.msg_count if self.msg_count > 0 else 0
                adj_pct = 100 * self.adjusted_stamp_count / self.msg_count if self.msg_count > 0 else 0
                self.get_logger().info(
                    f'[消息 #{self.msg_count}] dt={dt_ms_adj if dt_ms_adj is not None else dt_ms:.2f}ms, '
                    f'平均dt={self.estimated_dt_ms:.2f}ms, '
                    f'dt=0占比={zero_pct:.1f}%, dt<0占比={back_pct:.1f}%, 调整占比={adj_pct:.1f}%'
                )
        
        self.last_stamp = final_stamp_sec
        
        # ========== 构建输出消息 ==========
        out = Imu()
        out.header = msg.header
        out.header.stamp.sec = int(final_stamp_sec)
        out.header.stamp.nanosec = int((final_stamp_sec - out.header.stamp.sec) * 1e9)
        
        # 四元数变换
        out.orientation = quat_multiply(msg.orientation, Q_FLU_TO_FRD)
        out.orientation_covariance = msg.orientation_covariance

        # 角速度变换
        out.angular_velocity.x = msg.angular_velocity.x
        out.angular_velocity.y = -msg.angular_velocity.y
        out.angular_velocity.z = -msg.angular_velocity.z
        out.angular_velocity_covariance = msg.angular_velocity_covariance

        # 线加速度变换
        out.linear_acceleration.x = msg.linear_acceleration.x
        if self.flip_gravity:
            out.linear_acceleration.y = -msg.linear_acceleration.y
            out.linear_acceleration.z = -msg.linear_acceleration.z
        else:
            out.linear_acceleration.y = msg.linear_acceleration.y
            out.linear_acceleration.z = msg.linear_acceleration.z
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance

        self.pub.publish(out)

def main():
    rclpy.init()
    node = ImuFluToFrdImproved()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
