#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Quaternion
from gazebo_msgs.msg import EntityState, ModelState, ModelStates
from gazebo_msgs.srv import SetEntityState, SetModelState


def euler_to_quat(roll: float, pitch: float, yaw: float) -> Quaternion:
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


def angle_diff(a, b):
    d = a - b
    while d > math.pi:
        d -= 2.0 * math.pi
    while d < -math.pi:
        d += 2.0 * math.pi
    return d


class OdomToGazebo(Node):
    def __init__(self):
        super().__init__('odom_to_gazebo')

        # 参数
        self.declare_parameter('entity_name', 'replay_uav')
        self.declare_parameter('odom_topic',  '/kf_gins/odom_reliable')
        self.declare_parameter('decimate', 10)
        self.declare_parameter('z_offset', 0.1)   # 稍微抬高一点
        self.declare_parameter('vel_thresh', 0.5) # 速度低于这个就认为基本静止 [m/s]

        self.entity_name = self.get_parameter('entity_name').value
        odom_topic       = self.get_parameter('odom_topic').value
        self.decimate    = int(self.get_parameter('decimate').value)
        self.z_offset    = float(self.get_parameter('z_offset').value)
        self.vel_thresh  = float(self.get_parameter('vel_thresh').value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=50
        )

        # client & 模式
        self._cli  = None       # rclpy client
        self._mode = None       # 'entity' or 'model'
        self._cnt  = 0

        # 只保留一个 yaw 状态
        self._yaw  = 0.0

        # 候选服务名
        self._entity_srv_names = ['/set_entity_state', '/gazebo/set_entity_state']
        self._model_srv_names  = ['/set_model_state',  '/gazebo/set_model_state']

        self._try_connect_service()
        self.create_timer(1.0, self._retry_service)

        # odom 订阅
        self.create_subscription(Odometry, odom_topic, self._cb_odom, qos)
        self.get_logger().info(
            f'Listening odom: {odom_topic}, entity: {self.entity_name}, '
            f'z_offset={self.z_offset}, vel_thresh={self.vel_thresh}'
        )

        # 只有在 /model_states 里确认模型存在之后才开始推状态
        self._entity_ready = False
        self.create_subscription(ModelStates, '/model_states', self._cb_models, 10)

    # ----------- 连接服务 -----------
    def _try_connect_service(self):
        for name in self._entity_srv_names:
            cli = self.create_client(SetEntityState, name)
            if cli.wait_for_service(timeout_sec=0.5):
                self._cli = cli
                self._mode = 'entity'
                self.get_logger().info(f'Connected to service: {name} (SetEntityState)')
                return
            self.destroy_client(cli)

        for name in self._model_srv_names:
            cli = self.create_client(SetModelState, name)
            if cli.wait_for_service(timeout_sec=0.5):
                self._cli = cli
                self._mode = 'model'
                self.get_logger().info(f'Connected to service: {name} (SetModelState)')
                return
            self.destroy_client(cli)

        self.get_logger().warn('Gazebo set-state service not available yet...')

    def _retry_service(self):
        if self._cli is None:
            self._try_connect_service()

    # ----------- 订阅 /model_states，确认模型存在 -----------
    def _cb_models(self, msg: ModelStates):
        if self.entity_name in msg.name:
            if not self._entity_ready:
                self.get_logger().info(f'Entity [{self.entity_name}] is present in Gazebo.')
            self._entity_ready = True

    # ----------- Odom 回调：推到 Gazebo -----------
    def _cb_odom(self, msg: Odometry):
        if not self._entity_ready or self._cli is None:
            return

        self._cnt += 1
        if self._cnt % max(1, self.decimate) != 0:
            return

        p = msg.pose.pose.position

        # 用线速度估计 yaw，静止时不更新
        v = msg.twist.twist.linear
        speed_xy = math.hypot(v.x, v.y)

        if speed_xy > self.vel_thresh:
            target_yaw = math.atan2(v.y, v.x)
            alpha = 0.1  # 越小转向越慢
            self._yaw = self._yaw + alpha * angle_diff(target_yaw, self._yaw)

        # roll=pitch=0，只保留 yaw
        q = euler_to_quat(0.0, 0.0, self._yaw)

        # 位置 + 抬高，确保不穿地
        z = p.z + self.z_offset
        if z < 0.05:
            z = 0.05

        if self._mode == 'entity':
            st = EntityState()
            st.name = self.entity_name
            st.pose = Pose()
            st.pose.position.x = p.x
            st.pose.position.y = p.y
            st.pose.position.z = z
            st.pose.orientation = q
            st.twist = Twist()  # 纯回放

            req = SetEntityState.Request()
            req.state = st
            self._cli.call_async(req)

        else:
            st = ModelState()
            st.model_name = self.entity_name
            st.pose = Pose()
            st.pose.position.x = p.x
            st.pose.position.y = p.y
            st.pose.position.z = z
            st.pose.orientation = q
            st.twist = Twist()
            st.reference_frame = ''

            req = SetModelState.Request()
            req.model_state = st
            self._cli.call_async(req)


def main():
    rclpy.init()
    rclpy.spin(OdomToGazebo())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
