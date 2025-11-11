# kf_gins_ros2/kf_gins_ros2/error_monitor.py
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

def quat_to_euler(qx, qy, qz, qw):
    sinr_cosp = 2.0 * (qw*qx + qy*qz)
    cosr_cosp = 1.0 - 2.0 * (qx*qx + qy*qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (qw*qy - qz*qx)
    pitch = math.copysign(math.pi/2.0, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    siny_cosp = 2.0 * (qw*qz + qx*qy)
    cosy_cosp = 1.0 - 2.0 * (qy*qy + qz*qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw

def ang_diff_deg(a_deg, b_deg):
    d = a_deg - b_deg
    while d > 180.0: d -= 360.0
    while d < -180.0: d += 360.0
    return d

class ErrorMonitor(Node):
    def __init__(self):
        super().__init__('kf_gins_error_monitor')

        # 可被 launch 用 -p 覆盖；否则走默认
        odom_param  = self.declare_parameter('odom_topic',  '/kf_gins/odom').value
        truth_param = self.declare_parameter('truth_topic', '/truth/odom').value

        # 两种 QoS：传感器型 / 系统默认
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=50
        )
        default_qos = QoSProfile(depth=50)

        # **双订阅兜底**：既订 reliable 的 /kf_gins/odom_reliable，也订原始 /kf_gins/odom
        self.est = None
        self.truth = None
        self.sub_est_rel = self.create_subscription(
            Odometry, '/kf_gins/odom_reliable', self.cb_est, default_qos
        )
        self.sub_est_raw = self.create_subscription(
            Odometry, odom_param, self.cb_est, sensor_qos
        )
        self.sub_truth = self.create_subscription(
            Odometry, truth_param, self.cb_truth, default_qos
        )

        self.pub_pos_x   = self.create_publisher(Float64, '/kf_gins/error/pos/x',   10)
        self.pub_pos_y   = self.create_publisher(Float64, '/kf_gins/error/pos/y',   10)
        self.pub_pos_z   = self.create_publisher(Float64, '/kf_gins/error/pos/z',   10)
        self.pub_pos_n   = self.create_publisher(Float64, '/kf_gins/error/pos/norm',10)
        self.pub_vel_x   = self.create_publisher(Float64, '/kf_gins/error/vel/x',   10)
        self.pub_vel_y   = self.create_publisher(Float64, '/kf_gins/error/vel/y',   10)
        self.pub_vel_z   = self.create_publisher(Float64, '/kf_gins/error/vel/z',   10)
        self.pub_vel_n   = self.create_publisher(Float64, '/kf_gins/error/vel/norm',10)
        self.pub_att_yaw = self.create_publisher(Float64, '/kf_gins/error/att/yaw_deg',   10)
        self.pub_att_roll= self.create_publisher(Float64, '/kf_gins/error/att/roll_deg',  10)
        self.pub_att_pitch=self.create_publisher(Float64, '/kf_gins/error/att/pitch_deg', 10)

        self.get_logger().info(
            f'Listening odom on {{"/kf_gins/odom_reliable" (reliable), "{odom_param}" (sensor_qos)}} '
            f'and truth on "{truth_param}"'
        )

        # 每2秒提示一次是否已收到两路数据，帮助定位
        self.recv_est = 0
        self.recv_truth = 0
        self.create_timer(2.0, self._diag)

    def _diag(self):
        self.get_logger().info(
            f'received est msgs: {self.recv_est}, truth msgs: {self.recv_truth}'
        )

    def cb_est(self, msg):
        self.est = msg
        self.recv_est += 1
        self.compute()

    def cb_truth(self, msg):
        self.truth = msg
        self.recv_truth += 1
        self.compute()

    def compute(self):
        if self.est is None or self.truth is None:
            return

        ex, ey, ez = self.est.pose.pose.position.x, self.est.pose.pose.position.y, self.est.pose.pose.position.z
        tx, ty, tz = self.truth.pose.pose.position.x, self.truth.pose.pose.position.y, self.truth.pose.pose.position.z
        dx, dy, dz = ex - tx, ey - ty, ez - tz
        n = (dx*dx + dy*dy + dz*dz) ** 0.5
        for pub, val in [(self.pub_pos_x, dx),(self.pub_pos_y, dy),(self.pub_pos_z, dz),(self.pub_pos_n, n)]:
            m = Float64(); m.data = float(val); pub.publish(m)

        ev = self.est.twist.twist.linear; tv = self.truth.twist.twist.linear
        dvx, dvy, dvz = ev.x - tv.x, ev.y - tv.y, ev.z - tv.z
        vn = (dvx*dvx + dvy*dvy + dvz*dvz) ** 0.5
        for pub, val in [(self.pub_vel_x, dvx),(self.pub_vel_y, dvy),(self.pub_vel_z, dvz),(self.pub_vel_n, vn)]:
            m = Float64(); m.data = float(val); pub.publish(m)

        q1 = self.est.pose.pose.orientation; q2 = self.truth.pose.pose.orientation
        r1,p1,y1 = quat_to_euler(q1.x,q1.y,q1.z,q1.w)
        r2,p2,y2 = quat_to_euler(q2.x,q2.y,q2.z,q2.w)
        yaw_d   = ang_diff_deg(math.degrees(y1), math.degrees(y2))
        roll_d  = ang_diff_deg(math.degrees(r1), math.degrees(r2))
        pitch_d = ang_diff_deg(math.degrees(p1), math.degrees(p2))
        for pub, val in [(self.pub_att_yaw, yaw_d),(self.pub_att_roll, roll_d),(self.pub_att_pitch, pitch_d)]:
            m = Float64(); m.data = float(val); pub.publish(m)

def main():
    rclpy.init()
    node = ErrorMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
