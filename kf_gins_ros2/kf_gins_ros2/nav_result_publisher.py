import os, math, time
from typing import List, Tuple
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
from tf2_ros import TransformBroadcaster
from .geo import llh_to_ecef, ecef_to_enu, rpy_to_quat

class KFNavResultPublisher(Node):
    def __init__(self):
        super().__init__('kf_nav_result_publisher')
        # Params
        self.declare_parameter('nav_file', os.path.expanduser('~/KF-GINS/result/nav_result.txt'))
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('broadcast_tf', True)
        self.declare_parameter('publish_odom', True)
        self.declare_parameter('publish_path', True)
        self.declare_parameter('time_scale', 1.0)
        self.declare_parameter('loop', False)
        self.declare_parameter('path_publish_rate_hz', 5.0)
        self.declare_parameter('pose_decimation', 10)
        self.declare_parameter('max_path_points', 20000)
        self.declare_parameter('origin_lat', 0.0)
        self.declare_parameter('origin_lon', 0.0)
        self.declare_parameter('origin_h', 0.0)
        self.declare_parameter('use_first_sample_as_origin', True)

        self.nav_file = self.get_parameter('nav_file').value
        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.broadcast_tf = bool(self.get_parameter('broadcast_tf').value)
        self.publish_odom_flag = bool(self.get_parameter('publish_odom').value)
        self.publish_path_flag = bool(self.get_parameter('publish_path').value)
        self.time_scale = float(self.get_parameter('time_scale').value)
        self.loop = bool(self.get_parameter('loop').value)
        self.path_rate = float(self.get_parameter('path_publish_rate_hz').value)
        self.pose_decimation = max(1, int(self.get_parameter('pose_decimation').value))
        self.max_path_points = max(100, int(self.get_parameter('max_path_points').value))

        if not os.path.exists(self.nav_file):
            self.get_logger().error(f'nav_file not found: {self.nav_file}')
            raise FileNotFoundError(self.nav_file)
        self.samples = self._load_nav_result(self.nav_file)
        if len(self.samples) < 2:
            raise RuntimeError('insufficient samples in nav_file')

        use_first = bool(self.get_parameter('use_first_sample_as_origin').value)
        if use_first:
            lat0_deg, lon0_deg, h0 = self.samples[0][2], self.samples[0][3], self.samples[0][4]
        else:
            lat0_deg = self._as_float(self.get_parameter('origin_lat').value, self.samples[0][2])
            lon0_deg = self._as_float(self.get_parameter('origin_lon').value, self.samples[0][3])
            h0       = self._as_float(self.get_parameter('origin_h').value,   self.samples[0][4])
        self.lat0_rad = math.radians(lat0_deg); self.lon0_rad = math.radians(lon0_deg)
        self.x0, self.y0, self.z0 = llh_to_ecef(self.lat0_rad, self.lon0_rad, h0)

        self.odom_pub = self.create_publisher(Odometry, 'kf_gins/odom', 10)
        self.path_pub = self.create_publisher(Path, 'kf_gins/path', 10)
        self.pose_pub = self.create_publisher(PoseStamped, 'kf_gins/pose', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self._publish_static_map_to_odom()

        self.start_wall = time.time(); self.start_sow = self.samples[0][1]; self.idx = 0
        self.path_msg = Path(); self.path_msg.header.frame_id = self.map_frame
        self._dec_counter = 0

        self.timer = self.create_timer(0.002, self._tick)
        if self.publish_path_flag and self.path_rate > 0:
            self.path_timer = self.create_timer(1.0/self.path_rate, self._publish_path)

        self.get_logger().info(f'Loaded {len(self.samples)} samples from {self.nav_file}')
        self.get_logger().info(f'ENU origin lat={lat0_deg:.8f}, lon={lon0_deg:.8f}, h={h0:.3f}; decimation={self.pose_decimation}, max_path_points={self.max_path_points}')

    def _as_float(self, v, default=None):
        try: return float(v)
        except (TypeError, ValueError): return default

    def _load_nav_result(self, path: str) -> List[Tuple[float,...]]:
        out = []
        with open(path, 'r') as f:
            for line in f:
                s = line.strip()
                if not s or s.startswith('#'): continue
                p = s.replace(',', ' ').split()
                if len(p) < 11: continue
                try:
                    week = float(p[0]); sow = float(p[1])
                    lat_deg = float(p[2]); lon_deg = float(p[3]); h = float(p[4])
                    vN = float(p[5]); vE = float(p[6]); vD = float(p[7])
                    roll_deg = float(p[8]); pitch_deg = float(p[9]); yaw_deg = float(p[10])
                    out.append((week, sow, lat_deg, lon_deg, h, vN, vE, vD, roll_deg, pitch_deg, yaw_deg))
                except ValueError:
                    continue
        return out

    def _publish_static_map_to_odom(self):
        t = TransformStamped(); t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.map_frame; t.child_frame_id = self.odom_frame
        t.transform.translation.x = 0.0; t.transform.translation.y = 0.0; t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

    def _tick(self):
        now = time.time(); sim_elapsed = (now - self.start_wall) * max(self.time_scale, 1e-6)
        target_sow = self.start_sow + sim_elapsed
        while self.idx < len(self.samples) and self.samples[self.idx][1] <= target_sow:
            self._publish_sample(self.samples[self.idx]); self.idx += 1
        if self.idx >= len(self.samples):
            if bool(self.get_parameter('loop').value):
                self.start_wall = time.time(); self.start_sow = self.samples[0][1]; self.idx = 0
                self.path_msg = Path(); self.path_msg.header.frame_id = self.map_frame
            else:
                self.timer.cancel()

    def _publish_sample(self, s):
        _, sow, lat_deg, lon_deg, h, vN, vE, vD, roll_deg, pitch_deg, yaw_deg = s
        lat_rad = math.radians(lat_deg); lon_rad = math.radians(lon_deg)
        x, y, z = llh_to_ecef(lat_rad, lon_rad, h)
        e, n, u = ecef_to_enu(x, y, z, self.lat0_rad, self.lon0_rad, self.x0, self.y0, self.z0)
        roll = math.radians(roll_deg); pitch = math.radians(pitch_deg); yaw = math.radians(yaw_deg)
        qx, qy, qz, qw = rpy_to_quat(roll, pitch, yaw)
        stamp = self.get_clock().now().to_msg()

        if self.broadcast_tf:
            t = TransformStamped(); t.header.stamp = stamp
            t.header.frame_id = self.map_frame; t.child_frame_id = self.base_frame
            t.transform.translation.x = e; t.transform.translation.y = n; t.transform.translation.z = u
            t.transform.rotation.x = qx; t.transform.rotation.y = qy; t.transform.rotation.z = qz; t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)

        if self.publish_odom_flag:
            odom = Odometry(); odom.header.stamp = stamp
            odom.header.frame_id = self.map_frame; odom.child_frame_id = self.base_frame
            odom.pose.pose.position.x = e; odom.pose.pose.position.y = n; odom.pose.pose.position.z = u
            odom.pose.pose.orientation.x = qx; odom.pose.pose.orientation.y = qy; odom.pose.pose.orientation.z = qz; odom.pose.pose.orientation.w = qw
            odom.twist.twist.linear.x = vE; odom.twist.twist.linear.y = vN; odom.twist.twist.linear.z = -vD
            self.odom_pub.publish(odom)

        if self.publish_path_flag:
            self._dec_counter += 1
            if self._dec_counter >= self.pose_decimation:
                self._dec_counter = 0
                pose = PoseStamped(); pose.header.stamp = stamp; pose.header.frame_id = self.map_frame
                pose.pose.position.x = e; pose.pose.position.y = n; pose.pose.position.z = u
                pose.pose.orientation.x = qx; pose.pose.orientation.y = qy; pose.pose.orientation.z = qz; pose.pose.orientation.w = qw
                self.pose_pub.publish(pose)
                self.path_msg.header.stamp = stamp
                self.path_msg.poses.append(pose)
                if len(self.path_msg.poses) > self.max_path_points:
                    self.path_msg.poses = self.path_msg.poses[-self.max_path_points:]

    def _publish_path(self):
        self.path_pub.publish(self.path_msg)

def main(args=None):
    rclpy.init(args=args)
    node = KFNavResultPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node(); rclpy.shutdown()
