import os, time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
class KFDatasetPlayer(Node):
    def __init__(self):
        super().__init__('kf_dataset_player')
        self.declare_parameter('imu_file', os.path.expanduser('~/KF-GINS/dataset/Leador-A15.txt'))
        self.declare_parameter('gnss_file', os.path.expanduser('~/KF-GINS/dataset/gnss.txt'))
        self.declare_parameter('rate_scale', 1.0)
        self.declare_parameter('loop', False)
        self.imu_file = self.get_parameter('imu_file').value
        self.gnss_file = self.get_parameter('gnss_file').value
        self.rate_scale = float(self.get_parameter('rate_scale').value)
        self.loop = bool(self.get_parameter('loop').value)
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 50)
        self.gps_pub = self.create_publisher(NavSatFix, 'gps/fix_cov', 10)
        self.imu_data = self._load_imu(self.imu_file) if os.path.exists(self.imu_file) else []
        self.gps_data = self._load_gnss(self.gnss_file) if os.path.exists(self.gnss_file) else []
        self.start_wall = time.time()
        self.start_sow = min(self.imu_data[0][0] if self.imu_data else 0.0, self.gps_data[0][0] if self.gps_data else 0.0)
        self.i_imu = 0; self.i_gps = 0
        self.timer = self.create_timer(0.002, self._tick)
        self.get_logger().info(f'Loaded IMU samples: {len(self.imu_data)}, GNSS samples: {len(self.gps_data)}')
    def _load_imu(self, path):
        out = []
        with open(path, 'r') as f:
            for line in f:
                s = line.strip()
                if not s or s.startswith('#'): continue
                p = s.replace(',', ' ').split()
                if len(p) < 7: continue
                try:
                    sow = float(p[0])
                    dthx, dthy, dthz = float(p[1]), float(p[2]), float(p[3])
                    dvx, dvy, dvz = float(p[4]), float(p[5]), float(p[6])
                    out.append((sow, dthx, dthy, dthz, dvx, dvy, dvz))
                except ValueError:
                    continue
        return out
    def _load_gnss(self, path):
        out = []
        with open(path, 'r') as f:
            for line in f:
                s = line.strip()
                if not s or s.startswith('#'): continue
                p = s.replace(',', ' ').split()
                if len(p) < 7: continue
                try:
                    sow = float(p[0])
                    lat, lon, h = float(p[1]), float(p[2]), float(p[3])
                    stdN, stdE, stdD = float(p[4]), float(p[5]), float(p[6])
                    out.append((sow, lat, lon, h, stdN, stdE, stdD))
                except ValueError:
                    continue
        return out
    def _tick(self):
        now = time.time(); sim_elapsed = (now - self.start_wall) * max(self.rate_scale, 1e-6)
        target_sow = self.start_sow + sim_elapsed
        while self.i_imu + 1 < len(self.imu_data) and self.imu_data[self.i_imu][0] <= target_sow:
            sow0, dthx, dthy, dthz, dvx, dvy, dvz = self.imu_data[self.i_imu]
            sow1 = self.imu_data[self.i_imu + 1][0]
            dt = max(sow1 - sow0, 1e-3)
            imu = Imu(); imu.orientation_covariance[0] = -1.0
            imu.angular_velocity.x = dthx / dt; imu.angular_velocity.y = dthy / dt; imu.angular_velocity.z = dthz / dt
            imu.linear_acceleration.x = dvx / dt; imu.linear_acceleration.y = dvy / dt; imu.linear_acceleration.z = dvz / dt
            imu.header.stamp = self.get_clock().now().to_msg(); imu.header.frame_id = 'imu_link'
            self.imu_pub.publish(imu); self.i_imu += 1
        while self.i_gps < len(self.gps_data) and self.gps_data[self.i_gps][0] <= target_sow:
            sow, lat, lon, h, stdN, stdE, stdD = self.gps_data[self.i_gps]
            fix = NavSatFix(); fix.header.stamp = self.get_clock().now().to_msg(); fix.header.frame_id = 'gps_antenna'
            fix.status.status = NavSatFix.STATUS_FIX; fix.status.service = NavSatFix.SERVICE_GPS
            fix.latitude = lat; fix.longitude = lon; fix.altitude = h
            fix.position_covariance = [stdE*stdE, 0.0, 0.0, 0.0, 0.0, stdN*stdN, 0.0, 0.0, 0.0, stdD*stdD]
            fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
            self.gps_pub.publish(fix); self.i_gps += 1
        if self.i_imu >= len(self.imu_data) and self.i_gps >= len(self.gps_data):
            if self.loop: self.start_wall = time.time(); self.i_imu = 0; self.i_gps = 0
            else: self.timer.cancel()
def main(args=None):
    rclpy.init(args=args)
    node = KFDatasetPlayer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node(); rclpy.shutdown()
