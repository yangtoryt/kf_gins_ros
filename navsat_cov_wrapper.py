# navsat_cov_wrapper.py（关键片段）
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

class Wrapper(Node):
    def __init__(self):
        super().__init__('navsat_cov_wrapper')
        self.sub = self.create_subscription(
            NavSatFix, '/gps/fix', self.cb, 10)
        self.pub = self.create_publisher(
            NavSatFix, '/gps/fix_cov', 10)

    def cb(self, msg: NavSatFix):
        out = NavSatFix()
        # ★★ 关键：完全保留原 header（stamp/frame_id）
        out.header = msg.header

        out.status = msg.status
        out.latitude  = msg.latitude
        out.longitude = msg.longitude
        out.altitude  = msg.altitude

        # 你想加/改的协方差
        out.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        out.position_covariance[0] = 1.0   # σ_e^2
        out.position_covariance[4] = 1.0   # σ_n^2
        out.position_covariance[8] = 4.0   # σ_u^2

        self.pub.publish(out)

def main():
    rclpy.init()
    node = Wrapper()

    # ★★ 关键：使用仿真时钟（可从命令行传参，也可在代码里声明/参数服务端设定）
    node.declare_parameter('use_sim_time', True)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
