#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <chrono>

#include "kf_gins_ros2_native/kf_core_interface.hpp"
#include "kf_gins_ros2_native/geo.hpp"
#include "kf_gins_ros2_native/node_factory.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace kf_gins_ros2_native
{

class KFGinsNativeNode : public rclcpp::Node
{
public:
  KFGinsNativeNode() : Node("kf_gins_native")
  {
    // ---- parameters ----
    config_path_  = this->declare_parameter<std::string>("config_file", "");
    map_frame_    = this->declare_parameter<std::string>("map_frame", "map");
    base_frame_   = this->declare_parameter<std::string>("base_frame", "base_link");
    odom_frame_   = this->declare_parameter<std::string>("odom_frame", "odom");
    imu_topic_    = this->declare_parameter<std::string>("imu_topic", "/imu/data");
    gnss_topic_   = this->declare_parameter<std::string>("gnss_topic", "/gps/fix_cov");
    imu_is_delta_ = this->declare_parameter<bool>("imu_is_delta", false);

    path_rate_hz_    = this->declare_parameter<double>("path_publish_rate_hz", 5.0);
    pose_decimation_ = this->declare_parameter<int>("pose_decimation", 10);
    max_path_pts_    = this->declare_parameter<int>("max_path_points", 20000);
    use_gnss_llh_for_pose_ = this->declare_parameter<bool>("use_gnss_llh_for_pose", true);

    // 连贯相关参数（launch 可覆盖）
    v_limit_mps_ = this->declare_parameter<double>("v_limit_mps", 120.0); // 这里不再用于丢弃，只保留以兼容
    min_dist_m_  = this->declare_parameter<double>("min_dist_m", 0.20);   // 距离采样阈值

    max_jump_m_  = this->declare_parameter<double>("max_jump_m", 200.0);    // Path 点之间最大允许跳变
    align_gate_m_ = this->declare_parameter<double>("align_gate_m", 300.0); // 核心LLH与GNSS对齐门限

    // 追加参数（可被 launch 覆盖）
    start_after_gnss_sec_ = this->declare_parameter<double>("start_after_gnss_sec", 1.0); // 收到 GNSS 后先等一会再画
    publish_after_sec_    = this->declare_parameter<double>("publish_after_sec",    2.0); // 节点启动后整体预热
    reset_gate_m_         = this->declare_parameter<double>("reset_gate_m",        20.0); // 发生大跳变时，清空 Path 重画


    // ---- core ----
    core_ = kfcore::create_kf_core();
    if (!core_) throw std::runtime_error("no core");
    if (!config_path_.empty())
      if (!core_->configure(config_path_))
        throw std::runtime_error("configure failed");

    // ---- pubs/subs ----
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("kf_gins/odom", 10);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("kf_gins/path", 10);
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("kf_gins/pose", 10);
    path_msg_.header.frame_id = map_frame_;

    if (path_rate_hz_ > 0.0) {
      const int period_ms = static_cast<int>(1000.0 / std::max(1.0, path_rate_hz_));
      path_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(period_ms),
        [this]() {
          if (!have_origin_ || path_msg_.poses.empty()) return;
          path_msg_.header.stamp = this->now();
          path_pub_->publish(path_msg_);
        });
    }

    auto imu_qos = rclcpp::SensorDataQoS().keep_last(200);
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, imu_qos, std::bind(&KFGinsNativeNode::imuCb, this, _1));

    auto gnss_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
    gnss_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      gnss_topic_, gnss_qos, std::bind(&KFGinsNativeNode::gnssCb, this, _1));

    tf_broadcaster_     = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // map -> odom (identity)
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now();
    t.header.frame_id = map_frame_;
    t.child_frame_id  = odom_frame_;
    t.transform.rotation.w = 1.0;
    static_broadcaster_->sendTransform(t);

    node_start_time_ = now();

  }

private:
  // ------------------------- Callbacks -------------------------
  void imuCb(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    const double t = rclcpp::Time(msg->header.stamp).seconds();
    if (!have_prev_imu_) {
      prev_imu_time_ = t;
      have_prev_imu_ = true;
      return;
    }
    const double dt = std::max(1e-3, t - prev_imu_time_);
    prev_imu_time_  = t;

    Eigen::Vector3d dtheta, dvel;
    if (imu_is_delta_) {
      dtheta = {msg->angular_velocity.x,   msg->angular_velocity.y,   msg->angular_velocity.z};
      dvel   = {msg->linear_acceleration.x,msg->linear_acceleration.y,msg->linear_acceleration.z};
    } else {
      dtheta = Eigen::Vector3d(msg->angular_velocity.x,
                               msg->angular_velocity.y,
                               msg->angular_velocity.z) * dt;
      dvel   = Eigen::Vector3d(msg->linear_acceleration.x,
                               msg->linear_acceleration.y,
                               msg->linear_acceleration.z) * dt;
    }

    core_->ingestImu(t, dtheta, dvel, dt, imu_is_delta_);
    publishState();
  }

  void gnssCb(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    const double t = rclcpp::Time(msg->header.stamp).seconds();

    // 第一次用 GNSS 设置 ENU 原点
    if (!have_origin_) {
      double ox, oy, oz;
      geo::llh_to_ecef(msg->latitude * M_PI/180.0,
                       msg->longitude * M_PI/180.0,
                       msg->altitude, ox, oy, oz);
      origin_ecef_ = Eigen::Vector3d(ox, oy, oz);
      origin_lat_  = msg->latitude  * M_PI/180.0;
      origin_lon_  = msg->longitude * M_PI/180.0;
      have_origin_ = true;
      RCLCPP_INFO(this->get_logger(),
                  "ENU origin set by GNSS: lat=%.8f lon=%.8f h=%.3f",
                  msg->latitude, msg->longitude, msg->altitude);
    }

    Eigen::Vector3d std_ned(-1,-1,-1);
    if (msg->position_covariance_type == sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN) {
      const double std_e = std::sqrt(std::max(0.0, msg->position_covariance[0]));
      const double std_n = std::sqrt(std::max(0.0, msg->position_covariance[4]));
      const double std_u = std::sqrt(std::max(0.0, msg->position_covariance[8]));
      std_ned = {std_n, std_e, std_u};
    }

    core_->ingestGnss(t, msg->latitude, msg->longitude, msg->altitude, std_ned);

    last_gnss_lat_rad_ = msg->latitude  * M_PI/180.0;
    last_gnss_lon_rad_ = msg->longitude * M_PI/180.0;
    last_gnss_h_m_     = msg->altitude;
    last_gnss_valid_   = true;

    if (!have_first_gnss_stamp_) {
      first_gnss_stamp_ = now();
      have_first_gnss_stamp_ = true;
    }


    publishState();
  }

  // ------------------------- Publish -------------------------
  void publishState()
  {
    if (!have_origin_) return;
    // 预热：节点整体先等 publish_after_sec_ 秒；且收到 GNSS 后再等 start_after_gnss_sec_ 秒
    if (!allow_paint_) {
      const bool warmup_ok = (now() - node_start_time_).seconds() >= publish_after_sec_;
      const bool gnss_ok   = have_first_gnss_stamp_ &&
                            (now() - first_gnss_stamp_).seconds() >= start_after_gnss_sec_;
      if (warmup_ok && gnss_ok) allow_paint_ = true;
      else return; // 预热没过，不画
    }


    const auto st = core_->current();

    auto isfinite_d = [](double v){ return std::isfinite(v); };
    if (!isfinite_d(st.lat_deg) || !isfinite_d(st.lon_deg) || !isfinite_d(st.h_m) ||
        !isfinite_d(st.roll_deg) || !isfinite_d(st.pitch_deg) || !isfinite_d(st.yaw_deg)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "State NaN/Inf (lat=%.3f lon=%.3f h=%.3f rpy=%.3f/%.3f/%.3f)",
        st.lat_deg, st.lon_deg, st.h_m, st.roll_deg, st.pitch_deg, st.yaw_deg);
      return;
    }

    // 1) 姿态：明确用“度→弧度”
    const double roll_rad  = st.roll_deg  * M_PI/180.0;
    const double pitch_rad = st.pitch_deg * M_PI/180.0;
    const double yaw_rad   = st.yaw_deg   * M_PI/180.0;

    // 2) 位置：启动阶段即便 use_gnss_llh_for_pose_==false，也优先用 GNSS，直到核心LLH和GNSS对齐
    double lat_rad, lon_rad, h_m;

    if (last_gnss_valid_) {
      const double core_lat_rad = st.lat_deg * M_PI/180.0;
      const double core_lon_rad = st.lon_deg * M_PI/180.0;

      // 计算核心与GNSS的 ENU 差距，判断是否“对齐”
      double core_x, core_y, core_z;
      geo::llh_to_ecef(core_lat_rad, core_lon_rad, st.h_m, core_x, core_y, core_z);
      Eigen::Vector3d enu_core = geo::ecef_to_enu({core_x, core_y, core_z},
                                                  origin_ecef_, origin_lat_, origin_lon_);
      double gnss_x, gnss_y, gnss_z;
      geo::llh_to_ecef(last_gnss_lat_rad_, last_gnss_lon_rad_, last_gnss_h_m_, gnss_x, gnss_y, gnss_z);
      Eigen::Vector3d enu_gnss = geo::ecef_to_enu({gnss_x, gnss_y, gnss_z},
                                                  origin_ecef_, origin_lat_, origin_lon_);

      const double diff_m = (enu_core - enu_gnss).norm();
      if (!core_llh_aligned_ && std::isfinite(diff_m) && diff_m < align_gate_m_)
        core_llh_aligned_ = true;

      // 选择用于可视化的位置
      if (use_gnss_llh_for_pose_ || !core_llh_aligned_) {
        lat_rad = last_gnss_lat_rad_;
        lon_rad = last_gnss_lon_rad_;
        h_m     = last_gnss_h_m_;
      } else {
        lat_rad = core_lat_rad;
        lon_rad = core_lon_rad;
        h_m     = st.h_m;
      }
    } else {
      // 还没GNSS就别画（have_origin_也会阻止，但这里再兜一层）
      return;
    }


    // LLH->ECEF->ENU
    double x_ecef, y_ecef, z_ecef;
    geo::llh_to_ecef(lat_rad, lon_rad, h_m, x_ecef, y_ecef, z_ecef);
    Eigen::Vector3d enu = geo::ecef_to_enu({x_ecef, y_ecef, z_ecef},
                                           origin_ecef_, origin_lat_, origin_lon_);
    if (!isfinite_d(enu.x()) || !isfinite_d(enu.y()) || !isfinite_d(enu.z())) return;

    // 轻度平滑（只做可视化，不参与滤波）
    Eigen::Vector3d enu_vis = enu;
    const double alpha = 0.3;                     // 0~1，小=更平滑
    if (have_last_enu_) enu_vis = alpha * enu + (1.0 - alpha) * last_enu_;
    last_enu_ = enu_vis;
    have_last_enu_ = true;

    tf2::Quaternion q_tf;
    q_tf.setRPY(roll_rad, pitch_rad, yaw_rad);
    if (!isfinite_d(q_tf.x()) || !isfinite_d(q_tf.y()) ||
        !isfinite_d(q_tf.z()) || !isfinite_d(q_tf.w())) return;

    geometry_msgs::msg::Quaternion q_msg;
    q_msg.x = q_tf.x(); q_msg.y = q_tf.y(); q_msg.z = q_tf.z(); q_msg.w = q_tf.w();

    // 当前时间戳
    auto stamp = now();

    // ---------------- TF / Odom ----------------
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = map_frame_;
    tf.child_frame_id  = base_frame_;
    tf.transform.translation.x = enu_vis.x();
    tf.transform.translation.y = enu_vis.y();
    tf.transform.translation.z = enu_vis.z();
    tf.transform.rotation = q_msg;
    tf_broadcaster_->sendTransform(tf);

    nav_msgs::msg::Odometry od;
    od.header.stamp = stamp;
    od.header.frame_id = map_frame_;
    od.child_frame_id  = base_frame_;
    od.pose.pose.position.x = enu_vis.x();
    od.pose.pose.position.y = enu_vis.y();
    od.pose.pose.position.z = enu_vis.z();
    od.pose.pose.orientation = q_msg;
    // GIEngine: [vN, vE, vD] -> ENU: x=E, y=N, z=U
    od.twist.twist.linear.x = st.vE;
    od.twist.twist.linear.y = st.vN;
    od.twist.twist.linear.z = -st.vD;
    odom_pub_->publish(od);

    // ---------------- Path 追加（只按距离与抽稀，不丢弃） ----------------
    if (++dec_ >= pose_decimation_) {
      dec_ = 0;
        // —— 新增：如果和上一个点距离太远，直接丢弃，防止“竖线/天线”
      if (have_last_path_) {
        const double jump = (enu_vis - last_path_enu_).norm();
        if (std::isfinite(jump) && jump > max_jump_m_) {
          // 清空 Path，像“断线重连”一样从此点开始
          path_msg_.poses.clear();
          have_last_path_ = false;
          // 可选：如果跳得太离谱，再加一道距离门把此帧也略过
          if (jump > reset_gate_m_) return;
        }
      }


      const bool far_enough =
        !have_last_path_ || (enu_vis - last_path_enu_).norm() > std::max(0.0, min_dist_m_);

      if (far_enough) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header = od.header;
        ps.pose   = od.pose.pose;

        path_msg_.header.stamp = stamp;
        path_msg_.header.frame_id = map_frame_;
        path_msg_.poses.push_back(ps);

        if ((int)path_msg_.poses.size() > max_path_pts_) {
          path_msg_.poses.erase(
            path_msg_.poses.begin(),
            path_msg_.poses.begin() + (path_msg_.poses.size() - max_path_pts_));
        }

        path_pub_->publish(path_msg_);

        // 更新“上一次真正写入 Path 的基准”
        last_path_enu_      = enu_vis;
        have_last_path_     = true;
        last_path_stamp_    = stamp;
        have_last_path_stamp_ = true;
      }
    }
  }

  void publishPath()
  {
    path_msg_.header.frame_id = map_frame_;
    path_pub_->publish(path_msg_);
  }

private:
  // ---- params / topics ----
  std::string config_path_, map_frame_, base_frame_, odom_frame_, imu_topic_, gnss_topic_;
  bool imu_is_delta_{false};
  double path_rate_hz_{5.0};
  int    pose_decimation_{10};
  int    max_path_pts_{20000};
  bool   use_gnss_llh_for_pose_{true};
  double max_jump_m_{200.0};
  double align_gate_m_{300.0};
  bool   core_llh_aligned_{false};  // 核心的LLH是否已与最近GNSS对齐

  // 预热/启画控制
  double start_after_gnss_sec_{1.0};
  double publish_after_sec_{2.0};
  double reset_gate_m_{20.0};
  rclcpp::Time node_start_time_;
  bool allow_paint_{false};
  rclcpp::Time first_gnss_stamp_;
  bool have_first_gnss_stamp_{false};



  // ---- core ----
  std::unique_ptr<kfcore::KFCore> core_;

  // ---- frames origin ----
  bool have_origin_{false};
  Eigen::Vector3d origin_ecef_{0,0,0};
  double origin_lat_{0.0}, origin_lon_{0.0};

  // ---- pubs/subs ----
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr      path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_sub_;
  rclcpp::TimerBase::SharedPtr path_timer_;

  // ---- messages ----
  nav_msgs::msg::Path path_msg_;

  // ---- IMU dt ----
  bool   have_prev_imu_{false};
  double prev_imu_time_{0.0};

  // ---- smoothing (for visualization only) ----
  Eigen::Vector3d last_enu_{0,0,0};
  bool have_last_enu_{false};

  // ---- GNSS cache for pose ----
  bool   last_gnss_valid_{false};
  double last_gnss_lat_rad_{0.0}, last_gnss_lon_rad_{0.0}, last_gnss_h_m_{0.0};

  // ---- 连贯轨迹相关（仅作追加基准） ----
  rclcpp::Time   last_path_stamp_;
  bool           have_last_path_stamp_{false};
  Eigen::Vector3d last_path_enu_{0,0,0};
  bool            have_last_path_{false};

  // 参数（兼容保留）
  double v_limit_mps_{120.0};
  double min_dist_m_{0.20};

  // decimation 计数器
  int dec_{0};
};

std::shared_ptr<rclcpp::Node> make_node()
{
  return std::make_shared<KFGinsNativeNode>();
}

} // namespace kf_gins_ros2_native
