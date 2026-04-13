#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <px4_msgs/msg/vehicle_imu.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <limits>

#include "kf_gins_ros2_native/kf_core_interface.hpp"
#include "adapter.hpp"
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
    imu_source_   = this->declare_parameter<std::string>("imu_source", "mavros_raw");
    imu_topic_    = this->declare_parameter<std::string>("imu_topic", "/mavros/imu/data_raw");
    px4_sensor_combined_topic_ =
      this->declare_parameter<std::string>("px4_sensor_combined_topic", "/fmu/out/sensor_combined");
    px4_vehicle_imu_topic_ =
      this->declare_parameter<std::string>("px4_vehicle_imu_topic", "/fmu/out/vehicle_imu");
    gnss_topic_   = this->declare_parameter<std::string>("gnss_topic", "/gps/fix");
    imu_is_delta_ = this->declare_parameter<bool>("imu_is_delta", false);
    imu_input_is_flu_ = this->declare_parameter<bool>("imu_input_is_flu", true);
    use_source_dt_for_px4_imu_ =
      this->declare_parameter<bool>("use_source_dt_for_px4_imu", true);
    imu_gap_warn_ms_ = this->declare_parameter<double>("imu_gap_warn_ms", 60.0);
    delta_imu_source_gap_bridge_enable_ =
      this->declare_parameter<bool>("delta_imu_source_gap_bridge_enable", true);
    delta_imu_source_gap_bridge_min_sec_ =
      this->declare_parameter<double>("delta_imu_source_gap_bridge_min_sec", 0.05);
    delta_imu_source_gap_bridge_min_ratio_ =
      this->declare_parameter<double>("delta_imu_source_gap_bridge_min_ratio", 5.0);
    delta_imu_source_gap_bridge_max_sec_ =
      this->declare_parameter<double>("delta_imu_source_gap_bridge_max_sec", 0.30);
    delta_imu_source_gap_reset_sec_ =
      this->declare_parameter<double>("delta_imu_source_gap_reset_sec", 1.0);
    source_gap_clamp_enable_ =
      this->declare_parameter<bool>("source_gap_clamp_enable", true);
    source_gap_clamp_min_sec_ =
      this->declare_parameter<double>("source_gap_clamp_min_sec", 0.05);
    source_gap_clamp_min_ratio_ =
      this->declare_parameter<double>("source_gap_clamp_min_ratio", 5.0);
    source_gap_clamp_recv_dt_max_sec_ =
      this->declare_parameter<double>("source_gap_clamp_recv_dt_max_sec", 0.03);
    sensor_combined_source_gap_diag_enable_ =
      this->declare_parameter<bool>("sensor_combined_source_gap_diag_enable", true);
    sensor_combined_source_gap_diag_min_sec_ =
      this->declare_parameter<double>("sensor_combined_source_gap_diag_min_sec", 0.05);
    sensor_combined_source_gap_diag_min_ratio_ =
      this->declare_parameter<double>("sensor_combined_source_gap_diag_min_ratio", 5.0);
    sensor_combined_source_gap_reset_sec_ =
      this->declare_parameter<double>("sensor_combined_source_gap_reset_sec", 0.50);

    // 时间与鲁棒性：MAVROS 可能发生时间跳变；用 node clock（可为 /clock）能更稳定
    use_node_time_for_core_ = this->declare_parameter<bool>("use_node_time_for_core", true);
    // 以 IMU dt 积分出 core 时间轴：避免外部时间戳跳变/重复/回退导致发散
    use_integrated_time_for_core_ = this->declare_parameter<bool>("use_integrated_time_for_core", true);
    // 用稳态时钟计算 IMU dt，避免上游时间戳跳变/重复导致 dt=0 或时间回退
    use_steady_time_for_imu_dt_ = this->declare_parameter<bool>("use_steady_time_for_imu_dt", true);
    // 强制 core 的时间单调递增（当上游时间回退时，自动修正）
    force_monotonic_time_for_core_ = this->declare_parameter<bool>("force_monotonic_time_for_core", true);
    min_imu_dt_sec_ = this->declare_parameter<double>("min_imu_dt_sec", 1e-3);
    // 【v4.0修复】仿真环境IMU dt可能突变，从0.20改为0.50秒，以容忍偶发的大跳变
    max_imu_dt_sec_ = this->declare_parameter<double>("max_imu_dt_sec", 0.50);
    imu_dt_estimate_sec_ = this->declare_parameter<double>("imu_dt_estimate_sec", 0.01);
    // 当检测到时间回退/大跳变时，自动重置时间基与 core（避免“乱飞/蜘蛛网”）
    auto_reset_on_time_jump_ = this->declare_parameter<bool>("auto_reset_on_time_jump", true);
    time_jump_reset_threshold_sec_ = this->declare_parameter<double>("time_jump_reset_threshold_sec", 0.5);
    gnss_time_offset_sec_ = this->declare_parameter<double>("gnss_time_offset_sec", 0.001);

    path_rate_hz_    = this->declare_parameter<double>("path_publish_rate_hz", 5.0);
    pose_decimation_ = this->declare_parameter<int>("pose_decimation", 10);
    max_path_pts_    = this->declare_parameter<int>("max_path_points", 20000);
    use_gnss_llh_for_pose_ = this->declare_parameter<bool>("use_gnss_llh_for_pose", true);

    // 连贯相关参数（launch 可覆盖）
    v_limit_mps_ = this->declare_parameter<double>("v_limit_mps", 120.0); // 这里不再用于丢弃，只保留以兼容
    min_dist_m_  = this->declare_parameter<double>("min_dist_m", 0.20);   // 距离采样阈值

    max_jump_m_  = this->declare_parameter<double>("max_jump_m", 200.0);    // Path 点之间最大允许跳变
    align_gate_m_ = this->declare_parameter<double>("align_gate_m", 300.0); // 核心LLH与GNSS对齐门限

    // GNSS std：MAVROS 的 NavSatFix 常常不提供 COVARIANCE_TYPE_DIAGONAL_KNOWN
    // 若 std 为负/为 0，会导致核心数值崩坏（协方差非正定/NaN）。
    use_sim_gnss_std_ = this->declare_parameter<bool>("use_sim_gnss_std", true);  // 【修复】仿真模式标志
    gnss_default_std_h_m_ = this->declare_parameter<double>("gnss_default_std_h_m", 5.0);  // N/E default (实机)
    gnss_default_std_u_m_ = this->declare_parameter<double>("gnss_default_std_u_m", 10.0); // U default (实机)
    sim_gnss_std_h_m_ = this->declare_parameter<double>("sim_gnss_std_h_m", 0.1);  // 【修复】仿真 N/E std
    sim_gnss_std_u_m_ = this->declare_parameter<double>("sim_gnss_std_u_m", 0.2);  // 【修复】仿真 U std
    gnss_min_std_m_       = this->declare_parameter<double>("gnss_min_std_m", 0.5);        // clamp floor

    // 追加参数（可被 launch 覆盖）
    start_after_gnss_sec_ = this->declare_parameter<double>("start_after_gnss_sec", 1.0); // 收到 GNSS 后先等一会再画
    publish_after_sec_    = this->declare_parameter<double>("publish_after_sec",    2.0); // 节点启动后整体预热
    reset_gate_m_         = this->declare_parameter<double>("reset_gate_m",        20.0); // 发生大跳变时，清空 Path 重画
    // 当 KF-GINS 数值发散到 NaN/Inf 时，自动 reset（用最近 GNSS fix 重新初始化）
    auto_reset_on_invalid_state_ = this->declare_parameter<bool>("auto_reset_on_invalid_state", true);
    invalid_state_reset_cooldown_sec_ = this->declare_parameter<double>("invalid_state_reset_cooldown_sec", 5.0);

    // 可视化 gating：默认不影响行为（bringup/对比可按需打开）
    mavros_state_topic_ = this->declare_parameter<std::string>("mavros_state_topic", "/mavros/state");
    mavros_local_velocity_topic_ = this->declare_parameter<std::string>(
      "mavros_local_velocity_topic", "/mavros/local_position/velocity_local");
    path_require_armed_ = this->declare_parameter<bool>("path_require_armed", false);
    clear_path_on_arm_transition_ = this->declare_parameter<bool>("clear_path_on_arm_transition", false);
    use_gnss_llh_for_pose_when_disarmed_ =
      this->declare_parameter<bool>("use_gnss_llh_for_pose_when_disarmed", true);

    // 【修复】零速度更新 (ZUPT) - 无人机未启动时使用 GNSS 复位以强约束速度
    use_zero_velocity_update_when_disarmed_ = this->declare_parameter<bool>("use_zero_velocity_update_when_disarmed", true);
    zupt_reset_interval_sec_ = this->declare_parameter<double>("zupt_reset_interval_sec", 1.0);
    zupt_std_mps_ = this->declare_parameter<double>("zupt_std_mps", 0.05);  // 预留：未来可接入真 ZUPT 观测
    disarmed_yaw_lock_enable_ = this->declare_parameter<bool>("disarmed_yaw_lock_enable", true);
    disarmed_yaw_lock_interval_sec_ = this->declare_parameter<double>("disarmed_yaw_lock_interval_sec", 1.0);
    disarmed_yaw_lock_max_yaw_err_deg_ = this->declare_parameter<double>("disarmed_yaw_lock_max_yaw_err_deg", 5.0);
    force_zero_antlever_ = this->declare_parameter<bool>("force_zero_antlever", true);
    enable_gnss_velocity_update_ = this->declare_parameter<bool>("enable_gnss_velocity_update", false);
    gnss_vel_std_floor_h_mps_ = this->declare_parameter<double>("gnss_vel_std_floor_h_mps", 0.5);
    gnss_vel_std_floor_u_mps_ = this->declare_parameter<double>("gnss_vel_std_floor_u_mps", 1.0);
    use_online_reset_covariance_ = this->declare_parameter<bool>("use_online_reset_covariance", true);
    reset_pos_std_m_ = this->declare_parameter<double>("reset_pos_std_m", 5.0);
    reset_vel_std_mps_ = this->declare_parameter<double>("reset_vel_std_mps", 5.0);
    reset_roll_pitch_std_deg_ = this->declare_parameter<double>("reset_roll_pitch_std_deg", 5.0);
    reset_yaw_std_deg_ = this->declare_parameter<double>("reset_yaw_std_deg", 10.0);
    enable_heading_update_ = this->declare_parameter<bool>("enable_heading_update", true);
    heading_update_std_deg_ = this->declare_parameter<double>("heading_update_std_deg", 3.0);
    heading_update_max_rate_hz_ = this->declare_parameter<double>("heading_update_max_rate_hz", 10.0);
    heading_update_max_age_sec_ = this->declare_parameter<double>("heading_update_max_age_sec", 0.5);
    heading_update_innovation_gate_deg_ =
      this->declare_parameter<double>("heading_update_innovation_gate_deg", 20.0);
    heading_update_armed_innovation_gate_deg_ =
      this->declare_parameter<double>("heading_update_armed_innovation_gate_deg", 8.0);
    heading_update_turn_innovation_gate_deg_ =
      this->declare_parameter<double>("heading_update_turn_innovation_gate_deg", 60.0);
    heading_update_max_source_yaw_rate_deg_s_ =
      this->declare_parameter<double>("heading_update_max_source_yaw_rate_deg_s", 15.0);
    heading_update_source_jump_gate_deg_ =
      this->declare_parameter<double>("heading_update_source_jump_gate_deg", 20.0);
    heading_update_source_jump_block_sec_ =
      this->declare_parameter<double>("heading_update_source_jump_block_sec", 2.0);
    heading_update_hard_innovation_gate_deg_ =
      this->declare_parameter<double>("heading_update_hard_innovation_gate_deg", 15.0);
    heading_post_turn_reacquire_enable_ =
      this->declare_parameter<bool>("heading_post_turn_reacquire_enable", true);
    heading_post_turn_reacquire_window_sec_ =
      this->declare_parameter<double>("heading_post_turn_reacquire_window_sec", 2.0);
    heading_post_turn_reacquire_max_residual_deg_ =
      this->declare_parameter<double>("heading_post_turn_reacquire_max_residual_deg", 45.0);
    heading_post_turn_reacquire_std_deg_ =
      this->declare_parameter<double>("heading_post_turn_reacquire_std_deg", 2.0);
    heading_post_turn_reacquire_hold_sec_ =
      this->declare_parameter<double>("heading_post_turn_reacquire_hold_sec", 4.0);
    heading_post_turn_reacquire_max_rate_hz_ =
      this->declare_parameter<double>("heading_post_turn_reacquire_max_rate_hz", 5.0);
    heading_post_turn_force_relock_enable_ =
      this->declare_parameter<bool>("heading_post_turn_force_relock_enable", true);
    heading_post_turn_force_relock_min_residual_deg_ =
      this->declare_parameter<double>("heading_post_turn_force_relock_min_residual_deg", 20.0);
    heading_post_turn_force_relock_min_consecutive_blocks_ =
      this->declare_parameter<int>("heading_post_turn_force_relock_min_consecutive_blocks", 3);
    heading_post_turn_force_relock_yaw_std_deg_ =
      this->declare_parameter<double>("heading_post_turn_force_relock_yaw_std_deg", 5.0);
    heading_post_turn_force_relock_max_rate_hz_ =
      this->declare_parameter<double>("heading_post_turn_force_relock_max_rate_hz", 1.0);
    heading_post_turn_hold_force_relock_enable_ =
      this->declare_parameter<bool>("heading_post_turn_hold_force_relock_enable", true);
    heading_post_turn_hold_force_relock_min_residual_deg_ =
      this->declare_parameter<double>("heading_post_turn_hold_force_relock_min_residual_deg", 13.0);
    heading_post_turn_hold_force_relock_min_consecutive_blocks_ =
      this->declare_parameter<int>("heading_post_turn_hold_force_relock_min_consecutive_blocks", 20);
    heading_armed_cruise_force_relock_enable_ =
      this->declare_parameter<bool>("heading_armed_cruise_force_relock_enable", true);
    heading_armed_cruise_force_relock_min_residual_deg_ =
      this->declare_parameter<double>("heading_armed_cruise_force_relock_min_residual_deg", 13.0);
    heading_armed_cruise_force_relock_min_consecutive_blocks_ =
      this->declare_parameter<int>("heading_armed_cruise_force_relock_min_consecutive_blocks", 30);
    heading_armed_cruise_force_relock_min_horizontal_speed_mps_ =
      this->declare_parameter<double>("heading_armed_cruise_force_relock_min_horizontal_speed_mps", 4.0);
    heading_armed_cruise_force_relock_max_vertical_speed_mps_ =
      this->declare_parameter<double>("heading_armed_cruise_force_relock_max_vertical_speed_mps", 1.0);
    heading_armed_cruise_force_relock_max_gyro_deg_s_ =
      this->declare_parameter<double>("heading_armed_cruise_force_relock_max_gyro_deg_s", 2.0);
    heading_armed_cruise_force_relock_max_source_yaw_rate_deg_s_ =
      this->declare_parameter<double>("heading_armed_cruise_force_relock_max_source_yaw_rate_deg_s", 2.0);
    heading_armed_cruise_force_relock_yaw_std_deg_ =
      this->declare_parameter<double>("heading_armed_cruise_force_relock_yaw_std_deg", 5.0);
    heading_armed_cruise_force_relock_max_rate_hz_ =
      this->declare_parameter<double>("heading_armed_cruise_force_relock_max_rate_hz", 1.0);
    heading_recovery_enable_ =
      this->declare_parameter<bool>("heading_recovery_enable", true);
    heading_recovery_min_residual_deg_ =
      this->declare_parameter<double>("heading_recovery_min_residual_deg", 15.0);
    heading_recovery_max_residual_deg_ =
      this->declare_parameter<double>("heading_recovery_max_residual_deg", 80.0);
    heading_recovery_max_step_deg_ =
      this->declare_parameter<double>("heading_recovery_max_step_deg", 3.0);
    heading_recovery_std_deg_ =
      this->declare_parameter<double>("heading_recovery_std_deg", 10.0);
    heading_recovery_max_rate_hz_ =
      this->declare_parameter<double>("heading_recovery_max_rate_hz", 2.0);
    heading_recovery_max_gyro_deg_s_ =
      this->declare_parameter<double>("heading_recovery_max_gyro_deg_s", 2.0);
    heading_recovery_max_source_yaw_rate_deg_s_ =
      this->declare_parameter<double>("heading_recovery_max_source_yaw_rate_deg_s", 2.0);
    heading_recovery_max_horizontal_speed_mps_ =
      this->declare_parameter<double>("heading_recovery_max_horizontal_speed_mps", 0.5);
    heading_recovery_max_vertical_speed_mps_ =
      this->declare_parameter<double>("heading_recovery_max_vertical_speed_mps", 2.0);
    heading_recovery_min_consecutive_skips_ =
      this->declare_parameter<int>("heading_recovery_min_consecutive_skips", 2);
    heading_update_low_speed_thresh_mps_ =
      this->declare_parameter<double>("heading_update_low_speed_thresh_mps", 2.0);
    heading_update_armed_min_horizontal_speed_mps_ =
      this->declare_parameter<double>("heading_update_armed_min_horizontal_speed_mps", 0.0);
    heading_update_max_vertical_speed_mps_ =
      this->declare_parameter<double>("heading_update_max_vertical_speed_mps", 0.8);
    heading_update_when_armed_ = this->declare_parameter<bool>("heading_update_when_armed", false);
    heading_update_armed_max_speed_thresh_mps_ =
      this->declare_parameter<double>("heading_update_armed_max_speed_thresh_mps", 6.0);
    heading_update_when_disarmed_ = this->declare_parameter<bool>("heading_update_when_disarmed", true);
    heading_update_when_armed_low_speed_ =
      this->declare_parameter<bool>("heading_update_when_armed_low_speed", true);
    heading_update_when_gnss_no_fix_ =
      this->declare_parameter<bool>("heading_update_when_gnss_no_fix", true);
    skip_medium_imu_gap_when_turning_ =
      this->declare_parameter<bool>("skip_medium_imu_gap_when_turning", true);
    imu_gap_turn_rate_gate_deg_s_ =
      this->declare_parameter<double>("imu_gap_turn_rate_gate_deg_s", 10.0);
    imu_gap_vertical_speed_gate_mps_ =
      this->declare_parameter<double>("imu_gap_vertical_speed_gate_mps", 1.0);
    imu_gap_accel_deviation_gate_mps2_ =
      this->declare_parameter<double>("imu_gap_accel_deviation_gate_mps2", 2.0);
    imu_gap_maneuver_cooldown_sec_ =
      this->declare_parameter<double>("imu_gap_maneuver_cooldown_sec", 1.0);
    skip_large_imu_gap_samples_ =
      this->declare_parameter<bool>("skip_large_imu_gap_samples", true);
    severe_imu_gap_reset_sec_ =
      this->declare_parameter<double>("severe_imu_gap_reset_sec", 0.25);
    origin_rebuild_required_samples_ =
      this->declare_parameter<int>("origin_rebuild_required_samples", 3);
    origin_rebuild_gate_m_ =
      this->declare_parameter<double>("origin_rebuild_gate_m", 3.0);
    publish_max_core_gnss_diff_m_ =
      this->declare_parameter<double>("publish_max_core_gnss_diff_m", 5.0);
    preserved_core_yaw_max_mavros_diff_deg_ =
      this->declare_parameter<double>("preserved_core_yaw_max_mavros_diff_deg", 15.0);
    preserved_core_yaw_max_core_gnss_diff_m_ =
      this->declare_parameter<double>("preserved_core_yaw_max_core_gnss_diff_m", 5.0);

    // ---- core ----
    kfcore::AdapterConfig core_cfg;
    core_cfg.imu_is_delta = imu_is_delta_;
    core_cfg.force_zero_antlever = force_zero_antlever_;
    core_cfg.use_online_reset_covariance = use_online_reset_covariance_;
    core_cfg.reset_pos_std_m = reset_pos_std_m_;
    core_cfg.reset_vel_std_mps = reset_vel_std_mps_;
    core_cfg.reset_roll_pitch_std_deg = reset_roll_pitch_std_deg_;
    core_cfg.reset_yaw_std_deg = reset_yaw_std_deg_;
    core_ = kfcore::create_kf_core_adapter(core_cfg);
    if (!core_) throw std::runtime_error("no core");
    if (!config_path_.empty())
      if (!core_->configure(config_path_))
        throw std::runtime_error("configure failed");

    // ---- pubs/subs ----
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("kf_gins/odom", 10);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("kf_gins/path", 10);
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("kf_gins/pose", 10);
    reset_event_pub_ = this->create_publisher<std_msgs::msg::UInt32>("kf_gins/reset_event", 10);
    path_msg_.header.frame_id = map_frame_;
    path_msg_.header.stamp = now();
    path_pub_->publish(path_msg_);  // 清空 RViz 残留 Path（空 Path 覆盖）

    if (path_rate_hz_ > 0.0) {
      const int period_ms = static_cast<int>(1000.0 / std::max(1.0, path_rate_hz_));
      path_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(period_ms),
        [this]() {
          if (path_require_armed_ && !mavros_armed_) return;
          if (!have_origin_ || path_msg_.poses.empty()) return;
          path_msg_.header.stamp = this->now();
          path_pub_->publish(path_msg_);
        });
    }

    auto imu_qos = rclcpp::SensorDataQoS().keep_last(200);
    if (imu_source_ == "mavros_raw") {
      active_imu_topic_name_ = imu_topic_;
      imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, imu_qos, std::bind(&KFGinsNativeNode::imuCb, this, _1));
    } else if (imu_source_ == "px4_sensor_combined") {
      active_imu_topic_name_ = px4_sensor_combined_topic_;
      if (imu_is_delta_) {
        RCLCPP_WARN(
          get_logger(),
          "imu_source=px4_sensor_combined publishes averaged rates/accelerations, forcing imu_is_delta=false semantics at runtime.");
      }
      px4_sensor_combined_sub_ = this->create_subscription<px4_msgs::msg::SensorCombined>(
        px4_sensor_combined_topic_, imu_qos,
        std::bind(&KFGinsNativeNode::px4SensorCombinedCb, this, _1));
    } else if (imu_source_ == "px4_vehicle_imu") {
      active_imu_topic_name_ = px4_vehicle_imu_topic_;
      px4_vehicle_imu_sub_ = this->create_subscription<px4_msgs::msg::VehicleImu>(
        px4_vehicle_imu_topic_, imu_qos,
        std::bind(&KFGinsNativeNode::px4VehicleImuCb, this, _1));
    } else {
      throw std::runtime_error("Unsupported imu_source: " + imu_source_);
    }

    auto gnss_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
    gnss_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      gnss_topic_, gnss_qos, std::bind(&KFGinsNativeNode::gnssCb, this, _1));

    // MAVROS /state 在当前链路中与其它节点更兼容的 QoS 是 BEST_EFFORT。
    // 之前这里用 RELIABLE 时，kf_gins_node 很可能完全收不到 armed 翻转，
    // 导致飞行阶段仍执行 disarmed 逻辑（ZUPT / GNSS pose fallback）。
    auto state_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
    mavros_state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
      mavros_state_topic_, state_qos, std::bind(&KFGinsNativeNode::mavrosStateCb, this, _1));

    mavros_local_velocity_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      mavros_local_velocity_topic_, state_qos,
      [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        const auto & v = msg->twist.linear;
        const double speed_mps = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
        if (!std::isfinite(speed_mps)) return;
        last_mavros_speed_mps_ = speed_mps;
        last_mavros_horizontal_speed_mps_ = std::sqrt(v.x * v.x + v.y * v.y);
        last_mavros_vertical_speed_mps_ = std::abs(v.z);
        last_mavros_velocity_rx_time_sec_ = this->now().seconds();
      });

    // 订阅 MAVROS IMU 获取 EKF2 的磁力计融合航向
    // KF-GINS 没有磁力计，需要从 PX4 EKF2 获取初始航向
    auto mavros_imu_qos = rclcpp::SensorDataQoS().keep_last(10);
    mavros_imu_heading_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/mavros/imu/data", mavros_imu_qos,
      [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
        // Skip if no valid orientation
        if (msg->orientation.w == 0.0 && msg->orientation.x == 0.0 &&
            msg->orientation.y == 0.0 && msg->orientation.z == 0.0) return;
        // MAVROS orientation: body FLU → world ENU
        // Extract yaw in ENU convention (0=East, π/2=North)
        tf2::Quaternion q(msg->orientation.x, msg->orientation.y,
                          msg->orientation.z, msg->orientation.w);
        double roll_enu, pitch_enu, yaw_enu;
        tf2::Matrix3x3(q).getRPY(roll_enu, pitch_enu, yaw_enu);
        // Convert ENU yaw to NED yaw: yaw_NED = π/2 - yaw_ENU
        double yaw_ned = M_PI / 2.0 - yaw_enu;
        // Normalize to [-π, π]
        while (yaw_ned > M_PI) yaw_ned -= 2.0 * M_PI;
        while (yaw_ned < -M_PI) yaw_ned += 2.0 * M_PI;
        const double yaw_ned_deg = yaw_ned * 180.0 / M_PI;
        const double now_sec = this->now().seconds();
        if (have_prev_mavros_heading_sample_ &&
            std::isfinite(prev_mavros_heading_sample_time_sec_)) {
          const double yaw_step_deg =
            std::abs(shortestAngleDiffDeg_(yaw_ned_deg, prev_mavros_heading_sample_deg_));
          const double dt = now_sec - prev_mavros_heading_sample_time_sec_;
          if (heading_update_source_jump_gate_deg_ > 0.0 &&
              yaw_step_deg > heading_update_source_jump_gate_deg_) {
            last_large_mavros_heading_jump_time_sec_ = now_sec;
            last_large_mavros_heading_jump_deg_ = yaw_step_deg;
          }
          if (dt > 1e-3 && dt < 1.0) {
            last_mavros_heading_rate_deg_s_ =
              shortestAngleDiffDeg_(yaw_ned_deg, prev_mavros_heading_sample_deg_) / dt;
          }
        }
        prev_mavros_heading_sample_deg_ = yaw_ned_deg;
        prev_mavros_heading_sample_time_sec_ = now_sec;
        have_prev_mavros_heading_sample_ = true;
        mavros_heading_ned_deg_ = yaw_ned_deg;
        have_mavros_heading_ = true;
        last_mavros_heading_rx_time_sec_ = now_sec;
      });

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
    last_core_time_ = -std::numeric_limits<double>::infinity();

    RCLCPP_INFO(
      get_logger(),
      "IMU time settings: source=%s, topic=%s, input_is_flu=%s, use_node_time_for_core=%s, "
      "use_integrated_time_for_core=%s, use_steady_time_for_imu_dt=%s, "
      "max_imu_dt_sec=%.3f, skip_medium_imu_gap_when_turning=%s, imu_gap_turn_rate_gate_deg_s=%.2f, "
      "imu_gap_vertical_speed_gate_mps=%.2f, imu_gap_accel_deviation_gate_mps2=%.2f, "
      "imu_gap_maneuver_cooldown_sec=%.2f, "
      "skip_large_imu_gap_samples=%s, severe_imu_gap_reset_sec=%.3f, "
      "delta_imu_source_gap_bridge=%s (min=%.3f s, ratio=%.2f, max=%.3f s, reset=%.3f s), "
      "source_gap_clamp=%s (min=%.3f s, ratio=%.2f, recv<=%.3f s), "
      "sensor_combined_source_gap_diag=%s (min=%.3f s, ratio=%.2f, reset=%.3f s), "
      "mavros_velocity_topic=%s, heading_horiz_min=%.2f, heading_vert_max=%.2f, "
      "heading_src_yaw_rate_max=%.2f deg/s, publish_max_core_gnss_diff_m=%.2f",
      imu_source_.c_str(),
      active_imu_topic_name_.c_str(),
      imu_input_is_flu_ ? "true" : "false",
      use_node_time_for_core_ ? "true" : "false",
      use_integrated_time_for_core_ ? "true" : "false",
      use_steady_time_for_imu_dt_ ? "true" : "false",
      max_imu_dt_sec_,
      skip_medium_imu_gap_when_turning_ ? "true" : "false",
      imu_gap_turn_rate_gate_deg_s_,
      imu_gap_vertical_speed_gate_mps_,
      imu_gap_accel_deviation_gate_mps2_,
      imu_gap_maneuver_cooldown_sec_,
      skip_large_imu_gap_samples_ ? "true" : "false",
      severe_imu_gap_reset_sec_,
      delta_imu_source_gap_bridge_enable_ ? "true" : "false",
      delta_imu_source_gap_bridge_min_sec_,
      delta_imu_source_gap_bridge_min_ratio_,
      delta_imu_source_gap_bridge_max_sec_,
      delta_imu_source_gap_reset_sec_,
      source_gap_clamp_enable_ ? "true" : "false",
      source_gap_clamp_min_sec_,
      source_gap_clamp_min_ratio_,
      source_gap_clamp_recv_dt_max_sec_,
      sensor_combined_source_gap_diag_enable_ ? "true" : "false",
      sensor_combined_source_gap_diag_min_sec_,
      sensor_combined_source_gap_diag_min_ratio_,
      sensor_combined_source_gap_reset_sec_,
      mavros_local_velocity_topic_.c_str(),
      heading_update_armed_min_horizontal_speed_mps_,
      heading_update_max_vertical_speed_mps_,
      heading_update_max_source_yaw_rate_deg_s_,
      publish_max_core_gnss_diff_m_);

  }

private:
  struct ImuSample
  {
    double source_stamp_sec{std::numeric_limits<double>::quiet_NaN()};
    double sample_dt_sec{std::numeric_limits<double>::quiet_NaN()};
    double angular_dt_sec{std::numeric_limits<double>::quiet_NaN()};
    double linear_dt_sec{std::numeric_limits<double>::quiet_NaN()};
    Eigen::Vector3d angular{Eigen::Vector3d::Zero()};
    Eigen::Vector3d linear{Eigen::Vector3d::Zero()};
    bool data_is_delta{false};
    bool input_is_flu{false};
    bool enable_source_gap_diag{false};
    bool allow_source_gap_clamp{false};
  };

  struct ImuGapInfo
  {
    double input_dt_sec{std::numeric_limits<double>::quiet_NaN()};
    double recv_dt_sec{std::numeric_limits<double>::quiet_NaN()};
  };

  void clearPath_(const char* reason)
  {
    (void)reason;
    path_msg_.poses.clear();
    have_last_path_ = false;
    have_last_enu_ = false;
    dec_ = 0;
    path_msg_.header.stamp = now();
    path_msg_.header.frame_id = map_frame_;
    path_pub_->publish(path_msg_);  // 用空 Path 覆盖，清理 RViz “蜘蛛网”
  }

  void mavrosStateCb(const mavros_msgs::msg::State::SharedPtr msg)
  {
    const bool prev_armed = mavros_armed_;
    mavros_armed_ = msg->armed;

    if (prev_armed != mavros_armed_) {
      last_disarmed_yaw_lock_time_sec_ = std::numeric_limits<double>::quiet_NaN();
      last_heading_attempt_time_sec_ = std::numeric_limits<double>::quiet_NaN();
      last_heading_update_time_sec_ = std::numeric_limits<double>::quiet_NaN();
      last_heading_recovery_time_sec_ = std::numeric_limits<double>::quiet_NaN();
      last_turning_heading_time_sec_ = std::numeric_limits<double>::quiet_NaN();
      last_post_turn_reacquire_time_sec_ = std::numeric_limits<double>::quiet_NaN();
      last_post_turn_reacquire_apply_time_sec_ = std::numeric_limits<double>::quiet_NaN();
      post_turn_hold_end_time_sec_ = std::numeric_limits<double>::quiet_NaN();
      last_post_turn_force_relock_time_sec_ = std::numeric_limits<double>::quiet_NaN();
      heading_large_residual_skip_count_ = 0;
      post_turn_blocked_count_ = 0;
    }

    if (clear_path_on_arm_transition_ && prev_armed != mavros_armed_) {
      clearPath_(mavros_armed_ ? "armed transition" : "disarmed transition");
    }
    if (!prev_armed && mavros_armed_) {
      // 起飞时不再重置 IMU 时间基。
      // IMU 以 ~50Hz 连续发送，arm 前后没有时间间断。
      // 旧代码重置 have_prev_imu_=false 会导致下一次 imuCb 将
      // core_time_sec_ 重置为 0，而 engine 内部 imupre_.time 仍为
      // 旧值(~221s)，造成时间跳变，使 GNSS 更新失效。
      // 现在只清空 Path 可视化残留（如果启用了 clear_path_on_arm_transition_）。
      RCLCPP_INFO(get_logger(), "Armed transition: keeping IMU time base continuous.");
    }
  }

  double rawTimeSecFromMsg_(const builtin_interfaces::msg::Time& stamp) const
  {
    return rclcpp::Time(stamp).seconds();
  }

  double rawTimeSecNow_() const
  {
    return this->now().seconds();
  }

  // 获取来自 MAVROS (EKF2 磁力计融合) 的航向，如果不可用则返回 0
  double getInitialYawDeg_() const {
    return have_mavros_heading_ ? mavros_heading_ned_deg_ : 0.0;
  }

  double selectYawForCoreReset_(const char** source) const
  {
    if (source != nullptr) *source = "zero";

    if (prefer_preserved_yaw_on_next_core_reset_ && have_mavros_heading_) {
      if (source != nullptr) *source = "mavros_after_reset_disable_preserved";
      return mavros_heading_ned_deg_;
    }

    if (prefer_preserved_yaw_on_next_core_reset_ &&
        std::isfinite(last_trusted_core_yaw_deg_)) {
      bool preserved_core_ok = true;
      if (preserved_core_yaw_max_core_gnss_diff_m_ > 0.0 &&
          std::isfinite(last_core_gnss_diff_m_) &&
          last_core_gnss_diff_m_ > preserved_core_yaw_max_core_gnss_diff_m_) {
        preserved_core_ok = false;
      }
      if (preserved_core_yaw_max_mavros_diff_deg_ > 0.0 &&
          have_mavros_heading_) {
        const double yaw_gap_deg =
          std::abs(shortestAngleDiffDeg_(last_trusted_core_yaw_deg_, mavros_heading_ned_deg_));
        if (yaw_gap_deg > preserved_core_yaw_max_mavros_diff_deg_) {
          preserved_core_ok = false;
        }
      }
      if (preserved_core_ok) {
        if (source != nullptr) *source = "preserved_core";
        return last_trusted_core_yaw_deg_;
      }
    }

    if (have_mavros_heading_) {
      if (source != nullptr) {
        *source = prefer_preserved_yaw_on_next_core_reset_ ? "mavros_reject_preserved" : "mavros";
      }
      return mavros_heading_ned_deg_;
    }

    if (std::isfinite(last_trusted_core_yaw_deg_)) {
      if (source != nullptr) *source = "preserved_core_fallback";
      return last_trusted_core_yaw_deg_;
    }

    return 0.0;
  }

  static double normalizeAngleDeg_(double deg)
  {
    while (deg > 180.0) deg -= 360.0;
    while (deg < -180.0) deg += 360.0;
    return deg;
  }

  static double shortestAngleDiffDeg_(double a_deg, double b_deg)
  {
    return normalizeAngleDeg_(a_deg - b_deg);
  }

  ImuGapInfo maybeReportImuGap_(double input_stamp_sec, const rclcpp::Time& steady_now)
  {
    ImuGapInfo info;
    const double warn_gap_sec = std::max(0.001, imu_gap_warn_ms_ * 1e-3);

    double input_dt_sec = std::numeric_limits<double>::quiet_NaN();
    if (std::isfinite(input_stamp_sec) && std::isfinite(last_imu_input_stamp_sec_)) {
      input_dt_sec = input_stamp_sec - last_imu_input_stamp_sec_;
    }
    last_imu_input_stamp_sec_ = input_stamp_sec;
    info.input_dt_sec = input_dt_sec;

    double recv_dt_sec = std::numeric_limits<double>::quiet_NaN();
    if (have_last_imu_recv_steady_time_) {
      recv_dt_sec = (steady_now - last_imu_recv_steady_time_).seconds();
    }
    last_imu_recv_steady_time_ = steady_now;
    have_last_imu_recv_steady_time_ = true;
    info.recv_dt_sec = recv_dt_sec;

    if ((std::isfinite(input_dt_sec) && input_dt_sec > warn_gap_sec) ||
        (std::isfinite(recv_dt_sec) && recv_dt_sec > warn_gap_sec)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Raw IMU gap: input_dt=%.2fms recv_dt=%.2fms topic=%s",
        std::isfinite(input_dt_sec) ? input_dt_sec * 1000.0 : -1.0,
        std::isfinite(recv_dt_sec) ? recv_dt_sec * 1000.0 : -1.0,
        active_imu_topic_name_.c_str());
    }
    return info;
  }

  bool maybeHandleNonDeltaSourceGap_(
    const ImuSample& sample,
    const ImuGapInfo& imu_gap,
    bool source_gap_detected)
  {
    if (!sample.enable_source_gap_diag ||
        !sensor_combined_source_gap_diag_enable_ ||
        !std::isfinite(sample.sample_dt_sec) ||
        sample.sample_dt_sec <= 0.0 ||
        !std::isfinite(imu_gap.input_dt_sec)) {
      return false;
    }

    if (auto_reset_on_time_jump_ &&
        imu_gap.input_dt_sec < -time_jump_reset_threshold_sec_) {
      resetFilterAndTimeBase_("sensor_combined source time went backwards");
      return true;
    }

    if (!(imu_gap.input_dt_sec > 0.0)) {
      return false;
    }

    const double diag_min_sec = std::max(0.0, sensor_combined_source_gap_diag_min_sec_);
    const double diag_min_ratio = std::max(1.0, sensor_combined_source_gap_diag_min_ratio_);
    const double gap_ratio = imu_gap.input_dt_sec / std::max(1e-6, sample.sample_dt_sec);
    const bool should_report =
      imu_gap.input_dt_sec >= diag_min_sec &&
      gap_ratio >= diag_min_ratio;

    if (should_report) {
      ++sensor_combined_source_gap_diag_count_;
      if (mavros_armed_) {
        ++sensor_combined_source_gap_diag_armed_count_;
      }
      sensor_combined_source_gap_max_sec_ =
        std::max(sensor_combined_source_gap_max_sec_, imu_gap.input_dt_sec);
      sensor_combined_source_gap_max_ratio_ =
        std::max(sensor_combined_source_gap_max_ratio_, gap_ratio);

      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "SensorCombined source-gap diag: source_dt=%.2fms sample_dt=%.2fms ratio=%.2f recv_dt=%.2fms "
        "armed=%s count=%d armed_count=%d max=%.2fms max_ratio=%.2f topic=%s",
        imu_gap.input_dt_sec * 1000.0,
        sample.sample_dt_sec * 1000.0,
        gap_ratio,
        std::isfinite(imu_gap.recv_dt_sec) ? imu_gap.recv_dt_sec * 1000.0 : -1.0,
        mavros_armed_ ? "true" : "false",
        sensor_combined_source_gap_diag_count_,
        sensor_combined_source_gap_diag_armed_count_,
        sensor_combined_source_gap_max_sec_ * 1000.0,
        sensor_combined_source_gap_max_ratio_,
        active_imu_topic_name_.c_str());
    }

    if (sensor_combined_source_gap_reset_sec_ > 0.0 &&
        imu_gap.input_dt_sec >= sensor_combined_source_gap_reset_sec_) {
      if (!source_gap_detected) {
        resetFilterAndTimeBase_("sensor_combined source gap too large");
        return true;
      }
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "SensorCombined source-gap reset suppressed (source-gap pattern): source_dt=%.2fms recv_dt=%.2fms",
        imu_gap.input_dt_sec * 1000.0,
        std::isfinite(imu_gap.recv_dt_sec) ? imu_gap.recv_dt_sec * 1000.0 : -1.0);
    }

    return false;
  }

  void resetCoreForDisarmedYawLock_(double yaw_err_deg)
  {
    if (!last_gnss_valid_ || !have_mavros_heading_) return;

    const double yaw_lock_deg = getInitialYawDeg_();
    (void)core_->reset(last_gnss_lat_rad_ * 180.0 / M_PI,
                       last_gnss_lon_rad_ * 180.0 / M_PI,
                       last_gnss_h_m_,
                       yaw_lock_deg);

    core_initialized_ = true;
    core_llh_aligned_ = false;

    // Restart the inertial time base so the next IMU sample anchors the fresh core.
    have_prev_imu_ = false;
    have_raw_time_zero_ = false;
    prev_imu_raw_rel_sec_ = 0.0;
    last_imu_input_stamp_sec_ = std::numeric_limits<double>::quiet_NaN();
    have_last_imu_recv_steady_time_ = false;
    core_time_sec_ = 0.0;
    last_core_time_ = -std::numeric_limits<double>::infinity();
    last_gnss_time_sec_ = -std::numeric_limits<double>::infinity();
    have_prev_gnss_for_vel_ = false;
    last_zupt_reset_time_sec_ = std::numeric_limits<double>::quiet_NaN();
    last_heading_attempt_time_sec_ = std::numeric_limits<double>::quiet_NaN();
    last_heading_update_time_sec_ = std::numeric_limits<double>::quiet_NaN();
    last_heading_recovery_time_sec_ = std::numeric_limits<double>::quiet_NaN();
    last_turning_heading_time_sec_ = std::numeric_limits<double>::quiet_NaN();
    last_post_turn_reacquire_time_sec_ = std::numeric_limits<double>::quiet_NaN();
    last_post_turn_reacquire_apply_time_sec_ = std::numeric_limits<double>::quiet_NaN();
    post_turn_hold_end_time_sec_ = std::numeric_limits<double>::quiet_NaN();
    last_post_turn_force_relock_time_sec_ = std::numeric_limits<double>::quiet_NaN();
    heading_large_residual_skip_count_ = 0;
    post_turn_blocked_count_ = 0;

    publishResetEvent_("disarmed yaw lock");
    clearPath_("disarmed yaw lock");

    RCLCPP_WARN(
      get_logger(),
      "Disarmed yaw lock applied: reset core yaw to MAVROS heading %.2f deg (yaw error %.2f deg)",
      yaw_lock_deg, yaw_err_deg);
  }

  void resetFilterAndTimeBase_(const char* reason)
  {
    RCLCPP_WARN(get_logger(), "Resetting core/time-base: %s", reason);
    publishResetEvent_(reason);
    const bool had_origin = have_origin_;
    if (core_initialized_) {
      const auto st = core_->current();
      if (std::isfinite(st.yaw_deg)) {
        last_trusted_core_yaw_deg_ = normalizeAngleDeg_(st.yaw_deg);
      }
    }
    prefer_preserved_yaw_on_next_core_reset_ =
      mavros_armed_ && std::isfinite(last_trusted_core_yaw_deg_);

    have_prev_imu_ = false;
    have_raw_time_zero_ = false;
    prev_imu_raw_rel_sec_ = 0.0;
    last_imu_input_stamp_sec_ = std::numeric_limits<double>::quiet_NaN();
    have_last_imu_recv_steady_time_ = false;
    core_time_sec_ = 0.0;
    last_core_time_ = -std::numeric_limits<double>::infinity();
    last_gnss_time_sec_ = -std::numeric_limits<double>::infinity();

    // 清空可视化轨迹，避免“蜘蛛网”
    path_msg_.poses.clear();
    have_last_path_ = false;
    have_last_enu_ = false;
    allow_paint_ = false;
    have_first_gnss_stamp_ = false;

    // Preserve the mission map origin across in-flight resets. Rebuilding origin
    // from moving GNSS fixes can swap the height/reference layer and collapse the
    // path to the ground after a severe IMU gap.
    core_initialized_ = false;
    core_llh_aligned_ = false;
    waiting_for_origin_rebuild_ = false;
    have_origin_rebuild_candidate_ = false;
    origin_rebuild_candidate_count_ = 0;
    have_prev_gnss_for_vel_ = false;
    last_heading_attempt_time_sec_ = std::numeric_limits<double>::quiet_NaN();
    last_heading_update_time_sec_ = std::numeric_limits<double>::quiet_NaN();
    last_heading_recovery_time_sec_ = std::numeric_limits<double>::quiet_NaN();
    last_turning_heading_time_sec_ = std::numeric_limits<double>::quiet_NaN();
    last_post_turn_reacquire_time_sec_ = std::numeric_limits<double>::quiet_NaN();
    last_post_turn_reacquire_apply_time_sec_ = std::numeric_limits<double>::quiet_NaN();
    post_turn_hold_end_time_sec_ = std::numeric_limits<double>::quiet_NaN();
    last_post_turn_force_relock_time_sec_ = std::numeric_limits<double>::quiet_NaN();
    heading_large_residual_skip_count_ = 0;
    post_turn_blocked_count_ = 0;

    if (had_origin) {
      RCLCPP_WARN(
        get_logger(),
        "Preserving existing ENU origin across reset; next GNSS fix will only reinitialize core.");
    } else {
      RCLCPP_WARN(
        get_logger(),
        "No ENU origin available yet; next GNSS fix will initialize origin and core.");
    }
  }

  void setOriginFromLlh_(double lat_rad, double lon_rad, double h_m)
  {
    double ox, oy, oz;
    geo::llh_to_ecef(lat_rad, lon_rad, h_m, ox, oy, oz);
    origin_ecef_ = Eigen::Vector3d(ox, oy, oz);
    origin_lat_  = lat_rad;
    origin_lon_  = lon_rad;
    have_origin_ = true;
    waiting_for_origin_rebuild_ = false;
    have_origin_rebuild_candidate_ = false;
    origin_rebuild_candidate_count_ = 0;
  }

  bool tryRebuildOriginFromStableGnss_(double lat_rad, double lon_rad, double h_m)
  {
    const int required_samples = std::max(1, origin_rebuild_required_samples_);
    const double gate_m = std::max(0.0, origin_rebuild_gate_m_);

    if (!have_origin_rebuild_candidate_) {
      origin_rebuild_candidate_lat_rad_ = lat_rad;
      origin_rebuild_candidate_lon_rad_ = lon_rad;
      origin_rebuild_candidate_h_m_ = h_m;
      origin_rebuild_candidate_count_ = 1;
      have_origin_rebuild_candidate_ = true;
      RCLCPP_INFO(
        get_logger(),
        "Origin rebuild candidate started: need %d stable GNSS fixes within %.2f m.",
        required_samples,
        gate_m);
      return false;
    }

    double cx, cy, cz;
    geo::llh_to_ecef(
      origin_rebuild_candidate_lat_rad_,
      origin_rebuild_candidate_lon_rad_,
      origin_rebuild_candidate_h_m_,
      cx, cy, cz);
    double x, y, z;
    geo::llh_to_ecef(lat_rad, lon_rad, h_m, x, y, z);
    const Eigen::Vector3d enu = geo::ecef_to_enu(
      Eigen::Vector3d(x, y, z),
      Eigen::Vector3d(cx, cy, cz),
      origin_rebuild_candidate_lat_rad_,
      origin_rebuild_candidate_lon_rad_);
    const double e = enu.x();
    const double n = enu.y();
    const double u = enu.z();
    const double drift_m = std::sqrt(e * e + n * n + u * u);

    if (drift_m <= gate_m) {
      ++origin_rebuild_candidate_count_;
    } else {
      origin_rebuild_candidate_lat_rad_ = lat_rad;
      origin_rebuild_candidate_lon_rad_ = lon_rad;
      origin_rebuild_candidate_h_m_ = h_m;
      origin_rebuild_candidate_count_ = 1;
      RCLCPP_WARN(
        get_logger(),
        "Origin rebuild candidate reset: GNSS drift %.2f m exceeded gate %.2f m.",
        drift_m,
        gate_m);
      return false;
    }

    if (origin_rebuild_candidate_count_ < required_samples) {
      return false;
    }

    setOriginFromLlh_(lat_rad, lon_rad, h_m);
    RCLCPP_INFO(
      get_logger(),
      "ENU origin rebuilt after reset: lat=%.8f lon=%.8f h=%.3f (%d stable fixes)",
      lat_rad * 180.0 / M_PI,
      lon_rad * 180.0 / M_PI,
      h_m,
      origin_rebuild_candidate_count_);
    return true;
  }

  void publishResetEvent_(const char* reason)
  {
    if (!reset_event_pub_) return;
    std_msgs::msg::UInt32 msg;
    msg.data = ++reset_event_seq_;
    reset_event_pub_->publish(msg);
    RCLCPP_INFO(
      get_logger(),
      "Published IEKF reset event #%u (%s)",
      msg.data,
      reason != nullptr ? reason : "unknown");
  }

  void maybeApplyHeadingUpdate_(double t_sec)
  {
    if (!enable_heading_update_ || !core_initialized_ || !have_mavros_heading_) return;

    const double now_sec = now().seconds();
    if (!std::isfinite(last_mavros_heading_rx_time_sec_) ||
        (now_sec - last_mavros_heading_rx_time_sec_) > std::max(0.05, heading_update_max_age_sec_)) {
      return;
    }

    const double min_period_sec =
      heading_update_max_rate_hz_ > 0.0 ? 1.0 / heading_update_max_rate_hz_ : 0.0;
    if (std::isfinite(last_heading_attempt_time_sec_) &&
        (now_sec - last_heading_attempt_time_sec_) < min_period_sec) {
      return;
    }

    const bool have_fresh_mavros_speed =
      std::isfinite(last_mavros_speed_mps_) &&
      std::isfinite(last_mavros_horizontal_speed_mps_) &&
      std::isfinite(last_mavros_vertical_speed_mps_) &&
      std::isfinite(last_mavros_velocity_rx_time_sec_) &&
      (now_sec - last_mavros_velocity_rx_time_sec_) <= std::max(0.05, heading_update_max_age_sec_);

    const bool heading_horizontal_ok =
      !have_fresh_mavros_speed ||
      heading_update_armed_min_horizontal_speed_mps_ <= 0.0 ||
      last_mavros_horizontal_speed_mps_ >= heading_update_armed_min_horizontal_speed_mps_;
    const bool heading_vertical_ok =
      !have_fresh_mavros_speed ||
      heading_update_max_vertical_speed_mps_ <= 0.0 ||
      last_mavros_vertical_speed_mps_ <= heading_update_max_vertical_speed_mps_;
    const bool armed_motion_ok = heading_horizontal_ok && heading_vertical_ok;

    bool apply_heading = false;
    if (heading_update_when_disarmed_ && !mavros_armed_) {
      apply_heading = true;
    }
    if (heading_update_when_armed_low_speed_ && mavros_armed_ && have_fresh_mavros_speed &&
        last_mavros_speed_mps_ <= std::max(0.0, heading_update_low_speed_thresh_mps_) &&
        armed_motion_ok) {
      apply_heading = true;
    }
    if (heading_update_when_armed_ && mavros_armed_ && have_fresh_mavros_speed && armed_motion_ok) {
      const double max_speed = heading_update_armed_max_speed_thresh_mps_;
      if (max_speed <= 0.0 || last_mavros_speed_mps_ <= max_speed) {
        apply_heading = true;
      }
    }
    if (heading_update_when_gnss_no_fix_ && !last_gnss_has_fix_) {
      apply_heading = true;
    }
    if (!apply_heading && mavros_armed_ && have_fresh_mavros_speed && !armed_motion_ok) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Skip heading update: armed motion gate blocked update (horiz=%.2f m/s, vert=%.2f m/s, min_horiz=%.2f, max_vert=%.2f)",
        last_mavros_horizontal_speed_mps_, last_mavros_vertical_speed_mps_,
        heading_update_armed_min_horizontal_speed_mps_, heading_update_max_vertical_speed_mps_);
    }
    if (!apply_heading) return;

    const auto st = core_->current();
    if (!std::isfinite(st.yaw_deg)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Skip heading update: invalid current yaw");
      return;
    }

    const double core_yaw_before_deg = st.yaw_deg;
    const double raw_heading_deg = mavros_heading_ned_deg_;
    const double yaw_residual_deg =
      shortestAngleDiffDeg_(raw_heading_deg, core_yaw_before_deg);
    if (mavros_armed_ &&
        std::isfinite(last_large_mavros_heading_jump_time_sec_) &&
        heading_update_source_jump_block_sec_ > 0.0 &&
        (now_sec - last_large_mavros_heading_jump_time_sec_) <= heading_update_source_jump_block_sec_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Skip heading update: recent source heading jump %.2f deg within %.2f s cooldown",
        last_large_mavros_heading_jump_deg_, heading_update_source_jump_block_sec_);
      return;
    }
    if (std::isfinite(heading_update_max_source_yaw_rate_deg_s_) &&
        heading_update_max_source_yaw_rate_deg_s_ > 0.0 &&
        std::isfinite(last_mavros_heading_rate_deg_s_) &&
        std::abs(last_mavros_heading_rate_deg_s_) > heading_update_max_source_yaw_rate_deg_s_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Skip heading update: source yaw rate %.2f deg/s exceeds gate %.2f deg/s",
        last_mavros_heading_rate_deg_s_, heading_update_max_source_yaw_rate_deg_s_);
      return;
    }
    last_heading_attempt_time_sec_ = now_sec;
    double innovation_gate_deg =
      (mavros_armed_ &&
       std::isfinite(heading_update_armed_innovation_gate_deg_) &&
       heading_update_armed_innovation_gate_deg_ > 0.0)
        ? heading_update_armed_innovation_gate_deg_
        : heading_update_innovation_gate_deg_;
    const bool turning_now =
      std::isfinite(last_imu_gyro_norm_deg_s_) &&
      std::isfinite(imu_gap_turn_rate_gate_deg_s_) &&
      imu_gap_turn_rate_gate_deg_s_ > 0.0 &&
      last_imu_gyro_norm_deg_s_ >= imu_gap_turn_rate_gate_deg_s_;
    if (turning_now) {
      last_turning_heading_time_sec_ = now_sec;
    }
    const bool recent_turning =
      heading_post_turn_reacquire_enable_ &&
      mavros_armed_ &&
      !turning_now &&
      std::isfinite(last_turning_heading_time_sec_) &&
      heading_post_turn_reacquire_window_sec_ > 0.0 &&
      (now_sec - last_turning_heading_time_sec_) <= heading_post_turn_reacquire_window_sec_;
    const bool post_turn_hold_active =
      heading_post_turn_reacquire_enable_ &&
      mavros_armed_ &&
      std::isfinite(post_turn_hold_end_time_sec_) &&
      heading_post_turn_reacquire_hold_sec_ > 0.0 &&
      now_sec <= post_turn_hold_end_time_sec_;
    const bool post_turn_context = recent_turning || post_turn_hold_active;
    const bool armed_cruise_force_relock_context =
      heading_armed_cruise_force_relock_enable_ &&
      mavros_armed_ &&
      !turning_now &&
      !post_turn_context &&
      have_fresh_mavros_speed &&
      (heading_armed_cruise_force_relock_min_horizontal_speed_mps_ <= 0.0 ||
       last_mavros_horizontal_speed_mps_ >=
         heading_armed_cruise_force_relock_min_horizontal_speed_mps_) &&
      (heading_armed_cruise_force_relock_max_vertical_speed_mps_ <= 0.0 ||
       last_mavros_vertical_speed_mps_ <=
         heading_armed_cruise_force_relock_max_vertical_speed_mps_);
    if (mavros_armed_ && turning_now &&
        std::isfinite(heading_update_turn_innovation_gate_deg_) &&
        heading_update_turn_innovation_gate_deg_ > 0.0) {
      innovation_gate_deg =
        std::max(innovation_gate_deg, heading_update_turn_innovation_gate_deg_);
    } else if (post_turn_context &&
               std::isfinite(heading_update_turn_innovation_gate_deg_) &&
               heading_update_turn_innovation_gate_deg_ > 0.0) {
      innovation_gate_deg =
        std::max(innovation_gate_deg, heading_update_turn_innovation_gate_deg_);
    }
    if (std::isfinite(innovation_gate_deg) &&
        innovation_gate_deg > 0.0 &&
        std::isfinite(heading_update_hard_innovation_gate_deg_) &&
        heading_update_hard_innovation_gate_deg_ > 0.0) {
      innovation_gate_deg = std::min(innovation_gate_deg, heading_update_hard_innovation_gate_deg_);
    }

    const double residual_abs_deg = std::abs(yaw_residual_deg);
    double heading_measurement_deg = raw_heading_deg;
    double heading_measurement_std_deg = heading_update_std_deg_;
    bool recovery_update = false;
    bool post_turn_reacquire_update = false;
    const char* heading_mode = "update";

    if (std::isfinite(innovation_gate_deg) &&
        innovation_gate_deg > 0.0 &&
        residual_abs_deg > innovation_gate_deg) {
      heading_large_residual_skip_count_++;
      if (post_turn_context && !turning_now) {
        post_turn_blocked_count_++;
      } else {
        post_turn_blocked_count_ = 0;
      }
      if (armed_cruise_force_relock_context) {
        armed_cruise_blocked_count_++;
      } else {
        armed_cruise_blocked_count_ = 0;
      }

      const bool recovery_residual_ok =
        heading_recovery_enable_ &&
        residual_abs_deg >= std::max(0.0, heading_recovery_min_residual_deg_) &&
        (heading_recovery_max_residual_deg_ <= 0.0 ||
         residual_abs_deg <= heading_recovery_max_residual_deg_);
      const bool recovery_gyro_ok =
        !std::isfinite(last_imu_gyro_norm_deg_s_) ||
        heading_recovery_max_gyro_deg_s_ <= 0.0 ||
        last_imu_gyro_norm_deg_s_ <= heading_recovery_max_gyro_deg_s_;
      const bool recovery_source_rate_ok =
        !std::isfinite(last_mavros_heading_rate_deg_s_) ||
        heading_recovery_max_source_yaw_rate_deg_s_ <= 0.0 ||
        std::abs(last_mavros_heading_rate_deg_s_) <= heading_recovery_max_source_yaw_rate_deg_s_;
      const bool recovery_speed_ok =
        have_fresh_mavros_speed &&
        (heading_recovery_max_horizontal_speed_mps_ <= 0.0 ||
         last_mavros_horizontal_speed_mps_ <= heading_recovery_max_horizontal_speed_mps_) &&
        (heading_recovery_max_vertical_speed_mps_ <= 0.0 ||
         last_mavros_vertical_speed_mps_ <= heading_recovery_max_vertical_speed_mps_);
      const int min_recovery_skips =
        std::max(1, heading_recovery_min_consecutive_skips_);
      const bool recovery_count_ok =
        heading_large_residual_skip_count_ >= min_recovery_skips;
      const double recovery_min_period_sec =
        heading_recovery_max_rate_hz_ > 0.0 ? 1.0 / heading_recovery_max_rate_hz_ : 0.0;
      const bool recovery_rate_ok =
        !std::isfinite(last_heading_recovery_time_sec_) ||
        (now_sec - last_heading_recovery_time_sec_) >= recovery_min_period_sec;
      const double post_turn_reacquire_min_period_sec =
        heading_post_turn_reacquire_max_rate_hz_ > 0.0
          ? 1.0 / heading_post_turn_reacquire_max_rate_hz_
          : 0.0;
      const bool post_turn_reacquire_rate_ok =
        !std::isfinite(last_post_turn_reacquire_apply_time_sec_) ||
        (now_sec - last_post_turn_reacquire_apply_time_sec_) >=
          post_turn_reacquire_min_period_sec;
      const double post_turn_force_relock_min_period_sec =
        heading_post_turn_force_relock_max_rate_hz_ > 0.0
          ? 1.0 / heading_post_turn_force_relock_max_rate_hz_
          : 0.0;
      const bool post_turn_force_relock_rate_ok =
        !std::isfinite(last_post_turn_force_relock_time_sec_) ||
        (now_sec - last_post_turn_force_relock_time_sec_) >=
          post_turn_force_relock_min_period_sec;
      const double armed_cruise_force_relock_min_period_sec =
        heading_armed_cruise_force_relock_max_rate_hz_ > 0.0
          ? 1.0 / heading_armed_cruise_force_relock_max_rate_hz_
          : 0.0;
      const bool armed_cruise_force_relock_rate_ok =
        !std::isfinite(last_armed_cruise_force_relock_time_sec_) ||
        (now_sec - last_armed_cruise_force_relock_time_sec_) >=
          armed_cruise_force_relock_min_period_sec;

      const bool post_turn_reacquire_residual_ok =
        post_turn_context &&
        residual_abs_deg > innovation_gate_deg &&
        (heading_post_turn_reacquire_max_residual_deg_ <= 0.0 ||
         residual_abs_deg <= heading_post_turn_reacquire_max_residual_deg_);
      const bool post_turn_force_relock_residual_ok =
        heading_post_turn_force_relock_enable_ &&
        post_turn_context &&
        !turning_now &&
        residual_abs_deg >=
          std::max(0.0, heading_post_turn_force_relock_min_residual_deg_);
      const bool post_turn_force_relock_count_ok =
        post_turn_blocked_count_ >=
        std::max(1, heading_post_turn_force_relock_min_consecutive_blocks_);
      const bool post_turn_hold_force_relock_residual_ok =
        heading_post_turn_hold_force_relock_enable_ &&
        post_turn_hold_active &&
        !recent_turning &&
        !turning_now &&
        residual_abs_deg >=
          std::max(innovation_gate_deg,
                   std::max(0.0, heading_post_turn_hold_force_relock_min_residual_deg_));
      const bool post_turn_hold_force_relock_count_ok =
        post_turn_blocked_count_ >=
        std::max(std::max(1, heading_post_turn_force_relock_min_consecutive_blocks_),
                 std::max(1, heading_post_turn_hold_force_relock_min_consecutive_blocks_));
      const bool armed_cruise_force_relock_residual_ok =
        armed_cruise_force_relock_context &&
        residual_abs_deg >=
          std::max(innovation_gate_deg,
                   std::max(0.0, heading_armed_cruise_force_relock_min_residual_deg_));
      const bool armed_cruise_force_relock_count_ok =
        armed_cruise_blocked_count_ >=
        std::max(1, heading_armed_cruise_force_relock_min_consecutive_blocks_);
      const bool armed_cruise_force_relock_gyro_ok =
        !std::isfinite(last_imu_gyro_norm_deg_s_) ||
        heading_armed_cruise_force_relock_max_gyro_deg_s_ <= 0.0 ||
        last_imu_gyro_norm_deg_s_ <= heading_armed_cruise_force_relock_max_gyro_deg_s_;
      const bool armed_cruise_force_relock_source_rate_ok =
        !std::isfinite(last_mavros_heading_rate_deg_s_) ||
        heading_armed_cruise_force_relock_max_source_yaw_rate_deg_s_ <= 0.0 ||
        std::abs(last_mavros_heading_rate_deg_s_) <=
          heading_armed_cruise_force_relock_max_source_yaw_rate_deg_s_;
      const bool generic_recovery_allowed =
        !(mavros_armed_ && post_turn_context && heading_post_turn_reacquire_enable_) &&
        (!mavros_armed_ || !have_fresh_mavros_speed ||
         last_mavros_horizontal_speed_mps_ <= std::max(0.0, heading_update_low_speed_thresh_mps_));
      const bool post_turn_force_relock_due_to_hold_plateau =
        !post_turn_force_relock_residual_ok &&
        post_turn_hold_force_relock_residual_ok &&
        post_turn_hold_force_relock_count_ok;
      const bool post_turn_force_relock_allowed =
        (post_turn_force_relock_residual_ok && post_turn_force_relock_count_ok) ||
        post_turn_force_relock_due_to_hold_plateau;

      if (post_turn_force_relock_allowed &&
          recovery_gyro_ok && recovery_source_rate_ok && recovery_speed_ok &&
          post_turn_force_relock_rate_ok) {
        const double force_relock_std_deg =
          std::max(0.1, std::abs(heading_post_turn_force_relock_yaw_std_deg_));
        if (core_->forceYaw(t_sec, raw_heading_deg, force_relock_std_deg)) {
          const auto st_after = core_->current();
          const double core_yaw_after_deg = st_after.yaw_deg;
          const int post_turn_block_count = post_turn_blocked_count_;
          const char* relock_reason =
            post_turn_force_relock_due_to_hold_plateau ? "hold_plateau" : "large_residual";
          last_heading_recovery_time_sec_ = now_sec;
          last_post_turn_reacquire_time_sec_ = now_sec;
          last_post_turn_reacquire_apply_time_sec_ = now_sec;
          post_turn_hold_end_time_sec_ =
            heading_post_turn_reacquire_hold_sec_ > 0.0
              ? now_sec + heading_post_turn_reacquire_hold_sec_
              : std::numeric_limits<double>::quiet_NaN();
          last_post_turn_force_relock_time_sec_ = now_sec;
          heading_large_residual_skip_count_ = 0;
          post_turn_blocked_count_ = 0;
          last_heading_update_time_sec_ = now_sec;
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "Applied post-turn hard relock: residual=%.2f deg core_yaw_before=%.2f deg "
            "raw_heading=%.2f deg yaw_std=%.2f deg "
            "(gate=%.2f deg, armed=%s, gyro=%.2f deg/s, horiz=%.2f m/s, vert=%.2f m/s, "
            "source_yaw_rate=%.2f deg/s, recent_turning=%s, post_turn_hold=%s, "
            "block_count=%d, reason=%s, core_yaw_after=%.2f deg)",
            yaw_residual_deg, core_yaw_before_deg, raw_heading_deg,
            force_relock_std_deg, innovation_gate_deg,
            mavros_armed_ ? "true" : "false",
            last_imu_gyro_norm_deg_s_,
            last_mavros_horizontal_speed_mps_,
            last_mavros_vertical_speed_mps_,
            last_mavros_heading_rate_deg_s_,
            recent_turning ? "true" : "false",
            post_turn_hold_active ? "true" : "false",
            post_turn_block_count,
            relock_reason,
            core_yaw_after_deg);
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "Applied heading post_turn_force_relock: residual=%.2f deg core_yaw_before=%.2f deg "
            "raw_heading=%.2f deg measurement_heading=%.2f deg std=%.2f deg "
            "(gate=%.2f deg, armed=%s, turning=%s, recent_turning=%s, post_turn_hold=%s, "
            "gyro=%.2f deg/s, speed=%.2f m/s, horiz=%.2f m/s, vert=%.2f m/s, "
            "source_yaw_rate=%.2f deg/s, reason=%s, core_yaw_after=%.2f deg)",
            yaw_residual_deg, core_yaw_before_deg, raw_heading_deg,
            raw_heading_deg, force_relock_std_deg,
            innovation_gate_deg,
            mavros_armed_ ? "true" : "false",
            turning_now ? "true" : "false",
            recent_turning ? "true" : "false",
            post_turn_hold_active ? "true" : "false",
            last_imu_gyro_norm_deg_s_,
            last_mavros_speed_mps_,
            last_mavros_horizontal_speed_mps_,
            last_mavros_vertical_speed_mps_,
            last_mavros_heading_rate_deg_s_,
            relock_reason,
            core_yaw_after_deg);
          return;
        }
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "core_->forceYaw() failed during post-turn hard relock");
      }

      if (armed_cruise_force_relock_residual_ok &&
          armed_cruise_force_relock_count_ok &&
          armed_cruise_force_relock_gyro_ok &&
          armed_cruise_force_relock_source_rate_ok &&
          armed_cruise_force_relock_rate_ok) {
        const double force_relock_std_deg =
          std::max(0.1, std::abs(heading_armed_cruise_force_relock_yaw_std_deg_));
        if (core_->forceYaw(t_sec, raw_heading_deg, force_relock_std_deg)) {
          const auto st_after = core_->current();
          const double core_yaw_after_deg = st_after.yaw_deg;
          const int armed_cruise_block_count = armed_cruise_blocked_count_;
          last_heading_recovery_time_sec_ = now_sec;
          last_armed_cruise_force_relock_time_sec_ = now_sec;
          heading_large_residual_skip_count_ = 0;
          post_turn_blocked_count_ = 0;
          armed_cruise_blocked_count_ = 0;
          last_heading_update_time_sec_ = now_sec;
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "Applied armed-cruise hard relock: residual=%.2f deg core_yaw_before=%.2f deg "
            "raw_heading=%.2f deg yaw_std=%.2f deg "
            "(gate=%.2f deg, armed=%s, gyro=%.2f deg/s, horiz=%.2f m/s, vert=%.2f m/s, "
            "source_yaw_rate=%.2f deg/s, block_count=%d, core_yaw_after=%.2f deg)",
            yaw_residual_deg, core_yaw_before_deg, raw_heading_deg,
            force_relock_std_deg, innovation_gate_deg,
            mavros_armed_ ? "true" : "false",
            last_imu_gyro_norm_deg_s_,
            last_mavros_horizontal_speed_mps_,
            last_mavros_vertical_speed_mps_,
            last_mavros_heading_rate_deg_s_,
            armed_cruise_block_count,
            core_yaw_after_deg);
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "Applied heading armed_cruise_force_relock: residual=%.2f deg core_yaw_before=%.2f deg "
            "raw_heading=%.2f deg measurement_heading=%.2f deg std=%.2f deg "
            "(gate=%.2f deg, armed=%s, turning=%s, recent_turning=%s, post_turn_hold=%s, "
            "gyro=%.2f deg/s, speed=%.2f m/s, horiz=%.2f m/s, vert=%.2f m/s, "
            "source_yaw_rate=%.2f deg/s, block_count=%d, core_yaw_after=%.2f deg)",
            yaw_residual_deg, core_yaw_before_deg, raw_heading_deg,
            raw_heading_deg, force_relock_std_deg,
            innovation_gate_deg,
            mavros_armed_ ? "true" : "false",
            turning_now ? "true" : "false",
            recent_turning ? "true" : "false",
            post_turn_hold_active ? "true" : "false",
            last_imu_gyro_norm_deg_s_,
            last_mavros_speed_mps_,
            last_mavros_horizontal_speed_mps_,
            last_mavros_vertical_speed_mps_,
            last_mavros_heading_rate_deg_s_,
            armed_cruise_block_count,
            core_yaw_after_deg);
          return;
        }
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "core_->forceYaw() failed during armed-cruise hard relock");
      }

      if (post_turn_reacquire_residual_ok && recovery_gyro_ok &&
          recovery_source_rate_ok && recovery_speed_ok && recovery_count_ok &&
          post_turn_reacquire_rate_ok) {
        heading_measurement_deg = raw_heading_deg;
        heading_measurement_std_deg =
          std::max(0.1, std::abs(heading_post_turn_reacquire_std_deg_));
        post_turn_reacquire_update = true;
        heading_mode = "post_turn_reacquire";
      } else if (generic_recovery_allowed &&
          recovery_residual_ok && !turning_now && recovery_gyro_ok &&
          recovery_source_rate_ok && recovery_speed_ok && recovery_count_ok &&
          recovery_rate_ok) {
        const double recovery_step_deg =
          std::copysign(
            std::min(residual_abs_deg, std::max(0.1, heading_recovery_max_step_deg_)),
            yaw_residual_deg);
        heading_measurement_deg = normalizeAngleDeg_(core_yaw_before_deg + recovery_step_deg);
        heading_measurement_std_deg =
          std::max(heading_update_std_deg_, heading_recovery_std_deg_);
        recovery_update = true;
        heading_mode = "recovery";
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Skip heading update: innovation %.2f deg exceeds gate %.2f deg "
          "(core_yaw=%.2f deg, raw_heading=%.2f deg, armed=%s, turning=%s, recent_turning=%s, "
          "post_turn_hold=%s, "
          "gyro=%.2f deg/s, speed=%.2f m/s, horiz=%.2f m/s, vert=%.2f m/s, "
          "source_yaw_rate=%.2f deg/s, mode=%s, count=%d, post_turn_block_count=%d, "
          "armed_cruise_block_count=%d)",
          yaw_residual_deg, innovation_gate_deg,
          core_yaw_before_deg, raw_heading_deg,
          mavros_armed_ ? "true" : "false",
          turning_now ? "true" : "false",
          recent_turning ? "true" : "false",
          post_turn_hold_active ? "true" : "false",
          last_imu_gyro_norm_deg_s_,
          last_mavros_speed_mps_,
          last_mavros_horizontal_speed_mps_,
          last_mavros_vertical_speed_mps_,
          last_mavros_heading_rate_deg_s_,
          post_turn_context ? "post_turn_blocked" :
          (armed_cruise_force_relock_context ? "armed_cruise_blocked" :
           (heading_recovery_enable_ ? "recovery_blocked" : "disabled")),
          heading_large_residual_skip_count_,
          post_turn_blocked_count_,
          armed_cruise_blocked_count_);
        return;
      }
    } else {
      heading_large_residual_skip_count_ = 0;
      post_turn_blocked_count_ = 0;
      armed_cruise_blocked_count_ = 0;
    }

    if (!core_->ingestHeading(t_sec, heading_measurement_deg, heading_measurement_std_deg)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "core_->ingestHeading() failed");
      return;
    }

    const auto st_after = core_->current();
    const double core_yaw_after_deg = st_after.yaw_deg;

    if (post_turn_reacquire_update) {
      last_heading_recovery_time_sec_ = now_sec;
      last_post_turn_reacquire_time_sec_ = now_sec;
      last_post_turn_reacquire_apply_time_sec_ = now_sec;
      if (!post_turn_hold_active && heading_post_turn_reacquire_hold_sec_ > 0.0) {
        post_turn_hold_end_time_sec_ = now_sec + heading_post_turn_reacquire_hold_sec_;
      }
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Applied post-turn heading reacquire: residual=%.2f deg core_yaw_before=%.2f deg "
        "raw_heading=%.2f deg measurement_heading=%.2f deg std=%.2f deg "
        "(gate=%.2f deg, armed=%s, gyro=%.2f deg/s, horiz=%.2f m/s, vert=%.2f m/s, "
        "source_yaw_rate=%.2f deg/s, recent_turning=%s, post_turn_hold=%s, "
        "core_yaw_after=%.2f deg, count=%d)",
        yaw_residual_deg, core_yaw_before_deg, raw_heading_deg,
        heading_measurement_deg, heading_measurement_std_deg,
        innovation_gate_deg,
        mavros_armed_ ? "true" : "false",
        last_imu_gyro_norm_deg_s_,
        last_mavros_horizontal_speed_mps_,
        last_mavros_vertical_speed_mps_,
        last_mavros_heading_rate_deg_s_,
        recent_turning ? "true" : "false",
        post_turn_hold_active ? "true" : "false",
        core_yaw_after_deg,
        heading_large_residual_skip_count_);
    } else if (recovery_update) {
      last_heading_recovery_time_sec_ = now_sec;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Applied heading recovery: residual=%.2f deg core_yaw_before=%.2f deg raw_heading=%.2f deg "
        "measurement_heading=%.2f deg std=%.2f deg "
        "(gate=%.2f deg, step_limit=%.2f deg, armed=%s, gyro=%.2f deg/s, "
        "horiz=%.2f m/s, vert=%.2f m/s, source_yaw_rate=%.2f deg/s, recent_turning=%s, "
        "core_yaw_after=%.2f deg, count=%d)",
        yaw_residual_deg, core_yaw_before_deg, raw_heading_deg,
        heading_measurement_deg, heading_measurement_std_deg,
        innovation_gate_deg, heading_recovery_max_step_deg_,
        mavros_armed_ ? "true" : "false",
        last_imu_gyro_norm_deg_s_,
        last_mavros_horizontal_speed_mps_,
        last_mavros_vertical_speed_mps_,
        last_mavros_heading_rate_deg_s_,
        recent_turning ? "true" : "false",
        core_yaw_after_deg,
        heading_large_residual_skip_count_);
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Applied heading %s: residual=%.2f deg core_yaw_before=%.2f deg raw_heading=%.2f deg "
      "measurement_heading=%.2f deg std=%.2f deg "
      "(gate=%.2f deg, armed=%s, turning=%s, recent_turning=%s, post_turn_hold=%s, gyro=%.2f deg/s, speed=%.2f m/s, "
      "horiz=%.2f m/s, vert=%.2f m/s, source_yaw_rate=%.2f deg/s, core_yaw_after=%.2f deg)",
      heading_mode,
      yaw_residual_deg, core_yaw_before_deg, raw_heading_deg,
      heading_measurement_deg, heading_measurement_std_deg,
      innovation_gate_deg,
      mavros_armed_ ? "true" : "false",
      turning_now ? "true" : "false",
      recent_turning ? "true" : "false",
      post_turn_hold_active ? "true" : "false",
      last_imu_gyro_norm_deg_s_,
      last_mavros_speed_mps_,
      last_mavros_horizontal_speed_mps_,
      last_mavros_vertical_speed_mps_,
      last_mavros_heading_rate_deg_s_,
      core_yaw_after_deg);

    last_heading_update_time_sec_ = now_sec;
  }

  // ------------------------- Callbacks -------------------------
  void imuCb(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    const double raw_t = use_node_time_for_core_ ? rawTimeSecNow_() : rawTimeSecFromMsg_(msg->header.stamp);
    const rclcpp::Time steady_now = steady_clock_.now();
    ImuSample sample;
    sample.source_stamp_sec = rawTimeSecFromMsg_(msg->header.stamp);
    sample.angular = Eigen::Vector3d(
      msg->angular_velocity.x,
      msg->angular_velocity.y,
      msg->angular_velocity.z);
    sample.linear = Eigen::Vector3d(
      msg->linear_acceleration.x,
      msg->linear_acceleration.y,
      msg->linear_acceleration.z);
    sample.data_is_delta = imu_is_delta_;
    sample.input_is_flu = imu_input_is_flu_;
    processImuSample_(sample, raw_t, steady_now);
  }

  void px4SensorCombinedCb(const px4_msgs::msg::SensorCombined::SharedPtr msg)
  {
    const rclcpp::Time steady_now = steady_clock_.now();
    ImuSample sample;
    sample.source_stamp_sec = static_cast<double>(msg->timestamp) * 1e-6;
    const double gyro_dt_sec = static_cast<double>(msg->gyro_integral_dt) * 1e-6;
    const double accel_dt_sec = static_cast<double>(msg->accelerometer_integral_dt) * 1e-6;
    sample.angular_dt_sec = gyro_dt_sec;
    sample.linear_dt_sec = accel_dt_sec;
    if (use_source_dt_for_px4_imu_) {
      if (gyro_dt_sec > 0.0 && accel_dt_sec > 0.0) {
        sample.sample_dt_sec = std::max(gyro_dt_sec, accel_dt_sec);
        if (std::abs(gyro_dt_sec - accel_dt_sec) > 5e-4) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "PX4 SensorCombined dt mismatch: gyro_dt=%.6f s accel_dt=%.6f s. Using %.6f s.",
            gyro_dt_sec, accel_dt_sec, sample.sample_dt_sec);
        }
      } else if (gyro_dt_sec > 0.0) {
        sample.sample_dt_sec = gyro_dt_sec;
      } else if (accel_dt_sec > 0.0) {
        sample.sample_dt_sec = accel_dt_sec;
      }
    }
    sample.angular = Eigen::Vector3d(msg->gyro_rad[0], msg->gyro_rad[1], msg->gyro_rad[2]);
    sample.linear = Eigen::Vector3d(
      msg->accelerometer_m_s2[0],
      msg->accelerometer_m_s2[1],
      msg->accelerometer_m_s2[2]);
    sample.data_is_delta = false;
    sample.input_is_flu = false;  // PX4 DDS SensorCombined is already FRD.
    sample.enable_source_gap_diag = true;
    sample.allow_source_gap_clamp = true;
    processImuSample_(sample, rawTimeSecNow_(), steady_now);
  }

  void px4VehicleImuCb(const px4_msgs::msg::VehicleImu::SharedPtr msg)
  {
    const rclcpp::Time steady_now = steady_clock_.now();
    ImuSample sample;
    sample.source_stamp_sec = static_cast<double>(msg->timestamp_sample) * 1e-6;
    const double delta_angle_dt_sec = static_cast<double>(msg->delta_angle_dt) * 1e-6;
    const double delta_velocity_dt_sec = static_cast<double>(msg->delta_velocity_dt) * 1e-6;
    sample.angular_dt_sec = delta_angle_dt_sec;
    sample.linear_dt_sec = delta_velocity_dt_sec;
    if (use_source_dt_for_px4_imu_) {
      if (delta_angle_dt_sec > 0.0 && delta_velocity_dt_sec > 0.0) {
        sample.sample_dt_sec = std::max(delta_angle_dt_sec, delta_velocity_dt_sec);
        if (std::abs(delta_angle_dt_sec - delta_velocity_dt_sec) > 5e-4) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "PX4 VehicleImu dt mismatch: delta_angle_dt=%.6f s delta_velocity_dt=%.6f s. Using %.6f s.",
            delta_angle_dt_sec, delta_velocity_dt_sec, sample.sample_dt_sec);
        }
      } else if (delta_angle_dt_sec > 0.0) {
        sample.sample_dt_sec = delta_angle_dt_sec;
      } else if (delta_velocity_dt_sec > 0.0) {
        sample.sample_dt_sec = delta_velocity_dt_sec;
      }
    }
    sample.angular = Eigen::Vector3d(
      msg->delta_angle[0], msg->delta_angle[1], msg->delta_angle[2]);
    sample.linear = Eigen::Vector3d(
      msg->delta_velocity[0], msg->delta_velocity[1], msg->delta_velocity[2]);
    sample.data_is_delta = true;
    sample.input_is_flu = false;  // PX4 DDS VehicleImu is already FRD.
    sample.allow_source_gap_clamp = true;
    processImuSample_(sample, rawTimeSecNow_(), steady_now);
  }

  void processImuSample_(const ImuSample& sample, double raw_t, const rclcpp::Time& steady_now)
  {
    const ImuGapInfo imu_gap = maybeReportImuGap_(sample.source_stamp_sec, steady_now);
    const bool source_gap_detected =
      source_gap_clamp_enable_ &&
      sample.allow_source_gap_clamp &&
      std::isfinite(sample.sample_dt_sec) &&
      sample.sample_dt_sec > 0.0 &&
      std::isfinite(imu_gap.input_dt_sec) &&
      std::isfinite(imu_gap.recv_dt_sec) &&
      imu_gap.input_dt_sec >= source_gap_clamp_min_sec_ &&
      (imu_gap.input_dt_sec / std::max(1e-6, sample.sample_dt_sec)) >= source_gap_clamp_min_ratio_ &&
      imu_gap.recv_dt_sec <= source_gap_clamp_recv_dt_max_sec_;

    if (maybeHandleNonDeltaSourceGap_(sample, imu_gap, source_gap_detected)) {
      return;
    }

    if (!have_prev_imu_) {
      prev_imu_time_ = raw_t;
      prev_imu_steady_time_ = steady_now;
      have_prev_imu_ = true;
      have_raw_time_zero_ = false;
      prev_imu_raw_rel_sec_ = 0.0;
      core_time_sec_ = 0.0;
      last_core_time_ = -std::numeric_limits<double>::infinity();
      return;
    }

    // 在线仿真中，core 的有效初值来自第一次 GNSS reset，而不是 YAML 里的离线初值。
    // 在首次 GNSS 之前只建立 IMU 时间基，不推进 mechanization，避免错误初值导致协方差炸裂。
    if (!core_initialized_) {
      const bool use_source_sample_dt = std::isfinite(sample.sample_dt_sec) && sample.sample_dt_sec > 0.0;
      const bool use_raw_time_for_dt = !use_source_sample_dt && !use_steady_time_for_imu_dt_;
      if (use_raw_time_for_dt) {
        if (!have_raw_time_zero_) {
          raw_time_zero_sec_ = prev_imu_time_;
          have_raw_time_zero_ = true;
          prev_imu_raw_rel_sec_ = 0.0;
        }
        const double raw_rel = raw_t - raw_time_zero_sec_;
        prev_imu_raw_rel_sec_ = raw_rel;
      }
      prev_imu_time_ = raw_t;
      prev_imu_steady_time_ = steady_now;
      core_time_sec_ = 0.0;
      last_core_time_ = -std::numeric_limits<double>::infinity();
      return;
    }

    // 1) 计算 raw dt（保证 t 与 dt 使用同一时间基准）
    double dt_candidate = 0.0;
    const bool use_source_sample_dt = std::isfinite(sample.sample_dt_sec) && sample.sample_dt_sec > 0.0;
    const bool use_raw_time_for_dt = !use_source_sample_dt && !use_steady_time_for_imu_dt_;
    if (use_source_sample_dt) {
      dt_candidate = sample.sample_dt_sec;
      prev_imu_time_ = raw_t;
      prev_imu_steady_time_ = steady_now;
    } else if (use_raw_time_for_dt) {
      if (!have_raw_time_zero_) {
        raw_time_zero_sec_ = prev_imu_time_;
        have_raw_time_zero_ = true;
        prev_imu_raw_rel_sec_ = 0.0;
      }
      const double raw_rel = raw_t - raw_time_zero_sec_;
      const double prev_rel = prev_imu_raw_rel_sec_;
      dt_candidate = raw_rel - prev_rel;
      prev_imu_raw_rel_sec_ = raw_rel;
      prev_imu_time_ = raw_t;
    } else {
      dt_candidate = (steady_now - prev_imu_steady_time_).seconds();
      prev_imu_steady_time_ = steady_now;
      prev_imu_time_ = raw_t;
    }

    bool bridge_delta_source_gap_as_rates = false;
    if (sample.data_is_delta && use_source_sample_dt && std::isfinite(imu_gap.input_dt_sec)) {
      if (auto_reset_on_time_jump_ &&
          imu_gap.input_dt_sec < -time_jump_reset_threshold_sec_) {
        resetFilterAndTimeBase_("delta IMU source time went backwards");
        return;
      }

      if (delta_imu_source_gap_bridge_enable_ && imu_gap.input_dt_sec > 0.0) {
        const double min_bridge_sec = std::max(
          delta_imu_source_gap_bridge_min_sec_,
          sample.sample_dt_sec * delta_imu_source_gap_bridge_min_ratio_);

        if (imu_gap.input_dt_sec >= delta_imu_source_gap_reset_sec_) {
          if (!source_gap_detected) {
            resetFilterAndTimeBase_("delta IMU source gap too large");
            return;
          }
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Delta IMU source-gap reset suppressed (source-gap pattern): source_dt=%.2fms recv_dt=%.2fms",
            imu_gap.input_dt_sec * 1000.0,
            std::isfinite(imu_gap.recv_dt_sec) ? imu_gap.recv_dt_sec * 1000.0 : -1.0);
        }

        if (!source_gap_detected && imu_gap.input_dt_sec >= min_bridge_sec) {
          if (imu_gap.input_dt_sec > delta_imu_source_gap_bridge_max_sec_) {
            resetFilterAndTimeBase_("delta IMU source gap too large for bridge");
            return;
          }

          bridge_delta_source_gap_as_rates = true;
          dt_candidate = imu_gap.input_dt_sec;
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "Delta IMU source-gap bridge: source_dt=%.2fms sample_dt=%.2fms. "
            "Using rate-hold propagation over source gap.",
            imu_gap.input_dt_sec * 1000.0,
            sample.sample_dt_sec * 1000.0);
        } else if (source_gap_detected) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Delta IMU source-gap bridge skipped (source-gap pattern): source_dt=%.2fms recv_dt=%.2fms sample_dt=%.2fms",
            imu_gap.input_dt_sec * 1000.0,
            std::isfinite(imu_gap.recv_dt_sec) ? imu_gap.recv_dt_sec * 1000.0 : -1.0,
            sample.sample_dt_sec * 1000.0);
        }
      }
    }

    // 2) 时间跳变检测：/clock reset 或上游时间戳回退
    if (auto_reset_on_time_jump_ && !use_source_sample_dt &&
        std::isfinite(dt_candidate) && dt_candidate < -time_jump_reset_threshold_sec_) {
      resetFilterAndTimeBase_("time went backwards (large jump)");
      return;
    }

    const double gyro_aux_dt_sec =
      std::isfinite(sample.angular_dt_sec) && sample.angular_dt_sec > 0.0
        ? sample.angular_dt_sec
        : dt_candidate;
    const double accel_aux_dt_sec =
      std::isfinite(sample.linear_dt_sec) && sample.linear_dt_sec > 0.0
        ? sample.linear_dt_sec
        : dt_candidate;
    const Eigen::Vector3d aux_angular =
      (sample.data_is_delta && std::isfinite(gyro_aux_dt_sec) && gyro_aux_dt_sec > 0.0)
        ? (sample.angular / gyro_aux_dt_sec)
        : (sample.data_is_delta ? Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())
                                : sample.angular);
    const Eigen::Vector3d aux_linear =
      (sample.data_is_delta && std::isfinite(accel_aux_dt_sec) && accel_aux_dt_sec > 0.0)
        ? (sample.linear / accel_aux_dt_sec)
        : (sample.data_is_delta ? Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())
                                : sample.linear);

    const double gyro_norm_rad_s = aux_angular.norm();
    const double gyro_norm_deg_s = gyro_norm_rad_s * 180.0 / M_PI;
    last_imu_gyro_norm_deg_s_ = gyro_norm_deg_s;
    const double accel_norm_mps2 = aux_linear.norm();
    const double accel_deviation_mps2 = std::abs(accel_norm_mps2 - 9.81);
    const bool maneuver_turn_rate_triggered =
      std::isfinite(imu_gap_turn_rate_gate_deg_s_) &&
      imu_gap_turn_rate_gate_deg_s_ > 0.0 &&
      std::isfinite(gyro_norm_deg_s) &&
      gyro_norm_deg_s >= imu_gap_turn_rate_gate_deg_s_;
    const bool maneuver_vertical_speed_triggered =
      std::isfinite(imu_gap_vertical_speed_gate_mps_) &&
      imu_gap_vertical_speed_gate_mps_ > 0.0 &&
      std::isfinite(last_mavros_vertical_speed_mps_) &&
      last_mavros_vertical_speed_mps_ >= imu_gap_vertical_speed_gate_mps_;
    const bool maneuver_accel_triggered =
      std::isfinite(imu_gap_accel_deviation_gate_mps2_) &&
      imu_gap_accel_deviation_gate_mps2_ > 0.0 &&
      std::isfinite(accel_deviation_mps2) &&
      accel_deviation_mps2 >= imu_gap_accel_deviation_gate_mps2_;
    const bool vertical_or_accel_triggered_now =
      maneuver_vertical_speed_triggered ||
      maneuver_accel_triggered;
    const double steady_now_sec = steady_now.seconds();
    if (vertical_or_accel_triggered_now) {
      last_vertical_or_accel_trigger_time_sec_ = steady_now_sec;
    }
    const bool vertical_or_accel_cooldown_active =
      std::isfinite(last_vertical_or_accel_trigger_time_sec_) &&
      imu_gap_maneuver_cooldown_sec_ > 0.0 &&
      (steady_now_sec - last_vertical_or_accel_trigger_time_sec_) <= imu_gap_maneuver_cooldown_sec_;

    // 3) dt 过滤/估计
    double dt = dt_candidate;
    bool use_segmented_propagation = false;
    int segmented_propagation_steps = 1;
    if (!std::isfinite(dt) || dt <= 0.0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "IMU dt invalid (dt=%.6f s). Using estimate %.6f s.",
        dt, imu_dt_estimate_sec_);
      dt = imu_dt_estimate_sec_;
    } else if (dt > max_imu_dt_sec_) {
      // Distinguish between the current "slow-but-recurring" ~0.12 s input cadence
      // and genuinely dangerous larger gaps. The former may be clamped to preserve
      // continuity; the latter should reset rather than propagate a corrupted step.
      const double severe_gap_sec = std::max(max_imu_dt_sec_, severe_imu_gap_reset_sec_);
      const bool allow_bridged_delta_source_gap =
        bridge_delta_source_gap_as_rates &&
        std::isfinite(dt) &&
        dt <= delta_imu_source_gap_bridge_max_sec_;
      if (auto_reset_on_time_jump_ && std::isfinite(dt) && dt >= severe_gap_sec &&
          !allow_bridged_delta_source_gap) {
        resetFilterAndTimeBase_("severe IMU gap");
        return;
      }
      const bool conservative_gap =
        vertical_or_accel_triggered_now || vertical_or_accel_cooldown_active;
      const bool prefer_segmented_turn_rate =
        maneuver_turn_rate_triggered;
      if (skip_medium_imu_gap_when_turning_ && conservative_gap && !prefer_segmented_turn_rate) {
        const double dropped_dt = std::max(0.0, dt - max_imu_dt_sec_);
        dt = std::clamp(max_imu_dt_sec_, min_imu_dt_sec_, max_imu_dt_sec_);
        segmented_propagation_steps = 1;
        use_segmented_propagation = false;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Medium IMU gap during vertical/accel maneuver: raw dt=%.6f s exceeds max %.6f s. "
          "Using conservative single-step propagation dt=%.6f s and dropping %.6f s "
          "(vertical=%s, accel=%s, cooldown=%s, gyro=%.2f/%.2f deg/s, vz=%.2f/%.2f m/s, accel_dev=%.2f/%.2f m/s^2).",
          dt_candidate, max_imu_dt_sec_, dt, dropped_dt,
          maneuver_vertical_speed_triggered ? "true" : "false",
          maneuver_accel_triggered ? "true" : "false",
          vertical_or_accel_cooldown_active && !vertical_or_accel_triggered_now ? "true" : "false",
          gyro_norm_deg_s, imu_gap_turn_rate_gate_deg_s_,
          last_mavros_vertical_speed_mps_, imu_gap_vertical_speed_gate_mps_,
          accel_deviation_mps2, imu_gap_accel_deviation_gate_mps2_);
        imu_dt_estimate_sec_ = 0.80 * imu_dt_estimate_sec_ + 0.20 * dt;
      } else {
        segmented_propagation_steps = std::max(
          2, static_cast<int>(std::ceil(dt / std::max(1e-6, max_imu_dt_sec_))));
        use_segmented_propagation = segmented_propagation_steps > 1;
        const double dt_step = dt / static_cast<double>(segmented_propagation_steps);
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Medium IMU gap: dt=%.6f s exceeds max %.6f s. Using segmented propagation with %d steps "
          "(dt_step=%.6f s, turn_rate=%s, vertical=%s, accel=%s, vert_accel_cooldown=%s, gyro=%.2f/%.2f deg/s, vz=%.2f/%.2f m/s, accel_dev=%.2f/%.2f m/s^2).",
          dt, max_imu_dt_sec_, segmented_propagation_steps, dt_step,
          maneuver_turn_rate_triggered ? "true" : "false",
          maneuver_vertical_speed_triggered ? "true" : "false",
          maneuver_accel_triggered ? "true" : "false",
          vertical_or_accel_cooldown_active && !vertical_or_accel_triggered_now ? "true" : "false",
          gyro_norm_deg_s, imu_gap_turn_rate_gate_deg_s_,
          last_mavros_vertical_speed_mps_, imu_gap_vertical_speed_gate_mps_,
          accel_deviation_mps2, imu_gap_accel_deviation_gate_mps2_);
        imu_dt_estimate_sec_ = 0.80 * imu_dt_estimate_sec_ + 0.20 * dt_step;
      }
    } else {
      dt = std::clamp(dt, min_imu_dt_sec_, max_imu_dt_sec_);
      // EMA 更新估计值：新的valid dt贡献20%，历史贡献80%
      // 改为0.8 alpha，降低EMA权重，让估计值更快适应dt变化
      imu_dt_estimate_sec_ = 0.80 * imu_dt_estimate_sec_ + 0.20 * dt;
    }

    const bool feed_core_with_delta = sample.data_is_delta && !bridge_delta_source_gap_as_rates;
    Eigen::Vector3d dtheta = bridge_delta_source_gap_as_rates ? aux_angular : sample.angular;
    Eigen::Vector3d dvel = bridge_delta_source_gap_as_rates ? aux_linear : sample.linear;

    // MAVROS 原始 IMU 采用机体 FLU 坐标系，而 KF-GINS 采用机体 FRD 坐标系，因此需在处理过程中进行转换。
    // 将 Python 转换器从主估算链中移除。
    if (sample.input_is_flu) {
      dtheta.y() = -dtheta.y();
      dtheta.z() = -dtheta.z();
      dvel.y() = -dvel.y();
      dvel.z() = -dvel.z();
    }

    if (!dtheta.allFinite() || !dvel.allFinite()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "IMU contains NaN/Inf, skipping.");
      return;
    }

    double final_t = std::numeric_limits<double>::quiet_NaN();
    const int propagation_steps = use_segmented_propagation ? segmented_propagation_steps : 1;
    const double dt_step = dt / static_cast<double>(propagation_steps);
    const Eigen::Vector3d dtheta_step = feed_core_with_delta ? (dtheta / propagation_steps) : dtheta;
    const Eigen::Vector3d dvel_step = feed_core_with_delta ? (dvel / propagation_steps) : dvel;

    for (int step = 0; step < propagation_steps; ++step) {
      //  生成送入 core 的时间戳：推荐用 dt 积分出的单调时间轴
      double t = raw_t;
      if (use_integrated_time_for_core_) {
        core_time_sec_ += dt_step;
        t = core_time_sec_;
      } else if (propagation_steps > 1) {
        const double steps_remaining = static_cast<double>(propagation_steps - step - 1);
        t = raw_t - steps_remaining * dt_step;
      }
      if (force_monotonic_time_for_core_) {
        if (std::isfinite(last_core_time_) && t <= last_core_time_) {
          t = last_core_time_ + dt_step;
        }
      }

      if (!core_->ingestImu(t, dtheta_step, dvel_step, dt_step, feed_core_with_delta)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "core_->ingestImu() failed");
        return;
      }
      last_core_time_ = t;
      final_t = t;
    }

    if (std::isfinite(final_t)) {
      maybeApplyHeadingUpdate_(final_t);
    }
    publishState();
  }

  void gnssCb(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    double t_raw = use_node_time_for_core_ ? rawTimeSecNow_() : rawTimeSecFromMsg_(msg->header.stamp);
    double t = t_raw;
    bool reset_this_gnss = false;

    // 支持 dropzones：当上游发布 NO_FIX，这里直接跳过 GNSS 更新
    if (msg->status.status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX) {
      last_gnss_has_fix_ = false;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "GNSS status=NO_FIX, skipping update.");
      return;
    }
    last_gnss_has_fix_ = true;

    if (!std::isfinite(msg->latitude) || !std::isfinite(msg->longitude) || !std::isfinite(msg->altitude)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "GNSS contains NaN/Inf, skipping.");
      return;
    }

    last_gnss_lat_rad_ = msg->latitude  * M_PI/180.0;
    last_gnss_lon_rad_ = msg->longitude * M_PI/180.0;
    last_gnss_h_m_     = msg->altitude;
    last_gnss_valid_   = true;

    // 第一次用 GNSS 设置 ENU 原点；若刚经历 reset，则等待连续稳定 GNSS 样本再重建
    if (!have_origin_) {
      if (waiting_for_origin_rebuild_) {
        if (!tryRebuildOriginFromStableGnss_(last_gnss_lat_rad_, last_gnss_lon_rad_, last_gnss_h_m_)) {
          return;
        }
      } else {
        setOriginFromLlh_(last_gnss_lat_rad_, last_gnss_lon_rad_, last_gnss_h_m_);
        RCLCPP_INFO(this->get_logger(),
                    "ENU origin set by GNSS: lat=%.8f lon=%.8f h=%.3f",
                    msg->latitude, msg->longitude, msg->altitude);
      }
    }

    // 第一次 GNSS 同时用于初始化/重置核心，避免从 (0,0,0) 等不合理状态起算导致数值问题
    if (!core_initialized_) {
      const char* reset_yaw_source = "zero";
      const double init_yaw = selectYawForCoreReset_(&reset_yaw_source);
      (void)core_->reset(msg->latitude, msg->longitude, msg->altitude, init_yaw);
      core_initialized_ = true;
      reset_this_gnss = true;
      prefer_preserved_yaw_on_next_core_reset_ = false;
      last_disarmed_yaw_lock_time_sec_ = now().seconds();
      RCLCPP_INFO(
        this->get_logger(),
        "Core reset by first GNSS fix. yaw=%.1f deg (source=%s, mavros=%s, preserved_core=%s)",
        init_yaw,
        reset_yaw_source,
        have_mavros_heading_ ? "yes" : "no",
        std::isfinite(last_trusted_core_yaw_deg_) ? "yes" : "no");
    }

    // GNSS 时间戳必须落在 IMU 时间轴上（用于插值/更新）；绑定到“当前 core 时间”
    if (use_integrated_time_for_core_) {
      const double base = std::isfinite(last_core_time_) ? last_core_time_ : 0.0;
      const double offset = std::clamp(gnss_time_offset_sec_, 1e-6, 0.1);
      t = base + offset;
      if (std::isfinite(last_gnss_time_sec_)) {
        t = std::max(t, last_gnss_time_sec_ + 1e-3);
      }
    } else if (force_monotonic_time_for_core_) {
      if (std::isfinite(last_core_time_) && t <= last_core_time_) {
        // GNSS 是低频的：用一个很小的前移，确保严格单调
        t = last_core_time_ + 1e-3;
      }
    }

    Eigen::Vector3d std_ned(-1,-1,-1);
    
    // 始终检查仿真模式，如果启用则直接使用仿真参数
    // （即使msg中有协方差，也优先信任仿真参数）
    if (use_sim_gnss_std_) {
      // 仿真模式：直接使用仿真参数，不经过 gnss_min_std_m_ clamp
      // 原代码 std::max(gnss_min_std_m_=0.5, sim_gnss_std=0.1) 导致
      // GNSS std 始终被抬高到 0.5m，使 GNSS 观测信任度降低 25 倍 → 位置发散
      const double floor = 0.01;  // 仅防止 std=0 导致 R=0
      std_ned = Eigen::Vector3d(
        std::max(floor, sim_gnss_std_h_m_),
        std::max(floor, sim_gnss_std_h_m_),
        std::max(floor, sim_gnss_std_u_m_));
      if (!have_sim_gnss_logged_) {
        RCLCPP_INFO(get_logger(), "✓ Simulation GNSS mode: std_h=%.3fm, std_u=%.3fm (no min-clamp)", 
                    sim_gnss_std_h_m_, sim_gnss_std_u_m_);
        have_sim_gnss_logged_ = true;
      }
    } else if (msg->position_covariance_type == sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN) {
      // 实机模式 + msg有协方差：使用msg中的协方差
      const double std_e = std::sqrt(std::max(0.0, msg->position_covariance[0]));
      const double std_n = std::sqrt(std::max(0.0, msg->position_covariance[4]));
      const double std_u = std::sqrt(std::max(0.0, msg->position_covariance[8]));
      // clamp：避免 std=0 造成 R=0 或数值发散
      std_ned = Eigen::Vector3d(
        std::max(gnss_min_std_m_, std_n),
        std::max(gnss_min_std_m_, std_e),
        std::max(gnss_min_std_m_, std_u));
    } else {
      // 实机模式 + msg无协方差：使用默认实机参数
      std_ned = Eigen::Vector3d(
        std::max(gnss_min_std_m_, gnss_default_std_h_m_),
        std::max(gnss_min_std_m_, gnss_default_std_h_m_),
        std::max(gnss_min_std_m_, gnss_default_std_u_m_));
    }

    if (!reset_this_gnss &&
        disarmed_yaw_lock_enable_ && !mavros_armed_ && core_initialized_ && have_mavros_heading_) {
      const double now_sec = now().seconds();
      const bool interval_elapsed =
        !std::isfinite(last_disarmed_yaw_lock_time_sec_) ||
        (now_sec - last_disarmed_yaw_lock_time_sec_) >=
          std::max(0.1, disarmed_yaw_lock_interval_sec_);

      if (interval_elapsed) {
        const auto st = core_->current();
        const double yaw_err_deg = std::abs(shortestAngleDiffDeg_(st.yaw_deg, mavros_heading_ned_deg_));
        last_disarmed_yaw_lock_time_sec_ = now_sec;

        if (!std::isfinite(st.yaw_deg) ||
            yaw_err_deg >= std::max(0.0, disarmed_yaw_lock_max_yaw_err_deg_)) {
          resetCoreForDisarmedYawLock_(yaw_err_deg);
          return;
        }
      }
    }

    if (!have_first_gnss_stamp_) {
      first_gnss_stamp_ = now();
      have_first_gnss_stamp_ = true;
    }

    // ZUPT: 使用零速度观测代替破坏性全滤波器重置
    // 旧代码每秒调用 core_->reset(yaw=0)，完全销毁偏差估计、协方差矩阵和航向信息。
    // 当无人机起飞时航向=0（错误），导致 INS 机械编排方向错误 → 位置指数级发散。
    // 新代码注入零速度观测 (proper ZUPT)，保留航向和偏差估计。
    bool zupt_applied = false;
    if (use_zero_velocity_update_when_disarmed_ && !mavros_armed_) {
      const double now_sec = now().seconds();
      const bool need_zupt =
        !std::isfinite(last_zupt_reset_time_sec_) ||
        (now_sec - last_zupt_reset_time_sec_) >= zupt_reset_interval_sec_;
      if (need_zupt) {
        Eigen::Vector3d zero_vel_std(zupt_std_mps_, zupt_std_mps_, zupt_std_mps_);
        core_->ingestGnssVel(t, 0.0, 0.0, 0.0, zero_vel_std);
        last_zupt_reset_time_sec_ = now_sec;
        zupt_applied = true;
      }
      // 不再 return，让 GNSS 位置观测正常处理
    }

    if (!core_->ingestGnss(t, msg->latitude, msg->longitude, msg->altitude, std_ned)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "core_->ingestGnss() failed");
      return;
    }

    // GNSS 速度观测
    // GNSS velocity derived from consecutive position fixes
    // 这是解决姿态（尤其 yaw）不可观的关键：速度观测让 H 矩阵覆盖了 V 状态，
    // 通过 F 矩阵中的 V↔PHI 耦合，间接使姿态可观。
    // 当 ZUPT 已应用时跳过（ZUPT 的 0.05 m/s std 比位置差分的 ~0.7 m/s 更紧）
    if (enable_gnss_velocity_update_ && !zupt_applied && have_prev_gnss_for_vel_) {
      const double dt_gnss = t - prev_gnss_vel_time_;
      if (std::isfinite(dt_gnss) && dt_gnss > 0.05 && dt_gnss < 5.0) {
        const double lat_avg = (msg->latitude * M_PI/180.0 + prev_gnss_vel_lat_rad_) * 0.5;
        const double dlat_rad = msg->latitude * M_PI/180.0 - prev_gnss_vel_lat_rad_;
        const double dlon_rad = msg->longitude * M_PI/180.0 - prev_gnss_vel_lon_rad_;
        const double dh = msg->altitude - prev_gnss_vel_h_;

        // 使用简化的地球半径（WGS84 平均值）
        const double R_earth = 6371000.0;
        const double vN =  dlat_rad * R_earth / dt_gnss;
        const double vE =  dlon_rad * R_earth * std::cos(lat_avg) / dt_gnss;
        const double vD = -dh / dt_gnss;

        if (std::isfinite(vN) && std::isfinite(vE) && std::isfinite(vD)) {
          // 位置差分速度的 std ≈ sqrt(2) * pos_std / dt_gnss
          // 仿真模式下降低 vel_std 下限：
          // 旧代码 max(0.5, 0.14) = 0.5 m/s，R 膨胀 12.6× → yaw 可观性严重不足
          const double vel_floor_h = std::max(0.05, gnss_vel_std_floor_h_mps_);
          const double vel_floor_u = std::max(0.1, gnss_vel_std_floor_u_mps_);
          const double vel_std_h = std::max(vel_floor_h, std::sqrt(2.0) * std_ned.x() / dt_gnss);
          const double vel_std_u = std::max(vel_floor_u, std::sqrt(2.0) * std_ned.z() / dt_gnss);
          Eigen::Vector3d vel_std(vel_std_h, vel_std_h, vel_std_u);

          core_->ingestGnssVel(t, vN, vE, vD, vel_std);
        }
      }
    }
    prev_gnss_vel_lat_rad_ = msg->latitude  * M_PI/180.0;
    prev_gnss_vel_lon_rad_ = msg->longitude * M_PI/180.0;
    prev_gnss_vel_h_       = msg->altitude;
    prev_gnss_vel_time_    = t;
    have_prev_gnss_for_vel_ = true;

    last_core_time_ = t;
    last_gnss_time_sec_ = t;

    publishState();
  }

  // ------------------------- Publish -------------------------
  void publishState()
  {
    if (!have_origin_) return;
    // 预热逻辑只影响 Path，不应阻塞 /kf_gins/odom（否则看起来像 IEKF “没了”）
    if (!allow_paint_) {
      const bool warmup_ok = (now() - node_start_time_).seconds() >= publish_after_sec_;
      const bool gnss_ok   = have_first_gnss_stamp_ &&
                             (now() - first_gnss_stamp_).seconds() >= start_after_gnss_sec_;
      if (warmup_ok && gnss_ok) allow_paint_ = true;
    }


    const auto st = core_->current();

    auto isfinite_d = [](double v){ return std::isfinite(v); };
    if (!isfinite_d(st.lat_deg) || !isfinite_d(st.lon_deg) || !isfinite_d(st.h_m) ||
        !isfinite_d(st.roll_deg) || !isfinite_d(st.pitch_deg) || !isfinite_d(st.yaw_deg)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "State NaN/Inf (lat=%.3f lon=%.3f h=%.3f rpy=%.3f/%.3f/%.3f)",
        st.lat_deg, st.lon_deg, st.h_m, st.roll_deg, st.pitch_deg, st.yaw_deg);

      if (auto_reset_on_invalid_state_ && last_gnss_valid_) {
        const double now_sec = now().seconds();
        if (!std::isfinite(last_invalid_reset_time_sec_) ||
            (now_sec - last_invalid_reset_time_sec_) >= invalid_state_reset_cooldown_sec_) {
          last_invalid_reset_time_sec_ = now_sec;
          const double reset_yaw = getInitialYawDeg_();
          RCLCPP_WARN(get_logger(),
                      "Auto-reset core due to invalid state. yaw=%.1f deg", reset_yaw);
          publishResetEvent_("invalid state");
          (void)core_->reset(last_gnss_lat_rad_ * 180.0/M_PI,
                             last_gnss_lon_rad_ * 180.0/M_PI,
                             last_gnss_h_m_,
                             reset_yaw);
          core_initialized_ = true;
          core_llh_aligned_ = false;

          // 清空 Path，避免一堆“乱飞线”
          path_msg_.poses.clear();
          have_last_path_ = false;
          have_last_enu_ = false;

          // 重新等待 GNSS 对齐后再画 Path（odom 会继续发布）
          allow_paint_ = false;
          node_start_time_ = now();
          have_first_gnss_stamp_ = false;
          have_prev_imu_ = false;
        }
      }
      return;
    }

    last_trusted_core_yaw_deg_ = normalizeAngleDeg_(st.yaw_deg);

    // 1) 姿态：NED欧拉角 → ENU欧拉角
    // KF-GINS 内部使用 NED/FRD 坐标系，matrix2euler(Cbn) 输出 [roll_NED, pitch_NED, yaw_NED]
    // ROS/tf2 使用 ENU/FLU 坐标系，setRPY 期望 [roll_ENU, pitch_ENU, yaw_ENU]
    // 转换关系（通过 R_ENU_from_FLU = R_ENU_from_NED * Cbn * R_FRD_from_FLU 推导）：
    //   roll_ENU  =  roll_NED   （前轴方向相同）
    //   pitch_ENU = -pitch_NED  （Y轴翻转：Right→Left）
    //   yaw_ENU   = π/2 - yaw_NED （Z轴翻转 + 90°旋转：North→East参考）
    const double roll_rad  =  st.roll_deg  * M_PI/180.0;
    const double pitch_rad = -st.pitch_deg * M_PI/180.0;
    const double yaw_rad   =  M_PI/2.0 - st.yaw_deg * M_PI/180.0;

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
      if (std::isfinite(diff_m)) {
        last_core_gnss_diff_m_ = diff_m;
      }
      if (!core_llh_aligned_ && std::isfinite(diff_m) && diff_m < align_gate_m_)
        core_llh_aligned_ = true;
      const bool core_vs_gnss_diff_large =
        core_llh_aligned_ &&
        std::isfinite(diff_m) &&
        publish_max_core_gnss_diff_m_ > 0.0 &&
        diff_m > publish_max_core_gnss_diff_m_;
      if (core_vs_gnss_diff_large) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Core-vs-GNSS diff %.2f m exceeds %.2f m; keeping raw IEKF pose on odom/path for diagnosis",
          diff_m, publish_max_core_gnss_diff_m_);
      }

      // 选择用于可视化/发布的位置。保持 /kf_gins/odom 为真实 IEKF，避免在
      // core 发散时被 GNSS fallback 污染 comparison/path 诊断链路。
      if ((use_gnss_llh_for_pose_when_disarmed_ && !mavros_armed_) ||
          use_gnss_llh_for_pose_ || !core_llh_aligned_) {
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

    // 发布 raw ENU pose，避免拐点处的可视化低通滞后进入 odom/comparison/path 诊断链。
    Eigen::Vector3d enu_vis = enu;
    last_enu_ = enu;
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
    if (!allow_paint_) return;
    if (path_require_armed_ && !mavros_armed_) return;
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
  std::string config_path_, map_frame_, base_frame_, odom_frame_;
  std::string imu_source_, imu_topic_, px4_sensor_combined_topic_, px4_vehicle_imu_topic_, gnss_topic_;
  std::string active_imu_topic_name_;
  bool imu_is_delta_{false};
  bool imu_input_is_flu_{true};
  bool use_source_dt_for_px4_imu_{true};
  double imu_gap_warn_ms_{60.0};
  double path_rate_hz_{5.0};
  int    pose_decimation_{10};
  int    max_path_pts_{20000};
  bool   use_gnss_llh_for_pose_{true};
  bool   use_gnss_llh_for_pose_when_disarmed_{true};
  double max_jump_m_{200.0};
  double align_gate_m_{300.0};
  bool   core_llh_aligned_{false};  // 核心的LLH是否已与最近GNSS对齐

  // GNSS std fallback/clamp
  bool use_sim_gnss_std_{true};  // 【修复】仿真模式标志
  bool have_sim_gnss_logged_{false};  // 【修复】避免重复日志
  double gnss_default_std_h_m_{5.0};
  double gnss_default_std_u_m_{10.0};
  double sim_gnss_std_h_m_{0.1};  // 【修复】仿真水平std
  double sim_gnss_std_u_m_{0.2};  // 【修复】仿真竖直std
  double gnss_min_std_m_{0.5};

  // 预热/启画控制
  double start_after_gnss_sec_{1.0};
  double publish_after_sec_{2.0};
  double reset_gate_m_{20.0};
  rclcpp::Time node_start_time_;
  bool allow_paint_{false};
  rclcpp::Time first_gnss_stamp_;
  bool have_first_gnss_stamp_{false};

  // MAVROS state gating (optional)
  std::string mavros_state_topic_{"/mavros/state"};
  std::string mavros_local_velocity_topic_{"/mavros/local_position/velocity_local"};
  bool path_require_armed_{false};
  bool clear_path_on_arm_transition_{false};
  bool mavros_armed_{false};

  // MAVROS 航向 (来自 EKF2 磁力计融合)
  bool have_mavros_heading_{false};
  double mavros_heading_ned_deg_{0.0};
  double last_mavros_heading_rx_time_sec_{std::numeric_limits<double>::quiet_NaN()};
  bool have_prev_mavros_heading_sample_{false};
  double prev_mavros_heading_sample_deg_{0.0};
  double prev_mavros_heading_sample_time_sec_{std::numeric_limits<double>::quiet_NaN()};
  double last_mavros_heading_rate_deg_s_{std::numeric_limits<double>::quiet_NaN()};
  double last_large_mavros_heading_jump_time_sec_{std::numeric_limits<double>::quiet_NaN()};
  double last_large_mavros_heading_jump_deg_{0.0};
  double last_mavros_speed_mps_{std::numeric_limits<double>::quiet_NaN()};
  double last_mavros_horizontal_speed_mps_{std::numeric_limits<double>::quiet_NaN()};
  double last_mavros_vertical_speed_mps_{std::numeric_limits<double>::quiet_NaN()};
  double last_mavros_velocity_rx_time_sec_{std::numeric_limits<double>::quiet_NaN()};

  // 零速度更新 (ZUPT) - 无人机未启动时
  bool use_zero_velocity_update_when_disarmed_{true};
  double zupt_reset_interval_sec_{1.0};
  double zupt_std_mps_{0.05};
  double last_zupt_reset_time_sec_{std::numeric_limits<double>::quiet_NaN()};
  bool disarmed_yaw_lock_enable_{true};
  double disarmed_yaw_lock_interval_sec_{1.0};
  double disarmed_yaw_lock_max_yaw_err_deg_{5.0};
  bool force_zero_antlever_{true};
  bool enable_gnss_velocity_update_{false};
  double gnss_vel_std_floor_h_mps_{0.5};
  double gnss_vel_std_floor_u_mps_{1.0};
  bool use_online_reset_covariance_{true};
  double reset_pos_std_m_{5.0};
  double reset_vel_std_mps_{5.0};
  double reset_roll_pitch_std_deg_{5.0};
  double reset_yaw_std_deg_{10.0};
  bool enable_heading_update_{true};
  double heading_update_std_deg_{3.0};
  double heading_update_max_rate_hz_{10.0};
  double heading_update_max_age_sec_{0.5};
  double heading_update_innovation_gate_deg_{20.0};
  double heading_update_armed_innovation_gate_deg_{8.0};
  double heading_update_turn_innovation_gate_deg_{60.0};
  double heading_update_max_source_yaw_rate_deg_s_{15.0};
  double heading_update_source_jump_gate_deg_{20.0};
  double heading_update_source_jump_block_sec_{2.0};
  double heading_update_hard_innovation_gate_deg_{15.0};
  bool heading_post_turn_reacquire_enable_{true};
  double heading_post_turn_reacquire_window_sec_{2.0};
  double heading_post_turn_reacquire_max_residual_deg_{45.0};
  double heading_post_turn_reacquire_std_deg_{2.0};
  double heading_post_turn_reacquire_hold_sec_{4.0};
  double heading_post_turn_reacquire_max_rate_hz_{5.0};
  bool heading_post_turn_force_relock_enable_{true};
  double heading_post_turn_force_relock_min_residual_deg_{20.0};
  int heading_post_turn_force_relock_min_consecutive_blocks_{3};
  double heading_post_turn_force_relock_yaw_std_deg_{5.0};
  double heading_post_turn_force_relock_max_rate_hz_{1.0};
  bool heading_post_turn_hold_force_relock_enable_{true};
  double heading_post_turn_hold_force_relock_min_residual_deg_{13.0};
  int heading_post_turn_hold_force_relock_min_consecutive_blocks_{20};
  bool heading_armed_cruise_force_relock_enable_{true};
  double heading_armed_cruise_force_relock_min_residual_deg_{13.0};
  int heading_armed_cruise_force_relock_min_consecutive_blocks_{30};
  double heading_armed_cruise_force_relock_min_horizontal_speed_mps_{4.0};
  double heading_armed_cruise_force_relock_max_vertical_speed_mps_{1.0};
  double heading_armed_cruise_force_relock_max_gyro_deg_s_{2.0};
  double heading_armed_cruise_force_relock_max_source_yaw_rate_deg_s_{2.0};
  double heading_armed_cruise_force_relock_yaw_std_deg_{5.0};
  double heading_armed_cruise_force_relock_max_rate_hz_{1.0};
  bool heading_recovery_enable_{true};
  double heading_recovery_min_residual_deg_{15.0};
  double heading_recovery_max_residual_deg_{80.0};
  double heading_recovery_max_step_deg_{3.0};
  double heading_recovery_std_deg_{10.0};
  double heading_recovery_max_rate_hz_{2.0};
  double heading_recovery_max_gyro_deg_s_{2.0};
  double heading_recovery_max_source_yaw_rate_deg_s_{2.0};
  double heading_recovery_max_horizontal_speed_mps_{0.5};
  double heading_recovery_max_vertical_speed_mps_{2.0};
  int heading_recovery_min_consecutive_skips_{2};
  double heading_update_low_speed_thresh_mps_{2.0};
  double heading_update_armed_min_horizontal_speed_mps_{0.0};
  double heading_update_max_vertical_speed_mps_{0.8};
  bool heading_update_when_armed_{false};
  double heading_update_armed_max_speed_thresh_mps_{6.0};
  bool heading_update_when_disarmed_{true};
  bool heading_update_when_armed_low_speed_{true};
  bool heading_update_when_gnss_no_fix_{true};
  bool skip_medium_imu_gap_when_turning_{true};
  double imu_gap_turn_rate_gate_deg_s_{10.0};
  double imu_gap_vertical_speed_gate_mps_{1.0};
  double imu_gap_accel_deviation_gate_mps2_{2.0};
  double imu_gap_maneuver_cooldown_sec_{1.0};
  bool skip_large_imu_gap_samples_{true};
  double severe_imu_gap_reset_sec_{0.25};
  bool delta_imu_source_gap_bridge_enable_{true};
  double delta_imu_source_gap_bridge_min_sec_{0.05};
  double delta_imu_source_gap_bridge_min_ratio_{5.0};
  double delta_imu_source_gap_bridge_max_sec_{0.30};
  double delta_imu_source_gap_reset_sec_{1.0};
  bool source_gap_clamp_enable_{true};
  double source_gap_clamp_min_sec_{0.05};
  double source_gap_clamp_min_ratio_{5.0};
  double source_gap_clamp_recv_dt_max_sec_{0.03};
  bool sensor_combined_source_gap_diag_enable_{true};
  double sensor_combined_source_gap_diag_min_sec_{0.05};
  double sensor_combined_source_gap_diag_min_ratio_{5.0};
  double sensor_combined_source_gap_reset_sec_{0.50};
  int origin_rebuild_required_samples_{3};
  double origin_rebuild_gate_m_{3.0};
  double publish_max_core_gnss_diff_m_{5.0};
  double preserved_core_yaw_max_mavros_diff_deg_{15.0};
  double preserved_core_yaw_max_core_gnss_diff_m_{5.0};
  double last_trusted_core_yaw_deg_{std::numeric_limits<double>::quiet_NaN()};
  double last_core_gnss_diff_m_{std::numeric_limits<double>::quiet_NaN()};
  bool prefer_preserved_yaw_on_next_core_reset_{false};
  bool last_gnss_has_fix_{true};
  double last_heading_attempt_time_sec_{std::numeric_limits<double>::quiet_NaN()};
  double last_heading_update_time_sec_{std::numeric_limits<double>::quiet_NaN()};
  double last_heading_recovery_time_sec_{std::numeric_limits<double>::quiet_NaN()};
  double last_turning_heading_time_sec_{std::numeric_limits<double>::quiet_NaN()};
  double last_post_turn_reacquire_time_sec_{std::numeric_limits<double>::quiet_NaN()};
  double last_post_turn_reacquire_apply_time_sec_{std::numeric_limits<double>::quiet_NaN()};
  double post_turn_hold_end_time_sec_{std::numeric_limits<double>::quiet_NaN()};
  double last_post_turn_force_relock_time_sec_{std::numeric_limits<double>::quiet_NaN()};
  double last_armed_cruise_force_relock_time_sec_{std::numeric_limits<double>::quiet_NaN()};
  int heading_large_residual_skip_count_{0};
  int post_turn_blocked_count_{0};
  int armed_cruise_blocked_count_{0};
  double last_disarmed_yaw_lock_time_sec_{std::numeric_limits<double>::quiet_NaN()};
  double last_vertical_or_accel_trigger_time_sec_{std::numeric_limits<double>::quiet_NaN()};
  double last_imu_gyro_norm_deg_s_{std::numeric_limits<double>::quiet_NaN()};


  // ---- core ----
  std::unique_ptr<kfcore::KFCore> core_;

  // ---- frames origin ----
  bool have_origin_{false};
  bool waiting_for_origin_rebuild_{false};
  bool have_origin_rebuild_candidate_{false};
  int origin_rebuild_candidate_count_{0};
  Eigen::Vector3d origin_ecef_{0,0,0};
  double origin_lat_{0.0}, origin_lon_{0.0};
  double origin_rebuild_candidate_lat_rad_{0.0};
  double origin_rebuild_candidate_lon_rad_{0.0};
  double origin_rebuild_candidate_h_m_{0.0};

  // ---- pubs/subs ----
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr      path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr reset_event_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr px4_sensor_combined_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleImu>::SharedPtr px4_vehicle_imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_sub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr mavros_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr mavros_local_velocity_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr mavros_imu_heading_sub_;
  rclcpp::TimerBase::SharedPtr path_timer_;

  // ---- messages ----
  nav_msgs::msg::Path path_msg_;

  // ---- IMU dt ----
  bool   have_prev_imu_{false};
  double prev_imu_time_{0.0};
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  rclcpp::Time prev_imu_steady_time_{0, 0, RCL_STEADY_TIME};
  double last_imu_input_stamp_sec_{std::numeric_limits<double>::quiet_NaN()};
  rclcpp::Time last_imu_recv_steady_time_{0, 0, RCL_STEADY_TIME};
  bool have_last_imu_recv_steady_time_{false};
  int sensor_combined_source_gap_diag_count_{0};
  int sensor_combined_source_gap_diag_armed_count_{0};
  double sensor_combined_source_gap_max_sec_{0.0};
  double sensor_combined_source_gap_max_ratio_{0.0};

  // ---- time source / robustness ----
  bool use_node_time_for_core_{true};
  bool use_integrated_time_for_core_{true};
  bool use_steady_time_for_imu_dt_{true};
  bool force_monotonic_time_for_core_{true};
  double min_imu_dt_sec_{1e-3};
  double max_imu_dt_sec_{0.20};
  double imu_dt_estimate_sec_{0.01};
  bool auto_reset_on_time_jump_{true};
  double time_jump_reset_threshold_sec_{0.5};
  double gnss_time_offset_sec_{0.001};
  bool core_initialized_{false};
  double last_core_time_{-std::numeric_limits<double>::infinity()};
  double last_gnss_time_sec_{-std::numeric_limits<double>::infinity()};
  bool auto_reset_on_invalid_state_{true};
  double invalid_state_reset_cooldown_sec_{5.0};
  double last_invalid_reset_time_sec_{std::numeric_limits<double>::quiet_NaN()};
  uint32_t reset_event_seq_{0};

  // ---- internal time base (relative) ----
  bool have_raw_time_zero_{false};
  double raw_time_zero_sec_{0.0};
  double prev_imu_raw_rel_sec_{0.0};
  double core_time_sec_{0.0};

  // ---- smoothing (for visualization only) ----
  Eigen::Vector3d last_enu_{0,0,0};
  bool have_last_enu_{false};

  // ---- GNSS cache for pose ----
  bool   last_gnss_valid_{false};
  double last_gnss_lat_rad_{0.0}, last_gnss_lon_rad_{0.0}, last_gnss_h_m_{0.0};

  // ---- GNSS velocity derivation ----
  bool   have_prev_gnss_for_vel_{false};
  double prev_gnss_vel_lat_rad_{0.0}, prev_gnss_vel_lon_rad_{0.0}, prev_gnss_vel_h_{0.0};
  double prev_gnss_vel_time_{0.0};

  // ---- 连贯轨迹相关 ----
  rclcpp::Time   last_path_stamp_;
  bool           have_last_path_stamp_{false};
  Eigen::Vector3d last_path_enu_{0,0,0};
  bool            have_last_path_{false};

  // 参数
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
