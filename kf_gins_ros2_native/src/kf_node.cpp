#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <px4_msgs/msg/sensor_gps.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_imu.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/bool.hpp>
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
#include <cstring>
#include <filesystem>
#include <fstream>
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
    gnss_source_ = this->declare_parameter<std::string>("gnss_source", "navsatfix");
    gnss_topic_   = this->declare_parameter<std::string>("gnss_topic", "/gps/fix");
    px4_sensor_gps_topic_ =
      this->declare_parameter<std::string>("px4_sensor_gps_topic", "/fmu/out/vehicle_gps_position");
    px4_vehicle_global_position_topic_ = this->declare_parameter<std::string>(
      "px4_vehicle_global_position_topic", "/fmu/out/vehicle_global_position");
    imu_is_delta_ = this->declare_parameter<bool>("imu_is_delta", false);
    imu_input_is_flu_ = this->declare_parameter<bool>("imu_input_is_flu", true);
    px4_imu_qos_depth_ = std::max<int>(
      1, this->declare_parameter<int>("px4_imu_qos_depth", 200));
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
    speed_source_ = this->declare_parameter<std::string>("speed_source", "mavros_local_velocity");
    px4_vehicle_local_position_topic_ = this->declare_parameter<std::string>(
      "px4_vehicle_local_position_topic", "/fmu/out/vehicle_local_position");
    px4_vehicle_odometry_topic_ = this->declare_parameter<std::string>(
      "px4_vehicle_odometry_topic", "/fmu/out/vehicle_odometry");
    heading_source_ = this->declare_parameter<std::string>("heading_source", "mavros_imu");
    mavros_heading_topic_ = this->declare_parameter<std::string>(
      "mavros_heading_topic", "/mavros/imu/data");
    px4_vehicle_attitude_topic_ = this->declare_parameter<std::string>(
      "px4_vehicle_attitude_topic", "/fmu/out/vehicle_attitude");
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
    enable_native_sensor_gps_velocity_aid_ =
      this->declare_parameter<bool>("enable_native_sensor_gps_velocity_aid", true);
    native_sensor_gps_velocity_max_age_sec_ =
      this->declare_parameter<double>("native_sensor_gps_velocity_max_age_sec", 0.5);
    native_gnss_speed_std_scale_ =
      this->declare_parameter<double>("native_gnss_speed_std_scale", 1.0);
    gnss_vel_std_floor_h_mps_ = this->declare_parameter<double>("gnss_vel_std_floor_h_mps", 0.5);
    gnss_vel_std_floor_u_mps_ = this->declare_parameter<double>("gnss_vel_std_floor_u_mps", 1.0);
    armed_cruise_native_gnss_vel_override_enable_ =
      this->declare_parameter<bool>("armed_cruise_native_gnss_vel_override_enable", true);
    armed_cruise_gnss_pos_override_enable_ =
      this->declare_parameter<bool>("armed_cruise_gnss_pos_override_enable", false);
    armed_cruise_gnss_pos_std_h_m_ =
      this->declare_parameter<double>("armed_cruise_gnss_pos_std_h_m", 0.06);
    armed_cruise_gnss_pos_std_u_m_ =
      this->declare_parameter<double>("armed_cruise_gnss_pos_std_u_m", 0.08);
    armed_cruise_gnss_pos_residual_boost_enable_ =
      this->declare_parameter<bool>("armed_cruise_gnss_pos_residual_boost_enable", false);
    armed_cruise_gnss_pos_residual_boost_threshold_m_ =
      this->declare_parameter<double>("armed_cruise_gnss_pos_residual_boost_threshold_m", 0.12);
    armed_cruise_gnss_pos_residual_boost_hold_sec_ =
      this->declare_parameter<double>("armed_cruise_gnss_pos_residual_boost_hold_sec", 20.0);
    armed_cruise_native_gnss_vel_min_horizontal_speed_mps_ =
      this->declare_parameter<double>("armed_cruise_native_gnss_vel_min_horizontal_speed_mps", 0.5);
    armed_cruise_native_gnss_vel_std_h_mps_ =
      this->declare_parameter<double>("armed_cruise_native_gnss_vel_std_h_mps", 0.05);
    armed_cruise_native_gnss_vel_std_u_mps_ =
      this->declare_parameter<double>("armed_cruise_native_gnss_vel_std_u_mps", 0.10);
    armed_cruise_native_gnss_vel_residual_boost_enable_ =
      this->declare_parameter<bool>("armed_cruise_native_gnss_vel_residual_boost_enable", false);
    armed_cruise_native_gnss_vel_residual_boost_threshold_mps_ =
      this->declare_parameter<double>("armed_cruise_native_gnss_vel_residual_boost_threshold_mps", 0.10);
    armed_cruise_native_gnss_vel_residual_boost_hold_sec_ =
      this->declare_parameter<double>("armed_cruise_native_gnss_vel_residual_boost_hold_sec", 8.0);
    armed_cruise_native_gnss_vel_residual_boost_std_h_mps_ =
      this->declare_parameter<double>("armed_cruise_native_gnss_vel_residual_boost_std_h_mps", 0.03);
    armed_cruise_native_gnss_vel_residual_boost_std_u_mps_ =
      this->declare_parameter<double>("armed_cruise_native_gnss_vel_residual_boost_std_u_mps", 0.08);
    armed_cruise_vertical_cov_reopen_enable_ =
      this->declare_parameter<bool>("armed_cruise_vertical_cov_reopen_enable", false);
    armed_cruise_vertical_cov_reopen_threshold_m_ =
      this->declare_parameter<double>("armed_cruise_vertical_cov_reopen_threshold_m", 0.15);
    armed_cruise_vertical_cov_reopen_hold_sec_ =
      this->declare_parameter<double>("armed_cruise_vertical_cov_reopen_hold_sec", 8.0);
    armed_cruise_vertical_cov_reopen_pos_std_m_ =
      this->declare_parameter<double>("armed_cruise_vertical_cov_reopen_pos_std_m", 0.15);
    armed_cruise_vertical_cov_reopen_vel_std_mps_ =
      this->declare_parameter<double>("armed_cruise_vertical_cov_reopen_vel_std_mps", 0.10);
    armed_cruise_vertical_cov_reopen_accbias_std_z_mps2_ =
      this->declare_parameter<double>("armed_cruise_vertical_cov_reopen_accbias_std_z_mps2", 0.05);
    post_flight_vertical_cov_reopen_enable_ =
      this->declare_parameter<bool>("post_flight_vertical_cov_reopen_enable", true);
    post_flight_vertical_cov_reopen_threshold_m_ =
      this->declare_parameter<double>("post_flight_vertical_cov_reopen_threshold_m", 0.12);
    post_flight_vertical_cov_reopen_hold_sec_ =
      this->declare_parameter<double>("post_flight_vertical_cov_reopen_hold_sec", 20.0);
    post_flight_vertical_cov_reopen_grace_sec_ =
      this->declare_parameter<double>("post_flight_vertical_cov_reopen_grace_sec", 30.0);
    post_flight_vertical_cov_reopen_pos_std_m_ =
      this->declare_parameter<double>("post_flight_vertical_cov_reopen_pos_std_m", 0.25);
    post_flight_vertical_cov_reopen_vel_std_mps_ =
      this->declare_parameter<double>("post_flight_vertical_cov_reopen_vel_std_mps", 0.10);
    post_flight_vertical_cov_reopen_accbias_std_z_mps2_ =
      this->declare_parameter<double>("post_flight_vertical_cov_reopen_accbias_std_z_mps2", 0.05);
    gnss_update_debug_csv_path_ =
      this->declare_parameter<std::string>("gnss_update_debug_csv_path", "");
    heading_update_debug_csv_path_ =
      this->declare_parameter<std::string>("heading_update_debug_csv_path", "");
    state_publish_debug_csv_path_ =
      this->declare_parameter<std::string>("state_publish_debug_csv_path", "");
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
    heading_post_turn_cruise_track_std_deg_ =
      this->declare_parameter<double>(
        "heading_post_turn_cruise_track_std_deg", heading_post_turn_reacquire_std_deg_);
    heading_post_turn_cruise_track_max_gyro_deg_s_ =
      this->declare_parameter<double>("heading_post_turn_cruise_track_max_gyro_deg_s", 3.5);
    heading_post_turn_cruise_track_min_horizontal_speed_mps_ =
      this->declare_parameter<double>(
        "heading_post_turn_cruise_track_min_horizontal_speed_mps", 4.0);
    heading_post_turn_low_hspeed_cluster_enable_ =
      this->declare_parameter<bool>("heading_post_turn_low_hspeed_cluster_enable", false);
    heading_post_turn_low_hspeed_cluster_min_horizontal_speed_mps_ =
      this->declare_parameter<double>(
        "heading_post_turn_low_hspeed_cluster_min_horizontal_speed_mps", 4.3);
    heading_post_turn_low_hspeed_cluster_max_horizontal_speed_mps_ =
      this->declare_parameter<double>(
        "heading_post_turn_low_hspeed_cluster_max_horizontal_speed_mps", 4.6);
    heading_post_turn_low_hspeed_cluster_max_vertical_speed_mps_ =
      this->declare_parameter<double>(
        "heading_post_turn_low_hspeed_cluster_max_vertical_speed_mps", 0.2);
    heading_post_turn_low_hspeed_cluster_max_gyro_deg_s_ =
      this->declare_parameter<double>(
        "heading_post_turn_low_hspeed_cluster_max_gyro_deg_s", 2.0);
    heading_post_turn_low_hspeed_cluster_max_source_yaw_rate_deg_s_ =
      this->declare_parameter<double>(
        "heading_post_turn_low_hspeed_cluster_max_source_yaw_rate_deg_s", 0.5);
    heading_post_turn_reacquire_hold_sec_ =
      this->declare_parameter<double>("heading_post_turn_reacquire_hold_sec", 4.0);
    heading_post_turn_reacquire_max_rate_hz_ =
      this->declare_parameter<double>("heading_post_turn_reacquire_max_rate_hz", 5.0);
    heading_post_turn_force_relock_enable_ =
      this->declare_parameter<bool>("heading_post_turn_force_relock_enable", true);
    heading_post_turn_force_relock_min_residual_deg_ =
      this->declare_parameter<double>("heading_post_turn_force_relock_min_residual_deg", 20.0);
    heading_post_turn_cruise_track_continue_sec_ =
      this->declare_parameter<double>("heading_post_turn_cruise_track_continue_sec", 1.5);
    heading_post_turn_cruise_track_continue_min_residual_deg_ =
      this->declare_parameter<double>(
        "heading_post_turn_cruise_track_continue_min_residual_deg",
        0.8);
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
    heading_armed_cruise_track_min_residual_deg_ =
      this->declare_parameter<double>("heading_armed_cruise_track_min_residual_deg", 0.9);
    heading_post_turn_armed_cruise_track_window_sec_ =
      this->declare_parameter<double>("heading_post_turn_armed_cruise_track_window_sec", 0.0);
    heading_post_turn_armed_cruise_track_min_residual_deg_ =
      this->declare_parameter<double>(
        "heading_post_turn_armed_cruise_track_min_residual_deg",
        heading_armed_cruise_track_min_residual_deg_);
    heading_post_turn_armed_cruise_track_min_horizontal_speed_mps_ =
      this->declare_parameter<double>(
        "heading_post_turn_armed_cruise_track_min_horizontal_speed_mps",
        4.0);
    heading_post_turn_armed_cruise_track_followthrough_sec_ =
      this->declare_parameter<double>(
        "heading_post_turn_armed_cruise_track_followthrough_sec",
        0.0);
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
    heading_underreaction_force_relock_enable_ =
      this->declare_parameter<bool>("heading_underreaction_force_relock_enable", true);
    heading_underreaction_force_relock_min_residual_deg_ =
      this->declare_parameter<double>("heading_underreaction_force_relock_min_residual_deg", 1.5);
    heading_post_turn_soft_underreaction_min_residual_deg_ =
      this->declare_parameter<double>(
        "heading_post_turn_soft_underreaction_min_residual_deg",
        heading_underreaction_force_relock_min_residual_deg_);
    heading_underreaction_force_relock_min_remaining_ratio_ =
      this->declare_parameter<double>("heading_underreaction_force_relock_min_remaining_ratio", 0.85);
    heading_underreaction_force_relock_yaw_std_deg_ =
      this->declare_parameter<double>("heading_underreaction_force_relock_yaw_std_deg", 2.0);
    heading_underreaction_force_relock_max_rate_hz_ =
      this->declare_parameter<double>("heading_underreaction_force_relock_max_rate_hz", 2.0);
    heading_underreaction_force_relock_max_gyro_deg_s_ =
      this->declare_parameter<double>("heading_underreaction_force_relock_max_gyro_deg_s", 8.0);
    heading_underreaction_force_relock_max_source_yaw_rate_deg_s_ =
      this->declare_parameter<double>("heading_underreaction_force_relock_max_source_yaw_rate_deg_s", 8.0);
    tilt_force_relock_enable_ =
      this->declare_parameter<bool>("tilt_force_relock_enable", false);
    tilt_force_relock_min_residual_deg_ =
      this->declare_parameter<double>("tilt_force_relock_min_residual_deg", 2.0);
    tilt_force_relock_roll_pitch_std_deg_ =
      this->declare_parameter<double>("tilt_force_relock_roll_pitch_std_deg", 1.5);
    tilt_force_relock_once_per_motion_context_ =
      this->declare_parameter<bool>("tilt_force_relock_once_per_motion_context", true);
    tilt_force_relock_max_rate_hz_ =
      this->declare_parameter<double>("tilt_force_relock_max_rate_hz", 2.0);
    tilt_force_relock_max_gyro_deg_s_ =
      this->declare_parameter<double>("tilt_force_relock_max_gyro_deg_s", 8.0);
    tilt_force_relock_max_vertical_speed_mps_ =
      this->declare_parameter<double>("tilt_force_relock_max_vertical_speed_mps", 1.5);
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
    fallback_to_gnss_pose_on_large_core_diff_ =
      this->declare_parameter<bool>("fallback_to_gnss_pose_on_large_core_diff", false);
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
    odom_raw_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("kf_gins/odom_raw", 10);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("kf_gins/path", 10);
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("kf_gins/pose", 10);
    reset_event_pub_ = this->create_publisher<std_msgs::msg::UInt32>("kf_gins/reset_event", 10);
    fallback_active_pub_ = this->create_publisher<std_msgs::msg::Bool>("kf_gins/fallback_active", 10);
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

    auto mavros_imu_qos = rclcpp::SensorDataQoS().keep_last(200);
    auto px4_imu_qos = rclcpp::SensorDataQoS().keep_last(px4_imu_qos_depth_);
    if (imu_source_ == "mavros_raw") {
      active_imu_topic_name_ = imu_topic_;
      imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, mavros_imu_qos, std::bind(&KFGinsNativeNode::imuCb, this, _1));
    } else if (imu_source_ == "px4_sensor_combined") {
      active_imu_topic_name_ = px4_sensor_combined_topic_;
      if (imu_is_delta_) {
        RCLCPP_WARN(
          get_logger(),
          "imu_source=px4_sensor_combined publishes averaged rates/accelerations, forcing imu_is_delta=false semantics at runtime.");
      }
      px4_sensor_combined_sub_ = this->create_subscription<px4_msgs::msg::SensorCombined>(
        px4_sensor_combined_topic_, px4_imu_qos,
        std::bind(&KFGinsNativeNode::px4SensorCombinedCb, this, _1));
    } else if (imu_source_ == "px4_vehicle_imu") {
      active_imu_topic_name_ = px4_vehicle_imu_topic_;
      px4_vehicle_imu_sub_ = this->create_subscription<px4_msgs::msg::VehicleImu>(
        px4_vehicle_imu_topic_, px4_imu_qos,
        std::bind(&KFGinsNativeNode::px4VehicleImuCb, this, _1));
    } else {
      throw std::runtime_error("Unsupported imu_source: " + imu_source_);
    }

    auto gnss_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
    auto px4_gnss_qos = rclcpp::SensorDataQoS().keep_last(20);
    if (gnss_source_ == "navsatfix") {
      active_gnss_topic_name_ = gnss_topic_;
      gnss_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        gnss_topic_, gnss_qos, std::bind(&KFGinsNativeNode::gnssCb, this, _1));
      if (enable_gnss_velocity_update_ && enable_native_sensor_gps_velocity_aid_) {
        px4_sensor_gps_velocity_sub_ = this->create_subscription<px4_msgs::msg::SensorGps>(
          px4_sensor_gps_topic_, px4_gnss_qos,
          std::bind(&KFGinsNativeNode::px4SensorGpsVelocityAuxCb, this, _1));
      }
    } else if (gnss_source_ == "px4_sensor_gps") {
      active_gnss_topic_name_ = px4_sensor_gps_topic_;
      px4_sensor_gps_sub_ = this->create_subscription<px4_msgs::msg::SensorGps>(
        px4_sensor_gps_topic_, px4_gnss_qos,
        std::bind(&KFGinsNativeNode::px4SensorGpsCb, this, _1));
    } else if (gnss_source_ == "px4_vehicle_global_position") {
      active_gnss_topic_name_ = px4_vehicle_global_position_topic_;
      px4_vehicle_global_position_sub_ =
        this->create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
          px4_vehicle_global_position_topic_, px4_gnss_qos,
          std::bind(&KFGinsNativeNode::px4VehicleGlobalPositionCb, this, _1));
    } else if (gnss_source_ == "px4_vehicle_local_position") {
      active_gnss_topic_name_ = px4_vehicle_local_position_topic_;
      px4_vehicle_local_position_gnss_sub_ =
        this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
          px4_vehicle_local_position_topic_, px4_gnss_qos,
          std::bind(&KFGinsNativeNode::px4VehicleLocalPositionGnssCb, this, _1));
    } else {
      throw std::runtime_error("Unsupported gnss_source: " + gnss_source_);
    }

    // MAVROS /state 在当前链路中与其它节点更兼容的 QoS 是 BEST_EFFORT。
    // 之前这里用 RELIABLE 时，kf_gins_node 很可能完全收不到 armed 翻转，
    // 导致飞行阶段仍执行 disarmed 逻辑（ZUPT / GNSS pose fallback）。
    auto state_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
    mavros_state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
      mavros_state_topic_, state_qos, std::bind(&KFGinsNativeNode::mavrosStateCb, this, _1));

    if (speed_source_ == "mavros_local_velocity") {
      active_speed_topic_name_ = mavros_local_velocity_topic_;
      mavros_local_velocity_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        mavros_local_velocity_topic_, state_qos,
        [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
          const auto & v = msg->twist.linear;
          updateSpeedSample_(v.x, v.y, v.z, this->now().seconds());
        });
    } else if (speed_source_ == "px4_vehicle_local_position") {
      active_speed_topic_name_ = px4_vehicle_local_position_topic_;
      auto px4_state_qos = rclcpp::SensorDataQoS().keep_last(50);
      px4_vehicle_local_position_sub_ =
        this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
          px4_vehicle_local_position_topic_, px4_state_qos,
          [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
            if (!msg->v_xy_valid && !msg->v_z_valid) return;
            const double vx = msg->v_xy_valid ? static_cast<double>(msg->vx) : 0.0;
            const double vy = msg->v_xy_valid ? static_cast<double>(msg->vy) : 0.0;
            const double vz = msg->v_z_valid ? static_cast<double>(msg->vz) : 0.0;
            updateSpeedSample_(vx, vy, vz, this->now().seconds());
          });
    } else if (speed_source_ == "px4_vehicle_odometry") {
      active_speed_topic_name_ = px4_vehicle_odometry_topic_;
    } else {
      throw std::runtime_error("Unsupported speed_source: " + speed_source_);
    }

    // 订阅 MAVROS IMU 获取 EKF2 的磁力计融合航向
    // KF-GINS 没有磁力计，需要从 PX4 EKF2 获取初始航向
    auto mavros_heading_qos = rclcpp::SensorDataQoS().keep_last(10);
    if (heading_source_ == "mavros_imu") {
      active_heading_topic_name_ = mavros_heading_topic_;
      mavros_imu_heading_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        mavros_heading_topic_, mavros_heading_qos,
        [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
          if (msg->orientation.w == 0.0 && msg->orientation.x == 0.0 &&
              msg->orientation.y == 0.0 && msg->orientation.z == 0.0) {
            return;
          }
          tf2::Quaternion q_enu_flu(
            msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
          const tf2::Matrix3x3 r_enu_from_flu(q_enu_flu);
          const tf2::Matrix3x3 t_ned_from_enu(
            0.0, 1.0,  0.0,
            1.0, 0.0,  0.0,
            0.0, 0.0, -1.0);
          const tf2::Matrix3x3 t_flu_from_frd(
            1.0,  0.0,  0.0,
            0.0, -1.0,  0.0,
            0.0,  0.0, -1.0);
          const tf2::Matrix3x3 r_ned_from_frd =
            t_ned_from_enu * r_enu_from_flu * t_flu_from_frd;
          double roll_ned, pitch_ned, yaw_ned;
          r_ned_from_frd.getRPY(roll_ned, pitch_ned, yaw_ned);
          updateAttitudeSample_(
            roll_ned * 180.0 / M_PI,
            pitch_ned * 180.0 / M_PI,
            normalizeAngleDeg_(yaw_ned * 180.0 / M_PI),
            this->now().seconds());
        });
    } else if (heading_source_ == "px4_vehicle_attitude") {
      active_heading_topic_name_ = px4_vehicle_attitude_topic_;
      auto px4_heading_qos = rclcpp::SensorDataQoS().keep_last(20);
      px4_vehicle_attitude_sub_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
        px4_vehicle_attitude_topic_, px4_heading_qos,
        [this](const px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
          if (!std::isfinite(msg->q[0]) || !std::isfinite(msg->q[1]) ||
              !std::isfinite(msg->q[2]) || !std::isfinite(msg->q[3])) {
            return;
          }
          tf2::Quaternion q(msg->q[1], msg->q[2], msg->q[3], msg->q[0]);
          double roll_ned, pitch_ned, yaw_ned;
          tf2::Matrix3x3(q).getRPY(roll_ned, pitch_ned, yaw_ned);
          updateAttitudeSample_(
            roll_ned * 180.0 / M_PI,
            pitch_ned * 180.0 / M_PI,
            normalizeAngleDeg_(yaw_ned * 180.0 / M_PI),
            this->now().seconds());
        });
    } else if (heading_source_ == "px4_vehicle_odometry") {
      active_heading_topic_name_ = px4_vehicle_odometry_topic_;
    } else {
      throw std::runtime_error("Unsupported heading_source: " + heading_source_);
    }

    const bool need_vehicle_odometry_aux =
      speed_source_ == "px4_vehicle_odometry" || heading_source_ == "px4_vehicle_odometry";
    if (need_vehicle_odometry_aux) {
      auto px4_odometry_qos = rclcpp::SensorDataQoS().keep_last(20);
      px4_vehicle_odometry_aux_sub_ =
        this->create_subscription<px4_msgs::msg::VehicleOdometry>(
          px4_vehicle_odometry_topic_, px4_odometry_qos,
          [this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
            const bool need_heading = heading_source_ == "px4_vehicle_odometry";
            const bool need_speed = speed_source_ == "px4_vehicle_odometry";
            if (!need_heading && !need_speed) return;

            if (!std::isfinite(msg->q[0]) || !std::isfinite(msg->q[1]) ||
                !std::isfinite(msg->q[2]) || !std::isfinite(msg->q[3])) {
              return;
            }

            tf2::Quaternion q_ned(msg->q[1], msg->q[2], msg->q[3], msg->q[0]);
            tf2::Matrix3x3 q_ned_matrix(q_ned);

            if (need_heading) {
              double roll_ned, pitch_ned, yaw_ned;
              q_ned_matrix.getRPY(roll_ned, pitch_ned, yaw_ned);
              updateAttitudeSample_(
                roll_ned * 180.0 / M_PI,
                pitch_ned * 180.0 / M_PI,
                normalizeAngleDeg_(yaw_ned * 180.0 / M_PI),
                this->now().seconds());
            }

            if (need_speed) {
              const auto & velocity = msg->velocity;
              if (!std::isfinite(velocity[0]) || !std::isfinite(velocity[1]) || !std::isfinite(velocity[2])) {
                return;
              }

              double vx_ned = std::numeric_limits<double>::quiet_NaN();
              double vy_ned = std::numeric_limits<double>::quiet_NaN();
              double vz_ned = std::numeric_limits<double>::quiet_NaN();

              if (msg->velocity_frame == px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED) {
                vx_ned = static_cast<double>(velocity[0]);
                vy_ned = static_cast<double>(velocity[1]);
                vz_ned = static_cast<double>(velocity[2]);
              } else if (
                msg->velocity_frame == px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_BODY_FRD) {
                const tf2::Vector3 body_frd(
                  static_cast<double>(velocity[0]),
                  static_cast<double>(velocity[1]),
                  static_cast<double>(velocity[2]));
                const tf2::Vector3 vel_ned = q_ned_matrix * body_frd;
                vx_ned = vel_ned.x();
                vy_ned = vel_ned.y();
                vz_ned = vel_ned.z();
              } else if (!warned_vehicle_odometry_velocity_frame_) {
                RCLCPP_WARN(
                  get_logger(),
                  "Unsupported px4_vehicle_odometry velocity_frame=%u on topic=%s",
                  static_cast<unsigned>(msg->velocity_frame),
                  px4_vehicle_odometry_topic_.c_str());
                warned_vehicle_odometry_velocity_frame_ = true;
              }

              if (std::isfinite(vx_ned) && std::isfinite(vy_ned) && std::isfinite(vz_ned)) {
                updateSpeedSample_(vx_ned, vy_ned, vz_ned, this->now().seconds());
              }
            }
          });
    }

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
      "IMU time settings: source=%s, topic=%s, input_is_flu=%s, px4_imu_qos_depth=%d, use_node_time_for_core=%s, "
      "use_integrated_time_for_core=%s, use_steady_time_for_imu_dt=%s, "
      "max_imu_dt_sec=%.3f, skip_medium_imu_gap_when_turning=%s, imu_gap_turn_rate_gate_deg_s=%.2f, "
      "imu_gap_vertical_speed_gate_mps=%.2f, imu_gap_accel_deviation_gate_mps2=%.2f, "
      "imu_gap_maneuver_cooldown_sec=%.2f, "
      "skip_large_imu_gap_samples=%s, severe_imu_gap_reset_sec=%.3f, "
      "delta_imu_source_gap_bridge=%s (min=%.3f s, ratio=%.2f, max=%.3f s, reset=%.3f s), "
      "source_gap_clamp=%s (min=%.3f s, ratio=%.2f, recv<=%.3f s), "
      "sensor_combined_source_gap_diag=%s (min=%.3f s, ratio=%.2f, reset=%.3f s), "
      "speed_source=%s topic=%s, heading_source=%s topic=%s, heading_horiz_min=%.2f, heading_vert_max=%.2f, "
      "heading_src_yaw_rate_max=%.2f deg/s, publish_max_core_gnss_diff_m=%.2f",
      imu_source_.c_str(),
      active_imu_topic_name_.c_str(),
      imu_input_is_flu_ ? "true" : "false",
      px4_imu_qos_depth_,
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
      speed_source_.c_str(),
      active_speed_topic_name_.c_str(),
      heading_source_.c_str(),
      active_heading_topic_name_.c_str(),
      heading_update_armed_min_horizontal_speed_mps_,
      heading_update_max_vertical_speed_mps_,
      heading_update_max_source_yaw_rate_deg_s_,
      publish_max_core_gnss_diff_m_);
    if (fallback_to_gnss_pose_on_large_core_diff_) {
      RCLCPP_INFO(
        get_logger(),
        "fallback_to_gnss_pose_on_large_core_diff enabled: /kf_gins/odom uses GNSS pose when core-vs-GNSS diff exceeds %.2f m.",
        publish_max_core_gnss_diff_m_);
    }
    RCLCPP_INFO(
      get_logger(),
      "GNSS settings: source=%s topic=%s",
      gnss_source_.c_str(),
      active_gnss_topic_name_.c_str());
    openGnssUpdateDebugCsv_();
    openHeadingUpdateDebugCsv_();
    openStatePublishDebugCsv_();

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
    const double now_sec = now().seconds();
    mavros_armed_ = msg->armed;

    if (prev_armed != mavros_armed_) {
      last_disarmed_yaw_lock_time_sec_ = std::numeric_limits<double>::quiet_NaN();
      last_heading_attempt_time_sec_ = std::numeric_limits<double>::quiet_NaN();
      last_heading_update_time_sec_ = std::numeric_limits<double>::quiet_NaN();
      last_heading_recovery_time_sec_ = std::numeric_limits<double>::quiet_NaN();
      last_turning_heading_time_sec_ = std::numeric_limits<double>::quiet_NaN();
      last_heading_underreaction_force_relock_time_sec_ = std::numeric_limits<double>::quiet_NaN();
      last_tilt_force_relock_time_sec_ = std::numeric_limits<double>::quiet_NaN();
      active_tilt_force_relock_context_id_ = 0;
      tilt_force_relock_applied_in_active_context_ = false;
      last_post_turn_reacquire_time_sec_ = std::numeric_limits<double>::quiet_NaN();
      last_post_turn_reacquire_apply_time_sec_ = std::numeric_limits<double>::quiet_NaN();
      post_turn_hold_end_time_sec_ = std::numeric_limits<double>::quiet_NaN();
      post_turn_cruise_track_continue_until_sec_ = std::numeric_limits<double>::quiet_NaN();
      post_turn_armed_cruise_track_until_sec_ = std::numeric_limits<double>::quiet_NaN();
      post_turn_armed_cruise_track_followthrough_until_sec_ =
        std::numeric_limits<double>::quiet_NaN();
      post_turn_armed_cruise_track_followthrough_remaining_ = 0;
      last_post_turn_force_relock_time_sec_ = std::numeric_limits<double>::quiet_NaN();
      heading_large_residual_skip_count_ = 0;
      post_turn_blocked_count_ = 0;
    }

    if (!prev_armed && mavros_armed_) {
      have_completed_armed_flight_since_reset_ = true;
      last_armed_transition_time_sec_ = now_sec;
      last_disarmed_transition_time_sec_ = std::numeric_limits<double>::quiet_NaN();
      post_flight_vertical_cov_reopen_until_sec_ = std::numeric_limits<double>::quiet_NaN();
    } else if (prev_armed && !mavros_armed_) {
      last_disarmed_transition_time_sec_ = now_sec;
      if (post_flight_vertical_cov_reopen_enable_ && have_completed_armed_flight_since_reset_) {
        post_flight_vertical_cov_reopen_until_sec_ =
          now_sec + std::max(0.0, post_flight_vertical_cov_reopen_hold_sec_);
      } else {
        post_flight_vertical_cov_reopen_until_sec_ = std::numeric_limits<double>::quiet_NaN();
      }
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

  bool trySyncHeadingSampleBeforeFirstReset_()
  {
    if (have_mavros_heading_) {
      return true;
    }

    rclcpp::NodeOptions wait_node_options;
    wait_node_options.context(this->get_node_base_interface()->get_context());
    wait_node_options.use_intra_process_comms(false);
    wait_node_options.enable_rosout(false);
    auto wait_node = std::make_shared<rclcpp::Node>("kfgins_initial_heading_waiter", wait_node_options);

    if (heading_source_ == "mavros_imu") {
      sensor_msgs::msg::Imu msg;
      auto qos = rclcpp::SensorDataQoS().keep_last(10);
      if (!rclcpp::wait_for_message(
            msg, wait_node, mavros_heading_topic_, 300ms, qos)) {
        return false;
      }
      if (msg.orientation.w == 0.0 && msg.orientation.x == 0.0 &&
          msg.orientation.y == 0.0 && msg.orientation.z == 0.0) {
        return false;
      }
      tf2::Quaternion q(
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w);
      double roll_enu, pitch_enu, yaw_enu;
      tf2::Matrix3x3(q).getRPY(roll_enu, pitch_enu, yaw_enu);
      double yaw_ned = M_PI / 2.0 - yaw_enu;
      while (yaw_ned > M_PI) yaw_ned -= 2.0 * M_PI;
      while (yaw_ned < -M_PI) yaw_ned += 2.0 * M_PI;
      updateHeadingSample_(yaw_ned * 180.0 / M_PI, this->now().seconds());
      RCLCPP_INFO(
        get_logger(),
        "Synced initial heading before first GNSS reset: source=%s topic=%s yaw_ned=%.2f deg",
        heading_source_.c_str(),
        mavros_heading_topic_.c_str(),
        normalizeAngleDeg_(yaw_ned * 180.0 / M_PI));
      return true;
    }

    if (heading_source_ == "px4_vehicle_attitude") {
      px4_msgs::msg::VehicleAttitude msg;
      auto qos = rclcpp::SensorDataQoS().keep_last(20);
      if (!rclcpp::wait_for_message(
            msg, wait_node, px4_vehicle_attitude_topic_, 300ms, qos)) {
        return false;
      }
      if (!std::isfinite(msg.q[0]) || !std::isfinite(msg.q[1]) ||
          !std::isfinite(msg.q[2]) || !std::isfinite(msg.q[3])) {
        return false;
      }
      tf2::Quaternion q(msg.q[1], msg.q[2], msg.q[3], msg.q[0]);
      double roll_ned, pitch_ned, yaw_ned;
      tf2::Matrix3x3(q).getRPY(roll_ned, pitch_ned, yaw_ned);
      updateHeadingSample_(normalizeAngleDeg_(yaw_ned * 180.0 / M_PI), this->now().seconds());
      RCLCPP_INFO(
        get_logger(),
        "Synced initial heading before first GNSS reset: source=%s topic=%s yaw_ned=%.2f deg",
        heading_source_.c_str(),
        px4_vehicle_attitude_topic_.c_str(),
        normalizeAngleDeg_(yaw_ned * 180.0 / M_PI));
      return true;
    }

    if (heading_source_ == "px4_vehicle_odometry") {
      px4_msgs::msg::VehicleOdometry msg;
      auto qos = rclcpp::SensorDataQoS().keep_last(20);
      if (!rclcpp::wait_for_message(
            msg, wait_node, px4_vehicle_odometry_topic_, 300ms, qos)) {
        return false;
      }
      if (!std::isfinite(msg.q[0]) || !std::isfinite(msg.q[1]) ||
          !std::isfinite(msg.q[2]) || !std::isfinite(msg.q[3])) {
        return false;
      }
      tf2::Quaternion q(msg.q[1], msg.q[2], msg.q[3], msg.q[0]);
      double roll_ned, pitch_ned, yaw_ned;
      tf2::Matrix3x3(q).getRPY(roll_ned, pitch_ned, yaw_ned);
      updateHeadingSample_(normalizeAngleDeg_(yaw_ned * 180.0 / M_PI), this->now().seconds());
      RCLCPP_INFO(
        get_logger(),
        "Synced initial heading before first GNSS reset: source=%s topic=%s yaw_ned=%.2f deg",
        heading_source_.c_str(),
        px4_vehicle_odometry_topic_.c_str(),
        normalizeAngleDeg_(yaw_ned * 180.0 / M_PI));
      return true;
    }

    return false;
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

    const bool input_gap = std::isfinite(input_dt_sec) && input_dt_sec > warn_gap_sec;
    const bool recv_gap = std::isfinite(recv_dt_sec) && recv_dt_sec > warn_gap_sec;

    if (input_gap) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Raw IMU gap: input_dt=%.2fms recv_dt=%.2fms topic=%s",
        std::isfinite(input_dt_sec) ? input_dt_sec * 1000.0 : -1.0,
        std::isfinite(recv_dt_sec) ? recv_dt_sec * 1000.0 : -1.0,
        active_imu_topic_name_.c_str());
    } else if (recv_gap) {
      RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Raw IMU recv-only stall ignored: input_dt=%.2fms recv_dt=%.2fms topic=%s",
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
    last_gnss_source_time_sec_ = std::numeric_limits<double>::quiet_NaN();
    last_gnss_rx_ros_time_sec_ = std::numeric_limits<double>::quiet_NaN();
    have_prev_gnss_for_vel_ = false;
    last_zupt_reset_time_sec_ = std::numeric_limits<double>::quiet_NaN();
    last_heading_attempt_time_sec_ = std::numeric_limits<double>::quiet_NaN();
    last_heading_update_time_sec_ = std::numeric_limits<double>::quiet_NaN();
    last_heading_recovery_time_sec_ = std::numeric_limits<double>::quiet_NaN();
    last_turning_heading_time_sec_ = std::numeric_limits<double>::quiet_NaN();
    last_heading_underreaction_force_relock_time_sec_ = std::numeric_limits<double>::quiet_NaN();
    last_tilt_force_relock_time_sec_ = std::numeric_limits<double>::quiet_NaN();
    active_tilt_force_relock_context_id_ = 0;
    tilt_force_relock_applied_in_active_context_ = false;
    last_post_turn_reacquire_time_sec_ = std::numeric_limits<double>::quiet_NaN();
    last_post_turn_reacquire_apply_time_sec_ = std::numeric_limits<double>::quiet_NaN();
    post_turn_hold_end_time_sec_ = std::numeric_limits<double>::quiet_NaN();
    post_turn_cruise_track_continue_until_sec_ = std::numeric_limits<double>::quiet_NaN();
    post_turn_armed_cruise_track_until_sec_ = std::numeric_limits<double>::quiet_NaN();
    post_turn_armed_cruise_track_followthrough_until_sec_ =
      std::numeric_limits<double>::quiet_NaN();
    post_turn_armed_cruise_track_followthrough_remaining_ = 0;
    last_post_turn_force_relock_time_sec_ = std::numeric_limits<double>::quiet_NaN();
    heading_large_residual_skip_count_ = 0;
    post_turn_blocked_count_ = 0;
    post_flight_vertical_cov_reopen_until_sec_ = std::numeric_limits<double>::quiet_NaN();

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
    have_completed_armed_flight_since_reset_ = mavros_armed_;
    last_armed_transition_time_sec_ =
      mavros_armed_ ? now().seconds() : std::numeric_limits<double>::quiet_NaN();
    last_disarmed_transition_time_sec_ = std::numeric_limits<double>::quiet_NaN();
    post_flight_vertical_cov_reopen_until_sec_ = std::numeric_limits<double>::quiet_NaN();

    have_prev_imu_ = false;
    have_raw_time_zero_ = false;
    prev_imu_raw_rel_sec_ = 0.0;
    last_imu_input_stamp_sec_ = std::numeric_limits<double>::quiet_NaN();
    have_last_imu_recv_steady_time_ = false;
    core_time_sec_ = 0.0;
    last_core_time_ = -std::numeric_limits<double>::infinity();
    last_gnss_time_sec_ = -std::numeric_limits<double>::infinity();
    last_gnss_source_time_sec_ = std::numeric_limits<double>::quiet_NaN();
    last_gnss_rx_ros_time_sec_ = std::numeric_limits<double>::quiet_NaN();

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
    last_heading_underreaction_force_relock_time_sec_ = std::numeric_limits<double>::quiet_NaN();
    last_tilt_force_relock_time_sec_ = std::numeric_limits<double>::quiet_NaN();
    active_tilt_force_relock_context_id_ = 0;
    tilt_force_relock_applied_in_active_context_ = false;
    last_post_turn_reacquire_time_sec_ = std::numeric_limits<double>::quiet_NaN();
    last_post_turn_reacquire_apply_time_sec_ = std::numeric_limits<double>::quiet_NaN();
    post_turn_hold_end_time_sec_ = std::numeric_limits<double>::quiet_NaN();
    post_turn_cruise_track_continue_until_sec_ = std::numeric_limits<double>::quiet_NaN();
    post_turn_armed_cruise_track_until_sec_ = std::numeric_limits<double>::quiet_NaN();
    post_turn_armed_cruise_track_followthrough_until_sec_ =
      std::numeric_limits<double>::quiet_NaN();
    post_turn_armed_cruise_track_followthrough_remaining_ = 0;
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

    const HeadingMotionContext motion_ctx = buildHeadingMotionContext_(now_sec, true);
    const bool have_fresh_mavros_speed = motion_ctx.have_fresh_mavros_speed;
    const bool armed_motion_ok = motion_ctx.armed_motion_ok;

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
    const bool turning_now = motion_ctx.turning_now;
    const bool recent_turning = motion_ctx.recent_turning;
    const bool post_turn_hold_active = motion_ctx.post_turn_hold_active;
    const bool post_turn_context = motion_ctx.post_turn_context;
    const bool post_turn_tracking_motion_context =
      post_turn_context ||
      motion_ctx.post_turn_cruise_track_continue_active;
    const bool armed_cruise_force_relock_context = motion_ctx.armed_cruise_force_relock_context;
    const bool post_turn_armed_cruise_track_motion_ok =
      armed_cruise_force_relock_context &&
      motion_ctx.armed_cruise_force_relock_gyro_ok &&
      motion_ctx.armed_cruise_force_relock_source_rate_ok &&
      (heading_post_turn_armed_cruise_track_min_horizontal_speed_mps_ <= 0.0 ||
       last_mavros_horizontal_speed_mps_ >=
         heading_post_turn_armed_cruise_track_min_horizontal_speed_mps_);
    const bool post_turn_armed_cruise_track_window_active =
      post_turn_armed_cruise_track_motion_ok &&
      std::isfinite(post_turn_armed_cruise_track_until_sec_) &&
      heading_post_turn_armed_cruise_track_window_sec_ > 0.0 &&
      now_sec <= post_turn_armed_cruise_track_until_sec_;
    const bool post_turn_armed_cruise_track_followthrough_active =
      post_turn_armed_cruise_track_motion_ok &&
      post_turn_armed_cruise_track_followthrough_remaining_ > 0 &&
      std::isfinite(post_turn_armed_cruise_track_followthrough_until_sec_) &&
      heading_post_turn_armed_cruise_track_followthrough_sec_ > 0.0 &&
      now_sec <= post_turn_armed_cruise_track_followthrough_until_sec_;
    const bool post_turn_armed_cruise_track_context =
      post_turn_armed_cruise_track_window_active ||
      post_turn_armed_cruise_track_followthrough_active;
    if (mavros_armed_ && turning_now &&
        std::isfinite(heading_update_turn_innovation_gate_deg_) &&
        heading_update_turn_innovation_gate_deg_ > 0.0) {
      innovation_gate_deg =
        std::max(innovation_gate_deg, heading_update_turn_innovation_gate_deg_);
    } else if (post_turn_tracking_motion_context &&
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
    const bool post_turn_cruise_track_continue_context =
      motion_ctx.post_turn_cruise_track_continue_active &&
      residual_abs_deg >=
        std::max(0.0, heading_post_turn_cruise_track_continue_min_residual_deg_);
    const bool post_turn_tracking_context =
      post_turn_context ||
      post_turn_cruise_track_continue_context;
    const bool risky_armed_plain_heading_context =
      mavros_armed_ &&
      motion_ctx.vertical_dominant_low_horizontal_motion &&
      !motion_ctx.post_turn_cruise_motion_ok &&
      !armed_cruise_force_relock_context;
    const double risky_armed_plain_heading_skip_min_residual_deg =
      std::max(
        0.0,
        post_turn_context
          ? heading_post_turn_force_relock_min_residual_deg_
          : heading_underreaction_force_relock_min_residual_deg_);
    if (risky_armed_plain_heading_context &&
        residual_abs_deg >= risky_armed_plain_heading_skip_min_residual_deg) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Skip heading update: risky low-horizontal / vertical-dominant armed window "
        "(residual=%.2f deg, turning=%s, recent_turning=%s, horiz=%.2f m/s, vert=%.2f m/s, "
        "gyro=%.2f deg/s, source_yaw_rate=%.2f deg/s)",
        yaw_residual_deg,
        turning_now ? "true" : "false",
        recent_turning ? "true" : "false",
        last_mavros_horizontal_speed_mps_,
        last_mavros_vertical_speed_mps_,
        last_imu_gyro_norm_deg_s_,
        last_mavros_heading_rate_deg_s_);
      return;
    }
    double heading_measurement_deg = raw_heading_deg;
    double heading_measurement_std_deg = heading_update_std_deg_;
    bool recovery_update = false;
    bool post_turn_reacquire_update = false;
    const char* heading_mode = "update";

    // 正常 innovation gate 内也允许在关键阶段加大 heading 观测权重：
    // 1. turn/post-turn 段快速把姿态重新拉回 source，避免 turn 后 residual 留在几度量级；
    // 2. armed cruise 段 residual 累积到阈值后，主动提高 heading 约束，抑制后续 XY 漂移。
    if (mavros_armed_) {
      if (turning_now &&
          (motion_ctx.turn_track_motion_ok ||
           residual_abs_deg < std::max(0.0, heading_post_turn_force_relock_min_residual_deg_))) {
        const double turn_track_std_deg =
          std::max(0.1, std::abs(heading_post_turn_reacquire_std_deg_));
        if (turn_track_std_deg < heading_measurement_std_deg) {
          heading_measurement_std_deg = turn_track_std_deg;
          heading_mode = "turn_track";
        }
      } else if (post_turn_tracking_context && motion_ctx.post_turn_track_motion_ok) {
        const bool post_turn_cruise_track_context =
          motion_ctx.post_turn_cruise_motion_ok || post_turn_cruise_track_continue_context;
        const double post_turn_track_std_deg =
          std::max(
            0.1,
            std::abs(
              post_turn_cruise_track_context
                ? heading_post_turn_cruise_track_std_deg_
                : heading_post_turn_reacquire_std_deg_));
        if (post_turn_track_std_deg < heading_measurement_std_deg) {
          heading_measurement_std_deg = post_turn_track_std_deg;
          if (post_turn_cruise_track_continue_context) {
            heading_mode = "post_turn_cruise_track_continue";
          } else if (post_turn_cruise_track_context) {
            heading_mode = "post_turn_cruise_track";
          } else {
            heading_mode = "post_turn_track";
          }
        }
      }
    }

    // manual42 still showed repeated cruise-like drift growing from roughly
    // 0.9 deg into generic armed-cruise relock. Apply the tighter cruise
    // track earlier, but only inside the true cruise-like motion context.
    double armed_cruise_track_min_residual_deg =
      std::max(0.0, heading_armed_cruise_track_min_residual_deg_);
    if (post_turn_armed_cruise_track_context) {
      armed_cruise_track_min_residual_deg =
        std::min(
          armed_cruise_track_min_residual_deg,
          std::max(0.0, heading_post_turn_armed_cruise_track_min_residual_deg_));
    }
    if (armed_cruise_force_relock_context &&
        residual_abs_deg >= armed_cruise_track_min_residual_deg) {
      const double cruise_track_std_deg =
        std::max(0.1, std::abs(heading_armed_cruise_force_relock_yaw_std_deg_));
      if (cruise_track_std_deg < heading_measurement_std_deg) {
        heading_measurement_std_deg = cruise_track_std_deg;
        heading_mode = "armed_cruise_track";
      }
    }

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
        motion_ctx.armed_cruise_force_relock_gyro_ok;
      const bool armed_cruise_force_relock_source_rate_ok =
        motion_ctx.armed_cruise_force_relock_source_rate_ok;
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
    const double residual_after_deg =
      shortestAngleDiffDeg_(raw_heading_deg, core_yaw_after_deg);
    const double residual_after_abs_deg = std::abs(residual_after_deg);
    const double yaw_correction_abs_deg =
      std::abs(shortestAngleDiffDeg_(core_yaw_after_deg, core_yaw_before_deg));
    const bool used_post_turn_cruise_track =
      std::strcmp(heading_mode, "post_turn_cruise_track") == 0;
    const bool used_armed_cruise_track =
      std::strcmp(heading_mode, "armed_cruise_track") == 0;
    const bool post_turn_armed_cruise_track_window_active_before =
      post_turn_armed_cruise_track_window_active;
    const bool post_turn_armed_cruise_track_followthrough_active_before =
      post_turn_armed_cruise_track_followthrough_active;
    const bool post_turn_armed_cruise_track_context_before =
      post_turn_armed_cruise_track_context;
    const double post_turn_armed_cruise_track_until_before_sec =
      post_turn_armed_cruise_track_until_sec_;
    const double post_turn_armed_cruise_track_followthrough_until_before_sec =
      post_turn_armed_cruise_track_followthrough_until_sec_;
    const int post_turn_armed_cruise_track_followthrough_remaining_before =
      post_turn_armed_cruise_track_followthrough_remaining_;
    const bool post_turn_cruise_track_continue_needed =
      mavros_armed_ &&
      recent_turning &&
      heading_post_turn_cruise_track_continue_sec_ > 0.0 &&
      used_post_turn_cruise_track &&
      motion_ctx.post_turn_cruise_motion_ok &&
      residual_abs_deg >=
        std::max(0.0, heading_post_turn_cruise_track_continue_min_residual_deg_);
    if (post_turn_cruise_track_continue_needed) {
      post_turn_cruise_track_continue_until_sec_ =
        now_sec + heading_post_turn_cruise_track_continue_sec_;
    }
    if (used_post_turn_cruise_track &&
        heading_post_turn_armed_cruise_track_window_sec_ > 0.0) {
      post_turn_armed_cruise_track_until_sec_ =
        now_sec + heading_post_turn_armed_cruise_track_window_sec_;
      post_turn_armed_cruise_track_followthrough_until_sec_ =
        std::numeric_limits<double>::quiet_NaN();
      post_turn_armed_cruise_track_followthrough_remaining_ = 0;
    }
    if (used_armed_cruise_track) {
      if (post_turn_armed_cruise_track_followthrough_active) {
        post_turn_armed_cruise_track_followthrough_remaining_ =
          std::max(0, post_turn_armed_cruise_track_followthrough_remaining_ - 1);
        if (post_turn_armed_cruise_track_followthrough_remaining_ == 0) {
          post_turn_armed_cruise_track_followthrough_until_sec_ =
            std::numeric_limits<double>::quiet_NaN();
        }
      } else if (post_turn_armed_cruise_track_window_active &&
          heading_post_turn_armed_cruise_track_followthrough_sec_ > 0.0) {
        post_turn_armed_cruise_track_followthrough_until_sec_ =
          now_sec + heading_post_turn_armed_cruise_track_followthrough_sec_;
        post_turn_armed_cruise_track_followthrough_remaining_ = 1;
      }
      post_turn_armed_cruise_track_until_sec_ =
        std::numeric_limits<double>::quiet_NaN();
    }
    const bool post_turn_armed_cruise_track_window_active_after =
      post_turn_armed_cruise_track_motion_ok &&
      std::isfinite(post_turn_armed_cruise_track_until_sec_) &&
      heading_post_turn_armed_cruise_track_window_sec_ > 0.0 &&
      now_sec <= post_turn_armed_cruise_track_until_sec_;
    const bool post_turn_armed_cruise_track_followthrough_active_after =
      post_turn_armed_cruise_track_motion_ok &&
      post_turn_armed_cruise_track_followthrough_remaining_ > 0 &&
      std::isfinite(post_turn_armed_cruise_track_followthrough_until_sec_) &&
      heading_post_turn_armed_cruise_track_followthrough_sec_ > 0.0 &&
      now_sec <= post_turn_armed_cruise_track_followthrough_until_sec_;
    const bool post_turn_armed_cruise_track_context_after =
      post_turn_armed_cruise_track_window_active_after ||
      post_turn_armed_cruise_track_followthrough_active_after;
    const double underreaction_min_period_sec =
      heading_underreaction_force_relock_max_rate_hz_ > 0.0
        ? 1.0 / heading_underreaction_force_relock_max_rate_hz_
        : 0.0;
    const bool underreaction_rate_ok =
      !std::isfinite(last_heading_underreaction_force_relock_time_sec_) ||
      (now_sec - last_heading_underreaction_force_relock_time_sec_) >=
        underreaction_min_period_sec;
    const bool underreaction_gyro_ok =
      !std::isfinite(last_imu_gyro_norm_deg_s_) ||
      heading_underreaction_force_relock_max_gyro_deg_s_ <= 0.0 ||
      last_imu_gyro_norm_deg_s_ <= heading_underreaction_force_relock_max_gyro_deg_s_;
    const bool underreaction_source_rate_ok =
      !std::isfinite(last_mavros_heading_rate_deg_s_) ||
      heading_underreaction_force_relock_max_source_yaw_rate_deg_s_ <= 0.0 ||
      std::abs(last_mavros_heading_rate_deg_s_) <=
        heading_underreaction_force_relock_max_source_yaw_rate_deg_s_;
    // Underreaction relock is useful in stable armed segments, but manual21
    // and manual23 showed that letting it fire anywhere inside post-turn
    // hold/recent-turn windows creates regressions, especially in climb/descent
    // edges that are not really settled cruise. Keep the earlier post-turn
    // threshold only for recent-turn samples that already look like stable
    // cruise motion. Softer recent-turn samples still benefit from a
    // follow-up relock, but manual28 showed they need a higher residual gate
    // than steady cruise. manual30 then showed the remaining generic path also
    // needs to stay inside stable armed-cruise motion; low-speed vertical
    // edges should not reuse the steady-cruise 1.0 deg relock.
    const bool underreaction_in_post_turn_context =
      heading_post_turn_force_relock_enable_ &&
      recent_turning &&
      motion_ctx.post_turn_cruise_motion_ok;
    const bool soft_post_turn_underreaction_context =
      heading_underreaction_force_relock_enable_ &&
      recent_turning &&
      motion_ctx.post_turn_underreaction_motion_ok &&
      !motion_ctx.post_turn_cruise_motion_ok;
    const bool generic_underreaction_allowed =
      heading_underreaction_force_relock_enable_ &&
      !recent_turning &&
      armed_cruise_force_relock_context;
    double underreaction_min_residual_deg =
      std::max(0.0, heading_underreaction_force_relock_min_residual_deg_);
    if (soft_post_turn_underreaction_context) {
      underreaction_min_residual_deg =
        std::max(0.0, heading_post_turn_soft_underreaction_min_residual_deg_);
    }
    if (underreaction_in_post_turn_context) {
      underreaction_min_residual_deg =
        std::max(0.0, heading_post_turn_force_relock_min_residual_deg_);
    }
    const bool heading_update_underreacted =
      mavros_armed_ &&
      !turning_now &&
      (underreaction_in_post_turn_context ||
       soft_post_turn_underreaction_context ||
       generic_underreaction_allowed) &&
      residual_abs_deg >= underreaction_min_residual_deg &&
      residual_after_abs_deg >=
        residual_abs_deg * std::clamp(heading_underreaction_force_relock_min_remaining_ratio_, 0.0, 1.0) &&
      underreaction_rate_ok &&
      underreaction_gyro_ok &&
      underreaction_source_rate_ok;

    HeadingUpdateDebugEvent heading_debug_event;
    heading_debug_event.event_type = "ingest";
    heading_debug_event.heading_mode = heading_mode;
    heading_debug_event.ros_time_sec = now_sec;
    heading_debug_event.update_time_sec = t_sec;
    heading_debug_event.residual_before_deg = yaw_residual_deg;
    heading_debug_event.residual_after_deg = residual_after_deg;
    heading_debug_event.core_yaw_before_deg = core_yaw_before_deg;
    heading_debug_event.core_yaw_after_deg = core_yaw_after_deg;
    heading_debug_event.raw_heading_deg = raw_heading_deg;
    heading_debug_event.measurement_heading_deg = heading_measurement_deg;
    heading_debug_event.measurement_std_deg = heading_measurement_std_deg;
    heading_debug_event.innovation_gate_deg = innovation_gate_deg;
    heading_debug_event.yaw_correction_abs_deg = yaw_correction_abs_deg;
    heading_debug_event.armed = mavros_armed_;
    heading_debug_event.turning_now = turning_now;
    heading_debug_event.recent_turning = recent_turning;
    heading_debug_event.post_turn_hold_active = post_turn_hold_active;
    heading_debug_event.post_turn_context = post_turn_context;
    heading_debug_event.post_turn_track_motion_ok = motion_ctx.post_turn_track_motion_ok;
    heading_debug_event.post_turn_cruise_motion_ok = motion_ctx.post_turn_cruise_motion_ok;
    heading_debug_event.post_turn_low_hspeed_cluster_ok =
      motion_ctx.post_turn_low_hspeed_cluster_ok;
    heading_debug_event.post_turn_cruise_track_continue_active =
      motion_ctx.post_turn_cruise_track_continue_active;
    heading_debug_event.armed_cruise_context = armed_cruise_force_relock_context;
    heading_debug_event.armed_cruise_gyro_ok = motion_ctx.armed_cruise_force_relock_gyro_ok;
    heading_debug_event.armed_cruise_source_rate_ok =
      motion_ctx.armed_cruise_force_relock_source_rate_ok;
    heading_debug_event.post_turn_armed_cruise_track_motion_ok =
      post_turn_armed_cruise_track_motion_ok;
    heading_debug_event.post_turn_window_active_before =
      post_turn_armed_cruise_track_window_active_before;
    heading_debug_event.post_turn_followthrough_active_before =
      post_turn_armed_cruise_track_followthrough_active_before;
    heading_debug_event.post_turn_context_active_before =
      post_turn_armed_cruise_track_context_before;
    heading_debug_event.post_turn_window_active_after =
      post_turn_armed_cruise_track_window_active_after;
    heading_debug_event.post_turn_followthrough_active_after =
      post_turn_armed_cruise_track_followthrough_active_after;
    heading_debug_event.post_turn_context_active_after =
      post_turn_armed_cruise_track_context_after;
    heading_debug_event.post_turn_window_until_before_sec =
      post_turn_armed_cruise_track_until_before_sec;
    heading_debug_event.post_turn_followthrough_until_before_sec =
      post_turn_armed_cruise_track_followthrough_until_before_sec;
    heading_debug_event.post_turn_followthrough_remaining_before =
      post_turn_armed_cruise_track_followthrough_remaining_before;
    heading_debug_event.post_turn_window_until_after_sec =
      post_turn_armed_cruise_track_until_sec_;
    heading_debug_event.post_turn_followthrough_until_after_sec =
      post_turn_armed_cruise_track_followthrough_until_sec_;
    heading_debug_event.post_turn_followthrough_remaining_after =
      post_turn_armed_cruise_track_followthrough_remaining_;
    heading_debug_event.used_post_turn_cruise_track = used_post_turn_cruise_track;
    heading_debug_event.used_armed_cruise_track = used_armed_cruise_track;
    heading_debug_event.post_turn_cruise_track_continue_needed =
      post_turn_cruise_track_continue_needed;
    heading_debug_event.heading_update_underreacted = heading_update_underreacted;
    heading_debug_event.horizontal_speed_mps = last_mavros_horizontal_speed_mps_;
    heading_debug_event.vertical_speed_mps = last_mavros_vertical_speed_mps_;
    heading_debug_event.speed_mps = last_mavros_speed_mps_;
    heading_debug_event.gyro_deg_s = last_imu_gyro_norm_deg_s_;
    heading_debug_event.source_yaw_rate_deg_s = last_mavros_heading_rate_deg_s_;
    heading_debug_event.heading_large_residual_skip_count = heading_large_residual_skip_count_;
    heading_debug_event.post_turn_blocked_count = post_turn_blocked_count_;
    heading_debug_event.armed_cruise_blocked_count = armed_cruise_blocked_count_;
    logHeadingUpdateDebug_(heading_debug_event);

    if (heading_update_underreacted) {
      double force_relock_std_deg =
        std::max(0.1, std::abs(heading_underreaction_force_relock_yaw_std_deg_));
      if (post_turn_context) {
        force_relock_std_deg = std::min(
          force_relock_std_deg,
          std::max(0.1, std::abs(heading_post_turn_force_relock_yaw_std_deg_)));
      } else if (armed_cruise_force_relock_context) {
        force_relock_std_deg = std::min(
          force_relock_std_deg,
          std::max(0.1, std::abs(heading_armed_cruise_force_relock_yaw_std_deg_)));
      }
      if (core_->forceYaw(t_sec, raw_heading_deg, force_relock_std_deg)) {
        const auto st_force = core_->current();
        HeadingUpdateDebugEvent force_debug_event = heading_debug_event;
        force_debug_event.event_type = "underreaction_force_relock";
        force_debug_event.heading_mode = "underreaction_force_relock";
        force_debug_event.residual_after_deg =
          shortestAngleDiffDeg_(raw_heading_deg, st_force.yaw_deg);
        force_debug_event.core_yaw_after_deg = st_force.yaw_deg;
        force_debug_event.measurement_heading_deg = raw_heading_deg;
        force_debug_event.measurement_std_deg = force_relock_std_deg;
        force_debug_event.yaw_correction_abs_deg =
          std::abs(shortestAngleDiffDeg_(st_force.yaw_deg, core_yaw_before_deg));
        logHeadingUpdateDebug_(force_debug_event);
        last_heading_recovery_time_sec_ = now_sec;
        last_heading_underreaction_force_relock_time_sec_ = now_sec;
        last_heading_update_time_sec_ = now_sec;
        heading_large_residual_skip_count_ = 0;
        post_turn_blocked_count_ = 0;
        armed_cruise_blocked_count_ = 0;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Applied heading underreaction_force_relock: residual_before=%.2f deg residual_after=%.2f deg "
          "core_yaw_before=%.2f deg core_yaw_after_update=%.2f deg raw_heading=%.2f deg "
          "yaw_std=%.2f deg correction=%.2f deg "
          "(armed=%s, recent_turning=%s, post_turn_hold=%s, armed_cruise=%s, gyro=%.2f deg/s, "
          "source_yaw_rate=%.2f deg/s, core_yaw_after_force=%.2f deg)",
          yaw_residual_deg, residual_after_deg,
          core_yaw_before_deg, core_yaw_after_deg, raw_heading_deg,
          force_relock_std_deg, yaw_correction_abs_deg,
          mavros_armed_ ? "true" : "false",
          recent_turning ? "true" : "false",
          post_turn_hold_active ? "true" : "false",
          armed_cruise_force_relock_context ? "true" : "false",
          last_imu_gyro_norm_deg_s_,
          last_mavros_heading_rate_deg_s_,
          st_force.yaw_deg);
        return;
      }
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "core_->forceYaw() failed during heading underreaction relock");
    }

    const bool have_fresh_mavros_tilt =
      have_mavros_tilt_ &&
      std::isfinite(last_mavros_attitude_rx_time_sec_) &&
      (now_sec - last_mavros_attitude_rx_time_sec_) <= std::max(0.05, heading_update_max_age_sec_);
    const double roll_residual_deg =
      shortestAngleDiffDeg_(mavros_roll_ned_deg_, st_after.roll_deg);
    const double pitch_residual_deg =
      shortestAngleDiffDeg_(mavros_pitch_ned_deg_, st_after.pitch_deg);
    const double tilt_residual_abs_deg =
      std::max(std::abs(roll_residual_deg), std::abs(pitch_residual_deg));
    const bool tilt_context =
      tilt_force_relock_enable_ &&
      mavros_armed_ &&
      !turning_now &&
      have_fresh_mavros_tilt &&
      (post_turn_context || armed_cruise_force_relock_context);
    const int tilt_context_id =
      post_turn_context ? 1 : (armed_cruise_force_relock_context ? 2 : 0);
    if (tilt_context_id == 0) {
      active_tilt_force_relock_context_id_ = 0;
      tilt_force_relock_applied_in_active_context_ = false;
    } else if (tilt_context_id != active_tilt_force_relock_context_id_) {
      active_tilt_force_relock_context_id_ = tilt_context_id;
      tilt_force_relock_applied_in_active_context_ = false;
    }
    const double tilt_min_period_sec =
      tilt_force_relock_max_rate_hz_ > 0.0 ? 1.0 / tilt_force_relock_max_rate_hz_ : 0.0;
    const bool tilt_rate_ok =
      !std::isfinite(last_tilt_force_relock_time_sec_) ||
      (now_sec - last_tilt_force_relock_time_sec_) >= tilt_min_period_sec;
    const bool tilt_gyro_ok =
      !std::isfinite(last_imu_gyro_norm_deg_s_) ||
      tilt_force_relock_max_gyro_deg_s_ <= 0.0 ||
      last_imu_gyro_norm_deg_s_ <= tilt_force_relock_max_gyro_deg_s_;
    const bool tilt_vertical_ok =
      !std::isfinite(last_mavros_vertical_speed_mps_) ||
      tilt_force_relock_max_vertical_speed_mps_ <= 0.0 ||
      last_mavros_vertical_speed_mps_ <= tilt_force_relock_max_vertical_speed_mps_;
    const bool tilt_context_rearm_ok =
      !tilt_force_relock_once_per_motion_context_ ||
      !tilt_force_relock_applied_in_active_context_;

    if (tilt_context &&
        tilt_context_rearm_ok &&
        tilt_rate_ok &&
        tilt_gyro_ok &&
        tilt_vertical_ok &&
        tilt_residual_abs_deg >= std::max(0.0, tilt_force_relock_min_residual_deg_)) {
      const double tilt_std_deg =
        std::max(0.1, std::abs(tilt_force_relock_roll_pitch_std_deg_));
      if (core_->forceRollPitch(t_sec, mavros_roll_ned_deg_, mavros_pitch_ned_deg_, tilt_std_deg)) {
        const auto st_tilt = core_->current();
        last_tilt_force_relock_time_sec_ = now_sec;
        tilt_force_relock_applied_in_active_context_ = true;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Applied tilt_force_relock: roll_residual=%.2f deg pitch_residual=%.2f deg "
          "core_rp_before=%.2f/%.2f deg src_rp=%.2f/%.2f deg rp_std=%.2f deg "
          "(post_turn=%s, armed_cruise=%s, once_per_context=%s, gyro=%.2f deg/s, vert=%.2f m/s, core_rp_after=%.2f/%.2f deg)",
          roll_residual_deg, pitch_residual_deg,
          st_after.roll_deg, st_after.pitch_deg,
          mavros_roll_ned_deg_, mavros_pitch_ned_deg_,
          tilt_std_deg,
          post_turn_context ? "true" : "false",
          armed_cruise_force_relock_context ? "true" : "false",
          tilt_force_relock_once_per_motion_context_ ? "true" : "false",
          last_imu_gyro_norm_deg_s_,
          last_mavros_vertical_speed_mps_,
          st_tilt.roll_deg, st_tilt.pitch_deg);
        return;
      }
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "core_->forceRollPitch() failed during tilt relock");
    }

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
      "(gate=%.2f deg, armed=%s, turning=%s, recent_turning=%s, post_turn_hold=%s, "
      "post_turn_window_active=%s, post_turn_followthrough_active=%s, "
      "post_turn_context_active=%s, gyro=%.2f deg/s, speed=%.2f m/s, "
      "horiz=%.2f m/s, vert=%.2f m/s, source_yaw_rate=%.2f deg/s, core_yaw_after=%.2f deg)",
      heading_mode,
      yaw_residual_deg, core_yaw_before_deg, raw_heading_deg,
      heading_measurement_deg, heading_measurement_std_deg,
      innovation_gate_deg,
      mavros_armed_ ? "true" : "false",
      turning_now ? "true" : "false",
      recent_turning ? "true" : "false",
      post_turn_hold_active ? "true" : "false",
      post_turn_armed_cruise_track_window_active ? "true" : "false",
      post_turn_armed_cruise_track_followthrough_active ? "true" : "false",
      post_turn_armed_cruise_track_context ? "true" : "false",
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

    last_processed_imu_raw_dt_sec_ = dt_candidate;
    last_processed_imu_effective_dt_sec_ = std::numeric_limits<double>::quiet_NaN();
    last_medium_imu_gap_dropped_dt_sec_ = 0.0;
    last_medium_imu_gap_active_ = false;
    last_medium_imu_gap_segmented_ = false;
    last_medium_imu_gap_conservative_single_step_ = false;
    last_medium_imu_gap_segmented_steps_ = 1;

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
      last_processed_imu_effective_dt_sec_ = dt;
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
        last_medium_imu_gap_active_ = true;
        last_medium_imu_gap_conservative_single_step_ = true;
        last_medium_imu_gap_dropped_dt_sec_ = dropped_dt;
        last_processed_imu_effective_dt_sec_ = dt;
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
        last_medium_imu_gap_active_ = true;
        last_medium_imu_gap_segmented_ = use_segmented_propagation;
        last_medium_imu_gap_segmented_steps_ = segmented_propagation_steps;
        last_processed_imu_effective_dt_sec_ = dt_step;
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
      last_processed_imu_effective_dt_sec_ = dt;
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
    logObservationDebugIfNeeded_();
    publishState();
  }

  void handleGnssFix_(
    double t_raw,
    int navsat_status,
    double latitude,
    double longitude,
    double altitude,
    uint8_t covariance_type,
    double covariance_00,
    double covariance_44,
    double covariance_88,
    bool native_velocity_valid = false,
    double native_vN = std::numeric_limits<double>::quiet_NaN(),
    double native_vE = std::numeric_limits<double>::quiet_NaN(),
    double native_vD = std::numeric_limits<double>::quiet_NaN(),
    double native_speed_std_mps = std::numeric_limits<double>::quiet_NaN())
  {
    double t = t_raw;
    bool reset_this_gnss = false;
    const double now_sec = now().seconds();

    // 支持 dropzones：当上游发布 NO_FIX，这里直接跳过 GNSS 更新
    if (navsat_status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX) {
      last_gnss_has_fix_ = false;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "GNSS status=NO_FIX, skipping update.");
      return;
    }
    last_gnss_has_fix_ = true;

    if (!std::isfinite(latitude) || !std::isfinite(longitude) || !std::isfinite(altitude)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "GNSS contains NaN/Inf, skipping.");
      return;
    }

    last_gnss_lat_rad_ = latitude  * M_PI/180.0;
    last_gnss_lon_rad_ = longitude * M_PI/180.0;
    last_gnss_h_m_     = altitude;
    last_gnss_valid_   = true;
    last_gnss_source_time_sec_ = t_raw;
    last_gnss_rx_ros_time_sec_ = now_sec;

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
                    latitude, longitude, altitude);
      }
    }

    // 第一次 GNSS 同时用于初始化/重置核心，避免从 (0,0,0) 等不合理状态起算导致数值问题
    if (!core_initialized_) {
      if (!have_mavros_heading_ && !trySyncHeadingSampleBeforeFirstReset_()) {
        RCLCPP_WARN(
          this->get_logger(),
          "No heading sample available before first GNSS reset; falling back to yaw=0 from source=%s topic=%s",
          heading_source_.c_str(),
          active_heading_topic_name_.c_str());
      }
      const char* reset_yaw_source = "zero";
      const double init_yaw = selectYawForCoreReset_(&reset_yaw_source);
      (void)core_->reset(latitude, longitude, altitude, init_yaw);
      core_initialized_ = true;
      reset_this_gnss = true;
      prefer_preserved_yaw_on_next_core_reset_ = false;
      have_completed_armed_flight_since_reset_ = false;
      last_armed_transition_time_sec_ = std::numeric_limits<double>::quiet_NaN();
      last_disarmed_transition_time_sec_ = std::numeric_limits<double>::quiet_NaN();
      post_flight_vertical_cov_reopen_until_sec_ = std::numeric_limits<double>::quiet_NaN();
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

    const HeadingMotionContext motion_ctx = buildHeadingMotionContext_(now_sec, false);
    const bool position_override_context =
      mavros_armed_ &&
      motion_ctx.have_fresh_mavros_speed;
    const bool position_residual_boost_context = position_override_context;

    double last_position_residual_h_m = std::numeric_limits<double>::quiet_NaN();
    double last_position_residual_u_m = std::numeric_limits<double>::quiet_NaN();
    double last_velocity_residual_h_mps = std::numeric_limits<double>::quiet_NaN();
    double last_velocity_residual_e_mps = std::numeric_limits<double>::quiet_NaN();
    double last_velocity_residual_d_mps = std::numeric_limits<double>::quiet_NaN();
    if (core_) {
      const kfcore::ObservationDebug last_observation_debug = core_->lastObservationDebug();
      if (last_observation_debug.valid && last_observation_debug.gnss_position_applied) {
        const double residual_n_m = last_observation_debug.gnss_position_residual_neu_m.x();
        const double residual_e_m = last_observation_debug.gnss_position_residual_neu_m.y();
        if (std::isfinite(residual_n_m) && std::isfinite(residual_e_m)) {
          last_position_residual_h_m = std::hypot(residual_n_m, residual_e_m);
        }
        const double residual_u_m = last_observation_debug.gnss_position_residual_neu_m.z();
        if (std::isfinite(residual_u_m)) {
          last_position_residual_u_m = std::abs(residual_u_m);
        }
      }
      if (last_observation_debug.valid && last_observation_debug.gnss_velocity_applied) {
        const double residual_n_mps = last_observation_debug.gnss_velocity_residual_ned_mps.x();
        const double residual_e_mps = last_observation_debug.gnss_velocity_residual_ned_mps.y();
        const double residual_d_mps = last_observation_debug.gnss_velocity_residual_ned_mps.z();
        if (std::isfinite(residual_n_mps) && std::isfinite(residual_e_mps)) {
          last_velocity_residual_h_mps = std::hypot(residual_n_mps, residual_e_mps);
          last_velocity_residual_e_mps = residual_e_mps;
        }
        if (std::isfinite(residual_d_mps)) {
          last_velocity_residual_d_mps = residual_d_mps;
        }
      }
    }

    if (armed_cruise_gnss_pos_residual_boost_enable_ &&
        position_residual_boost_context &&
        ((std::isfinite(last_position_residual_h_m) &&
          last_position_residual_h_m >=
            std::max(0.0, armed_cruise_gnss_pos_residual_boost_threshold_m_)) ||
         (std::isfinite(last_position_residual_u_m) &&
          last_position_residual_u_m >=
            std::max(0.0, armed_cruise_gnss_pos_residual_boost_threshold_m_)))) {
      armed_cruise_gnss_pos_residual_boost_until_sec_ =
        now_sec + std::max(0.0, armed_cruise_gnss_pos_residual_boost_hold_sec_);
    }
    const bool residual_position_boost_active =
      armed_cruise_gnss_pos_residual_boost_enable_ &&
      std::isfinite(armed_cruise_gnss_pos_residual_boost_until_sec_) &&
      now_sec <= armed_cruise_gnss_pos_residual_boost_until_sec_ &&
      mavros_armed_ &&
      motion_ctx.have_fresh_mavros_speed;
    const bool velocity_residual_boost_context =
      mavros_armed_ &&
      motion_ctx.have_fresh_mavros_speed &&
      motion_ctx.native_velocity_tightening_context;
    if (armed_cruise_native_gnss_vel_residual_boost_enable_ &&
        velocity_residual_boost_context &&
        std::max(
          std::isfinite(last_velocity_residual_e_mps) ? std::abs(last_velocity_residual_e_mps) : 0.0,
          std::isfinite(last_velocity_residual_d_mps) ? std::abs(last_velocity_residual_d_mps) : 0.0) >=
          std::max(0.0, armed_cruise_native_gnss_vel_residual_boost_threshold_mps_)) {
      armed_cruise_native_gnss_vel_residual_boost_until_sec_ =
        now_sec + std::max(0.0, armed_cruise_native_gnss_vel_residual_boost_hold_sec_);
    }
    const bool residual_velocity_boost_active =
      armed_cruise_native_gnss_vel_residual_boost_enable_ &&
      std::isfinite(armed_cruise_native_gnss_vel_residual_boost_until_sec_) &&
      now_sec <= armed_cruise_native_gnss_vel_residual_boost_until_sec_ &&
      velocity_residual_boost_context;

    const auto compute_core_gnss_diff = [&](const kfcore::State & core_state,
                                            double * diff_h_m,
                                            double * diff_u_m) {
      if (diff_h_m != nullptr) {
        *diff_h_m = std::numeric_limits<double>::quiet_NaN();
      }
      if (diff_u_m != nullptr) {
        *diff_u_m = std::numeric_limits<double>::quiet_NaN();
      }
      if (!std::isfinite(core_state.lat_deg) ||
          !std::isfinite(core_state.lon_deg) ||
          !std::isfinite(core_state.h_m)) {
        return;
      }

      const double core_lat_rad = core_state.lat_deg * M_PI / 180.0;
      const double core_lon_rad = core_state.lon_deg * M_PI / 180.0;
      double core_x, core_y, core_z;
      double gnss_x, gnss_y, gnss_z;
      geo::llh_to_ecef(core_lat_rad, core_lon_rad, core_state.h_m, core_x, core_y, core_z);
      geo::llh_to_ecef(last_gnss_lat_rad_, last_gnss_lon_rad_, last_gnss_h_m_, gnss_x, gnss_y, gnss_z);
      const Eigen::Vector3d enu_core =
        geo::ecef_to_enu({core_x, core_y, core_z}, origin_ecef_, origin_lat_, origin_lon_);
      const Eigen::Vector3d enu_gnss =
        geo::ecef_to_enu({gnss_x, gnss_y, gnss_z}, origin_ecef_, origin_lat_, origin_lon_);
      if (diff_h_m != nullptr &&
          std::isfinite(enu_core.x()) && std::isfinite(enu_core.y()) &&
          std::isfinite(enu_gnss.x()) && std::isfinite(enu_gnss.y())) {
        *diff_h_m = std::hypot(enu_core.x() - enu_gnss.x(), enu_core.y() - enu_gnss.y());
      }
      if (diff_u_m != nullptr &&
          std::isfinite(enu_core.z()) && std::isfinite(enu_gnss.z())) {
        *diff_u_m = enu_core.z() - enu_gnss.z();
      }
    };

    double core_gnss_diff_h_m = std::numeric_limits<double>::quiet_NaN();
    double core_gnss_diff_u_m = std::numeric_limits<double>::quiet_NaN();
    double core_pos_std_d_m = std::numeric_limits<double>::quiet_NaN();
    double core_vel_std_d_mps = std::numeric_limits<double>::quiet_NaN();
    double core_accbias_std_z_mps2 = std::numeric_limits<double>::quiet_NaN();
    const auto refresh_core_vertical_debug = [&]() {
      const Eigen::MatrixXd core_covariance = core_->covariance();
      if (core_covariance.rows() < 15 || core_covariance.cols() < 15) {
        core_pos_std_d_m = std::numeric_limits<double>::quiet_NaN();
        core_vel_std_d_mps = std::numeric_limits<double>::quiet_NaN();
        core_accbias_std_z_mps2 = std::numeric_limits<double>::quiet_NaN();
        return;
      }
      const auto diag_std = [&](int idx) {
        const double value = core_covariance(idx, idx);
        return (std::isfinite(value) && value >= 0.0)
                 ? std::sqrt(value)
                 : std::numeric_limits<double>::quiet_NaN();
      };
      core_pos_std_d_m = diag_std(2);
      core_vel_std_d_mps = diag_std(5);
      core_accbias_std_z_mps2 = diag_std(14);
    };

    const kfcore::State core_state_before_update = core_->current();
    compute_core_gnss_diff(core_state_before_update, &core_gnss_diff_h_m, &core_gnss_diff_u_m);
    refresh_core_vertical_debug();

    if (armed_cruise_vertical_cov_reopen_enable_ &&
        velocity_residual_boost_context &&
        ((std::isfinite(last_position_residual_u_m) &&
          last_position_residual_u_m >=
            std::max(0.0, armed_cruise_vertical_cov_reopen_threshold_m_)) ||
         (std::isfinite(core_gnss_diff_u_m) &&
          std::abs(core_gnss_diff_u_m) >=
            std::max(0.0, armed_cruise_vertical_cov_reopen_threshold_m_)))) {
      armed_cruise_vertical_cov_reopen_until_sec_ =
        now_sec + std::max(0.0, armed_cruise_vertical_cov_reopen_hold_sec_);
    }
    const bool vertical_cov_reopen_active =
      armed_cruise_vertical_cov_reopen_enable_ &&
      std::isfinite(armed_cruise_vertical_cov_reopen_until_sec_) &&
      now_sec <= armed_cruise_vertical_cov_reopen_until_sec_ &&
      velocity_residual_boost_context;
    bool vertical_cov_reopen_applied = false;
    if (vertical_cov_reopen_active) {
      vertical_cov_reopen_applied = core_->reopenVerticalCovariance(
        std::max(0.01, std::abs(armed_cruise_vertical_cov_reopen_pos_std_m_)),
        std::max(0.01, std::abs(armed_cruise_vertical_cov_reopen_vel_std_mps_)),
        std::max(1e-4, std::abs(armed_cruise_vertical_cov_reopen_accbias_std_z_mps2_)));
      if (vertical_cov_reopen_applied) {
        refresh_core_vertical_debug();
      }
    }
    const bool post_flight_vertical_cov_reopen_context =
      post_flight_vertical_cov_reopen_enable_ &&
      !mavros_armed_ &&
      have_completed_armed_flight_since_reset_;
    const bool post_flight_vertical_cov_reopen_grace_active =
      post_flight_vertical_cov_reopen_context &&
      std::isfinite(last_disarmed_transition_time_sec_) &&
      std::max(0.0, post_flight_vertical_cov_reopen_grace_sec_) > 0.0 &&
      (now_sec - last_disarmed_transition_time_sec_) <=
        std::max(0.0, post_flight_vertical_cov_reopen_grace_sec_);
    if (post_flight_vertical_cov_reopen_context &&
        (post_flight_vertical_cov_reopen_grace_active ||
         (std::isfinite(last_position_residual_u_m) &&
          last_position_residual_u_m >=
            std::max(0.0, post_flight_vertical_cov_reopen_threshold_m_)) ||
         (std::isfinite(core_gnss_diff_u_m) &&
          std::abs(core_gnss_diff_u_m) >=
            std::max(0.0, post_flight_vertical_cov_reopen_threshold_m_)))) {
      post_flight_vertical_cov_reopen_until_sec_ =
        now_sec + std::max(0.0, post_flight_vertical_cov_reopen_hold_sec_);
    }
    const bool post_flight_vertical_cov_reopen_active =
      post_flight_vertical_cov_reopen_context &&
      (post_flight_vertical_cov_reopen_grace_active ||
       (std::isfinite(post_flight_vertical_cov_reopen_until_sec_) &&
        now_sec <= post_flight_vertical_cov_reopen_until_sec_));
    bool post_flight_vertical_cov_reopen_applied = false;
    if (post_flight_vertical_cov_reopen_active) {
      post_flight_vertical_cov_reopen_applied = core_->reopenVerticalCovariance(
        std::max(0.01, std::abs(post_flight_vertical_cov_reopen_pos_std_m_)),
        std::max(0.01, std::abs(post_flight_vertical_cov_reopen_vel_std_mps_)),
        std::max(1e-4, std::abs(post_flight_vertical_cov_reopen_accbias_std_z_mps2_)));
      if (post_flight_vertical_cov_reopen_applied) {
        refresh_core_vertical_debug();
      }
    }
    const bool any_vertical_cov_reopen_active =
      vertical_cov_reopen_active || post_flight_vertical_cov_reopen_active;
    const bool any_vertical_cov_reopen_applied =
      vertical_cov_reopen_applied || post_flight_vertical_cov_reopen_applied;

    Eigen::Vector3d std_ned(-1,-1,-1);
    bool position_override_active = false;
    
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
    } else if (covariance_type == sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN) {
      // 实机模式 + msg有协方差：使用msg中的协方差
      const double std_e = std::sqrt(std::max(0.0, covariance_00));
      const double std_n = std::sqrt(std::max(0.0, covariance_44));
      const double std_u = std::sqrt(std::max(0.0, covariance_88));
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

    if (armed_cruise_gnss_pos_override_enable_ && position_override_context) {
      const double position_std_floor_m = 0.01;
      const double override_std_h_m =
        std::max(position_std_floor_m, std::abs(armed_cruise_gnss_pos_std_h_m_));
      const double override_std_u_m =
        std::max(position_std_floor_m, std::abs(armed_cruise_gnss_pos_std_u_m_));
      std_ned.x() = std::min(std_ned.x(), override_std_h_m);
      std_ned.y() = std::min(std_ned.y(), override_std_h_m);
      std_ned.z() = std::min(std_ned.z(), override_std_u_m);
      position_override_active = true;
    }

    if (residual_position_boost_active) {
      const double position_std_floor_m = 0.01;
      const double override_std_h_m =
        std::max(position_std_floor_m, std::abs(armed_cruise_gnss_pos_std_h_m_));
      const double override_std_u_m =
        std::max(position_std_floor_m, std::abs(armed_cruise_gnss_pos_std_u_m_));
      std_ned.x() = std::min(std_ned.x(), override_std_h_m);
      std_ned.y() = std::min(std_ned.y(), override_std_h_m);
      std_ned.z() = std::min(std_ned.z(), override_std_u_m);
      position_override_active = true;
    }

    // Pre-arm yaw lock is useful when the first GNSS reset had to fall back to yaw=0
    // before MAVROS heading was ready. Keep it out of post-flight disarmed windows so
    // we do not perturb the landed sentinel checks or clear the path after mission end.
    if (!reset_this_gnss &&
        disarmed_yaw_lock_enable_ && !mavros_armed_ &&
        !have_completed_armed_flight_since_reset_ &&
        core_initialized_ && have_mavros_heading_) {
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

    if (!core_->ingestGnss(t, latitude, longitude, altitude, std_ned)) {
      pending_gnss_debug_context_.valid = false;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "core_->ingestGnss() failed");
      return;
    }

    // GNSS 速度观测
    // GNSS velocity derived from consecutive position fixes
    // 这是解决姿态（尤其 yaw）不可观的关键：速度观测让 H 矩阵覆盖了 V 状态，
    // 通过 F 矩阵中的 V↔PHI 耦合，间接使姿态可观。
    // 当 ZUPT 已应用时跳过（ZUPT 的 0.05 m/s std 比位置差分的 ~0.7 m/s 更紧）
    const bool have_native_gnss_velocity =
      native_velocity_valid &&
      std::isfinite(native_vN) &&
      std::isfinite(native_vE) &&
      std::isfinite(native_vD);
    bool native_velocity_used = false;
    bool native_velocity_override_active = false;
    bool residual_velocity_boost_applied = false;
    double native_velocity_std_h_mps = std::numeric_limits<double>::quiet_NaN();
    double native_velocity_std_u_mps = std::numeric_limits<double>::quiet_NaN();
    if (enable_gnss_velocity_update_ && !zupt_applied && have_native_gnss_velocity) {
      double vel_floor_h = std::max(0.03, gnss_vel_std_floor_h_mps_);
      double vel_floor_u = std::max(0.08, gnss_vel_std_floor_u_mps_);
      native_velocity_override_active =
        armed_cruise_native_gnss_vel_override_enable_ &&
        motion_ctx.native_velocity_tightening_context;
      if (native_velocity_override_active) {
        vel_floor_h = std::max(0.03, std::abs(armed_cruise_native_gnss_vel_std_h_mps_));
        vel_floor_u = std::max(0.08, std::abs(armed_cruise_native_gnss_vel_std_u_mps_));
      }
      if (residual_velocity_boost_active) {
        vel_floor_h =
          std::max(0.03, std::abs(armed_cruise_native_gnss_vel_residual_boost_std_h_mps_));
        vel_floor_u =
          std::max(0.08, std::abs(armed_cruise_native_gnss_vel_residual_boost_std_u_mps_));
        residual_velocity_boost_applied = true;
      }
      const double native_std =
        (std::isfinite(native_speed_std_mps) && native_speed_std_mps > 0.0)
          ? native_speed_std_mps * std::max(0.0, native_gnss_speed_std_scale_)
          : std::numeric_limits<double>::quiet_NaN();
      Eigen::Vector3d vel_std;
      if (native_velocity_override_active || residual_velocity_boost_applied) {
        vel_std = Eigen::Vector3d(vel_floor_h, vel_floor_h, vel_floor_u);
      } else {
        vel_std = Eigen::Vector3d(
          std::max(vel_floor_h, std::isfinite(native_std) ? native_std : vel_floor_h),
          std::max(vel_floor_h, std::isfinite(native_std) ? native_std : vel_floor_h),
          std::max(vel_floor_u, std::isfinite(native_std) ? native_std : vel_floor_u));
      }

      if (!logged_first_native_gnss_velocity_sample_) {
        RCLCPP_INFO(
          get_logger(),
          "Using native GNSS velocity observation: vN=%.3f vE=%.3f vD=%.3f std_h=%.3f std_u=%.3f",
          native_vN, native_vE, native_vD, vel_std.x(), vel_std.z());
        logged_first_native_gnss_velocity_sample_ = true;
      }
      core_->ingestGnssVel(t, native_vN, native_vE, native_vD, vel_std);
      native_velocity_used = true;
      native_velocity_std_h_mps = vel_std.x();
      native_velocity_std_u_mps = vel_std.z();
    } else if (enable_gnss_velocity_update_ && !zupt_applied && have_prev_gnss_for_vel_) {
      const double dt_gnss = t - prev_gnss_vel_time_;
      if (std::isfinite(dt_gnss) && dt_gnss > 0.05 && dt_gnss < 5.0) {
        const double lat_avg = (latitude * M_PI/180.0 + prev_gnss_vel_lat_rad_) * 0.5;
        const double dlat_rad = latitude * M_PI/180.0 - prev_gnss_vel_lat_rad_;
        const double dlon_rad = longitude * M_PI/180.0 - prev_gnss_vel_lon_rad_;
        const double dh = altitude - prev_gnss_vel_h_;

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
    pending_gnss_debug_context_.valid = true;
    pending_gnss_debug_context_.native_velocity_valid = have_native_gnss_velocity;
    pending_gnss_debug_context_.native_velocity_used = native_velocity_used;
    pending_gnss_debug_context_.native_velocity_override_active = native_velocity_override_active;
    pending_gnss_debug_context_.velocity_residual_boost_active = residual_velocity_boost_applied;
    pending_gnss_debug_context_.position_override_active = position_override_active;
    pending_gnss_debug_context_.position_residual_boost_active = residual_position_boost_active;
    pending_gnss_debug_context_.vertical_cov_reopen_active = any_vertical_cov_reopen_active;
    pending_gnss_debug_context_.vertical_cov_reopen_applied = any_vertical_cov_reopen_applied;
    pending_gnss_debug_context_.post_flight_vertical_cov_reopen_active =
      post_flight_vertical_cov_reopen_active;
    pending_gnss_debug_context_.post_flight_vertical_cov_reopen_applied =
      post_flight_vertical_cov_reopen_applied;
    pending_gnss_debug_context_.armed = mavros_armed_;
    pending_gnss_debug_context_.have_fresh_speed = motion_ctx.have_fresh_mavros_speed;
    pending_gnss_debug_context_.turning_now = motion_ctx.turning_now;
    pending_gnss_debug_context_.post_turn_context = motion_ctx.post_turn_context;
    pending_gnss_debug_context_.armed_cruise_context = motion_ctx.armed_cruise_force_relock_context;
    pending_gnss_debug_context_.native_velocity_tightening_context =
      motion_ctx.native_velocity_tightening_context;
    pending_gnss_debug_context_.medium_gap_active = last_medium_imu_gap_active_;
    pending_gnss_debug_context_.medium_gap_segmented = last_medium_imu_gap_segmented_;
    pending_gnss_debug_context_.medium_gap_conservative_single_step =
      last_medium_imu_gap_conservative_single_step_;
    pending_gnss_debug_context_.last_position_residual_h_m = last_position_residual_h_m;
    pending_gnss_debug_context_.last_position_residual_u_m = last_position_residual_u_m;
    pending_gnss_debug_context_.last_velocity_residual_h_mps = last_velocity_residual_h_mps;
    pending_gnss_debug_context_.last_velocity_residual_e_mps = last_velocity_residual_e_mps;
    pending_gnss_debug_context_.last_velocity_residual_d_mps = last_velocity_residual_d_mps;
    pending_gnss_debug_context_.core_gnss_diff_h_m = core_gnss_diff_h_m;
    pending_gnss_debug_context_.core_gnss_diff_u_m = core_gnss_diff_u_m;
    pending_gnss_debug_context_.core_pos_std_d_m = core_pos_std_d_m;
    pending_gnss_debug_context_.core_vel_std_d_mps = core_vel_std_d_mps;
    pending_gnss_debug_context_.core_accbias_std_z_mps2 = core_accbias_std_z_mps2;
    pending_gnss_debug_context_.ros_time_sec = now_sec;
    pending_gnss_debug_context_.update_time_sec = t;
    pending_gnss_debug_context_.horizontal_speed_mps = last_mavros_horizontal_speed_mps_;
    pending_gnss_debug_context_.vertical_speed_mps = last_mavros_vertical_speed_mps_;
    pending_gnss_debug_context_.gyro_deg_s = last_imu_gyro_norm_deg_s_;
    pending_gnss_debug_context_.source_yaw_rate_deg_s = last_mavros_heading_rate_deg_s_;
    pending_gnss_debug_context_.latest_imu_raw_dt_sec = last_processed_imu_raw_dt_sec_;
    pending_gnss_debug_context_.latest_imu_effective_dt_sec =
      last_processed_imu_effective_dt_sec_;
    pending_gnss_debug_context_.medium_gap_dropped_dt_sec =
      last_medium_imu_gap_dropped_dt_sec_;
    pending_gnss_debug_context_.medium_gap_segmented_steps =
      last_medium_imu_gap_segmented_steps_;
    pending_gnss_debug_context_.native_velocity_std_h_mps = native_velocity_std_h_mps;
    pending_gnss_debug_context_.native_velocity_std_u_mps = native_velocity_std_u_mps;
    pending_gnss_debug_context_.native_velocity_vN_mps =
      have_native_gnss_velocity ? native_vN : std::numeric_limits<double>::quiet_NaN();
    pending_gnss_debug_context_.native_velocity_vE_mps =
      have_native_gnss_velocity ? native_vE : std::numeric_limits<double>::quiet_NaN();
    pending_gnss_debug_context_.native_velocity_vD_mps =
      have_native_gnss_velocity ? native_vD : std::numeric_limits<double>::quiet_NaN();
    pending_gnss_debug_context_.core_velocity_vN_mps = core_state_before_update.vN;
    pending_gnss_debug_context_.core_velocity_vE_mps = core_state_before_update.vE;
    pending_gnss_debug_context_.core_velocity_vD_mps = core_state_before_update.vD;
    prev_gnss_vel_lat_rad_ = latitude  * M_PI/180.0;
    prev_gnss_vel_lon_rad_ = longitude * M_PI/180.0;
    prev_gnss_vel_h_       = altitude;
    prev_gnss_vel_time_    = t;
    have_prev_gnss_for_vel_ = true;

    last_core_time_ = t;
    last_gnss_time_sec_ = t;

    publishState();
  }

  void gnssCb(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    const double t_raw = use_node_time_for_core_ ? rawTimeSecNow_() : rawTimeSecFromMsg_(msg->header.stamp);
    const double now_sec = now().seconds();
    const bool have_fresh_native_sensor_gps_velocity =
      enable_gnss_velocity_update_ &&
      enable_native_sensor_gps_velocity_aid_ &&
      have_native_sensor_gps_velocity_ &&
      std::isfinite(last_native_sensor_gps_velocity_rx_time_sec_) &&
      (now_sec - last_native_sensor_gps_velocity_rx_time_sec_) <=
        std::max(0.05, native_sensor_gps_velocity_max_age_sec_);
    handleGnssFix_(
      t_raw,
      msg->status.status,
      msg->latitude,
      msg->longitude,
      msg->altitude,
      msg->position_covariance_type,
      msg->position_covariance[0],
      msg->position_covariance[4],
      msg->position_covariance[8],
      have_fresh_native_sensor_gps_velocity,
      native_sensor_gps_vN_mps_,
      native_sensor_gps_vE_mps_,
      native_sensor_gps_vD_mps_,
      native_sensor_gps_speed_std_mps_);
  }

  void px4SensorGpsCb(const px4_msgs::msg::SensorGps::SharedPtr msg)
  {
    updateNativeSensorGpsVelocityCache_(*msg);
    if (!logged_first_gnss_sample_) {
      RCLCPP_INFO(
        get_logger(),
        "Received first PX4 SensorGps sample: lat=%.8f lon=%.8f alt_msl=%.3f alt_ellipsoid=%.3f fix_type=%u eph=%.3f epv=%.3f",
        static_cast<double>(msg->lat) * 1e-7,
        static_cast<double>(msg->lon) * 1e-7,
        static_cast<double>(msg->alt) * 1e-3,
        static_cast<double>(msg->alt_ellipsoid) * 1e-3,
        static_cast<unsigned>(msg->fix_type),
        static_cast<double>(msg->eph),
        static_cast<double>(msg->epv));
      logged_first_gnss_sample_ = true;
    }
    const double t_raw = use_node_time_for_core_
      ? rawTimeSecNow_()
      : static_cast<double>(msg->timestamp_sample) * 1e-6;
    const bool have_cov = std::isfinite(msg->eph) && std::isfinite(msg->epv) &&
      msg->eph > 0.0f && msg->epv > 0.0f;
    const double alt_msl = static_cast<double>(msg->alt) * 1e-3;
    const double alt_ellipsoid = static_cast<double>(msg->alt_ellipsoid) * 1e-3;
    const double altitude_m =
      (msg->alt_ellipsoid != 0 || msg->alt == 0)
        ? alt_ellipsoid
        : alt_msl;
    handleGnssFix_(
      t_raw,
      msg->fix_type >= 2
        ? sensor_msgs::msg::NavSatStatus::STATUS_FIX
        : sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX,
      static_cast<double>(msg->lat) * 1e-7,
      static_cast<double>(msg->lon) * 1e-7,
      altitude_m,
      have_cov
        ? sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN
        : sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN,
      have_cov ? static_cast<double>(msg->eph) * static_cast<double>(msg->eph) : 0.0,
      have_cov ? static_cast<double>(msg->eph) * static_cast<double>(msg->eph) : 0.0,
      have_cov ? static_cast<double>(msg->epv) * static_cast<double>(msg->epv) : 0.0,
      msg->vel_ned_valid,
      static_cast<double>(msg->vel_n_m_s),
      static_cast<double>(msg->vel_e_m_s),
      static_cast<double>(msg->vel_d_m_s),
      std::isfinite(msg->s_variance_m_s)
        ? static_cast<double>(msg->s_variance_m_s)
        : std::numeric_limits<double>::quiet_NaN());
  }

  void px4SensorGpsVelocityAuxCb(const px4_msgs::msg::SensorGps::SharedPtr msg)
  {
    updateNativeSensorGpsVelocityCache_(*msg);
  }

  void px4VehicleGlobalPositionCb(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg)
  {
    if (!logged_first_gnss_sample_) {
      RCLCPP_INFO(
        get_logger(),
        "Received first PX4 VehicleGlobalPosition sample: lat=%.8f lon=%.8f alt_msl=%.3f alt_ellipsoid=%.3f dead_reckoning=%s eph=%.3f epv=%.3f",
        static_cast<double>(msg->lat),
        static_cast<double>(msg->lon),
        static_cast<double>(msg->alt),
        static_cast<double>(msg->alt_ellipsoid),
        msg->dead_reckoning ? "true" : "false",
        static_cast<double>(msg->eph),
        static_cast<double>(msg->epv));
      logged_first_gnss_sample_ = true;
    }
    const double t_raw = use_node_time_for_core_
      ? rawTimeSecNow_()
      : static_cast<double>(msg->timestamp_sample) * 1e-6;
    const bool have_cov = std::isfinite(msg->eph) && std::isfinite(msg->epv) &&
      msg->eph > 0.0f && msg->epv > 0.0f;
    const double altitude_m =
      (std::isfinite(msg->alt_ellipsoid) && std::abs(static_cast<double>(msg->alt_ellipsoid)) > 1e-3) ||
      (!std::isfinite(msg->alt))
        ? static_cast<double>(msg->alt_ellipsoid)
        : static_cast<double>(msg->alt);
    handleGnssFix_(
      t_raw,
      msg->dead_reckoning
        ? sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX
        : sensor_msgs::msg::NavSatStatus::STATUS_FIX,
      static_cast<double>(msg->lat),
      static_cast<double>(msg->lon),
      altitude_m,
      have_cov
        ? sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN
        : sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN,
      have_cov ? static_cast<double>(msg->eph) * static_cast<double>(msg->eph) : 0.0,
      have_cov ? static_cast<double>(msg->eph) * static_cast<double>(msg->eph) : 0.0,
      have_cov ? static_cast<double>(msg->epv) * static_cast<double>(msg->epv) : 0.0);
  }

  void px4VehicleLocalPositionGnssCb(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
  {
    const bool have_global_ref = msg->xy_global && msg->z_global &&
      std::isfinite(msg->ref_lat) && std::isfinite(msg->ref_lon) && std::isfinite(msg->ref_alt);
    if (!have_global_ref) {
      return;
    }

    const double ref_lat_rad = static_cast<double>(msg->ref_lat) * M_PI / 180.0;
    const double ref_lon_rad = static_cast<double>(msg->ref_lon) * M_PI / 180.0;
    const double ref_alt_m = static_cast<double>(msg->ref_alt);
    const double sin_lat = std::sin(ref_lat_rad);
    const double denom = std::sqrt(1.0 - geo::WGS84_E2 * sin_lat * sin_lat);
    const double rn = geo::WGS84_A / denom;
    const double rm = geo::WGS84_A * (1.0 - geo::WGS84_E2) / std::pow(1.0 - geo::WGS84_E2 * sin_lat * sin_lat, 1.5);

    const double north_m = static_cast<double>(msg->x);
    const double east_m = static_cast<double>(msg->y);
    const double down_m = static_cast<double>(msg->z);
    const double lat_deg =
      static_cast<double>(msg->ref_lat) + north_m / std::max(1.0, rm + ref_alt_m) * 180.0 / M_PI;
    const double lon_deg =
      static_cast<double>(msg->ref_lon) +
      east_m / std::max(1.0, (rn + ref_alt_m) * std::max(1e-6, std::cos(ref_lat_rad))) * 180.0 / M_PI;
    const double alt_m = ref_alt_m - down_m;

    if (!logged_first_gnss_sample_) {
      RCLCPP_INFO(
        get_logger(),
        "Received first PX4 VehicleLocalPosition GNSS proxy sample: ref_lat=%.8f ref_lon=%.8f ref_alt=%.3f "
        "x=%.3f y=%.3f z=%.3f eph=%.3f epv=%.3f",
        static_cast<double>(msg->ref_lat),
        static_cast<double>(msg->ref_lon),
        ref_alt_m,
        north_m,
        east_m,
        down_m,
        static_cast<double>(msg->eph),
        static_cast<double>(msg->epv));
      logged_first_gnss_sample_ = true;
    }

    const bool have_cov = std::isfinite(msg->eph) && std::isfinite(msg->epv) &&
      msg->eph > 0.0f && msg->epv > 0.0f;
    handleGnssFix_(
      use_node_time_for_core_ ? rawTimeSecNow_() : static_cast<double>(msg->timestamp_sample) * 1e-6,
      msg->dead_reckoning
        ? sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX
        : sensor_msgs::msg::NavSatStatus::STATUS_FIX,
      lat_deg,
      lon_deg,
      alt_m,
      have_cov
        ? sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN
        : sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN,
      have_cov ? static_cast<double>(msg->eph) * static_cast<double>(msg->eph) : 0.0,
      have_cov ? static_cast<double>(msg->eph) * static_cast<double>(msg->eph) : 0.0,
      have_cov ? static_cast<double>(msg->epv) * static_cast<double>(msg->epv) : 0.0);
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
    // KF-GINS 内部姿态是 NED/FRD。不能直接对欧拉角做符号翻转后塞给 ENU/FLU 的 setRPY，
    // 否则在 turn/climb 段存在非零 roll/pitch 时会引入明显的 yaw/attitude 耦合误差。
    // 正确做法是先重建 R_ned_from_frd，再做基变换：
    //   R_enu_from_flu = T_enu_from_ned * R_ned_from_frd * T_frd_from_flu
    const double roll_rad_ned  = st.roll_deg  * M_PI / 180.0;
    const double pitch_rad_ned = st.pitch_deg * M_PI / 180.0;
    const double yaw_rad_ned   = st.yaw_deg   * M_PI / 180.0;
    tf2::Matrix3x3 r_ned_from_frd;
    r_ned_from_frd.setRPY(roll_rad_ned, pitch_rad_ned, yaw_rad_ned);
    const tf2::Matrix3x3 t_enu_from_ned(
      0.0, 1.0,  0.0,
      1.0, 0.0,  0.0,
      0.0, 0.0, -1.0);
    const tf2::Matrix3x3 t_frd_from_flu(
      1.0,  0.0,  0.0,
      0.0, -1.0,  0.0,
      0.0,  0.0, -1.0);
    const tf2::Matrix3x3 r_enu_from_flu =
      t_enu_from_ned * r_ned_from_frd * t_frd_from_flu;

    // 2) 位置：启动阶段即便 use_gnss_llh_for_pose_==false，也优先用 GNSS，直到核心LLH和GNSS对齐
    double lat_rad, lon_rad, h_m;
    Eigen::Vector3d enu_core = Eigen::Vector3d::Zero();
    Eigen::Vector3d enu_gnss = Eigen::Vector3d::Zero();
    bool have_core_enu = false;
    bool have_gnss_enu = false;
    bool odom_uses_gnss_pose = false;

    if (last_gnss_valid_) {
      const double core_lat_rad = st.lat_deg * M_PI/180.0;
      const double core_lon_rad = st.lon_deg * M_PI/180.0;

      // 计算核心与GNSS的 ENU 差距，判断是否“对齐”
      double core_x, core_y, core_z;
      geo::llh_to_ecef(core_lat_rad, core_lon_rad, st.h_m, core_x, core_y, core_z);
      enu_core = geo::ecef_to_enu({core_x, core_y, core_z},
                                  origin_ecef_, origin_lat_, origin_lon_);
      have_core_enu = isfinite_d(enu_core.x()) && isfinite_d(enu_core.y()) && isfinite_d(enu_core.z());
      double gnss_x, gnss_y, gnss_z;
      geo::llh_to_ecef(last_gnss_lat_rad_, last_gnss_lon_rad_, last_gnss_h_m_, gnss_x, gnss_y, gnss_z);
      enu_gnss = geo::ecef_to_enu({gnss_x, gnss_y, gnss_z},
                                  origin_ecef_, origin_lat_, origin_lon_);
      have_gnss_enu = isfinite_d(enu_gnss.x()) && isfinite_d(enu_gnss.y()) &&
                      isfinite_d(enu_gnss.z());

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
          fallback_to_gnss_pose_on_large_core_diff_
            ? "Core-vs-GNSS diff %.2f m exceeds %.2f m; publishing GNSS pose fallback on /kf_gins/odom and path"
            : "Core-vs-GNSS diff %.2f m exceeds %.2f m; keeping raw IEKF pose on odom/path for diagnosis",
          diff_m, publish_max_core_gnss_diff_m_);
      }

      odom_uses_gnss_pose =
        (use_gnss_llh_for_pose_when_disarmed_ && !mavros_armed_) ||
        use_gnss_llh_for_pose_ || !core_llh_aligned_ ||
        (fallback_to_gnss_pose_on_large_core_diff_ && core_vs_gnss_diff_large);

      // 业务输出保留当前行为：当核心尚未对齐或触发 fallback 时继续发布 GNSS pose。
      if (odom_uses_gnss_pose) {
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

    // 业务 odom/path 使用当前发布位置；raw odom 始终保留 core 原始解，便于区分 fallback。
    Eigen::Vector3d enu_vis = enu;
    last_enu_ = enu_vis;
    have_last_enu_ = true;

    tf2::Quaternion q_tf;
    r_enu_from_flu.getRotation(q_tf);
    q_tf.normalize();
    if (!isfinite_d(q_tf.x()) || !isfinite_d(q_tf.y()) ||
        !isfinite_d(q_tf.z()) || !isfinite_d(q_tf.w())) return;

    geometry_msgs::msg::Quaternion q_msg;
    q_msg.x = q_tf.x(); q_msg.y = q_tf.y(); q_msg.z = q_tf.z(); q_msg.w = q_tf.w();

    const auto stamp = now();

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

    auto make_odom_msg = [&](const Eigen::Vector3d & position_enu) {
      nav_msgs::msg::Odometry od;
      od.header.stamp = stamp;
      od.header.frame_id = map_frame_;
      od.child_frame_id  = base_frame_;
      od.pose.pose.position.x = position_enu.x();
      od.pose.pose.position.y = position_enu.y();
      od.pose.pose.position.z = position_enu.z();
      od.pose.pose.orientation = q_msg;
      // GIEngine: [vN, vE, vD] -> ENU: x=E, y=N, z=U
      od.twist.twist.linear.x = st.vE;
      od.twist.twist.linear.y = st.vN;
      od.twist.twist.linear.z = -st.vD;
      return od;
    };

    nav_msgs::msg::Odometry od = make_odom_msg(enu_vis);
    odom_pub_->publish(od);
    if (have_core_enu) {
      odom_raw_pub_->publish(make_odom_msg(enu_core));
    }
    logStatePublishDebug_(
      stamp,
      odom_uses_gnss_pose,
      have_core_enu,
      enu_core,
      enu_vis,
      have_gnss_enu,
      enu_gnss,
      st.vN,
      st.vE,
      st.vD);

    std_msgs::msg::Bool fallback_active_msg;
    fallback_active_msg.data = odom_uses_gnss_pose;
    fallback_active_pub_->publish(fallback_active_msg);

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
  struct HeadingMotionContext
  {
    bool have_fresh_mavros_speed{false};
    bool heading_horizontal_ok{false};
    bool heading_vertical_ok{false};
    bool armed_motion_ok{false};
    bool vertical_dominant_low_horizontal_motion{false};
    bool turn_track_motion_ok{false};
    bool turning_now{false};
    bool recent_turning{false};
    bool post_turn_hold_active{false};
    bool post_turn_context{false};
    bool post_turn_cruise_track_continue_active{false};
    bool post_turn_track_motion_ok{false};
    bool post_turn_underreaction_motion_ok{false};
    bool post_turn_low_hspeed_cluster_ok{false};
    bool post_turn_cruise_motion_ok{false};
    bool armed_cruise_force_relock_context{false};
    bool native_velocity_tightening_context{false};
    bool armed_cruise_force_relock_gyro_ok{false};
    bool armed_cruise_force_relock_source_rate_ok{false};
  };

  struct HeadingUpdateDebugEvent
  {
    const char* event_type{"ingest"};
    const char* heading_mode{"unknown"};
    double ros_time_sec{std::numeric_limits<double>::quiet_NaN()};
    double update_time_sec{std::numeric_limits<double>::quiet_NaN()};
    double residual_before_deg{std::numeric_limits<double>::quiet_NaN()};
    double residual_after_deg{std::numeric_limits<double>::quiet_NaN()};
    double core_yaw_before_deg{std::numeric_limits<double>::quiet_NaN()};
    double core_yaw_after_deg{std::numeric_limits<double>::quiet_NaN()};
    double raw_heading_deg{std::numeric_limits<double>::quiet_NaN()};
    double measurement_heading_deg{std::numeric_limits<double>::quiet_NaN()};
    double measurement_std_deg{std::numeric_limits<double>::quiet_NaN()};
    double innovation_gate_deg{std::numeric_limits<double>::quiet_NaN()};
    double yaw_correction_abs_deg{std::numeric_limits<double>::quiet_NaN()};
    bool armed{false};
    bool turning_now{false};
    bool recent_turning{false};
    bool post_turn_hold_active{false};
    bool post_turn_context{false};
    bool post_turn_track_motion_ok{false};
    bool post_turn_cruise_motion_ok{false};
    bool post_turn_low_hspeed_cluster_ok{false};
    bool post_turn_cruise_track_continue_active{false};
    bool armed_cruise_context{false};
    bool armed_cruise_gyro_ok{false};
    bool armed_cruise_source_rate_ok{false};
    bool post_turn_armed_cruise_track_motion_ok{false};
    bool post_turn_window_active_before{false};
    bool post_turn_followthrough_active_before{false};
    bool post_turn_context_active_before{false};
    bool post_turn_window_active_after{false};
    bool post_turn_followthrough_active_after{false};
    bool post_turn_context_active_after{false};
    double post_turn_window_until_before_sec{std::numeric_limits<double>::quiet_NaN()};
    double post_turn_followthrough_until_before_sec{std::numeric_limits<double>::quiet_NaN()};
    int post_turn_followthrough_remaining_before{0};
    double post_turn_window_until_after_sec{std::numeric_limits<double>::quiet_NaN()};
    double post_turn_followthrough_until_after_sec{std::numeric_limits<double>::quiet_NaN()};
    int post_turn_followthrough_remaining_after{0};
    bool used_post_turn_cruise_track{false};
    bool used_armed_cruise_track{false};
    bool post_turn_cruise_track_continue_needed{false};
    bool heading_update_underreacted{false};
    double horizontal_speed_mps{std::numeric_limits<double>::quiet_NaN()};
    double vertical_speed_mps{std::numeric_limits<double>::quiet_NaN()};
    double speed_mps{std::numeric_limits<double>::quiet_NaN()};
    double gyro_deg_s{std::numeric_limits<double>::quiet_NaN()};
    double source_yaw_rate_deg_s{std::numeric_limits<double>::quiet_NaN()};
    int heading_large_residual_skip_count{0};
    int post_turn_blocked_count{0};
    int armed_cruise_blocked_count{0};
  };

  struct PendingGnssDebugContext
  {
    bool valid{false};
    bool native_velocity_valid{false};
    bool native_velocity_used{false};
    bool native_velocity_override_active{false};
    bool velocity_residual_boost_active{false};
    bool position_override_active{false};
    bool position_residual_boost_active{false};
    bool vertical_cov_reopen_active{false};
    bool vertical_cov_reopen_applied{false};
    bool post_flight_vertical_cov_reopen_active{false};
    bool post_flight_vertical_cov_reopen_applied{false};
    bool armed{false};
    bool have_fresh_speed{false};
    bool turning_now{false};
    bool post_turn_context{false};
    bool armed_cruise_context{false};
    bool native_velocity_tightening_context{false};
    bool medium_gap_active{false};
    bool medium_gap_segmented{false};
    bool medium_gap_conservative_single_step{false};
    double ros_time_sec{std::numeric_limits<double>::quiet_NaN()};
    double update_time_sec{std::numeric_limits<double>::quiet_NaN()};
    double last_position_residual_h_m{std::numeric_limits<double>::quiet_NaN()};
    double last_position_residual_u_m{std::numeric_limits<double>::quiet_NaN()};
    double last_velocity_residual_h_mps{std::numeric_limits<double>::quiet_NaN()};
    double last_velocity_residual_e_mps{std::numeric_limits<double>::quiet_NaN()};
    double last_velocity_residual_d_mps{std::numeric_limits<double>::quiet_NaN()};
    double core_gnss_diff_h_m{std::numeric_limits<double>::quiet_NaN()};
    double core_gnss_diff_u_m{std::numeric_limits<double>::quiet_NaN()};
    double core_pos_std_d_m{std::numeric_limits<double>::quiet_NaN()};
    double core_vel_std_d_mps{std::numeric_limits<double>::quiet_NaN()};
    double core_accbias_std_z_mps2{std::numeric_limits<double>::quiet_NaN()};
    double horizontal_speed_mps{std::numeric_limits<double>::quiet_NaN()};
    double vertical_speed_mps{std::numeric_limits<double>::quiet_NaN()};
    double gyro_deg_s{std::numeric_limits<double>::quiet_NaN()};
    double source_yaw_rate_deg_s{std::numeric_limits<double>::quiet_NaN()};
    double latest_imu_raw_dt_sec{std::numeric_limits<double>::quiet_NaN()};
    double latest_imu_effective_dt_sec{std::numeric_limits<double>::quiet_NaN()};
    double medium_gap_dropped_dt_sec{std::numeric_limits<double>::quiet_NaN()};
    int medium_gap_segmented_steps{1};
    double native_velocity_std_h_mps{std::numeric_limits<double>::quiet_NaN()};
    double native_velocity_std_u_mps{std::numeric_limits<double>::quiet_NaN()};
    double native_velocity_vN_mps{std::numeric_limits<double>::quiet_NaN()};
    double native_velocity_vE_mps{std::numeric_limits<double>::quiet_NaN()};
    double native_velocity_vD_mps{std::numeric_limits<double>::quiet_NaN()};
    double core_velocity_vN_mps{std::numeric_limits<double>::quiet_NaN()};
    double core_velocity_vE_mps{std::numeric_limits<double>::quiet_NaN()};
    double core_velocity_vD_mps{std::numeric_limits<double>::quiet_NaN()};
  };

  HeadingMotionContext buildHeadingMotionContext_(double now_sec, bool update_turning_history)
  {
    HeadingMotionContext ctx;
    ctx.have_fresh_mavros_speed =
      std::isfinite(last_mavros_speed_mps_) &&
      std::isfinite(last_mavros_horizontal_speed_mps_) &&
      std::isfinite(last_mavros_vertical_speed_mps_) &&
      std::isfinite(last_mavros_velocity_rx_time_sec_) &&
      (now_sec - last_mavros_velocity_rx_time_sec_) <= std::max(0.05, heading_update_max_age_sec_);

    ctx.heading_horizontal_ok =
      !ctx.have_fresh_mavros_speed ||
      heading_update_armed_min_horizontal_speed_mps_ <= 0.0 ||
      last_mavros_horizontal_speed_mps_ >= heading_update_armed_min_horizontal_speed_mps_;
    ctx.heading_vertical_ok =
      !ctx.have_fresh_mavros_speed ||
      heading_update_max_vertical_speed_mps_ <= 0.0 ||
      last_mavros_vertical_speed_mps_ <= heading_update_max_vertical_speed_mps_;
    ctx.armed_motion_ok = ctx.heading_horizontal_ok && ctx.heading_vertical_ok;

    ctx.turning_now =
      std::isfinite(last_imu_gyro_norm_deg_s_) &&
      std::isfinite(imu_gap_turn_rate_gate_deg_s_) &&
      imu_gap_turn_rate_gate_deg_s_ > 0.0 &&
      last_imu_gyro_norm_deg_s_ >= imu_gap_turn_rate_gate_deg_s_;
    // manual36 showed that aggressive turn_track updates during near-hover,
    // vertically dominated turns can seed the large post-turn tail. Keep the
    // stronger turn tracking for flatter or high-horizontal turns, but fall
    // back to the base heading update when vertical motion dominates.
    const bool turn_track_low_horizontal_motion =
      ctx.have_fresh_mavros_speed &&
      last_mavros_horizontal_speed_mps_ < std::max(0.0, heading_update_low_speed_thresh_mps_);
    const bool turn_track_vertical_dominant_motion =
      ctx.have_fresh_mavros_speed &&
      last_mavros_vertical_speed_mps_ > last_mavros_horizontal_speed_mps_;
    ctx.vertical_dominant_low_horizontal_motion =
      turn_track_low_horizontal_motion && turn_track_vertical_dominant_motion;
    ctx.turn_track_motion_ok =
      ctx.turning_now &&
      !ctx.vertical_dominant_low_horizontal_motion;
    if (update_turning_history && ctx.turning_now) {
      last_turning_heading_time_sec_ = now_sec;
      post_turn_cruise_track_continue_until_sec_ = std::numeric_limits<double>::quiet_NaN();
      post_turn_armed_cruise_track_until_sec_ = std::numeric_limits<double>::quiet_NaN();
      post_turn_armed_cruise_track_followthrough_until_sec_ =
        std::numeric_limits<double>::quiet_NaN();
      post_turn_armed_cruise_track_followthrough_remaining_ = 0;
    } else if (!mavros_armed_) {
      post_turn_armed_cruise_track_until_sec_ = std::numeric_limits<double>::quiet_NaN();
      post_turn_armed_cruise_track_followthrough_until_sec_ =
        std::numeric_limits<double>::quiet_NaN();
      post_turn_armed_cruise_track_followthrough_remaining_ = 0;
    }

    ctx.recent_turning =
      heading_post_turn_reacquire_enable_ &&
      mavros_armed_ &&
      !ctx.turning_now &&
      std::isfinite(last_turning_heading_time_sec_) &&
      heading_post_turn_reacquire_window_sec_ > 0.0 &&
      (now_sec - last_turning_heading_time_sec_) <= heading_post_turn_reacquire_window_sec_;
    ctx.post_turn_hold_active =
      heading_post_turn_reacquire_enable_ &&
      mavros_armed_ &&
      std::isfinite(post_turn_hold_end_time_sec_) &&
      heading_post_turn_reacquire_hold_sec_ > 0.0 &&
      now_sec <= post_turn_hold_end_time_sec_;

    ctx.armed_cruise_force_relock_gyro_ok =
      !std::isfinite(last_imu_gyro_norm_deg_s_) ||
      heading_armed_cruise_force_relock_max_gyro_deg_s_ <= 0.0 ||
      last_imu_gyro_norm_deg_s_ <= heading_armed_cruise_force_relock_max_gyro_deg_s_;
    ctx.armed_cruise_force_relock_source_rate_ok =
      !std::isfinite(last_mavros_heading_rate_deg_s_) ||
      heading_armed_cruise_force_relock_max_source_yaw_rate_deg_s_ <= 0.0 ||
      std::abs(last_mavros_heading_rate_deg_s_) <=
        heading_armed_cruise_force_relock_max_source_yaw_rate_deg_s_;
    const bool post_turn_track_base_motion_ok =
      ctx.have_fresh_mavros_speed &&
      last_mavros_horizontal_speed_mps_ >= std::max(0.0, heading_update_low_speed_thresh_mps_) &&
      (heading_armed_cruise_force_relock_max_vertical_speed_mps_ <= 0.0 ||
       last_mavros_vertical_speed_mps_ <=
         heading_armed_cruise_force_relock_max_vertical_speed_mps_);
    const bool post_turn_cruise_track_gyro_ok =
      !std::isfinite(last_imu_gyro_norm_deg_s_) ||
      heading_post_turn_cruise_track_max_gyro_deg_s_ <= 0.0 ||
      last_imu_gyro_norm_deg_s_ <= heading_post_turn_cruise_track_max_gyro_deg_s_;
    const bool post_turn_cruise_track_speed_ok =
      heading_post_turn_cruise_track_min_horizontal_speed_mps_ <= 0.0 ||
      last_mavros_horizontal_speed_mps_ >=
        heading_post_turn_cruise_track_min_horizontal_speed_mps_;
    const bool post_turn_low_hspeed_cluster_speed_ok =
      (heading_post_turn_low_hspeed_cluster_min_horizontal_speed_mps_ <= 0.0 ||
       last_mavros_horizontal_speed_mps_ >=
         heading_post_turn_low_hspeed_cluster_min_horizontal_speed_mps_) &&
      (heading_post_turn_low_hspeed_cluster_max_horizontal_speed_mps_ <= 0.0 ||
       last_mavros_horizontal_speed_mps_ <=
         heading_post_turn_low_hspeed_cluster_max_horizontal_speed_mps_);
    const bool post_turn_low_hspeed_cluster_vertical_ok =
      heading_post_turn_low_hspeed_cluster_max_vertical_speed_mps_ <= 0.0 ||
      std::abs(last_mavros_vertical_speed_mps_) <=
        heading_post_turn_low_hspeed_cluster_max_vertical_speed_mps_;
    const bool post_turn_low_hspeed_cluster_gyro_ok =
      !std::isfinite(last_imu_gyro_norm_deg_s_) ||
      heading_post_turn_low_hspeed_cluster_max_gyro_deg_s_ <= 0.0 ||
      last_imu_gyro_norm_deg_s_ <= heading_post_turn_low_hspeed_cluster_max_gyro_deg_s_;
    const bool post_turn_low_hspeed_cluster_source_rate_ok =
      !std::isfinite(last_mavros_heading_rate_deg_s_) ||
      heading_post_turn_low_hspeed_cluster_max_source_yaw_rate_deg_s_ <= 0.0 ||
      std::abs(last_mavros_heading_rate_deg_s_) <=
        heading_post_turn_low_hspeed_cluster_max_source_yaw_rate_deg_s_;
    // manual43 baseline keeps continuation on the stricter armed-cruise gyro
    // gate so that the short continuation window does not leak into broader
    // cruise-like drift.
    ctx.post_turn_cruise_track_continue_active =
      heading_post_turn_reacquire_enable_ &&
      mavros_armed_ &&
      !ctx.turning_now &&
      !ctx.recent_turning &&
      std::isfinite(post_turn_cruise_track_continue_until_sec_) &&
      heading_post_turn_cruise_track_continue_sec_ > 0.0 &&
      now_sec <= post_turn_cruise_track_continue_until_sec_ &&
      post_turn_track_base_motion_ok &&
      (heading_armed_cruise_force_relock_min_horizontal_speed_mps_ <= 0.0 ||
       last_mavros_horizontal_speed_mps_ >=
         heading_armed_cruise_force_relock_min_horizontal_speed_mps_) &&
      ctx.armed_cruise_force_relock_gyro_ok &&
      ctx.armed_cruise_force_relock_source_rate_ok;
    ctx.post_turn_context =
      ctx.recent_turning ||
      ctx.post_turn_hold_active;
    // manual33 showed that reusing the strict underreaction gyro/source-rate
    // gate for all post-turn tracking removes the low-speed/high-vertical
    // tail, but also blocks legitimate higher-rate post-turn convergence.
    ctx.post_turn_track_motion_ok =
      (ctx.post_turn_context ||
       ctx.post_turn_cruise_track_continue_active) &&
      post_turn_track_base_motion_ok;
    ctx.post_turn_low_hspeed_cluster_ok =
      heading_post_turn_low_hspeed_cluster_enable_ &&
      mavros_armed_ &&
      ctx.recent_turning &&
      ctx.post_turn_track_motion_ok &&
      post_turn_low_hspeed_cluster_speed_ok &&
      post_turn_low_hspeed_cluster_vertical_ok &&
      post_turn_low_hspeed_cluster_gyro_ok &&
      post_turn_low_hspeed_cluster_source_rate_ok;
    ctx.post_turn_underreaction_motion_ok =
      ctx.recent_turning &&
      ctx.post_turn_track_motion_ok &&
      ctx.armed_cruise_force_relock_gyro_ok &&
      ctx.armed_cruise_force_relock_source_rate_ok;
    // manual42 narrowed the remaining post-turn issue to high-speed windows
    // whose gyro is slightly above the stricter 2 deg/s armed-cruise relock
    // gate. Keep the tighter underreaction gate, but let cruise-track stay
    // active longer in those otherwise stable post-turn convergence samples.
    ctx.post_turn_cruise_motion_ok =
      (ctx.recent_turning ||
       ctx.post_turn_cruise_track_continue_active) &&
      ctx.post_turn_track_motion_ok &&
      (post_turn_cruise_track_speed_ok || ctx.post_turn_low_hspeed_cluster_ok) &&
      post_turn_cruise_track_gyro_ok &&
      ctx.armed_cruise_force_relock_source_rate_ok;

    ctx.armed_cruise_force_relock_context =
      heading_armed_cruise_force_relock_enable_ &&
      mavros_armed_ &&
      !ctx.turning_now &&
      !ctx.post_turn_context &&
      ctx.have_fresh_mavros_speed &&
      (heading_armed_cruise_force_relock_min_horizontal_speed_mps_ <= 0.0 ||
       last_mavros_horizontal_speed_mps_ >=
         heading_armed_cruise_force_relock_min_horizontal_speed_mps_) &&
      (heading_armed_cruise_force_relock_max_vertical_speed_mps_ <= 0.0 ||
       last_mavros_vertical_speed_mps_ <=
         heading_armed_cruise_force_relock_max_vertical_speed_mps_);
    ctx.native_velocity_tightening_context =
      heading_armed_cruise_force_relock_enable_ &&
      mavros_armed_ &&
      ctx.have_fresh_mavros_speed &&
      (armed_cruise_native_gnss_vel_min_horizontal_speed_mps_ <= 0.0 ||
       last_mavros_horizontal_speed_mps_ >=
         armed_cruise_native_gnss_vel_min_horizontal_speed_mps_) &&
      (heading_armed_cruise_force_relock_max_vertical_speed_mps_ <= 0.0 ||
       last_mavros_vertical_speed_mps_ <=
         heading_armed_cruise_force_relock_max_vertical_speed_mps_);
    return ctx;
  }

  const char * observationUpdateModeLabel_(int update_mode) const
  {
    switch (update_mode) {
      case 1:
        return "prev_imu";
      case 2:
        return "curr_imu";
      case 3:
        return "mid_imu";
      default:
        return "unknown";
    }
  }

  void openHeadingUpdateDebugCsv_()
  {
    if (heading_update_debug_csv_path_.empty()) {
      return;
    }

    try {
      const std::filesystem::path csv_path(heading_update_debug_csv_path_);
      if (csv_path.has_parent_path()) {
        std::filesystem::create_directories(csv_path.parent_path());
      }
      heading_update_debug_csv_.open(csv_path, std::ios::out | std::ios::trunc);
    } catch (const std::exception & e) {
      RCLCPP_WARN(
        get_logger(),
        "Failed to open heading update debug CSV %s: %s",
        heading_update_debug_csv_path_.c_str(),
        e.what());
      return;
    }

    if (!heading_update_debug_csv_.is_open()) {
      RCLCPP_WARN(
        get_logger(),
        "Failed to open heading update debug CSV %s",
        heading_update_debug_csv_path_.c_str());
      return;
    }

    heading_update_debug_csv_
      << "sequence,event_type,heading_mode,ros_time_sec,update_time_sec,"
      << "residual_before_deg,residual_after_deg,core_yaw_before_deg,core_yaw_after_deg,"
      << "raw_heading_deg,measurement_heading_deg,measurement_std_deg,innovation_gate_deg,"
      << "yaw_correction_abs_deg,"
      << "armed,turning_now,recent_turning,post_turn_hold_active,post_turn_context,"
      << "post_turn_track_motion_ok,post_turn_cruise_motion_ok,post_turn_low_hspeed_cluster_ok,"
      << "post_turn_cruise_track_continue_active,"
      << "armed_cruise_context,armed_cruise_gyro_ok,armed_cruise_source_rate_ok,"
      << "post_turn_armed_cruise_track_motion_ok,"
      << "post_turn_window_active_before,post_turn_followthrough_active_before,post_turn_context_active_before,"
      << "post_turn_window_active_after,post_turn_followthrough_active_after,post_turn_context_active_after,"
      << "post_turn_window_until_before_sec,post_turn_followthrough_until_before_sec,"
      << "post_turn_followthrough_remaining_before,"
      << "post_turn_window_until_after_sec,post_turn_followthrough_until_after_sec,"
      << "post_turn_followthrough_remaining_after,"
      << "used_post_turn_cruise_track,used_armed_cruise_track,post_turn_cruise_track_continue_needed,"
      << "heading_update_underreacted,"
      << "horizontal_speed_mps,vertical_speed_mps,speed_mps,gyro_deg_s,source_yaw_rate_deg_s,"
      << "heading_large_residual_skip_count,post_turn_blocked_count,armed_cruise_blocked_count\n";
    heading_update_debug_csv_.flush();
    heading_update_debug_rows_since_flush_ = 0;

    RCLCPP_INFO(
      get_logger(),
      "Heading update debug CSV enabled: %s",
      heading_update_debug_csv_path_.c_str());
  }

  void logHeadingUpdateDebug_(const HeadingUpdateDebugEvent & event)
  {
    if (!heading_update_debug_csv_.is_open()) {
      return;
    }

    heading_update_debug_csv_
      << (++heading_update_debug_sequence_) << ','
      << event.event_type << ','
      << event.heading_mode << ','
      << event.ros_time_sec << ','
      << event.update_time_sec << ','
      << event.residual_before_deg << ','
      << event.residual_after_deg << ','
      << event.core_yaw_before_deg << ','
      << event.core_yaw_after_deg << ','
      << event.raw_heading_deg << ','
      << event.measurement_heading_deg << ','
      << event.measurement_std_deg << ','
      << event.innovation_gate_deg << ','
      << event.yaw_correction_abs_deg << ','
      << (event.armed ? 1 : 0) << ','
      << (event.turning_now ? 1 : 0) << ','
      << (event.recent_turning ? 1 : 0) << ','
      << (event.post_turn_hold_active ? 1 : 0) << ','
      << (event.post_turn_context ? 1 : 0) << ','
      << (event.post_turn_track_motion_ok ? 1 : 0) << ','
      << (event.post_turn_cruise_motion_ok ? 1 : 0) << ','
      << (event.post_turn_low_hspeed_cluster_ok ? 1 : 0) << ','
      << (event.post_turn_cruise_track_continue_active ? 1 : 0) << ','
      << (event.armed_cruise_context ? 1 : 0) << ','
      << (event.armed_cruise_gyro_ok ? 1 : 0) << ','
      << (event.armed_cruise_source_rate_ok ? 1 : 0) << ','
      << (event.post_turn_armed_cruise_track_motion_ok ? 1 : 0) << ','
      << (event.post_turn_window_active_before ? 1 : 0) << ','
      << (event.post_turn_followthrough_active_before ? 1 : 0) << ','
      << (event.post_turn_context_active_before ? 1 : 0) << ','
      << (event.post_turn_window_active_after ? 1 : 0) << ','
      << (event.post_turn_followthrough_active_after ? 1 : 0) << ','
      << (event.post_turn_context_active_after ? 1 : 0) << ','
      << event.post_turn_window_until_before_sec << ','
      << event.post_turn_followthrough_until_before_sec << ','
      << event.post_turn_followthrough_remaining_before << ','
      << event.post_turn_window_until_after_sec << ','
      << event.post_turn_followthrough_until_after_sec << ','
      << event.post_turn_followthrough_remaining_after << ','
      << (event.used_post_turn_cruise_track ? 1 : 0) << ','
      << (event.used_armed_cruise_track ? 1 : 0) << ','
      << (event.post_turn_cruise_track_continue_needed ? 1 : 0) << ','
      << (event.heading_update_underreacted ? 1 : 0) << ','
      << event.horizontal_speed_mps << ','
      << event.vertical_speed_mps << ','
      << event.speed_mps << ','
      << event.gyro_deg_s << ','
      << event.source_yaw_rate_deg_s << ','
      << event.heading_large_residual_skip_count << ','
      << event.post_turn_blocked_count << ','
      << event.armed_cruise_blocked_count << '\n';
    ++heading_update_debug_rows_since_flush_;
    if (heading_update_debug_rows_since_flush_ >= heading_update_debug_flush_interval_) {
      heading_update_debug_csv_.flush();
      heading_update_debug_rows_since_flush_ = 0;
    }
  }

  void openStatePublishDebugCsv_()
  {
    if (state_publish_debug_csv_path_.empty()) {
      return;
    }

    try {
      const std::filesystem::path csv_path(state_publish_debug_csv_path_);
      if (csv_path.has_parent_path()) {
        std::filesystem::create_directories(csv_path.parent_path());
      }
      state_publish_debug_csv_.open(csv_path, std::ios::out | std::ios::trunc);
    } catch (const std::exception & e) {
      RCLCPP_WARN(
        get_logger(),
        "Failed to open state publish debug CSV %s: %s",
        state_publish_debug_csv_path_.c_str(),
        e.what());
      return;
    }

    if (!state_publish_debug_csv_.is_open()) {
      RCLCPP_WARN(
        get_logger(),
        "Failed to open state publish debug CSV %s",
        state_publish_debug_csv_path_.c_str());
      return;
    }

    state_publish_debug_csv_
      << "sequence,ros_time_sec,odom_stamp_sec,last_core_time_sec,last_gnss_update_time_sec,"
      << "last_gnss_source_time_sec,last_gnss_rx_ros_time_sec,"
      << "core_time_minus_ros_sec,gnss_update_time_minus_ros_sec,gnss_source_time_minus_ros_sec,"
      << "mavros_armed,core_initialized,odom_uses_gnss_pose,have_core_enu,have_gnss_enu,"
      << "published_enu_e_m,published_enu_n_m,published_enu_u_m,"
      << "core_enu_e_m,core_enu_n_m,core_enu_u_m,"
      << "gnss_enu_e_m,gnss_enu_n_m,gnss_enu_u_m,"
      << "core_minus_gnss_e_m,core_minus_gnss_n_m,core_minus_gnss_u_m,"
      << "core_gnss_diff_h_m,core_gnss_diff_3d_m,"
      << "core_velocity_vN_mps,core_velocity_vE_mps,core_velocity_vD_mps,core_velocity_vU_mps,"
      << "mavros_horizontal_speed_mps,mavros_vertical_speed_mps\n";
    state_publish_debug_csv_.flush();
    state_publish_debug_rows_since_flush_ = 0;

    RCLCPP_INFO(
      get_logger(),
      "State publish debug CSV enabled: %s",
      state_publish_debug_csv_path_.c_str());
  }

  void logStatePublishDebug_(
    const rclcpp::Time & stamp,
    bool odom_uses_gnss_pose,
    bool have_core_enu,
    const Eigen::Vector3d & enu_core,
    const Eigen::Vector3d & enu_published,
    bool have_gnss_enu,
    const Eigen::Vector3d & enu_gnss,
    double core_vN_mps,
    double core_vE_mps,
    double core_vD_mps)
  {
    if (!state_publish_debug_csv_.is_open()) {
      return;
    }

    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double stamp_sec = stamp.seconds();
    const bool have_core_time = std::isfinite(last_core_time_);
    const bool have_gnss_update_time = std::isfinite(last_gnss_time_sec_);
    const bool have_gnss_source_time = std::isfinite(last_gnss_source_time_sec_);
    const bool have_diff = have_core_enu && have_gnss_enu;
    Eigen::Vector3d diff = Eigen::Vector3d::Zero();
    if (have_diff) {
      diff = enu_core - enu_gnss;
    }
    const double diff_h_m = have_diff ? std::hypot(diff.x(), diff.y()) : nan;
    const double diff_3d_m = have_diff ? diff.norm() : nan;
    auto value_if = [nan](bool ok, double value) {
      return ok ? value : nan;
    };

    state_publish_debug_csv_
      << (++state_publish_debug_sequence_) << ','
      << now().seconds() << ','
      << stamp_sec << ','
      << value_if(have_core_time, last_core_time_) << ','
      << value_if(have_gnss_update_time, last_gnss_time_sec_) << ','
      << value_if(have_gnss_source_time, last_gnss_source_time_sec_) << ','
      << last_gnss_rx_ros_time_sec_ << ','
      << value_if(have_core_time, last_core_time_ - stamp_sec) << ','
      << value_if(have_gnss_update_time, last_gnss_time_sec_ - stamp_sec) << ','
      << value_if(have_gnss_source_time, last_gnss_source_time_sec_ - stamp_sec) << ','
      << (mavros_armed_ ? 1 : 0) << ','
      << (core_initialized_ ? 1 : 0) << ','
      << (odom_uses_gnss_pose ? 1 : 0) << ','
      << (have_core_enu ? 1 : 0) << ','
      << (have_gnss_enu ? 1 : 0) << ','
      << enu_published.x() << ','
      << enu_published.y() << ','
      << enu_published.z() << ','
      << value_if(have_core_enu, enu_core.x()) << ','
      << value_if(have_core_enu, enu_core.y()) << ','
      << value_if(have_core_enu, enu_core.z()) << ','
      << value_if(have_gnss_enu, enu_gnss.x()) << ','
      << value_if(have_gnss_enu, enu_gnss.y()) << ','
      << value_if(have_gnss_enu, enu_gnss.z()) << ','
      << value_if(have_diff, diff.x()) << ','
      << value_if(have_diff, diff.y()) << ','
      << value_if(have_diff, diff.z()) << ','
      << diff_h_m << ','
      << diff_3d_m << ','
      << core_vN_mps << ','
      << core_vE_mps << ','
      << core_vD_mps << ','
      << -core_vD_mps << ','
      << last_mavros_horizontal_speed_mps_ << ','
      << last_mavros_vertical_speed_mps_ << '\n';
    ++state_publish_debug_rows_since_flush_;
    if (state_publish_debug_rows_since_flush_ >= state_publish_debug_flush_interval_) {
      state_publish_debug_csv_.flush();
      state_publish_debug_rows_since_flush_ = 0;
    }
  }

  void openGnssUpdateDebugCsv_()
  {
    if (gnss_update_debug_csv_path_.empty()) {
      return;
    }

    try {
      const std::filesystem::path csv_path(gnss_update_debug_csv_path_);
      if (csv_path.has_parent_path()) {
        std::filesystem::create_directories(csv_path.parent_path());
      }
      gnss_update_debug_csv_.open(csv_path, std::ios::out | std::ios::trunc);
    } catch (const std::exception & e) {
      RCLCPP_WARN(
        get_logger(),
        "Failed to open GNSS update debug CSV %s: %s",
        gnss_update_debug_csv_path_.c_str(),
        e.what());
      return;
    }

    if (!gnss_update_debug_csv_.is_open()) {
      RCLCPP_WARN(
        get_logger(),
        "Failed to open GNSS update debug CSV %s",
        gnss_update_debug_csv_path_.c_str());
      return;
    }

    gnss_update_debug_csv_
      << "sequence,ros_time_sec,update_time_sec,update_mode,update_mode_label,"
      << "gnss_position_applied,gnss_velocity_applied,"
      << "gnss_position_residual_n_m,gnss_position_residual_e_m,gnss_position_residual_u_m,"
      << "gnss_position_std_n_m,gnss_position_std_e_m,gnss_position_std_u_m,"
      << "gnss_velocity_residual_n_mps,gnss_velocity_residual_e_mps,gnss_velocity_residual_d_mps,"
      << "gnss_velocity_std_n_mps,gnss_velocity_std_e_mps,gnss_velocity_std_d_mps,"
      << "pending_debug_matched,pending_native_velocity_valid,pending_native_velocity_used,"
      << "native_velocity_override_active,velocity_residual_boost_active,"
      << "position_override_active,position_residual_boost_active,"
      << "vertical_cov_reopen_active,vertical_cov_reopen_applied,"
      << "post_flight_vertical_cov_reopen_active,post_flight_vertical_cov_reopen_applied,"
      << "last_position_residual_h_m,last_position_residual_u_m,last_velocity_residual_h_mps,last_velocity_residual_e_mps,last_velocity_residual_d_mps,"
      << "core_gnss_diff_h_m,core_gnss_diff_u_m,core_pos_std_d_m,core_vel_std_d_mps,core_accbias_std_z_mps2,"
      << "mavros_armed,have_fresh_speed,turning_now,"
      << "post_turn_context,armed_cruise_context,native_velocity_tightening_context,"
      << "medium_gap_active,medium_gap_segmented,medium_gap_conservative_single_step,"
      << "horizontal_speed_mps,vertical_speed_mps,"
      << "gyro_deg_s,source_yaw_rate_deg_s,"
      << "latest_imu_raw_dt_sec,latest_imu_effective_dt_sec,medium_gap_dropped_dt_sec,medium_gap_segmented_steps,"
      << "native_velocity_std_h_mps,native_velocity_std_u_mps,"
      << "native_velocity_vN_mps,native_velocity_vE_mps,native_velocity_vD_mps,"
      << "core_velocity_vN_mps,core_velocity_vE_mps,core_velocity_vD_mps\n";
    gnss_update_debug_csv_.flush();
    gnss_update_debug_rows_since_flush_ = 0;

    RCLCPP_INFO(
      get_logger(),
      "GNSS update debug CSV enabled: %s",
      gnss_update_debug_csv_path_.c_str());
  }

  void logObservationDebugIfNeeded_()
  {
    if (!gnss_update_debug_csv_.is_open() || !core_) {
      return;
    }

    const kfcore::ObservationDebug debug = core_->lastObservationDebug();
    if (!debug.valid || debug.sequence == 0 || debug.sequence == last_logged_observation_debug_sequence_) {
      return;
    }

    const bool pending_debug_matched =
      pending_gnss_debug_context_.valid &&
      std::isfinite(pending_gnss_debug_context_.update_time_sec) &&
      std::isfinite(debug.update_time_sec) &&
      std::abs(pending_gnss_debug_context_.update_time_sec - debug.update_time_sec) <= 1e-6;

    gnss_update_debug_csv_
      << debug.sequence << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.ros_time_sec : std::numeric_limits<double>::quiet_NaN()) << ','
      << debug.update_time_sec << ','
      << debug.update_mode << ','
      << observationUpdateModeLabel_(debug.update_mode) << ','
      << (debug.gnss_position_applied ? 1 : 0) << ','
      << (debug.gnss_velocity_applied ? 1 : 0) << ','
      << debug.gnss_position_residual_neu_m.x() << ','
      << debug.gnss_position_residual_neu_m.y() << ','
      << debug.gnss_position_residual_neu_m.z() << ','
      << debug.gnss_position_std_neu_m.x() << ','
      << debug.gnss_position_std_neu_m.y() << ','
      << debug.gnss_position_std_neu_m.z() << ','
      << debug.gnss_velocity_residual_ned_mps.x() << ','
      << debug.gnss_velocity_residual_ned_mps.y() << ','
      << debug.gnss_velocity_residual_ned_mps.z() << ','
      << debug.gnss_velocity_std_ned_mps.x() << ','
      << debug.gnss_velocity_std_ned_mps.y() << ','
      << debug.gnss_velocity_std_ned_mps.z() << ','
      << (pending_debug_matched ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.native_velocity_valid) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.native_velocity_used) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.native_velocity_override_active) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.velocity_residual_boost_active) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.position_override_active) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.position_residual_boost_active) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.vertical_cov_reopen_active) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.vertical_cov_reopen_applied) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.post_flight_vertical_cov_reopen_active) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.post_flight_vertical_cov_reopen_applied) ? 1 : 0) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.last_position_residual_h_m : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.last_position_residual_u_m : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.last_velocity_residual_h_mps : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.last_velocity_residual_e_mps : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.last_velocity_residual_d_mps : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.core_gnss_diff_h_m : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.core_gnss_diff_u_m : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.core_pos_std_d_m : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.core_vel_std_d_mps : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.core_accbias_std_z_mps2 : std::numeric_limits<double>::quiet_NaN()) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.armed) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.have_fresh_speed) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.turning_now) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.post_turn_context) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.armed_cruise_context) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.native_velocity_tightening_context) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.medium_gap_active) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.medium_gap_segmented) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.medium_gap_conservative_single_step) ? 1 : 0) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.horizontal_speed_mps : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.vertical_speed_mps : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.gyro_deg_s : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.source_yaw_rate_deg_s : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.latest_imu_raw_dt_sec : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.latest_imu_effective_dt_sec : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.medium_gap_dropped_dt_sec : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.medium_gap_segmented_steps : 0) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.native_velocity_std_h_mps : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.native_velocity_std_u_mps : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.native_velocity_vN_mps : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.native_velocity_vE_mps : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.native_velocity_vD_mps : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.core_velocity_vN_mps : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.core_velocity_vE_mps : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.core_velocity_vD_mps : std::numeric_limits<double>::quiet_NaN()) << '\n';
    ++gnss_update_debug_rows_since_flush_;
    if (gnss_update_debug_rows_since_flush_ >= gnss_update_debug_flush_interval_) {
      gnss_update_debug_csv_.flush();
      gnss_update_debug_rows_since_flush_ = 0;
    }

    if (pending_debug_matched) {
      pending_gnss_debug_context_.valid = false;
    }
    last_logged_observation_debug_sequence_ = debug.sequence;
  }

  void updateSpeedSample_(double vx, double vy, double vz, double now_sec)
  {
    const double speed_mps = std::sqrt(vx * vx + vy * vy + vz * vz);
    if (!std::isfinite(speed_mps)) return;
    if (!logged_first_speed_sample_) {
      RCLCPP_INFO(
        get_logger(),
        "Received first speed sample: source=%s topic=%s vx=%.3f vy=%.3f vz=%.3f speed=%.3f",
        speed_source_.c_str(),
        active_speed_topic_name_.c_str(),
        vx, vy, vz, speed_mps);
      logged_first_speed_sample_ = true;
    }
    last_mavros_speed_mps_ = speed_mps;
    last_mavros_horizontal_speed_mps_ = std::sqrt(vx * vx + vy * vy);
    last_mavros_vertical_speed_mps_ = std::abs(vz);
    last_mavros_velocity_rx_time_sec_ = now_sec;
  }

  void updateHeadingSample_(double yaw_ned_deg, double now_sec)
  {
    if (!logged_first_heading_sample_) {
      RCLCPP_INFO(
        get_logger(),
        "Received first heading sample: source=%s topic=%s yaw_ned=%.2f deg",
        heading_source_.c_str(),
        active_heading_topic_name_.c_str(),
        yaw_ned_deg);
      logged_first_heading_sample_ = true;
    }
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
  }

  void updateAttitudeSample_(double roll_ned_deg, double pitch_ned_deg, double yaw_ned_deg, double now_sec)
  {
    mavros_roll_ned_deg_ = roll_ned_deg;
    mavros_pitch_ned_deg_ = pitch_ned_deg;
    have_mavros_tilt_ = std::isfinite(roll_ned_deg) && std::isfinite(pitch_ned_deg);
    last_mavros_attitude_rx_time_sec_ = now_sec;
    updateHeadingSample_(yaw_ned_deg, now_sec);
  }

  void updateNativeSensorGpsVelocityCache_(const px4_msgs::msg::SensorGps & msg)
  {
    if (!msg.vel_ned_valid) {
      return;
    }
    const double vN = static_cast<double>(msg.vel_n_m_s);
    const double vE = static_cast<double>(msg.vel_e_m_s);
    const double vD = static_cast<double>(msg.vel_d_m_s);
    if (!std::isfinite(vN) || !std::isfinite(vE) || !std::isfinite(vD)) {
      return;
    }
    native_sensor_gps_vN_mps_ = vN;
    native_sensor_gps_vE_mps_ = vE;
    native_sensor_gps_vD_mps_ = vD;
    native_sensor_gps_speed_std_mps_ =
      std::isfinite(msg.s_variance_m_s)
        ? static_cast<double>(msg.s_variance_m_s)
        : std::numeric_limits<double>::quiet_NaN();
    last_native_sensor_gps_velocity_sample_time_sec_ =
      use_node_time_for_core_ ? rawTimeSecNow_() : static_cast<double>(msg.timestamp_sample) * 1e-6;
    last_native_sensor_gps_velocity_rx_time_sec_ = now().seconds();
    have_native_sensor_gps_velocity_ = true;
  }

  // ---- params / topics ----
  std::string config_path_, map_frame_, base_frame_, odom_frame_;
  std::string imu_source_, imu_topic_, px4_sensor_combined_topic_, px4_vehicle_imu_topic_;
  std::string gnss_source_{"navsatfix"};
  std::string gnss_topic_;
  std::string px4_sensor_gps_topic_{"/fmu/out/vehicle_gps_position"};
  std::string px4_vehicle_global_position_topic_{"/fmu/out/vehicle_global_position"};
  std::string active_imu_topic_name_;
  std::string active_gnss_topic_name_;
  std::string speed_source_{"mavros_local_velocity"};
  std::string px4_vehicle_local_position_topic_{"/fmu/out/vehicle_local_position"};
  std::string px4_vehicle_odometry_topic_{"/fmu/out/vehicle_odometry"};
  std::string heading_source_{"mavros_imu"};
  std::string mavros_heading_topic_{"/mavros/imu/data"};
  std::string px4_vehicle_attitude_topic_{"/fmu/out/vehicle_attitude"};
  std::string active_speed_topic_name_;
  std::string active_heading_topic_name_;
  bool imu_is_delta_{false};
  bool imu_input_is_flu_{true};
  int px4_imu_qos_depth_{200};
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
  bool have_completed_armed_flight_since_reset_{false};
  double last_armed_transition_time_sec_{std::numeric_limits<double>::quiet_NaN()};
  double last_disarmed_transition_time_sec_{std::numeric_limits<double>::quiet_NaN()};

  // MAVROS 航向 (来自 EKF2 磁力计融合)
  bool have_mavros_heading_{false};
  double mavros_heading_ned_deg_{0.0};
  double last_mavros_heading_rx_time_sec_{std::numeric_limits<double>::quiet_NaN()};
  bool have_prev_mavros_heading_sample_{false};
  bool logged_first_speed_sample_{false};
  bool logged_first_heading_sample_{false};
  bool logged_first_gnss_sample_{false};
  bool logged_first_native_gnss_velocity_sample_{false};
  bool have_mavros_tilt_{false};
  bool have_native_sensor_gps_velocity_{false};
  bool warned_vehicle_odometry_velocity_frame_{false};
  double prev_mavros_heading_sample_deg_{0.0};
  double prev_mavros_heading_sample_time_sec_{std::numeric_limits<double>::quiet_NaN()};
  double last_mavros_heading_rate_deg_s_{std::numeric_limits<double>::quiet_NaN()};
  double last_large_mavros_heading_jump_time_sec_{std::numeric_limits<double>::quiet_NaN()};
  double last_large_mavros_heading_jump_deg_{0.0};
  double last_mavros_speed_mps_{std::numeric_limits<double>::quiet_NaN()};
  double last_mavros_horizontal_speed_mps_{std::numeric_limits<double>::quiet_NaN()};
  double last_mavros_vertical_speed_mps_{std::numeric_limits<double>::quiet_NaN()};
  double last_mavros_velocity_rx_time_sec_{std::numeric_limits<double>::quiet_NaN()};
  double mavros_roll_ned_deg_{0.0};
  double mavros_pitch_ned_deg_{0.0};
  double last_mavros_attitude_rx_time_sec_{std::numeric_limits<double>::quiet_NaN()};
  double native_sensor_gps_vN_mps_{std::numeric_limits<double>::quiet_NaN()};
  double native_sensor_gps_vE_mps_{std::numeric_limits<double>::quiet_NaN()};
  double native_sensor_gps_vD_mps_{std::numeric_limits<double>::quiet_NaN()};
  double native_sensor_gps_speed_std_mps_{std::numeric_limits<double>::quiet_NaN()};
  double last_native_sensor_gps_velocity_sample_time_sec_{std::numeric_limits<double>::quiet_NaN()};
  double last_native_sensor_gps_velocity_rx_time_sec_{std::numeric_limits<double>::quiet_NaN()};
  PendingGnssDebugContext pending_gnss_debug_context_{};
  std::ofstream gnss_update_debug_csv_;
  std::ofstream heading_update_debug_csv_;
  std::ofstream state_publish_debug_csv_;
  std::uint64_t last_logged_observation_debug_sequence_{0};
  std::uint64_t heading_update_debug_sequence_{0};
  std::uint64_t state_publish_debug_sequence_{0};
  std::size_t gnss_update_debug_rows_since_flush_{0};
  std::size_t gnss_update_debug_flush_interval_{20};
  std::size_t heading_update_debug_rows_since_flush_{0};
  std::size_t heading_update_debug_flush_interval_{1};
  std::size_t state_publish_debug_rows_since_flush_{0};
  std::size_t state_publish_debug_flush_interval_{50};

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
  bool enable_native_sensor_gps_velocity_aid_{true};
  double native_sensor_gps_velocity_max_age_sec_{0.5};
  double native_gnss_speed_std_scale_{1.0};
  double gnss_vel_std_floor_h_mps_{0.5};
  double gnss_vel_std_floor_u_mps_{1.0};
  bool armed_cruise_native_gnss_vel_override_enable_{true};
  bool armed_cruise_gnss_pos_override_enable_{false};
  double armed_cruise_gnss_pos_std_h_m_{0.06};
  double armed_cruise_gnss_pos_std_u_m_{0.08};
  bool armed_cruise_gnss_pos_residual_boost_enable_{false};
  double armed_cruise_gnss_pos_residual_boost_threshold_m_{0.12};
  double armed_cruise_gnss_pos_residual_boost_hold_sec_{20.0};
  double armed_cruise_native_gnss_vel_min_horizontal_speed_mps_{0.5};
  double armed_cruise_native_gnss_vel_std_h_mps_{0.05};
  double armed_cruise_native_gnss_vel_std_u_mps_{0.10};
  bool armed_cruise_native_gnss_vel_residual_boost_enable_{false};
  double armed_cruise_native_gnss_vel_residual_boost_threshold_mps_{0.10};
  double armed_cruise_native_gnss_vel_residual_boost_hold_sec_{8.0};
  double armed_cruise_native_gnss_vel_residual_boost_std_h_mps_{0.03};
  double armed_cruise_native_gnss_vel_residual_boost_std_u_mps_{0.08};
  bool armed_cruise_vertical_cov_reopen_enable_{false};
  double armed_cruise_vertical_cov_reopen_threshold_m_{0.15};
  double armed_cruise_vertical_cov_reopen_hold_sec_{8.0};
  double armed_cruise_vertical_cov_reopen_pos_std_m_{0.15};
  double armed_cruise_vertical_cov_reopen_vel_std_mps_{0.10};
  double armed_cruise_vertical_cov_reopen_accbias_std_z_mps2_{0.05};
  bool post_flight_vertical_cov_reopen_enable_{true};
  double post_flight_vertical_cov_reopen_threshold_m_{0.12};
  double post_flight_vertical_cov_reopen_hold_sec_{20.0};
  double post_flight_vertical_cov_reopen_grace_sec_{30.0};
  double post_flight_vertical_cov_reopen_pos_std_m_{0.25};
  double post_flight_vertical_cov_reopen_vel_std_mps_{0.10};
  double post_flight_vertical_cov_reopen_accbias_std_z_mps2_{0.05};
  double armed_cruise_gnss_pos_residual_boost_until_sec_{std::numeric_limits<double>::quiet_NaN()};
  double armed_cruise_native_gnss_vel_residual_boost_until_sec_{std::numeric_limits<double>::quiet_NaN()};
  double armed_cruise_vertical_cov_reopen_until_sec_{std::numeric_limits<double>::quiet_NaN()};
  double post_flight_vertical_cov_reopen_until_sec_{std::numeric_limits<double>::quiet_NaN()};
  std::string gnss_update_debug_csv_path_;
  std::string heading_update_debug_csv_path_;
  std::string state_publish_debug_csv_path_;
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
  double heading_post_turn_cruise_track_std_deg_{2.0};
  double heading_post_turn_cruise_track_max_gyro_deg_s_{3.5};
  double heading_post_turn_cruise_track_min_horizontal_speed_mps_{4.0};
  bool heading_post_turn_low_hspeed_cluster_enable_{false};
  double heading_post_turn_low_hspeed_cluster_min_horizontal_speed_mps_{4.3};
  double heading_post_turn_low_hspeed_cluster_max_horizontal_speed_mps_{4.6};
  double heading_post_turn_low_hspeed_cluster_max_vertical_speed_mps_{0.2};
  double heading_post_turn_low_hspeed_cluster_max_gyro_deg_s_{2.0};
  double heading_post_turn_low_hspeed_cluster_max_source_yaw_rate_deg_s_{0.5};
  double heading_post_turn_reacquire_hold_sec_{4.0};
  double heading_post_turn_reacquire_max_rate_hz_{5.0};
  bool heading_post_turn_force_relock_enable_{true};
  double heading_post_turn_force_relock_min_residual_deg_{20.0};
  double heading_post_turn_cruise_track_continue_sec_{1.5};
  double heading_post_turn_cruise_track_continue_min_residual_deg_{0.8};
  int heading_post_turn_force_relock_min_consecutive_blocks_{3};
  double heading_post_turn_force_relock_yaw_std_deg_{5.0};
  double heading_post_turn_force_relock_max_rate_hz_{1.0};
  bool heading_post_turn_hold_force_relock_enable_{true};
  double heading_post_turn_hold_force_relock_min_residual_deg_{13.0};
  int heading_post_turn_hold_force_relock_min_consecutive_blocks_{20};
  bool heading_armed_cruise_force_relock_enable_{true};
  double heading_armed_cruise_track_min_residual_deg_{0.9};
  double heading_post_turn_armed_cruise_track_window_sec_{0.0};
  double heading_post_turn_armed_cruise_track_min_residual_deg_{0.9};
  double heading_post_turn_armed_cruise_track_min_horizontal_speed_mps_{4.0};
  double heading_post_turn_armed_cruise_track_followthrough_sec_{0.0};
  double heading_armed_cruise_force_relock_min_residual_deg_{13.0};
  int heading_armed_cruise_force_relock_min_consecutive_blocks_{30};
  double heading_armed_cruise_force_relock_min_horizontal_speed_mps_{4.0};
  double heading_armed_cruise_force_relock_max_vertical_speed_mps_{1.0};
  double heading_armed_cruise_force_relock_max_gyro_deg_s_{2.0};
  double heading_armed_cruise_force_relock_max_source_yaw_rate_deg_s_{2.0};
  double heading_armed_cruise_force_relock_yaw_std_deg_{5.0};
  double heading_armed_cruise_force_relock_max_rate_hz_{1.0};
  bool heading_underreaction_force_relock_enable_{true};
  double heading_underreaction_force_relock_min_residual_deg_{1.5};
  double heading_post_turn_soft_underreaction_min_residual_deg_{1.5};
  double heading_underreaction_force_relock_min_remaining_ratio_{0.85};
  double heading_underreaction_force_relock_yaw_std_deg_{2.0};
  double heading_underreaction_force_relock_max_rate_hz_{2.0};
  double heading_underreaction_force_relock_max_gyro_deg_s_{8.0};
  double heading_underreaction_force_relock_max_source_yaw_rate_deg_s_{8.0};
  bool tilt_force_relock_enable_{false};
  double tilt_force_relock_min_residual_deg_{2.0};
  double tilt_force_relock_roll_pitch_std_deg_{1.5};
  bool tilt_force_relock_once_per_motion_context_{true};
  double tilt_force_relock_max_rate_hz_{2.0};
  double tilt_force_relock_max_gyro_deg_s_{8.0};
  double tilt_force_relock_max_vertical_speed_mps_{1.5};
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
  bool fallback_to_gnss_pose_on_large_core_diff_{false};
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
  double post_turn_cruise_track_continue_until_sec_{std::numeric_limits<double>::quiet_NaN()};
  double post_turn_armed_cruise_track_until_sec_{std::numeric_limits<double>::quiet_NaN()};
  double post_turn_armed_cruise_track_followthrough_until_sec_{
    std::numeric_limits<double>::quiet_NaN()};
  int post_turn_armed_cruise_track_followthrough_remaining_{0};
  double last_post_turn_force_relock_time_sec_{std::numeric_limits<double>::quiet_NaN()};
  double last_armed_cruise_force_relock_time_sec_{std::numeric_limits<double>::quiet_NaN()};
  double last_heading_underreaction_force_relock_time_sec_{std::numeric_limits<double>::quiet_NaN()};
  double last_tilt_force_relock_time_sec_{std::numeric_limits<double>::quiet_NaN()};
  int active_tilt_force_relock_context_id_{0};
  bool tilt_force_relock_applied_in_active_context_{false};
  int heading_large_residual_skip_count_{0};
  int post_turn_blocked_count_{0};
  int armed_cruise_blocked_count_{0};
  double last_disarmed_yaw_lock_time_sec_{std::numeric_limits<double>::quiet_NaN()};
  double last_vertical_or_accel_trigger_time_sec_{std::numeric_limits<double>::quiet_NaN()};
  double last_imu_gyro_norm_deg_s_{std::numeric_limits<double>::quiet_NaN()};
  double last_processed_imu_raw_dt_sec_{std::numeric_limits<double>::quiet_NaN()};
  double last_processed_imu_effective_dt_sec_{std::numeric_limits<double>::quiet_NaN()};
  double last_medium_imu_gap_dropped_dt_sec_{0.0};
  bool last_medium_imu_gap_active_{false};
  bool last_medium_imu_gap_segmented_{false};
  bool last_medium_imu_gap_conservative_single_step_{false};
  int last_medium_imu_gap_segmented_steps_{1};


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
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_raw_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr      path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr reset_event_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr fallback_active_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr px4_sensor_combined_sub_;
  rclcpp::Subscription<px4_msgs::msg::SensorGps>::SharedPtr px4_sensor_gps_sub_;
  rclcpp::Subscription<px4_msgs::msg::SensorGps>::SharedPtr px4_sensor_gps_velocity_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr px4_vehicle_attitude_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr px4_vehicle_global_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleImu>::SharedPtr px4_vehicle_imu_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr px4_vehicle_local_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr px4_vehicle_local_position_gnss_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr px4_vehicle_odometry_aux_sub_;
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
  double last_gnss_source_time_sec_{std::numeric_limits<double>::quiet_NaN()};
  double last_gnss_rx_ros_time_sec_{std::numeric_limits<double>::quiet_NaN()};
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
