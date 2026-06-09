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
#include <std_msgs/msg/float64_multi_array.hpp>
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
#include <cstdint>
#include <cstring>
#include <deque>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <string>
#include <vector>

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
    raw_odom_decimation_ =
      std::max(1, static_cast<int>(this->declare_parameter<int>("raw_odom_decimation", 50)));
    core_processing_enable_ = this->declare_parameter<bool>("core_processing_enable", true);
    core_imu_decimation_ =
      std::max(1, static_cast<int>(this->declare_parameter<int>("core_imu_decimation", 1)));
    core_max_imu_rate_hz_ =
      std::max(0.0, this->declare_parameter<double>("core_max_imu_rate_hz", 0.0));
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
    gnss_position_lag_compensation_enable_ =
      this->declare_parameter<bool>("gnss_position_lag_compensation_enable", false);
    gnss_position_lag_compensation_sec_ =
      this->declare_parameter<double>("gnss_position_lag_compensation_sec", 0.25);
    gnss_position_lag_compensation_max_sec_ =
      this->declare_parameter<double>("gnss_position_lag_compensation_max_sec", 0.50);
    gnss_position_lag_compensation_min_speed_mps_ =
      this->declare_parameter<double>("gnss_position_lag_compensation_min_speed_mps", 0.50);
    gnss_position_lag_compensation_armed_only_ =
      this->declare_parameter<bool>("gnss_position_lag_compensation_armed_only", true);
    experiment_initial_reset_offset_enable_ =
      this->declare_parameter<bool>("experiment_initial_reset_offset_enable", false);
    experiment_initial_reset_offset_n_m_ =
      this->declare_parameter<double>("experiment_initial_reset_offset_n_m", 0.0);
    experiment_initial_reset_offset_e_m_ =
      this->declare_parameter<double>("experiment_initial_reset_offset_e_m", 0.0);
    experiment_initial_reset_offset_u_m_ =
      this->declare_parameter<double>("experiment_initial_reset_offset_u_m", 0.0);
    experiment_initial_reset_yaw_offset_deg_ =
      this->declare_parameter<double>("experiment_initial_reset_yaw_offset_deg", 0.0);
    experiment_armed_reset_offset_enable_ =
      this->declare_parameter<bool>("experiment_armed_reset_offset_enable", false);

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
    armed_cruise_gnss_pos_override_apply_mission_ =
      this->declare_parameter<bool>("armed_cruise_gnss_pos_override_apply_mission", true);
    armed_cruise_gnss_pos_override_apply_rtl_ =
      this->declare_parameter<bool>("armed_cruise_gnss_pos_override_apply_rtl", true);
    armed_cruise_gnss_pos_override_apply_other_ =
      this->declare_parameter<bool>("armed_cruise_gnss_pos_override_apply_other", true);
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
    gnss_position_response_boost_enable_ =
      this->declare_parameter<bool>("gnss_position_response_boost_enable", false);
    gnss_position_response_boost_apply_mission_ =
      this->declare_parameter<bool>("gnss_position_response_boost_apply_mission", true);
    gnss_position_response_boost_apply_rtl_ =
      this->declare_parameter<bool>("gnss_position_response_boost_apply_rtl", false);
    gnss_position_response_boost_residual_start_h_m_ =
      this->declare_parameter<double>("gnss_position_response_boost_residual_start_h_m", 0.25);
    gnss_position_response_boost_residual_full_h_m_ =
      this->declare_parameter<double>("gnss_position_response_boost_residual_full_h_m", 0.45);
    gnss_position_response_boost_min_horizontal_speed_mps_ =
      this->declare_parameter<double>("gnss_position_response_boost_min_horizontal_speed_mps", 0.5);
    gnss_position_response_boost_max_abs_vertical_speed_mps_ =
      this->declare_parameter<double>("gnss_position_response_boost_max_abs_vertical_speed_mps", 1.0);
    gnss_position_response_boost_persistence_updates_ =
      std::max(
        1,
        static_cast<int>(
          this->declare_parameter<int>("gnss_position_response_boost_persistence_updates", 1)));
    gnss_position_response_boost_std_min_h_m_ =
      this->declare_parameter<double>("gnss_position_response_boost_std_min_h_m", 0.04);
    gnss_position_response_boost_std_max_h_m_ =
      this->declare_parameter<double>("gnss_position_response_boost_std_max_h_m", 0.06);
    gnss_position_gain_response_enable_ =
      this->declare_parameter<bool>("gnss_position_gain_response_enable", false);
    gnss_position_gain_response_apply_mission_ =
      this->declare_parameter<bool>("gnss_position_gain_response_apply_mission", true);
    gnss_position_gain_response_apply_rtl_ =
      this->declare_parameter<bool>("gnss_position_gain_response_apply_rtl", false);
    gnss_position_gain_response_require_armed_cruise_ =
      this->declare_parameter<bool>("gnss_position_gain_response_require_armed_cruise", true);
    gnss_position_gain_response_block_turning_ =
      this->declare_parameter<bool>("gnss_position_gain_response_block_turning", true);
    gnss_position_gain_response_block_post_turn_ =
      this->declare_parameter<bool>("gnss_position_gain_response_block_post_turn", true);
    gnss_position_gain_response_residual_start_h_m_ =
      this->declare_parameter<double>("gnss_position_gain_response_residual_start_h_m", 0.15);
    gnss_position_gain_response_residual_full_h_m_ =
      this->declare_parameter<double>("gnss_position_gain_response_residual_full_h_m", 0.35);
    gnss_position_gain_response_hnis_start_ =
      this->declare_parameter<double>("gnss_position_gain_response_hnis_start", 6.0);
    gnss_position_gain_response_hnis_full_ =
      this->declare_parameter<double>("gnss_position_gain_response_hnis_full", 20.0);
    gnss_position_gain_response_prev_gain_low_ =
      this->declare_parameter<double>("gnss_position_gain_response_prev_gain_low", 0.10);
    gnss_position_gain_response_prev_gain_high_ =
      this->declare_parameter<double>("gnss_position_gain_response_prev_gain_high", 0.18);
    gnss_position_gain_response_min_horizontal_speed_mps_ =
      this->declare_parameter<double>("gnss_position_gain_response_min_horizontal_speed_mps", 3.0);
    gnss_position_gain_response_max_abs_vertical_speed_mps_ =
      this->declare_parameter<double>("gnss_position_gain_response_max_abs_vertical_speed_mps", 1.0);
    gnss_position_gain_response_persistence_updates_ =
      std::max(
        1,
        static_cast<int>(
          this->declare_parameter<int>("gnss_position_gain_response_persistence_updates", 1)));
    gnss_position_gain_response_std_min_h_m_ =
      this->declare_parameter<double>("gnss_position_gain_response_std_min_h_m", 0.03);
    gnss_position_gain_response_std_max_h_m_ =
      this->declare_parameter<double>("gnss_position_gain_response_std_max_h_m", 0.06);
    gnss_velocity_outward_damping_enable_ =
      this->declare_parameter<bool>("gnss_velocity_outward_damping_enable", false);
    gnss_velocity_outward_damping_apply_mission_ =
      this->declare_parameter<bool>("gnss_velocity_outward_damping_apply_mission", true);
    gnss_velocity_outward_damping_apply_rtl_ =
      this->declare_parameter<bool>("gnss_velocity_outward_damping_apply_rtl", false);
    gnss_velocity_outward_damping_min_core_gnss_diff_h_m_ =
      this->declare_parameter<double>("gnss_velocity_outward_damping_min_core_gnss_diff_h_m", 0.20);
    gnss_velocity_outward_damping_radial_start_mps_ =
      this->declare_parameter<double>("gnss_velocity_outward_damping_radial_start_mps", 0.03);
    gnss_velocity_outward_damping_radial_full_mps_ =
      this->declare_parameter<double>("gnss_velocity_outward_damping_radial_full_mps", 0.12);
    gnss_velocity_outward_damping_min_horizontal_speed_mps_ =
      this->declare_parameter<double>("gnss_velocity_outward_damping_min_horizontal_speed_mps", 0.5);
    gnss_velocity_outward_damping_max_abs_vertical_speed_mps_ =
      this->declare_parameter<double>("gnss_velocity_outward_damping_max_abs_vertical_speed_mps", 1.0);
    gnss_velocity_outward_damping_persistence_updates_ =
      std::max(
        1,
        static_cast<int>(
          this->declare_parameter<int>("gnss_velocity_outward_damping_persistence_updates", 1)));
    gnss_velocity_outward_damping_hold_updates_ =
      std::max(
        0,
        static_cast<int>(
          this->declare_parameter<int>("gnss_velocity_outward_damping_hold_updates", 0)));
    gnss_velocity_outward_damping_std_min_h_mps_ =
      this->declare_parameter<double>("gnss_velocity_outward_damping_std_min_h_mps", 0.02);
    gnss_velocity_outward_damping_std_max_h_mps_ =
      this->declare_parameter<double>("gnss_velocity_outward_damping_std_max_h_mps", 0.05);
    turn_postturn_native_velocity_deweight_enable_ =
      this->declare_parameter<bool>("turn_postturn_native_velocity_deweight_enable", false);
    turn_postturn_native_velocity_deweight_apply_mission_ =
      this->declare_parameter<bool>("turn_postturn_native_velocity_deweight_apply_mission", true);
    turn_postturn_native_velocity_deweight_apply_rtl_ =
      this->declare_parameter<bool>("turn_postturn_native_velocity_deweight_apply_rtl", false);
    turn_postturn_native_velocity_deweight_apply_turning_ =
      this->declare_parameter<bool>("turn_postturn_native_velocity_deweight_apply_turning", true);
    turn_postturn_native_velocity_deweight_apply_post_turn_ =
      this->declare_parameter<bool>("turn_postturn_native_velocity_deweight_apply_post_turn", true);
    turn_postturn_native_velocity_deweight_min_core_gnss_diff_h_m_ =
      this->declare_parameter<double>(
        "turn_postturn_native_velocity_deweight_min_core_gnss_diff_h_m", 0.15);
    turn_postturn_native_velocity_deweight_radial_abs_start_mps_ =
      this->declare_parameter<double>(
        "turn_postturn_native_velocity_deweight_radial_abs_start_mps", 0.08);
    turn_postturn_native_velocity_deweight_radial_abs_full_mps_ =
      this->declare_parameter<double>(
        "turn_postturn_native_velocity_deweight_radial_abs_full_mps", 0.28);
    turn_postturn_native_velocity_deweight_core_residual_start_h_mps_ =
      this->declare_parameter<double>(
        "turn_postturn_native_velocity_deweight_core_residual_start_h_mps", 0.08);
    turn_postturn_native_velocity_deweight_core_residual_full_h_mps_ =
      this->declare_parameter<double>(
        "turn_postturn_native_velocity_deweight_core_residual_full_h_mps", 0.30);
    turn_postturn_native_velocity_deweight_min_horizontal_speed_mps_ =
      this->declare_parameter<double>(
        "turn_postturn_native_velocity_deweight_min_horizontal_speed_mps", 0.5);
    turn_postturn_native_velocity_deweight_max_abs_vertical_speed_mps_ =
      this->declare_parameter<double>(
        "turn_postturn_native_velocity_deweight_max_abs_vertical_speed_mps", 1.0);
    turn_postturn_native_velocity_deweight_persistence_updates_ =
      std::max(
        1,
        static_cast<int>(
          this->declare_parameter<int>(
            "turn_postturn_native_velocity_deweight_persistence_updates", 1)));
    turn_postturn_native_velocity_deweight_std_min_h_mps_ =
      this->declare_parameter<double>(
        "turn_postturn_native_velocity_deweight_std_min_h_mps", 0.10);
    turn_postturn_native_velocity_deweight_std_max_h_mps_ =
      this->declare_parameter<double>(
        "turn_postturn_native_velocity_deweight_std_max_h_mps", 0.18);
    phase_error_memory_debug_enable_ =
      this->declare_parameter<bool>("phase_error_memory_debug_enable", false);
    phase_error_memory_debug_residual_threshold_h_m_ =
      this->declare_parameter<double>("phase_error_memory_debug_residual_threshold_h_m", 0.16);
    phase_error_memory_debug_dx_over_residual_threshold_ =
      this->declare_parameter<double>(
        "phase_error_memory_debug_dx_over_residual_threshold", 0.22);
    phase_error_memory_debug_recent_turnpost_hold_sec_ =
      this->declare_parameter<double>(
        "phase_error_memory_debug_recent_turnpost_hold_sec", 15.0);
    adaptive_gnss_pos_weight_enable_ =
      this->declare_parameter<bool>("adaptive_gnss_pos_weight_enable", false);
    adaptive_gnss_pos_weight_apply_mission_ =
      this->declare_parameter<bool>("adaptive_gnss_pos_weight_apply_mission", true);
    adaptive_gnss_pos_weight_apply_rtl_ =
      this->declare_parameter<bool>("adaptive_gnss_pos_weight_apply_rtl", false);
    adaptive_gnss_pos_weight_trigger_source_ =
      this->declare_parameter<std::string>("adaptive_gnss_pos_weight_trigger_source", "residual_h");
    if (adaptive_gnss_pos_weight_trigger_source_ == "residual") {
      adaptive_gnss_pos_weight_trigger_source_ = "residual_h";
    } else if (adaptive_gnss_pos_weight_trigger_source_ == "nis" ||
               adaptive_gnss_pos_weight_trigger_source_ == "nis_h_2d") {
      adaptive_gnss_pos_weight_trigger_source_ = "nis_h";
    } else if (adaptive_gnss_pos_weight_trigger_source_ != "residual_h" &&
               adaptive_gnss_pos_weight_trigger_source_ != "nis_h") {
      RCLCPP_WARN(
        get_logger(),
        "Unsupported adaptive_gnss_pos_weight_trigger_source='%s'; using residual_h",
        adaptive_gnss_pos_weight_trigger_source_.c_str());
      adaptive_gnss_pos_weight_trigger_source_ = "residual_h";
    }
    adaptive_gnss_pos_weight_floor_min_h_m_ =
      this->declare_parameter<double>("adaptive_gnss_pos_weight_floor_min_h_m", 0.08);
    adaptive_gnss_pos_weight_floor_nominal_h_m_ =
      this->declare_parameter<double>("adaptive_gnss_pos_weight_floor_nominal_h_m", 0.12);
    adaptive_gnss_pos_weight_floor_max_h_m_ =
      this->declare_parameter<double>("adaptive_gnss_pos_weight_floor_max_h_m", 0.16);
    adaptive_gnss_pos_weight_residual_start_h_m_ =
      this->declare_parameter<double>("adaptive_gnss_pos_weight_residual_start_h_m", 0.24);
    adaptive_gnss_pos_weight_residual_full_h_m_ =
      this->declare_parameter<double>("adaptive_gnss_pos_weight_residual_full_h_m", 0.45);
    adaptive_gnss_pos_weight_nis_start_h_2d_ =
      this->declare_parameter<double>("adaptive_gnss_pos_weight_nis_start_h_2d", 5.991);
    adaptive_gnss_pos_weight_nis_full_h_2d_ =
      this->declare_parameter<double>("adaptive_gnss_pos_weight_nis_full_h_2d", 20.0);
    adaptive_gnss_pos_weight_nis_max_age_sec_ =
      this->declare_parameter<double>("adaptive_gnss_pos_weight_nis_max_age_sec", 2.0);
    adaptive_gnss_pos_weight_persistence_updates_ =
      std::max(
        1,
        static_cast<int>(
          this->declare_parameter<int>("adaptive_gnss_pos_weight_persistence_updates", 5)));
    adaptive_gnss_pos_weight_attack_sec_ =
      this->declare_parameter<double>("adaptive_gnss_pos_weight_attack_sec", 0.8);
    adaptive_gnss_pos_weight_decay_sec_ =
      this->declare_parameter<double>("adaptive_gnss_pos_weight_decay_sec", 1.2);
    adaptive_gnss_pos_weight_armed_cruise_gain_ =
      this->declare_parameter<double>("adaptive_gnss_pos_weight_armed_cruise_gain", 1.0);
    adaptive_gnss_pos_weight_turn_gain_ =
      this->declare_parameter<double>("adaptive_gnss_pos_weight_turn_gain", 1.15);
    adaptive_gnss_pos_weight_post_turn_gain_ =
      this->declare_parameter<double>("adaptive_gnss_pos_weight_post_turn_gain", 1.15);
    horizontal_consistency_supervisor_enable_ =
      this->declare_parameter<bool>("horizontal_consistency_supervisor_enable", false);
    horizontal_consistency_apply_mission_ =
      this->declare_parameter<bool>("horizontal_consistency_apply_mission", true);
    horizontal_consistency_apply_rtl_ =
      this->declare_parameter<bool>("horizontal_consistency_apply_rtl", false);
    horizontal_consistency_nis_start_h_2d_ =
      this->declare_parameter<double>("horizontal_consistency_nis_start_h_2d", 5.991);
    horizontal_consistency_nis_full_h_2d_ =
      this->declare_parameter<double>("horizontal_consistency_nis_full_h_2d", 20.0);
    horizontal_consistency_residual_start_h_m_ =
      this->declare_parameter<double>("horizontal_consistency_residual_start_h_m", 0.20);
    horizontal_consistency_residual_full_h_m_ =
      this->declare_parameter<double>("horizontal_consistency_residual_full_h_m", 0.45);
    horizontal_consistency_core_gnss_start_h_m_ =
      this->declare_parameter<double>("horizontal_consistency_core_gnss_start_h_m", 0.20);
    horizontal_consistency_core_gnss_full_h_m_ =
      this->declare_parameter<double>("horizontal_consistency_core_gnss_full_h_m", 0.45);
    horizontal_consistency_heading_residual_start_deg_ =
      this->declare_parameter<double>("horizontal_consistency_heading_residual_start_deg", 0.8);
    horizontal_consistency_heading_residual_full_deg_ =
      this->declare_parameter<double>("horizontal_consistency_heading_residual_full_deg", 2.0);
    horizontal_consistency_score_trigger_ =
      this->declare_parameter<double>("horizontal_consistency_score_trigger", 0.75);
    horizontal_consistency_persistence_updates_ =
      std::max(
        1,
        static_cast<int>(
          this->declare_parameter<int>("horizontal_consistency_persistence_updates", 3)));
    horizontal_consistency_min_horizontal_speed_mps_ =
      this->declare_parameter<double>("horizontal_consistency_min_horizontal_speed_mps", 3.0);
    horizontal_consistency_max_vertical_speed_mps_ =
      this->declare_parameter<double>("horizontal_consistency_max_vertical_speed_mps", 0.8);
    mission_cov_hygiene_enable_ =
      this->declare_parameter<bool>("mission_cov_hygiene_enable", false);
    mission_cov_hygiene_apply_mission_ =
      this->declare_parameter<bool>("mission_cov_hygiene_apply_mission", true);
    mission_cov_hygiene_apply_rtl_ =
      this->declare_parameter<bool>("mission_cov_hygiene_apply_rtl", false);
    mission_cov_hygiene_hnis_start_ =
      this->declare_parameter<double>("mission_cov_hygiene_hnis_start", 6.0);
    mission_cov_hygiene_hnis_full_ =
      this->declare_parameter<double>("mission_cov_hygiene_hnis_full", 20.0);
    mission_cov_hygiene_resid_start_h_m_ =
      this->declare_parameter<double>("mission_cov_hygiene_resid_start_h_m", 0.15);
    mission_cov_hygiene_resid_full_h_m_ =
      this->declare_parameter<double>("mission_cov_hygiene_resid_full_h_m", 0.35);
    mission_cov_hygiene_pos_std_tight_lo_h_m_ =
      this->declare_parameter<double>("mission_cov_hygiene_pos_std_tight_lo_h_m", 0.025);
    mission_cov_hygiene_pos_std_tight_hi_h_m_ =
      this->declare_parameter<double>("mission_cov_hygiene_pos_std_tight_hi_h_m", 0.050);
    mission_cov_hygiene_dx_ratio_low_ =
      this->declare_parameter<double>("mission_cov_hygiene_dx_ratio_low", 0.15);
    mission_cov_hygiene_dx_ratio_high_ =
      this->declare_parameter<double>("mission_cov_hygiene_dx_ratio_high", 0.35);
    mission_cov_hygiene_persistence_updates_ =
      std::max(
        1,
        static_cast<int>(
          this->declare_parameter<int>("mission_cov_hygiene_persistence_updates", 3)));
    mission_cov_hygiene_attack_sec_ =
      this->declare_parameter<double>("mission_cov_hygiene_attack_sec", 4.0);
    mission_cov_hygiene_decay_sec_ =
      this->declare_parameter<double>("mission_cov_hygiene_decay_sec", 15.0);
    mission_cov_hygiene_floor_min_h_m_ =
      this->declare_parameter<double>("mission_cov_hygiene_floor_min_h_m", 0.06);
    mission_cov_hygiene_floor_nominal_h_m_ =
      this->declare_parameter<double>("mission_cov_hygiene_floor_nominal_h_m", 0.08);
    mission_cov_hygiene_floor_max_h_m_ =
      this->declare_parameter<double>("mission_cov_hygiene_floor_max_h_m", 0.12);
    mission_cov_hygiene_offdiag_corr_limit_ =
      this->declare_parameter<double>("mission_cov_hygiene_offdiag_corr_limit", 0.70);
    turn_rate_propagation_noise_probe_enable_ =
      this->declare_parameter<bool>("turn_rate_propagation_noise_probe_enable", false);
    turn_rate_propagation_noise_probe_apply_mission_ =
      this->declare_parameter<bool>("turn_rate_propagation_noise_probe_apply_mission", true);
    turn_rate_propagation_noise_probe_apply_rtl_ =
      this->declare_parameter<bool>("turn_rate_propagation_noise_probe_apply_rtl", false);
    turn_rate_propagation_noise_probe_gyro_start_deg_s_ =
      this->declare_parameter<double>("turn_rate_propagation_noise_probe_gyro_start_deg_s", 6.0);
    turn_rate_propagation_noise_probe_gyro_full_deg_s_ =
      this->declare_parameter<double>("turn_rate_propagation_noise_probe_gyro_full_deg_s", 20.0);
    turn_rate_propagation_noise_probe_min_horizontal_speed_mps_ =
      this->declare_parameter<double>("turn_rate_propagation_noise_probe_min_horizontal_speed_mps", 0.5);
    turn_rate_propagation_noise_probe_max_abs_vertical_speed_mps_ =
      this->declare_parameter<double>("turn_rate_propagation_noise_probe_max_abs_vertical_speed_mps", 1.0);
    turn_rate_propagation_noise_probe_arw_q_scale_max_ =
      this->declare_parameter<double>("turn_rate_propagation_noise_probe_arw_q_scale_max", 9.0);
    turn_rate_propagation_noise_probe_vrw_q_scale_max_ =
      this->declare_parameter<double>("turn_rate_propagation_noise_probe_vrw_q_scale_max", 4.0);
    turn_rate_propagation_noise_probe_gyrbias_q_scale_max_ =
      this->declare_parameter<double>("turn_rate_propagation_noise_probe_gyrbias_q_scale_max", 4.0);
    turn_rate_propagation_noise_probe_accbias_q_scale_max_ =
      this->declare_parameter<double>("turn_rate_propagation_noise_probe_accbias_q_scale_max", 2.0);
    accbias_z_propagation_probe_enable_ =
      this->declare_parameter<bool>("accbias_z_propagation_probe_enable", false);
    accbias_z_propagation_probe_apply_noise_scale_ =
      this->declare_parameter<bool>("accbias_z_propagation_probe_apply_noise_scale", false);
    accbias_z_propagation_probe_apply_mission_ =
      this->declare_parameter<bool>("accbias_z_propagation_probe_apply_mission", true);
    accbias_z_propagation_probe_apply_rtl_ =
      this->declare_parameter<bool>("accbias_z_propagation_probe_apply_rtl", false);
    accbias_z_propagation_probe_trigger_mps2_ =
      this->declare_parameter<double>("accbias_z_propagation_probe_trigger_mps2", -0.18);
    accbias_z_propagation_probe_full_mps2_ =
      this->declare_parameter<double>("accbias_z_propagation_probe_full_mps2", -0.24);
    accbias_z_propagation_probe_min_horizontal_speed_mps_ =
      this->declare_parameter<double>("accbias_z_propagation_probe_min_horizontal_speed_mps", 3.0);
    accbias_z_propagation_probe_max_abs_vertical_speed_mps_ =
      this->declare_parameter<double>("accbias_z_propagation_probe_max_abs_vertical_speed_mps", 1.0);
    accbias_z_propagation_probe_arw_q_scale_max_ =
      this->declare_parameter<double>("accbias_z_propagation_probe_arw_q_scale_max", 1.0);
    accbias_z_propagation_probe_vrw_q_scale_max_ =
      this->declare_parameter<double>("accbias_z_propagation_probe_vrw_q_scale_max", 3.0);
    accbias_z_propagation_probe_gyrbias_q_scale_max_ =
      this->declare_parameter<double>("accbias_z_propagation_probe_gyrbias_q_scale_max", 1.0);
    accbias_z_propagation_probe_accbias_q_scale_max_ =
      this->declare_parameter<double>("accbias_z_propagation_probe_accbias_q_scale_max", 6.0);
    adaptive_rq_source_gate_enable_ =
      this->declare_parameter<bool>("adaptive_rq_source_gate_enable", true);
    adaptive_rq_source_gate_low_speed_max_mps_ =
      this->declare_parameter<double>("adaptive_rq_source_gate_low_speed_max_mps", 3.5);
    motion_gnss_pos_weight_enable_ =
      this->declare_parameter<bool>("motion_gnss_pos_weight_enable", false);
    motion_gnss_pos_weight_apply_mission_ =
      this->declare_parameter<bool>("motion_gnss_pos_weight_apply_mission", true);
    motion_gnss_pos_weight_apply_rtl_ =
      this->declare_parameter<bool>("motion_gnss_pos_weight_apply_rtl", false);
    motion_gnss_pos_weight_hspeed_start_mps_ =
      this->declare_parameter<double>("motion_gnss_pos_weight_hspeed_start_mps", 3.5);
    motion_gnss_pos_weight_hspeed_full_mps_ =
      this->declare_parameter<double>("motion_gnss_pos_weight_hspeed_full_mps", 5.0);
    motion_gnss_pos_weight_max_abs_vspeed_mps_ =
      this->declare_parameter<double>("motion_gnss_pos_weight_max_abs_vspeed_mps", 0.35);
    motion_gnss_pos_weight_std_min_h_m_ =
      this->declare_parameter<double>("motion_gnss_pos_weight_std_min_h_m", 0.08);
    motion_gnss_pos_weight_std_max_h_m_ =
      this->declare_parameter<double>("motion_gnss_pos_weight_std_max_h_m", 0.14);
    gnss_pos_recovery_weight_enable_ =
      this->declare_parameter<bool>("gnss_pos_recovery_weight_enable", false);
    gnss_pos_recovery_weight_apply_mission_ =
      this->declare_parameter<bool>("gnss_pos_recovery_weight_apply_mission", true);
    gnss_pos_recovery_weight_apply_rtl_ =
      this->declare_parameter<bool>("gnss_pos_recovery_weight_apply_rtl", false);
    gnss_pos_recovery_weight_residual_start_h_m_ =
      this->declare_parameter<double>("gnss_pos_recovery_weight_residual_start_h_m", 0.32);
    gnss_pos_recovery_weight_residual_full_h_m_ =
      this->declare_parameter<double>("gnss_pos_recovery_weight_residual_full_h_m", 0.45);
    gnss_pos_recovery_weight_core_start_h_m_ =
      this->declare_parameter<double>("gnss_pos_recovery_weight_core_start_h_m", 0.30);
    gnss_pos_recovery_weight_core_full_h_m_ =
      this->declare_parameter<double>("gnss_pos_recovery_weight_core_full_h_m", 0.55);
    gnss_pos_recovery_weight_core_score_gain_ =
      this->declare_parameter<double>("gnss_pos_recovery_weight_core_score_gain", 0.25);
    gnss_pos_recovery_weight_min_horizontal_speed_mps_ =
      this->declare_parameter<double>("gnss_pos_recovery_weight_min_horizontal_speed_mps", 3.0);
    gnss_pos_recovery_weight_max_abs_vertical_speed_mps_ =
      this->declare_parameter<double>("gnss_pos_recovery_weight_max_abs_vertical_speed_mps", 0.35);
    gnss_pos_recovery_weight_persistence_updates_ =
      std::max(
        1,
        static_cast<int>(
          this->declare_parameter<int>("gnss_pos_recovery_weight_persistence_updates", 3)));
    gnss_pos_recovery_weight_hold_sec_ =
      this->declare_parameter<double>("gnss_pos_recovery_weight_hold_sec", 3.0);
    gnss_pos_recovery_weight_attack_sec_ =
      this->declare_parameter<double>("gnss_pos_recovery_weight_attack_sec", 0.5);
    gnss_pos_recovery_weight_decay_sec_ =
      this->declare_parameter<double>("gnss_pos_recovery_weight_decay_sec", 2.0);
    gnss_pos_recovery_weight_std_min_h_m_ =
      this->declare_parameter<double>("gnss_pos_recovery_weight_std_min_h_m", 0.12);
    gnss_pos_recovery_weight_std_max_h_m_ =
      this->declare_parameter<double>("gnss_pos_recovery_weight_std_max_h_m", 0.16);
    context_gnss_pos_floor_enable_ =
      this->declare_parameter<bool>("context_gnss_pos_floor_enable", false);
    context_gnss_pos_floor_apply_mission_ =
      this->declare_parameter<bool>("context_gnss_pos_floor_apply_mission", true);
    context_gnss_pos_floor_apply_rtl_ =
      this->declare_parameter<bool>("context_gnss_pos_floor_apply_rtl", false);
    context_gnss_pos_floor_mission_base_h_m_ =
      this->declare_parameter<double>("context_gnss_pos_floor_mission_base_h_m", 0.08);
    context_gnss_pos_floor_turn_post_h_m_ =
      this->declare_parameter<double>("context_gnss_pos_floor_turn_post_h_m", 0.10);
    context_gnss_pos_floor_armed_cruise_h_m_ =
      this->declare_parameter<double>("context_gnss_pos_floor_armed_cruise_h_m", 0.09);
    context_gnss_pos_floor_rtl_h_m_ =
      this->declare_parameter<double>("context_gnss_pos_floor_rtl_h_m", 0.08);
    context_gnss_pos_floor_attack_sec_ =
      this->declare_parameter<double>("context_gnss_pos_floor_attack_sec", 0.5);
    context_gnss_pos_floor_decay_sec_ =
      this->declare_parameter<double>("context_gnss_pos_floor_decay_sec", 1.5);
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
    native_gnss_velocity_outlier_guard_enable_ =
      this->declare_parameter<bool>("native_gnss_velocity_outlier_guard_enable", false);
    native_gnss_velocity_outlier_guard_apply_mission_ =
      this->declare_parameter<bool>("native_gnss_velocity_outlier_guard_apply_mission", true);
    native_gnss_velocity_outlier_guard_apply_rtl_ =
      this->declare_parameter<bool>("native_gnss_velocity_outlier_guard_apply_rtl", false);
    native_gnss_velocity_outlier_guard_speed_mismatch_mps_ =
      this->declare_parameter<double>("native_gnss_velocity_outlier_guard_speed_mismatch_mps", 0.60);
    native_gnss_velocity_outlier_guard_core_residual_h_mps_ =
      this->declare_parameter<double>("native_gnss_velocity_outlier_guard_core_residual_h_mps", 0.80);
    native_gnss_velocity_outlier_guard_min_horizontal_speed_mps_ =
      this->declare_parameter<double>("native_gnss_velocity_outlier_guard_min_horizontal_speed_mps", 1.0);
    native_gnss_velocity_outlier_guard_max_abs_vertical_speed_mps_ =
      this->declare_parameter<double>("native_gnss_velocity_outlier_guard_max_abs_vertical_speed_mps", 0.5);
    native_gnss_velocity_outlier_guard_apply_turning_context_ =
      this->declare_parameter<bool>("native_gnss_velocity_outlier_guard_apply_turning_context", true);
    native_gnss_velocity_outlier_guard_action_ =
      this->declare_parameter<std::string>("native_gnss_velocity_outlier_guard_action", "skip");
    if (native_gnss_velocity_outlier_guard_action_ == "downweight") {
      native_gnss_velocity_outlier_guard_action_ = "reweight";
    } else if (native_gnss_velocity_outlier_guard_action_ != "skip" &&
               native_gnss_velocity_outlier_guard_action_ != "reweight") {
      RCLCPP_WARN(
        get_logger(),
        "Unsupported native_gnss_velocity_outlier_guard_action='%s'; using skip",
        native_gnss_velocity_outlier_guard_action_.c_str());
      native_gnss_velocity_outlier_guard_action_ = "skip";
    }
    native_gnss_velocity_outlier_guard_reweight_std_h_mps_ =
      this->declare_parameter<double>("native_gnss_velocity_outlier_guard_reweight_std_h_mps", 0.12);
    native_gnss_velocity_outlier_guard_reweight_std_u_mps_ =
      this->declare_parameter<double>("native_gnss_velocity_outlier_guard_reweight_std_u_mps", 0.20);
    native_gnss_velocity_low_speed_turn_source_guard_enable_ =
      this->declare_parameter<bool>("native_gnss_velocity_low_speed_turn_source_guard_enable", false);
    native_gnss_velocity_low_speed_turn_source_guard_max_horizontal_speed_mps_ =
      this->declare_parameter<double>(
        "native_gnss_velocity_low_speed_turn_source_guard_max_horizontal_speed_mps", 2.2);
    native_gnss_velocity_low_speed_turn_source_guard_min_gyro_deg_s_ =
      this->declare_parameter<double>(
        "native_gnss_velocity_low_speed_turn_source_guard_min_gyro_deg_s", 15.0);
    native_gnss_velocity_low_speed_turn_source_guard_core_residual_h_mps_ =
      this->declare_parameter<double>(
        "native_gnss_velocity_low_speed_turn_source_guard_core_residual_h_mps", 1.0);
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
    terminal_descent_observation_enable_ =
      this->declare_parameter<bool>("terminal_descent_observation_enable", false);
    terminal_descent_require_rtl_mode_ =
      this->declare_parameter<bool>("terminal_descent_require_rtl_mode", true);
    terminal_descent_max_horizontal_speed_mps_ =
      this->declare_parameter<double>("terminal_descent_max_horizontal_speed_mps", 1.6);
    terminal_descent_min_vertical_speed_mps_ =
      this->declare_parameter<double>("terminal_descent_min_vertical_speed_mps", 0.30);
    terminal_descent_max_gyro_deg_s_ =
      this->declare_parameter<double>("terminal_descent_max_gyro_deg_s", 30.0);
    terminal_descent_max_source_yaw_rate_deg_s_ =
      this->declare_parameter<double>("terminal_descent_max_source_yaw_rate_deg_s", 30.0);
    terminal_descent_min_armed_time_sec_ =
      this->declare_parameter<double>("terminal_descent_min_armed_time_sec", 0.0);
    terminal_descent_native_gnss_vel_override_enable_ =
      this->declare_parameter<bool>("terminal_descent_native_gnss_vel_override_enable", true);
    terminal_descent_native_gnss_vel_std_h_mps_ =
      this->declare_parameter<double>("terminal_descent_native_gnss_vel_std_h_mps", 0.03);
    terminal_descent_native_gnss_vel_std_u_mps_ =
      this->declare_parameter<double>("terminal_descent_native_gnss_vel_std_u_mps", 0.05);
    terminal_descent_vertical_cov_reopen_enable_ =
      this->declare_parameter<bool>("terminal_descent_vertical_cov_reopen_enable", true);
    terminal_descent_vertical_cov_reopen_pos_std_m_ =
      this->declare_parameter<double>("terminal_descent_vertical_cov_reopen_pos_std_m", 0.15);
    terminal_descent_vertical_cov_reopen_vel_std_mps_ =
      this->declare_parameter<double>("terminal_descent_vertical_cov_reopen_vel_std_mps", 0.05);
    terminal_descent_vertical_cov_reopen_accbias_std_z_mps2_ =
      this->declare_parameter<double>("terminal_descent_vertical_cov_reopen_accbias_std_z_mps2", 0.05);
    terminal_descent_horizontal_zero_vel_enable_ =
      this->declare_parameter<bool>("terminal_descent_horizontal_zero_vel_enable", false);
    terminal_descent_horizontal_zero_vel_max_hspeed_mps_ =
      this->declare_parameter<double>("terminal_descent_horizontal_zero_vel_max_hspeed_mps", 0.30);
    terminal_descent_horizontal_zero_vel_std_h_mps_ =
      this->declare_parameter<double>("terminal_descent_horizontal_zero_vel_std_h_mps", 0.05);
    terminal_descent_horizontal_zero_vel_std_u_mps_ =
      this->declare_parameter<double>("terminal_descent_horizontal_zero_vel_std_u_mps", 10.0);
    gnss_update_debug_csv_path_ =
      this->declare_parameter<std::string>("gnss_update_debug_csv_path", "");
    gnss_nis_debug_csv_path_ =
      this->declare_parameter<std::string>("gnss_nis_debug_csv_path", "");
    gnss_nis_debug_max_rate_hz_ =
      std::max(0.0, this->declare_parameter<double>("gnss_nis_debug_max_rate_hz", 2.0));
    horizontal_consistency_debug_csv_path_ =
      this->declare_parameter<std::string>("horizontal_consistency_debug_csv_path", "");
    heading_update_debug_csv_path_ =
      this->declare_parameter<std::string>("heading_update_debug_csv_path", "");
    state_publish_debug_csv_path_ =
      this->declare_parameter<std::string>("state_publish_debug_csv_path", "");
    shadow_restore_publish_enable_ =
      this->declare_parameter<bool>("shadow_restore_publish_enable", false);
    shadow_restore_subscribe_enable_ =
      this->declare_parameter<bool>("shadow_restore_subscribe_enable", false);
    shadow_restore_topic_ =
      this->declare_parameter<std::string>(
        "shadow_restore_topic", "/kf_gins/main_to_shadow_snapshot");
    shadow_restore_publish_after_core_sec_ =
      this->declare_parameter<double>("shadow_restore_publish_after_core_sec", 40.0);
    shadow_restore_publish_once_ =
      this->declare_parameter<bool>("shadow_restore_publish_once", true);
    shadow_restore_publish_period_sec_ =
      std::max(0.0, this->declare_parameter<double>("shadow_restore_publish_period_sec", 0.0));
    shadow_restore_max_age_sec_ =
      this->declare_parameter<double>("shadow_restore_max_age_sec", 0.75);
    shadow_restore_covariance_inflation_factor_ =
      this->declare_parameter<double>("shadow_restore_covariance_inflation_factor", 1.0);
    shadow_restore_require_core_initialized_ =
      this->declare_parameter<bool>("shadow_restore_require_core_initialized", true);
    shadow_restore_clear_path_on_apply_ =
      this->declare_parameter<bool>("shadow_restore_clear_path_on_apply", true);
    shadow_restore_event_csv_path_ =
      this->declare_parameter<std::string>("shadow_restore_event_csv_path", "");
    shadow_supervisor_fsm_debug_enable_ =
      this->declare_parameter<bool>("shadow_supervisor_fsm_debug_enable", false);
    shadow_supervisor_fsm_reference_odom_topic_ =
      this->declare_parameter<std::string>(
        "shadow_supervisor_fsm_reference_odom_topic", "/kf_gins/odom");
    shadow_supervisor_fsm_debug_csv_path_ =
      this->declare_parameter<std::string>("shadow_supervisor_fsm_debug_csv_path", "");
    shadow_supervisor_fsm_events_csv_path_ =
      this->declare_parameter<std::string>("shadow_supervisor_fsm_events_csv_path", "");
    shadow_supervisor_fsm_debug_max_rate_hz_ =
      std::max(
        0.0,
        this->declare_parameter<double>("shadow_supervisor_fsm_debug_max_rate_hz", 10.0));
    shadow_supervisor_fsm_reference_odom_max_age_sec_ =
      this->declare_parameter<double>("shadow_supervisor_fsm_reference_odom_max_age_sec", 0.5);
    shadow_supervisor_fsm_gamma_guard_enter_ =
      this->declare_parameter<double>("shadow_supervisor_fsm_gamma_guard_enter", 1.05);
    shadow_supervisor_fsm_gamma_shadow_max_ =
      this->declare_parameter<double>("shadow_supervisor_fsm_gamma_shadow_max", 1.05);
    shadow_supervisor_fsm_process_lambda_enter_ =
      this->declare_parameter<double>("shadow_supervisor_fsm_process_lambda_enter", 1.012);
    shadow_supervisor_fsm_process_score_min_ =
      this->declare_parameter<double>("shadow_supervisor_fsm_process_score_min", 0.30);
    shadow_supervisor_fsm_warmup_sec_ =
      std::max(0.0, this->declare_parameter<double>("shadow_supervisor_fsm_warmup_sec", 5.0));
    shadow_supervisor_fsm_ready_confirm_sec_ =
      std::max(
        0.0,
        this->declare_parameter<double>("shadow_supervisor_fsm_ready_confirm_sec", 2.0));
    shadow_supervisor_fsm_xy_delta_ready_max_m_ =
      this->declare_parameter<double>("shadow_supervisor_fsm_xy_delta_ready_max_m", 0.50);
    shadow_supervisor_fsm_z_delta_ready_max_m_ =
      this->declare_parameter<double>("shadow_supervisor_fsm_z_delta_ready_max_m", 0.50);
    shadow_supervisor_fsm_vel_delta_ready_max_mps_ =
      this->declare_parameter<double>("shadow_supervisor_fsm_vel_delta_ready_max_mps", 0.30);
    shadow_supervisor_fsm_yaw_delta_ready_max_deg_ =
      this->declare_parameter<double>("shadow_supervisor_fsm_yaw_delta_ready_max_deg", 5.0);
    shadow_supervisor_fsm_allow_mixed_trigger_ =
      this->declare_parameter<bool>("shadow_supervisor_fsm_allow_mixed_trigger", false);
    shadow_supervisor_fsm_observation_score_guard_enable_ =
      this->declare_parameter<bool>(
        "shadow_supervisor_fsm_observation_score_guard_enable", false);
    shadow_supervisor_fsm_observation_score_guard_enter_ =
      this->declare_parameter<double>(
        "shadow_supervisor_fsm_observation_score_guard_enter", 0.50);
    shadow_supervisor_perf_proxy_publish_enable_ =
      this->declare_parameter<bool>(
        "shadow_supervisor_perf_proxy_publish_enable", false);
    shadow_supervisor_perf_proxy_subscribe_enable_ =
      this->declare_parameter<bool>(
        "shadow_supervisor_perf_proxy_subscribe_enable", false);
    shadow_supervisor_perf_proxy_topic_ =
      this->declare_parameter<std::string>(
        "shadow_supervisor_perf_proxy_topic", "/kf_gins/supervisor_performance_proxy");
    shadow_supervisor_perf_proxy_max_age_sec_ =
      this->declare_parameter<double>(
        "shadow_supervisor_perf_proxy_max_age_sec", 0.75);
    shadow_supervisor_perf_proxy_short_window_sec_ =
      std::max(
        0.0,
        this->declare_parameter<double>(
          "shadow_supervisor_perf_proxy_short_window_sec", 5.0));
    shadow_supervisor_perf_proxy_long_window_sec_ =
      std::max(
        0.0,
        this->declare_parameter<double>(
          "shadow_supervisor_perf_proxy_long_window_sec", 10.0));
    shadow_supervisor_perf_proxy_residual_gap_soft_max_m_ =
      this->declare_parameter<double>(
        "shadow_supervisor_perf_proxy_residual_gap_soft_max_m", 0.0);
    shadow_supervisor_perf_proxy_nis_gap_permission_max_ =
      this->declare_parameter<double>(
        "shadow_supervisor_perf_proxy_nis_gap_permission_max", 0.0);
    shadow_supervisor_predictive_score_publish_enable_ =
      this->declare_parameter<bool>(
        "shadow_supervisor_predictive_score_publish_enable", false);
    shadow_supervisor_predictive_score_subscribe_enable_ =
      this->declare_parameter<bool>(
        "shadow_supervisor_predictive_score_subscribe_enable", false);
    shadow_supervisor_predictive_score_debug_enable_ =
      this->declare_parameter<bool>(
        "shadow_supervisor_predictive_score_debug_enable", false);
    shadow_supervisor_predictive_score_topic_ =
      this->declare_parameter<std::string>(
        "shadow_supervisor_predictive_score_topic",
        "/kf_gins/supervisor_predictive_score");
    shadow_supervisor_predictive_score_debug_csv_path_ =
      this->declare_parameter<std::string>(
        "shadow_supervisor_predictive_score_debug_csv_path", "");
    shadow_supervisor_predictive_score_source_id_ =
      this->declare_parameter<std::string>(
        "shadow_supervisor_predictive_score_source_id", "rq_shadow");
    shadow_supervisor_predictive_score_max_age_sec_ =
      this->declare_parameter<double>(
        "shadow_supervisor_predictive_score_max_age_sec", 0.75);
    shadow_supervisor_predictive_score_stamp_tolerance_sec_ =
      this->declare_parameter<double>(
        "shadow_supervisor_predictive_score_stamp_tolerance_sec", 0.20);
    shadow_supervisor_predictive_score_short_window_sec_ =
      std::max(
        0.0,
        this->declare_parameter<double>(
          "shadow_supervisor_predictive_score_short_window_sec", 5.0));
    shadow_supervisor_predictive_score_long_window_sec_ =
      std::max(
        0.0,
        this->declare_parameter<double>(
          "shadow_supervisor_predictive_score_long_window_sec", 10.0));
    shadow_supervisor_predictive_score_eval_std_h_m_ =
      std::max(
        0.01,
        this->declare_parameter<double>(
          "shadow_supervisor_predictive_score_eval_std_h_m", 0.10));
    shadow_supervisor_predictive_score_eval_std_u_m_ =
      std::max(
        0.01,
        this->declare_parameter<double>(
          "shadow_supervisor_predictive_score_eval_std_u_m", 0.20));
    shadow_supervisor_velocity_predictive_score_publish_enable_ =
      this->declare_parameter<bool>(
        "shadow_supervisor_velocity_predictive_score_publish_enable", false);
    shadow_supervisor_velocity_predictive_score_subscribe_enable_ =
      this->declare_parameter<bool>(
        "shadow_supervisor_velocity_predictive_score_subscribe_enable", false);
    shadow_supervisor_velocity_predictive_score_debug_enable_ =
      this->declare_parameter<bool>(
        "shadow_supervisor_velocity_predictive_score_debug_enable", false);
    shadow_supervisor_velocity_predictive_score_topic_ =
      this->declare_parameter<std::string>(
        "shadow_supervisor_velocity_predictive_score_topic",
        "/kf_gins/supervisor_velocity_predictive_score");
    shadow_supervisor_velocity_predictive_score_debug_csv_path_ =
      this->declare_parameter<std::string>(
        "shadow_supervisor_velocity_predictive_score_debug_csv_path", "");
    shadow_supervisor_velocity_predictive_score_source_id_ =
      this->declare_parameter<std::string>(
        "shadow_supervisor_velocity_predictive_score_source_id", "rq_shadow");
    shadow_supervisor_velocity_predictive_score_max_age_sec_ =
      this->declare_parameter<double>(
        "shadow_supervisor_velocity_predictive_score_max_age_sec", 0.75);
    shadow_supervisor_velocity_predictive_score_stamp_tolerance_sec_ =
      this->declare_parameter<double>(
        "shadow_supervisor_velocity_predictive_score_stamp_tolerance_sec", 0.20);
    shadow_supervisor_velocity_predictive_score_short_window_sec_ =
      std::max(
        0.0,
        this->declare_parameter<double>(
          "shadow_supervisor_velocity_predictive_score_short_window_sec", 5.0));
    shadow_supervisor_velocity_predictive_score_long_window_sec_ =
      std::max(
        0.0,
        this->declare_parameter<double>(
          "shadow_supervisor_velocity_predictive_score_long_window_sec", 10.0));
    shadow_supervisor_velocity_predictive_score_eval_std_h_mps_ =
      std::max(
        0.01,
        this->declare_parameter<double>(
          "shadow_supervisor_velocity_predictive_score_eval_std_h_mps", 0.20));
    shadow_supervisor_velocity_predictive_score_eval_std_u_mps_ =
      std::max(
        0.01,
        this->declare_parameter<double>(
          "shadow_supervisor_velocity_predictive_score_eval_std_u_mps", 0.30));
    shadow_supervisor_kinematic_predictive_score_publish_enable_ =
      this->declare_parameter<bool>(
        "shadow_supervisor_kinematic_predictive_score_publish_enable", false);
    shadow_supervisor_kinematic_predictive_score_subscribe_enable_ =
      this->declare_parameter<bool>(
        "shadow_supervisor_kinematic_predictive_score_subscribe_enable", false);
    shadow_supervisor_kinematic_predictive_score_debug_enable_ =
      this->declare_parameter<bool>(
        "shadow_supervisor_kinematic_predictive_score_debug_enable", false);
    shadow_supervisor_kinematic_predictive_score_topic_ =
      this->declare_parameter<std::string>(
        "shadow_supervisor_kinematic_predictive_score_topic",
        "/kf_gins/supervisor_kinematic_predictive_score");
    shadow_supervisor_kinematic_predictive_score_debug_csv_path_ =
      this->declare_parameter<std::string>(
        "shadow_supervisor_kinematic_predictive_score_debug_csv_path", "");
    shadow_supervisor_kinematic_predictive_score_source_id_ =
      this->declare_parameter<std::string>(
        "shadow_supervisor_kinematic_predictive_score_source_id", "rq_shadow");
    shadow_supervisor_kinematic_predictive_score_max_age_sec_ =
      this->declare_parameter<double>(
        "shadow_supervisor_kinematic_predictive_score_max_age_sec", 0.75);
    shadow_supervisor_kinematic_predictive_score_stamp_tolerance_sec_ =
      this->declare_parameter<double>(
        "shadow_supervisor_kinematic_predictive_score_stamp_tolerance_sec", 0.20);
    shadow_supervisor_kinematic_predictive_score_short_window_sec_ =
      std::max(
        0.0,
        this->declare_parameter<double>(
          "shadow_supervisor_kinematic_predictive_score_short_window_sec", 5.0));
    shadow_supervisor_kinematic_predictive_score_long_window_sec_ =
      std::max(
        0.0,
        this->declare_parameter<double>(
          "shadow_supervisor_kinematic_predictive_score_long_window_sec", 10.0));
    shadow_supervisor_kinematic_predictive_score_eval_std_h_m_ =
      std::max(
        0.01,
        this->declare_parameter<double>(
          "shadow_supervisor_kinematic_predictive_score_eval_std_h_m", 0.10));
    shadow_supervisor_kinematic_predictive_score_eval_std_u_m_ =
      std::max(
        0.01,
        this->declare_parameter<double>(
          "shadow_supervisor_kinematic_predictive_score_eval_std_u_m", 0.20));
    shadow_supervisor_kinematic_predictive_score_eval_std_h_mps_ =
      std::max(
        0.01,
        this->declare_parameter<double>(
          "shadow_supervisor_kinematic_predictive_score_eval_std_h_mps", 0.20));
    shadow_supervisor_kinematic_predictive_score_eval_std_u_mps_ =
      std::max(
        0.01,
        this->declare_parameter<double>(
          "shadow_supervisor_kinematic_predictive_score_eval_std_u_mps", 0.30));
    segment_timing_gate_debug_csv_path_ =
      this->declare_parameter<std::string>("segment_timing_gate_debug_csv_path", "");
    segment_timing_gate_segment_sec_ =
      std::max(0.1, this->declare_parameter<double>("segment_timing_gate_segment_sec", 2.0));
    segment_timing_gate_lag_threshold_sec_ =
      this->declare_parameter<double>("segment_timing_gate_lag_threshold_sec", 0.025);
    segment_timing_gate_gnss_source_age_max_sec_ =
      this->declare_parameter<double>("segment_timing_gate_gnss_source_age_max_sec", -1.0);
    segment_timing_gate_core_gnss_along_min_enable_ =
      this->declare_parameter<bool>("segment_timing_gate_core_gnss_along_min_enable", false);
    segment_timing_gate_core_gnss_along_min_threshold_m_ =
      this->declare_parameter<double>("segment_timing_gate_core_gnss_along_min_threshold_m", 0.0);
    segment_timing_gate_projection_alpha_ =
      this->declare_parameter<double>("segment_timing_gate_projection_alpha", 1.0);
    segment_timing_gate_projection_enable_ =
      this->declare_parameter<bool>("segment_timing_gate_projection_enable", false);
    segment_timing_gate_projection_apply_mission_ =
      this->declare_parameter<bool>("segment_timing_gate_projection_apply_mission", true);
    segment_timing_gate_projection_apply_rtl_ =
      this->declare_parameter<bool>("segment_timing_gate_projection_apply_rtl", false);
    segment_timing_gate_projection_apply_other_ =
      this->declare_parameter<bool>("segment_timing_gate_projection_apply_other", false);
    publish_state_after_gnss_update_ =
      this->declare_parameter<bool>("publish_state_after_gnss_update", true);
    publish_px4_sphere_projection_ =
      this->declare_parameter<bool>("publish_px4_sphere_projection", false);
    accbias_z_history_projection_enable_ =
      this->declare_parameter<bool>("accbias_z_history_projection_enable", false);
    accbias_z_history_projection_apply_mission_ =
      this->declare_parameter<bool>("accbias_z_history_projection_apply_mission", true);
    accbias_z_history_projection_apply_rtl_ =
      this->declare_parameter<bool>("accbias_z_history_projection_apply_rtl", false);
    accbias_z_history_projection_apply_other_ =
      this->declare_parameter<bool>("accbias_z_history_projection_apply_other", false);
    accbias_z_history_projection_alpha_ =
      this->declare_parameter<double>("accbias_z_history_projection_alpha", 1.2);
    accbias_z_history_projection_deep_threshold_mps2_ =
      this->declare_parameter<double>("accbias_z_history_projection_deep_threshold_mps2", -0.205);
    accbias_z_history_projection_frac_threshold_ =
      this->declare_parameter<double>("accbias_z_history_projection_frac_threshold", 0.4);
    accbias_z_history_projection_history_start_sec_ =
      std::max(
        0.0,
        this->declare_parameter<double>("accbias_z_history_projection_history_start_sec", 40.0));
    publish_stamp_mode_ =
      this->declare_parameter<std::string>("publish_stamp_mode", "ros_now");
    publish_core_stamp_max_future_sec_ =
      std::max(0.0, this->declare_parameter<double>("publish_core_stamp_max_future_sec", 0.08));
    publish_core_stamp_max_past_sec_ =
      std::max(0.0, this->declare_parameter<double>("publish_core_stamp_max_past_sec", 0.25));
    publish_core_stamp_offset_bias_sec_ =
      this->declare_parameter<double>("publish_core_stamp_offset_bias_sec", 0.0);
    if (publish_stamp_mode_ != "ros_now" && publish_stamp_mode_ != "core_fixed_offset") {
      RCLCPP_WARN(
        get_logger(),
        "Unsupported publish_stamp_mode=%s; falling back to ros_now.",
        publish_stamp_mode_.c_str());
      publish_stamp_mode_ = "ros_now";
    }
    state_update_debug_csv_path_ =
      this->declare_parameter<std::string>("state_update_debug_csv_path", "");
    state_update_debug_max_rate_hz_ =
      std::max(0.0, this->declare_parameter<double>("state_update_debug_max_rate_hz", 2.0));
    dtrq_runtime_feature_debug_csv_path_ =
      this->declare_parameter<std::string>("dtrq_runtime_feature_debug_csv_path", "");
    dtrq_runtime_feature_debug_max_rate_hz_ =
      std::max(0.0, this->declare_parameter<double>("dtrq_runtime_feature_debug_max_rate_hz", 2.0));
    early_recovery_bias_feedback_debug_enable_ =
      this->declare_parameter<bool>("early_recovery_bias_feedback_debug_enable", false);
    early_recovery_bias_feedback_apply_enable_ =
      this->declare_parameter<bool>("early_recovery_bias_feedback_apply_enable", false);
    early_recovery_bias_feedback_history_sec_ =
      std::max(
        0.1,
        this->declare_parameter<double>("early_recovery_bias_feedback_history_sec", 10.0));
    early_recovery_bias_feedback_min_armed_time_sec_ =
      this->declare_parameter<double>("early_recovery_bias_feedback_min_armed_time_sec", 35.0);
    early_recovery_bias_feedback_max_armed_time_sec_ =
      this->declare_parameter<double>("early_recovery_bias_feedback_max_armed_time_sec", 95.0);
    early_recovery_bias_feedback_ba_z_mean_max_mps2_ =
      this->declare_parameter<double>("early_recovery_bias_feedback_ba_z_mean_max_mps2", -0.18);
    early_recovery_bias_feedback_residual_u_mean_max_m_ =
      this->declare_parameter<double>("early_recovery_bias_feedback_residual_u_mean_max_m", -0.02);
    early_recovery_bias_feedback_core_gnss_u_mean_min_m_ =
      this->declare_parameter<double>("early_recovery_bias_feedback_core_gnss_u_mean_min_m", 0.02);
    early_recovery_bias_feedback_dx_ba_z_sum_max_mps2_ =
      this->declare_parameter<double>("early_recovery_bias_feedback_dx_ba_z_sum_max_mps2", 0.0);
    early_recovery_bias_feedback_min_history_rows_ =
      std::max(
        1,
        static_cast<int>(
          this->declare_parameter<int>("early_recovery_bias_feedback_min_history_rows", 5)));
    early_recovery_bias_feedback_negative_dx_scale_ =
      std::clamp(
        this->declare_parameter<double>("early_recovery_bias_feedback_negative_dx_scale", 0.0),
        0.0,
        1.0);
    auto normalize_optional_debug_csv_path = [](std::string & path) {
      if (path == "__disabled__" || path == "disabled" || path == "none" || path == "off") {
        path.clear();
      }
    };
    normalize_optional_debug_csv_path(gnss_nis_debug_csv_path_);
    normalize_optional_debug_csv_path(horizontal_consistency_debug_csv_path_);
    normalize_optional_debug_csv_path(state_update_debug_csv_path_);
    normalize_optional_debug_csv_path(dtrq_runtime_feature_debug_csv_path_);
    normalize_optional_debug_csv_path(segment_timing_gate_debug_csv_path_);
    normalize_optional_debug_csv_path(shadow_restore_event_csv_path_);
    normalize_optional_debug_csv_path(shadow_supervisor_fsm_debug_csv_path_);
    normalize_optional_debug_csv_path(shadow_supervisor_fsm_events_csv_path_);
    normalize_optional_debug_csv_path(shadow_supervisor_predictive_score_debug_csv_path_);
    normalize_optional_debug_csv_path(
      shadow_supervisor_velocity_predictive_score_debug_csv_path_);
    normalize_optional_debug_csv_path(
      shadow_supervisor_kinematic_predictive_score_debug_csv_path_);
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
    heading_track_validity_gate_enable_ =
      this->declare_parameter<bool>("heading_track_validity_gate_enable", false);
    heading_track_validity_gate_action_ =
      this->declare_parameter<std::string>("heading_track_validity_gate_action", "inflate");
    heading_track_validity_gate_apply_to_update_ =
      this->declare_parameter<bool>("heading_track_validity_gate_apply_to_update", true);
    heading_track_validity_gate_apply_to_turn_track_ =
      this->declare_parameter<bool>("heading_track_validity_gate_apply_to_turn_track", false);
    heading_track_validity_gate_apply_to_post_turn_ =
      this->declare_parameter<bool>("heading_track_validity_gate_apply_to_post_turn", true);
    heading_track_validity_gate_after_turn_sec_ =
      this->declare_parameter<double>("heading_track_validity_gate_after_turn_sec", 20.0);
    heading_track_validity_gate_min_horizontal_speed_mps_ =
      this->declare_parameter<double>("heading_track_validity_gate_min_horizontal_speed_mps", 3.0);
    heading_track_validity_gate_max_vertical_speed_mps_ =
      this->declare_parameter<double>("heading_track_validity_gate_max_vertical_speed_mps", 1.2);
    heading_track_validity_gate_max_gyro_deg_s_ =
      this->declare_parameter<double>("heading_track_validity_gate_max_gyro_deg_s", 6.0);
    heading_track_validity_gate_max_source_yaw_rate_deg_s_ =
      this->declare_parameter<double>("heading_track_validity_gate_max_source_yaw_rate_deg_s", 1.0);
    heading_track_validity_gate_max_residual_deg_ =
      this->declare_parameter<double>("heading_track_validity_gate_max_residual_deg", 2.0);
    heading_track_validity_gate_inflated_std_deg_ =
      this->declare_parameter<double>("heading_track_validity_gate_inflated_std_deg", 25.0);
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
    heading_cruise_micro_track_enable_ =
      this->declare_parameter<bool>("heading_cruise_micro_track_enable", false);
    heading_cruise_micro_track_std_deg_ =
      this->declare_parameter<double>("heading_cruise_micro_track_std_deg", 2.0);
    heading_cruise_micro_track_min_residual_deg_ =
      this->declare_parameter<double>("heading_cruise_micro_track_min_residual_deg", 0.15);
    heading_cruise_micro_track_max_residual_deg_ =
      this->declare_parameter<double>("heading_cruise_micro_track_max_residual_deg", 1.0);
    heading_cruise_micro_track_min_horizontal_speed_mps_ =
      this->declare_parameter<double>("heading_cruise_micro_track_min_horizontal_speed_mps", 4.5);
    heading_cruise_micro_track_max_vertical_speed_mps_ =
      this->declare_parameter<double>("heading_cruise_micro_track_max_vertical_speed_mps", 0.35);
    heading_cruise_micro_track_max_gyro_deg_s_ =
      this->declare_parameter<double>("heading_cruise_micro_track_max_gyro_deg_s", 3.5);
    heading_cruise_micro_track_max_source_yaw_rate_deg_s_ =
      this->declare_parameter<double>("heading_cruise_micro_track_max_source_yaw_rate_deg_s", 1.0);
    heading_yaw_gain_hygiene_enable_ =
      this->declare_parameter<bool>("heading_yaw_gain_hygiene_enable", false);
    heading_yaw_gain_hygiene_apply_mission_ =
      this->declare_parameter<bool>("heading_yaw_gain_hygiene_apply_mission", true);
    heading_yaw_gain_hygiene_apply_rtl_ =
      this->declare_parameter<bool>("heading_yaw_gain_hygiene_apply_rtl", false);
    heading_yaw_gain_hygiene_apply_armed_cruise_ =
      this->declare_parameter<bool>("heading_yaw_gain_hygiene_apply_armed_cruise", true);
    heading_yaw_gain_hygiene_apply_post_turn_ =
      this->declare_parameter<bool>("heading_yaw_gain_hygiene_apply_post_turn", true);
    heading_yaw_gain_hygiene_apply_turn_ =
      this->declare_parameter<bool>("heading_yaw_gain_hygiene_apply_turn", false);
    heading_yaw_gain_hygiene_apply_update_ =
      this->declare_parameter<bool>("heading_yaw_gain_hygiene_apply_update", false);
    heading_yaw_gain_hygiene_yaw_std_floor_deg_ =
      this->declare_parameter<double>("heading_yaw_gain_hygiene_yaw_std_floor_deg", 1.0);
    heading_yaw_gain_hygiene_min_residual_deg_ =
      this->declare_parameter<double>("heading_yaw_gain_hygiene_min_residual_deg", 0.5);
    heading_yaw_gain_hygiene_max_residual_deg_ =
      this->declare_parameter<double>("heading_yaw_gain_hygiene_max_residual_deg", 2.0);
    heading_yaw_gain_hygiene_min_horizontal_speed_mps_ =
      this->declare_parameter<double>("heading_yaw_gain_hygiene_min_horizontal_speed_mps", 3.0);
    heading_yaw_gain_hygiene_max_abs_vertical_speed_mps_ =
      this->declare_parameter<double>("heading_yaw_gain_hygiene_max_abs_vertical_speed_mps", 0.5);
    heading_yaw_gain_hygiene_max_gyro_deg_s_ =
      this->declare_parameter<double>("heading_yaw_gain_hygiene_max_gyro_deg_s", 8.0);
    heading_yaw_gain_hygiene_max_source_yaw_rate_deg_s_ =
      this->declare_parameter<double>("heading_yaw_gain_hygiene_max_source_yaw_rate_deg_s", 3.0);
    heading_yaw_gain_hygiene_max_rate_hz_ =
      this->declare_parameter<double>("heading_yaw_gain_hygiene_max_rate_hz", 5.0);
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
    configureEarlyRecoveryBiasFeedbackCore_();

    // ---- pubs/subs ----
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("kf_gins/odom", 10);
    odom_raw_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("kf_gins/odom_raw", 10);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("kf_gins/path", 10);
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("kf_gins/pose", 10);
    reset_event_pub_ = this->create_publisher<std_msgs::msg::UInt32>("kf_gins/reset_event", 10);
    fallback_active_pub_ = this->create_publisher<std_msgs::msg::Bool>("kf_gins/fallback_active", 10);
    configureShadowRestoreWiring_();
    configureShadowSupervisorPerformanceProxy_();
    configureShadowSupervisorPredictiveScore_();
    configureShadowSupervisorVelocityPredictiveScore_();
    configureShadowSupervisorKinematicPredictiveScore_();
    configureShadowSupervisorFsmDebug_();
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
    RCLCPP_INFO(
      get_logger(),
      "Raw odom publish decimation: %d (/kf_gins/odom_raw publishes every Nth core state)",
      raw_odom_decimation_);
    RCLCPP_INFO(
      get_logger(),
      "Core IMU rate limit: decimation=%d, max_rate_hz=%.2f (0 disables max-rate gate)",
      core_imu_decimation_, core_max_imu_rate_hz_);
    if (!core_processing_enable_) {
      RCLCPP_WARN(
        get_logger(),
        "KF-GINS core processing disabled: subscriptions stay active, but IMU/GNSS callbacks skip core updates and state publication.");
    }
    openGnssUpdateDebugCsv_();
    openGnssNisDebugCsv_();
    openHorizontalConsistencyDebugCsv_();
    openHeadingUpdateDebugCsv_();
    openStatePublishDebugCsv_();
    openSegmentTimingGateDebugCsv_();
    openStateUpdateDebugCsv_();
    openDtrqRuntimeFeatureDebugCsv_();
    updateStateUpdateDebugEnabled_();

  }

private:
  enum class ShadowSupervisorFsmState
  {
    MainSafe,
    ObservationGuard,
    ProcessCandidate,
    ShadowWarmup,
    ShadowReady,
    FailSafe
  };

  struct ShadowSupervisorReferenceOdom
  {
    bool valid{false};
    double stamp_sec{std::numeric_limits<double>::quiet_NaN()};
    double rx_ros_time_sec{std::numeric_limits<double>::quiet_NaN()};
    Eigen::Vector3d position_enu{Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN())};
    Eigen::Vector3d velocity_enu{Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN())};
    double yaw_deg{std::numeric_limits<double>::quiet_NaN()};
  };

  struct ShadowSupervisorPerformanceProxySample
  {
    bool valid{false};
    double publish_ros_time_sec{std::numeric_limits<double>::quiet_NaN()};
    double odom_stamp_sec{std::numeric_limits<double>::quiet_NaN()};
    double core_time_sec{std::numeric_limits<double>::quiet_NaN()};
    double observation_sequence{std::numeric_limits<double>::quiet_NaN()};
    double observation_update_time_sec{std::numeric_limits<double>::quiet_NaN()};
    double residual_h_m{std::numeric_limits<double>::quiet_NaN()};
    double nis_h_2d{std::numeric_limits<double>::quiet_NaN()};
    double nis_3d{std::numeric_limits<double>::quiet_NaN()};
    bool gnss_position_applied{false};
    double gamma_clipped{std::numeric_limits<double>::quiet_NaN()};
    double observation_score{std::numeric_limits<double>::quiet_NaN()};
    double process_score{std::numeric_limits<double>::quiet_NaN()};
    double lambda_vrw{std::numeric_limits<double>::quiet_NaN()};
    double lambda_accbias{std::numeric_limits<double>::quiet_NaN()};
  };

  struct ShadowSupervisorPerformanceProxyGapSample
  {
    double ros_time_sec{std::numeric_limits<double>::quiet_NaN()};
    double residual_gap_h_m{std::numeric_limits<double>::quiet_NaN()};
    double nis_gap_h_2d{std::numeric_limits<double>::quiet_NaN()};
  };

  struct ShadowSupervisorPredictiveScoreSample
  {
    bool valid{false};
    double publish_ros_time_sec{std::numeric_limits<double>::quiet_NaN()};
    double gnss_stamp_sec{std::numeric_limits<double>::quiet_NaN()};
    double core_time_sec{std::numeric_limits<double>::quiet_NaN()};
    double sequence{std::numeric_limits<double>::quiet_NaN()};
    double residual_n_m{std::numeric_limits<double>::quiet_NaN()};
    double residual_e_m{std::numeric_limits<double>::quiet_NaN()};
    double residual_h_m{std::numeric_limits<double>::quiet_NaN()};
    double eval_s_nn_m2{std::numeric_limits<double>::quiet_NaN()};
    double eval_s_ne_m2{std::numeric_limits<double>::quiet_NaN()};
    double eval_s_ee_m2{std::numeric_limits<double>::quiet_NaN()};
    double gpps_h{std::numeric_limits<double>::quiet_NaN()};
    double gamma_clipped{std::numeric_limits<double>::quiet_NaN()};
    double observation_score{std::numeric_limits<double>::quiet_NaN()};
  };

  struct ShadowSupervisorPredictiveScoreGapSample
  {
    double ros_time_sec{std::numeric_limits<double>::quiet_NaN()};
    double gpps_gap_h{std::numeric_limits<double>::quiet_NaN()};
    double residual_gap_h_m{std::numeric_limits<double>::quiet_NaN()};
    bool shadow_win{false};
  };

  struct ShadowSupervisorVelocityPredictiveScoreSample
  {
    bool valid{false};
    double publish_ros_time_sec{std::numeric_limits<double>::quiet_NaN()};
    double gnss_stamp_sec{std::numeric_limits<double>::quiet_NaN()};
    double core_time_sec{std::numeric_limits<double>::quiet_NaN()};
    double sequence{std::numeric_limits<double>::quiet_NaN()};
    double raw_vn_mps{std::numeric_limits<double>::quiet_NaN()};
    double raw_ve_mps{std::numeric_limits<double>::quiet_NaN()};
    double raw_vd_mps{std::numeric_limits<double>::quiet_NaN()};
    double raw_speed_accuracy_mps{std::numeric_limits<double>::quiet_NaN()};
    double raw_vertical_speed_accuracy_mps{std::numeric_limits<double>::quiet_NaN()};
    double prior_vn_mps{std::numeric_limits<double>::quiet_NaN()};
    double prior_ve_mps{std::numeric_limits<double>::quiet_NaN()};
    double prior_vd_mps{std::numeric_limits<double>::quiet_NaN()};
    double residual_n_mps{std::numeric_limits<double>::quiet_NaN()};
    double residual_e_mps{std::numeric_limits<double>::quiet_NaN()};
    double residual_d_mps{std::numeric_limits<double>::quiet_NaN()};
    double residual_h_mps{std::numeric_limits<double>::quiet_NaN()};
    double eval_sv_nn_m2ps2{std::numeric_limits<double>::quiet_NaN()};
    double eval_sv_ne_m2ps2{std::numeric_limits<double>::quiet_NaN()};
    double eval_sv_ee_m2ps2{std::numeric_limits<double>::quiet_NaN()};
    double eval_sv_dd_m2ps2{std::numeric_limits<double>::quiet_NaN()};
    double gvps_h{std::numeric_limits<double>::quiet_NaN()};
    double gvps_3d{std::numeric_limits<double>::quiet_NaN()};
    double velocity_gamma_clipped{std::numeric_limits<double>::quiet_NaN()};
    double velocity_observation_score{std::numeric_limits<double>::quiet_NaN()};
    double position_gamma_clipped{std::numeric_limits<double>::quiet_NaN()};
    double position_observation_score{std::numeric_limits<double>::quiet_NaN()};
  };

  struct ShadowSupervisorVelocityPredictiveScoreGapSample
  {
    double ros_time_sec{std::numeric_limits<double>::quiet_NaN()};
    double gvps_gap_h{std::numeric_limits<double>::quiet_NaN()};
    double velocity_residual_gap_h_mps{std::numeric_limits<double>::quiet_NaN()};
    bool shadow_win{false};
  };

  struct ShadowSupervisorKinematicPredictiveScoreSample
  {
    bool valid{false};
    double publish_ros_time_sec{std::numeric_limits<double>::quiet_NaN()};
    double gnss_stamp_sec{std::numeric_limits<double>::quiet_NaN()};
    double core_time_sec{std::numeric_limits<double>::quiet_NaN()};
    double sequence{std::numeric_limits<double>::quiet_NaN()};
    double raw_pn_m{std::numeric_limits<double>::quiet_NaN()};
    double raw_pe_m{std::numeric_limits<double>::quiet_NaN()};
    double raw_pu_m{std::numeric_limits<double>::quiet_NaN()};
    double raw_vn_mps{std::numeric_limits<double>::quiet_NaN()};
    double raw_ve_mps{std::numeric_limits<double>::quiet_NaN()};
    double raw_vd_mps{std::numeric_limits<double>::quiet_NaN()};
    double raw_position_accuracy_h_m{std::numeric_limits<double>::quiet_NaN()};
    double raw_position_accuracy_u_m{std::numeric_limits<double>::quiet_NaN()};
    double raw_speed_accuracy_mps{std::numeric_limits<double>::quiet_NaN()};
    double raw_vertical_speed_accuracy_mps{std::numeric_limits<double>::quiet_NaN()};
    double prior_pn_m{std::numeric_limits<double>::quiet_NaN()};
    double prior_pe_m{std::numeric_limits<double>::quiet_NaN()};
    double prior_pu_m{std::numeric_limits<double>::quiet_NaN()};
    double prior_vn_mps{std::numeric_limits<double>::quiet_NaN()};
    double prior_ve_mps{std::numeric_limits<double>::quiet_NaN()};
    double prior_vd_mps{std::numeric_limits<double>::quiet_NaN()};
    double residual_pn_m{std::numeric_limits<double>::quiet_NaN()};
    double residual_pe_m{std::numeric_limits<double>::quiet_NaN()};
    double residual_ph_m{std::numeric_limits<double>::quiet_NaN()};
    double residual_vn_mps{std::numeric_limits<double>::quiet_NaN()};
    double residual_ve_mps{std::numeric_limits<double>::quiet_NaN()};
    double residual_vd_mps{std::numeric_limits<double>::quiet_NaN()};
    double residual_vh_mps{std::numeric_limits<double>::quiet_NaN()};
    double eval_sp_nn_m2{std::numeric_limits<double>::quiet_NaN()};
    double eval_sp_ne_m2{std::numeric_limits<double>::quiet_NaN()};
    double eval_sp_ee_m2{std::numeric_limits<double>::quiet_NaN()};
    double eval_sv_nn_m2ps2{std::numeric_limits<double>::quiet_NaN()};
    double eval_sv_ne_m2ps2{std::numeric_limits<double>::quiet_NaN()};
    double eval_sv_ee_m2ps2{std::numeric_limits<double>::quiet_NaN()};
    double gpps_h{std::numeric_limits<double>::quiet_NaN()};
    double gvps_h{std::numeric_limits<double>::quiet_NaN()};
    double raw_delta_pn_2s_m{std::numeric_limits<double>::quiet_NaN()};
    double raw_delta_pe_2s_m{std::numeric_limits<double>::quiet_NaN()};
    double delta_consistency_2s_m{std::numeric_limits<double>::quiet_NaN()};
    double raw_delta_pn_5s_m{std::numeric_limits<double>::quiet_NaN()};
    double raw_delta_pe_5s_m{std::numeric_limits<double>::quiet_NaN()};
    double delta_consistency_5s_m{std::numeric_limits<double>::quiet_NaN()};
    double raw_delta_pn_10s_m{std::numeric_limits<double>::quiet_NaN()};
    double raw_delta_pe_10s_m{std::numeric_limits<double>::quiet_NaN()};
    double delta_consistency_10s_m{std::numeric_limits<double>::quiet_NaN()};
    double position_gamma_clipped{std::numeric_limits<double>::quiet_NaN()};
    double position_observation_score{std::numeric_limits<double>::quiet_NaN()};
    double velocity_gamma_clipped{std::numeric_limits<double>::quiet_NaN()};
    double velocity_observation_score{std::numeric_limits<double>::quiet_NaN()};
  };

  struct ShadowSupervisorKinematicPredictiveScoreGapSample
  {
    double ros_time_sec{std::numeric_limits<double>::quiet_NaN()};
    double gkps_composite_gap{std::numeric_limits<double>::quiet_NaN()};
    bool shadow_win{false};
  };

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
    mavros_mode_ = msg->mode;

    if (prev_armed != mavros_armed_) {
      last_disarmed_yaw_lock_time_sec_ = std::numeric_limits<double>::quiet_NaN();
      last_heading_attempt_time_sec_ = std::numeric_limits<double>::quiet_NaN();
      last_heading_update_time_sec_ = std::numeric_limits<double>::quiet_NaN();
      last_heading_recovery_time_sec_ = std::numeric_limits<double>::quiet_NaN();
      last_turning_heading_time_sec_ = std::numeric_limits<double>::quiet_NaN();
      last_heading_underreaction_force_relock_time_sec_ = std::numeric_limits<double>::quiet_NaN();
      last_tilt_force_relock_time_sec_ = std::numeric_limits<double>::quiet_NaN();
      resetHorizontalConsistencySupervisorState_();
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
      resetAccbiasZHistoryProjection_();
    }

    if (!prev_armed && mavros_armed_) {
      have_completed_armed_flight_since_reset_ = true;
      last_armed_transition_time_sec_ = now_sec;
      resetSegmentTimingGateAccumulator_();
      segment_timing_gate_active_duration_sec_ = 0.0;
      last_disarmed_transition_time_sec_ = std::numeric_limits<double>::quiet_NaN();
      post_flight_vertical_cov_reopen_until_sec_ = std::numeric_limits<double>::quiet_NaN();
    } else if (prev_armed && !mavros_armed_) {
      last_disarmed_transition_time_sec_ = now_sec;
      flushSegmentTimingGateDebug_(now_sec, true);
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
      resetPublishCoreStampOffset_("armed transition");
      RCLCPP_INFO(get_logger(), "Armed transition: keeping IMU time base continuous.");
      applyExperimentArmedResetOffset_();
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

  bool isTerminalDescentFlightMode_() const
  {
    return mavros_mode_.find("RTL") != std::string::npos ||
           mavros_mode_.find("LAND") != std::string::npos;
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

  void clearCoreImuRateLimitState_()
  {
    have_core_imu_accumulator_ = false;
    core_imu_accum_feed_delta_ = false;
    core_imu_accum_dt_sec_ = 0.0;
    core_imu_accum_dtheta_.setZero();
    core_imu_accum_dvel_.setZero();
    last_core_imu_rate_limit_process_steady_sec_ = std::numeric_limits<double>::quiet_NaN();
    core_imu_rate_limit_input_count_ = 0;
    core_imu_rate_limit_skip_count_ = 0;
    core_imu_rate_limit_process_count_ = 0;
  }

  bool maybeHoldCoreImuForRateLimit_(
    double steady_now_sec,
    bool feed_core_with_delta,
    double& dt,
    Eigen::Vector3d& dtheta,
    Eigen::Vector3d& dvel)
  {
    const bool decimation_active = core_imu_decimation_ > 1;
    const bool max_rate_active = std::isfinite(core_max_imu_rate_hz_) && core_max_imu_rate_hz_ > 0.0;
    if (!decimation_active && !max_rate_active) {
      return false;
    }
    if (!std::isfinite(dt) || dt <= 0.0) {
      return false;
    }

    if (have_core_imu_accumulator_ &&
        core_imu_accum_feed_delta_ != feed_core_with_delta) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Core IMU rate-limit accumulator reset because input representation changed (delta=%s).",
        feed_core_with_delta ? "true" : "false");
      have_core_imu_accumulator_ = false;
      core_imu_accum_dt_sec_ = 0.0;
      core_imu_accum_dtheta_.setZero();
      core_imu_accum_dvel_.setZero();
    }

    if (!have_core_imu_accumulator_) {
      have_core_imu_accumulator_ = true;
      core_imu_accum_feed_delta_ = feed_core_with_delta;
      core_imu_accum_dt_sec_ = 0.0;
      core_imu_accum_dtheta_.setZero();
      core_imu_accum_dvel_.setZero();
    }

    ++core_imu_rate_limit_input_count_;
    core_imu_accum_dt_sec_ += dt;
    if (feed_core_with_delta) {
      core_imu_accum_dtheta_ += dtheta;
      core_imu_accum_dvel_ += dvel;
    } else {
      core_imu_accum_dtheta_ += dtheta * dt;
      core_imu_accum_dvel_ += dvel * dt;
    }

    const bool decimation_ready =
      !decimation_active ||
      ((core_imu_rate_limit_input_count_ % static_cast<std::uint64_t>(core_imu_decimation_)) == 0);
    bool max_rate_ready = true;
    if (max_rate_active && std::isfinite(last_core_imu_rate_limit_process_steady_sec_)) {
      const double min_period_sec = 1.0 / std::max(1e-6, core_max_imu_rate_hz_);
      max_rate_ready =
        (steady_now_sec - last_core_imu_rate_limit_process_steady_sec_) >= min_period_sec;
    }

    if (!decimation_ready || !max_rate_ready) {
      ++core_imu_rate_limit_skip_count_;
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Core IMU rate limit holding samples: input=%lu processed=%lu skipped=%lu accum_dt=%.4f s "
        "(decimation=%d, max_rate_hz=%.2f)",
        static_cast<unsigned long>(core_imu_rate_limit_input_count_),
        static_cast<unsigned long>(core_imu_rate_limit_process_count_),
        static_cast<unsigned long>(core_imu_rate_limit_skip_count_),
        core_imu_accum_dt_sec_,
        core_imu_decimation_,
        core_max_imu_rate_hz_);
      return true;
    }

    dt = core_imu_accum_dt_sec_;
    if (!std::isfinite(dt) || dt <= 0.0) {
      have_core_imu_accumulator_ = false;
      core_imu_accum_dt_sec_ = 0.0;
      core_imu_accum_dtheta_.setZero();
      core_imu_accum_dvel_.setZero();
      return false;
    }
    dtheta = feed_core_with_delta ? core_imu_accum_dtheta_ : (core_imu_accum_dtheta_ / dt);
    dvel = feed_core_with_delta ? core_imu_accum_dvel_ : (core_imu_accum_dvel_ / dt);

    have_core_imu_accumulator_ = false;
    core_imu_accum_dt_sec_ = 0.0;
    core_imu_accum_dtheta_.setZero();
    core_imu_accum_dvel_.setZero();
    last_core_imu_rate_limit_process_steady_sec_ = steady_now_sec;
    ++core_imu_rate_limit_process_count_;
    last_processed_imu_effective_dt_sec_ = dt;
    last_medium_imu_gap_segmented_ = false;
    last_medium_imu_gap_segmented_steps_ = 1;
    return false;
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
    last_turn_rate_propagation_noise_debug_ = TurnRatePropagationNoiseDebug{};
    last_accbias_z_propagation_probe_debug_ = AccbiasZPropagationProbeDebug{};
    core_->setPropagationNoiseScale(1.0, 1.0, 1.0, 1.0);
    core_->setAdaptiveRQProcessContext(false, 0.0);

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
    resetAdaptiveGnssPositionWeightState_();
    resetContextGnssPosFloorState_();
    resetGnssPositionGainResponseState_();
    resetHorizontalConsistencySupervisorState_();
    clearCoreImuRateLimitState_();
    resetPublishCoreStampOffset_("disarmed yaw lock");
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
    last_turn_rate_propagation_noise_debug_ = TurnRatePropagationNoiseDebug{};
    last_accbias_z_propagation_probe_debug_ = AccbiasZPropagationProbeDebug{};
    if (core_) {
      core_->setPropagationNoiseScale(1.0, 1.0, 1.0, 1.0);
      core_->setAdaptiveRQProcessContext(false, 0.0);
    }

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
    resetPublishCoreStampOffset_("core/time-base reset");

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
    resetAdaptiveGnssPositionWeightState_();
    resetContextGnssPosFloorState_();
    resetGnssPositionGainResponseState_();
    resetHorizontalConsistencySupervisorState_();
    clearCoreImuRateLimitState_();
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

  struct ExperimentResetPose
  {
    double latitude_deg{std::numeric_limits<double>::quiet_NaN()};
    double longitude_deg{std::numeric_limits<double>::quiet_NaN()};
    double altitude_m{std::numeric_limits<double>::quiet_NaN()};
    double yaw_deg{std::numeric_limits<double>::quiet_NaN()};
  };

  ExperimentResetPose firstResetPoseWithOptionalExperimentOffset_(
    double latitude_deg,
    double longitude_deg,
    double altitude_m,
    double yaw_deg)
  {
    ExperimentResetPose pose{latitude_deg, longitude_deg, altitude_m, yaw_deg};
    if (!experiment_initial_reset_offset_enable_ ||
        experiment_initial_reset_offset_applied_) {
      return pose;
    }

    experiment_initial_reset_offset_applied_ = true;
    const double dn_m = experiment_initial_reset_offset_n_m_;
    const double de_m = experiment_initial_reset_offset_e_m_;
    const double du_m = experiment_initial_reset_offset_u_m_;
    const double dyaw_deg = experiment_initial_reset_yaw_offset_deg_;
    const bool has_offset =
      std::abs(dn_m) > 1e-9 ||
      std::abs(de_m) > 1e-9 ||
      std::abs(du_m) > 1e-9 ||
      std::abs(dyaw_deg) > 1e-9;
    if (!has_offset) {
      return pose;
    }

    pose = applyExperimentResetOffsetToPose_(pose, dn_m, de_m, du_m, dyaw_deg);

    RCLCPP_WARN(
      get_logger(),
      "Experiment initial reset offset applied once: dN=%.3f m dE=%.3f m dU=%.3f m dYaw=%.3f deg",
      dn_m,
      de_m,
      du_m,
      dyaw_deg);
    return pose;
  }

  ExperimentResetPose applyExperimentResetOffsetToPose_(
    const ExperimentResetPose& base_pose,
    double dn_m,
    double de_m,
    double du_m,
    double dyaw_deg) const
  {
    ExperimentResetPose pose = base_pose;
    // WGS-84 local tangent approximation; offsets are meter-scale experiment inputs.
    constexpr double a = 6378137.0;
    constexpr double f = 1.0 / 298.257223563;
    constexpr double e2 = f * (2.0 - f);
    const double lat_rad = base_pose.latitude_deg * M_PI / 180.0;
    const double sin_lat = std::sin(lat_rad);
    const double cos_lat = std::max(1e-9, std::cos(lat_rad));
    const double denom = std::sqrt(1.0 - e2 * sin_lat * sin_lat);
    const double rn = a / denom;
    const double rm = a * (1.0 - e2) / (denom * denom * denom);

    pose.latitude_deg = (lat_rad + dn_m / rm) * 180.0 / M_PI;
    pose.longitude_deg =
      (base_pose.longitude_deg * M_PI / 180.0 + de_m / (rn * cos_lat)) * 180.0 / M_PI;
    pose.altitude_m = base_pose.altitude_m + du_m;
    pose.yaw_deg = base_pose.yaw_deg + dyaw_deg;
    return pose;
  }

  void applyExperimentArmedResetOffset_()
  {
    if (!experiment_armed_reset_offset_enable_ ||
        experiment_armed_reset_offset_applied_ ||
        !core_ ||
        !core_initialized_) {
      return;
    }
    const double dn_m = experiment_initial_reset_offset_n_m_;
    const double de_m = experiment_initial_reset_offset_e_m_;
    const double du_m = experiment_initial_reset_offset_u_m_;
    const double dyaw_deg = experiment_initial_reset_yaw_offset_deg_;
    const bool has_offset =
      std::abs(dn_m) > 1e-9 ||
      std::abs(de_m) > 1e-9 ||
      std::abs(du_m) > 1e-9 ||
      std::abs(dyaw_deg) > 1e-9;
    if (!has_offset) {
      experiment_armed_reset_offset_applied_ = true;
      return;
    }
    if (!last_gnss_valid_) {
      RCLCPP_WARN(
        get_logger(),
        "Experiment armed reset offset requested but skipped because no valid GNSS is available.");
      return;
    }

    const auto current_state = core_->current();
    const double base_yaw_deg =
      std::isfinite(current_state.yaw_deg) ? current_state.yaw_deg : getInitialYawDeg_();
    const ExperimentResetPose base_pose{
      last_gnss_lat_rad_ * 180.0 / M_PI,
      last_gnss_lon_rad_ * 180.0 / M_PI,
      last_gnss_h_m_,
      base_yaw_deg};
    const ExperimentResetPose reset_pose =
      applyExperimentResetOffsetToPose_(base_pose, dn_m, de_m, du_m, dyaw_deg);

    experiment_armed_reset_offset_applied_ = true;
    (void)core_->reset(
      reset_pose.latitude_deg,
      reset_pose.longitude_deg,
      reset_pose.altitude_m,
      reset_pose.yaw_deg);
    last_turn_rate_propagation_noise_debug_ = TurnRatePropagationNoiseDebug{};
    last_accbias_z_propagation_probe_debug_ = AccbiasZPropagationProbeDebug{};
    core_->setPropagationNoiseScale(1.0, 1.0, 1.0, 1.0);
    core_->setAdaptiveRQProcessContext(false, 0.0);

    core_initialized_ = true;
    core_llh_aligned_ = false;
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
    resetAdaptiveGnssPositionWeightState_();
    resetContextGnssPosFloorState_();
    resetGnssPositionGainResponseState_();
    resetGnssVelocityOutwardDampingState_();
    resetHorizontalConsistencySupervisorState_();
    clearCoreImuRateLimitState_();
    resetPublishCoreStampOffset_("experiment armed reset offset");
    publishResetEvent_("experiment armed reset offset");
    clearPath_("experiment armed reset offset");

    RCLCPP_WARN(
      get_logger(),
      "Experiment armed reset offset applied once: dN=%.3f m dE=%.3f m dU=%.3f m dYaw=%.3f deg base_yaw=%.3f deg reset_yaw=%.3f deg",
      dn_m,
      de_m,
      du_m,
      dyaw_deg,
      base_yaw_deg,
      reset_pose.yaw_deg);
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

  void configureShadowRestoreWiring_()
  {
    if (!shadow_restore_publish_enable_ && !shadow_restore_subscribe_enable_ &&
        shadow_restore_event_csv_path_.empty()) {
      return;
    }

    if (shadow_restore_publish_enable_) {
      shadow_restore_pub_ =
        this->create_publisher<std_msgs::msg::Float64MultiArray>(
          shadow_restore_topic_, rclcpp::QoS(1).reliable().durability_volatile());
    }
    if (shadow_restore_subscribe_enable_) {
      shadow_restore_sub_ =
        this->create_subscription<std_msgs::msg::Float64MultiArray>(
          shadow_restore_topic_, rclcpp::QoS(1).reliable().durability_volatile(),
          [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
            handleShadowRestoreSnapshot_(msg);
          });
    }
    if (!shadow_restore_event_csv_path_.empty()) {
      const std::filesystem::path csv_path(shadow_restore_event_csv_path_);
      if (csv_path.has_parent_path()) {
        std::error_code ec;
        std::filesystem::create_directories(csv_path.parent_path(), ec);
      }
      shadow_restore_event_csv_.open(csv_path, std::ios::out | std::ios::trunc);
      if (shadow_restore_event_csv_.is_open()) {
        shadow_restore_event_csv_
          << "sequence,ros_time_sec,event,ok,reason,publish_ros_time_sec,"
          << "snapshot_time_sec,age_sec,last_core_time_sec,lat_deg,lon_deg,h_m,"
          << "vN_mps,vE_mps,vD_mps,roll_deg,pitch_deg,yaw_deg,"
          << "covariance_trace,covariance_inflation_factor\n";
        shadow_restore_event_csv_.flush();
      } else {
        RCLCPP_WARN(
          get_logger(),
          "Failed to open shadow restore event CSV: %s",
          shadow_restore_event_csv_path_.c_str());
      }
    }

    RCLCPP_INFO(
      get_logger(),
      "Shadow restore wiring: publish=%s subscribe=%s topic=%s publish_after_core_sec=%.3f "
      "publish_once=%s period=%.3f max_age=%.3f cov_inflation=%.3f event_csv=%s",
      shadow_restore_publish_enable_ ? "true" : "false",
      shadow_restore_subscribe_enable_ ? "true" : "false",
      shadow_restore_topic_.c_str(),
      shadow_restore_publish_after_core_sec_,
      shadow_restore_publish_once_ ? "true" : "false",
      shadow_restore_publish_period_sec_,
      shadow_restore_max_age_sec_,
      shadow_restore_covariance_inflation_factor_,
      shadow_restore_event_csv_path_.empty() ? "<disabled>" : shadow_restore_event_csv_path_.c_str());
  }

  void configureShadowSupervisorPerformanceProxy_()
  {
    if (!shadow_supervisor_perf_proxy_publish_enable_ &&
        !shadow_supervisor_perf_proxy_subscribe_enable_) {
      return;
    }

    if (shadow_supervisor_perf_proxy_publish_enable_) {
      shadow_supervisor_perf_proxy_pub_ =
        this->create_publisher<std_msgs::msg::Float64MultiArray>(
          shadow_supervisor_perf_proxy_topic_,
          rclcpp::QoS(10).reliable().durability_volatile());
    }
    if (shadow_supervisor_perf_proxy_subscribe_enable_) {
      shadow_supervisor_perf_proxy_sub_ =
        this->create_subscription<std_msgs::msg::Float64MultiArray>(
          shadow_supervisor_perf_proxy_topic_,
          rclcpp::QoS(10).reliable().durability_volatile(),
          [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
            handleShadowSupervisorPerformanceProxy_(msg);
          });
    }

    RCLCPP_INFO(
      get_logger(),
      "Shadow supervisor performance proxy: publish=%s subscribe=%s topic=%s "
      "max_age=%.3f short_window=%.3f long_window=%.3f",
      shadow_supervisor_perf_proxy_publish_enable_ ? "true" : "false",
      shadow_supervisor_perf_proxy_subscribe_enable_ ? "true" : "false",
      shadow_supervisor_perf_proxy_topic_.c_str(),
      shadow_supervisor_perf_proxy_max_age_sec_,
      shadow_supervisor_perf_proxy_short_window_sec_,
      shadow_supervisor_perf_proxy_long_window_sec_);
  }

  void configureShadowSupervisorPredictiveScore_()
  {
    if (!shadow_supervisor_predictive_score_publish_enable_ &&
        !shadow_supervisor_predictive_score_subscribe_enable_ &&
        !shadow_supervisor_predictive_score_debug_enable_) {
      return;
    }

    if (shadow_supervisor_predictive_score_publish_enable_) {
      shadow_supervisor_predictive_score_pub_ =
        this->create_publisher<std_msgs::msg::Float64MultiArray>(
          shadow_supervisor_predictive_score_topic_,
          rclcpp::QoS(10).reliable().durability_volatile());
    }
    if (shadow_supervisor_predictive_score_subscribe_enable_) {
      shadow_supervisor_predictive_score_sub_ =
        this->create_subscription<std_msgs::msg::Float64MultiArray>(
          shadow_supervisor_predictive_score_topic_,
          rclcpp::QoS(10).reliable().durability_volatile(),
          [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
            handleShadowSupervisorPredictiveScore_(msg);
          });
    }

    if (shadow_supervisor_predictive_score_debug_enable_ &&
        !shadow_supervisor_predictive_score_debug_csv_path_.empty()) {
      try {
        const std::filesystem::path csv_path(
          shadow_supervisor_predictive_score_debug_csv_path_);
        if (csv_path.has_parent_path()) {
          std::filesystem::create_directories(csv_path.parent_path());
        }
        shadow_supervisor_predictive_score_debug_csv_.open(
          csv_path, std::ios::out | std::ios::trunc);
      } catch (const std::exception & e) {
        RCLCPP_WARN(
          get_logger(),
          "Failed to open shadow supervisor predictive-score CSV %s: %s",
          shadow_supervisor_predictive_score_debug_csv_path_.c_str(),
          e.what());
      }
      if (shadow_supervisor_predictive_score_debug_csv_.is_open()) {
        shadow_supervisor_predictive_score_debug_csv_
          << "sequence,ros_time_sec,gnss_stamp_sec,core_time_sec,source_id,"
          << "restore_applied,time_since_restore_sec,prediction_valid,prediction_reason,"
          << "main_sample_age_sec,shadow_sample_age_sec,main_residual_n_m,"
          << "main_residual_e_m,main_residual_h_m,shadow_residual_n_m,"
          << "shadow_residual_e_m,shadow_residual_h_m,main_eval_s_nn_m2,"
          << "main_eval_s_ne_m2,main_eval_s_ee_m2,shadow_eval_s_nn_m2,"
          << "shadow_eval_s_ne_m2,shadow_eval_s_ee_m2,main_gpps_h,shadow_gpps_h,"
          << "gpps_gap_h,residual_gap_h_m,gpps_gap_mean_5s,gpps_gap_mean_10s,"
          << "gpps_shadow_win_frac_10s,residual_gap_mean_5s_m,observation_guard,"
          << "observation_reason,gamma_clipped,observation_score,output_policy,"
          << "main_gnss_stamp_sec,shadow_gnss_stamp_sec,gnss_stamp_gap_sec\n";
        shadow_supervisor_predictive_score_debug_csv_.flush();
      } else {
        RCLCPP_WARN(
          get_logger(),
          "Failed to open shadow supervisor predictive-score CSV: %s",
          shadow_supervisor_predictive_score_debug_csv_path_.c_str());
      }
    }

    RCLCPP_INFO(
      get_logger(),
      "Shadow supervisor predictive score: publish=%s subscribe=%s debug=%s topic=%s "
      "csv=%s max_age=%.3f stamp_tolerance=%.3f eval_std_h=%.3f eval_std_u=%.3f",
      shadow_supervisor_predictive_score_publish_enable_ ? "true" : "false",
      shadow_supervisor_predictive_score_subscribe_enable_ ? "true" : "false",
      shadow_supervisor_predictive_score_debug_enable_ ? "true" : "false",
      shadow_supervisor_predictive_score_topic_.c_str(),
      shadow_supervisor_predictive_score_debug_csv_path_.empty()
        ? "<disabled>" : shadow_supervisor_predictive_score_debug_csv_path_.c_str(),
      shadow_supervisor_predictive_score_max_age_sec_,
      shadow_supervisor_predictive_score_stamp_tolerance_sec_,
      shadow_supervisor_predictive_score_eval_std_h_m_,
      shadow_supervisor_predictive_score_eval_std_u_m_);
  }

  ShadowSupervisorPerformanceProxySample makeShadowSupervisorPerformanceProxySample_(
    double publish_ros_time_sec,
    double odom_stamp_sec) const
  {
    ShadowSupervisorPerformanceProxySample sample;
    sample.publish_ros_time_sec = publish_ros_time_sec;
    sample.odom_stamp_sec = odom_stamp_sec;
    sample.core_time_sec = last_core_time_;
    if (!core_) {
      return sample;
    }

    const kfcore::ObservationDebug observation_debug = core_->lastObservationDebug();
    const kfcore::BoundedAdaptiveRDebug & rq =
      observation_debug.gnss_position_adaptive_r;
    const double residual_n_m = observation_debug.gnss_position_residual_neu_m.x();
    const double residual_e_m = observation_debug.gnss_position_residual_neu_m.y();
    if (std::isfinite(residual_n_m) && std::isfinite(residual_e_m)) {
      sample.residual_h_m = std::hypot(residual_n_m, residual_e_m);
    }
    sample.valid =
      observation_debug.valid &&
      std::isfinite(sample.residual_h_m) &&
      std::isfinite(observation_debug.gnss_position_nis_h_2d);
    sample.observation_sequence =
      static_cast<double>(observation_debug.sequence);
    sample.observation_update_time_sec = observation_debug.update_time_sec;
    sample.nis_h_2d = observation_debug.gnss_position_nis_h_2d;
    sample.nis_3d = observation_debug.gnss_position_nis_3d;
    sample.gnss_position_applied = observation_debug.gnss_position_applied;
    sample.gamma_clipped = rq.gamma_clipped;
    sample.observation_score = rq.observation_score;
    sample.process_score = rq.process_score;
    sample.lambda_vrw = rq.q_lambda_vrw;
    sample.lambda_accbias = rq.q_lambda_accbias;
    return sample;
  }

  std_msgs::msg::Float64MultiArray encodeShadowSupervisorPerformanceProxy_(
    const ShadowSupervisorPerformanceProxySample & sample) const
  {
    std_msgs::msg::Float64MultiArray msg;
    auto & data = msg.data;
    data.reserve(15);
    data.push_back(1.0);  // version
    data.push_back(sample.publish_ros_time_sec);
    data.push_back(sample.odom_stamp_sec);
    data.push_back(sample.core_time_sec);
    data.push_back(sample.observation_sequence);
    data.push_back(sample.observation_update_time_sec);
    data.push_back(sample.residual_h_m);
    data.push_back(sample.nis_h_2d);
    data.push_back(sample.nis_3d);
    data.push_back(sample.gnss_position_applied ? 1.0 : 0.0);
    data.push_back(sample.gamma_clipped);
    data.push_back(sample.observation_score);
    data.push_back(sample.process_score);
    data.push_back(sample.lambda_vrw);
    data.push_back(sample.lambda_accbias);
    return msg;
  }

  bool decodeShadowSupervisorPerformanceProxy_(
    const std::vector<double> & data,
    ShadowSupervisorPerformanceProxySample * sample) const
  {
    if (sample == nullptr || data.size() < 15 ||
        !std::isfinite(data[0]) || std::abs(data[0] - 1.0) > 1.0e-6) {
      return false;
    }
    sample->publish_ros_time_sec = data[1];
    sample->odom_stamp_sec = data[2];
    sample->core_time_sec = data[3];
    sample->observation_sequence = data[4];
    sample->observation_update_time_sec = data[5];
    sample->residual_h_m = data[6];
    sample->nis_h_2d = data[7];
    sample->nis_3d = data[8];
    sample->gnss_position_applied = data[9] >= 0.5;
    sample->gamma_clipped = data[10];
    sample->observation_score = data[11];
    sample->process_score = data[12];
    sample->lambda_vrw = data[13];
    sample->lambda_accbias = data[14];
    sample->valid =
      std::isfinite(sample->publish_ros_time_sec) &&
      std::isfinite(sample->residual_h_m) &&
      std::isfinite(sample->nis_h_2d);
    return true;
  }

  void maybePublishShadowSupervisorPerformanceProxy_(
    const nav_msgs::msg::Odometry & local_odom)
  {
    if (!shadow_supervisor_perf_proxy_publish_enable_ ||
        !shadow_supervisor_perf_proxy_pub_) {
      return;
    }
    const ShadowSupervisorPerformanceProxySample sample =
      makeShadowSupervisorPerformanceProxySample_(
        now().seconds(),
        rclcpp::Time(local_odom.header.stamp).seconds());
    if (!sample.valid) {
      return;
    }
    shadow_supervisor_perf_proxy_pub_->publish(
      encodeShadowSupervisorPerformanceProxy_(sample));
  }

  void handleShadowSupervisorPerformanceProxy_(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (!shadow_supervisor_perf_proxy_subscribe_enable_ || !msg) {
      return;
    }
    ShadowSupervisorPerformanceProxySample sample;
    if (!decodeShadowSupervisorPerformanceProxy_(msg->data, &sample) ||
        !sample.valid) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignored malformed shadow supervisor performance proxy message.");
      return;
    }
    shadow_supervisor_latest_perf_proxy_ = sample;
  }

  ShadowSupervisorPredictiveScoreSample makeShadowSupervisorPredictiveScoreSample_(
    double publish_ros_time_sec,
    double gnss_stamp_sec,
    double measurement_latitude_deg,
    double measurement_longitude_deg,
    double measurement_altitude_m)
  {
    ShadowSupervisorPredictiveScoreSample sample;
    sample.publish_ros_time_sec = publish_ros_time_sec;
    sample.gnss_stamp_sec = gnss_stamp_sec;
    sample.core_time_sec = last_core_time_;
    sample.sequence =
      static_cast<double>(++shadow_supervisor_predictive_score_local_sequence_);

    if (!core_ || !core_initialized_ || !have_origin_) {
      return sample;
    }
    if (!std::isfinite(measurement_latitude_deg) ||
        !std::isfinite(measurement_longitude_deg) ||
        !std::isfinite(measurement_altitude_m)) {
      return sample;
    }

    const kfcore::State state = core_->current();
    if (!std::isfinite(state.lat_deg) ||
        !std::isfinite(state.lon_deg) ||
        !std::isfinite(state.h_m)) {
      return sample;
    }

    double core_x = std::numeric_limits<double>::quiet_NaN();
    double core_y = std::numeric_limits<double>::quiet_NaN();
    double core_z = std::numeric_limits<double>::quiet_NaN();
    double gnss_x = std::numeric_limits<double>::quiet_NaN();
    double gnss_y = std::numeric_limits<double>::quiet_NaN();
    double gnss_z = std::numeric_limits<double>::quiet_NaN();
    geo::llh_to_ecef(
      state.lat_deg * M_PI / 180.0,
      state.lon_deg * M_PI / 180.0,
      state.h_m,
      core_x,
      core_y,
      core_z);
    geo::llh_to_ecef(
      measurement_latitude_deg * M_PI / 180.0,
      measurement_longitude_deg * M_PI / 180.0,
      measurement_altitude_m,
      gnss_x,
      gnss_y,
      gnss_z);
    const Eigen::Vector3d enu_core =
      geo::ecef_to_enu({core_x, core_y, core_z}, origin_ecef_, origin_lat_, origin_lon_);
    const Eigen::Vector3d enu_gnss =
      geo::ecef_to_enu({gnss_x, gnss_y, gnss_z}, origin_ecef_, origin_lat_, origin_lon_);
    if (!enu_core.allFinite() || !enu_gnss.allFinite()) {
      return sample;
    }

    const Eigen::MatrixXd covariance = core_->covariance();
    if (covariance.rows() < 2 || covariance.cols() < 2 ||
        !std::isfinite(covariance(0, 0)) ||
        !std::isfinite(covariance(0, 1)) ||
        !std::isfinite(covariance(1, 0)) ||
        !std::isfinite(covariance(1, 1))) {
      return sample;
    }

    sample.residual_n_m = enu_gnss.y() - enu_core.y();
    sample.residual_e_m = enu_gnss.x() - enu_core.x();
    if (!std::isfinite(sample.residual_n_m) ||
        !std::isfinite(sample.residual_e_m)) {
      return sample;
    }
    sample.residual_h_m =
      std::hypot(sample.residual_n_m, sample.residual_e_m);

    const double eval_var_h =
      shadow_supervisor_predictive_score_eval_std_h_m_ *
      shadow_supervisor_predictive_score_eval_std_h_m_;
    sample.eval_s_nn_m2 = covariance(0, 0) + eval_var_h;
    sample.eval_s_ne_m2 = 0.5 * (covariance(0, 1) + covariance(1, 0));
    sample.eval_s_ee_m2 = covariance(1, 1) + eval_var_h;
    const double det =
      sample.eval_s_nn_m2 * sample.eval_s_ee_m2 -
      sample.eval_s_ne_m2 * sample.eval_s_ne_m2;
    if (!std::isfinite(sample.eval_s_nn_m2) ||
        !std::isfinite(sample.eval_s_ne_m2) ||
        !std::isfinite(sample.eval_s_ee_m2) ||
        !std::isfinite(det) ||
        det <= 1.0e-12 ||
        sample.eval_s_nn_m2 <= 0.0 ||
        sample.eval_s_ee_m2 <= 0.0) {
      return sample;
    }

    const double quad =
      (sample.eval_s_ee_m2 * sample.residual_n_m * sample.residual_n_m -
       2.0 * sample.eval_s_ne_m2 * sample.residual_n_m * sample.residual_e_m +
       sample.eval_s_nn_m2 * sample.residual_e_m * sample.residual_e_m) / det;
    sample.gpps_h = quad + std::log(det);

    const kfcore::ObservationDebug observation_debug = core_->lastObservationDebug();
    const kfcore::BoundedAdaptiveRDebug & rq =
      observation_debug.gnss_position_adaptive_r;
    sample.gamma_clipped = rq.gamma_clipped;
    sample.observation_score = rq.observation_score;
    sample.valid =
      std::isfinite(sample.residual_h_m) &&
      std::isfinite(sample.gpps_h);
    return sample;
  }

  std_msgs::msg::Float64MultiArray encodeShadowSupervisorPredictiveScore_(
    const ShadowSupervisorPredictiveScoreSample & sample) const
  {
    std_msgs::msg::Float64MultiArray msg;
    auto & data = msg.data;
    data.reserve(14);
    data.push_back(1.0);  // version
    data.push_back(sample.sequence);
    data.push_back(sample.publish_ros_time_sec);
    data.push_back(sample.gnss_stamp_sec);
    data.push_back(sample.core_time_sec);
    data.push_back(sample.residual_n_m);
    data.push_back(sample.residual_e_m);
    data.push_back(sample.residual_h_m);
    data.push_back(sample.eval_s_nn_m2);
    data.push_back(sample.eval_s_ne_m2);
    data.push_back(sample.eval_s_ee_m2);
    data.push_back(sample.gpps_h);
    data.push_back(sample.gamma_clipped);
    data.push_back(sample.observation_score);
    return msg;
  }

  bool decodeShadowSupervisorPredictiveScore_(
    const std::vector<double> & data,
    ShadowSupervisorPredictiveScoreSample * sample) const
  {
    if (sample == nullptr || data.size() < 14 ||
        !std::isfinite(data[0]) || std::abs(data[0] - 1.0) > 1.0e-6) {
      return false;
    }
    sample->sequence = data[1];
    sample->publish_ros_time_sec = data[2];
    sample->gnss_stamp_sec = data[3];
    sample->core_time_sec = data[4];
    sample->residual_n_m = data[5];
    sample->residual_e_m = data[6];
    sample->residual_h_m = data[7];
    sample->eval_s_nn_m2 = data[8];
    sample->eval_s_ne_m2 = data[9];
    sample->eval_s_ee_m2 = data[10];
    sample->gpps_h = data[11];
    sample->gamma_clipped = data[12];
    sample->observation_score = data[13];
    sample->valid =
      std::isfinite(sample->publish_ros_time_sec) &&
      std::isfinite(sample->gnss_stamp_sec) &&
      std::isfinite(sample->residual_h_m) &&
      std::isfinite(sample->gpps_h);
    return true;
  }

  void handleShadowSupervisorPredictiveScore_(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (!shadow_supervisor_predictive_score_subscribe_enable_ || !msg) {
      return;
    }
    ShadowSupervisorPredictiveScoreSample sample;
    if (!decodeShadowSupervisorPredictiveScore_(msg->data, &sample) ||
        !sample.valid) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignored malformed shadow supervisor predictive-score message.");
      return;
    }
    shadow_supervisor_latest_predictive_score_ = sample;
    if (shadow_supervisor_predictive_score_debug_enable_) {
      const double ros_now_sec = now().seconds();
      shadow_supervisor_predictive_score_reference_history_.push_back(sample);
      flushShadowSupervisorPredictiveScorePending_(ros_now_sec);
    }
  }

  double shadowSupervisorPredictiveScoreRetentionSec_() const
  {
    if (std::isfinite(shadow_supervisor_predictive_score_max_age_sec_) &&
        shadow_supervisor_predictive_score_max_age_sec_ >= 0.0) {
      return shadow_supervisor_predictive_score_max_age_sec_ + 1.0;
    }
    return 5.0;
  }

  void pruneShadowSupervisorPredictiveScoreReferenceHistory_(
    double ros_now_sec)
  {
    const double retention_sec = shadowSupervisorPredictiveScoreRetentionSec_();
    while (!shadow_supervisor_predictive_score_reference_history_.empty()) {
      const auto & sample =
        shadow_supervisor_predictive_score_reference_history_.front();
      if (!std::isfinite(ros_now_sec) ||
          !std::isfinite(sample.publish_ros_time_sec) ||
          (ros_now_sec - sample.publish_ros_time_sec) <= retention_sec) {
        break;
      }
      shadow_supervisor_predictive_score_reference_history_.pop_front();
    }
    while (shadow_supervisor_predictive_score_reference_history_.size() > 64) {
      shadow_supervisor_predictive_score_reference_history_.pop_front();
    }
  }

  const ShadowSupervisorPredictiveScoreSample *
  findShadowSupervisorPredictiveScoreReference_(
    const ShadowSupervisorPredictiveScoreSample & local_sample) const
  {
    if (!local_sample.valid ||
        !std::isfinite(local_sample.gnss_stamp_sec)) {
      return nullptr;
    }
    const bool enforce_tolerance =
      std::isfinite(shadow_supervisor_predictive_score_stamp_tolerance_sec_) &&
      shadow_supervisor_predictive_score_stamp_tolerance_sec_ >= 0.0;
    const ShadowSupervisorPredictiveScoreSample * best_sample = nullptr;
    double best_gap_sec = std::numeric_limits<double>::infinity();
    for (const auto & sample :
         shadow_supervisor_predictive_score_reference_history_) {
      if (!sample.valid || !std::isfinite(sample.gnss_stamp_sec)) {
        continue;
      }
      const double gap_sec =
        std::abs(sample.gnss_stamp_sec - local_sample.gnss_stamp_sec);
      if (!std::isfinite(gap_sec)) {
        continue;
      }
      if (enforce_tolerance &&
          gap_sec > shadow_supervisor_predictive_score_stamp_tolerance_sec_) {
        continue;
      }
      if (gap_sec < best_gap_sec) {
        best_gap_sec = gap_sec;
        best_sample = &sample;
      }
    }
    return best_sample;
  }

  void writeShadowSupervisorPredictiveScoreDebugRow_(
    double ros_now_sec,
    const ShadowSupervisorPredictiveScoreSample & local_sample,
    const ShadowSupervisorPredictiveScoreSample * main_sample_ptr)
  {
    if (!shadow_supervisor_predictive_score_debug_enable_ ||
        !shadow_supervisor_predictive_score_debug_csv_.is_open()) {
      return;
    }
    const ShadowSupervisorPredictiveScoreSample empty_main_sample;
    const bool have_reference =
      main_sample_ptr != nullptr && main_sample_ptr->valid;
    const ShadowSupervisorPredictiveScoreSample & main_sample =
      have_reference ? *main_sample_ptr : empty_main_sample;
    const double main_sample_age_sec =
      have_reference
        ? ros_now_sec - main_sample.publish_ros_time_sec
        : std::numeric_limits<double>::quiet_NaN();
    const double shadow_sample_age_sec =
      local_sample.valid ? ros_now_sec - local_sample.publish_ros_time_sec
                         : std::numeric_limits<double>::quiet_NaN();
    const double gnss_stamp_gap_sec =
      (have_reference &&
       std::isfinite(main_sample.gnss_stamp_sec) &&
       std::isfinite(local_sample.gnss_stamp_sec))
        ? std::abs(
            local_sample.gnss_stamp_sec -
            main_sample.gnss_stamp_sec)
        : std::numeric_limits<double>::quiet_NaN();
    const bool reference_fresh =
      have_reference &&
      std::isfinite(main_sample_age_sec) &&
      (shadow_supervisor_predictive_score_max_age_sec_ < 0.0 ||
       main_sample_age_sec <= shadow_supervisor_predictive_score_max_age_sec_);
    const bool shadow_fresh =
      local_sample.valid &&
      std::isfinite(shadow_sample_age_sec) &&
      (shadow_supervisor_predictive_score_max_age_sec_ < 0.0 ||
       shadow_sample_age_sec <= shadow_supervisor_predictive_score_max_age_sec_);
    const bool stamp_aligned =
      have_reference &&
      std::isfinite(gnss_stamp_gap_sec) &&
      (shadow_supervisor_predictive_score_stamp_tolerance_sec_ < 0.0 ||
       gnss_stamp_gap_sec <= shadow_supervisor_predictive_score_stamp_tolerance_sec_);

    bool prediction_valid = false;
    std::string prediction_reason = "valid";
    if (!local_sample.valid) {
      prediction_reason = "shadow_invalid";
    } else if (!shadow_fresh) {
      prediction_reason = "shadow_stale";
    } else if (!have_reference) {
      prediction_reason = "main_missing";
    } else if (!reference_fresh) {
      prediction_reason = "main_stale";
    } else if (!stamp_aligned) {
      prediction_reason = "stamp_mismatch";
    } else {
      prediction_valid = true;
    }

    double gpps_gap_h = std::numeric_limits<double>::quiet_NaN();
    double residual_gap_h_m = std::numeric_limits<double>::quiet_NaN();
    double gpps_gap_mean_5s = std::numeric_limits<double>::quiet_NaN();
    double gpps_gap_mean_10s = std::numeric_limits<double>::quiet_NaN();
    double gpps_shadow_win_frac_10s = std::numeric_limits<double>::quiet_NaN();
    double residual_gap_mean_5s_m = std::numeric_limits<double>::quiet_NaN();
    if (prediction_valid) {
      gpps_gap_h =
        local_sample.gpps_h - main_sample.gpps_h;
      residual_gap_h_m =
        local_sample.residual_h_m -
        main_sample.residual_h_m;
      shadow_supervisor_predictive_score_gap_history_.push_back(
        ShadowSupervisorPredictiveScoreGapSample{
          ros_now_sec,
          gpps_gap_h,
          residual_gap_h_m,
          gpps_gap_h < 0.0});
    }
    const double history_window_sec =
      std::max(
        shadow_supervisor_predictive_score_short_window_sec_,
        shadow_supervisor_predictive_score_long_window_sec_);
    while (!shadow_supervisor_predictive_score_gap_history_.empty() &&
           std::isfinite(history_window_sec) &&
           history_window_sec >= 0.0 &&
           (ros_now_sec -
            shadow_supervisor_predictive_score_gap_history_.front().ros_time_sec) >
             history_window_sec + 1.0) {
      shadow_supervisor_predictive_score_gap_history_.pop_front();
    }
    auto mean_recent = [&](
        auto value_fn,
        double window_sec) -> double {
      if (window_sec < 0.0) {
        return std::numeric_limits<double>::quiet_NaN();
      }
      double sum = 0.0;
      int count = 0;
      for (auto it = shadow_supervisor_predictive_score_gap_history_.rbegin();
           it != shadow_supervisor_predictive_score_gap_history_.rend();
           ++it) {
        if ((ros_now_sec - it->ros_time_sec) > window_sec) {
          break;
        }
        const double value = value_fn(*it);
        if (!std::isfinite(value)) {
          continue;
        }
        sum += value;
        ++count;
      }
      return count > 0 ? sum / static_cast<double>(count)
                       : std::numeric_limits<double>::quiet_NaN();
    };
    auto win_frac_recent = [&](double window_sec) -> double {
      if (window_sec < 0.0) {
        return std::numeric_limits<double>::quiet_NaN();
      }
      int count = 0;
      int wins = 0;
      for (auto it = shadow_supervisor_predictive_score_gap_history_.rbegin();
           it != shadow_supervisor_predictive_score_gap_history_.rend();
           ++it) {
        if ((ros_now_sec - it->ros_time_sec) > window_sec) {
          break;
        }
        if (!std::isfinite(it->gpps_gap_h)) {
          continue;
        }
        ++count;
        if (it->shadow_win) {
          ++wins;
        }
      }
      return count > 0 ? static_cast<double>(wins) / static_cast<double>(count)
                       : std::numeric_limits<double>::quiet_NaN();
    };
    gpps_gap_mean_5s =
      mean_recent(
        [](const ShadowSupervisorPredictiveScoreGapSample & sample) {
          return sample.gpps_gap_h;
        },
        shadow_supervisor_predictive_score_short_window_sec_);
    gpps_gap_mean_10s =
      mean_recent(
        [](const ShadowSupervisorPredictiveScoreGapSample & sample) {
          return sample.gpps_gap_h;
        },
        shadow_supervisor_predictive_score_long_window_sec_);
    residual_gap_mean_5s_m =
      mean_recent(
        [](const ShadowSupervisorPredictiveScoreGapSample & sample) {
          return sample.residual_gap_h_m;
        },
        shadow_supervisor_predictive_score_short_window_sec_);
    gpps_shadow_win_frac_10s =
      win_frac_recent(shadow_supervisor_predictive_score_long_window_sec_);

    const double main_gamma =
      have_reference ? main_sample.gamma_clipped :
        std::numeric_limits<double>::quiet_NaN();
    const double shadow_gamma = local_sample.gamma_clipped;
    const double gamma_clipped =
      std::max(
        std::isfinite(main_gamma) ? main_gamma : 1.0,
        std::isfinite(shadow_gamma) ? shadow_gamma : 1.0);
    const double main_observation_score =
      have_reference ? main_sample.observation_score :
        std::numeric_limits<double>::quiet_NaN();
    const double shadow_observation_score = local_sample.observation_score;
    const double observation_score =
      std::max(
        std::isfinite(main_observation_score) ? main_observation_score : 0.0,
        std::isfinite(shadow_observation_score) ? shadow_observation_score : 0.0);
    bool observation_guard = false;
    std::string observation_reason = "clear";
    if (std::isfinite(main_gamma) && main_gamma > 1.05) {
      observation_guard = true;
      observation_reason = "main_gamma_guard";
    } else if (std::isfinite(shadow_gamma) && shadow_gamma > 1.05) {
      observation_guard = true;
      observation_reason = "shadow_gamma_guard";
    } else if (std::isfinite(main_observation_score) &&
               main_observation_score > 0.50) {
      observation_guard = true;
      observation_reason = "main_observation_score_guard";
    } else if (std::isfinite(shadow_observation_score) &&
               shadow_observation_score > 0.50) {
      observation_guard = true;
      observation_reason = "shadow_observation_score_guard";
    }

    const double time_since_restore_sec =
      (shadow_supervisor_have_restore_ &&
       std::isfinite(shadow_supervisor_last_restore_core_time_sec_) &&
       std::isfinite(local_sample.core_time_sec))
        ? local_sample.core_time_sec - shadow_supervisor_last_restore_core_time_sec_
        : std::numeric_limits<double>::quiet_NaN();

    shadow_supervisor_predictive_score_debug_csv_
      << (++shadow_supervisor_predictive_score_debug_sequence_) << ','
      << ros_now_sec << ','
      << local_sample.gnss_stamp_sec << ','
      << local_sample.core_time_sec << ','
      << csvSafe_(shadow_supervisor_predictive_score_source_id_) << ','
      << (shadow_supervisor_have_restore_ ? 1 : 0) << ','
      << time_since_restore_sec << ','
      << (prediction_valid ? 1 : 0) << ','
      << csvSafe_(prediction_reason) << ','
      << main_sample_age_sec << ','
      << shadow_sample_age_sec << ','
      << main_sample.residual_n_m << ','
      << main_sample.residual_e_m << ','
      << main_sample.residual_h_m << ','
      << local_sample.residual_n_m << ','
      << local_sample.residual_e_m << ','
      << local_sample.residual_h_m << ','
      << main_sample.eval_s_nn_m2 << ','
      << main_sample.eval_s_ne_m2 << ','
      << main_sample.eval_s_ee_m2 << ','
      << local_sample.eval_s_nn_m2 << ','
      << local_sample.eval_s_ne_m2 << ','
      << local_sample.eval_s_ee_m2 << ','
      << main_sample.gpps_h << ','
      << local_sample.gpps_h << ','
      << gpps_gap_h << ','
      << residual_gap_h_m << ','
      << gpps_gap_mean_5s << ','
      << gpps_gap_mean_10s << ','
      << gpps_shadow_win_frac_10s << ','
      << residual_gap_mean_5s_m << ','
      << (observation_guard ? 1 : 0) << ','
      << csvSafe_(observation_reason) << ','
      << gamma_clipped << ','
      << observation_score << ','
      << "main_only" << ','
      << main_sample.gnss_stamp_sec << ','
      << local_sample.gnss_stamp_sec << ','
      << gnss_stamp_gap_sec << '\n';

    ++shadow_supervisor_predictive_score_debug_rows_since_flush_;
    if (shadow_supervisor_predictive_score_debug_rows_since_flush_ >=
        shadow_supervisor_predictive_score_debug_flush_interval_) {
      shadow_supervisor_predictive_score_debug_csv_.flush();
      shadow_supervisor_predictive_score_debug_rows_since_flush_ = 0;
    }
  }

  void flushShadowSupervisorPredictiveScorePending_(double ros_now_sec)
  {
    if (!shadow_supervisor_predictive_score_debug_enable_ ||
        !shadow_supervisor_predictive_score_debug_csv_.is_open()) {
      return;
    }
    pruneShadowSupervisorPredictiveScoreReferenceHistory_(ros_now_sec);
    const double retention_sec = shadowSupervisorPredictiveScoreRetentionSec_();
    for (auto it =
           shadow_supervisor_predictive_score_pending_local_samples_.begin();
         it != shadow_supervisor_predictive_score_pending_local_samples_.end(); ) {
      const ShadowSupervisorPredictiveScoreSample * reference =
        findShadowSupervisorPredictiveScoreReference_(*it);
      const double shadow_sample_age_sec =
        it->valid ? ros_now_sec - it->publish_ros_time_sec :
          std::numeric_limits<double>::quiet_NaN();
      const bool expired =
        !it->valid ||
        (std::isfinite(shadow_sample_age_sec) &&
         shadow_sample_age_sec > retention_sec);
      if (reference != nullptr || expired) {
        writeShadowSupervisorPredictiveScoreDebugRow_(
          ros_now_sec, *it, reference);
        it = shadow_supervisor_predictive_score_pending_local_samples_.erase(it);
      } else {
        ++it;
      }
    }
    while (shadow_supervisor_predictive_score_pending_local_samples_.size() > 64) {
      writeShadowSupervisorPredictiveScoreDebugRow_(
        ros_now_sec,
        shadow_supervisor_predictive_score_pending_local_samples_.front(),
        nullptr);
      shadow_supervisor_predictive_score_pending_local_samples_.pop_front();
    }
  }

  void maybeHandleShadowSupervisorPredictiveScore_(
    double gnss_stamp_sec,
    double measurement_latitude_deg,
    double measurement_longitude_deg,
    double measurement_altitude_m)
  {
    if (!shadow_supervisor_predictive_score_publish_enable_ &&
        !shadow_supervisor_predictive_score_debug_enable_) {
      return;
    }

    const double ros_now_sec = now().seconds();
    const ShadowSupervisorPredictiveScoreSample local_sample =
      makeShadowSupervisorPredictiveScoreSample_(
        ros_now_sec,
        gnss_stamp_sec,
        measurement_latitude_deg,
        measurement_longitude_deg,
        measurement_altitude_m);

    if (shadow_supervisor_predictive_score_publish_enable_ &&
        shadow_supervisor_predictive_score_pub_ &&
        local_sample.valid) {
      shadow_supervisor_predictive_score_pub_->publish(
        encodeShadowSupervisorPredictiveScore_(local_sample));
    }

    if (shadow_supervisor_predictive_score_debug_enable_ &&
        shadow_supervisor_predictive_score_debug_csv_.is_open()) {
      shadow_supervisor_predictive_score_pending_local_samples_.push_back(
        local_sample);
      flushShadowSupervisorPredictiveScorePending_(ros_now_sec);
    }
  }

  void configureShadowSupervisorVelocityPredictiveScore_()
  {
    if (!shadow_supervisor_velocity_predictive_score_publish_enable_ &&
        !shadow_supervisor_velocity_predictive_score_subscribe_enable_ &&
        !shadow_supervisor_velocity_predictive_score_debug_enable_) {
      return;
    }

    if (shadow_supervisor_velocity_predictive_score_publish_enable_) {
      shadow_supervisor_velocity_predictive_score_pub_ =
        this->create_publisher<std_msgs::msg::Float64MultiArray>(
          shadow_supervisor_velocity_predictive_score_topic_,
          rclcpp::QoS(10).reliable().durability_volatile());
    }
    if (shadow_supervisor_velocity_predictive_score_subscribe_enable_) {
      shadow_supervisor_velocity_predictive_score_sub_ =
        this->create_subscription<std_msgs::msg::Float64MultiArray>(
          shadow_supervisor_velocity_predictive_score_topic_,
          rclcpp::QoS(10).reliable().durability_volatile(),
          [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
            handleShadowSupervisorVelocityPredictiveScore_(msg);
          });
    }

    if (shadow_supervisor_velocity_predictive_score_debug_enable_ &&
        !shadow_supervisor_velocity_predictive_score_debug_csv_path_.empty()) {
      try {
        const std::filesystem::path csv_path(
          shadow_supervisor_velocity_predictive_score_debug_csv_path_);
        if (csv_path.has_parent_path()) {
          std::filesystem::create_directories(csv_path.parent_path());
        }
        shadow_supervisor_velocity_predictive_score_debug_csv_.open(
          csv_path, std::ios::out | std::ios::trunc);
      } catch (const std::exception & e) {
        RCLCPP_WARN(
          get_logger(),
          "Failed to open shadow supervisor velocity predictive-score CSV %s: %s",
          shadow_supervisor_velocity_predictive_score_debug_csv_path_.c_str(),
          e.what());
      }
      if (shadow_supervisor_velocity_predictive_score_debug_csv_.is_open()) {
        shadow_supervisor_velocity_predictive_score_debug_csv_
          << "schema_version,sequence,ros_time_sec,gnss_stamp_sec,"
          << "gnss_velocity_source_id,restore_applied,time_since_restore_sec,"
          << "prediction_valid,prediction_reason,main_sample_age_sec,"
          << "shadow_sample_age_sec,raw_velocity_n_mps,raw_velocity_e_mps,"
          << "raw_velocity_d_mps,raw_speed_accuracy_mps,"
          << "raw_vertical_speed_accuracy_mps,main_prior_vn_mps,"
          << "main_prior_ve_mps,main_prior_vd_mps,shadow_prior_vn_mps,"
          << "shadow_prior_ve_mps,shadow_prior_vd_mps,"
          << "main_velocity_residual_n_mps,main_velocity_residual_e_mps,"
          << "main_velocity_residual_d_mps,main_velocity_residual_h_mps,"
          << "shadow_velocity_residual_n_mps,shadow_velocity_residual_e_mps,"
          << "shadow_velocity_residual_d_mps,shadow_velocity_residual_h_mps,"
          << "main_eval_sv_nn_m2ps2,main_eval_sv_ne_m2ps2,"
          << "main_eval_sv_ee_m2ps2,shadow_eval_sv_nn_m2ps2,"
          << "shadow_eval_sv_ne_m2ps2,shadow_eval_sv_ee_m2ps2,"
          << "main_gvps_h,shadow_gvps_h,gvps_gap_h,"
          << "velocity_residual_gap_h_mps,gvps_gap_mean_3s,"
          << "gvps_gap_mean_5s,gvps_gap_mean_10s,"
          << "gvps_shadow_win_frac_10s,velocity_residual_gap_mean_5s_mps,"
          << "valid_recent_count_10s,velocity_observation_guard,"
          << "velocity_observation_reason,velocity_gamma_clipped,"
          << "velocity_observation_score,position_observation_guard,"
          << "position_observation_reason,output_policy,main_gnss_stamp_sec,"
          << "shadow_gnss_stamp_sec,gnss_stamp_gap_sec\n";
        shadow_supervisor_velocity_predictive_score_debug_csv_.flush();
      } else {
        RCLCPP_WARN(
          get_logger(),
          "Failed to open shadow supervisor velocity predictive-score CSV: %s",
          shadow_supervisor_velocity_predictive_score_debug_csv_path_.c_str());
      }
    }

    RCLCPP_INFO(
      get_logger(),
      "Shadow supervisor velocity predictive score: publish=%s subscribe=%s "
      "debug=%s topic=%s csv=%s max_age=%.3f stamp_tolerance=%.3f "
      "eval_std_h=%.3f eval_std_u=%.3f",
      shadow_supervisor_velocity_predictive_score_publish_enable_ ? "true" : "false",
      shadow_supervisor_velocity_predictive_score_subscribe_enable_ ? "true" : "false",
      shadow_supervisor_velocity_predictive_score_debug_enable_ ? "true" : "false",
      shadow_supervisor_velocity_predictive_score_topic_.c_str(),
      shadow_supervisor_velocity_predictive_score_debug_csv_path_.empty()
        ? "<disabled>" : shadow_supervisor_velocity_predictive_score_debug_csv_path_.c_str(),
      shadow_supervisor_velocity_predictive_score_max_age_sec_,
      shadow_supervisor_velocity_predictive_score_stamp_tolerance_sec_,
      shadow_supervisor_velocity_predictive_score_eval_std_h_mps_,
      shadow_supervisor_velocity_predictive_score_eval_std_u_mps_);
  }

  ShadowSupervisorVelocityPredictiveScoreSample
  makeShadowSupervisorVelocityPredictiveScoreSample_(
    double publish_ros_time_sec,
    double gnss_stamp_sec,
    double raw_vn_mps,
    double raw_ve_mps,
    double raw_vd_mps,
    double raw_speed_accuracy_mps,
    double raw_vertical_speed_accuracy_mps)
  {
    ShadowSupervisorVelocityPredictiveScoreSample sample;
    sample.publish_ros_time_sec = publish_ros_time_sec;
    sample.gnss_stamp_sec = gnss_stamp_sec;
    sample.core_time_sec = last_core_time_;
    sample.sequence =
      static_cast<double>(++shadow_supervisor_velocity_predictive_score_local_sequence_);
    sample.raw_vn_mps = raw_vn_mps;
    sample.raw_ve_mps = raw_ve_mps;
    sample.raw_vd_mps = raw_vd_mps;
    sample.raw_speed_accuracy_mps = raw_speed_accuracy_mps;
    sample.raw_vertical_speed_accuracy_mps = raw_vertical_speed_accuracy_mps;

    if (!core_ || !core_initialized_) {
      return sample;
    }
    if (!std::isfinite(raw_vn_mps) ||
        !std::isfinite(raw_ve_mps) ||
        !std::isfinite(raw_vd_mps)) {
      return sample;
    }

    const kfcore::State state = core_->current();
    if (!std::isfinite(state.vN) ||
        !std::isfinite(state.vE) ||
        !std::isfinite(state.vD)) {
      return sample;
    }
    sample.prior_vn_mps = state.vN;
    sample.prior_ve_mps = state.vE;
    sample.prior_vd_mps = state.vD;
    sample.residual_n_mps = raw_vn_mps - state.vN;
    sample.residual_e_mps = raw_ve_mps - state.vE;
    sample.residual_d_mps = raw_vd_mps - state.vD;
    sample.residual_h_mps =
      std::hypot(sample.residual_n_mps, sample.residual_e_mps);
    if (!std::isfinite(sample.residual_h_mps) ||
        !std::isfinite(sample.residual_d_mps)) {
      return sample;
    }

    const Eigen::MatrixXd covariance = core_->covariance();
    if (covariance.rows() < 6 || covariance.cols() < 6 ||
        !std::isfinite(covariance(3, 3)) ||
        !std::isfinite(covariance(3, 4)) ||
        !std::isfinite(covariance(4, 3)) ||
        !std::isfinite(covariance(4, 4)) ||
        !std::isfinite(covariance(5, 5))) {
      return sample;
    }

    const double raw_eval_std_h =
      (std::isfinite(raw_speed_accuracy_mps) && raw_speed_accuracy_mps > 0.0)
        ? raw_speed_accuracy_mps
        : 0.0;
    const double raw_eval_std_u =
      (std::isfinite(raw_vertical_speed_accuracy_mps) &&
       raw_vertical_speed_accuracy_mps > 0.0)
        ? raw_vertical_speed_accuracy_mps
        : 0.0;
    const double eval_std_h =
      std::max(shadow_supervisor_velocity_predictive_score_eval_std_h_mps_,
               raw_eval_std_h);
    const double eval_std_u =
      std::max(shadow_supervisor_velocity_predictive_score_eval_std_u_mps_,
               raw_eval_std_u);
    const double eval_var_h = eval_std_h * eval_std_h;
    const double eval_var_u = eval_std_u * eval_std_u;
    sample.eval_sv_nn_m2ps2 = covariance(3, 3) + eval_var_h;
    sample.eval_sv_ne_m2ps2 = 0.5 * (covariance(3, 4) + covariance(4, 3));
    sample.eval_sv_ee_m2ps2 = covariance(4, 4) + eval_var_h;
    sample.eval_sv_dd_m2ps2 = covariance(5, 5) + eval_var_u;
    const double det =
      sample.eval_sv_nn_m2ps2 * sample.eval_sv_ee_m2ps2 -
      sample.eval_sv_ne_m2ps2 * sample.eval_sv_ne_m2ps2;
    if (!std::isfinite(sample.eval_sv_nn_m2ps2) ||
        !std::isfinite(sample.eval_sv_ne_m2ps2) ||
        !std::isfinite(sample.eval_sv_ee_m2ps2) ||
        !std::isfinite(sample.eval_sv_dd_m2ps2) ||
        !std::isfinite(det) ||
        det <= 1.0e-12 ||
        sample.eval_sv_nn_m2ps2 <= 0.0 ||
        sample.eval_sv_ee_m2ps2 <= 0.0 ||
        sample.eval_sv_dd_m2ps2 <= 0.0) {
      return sample;
    }

    const double quad_h =
      (sample.eval_sv_ee_m2ps2 * sample.residual_n_mps * sample.residual_n_mps -
       2.0 * sample.eval_sv_ne_m2ps2 *
         sample.residual_n_mps * sample.residual_e_mps +
       sample.eval_sv_nn_m2ps2 * sample.residual_e_mps * sample.residual_e_mps) /
      det;
    sample.gvps_h = quad_h + std::log(det);
    sample.gvps_3d =
      sample.gvps_h +
      sample.residual_d_mps * sample.residual_d_mps / sample.eval_sv_dd_m2ps2 +
      std::log(sample.eval_sv_dd_m2ps2);

    const kfcore::ObservationDebug observation_debug = core_->lastObservationDebug();
    sample.velocity_gamma_clipped =
      observation_debug.gnss_velocity_adaptive_r.gamma_clipped;
    sample.velocity_observation_score =
      observation_debug.gnss_velocity_adaptive_r.observation_score;
    sample.position_gamma_clipped =
      observation_debug.gnss_position_adaptive_r.gamma_clipped;
    sample.position_observation_score =
      observation_debug.gnss_position_adaptive_r.observation_score;
    sample.valid =
      std::isfinite(sample.residual_h_mps) &&
      std::isfinite(sample.gvps_h);
    return sample;
  }

  std_msgs::msg::Float64MultiArray encodeShadowSupervisorVelocityPredictiveScore_(
    const ShadowSupervisorVelocityPredictiveScoreSample & sample) const
  {
    std_msgs::msg::Float64MultiArray msg;
    auto & data = msg.data;
    data.reserve(27);
    data.push_back(1.0);  // version
    data.push_back(sample.sequence);
    data.push_back(sample.publish_ros_time_sec);
    data.push_back(sample.gnss_stamp_sec);
    data.push_back(sample.core_time_sec);
    data.push_back(sample.raw_vn_mps);
    data.push_back(sample.raw_ve_mps);
    data.push_back(sample.raw_vd_mps);
    data.push_back(sample.raw_speed_accuracy_mps);
    data.push_back(sample.raw_vertical_speed_accuracy_mps);
    data.push_back(sample.prior_vn_mps);
    data.push_back(sample.prior_ve_mps);
    data.push_back(sample.prior_vd_mps);
    data.push_back(sample.residual_n_mps);
    data.push_back(sample.residual_e_mps);
    data.push_back(sample.residual_d_mps);
    data.push_back(sample.residual_h_mps);
    data.push_back(sample.eval_sv_nn_m2ps2);
    data.push_back(sample.eval_sv_ne_m2ps2);
    data.push_back(sample.eval_sv_ee_m2ps2);
    data.push_back(sample.eval_sv_dd_m2ps2);
    data.push_back(sample.gvps_h);
    data.push_back(sample.gvps_3d);
    data.push_back(sample.velocity_gamma_clipped);
    data.push_back(sample.velocity_observation_score);
    data.push_back(sample.position_gamma_clipped);
    data.push_back(sample.position_observation_score);
    return msg;
  }

  bool decodeShadowSupervisorVelocityPredictiveScore_(
    const std::vector<double> & data,
    ShadowSupervisorVelocityPredictiveScoreSample * sample) const
  {
    if (sample == nullptr || data.size() < 27 ||
        !std::isfinite(data[0]) || std::abs(data[0] - 1.0) > 1.0e-6) {
      return false;
    }
    sample->sequence = data[1];
    sample->publish_ros_time_sec = data[2];
    sample->gnss_stamp_sec = data[3];
    sample->core_time_sec = data[4];
    sample->raw_vn_mps = data[5];
    sample->raw_ve_mps = data[6];
    sample->raw_vd_mps = data[7];
    sample->raw_speed_accuracy_mps = data[8];
    sample->raw_vertical_speed_accuracy_mps = data[9];
    sample->prior_vn_mps = data[10];
    sample->prior_ve_mps = data[11];
    sample->prior_vd_mps = data[12];
    sample->residual_n_mps = data[13];
    sample->residual_e_mps = data[14];
    sample->residual_d_mps = data[15];
    sample->residual_h_mps = data[16];
    sample->eval_sv_nn_m2ps2 = data[17];
    sample->eval_sv_ne_m2ps2 = data[18];
    sample->eval_sv_ee_m2ps2 = data[19];
    sample->eval_sv_dd_m2ps2 = data[20];
    sample->gvps_h = data[21];
    sample->gvps_3d = data[22];
    sample->velocity_gamma_clipped = data[23];
    sample->velocity_observation_score = data[24];
    sample->position_gamma_clipped = data[25];
    sample->position_observation_score = data[26];
    sample->valid =
      std::isfinite(sample->publish_ros_time_sec) &&
      std::isfinite(sample->gnss_stamp_sec) &&
      std::isfinite(sample->residual_h_mps) &&
      std::isfinite(sample->gvps_h);
    return true;
  }

  void handleShadowSupervisorVelocityPredictiveScore_(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (!shadow_supervisor_velocity_predictive_score_subscribe_enable_ || !msg) {
      return;
    }
    ShadowSupervisorVelocityPredictiveScoreSample sample;
    if (!decodeShadowSupervisorVelocityPredictiveScore_(msg->data, &sample) ||
        !sample.valid) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignored malformed shadow supervisor velocity predictive-score message.");
      return;
    }
    shadow_supervisor_latest_velocity_predictive_score_ = sample;
    if (shadow_supervisor_velocity_predictive_score_debug_enable_) {
      const double ros_now_sec = now().seconds();
      shadow_supervisor_velocity_predictive_score_reference_history_.push_back(sample);
      flushShadowSupervisorVelocityPredictiveScorePending_(ros_now_sec);
    }
  }

  double shadowSupervisorVelocityPredictiveScoreRetentionSec_() const
  {
    if (std::isfinite(shadow_supervisor_velocity_predictive_score_max_age_sec_) &&
        shadow_supervisor_velocity_predictive_score_max_age_sec_ >= 0.0) {
      return shadow_supervisor_velocity_predictive_score_max_age_sec_ + 1.0;
    }
    return 5.0;
  }

  void pruneShadowSupervisorVelocityPredictiveScoreReferenceHistory_(
    double ros_now_sec)
  {
    const double retention_sec = shadowSupervisorVelocityPredictiveScoreRetentionSec_();
    while (!shadow_supervisor_velocity_predictive_score_reference_history_.empty()) {
      const auto & sample =
        shadow_supervisor_velocity_predictive_score_reference_history_.front();
      if (!std::isfinite(ros_now_sec) ||
          !std::isfinite(sample.publish_ros_time_sec) ||
          (ros_now_sec - sample.publish_ros_time_sec) <= retention_sec) {
        break;
      }
      shadow_supervisor_velocity_predictive_score_reference_history_.pop_front();
    }
    while (shadow_supervisor_velocity_predictive_score_reference_history_.size() > 64) {
      shadow_supervisor_velocity_predictive_score_reference_history_.pop_front();
    }
  }

  const ShadowSupervisorVelocityPredictiveScoreSample *
  findShadowSupervisorVelocityPredictiveScoreReference_(
    const ShadowSupervisorVelocityPredictiveScoreSample & local_sample) const
  {
    if (!local_sample.valid ||
        !std::isfinite(local_sample.gnss_stamp_sec)) {
      return nullptr;
    }
    const bool enforce_tolerance =
      std::isfinite(shadow_supervisor_velocity_predictive_score_stamp_tolerance_sec_) &&
      shadow_supervisor_velocity_predictive_score_stamp_tolerance_sec_ >= 0.0;
    const ShadowSupervisorVelocityPredictiveScoreSample * best_sample = nullptr;
    double best_gap_sec = std::numeric_limits<double>::infinity();
    for (const auto & sample :
         shadow_supervisor_velocity_predictive_score_reference_history_) {
      if (!sample.valid || !std::isfinite(sample.gnss_stamp_sec)) {
        continue;
      }
      const double gap_sec =
        std::abs(sample.gnss_stamp_sec - local_sample.gnss_stamp_sec);
      if (!std::isfinite(gap_sec)) {
        continue;
      }
      if (enforce_tolerance &&
          gap_sec > shadow_supervisor_velocity_predictive_score_stamp_tolerance_sec_) {
        continue;
      }
      if (gap_sec < best_gap_sec) {
        best_gap_sec = gap_sec;
        best_sample = &sample;
      }
    }
    return best_sample;
  }

  void writeShadowSupervisorVelocityPredictiveScoreDebugRow_(
    double ros_now_sec,
    const ShadowSupervisorVelocityPredictiveScoreSample & local_sample,
    const ShadowSupervisorVelocityPredictiveScoreSample * main_sample_ptr)
  {
    if (!shadow_supervisor_velocity_predictive_score_debug_enable_ ||
        !shadow_supervisor_velocity_predictive_score_debug_csv_.is_open()) {
      return;
    }
    const ShadowSupervisorVelocityPredictiveScoreSample empty_main_sample;
    const bool have_reference =
      main_sample_ptr != nullptr && main_sample_ptr->valid;
    const ShadowSupervisorVelocityPredictiveScoreSample & main_sample =
      have_reference ? *main_sample_ptr : empty_main_sample;
    const double main_sample_age_sec =
      have_reference
        ? ros_now_sec - main_sample.publish_ros_time_sec
        : std::numeric_limits<double>::quiet_NaN();
    const double shadow_sample_age_sec =
      local_sample.valid ? ros_now_sec - local_sample.publish_ros_time_sec
                         : std::numeric_limits<double>::quiet_NaN();
    const double gnss_stamp_gap_sec =
      (have_reference &&
       std::isfinite(main_sample.gnss_stamp_sec) &&
       std::isfinite(local_sample.gnss_stamp_sec))
        ? std::abs(local_sample.gnss_stamp_sec - main_sample.gnss_stamp_sec)
        : std::numeric_limits<double>::quiet_NaN();
    const bool reference_fresh =
      have_reference &&
      std::isfinite(main_sample_age_sec) &&
      (shadow_supervisor_velocity_predictive_score_max_age_sec_ < 0.0 ||
       main_sample_age_sec <= shadow_supervisor_velocity_predictive_score_max_age_sec_);
    const bool shadow_fresh =
      local_sample.valid &&
      std::isfinite(shadow_sample_age_sec) &&
      (shadow_supervisor_velocity_predictive_score_max_age_sec_ < 0.0 ||
       shadow_sample_age_sec <= shadow_supervisor_velocity_predictive_score_max_age_sec_);
    const bool stamp_aligned =
      have_reference &&
      std::isfinite(gnss_stamp_gap_sec) &&
      (shadow_supervisor_velocity_predictive_score_stamp_tolerance_sec_ < 0.0 ||
       gnss_stamp_gap_sec <=
         shadow_supervisor_velocity_predictive_score_stamp_tolerance_sec_);

    bool prediction_valid = false;
    std::string prediction_reason = "valid";
    if (!local_sample.valid) {
      prediction_reason = "shadow_invalid";
    } else if (!shadow_fresh) {
      prediction_reason = "shadow_stale";
    } else if (!have_reference) {
      prediction_reason = "main_missing";
    } else if (!reference_fresh) {
      prediction_reason = "main_stale";
    } else if (!stamp_aligned) {
      prediction_reason = "stamp_mismatch";
    } else {
      prediction_valid = true;
    }

    double gvps_gap_h = std::numeric_limits<double>::quiet_NaN();
    double velocity_residual_gap_h_mps =
      std::numeric_limits<double>::quiet_NaN();
    double gvps_gap_mean_3s = std::numeric_limits<double>::quiet_NaN();
    double gvps_gap_mean_5s = std::numeric_limits<double>::quiet_NaN();
    double gvps_gap_mean_10s = std::numeric_limits<double>::quiet_NaN();
    double gvps_shadow_win_frac_10s = std::numeric_limits<double>::quiet_NaN();
    double velocity_residual_gap_mean_5s_mps =
      std::numeric_limits<double>::quiet_NaN();
    int valid_recent_count_10s = 0;
    if (prediction_valid) {
      gvps_gap_h = local_sample.gvps_h - main_sample.gvps_h;
      velocity_residual_gap_h_mps =
        local_sample.residual_h_mps - main_sample.residual_h_mps;
      shadow_supervisor_velocity_predictive_score_gap_history_.push_back(
        ShadowSupervisorVelocityPredictiveScoreGapSample{
          ros_now_sec,
          gvps_gap_h,
          velocity_residual_gap_h_mps,
          gvps_gap_h < 0.0});
    }
    const double history_window_sec =
      std::max(
        3.0,
        std::max(
          shadow_supervisor_velocity_predictive_score_short_window_sec_,
          shadow_supervisor_velocity_predictive_score_long_window_sec_));
    while (!shadow_supervisor_velocity_predictive_score_gap_history_.empty() &&
           std::isfinite(history_window_sec) &&
           history_window_sec >= 0.0 &&
           (ros_now_sec -
            shadow_supervisor_velocity_predictive_score_gap_history_.front().ros_time_sec) >
             history_window_sec + 1.0) {
      shadow_supervisor_velocity_predictive_score_gap_history_.pop_front();
    }
    auto mean_recent = [&](
        auto value_fn,
        double window_sec) -> double {
      if (window_sec < 0.0) {
        return std::numeric_limits<double>::quiet_NaN();
      }
      double sum = 0.0;
      int count = 0;
      for (auto it = shadow_supervisor_velocity_predictive_score_gap_history_.rbegin();
           it != shadow_supervisor_velocity_predictive_score_gap_history_.rend();
           ++it) {
        if ((ros_now_sec - it->ros_time_sec) > window_sec) {
          break;
        }
        const double value = value_fn(*it);
        if (!std::isfinite(value)) {
          continue;
        }
        sum += value;
        ++count;
      }
      return count > 0 ? sum / static_cast<double>(count)
                       : std::numeric_limits<double>::quiet_NaN();
    };
    auto win_frac_recent = [&](double window_sec) -> double {
      if (window_sec < 0.0) {
        return std::numeric_limits<double>::quiet_NaN();
      }
      int count = 0;
      int wins = 0;
      for (auto it = shadow_supervisor_velocity_predictive_score_gap_history_.rbegin();
           it != shadow_supervisor_velocity_predictive_score_gap_history_.rend();
           ++it) {
        if ((ros_now_sec - it->ros_time_sec) > window_sec) {
          break;
        }
        if (!std::isfinite(it->gvps_gap_h)) {
          continue;
        }
        ++count;
        if (it->shadow_win) {
          ++wins;
        }
      }
      return count > 0 ? static_cast<double>(wins) / static_cast<double>(count)
                       : std::numeric_limits<double>::quiet_NaN();
    };
    auto count_recent = [&](double window_sec) -> int {
      if (window_sec < 0.0) {
        return 0;
      }
      int count = 0;
      for (auto it = shadow_supervisor_velocity_predictive_score_gap_history_.rbegin();
           it != shadow_supervisor_velocity_predictive_score_gap_history_.rend();
           ++it) {
        if ((ros_now_sec - it->ros_time_sec) > window_sec) {
          break;
        }
        if (std::isfinite(it->gvps_gap_h)) {
          ++count;
        }
      }
      return count;
    };
    gvps_gap_mean_3s =
      mean_recent(
        [](const ShadowSupervisorVelocityPredictiveScoreGapSample & sample) {
          return sample.gvps_gap_h;
        },
        3.0);
    gvps_gap_mean_5s =
      mean_recent(
        [](const ShadowSupervisorVelocityPredictiveScoreGapSample & sample) {
          return sample.gvps_gap_h;
        },
        shadow_supervisor_velocity_predictive_score_short_window_sec_);
    gvps_gap_mean_10s =
      mean_recent(
        [](const ShadowSupervisorVelocityPredictiveScoreGapSample & sample) {
          return sample.gvps_gap_h;
        },
        shadow_supervisor_velocity_predictive_score_long_window_sec_);
    velocity_residual_gap_mean_5s_mps =
      mean_recent(
        [](const ShadowSupervisorVelocityPredictiveScoreGapSample & sample) {
          return sample.velocity_residual_gap_h_mps;
        },
        shadow_supervisor_velocity_predictive_score_short_window_sec_);
    gvps_shadow_win_frac_10s =
      win_frac_recent(shadow_supervisor_velocity_predictive_score_long_window_sec_);
    valid_recent_count_10s =
      count_recent(shadow_supervisor_velocity_predictive_score_long_window_sec_);

    const double main_velocity_gamma =
      have_reference ? main_sample.velocity_gamma_clipped :
        std::numeric_limits<double>::quiet_NaN();
    const double shadow_velocity_gamma = local_sample.velocity_gamma_clipped;
    const double velocity_gamma_clipped =
      std::max(
        std::isfinite(main_velocity_gamma) ? main_velocity_gamma : 1.0,
        std::isfinite(shadow_velocity_gamma) ? shadow_velocity_gamma : 1.0);
    const double main_velocity_observation_score =
      have_reference ? main_sample.velocity_observation_score :
        std::numeric_limits<double>::quiet_NaN();
    const double shadow_velocity_observation_score =
      local_sample.velocity_observation_score;
    const double velocity_observation_score =
      std::max(
        std::isfinite(main_velocity_observation_score) ?
          main_velocity_observation_score : 0.0,
        std::isfinite(shadow_velocity_observation_score) ?
          shadow_velocity_observation_score : 0.0);
    bool velocity_observation_guard = false;
    std::string velocity_observation_reason = "clear";
    if (std::isfinite(main_velocity_gamma) && main_velocity_gamma > 1.05) {
      velocity_observation_guard = true;
      velocity_observation_reason = "main_velocity_gamma_guard";
    } else if (std::isfinite(shadow_velocity_gamma) && shadow_velocity_gamma > 1.05) {
      velocity_observation_guard = true;
      velocity_observation_reason = "shadow_velocity_gamma_guard";
    } else if (std::isfinite(main_velocity_observation_score) &&
               main_velocity_observation_score > 0.50) {
      velocity_observation_guard = true;
      velocity_observation_reason = "main_velocity_observation_score_guard";
    } else if (std::isfinite(shadow_velocity_observation_score) &&
               shadow_velocity_observation_score > 0.50) {
      velocity_observation_guard = true;
      velocity_observation_reason = "shadow_velocity_observation_score_guard";
    }

    const double main_position_gamma =
      have_reference ? main_sample.position_gamma_clipped :
        std::numeric_limits<double>::quiet_NaN();
    const double shadow_position_gamma = local_sample.position_gamma_clipped;
    const double main_position_observation_score =
      have_reference ? main_sample.position_observation_score :
        std::numeric_limits<double>::quiet_NaN();
    const double shadow_position_observation_score =
      local_sample.position_observation_score;
    bool position_observation_guard = false;
    std::string position_observation_reason = "clear";
    if (std::isfinite(main_position_gamma) && main_position_gamma > 1.05) {
      position_observation_guard = true;
      position_observation_reason = "main_position_gamma_guard";
    } else if (std::isfinite(shadow_position_gamma) && shadow_position_gamma > 1.05) {
      position_observation_guard = true;
      position_observation_reason = "shadow_position_gamma_guard";
    } else if (std::isfinite(main_position_observation_score) &&
               main_position_observation_score > 0.50) {
      position_observation_guard = true;
      position_observation_reason = "main_position_observation_score_guard";
    } else if (std::isfinite(shadow_position_observation_score) &&
               shadow_position_observation_score > 0.50) {
      position_observation_guard = true;
      position_observation_reason = "shadow_position_observation_score_guard";
    }

    const double time_since_restore_sec =
      (shadow_supervisor_have_restore_ &&
       std::isfinite(shadow_supervisor_last_restore_core_time_sec_) &&
       std::isfinite(local_sample.core_time_sec))
        ? local_sample.core_time_sec - shadow_supervisor_last_restore_core_time_sec_
        : std::numeric_limits<double>::quiet_NaN();

    shadow_supervisor_velocity_predictive_score_debug_csv_
      << 1 << ','
      << (++shadow_supervisor_velocity_predictive_score_debug_sequence_) << ','
      << ros_now_sec << ','
      << local_sample.gnss_stamp_sec << ','
      << csvSafe_(shadow_supervisor_velocity_predictive_score_source_id_) << ','
      << (shadow_supervisor_have_restore_ ? 1 : 0) << ','
      << time_since_restore_sec << ','
      << (prediction_valid ? 1 : 0) << ','
      << csvSafe_(prediction_reason) << ','
      << main_sample_age_sec << ','
      << shadow_sample_age_sec << ','
      << local_sample.raw_vn_mps << ','
      << local_sample.raw_ve_mps << ','
      << local_sample.raw_vd_mps << ','
      << local_sample.raw_speed_accuracy_mps << ','
      << local_sample.raw_vertical_speed_accuracy_mps << ','
      << main_sample.prior_vn_mps << ','
      << main_sample.prior_ve_mps << ','
      << main_sample.prior_vd_mps << ','
      << local_sample.prior_vn_mps << ','
      << local_sample.prior_ve_mps << ','
      << local_sample.prior_vd_mps << ','
      << main_sample.residual_n_mps << ','
      << main_sample.residual_e_mps << ','
      << main_sample.residual_d_mps << ','
      << main_sample.residual_h_mps << ','
      << local_sample.residual_n_mps << ','
      << local_sample.residual_e_mps << ','
      << local_sample.residual_d_mps << ','
      << local_sample.residual_h_mps << ','
      << main_sample.eval_sv_nn_m2ps2 << ','
      << main_sample.eval_sv_ne_m2ps2 << ','
      << main_sample.eval_sv_ee_m2ps2 << ','
      << local_sample.eval_sv_nn_m2ps2 << ','
      << local_sample.eval_sv_ne_m2ps2 << ','
      << local_sample.eval_sv_ee_m2ps2 << ','
      << main_sample.gvps_h << ','
      << local_sample.gvps_h << ','
      << gvps_gap_h << ','
      << velocity_residual_gap_h_mps << ','
      << gvps_gap_mean_3s << ','
      << gvps_gap_mean_5s << ','
      << gvps_gap_mean_10s << ','
      << gvps_shadow_win_frac_10s << ','
      << velocity_residual_gap_mean_5s_mps << ','
      << valid_recent_count_10s << ','
      << (velocity_observation_guard ? 1 : 0) << ','
      << csvSafe_(velocity_observation_reason) << ','
      << velocity_gamma_clipped << ','
      << velocity_observation_score << ','
      << (position_observation_guard ? 1 : 0) << ','
      << csvSafe_(position_observation_reason) << ','
      << "main_only" << ','
      << main_sample.gnss_stamp_sec << ','
      << local_sample.gnss_stamp_sec << ','
      << gnss_stamp_gap_sec << '\n';

    ++shadow_supervisor_velocity_predictive_score_debug_rows_since_flush_;
    if (shadow_supervisor_velocity_predictive_score_debug_rows_since_flush_ >=
        shadow_supervisor_velocity_predictive_score_debug_flush_interval_) {
      shadow_supervisor_velocity_predictive_score_debug_csv_.flush();
      shadow_supervisor_velocity_predictive_score_debug_rows_since_flush_ = 0;
    }
  }

  void flushShadowSupervisorVelocityPredictiveScorePending_(double ros_now_sec)
  {
    if (!shadow_supervisor_velocity_predictive_score_debug_enable_ ||
        !shadow_supervisor_velocity_predictive_score_debug_csv_.is_open()) {
      return;
    }
    pruneShadowSupervisorVelocityPredictiveScoreReferenceHistory_(ros_now_sec);
    const double retention_sec =
      shadowSupervisorVelocityPredictiveScoreRetentionSec_();
    for (auto it =
           shadow_supervisor_velocity_predictive_score_pending_local_samples_.begin();
         it != shadow_supervisor_velocity_predictive_score_pending_local_samples_.end(); ) {
      const ShadowSupervisorVelocityPredictiveScoreSample * reference =
        findShadowSupervisorVelocityPredictiveScoreReference_(*it);
      const double shadow_sample_age_sec =
        it->valid ? ros_now_sec - it->publish_ros_time_sec :
          std::numeric_limits<double>::quiet_NaN();
      const bool expired =
        !it->valid ||
        (std::isfinite(shadow_sample_age_sec) &&
         shadow_sample_age_sec > retention_sec);
      if (reference != nullptr || expired) {
        writeShadowSupervisorVelocityPredictiveScoreDebugRow_(
          ros_now_sec, *it, reference);
        it =
          shadow_supervisor_velocity_predictive_score_pending_local_samples_.erase(it);
      } else {
        ++it;
      }
    }
    while (shadow_supervisor_velocity_predictive_score_pending_local_samples_.size() > 64) {
      writeShadowSupervisorVelocityPredictiveScoreDebugRow_(
        ros_now_sec,
        shadow_supervisor_velocity_predictive_score_pending_local_samples_.front(),
        nullptr);
      shadow_supervisor_velocity_predictive_score_pending_local_samples_.pop_front();
    }
  }

  void maybeHandleShadowSupervisorVelocityPredictiveScore_(
    double gnss_stamp_sec,
    double raw_vn_mps,
    double raw_ve_mps,
    double raw_vd_mps,
    double raw_speed_accuracy_mps,
    double raw_vertical_speed_accuracy_mps)
  {
    if (!shadow_supervisor_velocity_predictive_score_publish_enable_ &&
        !shadow_supervisor_velocity_predictive_score_debug_enable_) {
      return;
    }

    const double ros_now_sec = now().seconds();
    const ShadowSupervisorVelocityPredictiveScoreSample local_sample =
      makeShadowSupervisorVelocityPredictiveScoreSample_(
        ros_now_sec,
        gnss_stamp_sec,
        raw_vn_mps,
        raw_ve_mps,
        raw_vd_mps,
        raw_speed_accuracy_mps,
        raw_vertical_speed_accuracy_mps);

    if (shadow_supervisor_velocity_predictive_score_publish_enable_ &&
        shadow_supervisor_velocity_predictive_score_pub_ &&
        local_sample.valid) {
      shadow_supervisor_velocity_predictive_score_pub_->publish(
        encodeShadowSupervisorVelocityPredictiveScore_(local_sample));
    }

    if (shadow_supervisor_velocity_predictive_score_debug_enable_ &&
        shadow_supervisor_velocity_predictive_score_debug_csv_.is_open()) {
      shadow_supervisor_velocity_predictive_score_pending_local_samples_.push_back(
        local_sample);
      flushShadowSupervisorVelocityPredictiveScorePending_(ros_now_sec);
    }
  }

  void configureShadowSupervisorKinematicPredictiveScore_()
  {
    if (!shadow_supervisor_kinematic_predictive_score_publish_enable_ &&
        !shadow_supervisor_kinematic_predictive_score_subscribe_enable_ &&
        !shadow_supervisor_kinematic_predictive_score_debug_enable_) {
      return;
    }

    if (shadow_supervisor_kinematic_predictive_score_publish_enable_) {
      shadow_supervisor_kinematic_predictive_score_pub_ =
        this->create_publisher<std_msgs::msg::Float64MultiArray>(
          shadow_supervisor_kinematic_predictive_score_topic_,
          rclcpp::QoS(10).reliable().durability_volatile());
    }
    if (shadow_supervisor_kinematic_predictive_score_subscribe_enable_) {
      shadow_supervisor_kinematic_predictive_score_sub_ =
        this->create_subscription<std_msgs::msg::Float64MultiArray>(
          shadow_supervisor_kinematic_predictive_score_topic_,
          rclcpp::QoS(10).reliable().durability_volatile(),
          [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
            handleShadowSupervisorKinematicPredictiveScore_(msg);
          });
    }

    if (shadow_supervisor_kinematic_predictive_score_debug_enable_ &&
        !shadow_supervisor_kinematic_predictive_score_debug_csv_path_.empty()) {
      try {
        const std::filesystem::path csv_path(
          shadow_supervisor_kinematic_predictive_score_debug_csv_path_);
        if (csv_path.has_parent_path()) {
          std::filesystem::create_directories(csv_path.parent_path());
        }
        shadow_supervisor_kinematic_predictive_score_debug_csv_.open(
          csv_path, std::ios::out | std::ios::trunc);
      } catch (const std::exception & e) {
        RCLCPP_WARN(
          get_logger(),
          "Failed to open shadow supervisor kinematic predictive-score CSV %s: %s",
          shadow_supervisor_kinematic_predictive_score_debug_csv_path_.c_str(),
          e.what());
      }
      if (shadow_supervisor_kinematic_predictive_score_debug_csv_.is_open()) {
        shadow_supervisor_kinematic_predictive_score_debug_csv_
          << "schema_version,sequence,ros_time_sec,gnss_stamp_sec,gnss_source_id,"
          << "restore_applied,time_since_restore_sec,prediction_valid,prediction_reason,"
          << "main_sample_age_sec,shadow_sample_age_sec,raw_position_n_m,"
          << "raw_position_e_m,raw_position_u_m,raw_velocity_n_mps,"
          << "raw_velocity_e_mps,raw_velocity_d_mps,raw_position_accuracy_h_m,"
          << "raw_position_accuracy_u_m,raw_speed_accuracy_mps,"
          << "raw_vertical_speed_accuracy_mps,main_prior_pn_m,main_prior_pe_m,"
          << "main_prior_pu_m,main_prior_vn_mps,main_prior_ve_mps,"
          << "main_prior_vd_mps,shadow_prior_pn_m,shadow_prior_pe_m,"
          << "shadow_prior_pu_m,shadow_prior_vn_mps,shadow_prior_ve_mps,"
          << "shadow_prior_vd_mps,main_gpps_h,shadow_gpps_h,gpps_gap_h,"
          << "main_gvps_h,shadow_gvps_h,gvps_gap_h,raw_delta_pn_2s_m,"
          << "raw_delta_pe_2s_m,main_delta_consistency_2s_m,"
          << "shadow_delta_consistency_2s_m,gkps_delta_gap_2s_m,"
          << "raw_delta_pn_5s_m,raw_delta_pe_5s_m,"
          << "main_delta_consistency_5s_m,shadow_delta_consistency_5s_m,"
          << "gkps_delta_gap_5s_m,raw_delta_pn_10s_m,raw_delta_pe_10s_m,"
          << "main_delta_consistency_10s_m,shadow_delta_consistency_10s_m,"
          << "gkps_delta_gap_10s_m,gkps_composite_gap,gkps_gap_mean_10s,"
          << "gkps_gap_median_10s,gkps_gap_trimmed_mean_10s,"
          << "gkps_shadow_win_frac_10s,gkps_sign_stability_streak,"
          << "gkps_tail_contribution_frac_10s,valid_recent_count_10s,"
          << "position_observation_guard,position_observation_reason,"
          << "velocity_observation_guard,velocity_observation_reason,"
          << "kinematic_observation_guard,kinematic_observation_reason,"
          << "output_policy\n";
        shadow_supervisor_kinematic_predictive_score_debug_csv_.flush();
      } else {
        RCLCPP_WARN(
          get_logger(),
          "Failed to open shadow supervisor kinematic predictive-score CSV: %s",
          shadow_supervisor_kinematic_predictive_score_debug_csv_path_.c_str());
      }
    }

    RCLCPP_INFO(
      get_logger(),
      "Shadow supervisor kinematic predictive score: publish=%s subscribe=%s "
      "debug=%s topic=%s csv=%s max_age=%.3f stamp_tolerance=%.3f",
      shadow_supervisor_kinematic_predictive_score_publish_enable_ ? "true" : "false",
      shadow_supervisor_kinematic_predictive_score_subscribe_enable_ ? "true" : "false",
      shadow_supervisor_kinematic_predictive_score_debug_enable_ ? "true" : "false",
      shadow_supervisor_kinematic_predictive_score_topic_.c_str(),
      shadow_supervisor_kinematic_predictive_score_debug_csv_path_.empty()
        ? "<disabled>"
        : shadow_supervisor_kinematic_predictive_score_debug_csv_path_.c_str(),
      shadow_supervisor_kinematic_predictive_score_max_age_sec_,
      shadow_supervisor_kinematic_predictive_score_stamp_tolerance_sec_);
  }

  const ShadowSupervisorKinematicPredictiveScoreSample *
  findShadowSupervisorKinematicPredictiveScoreHistory_(
    const std::deque<ShadowSupervisorKinematicPredictiveScoreSample> & history,
    double gnss_stamp_sec,
    double horizon_sec) const
  {
    if (!std::isfinite(gnss_stamp_sec) ||
        !std::isfinite(horizon_sec) ||
        horizon_sec <= 0.0) {
      return nullptr;
    }
    const ShadowSupervisorKinematicPredictiveScoreSample * best = nullptr;
    double best_gap = std::numeric_limits<double>::infinity();
    const double tolerance = std::max(0.25, std::min(1.0, horizon_sec * 0.25));
    for (const auto & sample : history) {
      if (!sample.valid || !std::isfinite(sample.gnss_stamp_sec)) {
        continue;
      }
      const double age_sec = gnss_stamp_sec - sample.gnss_stamp_sec;
      const double gap = std::abs(age_sec - horizon_sec);
      if (!std::isfinite(age_sec) || age_sec <= 0.0 || gap > tolerance) {
        continue;
      }
      if (gap < best_gap) {
        best_gap = gap;
        best = &sample;
      }
    }
    return best;
  }

  void fillShadowSupervisorKinematicDelta_(
    ShadowSupervisorKinematicPredictiveScoreSample * sample,
    double horizon_sec,
    double * raw_delta_pn_m,
    double * raw_delta_pe_m,
    double * delta_consistency_m) const
  {
    if (sample == nullptr ||
        raw_delta_pn_m == nullptr ||
        raw_delta_pe_m == nullptr ||
        delta_consistency_m == nullptr) {
      return;
    }
    const auto * previous =
      findShadowSupervisorKinematicPredictiveScoreHistory_(
        shadow_supervisor_kinematic_predictive_score_local_history_,
        sample->gnss_stamp_sec,
        horizon_sec);
    if (previous == nullptr) {
      return;
    }
    const double raw_delta_n = sample->raw_pn_m - previous->raw_pn_m;
    const double raw_delta_e = sample->raw_pe_m - previous->raw_pe_m;
    const double hyp_delta_n = sample->prior_pn_m - previous->prior_pn_m;
    const double hyp_delta_e = sample->prior_pe_m - previous->prior_pe_m;
    const double residual_n = raw_delta_n - hyp_delta_n;
    const double residual_e = raw_delta_e - hyp_delta_e;
    if (!std::isfinite(raw_delta_n) ||
        !std::isfinite(raw_delta_e) ||
        !std::isfinite(residual_n) ||
        !std::isfinite(residual_e)) {
      return;
    }
    *raw_delta_pn_m = raw_delta_n;
    *raw_delta_pe_m = raw_delta_e;
    *delta_consistency_m = std::hypot(residual_n, residual_e);
  }

  ShadowSupervisorKinematicPredictiveScoreSample
  makeShadowSupervisorKinematicPredictiveScoreSample_(
    double publish_ros_time_sec,
    double gnss_stamp_sec,
    double measurement_latitude_deg,
    double measurement_longitude_deg,
    double measurement_altitude_m,
    double raw_position_accuracy_h_m,
    double raw_position_accuracy_u_m,
    double raw_vn_mps,
    double raw_ve_mps,
    double raw_vd_mps,
    double raw_speed_accuracy_mps,
    double raw_vertical_speed_accuracy_mps)
  {
    ShadowSupervisorKinematicPredictiveScoreSample sample;
    sample.publish_ros_time_sec = publish_ros_time_sec;
    sample.gnss_stamp_sec = gnss_stamp_sec;
    sample.core_time_sec = last_core_time_;
    sample.sequence =
      static_cast<double>(++shadow_supervisor_kinematic_predictive_score_local_sequence_);
    sample.raw_vn_mps = raw_vn_mps;
    sample.raw_ve_mps = raw_ve_mps;
    sample.raw_vd_mps = raw_vd_mps;
    sample.raw_position_accuracy_h_m = raw_position_accuracy_h_m;
    sample.raw_position_accuracy_u_m = raw_position_accuracy_u_m;
    sample.raw_speed_accuracy_mps = raw_speed_accuracy_mps;
    sample.raw_vertical_speed_accuracy_mps = raw_vertical_speed_accuracy_mps;

    if (!core_ || !core_initialized_ || !have_origin_) {
      return sample;
    }
    if (!std::isfinite(measurement_latitude_deg) ||
        !std::isfinite(measurement_longitude_deg) ||
        !std::isfinite(measurement_altitude_m) ||
        !std::isfinite(raw_vn_mps) ||
        !std::isfinite(raw_ve_mps) ||
        !std::isfinite(raw_vd_mps)) {
      return sample;
    }

    const kfcore::State state = core_->current();
    if (!std::isfinite(state.lat_deg) ||
        !std::isfinite(state.lon_deg) ||
        !std::isfinite(state.h_m) ||
        !std::isfinite(state.vN) ||
        !std::isfinite(state.vE) ||
        !std::isfinite(state.vD)) {
      return sample;
    }

    double core_x = std::numeric_limits<double>::quiet_NaN();
    double core_y = std::numeric_limits<double>::quiet_NaN();
    double core_z = std::numeric_limits<double>::quiet_NaN();
    double gnss_x = std::numeric_limits<double>::quiet_NaN();
    double gnss_y = std::numeric_limits<double>::quiet_NaN();
    double gnss_z = std::numeric_limits<double>::quiet_NaN();
    geo::llh_to_ecef(
      state.lat_deg * M_PI / 180.0,
      state.lon_deg * M_PI / 180.0,
      state.h_m,
      core_x,
      core_y,
      core_z);
    geo::llh_to_ecef(
      measurement_latitude_deg * M_PI / 180.0,
      measurement_longitude_deg * M_PI / 180.0,
      measurement_altitude_m,
      gnss_x,
      gnss_y,
      gnss_z);
    const Eigen::Vector3d enu_core =
      geo::ecef_to_enu({core_x, core_y, core_z}, origin_ecef_, origin_lat_, origin_lon_);
    const Eigen::Vector3d enu_gnss =
      geo::ecef_to_enu({gnss_x, gnss_y, gnss_z}, origin_ecef_, origin_lat_, origin_lon_);
    if (!enu_core.allFinite() || !enu_gnss.allFinite()) {
      return sample;
    }

    const Eigen::MatrixXd covariance = core_->covariance();
    if (covariance.rows() < 6 || covariance.cols() < 6 ||
        !std::isfinite(covariance(0, 0)) ||
        !std::isfinite(covariance(0, 1)) ||
        !std::isfinite(covariance(1, 0)) ||
        !std::isfinite(covariance(1, 1)) ||
        !std::isfinite(covariance(3, 3)) ||
        !std::isfinite(covariance(3, 4)) ||
        !std::isfinite(covariance(4, 3)) ||
        !std::isfinite(covariance(4, 4))) {
      return sample;
    }

    sample.raw_pn_m = enu_gnss.y();
    sample.raw_pe_m = enu_gnss.x();
    sample.raw_pu_m = enu_gnss.z();
    sample.prior_pn_m = enu_core.y();
    sample.prior_pe_m = enu_core.x();
    sample.prior_pu_m = enu_core.z();
    sample.prior_vn_mps = state.vN;
    sample.prior_ve_mps = state.vE;
    sample.prior_vd_mps = state.vD;
    sample.residual_pn_m = sample.raw_pn_m - sample.prior_pn_m;
    sample.residual_pe_m = sample.raw_pe_m - sample.prior_pe_m;
    sample.residual_ph_m = std::hypot(sample.residual_pn_m, sample.residual_pe_m);
    sample.residual_vn_mps = raw_vn_mps - state.vN;
    sample.residual_ve_mps = raw_ve_mps - state.vE;
    sample.residual_vd_mps = raw_vd_mps - state.vD;
    sample.residual_vh_mps =
      std::hypot(sample.residual_vn_mps, sample.residual_ve_mps);
    if (!std::isfinite(sample.residual_ph_m) ||
        !std::isfinite(sample.residual_vh_mps) ||
        !std::isfinite(sample.residual_vd_mps)) {
      return sample;
    }

    const double raw_eval_pos_h =
      (std::isfinite(raw_position_accuracy_h_m) && raw_position_accuracy_h_m > 0.0)
        ? raw_position_accuracy_h_m
        : 0.0;
    const double raw_eval_vel_h =
      (std::isfinite(raw_speed_accuracy_mps) && raw_speed_accuracy_mps > 0.0)
        ? raw_speed_accuracy_mps
        : 0.0;
    const double eval_pos_h =
      std::max(shadow_supervisor_kinematic_predictive_score_eval_std_h_m_,
               raw_eval_pos_h);
    const double eval_vel_h =
      std::max(shadow_supervisor_kinematic_predictive_score_eval_std_h_mps_,
               raw_eval_vel_h);
    const double eval_pos_var_h = eval_pos_h * eval_pos_h;
    const double eval_vel_var_h = eval_vel_h * eval_vel_h;
    sample.eval_sp_nn_m2 = covariance(0, 0) + eval_pos_var_h;
    sample.eval_sp_ne_m2 = 0.5 * (covariance(0, 1) + covariance(1, 0));
    sample.eval_sp_ee_m2 = covariance(1, 1) + eval_pos_var_h;
    sample.eval_sv_nn_m2ps2 = covariance(3, 3) + eval_vel_var_h;
    sample.eval_sv_ne_m2ps2 = 0.5 * (covariance(3, 4) + covariance(4, 3));
    sample.eval_sv_ee_m2ps2 = covariance(4, 4) + eval_vel_var_h;

    const double det_p =
      sample.eval_sp_nn_m2 * sample.eval_sp_ee_m2 -
      sample.eval_sp_ne_m2 * sample.eval_sp_ne_m2;
    const double det_v =
      sample.eval_sv_nn_m2ps2 * sample.eval_sv_ee_m2ps2 -
      sample.eval_sv_ne_m2ps2 * sample.eval_sv_ne_m2ps2;
    if (!std::isfinite(det_p) || det_p <= 1.0e-12 ||
        !std::isfinite(det_v) || det_v <= 1.0e-12 ||
        sample.eval_sp_nn_m2 <= 0.0 ||
        sample.eval_sp_ee_m2 <= 0.0 ||
        sample.eval_sv_nn_m2ps2 <= 0.0 ||
        sample.eval_sv_ee_m2ps2 <= 0.0) {
      return sample;
    }

    const double quad_p =
      (sample.eval_sp_ee_m2 * sample.residual_pn_m * sample.residual_pn_m -
       2.0 * sample.eval_sp_ne_m2 * sample.residual_pn_m * sample.residual_pe_m +
       sample.eval_sp_nn_m2 * sample.residual_pe_m * sample.residual_pe_m) / det_p;
    const double quad_v =
      (sample.eval_sv_ee_m2ps2 * sample.residual_vn_mps * sample.residual_vn_mps -
       2.0 * sample.eval_sv_ne_m2ps2 *
         sample.residual_vn_mps * sample.residual_ve_mps +
       sample.eval_sv_nn_m2ps2 * sample.residual_ve_mps * sample.residual_ve_mps) /
      det_v;
    sample.gpps_h = quad_p + std::log(det_p);
    sample.gvps_h = quad_v + std::log(det_v);

    fillShadowSupervisorKinematicDelta_(
      &sample,
      2.0,
      &sample.raw_delta_pn_2s_m,
      &sample.raw_delta_pe_2s_m,
      &sample.delta_consistency_2s_m);
    fillShadowSupervisorKinematicDelta_(
      &sample,
      5.0,
      &sample.raw_delta_pn_5s_m,
      &sample.raw_delta_pe_5s_m,
      &sample.delta_consistency_5s_m);
    fillShadowSupervisorKinematicDelta_(
      &sample,
      10.0,
      &sample.raw_delta_pn_10s_m,
      &sample.raw_delta_pe_10s_m,
      &sample.delta_consistency_10s_m);

    const kfcore::ObservationDebug observation_debug = core_->lastObservationDebug();
    sample.position_gamma_clipped =
      observation_debug.gnss_position_adaptive_r.gamma_clipped;
    sample.position_observation_score =
      observation_debug.gnss_position_adaptive_r.observation_score;
    sample.velocity_gamma_clipped =
      observation_debug.gnss_velocity_adaptive_r.gamma_clipped;
    sample.velocity_observation_score =
      observation_debug.gnss_velocity_adaptive_r.observation_score;
    sample.valid =
      std::isfinite(sample.gpps_h) &&
      std::isfinite(sample.gvps_h);
    return sample;
  }

  void recordShadowSupervisorKinematicPredictiveScoreLocalHistory_(
    const ShadowSupervisorKinematicPredictiveScoreSample & sample,
    double ros_now_sec)
  {
    if (sample.valid) {
      shadow_supervisor_kinematic_predictive_score_local_history_.push_back(sample);
    }
    const double retention_sec =
      std::max(12.0, shadow_supervisor_kinematic_predictive_score_long_window_sec_ + 2.0);
    while (!shadow_supervisor_kinematic_predictive_score_local_history_.empty()) {
      const auto & front =
        shadow_supervisor_kinematic_predictive_score_local_history_.front();
      if (!std::isfinite(ros_now_sec) ||
          !std::isfinite(front.publish_ros_time_sec) ||
          (ros_now_sec - front.publish_ros_time_sec) <= retention_sec) {
        break;
      }
      shadow_supervisor_kinematic_predictive_score_local_history_.pop_front();
    }
    while (shadow_supervisor_kinematic_predictive_score_local_history_.size() > 512) {
      shadow_supervisor_kinematic_predictive_score_local_history_.pop_front();
    }
  }

  std_msgs::msg::Float64MultiArray encodeShadowSupervisorKinematicPredictiveScore_(
    const ShadowSupervisorKinematicPredictiveScoreSample & sample) const
  {
    std_msgs::msg::Float64MultiArray msg;
    auto & data = msg.data;
    data.reserve(49);
    data.push_back(1.0);
    data.push_back(sample.sequence);
    data.push_back(sample.publish_ros_time_sec);
    data.push_back(sample.gnss_stamp_sec);
    data.push_back(sample.core_time_sec);
    data.push_back(sample.raw_pn_m);
    data.push_back(sample.raw_pe_m);
    data.push_back(sample.raw_pu_m);
    data.push_back(sample.raw_vn_mps);
    data.push_back(sample.raw_ve_mps);
    data.push_back(sample.raw_vd_mps);
    data.push_back(sample.raw_position_accuracy_h_m);
    data.push_back(sample.raw_position_accuracy_u_m);
    data.push_back(sample.raw_speed_accuracy_mps);
    data.push_back(sample.raw_vertical_speed_accuracy_mps);
    data.push_back(sample.prior_pn_m);
    data.push_back(sample.prior_pe_m);
    data.push_back(sample.prior_pu_m);
    data.push_back(sample.prior_vn_mps);
    data.push_back(sample.prior_ve_mps);
    data.push_back(sample.prior_vd_mps);
    data.push_back(sample.residual_pn_m);
    data.push_back(sample.residual_pe_m);
    data.push_back(sample.residual_ph_m);
    data.push_back(sample.residual_vn_mps);
    data.push_back(sample.residual_ve_mps);
    data.push_back(sample.residual_vd_mps);
    data.push_back(sample.residual_vh_mps);
    data.push_back(sample.eval_sp_nn_m2);
    data.push_back(sample.eval_sp_ne_m2);
    data.push_back(sample.eval_sp_ee_m2);
    data.push_back(sample.eval_sv_nn_m2ps2);
    data.push_back(sample.eval_sv_ne_m2ps2);
    data.push_back(sample.eval_sv_ee_m2ps2);
    data.push_back(sample.gpps_h);
    data.push_back(sample.gvps_h);
    data.push_back(sample.raw_delta_pn_2s_m);
    data.push_back(sample.raw_delta_pe_2s_m);
    data.push_back(sample.delta_consistency_2s_m);
    data.push_back(sample.raw_delta_pn_5s_m);
    data.push_back(sample.raw_delta_pe_5s_m);
    data.push_back(sample.delta_consistency_5s_m);
    data.push_back(sample.raw_delta_pn_10s_m);
    data.push_back(sample.raw_delta_pe_10s_m);
    data.push_back(sample.delta_consistency_10s_m);
    data.push_back(sample.position_gamma_clipped);
    data.push_back(sample.position_observation_score);
    data.push_back(sample.velocity_gamma_clipped);
    data.push_back(sample.velocity_observation_score);
    return msg;
  }

  bool decodeShadowSupervisorKinematicPredictiveScore_(
    const std::vector<double> & data,
    ShadowSupervisorKinematicPredictiveScoreSample * sample) const
  {
    if (sample == nullptr || data.size() < 49 ||
        !std::isfinite(data[0]) || std::abs(data[0] - 1.0) > 1.0e-6) {
      return false;
    }
    sample->sequence = data[1];
    sample->publish_ros_time_sec = data[2];
    sample->gnss_stamp_sec = data[3];
    sample->core_time_sec = data[4];
    sample->raw_pn_m = data[5];
    sample->raw_pe_m = data[6];
    sample->raw_pu_m = data[7];
    sample->raw_vn_mps = data[8];
    sample->raw_ve_mps = data[9];
    sample->raw_vd_mps = data[10];
    sample->raw_position_accuracy_h_m = data[11];
    sample->raw_position_accuracy_u_m = data[12];
    sample->raw_speed_accuracy_mps = data[13];
    sample->raw_vertical_speed_accuracy_mps = data[14];
    sample->prior_pn_m = data[15];
    sample->prior_pe_m = data[16];
    sample->prior_pu_m = data[17];
    sample->prior_vn_mps = data[18];
    sample->prior_ve_mps = data[19];
    sample->prior_vd_mps = data[20];
    sample->residual_pn_m = data[21];
    sample->residual_pe_m = data[22];
    sample->residual_ph_m = data[23];
    sample->residual_vn_mps = data[24];
    sample->residual_ve_mps = data[25];
    sample->residual_vd_mps = data[26];
    sample->residual_vh_mps = data[27];
    sample->eval_sp_nn_m2 = data[28];
    sample->eval_sp_ne_m2 = data[29];
    sample->eval_sp_ee_m2 = data[30];
    sample->eval_sv_nn_m2ps2 = data[31];
    sample->eval_sv_ne_m2ps2 = data[32];
    sample->eval_sv_ee_m2ps2 = data[33];
    sample->gpps_h = data[34];
    sample->gvps_h = data[35];
    sample->raw_delta_pn_2s_m = data[36];
    sample->raw_delta_pe_2s_m = data[37];
    sample->delta_consistency_2s_m = data[38];
    sample->raw_delta_pn_5s_m = data[39];
    sample->raw_delta_pe_5s_m = data[40];
    sample->delta_consistency_5s_m = data[41];
    sample->raw_delta_pn_10s_m = data[42];
    sample->raw_delta_pe_10s_m = data[43];
    sample->delta_consistency_10s_m = data[44];
    sample->position_gamma_clipped = data[45];
    sample->position_observation_score = data[46];
    sample->velocity_gamma_clipped = data[47];
    sample->velocity_observation_score = data[48];
    sample->valid =
      std::isfinite(sample->publish_ros_time_sec) &&
      std::isfinite(sample->gnss_stamp_sec) &&
      std::isfinite(sample->gpps_h) &&
      std::isfinite(sample->gvps_h);
    return true;
  }

  void handleShadowSupervisorKinematicPredictiveScore_(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (!shadow_supervisor_kinematic_predictive_score_subscribe_enable_ || !msg) {
      return;
    }
    ShadowSupervisorKinematicPredictiveScoreSample sample;
    if (!decodeShadowSupervisorKinematicPredictiveScore_(msg->data, &sample) ||
        !sample.valid) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignored malformed shadow supervisor kinematic predictive-score message.");
      return;
    }
    shadow_supervisor_latest_kinematic_predictive_score_ = sample;
    if (shadow_supervisor_kinematic_predictive_score_debug_enable_) {
      const double ros_now_sec = now().seconds();
      shadow_supervisor_kinematic_predictive_score_reference_history_.push_back(sample);
      flushShadowSupervisorKinematicPredictiveScorePending_(ros_now_sec);
    }
  }

  double shadowSupervisorKinematicPredictiveScoreRetentionSec_() const
  {
    if (std::isfinite(shadow_supervisor_kinematic_predictive_score_max_age_sec_) &&
        shadow_supervisor_kinematic_predictive_score_max_age_sec_ >= 0.0) {
      return shadow_supervisor_kinematic_predictive_score_max_age_sec_ + 1.0;
    }
    return 5.0;
  }

  void pruneShadowSupervisorKinematicPredictiveScoreReferenceHistory_(
    double ros_now_sec)
  {
    const double retention_sec = shadowSupervisorKinematicPredictiveScoreRetentionSec_();
    while (!shadow_supervisor_kinematic_predictive_score_reference_history_.empty()) {
      const auto & sample =
        shadow_supervisor_kinematic_predictive_score_reference_history_.front();
      if (!std::isfinite(ros_now_sec) ||
          !std::isfinite(sample.publish_ros_time_sec) ||
          (ros_now_sec - sample.publish_ros_time_sec) <= retention_sec) {
        break;
      }
      shadow_supervisor_kinematic_predictive_score_reference_history_.pop_front();
    }
    while (shadow_supervisor_kinematic_predictive_score_reference_history_.size() > 64) {
      shadow_supervisor_kinematic_predictive_score_reference_history_.pop_front();
    }
  }

  const ShadowSupervisorKinematicPredictiveScoreSample *
  findShadowSupervisorKinematicPredictiveScoreReference_(
    const ShadowSupervisorKinematicPredictiveScoreSample & local_sample) const
  {
    if (!local_sample.valid ||
        !std::isfinite(local_sample.gnss_stamp_sec)) {
      return nullptr;
    }
    const bool enforce_tolerance =
      std::isfinite(shadow_supervisor_kinematic_predictive_score_stamp_tolerance_sec_) &&
      shadow_supervisor_kinematic_predictive_score_stamp_tolerance_sec_ >= 0.0;
    const ShadowSupervisorKinematicPredictiveScoreSample * best_sample = nullptr;
    double best_gap_sec = std::numeric_limits<double>::infinity();
    for (const auto & sample :
         shadow_supervisor_kinematic_predictive_score_reference_history_) {
      if (!sample.valid || !std::isfinite(sample.gnss_stamp_sec)) {
        continue;
      }
      const double gap_sec =
        std::abs(sample.gnss_stamp_sec - local_sample.gnss_stamp_sec);
      if (!std::isfinite(gap_sec)) {
        continue;
      }
      if (enforce_tolerance &&
          gap_sec > shadow_supervisor_kinematic_predictive_score_stamp_tolerance_sec_) {
        continue;
      }
      if (gap_sec < best_gap_sec) {
        best_gap_sec = gap_sec;
        best_sample = &sample;
      }
    }
    return best_sample;
  }

  static double squashSigned_(double value)
  {
    if (!std::isfinite(value)) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    return value / (1.0 + std::abs(value));
  }

  void writeShadowSupervisorKinematicPredictiveScoreDebugRow_(
    double ros_now_sec,
    const ShadowSupervisorKinematicPredictiveScoreSample & local_sample,
    const ShadowSupervisorKinematicPredictiveScoreSample * main_sample_ptr)
  {
    if (!shadow_supervisor_kinematic_predictive_score_debug_enable_ ||
        !shadow_supervisor_kinematic_predictive_score_debug_csv_.is_open()) {
      return;
    }
    const ShadowSupervisorKinematicPredictiveScoreSample empty_main_sample;
    const bool have_reference =
      main_sample_ptr != nullptr && main_sample_ptr->valid;
    const ShadowSupervisorKinematicPredictiveScoreSample & main_sample =
      have_reference ? *main_sample_ptr : empty_main_sample;
    const double main_sample_age_sec =
      have_reference
        ? ros_now_sec - main_sample.publish_ros_time_sec
        : std::numeric_limits<double>::quiet_NaN();
    const double shadow_sample_age_sec =
      local_sample.valid ? ros_now_sec - local_sample.publish_ros_time_sec
                         : std::numeric_limits<double>::quiet_NaN();
    const double gnss_stamp_gap_sec =
      (have_reference &&
       std::isfinite(main_sample.gnss_stamp_sec) &&
       std::isfinite(local_sample.gnss_stamp_sec))
        ? std::abs(local_sample.gnss_stamp_sec - main_sample.gnss_stamp_sec)
        : std::numeric_limits<double>::quiet_NaN();
    const bool reference_fresh =
      have_reference &&
      std::isfinite(main_sample_age_sec) &&
      (shadow_supervisor_kinematic_predictive_score_max_age_sec_ < 0.0 ||
       main_sample_age_sec <= shadow_supervisor_kinematic_predictive_score_max_age_sec_);
    const bool shadow_fresh =
      local_sample.valid &&
      std::isfinite(shadow_sample_age_sec) &&
      (shadow_supervisor_kinematic_predictive_score_max_age_sec_ < 0.0 ||
       shadow_sample_age_sec <= shadow_supervisor_kinematic_predictive_score_max_age_sec_);
    const bool stamp_aligned =
      have_reference &&
      std::isfinite(gnss_stamp_gap_sec) &&
      (shadow_supervisor_kinematic_predictive_score_stamp_tolerance_sec_ < 0.0 ||
       gnss_stamp_gap_sec <=
         shadow_supervisor_kinematic_predictive_score_stamp_tolerance_sec_);

    bool prediction_valid = false;
    std::string prediction_reason = "valid";
    if (!local_sample.valid) {
      prediction_reason = "shadow_invalid";
    } else if (!shadow_fresh) {
      prediction_reason = "shadow_stale";
    } else if (!have_reference) {
      prediction_reason = "main_missing";
    } else if (!reference_fresh) {
      prediction_reason = "main_stale";
    } else if (!stamp_aligned) {
      prediction_reason = "stamp_mismatch";
    } else {
      prediction_valid = true;
    }

    double gpps_gap_h = std::numeric_limits<double>::quiet_NaN();
    double gvps_gap_h = std::numeric_limits<double>::quiet_NaN();
    double gkps_delta_gap_2s_m = std::numeric_limits<double>::quiet_NaN();
    double gkps_delta_gap_5s_m = std::numeric_limits<double>::quiet_NaN();
    double gkps_delta_gap_10s_m = std::numeric_limits<double>::quiet_NaN();
    double gkps_composite_gap = std::numeric_limits<double>::quiet_NaN();
    if (prediction_valid) {
      gpps_gap_h = local_sample.gpps_h - main_sample.gpps_h;
      gvps_gap_h = local_sample.gvps_h - main_sample.gvps_h;
      gkps_delta_gap_2s_m =
        local_sample.delta_consistency_2s_m - main_sample.delta_consistency_2s_m;
      gkps_delta_gap_5s_m =
        local_sample.delta_consistency_5s_m - main_sample.delta_consistency_5s_m;
      gkps_delta_gap_10s_m =
        local_sample.delta_consistency_10s_m - main_sample.delta_consistency_10s_m;
      if (std::isfinite(gpps_gap_h) &&
          std::isfinite(gvps_gap_h) &&
          std::isfinite(gkps_delta_gap_5s_m) &&
          std::isfinite(gkps_delta_gap_10s_m)) {
        gkps_composite_gap =
          0.25 * squashSigned_(gpps_gap_h) +
          0.25 * squashSigned_(gvps_gap_h) +
          0.25 * squashSigned_(gkps_delta_gap_5s_m) +
          0.25 * squashSigned_(gkps_delta_gap_10s_m);
        shadow_supervisor_kinematic_predictive_score_gap_history_.push_back(
          ShadowSupervisorKinematicPredictiveScoreGapSample{
            ros_now_sec,
            gkps_composite_gap,
            gkps_composite_gap < 0.0});
      }
    }

    const double history_window_sec =
      std::max(
        shadow_supervisor_kinematic_predictive_score_short_window_sec_,
        shadow_supervisor_kinematic_predictive_score_long_window_sec_);
    while (!shadow_supervisor_kinematic_predictive_score_gap_history_.empty() &&
           std::isfinite(history_window_sec) &&
           history_window_sec >= 0.0 &&
           (ros_now_sec -
            shadow_supervisor_kinematic_predictive_score_gap_history_.front().ros_time_sec) >
             history_window_sec + 1.0) {
      shadow_supervisor_kinematic_predictive_score_gap_history_.pop_front();
    }

    std::vector<double> recent_gaps;
    int valid_recent_count_10s = 0;
    int recent_wins = 0;
    for (auto it = shadow_supervisor_kinematic_predictive_score_gap_history_.rbegin();
         it != shadow_supervisor_kinematic_predictive_score_gap_history_.rend();
         ++it) {
      if ((ros_now_sec - it->ros_time_sec) >
          shadow_supervisor_kinematic_predictive_score_long_window_sec_) {
        break;
      }
      if (!std::isfinite(it->gkps_composite_gap)) {
        continue;
      }
      recent_gaps.push_back(it->gkps_composite_gap);
      ++valid_recent_count_10s;
      if (it->shadow_win) {
        ++recent_wins;
      }
    }
    double gkps_gap_mean_10s = std::numeric_limits<double>::quiet_NaN();
    double gkps_gap_median_10s = std::numeric_limits<double>::quiet_NaN();
    double gkps_gap_trimmed_mean_10s = std::numeric_limits<double>::quiet_NaN();
    double gkps_shadow_win_frac_10s = std::numeric_limits<double>::quiet_NaN();
    double gkps_tail_contribution_frac_10s = std::numeric_limits<double>::quiet_NaN();
    int gkps_sign_stability_streak = 0;
    if (!recent_gaps.empty()) {
      double sum = 0.0;
      for (const double value : recent_gaps) {
        sum += value;
      }
      gkps_gap_mean_10s = sum / static_cast<double>(recent_gaps.size());
      gkps_shadow_win_frac_10s =
        static_cast<double>(recent_wins) / static_cast<double>(recent_gaps.size());
      std::vector<double> sorted = recent_gaps;
      std::sort(sorted.begin(), sorted.end());
      const std::size_t mid = sorted.size() / 2;
      gkps_gap_median_10s =
        (sorted.size() % 2 == 0)
          ? 0.5 * (sorted[mid - 1] + sorted[mid])
          : sorted[mid];
      const std::size_t trim = sorted.size() >= 10 ? sorted.size() / 10 : 0;
      double trimmed_sum = 0.0;
      std::size_t trimmed_count = 0;
      for (std::size_t i = trim; i < sorted.size() - trim; ++i) {
        trimmed_sum += sorted[i];
        ++trimmed_count;
      }
      if (trimmed_count > 0) {
        gkps_gap_trimmed_mean_10s =
          trimmed_sum / static_cast<double>(trimmed_count);
      }
      const int latest_sign =
        recent_gaps.front() < 0.0 ? -1 : (recent_gaps.front() > 0.0 ? 1 : 0);
      if (latest_sign != 0) {
        for (const double value : recent_gaps) {
          const int sign = value < 0.0 ? -1 : (value > 0.0 ? 1 : 0);
          if (sign != latest_sign) {
            break;
          }
          ++gkps_sign_stability_streak;
        }
      }
      std::vector<double> abs_gaps;
      abs_gaps.reserve(recent_gaps.size());
      double abs_sum = 0.0;
      for (const double value : recent_gaps) {
        const double abs_value = std::abs(value);
        abs_sum += abs_value;
        abs_gaps.push_back(abs_value);
      }
      if (abs_sum > 0.0) {
        std::sort(abs_gaps.begin(), abs_gaps.end(), std::greater<double>());
        const std::size_t tail_count =
          std::max<std::size_t>(1, (abs_gaps.size() + 9) / 10);
        double tail_sum = 0.0;
        for (std::size_t i = 0; i < tail_count && i < abs_gaps.size(); ++i) {
          tail_sum += abs_gaps[i];
        }
        gkps_tail_contribution_frac_10s = tail_sum / abs_sum;
      }
    }

    const double main_position_gamma =
      have_reference ? main_sample.position_gamma_clipped :
        std::numeric_limits<double>::quiet_NaN();
    const double shadow_position_gamma = local_sample.position_gamma_clipped;
    const double main_position_observation_score =
      have_reference ? main_sample.position_observation_score :
        std::numeric_limits<double>::quiet_NaN();
    const double shadow_position_observation_score =
      local_sample.position_observation_score;
    bool position_observation_guard = false;
    std::string position_observation_reason = "clear";
    if (std::isfinite(main_position_gamma) && main_position_gamma > 1.05) {
      position_observation_guard = true;
      position_observation_reason = "main_position_gamma_guard";
    } else if (std::isfinite(shadow_position_gamma) && shadow_position_gamma > 1.05) {
      position_observation_guard = true;
      position_observation_reason = "shadow_position_gamma_guard";
    } else if (std::isfinite(main_position_observation_score) &&
               main_position_observation_score > 0.50) {
      position_observation_guard = true;
      position_observation_reason = "main_position_observation_score_guard";
    } else if (std::isfinite(shadow_position_observation_score) &&
               shadow_position_observation_score > 0.50) {
      position_observation_guard = true;
      position_observation_reason = "shadow_position_observation_score_guard";
    }

    const double main_velocity_gamma =
      have_reference ? main_sample.velocity_gamma_clipped :
        std::numeric_limits<double>::quiet_NaN();
    const double shadow_velocity_gamma = local_sample.velocity_gamma_clipped;
    const double main_velocity_observation_score =
      have_reference ? main_sample.velocity_observation_score :
        std::numeric_limits<double>::quiet_NaN();
    const double shadow_velocity_observation_score =
      local_sample.velocity_observation_score;
    bool velocity_observation_guard = false;
    std::string velocity_observation_reason = "clear";
    if (std::isfinite(main_velocity_gamma) && main_velocity_gamma > 1.05) {
      velocity_observation_guard = true;
      velocity_observation_reason = "main_velocity_gamma_guard";
    } else if (std::isfinite(shadow_velocity_gamma) && shadow_velocity_gamma > 1.05) {
      velocity_observation_guard = true;
      velocity_observation_reason = "shadow_velocity_gamma_guard";
    } else if (std::isfinite(main_velocity_observation_score) &&
               main_velocity_observation_score > 0.50) {
      velocity_observation_guard = true;
      velocity_observation_reason = "main_velocity_observation_score_guard";
    } else if (std::isfinite(shadow_velocity_observation_score) &&
               shadow_velocity_observation_score > 0.50) {
      velocity_observation_guard = true;
      velocity_observation_reason = "shadow_velocity_observation_score_guard";
    }

    bool kinematic_observation_guard = false;
    std::string kinematic_observation_reason = "clear";
    if (prediction_valid &&
        (!std::isfinite(gkps_delta_gap_5s_m) ||
         !std::isfinite(gkps_delta_gap_10s_m))) {
      kinematic_observation_guard = true;
      kinematic_observation_reason = "delta_history_missing";
    } else if (position_observation_guard || velocity_observation_guard) {
      kinematic_observation_guard = true;
      kinematic_observation_reason = "component_observation_guard";
    }

    const double time_since_restore_sec =
      (shadow_supervisor_have_restore_ &&
       std::isfinite(shadow_supervisor_last_restore_core_time_sec_) &&
       std::isfinite(local_sample.core_time_sec))
        ? local_sample.core_time_sec - shadow_supervisor_last_restore_core_time_sec_
        : std::numeric_limits<double>::quiet_NaN();

    shadow_supervisor_kinematic_predictive_score_debug_csv_
      << 1 << ','
      << (++shadow_supervisor_kinematic_predictive_score_debug_sequence_) << ','
      << ros_now_sec << ','
      << local_sample.gnss_stamp_sec << ','
      << csvSafe_(shadow_supervisor_kinematic_predictive_score_source_id_) << ','
      << (shadow_supervisor_have_restore_ ? 1 : 0) << ','
      << time_since_restore_sec << ','
      << (prediction_valid ? 1 : 0) << ','
      << csvSafe_(prediction_reason) << ','
      << main_sample_age_sec << ','
      << shadow_sample_age_sec << ','
      << local_sample.raw_pn_m << ','
      << local_sample.raw_pe_m << ','
      << local_sample.raw_pu_m << ','
      << local_sample.raw_vn_mps << ','
      << local_sample.raw_ve_mps << ','
      << local_sample.raw_vd_mps << ','
      << local_sample.raw_position_accuracy_h_m << ','
      << local_sample.raw_position_accuracy_u_m << ','
      << local_sample.raw_speed_accuracy_mps << ','
      << local_sample.raw_vertical_speed_accuracy_mps << ','
      << main_sample.prior_pn_m << ','
      << main_sample.prior_pe_m << ','
      << main_sample.prior_pu_m << ','
      << main_sample.prior_vn_mps << ','
      << main_sample.prior_ve_mps << ','
      << main_sample.prior_vd_mps << ','
      << local_sample.prior_pn_m << ','
      << local_sample.prior_pe_m << ','
      << local_sample.prior_pu_m << ','
      << local_sample.prior_vn_mps << ','
      << local_sample.prior_ve_mps << ','
      << local_sample.prior_vd_mps << ','
      << main_sample.gpps_h << ','
      << local_sample.gpps_h << ','
      << gpps_gap_h << ','
      << main_sample.gvps_h << ','
      << local_sample.gvps_h << ','
      << gvps_gap_h << ','
      << local_sample.raw_delta_pn_2s_m << ','
      << local_sample.raw_delta_pe_2s_m << ','
      << main_sample.delta_consistency_2s_m << ','
      << local_sample.delta_consistency_2s_m << ','
      << gkps_delta_gap_2s_m << ','
      << local_sample.raw_delta_pn_5s_m << ','
      << local_sample.raw_delta_pe_5s_m << ','
      << main_sample.delta_consistency_5s_m << ','
      << local_sample.delta_consistency_5s_m << ','
      << gkps_delta_gap_5s_m << ','
      << local_sample.raw_delta_pn_10s_m << ','
      << local_sample.raw_delta_pe_10s_m << ','
      << main_sample.delta_consistency_10s_m << ','
      << local_sample.delta_consistency_10s_m << ','
      << gkps_delta_gap_10s_m << ','
      << gkps_composite_gap << ','
      << gkps_gap_mean_10s << ','
      << gkps_gap_median_10s << ','
      << gkps_gap_trimmed_mean_10s << ','
      << gkps_shadow_win_frac_10s << ','
      << gkps_sign_stability_streak << ','
      << gkps_tail_contribution_frac_10s << ','
      << valid_recent_count_10s << ','
      << (position_observation_guard ? 1 : 0) << ','
      << csvSafe_(position_observation_reason) << ','
      << (velocity_observation_guard ? 1 : 0) << ','
      << csvSafe_(velocity_observation_reason) << ','
      << (kinematic_observation_guard ? 1 : 0) << ','
      << csvSafe_(kinematic_observation_reason) << ','
      << "main_only\n";

    ++shadow_supervisor_kinematic_predictive_score_debug_rows_since_flush_;
    if (shadow_supervisor_kinematic_predictive_score_debug_rows_since_flush_ >=
        shadow_supervisor_kinematic_predictive_score_debug_flush_interval_) {
      shadow_supervisor_kinematic_predictive_score_debug_csv_.flush();
      shadow_supervisor_kinematic_predictive_score_debug_rows_since_flush_ = 0;
    }
  }

  void flushShadowSupervisorKinematicPredictiveScorePending_(double ros_now_sec)
  {
    if (!shadow_supervisor_kinematic_predictive_score_debug_enable_ ||
        !shadow_supervisor_kinematic_predictive_score_debug_csv_.is_open()) {
      return;
    }
    pruneShadowSupervisorKinematicPredictiveScoreReferenceHistory_(ros_now_sec);
    const double retention_sec =
      shadowSupervisorKinematicPredictiveScoreRetentionSec_();
    for (auto it =
           shadow_supervisor_kinematic_predictive_score_pending_local_samples_.begin();
         it != shadow_supervisor_kinematic_predictive_score_pending_local_samples_.end(); ) {
      const ShadowSupervisorKinematicPredictiveScoreSample * reference =
        findShadowSupervisorKinematicPredictiveScoreReference_(*it);
      const double shadow_sample_age_sec =
        it->valid ? ros_now_sec - it->publish_ros_time_sec :
          std::numeric_limits<double>::quiet_NaN();
      const bool expired =
        !it->valid ||
        (std::isfinite(shadow_sample_age_sec) &&
         shadow_sample_age_sec > retention_sec);
      if (reference != nullptr || expired) {
        writeShadowSupervisorKinematicPredictiveScoreDebugRow_(
          ros_now_sec, *it, reference);
        it =
          shadow_supervisor_kinematic_predictive_score_pending_local_samples_.erase(it);
      } else {
        ++it;
      }
    }
    while (shadow_supervisor_kinematic_predictive_score_pending_local_samples_.size() > 64) {
      writeShadowSupervisorKinematicPredictiveScoreDebugRow_(
        ros_now_sec,
        shadow_supervisor_kinematic_predictive_score_pending_local_samples_.front(),
        nullptr);
      shadow_supervisor_kinematic_predictive_score_pending_local_samples_.pop_front();
    }
  }

  void maybeHandleShadowSupervisorKinematicPredictiveScore_(
    double gnss_stamp_sec,
    double measurement_latitude_deg,
    double measurement_longitude_deg,
    double measurement_altitude_m,
    double raw_position_accuracy_h_m,
    double raw_position_accuracy_u_m,
    double raw_vn_mps,
    double raw_ve_mps,
    double raw_vd_mps,
    double raw_speed_accuracy_mps,
    double raw_vertical_speed_accuracy_mps)
  {
    if (!shadow_supervisor_kinematic_predictive_score_publish_enable_ &&
        !shadow_supervisor_kinematic_predictive_score_debug_enable_) {
      return;
    }

    const double ros_now_sec = now().seconds();
    const ShadowSupervisorKinematicPredictiveScoreSample local_sample =
      makeShadowSupervisorKinematicPredictiveScoreSample_(
        ros_now_sec,
        gnss_stamp_sec,
        measurement_latitude_deg,
        measurement_longitude_deg,
        measurement_altitude_m,
        raw_position_accuracy_h_m,
        raw_position_accuracy_u_m,
        raw_vn_mps,
        raw_ve_mps,
        raw_vd_mps,
        raw_speed_accuracy_mps,
        raw_vertical_speed_accuracy_mps);
    recordShadowSupervisorKinematicPredictiveScoreLocalHistory_(
      local_sample, ros_now_sec);

    if (shadow_supervisor_kinematic_predictive_score_publish_enable_ &&
        shadow_supervisor_kinematic_predictive_score_pub_ &&
        local_sample.valid) {
      shadow_supervisor_kinematic_predictive_score_pub_->publish(
        encodeShadowSupervisorKinematicPredictiveScore_(local_sample));
    }

    if (shadow_supervisor_kinematic_predictive_score_debug_enable_ &&
        shadow_supervisor_kinematic_predictive_score_debug_csv_.is_open()) {
      shadow_supervisor_kinematic_predictive_score_pending_local_samples_.push_back(
        local_sample);
      flushShadowSupervisorKinematicPredictiveScorePending_(ros_now_sec);
    }
  }

  static std::string csvSafe_(std::string value)
  {
    for (char & c : value) {
      if (c == ',' || c == '\n' || c == '\r') {
        c = ' ';
      }
    }
    return value;
  }

  static double matrixTrace_(const Eigen::MatrixXd & matrix)
  {
    if (matrix.rows() <= 0 || matrix.cols() <= 0) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    const int n = std::min(matrix.rows(), matrix.cols());
    double trace = 0.0;
    for (int i = 0; i < n; ++i) {
      const double value = matrix(i, i);
      if (!std::isfinite(value)) {
        return std::numeric_limits<double>::quiet_NaN();
      }
      trace += value;
    }
    return trace;
  }

  static const char * shadowSupervisorStateLabel_(ShadowSupervisorFsmState state)
  {
    switch (state) {
      case ShadowSupervisorFsmState::MainSafe:
        return "MAIN_SAFE";
      case ShadowSupervisorFsmState::ObservationGuard:
        return "OBSERVATION_GUARD";
      case ShadowSupervisorFsmState::ProcessCandidate:
        return "PROCESS_CANDIDATE";
      case ShadowSupervisorFsmState::ShadowWarmup:
        return "SHADOW_WARMUP";
      case ShadowSupervisorFsmState::ShadowReady:
        return "SHADOW_READY";
      case ShadowSupervisorFsmState::FailSafe:
        return "FAIL_SAFE";
    }
    return "UNKNOWN";
  }

  static double odomYawDeg_(const nav_msgs::msg::Odometry & odom)
  {
    tf2::Quaternion q(
      odom.pose.pose.orientation.x,
      odom.pose.pose.orientation.y,
      odom.pose.pose.orientation.z,
      odom.pose.pose.orientation.w);
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    return yaw * 180.0 / M_PI;
  }

  void configureShadowSupervisorFsmDebug_()
  {
    if (!shadow_supervisor_fsm_debug_enable_) {
      return;
    }

    if (!shadow_supervisor_fsm_debug_csv_path_.empty()) {
      try {
        const std::filesystem::path csv_path(shadow_supervisor_fsm_debug_csv_path_);
        if (csv_path.has_parent_path()) {
          std::filesystem::create_directories(csv_path.parent_path());
        }
        shadow_supervisor_fsm_debug_csv_.open(csv_path, std::ios::out | std::ios::trunc);
      } catch (const std::exception & e) {
        RCLCPP_WARN(
          get_logger(),
          "Failed to open shadow supervisor FSM debug CSV %s: %s",
          shadow_supervisor_fsm_debug_csv_path_.c_str(),
          e.what());
      }
      if (shadow_supervisor_fsm_debug_csv_.is_open()) {
        shadow_supervisor_fsm_debug_csv_
          << "sequence,ros_time_sec,odom_stamp_sec,core_time_sec,state,prev_state,"
          << "transition_reason,output_policy,observation_guard,observation_reason,"
          << "gamma_clipped,observation_score,process_candidate,process_reason,"
          << "lambda_vrw,lambda_accbias,process_score,source_gate_allowed,"
          << "source_gate_reason,restore_applied,restore_age_sec,warmup_elapsed_sec,"
          << "reference_odom_age_sec,main_shadow_xy_delta_m,main_shadow_z_delta_m,"
          << "main_shadow_vel_delta_mps,main_shadow_yaw_delta_deg,consistency_gate,"
          << "consistency_reason,perf_proxy_enabled,perf_proxy_reference_age_sec,"
          << "perf_proxy_reference_core_time_sec,perf_proxy_shadow_core_time_sec,"
          << "perf_proxy_reference_residual_h_m,perf_proxy_shadow_residual_h_m,"
          << "perf_proxy_residual_gap_h_m,perf_proxy_residual_gap_mean_5s_m,"
          << "perf_proxy_residual_gap_mean_10s_m,perf_proxy_residual_ratio_h,"
          << "perf_proxy_reference_nis_h_2d,perf_proxy_shadow_nis_h_2d,"
          << "perf_proxy_nis_gap_h_2d,perf_proxy_nis_gap_mean_5s,"
          << "perf_proxy_nis_gap_mean_10s,perf_proxy_nis_ratio_h,"
          << "perf_proxy_soft_res_gap_5s_pass,perf_proxy_permission_nis_gap_10s_pass,"
          << "perf_proxy_permission_res10_nis10_pass,perf_proxy_diag_reason,"
          << "shadow_ready,fail_safe\n";
        shadow_supervisor_fsm_debug_csv_.flush();
      } else {
        RCLCPP_WARN(
          get_logger(),
          "Failed to open shadow supervisor FSM debug CSV: %s",
          shadow_supervisor_fsm_debug_csv_path_.c_str());
      }
    }

    if (!shadow_supervisor_fsm_events_csv_path_.empty()) {
      try {
        const std::filesystem::path csv_path(shadow_supervisor_fsm_events_csv_path_);
        if (csv_path.has_parent_path()) {
          std::filesystem::create_directories(csv_path.parent_path());
        }
        shadow_supervisor_fsm_events_csv_.open(csv_path, std::ios::out | std::ios::trunc);
      } catch (const std::exception & e) {
        RCLCPP_WARN(
          get_logger(),
          "Failed to open shadow supervisor FSM events CSV %s: %s",
          shadow_supervisor_fsm_events_csv_path_.c_str(),
          e.what());
      }
      if (shadow_supervisor_fsm_events_csv_.is_open()) {
        shadow_supervisor_fsm_events_csv_
          << "sequence,ros_time_sec,core_time_sec,event,prev_state,state,reason\n";
        shadow_supervisor_fsm_events_csv_.flush();
      } else {
        RCLCPP_WARN(
          get_logger(),
          "Failed to open shadow supervisor FSM events CSV: %s",
          shadow_supervisor_fsm_events_csv_path_.c_str());
      }
    }

    shadow_supervisor_reference_odom_sub_ =
      this->create_subscription<nav_msgs::msg::Odometry>(
        shadow_supervisor_fsm_reference_odom_topic_, rclcpp::QoS(10),
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          handleShadowSupervisorReferenceOdom_(msg);
        });

    RCLCPP_INFO(
      get_logger(),
      "Shadow supervisor FSM debug enabled: reference_odom=%s debug_csv=%s events_csv=%s",
      shadow_supervisor_fsm_reference_odom_topic_.c_str(),
      shadow_supervisor_fsm_debug_csv_path_.empty()
        ? "<disabled>" : shadow_supervisor_fsm_debug_csv_path_.c_str(),
      shadow_supervisor_fsm_events_csv_path_.empty()
        ? "<disabled>" : shadow_supervisor_fsm_events_csv_path_.c_str());
  }

  void handleShadowSupervisorReferenceOdom_(
    const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!msg) {
      return;
    }
    shadow_supervisor_reference_odom_.valid = true;
    shadow_supervisor_reference_odom_.stamp_sec =
      rclcpp::Time(msg->header.stamp).seconds();
    shadow_supervisor_reference_odom_.rx_ros_time_sec = now().seconds();
    shadow_supervisor_reference_odom_.position_enu =
      Eigen::Vector3d(
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.position.z);
    shadow_supervisor_reference_odom_.velocity_enu =
      Eigen::Vector3d(
        msg->twist.twist.linear.x,
        msg->twist.twist.linear.y,
        msg->twist.twist.linear.z);
    shadow_supervisor_reference_odom_.yaw_deg = odomYawDeg_(*msg);
  }

  void logShadowSupervisorFsmEvent_(
    const std::string & event,
    ShadowSupervisorFsmState prev_state,
    ShadowSupervisorFsmState state,
    const std::string & reason)
  {
    if (!shadow_supervisor_fsm_events_csv_.is_open()) {
      return;
    }
    shadow_supervisor_fsm_events_csv_
      << (++shadow_supervisor_fsm_event_sequence_) << ','
      << now().seconds() << ','
      << last_core_time_ << ','
      << csvSafe_(event) << ','
      << shadowSupervisorStateLabel_(prev_state) << ','
      << shadowSupervisorStateLabel_(state) << ','
      << csvSafe_(reason) << '\n';
    shadow_supervisor_fsm_events_csv_.flush();
  }

  void transitionShadowSupervisorFsm_(
    ShadowSupervisorFsmState next_state,
    const std::string & reason)
  {
    if (next_state == shadow_supervisor_fsm_state_) {
      shadow_supervisor_fsm_last_transition_reason_ = reason;
      return;
    }
    const ShadowSupervisorFsmState prev = shadow_supervisor_fsm_state_;
    shadow_supervisor_fsm_state_ = next_state;
    shadow_supervisor_fsm_last_transition_reason_ = reason;
    logShadowSupervisorFsmEvent_("transition", prev, next_state, reason);
  }

  void noteShadowSupervisorRestoreApplied_(double restore_core_time_sec)
  {
    shadow_supervisor_restore_applied_pending_ = true;
    shadow_supervisor_have_restore_ = true;
    shadow_supervisor_last_restore_core_time_sec_ = restore_core_time_sec;
    shadow_supervisor_ready_candidate_since_core_time_sec_ =
      std::numeric_limits<double>::quiet_NaN();
    if (!shadow_supervisor_fsm_debug_enable_) {
      return;
    }
    transitionShadowSupervisorFsm_(
      ShadowSupervisorFsmState::ShadowWarmup, "restore_applied");
  }

  void logShadowSupervisorFsmDebug_(
    const nav_msgs::msg::Odometry & local_odom)
  {
    if (!shadow_supervisor_fsm_debug_enable_ ||
        !shadow_supervisor_fsm_debug_csv_.is_open()) {
      return;
    }
    const double ros_now_sec = now().seconds();
    if (shadow_supervisor_fsm_debug_max_rate_hz_ > 0.0 &&
        std::isfinite(shadow_supervisor_fsm_last_debug_log_ros_time_sec_) &&
        (ros_now_sec - shadow_supervisor_fsm_last_debug_log_ros_time_sec_) <
          (1.0 / shadow_supervisor_fsm_debug_max_rate_hz_)) {
      return;
    }
    shadow_supervisor_fsm_last_debug_log_ros_time_sec_ = ros_now_sec;
    const double odom_stamp_sec = rclcpp::Time(local_odom.header.stamp).seconds();

    const kfcore::ObservationDebug observation_debug =
      core_ ? core_->lastObservationDebug() : kfcore::ObservationDebug{};
    const kfcore::BoundedAdaptiveRDebug & rq =
      observation_debug.gnss_position_adaptive_r;
    const double gamma_clipped = rq.gamma_clipped;
    const double observation_score = rq.observation_score;
    const double lambda_vrw = rq.q_lambda_vrw;
    const double lambda_accbias = rq.q_lambda_accbias;
    const double process_score = rq.process_score;
    const bool source_gate_allowed = rq.q_source_gate_allowed;

    bool observation_guard =
      std::isfinite(gamma_clipped) &&
      gamma_clipped > shadow_supervisor_fsm_gamma_guard_enter_;
    std::string observation_reason =
      observation_guard ? "gamma_guard" : "clear";
    if (shadow_supervisor_fsm_observation_score_guard_enable_ &&
        std::isfinite(observation_score) &&
        observation_score > shadow_supervisor_fsm_observation_score_guard_enter_) {
      observation_guard = true;
      observation_reason = "observation_score_guard";
    }

    bool process_candidate = false;
    std::string process_reason = "below_trigger";
    const bool lambda_trigger =
      (std::isfinite(lambda_vrw) &&
       lambda_vrw >= shadow_supervisor_fsm_process_lambda_enter_) ||
      (std::isfinite(lambda_accbias) &&
       lambda_accbias >= shadow_supervisor_fsm_process_lambda_enter_);
    const bool score_trigger =
      std::isfinite(process_score) &&
      process_score >= shadow_supervisor_fsm_process_score_min_;
    const bool gamma_allows_shadow =
      !std::isfinite(gamma_clipped) ||
      gamma_clipped <= shadow_supervisor_fsm_gamma_shadow_max_;
    if (observation_guard) {
      process_reason = "observation_guard";
    } else if (!source_gate_allowed) {
      process_reason = "source_gate_block";
    } else if (!gamma_allows_shadow) {
      process_reason = "gamma_shadow_block";
    } else if (!shadow_supervisor_fsm_allow_mixed_trigger_ &&
               std::isfinite(gamma_clipped) &&
               gamma_clipped > 1.0 + 1e-6) {
      process_reason = "mixed_gamma_block";
    } else if (lambda_trigger || score_trigger) {
      process_candidate = true;
      process_reason = lambda_trigger ? "lambda_trigger" : "process_score_trigger";
    }

    const double restore_age_sec =
      (shadow_supervisor_have_restore_ &&
       std::isfinite(shadow_supervisor_last_restore_core_time_sec_) &&
       std::isfinite(last_core_time_))
        ? last_core_time_ - shadow_supervisor_last_restore_core_time_sec_
        : std::numeric_limits<double>::quiet_NaN();
    const double warmup_elapsed_sec = restore_age_sec;

    const Eigen::Vector3d local_position(
      local_odom.pose.pose.position.x,
      local_odom.pose.pose.position.y,
      local_odom.pose.pose.position.z);
    const Eigen::Vector3d local_velocity(
      local_odom.twist.twist.linear.x,
      local_odom.twist.twist.linear.y,
      local_odom.twist.twist.linear.z);
    const double local_yaw_deg = odomYawDeg_(local_odom);

    const double reference_age_sec =
      shadow_supervisor_reference_odom_.valid
        ? ros_now_sec - shadow_supervisor_reference_odom_.rx_ros_time_sec
        : std::numeric_limits<double>::quiet_NaN();
    const bool reference_fresh =
      shadow_supervisor_reference_odom_.valid &&
      std::isfinite(reference_age_sec) &&
      (shadow_supervisor_fsm_reference_odom_max_age_sec_ < 0.0 ||
       reference_age_sec <= shadow_supervisor_fsm_reference_odom_max_age_sec_);

    double xy_delta_m = std::numeric_limits<double>::quiet_NaN();
    double z_delta_m = std::numeric_limits<double>::quiet_NaN();
    double vel_delta_mps = std::numeric_limits<double>::quiet_NaN();
    double yaw_delta_deg = std::numeric_limits<double>::quiet_NaN();
    if (reference_fresh) {
      const Eigen::Vector3d position_delta =
        local_position - shadow_supervisor_reference_odom_.position_enu;
      const Eigen::Vector3d velocity_delta =
        local_velocity - shadow_supervisor_reference_odom_.velocity_enu;
      xy_delta_m = position_delta.head<2>().norm();
      z_delta_m = std::abs(position_delta.z());
      vel_delta_mps = velocity_delta.norm();
      yaw_delta_deg =
        std::abs(normalizeAngleDeg_(local_yaw_deg - shadow_supervisor_reference_odom_.yaw_deg));
    }

    bool consistency_gate = false;
    std::string consistency_reason = "reference_missing";
    if (reference_fresh) {
      consistency_gate =
        std::isfinite(xy_delta_m) &&
        std::isfinite(z_delta_m) &&
        std::isfinite(vel_delta_mps) &&
        std::isfinite(yaw_delta_deg) &&
        xy_delta_m <= shadow_supervisor_fsm_xy_delta_ready_max_m_ &&
        z_delta_m <= shadow_supervisor_fsm_z_delta_ready_max_m_ &&
        vel_delta_mps <= shadow_supervisor_fsm_vel_delta_ready_max_mps_ &&
        yaw_delta_deg <= shadow_supervisor_fsm_yaw_delta_ready_max_deg_;
      consistency_reason = consistency_gate ? "pass" : "delta_gate";
    } else if (shadow_supervisor_reference_odom_.valid) {
      consistency_reason = "reference_stale";
    }

    const ShadowSupervisorPerformanceProxySample local_perf_proxy =
      makeShadowSupervisorPerformanceProxySample_(ros_now_sec, odom_stamp_sec);
    const bool perf_proxy_enabled = shadow_supervisor_perf_proxy_subscribe_enable_;
    double perf_proxy_reference_age_sec =
      std::numeric_limits<double>::quiet_NaN();
    double perf_proxy_reference_core_time_sec =
      std::numeric_limits<double>::quiet_NaN();
    double perf_proxy_shadow_core_time_sec =
      std::numeric_limits<double>::quiet_NaN();
    double perf_proxy_reference_residual_h_m =
      std::numeric_limits<double>::quiet_NaN();
    double perf_proxy_shadow_residual_h_m =
      std::numeric_limits<double>::quiet_NaN();
    double perf_proxy_residual_gap_h_m =
      std::numeric_limits<double>::quiet_NaN();
    double perf_proxy_residual_gap_mean_5s_m =
      std::numeric_limits<double>::quiet_NaN();
    double perf_proxy_residual_gap_mean_10s_m =
      std::numeric_limits<double>::quiet_NaN();
    double perf_proxy_residual_ratio_h =
      std::numeric_limits<double>::quiet_NaN();
    double perf_proxy_reference_nis_h_2d =
      std::numeric_limits<double>::quiet_NaN();
    double perf_proxy_shadow_nis_h_2d =
      std::numeric_limits<double>::quiet_NaN();
    double perf_proxy_nis_gap_h_2d =
      std::numeric_limits<double>::quiet_NaN();
    double perf_proxy_nis_gap_mean_5s =
      std::numeric_limits<double>::quiet_NaN();
    double perf_proxy_nis_gap_mean_10s =
      std::numeric_limits<double>::quiet_NaN();
    double perf_proxy_nis_ratio_h =
      std::numeric_limits<double>::quiet_NaN();
    bool perf_proxy_soft_res_gap_5s_pass = false;
    bool perf_proxy_permission_nis_gap_10s_pass = false;
    bool perf_proxy_permission_res10_nis10_pass = false;
    std::string perf_proxy_diag_reason =
      perf_proxy_enabled ? "proxy_missing" : "disabled";

    if (perf_proxy_enabled) {
      perf_proxy_reference_age_sec =
        shadow_supervisor_latest_perf_proxy_.valid
          ? ros_now_sec - shadow_supervisor_latest_perf_proxy_.publish_ros_time_sec
          : std::numeric_limits<double>::quiet_NaN();
      const bool reference_proxy_fresh =
        shadow_supervisor_latest_perf_proxy_.valid &&
        std::isfinite(perf_proxy_reference_age_sec) &&
        (shadow_supervisor_perf_proxy_max_age_sec_ < 0.0 ||
         perf_proxy_reference_age_sec <= shadow_supervisor_perf_proxy_max_age_sec_);
      if (!local_perf_proxy.valid) {
        perf_proxy_diag_reason = "local_invalid";
      } else if (!shadow_supervisor_latest_perf_proxy_.valid) {
        perf_proxy_diag_reason = "proxy_missing";
      } else if (!reference_proxy_fresh) {
        perf_proxy_diag_reason = "proxy_stale";
      } else {
        perf_proxy_reference_core_time_sec =
          shadow_supervisor_latest_perf_proxy_.core_time_sec;
        perf_proxy_shadow_core_time_sec = local_perf_proxy.core_time_sec;
        perf_proxy_reference_residual_h_m =
          shadow_supervisor_latest_perf_proxy_.residual_h_m;
        perf_proxy_shadow_residual_h_m = local_perf_proxy.residual_h_m;
        perf_proxy_residual_gap_h_m =
          perf_proxy_shadow_residual_h_m - perf_proxy_reference_residual_h_m;
        perf_proxy_reference_nis_h_2d =
          shadow_supervisor_latest_perf_proxy_.nis_h_2d;
        perf_proxy_shadow_nis_h_2d = local_perf_proxy.nis_h_2d;
        perf_proxy_nis_gap_h_2d =
          perf_proxy_shadow_nis_h_2d - perf_proxy_reference_nis_h_2d;
        if (std::isfinite(perf_proxy_reference_residual_h_m) &&
            std::abs(perf_proxy_reference_residual_h_m) > 1.0e-9) {
          perf_proxy_residual_ratio_h =
            perf_proxy_shadow_residual_h_m / perf_proxy_reference_residual_h_m;
        }
        if (std::isfinite(perf_proxy_reference_nis_h_2d) &&
            std::abs(perf_proxy_reference_nis_h_2d) > 1.0e-9) {
          perf_proxy_nis_ratio_h =
            perf_proxy_shadow_nis_h_2d / perf_proxy_reference_nis_h_2d;
        }

        shadow_supervisor_perf_proxy_gap_history_.push_back(
          ShadowSupervisorPerformanceProxyGapSample{
            ros_now_sec,
            perf_proxy_residual_gap_h_m,
            perf_proxy_nis_gap_h_2d});
        const double history_window_sec =
          std::max(
            shadow_supervisor_perf_proxy_short_window_sec_,
            shadow_supervisor_perf_proxy_long_window_sec_);
        while (!shadow_supervisor_perf_proxy_gap_history_.empty() &&
               std::isfinite(history_window_sec) &&
               history_window_sec >= 0.0 &&
               (ros_now_sec - shadow_supervisor_perf_proxy_gap_history_.front().ros_time_sec) >
                 history_window_sec + 1.0) {
          shadow_supervisor_perf_proxy_gap_history_.pop_front();
        }
        auto mean_recent = [&](
            auto value_fn,
            double window_sec) -> double {
          if (window_sec < 0.0) {
            return std::numeric_limits<double>::quiet_NaN();
          }
          double sum = 0.0;
          int count = 0;
          for (auto it = shadow_supervisor_perf_proxy_gap_history_.rbegin();
               it != shadow_supervisor_perf_proxy_gap_history_.rend();
               ++it) {
            if ((ros_now_sec - it->ros_time_sec) > window_sec) {
              break;
            }
            const double value = value_fn(*it);
            if (!std::isfinite(value)) {
              continue;
            }
            sum += value;
            ++count;
          }
          return count > 0 ? sum / static_cast<double>(count)
                           : std::numeric_limits<double>::quiet_NaN();
        };
        perf_proxy_residual_gap_mean_5s_m =
          mean_recent(
            [](const ShadowSupervisorPerformanceProxyGapSample & sample) {
              return sample.residual_gap_h_m;
            },
            shadow_supervisor_perf_proxy_short_window_sec_);
        perf_proxy_residual_gap_mean_10s_m =
          mean_recent(
            [](const ShadowSupervisorPerformanceProxyGapSample & sample) {
              return sample.residual_gap_h_m;
            },
            shadow_supervisor_perf_proxy_long_window_sec_);
        perf_proxy_nis_gap_mean_5s =
          mean_recent(
            [](const ShadowSupervisorPerformanceProxyGapSample & sample) {
              return sample.nis_gap_h_2d;
            },
            shadow_supervisor_perf_proxy_short_window_sec_);
        perf_proxy_nis_gap_mean_10s =
          mean_recent(
            [](const ShadowSupervisorPerformanceProxyGapSample & sample) {
              return sample.nis_gap_h_2d;
            },
            shadow_supervisor_perf_proxy_long_window_sec_);
        perf_proxy_soft_res_gap_5s_pass =
          std::isfinite(perf_proxy_residual_gap_mean_5s_m) &&
          perf_proxy_residual_gap_mean_5s_m <=
            shadow_supervisor_perf_proxy_residual_gap_soft_max_m_;
        perf_proxy_permission_nis_gap_10s_pass =
          std::isfinite(perf_proxy_nis_gap_mean_10s) &&
          perf_proxy_nis_gap_mean_10s <=
            shadow_supervisor_perf_proxy_nis_gap_permission_max_;
        perf_proxy_permission_res10_nis10_pass =
          std::isfinite(perf_proxy_residual_gap_mean_10s_m) &&
          std::isfinite(perf_proxy_nis_gap_mean_10s) &&
          perf_proxy_residual_gap_mean_10s_m <=
            shadow_supervisor_perf_proxy_residual_gap_soft_max_m_ &&
          perf_proxy_nis_gap_mean_10s <=
            shadow_supervisor_perf_proxy_nis_gap_permission_max_;
        if (perf_proxy_permission_res10_nis10_pass) {
          perf_proxy_diag_reason = "res10_nis10_permission";
        } else if (perf_proxy_permission_nis_gap_10s_pass) {
          perf_proxy_diag_reason = "nis10_permission";
        } else if (perf_proxy_soft_res_gap_5s_pass) {
          perf_proxy_diag_reason = "res5_soft";
        } else {
          perf_proxy_diag_reason = "not_selected";
        }
      }
    }

    const ShadowSupervisorFsmState prev_state = shadow_supervisor_fsm_state_;
    if (shadow_supervisor_fsm_state_ == ShadowSupervisorFsmState::ShadowWarmup ||
        shadow_supervisor_fsm_state_ == ShadowSupervisorFsmState::ShadowReady) {
      if (observation_guard) {
        transitionShadowSupervisorFsm_(
          ShadowSupervisorFsmState::ObservationGuard, observation_reason);
      } else if (std::isfinite(warmup_elapsed_sec) &&
                 warmup_elapsed_sec >= shadow_supervisor_fsm_warmup_sec_) {
        if (consistency_gate) {
          if (!std::isfinite(shadow_supervisor_ready_candidate_since_core_time_sec_)) {
            shadow_supervisor_ready_candidate_since_core_time_sec_ = last_core_time_;
          }
          const double ready_confirm_elapsed =
            std::isfinite(last_core_time_)
              ? last_core_time_ - shadow_supervisor_ready_candidate_since_core_time_sec_
              : std::numeric_limits<double>::quiet_NaN();
          if (std::isfinite(ready_confirm_elapsed) &&
              ready_confirm_elapsed >= shadow_supervisor_fsm_ready_confirm_sec_) {
            transitionShadowSupervisorFsm_(
              ShadowSupervisorFsmState::ShadowReady, "warmup_consistency_confirmed");
          }
        } else {
          shadow_supervisor_ready_candidate_since_core_time_sec_ =
            std::numeric_limits<double>::quiet_NaN();
          transitionShadowSupervisorFsm_(
            ShadowSupervisorFsmState::ShadowWarmup, consistency_reason);
        }
      }
    } else if (observation_guard) {
      transitionShadowSupervisorFsm_(
        ShadowSupervisorFsmState::ObservationGuard, observation_reason);
    } else if (shadow_supervisor_have_restore_ && std::isfinite(restore_age_sec)) {
      transitionShadowSupervisorFsm_(
        ShadowSupervisorFsmState::ShadowWarmup, "restore_warmup_resume");
    } else if (process_candidate) {
      transitionShadowSupervisorFsm_(
        ShadowSupervisorFsmState::ProcessCandidate, process_reason);
    } else {
      transitionShadowSupervisorFsm_(
        ShadowSupervisorFsmState::MainSafe, process_reason);
    }

    const bool shadow_ready =
      shadow_supervisor_fsm_state_ == ShadowSupervisorFsmState::ShadowReady;
    const bool fail_safe =
      shadow_supervisor_fsm_state_ == ShadowSupervisorFsmState::FailSafe;
    const bool restore_applied = shadow_supervisor_restore_applied_pending_;
    shadow_supervisor_restore_applied_pending_ = false;

    shadow_supervisor_fsm_debug_csv_
      << (++shadow_supervisor_fsm_debug_sequence_) << ','
      << ros_now_sec << ','
      << odom_stamp_sec << ','
      << last_core_time_ << ','
      << shadowSupervisorStateLabel_(shadow_supervisor_fsm_state_) << ','
      << shadowSupervisorStateLabel_(prev_state) << ','
      << csvSafe_(shadow_supervisor_fsm_last_transition_reason_) << ','
      << "main_only" << ','
      << (observation_guard ? 1 : 0) << ','
      << csvSafe_(observation_reason) << ','
      << gamma_clipped << ','
      << observation_score << ','
      << (process_candidate ? 1 : 0) << ','
      << csvSafe_(process_reason) << ','
      << lambda_vrw << ','
      << lambda_accbias << ','
      << process_score << ','
      << (source_gate_allowed ? 1 : 0) << ','
      << csvSafe_(rq.q_source_gate_reason) << ','
      << (restore_applied ? 1 : 0) << ','
      << restore_age_sec << ','
      << warmup_elapsed_sec << ','
      << reference_age_sec << ','
      << xy_delta_m << ','
      << z_delta_m << ','
      << vel_delta_mps << ','
      << yaw_delta_deg << ','
      << (consistency_gate ? 1 : 0) << ','
      << csvSafe_(consistency_reason) << ','
      << (perf_proxy_enabled ? 1 : 0) << ','
      << perf_proxy_reference_age_sec << ','
      << perf_proxy_reference_core_time_sec << ','
      << perf_proxy_shadow_core_time_sec << ','
      << perf_proxy_reference_residual_h_m << ','
      << perf_proxy_shadow_residual_h_m << ','
      << perf_proxy_residual_gap_h_m << ','
      << perf_proxy_residual_gap_mean_5s_m << ','
      << perf_proxy_residual_gap_mean_10s_m << ','
      << perf_proxy_residual_ratio_h << ','
      << perf_proxy_reference_nis_h_2d << ','
      << perf_proxy_shadow_nis_h_2d << ','
      << perf_proxy_nis_gap_h_2d << ','
      << perf_proxy_nis_gap_mean_5s << ','
      << perf_proxy_nis_gap_mean_10s << ','
      << perf_proxy_nis_ratio_h << ','
      << (perf_proxy_soft_res_gap_5s_pass ? 1 : 0) << ','
      << (perf_proxy_permission_nis_gap_10s_pass ? 1 : 0) << ','
      << (perf_proxy_permission_res10_nis10_pass ? 1 : 0) << ','
      << csvSafe_(perf_proxy_diag_reason) << ','
      << (shadow_ready ? 1 : 0) << ','
      << (fail_safe ? 1 : 0) << '\n';

    ++shadow_supervisor_fsm_debug_rows_since_flush_;
    if (shadow_supervisor_fsm_debug_rows_since_flush_ >=
        shadow_supervisor_fsm_debug_flush_interval_) {
      shadow_supervisor_fsm_debug_csv_.flush();
      shadow_supervisor_fsm_debug_rows_since_flush_ = 0;
    }
  }

  void logShadowRestoreEvent_(
    const std::string & event,
    bool ok,
    const std::string & reason,
    double publish_ros_time_sec,
    double snapshot_time_sec,
    double age_sec,
    const kfcore::State & state,
    double covariance_trace)
  {
    if (!shadow_restore_event_csv_.is_open()) {
      return;
    }
    shadow_restore_event_csv_
      << (++shadow_restore_event_sequence_) << ','
      << now().seconds() << ','
      << csvSafe_(event) << ','
      << (ok ? 1 : 0) << ','
      << csvSafe_(reason) << ','
      << publish_ros_time_sec << ','
      << snapshot_time_sec << ','
      << age_sec << ','
      << last_core_time_ << ','
      << state.lat_deg << ','
      << state.lon_deg << ','
      << state.h_m << ','
      << state.vN << ','
      << state.vE << ','
      << state.vD << ','
      << state.roll_deg << ','
      << state.pitch_deg << ','
      << state.yaw_deg << ','
      << covariance_trace << ','
      << shadow_restore_covariance_inflation_factor_
      << '\n';
    shadow_restore_event_csv_.flush();
  }

  static void appendVector3_(std::vector<double> & data, const Eigen::Vector3d & value)
  {
    data.push_back(value.x());
    data.push_back(value.y());
    data.push_back(value.z());
  }

  static void appendMatrix_(std::vector<double> & data, const Eigen::MatrixXd & matrix)
  {
    data.push_back(static_cast<double>(matrix.rows()));
    data.push_back(static_cast<double>(matrix.cols()));
    for (int r = 0; r < matrix.rows(); ++r) {
      for (int c = 0; c < matrix.cols(); ++c) {
        data.push_back(matrix(r, c));
      }
    }
  }

  static void appendImuSampleSnapshot_(
    std::vector<double> & data, const kfcore::ImuSampleSnapshot & sample)
  {
    data.push_back(sample.time_sec);
    data.push_back(sample.dt_sec);
    appendVector3_(data, sample.dtheta);
    appendVector3_(data, sample.dvel);
    data.push_back(sample.odovel);
  }

  std_msgs::msg::Float64MultiArray encodeShadowRestoreSnapshot_(
    const kfcore::KFCoreSnapshot & snapshot,
    double publish_ros_time_sec) const
  {
    std_msgs::msg::Float64MultiArray msg;
    auto & data = msg.data;
    data.reserve(560);
    data.push_back(20260520.0);  // magic
    data.push_back(1.0);         // version
    data.push_back(publish_ros_time_sec);
    data.push_back(snapshot.valid ? 1.0 : 0.0);
    data.push_back(snapshot.time_sec);

    data.push_back(snapshot.state.sow);
    data.push_back(snapshot.state.lat_deg);
    data.push_back(snapshot.state.lon_deg);
    data.push_back(snapshot.state.h_m);
    data.push_back(snapshot.state.vN);
    data.push_back(snapshot.state.vE);
    data.push_back(snapshot.state.vD);
    data.push_back(snapshot.state.roll_deg);
    data.push_back(snapshot.state.pitch_deg);
    data.push_back(snapshot.state.yaw_deg);

    appendVector3_(data, snapshot.imu_error.gyro_bias_radps);
    appendVector3_(data, snapshot.imu_error.acc_bias_mps2);
    appendVector3_(data, snapshot.imu_error.gyro_scale);
    appendVector3_(data, snapshot.imu_error.acc_scale);
    appendMatrix_(data, snapshot.covariance);
    appendMatrix_(data, snapshot.error_state);
    appendImuSampleSnapshot_(data, snapshot.previous_imu);
    appendImuSampleSnapshot_(data, snapshot.current_imu);

    data.push_back(snapshot.adaptive_r.position_gamma);
    data.push_back(snapshot.adaptive_r.velocity_gamma);

    data.push_back(snapshot.adaptive_rq.position_gamma);
    data.push_back(snapshot.adaptive_rq.velocity_gamma);
    data.push_back(snapshot.adaptive_rq.vrw_q_scale);
    data.push_back(snapshot.adaptive_rq.arw_q_scale);
    data.push_back(snapshot.adaptive_rq.accbias_q_scale);
    data.push_back(snapshot.adaptive_rq.gyrbias_q_scale);
    data.push_back(static_cast<double>(snapshot.adaptive_rq.consecutive_exceed_count));
    data.push_back(static_cast<double>(snapshot.adaptive_rq.hold_remaining));
    data.push_back(snapshot.adaptive_rq.process_context_valid ? 1.0 : 0.0);
    data.push_back(snapshot.adaptive_rq.process_context_score);
    data.push_back(snapshot.adaptive_rq.source_gate_allowed ? 1.0 : 0.0);
    data.push_back(snapshot.adaptive_rq.source_confidence);
    data.push_back(snapshot.adaptive_rq.velocity_evidence_valid ? 1.0 : 0.0);
    data.push_back(snapshot.adaptive_rq.velocity_evidence_active ? 1.0 : 0.0);
    data.push_back(snapshot.adaptive_rq.velocity_evidence_nis_ratio);
    data.push_back(snapshot.adaptive_rq.velocity_evidence_residual_h_mps);
    data.push_back(snapshot.adaptive_rq.have_prev_position_r_diag ? 1.0 : 0.0);
    appendVector3_(data, snapshot.adaptive_rq.prev_position_r_diag);
    return msg;
  }

  static bool readShadowRestoreValue_(
    const std::vector<double> & data, std::size_t & idx, double * value)
  {
    if (idx >= data.size()) {
      return false;
    }
    *value = data[idx++];
    return true;
  }

  static bool readShadowRestoreBool_(
    const std::vector<double> & data, std::size_t & idx, bool * value)
  {
    double raw = 0.0;
    if (!readShadowRestoreValue_(data, idx, &raw)) {
      return false;
    }
    *value = raw >= 0.5;
    return true;
  }

  static bool readShadowRestoreInt_(
    const std::vector<double> & data, std::size_t & idx, int * value)
  {
    double raw = 0.0;
    if (!readShadowRestoreValue_(data, idx, &raw) || !std::isfinite(raw)) {
      return false;
    }
    *value = static_cast<int>(std::llround(raw));
    return true;
  }

  static bool readVector3_(
    const std::vector<double> & data, std::size_t & idx, Eigen::Vector3d * value)
  {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    if (!readShadowRestoreValue_(data, idx, &x) ||
        !readShadowRestoreValue_(data, idx, &y) ||
        !readShadowRestoreValue_(data, idx, &z)) {
      return false;
    }
    *value = Eigen::Vector3d(x, y, z);
    return true;
  }

  static bool readMatrix_(
    const std::vector<double> & data, std::size_t & idx, Eigen::MatrixXd * matrix)
  {
    double rows_raw = 0.0;
    double cols_raw = 0.0;
    if (!readShadowRestoreValue_(data, idx, &rows_raw) ||
        !readShadowRestoreValue_(data, idx, &cols_raw) ||
        !std::isfinite(rows_raw) ||
        !std::isfinite(cols_raw)) {
      return false;
    }
    const int rows = static_cast<int>(std::llround(rows_raw));
    const int cols = static_cast<int>(std::llround(cols_raw));
    if (rows < 0 || cols < 0 || rows > 128 || cols > 128) {
      return false;
    }
    if (data.size() - idx < static_cast<std::size_t>(rows * cols)) {
      return false;
    }
    matrix->resize(rows, cols);
    for (int r = 0; r < rows; ++r) {
      for (int c = 0; c < cols; ++c) {
        (*matrix)(r, c) = data[idx++];
      }
    }
    return true;
  }

  static bool readImuSampleSnapshot_(
    const std::vector<double> & data,
    std::size_t & idx,
    kfcore::ImuSampleSnapshot * sample)
  {
    if (!readShadowRestoreValue_(data, idx, &sample->time_sec) ||
        !readShadowRestoreValue_(data, idx, &sample->dt_sec) ||
        !readVector3_(data, idx, &sample->dtheta) ||
        !readVector3_(data, idx, &sample->dvel) ||
        !readShadowRestoreValue_(data, idx, &sample->odovel)) {
      return false;
    }
    return true;
  }

  bool decodeShadowRestoreSnapshot_(
    const std::vector<double> & data,
    kfcore::KFCoreSnapshot * snapshot,
    double * publish_ros_time_sec,
    std::string * error) const
  {
    std::size_t idx = 0;
    double magic = 0.0;
    double version = 0.0;
    if (!readShadowRestoreValue_(data, idx, &magic) ||
        !readShadowRestoreValue_(data, idx, &version)) {
      if (error != nullptr) *error = "short_header";
      return false;
    }
    if (std::abs(magic - 20260520.0) > 0.5 || std::abs(version - 1.0) > 1.0e-6) {
      if (error != nullptr) *error = "bad_magic_or_version";
      return false;
    }
    if (!readShadowRestoreValue_(data, idx, publish_ros_time_sec) ||
        !readShadowRestoreBool_(data, idx, &snapshot->valid) ||
        !readShadowRestoreValue_(data, idx, &snapshot->time_sec)) {
      if (error != nullptr) *error = "short_metadata";
      return false;
    }

    if (!readShadowRestoreValue_(data, idx, &snapshot->state.sow) ||
        !readShadowRestoreValue_(data, idx, &snapshot->state.lat_deg) ||
        !readShadowRestoreValue_(data, idx, &snapshot->state.lon_deg) ||
        !readShadowRestoreValue_(data, idx, &snapshot->state.h_m) ||
        !readShadowRestoreValue_(data, idx, &snapshot->state.vN) ||
        !readShadowRestoreValue_(data, idx, &snapshot->state.vE) ||
        !readShadowRestoreValue_(data, idx, &snapshot->state.vD) ||
        !readShadowRestoreValue_(data, idx, &snapshot->state.roll_deg) ||
        !readShadowRestoreValue_(data, idx, &snapshot->state.pitch_deg) ||
        !readShadowRestoreValue_(data, idx, &snapshot->state.yaw_deg) ||
        !readVector3_(data, idx, &snapshot->imu_error.gyro_bias_radps) ||
        !readVector3_(data, idx, &snapshot->imu_error.acc_bias_mps2) ||
        !readVector3_(data, idx, &snapshot->imu_error.gyro_scale) ||
        !readVector3_(data, idx, &snapshot->imu_error.acc_scale) ||
        !readMatrix_(data, idx, &snapshot->covariance) ||
        !readMatrix_(data, idx, &snapshot->error_state) ||
        !readImuSampleSnapshot_(data, idx, &snapshot->previous_imu) ||
        !readImuSampleSnapshot_(data, idx, &snapshot->current_imu) ||
        !readShadowRestoreValue_(data, idx, &snapshot->adaptive_r.position_gamma) ||
        !readShadowRestoreValue_(data, idx, &snapshot->adaptive_r.velocity_gamma) ||
        !readShadowRestoreValue_(data, idx, &snapshot->adaptive_rq.position_gamma) ||
        !readShadowRestoreValue_(data, idx, &snapshot->adaptive_rq.velocity_gamma) ||
        !readShadowRestoreValue_(data, idx, &snapshot->adaptive_rq.vrw_q_scale) ||
        !readShadowRestoreValue_(data, idx, &snapshot->adaptive_rq.arw_q_scale) ||
        !readShadowRestoreValue_(data, idx, &snapshot->adaptive_rq.accbias_q_scale) ||
        !readShadowRestoreValue_(data, idx, &snapshot->adaptive_rq.gyrbias_q_scale) ||
        !readShadowRestoreInt_(data, idx, &snapshot->adaptive_rq.consecutive_exceed_count) ||
        !readShadowRestoreInt_(data, idx, &snapshot->adaptive_rq.hold_remaining) ||
        !readShadowRestoreBool_(data, idx, &snapshot->adaptive_rq.process_context_valid) ||
        !readShadowRestoreValue_(data, idx, &snapshot->adaptive_rq.process_context_score) ||
        !readShadowRestoreBool_(data, idx, &snapshot->adaptive_rq.source_gate_allowed) ||
        !readShadowRestoreValue_(data, idx, &snapshot->adaptive_rq.source_confidence) ||
        !readShadowRestoreBool_(data, idx, &snapshot->adaptive_rq.velocity_evidence_valid) ||
        !readShadowRestoreBool_(data, idx, &snapshot->adaptive_rq.velocity_evidence_active) ||
        !readShadowRestoreValue_(data, idx, &snapshot->adaptive_rq.velocity_evidence_nis_ratio) ||
        !readShadowRestoreValue_(data, idx, &snapshot->adaptive_rq.velocity_evidence_residual_h_mps) ||
        !readShadowRestoreBool_(data, idx, &snapshot->adaptive_rq.have_prev_position_r_diag) ||
        !readVector3_(data, idx, &snapshot->adaptive_rq.prev_position_r_diag)) {
      if (error != nullptr) *error = "short_payload";
      return false;
    }
    snapshot->adaptive_rq.source_gate_reason = "restore_msg";
    return true;
  }

  void maybePublishShadowRestoreSnapshot_(const char * trigger)
  {
    if (!shadow_restore_publish_enable_ || !shadow_restore_pub_ || !core_ || !core_initialized_) {
      return;
    }
    if (shadow_restore_publish_once_ && shadow_restore_publish_done_) {
      return;
    }
    if (std::isfinite(shadow_restore_publish_after_core_sec_) &&
        shadow_restore_publish_after_core_sec_ >= 0.0 &&
        (!std::isfinite(last_core_time_) ||
         last_core_time_ < shadow_restore_publish_after_core_sec_)) {
      return;
    }
    if (!shadow_restore_publish_once_ &&
        shadow_restore_publish_period_sec_ > 0.0 &&
        std::isfinite(shadow_restore_last_publish_core_time_sec_) &&
        std::isfinite(last_core_time_) &&
        (last_core_time_ - shadow_restore_last_publish_core_time_sec_) <
          shadow_restore_publish_period_sec_) {
      return;
    }

    const kfcore::KFCoreSnapshot snapshot = core_->snapshot();
    const double covariance_trace = matrixTrace_(snapshot.covariance);
    if (!snapshot.valid) {
      logShadowRestoreEvent_(
        "publish", false, "invalid_snapshot",
        now().seconds(), snapshot.time_sec, 0.0, snapshot.state, covariance_trace);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Shadow restore snapshot publish skipped: core snapshot invalid.");
      return;
    }
    const double publish_ros_time_sec = now().seconds();
    shadow_restore_pub_->publish(
      encodeShadowRestoreSnapshot_(snapshot, publish_ros_time_sec));
    shadow_restore_publish_done_ = true;
    shadow_restore_last_publish_core_time_sec_ = last_core_time_;
    logShadowRestoreEvent_(
      "publish", true, trigger != nullptr ? trigger : "unknown",
      publish_ros_time_sec, snapshot.time_sec, 0.0, snapshot.state, covariance_trace);
    RCLCPP_INFO(
      get_logger(),
      "Published main-to-shadow snapshot: trigger=%s core_time=%.6f state=(%.8f, %.8f, %.3f) cov_trace=%.6f",
      trigger != nullptr ? trigger : "unknown",
      snapshot.time_sec,
      snapshot.state.lat_deg,
      snapshot.state.lon_deg,
      snapshot.state.h_m,
      covariance_trace);
  }

  void handleShadowRestoreSnapshot_(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (!shadow_restore_subscribe_enable_ || !msg || !core_) {
      return;
    }
    kfcore::KFCoreSnapshot snapshot;
    double publish_ros_time_sec = std::numeric_limits<double>::quiet_NaN();
    std::string decode_error;
    if (!decodeShadowRestoreSnapshot_(msg->data, &snapshot, &publish_ros_time_sec, &decode_error)) {
      logShadowRestoreEvent_(
        "apply", false, "decode_" + decode_error,
        publish_ros_time_sec, snapshot.time_sec,
        std::numeric_limits<double>::quiet_NaN(), snapshot.state,
        matrixTrace_(snapshot.covariance));
      RCLCPP_WARN(
        get_logger(),
        "Rejected shadow restore snapshot: %s",
        decode_error.c_str());
      return;
    }

    const double age_sec =
      std::isfinite(publish_ros_time_sec) ? now().seconds() - publish_ros_time_sec
                                          : std::numeric_limits<double>::quiet_NaN();
    if (shadow_restore_max_age_sec_ >= 0.0 &&
        (!std::isfinite(age_sec) || age_sec > shadow_restore_max_age_sec_)) {
      logShadowRestoreEvent_(
        "apply", false, "stale_snapshot",
        publish_ros_time_sec, snapshot.time_sec, age_sec, snapshot.state,
        matrixTrace_(snapshot.covariance));
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejected shadow restore snapshot: age=%.3f sec max=%.3f sec",
        age_sec, shadow_restore_max_age_sec_);
      return;
    }
    if (shadow_restore_require_core_initialized_ && !core_initialized_) {
      logShadowRestoreEvent_(
        "apply", false, "target_core_not_initialized",
        publish_ros_time_sec, snapshot.time_sec, age_sec, snapshot.state,
        matrixTrace_(snapshot.covariance));
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejected shadow restore snapshot: target core is not initialized yet.");
      return;
    }

    kfcore::KFCoreRestorePolicy policy;
    policy.copy_nominal_state = true;
    policy.copy_covariance = true;
    policy.copy_imu_error = true;
    policy.copy_imu_buffer = true;
    policy.copy_adaptive_r_memory = false;
    policy.copy_adaptive_rq_memory = false;
    policy.reset_adaptive_r_memory = true;
    policy.reset_adaptive_rq_memory = true;
    policy.covariance_inflation_factor =
      std::max(1.0, shadow_restore_covariance_inflation_factor_);
    policy.require_spd_covariance = true;
    policy.reason = "shadow_restore_topic";

    const bool ok = core_->restore(snapshot, policy);
    if (ok) {
      core_initialized_ = true;
      if (std::isfinite(snapshot.time_sec)) {
        core_time_sec_ = snapshot.time_sec;
        last_core_time_ = snapshot.time_sec;
      }
      have_core_imu_accumulator_ = false;
      core_imu_accum_dt_sec_ = 0.0;
      core_imu_accum_dtheta_.setZero();
      core_imu_accum_dvel_.setZero();
      if (shadow_restore_clear_path_on_apply_) {
        path_msg_.poses.clear();
        have_last_path_ = false;
        have_last_path_stamp_ = false;
      }
      resetPublishCoreStampOffset_("shadow restore");
      noteShadowSupervisorRestoreApplied_(snapshot.time_sec);
    }

    logShadowRestoreEvent_(
      "apply", ok, ok ? "restored" : "core_restore_failed",
      publish_ros_time_sec, snapshot.time_sec, age_sec, snapshot.state,
      matrixTrace_(snapshot.covariance));
    if (ok) {
      RCLCPP_INFO(
        get_logger(),
        "Applied main-to-shadow restore: age=%.3f core_time=%.6f state=(%.8f, %.8f, %.3f) cov_trace=%.6f",
        age_sec,
        snapshot.time_sec,
        snapshot.state.lat_deg,
        snapshot.state.lon_deg,
        snapshot.state.h_m,
        matrixTrace_(snapshot.covariance));
    } else {
      RCLCPP_WARN(
        get_logger(),
        "core_->restore() failed for shadow restore snapshot.");
    }
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

    if (heading_cruise_micro_track_enable_ &&
        std::strcmp(heading_mode, "update") == 0) {
      const double micro_min_residual_deg =
        std::max(0.0, std::abs(heading_cruise_micro_track_min_residual_deg_));
      const double micro_max_residual_deg =
        std::max(0.0, std::abs(heading_cruise_micro_track_max_residual_deg_));
      const bool micro_residual_ok =
        residual_abs_deg >= micro_min_residual_deg &&
        (micro_max_residual_deg <= 0.0 || residual_abs_deg <= micro_max_residual_deg);
      const bool micro_speed_ok =
        motion_ctx.have_fresh_mavros_speed &&
        (heading_cruise_micro_track_min_horizontal_speed_mps_ <= 0.0 ||
         last_mavros_horizontal_speed_mps_ >=
           heading_cruise_micro_track_min_horizontal_speed_mps_) &&
        (heading_cruise_micro_track_max_vertical_speed_mps_ <= 0.0 ||
         std::abs(last_mavros_vertical_speed_mps_) <=
           heading_cruise_micro_track_max_vertical_speed_mps_);
      const bool micro_gyro_ok =
        !std::isfinite(last_imu_gyro_norm_deg_s_) ||
        heading_cruise_micro_track_max_gyro_deg_s_ <= 0.0 ||
        last_imu_gyro_norm_deg_s_ <= heading_cruise_micro_track_max_gyro_deg_s_;
      const bool micro_source_rate_ok =
        !std::isfinite(last_mavros_heading_rate_deg_s_) ||
        heading_cruise_micro_track_max_source_yaw_rate_deg_s_ <= 0.0 ||
        std::abs(last_mavros_heading_rate_deg_s_) <=
          heading_cruise_micro_track_max_source_yaw_rate_deg_s_;
      const bool mission_cruise_context =
        mavros_mode_ == "AUTO.MISSION" &&
        armed_cruise_force_relock_context &&
        !post_turn_tracking_context &&
        micro_speed_ok &&
        micro_gyro_ok &&
        micro_source_rate_ok;
      if (mission_cruise_context && micro_residual_ok) {
        const double micro_track_std_deg =
          std::max(0.1, std::abs(heading_cruise_micro_track_std_deg_));
        if (micro_track_std_deg < heading_measurement_std_deg) {
          heading_measurement_std_deg = micro_track_std_deg;
          heading_mode = "cruise_micro_track";
        }
      }
    }

    if (heading_track_validity_gate_enable_ &&
        mavros_armed_ &&
        have_fresh_mavros_speed) {
      const bool h2_normal_update =
        std::strcmp(heading_mode, "update") == 0 ||
        std::strcmp(heading_mode, "armed_cruise_track") == 0 ||
        std::strcmp(heading_mode, "cruise_micro_track") == 0;
      const bool h2_turn_track = std::strcmp(heading_mode, "turn_track") == 0;
      const bool h2_post_turn_track = std::strncmp(heading_mode, "post_turn", 9) == 0;
      const bool h2_mode_ok =
        (h2_normal_update && heading_track_validity_gate_apply_to_update_) ||
        (h2_turn_track && heading_track_validity_gate_apply_to_turn_track_) ||
        (h2_post_turn_track && heading_track_validity_gate_apply_to_post_turn_);
      const double h2_since_turn_sec =
        std::isfinite(last_turning_heading_time_sec_)
          ? now_sec - last_turning_heading_time_sec_
          : std::numeric_limits<double>::infinity();
      const bool h2_after_turn_ok =
        heading_track_validity_gate_after_turn_sec_ <= 0.0 ||
        (std::isfinite(h2_since_turn_sec) &&
         h2_since_turn_sec >= 0.0 &&
         h2_since_turn_sec <= heading_track_validity_gate_after_turn_sec_);
      const bool h2_horiz_ok =
        heading_track_validity_gate_min_horizontal_speed_mps_ <= 0.0 ||
        last_mavros_horizontal_speed_mps_ >=
          heading_track_validity_gate_min_horizontal_speed_mps_;
      const bool h2_vert_ok =
        heading_track_validity_gate_max_vertical_speed_mps_ <= 0.0 ||
        std::abs(last_mavros_vertical_speed_mps_) <=
          heading_track_validity_gate_max_vertical_speed_mps_;
      const bool h2_gyro_ok =
        heading_track_validity_gate_max_gyro_deg_s_ <= 0.0 ||
        !std::isfinite(last_imu_gyro_norm_deg_s_) ||
        last_imu_gyro_norm_deg_s_ <= heading_track_validity_gate_max_gyro_deg_s_;
      const bool h2_source_rate_ok =
        heading_track_validity_gate_max_source_yaw_rate_deg_s_ <= 0.0 ||
        !std::isfinite(last_mavros_heading_rate_deg_s_) ||
        std::abs(last_mavros_heading_rate_deg_s_) <=
          heading_track_validity_gate_max_source_yaw_rate_deg_s_;
      const bool h2_residual_ok =
        heading_track_validity_gate_max_residual_deg_ <= 0.0 ||
        residual_abs_deg <= heading_track_validity_gate_max_residual_deg_;

      if (h2_mode_ok &&
          h2_after_turn_ok &&
          h2_horiz_ok &&
          h2_vert_ok &&
          h2_gyro_ok &&
          h2_source_rate_ok &&
          h2_residual_ok) {
        const char* h2_original_heading_mode = heading_mode;
        if (heading_track_validity_gate_action_ == "skip") {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "Skip heading update: H2 track-validity gate blocked %s "
            "(residual=%.2f deg, since_turn=%.2f s, horiz=%.2f m/s, vert=%.2f m/s, "
            "gyro=%.2f deg/s, source_yaw_rate=%.2f deg/s)",
            h2_original_heading_mode,
            yaw_residual_deg,
            h2_since_turn_sec,
            last_mavros_horizontal_speed_mps_,
            last_mavros_vertical_speed_mps_,
            last_imu_gyro_norm_deg_s_,
            last_mavros_heading_rate_deg_s_);
          return;
        }

        const double inflated_heading_std_deg =
          std::max(
            heading_measurement_std_deg,
            std::abs(heading_track_validity_gate_inflated_std_deg_));
        if (inflated_heading_std_deg > heading_measurement_std_deg) {
          heading_measurement_std_deg = inflated_heading_std_deg;
          heading_mode = "h2_track_validity_inflated";
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "Inflated heading update: H2 track-validity gate changed %s std to %.2f deg "
            "(residual=%.2f deg, since_turn=%.2f s, horiz=%.2f m/s, vert=%.2f m/s, "
            "gyro=%.2f deg/s, source_yaw_rate=%.2f deg/s)",
            h2_original_heading_mode,
            heading_measurement_std_deg,
            yaw_residual_deg,
            h2_since_turn_sec,
            last_mavros_horizontal_speed_mps_,
            last_mavros_vertical_speed_mps_,
            last_imu_gyro_norm_deg_s_,
            last_mavros_heading_rate_deg_s_);
        }
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

    if (heading_yaw_gain_hygiene_enable_ && mavros_armed_) {
      const bool hve_flight_mode_ok =
        (heading_yaw_gain_hygiene_apply_mission_ && mavros_mode_ == "AUTO.MISSION") ||
        (heading_yaw_gain_hygiene_apply_rtl_ && mavros_mode_ == "AUTO.RTL");
      const bool hve_armed_cruise_mode =
        std::strcmp(heading_mode, "armed_cruise_track") == 0;
      const bool hve_post_turn_mode =
        std::strncmp(heading_mode, "post_turn", 9) == 0;
      const bool hve_turn_mode =
        std::strcmp(heading_mode, "turn_track") == 0;
      const bool hve_plain_update_mode =
        std::strcmp(heading_mode, "update") == 0 ||
        std::strcmp(heading_mode, "cruise_micro_track") == 0;
      const bool hve_mode_ok =
        (hve_armed_cruise_mode && heading_yaw_gain_hygiene_apply_armed_cruise_) ||
        (hve_post_turn_mode && heading_yaw_gain_hygiene_apply_post_turn_) ||
        (hve_turn_mode && heading_yaw_gain_hygiene_apply_turn_) ||
        (hve_plain_update_mode && heading_yaw_gain_hygiene_apply_update_);
      const bool hve_residual_ok =
        residual_abs_deg >= std::max(0.0, heading_yaw_gain_hygiene_min_residual_deg_) &&
        (heading_yaw_gain_hygiene_max_residual_deg_ <= 0.0 ||
         residual_abs_deg <= heading_yaw_gain_hygiene_max_residual_deg_);
      const bool hve_speed_ok =
        have_fresh_mavros_speed &&
        (heading_yaw_gain_hygiene_min_horizontal_speed_mps_ <= 0.0 ||
         last_mavros_horizontal_speed_mps_ >=
           heading_yaw_gain_hygiene_min_horizontal_speed_mps_) &&
        (heading_yaw_gain_hygiene_max_abs_vertical_speed_mps_ <= 0.0 ||
         std::abs(last_mavros_vertical_speed_mps_) <=
           heading_yaw_gain_hygiene_max_abs_vertical_speed_mps_);
      const bool hve_gyro_ok =
        !std::isfinite(last_imu_gyro_norm_deg_s_) ||
        heading_yaw_gain_hygiene_max_gyro_deg_s_ <= 0.0 ||
        last_imu_gyro_norm_deg_s_ <= heading_yaw_gain_hygiene_max_gyro_deg_s_;
      const bool hve_source_rate_ok =
        !std::isfinite(last_mavros_heading_rate_deg_s_) ||
        heading_yaw_gain_hygiene_max_source_yaw_rate_deg_s_ <= 0.0 ||
        std::abs(last_mavros_heading_rate_deg_s_) <=
          heading_yaw_gain_hygiene_max_source_yaw_rate_deg_s_;
      const double hve_min_period_sec =
        heading_yaw_gain_hygiene_max_rate_hz_ > 0.0
          ? 1.0 / heading_yaw_gain_hygiene_max_rate_hz_
          : 0.0;
      const bool hve_rate_ok =
        !std::isfinite(last_heading_yaw_gain_hygiene_time_sec_) ||
        (now_sec - last_heading_yaw_gain_hygiene_time_sec_) >= hve_min_period_sec;

      if (hve_flight_mode_ok && hve_mode_ok && hve_residual_ok &&
          hve_speed_ok && hve_gyro_ok && hve_source_rate_ok && hve_rate_ok) {
        const double yaw_std_floor_deg =
          std::max(0.1, std::abs(heading_yaw_gain_hygiene_yaw_std_floor_deg_));
        const std::string hve_reason =
          std::string("hve1a_") + heading_mode;
        if (core_->reopenYawCovariance(t_sec, yaw_std_floor_deg, hve_reason)) {
          last_heading_yaw_gain_hygiene_time_sec_ = now_sec;
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "Applied heading yaw-gain hygiene: mode=%s residual=%.2f deg yaw_std_floor=%.2f deg "
            "(horiz=%.2f m/s, vert=%.2f m/s, gyro=%.2f deg/s, source_yaw_rate=%.2f deg/s)",
            heading_mode,
            yaw_residual_deg,
            yaw_std_floor_deg,
            last_mavros_horizontal_speed_mps_,
            last_mavros_vertical_speed_mps_,
            last_imu_gyro_norm_deg_s_,
            last_mavros_heading_rate_deg_s_);
        }
      }
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
    last_heading_residual_abs_deg_ = residual_abs_deg;
    last_heading_yaw_correction_abs_deg_ = yaw_correction_abs_deg;
    last_heading_mode_ = heading_mode;
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
        last_heading_residual_abs_deg_ = std::abs(force_debug_event.residual_after_deg);
        last_heading_yaw_correction_abs_deg_ = force_debug_event.yaw_correction_abs_deg;
        last_heading_mode_ = force_debug_event.heading_mode;
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
    if (!core_processing_enable_) {
      if (!reported_core_processing_disabled_imu_) {
        RCLCPP_WARN(
          get_logger(),
          "Core processing disabled: received first IMU sample on %s but skipped propagation.",
          active_imu_topic_name_.c_str());
        reported_core_processing_disabled_imu_ = true;
      }
      return;
    }

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

    bool feed_core_with_delta = sample.data_is_delta && !bridge_delta_source_gap_as_rates;
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

    if (maybeHoldCoreImuForRateLimit_(
          steady_now.seconds(), feed_core_with_delta, dt, dtheta, dvel)) {
      return;
    }
    if (core_imu_decimation_ > 1 ||
        (std::isfinite(core_max_imu_rate_hz_) && core_max_imu_rate_hz_ > 0.0)) {
      use_segmented_propagation = false;
      segmented_propagation_steps = 1;
    }

    double final_t = std::numeric_limits<double>::quiet_NaN();
    const int propagation_steps = use_segmented_propagation ? segmented_propagation_steps : 1;
    const double dt_step = dt / static_cast<double>(propagation_steps);
    const Eigen::Vector3d dtheta_step = feed_core_with_delta ? (dtheta / propagation_steps) : dtheta;
    const Eigen::Vector3d dvel_step = feed_core_with_delta ? (dvel / propagation_steps) : dvel;
    const HeadingMotionContext propagation_motion_ctx =
      buildHeadingMotionContext_(now().seconds(), false);
    last_turn_rate_propagation_noise_debug_ =
      updateTurnRatePropagationNoiseProbe_(propagation_motion_ctx);
    last_accbias_z_propagation_probe_debug_ =
      updateAccbiasZPropagationProbe_(propagation_motion_ctx);
    const bool adaptive_rq_source_gate_have_speed =
      propagation_motion_ctx.have_fresh_mavros_speed &&
      std::isfinite(last_mavros_horizontal_speed_mps_);
    const double adaptive_rq_source_gate_low_speed_max_mps =
      std::max(0.0, adaptive_rq_source_gate_low_speed_max_mps_);
    const bool adaptive_rq_low_speed_context =
      adaptive_rq_source_gate_have_speed &&
      last_mavros_horizontal_speed_mps_ <= adaptive_rq_source_gate_low_speed_max_mps;
    const bool adaptive_rq_source_gate_allowed =
      !adaptive_rq_source_gate_enable_ ||
      propagation_motion_ctx.turning_now ||
      propagation_motion_ctx.post_turn_context ||
      adaptive_rq_low_speed_context;
    double adaptive_rq_source_confidence = adaptive_rq_source_gate_allowed ? 1.0 : 0.0;
    std::string adaptive_rq_source_gate_reason{"disabled"};
    if (adaptive_rq_source_gate_enable_) {
      if (propagation_motion_ctx.turning_now) {
        adaptive_rq_source_gate_reason = "turning";
      } else if (propagation_motion_ctx.post_turn_context) {
        adaptive_rq_source_gate_reason = "post_turn";
      } else if (adaptive_rq_low_speed_context) {
        adaptive_rq_source_gate_reason = "low_speed";
      } else if (!adaptive_rq_source_gate_have_speed) {
        adaptive_rq_source_gate_reason = "missing_speed";
      } else {
        adaptive_rq_source_gate_reason = "high_speed_straight";
      }
    }
    const double adaptive_rq_process_context_score = std::max(
      (last_turn_rate_propagation_noise_debug_.active
         ? last_turn_rate_propagation_noise_debug_.gyro_score
         : 0.0),
      (last_accbias_z_propagation_probe_debug_.active
         ? last_accbias_z_propagation_probe_debug_.bias_score
         : 0.0));
    core_->setAdaptiveRQProcessContext(
      adaptive_rq_process_context_score > 1.0e-6,
      adaptive_rq_process_context_score);
    core_->setAdaptiveRQSourceGate(
      adaptive_rq_source_gate_allowed,
      adaptive_rq_source_confidence,
      adaptive_rq_source_gate_reason);
    core_->setPropagationNoiseScale(
      std::max(
        last_turn_rate_propagation_noise_debug_.arw_q_scale,
        last_accbias_z_propagation_probe_debug_.arw_q_scale),
      std::max(
        last_turn_rate_propagation_noise_debug_.vrw_q_scale,
        last_accbias_z_propagation_probe_debug_.vrw_q_scale),
      std::max(
        last_turn_rate_propagation_noise_debug_.gyrbias_q_scale,
        last_accbias_z_propagation_probe_debug_.gyrbias_q_scale),
      std::max(
        last_turn_rate_propagation_noise_debug_.accbias_q_scale,
        last_accbias_z_propagation_probe_debug_.accbias_q_scale));

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
    logGnssNisDebugIfNeeded_();
    logStateUpdateDebugIfNeeded_();
    logDtrqRuntimeFeatureDebugIfNeeded_();
    refreshMissionCovHygieneUpdateEffectiveness_();
    logObservationDebugIfNeeded_();
    refreshAdaptiveGnssPositionWeightNisCache_(now().seconds());
    publishState();
    maybePublishShadowRestoreSnapshot_("imu");
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
    if (!core_processing_enable_) {
      if (!reported_core_processing_disabled_gnss_) {
        RCLCPP_WARN(
          get_logger(),
          "Core processing disabled: received first GNSS sample on %s but skipped filter update.",
          active_gnss_topic_name_.c_str());
        reported_core_processing_disabled_gnss_ = true;
      }
      return;
    }

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

    double measurement_latitude = latitude;
    double measurement_longitude = longitude;
    double measurement_altitude = altitude;

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
      const ExperimentResetPose reset_pose =
        firstResetPoseWithOptionalExperimentOffset_(latitude, longitude, altitude, init_yaw);
      (void)core_->reset(
        reset_pose.latitude_deg,
        reset_pose.longitude_deg,
        reset_pose.altitude_m,
        reset_pose.yaw_deg);
      last_turn_rate_propagation_noise_debug_ = TurnRatePropagationNoiseDebug{};
      last_accbias_z_propagation_probe_debug_ = AccbiasZPropagationProbeDebug{};
      core_->setPropagationNoiseScale(1.0, 1.0, 1.0, 1.0);
      core_->setAdaptiveRQProcessContext(false, 0.0);
      core_initialized_ = true;
      reset_this_gnss = true;
      prefer_preserved_yaw_on_next_core_reset_ = false;
      have_completed_armed_flight_since_reset_ = false;
      last_armed_transition_time_sec_ = std::numeric_limits<double>::quiet_NaN();
      last_disarmed_transition_time_sec_ = std::numeric_limits<double>::quiet_NaN();
      post_flight_vertical_cov_reopen_until_sec_ = std::numeric_limits<double>::quiet_NaN();
      last_disarmed_yaw_lock_time_sec_ = now().seconds();
      resetAdaptiveGnssPositionWeightState_();
      resetContextGnssPosFloorState_();
      resetHorizontalConsistencySupervisorState_();
      resetGnssPositionGainResponseState_();
      resetGnssVelocityOutwardDampingState_();
      RCLCPP_INFO(
        this->get_logger(),
        "Core reset by first GNSS fix. yaw=%.1f deg (source=%s, mavros=%s, preserved_core=%s)",
        reset_pose.yaw_deg,
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

    const bool have_native_gnss_velocity =
      native_velocity_valid &&
      std::isfinite(native_vN) &&
      std::isfinite(native_vE) &&
      std::isfinite(native_vD);
    bool position_lag_compensation_active = false;
    double position_lag_compensation_sec = std::numeric_limits<double>::quiet_NaN();
    double position_lag_compensation_n_m = std::numeric_limits<double>::quiet_NaN();
    double position_lag_compensation_e_m = std::numeric_limits<double>::quiet_NaN();
    double position_lag_compensation_u_m = std::numeric_limits<double>::quiet_NaN();
    const double requested_lag_sec = std::abs(gnss_position_lag_compensation_sec_);
    const double max_lag_sec = std::max(0.0, std::abs(gnss_position_lag_compensation_max_sec_));
    const double lag_sec = std::min(requested_lag_sec, max_lag_sec);
    const double native_horizontal_speed_mps =
      have_native_gnss_velocity ? std::hypot(native_vN, native_vE) : std::numeric_limits<double>::quiet_NaN();
    const bool lag_compensation_context =
      gnss_position_lag_compensation_enable_ &&
      use_integrated_time_for_core_ &&
      (!gnss_position_lag_compensation_armed_only_ || mavros_armed_) &&
      have_native_gnss_velocity &&
      std::isfinite(native_horizontal_speed_mps) &&
      native_horizontal_speed_mps >= std::max(0.0, gnss_position_lag_compensation_min_speed_mps_) &&
      lag_sec > 0.0;
    if (lag_compensation_context) {
      const double lat_rad = latitude * M_PI / 180.0;
      const double lon_rad = longitude * M_PI / 180.0;
      const double sin_lat = std::sin(lat_rad);
      const double cos_lat = std::cos(lat_rad);
      const double denom = std::sqrt(1.0 - geo::WGS84_E2 * sin_lat * sin_lat);
      if (std::isfinite(denom) && denom > 0.0 && std::isfinite(cos_lat) && std::abs(cos_lat) > 1e-6) {
        const double rn = geo::WGS84_A / denom;
        const double rm =
          geo::WGS84_A * (1.0 - geo::WGS84_E2) /
          std::pow(1.0 - geo::WGS84_E2 * sin_lat * sin_lat, 1.5);
        const double dN_m = native_vN * lag_sec;
        const double dE_m = native_vE * lag_sec;
        const double dU_m = -native_vD * lag_sec;
        const double lat_comp_rad = lat_rad + dN_m / std::max(1.0, rm + altitude);
        const double lon_comp_rad =
          lon_rad +
          dE_m / std::max(1.0, (rn + altitude) * std::max(1e-6, std::abs(cos_lat)));
        const double alt_comp_m = altitude + dU_m;
        if (std::isfinite(lat_comp_rad) && std::isfinite(lon_comp_rad) && std::isfinite(alt_comp_m)) {
          measurement_latitude = lat_comp_rad * 180.0 / M_PI;
          measurement_longitude = lon_comp_rad * 180.0 / M_PI;
          measurement_altitude = alt_comp_m;
          position_lag_compensation_active = true;
          position_lag_compensation_sec = lag_sec;
          position_lag_compensation_n_m = dN_m;
          position_lag_compensation_e_m = dE_m;
          position_lag_compensation_u_m = dU_m;
          if (!logged_gnss_position_lag_compensation_) {
            RCLCPP_INFO(
              get_logger(),
              "GNSS position lag compensation enabled: lag=%.3fs min_speed=%.3fm/s "
              "first_delta_neu=(%.3f, %.3f, %.3f)m",
              lag_sec,
              std::max(0.0, gnss_position_lag_compensation_min_speed_mps_),
              dN_m,
              dE_m,
              dU_m);
            logged_gnss_position_lag_compensation_ = true;
          }
        }
      }
    }

    last_gnss_lat_rad_ = measurement_latitude  * M_PI/180.0;
    last_gnss_lon_rad_ = measurement_longitude * M_PI/180.0;
    last_gnss_h_m_     = measurement_altitude;

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
                                            double * diff_u_m,
                                            double * diff_n_m = nullptr,
                                            double * diff_e_m = nullptr) {
      if (diff_h_m != nullptr) {
        *diff_h_m = std::numeric_limits<double>::quiet_NaN();
      }
      if (diff_u_m != nullptr) {
        *diff_u_m = std::numeric_limits<double>::quiet_NaN();
      }
      if (diff_n_m != nullptr) {
        *diff_n_m = std::numeric_limits<double>::quiet_NaN();
      }
      if (diff_e_m != nullptr) {
        *diff_e_m = std::numeric_limits<double>::quiet_NaN();
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
      if (diff_n_m != nullptr &&
          std::isfinite(enu_core.y()) &&
          std::isfinite(enu_gnss.y())) {
        *diff_n_m = enu_core.y() - enu_gnss.y();
      }
      if (diff_e_m != nullptr &&
          std::isfinite(enu_core.x()) &&
          std::isfinite(enu_gnss.x())) {
        *diff_e_m = enu_core.x() - enu_gnss.x();
      }
      if (diff_u_m != nullptr &&
          std::isfinite(enu_core.z()) && std::isfinite(enu_gnss.z())) {
        *diff_u_m = enu_core.z() - enu_gnss.z();
      }
    };

    double core_gnss_diff_h_m = std::numeric_limits<double>::quiet_NaN();
    double core_gnss_diff_u_m = std::numeric_limits<double>::quiet_NaN();
    double core_gnss_diff_n_m = std::numeric_limits<double>::quiet_NaN();
    double core_gnss_diff_e_m = std::numeric_limits<double>::quiet_NaN();
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
    compute_core_gnss_diff(
      core_state_before_update,
      &core_gnss_diff_h_m,
      &core_gnss_diff_u_m,
      &core_gnss_diff_n_m,
      &core_gnss_diff_e_m);
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
    const bool terminal_descent_vertical_cov_reopen_active =
      terminal_descent_vertical_cov_reopen_enable_ &&
      motion_ctx.terminal_descent_context;
    bool terminal_descent_vertical_cov_reopen_applied = false;
    if (terminal_descent_vertical_cov_reopen_active) {
      terminal_descent_vertical_cov_reopen_applied = core_->reopenVerticalCovariance(
        std::max(0.01, std::abs(terminal_descent_vertical_cov_reopen_pos_std_m_)),
        std::max(0.01, std::abs(terminal_descent_vertical_cov_reopen_vel_std_mps_)),
        std::max(1e-4, std::abs(terminal_descent_vertical_cov_reopen_accbias_std_z_mps2_)));
      if (terminal_descent_vertical_cov_reopen_applied) {
        refresh_core_vertical_debug();
      }
    }
    const bool any_vertical_cov_reopen_active =
      vertical_cov_reopen_active || post_flight_vertical_cov_reopen_active ||
      terminal_descent_vertical_cov_reopen_active;
    const bool any_vertical_cov_reopen_applied =
      vertical_cov_reopen_applied || post_flight_vertical_cov_reopen_applied ||
      terminal_descent_vertical_cov_reopen_applied;

    Eigen::Vector3d std_ned(-1,-1,-1);
    bool position_override_active = false;
    std::string gnss_position_std_source_label{"unknown"};

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
      gnss_position_std_source_label = "sim_param";
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
      gnss_position_std_source_label = "navsat_covariance";
    } else {
      // 实机模式 + msg无协方差：使用默认实机参数
      std_ned = Eigen::Vector3d(
        std::max(gnss_min_std_m_, gnss_default_std_h_m_),
        std::max(gnss_min_std_m_, gnss_default_std_h_m_),
        std::max(gnss_min_std_m_, gnss_default_std_u_m_));
      gnss_position_std_source_label = "default_param";
    }

    const bool gnss_pos_override_in_mission = mavros_mode_ == "AUTO.MISSION";
    const bool gnss_pos_override_in_rtl = mavros_mode_ == "AUTO.RTL";
    const bool gnss_pos_override_phase_enabled =
      (gnss_pos_override_in_mission &&
       armed_cruise_gnss_pos_override_apply_mission_) ||
      (gnss_pos_override_in_rtl &&
       armed_cruise_gnss_pos_override_apply_rtl_) ||
      (!gnss_pos_override_in_mission &&
       !gnss_pos_override_in_rtl &&
       armed_cruise_gnss_pos_override_apply_other_);
    if (armed_cruise_gnss_pos_override_enable_ &&
        position_override_context &&
        gnss_pos_override_phase_enabled) {
      const double position_std_floor_m = 0.01;
      const double override_std_h_m =
        std::max(position_std_floor_m, std::abs(armed_cruise_gnss_pos_std_h_m_));
      const double override_std_u_m =
        std::max(position_std_floor_m, std::abs(armed_cruise_gnss_pos_std_u_m_));
      std_ned.x() = std::min(std_ned.x(), override_std_h_m);
      std_ned.y() = std::min(std_ned.y(), override_std_h_m);
      std_ned.z() = std::min(std_ned.z(), override_std_u_m);
      position_override_active = true;
      gnss_position_std_source_label += "+armed_cruise_override";
    } else if (armed_cruise_gnss_pos_override_enable_ &&
               position_override_context &&
               !gnss_pos_override_phase_enabled) {
      gnss_position_std_source_label += "+armed_cruise_override_phase_disabled";
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
      gnss_position_std_source_label += "+residual_boost";
    }

    const bool motion_weight_enabled = motion_gnss_pos_weight_enable_;
    const bool motion_weight_phase_enabled =
      (mavros_mode_ == "AUTO.MISSION" && motion_gnss_pos_weight_apply_mission_) ||
      (mavros_mode_ == "AUTO.RTL" && motion_gnss_pos_weight_apply_rtl_);
    const bool motion_weight_speed_valid =
      motion_ctx.have_fresh_mavros_speed &&
      std::isfinite(last_mavros_horizontal_speed_mps_) &&
      std::isfinite(last_mavros_vertical_speed_mps_) &&
      std::abs(last_mavros_vertical_speed_mps_) <=
        std::max(0.0, motion_gnss_pos_weight_max_abs_vspeed_mps_);
    bool motion_weight_active = false;
    bool motion_weight_applied = false;
    std::string motion_weight_reason{"disabled"};
    double motion_weight_speed_score = std::numeric_limits<double>::quiet_NaN();
    double motion_weight_target_std_h_m = std::numeric_limits<double>::quiet_NaN();
    double motion_weight_effective_std_h_m = std::numeric_limits<double>::quiet_NaN();
    if (motion_weight_enabled &&
        mavros_armed_ &&
        motion_weight_phase_enabled &&
        motion_weight_speed_valid) {
      motion_weight_active = true;
      const double start_hspeed =
        std::max(0.0, std::abs(motion_gnss_pos_weight_hspeed_start_mps_));
      const double full_hspeed =
        std::max(start_hspeed + 1.0e-6, std::abs(motion_gnss_pos_weight_hspeed_full_mps_));
      const double speed_score = std::clamp(
        (last_mavros_horizontal_speed_mps_ - start_hspeed) / (full_hspeed - start_hspeed),
        0.0,
        1.0);
      motion_weight_speed_score = speed_score;
      const double std_min_h =
        std::max(0.01, std::abs(motion_gnss_pos_weight_std_min_h_m_));
      const double std_max_h =
        std::max(std_min_h, std::abs(motion_gnss_pos_weight_std_max_h_m_));
      const double scheduled_std_h = std_min_h + speed_score * (std_max_h - std_min_h);
      motion_weight_target_std_h_m = scheduled_std_h;
      if (scheduled_std_h > std_ned.x() || scheduled_std_h > std_ned.y()) {
        std_ned.x() = std::max(std_ned.x(), scheduled_std_h);
        std_ned.y() = std::max(std_ned.y(), scheduled_std_h);
        motion_weight_applied = true;
        motion_weight_reason = "applied";
        gnss_position_std_source_label += "+motion_gnss_pos_weight";
      } else {
        motion_weight_reason = "noop";
        gnss_position_std_source_label += "+motion_gnss_pos_weight_noop";
      }
      motion_weight_effective_std_h_m =
        0.5 * (std::abs(std_ned.x()) + std::abs(std_ned.y()));
    } else if (motion_weight_enabled &&
               !mavros_armed_) {
      motion_weight_reason = "disarmed";
    } else if (motion_weight_enabled &&
               mavros_armed_ &&
               !motion_weight_phase_enabled) {
      motion_weight_reason = "phase_disabled";
      gnss_position_std_source_label += "+motion_gnss_pos_weight_phase_disabled";
    } else if (motion_weight_enabled &&
               mavros_armed_ &&
               motion_weight_phase_enabled &&
               !motion_weight_speed_valid) {
      motion_weight_reason = "motion_gate";
      gnss_position_std_source_label += "+motion_gnss_pos_weight_motion_gate";
    }

    const double context_floor_selected_std_h_m =
      0.5 * (std::abs(std_ned.x()) + std::abs(std_ned.y()));
    const ContextGnssPosFloorDebug context_gnss_pos_floor_debug =
      updateContextGnssPosFloor_(
        now_sec,
        motion_ctx,
        context_floor_selected_std_h_m);
    if (context_gnss_pos_floor_debug.applied &&
        std::isfinite(context_gnss_pos_floor_debug.effective_std_h_m)) {
      std_ned.x() = std::max(std_ned.x(), context_gnss_pos_floor_debug.effective_std_h_m);
      std_ned.y() = std::max(std_ned.y(), context_gnss_pos_floor_debug.effective_std_h_m);
      gnss_position_std_source_label += "+context_gnss_pos_floor";
    } else if (context_gnss_pos_floor_debug.enabled &&
               context_gnss_pos_floor_debug.phase_allowed &&
               context_gnss_pos_floor_debug.active) {
      gnss_position_std_source_label += "+context_gnss_pos_floor_noop";
    } else if (context_gnss_pos_floor_debug.enabled &&
               !context_gnss_pos_floor_debug.phase_allowed) {
      gnss_position_std_source_label += "+context_gnss_pos_floor_phase_disabled";
    }

    const double recovery_selected_std_h_m =
      0.5 * (std::abs(std_ned.x()) + std::abs(std_ned.y()));
    const GnssPosRecoveryWeightDebug gnss_pos_recovery_weight_debug =
      updateGnssPosRecoveryWeight_(
        now_sec,
        motion_ctx,
        last_position_residual_h_m,
        core_gnss_diff_h_m,
        recovery_selected_std_h_m);
    if (gnss_pos_recovery_weight_debug.applied &&
        std::isfinite(gnss_pos_recovery_weight_debug.effective_std_h_m)) {
      std_ned.x() = std::max(std_ned.x(), gnss_pos_recovery_weight_debug.effective_std_h_m);
      std_ned.y() = std::max(std_ned.y(), gnss_pos_recovery_weight_debug.effective_std_h_m);
      gnss_position_std_source_label +=
        gnss_pos_recovery_weight_debug.held ? "+gnss_pos_recovery_weight"
                                            : "+gnss_pos_recovery_weight_decay";
    } else if (gnss_pos_recovery_weight_debug.enabled &&
               gnss_pos_recovery_weight_debug.phase_allowed &&
               !gnss_pos_recovery_weight_debug.motion_ok) {
      gnss_position_std_source_label += "+gnss_pos_recovery_weight_motion_gate";
    } else if (gnss_pos_recovery_weight_debug.enabled &&
               !gnss_pos_recovery_weight_debug.phase_allowed) {
      gnss_position_std_source_label += "+gnss_pos_recovery_weight_phase_disabled";
    }

    const double selected_std_h_m =
      0.5 * (std::abs(std_ned.x()) + std::abs(std_ned.y()));
    const double adaptive_pos_weight_residual_h_m =
      std::isfinite(core_gnss_diff_h_m) ? core_gnss_diff_h_m : last_position_residual_h_m;
    const AdaptiveGnssPosWeightDebug adaptive_pos_weight_debug =
      updateAdaptiveGnssPositionWeight_(
        now_sec,
        motion_ctx,
        adaptive_pos_weight_residual_h_m,
        selected_std_h_m);
    if (adaptive_pos_weight_debug.applied &&
        std::isfinite(adaptive_pos_weight_debug.effective_std_h_m)) {
      std_ned.x() = std::max(std_ned.x(), adaptive_pos_weight_debug.effective_std_h_m);
      std_ned.y() = std::max(std_ned.y(), adaptive_pos_weight_debug.effective_std_h_m);
      gnss_position_std_source_label +=
        adaptive_pos_weight_debug.active ? "+adaptive_pos_weight"
                                         : "+adaptive_pos_weight_decay";
    }

    const double position_gain_response_selected_std_h_m =
      0.5 * (std::abs(std_ned.x()) + std::abs(std_ned.y()));
    const GnssPositionGainResponseDebug gnss_position_gain_response_debug =
      updateGnssPositionGainResponse_(
        motion_ctx,
        core_gnss_diff_h_m,
        core_gnss_diff_n_m,
        core_gnss_diff_e_m,
        std_ned.x(),
        std_ned.y(),
        position_gain_response_selected_std_h_m);
    if (gnss_position_gain_response_debug.applied &&
        std::isfinite(gnss_position_gain_response_debug.effective_std_h_m)) {
      std_ned.x() = std::min(std_ned.x(), gnss_position_gain_response_debug.effective_std_h_m);
      std_ned.y() = std::min(std_ned.y(), gnss_position_gain_response_debug.effective_std_h_m);
      gnss_position_std_source_label += "+gnss_position_gain_response";
    } else if (gnss_position_gain_response_debug.enabled &&
               gnss_position_gain_response_debug.phase_allowed &&
               gnss_position_gain_response_debug.context_ok &&
               gnss_position_gain_response_debug.active) {
      gnss_position_std_source_label += "+gnss_position_gain_response_noop";
    } else if (gnss_position_gain_response_debug.enabled &&
               !gnss_position_gain_response_debug.phase_allowed) {
      gnss_position_std_source_label += "+gnss_position_gain_response_phase_disabled";
    } else if (gnss_position_gain_response_debug.enabled &&
               gnss_position_gain_response_debug.phase_allowed &&
               !gnss_position_gain_response_debug.context_ok) {
      gnss_position_std_source_label += "+gnss_position_gain_response_context_gate";
    }

    const MissionCovHygieneDebug mission_cov_hygiene_debug =
      updateMissionCovHygiene_(
        now_sec,
        motion_ctx,
        adaptive_pos_weight_residual_h_m);

    const double position_response_selected_std_h_m =
      0.5 * (std::abs(std_ned.x()) + std::abs(std_ned.y()));
    const GnssPositionResponseBoostDebug gnss_position_response_boost_debug =
      updateGnssPositionResponseBoost_(
        motion_ctx,
        last_position_residual_h_m,
        core_gnss_diff_h_m,
        position_response_selected_std_h_m);
    if (gnss_position_response_boost_debug.applied &&
        std::isfinite(gnss_position_response_boost_debug.effective_std_h_m)) {
      std_ned.x() = std::min(std_ned.x(), gnss_position_response_boost_debug.effective_std_h_m);
      std_ned.y() = std::min(std_ned.y(), gnss_position_response_boost_debug.effective_std_h_m);
      gnss_position_std_source_label += "+gnss_position_response_boost";
    } else if (gnss_position_response_boost_debug.enabled &&
               gnss_position_response_boost_debug.phase_allowed &&
               gnss_position_response_boost_debug.active) {
      gnss_position_std_source_label += "+gnss_position_response_boost_noop";
    } else if (gnss_position_response_boost_debug.enabled &&
               !gnss_position_response_boost_debug.phase_allowed) {
      gnss_position_std_source_label += "+gnss_position_response_boost_phase_disabled";
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

    const double early_recovery_bias_feedback_armed_time_sec =
      (mavros_armed_ && std::isfinite(last_armed_transition_time_sec_))
        ? now_sec - last_armed_transition_time_sec_
        : std::numeric_limits<double>::quiet_NaN();
    core_->setEarlyRecoveryBiasFeedbackContext(
      early_recovery_bias_feedback_armed_time_sec,
      core_gnss_diff_u_m);

    double gkps_raw_vn_mps = std::numeric_limits<double>::quiet_NaN();
    double gkps_raw_ve_mps = std::numeric_limits<double>::quiet_NaN();
    double gkps_raw_vd_mps = std::numeric_limits<double>::quiet_NaN();
    double gkps_raw_speed_accuracy_mps = std::numeric_limits<double>::quiet_NaN();
    double gkps_raw_vertical_speed_accuracy_mps = std::numeric_limits<double>::quiet_NaN();
    if (have_native_gnss_velocity) {
      gkps_raw_vn_mps = native_vN;
      gkps_raw_ve_mps = native_vE;
      gkps_raw_vd_mps = native_vD;
      gkps_raw_speed_accuracy_mps = native_speed_std_mps;
    } else if (have_prev_gnss_for_vel_) {
      const double dt_gnss = t - prev_gnss_vel_time_;
      if (std::isfinite(dt_gnss) && dt_gnss > 0.05 && dt_gnss < 5.0) {
        const double lat_avg =
          (measurement_latitude * M_PI / 180.0 + prev_gnss_vel_lat_rad_) * 0.5;
        const double dlat_rad =
          measurement_latitude * M_PI / 180.0 - prev_gnss_vel_lat_rad_;
        const double dlon_rad =
          measurement_longitude * M_PI / 180.0 - prev_gnss_vel_lon_rad_;
        const double dh = measurement_altitude - prev_gnss_vel_h_;
        constexpr double kEarthRadiusM = 6371000.0;
        gkps_raw_vn_mps = dlat_rad * kEarthRadiusM / dt_gnss;
        gkps_raw_ve_mps =
          dlon_rad * kEarthRadiusM * std::cos(lat_avg) / dt_gnss;
        gkps_raw_vd_mps = -dh / dt_gnss;
        gkps_raw_speed_accuracy_mps =
          std::sqrt(2.0) * std::abs(std_ned.x()) / dt_gnss;
        gkps_raw_vertical_speed_accuracy_mps =
          std::sqrt(2.0) * std::abs(std_ned.z()) / dt_gnss;
      }
    }
    maybeHandleShadowSupervisorKinematicPredictiveScore_(
      std::isfinite(t_raw) ? t_raw : t,
      measurement_latitude,
      measurement_longitude,
      measurement_altitude,
      0.5 * (std::abs(std_ned.x()) + std::abs(std_ned.y())),
      std::abs(std_ned.z()),
      gkps_raw_vn_mps,
      gkps_raw_ve_mps,
      gkps_raw_vd_mps,
      gkps_raw_speed_accuracy_mps,
      gkps_raw_vertical_speed_accuracy_mps);

    maybeHandleShadowSupervisorPredictiveScore_(
      std::isfinite(t_raw) ? t_raw : t,
      measurement_latitude,
      measurement_longitude,
      measurement_altitude);

    if (!core_->ingestGnss(t, measurement_latitude, measurement_longitude, measurement_altitude, std_ned)) {
      pending_gnss_debug_context_.valid = false;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "core_->ingestGnss() failed");
      return;
    }

    // GNSS 速度观测
    // GNSS velocity derived from consecutive position fixes
    // 这是解决姿态（尤其 yaw）不可观的关键：速度观测让 H 矩阵覆盖了 V 状态，
    // 通过 F 矩阵中的 V↔PHI 耦合，间接使姿态可观。
    // 当 ZUPT 已应用时跳过（ZUPT 的 0.05 m/s std 比位置差分的 ~0.7 m/s 更紧）
    bool native_velocity_used = false;
    bool native_velocity_override_active = false;
    bool terminal_descent_native_velocity_override_active = false;
    bool terminal_descent_horizontal_zero_velocity_active = false;
    bool terminal_descent_horizontal_zero_velocity_applied = false;
    bool residual_velocity_boost_applied = false;
    GnssVelocityOutwardDampingDebug gnss_velocity_outward_damping_debug;
    TurnPostturnNativeVelocityDeweightDebug turn_postturn_native_velocity_deweight_debug;
    const bool native_velocity_broad_outlier_guard_enabled =
      native_gnss_velocity_outlier_guard_enable_;
    const bool native_velocity_low_speed_source_guard_enabled =
      native_gnss_velocity_low_speed_turn_source_guard_enable_;
    const bool native_velocity_outlier_guard_enabled =
      native_velocity_broad_outlier_guard_enabled ||
      native_velocity_low_speed_source_guard_enabled;
    const bool native_velocity_outlier_guard_phase_allowed =
      (mavros_mode_ == "AUTO.MISSION" && native_gnss_velocity_outlier_guard_apply_mission_) ||
      (mavros_mode_ == "AUTO.RTL" && native_gnss_velocity_outlier_guard_apply_rtl_);
    bool native_velocity_outlier_guard_motion_ok = false;
    bool native_velocity_outlier_guard_active = false;
    bool native_velocity_outlier_guard_reweight_applied = false;
    std::string native_velocity_outlier_guard_reason{
      native_velocity_outlier_guard_enabled ? "inactive" : "disabled"};
    std::string native_velocity_outlier_guard_action{"none"};
    double native_velocity_outlier_guard_speed_mismatch_mps =
      std::numeric_limits<double>::quiet_NaN();
    double native_velocity_outlier_guard_core_residual_h_mps =
      std::numeric_limits<double>::quiet_NaN();
    double native_velocity_outlier_guard_reweight_std_h_mps =
      std::numeric_limits<double>::quiet_NaN();
    double native_velocity_outlier_guard_reweight_std_u_mps =
      std::numeric_limits<double>::quiet_NaN();
    double native_velocity_std_h_mps = std::numeric_limits<double>::quiet_NaN();
    double native_velocity_std_u_mps = std::numeric_limits<double>::quiet_NaN();
    if (enable_gnss_velocity_update_ && !zupt_applied && have_native_gnss_velocity) {
      const double native_velocity_h_mps = std::hypot(native_vN, native_vE);
      const bool native_velocity_outlier_guard_speed_ok =
        !std::isfinite(native_gnss_velocity_outlier_guard_min_horizontal_speed_mps_) ||
        native_gnss_velocity_outlier_guard_min_horizontal_speed_mps_ <= 0.0 ||
        (std::isfinite(last_mavros_horizontal_speed_mps_) &&
         last_mavros_horizontal_speed_mps_ >=
           native_gnss_velocity_outlier_guard_min_horizontal_speed_mps_);
      const bool native_velocity_outlier_guard_vertical_ok =
        !std::isfinite(native_gnss_velocity_outlier_guard_max_abs_vertical_speed_mps_) ||
        native_gnss_velocity_outlier_guard_max_abs_vertical_speed_mps_ <= 0.0 ||
        (std::isfinite(last_mavros_vertical_speed_mps_) &&
         std::abs(last_mavros_vertical_speed_mps_) <=
           native_gnss_velocity_outlier_guard_max_abs_vertical_speed_mps_);
      const bool native_velocity_outlier_guard_turning_ok =
        native_gnss_velocity_outlier_guard_apply_turning_context_ ||
        !motion_ctx.turning_now;
      const bool native_velocity_broad_outlier_guard_motion_ok =
        motion_ctx.native_velocity_tightening_context &&
        native_velocity_outlier_guard_turning_ok &&
        native_velocity_outlier_guard_speed_ok &&
        native_velocity_outlier_guard_vertical_ok;
      native_velocity_outlier_guard_motion_ok =
        native_velocity_broad_outlier_guard_motion_ok;
      if (std::isfinite(native_velocity_h_mps) &&
          std::isfinite(last_mavros_horizontal_speed_mps_)) {
        native_velocity_outlier_guard_speed_mismatch_mps =
          std::abs(native_velocity_h_mps - last_mavros_horizontal_speed_mps_);
      }
      const kfcore::State core_state_before_velocity_update = core_->current();
      if (std::isfinite(core_state_before_velocity_update.vN) &&
          std::isfinite(core_state_before_velocity_update.vE)) {
        native_velocity_outlier_guard_core_residual_h_mps =
          std::hypot(
            native_vN - core_state_before_velocity_update.vN,
            native_vE - core_state_before_velocity_update.vE);
      }
      const bool native_velocity_outlier_guard_speed_mismatch_triggered =
        std::isfinite(native_velocity_outlier_guard_speed_mismatch_mps) &&
        native_velocity_outlier_guard_speed_mismatch_mps >=
          std::max(0.0, native_gnss_velocity_outlier_guard_speed_mismatch_mps_);
      const bool native_velocity_outlier_guard_core_residual_triggered =
        std::isfinite(native_velocity_outlier_guard_core_residual_h_mps) &&
        native_velocity_outlier_guard_core_residual_h_mps >=
          std::max(0.0, native_gnss_velocity_outlier_guard_core_residual_h_mps_);
      const bool native_velocity_low_speed_source_guard_speed_ok =
        motion_ctx.have_fresh_mavros_speed &&
        std::isfinite(last_mavros_horizontal_speed_mps_) &&
        (!std::isfinite(
           native_gnss_velocity_low_speed_turn_source_guard_max_horizontal_speed_mps_) ||
         native_gnss_velocity_low_speed_turn_source_guard_max_horizontal_speed_mps_ <= 0.0 ||
         last_mavros_horizontal_speed_mps_ <=
           native_gnss_velocity_low_speed_turn_source_guard_max_horizontal_speed_mps_);
      const bool native_velocity_low_speed_source_guard_gyro_ok =
        std::isfinite(last_imu_gyro_norm_deg_s_) &&
        (!std::isfinite(native_gnss_velocity_low_speed_turn_source_guard_min_gyro_deg_s_) ||
         native_gnss_velocity_low_speed_turn_source_guard_min_gyro_deg_s_ <= 0.0 ||
         last_imu_gyro_norm_deg_s_ >=
           native_gnss_velocity_low_speed_turn_source_guard_min_gyro_deg_s_);
      const bool native_velocity_low_speed_source_guard_motion_ok =
        motion_ctx.turning_now &&
        native_velocity_low_speed_source_guard_speed_ok &&
        native_velocity_low_speed_source_guard_gyro_ok;
      native_velocity_outlier_guard_motion_ok =
        native_velocity_outlier_guard_motion_ok ||
        native_velocity_low_speed_source_guard_motion_ok;
      const bool native_velocity_low_speed_source_guard_residual_triggered =
        std::isfinite(native_velocity_outlier_guard_core_residual_h_mps) &&
        native_velocity_outlier_guard_core_residual_h_mps >=
          std::max(
            0.0,
            native_gnss_velocity_low_speed_turn_source_guard_core_residual_h_mps_);
      const bool native_velocity_broad_outlier_guard_triggered =
        native_velocity_broad_outlier_guard_enabled &&
        mavros_armed_ &&
        native_velocity_outlier_guard_phase_allowed &&
        native_velocity_broad_outlier_guard_motion_ok &&
        (native_velocity_outlier_guard_speed_mismatch_triggered ||
         native_velocity_outlier_guard_core_residual_triggered);
      const bool native_velocity_low_speed_source_guard_triggered =
        native_velocity_low_speed_source_guard_enabled &&
        mavros_armed_ &&
        native_velocity_outlier_guard_phase_allowed &&
        native_velocity_low_speed_source_guard_motion_ok &&
        native_velocity_low_speed_source_guard_residual_triggered;
      if (native_velocity_broad_outlier_guard_triggered) {
        native_velocity_outlier_guard_active = true;
        if (native_velocity_outlier_guard_speed_mismatch_triggered &&
            native_velocity_outlier_guard_core_residual_triggered) {
          native_velocity_outlier_guard_reason = "speed_mismatch_and_core_residual";
        } else if (native_velocity_outlier_guard_speed_mismatch_triggered) {
          native_velocity_outlier_guard_reason = "speed_mismatch";
        } else {
          native_velocity_outlier_guard_reason = "core_residual";
        }
        native_velocity_outlier_guard_action =
          (native_gnss_velocity_outlier_guard_action_ == "reweight") ? "reweight" : "skip";
      } else if (native_velocity_low_speed_source_guard_triggered) {
        native_velocity_outlier_guard_active = true;
        native_velocity_outlier_guard_reason = "low_speed_turn_source_mismatch";
        native_velocity_outlier_guard_action =
          (native_gnss_velocity_outlier_guard_action_ == "reweight") ? "reweight" : "skip";
      } else if (native_velocity_outlier_guard_enabled && !mavros_armed_) {
        native_velocity_outlier_guard_reason = "disarmed";
      } else if (native_velocity_outlier_guard_enabled &&
                 !native_velocity_outlier_guard_phase_allowed) {
        native_velocity_outlier_guard_reason = "phase_disabled";
      } else if (native_velocity_broad_outlier_guard_enabled &&
                 !native_velocity_outlier_guard_turning_ok) {
        native_velocity_outlier_guard_reason = "turn_context_disabled";
      } else if (native_velocity_low_speed_source_guard_enabled &&
                 !native_velocity_low_speed_source_guard_motion_ok &&
                 !native_velocity_broad_outlier_guard_enabled) {
        native_velocity_outlier_guard_reason = "low_speed_turn_source_motion_gate";
      } else if (native_velocity_broad_outlier_guard_enabled &&
                 !native_velocity_broad_outlier_guard_motion_ok) {
        native_velocity_outlier_guard_reason = "motion_gate";
      } else if (native_velocity_outlier_guard_enabled) {
        native_velocity_outlier_guard_reason = "below_threshold";
      }

      double vel_floor_h = std::max(0.03, gnss_vel_std_floor_h_mps_);
      double vel_floor_u = std::max(0.08, gnss_vel_std_floor_u_mps_);
      native_velocity_override_active =
        armed_cruise_native_gnss_vel_override_enable_ &&
        motion_ctx.native_velocity_tightening_context;
      terminal_descent_native_velocity_override_active =
        terminal_descent_native_gnss_vel_override_enable_ &&
        motion_ctx.terminal_descent_context;
      if (terminal_descent_native_velocity_override_active) {
        vel_floor_h = std::max(0.03, std::abs(terminal_descent_native_gnss_vel_std_h_mps_));
        vel_floor_u = std::max(0.03, std::abs(terminal_descent_native_gnss_vel_std_u_mps_));
      } else if (native_velocity_override_active) {
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
      if (native_velocity_outlier_guard_active &&
          native_velocity_outlier_guard_action == "reweight") {
        native_velocity_outlier_guard_reweight_applied = true;
        native_velocity_outlier_guard_reweight_std_h_mps =
          std::max(0.03, std::abs(native_gnss_velocity_outlier_guard_reweight_std_h_mps_));
        native_velocity_outlier_guard_reweight_std_u_mps =
          std::max(0.08, std::abs(native_gnss_velocity_outlier_guard_reweight_std_u_mps_));
        vel_floor_h = native_velocity_outlier_guard_reweight_std_h_mps;
        vel_floor_u = native_velocity_outlier_guard_reweight_std_u_mps;
      }
      const double native_std =
        (std::isfinite(native_speed_std_mps) && native_speed_std_mps > 0.0)
          ? native_speed_std_mps * std::max(0.0, native_gnss_speed_std_scale_)
          : std::numeric_limits<double>::quiet_NaN();
      Eigen::Vector3d vel_std;
      if (terminal_descent_native_velocity_override_active ||
          native_velocity_override_active ||
          residual_velocity_boost_applied) {
        vel_std = Eigen::Vector3d(vel_floor_h, vel_floor_h, vel_floor_u);
      } else {
        vel_std = Eigen::Vector3d(
          std::max(vel_floor_h, std::isfinite(native_std) ? native_std : vel_floor_h),
          std::max(vel_floor_h, std::isfinite(native_std) ? native_std : vel_floor_h),
          std::max(vel_floor_u, std::isfinite(native_std) ? native_std : vel_floor_u));
      }
      gnss_velocity_outward_damping_debug =
        updateGnssVelocityOutwardDamping_(
          motion_ctx,
          core_state_before_velocity_update,
          native_vN,
          native_vE,
          core_gnss_diff_n_m,
          core_gnss_diff_e_m,
          core_gnss_diff_h_m,
          vel_std.x());
      if (gnss_velocity_outward_damping_debug.applied &&
          std::isfinite(gnss_velocity_outward_damping_debug.effective_std_h_mps)) {
        vel_std.x() = std::min(
          vel_std.x(),
          gnss_velocity_outward_damping_debug.effective_std_h_mps);
        vel_std.y() = std::min(
          vel_std.y(),
          gnss_velocity_outward_damping_debug.effective_std_h_mps);
      }
      turn_postturn_native_velocity_deweight_debug =
        updateTurnPostturnNativeVelocityDeweight_(
          motion_ctx,
          core_state_before_velocity_update,
          native_vN,
          native_vE,
          core_gnss_diff_n_m,
          core_gnss_diff_e_m,
          core_gnss_diff_h_m,
          vel_std.x());
      if (turn_postturn_native_velocity_deweight_debug.applied &&
          std::isfinite(turn_postturn_native_velocity_deweight_debug.effective_std_h_mps)) {
        vel_std.x() = std::max(
          vel_std.x(),
          turn_postturn_native_velocity_deweight_debug.effective_std_h_mps);
        vel_std.y() = std::max(
          vel_std.y(),
          turn_postturn_native_velocity_deweight_debug.effective_std_h_mps);
      }

      if (native_velocity_outlier_guard_active &&
          native_velocity_outlier_guard_action == "skip") {
        RCLCPP_DEBUG(
          get_logger(),
          "Skipping native GNSS velocity update: reason=%s speed_mismatch=%.3f core_residual=%.3f",
          native_velocity_outlier_guard_reason.c_str(),
          native_velocity_outlier_guard_speed_mismatch_mps,
          native_velocity_outlier_guard_core_residual_h_mps);
      } else if (native_velocity_outlier_guard_reweight_applied) {
        RCLCPP_DEBUG(
          get_logger(),
          "Reweighting native GNSS velocity update: reason=%s speed_mismatch=%.3f core_residual=%.3f std_h=%.3f std_u=%.3f",
          native_velocity_outlier_guard_reason.c_str(),
          native_velocity_outlier_guard_speed_mismatch_mps,
          native_velocity_outlier_guard_core_residual_h_mps,
          vel_std.x(),
          vel_std.z());
      } else if (!logged_first_native_gnss_velocity_sample_) {
        RCLCPP_INFO(
          get_logger(),
          "Using native GNSS velocity observation: vN=%.3f vE=%.3f vD=%.3f std_h=%.3f std_u=%.3f",
          native_vN, native_vE, native_vD, vel_std.x(), vel_std.z());
        logged_first_native_gnss_velocity_sample_ = true;
      }
      maybeHandleShadowSupervisorVelocityPredictiveScore_(
        std::isfinite(t_raw) ? t_raw : t,
        native_vN,
        native_vE,
        native_vD,
        native_speed_std_mps,
        std::numeric_limits<double>::quiet_NaN());
      if (!native_velocity_outlier_guard_active ||
          native_velocity_outlier_guard_action == "reweight") {
        core_->ingestGnssVel(t, native_vN, native_vE, native_vD, vel_std);
        native_velocity_used = true;
        native_velocity_std_h_mps = vel_std.x();
        native_velocity_std_u_mps = vel_std.z();
      }
    } else if (enable_gnss_velocity_update_ && !zupt_applied && have_prev_gnss_for_vel_) {
      const double dt_gnss = t - prev_gnss_vel_time_;
      if (std::isfinite(dt_gnss) && dt_gnss > 0.05 && dt_gnss < 5.0) {
        const double lat_avg = (measurement_latitude * M_PI/180.0 + prev_gnss_vel_lat_rad_) * 0.5;
        const double dlat_rad = measurement_latitude * M_PI/180.0 - prev_gnss_vel_lat_rad_;
        const double dlon_rad = measurement_longitude * M_PI/180.0 - prev_gnss_vel_lon_rad_;
        const double dh = measurement_altitude - prev_gnss_vel_h_;

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

          maybeHandleShadowSupervisorVelocityPredictiveScore_(
            std::isfinite(t_raw) ? t_raw : t,
            vN,
            vE,
            vD,
            vel_std_h,
            vel_std_u);
          core_->ingestGnssVel(t, vN, vE, vD, vel_std);
        }
      }
    }
    terminal_descent_horizontal_zero_velocity_active =
      enable_gnss_velocity_update_ &&
      terminal_descent_horizontal_zero_vel_enable_ &&
      motion_ctx.terminal_descent_context &&
      std::isfinite(last_mavros_horizontal_speed_mps_) &&
      last_mavros_horizontal_speed_mps_ <=
        std::max(0.0, terminal_descent_horizontal_zero_vel_max_hspeed_mps_);
    if (terminal_descent_horizontal_zero_velocity_active) {
      const double std_h_mps =
        std::max(0.01, std::abs(terminal_descent_horizontal_zero_vel_std_h_mps_));
      const double std_u_mps =
        std::max(0.01, std::abs(terminal_descent_horizontal_zero_vel_std_u_mps_));
      double vD_observation_mps =
        std::isfinite(native_vD) ? native_vD : core_state_before_update.vD;
      if (!std::isfinite(vD_observation_mps)) {
        vD_observation_mps = 0.0;
      }
      const Eigen::Vector3d zero_horiz_vel_std(std_h_mps, std_h_mps, std_u_mps);
      core_->ingestGnssVel(t, 0.0, 0.0, vD_observation_mps, zero_horiz_vel_std);
      terminal_descent_horizontal_zero_velocity_applied = true;
    }
    pending_gnss_debug_context_.valid = true;
    pending_gnss_debug_context_.native_velocity_valid = have_native_gnss_velocity;
    pending_gnss_debug_context_.native_velocity_used = native_velocity_used;
    pending_gnss_debug_context_.native_velocity_override_active =
      native_velocity_override_active || terminal_descent_native_velocity_override_active;
    pending_gnss_debug_context_.native_velocity_outlier_guard_enabled =
      native_velocity_outlier_guard_enabled;
    pending_gnss_debug_context_.native_velocity_outlier_guard_phase_allowed =
      native_velocity_outlier_guard_phase_allowed;
    pending_gnss_debug_context_.native_velocity_outlier_guard_motion_ok =
      native_velocity_outlier_guard_motion_ok;
    pending_gnss_debug_context_.native_velocity_outlier_guard_active =
      native_velocity_outlier_guard_active;
    pending_gnss_debug_context_.native_velocity_outlier_guard_reason =
      native_velocity_outlier_guard_reason;
    pending_gnss_debug_context_.native_velocity_outlier_guard_action =
      native_velocity_outlier_guard_action;
    pending_gnss_debug_context_.native_velocity_outlier_guard_reweight_applied =
      native_velocity_outlier_guard_reweight_applied;
    pending_gnss_debug_context_.native_velocity_outlier_guard_speed_mismatch_mps =
      native_velocity_outlier_guard_speed_mismatch_mps;
    pending_gnss_debug_context_.native_velocity_outlier_guard_core_residual_h_mps =
      native_velocity_outlier_guard_core_residual_h_mps;
    pending_gnss_debug_context_.native_velocity_outlier_guard_reweight_std_h_mps =
      native_velocity_outlier_guard_reweight_std_h_mps;
    pending_gnss_debug_context_.native_velocity_outlier_guard_reweight_std_u_mps =
      native_velocity_outlier_guard_reweight_std_u_mps;
    pending_gnss_debug_context_.terminal_descent_native_velocity_override_active =
      terminal_descent_native_velocity_override_active;
    pending_gnss_debug_context_.terminal_descent_horizontal_zero_velocity_active =
      terminal_descent_horizontal_zero_velocity_active;
    pending_gnss_debug_context_.terminal_descent_horizontal_zero_velocity_applied =
      terminal_descent_horizontal_zero_velocity_applied;
    pending_gnss_debug_context_.velocity_residual_boost_active = residual_velocity_boost_applied;
    pending_gnss_debug_context_.gnss_velocity_outward_damping_enabled =
      gnss_velocity_outward_damping_debug.enabled;
    pending_gnss_debug_context_.gnss_velocity_outward_damping_phase_allowed =
      gnss_velocity_outward_damping_debug.phase_allowed;
    pending_gnss_debug_context_.gnss_velocity_outward_damping_motion_ok =
      gnss_velocity_outward_damping_debug.motion_ok;
    pending_gnss_debug_context_.gnss_velocity_outward_damping_triggered =
      gnss_velocity_outward_damping_debug.triggered;
    pending_gnss_debug_context_.gnss_velocity_outward_damping_held =
      gnss_velocity_outward_damping_debug.held;
    pending_gnss_debug_context_.gnss_velocity_outward_damping_active =
      gnss_velocity_outward_damping_debug.active;
    pending_gnss_debug_context_.gnss_velocity_outward_damping_applied =
      gnss_velocity_outward_damping_debug.applied;
    pending_gnss_debug_context_.gnss_velocity_outward_damping_reason =
      gnss_velocity_outward_damping_debug.reason;
    pending_gnss_debug_context_.gnss_velocity_outward_damping_core_gnss_diff_n_m =
      gnss_velocity_outward_damping_debug.core_gnss_diff_n_m;
    pending_gnss_debug_context_.gnss_velocity_outward_damping_core_gnss_diff_e_m =
      gnss_velocity_outward_damping_debug.core_gnss_diff_e_m;
    pending_gnss_debug_context_.gnss_velocity_outward_damping_core_gnss_diff_h_m =
      gnss_velocity_outward_damping_debug.core_gnss_diff_h_m;
    pending_gnss_debug_context_.gnss_velocity_outward_damping_core_native_residual_n_mps =
      gnss_velocity_outward_damping_debug.core_native_velocity_residual_n_mps;
    pending_gnss_debug_context_.gnss_velocity_outward_damping_core_native_residual_e_mps =
      gnss_velocity_outward_damping_debug.core_native_velocity_residual_e_mps;
    pending_gnss_debug_context_.gnss_velocity_outward_damping_core_native_residual_h_mps =
      gnss_velocity_outward_damping_debug.core_native_velocity_residual_h_mps;
    pending_gnss_debug_context_.gnss_velocity_outward_damping_radial_outward_mps =
      gnss_velocity_outward_damping_debug.radial_outward_mps;
    pending_gnss_debug_context_.gnss_velocity_outward_damping_radial_score =
      gnss_velocity_outward_damping_debug.radial_score;
    pending_gnss_debug_context_.gnss_velocity_outward_damping_hold_score =
      gnss_velocity_outward_damping_debug.hold_score;
    pending_gnss_debug_context_.gnss_velocity_outward_damping_persistent_count =
      gnss_velocity_outward_damping_debug.persistent_count;
    pending_gnss_debug_context_.gnss_velocity_outward_damping_hold_remaining_updates =
      gnss_velocity_outward_damping_debug.hold_remaining_updates;
    pending_gnss_debug_context_.gnss_velocity_outward_damping_selected_std_h_mps =
      gnss_velocity_outward_damping_debug.selected_std_h_mps;
    pending_gnss_debug_context_.gnss_velocity_outward_damping_target_std_h_mps =
      gnss_velocity_outward_damping_debug.target_std_h_mps;
    pending_gnss_debug_context_.gnss_velocity_outward_damping_effective_std_h_mps =
      gnss_velocity_outward_damping_debug.effective_std_h_mps;
    pending_gnss_debug_context_.gnss_velocity_outward_damping_multiplier_h =
      gnss_velocity_outward_damping_debug.multiplier_h;
    pending_gnss_debug_context_.turn_postturn_native_velocity_deweight_enabled =
      turn_postturn_native_velocity_deweight_debug.enabled;
    pending_gnss_debug_context_.turn_postturn_native_velocity_deweight_phase_allowed =
      turn_postturn_native_velocity_deweight_debug.phase_allowed;
    pending_gnss_debug_context_.turn_postturn_native_velocity_deweight_context_ok =
      turn_postturn_native_velocity_deweight_debug.context_ok;
    pending_gnss_debug_context_.turn_postturn_native_velocity_deweight_motion_ok =
      turn_postturn_native_velocity_deweight_debug.motion_ok;
    pending_gnss_debug_context_.turn_postturn_native_velocity_deweight_triggered =
      turn_postturn_native_velocity_deweight_debug.triggered;
    pending_gnss_debug_context_.turn_postturn_native_velocity_deweight_active =
      turn_postturn_native_velocity_deweight_debug.active;
    pending_gnss_debug_context_.turn_postturn_native_velocity_deweight_applied =
      turn_postturn_native_velocity_deweight_debug.applied;
    pending_gnss_debug_context_.turn_postturn_native_velocity_deweight_reason =
      turn_postturn_native_velocity_deweight_debug.reason;
    pending_gnss_debug_context_.turn_postturn_native_velocity_deweight_core_gnss_diff_n_m =
      turn_postturn_native_velocity_deweight_debug.core_gnss_diff_n_m;
    pending_gnss_debug_context_.turn_postturn_native_velocity_deweight_core_gnss_diff_e_m =
      turn_postturn_native_velocity_deweight_debug.core_gnss_diff_e_m;
    pending_gnss_debug_context_.turn_postturn_native_velocity_deweight_core_gnss_diff_h_m =
      turn_postturn_native_velocity_deweight_debug.core_gnss_diff_h_m;
    pending_gnss_debug_context_.turn_postturn_native_velocity_deweight_core_native_residual_n_mps =
      turn_postturn_native_velocity_deweight_debug.core_native_velocity_residual_n_mps;
    pending_gnss_debug_context_.turn_postturn_native_velocity_deweight_core_native_residual_e_mps =
      turn_postturn_native_velocity_deweight_debug.core_native_velocity_residual_e_mps;
    pending_gnss_debug_context_.turn_postturn_native_velocity_deweight_core_native_residual_h_mps =
      turn_postturn_native_velocity_deweight_debug.core_native_velocity_residual_h_mps;
    pending_gnss_debug_context_.turn_postturn_native_velocity_deweight_radial_mps =
      turn_postturn_native_velocity_deweight_debug.radial_mps;
    pending_gnss_debug_context_.turn_postturn_native_velocity_deweight_radial_abs_score =
      turn_postturn_native_velocity_deweight_debug.radial_abs_score;
    pending_gnss_debug_context_.turn_postturn_native_velocity_deweight_residual_score =
      turn_postturn_native_velocity_deweight_debug.residual_score;
    pending_gnss_debug_context_.turn_postturn_native_velocity_deweight_score =
      turn_postturn_native_velocity_deweight_debug.score;
    pending_gnss_debug_context_.turn_postturn_native_velocity_deweight_persistent_count =
      turn_postturn_native_velocity_deweight_debug.persistent_count;
    pending_gnss_debug_context_.turn_postturn_native_velocity_deweight_selected_std_h_mps =
      turn_postturn_native_velocity_deweight_debug.selected_std_h_mps;
    pending_gnss_debug_context_.turn_postturn_native_velocity_deweight_target_std_h_mps =
      turn_postturn_native_velocity_deweight_debug.target_std_h_mps;
    pending_gnss_debug_context_.turn_postturn_native_velocity_deweight_effective_std_h_mps =
      turn_postturn_native_velocity_deweight_debug.effective_std_h_mps;
    pending_gnss_debug_context_.turn_postturn_native_velocity_deweight_multiplier_h =
      turn_postturn_native_velocity_deweight_debug.multiplier_h;
    pending_gnss_debug_context_.position_override_active = position_override_active;
    pending_gnss_debug_context_.position_residual_boost_active = residual_position_boost_active;
    pending_gnss_debug_context_.gnss_position_response_boost_enabled =
      gnss_position_response_boost_debug.enabled;
    pending_gnss_debug_context_.gnss_position_response_boost_phase_allowed =
      gnss_position_response_boost_debug.phase_allowed;
    pending_gnss_debug_context_.gnss_position_response_boost_motion_ok =
      gnss_position_response_boost_debug.motion_ok;
    pending_gnss_debug_context_.gnss_position_response_boost_triggered =
      gnss_position_response_boost_debug.triggered;
    pending_gnss_debug_context_.gnss_position_response_boost_active =
      gnss_position_response_boost_debug.active;
    pending_gnss_debug_context_.gnss_position_response_boost_applied =
      gnss_position_response_boost_debug.applied;
    pending_gnss_debug_context_.gnss_position_response_boost_reason =
      gnss_position_response_boost_debug.reason;
    pending_gnss_debug_context_.gnss_position_response_boost_residual_h_m =
      gnss_position_response_boost_debug.residual_h_m;
    pending_gnss_debug_context_.gnss_position_response_boost_last_residual_h_m =
      gnss_position_response_boost_debug.last_residual_h_m;
    pending_gnss_debug_context_.gnss_position_response_boost_core_gnss_diff_h_m =
      gnss_position_response_boost_debug.core_gnss_diff_h_m;
    pending_gnss_debug_context_.gnss_position_response_boost_residual_score =
      gnss_position_response_boost_debug.residual_score;
    pending_gnss_debug_context_.gnss_position_response_boost_persistent_count =
      gnss_position_response_boost_debug.persistent_count;
    pending_gnss_debug_context_.gnss_position_response_boost_selected_std_h_m =
      gnss_position_response_boost_debug.selected_std_h_m;
    pending_gnss_debug_context_.gnss_position_response_boost_target_std_h_m =
      gnss_position_response_boost_debug.target_std_h_m;
    pending_gnss_debug_context_.gnss_position_response_boost_effective_std_h_m =
      gnss_position_response_boost_debug.effective_std_h_m;
    pending_gnss_debug_context_.gnss_position_response_boost_multiplier_h =
      gnss_position_response_boost_debug.multiplier_h;
    pending_gnss_debug_context_.gnss_position_gain_response_enabled =
      gnss_position_gain_response_debug.enabled;
    pending_gnss_debug_context_.gnss_position_gain_response_phase_allowed =
      gnss_position_gain_response_debug.phase_allowed;
    pending_gnss_debug_context_.gnss_position_gain_response_context_ok =
      gnss_position_gain_response_debug.context_ok;
    pending_gnss_debug_context_.gnss_position_gain_response_motion_ok =
      gnss_position_gain_response_debug.motion_ok;
    pending_gnss_debug_context_.gnss_position_gain_response_gain_ok =
      gnss_position_gain_response_debug.gain_ok;
    pending_gnss_debug_context_.gnss_position_gain_response_triggered =
      gnss_position_gain_response_debug.triggered;
    pending_gnss_debug_context_.gnss_position_gain_response_active =
      gnss_position_gain_response_debug.active;
    pending_gnss_debug_context_.gnss_position_gain_response_applied =
      gnss_position_gain_response_debug.applied;
    pending_gnss_debug_context_.gnss_position_gain_response_reason =
      gnss_position_gain_response_debug.reason;
    pending_gnss_debug_context_.gnss_position_gain_response_residual_h_m =
      gnss_position_gain_response_debug.residual_h_m;
    pending_gnss_debug_context_.gnss_position_gain_response_hnis_h =
      gnss_position_gain_response_debug.hnis_h;
    pending_gnss_debug_context_.gnss_position_gain_response_prev_dx_pos_h_m =
      gnss_position_gain_response_debug.prev_dx_pos_h_m;
    pending_gnss_debug_context_.gnss_position_gain_response_prev_residual_h_m =
      gnss_position_gain_response_debug.prev_residual_h_m;
    pending_gnss_debug_context_.gnss_position_gain_response_prev_dx_over_residual_h =
      gnss_position_gain_response_debug.prev_dx_over_residual_h;
    pending_gnss_debug_context_.gnss_position_gain_response_residual_score =
      gnss_position_gain_response_debug.residual_score;
    pending_gnss_debug_context_.gnss_position_gain_response_hnis_score =
      gnss_position_gain_response_debug.hnis_score;
    pending_gnss_debug_context_.gnss_position_gain_response_gain_score =
      gnss_position_gain_response_debug.gain_score;
    pending_gnss_debug_context_.gnss_position_gain_response_score =
      gnss_position_gain_response_debug.score;
    pending_gnss_debug_context_.gnss_position_gain_response_persistent_count =
      gnss_position_gain_response_debug.persistent_count;
    pending_gnss_debug_context_.gnss_position_gain_response_selected_std_h_m =
      gnss_position_gain_response_debug.selected_std_h_m;
    pending_gnss_debug_context_.gnss_position_gain_response_target_std_h_m =
      gnss_position_gain_response_debug.target_std_h_m;
    pending_gnss_debug_context_.gnss_position_gain_response_effective_std_h_m =
      gnss_position_gain_response_debug.effective_std_h_m;
    pending_gnss_debug_context_.gnss_position_gain_response_multiplier_h =
      gnss_position_gain_response_debug.multiplier_h;
    pending_gnss_debug_context_.motion_gnss_pos_weight_enabled = motion_weight_enabled;
    pending_gnss_debug_context_.motion_gnss_pos_weight_phase_allowed =
      motion_weight_phase_enabled;
    pending_gnss_debug_context_.motion_gnss_pos_weight_motion_ok =
      motion_weight_speed_valid;
    pending_gnss_debug_context_.motion_gnss_pos_weight_active = motion_weight_active;
    pending_gnss_debug_context_.motion_gnss_pos_weight_applied = motion_weight_applied;
    pending_gnss_debug_context_.motion_gnss_pos_weight_reason = motion_weight_reason;
    pending_gnss_debug_context_.motion_gnss_pos_weight_speed_score =
      motion_weight_speed_score;
    pending_gnss_debug_context_.motion_gnss_pos_weight_target_std_h_m =
      motion_weight_target_std_h_m;
    pending_gnss_debug_context_.motion_gnss_pos_weight_effective_std_h_m =
      motion_weight_effective_std_h_m;
    pending_gnss_debug_context_.gnss_pos_recovery_weight_enabled =
      gnss_pos_recovery_weight_debug.enabled;
    pending_gnss_debug_context_.gnss_pos_recovery_weight_phase_allowed =
      gnss_pos_recovery_weight_debug.phase_allowed;
    pending_gnss_debug_context_.gnss_pos_recovery_weight_motion_ok =
      gnss_pos_recovery_weight_debug.motion_ok;
    pending_gnss_debug_context_.gnss_pos_recovery_weight_triggered =
      gnss_pos_recovery_weight_debug.triggered;
    pending_gnss_debug_context_.gnss_pos_recovery_weight_held =
      gnss_pos_recovery_weight_debug.held;
    pending_gnss_debug_context_.gnss_pos_recovery_weight_active =
      gnss_pos_recovery_weight_debug.active;
    pending_gnss_debug_context_.gnss_pos_recovery_weight_applied =
      gnss_pos_recovery_weight_debug.applied;
    pending_gnss_debug_context_.gnss_pos_recovery_weight_reason =
      gnss_pos_recovery_weight_debug.reason;
    pending_gnss_debug_context_.gnss_pos_recovery_weight_residual_h_m =
      gnss_pos_recovery_weight_debug.residual_h_m;
    pending_gnss_debug_context_.gnss_pos_recovery_weight_core_gnss_diff_h_m =
      gnss_pos_recovery_weight_debug.core_gnss_diff_h_m;
    pending_gnss_debug_context_.gnss_pos_recovery_weight_residual_score =
      gnss_pos_recovery_weight_debug.residual_score;
    pending_gnss_debug_context_.gnss_pos_recovery_weight_core_score =
      gnss_pos_recovery_weight_debug.core_score;
    pending_gnss_debug_context_.gnss_pos_recovery_weight_score =
      gnss_pos_recovery_weight_debug.score;
    pending_gnss_debug_context_.gnss_pos_recovery_weight_persistent_count =
      gnss_pos_recovery_weight_debug.persistent_count;
    pending_gnss_debug_context_.gnss_pos_recovery_weight_hold_remaining_sec =
      gnss_pos_recovery_weight_debug.hold_remaining_sec;
    pending_gnss_debug_context_.gnss_pos_recovery_weight_selected_std_h_m =
      gnss_pos_recovery_weight_debug.selected_std_h_m;
    pending_gnss_debug_context_.gnss_pos_recovery_weight_target_std_h_m =
      gnss_pos_recovery_weight_debug.target_std_h_m;
    pending_gnss_debug_context_.gnss_pos_recovery_weight_state_std_h_m =
      gnss_pos_recovery_weight_debug.state_std_h_m;
    pending_gnss_debug_context_.gnss_pos_recovery_weight_effective_std_h_m =
      gnss_pos_recovery_weight_debug.effective_std_h_m;
    pending_gnss_debug_context_.gnss_pos_recovery_weight_multiplier_h =
      gnss_pos_recovery_weight_debug.multiplier_h;
    pending_gnss_debug_context_.context_gnss_pos_floor_enabled =
      context_gnss_pos_floor_debug.enabled;
    pending_gnss_debug_context_.context_gnss_pos_floor_phase_allowed =
      context_gnss_pos_floor_debug.phase_allowed;
    pending_gnss_debug_context_.context_gnss_pos_floor_active =
      context_gnss_pos_floor_debug.active;
    pending_gnss_debug_context_.context_gnss_pos_floor_applied =
      context_gnss_pos_floor_debug.applied;
    pending_gnss_debug_context_.context_gnss_pos_floor_reason =
      context_gnss_pos_floor_debug.reason;
    pending_gnss_debug_context_.context_gnss_pos_floor_selected_std_h_m =
      context_gnss_pos_floor_debug.selected_std_h_m;
    pending_gnss_debug_context_.context_gnss_pos_floor_target_floor_h_m =
      context_gnss_pos_floor_debug.target_floor_h_m;
    pending_gnss_debug_context_.context_gnss_pos_floor_state_floor_h_m =
      context_gnss_pos_floor_debug.floor_state_h_m;
    pending_gnss_debug_context_.context_gnss_pos_floor_effective_std_h_m =
      context_gnss_pos_floor_debug.effective_std_h_m;
    pending_gnss_debug_context_.context_gnss_pos_floor_multiplier_h =
      context_gnss_pos_floor_debug.multiplier_h;
    pending_gnss_debug_context_.adaptive_gnss_pos_weight_enabled =
      adaptive_pos_weight_debug.enabled;
    pending_gnss_debug_context_.adaptive_gnss_pos_weight_active =
      adaptive_pos_weight_debug.active;
    pending_gnss_debug_context_.adaptive_gnss_pos_weight_applied =
      adaptive_pos_weight_debug.applied;
    pending_gnss_debug_context_.adaptive_gnss_pos_weight_phase =
      adaptive_pos_weight_debug.phase;
    pending_gnss_debug_context_.adaptive_gnss_pos_weight_reason =
      adaptive_pos_weight_debug.reason;
    pending_gnss_debug_context_.adaptive_gnss_pos_weight_residual_h_m =
      adaptive_pos_weight_debug.residual_h_m;
    pending_gnss_debug_context_.adaptive_gnss_pos_weight_selected_std_h_m =
      adaptive_pos_weight_debug.selected_std_h_m;
    pending_gnss_debug_context_.adaptive_gnss_pos_weight_target_floor_h_m =
      adaptive_pos_weight_debug.target_floor_h_m;
    pending_gnss_debug_context_.adaptive_gnss_pos_weight_floor_state_h_m =
      adaptive_pos_weight_debug.floor_state_h_m;
    pending_gnss_debug_context_.adaptive_gnss_pos_weight_effective_std_h_m =
      adaptive_pos_weight_debug.effective_std_h_m;
    pending_gnss_debug_context_.adaptive_gnss_pos_weight_multiplier_h =
      adaptive_pos_weight_debug.multiplier_h;
    pending_gnss_debug_context_.adaptive_gnss_pos_weight_persistent_count =
      adaptive_pos_weight_debug.persistent_count;
    pending_gnss_debug_context_.mission_cov_hygiene_enabled =
      mission_cov_hygiene_debug.enabled;
    pending_gnss_debug_context_.mission_cov_hygiene_phase_allowed =
      mission_cov_hygiene_debug.phase_allowed;
    pending_gnss_debug_context_.mission_cov_hygiene_accepted_recent =
      mission_cov_hygiene_debug.accepted_recent;
    pending_gnss_debug_context_.mission_cov_hygiene_triggered =
      mission_cov_hygiene_debug.triggered;
    pending_gnss_debug_context_.mission_cov_hygiene_active =
      mission_cov_hygiene_debug.active;
    pending_gnss_debug_context_.mission_cov_hygiene_applied =
      mission_cov_hygiene_debug.applied;
    pending_gnss_debug_context_.mission_cov_hygiene_phase =
      mission_cov_hygiene_debug.phase;
    pending_gnss_debug_context_.mission_cov_hygiene_reason =
      mission_cov_hygiene_debug.reason;
    pending_gnss_debug_context_.mission_cov_hygiene_residual_h_m =
      mission_cov_hygiene_debug.residual_h_m;
    pending_gnss_debug_context_.mission_cov_hygiene_hnis_h =
      mission_cov_hygiene_debug.hnis_h;
    pending_gnss_debug_context_.mission_cov_hygiene_score =
      mission_cov_hygiene_debug.score;
    pending_gnss_debug_context_.mission_cov_hygiene_nis_score =
      mission_cov_hygiene_debug.nis_score;
    pending_gnss_debug_context_.mission_cov_hygiene_residual_score =
      mission_cov_hygiene_debug.residual_score;
    pending_gnss_debug_context_.mission_cov_hygiene_cov_score =
      mission_cov_hygiene_debug.cov_score;
    pending_gnss_debug_context_.mission_cov_hygiene_weak_score =
      mission_cov_hygiene_debug.weak_score;
    pending_gnss_debug_context_.mission_cov_hygiene_persistent_count =
      mission_cov_hygiene_debug.persistent_count;
    pending_gnss_debug_context_.mission_cov_hygiene_pos_std_h_before_m =
      mission_cov_hygiene_debug.pos_std_h_before_m;
    pending_gnss_debug_context_.mission_cov_hygiene_floor_h_m =
      mission_cov_hygiene_debug.floor_h_m;
    pending_gnss_debug_context_.mission_cov_hygiene_pos_std_h_after_m =
      mission_cov_hygiene_debug.pos_std_h_after_m;
    pending_gnss_debug_context_.mission_cov_hygiene_dx_pos_h_prev_m =
      mission_cov_hygiene_debug.dx_pos_h_prev_m;
    pending_gnss_debug_context_.mission_cov_hygiene_dx_pos_over_resid_prev =
      mission_cov_hygiene_debug.dx_pos_over_resid_prev;
    pending_gnss_debug_context_.mission_cov_hygiene_p_nn_before =
      mission_cov_hygiene_debug.p_nn_before;
    pending_gnss_debug_context_.mission_cov_hygiene_p_ee_before =
      mission_cov_hygiene_debug.p_ee_before;
    pending_gnss_debug_context_.mission_cov_hygiene_p_ne_before =
      mission_cov_hygiene_debug.p_ne_before;
    pending_gnss_debug_context_.mission_cov_hygiene_p_nn_after =
      mission_cov_hygiene_debug.p_nn_after;
    pending_gnss_debug_context_.mission_cov_hygiene_p_ee_after =
      mission_cov_hygiene_debug.p_ee_after;
    pending_gnss_debug_context_.mission_cov_hygiene_p_ne_after =
      mission_cov_hygiene_debug.p_ne_after;
    pending_gnss_debug_context_.turn_rate_propagation_noise_probe_enabled =
      last_turn_rate_propagation_noise_debug_.enabled;
    pending_gnss_debug_context_.turn_rate_propagation_noise_probe_phase_allowed =
      last_turn_rate_propagation_noise_debug_.phase_allowed;
    pending_gnss_debug_context_.turn_rate_propagation_noise_probe_motion_ok =
      last_turn_rate_propagation_noise_debug_.motion_ok;
    pending_gnss_debug_context_.turn_rate_propagation_noise_probe_triggered =
      last_turn_rate_propagation_noise_debug_.triggered;
    pending_gnss_debug_context_.turn_rate_propagation_noise_probe_active =
      last_turn_rate_propagation_noise_debug_.active;
    pending_gnss_debug_context_.turn_rate_propagation_noise_probe_applied =
      last_turn_rate_propagation_noise_debug_.applied;
    pending_gnss_debug_context_.turn_rate_propagation_noise_probe_reason =
      last_turn_rate_propagation_noise_debug_.reason;
    pending_gnss_debug_context_.turn_rate_propagation_noise_probe_gyro_score =
      last_turn_rate_propagation_noise_debug_.gyro_score;
    pending_gnss_debug_context_.turn_rate_propagation_noise_probe_arw_q_scale =
      last_turn_rate_propagation_noise_debug_.arw_q_scale;
    pending_gnss_debug_context_.turn_rate_propagation_noise_probe_vrw_q_scale =
      last_turn_rate_propagation_noise_debug_.vrw_q_scale;
    pending_gnss_debug_context_.turn_rate_propagation_noise_probe_gyrbias_q_scale =
      last_turn_rate_propagation_noise_debug_.gyrbias_q_scale;
    pending_gnss_debug_context_.turn_rate_propagation_noise_probe_accbias_q_scale =
      last_turn_rate_propagation_noise_debug_.accbias_q_scale;
    pending_gnss_debug_context_.accbias_z_propagation_probe_enabled =
      last_accbias_z_propagation_probe_debug_.enabled;
    pending_gnss_debug_context_.accbias_z_propagation_probe_apply_noise_scale =
      last_accbias_z_propagation_probe_debug_.apply_noise_scale;
    pending_gnss_debug_context_.accbias_z_propagation_probe_have_bias =
      last_accbias_z_propagation_probe_debug_.have_bias;
    pending_gnss_debug_context_.accbias_z_propagation_probe_phase_allowed =
      last_accbias_z_propagation_probe_debug_.phase_allowed;
    pending_gnss_debug_context_.accbias_z_propagation_probe_motion_ok =
      last_accbias_z_propagation_probe_debug_.motion_ok;
    pending_gnss_debug_context_.accbias_z_propagation_probe_triggered =
      last_accbias_z_propagation_probe_debug_.triggered;
    pending_gnss_debug_context_.accbias_z_propagation_probe_active =
      last_accbias_z_propagation_probe_debug_.active;
    pending_gnss_debug_context_.accbias_z_propagation_probe_applied =
      last_accbias_z_propagation_probe_debug_.applied;
    pending_gnss_debug_context_.accbias_z_propagation_probe_reason =
      last_accbias_z_propagation_probe_debug_.reason;
    pending_gnss_debug_context_.accbias_z_propagation_probe_accbias_z_mps2 =
      last_accbias_z_propagation_probe_debug_.accbias_z_mps2;
    pending_gnss_debug_context_.accbias_z_propagation_probe_bias_score =
      last_accbias_z_propagation_probe_debug_.bias_score;
    pending_gnss_debug_context_.accbias_z_propagation_probe_arw_q_scale =
      last_accbias_z_propagation_probe_debug_.arw_q_scale;
    pending_gnss_debug_context_.accbias_z_propagation_probe_vrw_q_scale =
      last_accbias_z_propagation_probe_debug_.vrw_q_scale;
    pending_gnss_debug_context_.accbias_z_propagation_probe_gyrbias_q_scale =
      last_accbias_z_propagation_probe_debug_.gyrbias_q_scale;
    pending_gnss_debug_context_.accbias_z_propagation_probe_accbias_q_scale =
      last_accbias_z_propagation_probe_debug_.accbias_q_scale;
    pending_gnss_debug_context_.position_lag_compensation_active =
      position_lag_compensation_active;
    pending_gnss_debug_context_.position_lag_compensation_sec = position_lag_compensation_sec;
    pending_gnss_debug_context_.position_lag_compensation_n_m = position_lag_compensation_n_m;
    pending_gnss_debug_context_.position_lag_compensation_e_m = position_lag_compensation_e_m;
    pending_gnss_debug_context_.position_lag_compensation_u_m = position_lag_compensation_u_m;
    pending_gnss_debug_context_.vertical_cov_reopen_active = any_vertical_cov_reopen_active;
    pending_gnss_debug_context_.vertical_cov_reopen_applied = any_vertical_cov_reopen_applied;
    pending_gnss_debug_context_.post_flight_vertical_cov_reopen_active =
      post_flight_vertical_cov_reopen_active;
    pending_gnss_debug_context_.post_flight_vertical_cov_reopen_applied =
      post_flight_vertical_cov_reopen_applied;
    pending_gnss_debug_context_.terminal_descent_vertical_cov_reopen_active =
      terminal_descent_vertical_cov_reopen_active;
    pending_gnss_debug_context_.terminal_descent_vertical_cov_reopen_applied =
      terminal_descent_vertical_cov_reopen_applied;
    pending_gnss_debug_context_.armed = mavros_armed_;
    pending_gnss_debug_context_.have_fresh_speed = motion_ctx.have_fresh_mavros_speed;
    pending_gnss_debug_context_.turning_now = motion_ctx.turning_now;
    pending_gnss_debug_context_.post_turn_context = motion_ctx.post_turn_context;
    pending_gnss_debug_context_.armed_cruise_context = motion_ctx.armed_cruise_force_relock_context;
    pending_gnss_debug_context_.native_velocity_tightening_context =
      motion_ctx.native_velocity_tightening_context;
    pending_gnss_debug_context_.terminal_descent_context =
      motion_ctx.terminal_descent_context;
    if (motion_ctx.turning_now || motion_ctx.post_turn_context) {
      phase_error_memory_last_turnpost_context_sec_ = now_sec;
    }
    pending_gnss_debug_context_.phase_error_memory_recent_turnpost_age_sec =
      std::isfinite(phase_error_memory_last_turnpost_context_sec_)
        ? now_sec - phase_error_memory_last_turnpost_context_sec_
        : std::numeric_limits<double>::quiet_NaN();
    pending_gnss_debug_context_.phase_error_memory_recent_turnpost_active =
      std::isfinite(pending_gnss_debug_context_.phase_error_memory_recent_turnpost_age_sec) &&
      pending_gnss_debug_context_.phase_error_memory_recent_turnpost_age_sec <=
        std::max(0.0, std::abs(phase_error_memory_debug_recent_turnpost_hold_sec_));
    pending_gnss_debug_context_.last_turning_age_sec =
      std::isfinite(last_turning_heading_time_sec_)
        ? now_sec - last_turning_heading_time_sec_
        : std::numeric_limits<double>::quiet_NaN();
    pending_gnss_debug_context_.post_turn_hold_remaining_sec =
      std::isfinite(post_turn_hold_end_time_sec_) ? post_turn_hold_end_time_sec_ - now_sec
                                                  : std::numeric_limits<double>::quiet_NaN();
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
    pending_gnss_debug_context_.gnss_source_time_sec = last_gnss_source_time_sec_;
    pending_gnss_debug_context_.gnss_rx_ros_time_sec = last_gnss_rx_ros_time_sec_;
    pending_gnss_debug_context_.core_time_before_update_sec = last_core_time_;
    pending_gnss_debug_context_.gnss_source_age_at_update_sec =
      std::isfinite(last_gnss_source_time_sec_) ? now_sec - last_gnss_source_time_sec_
                                                : std::numeric_limits<double>::quiet_NaN();
    pending_gnss_debug_context_.gnss_rx_age_at_update_sec =
      std::isfinite(last_gnss_rx_ros_time_sec_) ? now_sec - last_gnss_rx_ros_time_sec_
                                                : std::numeric_limits<double>::quiet_NaN();
    pending_gnss_debug_context_.core_time_before_update_minus_update_sec =
      std::isfinite(last_core_time_) ? last_core_time_ - t
                                     : std::numeric_limits<double>::quiet_NaN();
    pending_gnss_debug_context_.gnss_update_minus_source_sec =
      std::isfinite(last_gnss_source_time_sec_) ? t - last_gnss_source_time_sec_
                                                : std::numeric_limits<double>::quiet_NaN();
    pending_gnss_debug_context_.update_time_minus_ros_sec = t - now_sec;
    pending_gnss_debug_context_.armed_time_sec =
      (mavros_armed_ && std::isfinite(last_armed_transition_time_sec_))
        ? now_sec - last_armed_transition_time_sec_
        : std::numeric_limits<double>::quiet_NaN();
    const double latest_heading_update_age_sec =
      std::isfinite(last_heading_update_time_sec_)
        ? now_sec - last_heading_update_time_sec_
        : std::numeric_limits<double>::quiet_NaN();
    pending_gnss_debug_context_.latest_heading_update_age_sec =
      latest_heading_update_age_sec;
    pending_gnss_debug_context_.latest_heading_residual_abs_deg =
      std::isfinite(latest_heading_update_age_sec) ? last_heading_residual_abs_deg_
                                                   : std::numeric_limits<double>::quiet_NaN();
    pending_gnss_debug_context_.latest_heading_yaw_correction_abs_deg =
      std::isfinite(latest_heading_update_age_sec) ? last_heading_yaw_correction_abs_deg_
                                                   : std::numeric_limits<double>::quiet_NaN();
    pending_gnss_debug_context_.latest_heading_mode = last_heading_mode_;
    pending_gnss_debug_context_.mavros_mode = mavros_mode_;
    pending_gnss_debug_context_.gnss_position_std_source_label =
      gnss_position_std_source_label;
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
    prev_gnss_vel_lat_rad_ = measurement_latitude  * M_PI/180.0;
    prev_gnss_vel_lon_rad_ = measurement_longitude * M_PI/180.0;
    prev_gnss_vel_h_       = measurement_altitude;
    prev_gnss_vel_time_    = t;
    have_prev_gnss_for_vel_ = true;

    last_core_time_ = t;
    last_gnss_time_sec_ = t;

    if (publish_state_after_gnss_update_) {
      publishState();
    }
    maybePublishShadowRestoreSnapshot_("gnss");
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
    updateDtrqGnssQualityFromNavSatFix_(
      *msg,
      t_raw,
      now_sec,
      have_fresh_native_sensor_gps_velocity,
      native_sensor_gps_vN_mps_,
      native_sensor_gps_vE_mps_,
      native_sensor_gps_vD_mps_);
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
    const double now_sec = now().seconds();
    updateDtrqGnssQualityFromSensorGps_(*msg, t_raw, now_sec);
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
    const double t_raw = use_node_time_for_core_
      ? rawTimeSecNow_()
      : static_cast<double>(msg->timestamp_sample) * 1e-6;
    updateDtrqGnssQualityFromSensorGps_(*msg, t_raw, now().seconds());
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
    updateDtrqGnssQualityFromVehicleGlobalPosition_(*msg, t_raw, now().seconds());
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
  bool shouldPublishRawOdom_()
  {
    if (raw_odom_decimation_ <= 1) {
      return true;
    }
    const bool publish =
      (raw_odom_publish_counter_ % static_cast<std::uint64_t>(raw_odom_decimation_)) == 0;
    ++raw_odom_publish_counter_;
    return publish;
  }

  Eigen::Vector2d px4SphereProjectionScales_() const
  {
    const double sin_lat = std::sin(origin_lat_);
    const double one_minus_e2_sin2 =
      1.0 - geo::WGS84_E2 * sin_lat * sin_lat;
    if (!(one_minus_e2_sin2 > 0.0)) {
      return Eigen::Vector2d::Ones();
    }
    const double denom = std::sqrt(one_minus_e2_sin2);
    const double rn = geo::WGS84_A / denom;
    const double rm = geo::WGS84_A * (1.0 - geo::WGS84_E2) /
                      (one_minus_e2_sin2 * denom);
    constexpr double kPx4SphereRadiusM = 6371000.0;
    if (!(rn > 0.0) || !(rm > 0.0)) {
      return Eigen::Vector2d::Ones();
    }
    return {kPx4SphereRadiusM / rn, kPx4SphereRadiusM / rm};
  }

  Eigen::Vector3d applyPx4SphereProjection_(const Eigen::Vector3d & enu) const
  {
    const Eigen::Vector2d scales = px4SphereProjectionScales_();
    return {enu.x() * scales.x(), enu.y() * scales.y(), enu.z()};
  }

  Eigen::Vector3d applyPx4SphereProjectionAlpha_(
    const Eigen::Vector3d & enu,
    double alpha) const
  {
    const Eigen::Vector3d projected = applyPx4SphereProjection_(enu);
    const double finite_alpha = std::isfinite(alpha) ? alpha : 1.0;
    return enu + finite_alpha * (projected - enu);
  }

  Eigen::Vector3d applyPublishProjection_(
    const Eigen::Vector3d & enu,
    bool segment_timing_gate_projection_active,
    bool accbias_z_history_projection_active) const
  {
    if (publish_px4_sphere_projection_) {
      return applyPx4SphereProjection_(enu);
    }
    if (segment_timing_gate_projection_active) {
      return applyPx4SphereProjectionAlpha_(enu, segment_timing_gate_projection_alpha_);
    }
    if (accbias_z_history_projection_active) {
      return applyPx4SphereProjectionAlpha_(enu, accbias_z_history_projection_alpha_);
    }
    return enu;
  }

  bool accbiasZHistoryProjectionPhaseAllowed_() const
  {
    if (!mavros_armed_) {
      return false;
    }
    if (mavros_mode_ == "AUTO.MISSION") {
      return accbias_z_history_projection_apply_mission_;
    }
    if (mavros_mode_.find("RTL") != std::string::npos) {
      return accbias_z_history_projection_apply_rtl_;
    }
    return accbias_z_history_projection_apply_other_;
  }

  double accbiasZHistoryProjectionDeepFrac_() const
  {
    if (accbias_z_history_projection_total_count_ <= 0) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    return static_cast<double>(accbias_z_history_projection_deep_count_) /
      static_cast<double>(accbias_z_history_projection_total_count_);
  }

  void resetAccbiasZHistoryProjection_()
  {
    accbias_z_history_projection_total_count_ = 0;
    accbias_z_history_projection_deep_count_ = 0;
    accbias_z_history_projection_last_sequence_ = 0;
    accbias_z_history_projection_latest_before_mps2_ =
      std::numeric_limits<double>::quiet_NaN();
    accbias_z_history_projection_latest_after_mps2_ =
      std::numeric_limits<double>::quiet_NaN();
    last_accbias_z_history_projection_active_ = false;
    last_accbias_z_history_projection_phase_allowed_ = false;
    last_accbias_z_history_projection_have_history_ = false;
  }

  void refreshAccbiasZHistoryProjection_()
  {
    if (!core_) {
      return;
    }
    const kfcore::StateUpdateDebug event = core_->lastGnssPositionStateUpdateDebug();
    if (!event.valid || event.sequence == 0 ||
        event.sequence == accbias_z_history_projection_last_sequence_ ||
        event.event_type != "gnss_position" || !event.applied ||
        !event.accbias_before_mps2.allFinite()) {
      return;
    }
    accbias_z_history_projection_last_sequence_ = event.sequence;
    accbias_z_history_projection_latest_before_mps2_ = event.accbias_before_mps2.z();
    accbias_z_history_projection_latest_after_mps2_ =
      event.accbias_after_mps2.allFinite()
        ? event.accbias_after_mps2.z()
        : std::numeric_limits<double>::quiet_NaN();

    const double armed_time_sec =
      (mavros_armed_ && std::isfinite(last_armed_transition_time_sec_))
        ? now().seconds() - last_armed_transition_time_sec_
        : std::numeric_limits<double>::quiet_NaN();
    if (!std::isfinite(armed_time_sec) ||
        armed_time_sec < accbias_z_history_projection_history_start_sec_) {
      return;
    }

    ++accbias_z_history_projection_total_count_;
    if (accbias_z_history_projection_latest_before_mps2_ <=
        accbias_z_history_projection_deep_threshold_mps2_) {
      ++accbias_z_history_projection_deep_count_;
    }
  }

  bool accbiasZHistoryProjectionActive_(
    bool odom_uses_gnss_pose,
    bool have_core_enu)
  {
    last_accbias_z_history_projection_phase_allowed_ =
      accbiasZHistoryProjectionPhaseAllowed_();
    const double history_frac = accbiasZHistoryProjectionDeepFrac_();
    last_accbias_z_history_projection_have_history_ =
      std::isfinite(history_frac) &&
      accbias_z_history_projection_total_count_ > 0;
    last_accbias_z_history_projection_active_ =
      accbias_z_history_projection_enable_ &&
      last_accbias_z_history_projection_phase_allowed_ &&
      last_accbias_z_history_projection_have_history_ &&
      history_frac >= accbias_z_history_projection_frac_threshold_ &&
      have_core_enu &&
      !odom_uses_gnss_pose;
    return last_accbias_z_history_projection_active_;
  }

  double selectedPublishProjectionAlpha_() const
  {
    if (publish_px4_sphere_projection_) {
      return 1.0;
    }
    if (last_segment_timing_gate_projection_active_) {
      return std::isfinite(segment_timing_gate_projection_alpha_)
        ? segment_timing_gate_projection_alpha_
        : 1.0;
    }
    if (last_accbias_z_history_projection_active_) {
      return std::isfinite(accbias_z_history_projection_alpha_)
        ? accbias_z_history_projection_alpha_
        : 1.0;
    }
    return 0.0;
  }

  bool publishProjectionNeedsDebugState_() const
  {
    return state_update_debug_csv_.is_open() || dtrq_runtime_feature_debug_csv_.is_open();
  }

  void updateStateUpdateDebugEnabled_()
  {
    if (core_) {
      core_->setStateUpdateDebugEnabled(publishProjectionNeedsDebugState_());
    }
  }

  void configureEarlyRecoveryBiasFeedbackCore_()
  {
    if (!core_) {
      return;
    }
    core_->configureEarlyRecoveryBiasFeedback(
      early_recovery_bias_feedback_debug_enable_,
      early_recovery_bias_feedback_apply_enable_,
      early_recovery_bias_feedback_history_sec_,
      early_recovery_bias_feedback_min_armed_time_sec_,
      early_recovery_bias_feedback_max_armed_time_sec_,
      early_recovery_bias_feedback_ba_z_mean_max_mps2_,
      early_recovery_bias_feedback_residual_u_mean_max_m_,
      early_recovery_bias_feedback_core_gnss_u_mean_min_m_,
      early_recovery_bias_feedback_dx_ba_z_sum_max_mps2_,
      early_recovery_bias_feedback_min_history_rows_,
      early_recovery_bias_feedback_negative_dx_scale_);
  }

  bool segmentTimingGateNeedsUpdate_() const
  {
    return segment_timing_gate_debug_csv_.is_open() || segment_timing_gate_projection_enable_;
  }

  bool segmentTimingGateGnssSourceAgeAllowed_(double gnss_source_age_mean_sec) const
  {
    if (segment_timing_gate_gnss_source_age_max_sec_ < 0.0) {
      return true;
    }
    return std::isfinite(gnss_source_age_mean_sec) &&
      gnss_source_age_mean_sec <= segment_timing_gate_gnss_source_age_max_sec_;
  }

  bool segmentTimingGateCoreGnssAlongMinAllowed_(double core_gnss_along_min_m) const
  {
    if (!segment_timing_gate_core_gnss_along_min_enable_) {
      return true;
    }
    return std::isfinite(core_gnss_along_min_m) &&
      core_gnss_along_min_m <= segment_timing_gate_core_gnss_along_min_threshold_m_;
  }

  bool segmentTimingGateProjectionPhaseAllowed_() const
  {
    if (!mavros_armed_) {
      return false;
    }
    if (mavros_mode_ == "AUTO.MISSION") {
      return segment_timing_gate_projection_apply_mission_;
    }
    if (mavros_mode_.find("RTL") != std::string::npos) {
      return segment_timing_gate_projection_apply_rtl_;
    }
    return segment_timing_gate_projection_apply_other_;
  }

  bool segmentTimingGateProjectionActive_(bool odom_uses_gnss_pose, bool have_core_enu) const
  {
    return segment_timing_gate_projection_enable_ &&
           segment_timing_gate_current_active_ &&
           segmentTimingGateProjectionPhaseAllowed_() &&
           have_core_enu &&
           !odom_uses_gnss_pose;
  }

  void resetPublishCoreStampOffset_(const char* reason)
  {
    publish_core_stamp_offset_initialized_ = false;
    publish_core_to_ros_offset_sec_ = std::numeric_limits<double>::quiet_NaN();
    if (publish_stamp_mode_ == "core_fixed_offset") {
      RCLCPP_INFO(get_logger(), "Publish core stamp offset reset: %s", reason);
    }
  }

  rclcpp::Time timeFromSeconds_(double stamp_sec) const
  {
    if (!std::isfinite(stamp_sec) || stamp_sec <= 0.0) {
      return now();
    }
    const double ns_double = stamp_sec * 1.0e9;
    const int64_t ns = static_cast<int64_t>(std::llround(ns_double));
    return rclcpp::Time(ns, this->get_clock()->get_clock_type());
  }

  rclcpp::Time selectPublishStamp_()
  {
    const rclcpp::Time now_stamp = now();
    const double now_sec = now_stamp.seconds();
    last_publish_stamp_selected_minus_now_sec_ = 0.0;
    last_publish_stamp_observed_offset_sec_ = std::numeric_limits<double>::quiet_NaN();
    last_publish_stamp_applied_mode_ = "ros_now";

    if (publish_stamp_mode_ != "core_fixed_offset" || !std::isfinite(last_core_time_)) {
      return now_stamp;
    }

    const double observed_offset = now_sec - last_core_time_;
    last_publish_stamp_observed_offset_sec_ = observed_offset;
    if (!publish_core_stamp_offset_initialized_) {
      if (!mavros_armed_) {
        return now_stamp;
      }
      publish_core_to_ros_offset_sec_ = observed_offset;
      publish_core_stamp_offset_initialized_ = true;
      RCLCPP_INFO(
        get_logger(),
        "Publish core stamp offset initialized: offset=%.6f sec",
        publish_core_to_ros_offset_sec_);
    }

    double selected_sec = last_core_time_ + publish_core_to_ros_offset_sec_ +
      publish_core_stamp_offset_bias_sec_;
    if (selected_sec > now_sec + publish_core_stamp_max_future_sec_) {
      selected_sec = now_sec + publish_core_stamp_max_future_sec_;
    }
    if (selected_sec < now_sec - publish_core_stamp_max_past_sec_) {
      selected_sec = now_sec - publish_core_stamp_max_past_sec_;
    }

    last_publish_stamp_selected_minus_now_sec_ = selected_sec - now_sec;
    last_publish_stamp_applied_mode_ = "core_fixed_offset";
    return timeFromSeconds_(selected_sec);
  }

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
          resetHorizontalConsistencySupervisorState_();

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

    // Use the gate state observed on the previous publish. The current publish
    // timestamp lag is only known after selectPublishStamp_(), which historically
    // happened later in this function.
    const bool segment_timing_gate_projection_active =
      segmentTimingGateProjectionActive_(odom_uses_gnss_pose, have_core_enu);
    const bool accbias_z_history_projection_active =
      accbiasZHistoryProjectionActive_(odom_uses_gnss_pose, have_core_enu);
    last_segment_timing_gate_projection_active_ = segment_timing_gate_projection_active;
    last_publish_projection_active_ =
      publish_px4_sphere_projection_ ||
      segment_timing_gate_projection_active ||
      accbias_z_history_projection_active;
    if (publish_px4_sphere_projection_) {
      last_publish_projection_action_ = "global_px4_sphere";
    } else if (segment_timing_gate_projection_active) {
      last_publish_projection_action_ = "segment_timing_gate_alpha";
    } else if (accbias_z_history_projection_active) {
      last_publish_projection_action_ = "accbias_z_history_alpha";
    } else {
      last_publish_projection_action_ = "raw";
    }

    // 业务 odom/path 使用当前发布位置；raw odom 始终保留 core 原始解，便于区分 fallback。
    Eigen::Vector3d enu_vis =
      applyPublishProjection_(
        enu,
        segment_timing_gate_projection_active,
        accbias_z_history_projection_active);
    last_enu_ = enu_vis;
    have_last_enu_ = true;

    tf2::Quaternion q_tf;
    r_enu_from_flu.getRotation(q_tf);
    q_tf.normalize();
    if (!isfinite_d(q_tf.x()) || !isfinite_d(q_tf.y()) ||
        !isfinite_d(q_tf.z()) || !isfinite_d(q_tf.w())) return;

    geometry_msgs::msg::Quaternion q_msg;
    q_msg.x = q_tf.x(); q_msg.y = q_tf.y(); q_msg.z = q_tf.z(); q_msg.w = q_tf.w();

    const auto stamp = selectPublishStamp_();

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

    const Eigen::Vector3d velocity_enu_raw(st.vE, st.vN, -st.vD);
    const Eigen::Vector3d velocity_enu_vis =
      applyPublishProjection_(
        velocity_enu_raw,
        segment_timing_gate_projection_active,
        accbias_z_history_projection_active);

    auto make_odom_msg = [&](const Eigen::Vector3d & position_enu,
                             const Eigen::Vector3d & velocity_enu) {
      nav_msgs::msg::Odometry od;
      od.header.stamp = stamp;
      od.header.frame_id = map_frame_;
      od.child_frame_id  = base_frame_;
      od.pose.pose.position.x = position_enu.x();
      od.pose.pose.position.y = position_enu.y();
      od.pose.pose.position.z = position_enu.z();
      od.pose.pose.orientation = q_msg;
      // GIEngine: [vN, vE, vD] -> ENU: x=E, y=N, z=U
      od.twist.twist.linear.x = velocity_enu.x();
      od.twist.twist.linear.y = velocity_enu.y();
      od.twist.twist.linear.z = velocity_enu.z();
      return od;
    };

    nav_msgs::msg::Odometry od = make_odom_msg(enu_vis, velocity_enu_vis);
    odom_pub_->publish(od);
    maybePublishShadowSupervisorPerformanceProxy_(od);
    logShadowSupervisorFsmDebug_(od);
    if (have_core_enu && shouldPublishRawOdom_()) {
      odom_raw_pub_->publish(make_odom_msg(enu_core, velocity_enu_raw));
    }
    logStatePublishDebug_(
      stamp,
      odom_uses_gnss_pose,
      have_core_enu,
      enu_core,
      enu,
      enu_vis,
      have_gnss_enu,
      enu_gnss,
      st.vN,
      st.vE,
      st.vD);
    refreshAccbiasZHistoryProjection_();
    updateSegmentTimingGateDebug_(
      now().seconds(),
      have_core_enu,
      enu_core,
      have_gnss_enu,
      enu_gnss,
      st.vN,
      st.vE);

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
    bool terminal_descent_context{false};
    bool terminal_descent_mode_ok{false};
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
    bool native_velocity_outlier_guard_enabled{false};
    bool native_velocity_outlier_guard_phase_allowed{false};
    bool native_velocity_outlier_guard_motion_ok{false};
    bool native_velocity_outlier_guard_active{false};
    bool native_velocity_outlier_guard_reweight_applied{false};
    bool velocity_residual_boost_active{false};
    bool gnss_velocity_outward_damping_enabled{false};
    bool gnss_velocity_outward_damping_phase_allowed{false};
    bool gnss_velocity_outward_damping_motion_ok{false};
    bool gnss_velocity_outward_damping_triggered{false};
    bool gnss_velocity_outward_damping_held{false};
    bool gnss_velocity_outward_damping_active{false};
    bool gnss_velocity_outward_damping_applied{false};
    bool turn_postturn_native_velocity_deweight_enabled{false};
    bool turn_postturn_native_velocity_deweight_phase_allowed{false};
    bool turn_postturn_native_velocity_deweight_context_ok{false};
    bool turn_postturn_native_velocity_deweight_motion_ok{false};
    bool turn_postturn_native_velocity_deweight_triggered{false};
    bool turn_postturn_native_velocity_deweight_active{false};
    bool turn_postturn_native_velocity_deweight_applied{false};
    bool position_override_active{false};
    bool position_residual_boost_active{false};
    bool gnss_position_response_boost_enabled{false};
    bool gnss_position_response_boost_phase_allowed{false};
    bool gnss_position_response_boost_motion_ok{false};
    bool gnss_position_response_boost_triggered{false};
    bool gnss_position_response_boost_active{false};
    bool gnss_position_response_boost_applied{false};
    bool gnss_position_gain_response_enabled{false};
    bool gnss_position_gain_response_phase_allowed{false};
    bool gnss_position_gain_response_context_ok{false};
    bool gnss_position_gain_response_motion_ok{false};
    bool gnss_position_gain_response_gain_ok{false};
    bool gnss_position_gain_response_triggered{false};
    bool gnss_position_gain_response_active{false};
    bool gnss_position_gain_response_applied{false};
    bool motion_gnss_pos_weight_enabled{false};
    bool motion_gnss_pos_weight_phase_allowed{false};
    bool motion_gnss_pos_weight_motion_ok{false};
    bool motion_gnss_pos_weight_active{false};
    bool motion_gnss_pos_weight_applied{false};
    bool gnss_pos_recovery_weight_enabled{false};
    bool gnss_pos_recovery_weight_phase_allowed{false};
    bool gnss_pos_recovery_weight_motion_ok{false};
    bool gnss_pos_recovery_weight_triggered{false};
    bool gnss_pos_recovery_weight_held{false};
    bool gnss_pos_recovery_weight_active{false};
    bool gnss_pos_recovery_weight_applied{false};
    bool context_gnss_pos_floor_enabled{false};
    bool context_gnss_pos_floor_phase_allowed{false};
    bool context_gnss_pos_floor_active{false};
    bool context_gnss_pos_floor_applied{false};
    bool adaptive_gnss_pos_weight_enabled{false};
    bool adaptive_gnss_pos_weight_active{false};
    bool adaptive_gnss_pos_weight_applied{false};
    bool mission_cov_hygiene_enabled{false};
    bool mission_cov_hygiene_phase_allowed{false};
    bool mission_cov_hygiene_accepted_recent{false};
    bool mission_cov_hygiene_triggered{false};
    bool mission_cov_hygiene_active{false};
    bool mission_cov_hygiene_applied{false};
    bool turn_rate_propagation_noise_probe_enabled{false};
    bool turn_rate_propagation_noise_probe_phase_allowed{false};
    bool turn_rate_propagation_noise_probe_motion_ok{false};
    bool turn_rate_propagation_noise_probe_triggered{false};
    bool turn_rate_propagation_noise_probe_active{false};
    bool turn_rate_propagation_noise_probe_applied{false};
    bool accbias_z_propagation_probe_enabled{false};
    bool accbias_z_propagation_probe_apply_noise_scale{false};
    bool accbias_z_propagation_probe_have_bias{false};
    bool accbias_z_propagation_probe_phase_allowed{false};
    bool accbias_z_propagation_probe_motion_ok{false};
    bool accbias_z_propagation_probe_triggered{false};
    bool accbias_z_propagation_probe_active{false};
    bool accbias_z_propagation_probe_applied{false};
    bool position_lag_compensation_active{false};
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
    bool terminal_descent_context{false};
    bool phase_error_memory_recent_turnpost_active{false};
    bool terminal_descent_native_velocity_override_active{false};
    bool terminal_descent_horizontal_zero_velocity_active{false};
    bool terminal_descent_horizontal_zero_velocity_applied{false};
    bool terminal_descent_vertical_cov_reopen_active{false};
    bool terminal_descent_vertical_cov_reopen_applied{false};
    bool medium_gap_active{false};
    bool medium_gap_segmented{false};
    bool medium_gap_conservative_single_step{false};
    double ros_time_sec{std::numeric_limits<double>::quiet_NaN()};
    double update_time_sec{std::numeric_limits<double>::quiet_NaN()};
    double armed_time_sec{std::numeric_limits<double>::quiet_NaN()};
    double gnss_source_time_sec{std::numeric_limits<double>::quiet_NaN()};
    double gnss_rx_ros_time_sec{std::numeric_limits<double>::quiet_NaN()};
    double core_time_before_update_sec{std::numeric_limits<double>::quiet_NaN()};
    double gnss_source_age_at_update_sec{std::numeric_limits<double>::quiet_NaN()};
    double gnss_rx_age_at_update_sec{std::numeric_limits<double>::quiet_NaN()};
    double core_time_before_update_minus_update_sec{std::numeric_limits<double>::quiet_NaN()};
    double gnss_update_minus_source_sec{std::numeric_limits<double>::quiet_NaN()};
    double update_time_minus_ros_sec{std::numeric_limits<double>::quiet_NaN()};
    double latest_heading_update_age_sec{std::numeric_limits<double>::quiet_NaN()};
    double latest_heading_residual_abs_deg{std::numeric_limits<double>::quiet_NaN()};
    double latest_heading_yaw_correction_abs_deg{std::numeric_limits<double>::quiet_NaN()};
    double phase_error_memory_recent_turnpost_age_sec{
      std::numeric_limits<double>::quiet_NaN()};
    double last_turning_age_sec{std::numeric_limits<double>::quiet_NaN()};
    double post_turn_hold_remaining_sec{std::numeric_limits<double>::quiet_NaN()};
    std::string mavros_mode;
    std::string gnss_position_std_source_label{"unknown"};
    std::string latest_heading_mode{"unknown"};
    std::string native_velocity_outlier_guard_reason{"disabled"};
    std::string native_velocity_outlier_guard_action{"none"};
    std::string gnss_velocity_outward_damping_reason{"disabled"};
    std::string turn_postturn_native_velocity_deweight_reason{"disabled"};
    std::string gnss_position_response_boost_reason{"disabled"};
    std::string gnss_position_gain_response_reason{"disabled"};
    std::string motion_gnss_pos_weight_reason{"disabled"};
    std::string gnss_pos_recovery_weight_reason{"disabled"};
    std::string context_gnss_pos_floor_reason{"disabled"};
    std::string adaptive_gnss_pos_weight_phase{"disabled"};
    std::string adaptive_gnss_pos_weight_reason{"disabled"};
    std::string mission_cov_hygiene_phase{"disabled"};
    std::string mission_cov_hygiene_reason{"disabled"};
    std::string turn_rate_propagation_noise_probe_reason{"disabled"};
    std::string accbias_z_propagation_probe_reason{"disabled"};
    double last_position_residual_h_m{std::numeric_limits<double>::quiet_NaN()};
    double last_position_residual_u_m{std::numeric_limits<double>::quiet_NaN()};
    double last_velocity_residual_h_mps{std::numeric_limits<double>::quiet_NaN()};
    double last_velocity_residual_e_mps{std::numeric_limits<double>::quiet_NaN()};
    double last_velocity_residual_d_mps{std::numeric_limits<double>::quiet_NaN()};
    double native_velocity_outlier_guard_speed_mismatch_mps{std::numeric_limits<double>::quiet_NaN()};
    double native_velocity_outlier_guard_core_residual_h_mps{std::numeric_limits<double>::quiet_NaN()};
    double native_velocity_outlier_guard_reweight_std_h_mps{std::numeric_limits<double>::quiet_NaN()};
    double native_velocity_outlier_guard_reweight_std_u_mps{std::numeric_limits<double>::quiet_NaN()};
    double gnss_velocity_outward_damping_core_gnss_diff_n_m{std::numeric_limits<double>::quiet_NaN()};
    double gnss_velocity_outward_damping_core_gnss_diff_e_m{std::numeric_limits<double>::quiet_NaN()};
    double gnss_velocity_outward_damping_core_gnss_diff_h_m{std::numeric_limits<double>::quiet_NaN()};
    double gnss_velocity_outward_damping_core_native_residual_n_mps{std::numeric_limits<double>::quiet_NaN()};
    double gnss_velocity_outward_damping_core_native_residual_e_mps{std::numeric_limits<double>::quiet_NaN()};
    double gnss_velocity_outward_damping_core_native_residual_h_mps{std::numeric_limits<double>::quiet_NaN()};
    double gnss_velocity_outward_damping_radial_outward_mps{std::numeric_limits<double>::quiet_NaN()};
    double gnss_velocity_outward_damping_radial_score{0.0};
    double gnss_velocity_outward_damping_hold_score{0.0};
    double gnss_velocity_outward_damping_selected_std_h_mps{std::numeric_limits<double>::quiet_NaN()};
    double gnss_velocity_outward_damping_target_std_h_mps{std::numeric_limits<double>::quiet_NaN()};
    double gnss_velocity_outward_damping_effective_std_h_mps{std::numeric_limits<double>::quiet_NaN()};
    double gnss_velocity_outward_damping_multiplier_h{std::numeric_limits<double>::quiet_NaN()};
    double turn_postturn_native_velocity_deweight_core_gnss_diff_n_m{std::numeric_limits<double>::quiet_NaN()};
    double turn_postturn_native_velocity_deweight_core_gnss_diff_e_m{std::numeric_limits<double>::quiet_NaN()};
    double turn_postturn_native_velocity_deweight_core_gnss_diff_h_m{std::numeric_limits<double>::quiet_NaN()};
    double turn_postturn_native_velocity_deweight_core_native_residual_n_mps{std::numeric_limits<double>::quiet_NaN()};
    double turn_postturn_native_velocity_deweight_core_native_residual_e_mps{std::numeric_limits<double>::quiet_NaN()};
    double turn_postturn_native_velocity_deweight_core_native_residual_h_mps{std::numeric_limits<double>::quiet_NaN()};
    double turn_postturn_native_velocity_deweight_radial_mps{std::numeric_limits<double>::quiet_NaN()};
    double turn_postturn_native_velocity_deweight_radial_abs_score{0.0};
    double turn_postturn_native_velocity_deweight_residual_score{0.0};
    double turn_postturn_native_velocity_deweight_score{0.0};
    double turn_postturn_native_velocity_deweight_selected_std_h_mps{std::numeric_limits<double>::quiet_NaN()};
    double turn_postturn_native_velocity_deweight_target_std_h_mps{std::numeric_limits<double>::quiet_NaN()};
    double turn_postturn_native_velocity_deweight_effective_std_h_mps{std::numeric_limits<double>::quiet_NaN()};
    double turn_postturn_native_velocity_deweight_multiplier_h{std::numeric_limits<double>::quiet_NaN()};
    double gnss_position_response_boost_residual_h_m{std::numeric_limits<double>::quiet_NaN()};
    double gnss_position_response_boost_last_residual_h_m{std::numeric_limits<double>::quiet_NaN()};
    double gnss_position_response_boost_core_gnss_diff_h_m{std::numeric_limits<double>::quiet_NaN()};
    double gnss_position_response_boost_residual_score{0.0};
    double gnss_position_response_boost_selected_std_h_m{std::numeric_limits<double>::quiet_NaN()};
    double gnss_position_response_boost_target_std_h_m{std::numeric_limits<double>::quiet_NaN()};
    double gnss_position_response_boost_effective_std_h_m{std::numeric_limits<double>::quiet_NaN()};
    double gnss_position_response_boost_multiplier_h{std::numeric_limits<double>::quiet_NaN()};
    double gnss_position_gain_response_residual_h_m{std::numeric_limits<double>::quiet_NaN()};
    double gnss_position_gain_response_hnis_h{std::numeric_limits<double>::quiet_NaN()};
    double gnss_position_gain_response_prev_dx_pos_h_m{std::numeric_limits<double>::quiet_NaN()};
    double gnss_position_gain_response_prev_residual_h_m{std::numeric_limits<double>::quiet_NaN()};
    double gnss_position_gain_response_prev_dx_over_residual_h{std::numeric_limits<double>::quiet_NaN()};
    double gnss_position_gain_response_residual_score{0.0};
    double gnss_position_gain_response_hnis_score{0.0};
    double gnss_position_gain_response_gain_score{0.0};
    double gnss_position_gain_response_score{0.0};
    double gnss_position_gain_response_selected_std_h_m{std::numeric_limits<double>::quiet_NaN()};
    double gnss_position_gain_response_target_std_h_m{std::numeric_limits<double>::quiet_NaN()};
    double gnss_position_gain_response_effective_std_h_m{std::numeric_limits<double>::quiet_NaN()};
    double gnss_position_gain_response_multiplier_h{std::numeric_limits<double>::quiet_NaN()};
    double core_gnss_diff_h_m{std::numeric_limits<double>::quiet_NaN()};
    double core_gnss_diff_u_m{std::numeric_limits<double>::quiet_NaN()};
    double core_pos_std_d_m{std::numeric_limits<double>::quiet_NaN()};
    double core_vel_std_d_mps{std::numeric_limits<double>::quiet_NaN()};
    double core_accbias_std_z_mps2{std::numeric_limits<double>::quiet_NaN()};
    double motion_gnss_pos_weight_speed_score{std::numeric_limits<double>::quiet_NaN()};
    double motion_gnss_pos_weight_target_std_h_m{std::numeric_limits<double>::quiet_NaN()};
    double motion_gnss_pos_weight_effective_std_h_m{std::numeric_limits<double>::quiet_NaN()};
    double gnss_pos_recovery_weight_residual_h_m{std::numeric_limits<double>::quiet_NaN()};
    double gnss_pos_recovery_weight_core_gnss_diff_h_m{std::numeric_limits<double>::quiet_NaN()};
    double gnss_pos_recovery_weight_residual_score{0.0};
    double gnss_pos_recovery_weight_core_score{0.0};
    double gnss_pos_recovery_weight_score{0.0};
    double gnss_pos_recovery_weight_hold_remaining_sec{0.0};
    double gnss_pos_recovery_weight_selected_std_h_m{std::numeric_limits<double>::quiet_NaN()};
    double gnss_pos_recovery_weight_target_std_h_m{0.0};
    double gnss_pos_recovery_weight_state_std_h_m{0.0};
    double gnss_pos_recovery_weight_effective_std_h_m{std::numeric_limits<double>::quiet_NaN()};
    double gnss_pos_recovery_weight_multiplier_h{std::numeric_limits<double>::quiet_NaN()};
    double context_gnss_pos_floor_selected_std_h_m{std::numeric_limits<double>::quiet_NaN()};
    double context_gnss_pos_floor_target_floor_h_m{0.0};
    double context_gnss_pos_floor_state_floor_h_m{0.0};
    double context_gnss_pos_floor_effective_std_h_m{std::numeric_limits<double>::quiet_NaN()};
    double context_gnss_pos_floor_multiplier_h{std::numeric_limits<double>::quiet_NaN()};
    double adaptive_gnss_pos_weight_residual_h_m{std::numeric_limits<double>::quiet_NaN()};
    double adaptive_gnss_pos_weight_selected_std_h_m{std::numeric_limits<double>::quiet_NaN()};
    double adaptive_gnss_pos_weight_target_floor_h_m{std::numeric_limits<double>::quiet_NaN()};
    double adaptive_gnss_pos_weight_floor_state_h_m{std::numeric_limits<double>::quiet_NaN()};
    double adaptive_gnss_pos_weight_effective_std_h_m{std::numeric_limits<double>::quiet_NaN()};
    double adaptive_gnss_pos_weight_multiplier_h{std::numeric_limits<double>::quiet_NaN()};
    double mission_cov_hygiene_residual_h_m{std::numeric_limits<double>::quiet_NaN()};
    double mission_cov_hygiene_hnis_h{std::numeric_limits<double>::quiet_NaN()};
    double mission_cov_hygiene_score{0.0};
    double mission_cov_hygiene_nis_score{std::numeric_limits<double>::quiet_NaN()};
    double mission_cov_hygiene_residual_score{std::numeric_limits<double>::quiet_NaN()};
    double mission_cov_hygiene_cov_score{std::numeric_limits<double>::quiet_NaN()};
    double mission_cov_hygiene_weak_score{std::numeric_limits<double>::quiet_NaN()};
    double mission_cov_hygiene_pos_std_h_before_m{std::numeric_limits<double>::quiet_NaN()};
    double mission_cov_hygiene_floor_h_m{0.0};
    double mission_cov_hygiene_pos_std_h_after_m{std::numeric_limits<double>::quiet_NaN()};
    double mission_cov_hygiene_dx_pos_h_prev_m{std::numeric_limits<double>::quiet_NaN()};
    double mission_cov_hygiene_dx_pos_over_resid_prev{std::numeric_limits<double>::quiet_NaN()};
    double mission_cov_hygiene_p_nn_before{std::numeric_limits<double>::quiet_NaN()};
    double mission_cov_hygiene_p_ee_before{std::numeric_limits<double>::quiet_NaN()};
    double mission_cov_hygiene_p_ne_before{std::numeric_limits<double>::quiet_NaN()};
    double mission_cov_hygiene_p_nn_after{std::numeric_limits<double>::quiet_NaN()};
    double mission_cov_hygiene_p_ee_after{std::numeric_limits<double>::quiet_NaN()};
    double mission_cov_hygiene_p_ne_after{std::numeric_limits<double>::quiet_NaN()};
    double turn_rate_propagation_noise_probe_gyro_score{0.0};
    double turn_rate_propagation_noise_probe_arw_q_scale{1.0};
    double turn_rate_propagation_noise_probe_vrw_q_scale{1.0};
    double turn_rate_propagation_noise_probe_gyrbias_q_scale{1.0};
    double turn_rate_propagation_noise_probe_accbias_q_scale{1.0};
    double accbias_z_propagation_probe_accbias_z_mps2{
      std::numeric_limits<double>::quiet_NaN()};
    double accbias_z_propagation_probe_bias_score{0.0};
    double accbias_z_propagation_probe_arw_q_scale{1.0};
    double accbias_z_propagation_probe_vrw_q_scale{1.0};
    double accbias_z_propagation_probe_gyrbias_q_scale{1.0};
    double accbias_z_propagation_probe_accbias_q_scale{1.0};
    int adaptive_gnss_pos_weight_persistent_count{0};
    int mission_cov_hygiene_persistent_count{0};
    int gnss_velocity_outward_damping_persistent_count{0};
    int gnss_velocity_outward_damping_hold_remaining_updates{0};
    int turn_postturn_native_velocity_deweight_persistent_count{0};
    int gnss_position_response_boost_persistent_count{0};
    int gnss_position_gain_response_persistent_count{0};
    int gnss_pos_recovery_weight_persistent_count{0};
    double position_lag_compensation_sec{std::numeric_limits<double>::quiet_NaN()};
    double position_lag_compensation_n_m{std::numeric_limits<double>::quiet_NaN()};
    double position_lag_compensation_e_m{std::numeric_limits<double>::quiet_NaN()};
    double position_lag_compensation_u_m{std::numeric_limits<double>::quiet_NaN()};
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

  struct DtrqGnssQualityCache
  {
    bool valid{false};
    std::string source_kind{"unknown"};
    double source_time_sec{std::numeric_limits<double>::quiet_NaN()};
    double rx_ros_time_sec{std::numeric_limits<double>::quiet_NaN()};
    double fix_type{std::numeric_limits<double>::quiet_NaN()};
    double satellites_used{std::numeric_limits<double>::quiet_NaN()};
    double hdop{std::numeric_limits<double>::quiet_NaN()};
    double vdop{std::numeric_limits<double>::quiet_NaN()};
    double eph_m{std::numeric_limits<double>::quiet_NaN()};
    double epv_m{std::numeric_limits<double>::quiet_NaN()};
    double hacc_m{std::numeric_limits<double>::quiet_NaN()};
    double vacc_m{std::numeric_limits<double>::quiet_NaN()};
    bool vel_ned_valid{false};
    double native_vel_n_mps{std::numeric_limits<double>::quiet_NaN()};
    double native_vel_e_mps{std::numeric_limits<double>::quiet_NaN()};
    double native_vel_d_mps{std::numeric_limits<double>::quiet_NaN()};
  };

  struct EarlyRecoveryBiasFeedbackSample
  {
    double armed_time_sec{std::numeric_limits<double>::quiet_NaN()};
    double ba_z_before_mps2{std::numeric_limits<double>::quiet_NaN()};
    double residual_u_m{std::numeric_limits<double>::quiet_NaN()};
    double core_gnss_u_m{std::numeric_limits<double>::quiet_NaN()};
    double dx_ba_z_mps2{std::numeric_limits<double>::quiet_NaN()};
  };

  struct EarlyRecoveryBiasFeedbackDebug
  {
    bool enabled{false};
    bool candidate{false};
    bool active{false};
    std::string reason{"disabled"};
    int history_rows{0};
    double history_sec{std::numeric_limits<double>::quiet_NaN()};
    double armed_time_sec{std::numeric_limits<double>::quiet_NaN()};
    double ba_z_mean_mps2{std::numeric_limits<double>::quiet_NaN()};
    double residual_u_mean_m{std::numeric_limits<double>::quiet_NaN()};
    double core_gnss_u_mean_m{std::numeric_limits<double>::quiet_NaN()};
    double dx_ba_z_sum_mps2{std::numeric_limits<double>::quiet_NaN()};
    double negative_dx_ba_z_sum_mps2{std::numeric_limits<double>::quiet_NaN()};
    double positive_dx_ba_z_sum_mps2{std::numeric_limits<double>::quiet_NaN()};
    double raw_dx_ba_z_mps2{std::numeric_limits<double>::quiet_NaN()};
    double selected_dx_ba_z_mps2{std::numeric_limits<double>::quiet_NaN()};
    double delta_dx_ba_z_mps2{std::numeric_limits<double>::quiet_NaN()};
    double selected_accbias_z_after_mps2{std::numeric_limits<double>::quiet_NaN()};
    double negative_dx_scale{std::numeric_limits<double>::quiet_NaN()};
  };

  struct TurnRatePropagationNoiseDebug
  {
    bool enabled{false};
    bool phase_allowed{false};
    bool motion_ok{false};
    bool triggered{false};
    bool active{false};
    bool applied{false};
    std::string reason{"disabled"};
    double gyro_deg_s{std::numeric_limits<double>::quiet_NaN()};
    double gyro_score{0.0};
    double horizontal_speed_mps{std::numeric_limits<double>::quiet_NaN()};
    double vertical_speed_mps{std::numeric_limits<double>::quiet_NaN()};
    double arw_q_scale{1.0};
    double vrw_q_scale{1.0};
    double gyrbias_q_scale{1.0};
    double accbias_q_scale{1.0};
  };

  struct AccbiasZPropagationProbeDebug
  {
    bool enabled{false};
    bool apply_noise_scale{false};
    bool have_bias{false};
    bool phase_allowed{false};
    bool motion_ok{false};
    bool triggered{false};
    bool active{false};
    bool applied{false};
    std::string reason{"disabled"};
    double accbias_z_mps2{std::numeric_limits<double>::quiet_NaN()};
    double bias_score{0.0};
    double horizontal_speed_mps{std::numeric_limits<double>::quiet_NaN()};
    double vertical_speed_mps{std::numeric_limits<double>::quiet_NaN()};
    double arw_q_scale{1.0};
    double vrw_q_scale{1.0};
    double gyrbias_q_scale{1.0};
    double accbias_q_scale{1.0};
  };

  struct AdaptiveGnssPosWeightDebug
  {
    bool enabled{false};
    bool active{false};
    bool applied{false};
    std::string phase{"disabled"};
    std::string reason{"disabled"};
    double residual_h_m{std::numeric_limits<double>::quiet_NaN()};
    double selected_std_h_m{std::numeric_limits<double>::quiet_NaN()};
    double target_floor_h_m{0.0};
    double floor_state_h_m{0.0};
    double effective_std_h_m{std::numeric_limits<double>::quiet_NaN()};
    double multiplier_h{std::numeric_limits<double>::quiet_NaN()};
    int persistent_count{0};
  };

  struct GnssPosRecoveryWeightDebug
  {
    bool enabled{false};
    bool phase_allowed{false};
    bool motion_ok{false};
    bool triggered{false};
    bool held{false};
    bool active{false};
    bool applied{false};
    std::string reason{"disabled"};
    double residual_h_m{std::numeric_limits<double>::quiet_NaN()};
    double core_gnss_diff_h_m{std::numeric_limits<double>::quiet_NaN()};
    double residual_score{0.0};
    double core_score{0.0};
    double score{0.0};
    int persistent_count{0};
    double hold_remaining_sec{0.0};
    double selected_std_h_m{std::numeric_limits<double>::quiet_NaN()};
    double target_std_h_m{0.0};
    double state_std_h_m{0.0};
    double effective_std_h_m{std::numeric_limits<double>::quiet_NaN()};
    double multiplier_h{std::numeric_limits<double>::quiet_NaN()};
  };

  struct GnssPositionResponseBoostDebug
  {
    bool enabled{false};
    bool phase_allowed{false};
    bool motion_ok{false};
    bool triggered{false};
    bool active{false};
    bool applied{false};
    std::string reason{"disabled"};
    double residual_h_m{std::numeric_limits<double>::quiet_NaN()};
    double last_residual_h_m{std::numeric_limits<double>::quiet_NaN()};
    double core_gnss_diff_h_m{std::numeric_limits<double>::quiet_NaN()};
    double residual_score{0.0};
    int persistent_count{0};
    double selected_std_h_m{std::numeric_limits<double>::quiet_NaN()};
    double target_std_h_m{std::numeric_limits<double>::quiet_NaN()};
    double effective_std_h_m{std::numeric_limits<double>::quiet_NaN()};
    double multiplier_h{std::numeric_limits<double>::quiet_NaN()};
  };

  struct GnssPositionGainResponseDebug
  {
    bool enabled{false};
    bool phase_allowed{false};
    bool context_ok{false};
    bool motion_ok{false};
    bool gain_ok{false};
    bool triggered{false};
    bool active{false};
    bool applied{false};
    std::string reason{"disabled"};
    double residual_h_m{std::numeric_limits<double>::quiet_NaN()};
    double hnis_h{std::numeric_limits<double>::quiet_NaN()};
    double prev_dx_pos_h_m{std::numeric_limits<double>::quiet_NaN()};
    double prev_residual_h_m{std::numeric_limits<double>::quiet_NaN()};
    double prev_dx_over_residual_h{std::numeric_limits<double>::quiet_NaN()};
    double residual_score{0.0};
    double hnis_score{0.0};
    double gain_score{0.0};
    double score{0.0};
    int persistent_count{0};
    double selected_std_h_m{std::numeric_limits<double>::quiet_NaN()};
    double target_std_h_m{std::numeric_limits<double>::quiet_NaN()};
    double effective_std_h_m{std::numeric_limits<double>::quiet_NaN()};
    double multiplier_h{std::numeric_limits<double>::quiet_NaN()};
  };

  struct GnssVelocityOutwardDampingDebug
  {
    bool enabled{false};
    bool phase_allowed{false};
    bool motion_ok{false};
    bool triggered{false};
    bool held{false};
    bool active{false};
    bool applied{false};
    std::string reason{"disabled"};
    double core_gnss_diff_n_m{std::numeric_limits<double>::quiet_NaN()};
    double core_gnss_diff_e_m{std::numeric_limits<double>::quiet_NaN()};
    double core_gnss_diff_h_m{std::numeric_limits<double>::quiet_NaN()};
    double core_native_velocity_residual_n_mps{std::numeric_limits<double>::quiet_NaN()};
    double core_native_velocity_residual_e_mps{std::numeric_limits<double>::quiet_NaN()};
    double core_native_velocity_residual_h_mps{std::numeric_limits<double>::quiet_NaN()};
    double radial_outward_mps{std::numeric_limits<double>::quiet_NaN()};
    double radial_score{0.0};
    double hold_score{0.0};
    int persistent_count{0};
    int hold_remaining_updates{0};
    double selected_std_h_mps{std::numeric_limits<double>::quiet_NaN()};
    double target_std_h_mps{std::numeric_limits<double>::quiet_NaN()};
    double effective_std_h_mps{std::numeric_limits<double>::quiet_NaN()};
    double multiplier_h{std::numeric_limits<double>::quiet_NaN()};
  };

  struct TurnPostturnNativeVelocityDeweightDebug
  {
    bool enabled{false};
    bool phase_allowed{false};
    bool context_ok{false};
    bool motion_ok{false};
    bool triggered{false};
    bool active{false};
    bool applied{false};
    std::string reason{"disabled"};
    double core_gnss_diff_n_m{std::numeric_limits<double>::quiet_NaN()};
    double core_gnss_diff_e_m{std::numeric_limits<double>::quiet_NaN()};
    double core_gnss_diff_h_m{std::numeric_limits<double>::quiet_NaN()};
    double core_native_velocity_residual_n_mps{std::numeric_limits<double>::quiet_NaN()};
    double core_native_velocity_residual_e_mps{std::numeric_limits<double>::quiet_NaN()};
    double core_native_velocity_residual_h_mps{std::numeric_limits<double>::quiet_NaN()};
    double radial_mps{std::numeric_limits<double>::quiet_NaN()};
    double radial_abs_score{0.0};
    double residual_score{0.0};
    double score{0.0};
    int persistent_count{0};
    double selected_std_h_mps{std::numeric_limits<double>::quiet_NaN()};
    double target_std_h_mps{std::numeric_limits<double>::quiet_NaN()};
    double effective_std_h_mps{std::numeric_limits<double>::quiet_NaN()};
    double multiplier_h{std::numeric_limits<double>::quiet_NaN()};
  };

  struct ContextGnssPosFloorDebug
  {
    bool enabled{false};
    bool phase_allowed{false};
    bool active{false};
    bool applied{false};
    std::string reason{"disabled"};
    double selected_std_h_m{std::numeric_limits<double>::quiet_NaN()};
    double target_floor_h_m{0.0};
    double floor_state_h_m{0.0};
    double effective_std_h_m{std::numeric_limits<double>::quiet_NaN()};
    double multiplier_h{std::numeric_limits<double>::quiet_NaN()};
  };

  struct HorizontalConsistencyDebug
  {
    bool enabled{false};
    bool context_enabled{false};
    bool phase_allowed{false};
    bool motion_ok{false};
    bool triggered{false};
    bool candidate_active{false};
    std::string reason{"disabled"};
    std::string proposed_action{"none"};
    double score{0.0};
    double nis_score{std::numeric_limits<double>::quiet_NaN()};
    double residual_score{std::numeric_limits<double>::quiet_NaN()};
    double core_gnss_score{std::numeric_limits<double>::quiet_NaN()};
    double heading_score{std::numeric_limits<double>::quiet_NaN()};
    int persistent_count{0};
  };

  struct MissionCovHygieneDebug
  {
    bool enabled{false};
    bool phase_allowed{false};
    bool accepted_recent{false};
    bool triggered{false};
    bool active{false};
    bool applied{false};
    std::string phase{"disabled"};
    std::string reason{"disabled"};
    double residual_h_m{std::numeric_limits<double>::quiet_NaN()};
    double hnis_h{std::numeric_limits<double>::quiet_NaN()};
    double score{0.0};
    double nis_score{std::numeric_limits<double>::quiet_NaN()};
    double residual_score{std::numeric_limits<double>::quiet_NaN()};
    double cov_score{std::numeric_limits<double>::quiet_NaN()};
    double weak_score{std::numeric_limits<double>::quiet_NaN()};
    int persistent_count{0};
    double pos_std_h_before_m{std::numeric_limits<double>::quiet_NaN()};
    double floor_h_m{0.0};
    double pos_std_h_after_m{std::numeric_limits<double>::quiet_NaN()};
    double dx_pos_h_prev_m{std::numeric_limits<double>::quiet_NaN()};
    double dx_pos_over_resid_prev{std::numeric_limits<double>::quiet_NaN()};
    double p_nn_before{std::numeric_limits<double>::quiet_NaN()};
    double p_ee_before{std::numeric_limits<double>::quiet_NaN()};
    double p_ne_before{std::numeric_limits<double>::quiet_NaN()};
    double p_nn_after{std::numeric_limits<double>::quiet_NaN()};
    double p_ee_after{std::numeric_limits<double>::quiet_NaN()};
    double p_ne_after{std::numeric_limits<double>::quiet_NaN()};
  };

  struct PhaseErrorMemoryDebug
  {
    bool enabled{false};
    bool state_update_matched{false};
    bool recent_turnpost_active{false};
    bool pressure_active{false};
    bool candidate_active{false};
    std::string reason{"disabled"};
    double residual_h_m{std::numeric_limits<double>::quiet_NaN()};
    double dx_pos_h_m{std::numeric_limits<double>::quiet_NaN()};
    double dx_over_residual_h{std::numeric_limits<double>::quiet_NaN()};
    double pos_std_h_before_m{std::numeric_limits<double>::quiet_NaN()};
    double recent_turnpost_age_sec{std::numeric_limits<double>::quiet_NaN()};
    double residual_threshold_h_m{0.16};
    double dx_over_residual_threshold{0.22};
    double recent_turnpost_hold_sec{15.0};
  };

  static double rampScore_(double value, double start, double full)
  {
    if (!std::isfinite(value)) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    const double start_abs = std::max(0.0, std::abs(start));
    const double full_abs = std::max(start_abs + 1.0e-9, std::abs(full));
    return std::clamp((value - start_abs) / (full_abs - start_abs), 0.0, 1.0);
  }

  static double inverseRampScore_(double value, double full_score_at, double zero_score_at)
  {
    if (!std::isfinite(value)) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    const double lo = std::abs(full_score_at);
    const double hi = std::max(lo + 1.0e-9, std::abs(zero_score_at));
    if (value <= lo) {
      return 1.0;
    }
    if (value >= hi) {
      return 0.0;
    }
    return std::clamp((hi - value) / (hi - lo), 0.0, 1.0);
  }

  void resetHorizontalConsistencySupervisorState_()
  {
    horizontal_consistency_persistent_count_ = 0;
  }

  void resetMissionCovHygieneState_()
  {
    mission_cov_hygiene_persistent_count_ = 0;
    mission_cov_hygiene_floor_state_h_m_ = 0.0;
    mission_cov_hygiene_last_update_sec_ = std::numeric_limits<double>::quiet_NaN();
    mission_cov_hygiene_prev_dx_pos_h_m_ = std::numeric_limits<double>::quiet_NaN();
    mission_cov_hygiene_prev_residual_h_m_ = std::numeric_limits<double>::quiet_NaN();
  }

  TurnRatePropagationNoiseDebug updateTurnRatePropagationNoiseProbe_(
    const HeadingMotionContext & motion_ctx)
  {
    TurnRatePropagationNoiseDebug debug;
    debug.enabled = turn_rate_propagation_noise_probe_enable_;
    debug.gyro_deg_s = last_imu_gyro_norm_deg_s_;
    debug.horizontal_speed_mps = last_mavros_horizontal_speed_mps_;
    debug.vertical_speed_mps = last_mavros_vertical_speed_mps_;

    auto q_scale_from_score = [](double score, double max_scale) {
      const double scale_max =
        std::isfinite(max_scale) ? std::clamp(std::abs(max_scale), 1.0, 100.0) : 1.0;
      const double finite_score = std::isfinite(score) ? std::clamp(score, 0.0, 1.0) : 0.0;
      return 1.0 + finite_score * (scale_max - 1.0);
    };

    if (!debug.enabled) {
      debug.reason = "disabled";
      return debug;
    }

    const bool in_mission = mavros_mode_ == "AUTO.MISSION";
    const bool in_rtl = mavros_mode_ == "AUTO.RTL";
    debug.phase_allowed =
      mavros_armed_ &&
      !motion_ctx.terminal_descent_context &&
      ((in_mission && turn_rate_propagation_noise_probe_apply_mission_) ||
       (in_rtl && turn_rate_propagation_noise_probe_apply_rtl_));

    const double min_horizontal_speed_mps =
      std::max(0.0, std::abs(turn_rate_propagation_noise_probe_min_horizontal_speed_mps_));
    const double max_abs_vertical_speed_mps =
      std::max(0.0, std::abs(turn_rate_propagation_noise_probe_max_abs_vertical_speed_mps_));
    const bool have_speed =
      motion_ctx.have_fresh_mavros_speed &&
      std::isfinite(last_mavros_horizontal_speed_mps_) &&
      std::isfinite(last_mavros_vertical_speed_mps_);
    const bool horizontal_ok =
      min_horizontal_speed_mps <= 1.0e-6 ||
      (have_speed && last_mavros_horizontal_speed_mps_ >= min_horizontal_speed_mps);
    const bool vertical_ok =
      max_abs_vertical_speed_mps <= 1.0e-6 ||
      (have_speed && std::abs(last_mavros_vertical_speed_mps_) <= max_abs_vertical_speed_mps);
    debug.motion_ok = horizontal_ok && vertical_ok;

    debug.gyro_score = rampScore_(
      debug.gyro_deg_s,
      turn_rate_propagation_noise_probe_gyro_start_deg_s_,
      turn_rate_propagation_noise_probe_gyro_full_deg_s_);
    debug.triggered = std::isfinite(debug.gyro_score) && debug.gyro_score > 1.0e-6;
    debug.active = debug.phase_allowed && debug.motion_ok && debug.triggered;

    if (debug.active) {
      debug.arw_q_scale = q_scale_from_score(
        debug.gyro_score, turn_rate_propagation_noise_probe_arw_q_scale_max_);
      debug.vrw_q_scale = q_scale_from_score(
        debug.gyro_score, turn_rate_propagation_noise_probe_vrw_q_scale_max_);
      debug.gyrbias_q_scale = q_scale_from_score(
        debug.gyro_score, turn_rate_propagation_noise_probe_gyrbias_q_scale_max_);
      debug.accbias_q_scale = q_scale_from_score(
        debug.gyro_score, turn_rate_propagation_noise_probe_accbias_q_scale_max_);
      debug.applied =
        debug.arw_q_scale > 1.0 + 1.0e-6 ||
        debug.vrw_q_scale > 1.0 + 1.0e-6 ||
        debug.gyrbias_q_scale > 1.0 + 1.0e-6 ||
        debug.accbias_q_scale > 1.0 + 1.0e-6;
      debug.reason = "turn_rate_q_scale";
    } else if (!debug.phase_allowed) {
      debug.reason =
        !mavros_armed_ ? "disarmed" :
        motion_ctx.terminal_descent_context ? "terminal" :
        (in_mission && !turn_rate_propagation_noise_probe_apply_mission_) ? "mission_disabled" :
        (in_rtl && !turn_rate_propagation_noise_probe_apply_rtl_) ? "rtl_disabled" :
        "phase";
    } else if (!debug.motion_ok) {
      debug.reason = "motion_gate";
    } else {
      debug.reason = "below_turn_rate";
    }

    return debug;
  }

  AccbiasZPropagationProbeDebug updateAccbiasZPropagationProbe_(
    const HeadingMotionContext & motion_ctx)
  {
    AccbiasZPropagationProbeDebug debug;
    debug.enabled = accbias_z_propagation_probe_enable_;
    debug.apply_noise_scale = accbias_z_propagation_probe_apply_noise_scale_;
    debug.horizontal_speed_mps = last_mavros_horizontal_speed_mps_;
    debug.vertical_speed_mps = last_mavros_vertical_speed_mps_;

    auto q_scale_from_score = [](double score, double max_scale) {
      const double scale_max =
        std::isfinite(max_scale) ? std::clamp(std::abs(max_scale), 1.0, 100.0) : 1.0;
      const double finite_score = std::isfinite(score) ? std::clamp(score, 0.0, 1.0) : 0.0;
      return 1.0 + finite_score * (scale_max - 1.0);
    };

    if (!debug.enabled) {
      debug.reason = "disabled";
      return debug;
    }

    if (core_) {
      const kfcore::StateUpdateDebug state_update =
        core_->lastGnssPositionStateUpdateDebug();
      debug.have_bias =
        state_update.valid &&
        state_update.applied &&
        state_update.event_type == "gnss_position" &&
        state_update.accbias_after_mps2.allFinite();
      if (debug.have_bias) {
        debug.accbias_z_mps2 = state_update.accbias_after_mps2.z();
      }
    }

    const bool in_mission = mavros_mode_ == "AUTO.MISSION";
    const bool in_rtl = mavros_mode_ == "AUTO.RTL";
    debug.phase_allowed =
      mavros_armed_ &&
      !motion_ctx.terminal_descent_context &&
      ((in_mission && accbias_z_propagation_probe_apply_mission_) ||
       (in_rtl && accbias_z_propagation_probe_apply_rtl_));

    const double min_horizontal_speed_mps =
      std::max(0.0, std::abs(accbias_z_propagation_probe_min_horizontal_speed_mps_));
    const double max_abs_vertical_speed_mps =
      std::max(0.0, std::abs(accbias_z_propagation_probe_max_abs_vertical_speed_mps_));
    const bool have_speed =
      motion_ctx.have_fresh_mavros_speed &&
      std::isfinite(last_mavros_horizontal_speed_mps_) &&
      std::isfinite(last_mavros_vertical_speed_mps_);
    const bool horizontal_ok =
      min_horizontal_speed_mps <= 1.0e-6 ||
      (have_speed && last_mavros_horizontal_speed_mps_ >= min_horizontal_speed_mps);
    const bool vertical_ok =
      max_abs_vertical_speed_mps <= 1.0e-6 ||
      (have_speed && std::abs(last_mavros_vertical_speed_mps_) <= max_abs_vertical_speed_mps);
    debug.motion_ok = horizontal_ok && vertical_ok;

    const double trigger_mag =
      std::max(0.0, std::abs(accbias_z_propagation_probe_trigger_mps2_));
    const double full_mag =
      std::max(trigger_mag + 1.0e-9, std::abs(accbias_z_propagation_probe_full_mps2_));
    const double negative_bias_mag =
      std::isfinite(debug.accbias_z_mps2)
        ? -debug.accbias_z_mps2
        : std::numeric_limits<double>::quiet_NaN();
    debug.bias_score = rampScore_(negative_bias_mag, trigger_mag, full_mag);
    debug.triggered = std::isfinite(debug.bias_score) && debug.bias_score > 1.0e-6;
    debug.active = debug.have_bias && debug.phase_allowed && debug.motion_ok && debug.triggered;

    const double suggested_arw_q_scale = q_scale_from_score(
      debug.bias_score, accbias_z_propagation_probe_arw_q_scale_max_);
    const double suggested_vrw_q_scale = q_scale_from_score(
      debug.bias_score, accbias_z_propagation_probe_vrw_q_scale_max_);
    const double suggested_gyrbias_q_scale = q_scale_from_score(
      debug.bias_score, accbias_z_propagation_probe_gyrbias_q_scale_max_);
    const double suggested_accbias_q_scale = q_scale_from_score(
      debug.bias_score, accbias_z_propagation_probe_accbias_q_scale_max_);

    if (debug.active && debug.apply_noise_scale) {
      debug.arw_q_scale = suggested_arw_q_scale;
      debug.vrw_q_scale = suggested_vrw_q_scale;
      debug.gyrbias_q_scale = suggested_gyrbias_q_scale;
      debug.accbias_q_scale = suggested_accbias_q_scale;
      debug.applied =
        debug.arw_q_scale > 1.0 + 1.0e-6 ||
        debug.vrw_q_scale > 1.0 + 1.0e-6 ||
        debug.gyrbias_q_scale > 1.0 + 1.0e-6 ||
        debug.accbias_q_scale > 1.0 + 1.0e-6;
      debug.reason = "accbias_z_q_scale";
    } else if (debug.active) {
      debug.reason = "diagnostic_only";
    } else if (!debug.have_bias) {
      debug.reason = "no_bias_state";
    } else if (!debug.phase_allowed) {
      debug.reason =
        !mavros_armed_ ? "disarmed" :
        motion_ctx.terminal_descent_context ? "terminal" :
        (in_mission && !accbias_z_propagation_probe_apply_mission_) ? "mission_disabled" :
        (in_rtl && !accbias_z_propagation_probe_apply_rtl_) ? "rtl_disabled" :
        "phase";
    } else if (!debug.motion_ok) {
      debug.reason = "motion_gate";
    } else {
      debug.reason = "below_accbias_z";
    }

    return debug;
  }

  void refreshMissionCovHygieneUpdateEffectiveness_()
  {
    if (!core_) {
      return;
    }
    const kfcore::StateUpdateDebug event = core_->lastGnssPositionStateUpdateDebug();
    if (!event.valid || event.sequence == 0 ||
        event.sequence == mission_cov_hygiene_last_state_update_sequence_ ||
        event.event_type != "gnss_position") {
      return;
    }

    const kfcore::ObservationDebug debug = core_->lastObservationDebug();
    const bool observation_matches =
      debug.valid &&
      debug.gnss_position_applied &&
      std::isfinite(debug.update_time_sec) &&
      std::isfinite(event.update_time_sec) &&
      std::abs(debug.update_time_sec - event.update_time_sec) <= 1.0e-6;

    mission_cov_hygiene_last_state_update_sequence_ = event.sequence;
    if (event.dx.size() >= 2 &&
        std::isfinite(event.dx[0]) &&
        std::isfinite(event.dx[1])) {
      mission_cov_hygiene_prev_dx_pos_h_m_ = std::hypot(event.dx[0], event.dx[1]);
    } else {
      mission_cov_hygiene_prev_dx_pos_h_m_ =
        std::numeric_limits<double>::quiet_NaN();
    }
    if (observation_matches) {
      mission_cov_hygiene_prev_residual_h_m_ = std::hypot(
        debug.gnss_position_residual_neu_m.x(),
        debug.gnss_position_residual_neu_m.y());
    } else {
      mission_cov_hygiene_prev_residual_h_m_ =
        std::numeric_limits<double>::quiet_NaN();
    }
  }

  PhaseErrorMemoryDebug evaluatePhaseErrorMemoryDebug_(
    const kfcore::ObservationDebug & debug,
    bool pending_debug_matched)
  {
    PhaseErrorMemoryDebug out;
    out.enabled = phase_error_memory_debug_enable_;
    out.residual_threshold_h_m =
      std::max(0.0, std::abs(phase_error_memory_debug_residual_threshold_h_m_));
    out.dx_over_residual_threshold =
      std::max(0.0, std::abs(phase_error_memory_debug_dx_over_residual_threshold_));
    out.recent_turnpost_hold_sec =
      std::max(0.0, std::abs(phase_error_memory_debug_recent_turnpost_hold_sec_));

    if (pending_debug_matched) {
      out.recent_turnpost_age_sec =
        pending_gnss_debug_context_.phase_error_memory_recent_turnpost_age_sec;
      out.recent_turnpost_active =
        pending_gnss_debug_context_.phase_error_memory_recent_turnpost_active;
    }

    if (!debug.gnss_position_applied) {
      out.reason = out.enabled ? "no_position_update" : "disabled";
      return out;
    }

    out.residual_h_m = std::hypot(
      debug.gnss_position_residual_neu_m.x(),
      debug.gnss_position_residual_neu_m.y());

    if (core_) {
      const kfcore::StateUpdateDebug event = core_->lastGnssPositionStateUpdateDebug();
      out.state_update_matched =
        event.valid &&
        event.event_type == "gnss_position" &&
        std::isfinite(event.update_time_sec) &&
        std::isfinite(debug.update_time_sec) &&
        std::abs(event.update_time_sec - debug.update_time_sec) <= 1.0e-6;
      if (out.state_update_matched) {
        if (event.dx.size() >= 2 && std::isfinite(event.dx[0]) &&
            std::isfinite(event.dx[1])) {
          out.dx_pos_h_m = std::hypot(event.dx[0], event.dx[1]);
        }
        if (event.covariance_before.rows() >= 2 &&
            event.covariance_before.cols() >= 2) {
          const double p_nn = event.covariance_before(0, 0);
          const double p_ee = event.covariance_before(1, 1);
          if (std::isfinite(p_nn) && std::isfinite(p_ee) &&
              p_nn >= 0.0 && p_ee >= 0.0) {
            out.pos_std_h_before_m = std::sqrt(0.5 * (p_nn + p_ee));
          }
        }
      }
    }

    if (std::isfinite(out.dx_pos_h_m) &&
        std::isfinite(out.residual_h_m) &&
        out.residual_h_m > 1.0e-6) {
      out.dx_over_residual_h = out.dx_pos_h_m / out.residual_h_m;
    }

    const bool residual_ok =
      std::isfinite(out.residual_h_m) &&
      out.residual_h_m >= out.residual_threshold_h_m;
    const bool gain_low =
      std::isfinite(out.dx_over_residual_h) &&
      out.dx_over_residual_h <= out.dx_over_residual_threshold;
    const bool pressure_candidate = residual_ok && gain_low;
    out.pressure_active = out.enabled && pressure_candidate;
    out.candidate_active = out.pressure_active && out.recent_turnpost_active;

    if (!out.enabled) {
      out.reason = "disabled";
    } else if (!pending_debug_matched) {
      out.reason = "no_pending_context";
    } else if (!out.state_update_matched) {
      out.reason = "no_state_update";
    } else if (!std::isfinite(out.residual_h_m)) {
      out.reason = "no_residual";
    } else if (!std::isfinite(out.dx_over_residual_h)) {
      out.reason = "no_dx_ratio";
    } else if (!residual_ok) {
      out.reason = "residual_low";
    } else if (!gain_low) {
      out.reason = "gain_not_low";
    } else if (!out.recent_turnpost_active) {
      out.reason = "pressure_no_recent_context";
    } else {
      out.reason = "candidate_active";
    }

    return out;
  }

  MissionCovHygieneDebug updateMissionCovHygiene_(
    double now_sec,
    const HeadingMotionContext & motion_ctx,
    double residual_h_m)
  {
    MissionCovHygieneDebug debug;
    debug.enabled = mission_cov_hygiene_enable_;
    debug.residual_h_m = residual_h_m;

    const kfcore::ObservationDebug last_observation = core_ ? core_->lastObservationDebug()
                                                            : kfcore::ObservationDebug{};
    debug.hnis_h = last_observation.gnss_position_nis_h_2d;
    debug.accepted_recent =
      last_observation.valid &&
      last_observation.gnss_position_update_accepted &&
      last_observation.gnss_position_applied;

    const Eigen::MatrixXd covariance_before = core_ ? core_->covariance() : Eigen::MatrixXd();
    if (covariance_before.rows() >= 2 && covariance_before.cols() >= 2) {
      debug.p_nn_before = covariance_before(0, 0);
      debug.p_ee_before = covariance_before(1, 1);
      debug.p_ne_before = 0.5 * (covariance_before(0, 1) + covariance_before(1, 0));
      if (std::isfinite(debug.p_nn_before) && std::isfinite(debug.p_ee_before) &&
          debug.p_nn_before >= 0.0 && debug.p_ee_before >= 0.0) {
        debug.pos_std_h_before_m =
          std::sqrt(0.5 * (debug.p_nn_before + debug.p_ee_before));
      }
    }

    debug.dx_pos_h_prev_m = mission_cov_hygiene_prev_dx_pos_h_m_;
    if (std::isfinite(mission_cov_hygiene_prev_dx_pos_h_m_) &&
        std::isfinite(mission_cov_hygiene_prev_residual_h_m_) &&
        mission_cov_hygiene_prev_residual_h_m_ > 1.0e-6) {
      debug.dx_pos_over_resid_prev =
        mission_cov_hygiene_prev_dx_pos_h_m_ / mission_cov_hygiene_prev_residual_h_m_;
    }

    const bool in_mission = mavros_mode_ == "AUTO.MISSION";
    const bool in_rtl = mavros_mode_ == "AUTO.RTL";
    debug.phase_allowed =
      mavros_armed_ &&
      !motion_ctx.terminal_descent_context &&
      ((in_mission && mission_cov_hygiene_apply_mission_) ||
       (in_rtl && mission_cov_hygiene_apply_rtl_));

    auto finite_or_zero = [](double value) {
      return std::isfinite(value) ? value : 0.0;
    };

    if (!mission_cov_hygiene_enable_) {
      resetMissionCovHygieneState_();
      debug.reason = "disabled";
      return debug;
    }
    if (!debug.phase_allowed) {
      resetMissionCovHygieneState_();
      debug.reason =
        !mavros_armed_ ? "disarmed_reset" :
        motion_ctx.terminal_descent_context ? "terminal_reset" :
        (in_rtl && !mission_cov_hygiene_apply_rtl_) ? "rtl_disabled_reset" :
        (in_mission && !mission_cov_hygiene_apply_mission_) ? "mission_disabled_reset" :
        "phase_reset";
      return debug;
    }
    if (!debug.accepted_recent) {
      mission_cov_hygiene_persistent_count_ = 0;
      debug.reason = "no_recent_accepted_gnss";
      return debug;
    }

    debug.nis_score = rampScore_(
      debug.hnis_h,
      mission_cov_hygiene_hnis_start_,
      mission_cov_hygiene_hnis_full_);
    debug.residual_score = rampScore_(
      debug.residual_h_m,
      mission_cov_hygiene_resid_start_h_m_,
      mission_cov_hygiene_resid_full_h_m_);
    debug.cov_score = inverseRampScore_(
      debug.pos_std_h_before_m,
      mission_cov_hygiene_pos_std_tight_lo_h_m_,
      mission_cov_hygiene_pos_std_tight_hi_h_m_);
    debug.weak_score =
      std::isfinite(debug.dx_pos_over_resid_prev)
        ? inverseRampScore_(
            debug.dx_pos_over_resid_prev,
            mission_cov_hygiene_dx_ratio_low_,
            mission_cov_hygiene_dx_ratio_high_)
        : 1.0;

    debug.score = std::clamp(
      std::max(finite_or_zero(debug.nis_score), finite_or_zero(debug.residual_score)) *
        finite_or_zero(debug.cov_score) *
        finite_or_zero(debug.weak_score),
      0.0,
      1.0);
    debug.triggered = debug.score > 1.0e-6;
    if (debug.triggered) {
      mission_cov_hygiene_persistent_count_ += 1;
    } else {
      mission_cov_hygiene_persistent_count_ = 0;
    }
    debug.persistent_count = mission_cov_hygiene_persistent_count_;

    const bool candidate_active =
      debug.triggered &&
      mission_cov_hygiene_persistent_count_ >= mission_cov_hygiene_persistence_updates_;
    double target_floor_h_m = 0.0;
    if (candidate_active) {
      const double floor_min =
        std::max(0.0, std::abs(mission_cov_hygiene_floor_min_h_m_));
      const double floor_max =
        std::max(floor_min, std::abs(mission_cov_hygiene_floor_max_h_m_));
      const double floor_nominal =
        std::clamp(
          std::abs(mission_cov_hygiene_floor_nominal_h_m_),
          floor_min,
          floor_max);
      target_floor_h_m = floor_min + debug.score * (floor_max - floor_min);
      if (debug.score >= 0.5) {
        target_floor_h_m = std::max(target_floor_h_m, floor_nominal);
      }
      debug.reason =
        std::string(in_rtl ? "rtl" : "mission") + "_persistent_cov_hygiene";
      if (!std::isfinite(debug.dx_pos_over_resid_prev)) {
        debug.reason += "+no_dx_ratio";
      }
    } else if (debug.triggered) {
      debug.reason = "accumulating";
    } else {
      debug.reason = "below_trigger";
    }

    const double dt_sec =
      (std::isfinite(mission_cov_hygiene_last_update_sec_) &&
       std::isfinite(now_sec))
        ? std::clamp(now_sec - mission_cov_hygiene_last_update_sec_, 0.0, 5.0)
        : 0.0;
    mission_cov_hygiene_last_update_sec_ = now_sec;
    const bool rising = target_floor_h_m > mission_cov_hygiene_floor_state_h_m_;
    const double time_constant_sec =
      rising ? std::max(0.0, mission_cov_hygiene_attack_sec_)
             : std::max(0.0, mission_cov_hygiene_decay_sec_);
    const double alpha =
      time_constant_sec <= 0.0 ? 1.0 : std::clamp(dt_sec / time_constant_sec, 0.0, 1.0);
    mission_cov_hygiene_floor_state_h_m_ +=
      alpha * (target_floor_h_m - mission_cov_hygiene_floor_state_h_m_);
    if (mission_cov_hygiene_floor_state_h_m_ < 1.0e-5) {
      mission_cov_hygiene_floor_state_h_m_ = 0.0;
    }

    debug.floor_h_m = mission_cov_hygiene_floor_state_h_m_;
    debug.active = candidate_active || debug.floor_h_m > 1.0e-4;
    debug.phase = debug.active ? (in_rtl ? "rtl_active" : "mission_active") : "mission_decay";
    if (debug.active && debug.floor_h_m > 1.0e-4 && core_) {
      debug.applied = core_->reopenHorizontalPositionCovariance(
        debug.floor_h_m,
        mission_cov_hygiene_offdiag_corr_limit_,
        debug.reason);
    }

    const Eigen::MatrixXd covariance_after = core_ ? core_->covariance() : Eigen::MatrixXd();
    if (covariance_after.rows() >= 2 && covariance_after.cols() >= 2) {
      debug.p_nn_after = covariance_after(0, 0);
      debug.p_ee_after = covariance_after(1, 1);
      debug.p_ne_after = 0.5 * (covariance_after(0, 1) + covariance_after(1, 0));
      if (std::isfinite(debug.p_nn_after) && std::isfinite(debug.p_ee_after) &&
          debug.p_nn_after >= 0.0 && debug.p_ee_after >= 0.0) {
        debug.pos_std_h_after_m =
          std::sqrt(0.5 * (debug.p_nn_after + debug.p_ee_after));
      }
    }

    return debug;
  }

  HorizontalConsistencyDebug evaluateHorizontalConsistencySupervisor_(
    const kfcore::ObservationDebug & debug,
    bool pending_debug_matched,
    double residual_h_m)
  {
    HorizontalConsistencyDebug out;
    out.enabled = horizontal_consistency_supervisor_enable_;
    out.reason = horizontal_consistency_supervisor_enable_ ? "below_trigger" : "disabled";

    const bool in_mission =
      pending_debug_matched && pending_gnss_debug_context_.mavros_mode == "AUTO.MISSION";
    const bool in_rtl =
      pending_debug_matched && pending_gnss_debug_context_.mavros_mode == "AUTO.RTL";
    out.phase_allowed =
      (in_mission && horizontal_consistency_apply_mission_) ||
      (in_rtl && horizontal_consistency_apply_rtl_);

    const bool have_horizontal_speed =
      pending_debug_matched &&
      std::isfinite(pending_gnss_debug_context_.horizontal_speed_mps);
    const bool have_vertical_speed =
      pending_debug_matched &&
      std::isfinite(pending_gnss_debug_context_.vertical_speed_mps);
    out.motion_ok =
      pending_debug_matched &&
      pending_gnss_debug_context_.have_fresh_speed &&
      have_horizontal_speed &&
      have_vertical_speed &&
      pending_gnss_debug_context_.horizontal_speed_mps >=
        std::max(0.0, horizontal_consistency_min_horizontal_speed_mps_) &&
      std::abs(pending_gnss_debug_context_.vertical_speed_mps) <=
        std::max(0.0, horizontal_consistency_max_vertical_speed_mps_);

    out.nis_score = rampScore_(
      debug.gnss_position_nis_h_2d,
      horizontal_consistency_nis_start_h_2d_,
      horizontal_consistency_nis_full_h_2d_);
    out.residual_score = rampScore_(
      residual_h_m,
      horizontal_consistency_residual_start_h_m_,
      horizontal_consistency_residual_full_h_m_);
    out.core_gnss_score = rampScore_(
      pending_debug_matched ? pending_gnss_debug_context_.core_gnss_diff_h_m
                            : std::numeric_limits<double>::quiet_NaN(),
      horizontal_consistency_core_gnss_start_h_m_,
      horizontal_consistency_core_gnss_full_h_m_);
    out.heading_score =
      0.5 * rampScore_(
        pending_debug_matched ? pending_gnss_debug_context_.latest_heading_residual_abs_deg
                              : std::numeric_limits<double>::quiet_NaN(),
        horizontal_consistency_heading_residual_start_deg_,
        horizontal_consistency_heading_residual_full_deg_);

    auto finite_or_zero = [](double value) {
      return std::isfinite(value) ? value : 0.0;
    };
    out.score = std::max(
      std::max(finite_or_zero(out.nis_score), finite_or_zero(out.residual_score)),
      std::max(finite_or_zero(out.core_gnss_score), finite_or_zero(out.heading_score)));

    out.context_enabled =
      out.enabled &&
      pending_debug_matched &&
      debug.gnss_position_update_accepted &&
      pending_gnss_debug_context_.armed &&
      !pending_gnss_debug_context_.terminal_descent_context &&
      out.phase_allowed &&
      out.motion_ok;
    out.triggered =
      out.context_enabled &&
      out.score >= std::clamp(horizontal_consistency_score_trigger_, 0.0, 1.0);

    if (out.triggered) {
      horizontal_consistency_persistent_count_ += 1;
    } else {
      horizontal_consistency_persistent_count_ = 0;
    }
    out.persistent_count = horizontal_consistency_persistent_count_;
    out.candidate_active =
      out.triggered &&
      horizontal_consistency_persistent_count_ >= horizontal_consistency_persistence_updates_;
    if (out.candidate_active) {
      out.reason = "persistent_trigger";
      out.proposed_action = "log_only_reopen_horizontal_covariance";
    } else if (out.triggered) {
      out.reason = "accumulating";
    } else if (!out.enabled) {
      out.reason = "disabled";
    } else if (!pending_debug_matched) {
      out.reason = "pending_unmatched";
    } else if (!debug.gnss_position_update_accepted) {
      out.reason = "update_not_accepted";
    } else if (!pending_gnss_debug_context_.armed) {
      out.reason = "disarmed";
    } else if (pending_gnss_debug_context_.terminal_descent_context) {
      out.reason = "terminal_descent";
    } else if (!out.phase_allowed) {
      out.reason = "phase_disabled";
    } else if (!out.motion_ok) {
      out.reason = "motion_gate";
    } else {
      out.reason = "below_trigger";
    }

    return out;
  }

  void resetGnssPosRecoveryWeightState_()
  {
    gnss_pos_recovery_weight_state_std_h_m_ = 0.0;
    gnss_pos_recovery_weight_persistent_count_ = 0;
    gnss_pos_recovery_weight_hold_until_sec_ =
      std::numeric_limits<double>::quiet_NaN();
    gnss_pos_recovery_weight_last_update_sec_ =
      std::numeric_limits<double>::quiet_NaN();
  }

  void resetContextGnssPosFloorState_()
  {
    context_gnss_pos_floor_state_h_m_ = 0.0;
    context_gnss_pos_floor_last_update_sec_ =
      std::numeric_limits<double>::quiet_NaN();
  }

  void resetGnssPositionResponseBoostState_()
  {
    gnss_position_response_boost_persistent_count_ = 0;
  }

  void resetGnssPositionGainResponseState_()
  {
    gnss_position_gain_response_persistent_count_ = 0;
  }

  void resetGnssVelocityOutwardDampingState_()
  {
    gnss_velocity_outward_damping_persistent_count_ = 0;
    gnss_velocity_outward_damping_hold_remaining_updates_ = 0;
    gnss_velocity_outward_damping_hold_score_ = 0.0;
  }

  void resetTurnPostturnNativeVelocityDeweightState_()
  {
    turn_postturn_native_velocity_deweight_persistent_count_ = 0;
  }

  GnssVelocityOutwardDampingDebug updateGnssVelocityOutwardDamping_(
    const HeadingMotionContext & motion_ctx,
    const kfcore::State & core_state,
    double native_vN_mps,
    double native_vE_mps,
    double core_gnss_diff_n_m,
    double core_gnss_diff_e_m,
    double core_gnss_diff_h_m,
    double selected_std_h_mps)
  {
    GnssVelocityOutwardDampingDebug debug;
    debug.enabled = gnss_velocity_outward_damping_enable_;
    debug.core_gnss_diff_n_m = core_gnss_diff_n_m;
    debug.core_gnss_diff_e_m = core_gnss_diff_e_m;
    debug.core_gnss_diff_h_m = core_gnss_diff_h_m;
    debug.selected_std_h_mps = selected_std_h_mps;
    debug.effective_std_h_mps = selected_std_h_mps;
    debug.multiplier_h =
      (std::isfinite(selected_std_h_mps) && selected_std_h_mps > 0.0)
        ? 1.0
        : std::numeric_limits<double>::quiet_NaN();

    if (std::isfinite(core_state.vN) && std::isfinite(core_state.vE) &&
        std::isfinite(native_vN_mps) && std::isfinite(native_vE_mps)) {
      debug.core_native_velocity_residual_n_mps = core_state.vN - native_vN_mps;
      debug.core_native_velocity_residual_e_mps = core_state.vE - native_vE_mps;
      debug.core_native_velocity_residual_h_mps = std::hypot(
        debug.core_native_velocity_residual_n_mps,
        debug.core_native_velocity_residual_e_mps);
    }

    if (std::isfinite(core_gnss_diff_h_m) &&
        core_gnss_diff_h_m > 1.0e-6 &&
        std::isfinite(core_gnss_diff_n_m) &&
        std::isfinite(core_gnss_diff_e_m) &&
        std::isfinite(debug.core_native_velocity_residual_n_mps) &&
        std::isfinite(debug.core_native_velocity_residual_e_mps)) {
      debug.radial_outward_mps =
        (core_gnss_diff_n_m * debug.core_native_velocity_residual_n_mps +
         core_gnss_diff_e_m * debug.core_native_velocity_residual_e_mps) /
        core_gnss_diff_h_m;
    }

    if (!gnss_velocity_outward_damping_enable_) {
      resetGnssVelocityOutwardDampingState_();
      return debug;
    }

    const bool in_mission = mavros_mode_ == "AUTO.MISSION";
    const bool in_rtl = mavros_mode_ == "AUTO.RTL";
    debug.phase_allowed =
      mavros_armed_ &&
      !motion_ctx.terminal_descent_context &&
      ((in_mission && gnss_velocity_outward_damping_apply_mission_) ||
       (in_rtl && gnss_velocity_outward_damping_apply_rtl_));

    const double min_horizontal_speed_mps =
      std::max(0.0, std::abs(gnss_velocity_outward_damping_min_horizontal_speed_mps_));
    const double max_abs_vertical_speed_mps =
      std::max(0.0, std::abs(gnss_velocity_outward_damping_max_abs_vertical_speed_mps_));
    const bool have_speed =
      motion_ctx.have_fresh_mavros_speed &&
      std::isfinite(last_mavros_horizontal_speed_mps_) &&
      std::isfinite(last_mavros_vertical_speed_mps_);
    const bool horizontal_ok =
      min_horizontal_speed_mps <= 1.0e-6 ||
      (have_speed && last_mavros_horizontal_speed_mps_ >= min_horizontal_speed_mps);
    const bool vertical_ok =
      max_abs_vertical_speed_mps <= 1.0e-6 ||
      (have_speed && std::abs(last_mavros_vertical_speed_mps_) <= max_abs_vertical_speed_mps);
    debug.motion_ok = horizontal_ok && vertical_ok;

    if (!debug.phase_allowed) {
      resetGnssVelocityOutwardDampingState_();
      debug.reason =
        !mavros_armed_ ? "disarmed_reset" :
        motion_ctx.terminal_descent_context ? "terminal_reset" :
        (in_mission && !gnss_velocity_outward_damping_apply_mission_) ? "mission_disabled_reset" :
        (in_rtl && !gnss_velocity_outward_damping_apply_rtl_) ? "rtl_disabled_reset" :
        "phase_reset";
      return debug;
    }
    if (!debug.motion_ok) {
      resetGnssVelocityOutwardDampingState_();
      debug.reason = "motion_gate";
      return debug;
    }
    if (!std::isfinite(core_gnss_diff_h_m) ||
        core_gnss_diff_h_m <
          std::max(0.0, std::abs(gnss_velocity_outward_damping_min_core_gnss_diff_h_m_))) {
      resetGnssVelocityOutwardDampingState_();
      debug.reason = "core_gnss_diff_gate";
      return debug;
    }
    if (!std::isfinite(debug.radial_outward_mps)) {
      resetGnssVelocityOutwardDampingState_();
      debug.reason = "no_radial_velocity";
      return debug;
    }

    debug.radial_score = rampScore_(
      debug.radial_outward_mps,
      gnss_velocity_outward_damping_radial_start_mps_,
      gnss_velocity_outward_damping_radial_full_mps_);
    debug.triggered = debug.radial_score > 1.0e-6;
    if (debug.triggered) {
      gnss_velocity_outward_damping_persistent_count_ += 1;
    } else {
      gnss_velocity_outward_damping_persistent_count_ = 0;
    }
    debug.persistent_count = gnss_velocity_outward_damping_persistent_count_;
    const bool trigger_active =
      debug.triggered &&
      gnss_velocity_outward_damping_persistent_count_ >=
        gnss_velocity_outward_damping_persistence_updates_;
    const int hold_updates =
      std::max(0, gnss_velocity_outward_damping_hold_updates_);
    if (trigger_active) {
      gnss_velocity_outward_damping_hold_remaining_updates_ = hold_updates;
      gnss_velocity_outward_damping_hold_score_ = debug.radial_score;
    } else if (!debug.triggered &&
               debug.radial_outward_mps > 0.0 &&
               gnss_velocity_outward_damping_hold_remaining_updates_ > 0 &&
               gnss_velocity_outward_damping_hold_score_ > 1.0e-6) {
      debug.held = true;
      gnss_velocity_outward_damping_hold_remaining_updates_ -= 1;
    } else if (!debug.triggered) {
      gnss_velocity_outward_damping_hold_remaining_updates_ = 0;
      gnss_velocity_outward_damping_hold_score_ = 0.0;
    }
    debug.active = trigger_active || debug.held;
    debug.hold_remaining_updates = gnss_velocity_outward_damping_hold_remaining_updates_;
    debug.hold_score =
      debug.held ? gnss_velocity_outward_damping_hold_score_ : 0.0;

    const double std_min_h_mps =
      std::max(0.005, std::abs(gnss_velocity_outward_damping_std_min_h_mps_));
    const double std_max_h_mps =
      std::max(std_min_h_mps, std::abs(gnss_velocity_outward_damping_std_max_h_mps_));
    const double effective_score =
      debug.held ? std::max(debug.radial_score, debug.hold_score) : debug.radial_score;
    debug.target_std_h_mps =
      std_max_h_mps - effective_score * (std_max_h_mps - std_min_h_mps);
    debug.target_std_h_mps = std::clamp(debug.target_std_h_mps, std_min_h_mps, std_max_h_mps);

    if (debug.active && std::isfinite(selected_std_h_mps)) {
      debug.effective_std_h_mps = std::min(selected_std_h_mps, debug.target_std_h_mps);
      debug.multiplier_h =
        selected_std_h_mps > 0.0
          ? debug.effective_std_h_mps / selected_std_h_mps
          : std::numeric_limits<double>::quiet_NaN();
      debug.applied = debug.effective_std_h_mps < selected_std_h_mps - 1.0e-4;
      debug.reason =
        std::string(in_rtl ? "rtl" : "mission") +
        (debug.held ? "_velocity_outward_damping_hold" :
         (debug.applied ? "_velocity_outward_damping" : "_velocity_outward_damping_noop"));
    } else if (debug.triggered) {
      debug.reason = "accumulating";
    } else {
      debug.reason = debug.radial_outward_mps > 0.0 ? "below_trigger" : "not_outward";
    }

    return debug;
  }

  TurnPostturnNativeVelocityDeweightDebug updateTurnPostturnNativeVelocityDeweight_(
    const HeadingMotionContext & motion_ctx,
    const kfcore::State & core_state,
    double native_vN_mps,
    double native_vE_mps,
    double core_gnss_diff_n_m,
    double core_gnss_diff_e_m,
    double core_gnss_diff_h_m,
    double selected_std_h_mps)
  {
    TurnPostturnNativeVelocityDeweightDebug debug;
    debug.enabled = turn_postturn_native_velocity_deweight_enable_;
    debug.core_gnss_diff_n_m = core_gnss_diff_n_m;
    debug.core_gnss_diff_e_m = core_gnss_diff_e_m;
    debug.core_gnss_diff_h_m = core_gnss_diff_h_m;
    debug.selected_std_h_mps = selected_std_h_mps;
    debug.effective_std_h_mps = selected_std_h_mps;
    debug.multiplier_h =
      (std::isfinite(selected_std_h_mps) && selected_std_h_mps > 0.0)
        ? 1.0
        : std::numeric_limits<double>::quiet_NaN();

    if (std::isfinite(core_state.vN) && std::isfinite(core_state.vE) &&
        std::isfinite(native_vN_mps) && std::isfinite(native_vE_mps)) {
      debug.core_native_velocity_residual_n_mps = core_state.vN - native_vN_mps;
      debug.core_native_velocity_residual_e_mps = core_state.vE - native_vE_mps;
      debug.core_native_velocity_residual_h_mps = std::hypot(
        debug.core_native_velocity_residual_n_mps,
        debug.core_native_velocity_residual_e_mps);
    }

    if (std::isfinite(core_gnss_diff_h_m) &&
        core_gnss_diff_h_m > 1.0e-6 &&
        std::isfinite(core_gnss_diff_n_m) &&
        std::isfinite(core_gnss_diff_e_m) &&
        std::isfinite(debug.core_native_velocity_residual_n_mps) &&
        std::isfinite(debug.core_native_velocity_residual_e_mps)) {
      debug.radial_mps =
        (core_gnss_diff_n_m * debug.core_native_velocity_residual_n_mps +
         core_gnss_diff_e_m * debug.core_native_velocity_residual_e_mps) /
        core_gnss_diff_h_m;
    }

    if (!turn_postturn_native_velocity_deweight_enable_) {
      resetTurnPostturnNativeVelocityDeweightState_();
      return debug;
    }

    const bool in_mission = mavros_mode_ == "AUTO.MISSION";
    const bool in_rtl = mavros_mode_ == "AUTO.RTL";
    debug.phase_allowed =
      mavros_armed_ &&
      !motion_ctx.terminal_descent_context &&
      ((in_mission && turn_postturn_native_velocity_deweight_apply_mission_) ||
       (in_rtl && turn_postturn_native_velocity_deweight_apply_rtl_));
    debug.context_ok =
      (motion_ctx.turning_now && turn_postturn_native_velocity_deweight_apply_turning_) ||
      (motion_ctx.post_turn_context && turn_postturn_native_velocity_deweight_apply_post_turn_);

    const double min_horizontal_speed_mps = std::max(
      0.0,
      std::abs(turn_postturn_native_velocity_deweight_min_horizontal_speed_mps_));
    const double max_abs_vertical_speed_mps = std::max(
      0.0,
      std::abs(turn_postturn_native_velocity_deweight_max_abs_vertical_speed_mps_));
    const bool have_speed =
      motion_ctx.have_fresh_mavros_speed &&
      std::isfinite(last_mavros_horizontal_speed_mps_) &&
      std::isfinite(last_mavros_vertical_speed_mps_);
    const bool horizontal_ok =
      min_horizontal_speed_mps <= 1.0e-6 ||
      (have_speed && last_mavros_horizontal_speed_mps_ >= min_horizontal_speed_mps);
    const bool vertical_ok =
      max_abs_vertical_speed_mps <= 1.0e-6 ||
      (have_speed && std::abs(last_mavros_vertical_speed_mps_) <= max_abs_vertical_speed_mps);
    debug.motion_ok = horizontal_ok && vertical_ok;

    if (!debug.phase_allowed) {
      resetTurnPostturnNativeVelocityDeweightState_();
      debug.reason =
        !mavros_armed_ ? "disarmed_reset" :
        motion_ctx.terminal_descent_context ? "terminal_reset" :
        (in_mission && !turn_postturn_native_velocity_deweight_apply_mission_)
          ? "mission_disabled_reset" :
        (in_rtl && !turn_postturn_native_velocity_deweight_apply_rtl_)
          ? "rtl_disabled_reset" :
        "phase_reset";
      return debug;
    }
    if (!debug.context_ok) {
      resetTurnPostturnNativeVelocityDeweightState_();
      debug.reason = "context_gate";
      return debug;
    }
    if (!debug.motion_ok) {
      resetTurnPostturnNativeVelocityDeweightState_();
      debug.reason = "motion_gate";
      return debug;
    }
    if (!std::isfinite(core_gnss_diff_h_m) ||
        core_gnss_diff_h_m <
          std::max(
            0.0,
            std::abs(turn_postturn_native_velocity_deweight_min_core_gnss_diff_h_m_))) {
      resetTurnPostturnNativeVelocityDeweightState_();
      debug.reason = "core_gnss_diff_gate";
      return debug;
    }
    if (!std::isfinite(debug.radial_mps) ||
        !std::isfinite(debug.core_native_velocity_residual_h_mps)) {
      resetTurnPostturnNativeVelocityDeweightState_();
      debug.reason = "no_velocity_coupling";
      return debug;
    }

    debug.radial_abs_score = rampScore_(
      std::abs(debug.radial_mps),
      turn_postturn_native_velocity_deweight_radial_abs_start_mps_,
      turn_postturn_native_velocity_deweight_radial_abs_full_mps_);
    debug.residual_score = rampScore_(
      debug.core_native_velocity_residual_h_mps,
      turn_postturn_native_velocity_deweight_core_residual_start_h_mps_,
      turn_postturn_native_velocity_deweight_core_residual_full_h_mps_);
    auto finite_or_zero = [](double value) {
      return std::isfinite(value) ? value : 0.0;
    };
    debug.score =
      std::min(finite_or_zero(debug.radial_abs_score), finite_or_zero(debug.residual_score));
    debug.triggered = debug.score > 1.0e-6;
    if (debug.triggered) {
      turn_postturn_native_velocity_deweight_persistent_count_ += 1;
    } else {
      turn_postturn_native_velocity_deweight_persistent_count_ = 0;
    }
    debug.persistent_count = turn_postturn_native_velocity_deweight_persistent_count_;
    debug.active =
      debug.triggered &&
      turn_postturn_native_velocity_deweight_persistent_count_ >=
        turn_postturn_native_velocity_deweight_persistence_updates_;

    const double std_min_h_mps =
      std::max(0.03, std::abs(turn_postturn_native_velocity_deweight_std_min_h_mps_));
    const double std_max_h_mps =
      std::max(std_min_h_mps, std::abs(turn_postturn_native_velocity_deweight_std_max_h_mps_));
    debug.target_std_h_mps =
      std_min_h_mps + debug.score * (std_max_h_mps - std_min_h_mps);
    debug.target_std_h_mps = std::clamp(debug.target_std_h_mps, std_min_h_mps, std_max_h_mps);

    if (debug.active && std::isfinite(selected_std_h_mps)) {
      debug.effective_std_h_mps = std::max(selected_std_h_mps, debug.target_std_h_mps);
      debug.multiplier_h =
        selected_std_h_mps > 0.0
          ? debug.effective_std_h_mps / selected_std_h_mps
          : std::numeric_limits<double>::quiet_NaN();
      debug.applied = debug.effective_std_h_mps > selected_std_h_mps + 1.0e-4;
      debug.reason =
        std::string(in_rtl ? "rtl" : "mission") +
        (debug.applied ? "_turn_postturn_velocity_deweight" :
         "_turn_postturn_velocity_deweight_noop");
    } else if (debug.triggered) {
      debug.reason = "accumulating";
    } else {
      debug.reason = "below_trigger";
    }

    return debug;
  }

  GnssPositionResponseBoostDebug updateGnssPositionResponseBoost_(
    const HeadingMotionContext & motion_ctx,
    double last_residual_h_m,
    double core_gnss_diff_h_m,
    double selected_std_h_m)
  {
    GnssPositionResponseBoostDebug debug;
    debug.enabled = gnss_position_response_boost_enable_;
    debug.last_residual_h_m = last_residual_h_m;
    debug.core_gnss_diff_h_m = core_gnss_diff_h_m;
    debug.selected_std_h_m = selected_std_h_m;
    debug.effective_std_h_m = selected_std_h_m;
    debug.multiplier_h =
      (std::isfinite(selected_std_h_m) && selected_std_h_m > 0.0)
        ? 1.0
        : std::numeric_limits<double>::quiet_NaN();

    auto max_finite = [](double a, double b) {
      const bool a_ok = std::isfinite(a);
      const bool b_ok = std::isfinite(b);
      if (a_ok && b_ok) {
        return std::max(a, b);
      }
      if (a_ok) {
        return a;
      }
      if (b_ok) {
        return b;
      }
      return std::numeric_limits<double>::quiet_NaN();
    };
    debug.residual_h_m = max_finite(last_residual_h_m, core_gnss_diff_h_m);

    if (!gnss_position_response_boost_enable_) {
      resetGnssPositionResponseBoostState_();
      return debug;
    }

    const bool in_mission = mavros_mode_ == "AUTO.MISSION";
    const bool in_rtl = mavros_mode_ == "AUTO.RTL";
    debug.phase_allowed =
      mavros_armed_ &&
      !motion_ctx.terminal_descent_context &&
      ((in_mission && gnss_position_response_boost_apply_mission_) ||
       (in_rtl && gnss_position_response_boost_apply_rtl_));

    const double min_horizontal_speed_mps =
      std::max(0.0, std::abs(gnss_position_response_boost_min_horizontal_speed_mps_));
    const double max_abs_vertical_speed_mps =
      std::max(0.0, std::abs(gnss_position_response_boost_max_abs_vertical_speed_mps_));
    const bool have_speed =
      motion_ctx.have_fresh_mavros_speed &&
      std::isfinite(last_mavros_horizontal_speed_mps_) &&
      std::isfinite(last_mavros_vertical_speed_mps_);
    const bool horizontal_ok =
      min_horizontal_speed_mps <= 1.0e-6 ||
      (have_speed && last_mavros_horizontal_speed_mps_ >= min_horizontal_speed_mps);
    const bool vertical_ok =
      max_abs_vertical_speed_mps <= 1.0e-6 ||
      (have_speed && std::abs(last_mavros_vertical_speed_mps_) <= max_abs_vertical_speed_mps);
    debug.motion_ok = horizontal_ok && vertical_ok;

    if (!debug.phase_allowed) {
      resetGnssPositionResponseBoostState_();
      debug.reason =
        !mavros_armed_ ? "disarmed_reset" :
        motion_ctx.terminal_descent_context ? "terminal_reset" :
        (in_mission && !gnss_position_response_boost_apply_mission_) ? "mission_disabled_reset" :
        (in_rtl && !gnss_position_response_boost_apply_rtl_) ? "rtl_disabled_reset" :
        "phase_reset";
      return debug;
    }
    if (!debug.motion_ok) {
      resetGnssPositionResponseBoostState_();
      debug.reason = "motion_gate";
      return debug;
    }
    if (!std::isfinite(debug.residual_h_m)) {
      resetGnssPositionResponseBoostState_();
      debug.reason = "no_residual";
      return debug;
    }

    debug.residual_score = rampScore_(
      debug.residual_h_m,
      gnss_position_response_boost_residual_start_h_m_,
      gnss_position_response_boost_residual_full_h_m_);
    debug.triggered = debug.residual_score > 1.0e-6;
    if (debug.triggered) {
      gnss_position_response_boost_persistent_count_ += 1;
    } else {
      gnss_position_response_boost_persistent_count_ = 0;
    }
    debug.persistent_count = gnss_position_response_boost_persistent_count_;
    debug.active =
      debug.triggered &&
      gnss_position_response_boost_persistent_count_ >=
        gnss_position_response_boost_persistence_updates_;

    const double std_min_h_m =
      std::max(0.01, std::abs(gnss_position_response_boost_std_min_h_m_));
    const double std_max_h_m =
      std::max(std_min_h_m, std::abs(gnss_position_response_boost_std_max_h_m_));
    debug.target_std_h_m =
      std_max_h_m - debug.residual_score * (std_max_h_m - std_min_h_m);
    debug.target_std_h_m = std::clamp(debug.target_std_h_m, std_min_h_m, std_max_h_m);

    if (debug.active && std::isfinite(selected_std_h_m)) {
      debug.effective_std_h_m = std::min(selected_std_h_m, debug.target_std_h_m);
      debug.multiplier_h =
        selected_std_h_m > 0.0
          ? debug.effective_std_h_m / selected_std_h_m
          : std::numeric_limits<double>::quiet_NaN();
      debug.applied = debug.effective_std_h_m < selected_std_h_m - 1.0e-4;
      debug.reason =
        std::string(in_rtl ? "rtl" : "mission") +
        (debug.applied ? "_response_boost" : "_response_boost_noop");
    } else if (debug.triggered) {
      debug.reason = "accumulating";
    } else {
      debug.reason = "below_trigger";
    }

    return debug;
  }

  GnssPositionGainResponseDebug updateGnssPositionGainResponse_(
    const HeadingMotionContext & motion_ctx,
    double core_gnss_diff_h_m,
    double core_gnss_diff_n_m,
    double core_gnss_diff_e_m,
    double selected_std_n_m,
    double selected_std_e_m,
    double selected_std_h_m)
  {
    GnssPositionGainResponseDebug debug;
    debug.enabled = gnss_position_gain_response_enable_;
    debug.residual_h_m = core_gnss_diff_h_m;
    debug.prev_dx_pos_h_m = mission_cov_hygiene_prev_dx_pos_h_m_;
    debug.prev_residual_h_m = mission_cov_hygiene_prev_residual_h_m_;
    debug.selected_std_h_m = selected_std_h_m;
    debug.effective_std_h_m = selected_std_h_m;
    debug.multiplier_h =
      (std::isfinite(selected_std_h_m) && selected_std_h_m > 0.0)
        ? 1.0
        : std::numeric_limits<double>::quiet_NaN();

    if (std::isfinite(debug.prev_dx_pos_h_m) &&
        std::isfinite(debug.prev_residual_h_m) &&
        debug.prev_residual_h_m > 1.0e-6) {
      debug.prev_dx_over_residual_h = debug.prev_dx_pos_h_m / debug.prev_residual_h_m;
    }

    if (core_) {
      const Eigen::MatrixXd covariance = core_->covariance();
      if (covariance.rows() >= 2 && covariance.cols() >= 2 &&
          std::isfinite(core_gnss_diff_n_m) &&
          std::isfinite(core_gnss_diff_e_m) &&
          std::isfinite(selected_std_n_m) &&
          std::isfinite(selected_std_e_m) &&
          selected_std_n_m > 0.0 &&
          selected_std_e_m > 0.0) {
        const double p_nn = covariance(0, 0);
        const double p_ee = covariance(1, 1);
        const double p_ne = 0.5 * (covariance(0, 1) + covariance(1, 0));
        const double s_nn = p_nn + selected_std_n_m * selected_std_n_m;
        const double s_ee = p_ee + selected_std_e_m * selected_std_e_m;
        const double s_ne = p_ne;
        const double det = s_nn * s_ee - s_ne * s_ne;
        if (std::isfinite(s_nn) && std::isfinite(s_ee) && std::isfinite(s_ne) &&
            s_nn > 0.0 && s_ee > 0.0 && std::isfinite(det) && det > 1.0e-12) {
          debug.hnis_h =
            (s_ee * core_gnss_diff_n_m * core_gnss_diff_n_m -
             2.0 * s_ne * core_gnss_diff_n_m * core_gnss_diff_e_m +
             s_nn * core_gnss_diff_e_m * core_gnss_diff_e_m) /
            det;
        }
      }
    }

    if (!gnss_position_gain_response_enable_) {
      resetGnssPositionGainResponseState_();
      return debug;
    }

    const bool in_mission = mavros_mode_ == "AUTO.MISSION";
    const bool in_rtl = mavros_mode_ == "AUTO.RTL";
    debug.phase_allowed =
      mavros_armed_ &&
      !motion_ctx.terminal_descent_context &&
      ((in_mission && gnss_position_gain_response_apply_mission_) ||
       (in_rtl && gnss_position_gain_response_apply_rtl_));

    debug.context_ok =
      (!gnss_position_gain_response_require_armed_cruise_ ||
       motion_ctx.armed_cruise_force_relock_context) &&
      (!gnss_position_gain_response_block_turning_ || !motion_ctx.turning_now) &&
      (!gnss_position_gain_response_block_post_turn_ || !motion_ctx.post_turn_context);

    const double min_horizontal_speed_mps =
      std::max(0.0, std::abs(gnss_position_gain_response_min_horizontal_speed_mps_));
    const double max_abs_vertical_speed_mps =
      std::max(0.0, std::abs(gnss_position_gain_response_max_abs_vertical_speed_mps_));
    const bool have_speed =
      motion_ctx.have_fresh_mavros_speed &&
      std::isfinite(last_mavros_horizontal_speed_mps_) &&
      std::isfinite(last_mavros_vertical_speed_mps_);
    const bool horizontal_ok =
      min_horizontal_speed_mps <= 1.0e-6 ||
      (have_speed && last_mavros_horizontal_speed_mps_ >= min_horizontal_speed_mps);
    const bool vertical_ok =
      max_abs_vertical_speed_mps <= 1.0e-6 ||
      (have_speed && std::abs(last_mavros_vertical_speed_mps_) <= max_abs_vertical_speed_mps);
    debug.motion_ok = horizontal_ok && vertical_ok;

    debug.residual_score = rampScore_(
      debug.residual_h_m,
      gnss_position_gain_response_residual_start_h_m_,
      gnss_position_gain_response_residual_full_h_m_);
    debug.hnis_score = rampScore_(
      debug.hnis_h,
      gnss_position_gain_response_hnis_start_,
      gnss_position_gain_response_hnis_full_);
    debug.gain_score = inverseRampScore_(
      debug.prev_dx_over_residual_h,
      gnss_position_gain_response_prev_gain_low_,
      gnss_position_gain_response_prev_gain_high_);
    debug.gain_ok = std::isfinite(debug.gain_score) && debug.gain_score > 1.0e-6;

    if (!debug.phase_allowed) {
      resetGnssPositionGainResponseState_();
      debug.reason =
        !mavros_armed_ ? "disarmed_reset" :
        motion_ctx.terminal_descent_context ? "terminal_reset" :
        (in_mission && !gnss_position_gain_response_apply_mission_) ? "mission_disabled_reset" :
        (in_rtl && !gnss_position_gain_response_apply_rtl_) ? "rtl_disabled_reset" :
        "phase_reset";
      return debug;
    }
    if (!debug.context_ok) {
      resetGnssPositionGainResponseState_();
      debug.reason =
        (gnss_position_gain_response_require_armed_cruise_ &&
         !motion_ctx.armed_cruise_force_relock_context) ? "not_armed_cruise" :
        (gnss_position_gain_response_block_turning_ && motion_ctx.turning_now) ? "turning" :
        (gnss_position_gain_response_block_post_turn_ && motion_ctx.post_turn_context) ? "post_turn" :
        "context_gate";
      return debug;
    }
    if (!debug.motion_ok) {
      resetGnssPositionGainResponseState_();
      debug.reason = "motion_gate";
      return debug;
    }
    if (!std::isfinite(debug.residual_score)) {
      resetGnssPositionGainResponseState_();
      debug.reason = "no_residual";
      return debug;
    }
    if (!std::isfinite(debug.hnis_score)) {
      resetGnssPositionGainResponseState_();
      debug.reason = "no_hnis";
      return debug;
    }
    if (!debug.gain_ok) {
      resetGnssPositionGainResponseState_();
      debug.reason = std::isfinite(debug.prev_dx_over_residual_h) ? "prev_gain_not_low"
                                                                  : "no_prev_gain";
      return debug;
    }

    debug.score = std::min({debug.residual_score, debug.hnis_score, debug.gain_score});
    debug.triggered = debug.score > 1.0e-6;
    if (debug.triggered) {
      gnss_position_gain_response_persistent_count_ += 1;
    } else {
      gnss_position_gain_response_persistent_count_ = 0;
    }
    debug.persistent_count = gnss_position_gain_response_persistent_count_;
    debug.active =
      debug.triggered &&
      gnss_position_gain_response_persistent_count_ >=
        gnss_position_gain_response_persistence_updates_;

    const double std_min_h_m =
      std::max(0.01, std::abs(gnss_position_gain_response_std_min_h_m_));
    const double std_max_h_m =
      std::max(std_min_h_m, std::abs(gnss_position_gain_response_std_max_h_m_));
    debug.target_std_h_m = std_max_h_m - debug.score * (std_max_h_m - std_min_h_m);
    debug.target_std_h_m = std::clamp(debug.target_std_h_m, std_min_h_m, std_max_h_m);

    if (debug.active && std::isfinite(selected_std_h_m)) {
      debug.effective_std_h_m = std::min(selected_std_h_m, debug.target_std_h_m);
      debug.multiplier_h =
        selected_std_h_m > 0.0
          ? debug.effective_std_h_m / selected_std_h_m
          : std::numeric_limits<double>::quiet_NaN();
      debug.applied = debug.effective_std_h_m < selected_std_h_m - 1.0e-4;
      debug.reason =
        std::string(in_rtl ? "rtl" : "mission") +
        (debug.applied ? "_gain_response" : "_gain_response_noop");
    } else if (debug.triggered) {
      debug.reason = "accumulating";
    } else {
      debug.reason = "below_trigger";
    }

    return debug;
  }

  ContextGnssPosFloorDebug updateContextGnssPosFloor_(
    double now_sec,
    const HeadingMotionContext & motion_ctx,
    double selected_std_h_m)
  {
    ContextGnssPosFloorDebug debug;
    debug.enabled = context_gnss_pos_floor_enable_;
    debug.selected_std_h_m = selected_std_h_m;

    auto set_noop_effective = [&]() {
      debug.floor_state_h_m = context_gnss_pos_floor_state_h_m_;
      debug.effective_std_h_m = selected_std_h_m;
      debug.multiplier_h =
        (std::isfinite(selected_std_h_m) && selected_std_h_m > 0.0)
          ? 1.0
          : std::numeric_limits<double>::quiet_NaN();
    };

    if (!context_gnss_pos_floor_enable_) {
      resetContextGnssPosFloorState_();
      set_noop_effective();
      return debug;
    }

    const bool in_mission = mavros_mode_ == "AUTO.MISSION";
    const bool in_rtl = mavros_mode_ == "AUTO.RTL";
    const bool terminal_disabled = motion_ctx.terminal_descent_context;
    const bool phase_allowed =
      mavros_armed_ &&
      !terminal_disabled &&
      ((in_mission && context_gnss_pos_floor_apply_mission_) ||
       (in_rtl && context_gnss_pos_floor_apply_rtl_));
    debug.phase_allowed = phase_allowed;

    if (!phase_allowed) {
      resetContextGnssPosFloorState_();
      debug.target_floor_h_m = 0.0;
      set_noop_effective();
      if (!mavros_armed_) {
        debug.reason = "disarmed_reset";
      } else if (terminal_disabled) {
        debug.reason = "terminal_reset";
      } else if (in_mission && !context_gnss_pos_floor_apply_mission_) {
        debug.reason = "mission_disabled_reset";
      } else if (in_rtl && !context_gnss_pos_floor_apply_rtl_) {
        debug.reason = "rtl_disabled_reset";
      } else {
        debug.reason = "phase_reset";
      }
      return debug;
    }

    double target_floor_h_m = 0.0;
    if (in_mission) {
      target_floor_h_m =
        std::max(0.0, std::abs(context_gnss_pos_floor_mission_base_h_m_));
      debug.reason = "mission_base";
      if (motion_ctx.turning_now || motion_ctx.post_turn_context) {
        target_floor_h_m =
          std::max(target_floor_h_m, std::abs(context_gnss_pos_floor_turn_post_h_m_));
        debug.reason = "turn_post";
      } else if (motion_ctx.armed_cruise_force_relock_context) {
        target_floor_h_m =
          std::max(target_floor_h_m, std::abs(context_gnss_pos_floor_armed_cruise_h_m_));
        debug.reason = "armed_cruise";
      }
    } else if (in_rtl) {
      target_floor_h_m = std::max(0.0, std::abs(context_gnss_pos_floor_rtl_h_m_));
      debug.reason = "rtl";
    } else {
      debug.reason = "phase_decay";
    }

    const double dt_sec =
      (std::isfinite(context_gnss_pos_floor_last_update_sec_) &&
       std::isfinite(now_sec))
        ? std::clamp(now_sec - context_gnss_pos_floor_last_update_sec_, 0.0, 5.0)
        : 0.0;
    context_gnss_pos_floor_last_update_sec_ = now_sec;

    const bool rising = target_floor_h_m > context_gnss_pos_floor_state_h_m_;
    const double time_constant_sec =
      rising ? std::max(0.0, context_gnss_pos_floor_attack_sec_)
             : std::max(0.0, context_gnss_pos_floor_decay_sec_);
    const double alpha =
      time_constant_sec <= 0.0 ? 1.0 : std::clamp(dt_sec / time_constant_sec, 0.0, 1.0);
    context_gnss_pos_floor_state_h_m_ +=
      alpha * (target_floor_h_m - context_gnss_pos_floor_state_h_m_);
    if (context_gnss_pos_floor_state_h_m_ < 1.0e-5) {
      context_gnss_pos_floor_state_h_m_ = 0.0;
    }

    debug.target_floor_h_m = target_floor_h_m;
    debug.floor_state_h_m = context_gnss_pos_floor_state_h_m_;
    debug.effective_std_h_m =
      std::isfinite(selected_std_h_m)
        ? std::max(selected_std_h_m, debug.floor_state_h_m)
        : debug.floor_state_h_m;
    debug.multiplier_h =
      (std::isfinite(selected_std_h_m) && selected_std_h_m > 0.0)
        ? debug.effective_std_h_m / selected_std_h_m
        : std::numeric_limits<double>::quiet_NaN();
    debug.applied =
      std::isfinite(selected_std_h_m) &&
      debug.effective_std_h_m > selected_std_h_m + 1.0e-4;
    debug.active =
      phase_allowed &&
      (target_floor_h_m > 1.0e-4 || context_gnss_pos_floor_state_h_m_ > 1.0e-4);

    return debug;
  }

  GnssPosRecoveryWeightDebug updateGnssPosRecoveryWeight_(
    double now_sec,
    const HeadingMotionContext & motion_ctx,
    double residual_h_m,
    double core_gnss_diff_h_m,
    double selected_std_h_m)
  {
    GnssPosRecoveryWeightDebug debug;
    debug.enabled = gnss_pos_recovery_weight_enable_;
    debug.residual_h_m = residual_h_m;
    debug.core_gnss_diff_h_m = core_gnss_diff_h_m;
    debug.selected_std_h_m = selected_std_h_m;

    if (!gnss_pos_recovery_weight_enable_) {
      resetGnssPosRecoveryWeightState_();
      debug.effective_std_h_m = selected_std_h_m;
      debug.multiplier_h =
        (std::isfinite(selected_std_h_m) && selected_std_h_m > 0.0)
          ? 1.0
          : std::numeric_limits<double>::quiet_NaN();
      return debug;
    }

    const bool in_mission = mavros_mode_ == "AUTO.MISSION";
    const bool in_rtl = mavros_mode_ == "AUTO.RTL";
    const bool phase_allowed =
      mavros_armed_ &&
      !motion_ctx.terminal_descent_context &&
      ((in_mission && gnss_pos_recovery_weight_apply_mission_) ||
       (in_rtl && gnss_pos_recovery_weight_apply_rtl_));
    const bool motion_ok =
      motion_ctx.have_fresh_mavros_speed &&
      std::isfinite(last_mavros_horizontal_speed_mps_) &&
      std::isfinite(last_mavros_vertical_speed_mps_) &&
      last_mavros_horizontal_speed_mps_ >=
        std::max(0.0, gnss_pos_recovery_weight_min_horizontal_speed_mps_) &&
      std::abs(last_mavros_vertical_speed_mps_) <=
        std::max(0.0, gnss_pos_recovery_weight_max_abs_vertical_speed_mps_);
    debug.phase_allowed = phase_allowed;
    debug.motion_ok = motion_ok;

    const double residual_start =
      std::max(0.0, std::abs(gnss_pos_recovery_weight_residual_start_h_m_));
    const double residual_full =
      std::max(
        residual_start + 1.0e-9,
        std::abs(gnss_pos_recovery_weight_residual_full_h_m_));
    const double core_start =
      std::max(0.0, std::abs(gnss_pos_recovery_weight_core_start_h_m_));
    const double core_full =
      std::max(
        core_start + 1.0e-9,
        std::abs(gnss_pos_recovery_weight_core_full_h_m_));

    const bool residual_valid = std::isfinite(residual_h_m);
    const bool core_valid = std::isfinite(core_gnss_diff_h_m);
    debug.residual_score =
      residual_valid
        ? std::clamp((residual_h_m - residual_start) / (residual_full - residual_start), 0.0, 1.0)
        : 0.0;
    debug.core_score =
      core_valid
        ? std::clamp((core_gnss_diff_h_m - core_start) / (core_full - core_start), 0.0, 1.0)
        : 0.0;
    debug.score = std::clamp(
      debug.residual_score +
        std::max(0.0, gnss_pos_recovery_weight_core_score_gain_) * debug.core_score,
      0.0,
      1.0);
    debug.triggered =
      phase_allowed &&
      motion_ok &&
      residual_valid &&
      residual_h_m >= residual_start;

    if (!phase_allowed) {
      resetGnssPosRecoveryWeightState_();
      debug.reason =
        !mavros_armed_ ? "disarmed_reset" :
        motion_ctx.terminal_descent_context ? "terminal_reset" :
        (in_mission && !gnss_pos_recovery_weight_apply_mission_) ? "mission_disabled_reset" :
        (in_rtl && !gnss_pos_recovery_weight_apply_rtl_) ? "rtl_disabled_reset" :
        "phase_reset";
      debug.effective_std_h_m = selected_std_h_m;
      debug.multiplier_h =
        (std::isfinite(selected_std_h_m) && selected_std_h_m > 0.0)
          ? 1.0
          : std::numeric_limits<double>::quiet_NaN();
      return debug;
    }

    if (debug.triggered) {
      gnss_pos_recovery_weight_persistent_count_ += 1;
    } else if (residual_valid && residual_h_m >= 0.8 * residual_start) {
      gnss_pos_recovery_weight_persistent_count_ =
        std::max(0, gnss_pos_recovery_weight_persistent_count_ - 1);
    } else {
      gnss_pos_recovery_weight_persistent_count_ = 0;
    }

    if (debug.triggered &&
        gnss_pos_recovery_weight_persistent_count_ >=
          gnss_pos_recovery_weight_persistence_updates_) {
      gnss_pos_recovery_weight_hold_until_sec_ =
        now_sec + std::max(0.0, gnss_pos_recovery_weight_hold_sec_);
    }

    debug.persistent_count = gnss_pos_recovery_weight_persistent_count_;
    debug.hold_remaining_sec =
      (std::isfinite(gnss_pos_recovery_weight_hold_until_sec_) && std::isfinite(now_sec))
        ? std::max(0.0, gnss_pos_recovery_weight_hold_until_sec_ - now_sec)
        : 0.0;
    debug.held = debug.hold_remaining_sec > 0.0;

    const double std_min =
      std::max(0.01, std::abs(gnss_pos_recovery_weight_std_min_h_m_));
    const double std_max =
      std::max(std_min, std::abs(gnss_pos_recovery_weight_std_max_h_m_));
    double target_std_h_m = 0.0;
    if (debug.held) {
      target_std_h_m = std_min + debug.score * (std_max - std_min);
      target_std_h_m = std::clamp(target_std_h_m, std_min, std_max);
      debug.reason =
        std::string(in_rtl ? "rtl" : "mission") +
        (debug.triggered ? "_triggered_hold" : "_held_decay");
    } else if (!motion_ok) {
      debug.reason = "motion_gate_decay";
    } else if (!residual_valid) {
      debug.reason = "no_residual_decay";
    } else {
      debug.reason = "below_trigger_decay";
    }

    const double dt_sec =
      (std::isfinite(gnss_pos_recovery_weight_last_update_sec_) &&
       std::isfinite(now_sec))
        ? std::clamp(now_sec - gnss_pos_recovery_weight_last_update_sec_, 0.0, 5.0)
        : 0.0;
    gnss_pos_recovery_weight_last_update_sec_ = now_sec;

    const bool rising = target_std_h_m > gnss_pos_recovery_weight_state_std_h_m_;
    const double time_constant_sec =
      rising ? std::max(0.0, gnss_pos_recovery_weight_attack_sec_)
             : std::max(0.0, gnss_pos_recovery_weight_decay_sec_);
    const double alpha =
      time_constant_sec <= 0.0 ? 1.0 : std::clamp(dt_sec / time_constant_sec, 0.0, 1.0);
    gnss_pos_recovery_weight_state_std_h_m_ +=
      alpha * (target_std_h_m - gnss_pos_recovery_weight_state_std_h_m_);
    if (gnss_pos_recovery_weight_state_std_h_m_ < 1.0e-5) {
      gnss_pos_recovery_weight_state_std_h_m_ = 0.0;
    }

    debug.target_std_h_m = target_std_h_m;
    debug.state_std_h_m = gnss_pos_recovery_weight_state_std_h_m_;
    debug.effective_std_h_m =
      std::isfinite(selected_std_h_m)
        ? std::max(selected_std_h_m, debug.state_std_h_m)
        : debug.state_std_h_m;
    debug.multiplier_h =
      (std::isfinite(selected_std_h_m) && selected_std_h_m > 0.0)
        ? debug.effective_std_h_m / selected_std_h_m
        : std::numeric_limits<double>::quiet_NaN();
    debug.applied =
      std::isfinite(selected_std_h_m) &&
      debug.effective_std_h_m > selected_std_h_m + 1.0e-4;
    debug.active = debug.held || debug.applied || debug.state_std_h_m > 1.0e-4;

    return debug;
  }

  void clearAdaptiveGnssPositionWeightNisCache_()
  {
    adaptive_gnss_pos_weight_last_nis_h_2d_ =
      std::numeric_limits<double>::quiet_NaN();
    adaptive_gnss_pos_weight_last_nis_ros_time_sec_ =
      std::numeric_limits<double>::quiet_NaN();
    adaptive_gnss_pos_weight_last_nis_sequence_ = 0;
  }

  void resetAdaptiveGnssPositionWeightState_()
  {
    adaptive_gnss_pos_weight_floor_state_h_m_ = 0.0;
    adaptive_gnss_pos_weight_persistent_count_ = 0;
    adaptive_gnss_pos_weight_last_update_sec_ =
      std::numeric_limits<double>::quiet_NaN();
    clearAdaptiveGnssPositionWeightNisCache_();
  }

  void refreshAdaptiveGnssPositionWeightNisCache_(double now_sec)
  {
    if (!adaptive_gnss_pos_weight_enable_ ||
        adaptive_gnss_pos_weight_trigger_source_ != "nis_h") {
      return;
    }
    const bool adaptive_context =
      mavros_armed_ &&
      ((adaptive_gnss_pos_weight_apply_mission_ && mavros_mode_ == "AUTO.MISSION") ||
       (adaptive_gnss_pos_weight_apply_rtl_ && mavros_mode_ == "AUTO.RTL"));
    if (!adaptive_context || !core_) {
      clearAdaptiveGnssPositionWeightNisCache_();
      return;
    }

    const kfcore::ObservationDebug debug = core_->lastObservationDebug();
    if (!debug.valid || !debug.gnss_position_applied ||
        debug.sequence == adaptive_gnss_pos_weight_last_nis_sequence_ ||
        !std::isfinite(debug.gnss_position_nis_h_2d)) {
      return;
    }

    adaptive_gnss_pos_weight_last_nis_sequence_ = debug.sequence;
    adaptive_gnss_pos_weight_last_nis_h_2d_ = debug.gnss_position_nis_h_2d;
    adaptive_gnss_pos_weight_last_nis_ros_time_sec_ = now_sec;
  }

  AdaptiveGnssPosWeightDebug updateAdaptiveGnssPositionWeight_(
    double now_sec,
    const HeadingMotionContext & motion_ctx,
    double residual_h_m,
    double selected_std_h_m)
  {
    AdaptiveGnssPosWeightDebug debug;
    debug.enabled = adaptive_gnss_pos_weight_enable_;
    debug.residual_h_m = residual_h_m;
    debug.selected_std_h_m = selected_std_h_m;

    if (!adaptive_gnss_pos_weight_enable_) {
      resetAdaptiveGnssPositionWeightState_();
      debug.floor_state_h_m = adaptive_gnss_pos_weight_floor_state_h_m_;
      debug.effective_std_h_m = selected_std_h_m;
      debug.multiplier_h =
        (std::isfinite(selected_std_h_m) && selected_std_h_m > 0.0) ? 1.0
                                                                    : std::numeric_limits<double>::quiet_NaN();
      return debug;
    }

    const double floor_min_h_m =
      std::max(0.0, std::abs(adaptive_gnss_pos_weight_floor_min_h_m_));
    const double floor_max_h_m =
      std::max(floor_min_h_m, std::abs(adaptive_gnss_pos_weight_floor_max_h_m_));
    const double floor_nominal_h_m =
      std::clamp(
        std::abs(adaptive_gnss_pos_weight_floor_nominal_h_m_),
        floor_min_h_m,
        floor_max_h_m);
    const double residual_start_h_m =
      std::max(0.0, std::abs(adaptive_gnss_pos_weight_residual_start_h_m_));
    const double residual_full_h_m =
      std::max(
        residual_start_h_m + 1.0e-9,
        std::abs(adaptive_gnss_pos_weight_residual_full_h_m_));
    const bool use_nis_h_trigger = adaptive_gnss_pos_weight_trigger_source_ == "nis_h";
    const std::string trigger_phase = use_nis_h_trigger ? "nis_h" : "residual_h";
    double trigger_value = residual_h_m;
    double trigger_start = residual_start_h_m;
    double trigger_full = residual_full_h_m;
    bool trigger_stale = false;
    if (use_nis_h_trigger) {
      const double max_age_sec =
        std::max(0.0, std::abs(adaptive_gnss_pos_weight_nis_max_age_sec_));
      const bool age_valid =
        std::isfinite(now_sec) &&
        std::isfinite(adaptive_gnss_pos_weight_last_nis_ros_time_sec_);
      const double age_sec =
        age_valid ? now_sec - adaptive_gnss_pos_weight_last_nis_ros_time_sec_
                  : std::numeric_limits<double>::quiet_NaN();
      const bool nis_fresh =
        std::isfinite(adaptive_gnss_pos_weight_last_nis_h_2d_) &&
        age_valid &&
        age_sec >= -1.0e-3 &&
        (max_age_sec <= 0.0 || age_sec <= max_age_sec);
      trigger_stale =
        std::isfinite(adaptive_gnss_pos_weight_last_nis_h_2d_) && !nis_fresh;
      trigger_value =
        nis_fresh ? adaptive_gnss_pos_weight_last_nis_h_2d_
                  : std::numeric_limits<double>::quiet_NaN();
      trigger_start = std::max(0.0, std::abs(adaptive_gnss_pos_weight_nis_start_h_2d_));
      trigger_full =
        std::max(
          trigger_start + 1.0e-9,
          std::abs(adaptive_gnss_pos_weight_nis_full_h_2d_));
    }
    debug.residual_h_m = trigger_value;

    const bool in_mission = mavros_mode_ == "AUTO.MISSION";
    const bool in_rtl = mavros_mode_ == "AUTO.RTL";
    const bool terminal_disabled = motion_ctx.terminal_descent_context;
    const bool mission_enabled =
      adaptive_gnss_pos_weight_apply_mission_ && in_mission &&
      mavros_armed_ && !terminal_disabled;
    const bool rtl_enabled =
      adaptive_gnss_pos_weight_apply_rtl_ && in_rtl &&
      mavros_armed_ && !terminal_disabled;
    const bool adaptive_context = mission_enabled || rtl_enabled;
    const bool trigger_valid = std::isfinite(trigger_value);

    if (!adaptive_context) {
      resetAdaptiveGnssPositionWeightState_();
      adaptive_gnss_pos_weight_last_update_sec_ = now_sec;
      debug.target_floor_h_m = 0.0;
      debug.floor_state_h_m = adaptive_gnss_pos_weight_floor_state_h_m_;
      debug.persistent_count = adaptive_gnss_pos_weight_persistent_count_;
      debug.effective_std_h_m = selected_std_h_m;
      debug.multiplier_h =
        (std::isfinite(selected_std_h_m) && selected_std_h_m > 0.0)
          ? 1.0
          : std::numeric_limits<double>::quiet_NaN();
      if (in_rtl && !adaptive_gnss_pos_weight_apply_rtl_) {
        debug.reason = "rtl_disabled_reset";
      } else if (terminal_disabled) {
        debug.reason = "terminal_disabled_reset";
      } else if (!mavros_armed_) {
        debug.reason = "disarmed_disabled_reset";
      } else if (in_mission && !adaptive_gnss_pos_weight_apply_mission_) {
        debug.reason = "mission_disabled_reset";
      } else if (!in_mission && !in_rtl) {
        debug.reason = "non_auto_reset";
      } else {
        debug.reason = "context_reset";
      }
      return debug;
    }

    if (trigger_valid && trigger_value >= trigger_start) {
      adaptive_gnss_pos_weight_persistent_count_ += 1;
    } else if (trigger_valid && trigger_value >= 0.8 * trigger_start) {
      adaptive_gnss_pos_weight_persistent_count_ =
        std::max(0, adaptive_gnss_pos_weight_persistent_count_ - 1);
    } else {
      adaptive_gnss_pos_weight_persistent_count_ = 0;
    }

    double target_floor_h_m = 0.0;
    if (trigger_valid &&
        adaptive_gnss_pos_weight_persistent_count_ >=
          adaptive_gnss_pos_weight_persistence_updates_) {
      double context_gain = 1.0;
      if (motion_ctx.armed_cruise_force_relock_context) {
        context_gain *= std::max(0.0, adaptive_gnss_pos_weight_armed_cruise_gain_);
      }
      if (motion_ctx.turning_now) {
        context_gain *= std::max(0.0, adaptive_gnss_pos_weight_turn_gain_);
      }
      if (motion_ctx.post_turn_context) {
        context_gain *= std::max(0.0, adaptive_gnss_pos_weight_post_turn_gain_);
      }

      const double raw_score =
        (trigger_value - trigger_start) /
        std::max(1.0e-9, trigger_full - trigger_start);
      const double score = std::clamp(raw_score * context_gain, 0.0, 1.0);
      target_floor_h_m =
        floor_min_h_m + score * (floor_max_h_m - floor_min_h_m);
      target_floor_h_m =
        std::max(target_floor_h_m, score > 0.5 ? floor_nominal_h_m : floor_min_h_m);

      debug.reason =
        std::string(in_rtl ? "rtl_" : "mission_") +
        (use_nis_h_trigger ? "nis_persistent" : "residual_persistent");
      if (motion_ctx.armed_cruise_force_relock_context) {
        debug.reason += "+armed_cruise";
      }
      if (motion_ctx.turning_now) {
        debug.reason += "+turning";
      }
      if (motion_ctx.post_turn_context) {
        debug.reason += "+post_turn";
      }
    } else if (in_rtl) {
      debug.reason = "rtl_decay";
    } else if (terminal_disabled) {
      debug.reason = "terminal_disabled";
    } else if (!mavros_armed_) {
      debug.reason = "disarmed_disabled";
    } else if (in_mission && !adaptive_gnss_pos_weight_apply_mission_) {
      debug.reason = "mission_disabled";
    } else if (in_rtl && !adaptive_gnss_pos_weight_apply_rtl_) {
      debug.reason = "rtl_disabled";
    } else if (!in_mission && !in_rtl) {
      debug.reason = "non_auto_disabled";
    } else if (!trigger_valid) {
      debug.reason =
        use_nis_h_trigger ? (trigger_stale ? "stale_nis" : "no_nis")
                          : "no_residual";
    } else {
      debug.reason = "below_persistence";
    }

    const double dt_sec =
      (std::isfinite(adaptive_gnss_pos_weight_last_update_sec_) &&
       std::isfinite(now_sec))
        ? std::clamp(now_sec - adaptive_gnss_pos_weight_last_update_sec_, 0.0, 5.0)
        : 0.0;
    adaptive_gnss_pos_weight_last_update_sec_ = now_sec;

    const bool rising = target_floor_h_m > adaptive_gnss_pos_weight_floor_state_h_m_;
    const double time_constant_sec =
      rising ? std::max(0.0, adaptive_gnss_pos_weight_attack_sec_)
             : std::max(0.0, adaptive_gnss_pos_weight_decay_sec_);
    const double alpha =
      time_constant_sec <= 0.0 ? 1.0 : std::clamp(dt_sec / time_constant_sec, 0.0, 1.0);
    adaptive_gnss_pos_weight_floor_state_h_m_ +=
      alpha * (target_floor_h_m - adaptive_gnss_pos_weight_floor_state_h_m_);
    if (adaptive_gnss_pos_weight_floor_state_h_m_ < 1.0e-5) {
      adaptive_gnss_pos_weight_floor_state_h_m_ = 0.0;
    }

    debug.target_floor_h_m = target_floor_h_m;
    debug.floor_state_h_m = adaptive_gnss_pos_weight_floor_state_h_m_;
    debug.persistent_count = adaptive_gnss_pos_weight_persistent_count_;
    debug.effective_std_h_m =
      std::isfinite(selected_std_h_m)
        ? std::max(selected_std_h_m, debug.floor_state_h_m)
        : debug.floor_state_h_m;
    debug.multiplier_h =
      (std::isfinite(selected_std_h_m) && selected_std_h_m > 0.0)
        ? debug.effective_std_h_m / selected_std_h_m
        : std::numeric_limits<double>::quiet_NaN();
    debug.applied =
      std::isfinite(selected_std_h_m) &&
      debug.effective_std_h_m > selected_std_h_m + 1.0e-4;
    debug.active = adaptive_context && debug.applied;
    if (debug.active) {
      debug.phase = std::string(in_rtl ? "rtl_" : "mission_") + trigger_phase + "_active";
    } else if ((in_rtl || in_mission) && debug.applied) {
      debug.phase = std::string(in_rtl ? "rtl_" : "mission_") + trigger_phase + "_decay";
    } else {
      debug.phase = "disabled";
    }

    return debug;
  }

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
    const bool terminal_descent_armed_time_ok =
      terminal_descent_min_armed_time_sec_ <= 0.0 ||
      (std::isfinite(last_armed_transition_time_sec_) &&
       (now_sec - last_armed_transition_time_sec_) >= terminal_descent_min_armed_time_sec_);
    const bool terminal_descent_gyro_ok =
      !std::isfinite(last_imu_gyro_norm_deg_s_) ||
      terminal_descent_max_gyro_deg_s_ <= 0.0 ||
      last_imu_gyro_norm_deg_s_ <= terminal_descent_max_gyro_deg_s_;
    const bool terminal_descent_source_rate_ok =
      !std::isfinite(last_mavros_heading_rate_deg_s_) ||
      terminal_descent_max_source_yaw_rate_deg_s_ <= 0.0 ||
      std::abs(last_mavros_heading_rate_deg_s_) <=
        terminal_descent_max_source_yaw_rate_deg_s_;
    ctx.terminal_descent_mode_ok =
      !terminal_descent_require_rtl_mode_ || isTerminalDescentFlightMode_();
    ctx.terminal_descent_context =
      terminal_descent_observation_enable_ &&
      mavros_armed_ &&
      ctx.have_fresh_mavros_speed &&
      ctx.terminal_descent_mode_ok &&
      terminal_descent_armed_time_ok &&
      (terminal_descent_max_horizontal_speed_mps_ <= 0.0 ||
       last_mavros_horizontal_speed_mps_ <= terminal_descent_max_horizontal_speed_mps_) &&
      (terminal_descent_min_vertical_speed_mps_ <= 0.0 ||
       last_mavros_vertical_speed_mps_ >= terminal_descent_min_vertical_speed_mps_) &&
      terminal_descent_gyro_ok &&
      terminal_descent_source_rate_ok;
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
	      << "publish_stamp_mode,publish_core_to_ros_offset_sec,publish_stamp_observed_offset_sec,"
	      << "publish_stamp_selected_minus_now_sec,publish_core_stamp_offset_bias_sec,"
	      << "core_time_minus_ros_sec,gnss_update_time_minus_ros_sec,gnss_source_time_minus_ros_sec,"
      << "mavros_armed,core_initialized,odom_uses_gnss_pose,have_core_enu,have_gnss_enu,"
      << "published_enu_e_m,published_enu_n_m,published_enu_u_m,"
      << "raw_publish_enu_e_m,raw_publish_enu_n_m,raw_publish_enu_u_m,"
      << "px4_sphere_publish_enu_e_m,px4_sphere_publish_enu_n_m,px4_sphere_publish_enu_u_m,"
      << "alpha_projected_publish_enu_e_m,alpha_projected_publish_enu_n_m,alpha_projected_publish_enu_u_m,"
      << "core_enu_e_m,core_enu_n_m,core_enu_u_m,"
      << "gnss_enu_e_m,gnss_enu_n_m,gnss_enu_u_m,"
      << "core_minus_gnss_e_m,core_minus_gnss_n_m,core_minus_gnss_u_m,"
      << "core_gnss_diff_h_m,core_gnss_diff_3d_m,"
      << "core_velocity_vN_mps,core_velocity_vE_mps,core_velocity_vD_mps,core_velocity_vU_mps,"
      << "mavros_horizontal_speed_mps,mavros_vertical_speed_mps,"
      << "publish_projection_action,publish_projection_active,publish_projection_alpha,"
      << "segment_timing_gate_projection_enabled,segment_timing_gate_projection_active,"
      << "segment_timing_gate_phase_allowed,segment_timing_gate_current_active,"
      << "segment_timing_gate_current_lag_mean_sec,segment_timing_gate_current_lag_latest_sec,"
      << "segment_timing_gate_current_gnss_source_age_mean_sec,"
      << "segment_timing_gate_core_gnss_along_min_enable,"
      << "segment_timing_gate_current_core_gnss_along_min_m,"
      << "segment_timing_gate_core_gnss_along_min_threshold_m,"
      << "segment_timing_gate_gnss_source_age_max_sec,segment_timing_gate_projection_alpha,"
      << "accbias_z_history_projection_enabled,accbias_z_history_projection_active,"
      << "accbias_z_history_projection_phase_allowed,accbias_z_history_projection_have_history,"
      << "accbias_z_history_deep_frac,accbias_z_history_total_count,accbias_z_history_deep_count,"
      << "accbias_z_history_latest_before_mps2,accbias_z_history_latest_after_mps2,"
      << "accbias_z_history_deep_threshold_mps2,accbias_z_history_frac_threshold,"
      << "accbias_z_history_start_sec,"
      << "segment_timing_gate_lag_threshold_sec,mavros_mode\n";
    state_publish_debug_csv_.flush();
    state_publish_debug_rows_since_flush_ = 0;

    RCLCPP_INFO(
      get_logger(),
      "State publish debug CSV enabled: %s",
      state_publish_debug_csv_path_.c_str());
  }

  void openSegmentTimingGateDebugCsv_()
  {
    if (segment_timing_gate_debug_csv_path_.empty()) {
      return;
    }

    try {
      const std::filesystem::path csv_path(segment_timing_gate_debug_csv_path_);
      if (csv_path.has_parent_path()) {
        std::filesystem::create_directories(csv_path.parent_path());
      }
      segment_timing_gate_debug_csv_.open(csv_path, std::ios::out | std::ios::trunc);
    } catch (const std::exception & e) {
      RCLCPP_WARN(
        get_logger(),
        "Failed to open segment timing gate debug CSV %s: %s",
        segment_timing_gate_debug_csv_path_.c_str(),
        e.what());
      return;
    }

    if (!segment_timing_gate_debug_csv_.is_open()) {
      RCLCPP_WARN(
        get_logger(),
        "Failed to open segment timing gate debug CSV %s",
        segment_timing_gate_debug_csv_path_.c_str());
      return;
    }

    segment_timing_gate_debug_csv_ << std::setprecision(std::numeric_limits<double>::max_digits10);
    segment_timing_gate_debug_csv_
      << "sequence,ros_time_sec,segment_index,segment_start_armed_time_sec,"
      << "segment_end_armed_time_sec,segment_duration_sec,row_count,"
      << "stamp_lag_mean_sec,stamp_lag_min_sec,stamp_lag_max_sec,"
      << "stamp_lag_threshold_sec,gnss_source_age_max_sec,"
      << "core_gnss_along_min_enable,core_gnss_along_min_m,"
      << "core_gnss_along_min_threshold_m,gate_active,active_duration_sec,"
      << "mavros_mode_last,mavros_armed,turning_now_mean,post_turn_context_mean,"
      << "armed_cruise_context_mean,horizontal_speed_mean_mps,vertical_speed_mean_mps,"
      << "core_age_mean_sec,gnss_source_age_mean_sec,publish_stamp_mode\n";
    segment_timing_gate_debug_csv_.flush();
    segment_timing_gate_debug_rows_since_flush_ = 0;
    resetSegmentTimingGateAccumulator_();

    RCLCPP_INFO(
      get_logger(),
      "Segment timing gate debug CSV enabled: %s (segment=%.3fs, lag_threshold=%.3fs)",
      segment_timing_gate_debug_csv_path_.c_str(),
      segment_timing_gate_segment_sec_,
      segment_timing_gate_lag_threshold_sec_);
  }

  void resetSegmentTimingGateAccumulator_()
  {
    segment_timing_gate_current_index_ = -1;
    segment_timing_gate_row_count_ = 0;
    segment_timing_gate_stamp_lag_sum_sec_ = 0.0;
    segment_timing_gate_stamp_lag_min_sec_ = std::numeric_limits<double>::infinity();
    segment_timing_gate_stamp_lag_max_sec_ = -std::numeric_limits<double>::infinity();
    segment_timing_gate_turning_sum_ = 0.0;
    segment_timing_gate_post_turn_sum_ = 0.0;
    segment_timing_gate_armed_cruise_sum_ = 0.0;
    segment_timing_gate_hspeed_sum_mps_ = 0.0;
    segment_timing_gate_vspeed_sum_mps_ = 0.0;
    segment_timing_gate_core_age_sum_sec_ = 0.0;
    segment_timing_gate_gnss_source_age_sum_sec_ = 0.0;
    segment_timing_gate_core_gnss_along_min_m_ =
      std::numeric_limits<double>::infinity();
    segment_timing_gate_hspeed_count_ = 0;
    segment_timing_gate_vspeed_count_ = 0;
    segment_timing_gate_core_age_count_ = 0;
    segment_timing_gate_gnss_source_age_count_ = 0;
    segment_timing_gate_core_gnss_along_count_ = 0;
    segment_timing_gate_armed_last_ = false;
    segment_timing_gate_mode_last_.clear();
    segment_timing_gate_current_active_ = false;
    segment_timing_gate_current_lag_mean_sec_ = std::numeric_limits<double>::quiet_NaN();
    segment_timing_gate_current_lag_latest_sec_ = std::numeric_limits<double>::quiet_NaN();
    segment_timing_gate_current_gnss_source_age_mean_sec_ =
      std::numeric_limits<double>::quiet_NaN();
    segment_timing_gate_current_core_gnss_along_min_m_ =
      std::numeric_limits<double>::quiet_NaN();
  }

  void flushSegmentTimingGateDebug_(double ros_time_sec, bool force)
  {
    if (segment_timing_gate_current_index_ < 0 ||
        segment_timing_gate_row_count_ == 0) {
      return;
    }

    const double segment_start_sec =
      static_cast<double>(segment_timing_gate_current_index_) * segment_timing_gate_segment_sec_;
    const double segment_end_sec = segment_start_sec + segment_timing_gate_segment_sec_;
    double segment_log_end_sec = segment_end_sec;
    if (force && std::isfinite(last_armed_transition_time_sec_)) {
      const double armed_time_sec = ros_time_sec - last_armed_transition_time_sec_;
      if (std::isfinite(armed_time_sec)) {
        segment_log_end_sec = std::min(segment_end_sec, std::max(segment_start_sec, armed_time_sec));
      }
    }
    const double segment_duration_sec = std::max(0.0, segment_log_end_sec - segment_start_sec);
    if (!force && std::isfinite(last_armed_transition_time_sec_)) {
      const double armed_time_sec = ros_time_sec - last_armed_transition_time_sec_;
      if (armed_time_sec < segment_end_sec) {
        return;
      }
    }

    const double count = static_cast<double>(segment_timing_gate_row_count_);
    const double stamp_lag_mean = segment_timing_gate_stamp_lag_sum_sec_ / count;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    auto mean_or_nan = [nan](double sum, std::size_t n) {
      return n > 0 ? sum / static_cast<double>(n) : nan;
    };
    const double gnss_source_age_mean_sec = mean_or_nan(
      segment_timing_gate_gnss_source_age_sum_sec_,
      segment_timing_gate_gnss_source_age_count_);
    const double core_gnss_along_min_m =
      segment_timing_gate_core_gnss_along_count_ > 0
        ? segment_timing_gate_core_gnss_along_min_m_
        : nan;
    const bool gate_active =
      stamp_lag_mean >= segment_timing_gate_lag_threshold_sec_ &&
      segmentTimingGateGnssSourceAgeAllowed_(gnss_source_age_mean_sec) &&
      segmentTimingGateCoreGnssAlongMinAllowed_(core_gnss_along_min_m);
    if (gate_active) {
      segment_timing_gate_active_duration_sec_ += segment_duration_sec;
    }

    if (segment_timing_gate_debug_csv_.is_open()) {
      segment_timing_gate_debug_csv_
        << (++segment_timing_gate_debug_sequence_) << ','
        << ros_time_sec << ','
        << segment_timing_gate_current_index_ << ','
        << segment_start_sec << ','
        << segment_log_end_sec << ','
        << segment_duration_sec << ','
        << segment_timing_gate_row_count_ << ','
        << stamp_lag_mean << ','
        << segment_timing_gate_stamp_lag_min_sec_ << ','
        << segment_timing_gate_stamp_lag_max_sec_ << ','
        << segment_timing_gate_lag_threshold_sec_ << ','
        << segment_timing_gate_gnss_source_age_max_sec_ << ','
        << (segment_timing_gate_core_gnss_along_min_enable_ ? 1 : 0) << ','
        << core_gnss_along_min_m << ','
        << segment_timing_gate_core_gnss_along_min_threshold_m_ << ','
        << (gate_active ? 1 : 0) << ','
        << segment_timing_gate_active_duration_sec_ << ','
        << segment_timing_gate_mode_last_ << ','
        << (segment_timing_gate_armed_last_ ? 1 : 0) << ','
        << (segment_timing_gate_turning_sum_ / count) << ','
        << (segment_timing_gate_post_turn_sum_ / count) << ','
        << (segment_timing_gate_armed_cruise_sum_ / count) << ','
        << mean_or_nan(segment_timing_gate_hspeed_sum_mps_, segment_timing_gate_hspeed_count_) << ','
        << mean_or_nan(segment_timing_gate_vspeed_sum_mps_, segment_timing_gate_vspeed_count_) << ','
        << mean_or_nan(segment_timing_gate_core_age_sum_sec_, segment_timing_gate_core_age_count_) << ','
        << gnss_source_age_mean_sec << ','
        << last_publish_stamp_applied_mode_ << '\n';

      ++segment_timing_gate_debug_rows_since_flush_;
      if (segment_timing_gate_debug_rows_since_flush_ >= segment_timing_gate_debug_flush_interval_) {
        segment_timing_gate_debug_csv_.flush();
        segment_timing_gate_debug_rows_since_flush_ = 0;
      }
    }
    resetSegmentTimingGateAccumulator_();
  }

  void updateSegmentTimingGateDebug_(
    double ros_time_sec,
    bool have_core_enu,
    const Eigen::Vector3d & enu_core,
    bool have_gnss_enu,
    const Eigen::Vector3d & enu_gnss,
    double core_vN_mps,
    double core_vE_mps)
  {
    if (!segmentTimingGateNeedsUpdate_()) {
      return;
    }
    if (!mavros_armed_ || !std::isfinite(last_armed_transition_time_sec_)) {
      flushSegmentTimingGateDebug_(ros_time_sec, true);
      resetSegmentTimingGateAccumulator_();
      segment_timing_gate_active_duration_sec_ = 0.0;
      return;
    }

    const double armed_time_sec = ros_time_sec - last_armed_transition_time_sec_;
    if (!std::isfinite(armed_time_sec) || armed_time_sec < 0.0) {
      return;
    }
    const int segment_index =
      static_cast<int>(std::floor(armed_time_sec / segment_timing_gate_segment_sec_));
    if (segment_timing_gate_current_index_ >= 0 &&
        segment_index != segment_timing_gate_current_index_) {
      flushSegmentTimingGateDebug_(ros_time_sec, true);
    }
    if (segment_timing_gate_current_index_ < 0) {
      segment_timing_gate_current_index_ = segment_index;
    }

    const double stamp_lag_sec = -last_publish_stamp_selected_minus_now_sec_;
    if (!std::isfinite(stamp_lag_sec)) {
      return;
    }
    const HeadingMotionContext motion_ctx = buildHeadingMotionContext_(ros_time_sec, false);
    const bool have_core_time =
      std::isfinite(last_core_time_) && std::isfinite(publish_core_to_ros_offset_sec_);
    const double core_age_sec = have_core_time
      ? ros_time_sec - (last_core_time_ + publish_core_to_ros_offset_sec_)
      : std::numeric_limits<double>::quiet_NaN();
    const double gnss_source_age_sec = std::isfinite(last_gnss_source_time_sec_)
      ? ros_time_sec - last_gnss_source_time_sec_
      : std::numeric_limits<double>::quiet_NaN();
    double core_gnss_along_velocity_m = std::numeric_limits<double>::quiet_NaN();
    const bool have_core_gnss_diff = have_core_enu && have_gnss_enu;
    const double core_hspeed_mps = std::hypot(core_vE_mps, core_vN_mps);
    if (have_core_gnss_diff &&
        std::isfinite(core_vE_mps) &&
        std::isfinite(core_vN_mps) &&
        core_hspeed_mps > 1e-6) {
      const double unit_e = core_vE_mps / core_hspeed_mps;
      const double unit_n = core_vN_mps / core_hspeed_mps;
      const Eigen::Vector3d diff = enu_core - enu_gnss;
      core_gnss_along_velocity_m = diff.x() * unit_e + diff.y() * unit_n;
    }

    ++segment_timing_gate_row_count_;
    segment_timing_gate_stamp_lag_sum_sec_ += stamp_lag_sec;
    segment_timing_gate_stamp_lag_min_sec_ =
      std::min(segment_timing_gate_stamp_lag_min_sec_, stamp_lag_sec);
    segment_timing_gate_stamp_lag_max_sec_ =
      std::max(segment_timing_gate_stamp_lag_max_sec_, stamp_lag_sec);
    segment_timing_gate_turning_sum_ += motion_ctx.turning_now ? 1.0 : 0.0;
    segment_timing_gate_post_turn_sum_ += motion_ctx.post_turn_context ? 1.0 : 0.0;
    segment_timing_gate_armed_cruise_sum_ +=
      motion_ctx.armed_cruise_force_relock_context ? 1.0 : 0.0;
    if (std::isfinite(last_mavros_horizontal_speed_mps_)) {
      segment_timing_gate_hspeed_sum_mps_ += last_mavros_horizontal_speed_mps_;
      ++segment_timing_gate_hspeed_count_;
    }
    if (std::isfinite(last_mavros_vertical_speed_mps_)) {
      segment_timing_gate_vspeed_sum_mps_ += last_mavros_vertical_speed_mps_;
      ++segment_timing_gate_vspeed_count_;
    }
    if (std::isfinite(core_age_sec)) {
      segment_timing_gate_core_age_sum_sec_ += core_age_sec;
      ++segment_timing_gate_core_age_count_;
    }
    if (std::isfinite(gnss_source_age_sec)) {
      segment_timing_gate_gnss_source_age_sum_sec_ += gnss_source_age_sec;
      ++segment_timing_gate_gnss_source_age_count_;
    }
    if (std::isfinite(core_gnss_along_velocity_m)) {
      segment_timing_gate_core_gnss_along_min_m_ =
        std::min(segment_timing_gate_core_gnss_along_min_m_, core_gnss_along_velocity_m);
      ++segment_timing_gate_core_gnss_along_count_;
    }
    segment_timing_gate_armed_last_ = mavros_armed_;
    segment_timing_gate_mode_last_ = mavros_mode_;
    segment_timing_gate_current_lag_latest_sec_ = stamp_lag_sec;
    segment_timing_gate_current_lag_mean_sec_ =
      segment_timing_gate_stamp_lag_sum_sec_ /
      static_cast<double>(segment_timing_gate_row_count_);
    segment_timing_gate_current_gnss_source_age_mean_sec_ =
      segment_timing_gate_gnss_source_age_count_ > 0
        ? segment_timing_gate_gnss_source_age_sum_sec_ /
          static_cast<double>(segment_timing_gate_gnss_source_age_count_)
        : std::numeric_limits<double>::quiet_NaN();
    segment_timing_gate_current_core_gnss_along_min_m_ =
      segment_timing_gate_core_gnss_along_count_ > 0
        ? segment_timing_gate_core_gnss_along_min_m_
        : std::numeric_limits<double>::quiet_NaN();
    segment_timing_gate_current_active_ =
      segment_timing_gate_current_lag_mean_sec_ >= segment_timing_gate_lag_threshold_sec_ &&
      segmentTimingGateGnssSourceAgeAllowed_(
        segment_timing_gate_current_gnss_source_age_mean_sec_) &&
      segmentTimingGateCoreGnssAlongMinAllowed_(
        segment_timing_gate_current_core_gnss_along_min_m_);
  }

  EarlyRecoveryBiasFeedbackDebug evaluateEarlyRecoveryBiasFeedbackCounterfactual_(
    const kfcore::StateUpdateDebug & event,
    double armed_time_sec,
    double accbias_z_before_mps2,
    double residual_u_m,
    double core_gnss_u_m,
    double raw_dx_ba_z_mps2)
  {
    EarlyRecoveryBiasFeedbackDebug debug;
    debug.enabled = early_recovery_bias_feedback_debug_enable_;
    debug.history_sec = early_recovery_bias_feedback_history_sec_;
    debug.armed_time_sec = armed_time_sec;
    debug.raw_dx_ba_z_mps2 = raw_dx_ba_z_mps2;
    debug.selected_dx_ba_z_mps2 = raw_dx_ba_z_mps2;
    debug.delta_dx_ba_z_mps2 = 0.0;
    debug.negative_dx_scale = early_recovery_bias_feedback_negative_dx_scale_;
    if (std::isfinite(accbias_z_before_mps2) && std::isfinite(raw_dx_ba_z_mps2)) {
      debug.selected_accbias_z_after_mps2 = accbias_z_before_mps2 + raw_dx_ba_z_mps2;
    }

    if (!early_recovery_bias_feedback_debug_enable_) {
      debug.reason = "disabled";
      return debug;
    }
    if (event.event_type != "gnss_position") {
      debug.reason = "non_gnss_position";
      return debug;
    }
    debug.candidate = true;

    if (!std::isfinite(armed_time_sec) ||
        !std::isfinite(accbias_z_before_mps2) ||
        !std::isfinite(residual_u_m) ||
        !std::isfinite(core_gnss_u_m) ||
        !std::isfinite(raw_dx_ba_z_mps2)) {
      debug.reason = "missing_signal";
      return debug;
    }

    if (!early_recovery_bias_feedback_history_.empty() &&
        std::isfinite(early_recovery_bias_feedback_history_.back().armed_time_sec) &&
        armed_time_sec <
          early_recovery_bias_feedback_history_.back().armed_time_sec - 1.0e-6) {
      early_recovery_bias_feedback_history_.clear();
    }

    early_recovery_bias_feedback_history_.push_back(
      EarlyRecoveryBiasFeedbackSample{
        armed_time_sec,
        accbias_z_before_mps2,
        residual_u_m,
        core_gnss_u_m,
        raw_dx_ba_z_mps2});
    while (!early_recovery_bias_feedback_history_.empty()) {
      const auto & oldest = early_recovery_bias_feedback_history_.front();
      if (!std::isfinite(oldest.armed_time_sec) ||
          armed_time_sec - oldest.armed_time_sec >
            early_recovery_bias_feedback_history_sec_ + 1.0e-9) {
        early_recovery_bias_feedback_history_.pop_front();
      } else {
        break;
      }
    }

    double ba_sum = 0.0;
    double residual_sum = 0.0;
    double core_sum = 0.0;
    double dx_sum = 0.0;
    double neg_dx_sum = 0.0;
    double pos_dx_sum = 0.0;
    int rows = 0;
    for (const auto & sample : early_recovery_bias_feedback_history_) {
      if (!std::isfinite(sample.armed_time_sec) ||
          !std::isfinite(sample.ba_z_before_mps2) ||
          !std::isfinite(sample.residual_u_m) ||
          !std::isfinite(sample.core_gnss_u_m) ||
          !std::isfinite(sample.dx_ba_z_mps2)) {
        continue;
      }
      ++rows;
      ba_sum += sample.ba_z_before_mps2;
      residual_sum += sample.residual_u_m;
      core_sum += sample.core_gnss_u_m;
      dx_sum += sample.dx_ba_z_mps2;
      if (sample.dx_ba_z_mps2 < 0.0) {
        neg_dx_sum += sample.dx_ba_z_mps2;
      } else if (sample.dx_ba_z_mps2 > 0.0) {
        pos_dx_sum += sample.dx_ba_z_mps2;
      }
    }

    debug.history_rows = rows;
    if (rows > 0) {
      const double denom = static_cast<double>(rows);
      debug.ba_z_mean_mps2 = ba_sum / denom;
      debug.residual_u_mean_m = residual_sum / denom;
      debug.core_gnss_u_mean_m = core_sum / denom;
      debug.dx_ba_z_sum_mps2 = dx_sum;
      debug.negative_dx_ba_z_sum_mps2 = neg_dx_sum;
      debug.positive_dx_ba_z_sum_mps2 = pos_dx_sum;
    }

    if (armed_time_sec < early_recovery_bias_feedback_min_armed_time_sec_ ||
        armed_time_sec > early_recovery_bias_feedback_max_armed_time_sec_) {
      debug.reason = "outside_time_window";
      return debug;
    }
    if (rows < early_recovery_bias_feedback_min_history_rows_) {
      debug.reason = "insufficient_history";
      return debug;
    }
    if (!std::isfinite(debug.ba_z_mean_mps2) ||
        debug.ba_z_mean_mps2 > early_recovery_bias_feedback_ba_z_mean_max_mps2_) {
      debug.reason = "ba_z_mean_high";
      return debug;
    }
    if (!std::isfinite(debug.residual_u_mean_m) ||
        debug.residual_u_mean_m > early_recovery_bias_feedback_residual_u_mean_max_m_) {
      debug.reason = "residual_u_mean_high";
      return debug;
    }
    if (!std::isfinite(debug.core_gnss_u_mean_m) ||
        debug.core_gnss_u_mean_m < early_recovery_bias_feedback_core_gnss_u_mean_min_m_) {
      debug.reason = "core_gnss_u_mean_low";
      return debug;
    }
    if (!std::isfinite(debug.dx_ba_z_sum_mps2) ||
        debug.dx_ba_z_sum_mps2 >= early_recovery_bias_feedback_dx_ba_z_sum_max_mps2_) {
      debug.reason = "dx_ba_z_sum_nonnegative";
      return debug;
    }

    debug.active = true;
    debug.reason = "active";
    if (raw_dx_ba_z_mps2 < 0.0) {
      debug.selected_dx_ba_z_mps2 =
        raw_dx_ba_z_mps2 * early_recovery_bias_feedback_negative_dx_scale_;
      debug.delta_dx_ba_z_mps2 = debug.selected_dx_ba_z_mps2 - raw_dx_ba_z_mps2;
      debug.selected_accbias_z_after_mps2 =
        accbias_z_before_mps2 + debug.selected_dx_ba_z_mps2;
    }
    return debug;
  }

  void appendBoundedAdaptiveRHeader_(std::ostream & os, const std::string & prefix)
  {
    os
      << prefix << "_adaptive_r_enabled,"
      << prefix << "_adaptive_r_applied,"
      << prefix << "_adaptive_r_exceeded,"
      << prefix << "_adaptive_r_update_type,"
      << prefix << "_adaptive_r_mode,"
      << prefix << "_adaptive_rq_trigger,"
      << prefix << "_adaptive_rq_reason,"
      << prefix << "_adaptive_r_nis,"
      << prefix << "_adaptive_r_chi2_threshold,"
      << prefix << "_adaptive_r_nis_ratio,"
      << prefix << "_adaptive_rq_consecutive_exceed_count,"
      << prefix << "_adaptive_rq_hold_remaining,"
      << prefix << "_adaptive_rq_observation_score,"
      << prefix << "_adaptive_rq_process_score,"
      << prefix << "_adaptive_r_gamma_raw,"
      << prefix << "_adaptive_r_gamma_smoothed,"
      << prefix << "_adaptive_r_gamma_clipped,"
      << prefix << "_adaptive_r_gamma_limit,"
      << prefix << "_adaptive_rq_lambda_vrw,"
      << prefix << "_adaptive_rq_lambda_arw,"
      << prefix << "_adaptive_rq_lambda_accbias,"
      << prefix << "_adaptive_rq_lambda_gyrbias,"
      << prefix << "_adaptive_rq_gps_quality_stable,"
      << prefix << "_adaptive_rq_motion_context_ok,"
      << prefix << "_adaptive_rq_source_confidence,"
      << prefix << "_adaptive_rq_source_gate_allowed,"
      << prefix << "_adaptive_rq_source_gate_reason,"
      << prefix << "_adaptive_rq_velocity_evidence,"
      << prefix << "_adaptive_rq_velocity_nis_ratio,"
      << prefix << "_adaptive_rq_velocity_residual_h_mps,"
      << prefix << "_adaptive_r_base_diag_0,"
      << prefix << "_adaptive_r_base_diag_1,"
      << prefix << "_adaptive_r_base_diag_2,"
      << prefix << "_adaptive_r_eff_diag_0,"
      << prefix << "_adaptive_r_eff_diag_1,"
      << prefix << "_adaptive_r_eff_diag_2,";
  }

  void appendBoundedAdaptiveRValues_(
    std::ostream & os,
    const kfcore::BoundedAdaptiveRDebug & debug)
  {
    os
      << (debug.enabled ? 1 : 0) << ','
      << (debug.applied ? 1 : 0) << ','
      << (debug.exceeded ? 1 : 0) << ','
      << debug.update_type << ','
      << debug.mode << ','
      << debug.rq_selector_trigger << ','
      << debug.rq_selector_reason << ','
      << debug.nis << ','
      << debug.chi2_threshold << ','
      << debug.nis_ratio << ','
      << debug.consecutive_exceed_count << ','
      << debug.hold_remaining << ','
      << debug.observation_score << ','
      << debug.process_score << ','
      << debug.gamma_raw << ','
      << debug.gamma_smoothed << ','
      << debug.gamma_clipped << ','
      << debug.r_gamma_limit << ','
      << debug.q_lambda_vrw << ','
      << debug.q_lambda_arw << ','
      << debug.q_lambda_accbias << ','
      << debug.q_lambda_gyrbias << ','
      << (debug.gps_quality_stable ? 1 : 0) << ','
      << (debug.motion_context_ok ? 1 : 0) << ','
      << debug.q_source_confidence << ','
      << (debug.q_source_gate_allowed ? 1 : 0) << ','
      << debug.q_source_gate_reason << ','
      << (debug.q_velocity_evidence ? 1 : 0) << ','
      << debug.q_velocity_nis_ratio << ','
      << debug.q_velocity_residual_h_mps << ','
      << debug.r_base_diag.x() << ','
      << debug.r_base_diag.y() << ','
      << debug.r_base_diag.z() << ','
      << debug.r_eff_diag.x() << ','
      << debug.r_eff_diag.y() << ','
      << debug.r_eff_diag.z() << ',';
  }

  void openStateUpdateDebugCsv_()
  {
    if (state_update_debug_csv_path_.empty()) {
      return;
    }

    try {
      const std::filesystem::path csv_path(state_update_debug_csv_path_);
      if (csv_path.has_parent_path()) {
        std::filesystem::create_directories(csv_path.parent_path());
      }
      state_update_debug_csv_.open(csv_path, std::ios::out | std::ios::trunc);
    } catch (const std::exception & e) {
      RCLCPP_WARN(
        get_logger(),
        "Failed to open state update debug CSV %s: %s",
        state_update_debug_csv_path_.c_str(),
        e.what());
      return;
    }

    if (!state_update_debug_csv_.is_open()) {
      RCLCPP_WARN(
        get_logger(),
        "Failed to open state update debug CSV %s",
        state_update_debug_csv_path_.c_str());
      return;
    }

    state_update_debug_csv_ << std::setprecision(std::numeric_limits<double>::max_digits10);
    state_update_debug_csv_
      << "sequence,ros_time_sec,update_time_sec,event_type,reason,update_mode,update_mode_label,"
      << "applied,pending_debug_matched,mavros_mode,mavros_armed,armed_time_sec,"
      << "turning_now,post_turn_context,armed_cruise_context,terminal_descent_context,"
      << "horizontal_speed_mps,vertical_speed_mps,gyro_deg_s,source_yaw_rate_deg_s,"
      << "lat_before_deg,lon_before_deg,h_before_m,lat_after_deg,lon_after_deg,h_after_m,"
      << "state_update_have_enu,"
      << "core_enu_e_before_m,core_enu_n_before_m,core_enu_u_before_m,"
      << "core_enu_e_after_m,core_enu_n_after_m,core_enu_u_after_m,"
      << "px4_sphere_enu_e_before_m,px4_sphere_enu_n_before_m,px4_sphere_enu_u_before_m,"
      << "px4_sphere_enu_e_after_m,px4_sphere_enu_n_after_m,px4_sphere_enu_u_after_m,"
      << "publish_projection_active,publish_projection_action,px4_sphere_scale_e,px4_sphere_scale_n,"
      << "vN_before_mps,vE_before_mps,vD_before_mps,vN_after_mps,vE_after_mps,vD_after_mps,"
      << "roll_before_deg,pitch_before_deg,yaw_before_deg,roll_after_deg,pitch_after_deg,yaw_after_deg,"
      << "yaw_delta_deg,"
      << "gyrbias_x_before_radps,gyrbias_y_before_radps,gyrbias_z_before_radps,"
      << "gyrbias_x_after_radps,gyrbias_y_after_radps,gyrbias_z_after_radps,"
      << "accbias_x_before_mps2,accbias_y_before_mps2,accbias_z_before_mps2,"
      << "accbias_x_after_mps2,accbias_y_after_mps2,accbias_z_after_mps2,"
      << "dx_pos_n_m,dx_pos_e_m,dx_pos_u_m,dx_pos_h_norm_m,"
      << "gnss_position_observation_valid,observation_sequence,"
      << "gnss_position_residual_n_m,gnss_position_residual_e_m,gnss_position_residual_u_m,"
      << "gnss_position_residual_h_m,"
      << "gnss_position_std_n_m,gnss_position_std_e_m,gnss_position_std_u_m,"
      << "gnss_position_s_nn_m2,gnss_position_s_ne_m2,gnss_position_s_nu_m2,"
      << "gnss_position_s_ee_m2,gnss_position_s_eu_m2,gnss_position_s_uu_m2,"
      << "gnss_position_nis_h_2d,gnss_position_nis_u_1d,gnss_position_nis_3d,"
      << "gnss_position_gate_threshold_nis,"
      << "gnss_position_update_accepted,gnss_position_update_rejected,"
      << "gnss_position_update_reason,";
    appendBoundedAdaptiveRHeader_(state_update_debug_csv_, "gnss_position");
    appendBoundedAdaptiveRHeader_(state_update_debug_csv_, "gnss_velocity");
    state_update_debug_csv_
      << "dx_pos_h_over_residual_h,"
      << "kalman_gain_pn_from_meas_n,kalman_gain_pn_from_meas_e,"
      << "kalman_gain_pe_from_meas_n,kalman_gain_pe_from_meas_e,"
      << "kalman_gain_pu_from_meas_u,"
      << "kalman_gain_vn_from_meas_n,kalman_gain_ve_from_meas_e,"
      << "kalman_gain_yaw_from_meas_n,kalman_gain_yaw_from_meas_e,"
      << "kalman_gain_horizontal_trace,kalman_gain_horizontal_fro,"
      << "dx_vel_n_mps,dx_vel_e_mps,dx_vel_d_mps,dx_vel_h_norm_mps,"
      << "dx_phi_roll_rad,dx_phi_pitch_rad,dx_phi_yaw_rad,dx_phi_yaw_deg,"
      << "dx_bg_x_radps,dx_bg_y_radps,dx_bg_z_radps,"
      << "dx_ba_x_mps2,dx_ba_y_mps2,dx_ba_z_mps2,"
      << "early_recovery_bias_feedback_debug_enabled,"
      << "early_recovery_bias_feedback_apply_enabled,"
      << "early_recovery_bias_feedback_candidate,"
      << "early_recovery_bias_feedback_active,"
      << "early_recovery_bias_feedback_applied,"
      << "early_recovery_bias_feedback_reason,"
      << "early_recovery_bias_feedback_history_rows,"
      << "early_recovery_bias_feedback_history_sec,"
      << "early_recovery_bias_feedback_armed_time_sec,"
      << "early_recovery_bias_feedback_ba_z_mean_mps2,"
      << "early_recovery_bias_feedback_residual_u_mean_m,"
      << "early_recovery_bias_feedback_core_gnss_u_mean_m,"
      << "early_recovery_bias_feedback_dx_ba_z_sum_mps2,"
      << "early_recovery_bias_feedback_negative_dx_ba_z_sum_mps2,"
      << "early_recovery_bias_feedback_positive_dx_ba_z_sum_mps2,"
      << "early_recovery_bias_feedback_raw_dx_ba_z_mps2,"
      << "early_recovery_bias_feedback_selected_dx_ba_z_mps2,"
      << "early_recovery_bias_feedback_delta_dx_ba_z_mps2,"
      << "early_recovery_bias_feedback_selected_accbias_z_after_mps2,"
      << "early_recovery_bias_feedback_negative_dx_scale,"
      << "cov_pos_n_before_m2,cov_pos_e_before_m2,cov_pos_u_before_m2,cov_pos_ne_before_m2,"
      << "cov_pos_n_after_m2,cov_pos_e_after_m2,cov_pos_u_after_m2,cov_pos_ne_after_m2,"
      << "cov_vel_n_before_m2ps2,cov_vel_e_before_m2ps2,cov_vel_d_before_m2ps2,cov_vel_ne_before_m2ps2,"
      << "cov_vel_n_after_m2ps2,cov_vel_e_after_m2ps2,cov_vel_d_after_m2ps2,cov_vel_ne_after_m2ps2,"
      << "cov_roll_before_rad2,cov_pitch_before_rad2,cov_yaw_before_rad2,"
      << "cov_roll_after_rad2,cov_pitch_after_rad2,cov_yaw_after_rad2,"
      << "cov_bg_x_before_rad2ps2,cov_bg_y_before_rad2ps2,cov_bg_z_before_rad2ps2,"
      << "cov_bg_x_after_rad2ps2,cov_bg_y_after_rad2ps2,cov_bg_z_after_rad2ps2,"
      << "cov_ba_x_before_m2ps4,cov_ba_y_before_m2ps4,cov_ba_z_before_m2ps4,"
      << "cov_ba_x_after_m2ps4,cov_ba_y_after_m2ps4,cov_ba_z_after_m2ps4,"
      << "diag_gnss_source_time_sec,diag_gnss_rx_ros_time_sec,"
      << "diag_core_time_before_update_sec,diag_gnss_source_age_at_update_sec,"
      << "diag_gnss_rx_age_at_update_sec,"
      << "diag_core_time_before_update_minus_update_sec,"
      << "diag_gnss_update_minus_source_sec,diag_update_time_minus_ros_sec,"
      << "diag_latest_heading_update_age_sec,diag_recent_turnpost_age_sec,"
      << "diag_last_turning_age_sec,diag_post_turn_hold_remaining_sec,"
      << "diag_update_move_enu_e_m,diag_update_move_enu_n_m,diag_update_move_enu_u_m,"
      << "diag_update_move_h_m,diag_residual_enu_e_m,diag_residual_enu_n_m,"
      << "diag_residual_enu_u_m,diag_update_move_dot_residual_m2,"
      << "diag_update_move_residual_cos,"
      << "diag_core_velocity_before_enu_e_mps,diag_core_velocity_before_enu_n_mps,"
      << "diag_core_velocity_before_enu_u_mps,"
      << "diag_core_velocity_after_enu_e_mps,diag_core_velocity_after_enu_n_mps,"
      << "diag_core_velocity_after_enu_u_mps,"
      << "diag_projection_axis_enu_e_m,diag_projection_axis_enu_n_m,"
      << "diag_projection_axis_h_m,diag_update_move_along_projection_axis_m,"
      << "diag_update_move_perp_projection_axis_m,"
      << "diag_residual_along_projection_axis_m,"
      << "diag_residual_perp_projection_axis_m\n";
    state_update_debug_csv_.flush();
    state_update_debug_rows_since_flush_ = 0;

    RCLCPP_INFO(
      get_logger(),
      "State update debug CSV enabled: %s (heading_max_rate=%.3f Hz, non-heading updates unthrottled)",
      state_update_debug_csv_path_.c_str(),
      state_update_debug_max_rate_hz_);
  }

  void openDtrqRuntimeFeatureDebugCsv_()
  {
    if (dtrq_runtime_feature_debug_csv_path_.empty()) {
      return;
    }

    try {
      const std::filesystem::path csv_path(dtrq_runtime_feature_debug_csv_path_);
      if (csv_path.has_parent_path()) {
        std::filesystem::create_directories(csv_path.parent_path());
      }
      dtrq_runtime_feature_debug_csv_.open(csv_path, std::ios::out | std::ios::trunc);
    } catch (const std::exception & e) {
      RCLCPP_WARN(
        get_logger(),
        "Failed to open DTRQ runtime feature debug CSV %s: %s",
        dtrq_runtime_feature_debug_csv_path_.c_str(),
        e.what());
      return;
    }

    if (!dtrq_runtime_feature_debug_csv_.is_open()) {
      RCLCPP_WARN(
        get_logger(),
        "Failed to open DTRQ runtime feature debug CSV %s",
        dtrq_runtime_feature_debug_csv_path_.c_str());
      return;
    }

    dtrq_runtime_feature_debug_csv_ << std::setprecision(std::numeric_limits<double>::max_digits10);
    dtrq_runtime_feature_debug_csv_
      << "sequence,ros_time_sec,update_time_sec,armed_time_sec,mavros_armed,"
      << "dtrq_row_valid,dtrq_missing_required_fields,gnss_quality_source_kind,"
      << "last_gnss_source_time_sec,last_gnss_rx_ros_time_sec,"
      << "diag_gnss_source_age_at_update_sec,diag_gnss_rx_age_at_update_sec,"
      << "gnss_position_std_n_m,gnss_position_std_e_m,gnss_position_std_u_m,"
      << "gnss_fix_type,gnss_satellites_used,gnss_hdop,gnss_vdop,"
      << "gnss_eph_m,gnss_epv_m,gnss_hacc_m,gnss_vacc_m,"
      << "gnss_vel_ned_valid,gnss_native_vel_n_mps,gnss_native_vel_e_mps,gnss_native_vel_d_mps,"
      << "gnss_enu_e_m,gnss_enu_n_m,gnss_enu_u_m,"
      << "dtrq_gnss_dt_sec,dtrq_gnss_dp_h_m,dtrq_gnss_dp_u_m,"
      << "horizontal_speed_mps,vertical_speed_mps,gyro_deg_s,source_yaw_rate_deg_s,"
      << "turning_now,post_turn_context,dtrq_speed_change_abs_mps2\n";
    dtrq_runtime_feature_debug_csv_.flush();
    RCLCPP_INFO(
      get_logger(),
      "DTRQ runtime feature debug CSV enabled: %s (max_rate_hz=%.2f)",
      dtrq_runtime_feature_debug_csv_path_.c_str(),
      dtrq_runtime_feature_debug_max_rate_hz_);
  }

  void logStatePublishDebug_(
    const rclcpp::Time & stamp,
    bool odom_uses_gnss_pose,
    bool have_core_enu,
    const Eigen::Vector3d & enu_core,
    const Eigen::Vector3d & enu_raw_publish,
    const Eigen::Vector3d & enu_published,
    bool have_gnss_enu,
    const Eigen::Vector3d & enu_gnss,
    double core_vN_mps,
    double core_vE_mps,
    double core_vD_mps)
  {
    const double ros_time_sec = now().seconds();

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
    const Eigen::Vector3d enu_px4_sphere = applyPx4SphereProjection_(enu_raw_publish);
    const Eigen::Vector3d enu_alpha_projected =
      applyPx4SphereProjectionAlpha_(enu_raw_publish, accbias_z_history_projection_alpha_);
    const double accbias_history_frac = accbiasZHistoryProjectionDeepFrac_();
    auto value_if = [nan](bool ok, double value) {
      return ok ? value : nan;
    };

    state_publish_debug_csv_
      << (++state_publish_debug_sequence_) << ','
      << ros_time_sec << ','
      << stamp_sec << ','
	      << value_if(have_core_time, last_core_time_) << ','
	      << value_if(have_gnss_update_time, last_gnss_time_sec_) << ','
	      << value_if(have_gnss_source_time, last_gnss_source_time_sec_) << ','
	      << last_gnss_rx_ros_time_sec_ << ','
	      << last_publish_stamp_applied_mode_ << ','
	      << value_if(publish_core_stamp_offset_initialized_, publish_core_to_ros_offset_sec_) << ','
	      << last_publish_stamp_observed_offset_sec_ << ','
	      << last_publish_stamp_selected_minus_now_sec_ << ','
	      << publish_core_stamp_offset_bias_sec_ << ','
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
      << enu_raw_publish.x() << ','
      << enu_raw_publish.y() << ','
      << enu_raw_publish.z() << ','
      << enu_px4_sphere.x() << ','
      << enu_px4_sphere.y() << ','
      << enu_px4_sphere.z() << ','
      << enu_alpha_projected.x() << ','
      << enu_alpha_projected.y() << ','
      << enu_alpha_projected.z() << ','
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
      << last_mavros_vertical_speed_mps_ << ','
      << last_publish_projection_action_ << ','
      << (last_publish_projection_active_ ? 1 : 0) << ','
      << selectedPublishProjectionAlpha_() << ','
      << (segment_timing_gate_projection_enable_ ? 1 : 0) << ','
      << (last_segment_timing_gate_projection_active_ ? 1 : 0) << ','
      << (segmentTimingGateProjectionPhaseAllowed_() ? 1 : 0) << ','
      << (segment_timing_gate_current_active_ ? 1 : 0) << ','
      << segment_timing_gate_current_lag_mean_sec_ << ','
      << segment_timing_gate_current_lag_latest_sec_ << ','
      << segment_timing_gate_current_gnss_source_age_mean_sec_ << ','
      << (segment_timing_gate_core_gnss_along_min_enable_ ? 1 : 0) << ','
      << segment_timing_gate_current_core_gnss_along_min_m_ << ','
      << segment_timing_gate_core_gnss_along_min_threshold_m_ << ','
      << segment_timing_gate_gnss_source_age_max_sec_ << ','
      << segment_timing_gate_projection_alpha_ << ','
      << (accbias_z_history_projection_enable_ ? 1 : 0) << ','
      << (last_accbias_z_history_projection_active_ ? 1 : 0) << ','
      << (last_accbias_z_history_projection_phase_allowed_ ? 1 : 0) << ','
      << (last_accbias_z_history_projection_have_history_ ? 1 : 0) << ','
      << accbias_history_frac << ','
      << accbias_z_history_projection_total_count_ << ','
      << accbias_z_history_projection_deep_count_ << ','
      << accbias_z_history_projection_latest_before_mps2_ << ','
      << accbias_z_history_projection_latest_after_mps2_ << ','
      << accbias_z_history_projection_deep_threshold_mps2_ << ','
      << accbias_z_history_projection_frac_threshold_ << ','
      << accbias_z_history_projection_history_start_sec_ << ','
      << segment_timing_gate_lag_threshold_sec_ << ','
      << mavros_mode_ << '\n';
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
      << "gnss_velocity_std_n_mps,gnss_velocity_std_e_mps,gnss_velocity_std_d_mps,";
    appendBoundedAdaptiveRHeader_(gnss_update_debug_csv_, "gnss_position");
    appendBoundedAdaptiveRHeader_(gnss_update_debug_csv_, "gnss_velocity");
	    gnss_update_debug_csv_
	      << "pending_debug_matched,pending_native_velocity_valid,pending_native_velocity_used,"
	      << "native_velocity_override_active,"
      << "native_velocity_outlier_guard_enabled,native_velocity_outlier_guard_phase_allowed,"
      << "native_velocity_outlier_guard_motion_ok,native_velocity_outlier_guard_active,"
      << "native_velocity_outlier_guard_reason,"
      << "native_velocity_outlier_guard_action,"
      << "native_velocity_outlier_guard_reweight_applied,"
      << "native_velocity_outlier_guard_speed_mismatch_mps,"
      << "native_velocity_outlier_guard_core_residual_h_mps,"
      << "native_velocity_outlier_guard_reweight_std_h_mps,"
      << "native_velocity_outlier_guard_reweight_std_u_mps,"
      << "velocity_residual_boost_active,"
      << "gnss_velocity_outward_damping_enabled,"
      << "gnss_velocity_outward_damping_phase_allowed,"
      << "gnss_velocity_outward_damping_motion_ok,"
      << "gnss_velocity_outward_damping_triggered,"
      << "gnss_velocity_outward_damping_held,"
      << "gnss_velocity_outward_damping_active,"
      << "gnss_velocity_outward_damping_applied,"
      << "gnss_velocity_outward_damping_reason,"
      << "gnss_velocity_outward_damping_core_gnss_diff_n_m,"
      << "gnss_velocity_outward_damping_core_gnss_diff_e_m,"
      << "gnss_velocity_outward_damping_core_gnss_diff_h_m,"
      << "gnss_velocity_outward_damping_core_native_residual_n_mps,"
      << "gnss_velocity_outward_damping_core_native_residual_e_mps,"
      << "gnss_velocity_outward_damping_core_native_residual_h_mps,"
      << "gnss_velocity_outward_damping_radial_outward_mps,"
      << "gnss_velocity_outward_damping_radial_score,"
      << "gnss_velocity_outward_damping_hold_score,"
      << "gnss_velocity_outward_damping_persistent_count,"
      << "gnss_velocity_outward_damping_hold_remaining_updates,"
      << "gnss_velocity_outward_damping_selected_std_h_mps,"
      << "gnss_velocity_outward_damping_target_std_h_mps,"
      << "gnss_velocity_outward_damping_effective_std_h_mps,"
      << "gnss_velocity_outward_damping_multiplier_h,"
      << "turn_postturn_native_velocity_deweight_enabled,"
      << "turn_postturn_native_velocity_deweight_phase_allowed,"
      << "turn_postturn_native_velocity_deweight_context_ok,"
      << "turn_postturn_native_velocity_deweight_motion_ok,"
      << "turn_postturn_native_velocity_deweight_triggered,"
      << "turn_postturn_native_velocity_deweight_active,"
      << "turn_postturn_native_velocity_deweight_applied,"
      << "turn_postturn_native_velocity_deweight_reason,"
      << "turn_postturn_native_velocity_deweight_core_gnss_diff_n_m,"
      << "turn_postturn_native_velocity_deweight_core_gnss_diff_e_m,"
      << "turn_postturn_native_velocity_deweight_core_gnss_diff_h_m,"
      << "turn_postturn_native_velocity_deweight_core_native_residual_n_mps,"
      << "turn_postturn_native_velocity_deweight_core_native_residual_e_mps,"
      << "turn_postturn_native_velocity_deweight_core_native_residual_h_mps,"
      << "turn_postturn_native_velocity_deweight_radial_mps,"
      << "turn_postturn_native_velocity_deweight_radial_abs_score,"
      << "turn_postturn_native_velocity_deweight_residual_score,"
      << "turn_postturn_native_velocity_deweight_score,"
      << "turn_postturn_native_velocity_deweight_persistent_count,"
      << "turn_postturn_native_velocity_deweight_selected_std_h_mps,"
      << "turn_postturn_native_velocity_deweight_target_std_h_mps,"
      << "turn_postturn_native_velocity_deweight_effective_std_h_mps,"
      << "turn_postturn_native_velocity_deweight_multiplier_h,"
      << "position_override_active,position_residual_boost_active,"
      << "gnss_position_std_source,"
      << "gnss_position_response_boost_enabled,gnss_position_response_boost_phase_allowed,"
      << "gnss_position_response_boost_motion_ok,gnss_position_response_boost_triggered,"
      << "gnss_position_response_boost_active,gnss_position_response_boost_applied,"
      << "gnss_position_response_boost_reason,gnss_position_response_boost_residual_h_m,"
      << "gnss_position_response_boost_last_residual_h_m,"
      << "gnss_position_response_boost_core_gnss_diff_h_m,"
      << "gnss_position_response_boost_residual_score,"
      << "gnss_position_response_boost_persistent_count,"
      << "gnss_position_response_boost_selected_std_h_m,"
      << "gnss_position_response_boost_target_std_h_m,"
      << "gnss_position_response_boost_effective_std_h_m,"
      << "gnss_position_response_boost_multiplier_h,"
      << "gnss_position_gain_response_enabled,gnss_position_gain_response_phase_allowed,"
      << "gnss_position_gain_response_context_ok,gnss_position_gain_response_motion_ok,"
      << "gnss_position_gain_response_gain_ok,gnss_position_gain_response_triggered,"
      << "gnss_position_gain_response_active,gnss_position_gain_response_applied,"
      << "gnss_position_gain_response_reason,gnss_position_gain_response_residual_h_m,"
      << "gnss_position_gain_response_hnis_h,"
      << "gnss_position_gain_response_prev_dx_pos_h_m,"
      << "gnss_position_gain_response_prev_residual_h_m,"
      << "gnss_position_gain_response_prev_dx_over_residual_h,"
      << "gnss_position_gain_response_residual_score,"
      << "gnss_position_gain_response_hnis_score,"
      << "gnss_position_gain_response_gain_score,"
      << "gnss_position_gain_response_score,"
      << "gnss_position_gain_response_persistent_count,"
      << "gnss_position_gain_response_selected_std_h_m,"
      << "gnss_position_gain_response_target_std_h_m,"
      << "gnss_position_gain_response_effective_std_h_m,"
      << "gnss_position_gain_response_multiplier_h,"
      << "motion_gnss_pos_weight_enabled,motion_gnss_pos_weight_phase_allowed,"
      << "motion_gnss_pos_weight_motion_ok,motion_gnss_pos_weight_active,"
      << "motion_gnss_pos_weight_applied,motion_gnss_pos_weight_reason,"
      << "motion_gnss_pos_weight_speed_score,"
      << "motion_gnss_pos_weight_target_std_h_m,"
      << "motion_gnss_pos_weight_effective_std_h_m,"
      << "gnss_pos_recovery_weight_enabled,gnss_pos_recovery_weight_phase_allowed,"
      << "gnss_pos_recovery_weight_motion_ok,gnss_pos_recovery_weight_triggered,"
      << "gnss_pos_recovery_weight_held,gnss_pos_recovery_weight_active,"
      << "gnss_pos_recovery_weight_applied,gnss_pos_recovery_weight_reason,"
      << "gnss_pos_recovery_weight_residual_h_m,"
      << "gnss_pos_recovery_weight_core_gnss_diff_h_m,"
      << "gnss_pos_recovery_weight_residual_score,"
      << "gnss_pos_recovery_weight_core_score,"
      << "gnss_pos_recovery_weight_score,"
      << "gnss_pos_recovery_weight_persistent_count,"
      << "gnss_pos_recovery_weight_hold_remaining_sec,"
      << "gnss_pos_recovery_weight_selected_std_h_m,"
      << "gnss_pos_recovery_weight_target_std_h_m,"
      << "gnss_pos_recovery_weight_state_std_h_m,"
      << "gnss_pos_recovery_weight_effective_std_h_m,"
      << "gnss_pos_recovery_weight_multiplier_h,"
      << "context_gnss_pos_floor_enabled,context_gnss_pos_floor_phase_allowed,"
      << "context_gnss_pos_floor_active,context_gnss_pos_floor_applied,"
      << "context_gnss_pos_floor_reason,"
      << "context_gnss_pos_floor_selected_std_h_m,"
      << "context_gnss_pos_floor_target_floor_h_m,"
      << "context_gnss_pos_floor_state_floor_h_m,"
      << "context_gnss_pos_floor_effective_std_h_m,"
      << "context_gnss_pos_floor_multiplier_h,"
      << "adaptive_gnss_pos_weight_enabled,adaptive_gnss_pos_weight_active,"
      << "adaptive_gnss_pos_weight_applied,adaptive_gnss_pos_weight_phase,"
      << "adaptive_gnss_pos_weight_reason,adaptive_gnss_pos_weight_residual_h_m,"
      << "adaptive_gnss_pos_weight_persistent_count,"
      << "adaptive_gnss_pos_weight_selected_std_h_m,"
      << "adaptive_gnss_pos_weight_target_floor_h_m,"
      << "adaptive_gnss_pos_weight_floor_state_h_m,"
      << "adaptive_gnss_pos_weight_effective_std_h_m,"
      << "adaptive_gnss_pos_weight_multiplier_h,"
      << "mission_cov_hygiene_enabled,mission_cov_hygiene_phase_allowed,"
      << "mission_cov_hygiene_accepted_recent,mission_cov_hygiene_triggered,"
      << "mission_cov_hygiene_active,mission_cov_hygiene_applied,"
      << "mission_cov_hygiene_phase,mission_cov_hygiene_reason,"
      << "mission_cov_hygiene_residual_h_m,mission_cov_hygiene_hnis_h,"
      << "mission_cov_hygiene_score,mission_cov_hygiene_nis_score,"
      << "mission_cov_hygiene_residual_score,mission_cov_hygiene_cov_score,"
      << "mission_cov_hygiene_weak_score,mission_cov_hygiene_persistent_count,"
      << "mission_cov_hygiene_pos_std_h_before_m,mission_cov_hygiene_floor_h_m,"
      << "mission_cov_hygiene_pos_std_h_after_m,"
      << "mission_cov_hygiene_dx_pos_h_prev_m,"
      << "mission_cov_hygiene_dx_pos_over_resid_prev,"
      << "mission_cov_hygiene_p_nn_before,mission_cov_hygiene_p_ee_before,"
      << "mission_cov_hygiene_p_ne_before,mission_cov_hygiene_p_nn_after,"
      << "mission_cov_hygiene_p_ee_after,mission_cov_hygiene_p_ne_after,"
      << "position_lag_compensation_active,position_lag_compensation_sec,"
      << "position_lag_compensation_n_m,position_lag_compensation_e_m,position_lag_compensation_u_m,"
      << "vertical_cov_reopen_active,vertical_cov_reopen_applied,"
      << "post_flight_vertical_cov_reopen_active,post_flight_vertical_cov_reopen_applied,"
      << "terminal_descent_vertical_cov_reopen_active,terminal_descent_vertical_cov_reopen_applied,"
      << "last_position_residual_h_m,last_position_residual_u_m,last_velocity_residual_h_mps,last_velocity_residual_e_mps,last_velocity_residual_d_mps,"
      << "core_gnss_diff_h_m,core_gnss_diff_u_m,core_pos_std_d_m,core_vel_std_d_mps,core_accbias_std_z_mps2,"
      << "mavros_armed,have_fresh_speed,turning_now,"
      << "post_turn_context,armed_cruise_context,native_velocity_tightening_context,"
      << "terminal_descent_context,terminal_descent_native_velocity_override_active,"
      << "terminal_descent_horizontal_zero_velocity_active,"
      << "terminal_descent_horizontal_zero_velocity_applied,"
      << "medium_gap_active,medium_gap_segmented,medium_gap_conservative_single_step,"
      << "horizontal_speed_mps,vertical_speed_mps,"
      << "gyro_deg_s,source_yaw_rate_deg_s,"
      << "latest_imu_raw_dt_sec,latest_imu_effective_dt_sec,medium_gap_dropped_dt_sec,medium_gap_segmented_steps,"
      << "turn_rate_propagation_noise_probe_enabled,turn_rate_propagation_noise_probe_phase_allowed,"
      << "turn_rate_propagation_noise_probe_motion_ok,turn_rate_propagation_noise_probe_triggered,"
      << "turn_rate_propagation_noise_probe_active,turn_rate_propagation_noise_probe_applied,"
      << "turn_rate_propagation_noise_probe_reason,turn_rate_propagation_noise_probe_gyro_score,"
      << "turn_rate_propagation_noise_probe_arw_q_scale,turn_rate_propagation_noise_probe_vrw_q_scale,"
      << "turn_rate_propagation_noise_probe_gyrbias_q_scale,turn_rate_propagation_noise_probe_accbias_q_scale,"
      << "accbias_z_propagation_probe_enabled,accbias_z_propagation_probe_apply_noise_scale,"
      << "accbias_z_propagation_probe_have_bias,accbias_z_propagation_probe_phase_allowed,"
      << "accbias_z_propagation_probe_motion_ok,accbias_z_propagation_probe_triggered,"
      << "accbias_z_propagation_probe_active,accbias_z_propagation_probe_applied,"
      << "accbias_z_propagation_probe_reason,accbias_z_propagation_probe_accbias_z_mps2,"
      << "accbias_z_propagation_probe_bias_score,"
      << "accbias_z_propagation_probe_arw_q_scale,accbias_z_propagation_probe_vrw_q_scale,"
      << "accbias_z_propagation_probe_gyrbias_q_scale,accbias_z_propagation_probe_accbias_q_scale,"
      << "native_velocity_std_h_mps,native_velocity_std_u_mps,"
      << "native_velocity_vN_mps,native_velocity_vE_mps,native_velocity_vD_mps,"
      << "core_velocity_vN_mps,core_velocity_vE_mps,core_velocity_vD_mps,"
      << "phase_error_memory_debug_enabled,phase_error_memory_state_update_matched,"
      << "phase_error_memory_recent_turnpost_age_sec,"
      << "phase_error_memory_recent_turnpost_active,"
      << "phase_error_memory_residual_h_m,phase_error_memory_dx_pos_h_m,"
      << "phase_error_memory_dx_over_residual_h,"
      << "phase_error_memory_pos_std_h_before_m,"
      << "phase_error_memory_pressure_active,"
      << "phase_error_memory_candidate_active,"
      << "phase_error_memory_reason,"
      << "phase_error_memory_residual_threshold_h_m,"
      << "phase_error_memory_dx_over_residual_threshold,"
      << "phase_error_memory_recent_turnpost_hold_sec\n";
    gnss_update_debug_csv_.flush();
    gnss_update_debug_rows_since_flush_ = 0;

    RCLCPP_INFO(
      get_logger(),
      "GNSS update debug CSV enabled: %s",
      gnss_update_debug_csv_path_.c_str());
  }

  void openGnssNisDebugCsv_()
  {
    if (gnss_nis_debug_csv_path_.empty()) {
      return;
    }

    try {
      const std::filesystem::path csv_path(gnss_nis_debug_csv_path_);
      if (csv_path.has_parent_path()) {
        std::filesystem::create_directories(csv_path.parent_path());
      }
      gnss_nis_debug_csv_.open(csv_path, std::ios::out | std::ios::trunc);
    } catch (const std::exception & e) {
      RCLCPP_WARN(
        get_logger(),
        "Failed to open GNSS NIS debug CSV %s: %s",
        gnss_nis_debug_csv_path_.c_str(),
        e.what());
      return;
    }

    if (!gnss_nis_debug_csv_.is_open()) {
      RCLCPP_WARN(
        get_logger(),
        "Failed to open GNSS NIS debug CSV %s",
        gnss_nis_debug_csv_path_.c_str());
      return;
    }

    gnss_nis_debug_csv_
      << "sequence,ros_time_sec,update_time_sec,armed_time_sec,mavros_mode,mavros_armed,"
      << "update_mode,update_mode_label,pending_debug_matched,"
      << "turning_now,post_turn_context,armed_cruise_context,terminal_descent_context,"
      << "gnss_position_residual_n_m,gnss_position_residual_e_m,gnss_position_residual_u_m,"
      << "gnss_position_residual_h_m,"
      << "gnss_position_std_n_m,gnss_position_std_e_m,gnss_position_std_u_m,"
      << "gnss_position_std_source_label,";
    appendBoundedAdaptiveRHeader_(gnss_nis_debug_csv_, "gnss_position");
    appendBoundedAdaptiveRHeader_(gnss_nis_debug_csv_, "gnss_velocity");
	    gnss_nis_debug_csv_
	      << "adaptive_gnss_pos_weight_enabled,adaptive_gnss_pos_weight_active,"
	      << "adaptive_gnss_pos_weight_applied,adaptive_gnss_pos_weight_phase,"
      << "adaptive_gnss_pos_weight_reason,adaptive_gnss_pos_weight_residual_h_m,"
      << "adaptive_gnss_pos_weight_persistent_count,"
      << "adaptive_gnss_pos_weight_selected_std_h_m,"
      << "adaptive_gnss_pos_weight_target_floor_h_m,"
      << "adaptive_gnss_pos_weight_floor_state_h_m,"
      << "adaptive_gnss_pos_weight_effective_std_h_m,"
      << "adaptive_gnss_pos_weight_multiplier_h,"
      << "mission_cov_hygiene_enabled,mission_cov_hygiene_phase_allowed,"
      << "mission_cov_hygiene_accepted_recent,mission_cov_hygiene_triggered,"
      << "mission_cov_hygiene_active,mission_cov_hygiene_applied,"
      << "mission_cov_hygiene_phase,mission_cov_hygiene_reason,"
      << "mission_cov_hygiene_residual_h_m,mission_cov_hygiene_hnis_h,"
      << "mission_cov_hygiene_score,mission_cov_hygiene_nis_score,"
      << "mission_cov_hygiene_residual_score,mission_cov_hygiene_cov_score,"
      << "mission_cov_hygiene_weak_score,mission_cov_hygiene_persistent_count,"
      << "mission_cov_hygiene_pos_std_h_before_m,mission_cov_hygiene_floor_h_m,"
      << "mission_cov_hygiene_pos_std_h_after_m,"
      << "mission_cov_hygiene_dx_pos_h_prev_m,"
      << "mission_cov_hygiene_dx_pos_over_resid_prev,"
      << "mission_cov_hygiene_p_nn_before,mission_cov_hygiene_p_ee_before,"
      << "mission_cov_hygiene_p_ne_before,mission_cov_hygiene_p_nn_after,"
      << "mission_cov_hygiene_p_ee_after,mission_cov_hygiene_p_ne_after,"
      << "turn_rate_propagation_noise_probe_enabled,turn_rate_propagation_noise_probe_phase_allowed,"
      << "turn_rate_propagation_noise_probe_motion_ok,turn_rate_propagation_noise_probe_triggered,"
      << "turn_rate_propagation_noise_probe_active,turn_rate_propagation_noise_probe_applied,"
      << "turn_rate_propagation_noise_probe_reason,turn_rate_propagation_noise_probe_gyro_score,"
      << "turn_rate_propagation_noise_probe_arw_q_scale,turn_rate_propagation_noise_probe_vrw_q_scale,"
      << "turn_rate_propagation_noise_probe_gyrbias_q_scale,turn_rate_propagation_noise_probe_accbias_q_scale,"
      << "gnss_position_s_nn_m2,gnss_position_s_ne_m2,gnss_position_s_nu_m2,"
      << "gnss_position_s_ee_m2,gnss_position_s_eu_m2,gnss_position_s_uu_m2,"
      << "gnss_position_nis_h_2d,gnss_position_nis_u_1d,gnss_position_nis_3d,"
      << "gnss_position_gate_threshold_nis,"
      << "gnss_position_update_accepted,gnss_position_update_rejected,"
      << "gnss_position_update_reason,position_override_active,position_residual_boost_active,"
      << "gnss_position_response_boost_enabled,gnss_position_response_boost_phase_allowed,"
      << "gnss_position_response_boost_motion_ok,gnss_position_response_boost_triggered,"
      << "gnss_position_response_boost_active,gnss_position_response_boost_applied,"
      << "gnss_position_response_boost_reason,gnss_position_response_boost_residual_h_m,"
      << "gnss_position_response_boost_last_residual_h_m,"
      << "gnss_position_response_boost_core_gnss_diff_h_m,"
      << "gnss_position_response_boost_residual_score,"
      << "gnss_position_response_boost_persistent_count,"
      << "gnss_position_response_boost_selected_std_h_m,"
      << "gnss_position_response_boost_target_std_h_m,"
      << "gnss_position_response_boost_effective_std_h_m,"
      << "gnss_position_response_boost_multiplier_h,"
      << "gnss_position_gain_response_enabled,gnss_position_gain_response_phase_allowed,"
      << "gnss_position_gain_response_context_ok,gnss_position_gain_response_motion_ok,"
      << "gnss_position_gain_response_gain_ok,gnss_position_gain_response_triggered,"
      << "gnss_position_gain_response_active,gnss_position_gain_response_applied,"
      << "gnss_position_gain_response_reason,gnss_position_gain_response_residual_h_m,"
      << "gnss_position_gain_response_hnis_h,"
      << "gnss_position_gain_response_prev_dx_pos_h_m,"
      << "gnss_position_gain_response_prev_residual_h_m,"
      << "gnss_position_gain_response_prev_dx_over_residual_h,"
      << "gnss_position_gain_response_residual_score,"
      << "gnss_position_gain_response_hnis_score,"
      << "gnss_position_gain_response_gain_score,"
      << "gnss_position_gain_response_score,"
      << "gnss_position_gain_response_persistent_count,"
      << "gnss_position_gain_response_selected_std_h_m,"
      << "gnss_position_gain_response_target_std_h_m,"
      << "gnss_position_gain_response_effective_std_h_m,"
      << "gnss_position_gain_response_multiplier_h,"
      << "pos_cov_n_m2,pos_cov_e_m2,pos_cov_u_m2,"
      << "vel_cov_n_m2ps2,vel_cov_e_m2ps2,vel_cov_d_m2ps2,"
      << "core_gnss_diff_h_m,core_gnss_diff_u_m,"
      << "horizontal_speed_mps,vertical_speed_mps,gyro_deg_s,source_yaw_rate_deg_s,"
      << "latest_heading_update_age_sec,"
      << "phase_error_memory_debug_enabled,phase_error_memory_state_update_matched,"
      << "phase_error_memory_recent_turnpost_age_sec,"
      << "phase_error_memory_recent_turnpost_active,"
      << "phase_error_memory_residual_h_m,phase_error_memory_dx_pos_h_m,"
      << "phase_error_memory_dx_over_residual_h,"
      << "phase_error_memory_pos_std_h_before_m,"
      << "phase_error_memory_pressure_active,"
      << "phase_error_memory_candidate_active,"
      << "phase_error_memory_reason,"
      << "phase_error_memory_residual_threshold_h_m,"
      << "phase_error_memory_dx_over_residual_threshold,"
      << "phase_error_memory_recent_turnpost_hold_sec\n";
    gnss_nis_debug_csv_.flush();

    RCLCPP_INFO(
      get_logger(),
      "GNSS NIS debug CSV enabled: %s (max_rate=%.3f Hz)",
      gnss_nis_debug_csv_path_.c_str(),
      gnss_nis_debug_max_rate_hz_);
  }

  void openHorizontalConsistencyDebugCsv_()
  {
    if (horizontal_consistency_debug_csv_path_.empty()) {
      return;
    }

    try {
      const std::filesystem::path csv_path(horizontal_consistency_debug_csv_path_);
      if (csv_path.has_parent_path()) {
        std::filesystem::create_directories(csv_path.parent_path());
      }
      horizontal_consistency_debug_csv_.open(csv_path, std::ios::out | std::ios::trunc);
    } catch (const std::exception & e) {
      RCLCPP_WARN(
        get_logger(),
        "Failed to open horizontal consistency debug CSV %s: %s",
        horizontal_consistency_debug_csv_path_.c_str(),
        e.what());
      return;
    }

    if (!horizontal_consistency_debug_csv_.is_open()) {
      RCLCPP_WARN(
        get_logger(),
        "Failed to open horizontal consistency debug CSV %s",
        horizontal_consistency_debug_csv_path_.c_str());
      return;
    }

    horizontal_consistency_debug_csv_
      << "sequence,ros_time_sec,update_time_sec,armed_time_sec,mavros_mode,mavros_armed,"
      << "update_mode,update_mode_label,pending_debug_matched,"
      << "gnss_position_update_accepted,gnss_position_update_rejected,gnss_position_update_reason,"
      << "context_enabled,phase_allowed,motion_ok,triggered,candidate_active,"
      << "reason,proposed_action,persistent_count,"
      << "score,nis_score,residual_score,core_gnss_score,heading_score,"
      << "gnss_position_nis_h_2d,gnss_position_residual_h_m,core_gnss_diff_h_m,"
      << "latest_heading_update_age_sec,latest_heading_residual_abs_deg,"
      << "latest_heading_yaw_correction_abs_deg,latest_heading_mode,"
      << "gnss_position_std_n_m,gnss_position_std_e_m,gnss_position_std_u_m,"
      << "gnss_position_std_source_label,"
      << "position_override_active,position_residual_boost_active,"
      << "adaptive_gnss_pos_weight_enabled,adaptive_gnss_pos_weight_active,"
      << "adaptive_gnss_pos_weight_applied,adaptive_gnss_pos_weight_phase,"
      << "adaptive_gnss_pos_weight_reason,adaptive_gnss_pos_weight_effective_std_h_m,"
      << "gnss_position_s_nn_m2,gnss_position_s_ne_m2,gnss_position_s_ee_m2,"
      << "gnss_position_gate_threshold_nis,"
      << "pos_cov_n_m2,pos_cov_e_m2,pos_cov_u_m2,"
      << "vel_cov_n_m2ps2,vel_cov_e_m2ps2,vel_cov_d_m2ps2,"
      << "turning_now,post_turn_context,armed_cruise_context,terminal_descent_context,"
      << "horizontal_speed_mps,vertical_speed_mps,gyro_deg_s,source_yaw_rate_deg_s,"
      << "threshold_score_trigger,threshold_persistence_updates,"
      << "threshold_nis_start_h_2d,threshold_nis_full_h_2d,"
      << "threshold_residual_start_h_m,threshold_residual_full_h_m,"
      << "threshold_core_gnss_start_h_m,threshold_core_gnss_full_h_m,"
      << "threshold_heading_residual_start_deg,threshold_heading_residual_full_deg,"
      << "threshold_min_horizontal_speed_mps,threshold_max_vertical_speed_mps\n";
    horizontal_consistency_debug_csv_.flush();

    RCLCPP_INFO(
      get_logger(),
      "Horizontal consistency debug CSV enabled: %s (supervisor_enable=%s)",
      horizontal_consistency_debug_csv_path_.c_str(),
      horizontal_consistency_supervisor_enable_ ? "true" : "false");
  }

  void logGnssNisDebugIfNeeded_()
  {
    if ((!gnss_nis_debug_csv_.is_open() &&
         !horizontal_consistency_debug_csv_.is_open()) ||
        !core_) {
      return;
    }

    const kfcore::ObservationDebug debug = core_->lastObservationDebug();
    if (!debug.valid || debug.sequence == 0 ||
        debug.sequence == last_logged_gnss_nis_debug_sequence_) {
      return;
    }
    last_logged_gnss_nis_debug_sequence_ = debug.sequence;

    if (!debug.gnss_position_applied) {
      return;
    }

    const double log_time_sec =
      std::isfinite(debug.update_time_sec) ? debug.update_time_sec : now().seconds();
    if (gnss_nis_debug_max_rate_hz_ > 0.0 &&
        std::isfinite(last_gnss_nis_debug_log_update_time_sec_) &&
        std::isfinite(log_time_sec)) {
      const double min_period_sec = 1.0 / gnss_nis_debug_max_rate_hz_;
      if ((log_time_sec - last_gnss_nis_debug_log_update_time_sec_) < min_period_sec) {
        return;
      }
    }
    last_gnss_nis_debug_log_update_time_sec_ = log_time_sec;

    const bool pending_debug_matched =
      pending_gnss_debug_context_.valid &&
      std::isfinite(pending_gnss_debug_context_.update_time_sec) &&
      std::isfinite(debug.update_time_sec) &&
      std::abs(pending_gnss_debug_context_.update_time_sec - debug.update_time_sec) <= 1e-6;

    const double nan = std::numeric_limits<double>::quiet_NaN();
    auto pending_value = [&](double value) {
      return pending_debug_matched ? value : nan;
    };
    auto pending_flag = [&](bool value) {
      return (pending_debug_matched && value) ? 1 : 0;
    };

    double pos_cov_n_m2 = nan;
    double pos_cov_e_m2 = nan;
    double pos_cov_u_m2 = nan;
    double vel_cov_n_m2ps2 = nan;
    double vel_cov_e_m2ps2 = nan;
    double vel_cov_d_m2ps2 = nan;
    const Eigen::MatrixXd core_covariance = core_->covariance();
    if (core_covariance.rows() >= 6 && core_covariance.cols() >= 6) {
      auto cov_diag = [&](int idx) {
        const double value = core_covariance(idx, idx);
        return std::isfinite(value) ? value : nan;
      };
      pos_cov_n_m2 = cov_diag(0);
      pos_cov_e_m2 = cov_diag(1);
      pos_cov_u_m2 = cov_diag(2);
      vel_cov_n_m2ps2 = cov_diag(3);
      vel_cov_e_m2ps2 = cov_diag(4);
      vel_cov_d_m2ps2 = cov_diag(5);
    }

    const double residual_h_m = std::hypot(
      debug.gnss_position_residual_neu_m.x(),
      debug.gnss_position_residual_neu_m.y());
    const Eigen::Matrix3d & S = debug.gnss_position_innovation_cov_neu_m2;
    static const std::string empty_csv_label{"unknown"};
    const std::string & mode =
      pending_debug_matched ? pending_gnss_debug_context_.mavros_mode : empty_csv_label;
    const std::string & std_source =
      pending_debug_matched ? pending_gnss_debug_context_.gnss_position_std_source_label
                            : empty_csv_label;

    const HorizontalConsistencyDebug horizontal_consistency_debug =
      evaluateHorizontalConsistencySupervisor_(debug, pending_debug_matched, residual_h_m);
    const PhaseErrorMemoryDebug phase_error_memory_debug =
      evaluatePhaseErrorMemoryDebug_(debug, pending_debug_matched);

    if (gnss_nis_debug_csv_.is_open()) {
      gnss_nis_debug_csv_
        << debug.sequence << ','
        << pending_value(pending_gnss_debug_context_.ros_time_sec) << ','
        << debug.update_time_sec << ','
        << pending_value(pending_gnss_debug_context_.armed_time_sec) << ','
        << (mode.empty() ? empty_csv_label : mode) << ','
        << pending_flag(pending_gnss_debug_context_.armed) << ','
        << debug.update_mode << ','
        << observationUpdateModeLabel_(debug.update_mode) << ','
        << (pending_debug_matched ? 1 : 0) << ','
        << pending_flag(pending_gnss_debug_context_.turning_now) << ','
        << pending_flag(pending_gnss_debug_context_.post_turn_context) << ','
        << pending_flag(pending_gnss_debug_context_.armed_cruise_context) << ','
        << pending_flag(pending_gnss_debug_context_.terminal_descent_context) << ','
        << debug.gnss_position_residual_neu_m.x() << ','
        << debug.gnss_position_residual_neu_m.y() << ','
        << debug.gnss_position_residual_neu_m.z() << ','
        << residual_h_m << ','
        << debug.gnss_position_std_neu_m.x() << ','
        << debug.gnss_position_std_neu_m.y() << ','
        << debug.gnss_position_std_neu_m.z() << ','
        << (std_source.empty() ? empty_csv_label : std_source) << ',';
      appendBoundedAdaptiveRValues_(gnss_nis_debug_csv_, debug.gnss_position_adaptive_r);
      appendBoundedAdaptiveRValues_(gnss_nis_debug_csv_, debug.gnss_velocity_adaptive_r);
      gnss_nis_debug_csv_
        << pending_flag(pending_gnss_debug_context_.adaptive_gnss_pos_weight_enabled) << ','
        << pending_flag(pending_gnss_debug_context_.adaptive_gnss_pos_weight_active) << ','
        << pending_flag(pending_gnss_debug_context_.adaptive_gnss_pos_weight_applied) << ','
        << (pending_debug_matched ? pending_gnss_debug_context_.adaptive_gnss_pos_weight_phase
                                  : empty_csv_label) << ','
        << (pending_debug_matched ? pending_gnss_debug_context_.adaptive_gnss_pos_weight_reason
                                  : empty_csv_label) << ','
        << pending_value(pending_gnss_debug_context_.adaptive_gnss_pos_weight_residual_h_m) << ','
        << (pending_debug_matched
              ? pending_gnss_debug_context_.adaptive_gnss_pos_weight_persistent_count
              : 0) << ','
        << pending_value(pending_gnss_debug_context_.adaptive_gnss_pos_weight_selected_std_h_m) << ','
        << pending_value(pending_gnss_debug_context_.adaptive_gnss_pos_weight_target_floor_h_m) << ','
        << pending_value(pending_gnss_debug_context_.adaptive_gnss_pos_weight_floor_state_h_m) << ','
        << pending_value(pending_gnss_debug_context_.adaptive_gnss_pos_weight_effective_std_h_m) << ','
        << pending_value(pending_gnss_debug_context_.adaptive_gnss_pos_weight_multiplier_h) << ','
        << pending_flag(pending_gnss_debug_context_.mission_cov_hygiene_enabled) << ','
        << pending_flag(pending_gnss_debug_context_.mission_cov_hygiene_phase_allowed) << ','
        << pending_flag(pending_gnss_debug_context_.mission_cov_hygiene_accepted_recent) << ','
        << pending_flag(pending_gnss_debug_context_.mission_cov_hygiene_triggered) << ','
        << pending_flag(pending_gnss_debug_context_.mission_cov_hygiene_active) << ','
        << pending_flag(pending_gnss_debug_context_.mission_cov_hygiene_applied) << ','
        << (pending_debug_matched ? pending_gnss_debug_context_.mission_cov_hygiene_phase
                                  : empty_csv_label) << ','
        << (pending_debug_matched ? pending_gnss_debug_context_.mission_cov_hygiene_reason
                                  : empty_csv_label) << ','
        << pending_value(pending_gnss_debug_context_.mission_cov_hygiene_residual_h_m) << ','
        << pending_value(pending_gnss_debug_context_.mission_cov_hygiene_hnis_h) << ','
        << pending_value(pending_gnss_debug_context_.mission_cov_hygiene_score) << ','
        << pending_value(pending_gnss_debug_context_.mission_cov_hygiene_nis_score) << ','
        << pending_value(pending_gnss_debug_context_.mission_cov_hygiene_residual_score) << ','
        << pending_value(pending_gnss_debug_context_.mission_cov_hygiene_cov_score) << ','
        << pending_value(pending_gnss_debug_context_.mission_cov_hygiene_weak_score) << ','
        << (pending_debug_matched
              ? pending_gnss_debug_context_.mission_cov_hygiene_persistent_count
              : 0) << ','
        << pending_value(pending_gnss_debug_context_.mission_cov_hygiene_pos_std_h_before_m) << ','
        << pending_value(pending_gnss_debug_context_.mission_cov_hygiene_floor_h_m) << ','
        << pending_value(pending_gnss_debug_context_.mission_cov_hygiene_pos_std_h_after_m) << ','
        << pending_value(pending_gnss_debug_context_.mission_cov_hygiene_dx_pos_h_prev_m) << ','
        << pending_value(pending_gnss_debug_context_.mission_cov_hygiene_dx_pos_over_resid_prev) << ','
        << pending_value(pending_gnss_debug_context_.mission_cov_hygiene_p_nn_before) << ','
        << pending_value(pending_gnss_debug_context_.mission_cov_hygiene_p_ee_before) << ','
        << pending_value(pending_gnss_debug_context_.mission_cov_hygiene_p_ne_before) << ','
        << pending_value(pending_gnss_debug_context_.mission_cov_hygiene_p_nn_after) << ','
        << pending_value(pending_gnss_debug_context_.mission_cov_hygiene_p_ee_after) << ','
        << pending_value(pending_gnss_debug_context_.mission_cov_hygiene_p_ne_after) << ','
        << pending_flag(pending_gnss_debug_context_.turn_rate_propagation_noise_probe_enabled) << ','
        << pending_flag(pending_gnss_debug_context_.turn_rate_propagation_noise_probe_phase_allowed) << ','
        << pending_flag(pending_gnss_debug_context_.turn_rate_propagation_noise_probe_motion_ok) << ','
        << pending_flag(pending_gnss_debug_context_.turn_rate_propagation_noise_probe_triggered) << ','
        << pending_flag(pending_gnss_debug_context_.turn_rate_propagation_noise_probe_active) << ','
        << pending_flag(pending_gnss_debug_context_.turn_rate_propagation_noise_probe_applied) << ','
        << (pending_debug_matched ? pending_gnss_debug_context_.turn_rate_propagation_noise_probe_reason
                                  : empty_csv_label) << ','
        << pending_value(pending_gnss_debug_context_.turn_rate_propagation_noise_probe_gyro_score) << ','
        << pending_value(pending_gnss_debug_context_.turn_rate_propagation_noise_probe_arw_q_scale) << ','
        << pending_value(pending_gnss_debug_context_.turn_rate_propagation_noise_probe_vrw_q_scale) << ','
        << pending_value(pending_gnss_debug_context_.turn_rate_propagation_noise_probe_gyrbias_q_scale) << ','
        << pending_value(pending_gnss_debug_context_.turn_rate_propagation_noise_probe_accbias_q_scale) << ','
        << S(0, 0) << ',' << S(0, 1) << ',' << S(0, 2) << ','
        << S(1, 1) << ',' << S(1, 2) << ',' << S(2, 2) << ','
        << debug.gnss_position_nis_h_2d << ','
        << debug.gnss_position_nis_u_1d << ','
        << debug.gnss_position_nis_3d << ','
        << debug.gnss_position_gate_threshold_nis << ','
        << (debug.gnss_position_update_accepted ? 1 : 0) << ','
        << (debug.gnss_position_update_rejected ? 1 : 0) << ','
        << debug.gnss_position_update_reason << ','
        << pending_flag(pending_gnss_debug_context_.position_override_active) << ','
        << pending_flag(pending_gnss_debug_context_.position_residual_boost_active) << ','
        << pending_flag(pending_gnss_debug_context_.gnss_position_response_boost_enabled) << ','
        << pending_flag(pending_gnss_debug_context_.gnss_position_response_boost_phase_allowed) << ','
        << pending_flag(pending_gnss_debug_context_.gnss_position_response_boost_motion_ok) << ','
        << pending_flag(pending_gnss_debug_context_.gnss_position_response_boost_triggered) << ','
        << pending_flag(pending_gnss_debug_context_.gnss_position_response_boost_active) << ','
        << pending_flag(pending_gnss_debug_context_.gnss_position_response_boost_applied) << ','
        << (pending_debug_matched ? pending_gnss_debug_context_.gnss_position_response_boost_reason
                                  : empty_csv_label) << ','
        << pending_value(pending_gnss_debug_context_.gnss_position_response_boost_residual_h_m) << ','
        << pending_value(pending_gnss_debug_context_.gnss_position_response_boost_last_residual_h_m) << ','
        << pending_value(pending_gnss_debug_context_.gnss_position_response_boost_core_gnss_diff_h_m) << ','
        << pending_value(pending_gnss_debug_context_.gnss_position_response_boost_residual_score) << ','
        << (pending_debug_matched
              ? pending_gnss_debug_context_.gnss_position_response_boost_persistent_count
              : 0) << ','
        << pending_value(pending_gnss_debug_context_.gnss_position_response_boost_selected_std_h_m) << ','
        << pending_value(pending_gnss_debug_context_.gnss_position_response_boost_target_std_h_m) << ','
        << pending_value(pending_gnss_debug_context_.gnss_position_response_boost_effective_std_h_m) << ','
        << pending_value(pending_gnss_debug_context_.gnss_position_response_boost_multiplier_h) << ','
        << pending_flag(pending_gnss_debug_context_.gnss_position_gain_response_enabled) << ','
        << pending_flag(pending_gnss_debug_context_.gnss_position_gain_response_phase_allowed) << ','
        << pending_flag(pending_gnss_debug_context_.gnss_position_gain_response_context_ok) << ','
        << pending_flag(pending_gnss_debug_context_.gnss_position_gain_response_motion_ok) << ','
        << pending_flag(pending_gnss_debug_context_.gnss_position_gain_response_gain_ok) << ','
        << pending_flag(pending_gnss_debug_context_.gnss_position_gain_response_triggered) << ','
        << pending_flag(pending_gnss_debug_context_.gnss_position_gain_response_active) << ','
        << pending_flag(pending_gnss_debug_context_.gnss_position_gain_response_applied) << ','
        << (pending_debug_matched ? pending_gnss_debug_context_.gnss_position_gain_response_reason
                                  : empty_csv_label) << ','
        << pending_value(pending_gnss_debug_context_.gnss_position_gain_response_residual_h_m) << ','
        << pending_value(pending_gnss_debug_context_.gnss_position_gain_response_hnis_h) << ','
        << pending_value(pending_gnss_debug_context_.gnss_position_gain_response_prev_dx_pos_h_m) << ','
        << pending_value(pending_gnss_debug_context_.gnss_position_gain_response_prev_residual_h_m) << ','
        << pending_value(pending_gnss_debug_context_.gnss_position_gain_response_prev_dx_over_residual_h) << ','
        << pending_value(pending_gnss_debug_context_.gnss_position_gain_response_residual_score) << ','
        << pending_value(pending_gnss_debug_context_.gnss_position_gain_response_hnis_score) << ','
        << pending_value(pending_gnss_debug_context_.gnss_position_gain_response_gain_score) << ','
        << pending_value(pending_gnss_debug_context_.gnss_position_gain_response_score) << ','
        << (pending_debug_matched
              ? pending_gnss_debug_context_.gnss_position_gain_response_persistent_count
              : 0) << ','
        << pending_value(pending_gnss_debug_context_.gnss_position_gain_response_selected_std_h_m) << ','
        << pending_value(pending_gnss_debug_context_.gnss_position_gain_response_target_std_h_m) << ','
        << pending_value(pending_gnss_debug_context_.gnss_position_gain_response_effective_std_h_m) << ','
        << pending_value(pending_gnss_debug_context_.gnss_position_gain_response_multiplier_h) << ','
        << pos_cov_n_m2 << ',' << pos_cov_e_m2 << ',' << pos_cov_u_m2 << ','
        << vel_cov_n_m2ps2 << ',' << vel_cov_e_m2ps2 << ',' << vel_cov_d_m2ps2 << ','
        << pending_value(pending_gnss_debug_context_.core_gnss_diff_h_m) << ','
        << pending_value(pending_gnss_debug_context_.core_gnss_diff_u_m) << ','
        << pending_value(pending_gnss_debug_context_.horizontal_speed_mps) << ','
        << pending_value(pending_gnss_debug_context_.vertical_speed_mps) << ','
        << pending_value(pending_gnss_debug_context_.gyro_deg_s) << ','
        << pending_value(pending_gnss_debug_context_.source_yaw_rate_deg_s) << ','
        << pending_value(pending_gnss_debug_context_.latest_heading_update_age_sec) << ','
        << (phase_error_memory_debug.enabled ? 1 : 0) << ','
        << (phase_error_memory_debug.state_update_matched ? 1 : 0) << ','
        << phase_error_memory_debug.recent_turnpost_age_sec << ','
        << (phase_error_memory_debug.recent_turnpost_active ? 1 : 0) << ','
        << phase_error_memory_debug.residual_h_m << ','
        << phase_error_memory_debug.dx_pos_h_m << ','
        << phase_error_memory_debug.dx_over_residual_h << ','
        << phase_error_memory_debug.pos_std_h_before_m << ','
        << (phase_error_memory_debug.pressure_active ? 1 : 0) << ','
        << (phase_error_memory_debug.candidate_active ? 1 : 0) << ','
        << phase_error_memory_debug.reason << ','
        << phase_error_memory_debug.residual_threshold_h_m << ','
        << phase_error_memory_debug.dx_over_residual_threshold << ','
        << phase_error_memory_debug.recent_turnpost_hold_sec << '\n';
      gnss_nis_debug_csv_.flush();
    }

    if (horizontal_consistency_debug_csv_.is_open()) {
      const std::string & heading_mode =
        pending_debug_matched ? pending_gnss_debug_context_.latest_heading_mode
                              : empty_csv_label;
      horizontal_consistency_debug_csv_
        << debug.sequence << ','
        << pending_value(pending_gnss_debug_context_.ros_time_sec) << ','
        << debug.update_time_sec << ','
        << pending_value(pending_gnss_debug_context_.armed_time_sec) << ','
        << (mode.empty() ? empty_csv_label : mode) << ','
        << pending_flag(pending_gnss_debug_context_.armed) << ','
        << debug.update_mode << ','
        << observationUpdateModeLabel_(debug.update_mode) << ','
        << (pending_debug_matched ? 1 : 0) << ','
        << (debug.gnss_position_update_accepted ? 1 : 0) << ','
        << (debug.gnss_position_update_rejected ? 1 : 0) << ','
        << debug.gnss_position_update_reason << ','
        << (horizontal_consistency_debug.context_enabled ? 1 : 0) << ','
        << (horizontal_consistency_debug.phase_allowed ? 1 : 0) << ','
        << (horizontal_consistency_debug.motion_ok ? 1 : 0) << ','
        << (horizontal_consistency_debug.triggered ? 1 : 0) << ','
        << (horizontal_consistency_debug.candidate_active ? 1 : 0) << ','
        << horizontal_consistency_debug.reason << ','
        << horizontal_consistency_debug.proposed_action << ','
        << horizontal_consistency_debug.persistent_count << ','
        << horizontal_consistency_debug.score << ','
        << horizontal_consistency_debug.nis_score << ','
        << horizontal_consistency_debug.residual_score << ','
        << horizontal_consistency_debug.core_gnss_score << ','
        << horizontal_consistency_debug.heading_score << ','
        << debug.gnss_position_nis_h_2d << ','
        << residual_h_m << ','
        << pending_value(pending_gnss_debug_context_.core_gnss_diff_h_m) << ','
        << pending_value(pending_gnss_debug_context_.latest_heading_update_age_sec) << ','
        << pending_value(pending_gnss_debug_context_.latest_heading_residual_abs_deg) << ','
        << pending_value(pending_gnss_debug_context_.latest_heading_yaw_correction_abs_deg) << ','
        << (heading_mode.empty() ? empty_csv_label : heading_mode) << ','
        << debug.gnss_position_std_neu_m.x() << ','
        << debug.gnss_position_std_neu_m.y() << ','
        << debug.gnss_position_std_neu_m.z() << ','
        << (std_source.empty() ? empty_csv_label : std_source) << ','
        << pending_flag(pending_gnss_debug_context_.position_override_active) << ','
        << pending_flag(pending_gnss_debug_context_.position_residual_boost_active) << ','
        << pending_flag(pending_gnss_debug_context_.adaptive_gnss_pos_weight_enabled) << ','
        << pending_flag(pending_gnss_debug_context_.adaptive_gnss_pos_weight_active) << ','
        << pending_flag(pending_gnss_debug_context_.adaptive_gnss_pos_weight_applied) << ','
        << (pending_debug_matched ? pending_gnss_debug_context_.adaptive_gnss_pos_weight_phase
                                  : empty_csv_label) << ','
        << (pending_debug_matched ? pending_gnss_debug_context_.adaptive_gnss_pos_weight_reason
                                  : empty_csv_label) << ','
        << pending_value(pending_gnss_debug_context_.adaptive_gnss_pos_weight_effective_std_h_m) << ','
        << S(0, 0) << ',' << S(0, 1) << ',' << S(1, 1) << ','
        << debug.gnss_position_gate_threshold_nis << ','
        << pos_cov_n_m2 << ',' << pos_cov_e_m2 << ',' << pos_cov_u_m2 << ','
        << vel_cov_n_m2ps2 << ',' << vel_cov_e_m2ps2 << ',' << vel_cov_d_m2ps2 << ','
        << pending_flag(pending_gnss_debug_context_.turning_now) << ','
        << pending_flag(pending_gnss_debug_context_.post_turn_context) << ','
        << pending_flag(pending_gnss_debug_context_.armed_cruise_context) << ','
        << pending_flag(pending_gnss_debug_context_.terminal_descent_context) << ','
        << pending_value(pending_gnss_debug_context_.horizontal_speed_mps) << ','
        << pending_value(pending_gnss_debug_context_.vertical_speed_mps) << ','
        << pending_value(pending_gnss_debug_context_.gyro_deg_s) << ','
        << pending_value(pending_gnss_debug_context_.source_yaw_rate_deg_s) << ','
        << std::clamp(horizontal_consistency_score_trigger_, 0.0, 1.0) << ','
        << horizontal_consistency_persistence_updates_ << ','
        << horizontal_consistency_nis_start_h_2d_ << ','
        << horizontal_consistency_nis_full_h_2d_ << ','
        << horizontal_consistency_residual_start_h_m_ << ','
        << horizontal_consistency_residual_full_h_m_ << ','
        << horizontal_consistency_core_gnss_start_h_m_ << ','
        << horizontal_consistency_core_gnss_full_h_m_ << ','
        << horizontal_consistency_heading_residual_start_deg_ << ','
        << horizontal_consistency_heading_residual_full_deg_ << ','
        << horizontal_consistency_min_horizontal_speed_mps_ << ','
        << horizontal_consistency_max_vertical_speed_mps_ << '\n';
      horizontal_consistency_debug_csv_.flush();
    }
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
    const PhaseErrorMemoryDebug phase_error_memory_debug =
      evaluatePhaseErrorMemoryDebug_(debug, pending_debug_matched);

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
      << debug.gnss_velocity_std_ned_mps.z() << ',';
    appendBoundedAdaptiveRValues_(gnss_update_debug_csv_, debug.gnss_position_adaptive_r);
    appendBoundedAdaptiveRValues_(gnss_update_debug_csv_, debug.gnss_velocity_adaptive_r);
    gnss_update_debug_csv_
      << (pending_debug_matched ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.native_velocity_valid) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.native_velocity_used) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.native_velocity_override_active) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.native_velocity_outlier_guard_enabled) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.native_velocity_outlier_guard_phase_allowed) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.native_velocity_outlier_guard_motion_ok) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.native_velocity_outlier_guard_active) ? 1 : 0) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.native_velocity_outlier_guard_reason : "unknown") << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.native_velocity_outlier_guard_action : "unknown") << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.native_velocity_outlier_guard_reweight_applied) ? 1 : 0) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.native_velocity_outlier_guard_speed_mismatch_mps : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.native_velocity_outlier_guard_core_residual_h_mps : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.native_velocity_outlier_guard_reweight_std_h_mps : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.native_velocity_outlier_guard_reweight_std_u_mps : std::numeric_limits<double>::quiet_NaN()) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.velocity_residual_boost_active) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.gnss_velocity_outward_damping_enabled) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.gnss_velocity_outward_damping_phase_allowed) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.gnss_velocity_outward_damping_motion_ok) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.gnss_velocity_outward_damping_triggered) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.gnss_velocity_outward_damping_held) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.gnss_velocity_outward_damping_active) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.gnss_velocity_outward_damping_applied) ? 1 : 0) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.gnss_velocity_outward_damping_reason : "unknown") << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.gnss_velocity_outward_damping_core_gnss_diff_n_m : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.gnss_velocity_outward_damping_core_gnss_diff_e_m : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.gnss_velocity_outward_damping_core_gnss_diff_h_m : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.gnss_velocity_outward_damping_core_native_residual_n_mps : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.gnss_velocity_outward_damping_core_native_residual_e_mps : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.gnss_velocity_outward_damping_core_native_residual_h_mps : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.gnss_velocity_outward_damping_radial_outward_mps : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.gnss_velocity_outward_damping_radial_score : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.gnss_velocity_outward_damping_hold_score : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.gnss_velocity_outward_damping_persistent_count : 0) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.gnss_velocity_outward_damping_hold_remaining_updates : 0) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.gnss_velocity_outward_damping_selected_std_h_mps : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.gnss_velocity_outward_damping_target_std_h_mps : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.gnss_velocity_outward_damping_effective_std_h_mps : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.gnss_velocity_outward_damping_multiplier_h : std::numeric_limits<double>::quiet_NaN()) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.turn_postturn_native_velocity_deweight_enabled) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.turn_postturn_native_velocity_deweight_phase_allowed) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.turn_postturn_native_velocity_deweight_context_ok) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.turn_postturn_native_velocity_deweight_motion_ok) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.turn_postturn_native_velocity_deweight_triggered) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.turn_postturn_native_velocity_deweight_active) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.turn_postturn_native_velocity_deweight_applied) ? 1 : 0) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.turn_postturn_native_velocity_deweight_reason : "unknown") << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.turn_postturn_native_velocity_deweight_core_gnss_diff_n_m : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.turn_postturn_native_velocity_deweight_core_gnss_diff_e_m : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.turn_postturn_native_velocity_deweight_core_gnss_diff_h_m : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.turn_postturn_native_velocity_deweight_core_native_residual_n_mps : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.turn_postturn_native_velocity_deweight_core_native_residual_e_mps : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.turn_postturn_native_velocity_deweight_core_native_residual_h_mps : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.turn_postturn_native_velocity_deweight_radial_mps : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.turn_postturn_native_velocity_deweight_radial_abs_score : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.turn_postturn_native_velocity_deweight_residual_score : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.turn_postturn_native_velocity_deweight_score : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.turn_postturn_native_velocity_deweight_persistent_count : 0) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.turn_postturn_native_velocity_deweight_selected_std_h_mps : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.turn_postturn_native_velocity_deweight_target_std_h_mps : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.turn_postturn_native_velocity_deweight_effective_std_h_mps : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.turn_postturn_native_velocity_deweight_multiplier_h : std::numeric_limits<double>::quiet_NaN()) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.position_override_active) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.position_residual_boost_active) ? 1 : 0) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.gnss_position_std_source_label : "unknown") << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.gnss_position_response_boost_enabled) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.gnss_position_response_boost_phase_allowed) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.gnss_position_response_boost_motion_ok) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.gnss_position_response_boost_triggered) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.gnss_position_response_boost_active) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.gnss_position_response_boost_applied) ? 1 : 0) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.gnss_position_response_boost_reason : "unknown") << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.gnss_position_response_boost_residual_h_m : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.gnss_position_response_boost_last_residual_h_m : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.gnss_position_response_boost_core_gnss_diff_h_m : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.gnss_position_response_boost_residual_score : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.gnss_position_response_boost_persistent_count : 0) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.gnss_position_response_boost_selected_std_h_m : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.gnss_position_response_boost_target_std_h_m : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.gnss_position_response_boost_effective_std_h_m : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.gnss_position_response_boost_multiplier_h : std::numeric_limits<double>::quiet_NaN()) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.gnss_position_gain_response_enabled) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.gnss_position_gain_response_phase_allowed) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.gnss_position_gain_response_context_ok) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.gnss_position_gain_response_motion_ok) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.gnss_position_gain_response_gain_ok) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.gnss_position_gain_response_triggered) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.gnss_position_gain_response_active) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.gnss_position_gain_response_applied) ? 1 : 0) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.gnss_position_gain_response_reason : "unknown") << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.gnss_position_gain_response_residual_h_m : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.gnss_position_gain_response_hnis_h : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.gnss_position_gain_response_prev_dx_pos_h_m : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.gnss_position_gain_response_prev_residual_h_m : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.gnss_position_gain_response_prev_dx_over_residual_h : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.gnss_position_gain_response_residual_score : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.gnss_position_gain_response_hnis_score : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.gnss_position_gain_response_gain_score : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.gnss_position_gain_response_score : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.gnss_position_gain_response_persistent_count : 0) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.gnss_position_gain_response_selected_std_h_m : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.gnss_position_gain_response_target_std_h_m : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.gnss_position_gain_response_effective_std_h_m : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.gnss_position_gain_response_multiplier_h : std::numeric_limits<double>::quiet_NaN()) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.motion_gnss_pos_weight_enabled) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.motion_gnss_pos_weight_phase_allowed) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.motion_gnss_pos_weight_motion_ok) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.motion_gnss_pos_weight_active) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.motion_gnss_pos_weight_applied) ? 1 : 0) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.motion_gnss_pos_weight_reason : "unknown") << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.motion_gnss_pos_weight_speed_score : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.motion_gnss_pos_weight_target_std_h_m : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.motion_gnss_pos_weight_effective_std_h_m : std::numeric_limits<double>::quiet_NaN()) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.gnss_pos_recovery_weight_enabled) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.gnss_pos_recovery_weight_phase_allowed) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.gnss_pos_recovery_weight_motion_ok) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.gnss_pos_recovery_weight_triggered) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.gnss_pos_recovery_weight_held) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.gnss_pos_recovery_weight_active) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.gnss_pos_recovery_weight_applied) ? 1 : 0) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.gnss_pos_recovery_weight_reason : "unknown") << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.gnss_pos_recovery_weight_residual_h_m : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.gnss_pos_recovery_weight_core_gnss_diff_h_m : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.gnss_pos_recovery_weight_residual_score : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.gnss_pos_recovery_weight_core_score : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.gnss_pos_recovery_weight_score : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.gnss_pos_recovery_weight_persistent_count : 0) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.gnss_pos_recovery_weight_hold_remaining_sec : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.gnss_pos_recovery_weight_selected_std_h_m : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.gnss_pos_recovery_weight_target_std_h_m : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.gnss_pos_recovery_weight_state_std_h_m : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.gnss_pos_recovery_weight_effective_std_h_m : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.gnss_pos_recovery_weight_multiplier_h : std::numeric_limits<double>::quiet_NaN()) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.context_gnss_pos_floor_enabled) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.context_gnss_pos_floor_phase_allowed) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.context_gnss_pos_floor_active) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.context_gnss_pos_floor_applied) ? 1 : 0) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.context_gnss_pos_floor_reason : "unknown") << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.context_gnss_pos_floor_selected_std_h_m : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.context_gnss_pos_floor_target_floor_h_m : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.context_gnss_pos_floor_state_floor_h_m : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.context_gnss_pos_floor_effective_std_h_m : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.context_gnss_pos_floor_multiplier_h : std::numeric_limits<double>::quiet_NaN()) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.adaptive_gnss_pos_weight_enabled) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.adaptive_gnss_pos_weight_active) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.adaptive_gnss_pos_weight_applied) ? 1 : 0) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.adaptive_gnss_pos_weight_phase : "unknown") << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.adaptive_gnss_pos_weight_reason : "unknown") << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.adaptive_gnss_pos_weight_residual_h_m : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.adaptive_gnss_pos_weight_persistent_count : 0) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.adaptive_gnss_pos_weight_selected_std_h_m : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.adaptive_gnss_pos_weight_target_floor_h_m : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.adaptive_gnss_pos_weight_floor_state_h_m : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.adaptive_gnss_pos_weight_effective_std_h_m : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.adaptive_gnss_pos_weight_multiplier_h : std::numeric_limits<double>::quiet_NaN()) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.mission_cov_hygiene_enabled) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.mission_cov_hygiene_phase_allowed) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.mission_cov_hygiene_accepted_recent) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.mission_cov_hygiene_triggered) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.mission_cov_hygiene_active) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.mission_cov_hygiene_applied) ? 1 : 0) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.mission_cov_hygiene_phase : "unknown") << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.mission_cov_hygiene_reason : "unknown") << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.mission_cov_hygiene_residual_h_m : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.mission_cov_hygiene_hnis_h : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.mission_cov_hygiene_score : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.mission_cov_hygiene_nis_score : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.mission_cov_hygiene_residual_score : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.mission_cov_hygiene_cov_score : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.mission_cov_hygiene_weak_score : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.mission_cov_hygiene_persistent_count : 0) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.mission_cov_hygiene_pos_std_h_before_m : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.mission_cov_hygiene_floor_h_m : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.mission_cov_hygiene_pos_std_h_after_m : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.mission_cov_hygiene_dx_pos_h_prev_m : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.mission_cov_hygiene_dx_pos_over_resid_prev : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.mission_cov_hygiene_p_nn_before : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.mission_cov_hygiene_p_ee_before : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.mission_cov_hygiene_p_ne_before : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.mission_cov_hygiene_p_nn_after : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.mission_cov_hygiene_p_ee_after : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.mission_cov_hygiene_p_ne_after : std::numeric_limits<double>::quiet_NaN()) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.position_lag_compensation_active) ? 1 : 0) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.position_lag_compensation_sec : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.position_lag_compensation_n_m : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.position_lag_compensation_e_m : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.position_lag_compensation_u_m : std::numeric_limits<double>::quiet_NaN()) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.vertical_cov_reopen_active) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.vertical_cov_reopen_applied) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.post_flight_vertical_cov_reopen_active) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.post_flight_vertical_cov_reopen_applied) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.terminal_descent_vertical_cov_reopen_active) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.terminal_descent_vertical_cov_reopen_applied) ? 1 : 0) << ','
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
      << ((pending_debug_matched && pending_gnss_debug_context_.terminal_descent_context) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.terminal_descent_native_velocity_override_active) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.terminal_descent_horizontal_zero_velocity_active) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.terminal_descent_horizontal_zero_velocity_applied) ? 1 : 0) << ','
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
      << ((pending_debug_matched && pending_gnss_debug_context_.turn_rate_propagation_noise_probe_enabled) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.turn_rate_propagation_noise_probe_phase_allowed) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.turn_rate_propagation_noise_probe_motion_ok) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.turn_rate_propagation_noise_probe_triggered) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.turn_rate_propagation_noise_probe_active) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.turn_rate_propagation_noise_probe_applied) ? 1 : 0) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.turn_rate_propagation_noise_probe_reason : "unknown") << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.turn_rate_propagation_noise_probe_gyro_score : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.turn_rate_propagation_noise_probe_arw_q_scale : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.turn_rate_propagation_noise_probe_vrw_q_scale : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.turn_rate_propagation_noise_probe_gyrbias_q_scale : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.turn_rate_propagation_noise_probe_accbias_q_scale : std::numeric_limits<double>::quiet_NaN()) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.accbias_z_propagation_probe_enabled) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.accbias_z_propagation_probe_apply_noise_scale) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.accbias_z_propagation_probe_have_bias) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.accbias_z_propagation_probe_phase_allowed) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.accbias_z_propagation_probe_motion_ok) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.accbias_z_propagation_probe_triggered) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.accbias_z_propagation_probe_active) ? 1 : 0) << ','
      << ((pending_debug_matched && pending_gnss_debug_context_.accbias_z_propagation_probe_applied) ? 1 : 0) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.accbias_z_propagation_probe_reason : "unknown") << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.accbias_z_propagation_probe_accbias_z_mps2 : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.accbias_z_propagation_probe_bias_score : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.accbias_z_propagation_probe_arw_q_scale : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.accbias_z_propagation_probe_vrw_q_scale : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.accbias_z_propagation_probe_gyrbias_q_scale : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.accbias_z_propagation_probe_accbias_q_scale : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.native_velocity_std_h_mps : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.native_velocity_std_u_mps : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.native_velocity_vN_mps : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.native_velocity_vE_mps : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.native_velocity_vD_mps : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.core_velocity_vN_mps : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.core_velocity_vE_mps : std::numeric_limits<double>::quiet_NaN()) << ','
      << (pending_debug_matched ? pending_gnss_debug_context_.core_velocity_vD_mps : std::numeric_limits<double>::quiet_NaN()) << ','
      << (phase_error_memory_debug.enabled ? 1 : 0) << ','
      << (phase_error_memory_debug.state_update_matched ? 1 : 0) << ','
      << phase_error_memory_debug.recent_turnpost_age_sec << ','
      << (phase_error_memory_debug.recent_turnpost_active ? 1 : 0) << ','
      << phase_error_memory_debug.residual_h_m << ','
      << phase_error_memory_debug.dx_pos_h_m << ','
      << phase_error_memory_debug.dx_over_residual_h << ','
      << phase_error_memory_debug.pos_std_h_before_m << ','
      << (phase_error_memory_debug.pressure_active ? 1 : 0) << ','
      << (phase_error_memory_debug.candidate_active ? 1 : 0) << ','
      << phase_error_memory_debug.reason << ','
      << phase_error_memory_debug.residual_threshold_h_m << ','
      << phase_error_memory_debug.dx_over_residual_threshold << ','
      << phase_error_memory_debug.recent_turnpost_hold_sec << '\n';
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

  void logStateUpdateDebugIfNeeded_()
  {
    if (!state_update_debug_csv_.is_open() || !core_) {
      return;
    }

    const std::vector<kfcore::StateUpdateDebug> events = core_->stateUpdateDebugEvents();
    if (events.empty()) {
      return;
    }

    const double now_sec = now().seconds();
    const HeadingMotionContext motion_ctx = buildHeadingMotionContext_(now_sec, false);
    const double current_armed_time_sec =
      (mavros_armed_ && std::isfinite(last_armed_transition_time_sec_))
        ? now_sec - last_armed_transition_time_sec_
        : std::numeric_limits<double>::quiet_NaN();
    const double nan = std::numeric_limits<double>::quiet_NaN();

    const auto vec_value = [](const Eigen::Vector3d & v, int idx) {
      return (idx >= 0 && idx < 3 && std::isfinite(v(idx)))
        ? v(idx)
        : std::numeric_limits<double>::quiet_NaN();
    };
    const auto dx_value = [](const Eigen::VectorXd & v, int idx) {
      return (idx >= 0 && idx < v.size() && std::isfinite(v(idx)))
        ? v(idx)
        : std::numeric_limits<double>::quiet_NaN();
    };
    const auto cov_value = [](const Eigen::MatrixXd & m, int r, int c) {
      return (r >= 0 && c >= 0 && r < m.rows() && c < m.cols() && std::isfinite(m(r, c)))
        ? m(r, c)
        : std::numeric_limits<double>::quiet_NaN();
    };
    const auto gain_value = [](const Eigen::MatrixXd & m, int r, int c) {
      return (r >= 0 && c >= 0 && r < m.rows() && c < m.cols() && std::isfinite(m(r, c)))
        ? m(r, c)
        : std::numeric_limits<double>::quiet_NaN();
    };
    const auto finite_value = [](double value) {
      return std::isfinite(value) ? value : std::numeric_limits<double>::quiet_NaN();
    };
    const auto mat3_value = [](const Eigen::Matrix3d & m, int r, int c) {
      return (r >= 0 && c >= 0 && r < 3 && c < 3 && std::isfinite(m(r, c)))
        ? m(r, c)
        : std::numeric_limits<double>::quiet_NaN();
    };
    const auto deg = [](double rad) {
      return std::isfinite(rad) ? rad * 180.0 / M_PI : std::numeric_limits<double>::quiet_NaN();
    };
    const auto hypot2 = [](double x, double y) {
      return (std::isfinite(x) && std::isfinite(y)) ? std::hypot(x, y)
                                                    : std::numeric_limits<double>::quiet_NaN();
    };
    const auto blh_to_enu = [&](const Eigen::Vector3d & blh) -> Eigen::Vector3d {
      Eigen::Vector3d enu = Eigen::Vector3d::Constant(nan);
      if (!have_origin_ ||
          !std::isfinite(blh.x()) ||
          !std::isfinite(blh.y()) ||
          !std::isfinite(blh.z())) {
        return enu;
      }
      double x_ecef = nan;
      double y_ecef = nan;
      double z_ecef = nan;
      geo::llh_to_ecef(blh.x(), blh.y(), blh.z(), x_ecef, y_ecef, z_ecef);
      enu = geo::ecef_to_enu({x_ecef, y_ecef, z_ecef},
                             origin_ecef_, origin_lat_, origin_lon_);
      if (!std::isfinite(enu.x()) || !std::isfinite(enu.y()) || !std::isfinite(enu.z())) {
        enu = Eigen::Vector3d::Constant(nan);
      }
      return enu;
    };

    for (const auto & event : events) {
      if (!event.valid || event.sequence == 0 ||
          event.sequence <= last_logged_state_update_debug_sequence_) {
        continue;
      }

      const double log_time_sec =
        std::isfinite(event.update_time_sec) ? event.update_time_sec : now_sec;
      const bool rate_limited_event = event.event_type == "heading";
      const bool same_time_as_last =
        rate_limited_event &&
        std::isfinite(log_time_sec) &&
        std::isfinite(last_state_update_debug_log_update_time_sec_) &&
        std::abs(log_time_sec - last_state_update_debug_log_update_time_sec_) <= 1.0e-6;
      if (rate_limited_event &&
          state_update_debug_max_rate_hz_ > 0.0 &&
          std::isfinite(log_time_sec) &&
          std::isfinite(last_state_update_debug_log_update_time_sec_) &&
          !same_time_as_last) {
        const double min_period_sec = 1.0 / state_update_debug_max_rate_hz_;
        if ((log_time_sec - last_state_update_debug_log_update_time_sec_) < min_period_sec) {
          last_logged_state_update_debug_sequence_ = event.sequence;
          continue;
        }
      }

      const bool pending_debug_matched =
        pending_gnss_debug_context_.valid &&
        std::isfinite(pending_gnss_debug_context_.update_time_sec) &&
        std::isfinite(event.update_time_sec) &&
        std::abs(pending_gnss_debug_context_.update_time_sec - event.update_time_sec) <= 1e-6;

      const std::string & mode =
        pending_debug_matched ? pending_gnss_debug_context_.mavros_mode : mavros_mode_;
      const bool armed =
        pending_debug_matched ? pending_gnss_debug_context_.armed : mavros_armed_;
      const double armed_time_sec =
        pending_debug_matched ? pending_gnss_debug_context_.armed_time_sec
                              : current_armed_time_sec;
      const bool turning_now =
        pending_debug_matched ? pending_gnss_debug_context_.turning_now
                              : motion_ctx.turning_now;
      const bool post_turn_context =
        pending_debug_matched ? pending_gnss_debug_context_.post_turn_context
                              : motion_ctx.post_turn_context;
      const bool armed_cruise_context =
        pending_debug_matched ? pending_gnss_debug_context_.armed_cruise_context
                              : motion_ctx.armed_cruise_force_relock_context;
      const bool terminal_descent_context =
        pending_debug_matched ? pending_gnss_debug_context_.terminal_descent_context
                              : motion_ctx.terminal_descent_context;
      const double horizontal_speed_mps =
        pending_debug_matched ? pending_gnss_debug_context_.horizontal_speed_mps
                              : last_mavros_horizontal_speed_mps_;
      const double vertical_speed_mps =
        pending_debug_matched ? pending_gnss_debug_context_.vertical_speed_mps
                              : last_mavros_vertical_speed_mps_;
      const double gyro_deg_s =
        pending_debug_matched ? pending_gnss_debug_context_.gyro_deg_s
                              : last_imu_gyro_norm_deg_s_;
      const double source_yaw_rate_deg_s =
        pending_debug_matched ? pending_gnss_debug_context_.source_yaw_rate_deg_s
                              : last_mavros_heading_rate_deg_s_;
      const double diag_gnss_source_time_sec =
        pending_debug_matched ? pending_gnss_debug_context_.gnss_source_time_sec
                              : last_gnss_source_time_sec_;
      const double diag_gnss_rx_ros_time_sec =
        pending_debug_matched ? pending_gnss_debug_context_.gnss_rx_ros_time_sec
                              : last_gnss_rx_ros_time_sec_;
      const double diag_core_time_before_update_sec =
        pending_debug_matched ? pending_gnss_debug_context_.core_time_before_update_sec
                              : last_core_time_;
      const double diag_ros_time_sec =
        pending_debug_matched ? pending_gnss_debug_context_.ros_time_sec : now_sec;
      const double diag_gnss_source_age_at_update_sec =
        pending_debug_matched
          ? pending_gnss_debug_context_.gnss_source_age_at_update_sec
          : (std::isfinite(diag_gnss_source_time_sec)
               ? diag_ros_time_sec - diag_gnss_source_time_sec
               : nan);
      const double diag_gnss_rx_age_at_update_sec =
        pending_debug_matched
          ? pending_gnss_debug_context_.gnss_rx_age_at_update_sec
          : (std::isfinite(diag_gnss_rx_ros_time_sec)
               ? diag_ros_time_sec - diag_gnss_rx_ros_time_sec
               : nan);
      const double diag_core_time_before_update_minus_update_sec =
        pending_debug_matched
          ? pending_gnss_debug_context_.core_time_before_update_minus_update_sec
          : (std::isfinite(diag_core_time_before_update_sec) &&
             std::isfinite(event.update_time_sec)
               ? diag_core_time_before_update_sec - event.update_time_sec
               : nan);
      const double diag_gnss_update_minus_source_sec =
        pending_debug_matched
          ? pending_gnss_debug_context_.gnss_update_minus_source_sec
          : (std::isfinite(event.update_time_sec) && std::isfinite(diag_gnss_source_time_sec)
               ? event.update_time_sec - diag_gnss_source_time_sec
               : nan);
      const double diag_update_time_minus_ros_sec =
        pending_debug_matched
          ? pending_gnss_debug_context_.update_time_minus_ros_sec
          : (std::isfinite(event.update_time_sec) ? event.update_time_sec - diag_ros_time_sec
                                                  : nan);
      const double diag_latest_heading_update_age_sec =
        pending_debug_matched
          ? pending_gnss_debug_context_.latest_heading_update_age_sec
          : (std::isfinite(last_heading_update_time_sec_)
               ? now_sec - last_heading_update_time_sec_
               : nan);
      const double diag_recent_turnpost_age_sec =
        pending_debug_matched
          ? pending_gnss_debug_context_.phase_error_memory_recent_turnpost_age_sec
          : (std::isfinite(phase_error_memory_last_turnpost_context_sec_)
               ? now_sec - phase_error_memory_last_turnpost_context_sec_
               : nan);
      const double diag_last_turning_age_sec =
        pending_debug_matched
          ? pending_gnss_debug_context_.last_turning_age_sec
          : (std::isfinite(last_turning_heading_time_sec_)
               ? now_sec - last_turning_heading_time_sec_
               : nan);
      const double diag_post_turn_hold_remaining_sec =
        pending_debug_matched
          ? pending_gnss_debug_context_.post_turn_hold_remaining_sec
          : (std::isfinite(post_turn_hold_end_time_sec_) ? post_turn_hold_end_time_sec_ - now_sec
                                                         : nan);

      const double dx_pos_n = dx_value(event.dx, 0);
      const double dx_pos_e = dx_value(event.dx, 1);
      const double dx_pos_h = hypot2(dx_pos_n, dx_pos_e);
      const double dx_vel_n = dx_value(event.dx, 3);
      const double dx_vel_e = dx_value(event.dx, 4);
      const double yaw_before_deg = deg(vec_value(event.euler_before_rad, 2));
      const double yaw_after_deg = deg(vec_value(event.euler_after_rad, 2));
      const Eigen::Vector3d core_enu_before = blh_to_enu(event.pos_blh_before_rad_m);
      const Eigen::Vector3d core_enu_after = blh_to_enu(event.pos_blh_after_rad_m);
      const bool have_state_update_enu =
        std::isfinite(core_enu_before.x()) && std::isfinite(core_enu_before.y()) &&
        std::isfinite(core_enu_before.z()) && std::isfinite(core_enu_after.x()) &&
        std::isfinite(core_enu_after.y()) && std::isfinite(core_enu_after.z());
      const Eigen::Vector3d px4_sphere_enu_before =
        have_state_update_enu ? applyPx4SphereProjection_(core_enu_before)
                              : Eigen::Vector3d::Constant(nan);
      const Eigen::Vector3d px4_sphere_enu_after =
        have_state_update_enu ? applyPx4SphereProjection_(core_enu_after)
                              : Eigen::Vector3d::Constant(nan);
      const Eigen::Vector2d px4_sphere_scales =
        have_origin_ ? px4SphereProjectionScales_() : Eigen::Vector2d::Constant(nan);
      const double residual_n = vec_value(event.gnss_position_residual_neu_m, 0);
      const double residual_e = vec_value(event.gnss_position_residual_neu_m, 1);
      const double residual_u = vec_value(event.gnss_position_residual_neu_m, 2);
      const double residual_h = hypot2(residual_n, residual_e);
      Eigen::Vector3d diag_update_move_enu = Eigen::Vector3d::Constant(nan);
      if (have_state_update_enu) {
        diag_update_move_enu = core_enu_after - core_enu_before;
      }
      const double diag_update_move_h =
        hypot2(diag_update_move_enu.x(), diag_update_move_enu.y());
      const double diag_update_move_dot_residual =
        (std::isfinite(diag_update_move_enu.x()) &&
         std::isfinite(diag_update_move_enu.y()) &&
         std::isfinite(residual_e) &&
         std::isfinite(residual_n))
          ? diag_update_move_enu.x() * residual_e + diag_update_move_enu.y() * residual_n
          : nan;
      const double diag_update_move_residual_cos =
        (std::isfinite(diag_update_move_dot_residual) &&
         std::isfinite(diag_update_move_h) &&
         std::isfinite(residual_h) &&
         diag_update_move_h > 1.0e-9 &&
         residual_h > 1.0e-9)
          ? diag_update_move_dot_residual / (diag_update_move_h * residual_h)
          : nan;
      const double diag_core_velocity_before_enu_e_mps =
        vec_value(event.vel_before_ned_mps, 1);
      const double diag_core_velocity_before_enu_n_mps =
        vec_value(event.vel_before_ned_mps, 0);
      const double diag_core_velocity_before_enu_u_mps =
        std::isfinite(vec_value(event.vel_before_ned_mps, 2))
          ? -vec_value(event.vel_before_ned_mps, 2)
          : nan;
      const double diag_core_velocity_after_enu_e_mps =
        vec_value(event.vel_after_ned_mps, 1);
      const double diag_core_velocity_after_enu_n_mps =
        vec_value(event.vel_after_ned_mps, 0);
      const double diag_core_velocity_after_enu_u_mps =
        std::isfinite(vec_value(event.vel_after_ned_mps, 2))
          ? -vec_value(event.vel_after_ned_mps, 2)
          : nan;
      Eigen::Vector3d diag_projection_axis_enu = Eigen::Vector3d::Constant(nan);
      if (have_state_update_enu) {
        diag_projection_axis_enu = px4_sphere_enu_before - core_enu_before;
      }
      const double diag_projection_axis_h =
        hypot2(diag_projection_axis_enu.x(), diag_projection_axis_enu.y());
      const double diag_projection_axis_e_unit =
        (std::isfinite(diag_projection_axis_h) && diag_projection_axis_h > 1.0e-9)
          ? diag_projection_axis_enu.x() / diag_projection_axis_h
          : nan;
      const double diag_projection_axis_n_unit =
        (std::isfinite(diag_projection_axis_h) && diag_projection_axis_h > 1.0e-9)
          ? diag_projection_axis_enu.y() / diag_projection_axis_h
          : nan;
      const double diag_update_move_along_projection_axis =
        (std::isfinite(diag_projection_axis_e_unit) &&
         std::isfinite(diag_projection_axis_n_unit) &&
         std::isfinite(diag_update_move_enu.x()) &&
         std::isfinite(diag_update_move_enu.y()))
          ? diag_update_move_enu.x() * diag_projection_axis_e_unit +
              diag_update_move_enu.y() * diag_projection_axis_n_unit
          : nan;
      const double diag_update_move_perp_projection_axis =
        (std::isfinite(diag_projection_axis_e_unit) &&
         std::isfinite(diag_projection_axis_n_unit) &&
         std::isfinite(diag_update_move_enu.x()) &&
         std::isfinite(diag_update_move_enu.y()))
          ? -diag_update_move_enu.x() * diag_projection_axis_n_unit +
              diag_update_move_enu.y() * diag_projection_axis_e_unit
          : nan;
      const double diag_residual_along_projection_axis =
        (std::isfinite(diag_projection_axis_e_unit) &&
         std::isfinite(diag_projection_axis_n_unit) &&
         std::isfinite(residual_e) &&
         std::isfinite(residual_n))
          ? residual_e * diag_projection_axis_e_unit + residual_n * diag_projection_axis_n_unit
          : nan;
      const double diag_residual_perp_projection_axis =
        (std::isfinite(diag_projection_axis_e_unit) &&
         std::isfinite(diag_projection_axis_n_unit) &&
         std::isfinite(residual_e) &&
         std::isfinite(residual_n))
          ? -residual_e * diag_projection_axis_n_unit + residual_n * diag_projection_axis_e_unit
          : nan;
      const double dx_pos_h_over_residual_h =
        (std::isfinite(dx_pos_h) && std::isfinite(residual_h) && residual_h > 1.0e-9)
          ? (dx_pos_h / residual_h)
          : nan;
      const double k_pn_n = gain_value(event.kalman_gain, 0, 0);
      const double k_pn_e = gain_value(event.kalman_gain, 0, 1);
      const double k_pe_n = gain_value(event.kalman_gain, 1, 0);
      const double k_pe_e = gain_value(event.kalman_gain, 1, 1);
      const double k_h_trace =
        (std::isfinite(k_pn_n) && std::isfinite(k_pe_e)) ? (k_pn_n + k_pe_e) : nan;
      const double k_h_fro =
        (std::isfinite(k_pn_n) && std::isfinite(k_pn_e) &&
         std::isfinite(k_pe_n) && std::isfinite(k_pe_e))
          ? std::sqrt(k_pn_n * k_pn_n + k_pn_e * k_pn_e +
                      k_pe_n * k_pe_n + k_pe_e * k_pe_e)
          : nan;
      const double raw_dx_ba_z = dx_value(event.dx, 14);
      const kfcore::EarlyRecoveryBiasFeedbackDebug & early_recovery_bias_feedback_debug =
        event.early_recovery_bias_feedback;

      state_update_debug_csv_
        << event.sequence << ','
        << now_sec << ','
        << event.update_time_sec << ','
        << event.event_type << ','
        << event.reason << ','
        << event.update_mode << ','
        << observationUpdateModeLabel_(event.update_mode) << ','
        << (event.applied ? 1 : 0) << ','
        << (pending_debug_matched ? 1 : 0) << ','
        << (mode.empty() ? "unknown" : mode) << ','
        << (armed ? 1 : 0) << ','
        << armed_time_sec << ','
        << (turning_now ? 1 : 0) << ','
        << (post_turn_context ? 1 : 0) << ','
        << (armed_cruise_context ? 1 : 0) << ','
        << (terminal_descent_context ? 1 : 0) << ','
        << horizontal_speed_mps << ','
        << vertical_speed_mps << ','
        << gyro_deg_s << ','
        << source_yaw_rate_deg_s << ','
        << deg(vec_value(event.pos_blh_before_rad_m, 0)) << ','
        << deg(vec_value(event.pos_blh_before_rad_m, 1)) << ','
        << vec_value(event.pos_blh_before_rad_m, 2) << ','
        << deg(vec_value(event.pos_blh_after_rad_m, 0)) << ','
        << deg(vec_value(event.pos_blh_after_rad_m, 1)) << ','
        << vec_value(event.pos_blh_after_rad_m, 2) << ','
        << (have_state_update_enu ? 1 : 0) << ','
        << core_enu_before.x() << ','
        << core_enu_before.y() << ','
        << core_enu_before.z() << ','
        << core_enu_after.x() << ','
        << core_enu_after.y() << ','
        << core_enu_after.z() << ','
        << px4_sphere_enu_before.x() << ','
        << px4_sphere_enu_before.y() << ','
        << px4_sphere_enu_before.z() << ','
        << px4_sphere_enu_after.x() << ','
        << px4_sphere_enu_after.y() << ','
        << px4_sphere_enu_after.z() << ','
        << (last_publish_projection_active_ ? 1 : 0) << ','
        << (last_publish_projection_action_.empty() ? "unknown" : last_publish_projection_action_) << ','
        << px4_sphere_scales.x() << ','
        << px4_sphere_scales.y() << ','
        << vec_value(event.vel_before_ned_mps, 0) << ','
        << vec_value(event.vel_before_ned_mps, 1) << ','
        << vec_value(event.vel_before_ned_mps, 2) << ','
        << vec_value(event.vel_after_ned_mps, 0) << ','
        << vec_value(event.vel_after_ned_mps, 1) << ','
        << vec_value(event.vel_after_ned_mps, 2) << ','
        << deg(vec_value(event.euler_before_rad, 0)) << ','
        << deg(vec_value(event.euler_before_rad, 1)) << ','
        << yaw_before_deg << ','
        << deg(vec_value(event.euler_after_rad, 0)) << ','
        << deg(vec_value(event.euler_after_rad, 1)) << ','
        << yaw_after_deg << ','
        << shortestAngleDiffDeg_(yaw_after_deg, yaw_before_deg) << ','
        << vec_value(event.gyrbias_before_radps, 0) << ','
        << vec_value(event.gyrbias_before_radps, 1) << ','
        << vec_value(event.gyrbias_before_radps, 2) << ','
        << vec_value(event.gyrbias_after_radps, 0) << ','
        << vec_value(event.gyrbias_after_radps, 1) << ','
        << vec_value(event.gyrbias_after_radps, 2) << ','
        << vec_value(event.accbias_before_mps2, 0) << ','
        << vec_value(event.accbias_before_mps2, 1) << ','
        << vec_value(event.accbias_before_mps2, 2) << ','
        << vec_value(event.accbias_after_mps2, 0) << ','
        << vec_value(event.accbias_after_mps2, 1) << ','
        << vec_value(event.accbias_after_mps2, 2) << ','
        << dx_pos_n << ','
        << dx_pos_e << ','
        << dx_value(event.dx, 2) << ','
        << dx_pos_h << ','
        << (event.gnss_position_observation_valid ? 1 : 0) << ','
        << event.observation_sequence << ','
        << residual_n << ','
        << residual_e << ','
        << vec_value(event.gnss_position_residual_neu_m, 2) << ','
        << residual_h << ','
        << vec_value(event.gnss_position_std_neu_m, 0) << ','
        << vec_value(event.gnss_position_std_neu_m, 1) << ','
        << vec_value(event.gnss_position_std_neu_m, 2) << ','
        << mat3_value(event.gnss_position_innovation_cov_neu_m2, 0, 0) << ','
        << mat3_value(event.gnss_position_innovation_cov_neu_m2, 0, 1) << ','
        << mat3_value(event.gnss_position_innovation_cov_neu_m2, 0, 2) << ','
        << mat3_value(event.gnss_position_innovation_cov_neu_m2, 1, 1) << ','
        << mat3_value(event.gnss_position_innovation_cov_neu_m2, 1, 2) << ','
        << mat3_value(event.gnss_position_innovation_cov_neu_m2, 2, 2) << ','
        << event.gnss_position_nis_h_2d << ','
        << event.gnss_position_nis_u_1d << ','
        << event.gnss_position_nis_3d << ','
        << event.gnss_position_gate_threshold_nis << ','
        << (event.gnss_position_update_accepted ? 1 : 0) << ','
        << (event.gnss_position_update_rejected ? 1 : 0) << ','
        << event.gnss_position_update_reason << ',';
      appendBoundedAdaptiveRValues_(state_update_debug_csv_, event.gnss_position_adaptive_r);
      appendBoundedAdaptiveRValues_(state_update_debug_csv_, event.gnss_velocity_adaptive_r);
      state_update_debug_csv_
        << dx_pos_h_over_residual_h << ','
        << k_pn_n << ','
        << k_pn_e << ','
        << k_pe_n << ','
        << k_pe_e << ','
        << gain_value(event.kalman_gain, 2, 2) << ','
        << gain_value(event.kalman_gain, 3, 0) << ','
        << gain_value(event.kalman_gain, 4, 1) << ','
        << gain_value(event.kalman_gain, 8, 0) << ','
        << gain_value(event.kalman_gain, 8, 1) << ','
        << k_h_trace << ','
        << k_h_fro << ','
        << dx_vel_n << ','
        << dx_vel_e << ','
        << dx_value(event.dx, 5) << ','
        << hypot2(dx_vel_n, dx_vel_e) << ','
        << dx_value(event.dx, 6) << ','
        << dx_value(event.dx, 7) << ','
        << dx_value(event.dx, 8) << ','
        << deg(dx_value(event.dx, 8)) << ','
        << dx_value(event.dx, 9) << ','
        << dx_value(event.dx, 10) << ','
        << dx_value(event.dx, 11) << ','
        << dx_value(event.dx, 12) << ','
        << dx_value(event.dx, 13) << ','
        << raw_dx_ba_z << ','
        << (early_recovery_bias_feedback_debug.enabled ? 1 : 0) << ','
        << (early_recovery_bias_feedback_debug.apply_enabled ? 1 : 0) << ','
        << (early_recovery_bias_feedback_debug.candidate ? 1 : 0) << ','
        << (early_recovery_bias_feedback_debug.active ? 1 : 0) << ','
        << (early_recovery_bias_feedback_debug.applied ? 1 : 0) << ','
        << early_recovery_bias_feedback_debug.reason << ','
        << early_recovery_bias_feedback_debug.history_rows << ','
        << early_recovery_bias_feedback_debug.history_sec << ','
        << early_recovery_bias_feedback_debug.armed_time_sec << ','
        << early_recovery_bias_feedback_debug.ba_z_mean_mps2 << ','
        << early_recovery_bias_feedback_debug.residual_u_mean_m << ','
        << early_recovery_bias_feedback_debug.core_gnss_u_mean_m << ','
        << early_recovery_bias_feedback_debug.dx_ba_z_sum_mps2 << ','
        << early_recovery_bias_feedback_debug.negative_dx_ba_z_sum_mps2 << ','
        << early_recovery_bias_feedback_debug.positive_dx_ba_z_sum_mps2 << ','
        << early_recovery_bias_feedback_debug.raw_dx_ba_z_mps2 << ','
        << early_recovery_bias_feedback_debug.selected_dx_ba_z_mps2 << ','
        << early_recovery_bias_feedback_debug.delta_dx_ba_z_mps2 << ','
        << early_recovery_bias_feedback_debug.selected_accbias_z_after_mps2 << ','
        << early_recovery_bias_feedback_debug.negative_dx_scale << ','
        << cov_value(event.covariance_before, 0, 0) << ','
        << cov_value(event.covariance_before, 1, 1) << ','
        << cov_value(event.covariance_before, 2, 2) << ','
        << cov_value(event.covariance_before, 0, 1) << ','
        << cov_value(event.covariance_after, 0, 0) << ','
        << cov_value(event.covariance_after, 1, 1) << ','
        << cov_value(event.covariance_after, 2, 2) << ','
        << cov_value(event.covariance_after, 0, 1) << ','
        << cov_value(event.covariance_before, 3, 3) << ','
        << cov_value(event.covariance_before, 4, 4) << ','
        << cov_value(event.covariance_before, 5, 5) << ','
        << cov_value(event.covariance_before, 3, 4) << ','
        << cov_value(event.covariance_after, 3, 3) << ','
        << cov_value(event.covariance_after, 4, 4) << ','
        << cov_value(event.covariance_after, 5, 5) << ','
        << cov_value(event.covariance_after, 3, 4) << ','
        << cov_value(event.covariance_before, 6, 6) << ','
        << cov_value(event.covariance_before, 7, 7) << ','
        << cov_value(event.covariance_before, 8, 8) << ','
        << cov_value(event.covariance_after, 6, 6) << ','
        << cov_value(event.covariance_after, 7, 7) << ','
        << cov_value(event.covariance_after, 8, 8) << ','
        << cov_value(event.covariance_before, 9, 9) << ','
        << cov_value(event.covariance_before, 10, 10) << ','
        << cov_value(event.covariance_before, 11, 11) << ','
        << cov_value(event.covariance_after, 9, 9) << ','
        << cov_value(event.covariance_after, 10, 10) << ','
        << cov_value(event.covariance_after, 11, 11) << ','
        << cov_value(event.covariance_before, 12, 12) << ','
        << cov_value(event.covariance_before, 13, 13) << ','
        << cov_value(event.covariance_before, 14, 14) << ','
        << cov_value(event.covariance_after, 12, 12) << ','
        << cov_value(event.covariance_after, 13, 13) << ','
        << cov_value(event.covariance_after, 14, 14) << ','
        << finite_value(diag_gnss_source_time_sec) << ','
        << finite_value(diag_gnss_rx_ros_time_sec) << ','
        << finite_value(diag_core_time_before_update_sec) << ','
        << finite_value(diag_gnss_source_age_at_update_sec) << ','
        << finite_value(diag_gnss_rx_age_at_update_sec) << ','
        << finite_value(diag_core_time_before_update_minus_update_sec) << ','
        << finite_value(diag_gnss_update_minus_source_sec) << ','
        << finite_value(diag_update_time_minus_ros_sec) << ','
        << finite_value(diag_latest_heading_update_age_sec) << ','
        << finite_value(diag_recent_turnpost_age_sec) << ','
        << finite_value(diag_last_turning_age_sec) << ','
        << finite_value(diag_post_turn_hold_remaining_sec) << ','
        << diag_update_move_enu.x() << ','
        << diag_update_move_enu.y() << ','
        << diag_update_move_enu.z() << ','
        << finite_value(diag_update_move_h) << ','
        << residual_e << ','
        << residual_n << ','
        << residual_u << ','
        << finite_value(diag_update_move_dot_residual) << ','
        << finite_value(diag_update_move_residual_cos) << ','
        << finite_value(diag_core_velocity_before_enu_e_mps) << ','
        << finite_value(diag_core_velocity_before_enu_n_mps) << ','
        << finite_value(diag_core_velocity_before_enu_u_mps) << ','
        << finite_value(diag_core_velocity_after_enu_e_mps) << ','
        << finite_value(diag_core_velocity_after_enu_n_mps) << ','
        << finite_value(diag_core_velocity_after_enu_u_mps) << ','
        << diag_projection_axis_enu.x() << ','
        << diag_projection_axis_enu.y() << ','
        << finite_value(diag_projection_axis_h) << ','
        << finite_value(diag_update_move_along_projection_axis) << ','
        << finite_value(diag_update_move_perp_projection_axis) << ','
        << finite_value(diag_residual_along_projection_axis) << ','
        << finite_value(diag_residual_perp_projection_axis) << '\n';

      ++state_update_debug_rows_since_flush_;
      if (state_update_debug_rows_since_flush_ >= state_update_debug_flush_interval_) {
        state_update_debug_csv_.flush();
        state_update_debug_rows_since_flush_ = 0;
      }
      if (rate_limited_event) {
        last_state_update_debug_log_update_time_sec_ = log_time_sec;
      }
      last_logged_state_update_debug_sequence_ = event.sequence;
    }
  }

  void logDtrqRuntimeFeatureDebugIfNeeded_()
  {
    if (!dtrq_runtime_feature_debug_csv_.is_open() || !core_) {
      return;
    }

    const std::vector<kfcore::StateUpdateDebug> events = core_->stateUpdateDebugEvents();
    if (events.empty()) {
      return;
    }

    const double now_sec = now().seconds();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const HeadingMotionContext motion_ctx = buildHeadingMotionContext_(now_sec, false);
    const double armed_time_sec =
      (mavros_armed_ && std::isfinite(last_armed_transition_time_sec_))
        ? now_sec - last_armed_transition_time_sec_
        : nan;
    const auto vec_value = [](const Eigen::Vector3d & v, int idx) {
      return (idx >= 0 && idx < 3 && std::isfinite(v(idx)))
        ? v(idx)
        : std::numeric_limits<double>::quiet_NaN();
    };
    const auto finite = [](double value) {
      return std::isfinite(value);
    };
    const auto missing_append = [](std::string & text, const char * item) {
      if (!text.empty()) {
        text += "|";
      }
      text += item;
    };
    const auto hypot2 = [](double x, double y) {
      return (std::isfinite(x) && std::isfinite(y)) ? std::hypot(x, y)
                                                    : std::numeric_limits<double>::quiet_NaN();
    };

    Eigen::Vector3d gnss_enu = Eigen::Vector3d::Constant(nan);
    if (have_origin_ && last_gnss_valid_) {
      double x_ecef = nan;
      double y_ecef = nan;
      double z_ecef = nan;
      geo::llh_to_ecef(last_gnss_lat_rad_, last_gnss_lon_rad_, last_gnss_h_m_, x_ecef, y_ecef, z_ecef);
      gnss_enu = geo::ecef_to_enu({x_ecef, y_ecef, z_ecef}, origin_ecef_, origin_lat_, origin_lon_);
      if (!gnss_enu.allFinite()) {
        gnss_enu = Eigen::Vector3d::Constant(nan);
      }
    }

    for (const auto & event : events) {
      if (!event.valid || event.sequence == 0 ||
          event.sequence <= last_logged_dtrq_runtime_feature_debug_sequence_) {
        continue;
      }
      if (event.event_type != "gnss_position") {
        last_logged_dtrq_runtime_feature_debug_sequence_ = event.sequence;
        continue;
      }

      const double log_time_sec =
        std::isfinite(event.update_time_sec) ? event.update_time_sec : now_sec;
      if (dtrq_runtime_feature_debug_max_rate_hz_ > 0.0 &&
          std::isfinite(last_dtrq_runtime_feature_debug_log_update_time_sec_) &&
          std::isfinite(log_time_sec)) {
        const double min_period_sec = 1.0 / dtrq_runtime_feature_debug_max_rate_hz_;
        if ((log_time_sec - last_dtrq_runtime_feature_debug_log_update_time_sec_) < min_period_sec) {
          last_logged_dtrq_runtime_feature_debug_sequence_ = event.sequence;
          continue;
        }
      }

      const double gnss_source_time_sec = dtrq_gnss_quality_cache_.valid
        ? dtrq_gnss_quality_cache_.source_time_sec
        : last_gnss_source_time_sec_;
      const double gnss_rx_ros_time_sec = dtrq_gnss_quality_cache_.valid
        ? dtrq_gnss_quality_cache_.rx_ros_time_sec
        : last_gnss_rx_ros_time_sec_;
      const double gnss_source_age_sec =
        finite(gnss_source_time_sec) ? now_sec - gnss_source_time_sec : nan;
      const double gnss_rx_age_sec =
        finite(gnss_rx_ros_time_sec) ? now_sec - gnss_rx_ros_time_sec : nan;

      double dtrq_gnss_dt_sec = nan;
      double dtrq_gnss_dp_h_m = nan;
      double dtrq_gnss_dp_u_m = nan;
      if (finite(gnss_source_time_sec) && finite(prev_dtrq_gnss_source_time_sec_)) {
        dtrq_gnss_dt_sec = gnss_source_time_sec - prev_dtrq_gnss_source_time_sec_;
      }
      if (gnss_enu.allFinite() && prev_dtrq_gnss_enu_.allFinite()) {
        const Eigen::Vector3d delta = gnss_enu - prev_dtrq_gnss_enu_;
        dtrq_gnss_dp_h_m = std::hypot(delta.x(), delta.y());
        dtrq_gnss_dp_u_m = delta.z();
      }

      const double horizontal_speed_mps = finite(last_mavros_horizontal_speed_mps_)
        ? last_mavros_horizontal_speed_mps_
        : nan;
      const double vertical_speed_mps = finite(last_mavros_vertical_speed_mps_)
        ? last_mavros_vertical_speed_mps_
        : nan;
      double dtrq_speed_change_abs_mps2 = nan;
      if (finite(horizontal_speed_mps) && finite(prev_dtrq_horizontal_speed_mps_) &&
          finite(log_time_sec) && finite(prev_dtrq_speed_update_time_sec_)) {
        const double dt = log_time_sec - prev_dtrq_speed_update_time_sec_;
        if (dt > 1.0e-6) {
          dtrq_speed_change_abs_mps2 = std::abs(horizontal_speed_mps - prev_dtrq_horizontal_speed_mps_) / dt;
        }
      }

      const double std_n = vec_value(event.gnss_position_std_neu_m, 0);
      const double std_e = vec_value(event.gnss_position_std_neu_m, 1);
      const double std_u = vec_value(event.gnss_position_std_neu_m, 2);
      std::string missing;
      if (!event.applied) missing_append(missing, "not_applied");
      if (!dtrq_gnss_quality_cache_.valid) missing_append(missing, "no_gnss_quality_cache");
      if (!finite(gnss_source_age_sec)) missing_append(missing, "no_gnss_source_age");
      if (!finite(gnss_rx_age_sec)) missing_append(missing, "no_gnss_rx_age");
      if (!finite(std_n) || !finite(std_e)) missing_append(missing, "no_gnss_std_h");
      if (!finite(horizontal_speed_mps)) missing_append(missing, "no_horizontal_speed");
      if (!finite(last_imu_gyro_norm_deg_s_)) missing_append(missing, "no_gyro");
      if (!finite(last_mavros_heading_rate_deg_s_)) missing_append(missing, "no_source_yaw_rate");
      const bool row_valid = missing.empty();
      if (missing.empty()) {
        missing = "none";
      }

      dtrq_runtime_feature_debug_csv_
        << event.sequence << ','
        << now_sec << ','
        << event.update_time_sec << ','
        << armed_time_sec << ','
        << (mavros_armed_ ? 1 : 0) << ','
        << (row_valid ? 1 : 0) << ','
        << missing << ','
        << (dtrq_gnss_quality_cache_.source_kind.empty() ? "unknown" : dtrq_gnss_quality_cache_.source_kind) << ','
        << gnss_source_time_sec << ','
        << gnss_rx_ros_time_sec << ','
        << gnss_source_age_sec << ','
        << gnss_rx_age_sec << ','
        << std_n << ','
        << std_e << ','
        << std_u << ','
        << dtrq_gnss_quality_cache_.fix_type << ','
        << dtrq_gnss_quality_cache_.satellites_used << ','
        << dtrq_gnss_quality_cache_.hdop << ','
        << dtrq_gnss_quality_cache_.vdop << ','
        << dtrq_gnss_quality_cache_.eph_m << ','
        << dtrq_gnss_quality_cache_.epv_m << ','
        << dtrq_gnss_quality_cache_.hacc_m << ','
        << dtrq_gnss_quality_cache_.vacc_m << ','
        << (dtrq_gnss_quality_cache_.vel_ned_valid ? 1 : 0) << ','
        << dtrq_gnss_quality_cache_.native_vel_n_mps << ','
        << dtrq_gnss_quality_cache_.native_vel_e_mps << ','
        << dtrq_gnss_quality_cache_.native_vel_d_mps << ','
        << gnss_enu.x() << ','
        << gnss_enu.y() << ','
        << gnss_enu.z() << ','
        << dtrq_gnss_dt_sec << ','
        << dtrq_gnss_dp_h_m << ','
        << dtrq_gnss_dp_u_m << ','
        << horizontal_speed_mps << ','
        << vertical_speed_mps << ','
        << last_imu_gyro_norm_deg_s_ << ','
        << last_mavros_heading_rate_deg_s_ << ','
        << (motion_ctx.turning_now ? 1 : 0) << ','
        << (motion_ctx.post_turn_context ? 1 : 0) << ','
        << dtrq_speed_change_abs_mps2 << '\n';

      if (finite(gnss_source_time_sec) && gnss_enu.allFinite()) {
        prev_dtrq_gnss_source_time_sec_ = gnss_source_time_sec;
        prev_dtrq_gnss_enu_ = gnss_enu;
      }
      if (finite(horizontal_speed_mps) && finite(log_time_sec)) {
        prev_dtrq_horizontal_speed_mps_ = horizontal_speed_mps;
        prev_dtrq_speed_update_time_sec_ = log_time_sec;
      }
      last_dtrq_runtime_feature_debug_log_update_time_sec_ = log_time_sec;
      last_logged_dtrq_runtime_feature_debug_sequence_ = event.sequence;
      ++dtrq_runtime_feature_debug_rows_since_flush_;
      if (dtrq_runtime_feature_debug_rows_since_flush_ >= dtrq_runtime_feature_debug_flush_interval_) {
        dtrq_runtime_feature_debug_csv_.flush();
        dtrq_runtime_feature_debug_rows_since_flush_ = 0;
      }
    }
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

  static double dtrqStdFromVariance_(double variance)
  {
    return std::isfinite(variance) && variance > 0.0
      ? std::sqrt(variance)
      : std::numeric_limits<double>::quiet_NaN();
  }

  void updateDtrqGnssQualityFromNavSatFix_(
    const sensor_msgs::msg::NavSatFix & msg,
    double source_time_sec,
    double rx_ros_time_sec,
    bool native_velocity_valid,
    double native_vN,
    double native_vE,
    double native_vD)
  {
    dtrq_gnss_quality_cache_.valid = true;
    dtrq_gnss_quality_cache_.source_kind = "navsatfix";
    dtrq_gnss_quality_cache_.source_time_sec = source_time_sec;
    dtrq_gnss_quality_cache_.rx_ros_time_sec = rx_ros_time_sec;
    dtrq_gnss_quality_cache_.fix_type = static_cast<double>(msg.status.status);
    dtrq_gnss_quality_cache_.satellites_used = std::numeric_limits<double>::quiet_NaN();
    dtrq_gnss_quality_cache_.hdop = std::numeric_limits<double>::quiet_NaN();
    dtrq_gnss_quality_cache_.vdop = std::numeric_limits<double>::quiet_NaN();
    dtrq_gnss_quality_cache_.eph_m = dtrqStdFromVariance_(msg.position_covariance[0]);
    dtrq_gnss_quality_cache_.epv_m = dtrqStdFromVariance_(msg.position_covariance[8]);
    dtrq_gnss_quality_cache_.hacc_m = dtrq_gnss_quality_cache_.eph_m;
    dtrq_gnss_quality_cache_.vacc_m = dtrq_gnss_quality_cache_.epv_m;
    dtrq_gnss_quality_cache_.vel_ned_valid =
      native_velocity_valid &&
      std::isfinite(native_vN) &&
      std::isfinite(native_vE) &&
      std::isfinite(native_vD);
    dtrq_gnss_quality_cache_.native_vel_n_mps =
      dtrq_gnss_quality_cache_.vel_ned_valid ? native_vN : std::numeric_limits<double>::quiet_NaN();
    dtrq_gnss_quality_cache_.native_vel_e_mps =
      dtrq_gnss_quality_cache_.vel_ned_valid ? native_vE : std::numeric_limits<double>::quiet_NaN();
    dtrq_gnss_quality_cache_.native_vel_d_mps =
      dtrq_gnss_quality_cache_.vel_ned_valid ? native_vD : std::numeric_limits<double>::quiet_NaN();
  }

  void updateDtrqGnssQualityFromSensorGps_(
    const px4_msgs::msg::SensorGps & msg,
    double source_time_sec,
    double rx_ros_time_sec)
  {
    dtrq_gnss_quality_cache_.valid = true;
    dtrq_gnss_quality_cache_.source_kind = "px4_sensor_gps";
    dtrq_gnss_quality_cache_.source_time_sec = source_time_sec;
    dtrq_gnss_quality_cache_.rx_ros_time_sec = rx_ros_time_sec;
    dtrq_gnss_quality_cache_.fix_type = static_cast<double>(msg.fix_type);
    dtrq_gnss_quality_cache_.satellites_used = static_cast<double>(msg.satellites_used);
    dtrq_gnss_quality_cache_.hdop =
      std::isfinite(msg.hdop) ? static_cast<double>(msg.hdop) : std::numeric_limits<double>::quiet_NaN();
    dtrq_gnss_quality_cache_.vdop =
      std::isfinite(msg.vdop) ? static_cast<double>(msg.vdop) : std::numeric_limits<double>::quiet_NaN();
    dtrq_gnss_quality_cache_.eph_m =
      std::isfinite(msg.eph) ? static_cast<double>(msg.eph) : std::numeric_limits<double>::quiet_NaN();
    dtrq_gnss_quality_cache_.epv_m =
      std::isfinite(msg.epv) ? static_cast<double>(msg.epv) : std::numeric_limits<double>::quiet_NaN();
    dtrq_gnss_quality_cache_.hacc_m = dtrq_gnss_quality_cache_.eph_m;
    dtrq_gnss_quality_cache_.vacc_m = dtrq_gnss_quality_cache_.epv_m;
    dtrq_gnss_quality_cache_.vel_ned_valid =
      msg.vel_ned_valid &&
      std::isfinite(msg.vel_n_m_s) &&
      std::isfinite(msg.vel_e_m_s) &&
      std::isfinite(msg.vel_d_m_s);
    dtrq_gnss_quality_cache_.native_vel_n_mps =
      dtrq_gnss_quality_cache_.vel_ned_valid
        ? static_cast<double>(msg.vel_n_m_s)
        : std::numeric_limits<double>::quiet_NaN();
    dtrq_gnss_quality_cache_.native_vel_e_mps =
      dtrq_gnss_quality_cache_.vel_ned_valid
        ? static_cast<double>(msg.vel_e_m_s)
        : std::numeric_limits<double>::quiet_NaN();
    dtrq_gnss_quality_cache_.native_vel_d_mps =
      dtrq_gnss_quality_cache_.vel_ned_valid
        ? static_cast<double>(msg.vel_d_m_s)
        : std::numeric_limits<double>::quiet_NaN();
  }

  void updateDtrqGnssQualityFromVehicleGlobalPosition_(
    const px4_msgs::msg::VehicleGlobalPosition & msg,
    double source_time_sec,
    double rx_ros_time_sec)
  {
    dtrq_gnss_quality_cache_.valid = true;
    dtrq_gnss_quality_cache_.source_kind = "px4_vehicle_global_position";
    dtrq_gnss_quality_cache_.source_time_sec = source_time_sec;
    dtrq_gnss_quality_cache_.rx_ros_time_sec = rx_ros_time_sec;
    dtrq_gnss_quality_cache_.fix_type = msg.dead_reckoning ? 0.0 : 3.0;
    dtrq_gnss_quality_cache_.satellites_used = std::numeric_limits<double>::quiet_NaN();
    dtrq_gnss_quality_cache_.hdop = std::numeric_limits<double>::quiet_NaN();
    dtrq_gnss_quality_cache_.vdop = std::numeric_limits<double>::quiet_NaN();
    dtrq_gnss_quality_cache_.eph_m =
      std::isfinite(msg.eph) ? static_cast<double>(msg.eph) : std::numeric_limits<double>::quiet_NaN();
    dtrq_gnss_quality_cache_.epv_m =
      std::isfinite(msg.epv) ? static_cast<double>(msg.epv) : std::numeric_limits<double>::quiet_NaN();
    dtrq_gnss_quality_cache_.hacc_m = dtrq_gnss_quality_cache_.eph_m;
    dtrq_gnss_quality_cache_.vacc_m = dtrq_gnss_quality_cache_.epv_m;
    dtrq_gnss_quality_cache_.vel_ned_valid = false;
    dtrq_gnss_quality_cache_.native_vel_n_mps = std::numeric_limits<double>::quiet_NaN();
    dtrq_gnss_quality_cache_.native_vel_e_mps = std::numeric_limits<double>::quiet_NaN();
    dtrq_gnss_quality_cache_.native_vel_d_mps = std::numeric_limits<double>::quiet_NaN();
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
  int    raw_odom_decimation_{50};
  bool   core_processing_enable_{true};
  int    core_imu_decimation_{1};
  double core_max_imu_rate_hz_{0.0};
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
  bool gnss_position_lag_compensation_enable_{false};
  bool gnss_position_lag_compensation_armed_only_{true};
  bool logged_gnss_position_lag_compensation_{false};
  double gnss_position_lag_compensation_sec_{0.25};
  double gnss_position_lag_compensation_max_sec_{0.50};
  double gnss_position_lag_compensation_min_speed_mps_{0.50};
  bool experiment_initial_reset_offset_enable_{false};
  bool experiment_initial_reset_offset_applied_{false};
  bool experiment_armed_reset_offset_enable_{false};
  bool experiment_armed_reset_offset_applied_{false};
  double experiment_initial_reset_offset_n_m_{0.0};
  double experiment_initial_reset_offset_e_m_{0.0};
  double experiment_initial_reset_offset_u_m_{0.0};
  double experiment_initial_reset_yaw_offset_deg_{0.0};

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
  std::string mavros_mode_;
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
  bool reported_core_processing_disabled_imu_{false};
  bool reported_core_processing_disabled_gnss_{false};
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
  DtrqGnssQualityCache dtrq_gnss_quality_cache_{};
  TurnRatePropagationNoiseDebug last_turn_rate_propagation_noise_debug_{};
  AccbiasZPropagationProbeDebug last_accbias_z_propagation_probe_debug_{};
  std::ofstream gnss_update_debug_csv_;
  std::ofstream gnss_nis_debug_csv_;
  std::ofstream horizontal_consistency_debug_csv_;
  std::ofstream heading_update_debug_csv_;
  std::ofstream state_publish_debug_csv_;
  std::ofstream segment_timing_gate_debug_csv_;
  std::ofstream state_update_debug_csv_;
  std::ofstream dtrq_runtime_feature_debug_csv_;
  std::ofstream shadow_restore_event_csv_;
  std::ofstream shadow_supervisor_fsm_debug_csv_;
  std::ofstream shadow_supervisor_fsm_events_csv_;
  std::ofstream shadow_supervisor_predictive_score_debug_csv_;
  std::ofstream shadow_supervisor_velocity_predictive_score_debug_csv_;
  std::ofstream shadow_supervisor_kinematic_predictive_score_debug_csv_;
  std::uint64_t last_logged_observation_debug_sequence_{0};
  std::uint64_t last_logged_gnss_nis_debug_sequence_{0};
  std::uint64_t last_logged_state_update_debug_sequence_{0};
  std::uint64_t last_logged_dtrq_runtime_feature_debug_sequence_{0};
  std::uint64_t heading_update_debug_sequence_{0};
  std::uint64_t state_publish_debug_sequence_{0};
  std::uint64_t segment_timing_gate_debug_sequence_{0};
  std::uint64_t shadow_restore_event_sequence_{0};
  std::uint64_t shadow_supervisor_fsm_debug_sequence_{0};
  std::uint64_t shadow_supervisor_fsm_event_sequence_{0};
  std::uint64_t shadow_supervisor_predictive_score_local_sequence_{0};
  std::uint64_t shadow_supervisor_predictive_score_debug_sequence_{0};
  std::uint64_t shadow_supervisor_velocity_predictive_score_local_sequence_{0};
  std::uint64_t shadow_supervisor_velocity_predictive_score_debug_sequence_{0};
  std::uint64_t shadow_supervisor_kinematic_predictive_score_local_sequence_{0};
  std::uint64_t shadow_supervisor_kinematic_predictive_score_debug_sequence_{0};
  std::size_t gnss_update_debug_rows_since_flush_{0};
  std::size_t gnss_update_debug_flush_interval_{20};
  std::size_t heading_update_debug_rows_since_flush_{0};
  std::size_t heading_update_debug_flush_interval_{1};
  std::size_t state_publish_debug_rows_since_flush_{0};
  std::size_t state_publish_debug_flush_interval_{50};
  std::size_t segment_timing_gate_debug_rows_since_flush_{0};
  std::size_t segment_timing_gate_debug_flush_interval_{1};
  std::size_t state_update_debug_rows_since_flush_{0};
  std::size_t state_update_debug_flush_interval_{10};
  std::size_t dtrq_runtime_feature_debug_rows_since_flush_{0};
  std::size_t dtrq_runtime_feature_debug_flush_interval_{10};
  std::size_t shadow_supervisor_fsm_debug_rows_since_flush_{0};
  std::size_t shadow_supervisor_fsm_debug_flush_interval_{20};
  std::size_t shadow_supervisor_predictive_score_debug_rows_since_flush_{0};
  std::size_t shadow_supervisor_predictive_score_debug_flush_interval_{20};
  std::size_t shadow_supervisor_velocity_predictive_score_debug_rows_since_flush_{0};
  std::size_t shadow_supervisor_velocity_predictive_score_debug_flush_interval_{20};
  std::size_t shadow_supervisor_kinematic_predictive_score_debug_rows_since_flush_{0};
  std::size_t shadow_supervisor_kinematic_predictive_score_debug_flush_interval_{20};
  double state_update_debug_max_rate_hz_{2.0};
  double last_state_update_debug_log_update_time_sec_{std::numeric_limits<double>::quiet_NaN()};
  double dtrq_runtime_feature_debug_max_rate_hz_{2.0};
  double last_dtrq_runtime_feature_debug_log_update_time_sec_{std::numeric_limits<double>::quiet_NaN()};
  double prev_dtrq_gnss_source_time_sec_{std::numeric_limits<double>::quiet_NaN()};
  Eigen::Vector3d prev_dtrq_gnss_enu_{Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())};
  double prev_dtrq_horizontal_speed_mps_{std::numeric_limits<double>::quiet_NaN()};
  double prev_dtrq_speed_update_time_sec_{std::numeric_limits<double>::quiet_NaN()};

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
  bool armed_cruise_gnss_pos_override_apply_mission_{true};
  bool armed_cruise_gnss_pos_override_apply_rtl_{true};
  bool armed_cruise_gnss_pos_override_apply_other_{true};
  double armed_cruise_gnss_pos_std_h_m_{0.06};
  double armed_cruise_gnss_pos_std_u_m_{0.08};
  bool armed_cruise_gnss_pos_residual_boost_enable_{false};
  double armed_cruise_gnss_pos_residual_boost_threshold_m_{0.12};
  double armed_cruise_gnss_pos_residual_boost_hold_sec_{20.0};
  bool gnss_position_response_boost_enable_{false};
  bool gnss_position_response_boost_apply_mission_{true};
  bool gnss_position_response_boost_apply_rtl_{false};
  double gnss_position_response_boost_residual_start_h_m_{0.25};
  double gnss_position_response_boost_residual_full_h_m_{0.45};
  double gnss_position_response_boost_min_horizontal_speed_mps_{0.5};
  double gnss_position_response_boost_max_abs_vertical_speed_mps_{1.0};
  int gnss_position_response_boost_persistence_updates_{1};
  double gnss_position_response_boost_std_min_h_m_{0.04};
  double gnss_position_response_boost_std_max_h_m_{0.06};
  int gnss_position_response_boost_persistent_count_{0};
  bool gnss_position_gain_response_enable_{false};
  bool gnss_position_gain_response_apply_mission_{true};
  bool gnss_position_gain_response_apply_rtl_{false};
  bool gnss_position_gain_response_require_armed_cruise_{true};
  bool gnss_position_gain_response_block_turning_{true};
  bool gnss_position_gain_response_block_post_turn_{true};
  double gnss_position_gain_response_residual_start_h_m_{0.15};
  double gnss_position_gain_response_residual_full_h_m_{0.35};
  double gnss_position_gain_response_hnis_start_{6.0};
  double gnss_position_gain_response_hnis_full_{20.0};
  double gnss_position_gain_response_prev_gain_low_{0.10};
  double gnss_position_gain_response_prev_gain_high_{0.18};
  double gnss_position_gain_response_min_horizontal_speed_mps_{3.0};
  double gnss_position_gain_response_max_abs_vertical_speed_mps_{1.0};
  int gnss_position_gain_response_persistence_updates_{1};
  double gnss_position_gain_response_std_min_h_m_{0.03};
  double gnss_position_gain_response_std_max_h_m_{0.06};
  int gnss_position_gain_response_persistent_count_{0};
  bool gnss_velocity_outward_damping_enable_{false};
  bool gnss_velocity_outward_damping_apply_mission_{true};
  bool gnss_velocity_outward_damping_apply_rtl_{false};
  double gnss_velocity_outward_damping_min_core_gnss_diff_h_m_{0.20};
  double gnss_velocity_outward_damping_radial_start_mps_{0.03};
  double gnss_velocity_outward_damping_radial_full_mps_{0.12};
  double gnss_velocity_outward_damping_min_horizontal_speed_mps_{0.5};
  double gnss_velocity_outward_damping_max_abs_vertical_speed_mps_{1.0};
  int gnss_velocity_outward_damping_persistence_updates_{1};
  int gnss_velocity_outward_damping_hold_updates_{0};
  double gnss_velocity_outward_damping_std_min_h_mps_{0.02};
  double gnss_velocity_outward_damping_std_max_h_mps_{0.05};
  int gnss_velocity_outward_damping_persistent_count_{0};
  int gnss_velocity_outward_damping_hold_remaining_updates_{0};
  double gnss_velocity_outward_damping_hold_score_{0.0};
  bool turn_postturn_native_velocity_deweight_enable_{false};
  bool turn_postturn_native_velocity_deweight_apply_mission_{true};
  bool turn_postturn_native_velocity_deweight_apply_rtl_{false};
  bool turn_postturn_native_velocity_deweight_apply_turning_{true};
  bool turn_postturn_native_velocity_deweight_apply_post_turn_{true};
  double turn_postturn_native_velocity_deweight_min_core_gnss_diff_h_m_{0.15};
  double turn_postturn_native_velocity_deweight_radial_abs_start_mps_{0.08};
  double turn_postturn_native_velocity_deweight_radial_abs_full_mps_{0.28};
  double turn_postturn_native_velocity_deweight_core_residual_start_h_mps_{0.08};
  double turn_postturn_native_velocity_deweight_core_residual_full_h_mps_{0.30};
  double turn_postturn_native_velocity_deweight_min_horizontal_speed_mps_{0.5};
  double turn_postturn_native_velocity_deweight_max_abs_vertical_speed_mps_{1.0};
  int turn_postturn_native_velocity_deweight_persistence_updates_{1};
  double turn_postturn_native_velocity_deweight_std_min_h_mps_{0.10};
  double turn_postturn_native_velocity_deweight_std_max_h_mps_{0.18};
  int turn_postturn_native_velocity_deweight_persistent_count_{0};
  bool phase_error_memory_debug_enable_{false};
  double phase_error_memory_debug_residual_threshold_h_m_{0.16};
  double phase_error_memory_debug_dx_over_residual_threshold_{0.22};
  double phase_error_memory_debug_recent_turnpost_hold_sec_{15.0};
  bool adaptive_gnss_pos_weight_enable_{false};
  bool adaptive_gnss_pos_weight_apply_mission_{true};
  bool adaptive_gnss_pos_weight_apply_rtl_{false};
  std::string adaptive_gnss_pos_weight_trigger_source_{"residual_h"};
  double adaptive_gnss_pos_weight_floor_min_h_m_{0.08};
  double adaptive_gnss_pos_weight_floor_nominal_h_m_{0.12};
  double adaptive_gnss_pos_weight_floor_max_h_m_{0.16};
  double adaptive_gnss_pos_weight_residual_start_h_m_{0.24};
  double adaptive_gnss_pos_weight_residual_full_h_m_{0.45};
  double adaptive_gnss_pos_weight_nis_start_h_2d_{5.991};
  double adaptive_gnss_pos_weight_nis_full_h_2d_{20.0};
  double adaptive_gnss_pos_weight_nis_max_age_sec_{2.0};
  int adaptive_gnss_pos_weight_persistence_updates_{5};
  double adaptive_gnss_pos_weight_attack_sec_{0.8};
  double adaptive_gnss_pos_weight_decay_sec_{1.2};
  double adaptive_gnss_pos_weight_armed_cruise_gain_{1.0};
  double adaptive_gnss_pos_weight_turn_gain_{1.15};
  double adaptive_gnss_pos_weight_post_turn_gain_{1.15};
  double adaptive_gnss_pos_weight_floor_state_h_m_{0.0};
  int adaptive_gnss_pos_weight_persistent_count_{0};
  double adaptive_gnss_pos_weight_last_update_sec_{std::numeric_limits<double>::quiet_NaN()};
  double adaptive_gnss_pos_weight_last_nis_h_2d_{std::numeric_limits<double>::quiet_NaN()};
  double adaptive_gnss_pos_weight_last_nis_ros_time_sec_{std::numeric_limits<double>::quiet_NaN()};
  std::uint64_t adaptive_gnss_pos_weight_last_nis_sequence_{0};
  double armed_cruise_native_gnss_vel_min_horizontal_speed_mps_{0.5};
  double armed_cruise_native_gnss_vel_std_h_mps_{0.05};
  double armed_cruise_native_gnss_vel_std_u_mps_{0.10};
  bool armed_cruise_native_gnss_vel_residual_boost_enable_{false};
  double armed_cruise_native_gnss_vel_residual_boost_threshold_mps_{0.10};
  double armed_cruise_native_gnss_vel_residual_boost_hold_sec_{8.0};
  double armed_cruise_native_gnss_vel_residual_boost_std_h_mps_{0.03};
  double armed_cruise_native_gnss_vel_residual_boost_std_u_mps_{0.08};
  bool native_gnss_velocity_outlier_guard_enable_{false};
  bool native_gnss_velocity_outlier_guard_apply_mission_{true};
  bool native_gnss_velocity_outlier_guard_apply_rtl_{false};
  double native_gnss_velocity_outlier_guard_speed_mismatch_mps_{0.60};
  double native_gnss_velocity_outlier_guard_core_residual_h_mps_{0.80};
  double native_gnss_velocity_outlier_guard_min_horizontal_speed_mps_{1.0};
  double native_gnss_velocity_outlier_guard_max_abs_vertical_speed_mps_{0.5};
  bool native_gnss_velocity_outlier_guard_apply_turning_context_{true};
  std::string native_gnss_velocity_outlier_guard_action_{"skip"};
  double native_gnss_velocity_outlier_guard_reweight_std_h_mps_{0.12};
  double native_gnss_velocity_outlier_guard_reweight_std_u_mps_{0.20};
  bool native_gnss_velocity_low_speed_turn_source_guard_enable_{false};
  double native_gnss_velocity_low_speed_turn_source_guard_max_horizontal_speed_mps_{2.2};
  double native_gnss_velocity_low_speed_turn_source_guard_min_gyro_deg_s_{15.0};
  double native_gnss_velocity_low_speed_turn_source_guard_core_residual_h_mps_{1.0};
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
  bool terminal_descent_observation_enable_{false};
  bool terminal_descent_require_rtl_mode_{true};
  double terminal_descent_max_horizontal_speed_mps_{1.6};
  double terminal_descent_min_vertical_speed_mps_{0.30};
  double terminal_descent_max_gyro_deg_s_{30.0};
  double terminal_descent_max_source_yaw_rate_deg_s_{30.0};
  double terminal_descent_min_armed_time_sec_{0.0};
  bool terminal_descent_native_gnss_vel_override_enable_{true};
  double terminal_descent_native_gnss_vel_std_h_mps_{0.03};
  double terminal_descent_native_gnss_vel_std_u_mps_{0.05};
  bool terminal_descent_vertical_cov_reopen_enable_{true};
  double terminal_descent_vertical_cov_reopen_pos_std_m_{0.15};
  double terminal_descent_vertical_cov_reopen_vel_std_mps_{0.05};
  double terminal_descent_vertical_cov_reopen_accbias_std_z_mps2_{0.05};
  bool terminal_descent_horizontal_zero_vel_enable_{false};
  double terminal_descent_horizontal_zero_vel_max_hspeed_mps_{0.30};
  double terminal_descent_horizontal_zero_vel_std_h_mps_{0.05};
  double terminal_descent_horizontal_zero_vel_std_u_mps_{10.0};
  double armed_cruise_gnss_pos_residual_boost_until_sec_{std::numeric_limits<double>::quiet_NaN()};
  double armed_cruise_native_gnss_vel_residual_boost_until_sec_{std::numeric_limits<double>::quiet_NaN()};
  double armed_cruise_vertical_cov_reopen_until_sec_{std::numeric_limits<double>::quiet_NaN()};
  double post_flight_vertical_cov_reopen_until_sec_{std::numeric_limits<double>::quiet_NaN()};
  std::string gnss_update_debug_csv_path_;
  std::string gnss_nis_debug_csv_path_;
  double gnss_nis_debug_max_rate_hz_{2.0};
  double last_gnss_nis_debug_log_update_time_sec_{std::numeric_limits<double>::quiet_NaN()};
  std::string horizontal_consistency_debug_csv_path_;
  bool horizontal_consistency_supervisor_enable_{false};
  bool horizontal_consistency_apply_mission_{true};
  bool horizontal_consistency_apply_rtl_{false};
  double horizontal_consistency_nis_start_h_2d_{5.991};
  double horizontal_consistency_nis_full_h_2d_{20.0};
  double horizontal_consistency_residual_start_h_m_{0.20};
  double horizontal_consistency_residual_full_h_m_{0.45};
  double horizontal_consistency_core_gnss_start_h_m_{0.20};
  double horizontal_consistency_core_gnss_full_h_m_{0.45};
  double horizontal_consistency_heading_residual_start_deg_{0.8};
  double horizontal_consistency_heading_residual_full_deg_{2.0};
  double horizontal_consistency_score_trigger_{0.75};
  int horizontal_consistency_persistence_updates_{3};
  double horizontal_consistency_min_horizontal_speed_mps_{3.0};
  double horizontal_consistency_max_vertical_speed_mps_{0.8};
  int horizontal_consistency_persistent_count_{0};
  bool mission_cov_hygiene_enable_{false};
  bool mission_cov_hygiene_apply_mission_{true};
  bool mission_cov_hygiene_apply_rtl_{false};
  double mission_cov_hygiene_hnis_start_{6.0};
  double mission_cov_hygiene_hnis_full_{20.0};
  double mission_cov_hygiene_resid_start_h_m_{0.15};
  double mission_cov_hygiene_resid_full_h_m_{0.35};
  double mission_cov_hygiene_pos_std_tight_lo_h_m_{0.025};
  double mission_cov_hygiene_pos_std_tight_hi_h_m_{0.050};
  double mission_cov_hygiene_dx_ratio_low_{0.15};
  double mission_cov_hygiene_dx_ratio_high_{0.35};
  int mission_cov_hygiene_persistence_updates_{3};
  double mission_cov_hygiene_attack_sec_{4.0};
  double mission_cov_hygiene_decay_sec_{15.0};
  double mission_cov_hygiene_floor_min_h_m_{0.06};
  double mission_cov_hygiene_floor_nominal_h_m_{0.08};
  double mission_cov_hygiene_floor_max_h_m_{0.12};
  double mission_cov_hygiene_offdiag_corr_limit_{0.70};
  int mission_cov_hygiene_persistent_count_{0};
  double mission_cov_hygiene_floor_state_h_m_{0.0};
  double mission_cov_hygiene_last_update_sec_{std::numeric_limits<double>::quiet_NaN()};
  std::uint64_t mission_cov_hygiene_last_state_update_sequence_{0};
  double mission_cov_hygiene_prev_dx_pos_h_m_{std::numeric_limits<double>::quiet_NaN()};
  double mission_cov_hygiene_prev_residual_h_m_{std::numeric_limits<double>::quiet_NaN()};
  bool turn_rate_propagation_noise_probe_enable_{false};
  bool turn_rate_propagation_noise_probe_apply_mission_{true};
  bool turn_rate_propagation_noise_probe_apply_rtl_{false};
  double turn_rate_propagation_noise_probe_gyro_start_deg_s_{6.0};
  double turn_rate_propagation_noise_probe_gyro_full_deg_s_{20.0};
  double turn_rate_propagation_noise_probe_min_horizontal_speed_mps_{0.5};
  double turn_rate_propagation_noise_probe_max_abs_vertical_speed_mps_{1.0};
  double turn_rate_propagation_noise_probe_arw_q_scale_max_{9.0};
  double turn_rate_propagation_noise_probe_vrw_q_scale_max_{4.0};
  double turn_rate_propagation_noise_probe_gyrbias_q_scale_max_{4.0};
  double turn_rate_propagation_noise_probe_accbias_q_scale_max_{2.0};
  bool accbias_z_propagation_probe_enable_{false};
  bool accbias_z_propagation_probe_apply_noise_scale_{false};
  bool accbias_z_propagation_probe_apply_mission_{true};
  bool accbias_z_propagation_probe_apply_rtl_{false};
  double accbias_z_propagation_probe_trigger_mps2_{-0.18};
  double accbias_z_propagation_probe_full_mps2_{-0.24};
  double accbias_z_propagation_probe_min_horizontal_speed_mps_{3.0};
  double accbias_z_propagation_probe_max_abs_vertical_speed_mps_{1.0};
  double accbias_z_propagation_probe_arw_q_scale_max_{1.0};
  double accbias_z_propagation_probe_vrw_q_scale_max_{3.0};
  double accbias_z_propagation_probe_gyrbias_q_scale_max_{1.0};
  double accbias_z_propagation_probe_accbias_q_scale_max_{6.0};
  bool adaptive_rq_source_gate_enable_{true};
  double adaptive_rq_source_gate_low_speed_max_mps_{3.5};
  bool motion_gnss_pos_weight_enable_{false};
  bool motion_gnss_pos_weight_apply_mission_{true};
  bool motion_gnss_pos_weight_apply_rtl_{false};
  double motion_gnss_pos_weight_hspeed_start_mps_{3.5};
  double motion_gnss_pos_weight_hspeed_full_mps_{5.0};
  double motion_gnss_pos_weight_max_abs_vspeed_mps_{0.35};
  double motion_gnss_pos_weight_std_min_h_m_{0.08};
  double motion_gnss_pos_weight_std_max_h_m_{0.14};
  bool gnss_pos_recovery_weight_enable_{false};
  bool gnss_pos_recovery_weight_apply_mission_{true};
  bool gnss_pos_recovery_weight_apply_rtl_{false};
  double gnss_pos_recovery_weight_residual_start_h_m_{0.32};
  double gnss_pos_recovery_weight_residual_full_h_m_{0.45};
  double gnss_pos_recovery_weight_core_start_h_m_{0.30};
  double gnss_pos_recovery_weight_core_full_h_m_{0.55};
  double gnss_pos_recovery_weight_core_score_gain_{0.25};
  double gnss_pos_recovery_weight_min_horizontal_speed_mps_{3.0};
  double gnss_pos_recovery_weight_max_abs_vertical_speed_mps_{0.35};
  int gnss_pos_recovery_weight_persistence_updates_{3};
  double gnss_pos_recovery_weight_hold_sec_{3.0};
  double gnss_pos_recovery_weight_attack_sec_{0.5};
  double gnss_pos_recovery_weight_decay_sec_{2.0};
  double gnss_pos_recovery_weight_std_min_h_m_{0.12};
  double gnss_pos_recovery_weight_std_max_h_m_{0.16};
  int gnss_pos_recovery_weight_persistent_count_{0};
  double gnss_pos_recovery_weight_hold_until_sec_{std::numeric_limits<double>::quiet_NaN()};
  double gnss_pos_recovery_weight_last_update_sec_{std::numeric_limits<double>::quiet_NaN()};
  double gnss_pos_recovery_weight_state_std_h_m_{0.0};
  bool context_gnss_pos_floor_enable_{false};
  bool context_gnss_pos_floor_apply_mission_{true};
  bool context_gnss_pos_floor_apply_rtl_{false};
  double context_gnss_pos_floor_mission_base_h_m_{0.08};
  double context_gnss_pos_floor_turn_post_h_m_{0.10};
  double context_gnss_pos_floor_armed_cruise_h_m_{0.09};
  double context_gnss_pos_floor_rtl_h_m_{0.08};
  double context_gnss_pos_floor_attack_sec_{0.5};
  double context_gnss_pos_floor_decay_sec_{1.5};
  double context_gnss_pos_floor_last_update_sec_{std::numeric_limits<double>::quiet_NaN()};
  double context_gnss_pos_floor_state_h_m_{0.0};
  std::string heading_update_debug_csv_path_;
  std::string state_publish_debug_csv_path_;
  bool shadow_restore_publish_enable_{false};
  bool shadow_restore_subscribe_enable_{false};
  std::string shadow_restore_topic_{"/kf_gins/main_to_shadow_snapshot"};
  double shadow_restore_publish_after_core_sec_{40.0};
  bool shadow_restore_publish_once_{true};
  double shadow_restore_publish_period_sec_{0.0};
  double shadow_restore_max_age_sec_{0.75};
  double shadow_restore_covariance_inflation_factor_{1.0};
  bool shadow_restore_require_core_initialized_{true};
  bool shadow_restore_clear_path_on_apply_{true};
  std::string shadow_restore_event_csv_path_;
  bool shadow_restore_publish_done_{false};
  double shadow_restore_last_publish_core_time_sec_{
    std::numeric_limits<double>::quiet_NaN()};
  bool shadow_supervisor_fsm_debug_enable_{false};
  std::string shadow_supervisor_fsm_reference_odom_topic_{"/kf_gins/odom"};
  std::string shadow_supervisor_fsm_debug_csv_path_;
  std::string shadow_supervisor_fsm_events_csv_path_;
  double shadow_supervisor_fsm_debug_max_rate_hz_{10.0};
  double shadow_supervisor_fsm_reference_odom_max_age_sec_{0.5};
  double shadow_supervisor_fsm_gamma_guard_enter_{1.05};
  double shadow_supervisor_fsm_gamma_shadow_max_{1.05};
  double shadow_supervisor_fsm_process_lambda_enter_{1.012};
  double shadow_supervisor_fsm_process_score_min_{0.30};
  double shadow_supervisor_fsm_warmup_sec_{5.0};
  double shadow_supervisor_fsm_ready_confirm_sec_{2.0};
  double shadow_supervisor_fsm_xy_delta_ready_max_m_{0.50};
  double shadow_supervisor_fsm_z_delta_ready_max_m_{0.50};
  double shadow_supervisor_fsm_vel_delta_ready_max_mps_{0.30};
  double shadow_supervisor_fsm_yaw_delta_ready_max_deg_{5.0};
  bool shadow_supervisor_fsm_allow_mixed_trigger_{false};
  bool shadow_supervisor_fsm_observation_score_guard_enable_{false};
  double shadow_supervisor_fsm_observation_score_guard_enter_{0.50};
  bool shadow_supervisor_perf_proxy_publish_enable_{false};
  bool shadow_supervisor_perf_proxy_subscribe_enable_{false};
  std::string shadow_supervisor_perf_proxy_topic_{"/kf_gins/supervisor_performance_proxy"};
  double shadow_supervisor_perf_proxy_max_age_sec_{0.75};
  double shadow_supervisor_perf_proxy_short_window_sec_{5.0};
  double shadow_supervisor_perf_proxy_long_window_sec_{10.0};
  double shadow_supervisor_perf_proxy_residual_gap_soft_max_m_{0.0};
  double shadow_supervisor_perf_proxy_nis_gap_permission_max_{0.0};
  bool shadow_supervisor_predictive_score_publish_enable_{false};
  bool shadow_supervisor_predictive_score_subscribe_enable_{false};
  bool shadow_supervisor_predictive_score_debug_enable_{false};
  std::string shadow_supervisor_predictive_score_topic_{
    "/kf_gins/supervisor_predictive_score"};
  std::string shadow_supervisor_predictive_score_debug_csv_path_;
  std::string shadow_supervisor_predictive_score_source_id_{"rq_shadow"};
  double shadow_supervisor_predictive_score_max_age_sec_{0.75};
  double shadow_supervisor_predictive_score_stamp_tolerance_sec_{0.20};
  double shadow_supervisor_predictive_score_short_window_sec_{5.0};
  double shadow_supervisor_predictive_score_long_window_sec_{10.0};
  double shadow_supervisor_predictive_score_eval_std_h_m_{0.10};
  double shadow_supervisor_predictive_score_eval_std_u_m_{0.20};
  bool shadow_supervisor_velocity_predictive_score_publish_enable_{false};
  bool shadow_supervisor_velocity_predictive_score_subscribe_enable_{false};
  bool shadow_supervisor_velocity_predictive_score_debug_enable_{false};
  std::string shadow_supervisor_velocity_predictive_score_topic_{
    "/kf_gins/supervisor_velocity_predictive_score"};
  std::string shadow_supervisor_velocity_predictive_score_debug_csv_path_;
  std::string shadow_supervisor_velocity_predictive_score_source_id_{"rq_shadow"};
  double shadow_supervisor_velocity_predictive_score_max_age_sec_{0.75};
  double shadow_supervisor_velocity_predictive_score_stamp_tolerance_sec_{0.20};
  double shadow_supervisor_velocity_predictive_score_short_window_sec_{5.0};
  double shadow_supervisor_velocity_predictive_score_long_window_sec_{10.0};
  double shadow_supervisor_velocity_predictive_score_eval_std_h_mps_{0.20};
  double shadow_supervisor_velocity_predictive_score_eval_std_u_mps_{0.30};
  bool shadow_supervisor_kinematic_predictive_score_publish_enable_{false};
  bool shadow_supervisor_kinematic_predictive_score_subscribe_enable_{false};
  bool shadow_supervisor_kinematic_predictive_score_debug_enable_{false};
  std::string shadow_supervisor_kinematic_predictive_score_topic_{
    "/kf_gins/supervisor_kinematic_predictive_score"};
  std::string shadow_supervisor_kinematic_predictive_score_debug_csv_path_;
  std::string shadow_supervisor_kinematic_predictive_score_source_id_{"rq_shadow"};
  double shadow_supervisor_kinematic_predictive_score_max_age_sec_{0.75};
  double shadow_supervisor_kinematic_predictive_score_stamp_tolerance_sec_{0.20};
  double shadow_supervisor_kinematic_predictive_score_short_window_sec_{5.0};
  double shadow_supervisor_kinematic_predictive_score_long_window_sec_{10.0};
  double shadow_supervisor_kinematic_predictive_score_eval_std_h_m_{0.10};
  double shadow_supervisor_kinematic_predictive_score_eval_std_u_m_{0.20};
  double shadow_supervisor_kinematic_predictive_score_eval_std_h_mps_{0.20};
  double shadow_supervisor_kinematic_predictive_score_eval_std_u_mps_{0.30};
  ShadowSupervisorFsmState shadow_supervisor_fsm_state_{
    ShadowSupervisorFsmState::MainSafe};
  std::string shadow_supervisor_fsm_last_transition_reason_{"init"};
  ShadowSupervisorReferenceOdom shadow_supervisor_reference_odom_{};
  ShadowSupervisorPerformanceProxySample shadow_supervisor_latest_perf_proxy_{};
  std::deque<ShadowSupervisorPerformanceProxyGapSample>
    shadow_supervisor_perf_proxy_gap_history_;
  ShadowSupervisorPredictiveScoreSample shadow_supervisor_latest_predictive_score_{};
  std::deque<ShadowSupervisorPredictiveScoreSample>
    shadow_supervisor_predictive_score_reference_history_;
  std::deque<ShadowSupervisorPredictiveScoreSample>
    shadow_supervisor_predictive_score_pending_local_samples_;
  std::deque<ShadowSupervisorPredictiveScoreGapSample>
    shadow_supervisor_predictive_score_gap_history_;
  ShadowSupervisorVelocityPredictiveScoreSample
    shadow_supervisor_latest_velocity_predictive_score_{};
  std::deque<ShadowSupervisorVelocityPredictiveScoreSample>
    shadow_supervisor_velocity_predictive_score_reference_history_;
  std::deque<ShadowSupervisorVelocityPredictiveScoreSample>
    shadow_supervisor_velocity_predictive_score_pending_local_samples_;
  std::deque<ShadowSupervisorVelocityPredictiveScoreGapSample>
    shadow_supervisor_velocity_predictive_score_gap_history_;
  ShadowSupervisorKinematicPredictiveScoreSample
    shadow_supervisor_latest_kinematic_predictive_score_{};
  std::deque<ShadowSupervisorKinematicPredictiveScoreSample>
    shadow_supervisor_kinematic_predictive_score_reference_history_;
  std::deque<ShadowSupervisorKinematicPredictiveScoreSample>
    shadow_supervisor_kinematic_predictive_score_pending_local_samples_;
  std::deque<ShadowSupervisorKinematicPredictiveScoreSample>
    shadow_supervisor_kinematic_predictive_score_local_history_;
  std::deque<ShadowSupervisorKinematicPredictiveScoreGapSample>
    shadow_supervisor_kinematic_predictive_score_gap_history_;
  bool shadow_supervisor_restore_applied_pending_{false};
  bool shadow_supervisor_have_restore_{false};
  double shadow_supervisor_last_restore_core_time_sec_{
    std::numeric_limits<double>::quiet_NaN()};
  double shadow_supervisor_ready_candidate_since_core_time_sec_{
    std::numeric_limits<double>::quiet_NaN()};
  double shadow_supervisor_fsm_last_debug_log_ros_time_sec_{
    std::numeric_limits<double>::quiet_NaN()};
  std::string segment_timing_gate_debug_csv_path_;
  std::string state_update_debug_csv_path_;
  std::string dtrq_runtime_feature_debug_csv_path_;
  bool early_recovery_bias_feedback_debug_enable_{false};
  bool early_recovery_bias_feedback_apply_enable_{false};
  double early_recovery_bias_feedback_history_sec_{10.0};
  double early_recovery_bias_feedback_min_armed_time_sec_{35.0};
  double early_recovery_bias_feedback_max_armed_time_sec_{95.0};
  double early_recovery_bias_feedback_ba_z_mean_max_mps2_{-0.18};
  double early_recovery_bias_feedback_residual_u_mean_max_m_{-0.02};
  double early_recovery_bias_feedback_core_gnss_u_mean_min_m_{0.02};
  double early_recovery_bias_feedback_dx_ba_z_sum_max_mps2_{0.0};
  int early_recovery_bias_feedback_min_history_rows_{5};
  double early_recovery_bias_feedback_negative_dx_scale_{0.0};
  std::deque<EarlyRecoveryBiasFeedbackSample> early_recovery_bias_feedback_history_;
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
  bool heading_track_validity_gate_enable_{false};
  std::string heading_track_validity_gate_action_{"inflate"};
  bool heading_track_validity_gate_apply_to_update_{true};
  bool heading_track_validity_gate_apply_to_turn_track_{false};
  bool heading_track_validity_gate_apply_to_post_turn_{true};
  double heading_track_validity_gate_after_turn_sec_{20.0};
  double heading_track_validity_gate_min_horizontal_speed_mps_{3.0};
  double heading_track_validity_gate_max_vertical_speed_mps_{1.2};
  double heading_track_validity_gate_max_gyro_deg_s_{6.0};
  double heading_track_validity_gate_max_source_yaw_rate_deg_s_{1.0};
  double heading_track_validity_gate_max_residual_deg_{2.0};
  double heading_track_validity_gate_inflated_std_deg_{25.0};
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
  bool heading_cruise_micro_track_enable_{false};
  double heading_cruise_micro_track_std_deg_{2.0};
  double heading_cruise_micro_track_min_residual_deg_{0.15};
  double heading_cruise_micro_track_max_residual_deg_{1.0};
  double heading_cruise_micro_track_min_horizontal_speed_mps_{4.5};
  double heading_cruise_micro_track_max_vertical_speed_mps_{0.35};
  double heading_cruise_micro_track_max_gyro_deg_s_{3.5};
  double heading_cruise_micro_track_max_source_yaw_rate_deg_s_{1.0};
  bool heading_yaw_gain_hygiene_enable_{false};
  bool heading_yaw_gain_hygiene_apply_mission_{true};
  bool heading_yaw_gain_hygiene_apply_rtl_{false};
  bool heading_yaw_gain_hygiene_apply_armed_cruise_{true};
  bool heading_yaw_gain_hygiene_apply_post_turn_{true};
  bool heading_yaw_gain_hygiene_apply_turn_{false};
  bool heading_yaw_gain_hygiene_apply_update_{false};
  double heading_yaw_gain_hygiene_yaw_std_floor_deg_{1.0};
  double heading_yaw_gain_hygiene_min_residual_deg_{0.5};
  double heading_yaw_gain_hygiene_max_residual_deg_{2.0};
  double heading_yaw_gain_hygiene_min_horizontal_speed_mps_{3.0};
  double heading_yaw_gain_hygiene_max_abs_vertical_speed_mps_{0.5};
  double heading_yaw_gain_hygiene_max_gyro_deg_s_{8.0};
  double heading_yaw_gain_hygiene_max_source_yaw_rate_deg_s_{3.0};
  double heading_yaw_gain_hygiene_max_rate_hz_{5.0};
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
  bool publish_state_after_gnss_update_{true};
  bool publish_px4_sphere_projection_{false};
  bool accbias_z_history_projection_enable_{false};
  bool accbias_z_history_projection_apply_mission_{true};
  bool accbias_z_history_projection_apply_rtl_{false};
  bool accbias_z_history_projection_apply_other_{false};
  double accbias_z_history_projection_alpha_{1.2};
  double accbias_z_history_projection_deep_threshold_mps2_{-0.205};
  double accbias_z_history_projection_frac_threshold_{0.4};
  double accbias_z_history_projection_history_start_sec_{40.0};
  int accbias_z_history_projection_total_count_{0};
  int accbias_z_history_projection_deep_count_{0};
  std::uint64_t accbias_z_history_projection_last_sequence_{0};
  double accbias_z_history_projection_latest_before_mps2_{
    std::numeric_limits<double>::quiet_NaN()};
  double accbias_z_history_projection_latest_after_mps2_{
    std::numeric_limits<double>::quiet_NaN()};
  bool segment_timing_gate_projection_enable_{false};
  bool segment_timing_gate_projection_apply_mission_{true};
  bool segment_timing_gate_projection_apply_rtl_{false};
  bool segment_timing_gate_projection_apply_other_{false};
  bool segment_timing_gate_core_gnss_along_min_enable_{false};
  double segment_timing_gate_segment_sec_{2.0};
  double segment_timing_gate_lag_threshold_sec_{0.025};
  double segment_timing_gate_gnss_source_age_max_sec_{-1.0};
  double segment_timing_gate_core_gnss_along_min_threshold_m_{0.0};
  double segment_timing_gate_projection_alpha_{1.0};
  int segment_timing_gate_current_index_{-1};
  std::size_t segment_timing_gate_row_count_{0};
  double segment_timing_gate_stamp_lag_sum_sec_{0.0};
  double segment_timing_gate_stamp_lag_min_sec_{std::numeric_limits<double>::infinity()};
  double segment_timing_gate_stamp_lag_max_sec_{-std::numeric_limits<double>::infinity()};
  double segment_timing_gate_turning_sum_{0.0};
  double segment_timing_gate_post_turn_sum_{0.0};
  double segment_timing_gate_armed_cruise_sum_{0.0};
  double segment_timing_gate_hspeed_sum_mps_{0.0};
  double segment_timing_gate_vspeed_sum_mps_{0.0};
  double segment_timing_gate_core_age_sum_sec_{0.0};
  double segment_timing_gate_gnss_source_age_sum_sec_{0.0};
  double segment_timing_gate_core_gnss_along_min_m_{std::numeric_limits<double>::infinity()};
  std::size_t segment_timing_gate_hspeed_count_{0};
  std::size_t segment_timing_gate_vspeed_count_{0};
  std::size_t segment_timing_gate_core_age_count_{0};
  std::size_t segment_timing_gate_gnss_source_age_count_{0};
  std::size_t segment_timing_gate_core_gnss_along_count_{0};
  bool segment_timing_gate_armed_last_{false};
  double segment_timing_gate_active_duration_sec_{0.0};
  bool segment_timing_gate_current_active_{false};
  double segment_timing_gate_current_lag_mean_sec_{std::numeric_limits<double>::quiet_NaN()};
  double segment_timing_gate_current_lag_latest_sec_{std::numeric_limits<double>::quiet_NaN()};
  double segment_timing_gate_current_gnss_source_age_mean_sec_{
    std::numeric_limits<double>::quiet_NaN()};
  double segment_timing_gate_current_core_gnss_along_min_m_{
    std::numeric_limits<double>::quiet_NaN()};
  bool last_segment_timing_gate_projection_active_{false};
  bool last_accbias_z_history_projection_active_{false};
  bool last_accbias_z_history_projection_phase_allowed_{false};
  bool last_accbias_z_history_projection_have_history_{false};
  bool last_publish_projection_active_{false};
  std::string last_publish_projection_action_{"raw"};
  std::string segment_timing_gate_mode_last_;
  std::string publish_stamp_mode_{"ros_now"};
  double publish_core_stamp_max_future_sec_{0.08};
  double publish_core_stamp_max_past_sec_{0.25};
  double publish_core_stamp_offset_bias_sec_{0.0};
  bool publish_core_stamp_offset_initialized_{false};
  double publish_core_to_ros_offset_sec_{std::numeric_limits<double>::quiet_NaN()};
  double last_publish_stamp_observed_offset_sec_{std::numeric_limits<double>::quiet_NaN()};
  double last_publish_stamp_selected_minus_now_sec_{0.0};
  std::string last_publish_stamp_applied_mode_{"ros_now"};
  double preserved_core_yaw_max_mavros_diff_deg_{15.0};
  double preserved_core_yaw_max_core_gnss_diff_m_{5.0};
  double last_trusted_core_yaw_deg_{std::numeric_limits<double>::quiet_NaN()};
  double last_core_gnss_diff_m_{std::numeric_limits<double>::quiet_NaN()};
  bool prefer_preserved_yaw_on_next_core_reset_{false};
  bool last_gnss_has_fix_{true};
  double last_heading_attempt_time_sec_{std::numeric_limits<double>::quiet_NaN()};
  double last_heading_update_time_sec_{std::numeric_limits<double>::quiet_NaN()};
  double last_heading_recovery_time_sec_{std::numeric_limits<double>::quiet_NaN()};
  double last_heading_residual_abs_deg_{std::numeric_limits<double>::quiet_NaN()};
  double last_heading_yaw_correction_abs_deg_{std::numeric_limits<double>::quiet_NaN()};
  std::string last_heading_mode_{"unknown"};
  double phase_error_memory_last_turnpost_context_sec_{
    std::numeric_limits<double>::quiet_NaN()};
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
  double last_heading_yaw_gain_hygiene_time_sec_{std::numeric_limits<double>::quiet_NaN()};
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
  bool have_core_imu_accumulator_{false};
  bool core_imu_accum_feed_delta_{false};
  double core_imu_accum_dt_sec_{0.0};
  Eigen::Vector3d core_imu_accum_dtheta_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d core_imu_accum_dvel_{Eigen::Vector3d::Zero()};
  double last_core_imu_rate_limit_process_steady_sec_{std::numeric_limits<double>::quiet_NaN()};
  std::uint64_t core_imu_rate_limit_input_count_{0};
  std::uint64_t core_imu_rate_limit_skip_count_{0};
  std::uint64_t core_imu_rate_limit_process_count_{0};


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
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr shadow_restore_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
    shadow_supervisor_perf_proxy_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
    shadow_supervisor_predictive_score_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
    shadow_supervisor_velocity_predictive_score_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
    shadow_supervisor_kinematic_predictive_score_pub_;
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
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr shadow_restore_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr shadow_supervisor_reference_odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
    shadow_supervisor_perf_proxy_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
    shadow_supervisor_predictive_score_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
    shadow_supervisor_velocity_predictive_score_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
    shadow_supervisor_kinematic_predictive_score_sub_;
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
  std::uint64_t raw_odom_publish_counter_{0};
};

std::shared_ptr<rclcpp::Node> make_node()
{
  return std::make_shared<KFGinsNativeNode>();
}

} // namespace kf_gins_ros2_native
