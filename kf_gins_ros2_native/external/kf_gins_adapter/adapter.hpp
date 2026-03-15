
#pragma once
#include <Eigen/Dense>
#include <memory>
#include <string>
#include "kf_gins_ros2_native/kf_core_interface.hpp"

namespace kfcore {

struct AdapterConfig {
  bool imu_is_delta = true;   // true if IMU inputs are Δθ/Δv; false if angular vel / linear acc
  double init_roll_deg  = 0.0;
  double init_pitch_deg = 0.0;
  double init_yaw_deg   = 0.0;
  bool force_zero_antlever = false;
  bool use_online_reset_covariance = true;
  double reset_pos_std_m = 5.0;
  double reset_vel_std_mps = 5.0;
  double reset_roll_pitch_std_deg = 5.0;
  double reset_yaw_std_deg = 10.0;
};

std::unique_ptr<KFCore> create_kf_core_adapter(const AdapterConfig& cfg);
std::unique_ptr<KFCore> create_kf_core(); // compatibility

} // namespace kfcore
