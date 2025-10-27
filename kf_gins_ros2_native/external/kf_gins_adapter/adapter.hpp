
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
};

std::unique_ptr<KFCore> create_kf_core_adapter(const AdapterConfig& cfg);
std::unique_ptr<KFCore> create_kf_core(); // compatibility

} // namespace kfcore
