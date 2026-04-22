#pragma once
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <Eigen/Dense>
namespace kfcore {
struct State {
  double sow{0.0};
  double lat_deg{0.0}, lon_deg{0.0}, h_m{0.0};
  double vN{0.0}, vE{0.0}, vD{0.0};
  double roll_deg{0.0}, pitch_deg{0.0}, yaw_deg{0.0};
};

struct ObservationDebug {
  std::uint64_t sequence{0};
  bool valid{false};
  bool gnss_position_applied{false};
  bool gnss_velocity_applied{false};
  double update_time_sec{std::numeric_limits<double>::quiet_NaN()};
  int update_mode{0};
  Eigen::Vector3d gnss_position_residual_neu_m{
    Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())};
  Eigen::Vector3d gnss_position_std_neu_m{
    Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())};
  Eigen::Vector3d gnss_velocity_residual_ned_mps{
    Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())};
  Eigen::Vector3d gnss_velocity_std_ned_mps{
    Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())};
};

class KFCore {
public:
  virtual ~KFCore() = default;
  virtual bool configure(const std::string& config_path) = 0;
  virtual bool reset(double lat_deg, double lon_deg, double h_m, double yaw_deg) = 0;
  virtual bool ingestImu(double t_sec, const Eigen::Vector3d& dtheta, const Eigen::Vector3d& dvel,
                         double dt_sec, bool imu_is_delta) = 0;
  virtual bool ingestGnss(double t_sec, double lat_deg, double lon_deg, double h_m,
                          const Eigen::Vector3d& std_ned) = 0;
  virtual bool ingestGnssVel(double t_sec, double vN, double vE, double vD,
                             const Eigen::Vector3d& std_vel) = 0;
  virtual bool ingestHeading(double t_sec, double yaw_deg, double yaw_std_deg) = 0;
  virtual bool forceYaw(double t_sec, double yaw_deg, double yaw_std_deg) = 0;
  virtual bool forceRollPitch(double t_sec, double roll_deg, double pitch_deg,
                              double roll_pitch_std_deg) = 0;
  virtual bool reopenVerticalCovariance(double pos_std_m, double vel_std_mps,
                                        double accbias_std_z_mps2) = 0;
  virtual State current() const = 0;
  virtual Eigen::MatrixXd covariance() const = 0;
  virtual ObservationDebug lastObservationDebug() const = 0;
};
std::unique_ptr<KFCore> create_kf_core();
}
