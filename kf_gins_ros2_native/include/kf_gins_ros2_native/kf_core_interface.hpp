#pragma once
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
class KFCore {
public:
  virtual ~KFCore() = default;
  virtual bool configure(const std::string& config_path) = 0;
  virtual bool reset(double lat_deg, double lon_deg, double h_m, double yaw_deg) = 0;
  virtual bool ingestImu(double t_sec, const Eigen::Vector3d& dtheta, const Eigen::Vector3d& dvel,
                         double dt_sec, bool imu_is_delta) = 0;
  virtual bool ingestGnss(double t_sec, double lat_deg, double lon_deg, double h_m,
                          const Eigen::Vector3d& std_ned) = 0;
  virtual State current() const = 0;
};
std::unique_ptr<KFCore> create_kf_core();
}
