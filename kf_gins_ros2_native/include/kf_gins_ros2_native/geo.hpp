#pragma once
#include <Eigen/Dense>
namespace geo {
constexpr double WGS84_A = 6378137.0;
constexpr double WGS84_E2 = 6.69437999014e-3;
void llh_to_ecef(double lat_rad, double lon_rad, double h, double& x, double& y, double& z);
Eigen::Vector3d ecef_to_enu(const Eigen::Vector3d& ecef, const Eigen::Vector3d& ecef0, double lat0, double lon0);
Eigen::Quaterniond rpy_to_quat(double roll, double pitch, double yaw);
}
