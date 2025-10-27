#include "kf_gins_ros2_native/geo.hpp"
#include <cmath>
namespace geo {
void llh_to_ecef(double lat_rad, double lon_rad, double h, double& x, double& y, double& z){
  const double a=WGS84_A, e2=WGS84_E2;
  const double sl=std::sin(lat_rad), cl=std::cos(lat_rad);
  const double slo=std::sin(lon_rad), clo=std::cos(lon_rad);
  const double N=a/std::sqrt(1.0 - e2*sl*sl);
  x=(N+h)*cl*clo; y=(N+h)*cl*slo; z=(N*(1.0-e2)+h)*sl;
}
Eigen::Vector3d ecef_to_enu(const Eigen::Vector3d& ecef,const Eigen::Vector3d& ecef0,double lat0,double lon0){
  const double sl=std::sin(lat0), cl=std::cos(lat0);
  const double slo=std::sin(lon0), clo=std::cos(lon0);
  Eigen::Vector3d d = ecef - ecef0;
  return {-slo*d.x()+clo*d.y(), -clo*sl*d.x()-slo*sl*d.y()+cl*d.z(), clo*cl*d.x()+slo*cl*d.y()+sl*d.z()};
}
Eigen::Quaterniond rpy_to_quat(double r,double p,double y){
  const double cr=std::cos(r*0.5), sr=std::sin(r*0.5);
  const double cp=std::cos(p*0.5), sp=std::sin(p*0.5);
  const double cy=std::cos(y*0.5), sy=std::sin(y*0.5);
  Eigen::Quaterniond q; q.w()=cr*cp*cy+sr*sp*sy; q.x()=sr*cp*cy-cr*sp*sy; q.y()=cr*sp*cy+sr*cp*sy; q.z()=cr*cp*sy-sr*cp*cy; return q;
}
}
