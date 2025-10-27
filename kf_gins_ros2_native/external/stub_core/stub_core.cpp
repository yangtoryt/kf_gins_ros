#include "kf_gins_ros2_native/kf_core_interface.hpp"
#include <cmath>
namespace kfcore {
class StubCore : public KFCore {
public:
  bool configure(const std::string&) override { return true; }
  bool reset(double lat_deg, double lon_deg, double h_m, double yaw_deg) override {
    (void)yaw_deg; st_.lat_deg=lat_deg; st_.lon_deg=lon_deg; st_.h_m=h_m; st_.vN=st_.vE=st_.vD=0.0; st_.roll_deg=st_.pitch_deg=st_.yaw_deg=0.0; return true;
  }
  bool ingestImu(double t,const Eigen::Vector3d& dtheta,const Eigen::Vector3d& dvel,double dt,bool imu_is_delta) override {
    (void)imu_is_delta; (void)dvel; (void)dt; st_.sow=t; st_.yaw_deg += dtheta.z() * 180.0/M_PI; return true;
  }
  bool ingestGnss(double t,double lat,double lon,double h,const Eigen::Vector3d& std_ned) override {
    (void)std_ned; st_.sow=t; st_.lat_deg=lat; st_.lon_deg=lon; st_.h_m=h; return true;
  }
  State current() const override { return st_; }
private: State st_;
};
std::unique_ptr<KFCore> create_kf_core(){ return std::unique_ptr<KFCore>(new StubCore()); }
} // namespace kfcore
