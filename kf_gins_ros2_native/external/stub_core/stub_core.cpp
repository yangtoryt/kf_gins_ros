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
  bool ingestHeading(double t, double yaw_deg, double yaw_std_deg) override {
    (void)yaw_std_deg; st_.sow = t; st_.yaw_deg = yaw_deg; return true;
  }
  bool forceYaw(double t, double yaw_deg, double yaw_std_deg) override {
    (void)yaw_std_deg; st_.sow = t; st_.yaw_deg = yaw_deg; return true;
  }
  bool forceRollPitch(double t, double roll_deg, double pitch_deg,
                      double roll_pitch_std_deg) override {
    (void)roll_pitch_std_deg; st_.sow = t; st_.roll_deg = roll_deg; st_.pitch_deg = pitch_deg; return true;
  }
  bool ingestGnssVel(double t, double vN, double vE, double vD,
                     const Eigen::Vector3d& std_vel) override {
    (void)std_vel; st_.sow = t; st_.vN = vN; st_.vE = vE; st_.vD = vD; return true;
  }
  bool reopenVerticalCovariance(double pos_std_m, double vel_std_mps,
                                double accbias_std_z_mps2) override {
    (void)pos_std_m; (void)vel_std_mps; (void)accbias_std_z_mps2; return false;
  }
  bool reopenYawCovariance(double t, double yaw_std_deg,
                           const std::string& reason) override {
    (void)t; (void)yaw_std_deg; (void)reason; return false;
  }
  bool reopenHorizontalPositionCovariance(double pos_std_h_m,
                                          double offdiag_corr_limit,
                                          const std::string& reason) override {
    (void)pos_std_h_m; (void)offdiag_corr_limit; (void)reason; return false;
  }
  void setPropagationNoiseScale(double arw_q_scale, double vrw_q_scale,
                                double gyrbias_q_scale,
                                double accbias_q_scale) override {
    (void)arw_q_scale; (void)vrw_q_scale; (void)gyrbias_q_scale; (void)accbias_q_scale;
  }
  void setAdaptiveRQProcessContext(bool valid, double score) override {
    (void)valid; (void)score;
  }
  void setAdaptiveRQSourceGate(
    bool allowed, double confidence, const std::string& reason) override {
    (void)allowed; (void)confidence; (void)reason;
  }
  void configureEarlyRecoveryBiasFeedback(
    bool debug_enable, bool apply_enable, double history_sec,
    double min_armed_time_sec, double max_armed_time_sec,
    double ba_z_mean_max_mps2, double residual_u_mean_max_m,
    double core_gnss_u_mean_min_m, double dx_ba_z_sum_max_mps2,
    int min_history_rows, double negative_dx_scale) override {
    (void)debug_enable; (void)apply_enable; (void)history_sec;
    (void)min_armed_time_sec; (void)max_armed_time_sec;
    (void)ba_z_mean_max_mps2; (void)residual_u_mean_max_m;
    (void)core_gnss_u_mean_min_m; (void)dx_ba_z_sum_max_mps2;
    (void)min_history_rows; (void)negative_dx_scale;
  }
  void setEarlyRecoveryBiasFeedbackContext(
    double armed_time_sec, double core_gnss_u_m) override {
    (void)armed_time_sec; (void)core_gnss_u_m;
  }
  KFCoreSnapshot snapshot() const override {
    KFCoreSnapshot snap;
    snap.valid = true;
    snap.time_sec = st_.sow;
    snap.state = st_;
    return snap;
  }
  bool restore(const KFCoreSnapshot& snapshot,
               const KFCoreRestorePolicy& policy) override {
    (void)policy;
    if (!snapshot.valid) {
      return false;
    }
    st_ = snapshot.state;
    st_.sow = snapshot.time_sec;
    return true;
  }
  State current() const override { return st_; }
  Eigen::MatrixXd covariance() const override { return Eigen::MatrixXd(); }
  ObservationDebug lastObservationDebug() const override { return {}; }
  StateUpdateDebug lastStateUpdateDebug() const override { return {}; }
  StateUpdateDebug lastGnssPositionStateUpdateDebug() const override { return {}; }
  void setStateUpdateDebugEnabled(bool enabled) override { (void)enabled; }
  std::vector<StateUpdateDebug> stateUpdateDebugEvents() const override { return {}; }
private: State st_;
};
std::unique_ptr<KFCore> create_kf_core(){ return std::unique_ptr<KFCore>(new StubCore()); }
} // namespace kfcore
