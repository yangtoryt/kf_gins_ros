#pragma once
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <Eigen/Dense>
namespace kfcore {
struct State {
  double sow{0.0};
  double lat_deg{0.0}, lon_deg{0.0}, h_m{0.0};
  double vN{0.0}, vE{0.0}, vD{0.0};
  double roll_deg{0.0}, pitch_deg{0.0}, yaw_deg{0.0};
};

struct ImuErrorState {
  Eigen::Vector3d gyro_bias_radps{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acc_bias_mps2{Eigen::Vector3d::Zero()};
  Eigen::Vector3d gyro_scale{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acc_scale{Eigen::Vector3d::Zero()};
};

struct AdaptiveRMemory {
  double position_gamma{1.0};
  double velocity_gamma{1.0};
};

struct AdaptiveRQMemory {
  double position_gamma{1.0};
  double velocity_gamma{1.0};
  double vrw_q_scale{1.0};
  double arw_q_scale{1.0};
  double accbias_q_scale{1.0};
  double gyrbias_q_scale{1.0};
  int consecutive_exceed_count{0};
  int hold_remaining{0};
  bool process_context_valid{false};
  double process_context_score{0.0};
  bool source_gate_allowed{true};
  double source_confidence{1.0};
  std::string source_gate_reason{"restore"};
  bool velocity_evidence_valid{false};
  bool velocity_evidence_active{false};
  double velocity_evidence_nis_ratio{std::numeric_limits<double>::quiet_NaN()};
  double velocity_evidence_residual_h_mps{std::numeric_limits<double>::quiet_NaN()};
  bool have_prev_position_r_diag{false};
  Eigen::Vector3d prev_position_r_diag{
    Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())};
};

struct ImuSampleSnapshot {
  double time_sec{std::numeric_limits<double>::quiet_NaN()};
  double dt_sec{std::numeric_limits<double>::quiet_NaN()};
  Eigen::Vector3d dtheta{Eigen::Vector3d::Zero()};
  Eigen::Vector3d dvel{Eigen::Vector3d::Zero()};
  double odovel{0.0};
};

struct KFCoreSnapshot {
  bool valid{false};
  double time_sec{std::numeric_limits<double>::quiet_NaN()};
  State state{};
  ImuErrorState imu_error{};
  Eigen::MatrixXd covariance{};
  Eigen::MatrixXd error_state{};
  ImuSampleSnapshot previous_imu{};
  ImuSampleSnapshot current_imu{};
  AdaptiveRMemory adaptive_r{};
  AdaptiveRQMemory adaptive_rq{};
};

struct KFCoreRestorePolicy {
  bool copy_nominal_state{true};
  bool copy_covariance{true};
  bool copy_imu_error{true};
  bool copy_imu_buffer{true};
  bool copy_adaptive_r_memory{false};
  bool copy_adaptive_rq_memory{false};
  bool reset_adaptive_r_memory{true};
  bool reset_adaptive_rq_memory{true};
  double covariance_inflation_factor{1.0};
  bool require_spd_covariance{true};
  std::string reason{"main_to_shadow_restore"};
};

struct BoundedAdaptiveRDebug {
  bool enabled{false};
  bool applied{false};
  bool exceeded{false};
  std::string update_type{"unknown"};
  std::string mode{"disabled"};
  std::string rq_selector_trigger{"none"};
  std::string rq_selector_reason{"disabled"};
  double nis{std::numeric_limits<double>::quiet_NaN()};
  double chi2_threshold{std::numeric_limits<double>::quiet_NaN()};
  double nis_ratio{std::numeric_limits<double>::quiet_NaN()};
  int consecutive_exceed_count{0};
  int hold_remaining{0};
  double observation_score{0.0};
  double process_score{0.0};
  double gamma_raw{1.0};
  double gamma_smoothed{1.0};
  double gamma_clipped{1.0};
  double r_gamma_limit{1.0};
  double q_lambda_vrw{1.0};
  double q_lambda_arw{1.0};
  double q_lambda_accbias{1.0};
  double q_lambda_gyrbias{1.0};
  bool gps_quality_stable{false};
  bool motion_context_ok{false};
  double q_source_confidence{1.0};
  bool q_source_gate_allowed{true};
  std::string q_source_gate_reason{"disabled"};
  bool q_velocity_evidence{false};
  double q_velocity_nis_ratio{std::numeric_limits<double>::quiet_NaN()};
  double q_velocity_residual_h_mps{std::numeric_limits<double>::quiet_NaN()};
  Eigen::Vector3d r_base_diag{
    Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())};
  Eigen::Vector3d r_eff_diag{
    Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())};
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
  Eigen::Matrix3d gnss_position_innovation_cov_neu_m2{
    Eigen::Matrix3d::Constant(std::numeric_limits<double>::quiet_NaN())};
  double gnss_position_nis_h_2d{std::numeric_limits<double>::quiet_NaN()};
  double gnss_position_nis_u_1d{std::numeric_limits<double>::quiet_NaN()};
  double gnss_position_nis_3d{std::numeric_limits<double>::quiet_NaN()};
  double gnss_position_gate_threshold_nis{std::numeric_limits<double>::quiet_NaN()};
  bool gnss_position_update_accepted{false};
  bool gnss_position_update_rejected{false};
  std::string gnss_position_update_reason{"not_available"};
  Eigen::Vector3d gnss_velocity_residual_ned_mps{
    Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())};
  Eigen::Vector3d gnss_velocity_std_ned_mps{
    Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())};
  BoundedAdaptiveRDebug gnss_position_adaptive_r{};
  BoundedAdaptiveRDebug gnss_velocity_adaptive_r{};
};

struct EarlyRecoveryBiasFeedbackDebug {
  bool enabled{false};
  bool apply_enabled{false};
  bool candidate{false};
  bool active{false};
  bool applied{false};
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

struct StateUpdateDebug {
  std::uint64_t sequence{0};
  bool valid{false};
  bool applied{false};
  std::string event_type{"unknown"};
  std::string reason{"unknown"};
  double update_time_sec{std::numeric_limits<double>::quiet_NaN()};
  int update_mode{0};
  Eigen::VectorXd dx{Eigen::VectorXd()};
  Eigen::Vector3d pos_blh_before_rad_m{
    Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())};
  Eigen::Vector3d pos_blh_after_rad_m{
    Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())};
  Eigen::Vector3d vel_before_ned_mps{
    Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())};
  Eigen::Vector3d vel_after_ned_mps{
    Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())};
  Eigen::Vector3d euler_before_rad{
    Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())};
  Eigen::Vector3d euler_after_rad{
    Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())};
  Eigen::Vector3d gyrbias_before_radps{
    Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())};
  Eigen::Vector3d gyrbias_after_radps{
    Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())};
  Eigen::Vector3d accbias_before_mps2{
    Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())};
  Eigen::Vector3d accbias_after_mps2{
    Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())};
  Eigen::MatrixXd covariance_before{Eigen::MatrixXd()};
  Eigen::MatrixXd covariance_after{Eigen::MatrixXd()};
  Eigen::MatrixXd kalman_gain{Eigen::MatrixXd()};
  bool gnss_position_observation_valid{false};
  std::uint64_t observation_sequence{0};
  Eigen::Vector3d gnss_position_residual_neu_m{
    Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())};
  Eigen::Vector3d gnss_position_std_neu_m{
    Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())};
  Eigen::Matrix3d gnss_position_innovation_cov_neu_m2{
    Eigen::Matrix3d::Constant(std::numeric_limits<double>::quiet_NaN())};
  double gnss_position_nis_h_2d{std::numeric_limits<double>::quiet_NaN()};
  double gnss_position_nis_u_1d{std::numeric_limits<double>::quiet_NaN()};
  double gnss_position_nis_3d{std::numeric_limits<double>::quiet_NaN()};
  double gnss_position_gate_threshold_nis{std::numeric_limits<double>::quiet_NaN()};
  bool gnss_position_update_accepted{false};
  bool gnss_position_update_rejected{false};
  std::string gnss_position_update_reason{"not_available"};
  BoundedAdaptiveRDebug gnss_position_adaptive_r{};
  BoundedAdaptiveRDebug gnss_velocity_adaptive_r{};
  EarlyRecoveryBiasFeedbackDebug early_recovery_bias_feedback{};
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
  virtual bool reopenYawCovariance(double t_sec, double yaw_std_deg,
                                   const std::string& reason) = 0;
  virtual bool reopenHorizontalPositionCovariance(double pos_std_h_m,
                                                  double offdiag_corr_limit,
                                                  const std::string& reason) = 0;
  virtual void setPropagationNoiseScale(double arw_q_scale, double vrw_q_scale,
                                        double gyrbias_q_scale,
                                        double accbias_q_scale) = 0;
  virtual void setAdaptiveRQProcessContext(bool valid, double score) = 0;
  virtual void setAdaptiveRQSourceGate(
    bool allowed, double confidence, const std::string& reason) = 0;
  virtual void configureEarlyRecoveryBiasFeedback(
    bool debug_enable, bool apply_enable, double history_sec,
    double min_armed_time_sec, double max_armed_time_sec,
    double ba_z_mean_max_mps2, double residual_u_mean_max_m,
    double core_gnss_u_mean_min_m, double dx_ba_z_sum_max_mps2,
    int min_history_rows, double negative_dx_scale) = 0;
  virtual void setEarlyRecoveryBiasFeedbackContext(
    double armed_time_sec, double core_gnss_u_m) = 0;
  virtual KFCoreSnapshot snapshot() const = 0;
  virtual bool restore(const KFCoreSnapshot& snapshot,
                       const KFCoreRestorePolicy& policy) = 0;
  virtual State current() const = 0;
  virtual Eigen::MatrixXd covariance() const = 0;
  virtual ObservationDebug lastObservationDebug() const = 0;
  virtual StateUpdateDebug lastStateUpdateDebug() const = 0;
  virtual StateUpdateDebug lastGnssPositionStateUpdateDebug() const = 0;
  virtual void setStateUpdateDebugEnabled(bool enabled) = 0;
  virtual std::vector<StateUpdateDebug> stateUpdateDebugEvents() const = 0;
};
std::unique_ptr<KFCore> create_kf_core();
}
