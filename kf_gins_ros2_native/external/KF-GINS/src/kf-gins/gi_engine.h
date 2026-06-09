/*
 * KF-GINS: An EKF-Based GNSS/INS Integrated Navigation System
 *
 * Copyright (C) 2022 i2Nav Group, Wuhan University
 *
 *     Author : Liqiang Wang
 *    Contact : wlq@whu.edu.cn
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef GI_ENGINE_H
#define GI_ENGINE_H

#include <cstdint>
#include <deque>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include "common/types.h"

#include "kf_gins_types.h"

struct BoundedAdaptiveRDebugInfo {
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

struct ObservationDebugInfo {
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
    BoundedAdaptiveRDebugInfo gnss_position_adaptive_r{};
    BoundedAdaptiveRDebugInfo gnss_velocity_adaptive_r{};
};

struct EarlyRecoveryBiasFeedbackDebugInfo {
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

struct StateUpdateDebugInfo {
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
    BoundedAdaptiveRDebugInfo gnss_position_adaptive_r{};
    BoundedAdaptiveRDebugInfo gnss_velocity_adaptive_r{};
    EarlyRecoveryBiasFeedbackDebugInfo early_recovery_bias_feedback{};
};

struct BoundedAdaptiveREngineMemory {
    double position_gamma{1.0};
    double velocity_gamma{1.0};
};

struct BoundedAdaptiveRQEngineMemory {
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

struct GIEngineSnapshot {
    bool valid{false};
    double timestamp{std::numeric_limits<double>::quiet_NaN()};
    NavState nav_state{};
    Eigen::MatrixXd covariance{};
    Eigen::MatrixXd dx{};
    IMU imupre{};
    IMU imucur{};
    BoundedAdaptiveREngineMemory adaptive_r{};
    BoundedAdaptiveRQEngineMemory adaptive_rq{};
};

struct GIEngineRestorePolicy {
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

class GIEngine {

public:
    explicit GIEngine(GINSOptions &options);

    ~GIEngine() = default;

    /**
     * @brief 添加新的IMU数据，(不)补偿IMU误差
     *        add new imudata, do (not) compensate imu error
     * @param [in] imu        新的IMU原始数据
     *                        new raw imudata
     * @param [in] compensate 是否补偿IMU误差
     *                        if compensate imu error to new imudata
     * */
    void addImuData(const IMU &imu, bool compensate = false) {

        imupre_ = imucur_;
        imucur_ = imu;

        if (compensate) {
            imuCompensate(imucur_);
        }
    }

    /**
     * @brief 添加新的GNSS数据
     *        add new gnssdata
     * @param [in] gnss 新的GNSS数据
     *                  new gnssdata
     * */
    void addGnssData(const GNSS &gnss) {

        gnssdata_ = gnss;
        // 暂不进行数据有效性检查，GNSS数据默认有效
        // do not check the validity of gnssdata, the gnssdata is valid by default
        gnssdata_.isvalid = true;
    }

    /**
     * @brief 处理新的IMU数据
     *        process new imudata
     * */
    void newImuProcess();

    /**
     * @brief 内插增量形式的IMU数据到指定时刻
     *        interpolate incremental imudata to given timestamp
     * @param [in]     imu1      前一时刻IMU数据
     *                           the previous imudata
     * @param [in,out] imu2      当前时刻IMU数据
     *                           the current imudata
     * @param [in]     timestamp 给定内插到的时刻
     *                           given interpolate timestamp
     * @param [in,out] midimu    输出内插时刻的IMU数据
     *                           output imudata at given timestamp
     * */
    static void imuInterpolate(const IMU &imu1, IMU &imu2, const double timestamp, IMU &midimu) {

        if (imu1.time > timestamp || imu2.time < timestamp) {
            return;
        }

        double lamda = (timestamp - imu1.time) / (imu2.time - imu1.time);

        midimu.time   = timestamp;
        midimu.dtheta = imu2.dtheta * lamda;
        midimu.dvel   = imu2.dvel * lamda;
        midimu.dt     = timestamp - imu1.time;

        imu2.dtheta = imu2.dtheta - midimu.dtheta;
        imu2.dvel   = imu2.dvel - midimu.dvel;
        imu2.dt     = imu2.dt - midimu.dt;
    }

    /**
     * @brief 添加GNSS速度观测数据
     *        add GNSS velocity observation
     * @param [in] vel_ned  NED速度 (m/s)
     * @param [in] vel_std  速度标准差 (m/s)
     * @param [in] time     观测时间
     * */
    void addVelData(const Eigen::Vector3d& vel_ned, const Eigen::Vector3d& vel_std, double time) {
        vel_obs_      = vel_ned;
        vel_obs_std_  = vel_std;
        vel_obs_time_ = time;
        has_vel_obs_  = true;
    }

    /**
     * @brief 添加外部 heading/yaw 观测（NED yaw）
     *        add external heading/yaw observation (NED yaw)
     * @param [in] yaw_rad      heading measurement in radians
     * @param [in] yaw_std_rad  heading std in radians
     * @param [in] time         observation time
     * */
    void addHeadingData(double yaw_rad, double yaw_std_rad, double time) {
        heading_obs_yaw_rad_ = yaw_rad;
        heading_obs_std_rad_ = std::max(1e-4, std::abs(yaw_std_rad));
        heading_obs_time_    = time;
        has_heading_obs_     = true;
    }

    /**
     * @brief 立即应用 heading 观测更新
     *        apply heading observation update immediately
     * */
    void headingUpdate();

    /**
     * @brief 强制将当前 yaw 对齐到外部 heading，并重置 yaw 相关协方差
     *        force current yaw to external heading and reopen yaw covariance
     * @param [in] yaw_rad      heading in radians
     * @param [in] yaw_std_rad  yaw std in radians
     * @param [in] time         update time
     * */
    void forceYaw(double yaw_rad, double yaw_std_rad, double time);

    /**
     * @brief 强制将当前 roll/pitch 对齐到外部姿态，并重置对应协方差
     *        force current roll/pitch to external attitude and reopen covariance
     * @param [in] roll_rad            roll in radians
     * @param [in] pitch_rad           pitch in radians
     * @param [in] roll_pitch_std_rad  roll/pitch std in radians
     * @param [in] time                update time
     * */
    void forceRollPitch(double roll_rad, double pitch_rad, double roll_pitch_std_rad, double time);

    /**
     * @brief 重开垂向位置/速度/加速度零偏协方差
     *        reopen vertical position/velocity/acc-bias covariance
     * @param [in] pos_std_m           target vertical position std
     * @param [in] vel_std_mps         target vertical velocity std
     * @param [in] accbias_std_z_mps2  target vertical accelerometer bias std
     * @return true if any covariance diagonal entry was increased
     * */
    bool reopenVerticalCovariance(double pos_std_m, double vel_std_mps, double accbias_std_z_mps2);

    /**
     * @brief 重开 yaw 误差协方差，不直接改写 yaw 状态
     *        reopen yaw error covariance without forcing the yaw state
     * @param [in] yaw_std_rad         target yaw std in radians
     * @param [in] time                update time
     * @param [in] reason              debug reason label
     * @return true if the yaw covariance diagonal entry was increased
     * */
    bool reopenYawCovariance(double yaw_std_rad, double time, const std::string &reason);

    /**
     * @brief 重开水平位置协方差
     *        reopen horizontal position covariance
     * @param [in] pos_std_h_m         target horizontal position std
     * @param [in] offdiag_corr_limit  maximum absolute NE correlation after clamp
     * @param [in] reason              debug reason label
     * @return true if any covariance element was changed
     * */
    bool reopenHorizontalPositionCovariance(
        double pos_std_h_m, double offdiag_corr_limit, const std::string &reason);

    /**
     * @brief Set runtime process-noise multipliers used during INS propagation.
     *        Values multiply the continuous-time Qc variance blocks and are clamped.
     * */
    void setPropagationNoiseScale(
        double arw_q_scale, double vrw_q_scale,
        double gyrbias_q_scale, double accbias_q_scale);
    void setAdaptiveRQProcessContext(bool valid, double score);
    void setAdaptiveRQSourceGate(
        bool allowed, double confidence, const std::string &reason);
    void configureEarlyRecoveryBiasFeedback(
        bool debug_enable,
        bool apply_enable,
        double history_sec,
        double min_armed_time_sec,
        double max_armed_time_sec,
        double ba_z_mean_max_mps2,
        double residual_u_mean_max_m,
        double core_gnss_u_mean_min_m,
        double dx_ba_z_sum_max_mps2,
        int min_history_rows,
        double negative_dx_scale);
    void setEarlyRecoveryBiasFeedbackContext(double armed_time_sec, double core_gnss_u_m);

    /**
     * @brief 获取当前时间
     *        get current time
     * */
    double timestamp() const {
        return timestamp_;
    }

    /**
     * @brief 获取当前IMU状态
     *        get current navigation state
     * */
    NavState getNavState();

    /**
     * @brief 获取当前状态协方差
     *        get current state covariance
     * */
    Eigen::MatrixXd getCovariance() {
        return Cov_;
    }

    GIEngineSnapshot snapshot() const;
    bool restore(const GIEngineSnapshot &snapshot, const GIEngineRestorePolicy &policy);

    const ObservationDebugInfo &lastObservationDebug() const {
        return last_observation_debug_;
    }

    void setStateUpdateDebugEnabled(bool enabled) {
        state_update_debug_enabled_ = enabled;
        if (!state_update_debug_enabled_) {
            state_update_debug_events_.clear();
        }
    }

    const std::vector<StateUpdateDebugInfo> &stateUpdateDebugEvents() const {
        return state_update_debug_events_;
    }

    const StateUpdateDebugInfo &lastStateUpdateDebug() const {
        return last_state_update_debug_event_;
    }

    const StateUpdateDebugInfo &lastGnssPositionStateUpdateDebug() const {
        return last_gnss_position_state_update_debug_event_;
    }

private:
    /**
     * @brief 初始化系统状态和协方差
     *        initialize state and state covariance
     * @param [in] initstate     初始状态
     *                           initial state
     * @param [in] initstate_std 初始状态标准差
     *                           initial state std
     * */
    void initialize(const NavState &initstate, const NavState &initstate_std);

    /**
     * @brief 当前IMU误差补偿到IMU数据中
     *        componsate imu error to the imudata
     * @param [in,out] imu 需要补偿的IMU数据
     *                     imudata to be compensated
     * */
    void imuCompensate(IMU &imu);

    /**
     * @brief 判断是否需要更新,以及更新哪一时刻系统状态
     *        determine if we should do upate and which navstate to update
     * @param [in] imutime1   上一IMU状态时间
     *                        the last state time
     * @param [in] imutime2   当前IMU状态时间
     *                        the current state time
     * @param [in] updatetime 状态更新的时间
     *                        time to update state
     * @return 0: 不需要更新
     *            donot need update
     *         1: 需要更新上一IMU状态
     *            update the last navstate
     *         2: 需要更新当前IMU状态
     *            update the current navstate
     *         3: 需要将IMU进行内插到状态更新时间
     *            need interpolate imudata to updatetime
     * */
    int isToUpdate(double imutime1, double imutime2, double updatetime) const;

    /**
     * @brief 进行INS状态更新(IMU机械编排算法), 并计算IMU状态转移矩阵和噪声阵
     *        do INS state update(INS mechanization), and compute state transition matrix and noise matrix
     * @param [in,out] imupre 前一时刻IMU数据
     *                        imudata at the previous epoch
     * @param [in,out] imucur 当前时刻IMU数据
     *                        imudata at the current epoch
     * */
    void insPropagation(IMU &imupre, IMU &imucur);

    /**
     * @brief 使用GNSS位置观测更新系统状态
     *        update state using gnss position
     * @param [in,out] gnssdata
     * */
    void gnssUpdate(GNSS &gnssdata);

    /**
     * @brief Kalman 预测,
     *        Kalman Filter Predict process
     * @param [in,out] Phi 状态转移矩阵
     *                     state transition matrix
     * @param [in,out] Qd  传播噪声矩阵
     *                     propagation noise matrix
     * */
	void EKFPredict(Eigen::MatrixXd &Phi, Eigen::MatrixXd &Qd);

	/**
	 * @brief 误差状态IEKF更新 (Iterated Error-State EKF)
	 *        Iterated error-state update used by nonlinear GNSS position / heading measurements.
	 *        The prior covariance is kept fixed across the inner re-linearization loop.
	 * @param [in] meas_model  给定线性化点时，输出 dz 和 H 的观测模型
	 *                         measurement model (outputs dz and H at a given linearization point)
	 * @param [in] R           观测噪声阵 / measurement noise matrix
	 * */
	void IEKFUpdate(
	    const std::function<void(const PVA &, const ImuError &, Eigen::MatrixXd &, Eigen::MatrixXd &)> &meas_model,
	    Eigen::MatrixXd &R, const Eigen::MatrixXd &P0);

    /**
     * @brief Sage-Husa 自适应噪声估计算法 (更新 R)
     *        Sage-Husa adaptive noise estimation (updates R)
     * @param [in] dz     观测残差 (innovation)
     * @param [in] H      观测矩阵
     * @param [in,out] R  观测噪声阵
     * @param [in] P0     先验误差协方差
     * */
    void adaptiveREstimation(const Eigen::VectorXd &dz, const Eigen::MatrixXd &H, Eigen::MatrixXd &R, const Eigen::MatrixXd &P0);

    /**
     * @brief 反馈误差状态到当前状态
     *        feedback error state to the current state
     * */
    void stateFeedback();
    void feedbackAndRecordStateUpdate(
        const std::string &event_type,
        const std::string &reason,
        double update_time_sec,
        int update_mode,
        const PVA &pva_before,
        const ImuError &imuerror_before,
        const Eigen::MatrixXd &covariance_before);
    void recordStateUpdateDebug(
        const std::string &event_type,
        const std::string &reason,
        double update_time_sec,
        int update_mode,
        bool applied,
        const Eigen::MatrixXd &dx,
        const PVA &pva_before,
        const ImuError &imuerror_before,
        const Eigen::MatrixXd &covariance_before,
        const PVA &pva_after,
        const ImuError &imuerror_after,
        const Eigen::MatrixXd &covariance_after,
        const EarlyRecoveryBiasFeedbackDebugInfo &early_recovery_bias_feedback =
            EarlyRecoveryBiasFeedbackDebugInfo{});

    /**
     * @brief 检查协方差对角线元素是否都为正
     *        Check if covariance diagonal elements are all positive
     * */
    void checkCov() {

        bool bad_diag = false;
        for (int i = 0; i < RANK; i++) {
            if (!std::isfinite(Cov_(i, i)) || Cov_(i, i) < 0) {
                bad_diag = true;
                break;
            }
        }
        if (!bad_diag) return;

        // Numerical issues can make Cov_ lose SPD, especially under long GNSS outages / timing glitches.
        // Instead of exiting, project Cov_ back to the nearest SPD matrix (clamp eigenvalues).
        const double eps = 1e-12;

        Cov_ = 0.5 * (Cov_ + Cov_.transpose());
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(Cov_);
        if (es.info() != Eigen::Success) {
            std::cout << "Covariance is invalid at " << std::setprecision(10) << timestamp_
                      << " (eigen solve failed). Resetting covariance to identity." << std::endl;
            Cov_.setIdentity();
            Cov_ *= 1.0;
            return;
        }

        Eigen::VectorXd eval = es.eigenvalues();
        const double min_eig = eval.minCoeff();
        if (!std::isfinite(min_eig) || min_eig <= eps) {
            std::cout << "Covariance is not SPD at " << std::setprecision(10) << timestamp_
                      << " (min_eig=" << min_eig << "). Projecting to SPD." << std::endl;
            eval = eval.cwiseMax(eps);
            Cov_ = es.eigenvectors() * eval.asDiagonal() * es.eigenvectors().transpose();
            Cov_ = 0.5 * (Cov_ + Cov_.transpose());
        }
    }

private:
    GINSOptions options_;

    double timestamp_{0.0};

    // 更新时间对齐误差，IMU状态和观测信息误差小于它则认为两者对齐
    // updata time align error
    const double TIME_ALIGN_ERR = 0.001;

    // IMU/GNSS raw samples participate in the very first post-reset update decision.
    // Keep them value-initialized so reset -> first GNSS/IMU cannot read indeterminate data.
    IMU imupre_{};
    IMU imucur_{};
    GNSS gnssdata_{};

    // IMU状态（位置、速度、姿态和IMU误差）
    // imu state (position, velocity, attitude and imu error)
    PVA pvacur_{};
    PVA pvapre_{};
    ImuError imuerror_{};

    // Kalman滤波相关
    // ekf variables
	Eigen::MatrixXd Cov_;
	Eigen::MatrixXd Qc_;
	Eigen::MatrixXd dx_;
	double propagation_arw_q_scale_{1.0};
	double propagation_vrw_q_scale_{1.0};
	double propagation_gyrbias_q_scale_{1.0};
	double propagation_accbias_q_scale_{1.0};
	    
	// IESKF/IEKF iteration parameters for nonlinear error-state measurement updates.
	// Used by GNSS position and heading; GNSS velocity remains a linear error-state update.
	int iekf_max_iterations_{5};              // 最大迭代次数 / max iterations
	double iekf_convergence_threshold_{1e-6}; // 收敛阈值 / convergence threshold
	
	void applyErrorState(const Eigen::MatrixXd &dx, PVA &pva, ImuError &imuerror) const;

	const int RANK      = 21;
	const int NOISERANK = 18;

    /**
     * @brief 使用GNSS速度观测更新系统状态（线性观测，标准误差状态更新）
     *        Standard linear error-state update for GNSS velocity; not iterated.
     * */
    void gnssVelUpdate();
    void beginObservationDebug(int update_mode, double update_time_sec);
    Eigen::MatrixXd boundedAdaptiveREffective_(
        const std::string &update_type,
        bool apply_enabled,
        double nis,
        const Eigen::MatrixXd &R_base,
        double &gamma_state,
        BoundedAdaptiveRDebugInfo &debug);
    Eigen::MatrixXd boundedAdaptiveRQEffective_(
        const std::string &update_type,
        bool apply_enabled,
        bool update_process_q,
        double nis,
        const Eigen::MatrixXd &R_base,
        double &gamma_state,
        BoundedAdaptiveRDebugInfo &debug);
    void resetBoundedAdaptiveRQScales_();
    void updateBoundedAdaptiveRQProcessScales_(
        bool process_triggered,
        double nis_ratio,
        BoundedAdaptiveRDebugInfo &debug);
    double computeNis_(const Eigen::MatrixXd &S, const Eigen::MatrixXd &r) const;
    EarlyRecoveryBiasFeedbackDebugInfo evaluateEarlyRecoveryBiasFeedback_(
        const std::string &event_type,
        double update_time_sec,
        const ImuError &imuerror_before,
        double residual_u_m,
        double raw_dx_ba_z_mps2);

    struct EarlyRecoveryBiasFeedbackSample {
        double armed_time_sec{std::numeric_limits<double>::quiet_NaN()};
        double ba_z_before_mps2{std::numeric_limits<double>::quiet_NaN()};
        double residual_u_m{std::numeric_limits<double>::quiet_NaN()};
        double core_gnss_u_m{std::numeric_limits<double>::quiet_NaN()};
        double dx_ba_z_mps2{std::numeric_limits<double>::quiet_NaN()};
    };

    // 外部 heading 观测数据
    bool has_heading_obs_{false};
    double heading_obs_yaw_rad_{0.0};
    double heading_obs_std_rad_{1.0};
    double heading_obs_time_{0.0};

    // GNSS 速度观测数据
    // GNSS velocity observation data
    bool has_vel_obs_{false};
    Eigen::Vector3d vel_obs_{0, 0, 0};       // NED velocity (m/s)
    Eigen::Vector3d vel_obs_std_{1, 1, 1};   // velocity std (m/s)
    double vel_obs_time_{0};

    // Sage-Husa 自适应相关变量（用于在线更新 R）
    bool use_sage_husa_{true};
    double sage_husa_alpha_{0.95};
    bool sage_husa_diag_only_{true};
    double sage_husa_min_var_factor_{0.1};
    double sage_husa_min_var_abs_{0.0};
    std::uint64_t sage_husa_k_{0};
    Eigen::MatrixXd R_gnsspos_;       // GNSS 观测噪声阵（持久化）/ GNSS noise matrix (persisted)
    Eigen::MatrixXd R_gnsspos_init_;  // 初始 R (用于下限) / initial R floor reference
    bool bounded_adaptive_r_enable_{false};
    std::string bounded_adaptive_r_mode_{"nis_bounded"};
    bool bounded_adaptive_r_apply_position_{true};
    bool bounded_adaptive_r_apply_velocity_{true};
    double bounded_adaptive_r_alpha_{1.0};
    double bounded_adaptive_r_beta_{0.2};
    double bounded_adaptive_r_gamma_min_{1.0};
    double bounded_adaptive_r_gamma_max_{10.0};
    double bounded_adaptive_r_chi2_prob_{0.95};
    double bounded_adaptive_r_chi2_threshold_3d_{7.814727903251179};
    double bounded_adaptive_r_position_gamma_{1.0};
    double bounded_adaptive_r_velocity_gamma_{1.0};
    bool bounded_adaptive_rq_enable_{false};
    std::string bounded_adaptive_rq_mode_{"nis_rq_selector"};
    bool bounded_adaptive_rq_apply_position_{true};
    bool bounded_adaptive_rq_apply_velocity_{false};
    bool bounded_adaptive_rq_r_only_on_observation_disturbance_{true};
    bool bounded_adaptive_rq_q_on_process_disturbance_{true};
    double bounded_adaptive_rq_chi2_prob_{0.95};
    double bounded_adaptive_rq_chi2_threshold_3d_{7.814727903251179};
    double bounded_adaptive_rq_nis_ratio_start_{1.0};
    double bounded_adaptive_rq_nis_ratio_full_{3.0};
    int bounded_adaptive_rq_consecutive_exceed_min_{3};
    int bounded_adaptive_rq_hold_updates_{5};
    double bounded_adaptive_rq_alpha_r_{1.0};
    double bounded_adaptive_rq_alpha_r_process_{0.25};
    double bounded_adaptive_rq_alpha_q_{1.0};
    double bounded_adaptive_rq_beta_r_{0.2};
    double bounded_adaptive_rq_beta_q_{0.1};
    double bounded_adaptive_rq_gamma_r_min_{1.0};
    double bounded_adaptive_rq_gamma_r_max_observation_{6.0};
    double bounded_adaptive_rq_gamma_r_max_process_{1.5};
    double bounded_adaptive_rq_lambda_vrw_max_{3.0};
    double bounded_adaptive_rq_lambda_arw_max_{1.5};
    double bounded_adaptive_rq_lambda_accbias_max_{2.0};
    double bounded_adaptive_rq_lambda_gyrbias_max_{1.0};
    bool bounded_adaptive_rq_require_gps_quality_stable_for_q_{true};
    bool bounded_adaptive_rq_require_process_context_for_q_{true};
    double bounded_adaptive_rq_process_context_score_start_{0.20};
    double bounded_adaptive_rq_mixed_observation_score_start_{0.50};
    bool bounded_adaptive_rq_source_gate_enable_{false};
    double bounded_adaptive_rq_q_source_observation_score_max_{1.0};
    bool bounded_adaptive_rq_velocity_evidence_gate_enable_{false};
    double bounded_adaptive_rq_q_high_observation_velocity_nis_ratio_min_{2.0};
    double bounded_adaptive_rq_q_high_observation_velocity_residual_h_min_{0.15};
    double bounded_adaptive_rq_velocity_evidence_time_tolerance_sec_{0.5};
    int bounded_adaptive_rq_consecutive_exceed_count_{0};
    int bounded_adaptive_rq_hold_remaining_{0};
    bool bounded_adaptive_rq_process_context_valid_{false};
    double bounded_adaptive_rq_process_context_score_{0.0};
    bool bounded_adaptive_rq_source_gate_allowed_{true};
    double bounded_adaptive_rq_source_confidence_{1.0};
    std::string bounded_adaptive_rq_source_gate_reason_{"disabled"};
    bool bounded_adaptive_rq_velocity_evidence_valid_{false};
    bool bounded_adaptive_rq_velocity_evidence_active_{false};
    double bounded_adaptive_rq_velocity_evidence_nis_ratio_{
        std::numeric_limits<double>::quiet_NaN()};
    double bounded_adaptive_rq_velocity_evidence_residual_h_mps_{
        std::numeric_limits<double>::quiet_NaN()};
    double bounded_adaptive_rq_position_gamma_{1.0};
    double bounded_adaptive_rq_velocity_gamma_{1.0};
    double bounded_adaptive_rq_vrw_q_scale_{1.0};
    double bounded_adaptive_rq_arw_q_scale_{1.0};
    double bounded_adaptive_rq_accbias_q_scale_{1.0};
    double bounded_adaptive_rq_gyrbias_q_scale_{1.0};
    bool bounded_adaptive_rq_have_prev_position_r_diag_{false};
    Eigen::Vector3d bounded_adaptive_rq_prev_position_r_diag_{
        Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())};
    ObservationDebugInfo last_observation_debug_{};
    bool state_update_debug_enabled_{false};
    std::uint64_t state_update_debug_sequence_{0};
    StateUpdateDebugInfo last_state_update_debug_event_{};
    StateUpdateDebugInfo last_gnss_position_state_update_debug_event_{};
    std::vector<StateUpdateDebugInfo> state_update_debug_events_{};
    Eigen::MatrixXd last_update_kalman_gain_{Eigen::MatrixXd()};

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
    double early_recovery_bias_feedback_context_armed_time_sec_{
        std::numeric_limits<double>::quiet_NaN()};
    double early_recovery_bias_feedback_context_core_gnss_u_m_{
        std::numeric_limits<double>::quiet_NaN()};
    std::deque<EarlyRecoveryBiasFeedbackSample> early_recovery_bias_feedback_history_{};

    // 状态ID和噪声ID
    // state ID and noise ID
    enum StateID { P_ID = 0, V_ID = 3, PHI_ID = 6, BG_ID = 9, BA_ID = 12, SG_ID = 15, SA_ID = 18 };
    enum NoiseID { VRW_ID = 0, ARW_ID = 3, BGSTD_ID = 6, BASTD_ID = 9, SGSTD_ID = 12, SASTD_ID = 15 };


};

#endif // GI_ENGINE_H
