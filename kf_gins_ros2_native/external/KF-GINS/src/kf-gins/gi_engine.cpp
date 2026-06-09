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

#include "common/earth.h"
#include "common/rotation.h"

#include <algorithm>
#include <cmath>

#include "gi_engine.h"
#include "insmech.h"

namespace {
double normalizeAngleRad(double rad) {
    return std::atan2(std::sin(rad), std::cos(rad));
}

bool imuErrorFinite(const ImuError &imuerror) {
    return imuerror.gyrbias.allFinite() &&
           imuerror.accbias.allFinite() &&
           imuerror.gyrscale.allFinite() &&
           imuerror.accscale.allFinite();
}

bool navStateFinite(const NavState &state) {
    return state.pos.allFinite() &&
           state.vel.allFinite() &&
           state.euler.allFinite() &&
           imuErrorFinite(state.imuerror);
}

bool imuSampleFinite(const IMU &imu) {
    return std::isfinite(imu.time) &&
           std::isfinite(imu.dt) &&
           imu.dtheta.allFinite() &&
           imu.dvel.allFinite() &&
           std::isfinite(imu.odovel);
}

double chiSquareThreshold(int dof, double prob) {
    const double p = std::clamp(prob, 0.0, 1.0);
    if (dof == 1) {
        if (std::abs(p - 0.90) <= 0.025) return 2.705543454095404;
        if (std::abs(p - 0.99) <= 0.025) return 6.6348966010212145;
        return 3.841458820694124;
    }
    if (dof == 2) {
        if (std::abs(p - 0.90) <= 0.025) return 4.605170185988092;
        if (std::abs(p - 0.99) <= 0.025) return 9.210340371976184;
        return 5.991464547107979;
    }
    if (dof == 3) {
        if (std::abs(p - 0.90) <= 0.025) return 6.251388631170325;
        if (std::abs(p - 0.99) <= 0.025) return 11.344866730144373;
        return 7.814727903251179;
    }
    return static_cast<double>(std::max(1, dof));
}
}  // namespace

GIEngine::GIEngine(GINSOptions &options) {

    this->options_ = options;
    options_.print_options();
    timestamp_ = 0;

    // 从配置中读取IEKF参数
    // read IEKF parameters from options
    iekf_max_iterations_       = options_.iekf_max_iterations;
    iekf_convergence_threshold_ = options_.iekf_convergence_threshold;

    // 从配置中读取 Sage-Husa 自适应参数
    use_sage_husa_            = options_.sage_husa.enable;
    sage_husa_alpha_          = options_.sage_husa.alpha;
    sage_husa_diag_only_      = options_.sage_husa.diag_only;
    sage_husa_min_var_factor_ = options_.sage_husa.min_var_factor;
    sage_husa_min_var_abs_    = options_.sage_husa.min_var_abs;
    sage_husa_k_              = 0;

    // 从配置中读取 NIS 约束有界自适应 R 参数，默认关闭以保持固定参数 baseline。
    bounded_adaptive_r_enable_ = options_.bounded_adaptive_r.enable;
    bounded_adaptive_r_mode_ = options_.bounded_adaptive_r.mode;
    bounded_adaptive_r_apply_position_ = options_.bounded_adaptive_r.apply_position;
    bounded_adaptive_r_apply_velocity_ = options_.bounded_adaptive_r.apply_velocity;
    bounded_adaptive_r_alpha_ =
        std::isfinite(options_.bounded_adaptive_r.alpha)
            ? std::max(0.0, options_.bounded_adaptive_r.alpha)
            : 1.0;
    bounded_adaptive_r_beta_ =
        std::isfinite(options_.bounded_adaptive_r.beta)
            ? std::clamp(options_.bounded_adaptive_r.beta, 0.0, 1.0)
            : 0.2;
    bounded_adaptive_r_gamma_min_ =
        std::isfinite(options_.bounded_adaptive_r.gamma_min)
            ? std::max(1.0, options_.bounded_adaptive_r.gamma_min)
            : 1.0;
    bounded_adaptive_r_gamma_max_ =
        std::isfinite(options_.bounded_adaptive_r.gamma_max)
            ? std::max(bounded_adaptive_r_gamma_min_, options_.bounded_adaptive_r.gamma_max)
            : std::max(bounded_adaptive_r_gamma_min_, 10.0);
    bounded_adaptive_r_chi2_prob_ =
        std::isfinite(options_.bounded_adaptive_r.chi2_prob)
            ? options_.bounded_adaptive_r.chi2_prob
            : 0.95;
    bounded_adaptive_r_chi2_threshold_3d_ =
        chiSquareThreshold(3, bounded_adaptive_r_chi2_prob_);
    bounded_adaptive_r_position_gamma_ = 1.0;
    bounded_adaptive_r_velocity_gamma_ = 1.0;

    // 从配置中读取 NIS 约束 R/Q selector 参数。默认关闭；开启时由 selector
    // 区分 GNSS 观测异常和过程扰动，避免把 wind/motion 误差都当作 R 放大。
    bounded_adaptive_rq_enable_ = options_.bounded_adaptive_rq.enable;
    bounded_adaptive_rq_mode_ = options_.bounded_adaptive_rq.mode;
    bounded_adaptive_rq_apply_position_ = options_.bounded_adaptive_rq.apply_position;
    bounded_adaptive_rq_apply_velocity_ = options_.bounded_adaptive_rq.apply_velocity;
    bounded_adaptive_rq_r_only_on_observation_disturbance_ =
        options_.bounded_adaptive_rq.r_only_on_observation_disturbance;
    bounded_adaptive_rq_q_on_process_disturbance_ =
        options_.bounded_adaptive_rq.q_on_process_disturbance;
    bounded_adaptive_rq_chi2_prob_ =
        std::isfinite(options_.bounded_adaptive_rq.chi2_prob)
            ? options_.bounded_adaptive_rq.chi2_prob
            : 0.95;
    bounded_adaptive_rq_chi2_threshold_3d_ =
        chiSquareThreshold(3, bounded_adaptive_rq_chi2_prob_);
    bounded_adaptive_rq_nis_ratio_start_ =
        std::isfinite(options_.bounded_adaptive_rq.nis_ratio_start)
            ? std::max(0.0, options_.bounded_adaptive_rq.nis_ratio_start)
            : 1.0;
    bounded_adaptive_rq_nis_ratio_full_ =
        std::isfinite(options_.bounded_adaptive_rq.nis_ratio_full)
            ? std::max(
                  bounded_adaptive_rq_nis_ratio_start_ + 1.0e-6,
                  options_.bounded_adaptive_rq.nis_ratio_full)
            : 3.0;
    bounded_adaptive_rq_consecutive_exceed_min_ =
        std::max(1, options_.bounded_adaptive_rq.consecutive_exceed_min);
    bounded_adaptive_rq_hold_updates_ =
        std::max(0, options_.bounded_adaptive_rq.hold_updates);
    bounded_adaptive_rq_alpha_r_ =
        std::isfinite(options_.bounded_adaptive_rq.alpha_r)
            ? std::max(0.0, options_.bounded_adaptive_rq.alpha_r)
            : 1.0;
    bounded_adaptive_rq_alpha_r_process_ =
        std::isfinite(options_.bounded_adaptive_rq.alpha_r_process)
            ? std::max(0.0, options_.bounded_adaptive_rq.alpha_r_process)
            : 0.25;
    bounded_adaptive_rq_alpha_q_ =
        std::isfinite(options_.bounded_adaptive_rq.alpha_q)
            ? std::max(0.0, options_.bounded_adaptive_rq.alpha_q)
            : 1.0;
    bounded_adaptive_rq_beta_r_ =
        std::isfinite(options_.bounded_adaptive_rq.beta_r)
            ? std::clamp(options_.bounded_adaptive_rq.beta_r, 0.0, 1.0)
            : 0.2;
    bounded_adaptive_rq_beta_q_ =
        std::isfinite(options_.bounded_adaptive_rq.beta_q)
            ? std::clamp(options_.bounded_adaptive_rq.beta_q, 0.0, 1.0)
            : 0.1;
    bounded_adaptive_rq_gamma_r_min_ =
        std::isfinite(options_.bounded_adaptive_rq.gamma_r_min)
            ? std::max(1.0, options_.bounded_adaptive_rq.gamma_r_min)
            : 1.0;
    bounded_adaptive_rq_gamma_r_max_observation_ =
        std::isfinite(options_.bounded_adaptive_rq.gamma_r_max_observation)
            ? std::max(
                  bounded_adaptive_rq_gamma_r_min_,
                  options_.bounded_adaptive_rq.gamma_r_max_observation)
            : 6.0;
    bounded_adaptive_rq_gamma_r_max_process_ =
        std::isfinite(options_.bounded_adaptive_rq.gamma_r_max_process)
            ? std::max(
                  bounded_adaptive_rq_gamma_r_min_,
                  options_.bounded_adaptive_rq.gamma_r_max_process)
            : 1.5;
    bounded_adaptive_rq_lambda_vrw_max_ =
        std::isfinite(options_.bounded_adaptive_rq.lambda_vrw_max)
            ? std::max(1.0, options_.bounded_adaptive_rq.lambda_vrw_max)
            : 3.0;
    bounded_adaptive_rq_lambda_arw_max_ =
        std::isfinite(options_.bounded_adaptive_rq.lambda_arw_max)
            ? std::max(1.0, options_.bounded_adaptive_rq.lambda_arw_max)
            : 1.5;
    bounded_adaptive_rq_lambda_accbias_max_ =
        std::isfinite(options_.bounded_adaptive_rq.lambda_accbias_max)
            ? std::max(1.0, options_.bounded_adaptive_rq.lambda_accbias_max)
            : 2.0;
    bounded_adaptive_rq_lambda_gyrbias_max_ =
        std::isfinite(options_.bounded_adaptive_rq.lambda_gyrbias_max)
            ? std::max(1.0, options_.bounded_adaptive_rq.lambda_gyrbias_max)
            : 1.0;
    bounded_adaptive_rq_require_gps_quality_stable_for_q_ =
        options_.bounded_adaptive_rq.require_gps_quality_stable_for_q;
    bounded_adaptive_rq_require_process_context_for_q_ =
        options_.bounded_adaptive_rq.require_process_context_for_q;
    bounded_adaptive_rq_process_context_score_start_ =
        std::isfinite(options_.bounded_adaptive_rq.process_context_score_start)
            ? std::clamp(options_.bounded_adaptive_rq.process_context_score_start, 0.0, 1.0)
            : 0.20;
    bounded_adaptive_rq_mixed_observation_score_start_ =
        std::isfinite(options_.bounded_adaptive_rq.mixed_observation_score_start)
            ? std::clamp(options_.bounded_adaptive_rq.mixed_observation_score_start, 0.0, 1.0)
            : 0.50;
    bounded_adaptive_rq_source_gate_enable_ =
        options_.bounded_adaptive_rq.source_gate_enable;
    bounded_adaptive_rq_q_source_observation_score_max_ =
        std::isfinite(options_.bounded_adaptive_rq.q_source_observation_score_max)
            ? std::clamp(options_.bounded_adaptive_rq.q_source_observation_score_max, 0.0, 1.0)
            : 1.0;
    bounded_adaptive_rq_velocity_evidence_gate_enable_ =
        options_.bounded_adaptive_rq.velocity_evidence_gate_enable;
    bounded_adaptive_rq_q_high_observation_velocity_nis_ratio_min_ =
        std::isfinite(options_.bounded_adaptive_rq.q_high_observation_velocity_nis_ratio_min)
            ? std::max(0.0, options_.bounded_adaptive_rq.q_high_observation_velocity_nis_ratio_min)
            : 2.0;
    bounded_adaptive_rq_q_high_observation_velocity_residual_h_min_ =
        std::isfinite(options_.bounded_adaptive_rq.q_high_observation_velocity_residual_h_min)
            ? std::max(0.0, options_.bounded_adaptive_rq.q_high_observation_velocity_residual_h_min)
            : 0.15;
    bounded_adaptive_rq_velocity_evidence_time_tolerance_sec_ =
        std::isfinite(options_.bounded_adaptive_rq.velocity_evidence_time_tolerance_sec)
            ? std::max(0.0, options_.bounded_adaptive_rq.velocity_evidence_time_tolerance_sec)
            : 0.5;
    resetBoundedAdaptiveRQScales_();

    // 设置协方差矩阵，系统噪声阵和系统误差状态矩阵大小
    // resize covariance matrix, system noise matrix, and system error state matrix
    Cov_.resize(RANK, RANK);
    Qc_.resize(NOISERANK, NOISERANK);
    dx_.resize(RANK, 1);
    Cov_.setZero();
    Qc_.setZero();
    dx_.setZero();



    // 初始化系统噪声阵
    // initialize noise matrix
    auto imunoise                   = options_.imunoise;
    Qc_.block(ARW_ID, ARW_ID, 3, 3) = imunoise.gyr_arw.cwiseProduct(imunoise.gyr_arw).asDiagonal();
    Qc_.block(VRW_ID, VRW_ID, 3, 3) = imunoise.acc_vrw.cwiseProduct(imunoise.acc_vrw).asDiagonal();
    Qc_.block(BGSTD_ID, BGSTD_ID, 3, 3) =
        2 / imunoise.corr_time * imunoise.gyrbias_std.cwiseProduct(imunoise.gyrbias_std).asDiagonal();
    Qc_.block(BASTD_ID, BASTD_ID, 3, 3) =
        2 / imunoise.corr_time * imunoise.accbias_std.cwiseProduct(imunoise.accbias_std).asDiagonal();
    Qc_.block(SGSTD_ID, SGSTD_ID, 3, 3) =
        2 / imunoise.corr_time * imunoise.gyrscale_std.cwiseProduct(imunoise.gyrscale_std).asDiagonal();
    Qc_.block(SASTD_ID, SASTD_ID, 3, 3) =
        2 / imunoise.corr_time * imunoise.accscale_std.cwiseProduct(imunoise.accscale_std).asDiagonal();

    // 初始化系统状态(位置、速度、姿态和IMU误差)初值和初始协方差
    // set initial state (position, velocity, attitude and IMU error) and covariance
    initialize(options_.initstate, options_.initstate_std);

    // 初始化持续更新的观测噪声阵
    R_gnsspos_.resize(3, 3);
    R_gnsspos_.setZero();

    R_gnsspos_init_.resize(3, 3);
    R_gnsspos_init_.setZero();
}

void GIEngine::initialize(const NavState &initstate, const NavState &initstate_std) {

    // 初始化位置、速度、姿态
    // initialize position, velocity and attitude
    pvacur_.pos       = initstate.pos;
    pvacur_.vel       = initstate.vel;
    pvacur_.att.euler = initstate.euler;
    pvacur_.att.cbn   = Rotation::euler2matrix(pvacur_.att.euler);
    pvacur_.att.qbn   = Rotation::euler2quaternion(pvacur_.att.euler);
    // 初始化IMU误差
    // initialize imu error
    imuerror_ = initstate.imuerror;

    // 给上一时刻状态赋同样的初值
    // set the same value to the previous state
    pvapre_ = pvacur_;

    // 初始化协方差
    // initialize covariance
    ImuError imuerror_std            = initstate_std.imuerror;
    Cov_.block(P_ID, P_ID, 3, 3)     = initstate_std.pos.cwiseProduct(initstate_std.pos).asDiagonal();
    Cov_.block(V_ID, V_ID, 3, 3)     = initstate_std.vel.cwiseProduct(initstate_std.vel).asDiagonal();
    Cov_.block(PHI_ID, PHI_ID, 3, 3) = initstate_std.euler.cwiseProduct(initstate_std.euler).asDiagonal();
    Cov_.block(BG_ID, BG_ID, 3, 3)   = imuerror_std.gyrbias.cwiseProduct(imuerror_std.gyrbias).asDiagonal();
    Cov_.block(BA_ID, BA_ID, 3, 3)   = imuerror_std.accbias.cwiseProduct(imuerror_std.accbias).asDiagonal();
    Cov_.block(SG_ID, SG_ID, 3, 3)   = imuerror_std.gyrscale.cwiseProduct(imuerror_std.gyrscale).asDiagonal();
    Cov_.block(SA_ID, SA_ID, 3, 3)   = imuerror_std.accscale.cwiseProduct(imuerror_std.accscale).asDiagonal();
}

void GIEngine::setPropagationNoiseScale(
    double arw_q_scale, double vrw_q_scale,
    double gyrbias_q_scale, double accbias_q_scale) {

    auto sanitize_scale = [](double value) {
        if (!std::isfinite(value)) {
            return 1.0;
        }
        return std::clamp(std::abs(value), 1.0, 100.0);
    };

    propagation_arw_q_scale_     = sanitize_scale(arw_q_scale);
    propagation_vrw_q_scale_     = sanitize_scale(vrw_q_scale);
    propagation_gyrbias_q_scale_ = sanitize_scale(gyrbias_q_scale);
    propagation_accbias_q_scale_ = sanitize_scale(accbias_q_scale);
}

void GIEngine::setAdaptiveRQProcessContext(bool valid, double score) {

    bounded_adaptive_rq_process_context_valid_ = valid && std::isfinite(score);
    bounded_adaptive_rq_process_context_score_ =
        bounded_adaptive_rq_process_context_valid_
            ? std::clamp(score, 0.0, 1.0)
            : 0.0;
}

void GIEngine::setAdaptiveRQSourceGate(
    bool allowed, double confidence, const std::string &reason) {

    bounded_adaptive_rq_source_gate_allowed_ = allowed;
    bounded_adaptive_rq_source_confidence_ =
        std::isfinite(confidence) ? std::clamp(confidence, 0.0, 1.0) : 0.0;
    bounded_adaptive_rq_source_gate_reason_ =
        reason.empty() ? (allowed ? "allowed" : "blocked") : reason;
}

void GIEngine::configureEarlyRecoveryBiasFeedback(
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
    double negative_dx_scale) {

    early_recovery_bias_feedback_debug_enable_ = debug_enable;
    early_recovery_bias_feedback_apply_enable_ = apply_enable;
    early_recovery_bias_feedback_history_sec_ =
        std::max(0.1, std::abs(history_sec));
    early_recovery_bias_feedback_min_armed_time_sec_ = min_armed_time_sec;
    early_recovery_bias_feedback_max_armed_time_sec_ = max_armed_time_sec;
    early_recovery_bias_feedback_ba_z_mean_max_mps2_ = ba_z_mean_max_mps2;
    early_recovery_bias_feedback_residual_u_mean_max_m_ = residual_u_mean_max_m;
    early_recovery_bias_feedback_core_gnss_u_mean_min_m_ = core_gnss_u_mean_min_m;
    early_recovery_bias_feedback_dx_ba_z_sum_max_mps2_ = dx_ba_z_sum_max_mps2;
    early_recovery_bias_feedback_min_history_rows_ = std::max(1, min_history_rows);
    early_recovery_bias_feedback_negative_dx_scale_ =
        std::clamp(negative_dx_scale, 0.0, 1.0);
    if (!early_recovery_bias_feedback_debug_enable_ &&
        !early_recovery_bias_feedback_apply_enable_) {
        early_recovery_bias_feedback_history_.clear();
    }
}

void GIEngine::setEarlyRecoveryBiasFeedbackContext(
    double armed_time_sec, double core_gnss_u_m) {
    early_recovery_bias_feedback_context_armed_time_sec_ = armed_time_sec;
    early_recovery_bias_feedback_context_core_gnss_u_m_ = core_gnss_u_m;
}

void GIEngine::newImuProcess() {

    // 当前IMU时间作为系统当前状态时间,
    // set current IMU time as the current state time
    timestamp_ = imucur_.time;

    // 如果GNSS有效，则将更新时间设置为GNSS时间
    // set update time as the gnss time if gnssdata is valid
    double updatetime = gnssdata_.isvalid ? gnssdata_.time : -1;

    // 判断是否需要进行GNSS更新
    // determine if we should do GNSS update
    int res = isToUpdate(imupre_.time, imucur_.time, updatetime);

    if (res == 0) {
        // 只传播导航状态
        // only propagate navigation state
        insPropagation(imupre_, imucur_);
    } else if (res == 1) {
        beginObservationDebug(res, gnssdata_.time);
        // GNSS数据靠近上一历元，先对上一历元进行GNSS更新
        // gnssdata is near to the previous imudata, we should firstly do gnss update
        const PVA pva_before = pvacur_;
        const ImuError imuerror_before = imuerror_;
        const Eigen::MatrixXd covariance_before = Cov_;
        gnssUpdate(gnssdata_);
        feedbackAndRecordStateUpdate(
            "gnss_position",
            last_observation_debug_.gnss_position_update_reason,
            gnssdata_.time,
            res,
            pva_before,
            imuerror_before,
            covariance_before);
        // 速度观测更新（若有）—— 在位置更新后立即应用
        if (has_vel_obs_) {
            const double velocity_update_time = vel_obs_time_;
            const PVA vel_pva_before = pvacur_;
            const ImuError vel_imuerror_before = imuerror_;
            const Eigen::MatrixXd vel_covariance_before = Cov_;
            gnssVelUpdate();
            feedbackAndRecordStateUpdate(
                "gnss_velocity",
                "applied",
                velocity_update_time,
                res,
                vel_pva_before,
                vel_imuerror_before,
                vel_covariance_before);
        }

        pvapre_ = pvacur_;
        insPropagation(imupre_, imucur_);
    } else if (res == 2) {
        beginObservationDebug(res, gnssdata_.time);
        // GNSS数据靠近当前历元，先对当前IMU进行状态传播
        // gnssdata is near current imudata, we should firstly propagate navigation state
        insPropagation(imupre_, imucur_);
        const PVA pva_before = pvacur_;
        const ImuError imuerror_before = imuerror_;
        const Eigen::MatrixXd covariance_before = Cov_;
        gnssUpdate(gnssdata_);
        feedbackAndRecordStateUpdate(
            "gnss_position",
            last_observation_debug_.gnss_position_update_reason,
            gnssdata_.time,
            res,
            pva_before,
            imuerror_before,
            covariance_before);
        // 速度观测更新（若有）
        if (has_vel_obs_) {
            const double velocity_update_time = vel_obs_time_;
            const PVA vel_pva_before = pvacur_;
            const ImuError vel_imuerror_before = imuerror_;
            const Eigen::MatrixXd vel_covariance_before = Cov_;
            gnssVelUpdate();
            feedbackAndRecordStateUpdate(
                "gnss_velocity",
                "applied",
                velocity_update_time,
                res,
                vel_pva_before,
                vel_imuerror_before,
                vel_covariance_before);
        }
    } else {
        // GNSS数据在两个IMU数据之间(不靠近任何一个), 将当前IMU内插到整秒时刻
        // gnssdata is between the two imudata, we interpolate current imudata to gnss time
        IMU midimu;
        imuInterpolate(imupre_, imucur_, updatetime, midimu);
        beginObservationDebug(res, gnssdata_.time);

        // 对前一半IMU进行状态传播
        // propagate navigation state for the first half imudata
        insPropagation(imupre_, midimu);

        // 整秒时刻进行GNSS更新，并反馈系统状态
        // do GNSS position update at the whole second and feedback system states
        const PVA pva_before = pvacur_;
        const ImuError imuerror_before = imuerror_;
        const Eigen::MatrixXd covariance_before = Cov_;
        gnssUpdate(gnssdata_);
        feedbackAndRecordStateUpdate(
            "gnss_position",
            last_observation_debug_.gnss_position_update_reason,
            gnssdata_.time,
            res,
            pva_before,
            imuerror_before,
            covariance_before);
        // 速度观测更新（若有）
        if (has_vel_obs_) {
            const double velocity_update_time = vel_obs_time_;
            const PVA vel_pva_before = pvacur_;
            const ImuError vel_imuerror_before = imuerror_;
            const Eigen::MatrixXd vel_covariance_before = Cov_;
            gnssVelUpdate();
            feedbackAndRecordStateUpdate(
                "gnss_velocity",
                "applied",
                velocity_update_time,
                res,
                vel_pva_before,
                vel_imuerror_before,
                vel_covariance_before);
        }

        // 对后一半IMU进行状态传播
        // propagate navigation state for the second half imudata
        pvapre_ = pvacur_;
        insPropagation(midimu, imucur_);
    }

    // 检查协方差矩阵对角线元素
    // check diagonal elements of current covariance matrix
    checkCov();

    // 更新上一时刻的状态和IMU数据
    // update system state and imudata at the previous epoch
    pvapre_ = pvacur_;
    imupre_ = imucur_;
}

void GIEngine::beginObservationDebug(int update_mode, double update_time_sec) {

    last_observation_debug_.sequence += 1;
    last_observation_debug_.valid = true;
    last_observation_debug_.gnss_position_applied = false;
    last_observation_debug_.gnss_velocity_applied = false;
    last_observation_debug_.update_time_sec = update_time_sec;
    last_observation_debug_.update_mode = update_mode;
    last_observation_debug_.gnss_position_residual_neu_m =
        Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    last_observation_debug_.gnss_position_std_neu_m =
        Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    last_observation_debug_.gnss_position_innovation_cov_neu_m2 =
        Eigen::Matrix3d::Constant(std::numeric_limits<double>::quiet_NaN());
    last_observation_debug_.gnss_position_nis_h_2d =
        std::numeric_limits<double>::quiet_NaN();
    last_observation_debug_.gnss_position_nis_u_1d =
        std::numeric_limits<double>::quiet_NaN();
    last_observation_debug_.gnss_position_nis_3d =
        std::numeric_limits<double>::quiet_NaN();
    last_observation_debug_.gnss_position_gate_threshold_nis =
        std::numeric_limits<double>::quiet_NaN();
    last_observation_debug_.gnss_position_update_accepted = false;
    last_observation_debug_.gnss_position_update_rejected = false;
    last_observation_debug_.gnss_position_update_reason = "not_available";
    last_observation_debug_.gnss_velocity_residual_ned_mps =
        Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    last_observation_debug_.gnss_velocity_std_ned_mps =
        Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    last_observation_debug_.gnss_position_adaptive_r = BoundedAdaptiveRDebugInfo{};
    last_observation_debug_.gnss_velocity_adaptive_r = BoundedAdaptiveRDebugInfo{};
}

double GIEngine::computeNis_(const Eigen::MatrixXd &S, const Eigen::MatrixXd &r) const {

    if (S.rows() != S.cols() || S.rows() != r.rows() || r.cols() != 1) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const auto ldlt = S.ldlt();
    if (ldlt.info() != Eigen::Success) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const Eigen::MatrixXd solved = ldlt.solve(r);
    if (solved.rows() != r.rows() || solved.cols() != 1 || !solved.allFinite()) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const double nis = (r.transpose() * solved)(0, 0);
    return std::isfinite(nis) ? nis : std::numeric_limits<double>::quiet_NaN();
}

Eigen::MatrixXd GIEngine::boundedAdaptiveREffective_(
    const std::string &update_type,
    bool apply_enabled,
    double nis,
    const Eigen::MatrixXd &R_base,
    double &gamma_state,
    BoundedAdaptiveRDebugInfo &debug) {

    debug = BoundedAdaptiveRDebugInfo{};
    debug.enabled = bounded_adaptive_r_enable_;
    debug.update_type = update_type;
    debug.mode = bounded_adaptive_r_enable_ ? bounded_adaptive_r_mode_ : "disabled";
    debug.nis = nis;
    debug.chi2_threshold = bounded_adaptive_r_chi2_threshold_3d_;
    debug.gamma_raw = 1.0;
    debug.gamma_smoothed = gamma_state;
    debug.gamma_clipped = gamma_state;

    Eigen::MatrixXd R_eff = R_base;
    if (R_base.rows() >= 3 && R_base.cols() >= 3) {
        debug.r_base_diag = R_base.diagonal().head(3);
        debug.r_eff_diag = debug.r_base_diag;
    }

    const bool usable =
        bounded_adaptive_r_enable_ &&
        bounded_adaptive_r_mode_ == "nis_bounded" &&
        apply_enabled &&
        R_base.rows() == R_base.cols() &&
        R_base.rows() > 0 &&
        std::isfinite(nis) &&
        std::isfinite(bounded_adaptive_r_chi2_threshold_3d_) &&
        bounded_adaptive_r_chi2_threshold_3d_ > 0.0;
    if (!usable) {
        gamma_state = 1.0;
        debug.gamma_smoothed = gamma_state;
        debug.gamma_clipped = gamma_state;
        return R_eff;
    }

    debug.applied = true;
    debug.exceeded = nis > bounded_adaptive_r_chi2_threshold_3d_;
    if (debug.exceeded) {
        const double ratio = nis / bounded_adaptive_r_chi2_threshold_3d_;
        debug.gamma_raw = 1.0 + bounded_adaptive_r_alpha_ * std::max(0.0, ratio - 1.0);
    }

    const double gamma_previous =
        std::isfinite(gamma_state) ? gamma_state : 1.0;
    debug.gamma_smoothed =
        (1.0 - bounded_adaptive_r_beta_) * gamma_previous +
        bounded_adaptive_r_beta_ * debug.gamma_raw;
    if (!std::isfinite(debug.gamma_smoothed)) {
        debug.gamma_smoothed = 1.0;
    }
    debug.gamma_clipped =
        std::clamp(
            debug.gamma_smoothed,
            bounded_adaptive_r_gamma_min_,
            bounded_adaptive_r_gamma_max_);
    gamma_state = debug.gamma_clipped;

    R_eff = debug.gamma_clipped * R_base;
    if (R_eff.rows() >= 3 && R_eff.cols() >= 3) {
        debug.r_eff_diag = R_eff.diagonal().head(3);
    }
    return R_eff;
}

void GIEngine::resetBoundedAdaptiveRQScales_() {

    bounded_adaptive_rq_position_gamma_ = 1.0;
    bounded_adaptive_rq_velocity_gamma_ = 1.0;
    bounded_adaptive_rq_vrw_q_scale_ = 1.0;
    bounded_adaptive_rq_arw_q_scale_ = 1.0;
    bounded_adaptive_rq_accbias_q_scale_ = 1.0;
    bounded_adaptive_rq_gyrbias_q_scale_ = 1.0;
    bounded_adaptive_rq_consecutive_exceed_count_ = 0;
    bounded_adaptive_rq_hold_remaining_ = 0;
    bounded_adaptive_rq_process_context_valid_ = false;
    bounded_adaptive_rq_process_context_score_ = 0.0;
    bounded_adaptive_rq_source_gate_allowed_ = !bounded_adaptive_rq_source_gate_enable_;
    bounded_adaptive_rq_source_confidence_ = bounded_adaptive_rq_source_gate_enable_ ? 0.0 : 1.0;
    bounded_adaptive_rq_source_gate_reason_ =
        bounded_adaptive_rq_source_gate_enable_ ? "reset" : "disabled";
    bounded_adaptive_rq_velocity_evidence_valid_ = false;
    bounded_adaptive_rq_velocity_evidence_active_ = false;
    bounded_adaptive_rq_velocity_evidence_nis_ratio_ =
        std::numeric_limits<double>::quiet_NaN();
    bounded_adaptive_rq_velocity_evidence_residual_h_mps_ =
        std::numeric_limits<double>::quiet_NaN();
    bounded_adaptive_rq_have_prev_position_r_diag_ = false;
    bounded_adaptive_rq_prev_position_r_diag_.setConstant(
        std::numeric_limits<double>::quiet_NaN());
}

void GIEngine::updateBoundedAdaptiveRQProcessScales_(
    bool process_triggered,
    double nis_ratio,
    BoundedAdaptiveRDebugInfo &debug) {

    const double e = std::isfinite(nis_ratio)
        ? std::max(0.0, nis_ratio - bounded_adaptive_rq_nis_ratio_start_)
        : 0.0;
    const double lambda_raw =
        (process_triggered && bounded_adaptive_rq_q_on_process_disturbance_)
            ? (1.0 + bounded_adaptive_rq_alpha_q_ * e)
            : 1.0;
    auto smooth_and_clip = [&](double previous, double upper) {
        const double smoothed =
            (1.0 - bounded_adaptive_rq_beta_q_) *
                (std::isfinite(previous) ? previous : 1.0) +
            bounded_adaptive_rq_beta_q_ * lambda_raw;
        return std::clamp(
            std::isfinite(smoothed) ? smoothed : 1.0,
            1.0,
            std::max(1.0, upper));
    };

    bounded_adaptive_rq_vrw_q_scale_ =
        smooth_and_clip(bounded_adaptive_rq_vrw_q_scale_,
                        bounded_adaptive_rq_lambda_vrw_max_);
    bounded_adaptive_rq_arw_q_scale_ =
        smooth_and_clip(bounded_adaptive_rq_arw_q_scale_,
                        bounded_adaptive_rq_lambda_arw_max_);
    bounded_adaptive_rq_accbias_q_scale_ =
        smooth_and_clip(bounded_adaptive_rq_accbias_q_scale_,
                        bounded_adaptive_rq_lambda_accbias_max_);
    bounded_adaptive_rq_gyrbias_q_scale_ =
        smooth_and_clip(bounded_adaptive_rq_gyrbias_q_scale_,
                        bounded_adaptive_rq_lambda_gyrbias_max_);

    debug.q_lambda_vrw = bounded_adaptive_rq_vrw_q_scale_;
    debug.q_lambda_arw = bounded_adaptive_rq_arw_q_scale_;
    debug.q_lambda_accbias = bounded_adaptive_rq_accbias_q_scale_;
    debug.q_lambda_gyrbias = bounded_adaptive_rq_gyrbias_q_scale_;
}

Eigen::MatrixXd GIEngine::boundedAdaptiveRQEffective_(
    const std::string &update_type,
    bool apply_enabled,
    bool update_process_q,
    double nis,
    const Eigen::MatrixXd &R_base,
    double &gamma_state,
    BoundedAdaptiveRDebugInfo &debug) {

    debug = BoundedAdaptiveRDebugInfo{};
    debug.enabled = bounded_adaptive_rq_enable_;
    debug.update_type = update_type;
    debug.mode = bounded_adaptive_rq_enable_ ? bounded_adaptive_rq_mode_ : "disabled";
    debug.nis = nis;
    debug.chi2_threshold = bounded_adaptive_rq_chi2_threshold_3d_;
    debug.gamma_raw = 1.0;
    debug.gamma_smoothed = std::isfinite(gamma_state) ? gamma_state : 1.0;
    debug.gamma_clipped = debug.gamma_smoothed;
    debug.r_gamma_limit = bounded_adaptive_rq_gamma_r_min_;
    debug.q_lambda_vrw = bounded_adaptive_rq_vrw_q_scale_;
    debug.q_lambda_arw = bounded_adaptive_rq_arw_q_scale_;
    debug.q_lambda_accbias = bounded_adaptive_rq_accbias_q_scale_;
    debug.q_lambda_gyrbias = bounded_adaptive_rq_gyrbias_q_scale_;

    Eigen::MatrixXd R_eff = R_base;
    if (R_base.rows() >= 3 && R_base.cols() >= 3) {
        debug.r_base_diag = R_base.diagonal().head(3);
        debug.r_eff_diag = debug.r_base_diag;
    }

    const bool usable =
        bounded_adaptive_rq_enable_ &&
        bounded_adaptive_rq_mode_ == "nis_rq_selector" &&
        apply_enabled &&
        R_base.rows() == R_base.cols() &&
        R_base.rows() > 0 &&
        std::isfinite(nis) &&
        std::isfinite(bounded_adaptive_rq_chi2_threshold_3d_) &&
        bounded_adaptive_rq_chi2_threshold_3d_ > 0.0;
    if (!usable) {
        if (!bounded_adaptive_rq_enable_) {
            resetBoundedAdaptiveRQScales_();
        } else if (!apply_enabled) {
            debug.rq_selector_reason = "apply_disabled";
        } else {
            debug.rq_selector_reason = "unusable";
        }
        gamma_state = 1.0;
        debug.gamma_smoothed = gamma_state;
        debug.gamma_clipped = gamma_state;
        debug.r_gamma_limit = 1.0;
        return R_eff;
    }

    debug.applied = true;
    debug.rq_selector_reason = "active";
    debug.nis_ratio = nis / bounded_adaptive_rq_chi2_threshold_3d_;
    debug.exceeded = debug.nis_ratio > bounded_adaptive_rq_nis_ratio_start_;
    debug.observation_score = debug.exceeded
        ? std::clamp(
              (debug.nis_ratio - bounded_adaptive_rq_nis_ratio_start_) /
                  (bounded_adaptive_rq_nis_ratio_full_ -
                   bounded_adaptive_rq_nis_ratio_start_),
              0.0,
              1.0)
        : 0.0;

    bool gps_quality_stable = true;
    if (update_process_q && R_base.rows() >= 3 && R_base.cols() >= 3) {
        const Eigen::Vector3d r_diag = R_base.diagonal().head(3);
        if (bounded_adaptive_rq_require_gps_quality_stable_for_q_ &&
            bounded_adaptive_rq_have_prev_position_r_diag_) {
            double max_rel_change = 0.0;
            for (int i = 0; i < 3; ++i) {
                const double prev = bounded_adaptive_rq_prev_position_r_diag_(i);
                const double cur = r_diag(i);
                if (std::isfinite(prev) && std::isfinite(cur) &&
                    std::abs(prev) > 1.0e-12) {
                    max_rel_change =
                        std::max(max_rel_change, std::abs(cur - prev) / std::abs(prev));
                }
            }
            gps_quality_stable = max_rel_change <= 0.20;
        }
        bounded_adaptive_rq_prev_position_r_diag_ = r_diag;
        bounded_adaptive_rq_have_prev_position_r_diag_ = true;
    }
    const bool process_context_ok =
        !bounded_adaptive_rq_require_process_context_for_q_ ||
        (bounded_adaptive_rq_process_context_valid_ &&
         bounded_adaptive_rq_process_context_score_ >=
             bounded_adaptive_rq_process_context_score_start_);
    debug.gps_quality_stable = gps_quality_stable;
    debug.motion_context_ok = process_context_ok;

    bool q_source_context_allowed = true;
    bool q_source_gate_allowed = true;
    std::string q_source_gate_reason =
        bounded_adaptive_rq_source_gate_enable_
            ? bounded_adaptive_rq_source_gate_reason_
            : "disabled";
    if (bounded_adaptive_rq_source_gate_enable_) {
        q_source_context_allowed =
            bounded_adaptive_rq_source_gate_allowed_ &&
            bounded_adaptive_rq_source_confidence_ > 0.0;
        q_source_gate_allowed = q_source_context_allowed;
        if (!q_source_context_allowed && q_source_gate_reason.empty()) {
            q_source_gate_reason = "source_context_blocked";
        }
        if (q_source_gate_allowed &&
            debug.observation_score >
                bounded_adaptive_rq_q_source_observation_score_max_) {
            if (bounded_adaptive_rq_velocity_evidence_gate_enable_ &&
                bounded_adaptive_rq_velocity_evidence_active_) {
                q_source_gate_reason = "velocity_evidence_high_observation";
            } else {
                q_source_gate_allowed = false;
                q_source_gate_reason = "observation_score_high";
            }
        }
    }
    debug.q_source_confidence =
        bounded_adaptive_rq_source_gate_enable_
            ? bounded_adaptive_rq_source_confidence_
            : 1.0;
    debug.q_velocity_evidence = bounded_adaptive_rq_velocity_evidence_active_;
    debug.q_velocity_nis_ratio = bounded_adaptive_rq_velocity_evidence_nis_ratio_;
    debug.q_velocity_residual_h_mps =
        bounded_adaptive_rq_velocity_evidence_residual_h_mps_;
    debug.q_source_gate_allowed = q_source_gate_allowed;
    debug.q_source_gate_reason = q_source_gate_reason;

    if (update_process_q) {
        if (debug.exceeded) {
            ++bounded_adaptive_rq_consecutive_exceed_count_;
        } else {
            bounded_adaptive_rq_consecutive_exceed_count_ = 0;
        }
        if (bounded_adaptive_rq_hold_remaining_ > 0) {
            --bounded_adaptive_rq_hold_remaining_;
        }
    }
    debug.consecutive_exceed_count = bounded_adaptive_rq_consecutive_exceed_count_;

    const bool process_now =
        update_process_q &&
        bounded_adaptive_rq_q_on_process_disturbance_ &&
        debug.exceeded &&
        gps_quality_stable &&
        process_context_ok &&
        q_source_gate_allowed &&
        bounded_adaptive_rq_consecutive_exceed_count_ >=
            bounded_adaptive_rq_consecutive_exceed_min_;
    if (process_now) {
        bounded_adaptive_rq_hold_remaining_ =
            std::max(bounded_adaptive_rq_hold_remaining_,
                     bounded_adaptive_rq_hold_updates_);
    } else if (update_process_q && bounded_adaptive_rq_source_gate_enable_ &&
               !q_source_context_allowed) {
        bounded_adaptive_rq_hold_remaining_ = 0;
        bounded_adaptive_rq_vrw_q_scale_ = 1.0;
        bounded_adaptive_rq_arw_q_scale_ = 1.0;
        bounded_adaptive_rq_accbias_q_scale_ = 1.0;
        bounded_adaptive_rq_gyrbias_q_scale_ = 1.0;
    } else if (update_process_q && bounded_adaptive_rq_source_gate_enable_ &&
               !q_source_gate_allowed) {
        bounded_adaptive_rq_hold_remaining_ = 0;
    }
    const bool process_triggered =
        process_now ||
        (update_process_q &&
         (!bounded_adaptive_rq_source_gate_enable_ || q_source_context_allowed) &&
         bounded_adaptive_rq_hold_remaining_ > 0);
    debug.hold_remaining = bounded_adaptive_rq_hold_remaining_;
    debug.process_score = process_triggered ? 1.0 : 0.0;

    const double ratio_error =
        std::isfinite(debug.nis_ratio)
            ? std::max(0.0, debug.nis_ratio - bounded_adaptive_rq_nis_ratio_start_)
            : 0.0;
    const bool mixed_triggered =
        process_triggered &&
        debug.observation_score >= bounded_adaptive_rq_mixed_observation_score_start_;
    if (mixed_triggered) {
        debug.rq_selector_trigger = "mixed_disturbance";
        debug.r_gamma_limit = bounded_adaptive_rq_gamma_r_max_observation_;
        debug.gamma_raw = 1.0 + bounded_adaptive_rq_alpha_r_ * ratio_error;
    } else if (process_triggered) {
        debug.rq_selector_trigger = "process_disturbance";
        debug.r_gamma_limit = bounded_adaptive_rq_gamma_r_max_process_;
        debug.gamma_raw = 1.0 + bounded_adaptive_rq_alpha_r_process_ * ratio_error;
    } else if (debug.exceeded &&
               bounded_adaptive_rq_r_only_on_observation_disturbance_) {
        debug.rq_selector_trigger = "observation_disturbance";
        debug.r_gamma_limit = bounded_adaptive_rq_gamma_r_max_observation_;
        debug.gamma_raw = 1.0 + bounded_adaptive_rq_alpha_r_ * ratio_error;
    } else {
        debug.rq_selector_trigger = "none";
        debug.r_gamma_limit = bounded_adaptive_rq_gamma_r_min_;
        debug.gamma_raw = 1.0;
    }

    const double gamma_previous = std::isfinite(gamma_state) ? gamma_state : 1.0;
    debug.gamma_smoothed =
        (1.0 - bounded_adaptive_rq_beta_r_) * gamma_previous +
        bounded_adaptive_rq_beta_r_ * debug.gamma_raw;
    if (!std::isfinite(debug.gamma_smoothed)) {
        debug.gamma_smoothed = 1.0;
    }
    debug.gamma_clipped =
        std::clamp(
            debug.gamma_smoothed,
            bounded_adaptive_rq_gamma_r_min_,
            std::max(bounded_adaptive_rq_gamma_r_min_, debug.r_gamma_limit));
    gamma_state = debug.gamma_clipped;

    if (update_process_q) {
        updateBoundedAdaptiveRQProcessScales_(
            process_triggered, debug.nis_ratio, debug);
    }

    R_eff = debug.gamma_clipped * R_base;
    if (R_eff.rows() >= 3 && R_eff.cols() >= 3) {
        debug.r_eff_diag = R_eff.diagonal().head(3);
    }
    return R_eff;
}

void GIEngine::imuCompensate(IMU &imu) {

    // 补偿IMU零偏
    // compensate the imu bias
    imu.dtheta -= imuerror_.gyrbias * imu.dt;
    imu.dvel -= imuerror_.accbias * imu.dt;

    // 补偿IMU比例因子
    // compensate the imu scale
    Eigen::Vector3d gyrscale, accscale;
    gyrscale   = Eigen::Vector3d::Ones() + imuerror_.gyrscale;
    accscale   = Eigen::Vector3d::Ones() + imuerror_.accscale;
    imu.dtheta = imu.dtheta.cwiseProduct(gyrscale.cwiseInverse());
    imu.dvel   = imu.dvel.cwiseProduct(accscale.cwiseInverse());
}

void GIEngine::insPropagation(IMU &imupre, IMU &imucur) {

    // 对当前IMU数据(imucur)补偿误差, 上一IMU数据(imupre)已经补偿过了
    // compensate imu error to 'imucur', 'imupre' has been compensated
    imuCompensate(imucur);
    // IMU状态更新(机械编排算法)
    // update imustate(mechanization)
    INSMech::insMech(pvapre_, pvacur_, imupre, imucur);

    // 系统噪声传播，姿态误差采用phi角误差模型
    // system noise propagate, phi-angle error model for attitude error
    Eigen::MatrixXd Phi, F, Qd, G;

    // 初始化Phi阵(状态转移矩阵)，F阵，Qd阵(传播噪声阵)，G阵(噪声驱动阵)
    // initialize Phi (state transition), F matrix, Qd(propagation noise) and G(noise driven) matrix
    Phi.resizeLike(Cov_);
    F.resizeLike(Cov_);
    Qd.resizeLike(Cov_);
    G.resize(RANK, NOISERANK);
    Phi.setIdentity();
    F.setZero();
    Qd.setZero();
    G.setZero();

    // 使用上一历元状态计算状态转移矩阵
    // compute state transition matrix using the previous state
    Eigen::Vector2d rmrn;
    Eigen::Vector3d wie_n, wen_n;
    double gravity;
    rmrn    = Earth::meridianPrimeVerticalRadius(pvapre_.pos[0]);
    gravity = Earth::gravity(pvapre_.pos);
    wie_n << WGS84_WIE * cos(pvapre_.pos[0]), 0, -WGS84_WIE * sin(pvapre_.pos[0]);
    wen_n << pvapre_.vel[1] / (rmrn[1] + pvapre_.pos[2]), -pvapre_.vel[0] / (rmrn[0] + pvapre_.pos[2]),
        -pvapre_.vel[1] * tan(pvapre_.pos[0]) / (rmrn[1] + pvapre_.pos[2]);

    Eigen::Matrix3d temp;
    Eigen::Vector3d accel, omega;
    double rmh, rnh;

    rmh   = rmrn[0] + pvapre_.pos[2];
    rnh   = rmrn[1] + pvapre_.pos[2];
    accel = imucur.dvel / imucur.dt;
    omega = imucur.dtheta / imucur.dt;

    // 位置误差
    // position error
    temp.setZero();
    temp(0, 0)                = -pvapre_.vel[2] / rmh;
    temp(0, 2)                = pvapre_.vel[0] / rmh;
    temp(1, 0)                = pvapre_.vel[1] * tan(pvapre_.pos[0]) / rnh;
    temp(1, 1)                = -(pvapre_.vel[2] + pvapre_.vel[0] * tan(pvapre_.pos[0])) / rnh;
    temp(1, 2)                = pvapre_.vel[1] / rnh;
    F.block(P_ID, P_ID, 3, 3) = temp;
    F.block(P_ID, V_ID, 3, 3) = Eigen::Matrix3d::Identity();

    // 速度误差
    // velocity error
    temp.setZero();
    temp(0, 0) = -2 * pvapre_.vel[1] * WGS84_WIE * cos(pvapre_.pos[0]) / rmh -
                 pow(pvapre_.vel[1], 2) / rmh / rnh / pow(cos(pvapre_.pos[0]), 2);
    temp(0, 2) = pvapre_.vel[0] * pvapre_.vel[2] / rmh / rmh - pow(pvapre_.vel[1], 2) * tan(pvapre_.pos[0]) / rnh / rnh;
    temp(1, 0) = 2 * WGS84_WIE * (pvapre_.vel[0] * cos(pvapre_.pos[0]) - pvapre_.vel[2] * sin(pvapre_.pos[0])) / rmh +
                 pvapre_.vel[0] * pvapre_.vel[1] / rmh / rnh / pow(cos(pvapre_.pos[0]), 2);
    temp(1, 2) = (pvapre_.vel[1] * pvapre_.vel[2] + pvapre_.vel[0] * pvapre_.vel[1] * tan(pvapre_.pos[0])) / rnh / rnh;
    temp(2, 0) = 2 * WGS84_WIE * pvapre_.vel[1] * sin(pvapre_.pos[0]) / rmh;
    temp(2, 2) = -pow(pvapre_.vel[1], 2) / rnh / rnh - pow(pvapre_.vel[0], 2) / rmh / rmh +
                 2 * gravity / (sqrt(rmrn[0] * rmrn[1]) + pvapre_.pos[2]);
    F.block(V_ID, P_ID, 3, 3) = temp;
    temp.setZero();
    temp(0, 0)                  = pvapre_.vel[2] / rmh;
    temp(0, 1)                  = -2 * (WGS84_WIE * sin(pvapre_.pos[0]) + pvapre_.vel[1] * tan(pvapre_.pos[0]) / rnh);
    temp(0, 2)                  = pvapre_.vel[0] / rmh;
    temp(1, 0)                  = 2 * WGS84_WIE * sin(pvapre_.pos[0]) + pvapre_.vel[1] * tan(pvapre_.pos[0]) / rnh;
    temp(1, 1)                  = (pvapre_.vel[2] + pvapre_.vel[0] * tan(pvapre_.pos[0])) / rnh;
    temp(1, 2)                  = 2 * WGS84_WIE * cos(pvapre_.pos[0]) + pvapre_.vel[1] / rnh;
    temp(2, 0)                  = -2 * pvapre_.vel[0] / rmh;
    temp(2, 1)                  = -2 * (WGS84_WIE * cos(pvapre_.pos(0)) + pvapre_.vel[1] / rnh);
    F.block(V_ID, V_ID, 3, 3)   = temp;
    F.block(V_ID, PHI_ID, 3, 3) = Rotation::skewSymmetric(pvapre_.att.cbn * accel);
    F.block(V_ID, BA_ID, 3, 3)  = pvapre_.att.cbn;
    F.block(V_ID, SA_ID, 3, 3)  = pvapre_.att.cbn * (accel.asDiagonal());

    // 姿态误差
    // attitude error
    temp.setZero();
    temp(0, 0) = -WGS84_WIE * sin(pvapre_.pos[0]) / rmh;
    temp(0, 2) = pvapre_.vel[1] / rnh / rnh;
    temp(1, 2) = -pvapre_.vel[0] / rmh / rmh;
    temp(2, 0) = -WGS84_WIE * cos(pvapre_.pos[0]) / rmh - pvapre_.vel[1] / rmh / rnh / pow(cos(pvapre_.pos[0]), 2);
    temp(2, 2) = -pvapre_.vel[1] * tan(pvapre_.pos[0]) / rnh / rnh;
    F.block(PHI_ID, P_ID, 3, 3) = temp;
    temp.setZero();
    temp(0, 1)                    = 1 / rnh;
    temp(1, 0)                    = -1 / rmh;
    temp(2, 1)                    = -tan(pvapre_.pos[0]) / rnh;
    F.block(PHI_ID, V_ID, 3, 3)   = temp;
    F.block(PHI_ID, PHI_ID, 3, 3) = -Rotation::skewSymmetric(wie_n + wen_n);
    F.block(PHI_ID, BG_ID, 3, 3)  = -pvapre_.att.cbn;
    F.block(PHI_ID, SG_ID, 3, 3)  = -pvapre_.att.cbn * (omega.asDiagonal());

    // IMU零偏误差和比例因子误差，建模成一阶高斯-马尔科夫过程
    // imu bias error and scale error, modeled as the first-order Gauss-Markov process
    F.block(BG_ID, BG_ID, 3, 3) = -1 / options_.imunoise.corr_time * Eigen::Matrix3d::Identity();
    F.block(BA_ID, BA_ID, 3, 3) = -1 / options_.imunoise.corr_time * Eigen::Matrix3d::Identity();
    F.block(SG_ID, SG_ID, 3, 3) = -1 / options_.imunoise.corr_time * Eigen::Matrix3d::Identity();
    F.block(SA_ID, SA_ID, 3, 3) = -1 / options_.imunoise.corr_time * Eigen::Matrix3d::Identity();

    // 系统噪声驱动矩阵
    // system noise driven matrix
    G.block(V_ID, VRW_ID, 3, 3)    = pvapre_.att.cbn;
    G.block(PHI_ID, ARW_ID, 3, 3)  = pvapre_.att.cbn;
    G.block(BG_ID, BGSTD_ID, 3, 3) = Eigen::Matrix3d::Identity();
    G.block(BA_ID, BASTD_ID, 3, 3) = Eigen::Matrix3d::Identity();
    G.block(SG_ID, SGSTD_ID, 3, 3) = Eigen::Matrix3d::Identity();
    G.block(SA_ID, SASTD_ID, 3, 3) = Eigen::Matrix3d::Identity();

    // 状态转移矩阵
    // compute the state transition matrix
    Phi.setIdentity();
    Phi = Phi + F * imucur.dt;

    // 计算系统传播噪声
    // compute system propagation noise
    Eigen::MatrixXd Qc_effective = Qc_;
    const double arw_q_scale =
        std::max(propagation_arw_q_scale_, bounded_adaptive_rq_arw_q_scale_);
    const double vrw_q_scale =
        std::max(propagation_vrw_q_scale_, bounded_adaptive_rq_vrw_q_scale_);
    const double gyrbias_q_scale =
        std::max(propagation_gyrbias_q_scale_, bounded_adaptive_rq_gyrbias_q_scale_);
    const double accbias_q_scale =
        std::max(propagation_accbias_q_scale_, bounded_adaptive_rq_accbias_q_scale_);
    if (arw_q_scale != 1.0) {
        Qc_effective.block(ARW_ID, ARW_ID, 3, 3) *= arw_q_scale;
    }
    if (vrw_q_scale != 1.0) {
        Qc_effective.block(VRW_ID, VRW_ID, 3, 3) *= vrw_q_scale;
    }
    if (gyrbias_q_scale != 1.0) {
        Qc_effective.block(BGSTD_ID, BGSTD_ID, 3, 3) *= gyrbias_q_scale;
    }
    if (accbias_q_scale != 1.0) {
        Qc_effective.block(BASTD_ID, BASTD_ID, 3, 3) *= accbias_q_scale;
    }
    Qd = G * Qc_effective * G.transpose() * imucur.dt;
    Qd = (Phi * Qd * Phi.transpose() + Qd) / 2;

    // EKF预测传播系统协方差和系统误差状态
    // do EKF predict to propagate covariance and error state
    EKFPredict(Phi, Qd);
}

void GIEngine::gnssUpdate(GNSS &gnssdata) {

    const Eigen::Matrix3d gnss_meas_R =
        gnssdata.std.cwiseProduct(gnssdata.std).asDiagonal();

    bounded_adaptive_rq_velocity_evidence_valid_ = false;
    bounded_adaptive_rq_velocity_evidence_active_ = false;
    bounded_adaptive_rq_velocity_evidence_nis_ratio_ =
        std::numeric_limits<double>::quiet_NaN();
    bounded_adaptive_rq_velocity_evidence_residual_h_mps_ =
        std::numeric_limits<double>::quiet_NaN();
    if (bounded_adaptive_rq_velocity_evidence_gate_enable_ && has_vel_obs_ &&
        std::isfinite(vel_obs_time_) && std::isfinite(gnssdata.time) &&
        std::abs(vel_obs_time_ - gnssdata.time) <=
            bounded_adaptive_rq_velocity_evidence_time_tolerance_sec_) {
        Eigen::MatrixXd H_vel(3, RANK);
        H_vel.setZero();
        H_vel.block(0, V_ID, 3, 3) = Eigen::Matrix3d::Identity();
        const Eigen::Matrix3d R_vel_base =
            vel_obs_std_.cwiseProduct(vel_obs_std_).asDiagonal();
        const Eigen::Vector3d dz_vel = pvacur_.vel - vel_obs_;
        const Eigen::MatrixXd S_vel = H_vel * Cov_ * H_vel.transpose() + R_vel_base;
        const double vel_nis = computeNis_(S_vel, dz_vel);
        bounded_adaptive_rq_velocity_evidence_residual_h_mps_ =
            dz_vel.head<2>().norm();
        if (std::isfinite(vel_nis) &&
            std::isfinite(bounded_adaptive_rq_chi2_threshold_3d_) &&
            bounded_adaptive_rq_chi2_threshold_3d_ > 0.0) {
            bounded_adaptive_rq_velocity_evidence_valid_ = true;
            bounded_adaptive_rq_velocity_evidence_nis_ratio_ =
                vel_nis / bounded_adaptive_rq_chi2_threshold_3d_;
            bounded_adaptive_rq_velocity_evidence_active_ =
                bounded_adaptive_rq_velocity_evidence_nis_ratio_ >=
                    bounded_adaptive_rq_q_high_observation_velocity_nis_ratio_min_ ||
                bounded_adaptive_rq_velocity_evidence_residual_h_mps_ >=
                    bounded_adaptive_rq_q_high_observation_velocity_residual_h_min_;
        }
    }

    // no_sage 主线需要每次都吃到当前观测 std；否则首帧 R 会把后续 override 锁死。
    if (R_gnsspos_init_.norm() < 1e-12) {
        R_gnsspos_init_ = gnss_meas_R;
        sage_husa_k_ = 0;
    }
    if (!use_sage_husa_) {
        R_gnsspos_ = gnss_meas_R;
    } else if (R_gnsspos_.norm() < 1e-12) {
        R_gnsspos_ = gnss_meas_R;
    }

    // GNSS position uses the iterated error-state EKF path.
    // Iteratively re-linearize dz/H at the refined nominal state.
    auto meas_model = [&](const PVA &pva, const ImuError &, Eigen::MatrixXd &dz, Eigen::MatrixXd &H) {
        Eigen::Matrix3d Dr, Dr_inv;
        Dr_inv = Earth::DRi(pva.pos);
        Dr     = Earth::DR(pva.pos);

        const Eigen::Vector3d antenna_pos = pva.pos + Dr_inv * pva.att.cbn * options_.antlever;

        dz = Dr * (antenna_pos - gnssdata.blh);

        H.resize(3, RANK);
        H.setZero();
        H.block(0, P_ID, 3, 3)   = Eigen::Matrix3d::Identity();
        H.block(0, PHI_ID, 3, 3) = Rotation::skewSymmetric(pva.att.cbn * options_.antlever);
    };
    Eigen::MatrixXd dz_prefit, H_prefit;
    meas_model(pvacur_, imuerror_, dz_prefit, H_prefit);
    const Eigen::MatrixXd R_gnsspos_base = R_gnsspos_;
    const Eigen::MatrixXd S_prefit_base =
        H_prefit * Cov_ * H_prefit.transpose() + R_gnsspos_base;
    const double nis_base = computeNis_(S_prefit_base, dz_prefit);
    Eigen::MatrixXd R_gnsspos_effective =
        bounded_adaptive_rq_enable_
            ? boundedAdaptiveRQEffective_(
                  "gnss_position",
                  bounded_adaptive_rq_apply_position_,
                  true,
                  nis_base,
                  R_gnsspos_base,
                  bounded_adaptive_rq_position_gamma_,
                  last_observation_debug_.gnss_position_adaptive_r)
            : boundedAdaptiveREffective_(
                  "gnss_position",
                  bounded_adaptive_r_apply_position_,
                  nis_base,
                  R_gnsspos_base,
                  bounded_adaptive_r_position_gamma_,
                  last_observation_debug_.gnss_position_adaptive_r);
    const Eigen::MatrixXd S_prefit =
        H_prefit * Cov_ * H_prefit.transpose() + R_gnsspos_effective;
    last_observation_debug_.gnss_position_applied = true;
    last_observation_debug_.gnss_position_residual_neu_m = dz_prefit.col(0);
    last_observation_debug_.gnss_position_std_neu_m =
        R_gnsspos_effective.diagonal().cwiseMax(0.0).cwiseSqrt();
    last_observation_debug_.gnss_position_innovation_cov_neu_m2 = S_prefit;
    last_observation_debug_.gnss_position_nis_h_2d =
        computeNis_(S_prefit.topLeftCorner(2, 2), dz_prefit.topRows(2));
    if (S_prefit.rows() >= 3 && S_prefit.cols() >= 3 && dz_prefit.rows() >= 3) {
        const double s_uu = S_prefit(2, 2);
        const double r_u = dz_prefit(2, 0);
        last_observation_debug_.gnss_position_nis_u_1d =
            (std::isfinite(s_uu) && s_uu > 0.0 && std::isfinite(r_u))
                ? (r_u * r_u / s_uu)
                : std::numeric_limits<double>::quiet_NaN();
    }
    last_observation_debug_.gnss_position_nis_3d = computeNis_(S_prefit, dz_prefit);
    last_observation_debug_.gnss_position_gate_threshold_nis =
        std::numeric_limits<double>::quiet_NaN();
    last_observation_debug_.gnss_position_update_accepted = true;
    last_observation_debug_.gnss_position_update_rejected = false;
    last_observation_debug_.gnss_position_update_reason = "accepted_no_gnss_gate";
    if (last_observation_debug_.gnss_position_adaptive_r.applied) {
        IEKFUpdate(meas_model, R_gnsspos_effective, Cov_);
    } else {
        IEKFUpdate(meas_model, R_gnsspos_, Cov_);
    }

    // GNSS更新之后设置为不可用
    // Set GNSS invalid after update
    gnssdata.isvalid = false;
}

int GIEngine::isToUpdate(double imutime1, double imutime2, double updatetime) const {

    if (abs(imutime1 - updatetime) < TIME_ALIGN_ERR) {
        // 更新时间靠近imutime1
        // updatetime is near to imutime1
        return 1;
    } else if (abs(imutime2 - updatetime) <= TIME_ALIGN_ERR) {
        // 更新时间靠近imutime2
        // updatetime is near to imutime2
        return 2;
    } else if (imutime1 < updatetime && updatetime < imutime2) {
        // 更新时间在imutime1和imutime2之间, 但不靠近任何一个
        // updatetime is between imutime1 and imutime2, but not near to either
        return 3;
    } else {
        // 更新时间不在imutimt1和imutime2之间，且不靠近任何一个
        // updatetime is not bewteen imutime1 and imutime2, and not near to either.
        return 0;
    }
}

void GIEngine::EKFPredict(Eigen::MatrixXd &Phi, Eigen::MatrixXd &Qd) {

    assert(Phi.rows() == Cov_.rows());
    assert(Qd.rows() == Cov_.rows());

    // 传播系统协方差和误差状态
    // propagate system covariance and error state
    Cov_ = Phi * Cov_ * Phi.transpose() + Qd;
    dx_  = Phi * dx_;
}

void GIEngine::applyErrorState(const Eigen::MatrixXd &dx, PVA &pva, ImuError &imuerror) const {

    // 位置误差反馈 (NED/m to BLH/rad,m)
    const Eigen::Vector3d delta_r = dx.block(P_ID, 0, 3, 1);
    const Eigen::Matrix3d Dr_inv  = Earth::DRi(pva.pos);
    pva.pos -= Dr_inv * delta_r;

    // 速度误差反馈
    pva.vel -= dx.block(V_ID, 0, 3, 1);

    // 姿态误差反馈
    const Eigen::Vector3d delta_phi = dx.block(PHI_ID, 0, 3, 1);
    const Eigen::Quaterniond qpn    = Rotation::rotvec2quaternion(delta_phi);
    pva.att.qbn                     = qpn * pva.att.qbn;
    pva.att.cbn                     = Rotation::quaternion2matrix(pva.att.qbn);
    pva.att.euler                   = Rotation::matrix2euler(pva.att.cbn);

    // IMU零偏误差反馈
    imuerror.gyrbias += dx.block(BG_ID, 0, 3, 1);
    imuerror.accbias += dx.block(BA_ID, 0, 3, 1);

    // IMU比例因子误差反馈
    imuerror.gyrscale += dx.block(SG_ID, 0, 3, 1);
    imuerror.accscale += dx.block(SA_ID, 0, 3, 1);
}

void GIEngine::IEKFUpdate(
    const std::function<void(const PVA &, const ImuError &, Eigen::MatrixXd &, Eigen::MatrixXd &)> &meas_model,
    Eigen::MatrixXd &R, const Eigen::MatrixXd &P0) {

    // Prior (fixed during iteration)
    const PVA pva_prior       = pvacur_;
    const ImuError imu_prior  = imuerror_;
    // P0 已经在参数中传入

    // 预先计算 pre-fit innovation (在先验线性化点)
    // Use pre-fit innovation for adaptive R estimation
    Eigen::MatrixXd dz_prefit, H_prefit;
    meas_model(pva_prior, imu_prior, dz_prefit, H_prefit);

    Eigen::MatrixXd dx_iter;
    dx_iter.resize(RANK, 1);
    dx_iter.setZero();

    Eigen::MatrixXd dz, H;
    const int max_iters = std::max(1, iekf_max_iterations_);

    for (int iter = 0; iter < max_iters; ++iter) {
        const Eigen::MatrixXd dx_prev = dx_iter;

        // Linearization point: x_iter = x_prior ⊖ dx_iter
        PVA pva_iter      = pva_prior;
        ImuError imu_iter = imu_prior;
        applyErrorState(dx_iter, pva_iter, imu_iter);

        meas_model(pva_iter, imu_iter, dz, H);

        assert(dz.cols() == 1);
        assert(H.cols() == RANK);
        assert(dz.rows() == H.rows());
        assert(R.rows() == dz.rows());
        assert(R.cols() == dz.rows());

        const Eigen::MatrixXd S = H * P0 * H.transpose() + R;
        const auto S_ldlt       = S.ldlt();

        const Eigen::MatrixXd PHt = P0 * H.transpose();
        const Eigen::MatrixXd rhs = dz + H * dx_iter;

        // dx_{k+1} = K_k * (dz_k + H_k * dx_k)
        const Eigen::MatrixXd dx_new = PHt * S_ldlt.solve(rhs);
        dx_iter                      = dx_new;

        if ((dx_new - dx_prev).norm() < iekf_convergence_threshold_) {
            break;
        }
    }

    dx_ = dx_iter;

    // Re-linearize at the final state for a consistent covariance update
    PVA pva_final      = pva_prior;
    ImuError imu_final = imu_prior;
    applyErrorState(dx_iter, pva_final, imu_final);
    meas_model(pva_final, imu_final, dz, H);

    const Eigen::MatrixXd S_final = H * P0 * H.transpose() + R;
    const auto S_final_ldlt       = S_final.ldlt();
    const Eigen::MatrixXd PHt     = P0 * H.transpose();
    const Eigen::MatrixXd K       = PHt * S_final_ldlt.solve(Eigen::MatrixXd::Identity(dz.rows(), dz.rows()));
    last_update_kalman_gain_      = K;

    // Joseph stabilized covariance update with the final linearization (H, K)
    Eigen::MatrixXd I;
    I.resize(RANK, RANK);
    I.setIdentity();
    const Eigen::MatrixXd I_KH = I - K * H;
    Cov_                       = I_KH * P0 * I_KH.transpose() + K * R * K.transpose();
    Cov_                       = 0.5 * (Cov_ + Cov_.transpose());

    // Sage-Husa 自适应更新 R
    if (use_sage_husa_) {
        adaptiveREstimation(dz_prefit, H_prefit, R, P0);
    }
}

void GIEngine::adaptiveREstimation(const Eigen::VectorXd &dz, const Eigen::MatrixXd &H, Eigen::MatrixXd &R, const Eigen::MatrixXd &P0) {

    if (dz.size() == 0) return;
    if (H.rows() != dz.rows()) return;
    if (H.cols() != RANK) return;
    if (R.rows() != dz.rows() || R.cols() != dz.rows()) return;

    // alpha should be in (0,1). If invalid, skip adaptation.
    if (!(sage_husa_alpha_ > 0.0 && sage_husa_alpha_ < 1.0)) return;

    // Step index k starts from 1
    sage_husa_k_ += 1;

    // d_k = (1-alpha) / (1-alpha^k)
    double denom = 1.0 - std::pow(sage_husa_alpha_, static_cast<double>(sage_husa_k_));
    if (std::abs(denom) < 1e-12) denom = 1e-12;
    double dk = (1.0 - sage_husa_alpha_) / denom;
    if (!std::isfinite(dk)) return;
    dk = std::clamp(dk, 0.0, 1.0);

    // Innovation covariance estimate: R_hat = v v^T - H P_{k|k-1} H^T
    Eigen::MatrixXd Ck = dz * dz.transpose();
    Eigen::MatrixXd R_hat = Ck - H * P0 * H.transpose();
    R_hat = 0.5 * (R_hat + R_hat.transpose());

    // Exponential smoothing with scheduled gain dk
    Eigen::MatrixXd R_new = (1.0 - dk) * R + dk * R_hat;
    R_new = 0.5 * (R_new + R_new.transpose());

    if (sage_husa_diag_only_) {
        Eigen::VectorXd diag = R_new.diagonal();
        Eigen::VectorXd diag0 = (R_gnsspos_init_.size() == R.size()) ? R_gnsspos_init_.diagonal() : R.diagonal();

        for (int i = 0; i < diag.size(); ++i) {
            double floor_i = 0.0;
            floor_i = std::max(floor_i, sage_husa_min_var_factor_ * diag0(i));
            floor_i = std::max(floor_i, sage_husa_min_var_abs_);
            floor_i = std::max(floor_i, 1e-18);
            if (!std::isfinite(diag(i)) || diag(i) < floor_i) {
                diag(i) = floor_i;
            }
            // 【关键修复】R 上限：防止 heading 错误 → 大 innovation →
            // R 无限膨胀 → GNSS 权重趋零 的正反馈环路
            const double ceiling_i = 10.0 * diag0(i);  // 最多 10×R_init
            if (std::isfinite(ceiling_i) && ceiling_i > floor_i && diag(i) > ceiling_i) {
                diag(i) = ceiling_i;
            }
        }

        R = diag.asDiagonal();
        return;
    }

    // Full R: project to SPD by clamping eigenvalues
    const double eps = std::max(1e-18, sage_husa_min_var_abs_);
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(R_new);
    if (es.info() != Eigen::Success) return;
    Eigen::VectorXd eval = es.eigenvalues();
    eval = eval.cwiseMax(eps);
    R = es.eigenvectors() * eval.asDiagonal() * es.eigenvectors().transpose();
    R = 0.5 * (R + R.transpose());
}

EarlyRecoveryBiasFeedbackDebugInfo GIEngine::evaluateEarlyRecoveryBiasFeedback_(
    const std::string &event_type,
    double update_time_sec,
    const ImuError &imuerror_before,
    double residual_u_m,
    double raw_dx_ba_z_mps2) {

    EarlyRecoveryBiasFeedbackDebugInfo debug;
    debug.enabled =
        early_recovery_bias_feedback_debug_enable_ ||
        early_recovery_bias_feedback_apply_enable_;
    debug.apply_enabled = early_recovery_bias_feedback_apply_enable_;
    debug.history_sec = early_recovery_bias_feedback_history_sec_;
    debug.armed_time_sec = early_recovery_bias_feedback_context_armed_time_sec_;
    debug.raw_dx_ba_z_mps2 = raw_dx_ba_z_mps2;
    debug.selected_dx_ba_z_mps2 = raw_dx_ba_z_mps2;
    debug.delta_dx_ba_z_mps2 = 0.0;
    debug.negative_dx_scale = early_recovery_bias_feedback_negative_dx_scale_;

    const double accbias_z_before_mps2 = imuerror_before.accbias.z();
    if (std::isfinite(accbias_z_before_mps2) && std::isfinite(raw_dx_ba_z_mps2)) {
        debug.selected_accbias_z_after_mps2 =
            accbias_z_before_mps2 + raw_dx_ba_z_mps2;
    }

    if (!debug.enabled) {
        debug.reason = "disabled";
        return debug;
    }
    if (event_type != "gnss_position") {
        debug.reason = "non_gnss_position";
        return debug;
    }
    debug.candidate = true;

    const double armed_time_sec =
        early_recovery_bias_feedback_context_armed_time_sec_;
    const double core_gnss_u_m =
        early_recovery_bias_feedback_context_core_gnss_u_m_;
    if (!std::isfinite(update_time_sec) ||
        !std::isfinite(armed_time_sec) ||
        !std::isfinite(accbias_z_before_mps2) ||
        !std::isfinite(residual_u_m) ||
        !std::isfinite(core_gnss_u_m) ||
        !std::isfinite(raw_dx_ba_z_mps2)) {
        debug.reason = "missing_signal";
        return debug;
    }

    if (!early_recovery_bias_feedback_history_.empty() &&
        std::isfinite(early_recovery_bias_feedback_history_.back().armed_time_sec) &&
        armed_time_sec <
            early_recovery_bias_feedback_history_.back().armed_time_sec - 1.0e-6) {
        early_recovery_bias_feedback_history_.clear();
    }

    early_recovery_bias_feedback_history_.push_back(
        EarlyRecoveryBiasFeedbackSample{
            armed_time_sec,
            accbias_z_before_mps2,
            residual_u_m,
            core_gnss_u_m,
            raw_dx_ba_z_mps2});
    while (!early_recovery_bias_feedback_history_.empty()) {
        const auto &oldest = early_recovery_bias_feedback_history_.front();
        if (!std::isfinite(oldest.armed_time_sec) ||
            armed_time_sec - oldest.armed_time_sec >
                early_recovery_bias_feedback_history_sec_ + 1.0e-9) {
            early_recovery_bias_feedback_history_.pop_front();
        } else {
            break;
        }
    }

    double ba_sum = 0.0;
    double residual_sum = 0.0;
    double core_sum = 0.0;
    double dx_sum = 0.0;
    double neg_dx_sum = 0.0;
    double pos_dx_sum = 0.0;
    int rows = 0;
    for (const auto &sample : early_recovery_bias_feedback_history_) {
        if (!std::isfinite(sample.armed_time_sec) ||
            !std::isfinite(sample.ba_z_before_mps2) ||
            !std::isfinite(sample.residual_u_m) ||
            !std::isfinite(sample.core_gnss_u_m) ||
            !std::isfinite(sample.dx_ba_z_mps2)) {
            continue;
        }
        ++rows;
        ba_sum += sample.ba_z_before_mps2;
        residual_sum += sample.residual_u_m;
        core_sum += sample.core_gnss_u_m;
        dx_sum += sample.dx_ba_z_mps2;
        if (sample.dx_ba_z_mps2 < 0.0) {
            neg_dx_sum += sample.dx_ba_z_mps2;
        } else if (sample.dx_ba_z_mps2 > 0.0) {
            pos_dx_sum += sample.dx_ba_z_mps2;
        }
    }

    debug.history_rows = rows;
    if (rows > 0) {
        const double denom = static_cast<double>(rows);
        debug.ba_z_mean_mps2 = ba_sum / denom;
        debug.residual_u_mean_m = residual_sum / denom;
        debug.core_gnss_u_mean_m = core_sum / denom;
        debug.dx_ba_z_sum_mps2 = dx_sum;
        debug.negative_dx_ba_z_sum_mps2 = neg_dx_sum;
        debug.positive_dx_ba_z_sum_mps2 = pos_dx_sum;
    }

    if (armed_time_sec < early_recovery_bias_feedback_min_armed_time_sec_ ||
        armed_time_sec > early_recovery_bias_feedback_max_armed_time_sec_) {
        debug.reason = "outside_time_window";
        return debug;
    }
    if (rows < early_recovery_bias_feedback_min_history_rows_) {
        debug.reason = "insufficient_history";
        return debug;
    }
    if (!std::isfinite(debug.ba_z_mean_mps2) ||
        debug.ba_z_mean_mps2 > early_recovery_bias_feedback_ba_z_mean_max_mps2_) {
        debug.reason = "ba_z_mean_high";
        return debug;
    }
    if (!std::isfinite(debug.residual_u_mean_m) ||
        debug.residual_u_mean_m >
            early_recovery_bias_feedback_residual_u_mean_max_m_) {
        debug.reason = "residual_u_mean_high";
        return debug;
    }
    if (!std::isfinite(debug.core_gnss_u_mean_m) ||
        debug.core_gnss_u_mean_m <
            early_recovery_bias_feedback_core_gnss_u_mean_min_m_) {
        debug.reason = "core_gnss_u_mean_low";
        return debug;
    }
    if (!std::isfinite(debug.dx_ba_z_sum_mps2) ||
        debug.dx_ba_z_sum_mps2 >=
            early_recovery_bias_feedback_dx_ba_z_sum_max_mps2_) {
        debug.reason = "dx_ba_z_sum_nonnegative";
        return debug;
    }

    debug.active = true;
    debug.reason = "active";
    if (raw_dx_ba_z_mps2 < 0.0) {
        debug.selected_dx_ba_z_mps2 =
            raw_dx_ba_z_mps2 * early_recovery_bias_feedback_negative_dx_scale_;
        debug.delta_dx_ba_z_mps2 =
            debug.selected_dx_ba_z_mps2 - raw_dx_ba_z_mps2;
        debug.selected_accbias_z_after_mps2 =
            accbias_z_before_mps2 + debug.selected_dx_ba_z_mps2;
        debug.applied =
            early_recovery_bias_feedback_apply_enable_ &&
            std::abs(debug.delta_dx_ba_z_mps2) > 1.0e-15;
    }
    return debug;
}

void GIEngine::stateFeedback() {

    Eigen::Vector3d vectemp;

    // 位置误差反馈
    // posisiton error feedback
    Eigen::Vector3d delta_r = dx_.block(P_ID, 0, 3, 1);
    Eigen::Matrix3d Dr_inv  = Earth::DRi(pvacur_.pos);
    pvacur_.pos -= Dr_inv * delta_r;

    // 速度误差反馈
    // velocity error feedback
    vectemp = dx_.block(V_ID, 0, 3, 1);
    pvacur_.vel -= vectemp;

    // 姿态误差反馈
    // attitude error feedback
    vectemp                = dx_.block(PHI_ID, 0, 3, 1);
    Eigen::Quaterniond qpn = Rotation::rotvec2quaternion(vectemp);
    pvacur_.att.qbn        = qpn * pvacur_.att.qbn;
    pvacur_.att.cbn        = Rotation::quaternion2matrix(pvacur_.att.qbn);
    pvacur_.att.euler      = Rotation::matrix2euler(pvacur_.att.cbn);

    // IMU零偏误差反馈
    // IMU bias error feedback
    vectemp = dx_.block(BG_ID, 0, 3, 1);
    imuerror_.gyrbias += vectemp;
    vectemp = dx_.block(BA_ID, 0, 3, 1);
    imuerror_.accbias += vectemp;

    // IMU比例因子误差反馈
    // IMU sacle error feedback
    vectemp = dx_.block(SG_ID, 0, 3, 1);
    imuerror_.gyrscale += vectemp;
    vectemp = dx_.block(SA_ID, 0, 3, 1);
    imuerror_.accscale += vectemp;

    // 误差状态反馈到系统状态后,将误差状态清零
    // set 'dx' to zero after feedback error state to system state
    dx_.setZero();
}

void GIEngine::feedbackAndRecordStateUpdate(
    const std::string &event_type,
    const std::string &reason,
    double update_time_sec,
    int update_mode,
    const PVA &pva_before,
    const ImuError &imuerror_before,
    const Eigen::MatrixXd &covariance_before) {

    const Eigen::MatrixXd dx_before_feedback = dx_;
    EarlyRecoveryBiasFeedbackDebugInfo early_recovery_bias_feedback;
    if (dx_.rows() > BA_ID + 2 && dx_.cols() > 0) {
        early_recovery_bias_feedback =
            evaluateEarlyRecoveryBiasFeedback_(
                event_type,
                update_time_sec,
                imuerror_before,
                last_observation_debug_.gnss_position_residual_neu_m.z(),
                dx_(BA_ID + 2, 0));
        if (early_recovery_bias_feedback.applied) {
            dx_(BA_ID + 2, 0) =
                early_recovery_bias_feedback.selected_dx_ba_z_mps2;
        }
    }
    const Eigen::MatrixXd dx_selected_before_feedback = dx_;
    const Eigen::MatrixXd covariance_after_update = Cov_;
    stateFeedback();
    recordStateUpdateDebug(
        event_type,
        reason,
        update_time_sec,
        update_mode,
        true,
        early_recovery_bias_feedback.applied ? dx_selected_before_feedback : dx_before_feedback,
        pva_before,
        imuerror_before,
        covariance_before,
        pvacur_,
        imuerror_,
        covariance_after_update,
        early_recovery_bias_feedback);
}

void GIEngine::recordStateUpdateDebug(
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
    const EarlyRecoveryBiasFeedbackDebugInfo &early_recovery_bias_feedback) {

    StateUpdateDebugInfo event;
    event.sequence = ++state_update_debug_sequence_;
    event.valid = true;
    event.applied = applied;
    event.event_type = event_type;
    event.reason = reason;
    event.update_time_sec = update_time_sec;
    event.update_mode = update_mode;
    if (dx.rows() > 0 && dx.cols() == 1) {
        event.dx = dx.col(0);
    } else if (dx.rows() == 1 && dx.cols() > 0) {
        event.dx = dx.row(0).transpose();
    } else {
        event.dx = Eigen::VectorXd::Constant(RANK, std::numeric_limits<double>::quiet_NaN());
    }
    event.pos_blh_before_rad_m = pva_before.pos;
    event.pos_blh_after_rad_m = pva_after.pos;
    event.vel_before_ned_mps = pva_before.vel;
    event.vel_after_ned_mps = pva_after.vel;
    event.euler_before_rad = pva_before.att.euler;
    event.euler_after_rad = pva_after.att.euler;
    event.gyrbias_before_radps = imuerror_before.gyrbias;
    event.gyrbias_after_radps = imuerror_after.gyrbias;
    event.accbias_before_mps2 = imuerror_before.accbias;
    event.accbias_after_mps2 = imuerror_after.accbias;
    event.covariance_before = covariance_before;
    event.covariance_after = covariance_after;
    event.early_recovery_bias_feedback = early_recovery_bias_feedback;
    if (event_type == "gnss_position" || event_type == "gnss_velocity" || event_type == "heading") {
        event.kalman_gain = last_update_kalman_gain_;
    }
    const bool observation_same_time =
        last_observation_debug_.valid &&
        std::isfinite(last_observation_debug_.update_time_sec) &&
        std::isfinite(update_time_sec) &&
        std::abs(last_observation_debug_.update_time_sec - update_time_sec) <= 1.0e-6;
    if (observation_same_time) {
        event.gnss_position_adaptive_r =
            last_observation_debug_.gnss_position_adaptive_r;
        event.gnss_velocity_adaptive_r =
            last_observation_debug_.gnss_velocity_adaptive_r;
    }
    if (event_type == "gnss_position" &&
        observation_same_time &&
        last_observation_debug_.gnss_position_applied) {
        event.gnss_position_observation_valid = true;
        event.observation_sequence = last_observation_debug_.sequence;
        event.gnss_position_residual_neu_m =
            last_observation_debug_.gnss_position_residual_neu_m;
        event.gnss_position_std_neu_m =
            last_observation_debug_.gnss_position_std_neu_m;
        event.gnss_position_innovation_cov_neu_m2 =
            last_observation_debug_.gnss_position_innovation_cov_neu_m2;
        event.gnss_position_nis_h_2d =
            last_observation_debug_.gnss_position_nis_h_2d;
        event.gnss_position_nis_u_1d =
            last_observation_debug_.gnss_position_nis_u_1d;
        event.gnss_position_nis_3d =
            last_observation_debug_.gnss_position_nis_3d;
        event.gnss_position_gate_threshold_nis =
            last_observation_debug_.gnss_position_gate_threshold_nis;
        event.gnss_position_update_accepted =
            last_observation_debug_.gnss_position_update_accepted;
        event.gnss_position_update_rejected =
            last_observation_debug_.gnss_position_update_rejected;
        event.gnss_position_update_reason =
            last_observation_debug_.gnss_position_update_reason;
    }
    last_state_update_debug_event_ = event;
    if (event.event_type == "gnss_position") {
        last_gnss_position_state_update_debug_event_ = event;
    }
    if (!state_update_debug_enabled_) {
        return;
    }
    state_update_debug_events_.push_back(event);
    if (state_update_debug_events_.size() > 4096) {
        state_update_debug_events_.erase(
            state_update_debug_events_.begin(),
            state_update_debug_events_.begin() + 2048);
    }
}

bool GIEngine::reopenHorizontalPositionCovariance(
    double pos_std_h_m, double offdiag_corr_limit, const std::string &reason) {
    if (!std::isfinite(pos_std_h_m) || pos_std_h_m <= 0.0 || Cov_.rows() < 2 || Cov_.cols() < 2) {
        return false;
    }

    const PVA pva_before = pvacur_;
    const ImuError imuerror_before = imuerror_;
    const Eigen::MatrixXd covariance_before = Cov_;
    const double target_var = pos_std_h_m * pos_std_h_m;
    if (!std::isfinite(target_var) || target_var <= 0.0) {
        return false;
    }

    bool changed = false;
    for (int idx = P_ID; idx <= P_ID + 1; ++idx) {
        if (!std::isfinite(Cov_(idx, idx)) || Cov_(idx, idx) < target_var) {
            Cov_(idx, idx) = target_var;
            changed = true;
        }
    }

    const double p_nn = Cov_(P_ID, P_ID);
    const double p_ee = Cov_(P_ID + 1, P_ID + 1);
    const double corr_limit = std::clamp(std::abs(offdiag_corr_limit), 0.0, 0.999);
    if (std::isfinite(p_nn) && std::isfinite(p_ee) && p_nn > 0.0 && p_ee > 0.0) {
        const double cov_ne_limit = corr_limit * std::sqrt(p_nn * p_ee);
        if (std::isfinite(cov_ne_limit)) {
            const double cov_ne = 0.5 * (Cov_(P_ID, P_ID + 1) + Cov_(P_ID + 1, P_ID));
            const double clamped_ne = std::clamp(cov_ne, -cov_ne_limit, cov_ne_limit);
            if (!std::isfinite(cov_ne) ||
                std::abs(Cov_(P_ID, P_ID + 1) - clamped_ne) > 1.0e-15 ||
                std::abs(Cov_(P_ID + 1, P_ID) - clamped_ne) > 1.0e-15) {
                Cov_(P_ID, P_ID + 1) = clamped_ne;
                Cov_(P_ID + 1, P_ID) = clamped_ne;
                changed = true;
            }
        }
    }

    if (changed) {
        Cov_ = 0.5 * (Cov_ + Cov_.transpose());
        checkCov();
        recordStateUpdateDebug(
            "mission_cov_hygiene",
            reason.empty() ? "applied" : reason,
            timestamp_,
            -5,
            true,
            Eigen::MatrixXd(),
            pva_before,
            imuerror_before,
            covariance_before,
            pvacur_,
            imuerror_,
            Cov_);
    }
    return changed;
}

void GIEngine::gnssVelUpdate() {

    // GNSS velocity is a linear error-state measurement:
    // dz = v_ins - v_gnss, H = [0, I_3, 0, ...].
    // It intentionally uses a standard non-iterated Kalman update.

    const Eigen::Matrix3d R_vel_base =
        vel_obs_std_.cwiseProduct(vel_obs_std_).asDiagonal();

    // 创新量 (innovation): 预测速度 - 观测速度
    Eigen::Vector3d dz = pvacur_.vel - vel_obs_;
    last_observation_debug_.gnss_velocity_applied = true;
    last_observation_debug_.gnss_velocity_residual_ned_mps = dz;
    last_observation_debug_.gnss_velocity_std_ned_mps = vel_obs_std_;

    // 观测矩阵 (3x21): 速度状态直接可观
    Eigen::MatrixXd H(3, RANK);
    H.setZero();
    H.block(0, V_ID, 3, 3) = Eigen::Matrix3d::Identity();

    const Eigen::MatrixXd S_base = H * Cov_ * H.transpose() + R_vel_base;
    const double nis_base = computeNis_(S_base, dz);
    Eigen::MatrixXd R_vel =
        bounded_adaptive_rq_enable_
            ? boundedAdaptiveRQEffective_(
                  "gnss_velocity",
                  bounded_adaptive_rq_apply_velocity_,
                  false,
                  nis_base,
                  R_vel_base,
                  bounded_adaptive_rq_velocity_gamma_,
                  last_observation_debug_.gnss_velocity_adaptive_r)
            : boundedAdaptiveREffective_(
                  "gnss_velocity",
                  bounded_adaptive_r_apply_velocity_,
                  nis_base,
                  R_vel_base,
                  bounded_adaptive_r_velocity_gamma_,
                  last_observation_debug_.gnss_velocity_adaptive_r);
    last_observation_debug_.gnss_velocity_std_ned_mps =
        R_vel.diagonal().cwiseMax(0.0).cwiseSqrt();

    // 标准 EKF 更新 (Joseph 形式保证数值稳定)
    Eigen::MatrixXd S = H * Cov_ * H.transpose() + R_vel;
    Eigen::MatrixXd K = Cov_ * H.transpose() * S.ldlt().solve(Eigen::MatrixXd::Identity(3, 3));
    last_update_kalman_gain_ = K;

    dx_ = K * dz;

    Eigen::MatrixXd I_mat = Eigen::MatrixXd::Identity(RANK, RANK);
    Eigen::MatrixXd IKH = I_mat - K * H;
    Cov_ = IKH * Cov_ * IKH.transpose() + K * R_vel * K.transpose();
    Cov_ = 0.5 * (Cov_ + Cov_.transpose());

    has_vel_obs_ = false;
}

void GIEngine::headingUpdate() {

    if (!has_heading_obs_) return;
    const PVA pva_before = pvacur_;
    const ImuError imuerror_before = imuerror_;
    const Eigen::MatrixXd covariance_before = Cov_;
    const double update_time = heading_obs_time_;

    Eigen::MatrixXd R_heading(1, 1);
    R_heading(0, 0) = heading_obs_std_rad_ * heading_obs_std_rad_;

    // Heading uses the same iterated error-state EKF path as GNSS position.
    auto meas_model = [&](const PVA &pva, const ImuError &, Eigen::MatrixXd &dz, Eigen::MatrixXd &H) {
        dz.resize(1, 1);
        dz(0, 0) = normalizeAngleRad(pva.att.euler[2] - heading_obs_yaw_rad_);

        H.resize(1, RANK);
        H.setZero();
        // 对于当前误差状态反馈约定，yaw 量测对 phi_z 的线性化符号应为负，
        // 这样当预测 yaw 偏大时，更新会给出负的 delta_phi_z，把姿态拉回量测。
        H(0, PHI_ID + 2) = -1.0;
    };

    IEKFUpdate(meas_model, R_heading, Cov_);
    feedbackAndRecordStateUpdate(
        "heading",
        "applied",
        update_time,
        -1,
        pva_before,
        imuerror_before,
        covariance_before);
    checkCov();

    has_heading_obs_ = false;
}

void GIEngine::forceYaw(double yaw_rad, double yaw_std_rad, double time) {

    timestamp_ = time;
    const PVA pva_before = pvacur_;
    const ImuError imuerror_before = imuerror_;
    const Eigen::MatrixXd covariance_before = Cov_;

    pvacur_.att.euler[2] = normalizeAngleRad(yaw_rad);
    pvacur_.att.cbn      = Rotation::euler2matrix(pvacur_.att.euler);
    pvacur_.att.qbn      = Rotation::euler2quaternion(pvacur_.att.euler);

    // Keep the previous propagation anchor aligned with the corrected state.
    // Otherwise the next IMU propagation can pull yaw back toward the stale branch.
    pvapre_ = pvacur_;

    const int yaw_idx = PHI_ID + 2;
    const double yaw_var =
        std::pow(std::max(1e-4, std::abs(yaw_std_rad)), 2.0);
    Cov_.row(yaw_idx).setZero();
    Cov_.col(yaw_idx).setZero();
    Cov_(yaw_idx, yaw_idx) = yaw_var;

    dx_.setZero();
    has_heading_obs_ = false;
    checkCov();
    recordStateUpdateDebug(
        "force_yaw",
        "forced",
        time,
        -2,
        true,
        Eigen::MatrixXd(),
        pva_before,
        imuerror_before,
        covariance_before,
        pvacur_,
        imuerror_,
        Cov_);
}

void GIEngine::forceRollPitch(double roll_rad, double pitch_rad, double roll_pitch_std_rad, double time) {

    timestamp_ = time;
    const PVA pva_before = pvacur_;
    const ImuError imuerror_before = imuerror_;
    const Eigen::MatrixXd covariance_before = Cov_;

    pvacur_.att.euler[0] = roll_rad;
    pvacur_.att.euler[1] = pitch_rad;
    pvacur_.att.cbn      = Rotation::euler2matrix(pvacur_.att.euler);
    pvacur_.att.qbn      = Rotation::euler2quaternion(pvacur_.att.euler);

    pvapre_ = pvacur_;

    const double rp_var =
        std::pow(std::max(1e-4, std::abs(roll_pitch_std_rad)), 2.0);
    const int roll_idx = PHI_ID + 0;
    const int pitch_idx = PHI_ID + 1;
    for (const int idx : {roll_idx, pitch_idx}) {
        Cov_.row(idx).setZero();
        Cov_.col(idx).setZero();
        Cov_(idx, idx) = rp_var;
    }

    dx_.setZero();
    has_heading_obs_ = false;
    checkCov();
    recordStateUpdateDebug(
        "force_roll_pitch",
        "forced",
        time,
        -3,
        true,
        Eigen::MatrixXd(),
        pva_before,
        imuerror_before,
        covariance_before,
        pvacur_,
        imuerror_,
        Cov_);
}

bool GIEngine::reopenYawCovariance(double yaw_std_rad, double time, const std::string &reason) {
    const int yaw_idx = PHI_ID + 2;
    if (Cov_.rows() <= yaw_idx || Cov_.cols() <= yaw_idx ||
        !std::isfinite(yaw_std_rad) || yaw_std_rad <= 0.0) {
        return false;
    }

    const double target_var = std::pow(std::max(1e-4, std::abs(yaw_std_rad)), 2.0);
    if (!std::isfinite(target_var) || target_var <= 0.0) {
        return false;
    }
    if (std::isfinite(Cov_(yaw_idx, yaw_idx)) && Cov_(yaw_idx, yaw_idx) >= target_var) {
        return false;
    }

    timestamp_ = time;
    const PVA pva_before = pvacur_;
    const ImuError imuerror_before = imuerror_;
    const Eigen::MatrixXd covariance_before = Cov_;

    Cov_(yaw_idx, yaw_idx) = target_var;
    Cov_ = 0.5 * (Cov_ + Cov_.transpose());
    checkCov();
    recordStateUpdateDebug(
        "heading_yaw_cov_reopen",
        reason.empty() ? "applied" : reason,
        time,
        -6,
        true,
        Eigen::MatrixXd(),
        pva_before,
        imuerror_before,
        covariance_before,
        pvacur_,
        imuerror_,
        Cov_);
    return true;
}

bool GIEngine::reopenVerticalCovariance(
    double pos_std_m, double vel_std_mps, double accbias_std_z_mps2) {
    const PVA pva_before = pvacur_;
    const ImuError imuerror_before = imuerror_;
    const Eigen::MatrixXd covariance_before = Cov_;

    const auto reopen_diag = [&](int idx, double std_value) {
        if (!std::isfinite(std_value) || std_value <= 0.0) {
            return false;
        }
        const double target_var = std_value * std_value;
        if (!std::isfinite(target_var) || target_var <= 0.0) {
            return false;
        }
        if (!std::isfinite(Cov_(idx, idx)) || Cov_(idx, idx) < target_var) {
            Cov_(idx, idx) = target_var;
            return true;
        }
        return false;
    };

    bool changed = false;
    changed = reopen_diag(P_ID + 2, pos_std_m) || changed;
    changed = reopen_diag(V_ID + 2, vel_std_mps) || changed;
    changed = reopen_diag(BA_ID + 2, accbias_std_z_mps2) || changed;

    if (changed) {
        Cov_ = 0.5 * (Cov_ + Cov_.transpose());
        checkCov();
        recordStateUpdateDebug(
            "vertical_cov_reopen",
            "applied",
            timestamp_,
            -4,
            true,
            Eigen::MatrixXd(),
            pva_before,
            imuerror_before,
            covariance_before,
            pvacur_,
            imuerror_,
            Cov_);
    }
    return changed;
}

GIEngineSnapshot GIEngine::snapshot() const {

    GIEngineSnapshot snapshot;
    snapshot.timestamp = timestamp_;
    snapshot.nav_state.pos = pvacur_.pos;
    snapshot.nav_state.vel = pvacur_.vel;
    snapshot.nav_state.euler = pvacur_.att.euler;
    snapshot.nav_state.imuerror = imuerror_;
    snapshot.covariance = Cov_;
    snapshot.dx = dx_;
    snapshot.imupre = imupre_;
    snapshot.imucur = imucur_;

    snapshot.adaptive_r.position_gamma = bounded_adaptive_r_position_gamma_;
    snapshot.adaptive_r.velocity_gamma = bounded_adaptive_r_velocity_gamma_;

    snapshot.adaptive_rq.position_gamma = bounded_adaptive_rq_position_gamma_;
    snapshot.adaptive_rq.velocity_gamma = bounded_adaptive_rq_velocity_gamma_;
    snapshot.adaptive_rq.vrw_q_scale = bounded_adaptive_rq_vrw_q_scale_;
    snapshot.adaptive_rq.arw_q_scale = bounded_adaptive_rq_arw_q_scale_;
    snapshot.adaptive_rq.accbias_q_scale = bounded_adaptive_rq_accbias_q_scale_;
    snapshot.adaptive_rq.gyrbias_q_scale = bounded_adaptive_rq_gyrbias_q_scale_;
    snapshot.adaptive_rq.consecutive_exceed_count =
        bounded_adaptive_rq_consecutive_exceed_count_;
    snapshot.adaptive_rq.hold_remaining = bounded_adaptive_rq_hold_remaining_;
    snapshot.adaptive_rq.process_context_valid =
        bounded_adaptive_rq_process_context_valid_;
    snapshot.adaptive_rq.process_context_score =
        bounded_adaptive_rq_process_context_score_;
    snapshot.adaptive_rq.source_gate_allowed =
        bounded_adaptive_rq_source_gate_allowed_;
    snapshot.adaptive_rq.source_confidence =
        bounded_adaptive_rq_source_confidence_;
    snapshot.adaptive_rq.source_gate_reason =
        bounded_adaptive_rq_source_gate_reason_;
    snapshot.adaptive_rq.velocity_evidence_valid =
        bounded_adaptive_rq_velocity_evidence_valid_;
    snapshot.adaptive_rq.velocity_evidence_active =
        bounded_adaptive_rq_velocity_evidence_active_;
    snapshot.adaptive_rq.velocity_evidence_nis_ratio =
        bounded_adaptive_rq_velocity_evidence_nis_ratio_;
    snapshot.adaptive_rq.velocity_evidence_residual_h_mps =
        bounded_adaptive_rq_velocity_evidence_residual_h_mps_;
    snapshot.adaptive_rq.have_prev_position_r_diag =
        bounded_adaptive_rq_have_prev_position_r_diag_;
    snapshot.adaptive_rq.prev_position_r_diag =
        bounded_adaptive_rq_prev_position_r_diag_;

    snapshot.valid =
        std::isfinite(snapshot.timestamp) &&
        navStateFinite(snapshot.nav_state) &&
        snapshot.covariance.rows() == RANK &&
        snapshot.covariance.cols() == RANK &&
        snapshot.covariance.allFinite() &&
        snapshot.dx.rows() == RANK &&
        snapshot.dx.cols() == 1 &&
        snapshot.dx.allFinite() &&
        imuSampleFinite(snapshot.imupre) &&
        imuSampleFinite(snapshot.imucur);
    return snapshot;
}

bool GIEngine::restore(
    const GIEngineSnapshot &snapshot, const GIEngineRestorePolicy &policy) {

    if (!snapshot.valid) {
        return false;
    }

    if (policy.copy_nominal_state && !navStateFinite(snapshot.nav_state)) {
        return false;
    }
    if (policy.copy_imu_error && !imuErrorFinite(snapshot.nav_state.imuerror)) {
        return false;
    }
    if (policy.copy_imu_buffer &&
        (!imuSampleFinite(snapshot.imupre) || !imuSampleFinite(snapshot.imucur))) {
        return false;
    }

    Eigen::MatrixXd restored_covariance;
    if (policy.copy_covariance) {
        if (snapshot.covariance.rows() != RANK ||
            snapshot.covariance.cols() != RANK ||
            !snapshot.covariance.allFinite()) {
            return false;
        }
        double inflation = policy.covariance_inflation_factor;
        if (!std::isfinite(inflation) || inflation <= 0.0) {
            inflation = 1.0;
        }
        restored_covariance =
            0.5 * (snapshot.covariance + snapshot.covariance.transpose());
        restored_covariance *= inflation;
        if (policy.require_spd_covariance) {
            Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(restored_covariance);
            if (es.info() != Eigen::Success || !es.eigenvalues().allFinite()) {
                return false;
            }
            Eigen::VectorXd eval = es.eigenvalues().cwiseMax(1.0e-12);
            restored_covariance =
                es.eigenvectors() * eval.asDiagonal() * es.eigenvectors().transpose();
            restored_covariance =
                0.5 * (restored_covariance + restored_covariance.transpose());
            if (!restored_covariance.allFinite()) {
                return false;
            }
        }
    }

    if (policy.copy_nominal_state) {
        pvacur_.pos = snapshot.nav_state.pos;
        pvacur_.vel = snapshot.nav_state.vel;
        pvacur_.att.euler = snapshot.nav_state.euler;
        pvacur_.att.cbn = Rotation::euler2matrix(pvacur_.att.euler);
        pvacur_.att.qbn = Rotation::euler2quaternion(pvacur_.att.euler);
        pvapre_ = pvacur_;
    }
    if (policy.copy_imu_error) {
        imuerror_ = snapshot.nav_state.imuerror;
    }
    if (policy.copy_imu_buffer) {
        imupre_ = snapshot.imupre;
        imucur_ = snapshot.imucur;
    }
    if (policy.copy_covariance) {
        Cov_ = restored_covariance;
    }
    if (snapshot.dx.rows() == RANK && snapshot.dx.cols() == 1 &&
        snapshot.dx.allFinite()) {
        dx_ = snapshot.dx;
    } else {
        dx_.setZero();
    }
    if (std::isfinite(snapshot.timestamp)) {
        timestamp_ = snapshot.timestamp;
    }

    if (policy.copy_adaptive_r_memory) {
        bounded_adaptive_r_position_gamma_ =
            std::isfinite(snapshot.adaptive_r.position_gamma)
                ? snapshot.adaptive_r.position_gamma
                : 1.0;
        bounded_adaptive_r_velocity_gamma_ =
            std::isfinite(snapshot.adaptive_r.velocity_gamma)
                ? snapshot.adaptive_r.velocity_gamma
                : 1.0;
    } else if (policy.reset_adaptive_r_memory) {
        bounded_adaptive_r_position_gamma_ = 1.0;
        bounded_adaptive_r_velocity_gamma_ = 1.0;
    }

    if (policy.copy_adaptive_rq_memory) {
        bounded_adaptive_rq_position_gamma_ =
            std::isfinite(snapshot.adaptive_rq.position_gamma)
                ? snapshot.adaptive_rq.position_gamma
                : 1.0;
        bounded_adaptive_rq_velocity_gamma_ =
            std::isfinite(snapshot.adaptive_rq.velocity_gamma)
                ? snapshot.adaptive_rq.velocity_gamma
                : 1.0;
        bounded_adaptive_rq_vrw_q_scale_ =
            std::isfinite(snapshot.adaptive_rq.vrw_q_scale)
                ? snapshot.adaptive_rq.vrw_q_scale
                : 1.0;
        bounded_adaptive_rq_arw_q_scale_ =
            std::isfinite(snapshot.adaptive_rq.arw_q_scale)
                ? snapshot.adaptive_rq.arw_q_scale
                : 1.0;
        bounded_adaptive_rq_accbias_q_scale_ =
            std::isfinite(snapshot.adaptive_rq.accbias_q_scale)
                ? snapshot.adaptive_rq.accbias_q_scale
                : 1.0;
        bounded_adaptive_rq_gyrbias_q_scale_ =
            std::isfinite(snapshot.adaptive_rq.gyrbias_q_scale)
                ? snapshot.adaptive_rq.gyrbias_q_scale
                : 1.0;
        bounded_adaptive_rq_consecutive_exceed_count_ =
            std::max(0, snapshot.adaptive_rq.consecutive_exceed_count);
        bounded_adaptive_rq_hold_remaining_ =
            std::max(0, snapshot.adaptive_rq.hold_remaining);
        bounded_adaptive_rq_process_context_valid_ =
            snapshot.adaptive_rq.process_context_valid;
        bounded_adaptive_rq_process_context_score_ =
            std::isfinite(snapshot.adaptive_rq.process_context_score)
                ? std::clamp(snapshot.adaptive_rq.process_context_score, 0.0, 1.0)
                : 0.0;
        bounded_adaptive_rq_source_gate_allowed_ =
            snapshot.adaptive_rq.source_gate_allowed;
        bounded_adaptive_rq_source_confidence_ =
            std::isfinite(snapshot.adaptive_rq.source_confidence)
                ? std::clamp(snapshot.adaptive_rq.source_confidence, 0.0, 1.0)
                : 0.0;
        bounded_adaptive_rq_source_gate_reason_ =
            snapshot.adaptive_rq.source_gate_reason.empty()
                ? policy.reason
                : snapshot.adaptive_rq.source_gate_reason;
        bounded_adaptive_rq_velocity_evidence_valid_ =
            snapshot.adaptive_rq.velocity_evidence_valid;
        bounded_adaptive_rq_velocity_evidence_active_ =
            snapshot.adaptive_rq.velocity_evidence_active;
        bounded_adaptive_rq_velocity_evidence_nis_ratio_ =
            snapshot.adaptive_rq.velocity_evidence_nis_ratio;
        bounded_adaptive_rq_velocity_evidence_residual_h_mps_ =
            snapshot.adaptive_rq.velocity_evidence_residual_h_mps;
        bounded_adaptive_rq_have_prev_position_r_diag_ =
            snapshot.adaptive_rq.have_prev_position_r_diag &&
            snapshot.adaptive_rq.prev_position_r_diag.allFinite();
        bounded_adaptive_rq_prev_position_r_diag_ =
            bounded_adaptive_rq_have_prev_position_r_diag_
                ? snapshot.adaptive_rq.prev_position_r_diag
                : Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    } else if (policy.reset_adaptive_rq_memory) {
        resetBoundedAdaptiveRQScales_();
    }

    last_observation_debug_ = ObservationDebugInfo{};
    last_state_update_debug_event_ = StateUpdateDebugInfo{};
    last_gnss_position_state_update_debug_event_ = StateUpdateDebugInfo{};
    state_update_debug_events_.clear();
    return true;
}

NavState GIEngine::getNavState() {

    NavState state;

    state.pos      = pvacur_.pos;
    state.vel      = pvacur_.vel;
    state.euler    = pvacur_.att.euler;
    state.imuerror = imuerror_;

    return state;
}
