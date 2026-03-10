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
        // GNSS数据靠近上一历元，先对上一历元进行GNSS更新
        // gnssdata is near to the previous imudata, we should firstly do gnss update
        gnssUpdate(gnssdata_);
        stateFeedback();
        // 速度观测更新（若有）—— 在位置更新后立即应用
        if (has_vel_obs_) { gnssVelUpdate(); stateFeedback(); }

        pvapre_ = pvacur_;
        insPropagation(imupre_, imucur_);
    } else if (res == 2) {
        // GNSS数据靠近当前历元，先对当前IMU进行状态传播
        // gnssdata is near current imudata, we should firstly propagate navigation state
        insPropagation(imupre_, imucur_);
        gnssUpdate(gnssdata_);
        stateFeedback();
        // 速度观测更新（若有）
        if (has_vel_obs_) { gnssVelUpdate(); stateFeedback(); }
    } else {
        // GNSS数据在两个IMU数据之间(不靠近任何一个), 将当前IMU内插到整秒时刻
        // gnssdata is between the two imudata, we interpolate current imudata to gnss time
        IMU midimu;
        imuInterpolate(imupre_, imucur_, updatetime, midimu);

        // 对前一半IMU进行状态传播
        // propagate navigation state for the first half imudata
        insPropagation(imupre_, midimu);

        // 整秒时刻进行GNSS更新，并反馈系统状态
        // do GNSS position update at the whole second and feedback system states
        gnssUpdate(gnssdata_);
        stateFeedback();
        // 速度观测更新（若有）
        if (has_vel_obs_) { gnssVelUpdate(); stateFeedback(); }

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
    Qd = G * Qc_ * G.transpose() * imucur.dt;
    Qd = (Phi * Qd * Phi.transpose() + Qd) / 2;

    // EKF预测传播系统协方差和系统误差状态
    // do EKF predict to propagate covariance and error state
    EKFPredict(Phi, Qd);
}

void GIEngine::gnssUpdate(GNSS &gnssdata) {

    // 如果是第一次进入，则使用观测数据初始化 R_gnsspos_
    if (R_gnsspos_.norm() < 1e-12) {
        R_gnsspos_ = gnssdata.std.cwiseProduct(gnssdata.std).asDiagonal();
        R_gnsspos_init_ = R_gnsspos_;
        sage_husa_k_ = 0;
    }

    // IEKF (iterated error-state EKF) GNSS位置更新
    // Iteratively re-linearize dz/H at the refined state
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
    IEKFUpdate(meas_model, R_gnsspos_, Cov_);

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

void GIEngine::gnssVelUpdate() {

    // GNSS 速度观测更新
    // 观测模型是线性的：dz = v_ins - v_gnss, H = [0, I_3, 0, ...]
    // 无需迭代重线性化，使用标准 EKF 更新即可
    // Velocity observation is LINEAR: no need for iterative re-linearization

    Eigen::Matrix3d R_vel = vel_obs_std_.cwiseProduct(vel_obs_std_).asDiagonal();

    // 创新量 (innovation): 预测速度 - 观测速度
    Eigen::Vector3d dz = pvacur_.vel - vel_obs_;

    // 观测矩阵 (3x21): 速度状态直接可观
    Eigen::MatrixXd H(3, RANK);
    H.setZero();
    H.block(0, V_ID, 3, 3) = Eigen::Matrix3d::Identity();

    // 标准 EKF 更新 (Joseph 形式保证数值稳定)
    Eigen::MatrixXd S = H * Cov_ * H.transpose() + R_vel;
    Eigen::MatrixXd K = Cov_ * H.transpose() * S.ldlt().solve(Eigen::MatrixXd::Identity(3, 3));

    dx_ = K * dz;

    Eigen::MatrixXd I_mat = Eigen::MatrixXd::Identity(RANK, RANK);
    Eigen::MatrixXd IKH = I_mat - K * H;
    Cov_ = IKH * Cov_ * IKH.transpose() + K * R_vel * K.transpose();
    Cov_ = 0.5 * (Cov_ + Cov_.transpose());

    has_vel_obs_ = false;
}

NavState GIEngine::getNavState() {

    NavState state;

    state.pos      = pvacur_.pos;
    state.vel      = pvacur_.vel;
    state.euler    = pvacur_.att.euler;
    state.imuerror = imuerror_;

    return state;
}
