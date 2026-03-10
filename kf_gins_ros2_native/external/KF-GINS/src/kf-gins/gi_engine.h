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

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <functional>
#include <iomanip>
#include <iostream>
#include <vector>

#include "common/types.h"

#include "kf_gins_types.h"

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
	 *        Update using iterated linearization with fixed prior covariance.
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

    double timestamp_;

    // 更新时间对齐误差，IMU状态和观测信息误差小于它则认为两者对齐
    // updata time align error
    const double TIME_ALIGN_ERR = 0.001;

    // IMU和GNSS原始数据
    // raw imudata and gnssdata
    IMU imupre_;
    IMU imucur_;
    GNSS gnssdata_;

    // IMU状态（位置、速度、姿态和IMU误差）
    // imu state (position, velocity, attitude and imu error)
    PVA pvacur_;
    PVA pvapre_;
    ImuError imuerror_;

    // Kalman滤波相关
    // ekf variables
	Eigen::MatrixXd Cov_;
	Eigen::MatrixXd Qc_;
	Eigen::MatrixXd dx_;
	    
	// IEKF (Iterated Extended Kalman Filter) 相关变量
	// IEKF parameters and temporary variables
	int iekf_max_iterations_{5};              // 最大迭代次数 / max iterations
	double iekf_convergence_threshold_{1e-6}; // 收敛阈值 / convergence threshold
	
	void applyErrorState(const Eigen::MatrixXd &dx, PVA &pva, ImuError &imuerror) const;

	const int RANK      = 21;
	const int NOISERANK = 18;

    /**
     * @brief 使用GNSS速度观测更新系统状态（线性观测，标准EKF更新）
     *        update state using GNSS velocity observation
     * */
    void gnssVelUpdate();

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

    // 状态ID和噪声ID
    // state ID and noise ID
    enum StateID { P_ID = 0, V_ID = 3, PHI_ID = 6, BG_ID = 9, BA_ID = 12, SG_ID = 15, SA_ID = 18 };
    enum NoiseID { VRW_ID = 0, ARW_ID = 3, BGSTD_ID = 6, BASTD_ID = 9, SGSTD_ID = 12, SASTD_ID = 15 };


};

#endif // GI_ENGINE_H
