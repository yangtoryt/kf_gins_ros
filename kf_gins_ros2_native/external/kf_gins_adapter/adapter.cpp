#include "adapter.hpp"

// KF-GINS core headers
#include "kf-gins/gi_engine.h"
#include "kf-gins/kf_gins_types.h"
#include "common/types.h"

#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace kfcore {

using Eigen::Vector3d;

static inline double D2R(double d){ return d * M_PI / 180.0; }
static inline double R2D(double r){ return r * 180.0 / M_PI; }

namespace {

bool loadOptionsFromYaml(const std::string& yaml_path, GINSOptions& options) {
  YAML::Node config;
  try {
    config = YAML::LoadFile(yaml_path);
  } catch (const YAML::Exception& exception) {
    std::cerr << "Failed to read KF-GINS config '" << yaml_path
              << "': " << exception.what() << std::endl;
    return false;
  }

  std::vector<double> vec1, vec2, vec3, vec4, vec5, vec6;

  try {
    vec1 = config["initpos"].as<std::vector<double>>();
    vec2 = config["initvel"].as<std::vector<double>>();
    vec3 = config["initatt"].as<std::vector<double>>();
    if (vec1.size() != 3 || vec2.size() != 3 || vec3.size() != 3) {
      throw YAML::Exception(config.Mark(), "initpos/initvel/initatt must have 3 elements");
    }
  } catch (const YAML::Exception& exception) {
    std::cerr << "Failed to load KF-GINS initial state from '" << yaml_path
              << "': " << exception.what() << std::endl;
    return false;
  }

  options.initstate.pos << D2R(vec1[0]), D2R(vec1[1]), vec1[2];
  options.initstate.vel << vec2[0], vec2[1], vec2[2];
  options.initstate.euler << D2R(vec3[0]), D2R(vec3[1]), D2R(vec3[2]);

  try {
    vec1 = config["initgyrbias"].as<std::vector<double>>();
    vec2 = config["initaccbias"].as<std::vector<double>>();
    vec3 = config["initgyrscale"].as<std::vector<double>>();
    vec4 = config["initaccscale"].as<std::vector<double>>();
    if (vec1.size() != 3 || vec2.size() != 3 || vec3.size() != 3 || vec4.size() != 3) {
      throw YAML::Exception(config.Mark(), "initial IMU error arrays must have 3 elements");
    }
  } catch (const YAML::Exception& exception) {
    std::cerr << "Failed to load KF-GINS initial IMU error from '" << yaml_path
              << "': " << exception.what() << std::endl;
    return false;
  }

  for (int i = 0; i < 3; ++i) {
    options.initstate.imuerror.gyrbias[i]  = D2R(vec1[i]) / 3600.0;
    options.initstate.imuerror.accbias[i]  = vec2[i] * 1e-5;
    options.initstate.imuerror.gyrscale[i] = vec3[i] * 1e-6;
    options.initstate.imuerror.accscale[i] = vec4[i] * 1e-6;
  }

  try {
    vec1 = config["initposstd"].as<std::vector<double>>();
    vec2 = config["initvelstd"].as<std::vector<double>>();
    vec3 = config["initattstd"].as<std::vector<double>>();
    if (vec1.size() != 3 || vec2.size() != 3 || vec3.size() != 3) {
      throw YAML::Exception(config.Mark(), "initial state std arrays must have 3 elements");
    }
  } catch (const YAML::Exception& exception) {
    std::cerr << "Failed to load KF-GINS initial covariance from '" << yaml_path
              << "': " << exception.what() << std::endl;
    return false;
  }

  for (int i = 0; i < 3; ++i) {
    options.initstate_std.pos[i]   = vec1[i];
    options.initstate_std.vel[i]   = vec2[i];
    options.initstate_std.euler[i] = D2R(vec3[i]);
  }

  try {
    vec1 = config["imunoise"]["arw"].as<std::vector<double>>();
    vec2 = config["imunoise"]["vrw"].as<std::vector<double>>();
    vec3 = config["imunoise"]["gbstd"].as<std::vector<double>>();
    vec4 = config["imunoise"]["abstd"].as<std::vector<double>>();
    vec5 = config["imunoise"]["gsstd"].as<std::vector<double>>();
    vec6 = config["imunoise"]["asstd"].as<std::vector<double>>();
    options.imunoise.corr_time = config["imunoise"]["corrtime"].as<double>();
    if (vec1.size() != 3 || vec2.size() != 3 || vec3.size() != 3 ||
        vec4.size() != 3 || vec5.size() != 3 || vec6.size() != 3) {
      throw YAML::Exception(config.Mark(), "IMU noise arrays must have 3 elements");
    }
  } catch (const YAML::Exception& exception) {
    std::cerr << "Failed to load KF-GINS IMU noise from '" << yaml_path
              << "': " << exception.what() << std::endl;
    return false;
  }

  for (int i = 0; i < 3; ++i) {
    options.imunoise.gyr_arw[i]      = vec1[i];
    options.imunoise.acc_vrw[i]      = vec2[i];
    options.imunoise.gyrbias_std[i]  = vec3[i];
    options.imunoise.accbias_std[i]  = vec4[i];
    options.imunoise.gyrscale_std[i] = vec5[i];
    options.imunoise.accscale_std[i] = vec6[i];
  }

  try {
    vec1 = config["initbgstd"].as<std::vector<double>>();
  } catch (const YAML::Exception&) {
    vec1 = {options.imunoise.gyrbias_std.x(), options.imunoise.gyrbias_std.y(), options.imunoise.gyrbias_std.z()};
  }
  try {
    vec2 = config["initbastd"].as<std::vector<double>>();
  } catch (const YAML::Exception&) {
    vec2 = {options.imunoise.accbias_std.x(), options.imunoise.accbias_std.y(), options.imunoise.accbias_std.z()};
  }
  try {
    vec3 = config["initsgstd"].as<std::vector<double>>();
  } catch (const YAML::Exception&) {
    vec3 = {options.imunoise.gyrscale_std.x(), options.imunoise.gyrscale_std.y(), options.imunoise.gyrscale_std.z()};
  }
  try {
    vec4 = config["initsastd"].as<std::vector<double>>();
  } catch (const YAML::Exception&) {
    vec4 = {options.imunoise.accscale_std.x(), options.imunoise.accscale_std.y(), options.imunoise.accscale_std.z()};
  }

  for (int i = 0; i < 3; ++i) {
    options.initstate_std.imuerror.gyrbias[i]  = D2R(vec1[i]) / 3600.0;
    options.initstate_std.imuerror.accbias[i]  = vec2[i] * 1e-5;
    options.initstate_std.imuerror.gyrscale[i] = vec3[i] * 1e-6;
    options.initstate_std.imuerror.accscale[i] = vec4[i] * 1e-6;
  }

  options.imunoise.gyr_arw *= (M_PI / 180.0 / 60.0);
  options.imunoise.acc_vrw /= 60.0;
  options.imunoise.gyrbias_std *= (M_PI / 180.0 / 3600.0);
  options.imunoise.accbias_std *= 1e-5;
  options.imunoise.gyrscale_std *= 1e-6;
  options.imunoise.accscale_std *= 1e-6;
  options.imunoise.corr_time *= 3600.0;

  try {
    vec1 = config["antlever"].as<std::vector<double>>();
    if (vec1.size() != 3) {
      throw YAML::Exception(config.Mark(), "antlever must have 3 elements");
    }
  } catch (const YAML::Exception& exception) {
    std::cerr << "Failed to load KF-GINS antenna lever arm from '" << yaml_path
              << "': " << exception.what() << std::endl;
    return false;
  }
  options.antlever = Eigen::Vector3d(vec1[0], vec1[1], vec1[2]);

  try {
    if (config["sage_husa"]) {
      if (config["sage_husa"]["enable"]) {
        options.sage_husa.enable = config["sage_husa"]["enable"].as<bool>();
      }
      if (config["sage_husa"]["alpha"]) {
        options.sage_husa.alpha = config["sage_husa"]["alpha"].as<double>();
      }
      if (config["sage_husa"]["diag_only"]) {
        options.sage_husa.diag_only = config["sage_husa"]["diag_only"].as<bool>();
      }
      if (config["sage_husa"]["min_var_factor"]) {
        options.sage_husa.min_var_factor = config["sage_husa"]["min_var_factor"].as<double>();
      }
      if (config["sage_husa"]["min_var_abs"]) {
        options.sage_husa.min_var_abs = config["sage_husa"]["min_var_abs"].as<double>();
      }
    }
  } catch (const YAML::Exception& exception) {
    std::cerr << "Failed to load KF-GINS Sage-Husa options from '" << yaml_path
              << "': " << exception.what() << std::endl;
    return false;
  }

  try {
    if (config["iekf_max_iterations"]) {
      options.iekf_max_iterations = config["iekf_max_iterations"].as<int>();
    }
    if (config["iekf_convergence_threshold"]) {
      options.iekf_convergence_threshold = config["iekf_convergence_threshold"].as<double>();
    }
    if (config["filter_mode"] && config["filter_mode"].as<int>() == 0) {
      options.iekf_max_iterations = 1;
    }
  } catch (const YAML::Exception& exception) {
    std::cerr << "Failed to load KF-GINS filter mode from '" << yaml_path
              << "': " << exception.what() << std::endl;
    return false;
  }

  return true;
}

}  // namespace

class KFGinsCoreImpl : public KFCore {
public:
  explicit KFGinsCoreImpl(const AdapterConfig& cfg) : cfg_(cfg) {
    zeroOptions_(options_);
    engine_.reset(new GIEngine(options_));
    // 不直接调用私有 initialize；让引擎在 IMU 传播中按内部逻辑起算
  }

  bool configure(const std::string& yaml_path) override {
    if (yaml_path.empty()) {
      return true;
    }

    GINSOptions loaded_options;
    zeroOptions_(loaded_options);
    if (!loadOptionsFromYaml(yaml_path, loaded_options)) {
      return false;
    }

    options_ = loaded_options;
    if (cfg_.force_zero_antlever) {
      options_.antlever.setZero();
    }
    have_loaded_options_ = true;
    engine_.reset(new GIEngine(options_));
    last_state_ = State{};
    last_time_ = 0.0;
    std::cout << "Loaded KF-GINS core config: " << yaml_path << std::endl;
    return true;
  }

  // ★ 与接口一致：返回 bool
  bool ingestImu(double t, const Vector3d& dtheta_or_gyr,
                 const Vector3d& dvel_or_acc, double dt,
                 bool data_is_delta) override
  {
    IMU imu{};
    imu.time = t;
    imu.dt   = dt;

    if (data_is_delta || cfg_.imu_is_delta) {
      imu.dtheta = dtheta_or_gyr;
      imu.dvel   = dvel_or_acc;
    } else {
      // 角速度/比力 -> 增量
      imu.dtheta = dtheta_or_gyr * dt;
      imu.dvel   = dvel_or_acc   * dt;
    }

    // 【关键修复】compensate 必须为 false！
    // insPropagation() 内部已经会调用 imuCompensate(imucur)，
    // 如果此处也 compensate=true，IMU 偏差/比例因子会被扣除/缩放两次，
    // 导致一旦滤波器估计出非零 bias/scale，IMU 数据就被系统性破坏，
    // 进而引发姿态发散 → 重力投影错误 → 位置/速度指数级发散。
    engine_->addImuData(imu, /*compensate=*/false);
    engine_->newImuProcess();   // 推进 +（需要时）更新

    const auto st = engine_->getNavState();
    fillState_(st, last_state_);
    last_time_ = t;
    return true;
  }

  // ★ 与接口一致：返回 bool
  bool ingestGnss(double t, double lat_deg, double lon_deg, double h_m,
                  const Vector3d& std_ned) override
  {
    GNSS gnss{};
    gnss.time = t;
    // 注：GIEngine 用 BLH (rad, rad, m)
    gnss.blh << D2R(lat_deg), D2R(lon_deg), h_m;
    // GIEngine 期望 NEU 标准差（米）；必须为正，否则协方差会数值崩坏。
    const auto clamp_std = [](double v) {
      const double a = std::abs(v);
      // 【关键修复】旧值 0.5m 使仿真 GNSS std(0.1m) 被抬高 5 倍
      // 降为 0.01m，仅防止 std=0 造成 R=0 数值崩坏
      return std::max(0.01, a);
    };
    gnss.std << clamp_std(std_ned.x()), clamp_std(std_ned.y()), clamp_std(std_ned.z()); // N, E, U (m)
    // 上面一行如果你的 std 顺序就是 N,E,D，请改为：
    // gnss.std << std_ned.x(), std_ned.y(), std::abs(std_ned.z());
    gnss.isvalid = true;

    engine_->addGnssData(gnss);
    last_time_ = t;
    return true;
  }

  // ★ GNSS 速度观测输入
  bool ingestGnssVel(double t, double vN, double vE, double vD,
                     const Eigen::Vector3d& std_vel) override
  {
    // GIEngine 使用 NED 速度
    Eigen::Vector3d vel_ned(vN, vE, vD);
    // clamp std: 最小 0.1 m/s，避免数值问题
    Eigen::Vector3d std_clamped;
    std_clamped << std::max(0.1, std::abs(std_vel.x())),
                   std::max(0.1, std::abs(std_vel.y())),
                   std::max(0.1, std::abs(std_vel.z()));
    engine_->addVelData(vel_ned, std_clamped, t);
    return true;
  }

  bool ingestHeading(double t, double yaw_deg, double yaw_std_deg) override
  {
    const double yaw_rad = D2R(yaw_deg);
    const double yaw_std_rad = D2R(std::max(0.1, std::abs(yaw_std_deg)));
    engine_->addHeadingData(yaw_rad, yaw_std_rad, t);
    engine_->headingUpdate();
    last_time_ = t;
    fillState_(engine_->getNavState(), last_state_);
    return true;
  }

  bool forceYaw(double t, double yaw_deg, double yaw_std_deg) override
  {
    const double yaw_rad = D2R(yaw_deg);
    const double yaw_std_rad = D2R(std::max(0.1, std::abs(yaw_std_deg)));
    engine_->forceYaw(yaw_rad, yaw_std_rad, t);
    last_time_ = t;
    fillState_(engine_->getNavState(), last_state_);
    return true;
  }

  // ★ 新增：实现接口所需 reset（不直接调私有 initialize）
  bool reset(double lat_deg, double lon_deg, double h_m, double yaw_deg) override
  {
    // 更新 options_ 初值（位置、姿态），然后重建引擎
    options_.initstate.pos   << D2R(lat_deg), D2R(lon_deg), h_m;   // BLH(rad,rad,m)
    options_.initstate.vel.setZero();
    options_.initstate.euler << D2R(cfg_.init_roll_deg),
                                D2R(cfg_.init_pitch_deg),
                                D2R(yaw_deg);

    if (cfg_.use_online_reset_covariance) {
      // 在线 reset 与原始离线后处理的初始协方差需求不同：
      // 即使已经加载了 KF-GINS YAML，也要用一套更宽松的 reset 协方差，
      // 否则会对初始 yaw/速度过度自信，导致起飞后难以拉回。
      options_.initstate_std.pos.setConstant(std::max(0.1, cfg_.reset_pos_std_m));
      options_.initstate_std.vel.setConstant(std::max(0.1, cfg_.reset_vel_std_mps));
      options_.initstate_std.euler
        << D2R(std::max(0.1, cfg_.reset_roll_pitch_std_deg)),
           D2R(std::max(0.1, cfg_.reset_roll_pitch_std_deg)),
           D2R(std::max(0.1, cfg_.reset_yaw_std_deg));
      options_.initstate_std.imuerror.gyrbias  = Vector3d::Constant(D2R(500.0) / 3600.0);
      options_.initstate_std.imuerror.accbias  = Vector3d::Constant(5000e-6 * 9.80665);
      options_.initstate_std.imuerror.gyrscale = Vector3d::Constant(5000.0e-6);
      options_.initstate_std.imuerror.accscale = Vector3d::Constant(5000.0e-6);
    } else if (!have_loaded_options_) {
      // 没有外部 YAML 且未显式启用在线 reset 协方差时，保留默认值。
      options_.initstate_std.pos.setConstant(5.0);
      options_.initstate_std.vel.setConstant(5.0);
      options_.initstate_std.euler << D2R(5.0), D2R(5.0), D2R(10.0);
      options_.initstate_std.imuerror.gyrbias  = Vector3d::Constant(D2R(500.0) / 3600.0);
      options_.initstate_std.imuerror.accbias  = Vector3d::Constant(5000e-6 * 9.80665);
      options_.initstate_std.imuerror.gyrscale = Vector3d::Constant(5000.0e-6);
      options_.initstate_std.imuerror.accscale = Vector3d::Constant(5000.0e-6);
    }

    engine_.reset(new GIEngine(options_));
    // 让 current() 在首次 IMU 推进前也能反映 reset 后的初值，避免上层读到空状态。
    last_state_.lat_deg = lat_deg;
    last_state_.lon_deg = lon_deg;
    last_state_.h_m = h_m;
    last_state_.roll_deg = cfg_.init_roll_deg;
    last_state_.pitch_deg = cfg_.init_pitch_deg;
    last_state_.yaw_deg = yaw_deg;
    last_state_.vN = 0.0;
    last_state_.vE = 0.0;
    last_state_.vD = 0.0;
    last_time_  = 0.0;
    return true;
  }

  State current() const override { return last_state_; }

private:
  void zeroOptions_(GINSOptions& opt) {
    opt.initstate.pos.setZero();   // lat(rad), lon(rad), h(m)
    opt.initstate.vel.setZero();   // vN, vE, vD
    opt.initstate.euler.setZero(); // roll, pitch, yaw (rad)

    // 初始方差设得宽松些，方便滤波“拉回”
    opt.initstate_std.pos.setConstant(100.0);
    opt.initstate_std.vel.setConstant(10.0);
    opt.initstate_std.euler.setConstant(D2R(30.0));
    // 【关键修复】IMU 偏差/比例因子的初始 STD
    // 旧代码未设置这些值 → 默认为 0 → P[BG]=P[BA]=P[SG]=P[SA]=0
    // → 滤波器认为 IMU 偏差"完美已知为零"，永远不更新
    // → 陀螺偏差无法估计 → yaw 不可控制地漂移
    // 仿真 MEMS-IMU 的合理初始不确定度：
    opt.initstate_std.imuerror.gyrbias  = Vector3d::Constant(D2R(500.0) / 3600.0);  // 500 deg/h
    opt.initstate_std.imuerror.accbias  = Vector3d::Constant(5000e-6 * 9.80665);    // 5000 mGal
    opt.initstate_std.imuerror.gyrscale = Vector3d::Constant(5000.0e-6);             // 5000 ppm
    opt.initstate_std.imuerror.accscale = Vector3d::Constant(5000.0e-6);             // 5000 ppm
    // 简单默认 IMU 噪声（可从 YAML/参数替换以对齐原仓库）
    opt.imunoise.gyr_arw      = Vector3d::Constant(D2R(0.2) / std::sqrt(3600.0));
    opt.imunoise.acc_vrw      = Vector3d::Constant(0.2 / std::sqrt(3600.0));
    opt.imunoise.gyrbias_std  = Vector3d::Constant(D2R(10.0) / 3600.0);
    opt.imunoise.accbias_std  = Vector3d::Constant(100e-6 * 9.80665);
    opt.imunoise.gyrscale_std = Vector3d::Constant(100.0e-6);
    opt.imunoise.accscale_std = Vector3d::Constant(100.0e-6);
    opt.imunoise.corr_time    = 3600.0;

    opt.antlever.setZero();
  }

  static void fillState_(const NavState& s, State& out) {
    out.lat_deg   = R2D(s.pos[0]);
    out.lon_deg   = R2D(s.pos[1]);
    out.h_m       = s.pos[2];
    out.roll_deg  = R2D(s.euler[0]);
    out.pitch_deg = R2D(s.euler[1]);
    out.yaw_deg   = R2D(s.euler[2]);
    // vel 是 [vN, vE, vD]
    out.vN = s.vel[0];
    out.vE = s.vel[1];
    out.vD = s.vel[2];
  }

private:
  AdapterConfig cfg_;
  std::unique_ptr<GIEngine> engine_;
  GINSOptions options_{};
  bool have_loaded_options_{false};

  State  last_state_{};
  double last_time_{0.0};
};

// 工厂
std::unique_ptr<KFCore> create_kf_core_adapter(const AdapterConfig& cfg) {
  return std::unique_ptr<KFCore>(new KFGinsCoreImpl(cfg));
}
std::unique_ptr<KFCore> create_kf_core() {
  AdapterConfig cfg;
  cfg.imu_is_delta = false;
  return create_kf_core_adapter(cfg);
}

} // namespace kfcore
