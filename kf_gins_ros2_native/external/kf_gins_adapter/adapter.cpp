#include "adapter.hpp"

// KF-GINS core headers
#include "kf-gins/gi_engine.h"
#include "kf-gins/kf_gins_types.h"
#include "common/types.h"

#include <cmath>
#include <memory>

namespace kfcore {

using Eigen::Vector3d;

static inline double D2R(double d){ return d * M_PI / 180.0; }
static inline double R2D(double r){ return r * 180.0 / M_PI; }

class KFGinsCoreImpl : public KFCore {
public:
  explicit KFGinsCoreImpl(const AdapterConfig& cfg) : cfg_(cfg) {
    zeroOptions_(options_);
    engine_.reset(new GIEngine(options_));
    // 不直接调用私有 initialize；让引擎在 IMU 传播中按内部逻辑起算
  }

  bool configure(const std::string& /*yaml_path*/) override {
    // 这里如果你想从 YAML 读参数，可扩展填充 options_
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

    engine_->addImuData(imu, /*compensate=*/true);
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
    // GIEngine 期望 NEU 标准差（米）；std_ned 第三项取绝对值再当 U
    gnss.std << std_ned.x(), std_ned.y(), std::abs(std_ned.z()); // N, E, U (米)
    // 上面一行如果你的 std 顺序就是 N,E,D，请改为：
    // gnss.std << std_ned.x(), std_ned.y(), std::abs(std_ned.z());
    gnss.isvalid = true;

    engine_->addGnssData(gnss);
    last_time_ = t;
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

    // 合理的初始方差
    options_.initstate_std.pos.setConstant(5.0);
    options_.initstate_std.vel.setConstant(1.0);
    options_.initstate_std.euler << D2R(5.0), D2R(5.0), D2R(10.0);

    engine_.reset(new GIEngine(options_));
    // 让引擎后续在 IMU 推进中根据 GNSS/内逻辑进入工作
    last_state_ = State{};
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

  State  last_state_{};
  double last_time_{0.0};
};

// 工厂
std::unique_ptr<KFCore> create_kf_core_adapter(const AdapterConfig& cfg) {
  return std::unique_ptr<KFCore>(new KFGinsCoreImpl(cfg));
}
std::unique_ptr<KFCore> create_kf_core() {
  AdapterConfig cfg;
  return create_kf_core_adapter(cfg);
}

} // namespace kfcore
