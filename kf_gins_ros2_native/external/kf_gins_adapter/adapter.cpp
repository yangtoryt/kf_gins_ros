#include "adapter.hpp"

// KF-GINS core headers
#include "kf-gins/gi_engine.h"
#include "kf-gins/kf_gins_types.h"
#include "common/earth.h"
#include "common/rotation.h"
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

static BoundedAdaptiveRDebug convertBoundedAdaptiveR_(
  const BoundedAdaptiveRDebugInfo & engine_debug)
{
  BoundedAdaptiveRDebug debug;
  debug.enabled = engine_debug.enabled;
  debug.applied = engine_debug.applied;
  debug.exceeded = engine_debug.exceeded;
  debug.update_type = engine_debug.update_type;
  debug.mode = engine_debug.mode;
  debug.rq_selector_trigger = engine_debug.rq_selector_trigger;
  debug.rq_selector_reason = engine_debug.rq_selector_reason;
  debug.nis = engine_debug.nis;
  debug.chi2_threshold = engine_debug.chi2_threshold;
  debug.nis_ratio = engine_debug.nis_ratio;
  debug.consecutive_exceed_count = engine_debug.consecutive_exceed_count;
  debug.hold_remaining = engine_debug.hold_remaining;
  debug.observation_score = engine_debug.observation_score;
  debug.process_score = engine_debug.process_score;
  debug.gamma_raw = engine_debug.gamma_raw;
  debug.gamma_smoothed = engine_debug.gamma_smoothed;
  debug.gamma_clipped = engine_debug.gamma_clipped;
  debug.r_gamma_limit = engine_debug.r_gamma_limit;
  debug.q_lambda_vrw = engine_debug.q_lambda_vrw;
  debug.q_lambda_arw = engine_debug.q_lambda_arw;
  debug.q_lambda_accbias = engine_debug.q_lambda_accbias;
  debug.q_lambda_gyrbias = engine_debug.q_lambda_gyrbias;
  debug.gps_quality_stable = engine_debug.gps_quality_stable;
  debug.motion_context_ok = engine_debug.motion_context_ok;
  debug.q_source_confidence = engine_debug.q_source_confidence;
  debug.q_source_gate_allowed = engine_debug.q_source_gate_allowed;
  debug.q_source_gate_reason = engine_debug.q_source_gate_reason;
  debug.q_velocity_evidence = engine_debug.q_velocity_evidence;
  debug.q_velocity_nis_ratio = engine_debug.q_velocity_nis_ratio;
  debug.q_velocity_residual_h_mps = engine_debug.q_velocity_residual_h_mps;
  debug.r_base_diag = engine_debug.r_base_diag;
  debug.r_eff_diag = engine_debug.r_eff_diag;
  return debug;
}

static EarlyRecoveryBiasFeedbackDebug convertEarlyRecoveryBiasFeedback_(
  const EarlyRecoveryBiasFeedbackDebugInfo & engine_debug)
{
  EarlyRecoveryBiasFeedbackDebug debug;
  debug.enabled = engine_debug.enabled;
  debug.apply_enabled = engine_debug.apply_enabled;
  debug.candidate = engine_debug.candidate;
  debug.active = engine_debug.active;
  debug.applied = engine_debug.applied;
  debug.reason = engine_debug.reason;
  debug.history_rows = engine_debug.history_rows;
  debug.history_sec = engine_debug.history_sec;
  debug.armed_time_sec = engine_debug.armed_time_sec;
  debug.ba_z_mean_mps2 = engine_debug.ba_z_mean_mps2;
  debug.residual_u_mean_m = engine_debug.residual_u_mean_m;
  debug.core_gnss_u_mean_m = engine_debug.core_gnss_u_mean_m;
  debug.dx_ba_z_sum_mps2 = engine_debug.dx_ba_z_sum_mps2;
  debug.negative_dx_ba_z_sum_mps2 = engine_debug.negative_dx_ba_z_sum_mps2;
  debug.positive_dx_ba_z_sum_mps2 = engine_debug.positive_dx_ba_z_sum_mps2;
  debug.raw_dx_ba_z_mps2 = engine_debug.raw_dx_ba_z_mps2;
  debug.selected_dx_ba_z_mps2 = engine_debug.selected_dx_ba_z_mps2;
  debug.delta_dx_ba_z_mps2 = engine_debug.delta_dx_ba_z_mps2;
  debug.selected_accbias_z_after_mps2 =
    engine_debug.selected_accbias_z_after_mps2;
  debug.negative_dx_scale = engine_debug.negative_dx_scale;
  return debug;
}

static ImuErrorState convertImuError_(const ImuError & engine_imu_error)
{
  ImuErrorState imu_error;
  imu_error.gyro_bias_radps = engine_imu_error.gyrbias;
  imu_error.acc_bias_mps2 = engine_imu_error.accbias;
  imu_error.gyro_scale = engine_imu_error.gyrscale;
  imu_error.acc_scale = engine_imu_error.accscale;
  return imu_error;
}

static ImuError convertImuError_(const ImuErrorState & imu_error)
{
  ImuError engine_imu_error;
  engine_imu_error.gyrbias = imu_error.gyro_bias_radps;
  engine_imu_error.accbias = imu_error.acc_bias_mps2;
  engine_imu_error.gyrscale = imu_error.gyro_scale;
  engine_imu_error.accscale = imu_error.acc_scale;
  return engine_imu_error;
}

static AdaptiveRMemory convertAdaptiveRMemory_(
  const BoundedAdaptiveREngineMemory & engine_memory)
{
  AdaptiveRMemory memory;
  memory.position_gamma = engine_memory.position_gamma;
  memory.velocity_gamma = engine_memory.velocity_gamma;
  return memory;
}

static BoundedAdaptiveREngineMemory convertAdaptiveRMemory_(
  const AdaptiveRMemory & memory)
{
  BoundedAdaptiveREngineMemory engine_memory;
  engine_memory.position_gamma = memory.position_gamma;
  engine_memory.velocity_gamma = memory.velocity_gamma;
  return engine_memory;
}

static AdaptiveRQMemory convertAdaptiveRQMemory_(
  const BoundedAdaptiveRQEngineMemory & engine_memory)
{
  AdaptiveRQMemory memory;
  memory.position_gamma = engine_memory.position_gamma;
  memory.velocity_gamma = engine_memory.velocity_gamma;
  memory.vrw_q_scale = engine_memory.vrw_q_scale;
  memory.arw_q_scale = engine_memory.arw_q_scale;
  memory.accbias_q_scale = engine_memory.accbias_q_scale;
  memory.gyrbias_q_scale = engine_memory.gyrbias_q_scale;
  memory.consecutive_exceed_count = engine_memory.consecutive_exceed_count;
  memory.hold_remaining = engine_memory.hold_remaining;
  memory.process_context_valid = engine_memory.process_context_valid;
  memory.process_context_score = engine_memory.process_context_score;
  memory.source_gate_allowed = engine_memory.source_gate_allowed;
  memory.source_confidence = engine_memory.source_confidence;
  memory.source_gate_reason = engine_memory.source_gate_reason;
  memory.velocity_evidence_valid = engine_memory.velocity_evidence_valid;
  memory.velocity_evidence_active = engine_memory.velocity_evidence_active;
  memory.velocity_evidence_nis_ratio = engine_memory.velocity_evidence_nis_ratio;
  memory.velocity_evidence_residual_h_mps =
    engine_memory.velocity_evidence_residual_h_mps;
  memory.have_prev_position_r_diag = engine_memory.have_prev_position_r_diag;
  memory.prev_position_r_diag = engine_memory.prev_position_r_diag;
  return memory;
}

static BoundedAdaptiveRQEngineMemory convertAdaptiveRQMemory_(
  const AdaptiveRQMemory & memory)
{
  BoundedAdaptiveRQEngineMemory engine_memory;
  engine_memory.position_gamma = memory.position_gamma;
  engine_memory.velocity_gamma = memory.velocity_gamma;
  engine_memory.vrw_q_scale = memory.vrw_q_scale;
  engine_memory.arw_q_scale = memory.arw_q_scale;
  engine_memory.accbias_q_scale = memory.accbias_q_scale;
  engine_memory.gyrbias_q_scale = memory.gyrbias_q_scale;
  engine_memory.consecutive_exceed_count = memory.consecutive_exceed_count;
  engine_memory.hold_remaining = memory.hold_remaining;
  engine_memory.process_context_valid = memory.process_context_valid;
  engine_memory.process_context_score = memory.process_context_score;
  engine_memory.source_gate_allowed = memory.source_gate_allowed;
  engine_memory.source_confidence = memory.source_confidence;
  engine_memory.source_gate_reason = memory.source_gate_reason;
  engine_memory.velocity_evidence_valid = memory.velocity_evidence_valid;
  engine_memory.velocity_evidence_active = memory.velocity_evidence_active;
  engine_memory.velocity_evidence_nis_ratio = memory.velocity_evidence_nis_ratio;
  engine_memory.velocity_evidence_residual_h_mps =
    memory.velocity_evidence_residual_h_mps;
  engine_memory.have_prev_position_r_diag = memory.have_prev_position_r_diag;
  engine_memory.prev_position_r_diag = memory.prev_position_r_diag;
  return engine_memory;
}

static ImuSampleSnapshot convertImuSample_(const IMU & imu)
{
  ImuSampleSnapshot sample;
  sample.time_sec = imu.time;
  sample.dt_sec = imu.dt;
  sample.dtheta = imu.dtheta;
  sample.dvel = imu.dvel;
  sample.odovel = imu.odovel;
  return sample;
}

static IMU convertImuSample_(const ImuSampleSnapshot & sample)
{
  IMU imu{};
  imu.time = sample.time_sec;
  imu.dt = sample.dt_sec;
  imu.dtheta = sample.dtheta;
  imu.dvel = sample.dvel;
  imu.odovel = sample.odovel;
  return imu;
}

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
    if (config["bounded_adaptive_r"]) {
      const YAML::Node bounded = config["bounded_adaptive_r"];
      if (bounded["enable"]) {
        options.bounded_adaptive_r.enable = bounded["enable"].as<bool>();
      }
      if (bounded["mode"]) {
        options.bounded_adaptive_r.mode = bounded["mode"].as<std::string>();
      }
      if (bounded["apply_position"]) {
        options.bounded_adaptive_r.apply_position = bounded["apply_position"].as<bool>();
      }
      if (bounded["apply_velocity"]) {
        options.bounded_adaptive_r.apply_velocity = bounded["apply_velocity"].as<bool>();
      }
      if (bounded["alpha"]) {
        options.bounded_adaptive_r.alpha = bounded["alpha"].as<double>();
      }
      if (bounded["beta"]) {
        options.bounded_adaptive_r.beta = bounded["beta"].as<double>();
      }
      if (bounded["gamma_min"]) {
        options.bounded_adaptive_r.gamma_min = bounded["gamma_min"].as<double>();
      }
      if (bounded["gamma_max"]) {
        options.bounded_adaptive_r.gamma_max = bounded["gamma_max"].as<double>();
      }
      if (bounded["chi2_prob"]) {
        options.bounded_adaptive_r.chi2_prob = bounded["chi2_prob"].as<double>();
      }
    }
  } catch (const YAML::Exception& exception) {
    std::cerr << "Failed to load KF-GINS bounded adaptive R options from '" << yaml_path
              << "': " << exception.what() << std::endl;
    return false;
  }

  try {
    if (config["bounded_adaptive_rq"]) {
      const YAML::Node bounded = config["bounded_adaptive_rq"];
      if (bounded["enable"]) {
        options.bounded_adaptive_rq.enable = bounded["enable"].as<bool>();
      }
      if (bounded["mode"]) {
        options.bounded_adaptive_rq.mode = bounded["mode"].as<std::string>();
      }
      if (bounded["apply_position"]) {
        options.bounded_adaptive_rq.apply_position = bounded["apply_position"].as<bool>();
      }
      if (bounded["apply_velocity"]) {
        options.bounded_adaptive_rq.apply_velocity = bounded["apply_velocity"].as<bool>();
      }
      if (bounded["r_only_on_observation_disturbance"]) {
        options.bounded_adaptive_rq.r_only_on_observation_disturbance =
          bounded["r_only_on_observation_disturbance"].as<bool>();
      }
      if (bounded["q_on_process_disturbance"]) {
        options.bounded_adaptive_rq.q_on_process_disturbance =
          bounded["q_on_process_disturbance"].as<bool>();
      }
      if (bounded["chi2_prob"]) {
        options.bounded_adaptive_rq.chi2_prob = bounded["chi2_prob"].as<double>();
      }
      if (bounded["nis_ratio_start"]) {
        options.bounded_adaptive_rq.nis_ratio_start =
          bounded["nis_ratio_start"].as<double>();
      }
      if (bounded["nis_ratio_full"]) {
        options.bounded_adaptive_rq.nis_ratio_full =
          bounded["nis_ratio_full"].as<double>();
      }
      if (bounded["consecutive_exceed_min"]) {
        options.bounded_adaptive_rq.consecutive_exceed_min =
          bounded["consecutive_exceed_min"].as<int>();
      }
      if (bounded["hold_updates"]) {
        options.bounded_adaptive_rq.hold_updates = bounded["hold_updates"].as<int>();
      }
      if (bounded["alpha_r"]) {
        options.bounded_adaptive_rq.alpha_r = bounded["alpha_r"].as<double>();
      }
      if (bounded["alpha_r_process"]) {
        options.bounded_adaptive_rq.alpha_r_process =
          bounded["alpha_r_process"].as<double>();
      }
      if (bounded["alpha_q"]) {
        options.bounded_adaptive_rq.alpha_q = bounded["alpha_q"].as<double>();
      }
      if (bounded["beta_r"]) {
        options.bounded_adaptive_rq.beta_r = bounded["beta_r"].as<double>();
      }
      if (bounded["beta_q"]) {
        options.bounded_adaptive_rq.beta_q = bounded["beta_q"].as<double>();
      }
      if (bounded["gamma_r_min"]) {
        options.bounded_adaptive_rq.gamma_r_min = bounded["gamma_r_min"].as<double>();
      }
      if (bounded["gamma_r_max_observation"]) {
        options.bounded_adaptive_rq.gamma_r_max_observation =
          bounded["gamma_r_max_observation"].as<double>();
      }
      if (bounded["gamma_r_max_process"]) {
        options.bounded_adaptive_rq.gamma_r_max_process =
          bounded["gamma_r_max_process"].as<double>();
      }
      if (bounded["lambda_vrw_max"]) {
        options.bounded_adaptive_rq.lambda_vrw_max =
          bounded["lambda_vrw_max"].as<double>();
      }
      if (bounded["lambda_arw_max"]) {
        options.bounded_adaptive_rq.lambda_arw_max =
          bounded["lambda_arw_max"].as<double>();
      }
      if (bounded["lambda_accbias_max"]) {
        options.bounded_adaptive_rq.lambda_accbias_max =
          bounded["lambda_accbias_max"].as<double>();
      }
      if (bounded["lambda_gyrbias_max"]) {
        options.bounded_adaptive_rq.lambda_gyrbias_max =
          bounded["lambda_gyrbias_max"].as<double>();
      }
      if (bounded["require_gps_quality_stable_for_q"]) {
        options.bounded_adaptive_rq.require_gps_quality_stable_for_q =
          bounded["require_gps_quality_stable_for_q"].as<bool>();
      }
      if (bounded["require_process_context_for_q"]) {
        options.bounded_adaptive_rq.require_process_context_for_q =
          bounded["require_process_context_for_q"].as<bool>();
      }
      if (bounded["process_context_score_start"]) {
        options.bounded_adaptive_rq.process_context_score_start =
          bounded["process_context_score_start"].as<double>();
      }
      if (bounded["mixed_observation_score_start"]) {
        options.bounded_adaptive_rq.mixed_observation_score_start =
          bounded["mixed_observation_score_start"].as<double>();
      }
      if (bounded["source_gate_enable"]) {
        options.bounded_adaptive_rq.source_gate_enable =
          bounded["source_gate_enable"].as<bool>();
      }
      if (bounded["q_source_observation_score_max"]) {
        options.bounded_adaptive_rq.q_source_observation_score_max =
          bounded["q_source_observation_score_max"].as<double>();
      }
      if (bounded["velocity_evidence_gate_enable"]) {
        options.bounded_adaptive_rq.velocity_evidence_gate_enable =
          bounded["velocity_evidence_gate_enable"].as<bool>();
      }
      if (bounded["q_high_observation_velocity_nis_ratio_min"]) {
        options.bounded_adaptive_rq.q_high_observation_velocity_nis_ratio_min =
          bounded["q_high_observation_velocity_nis_ratio_min"].as<double>();
      }
      if (bounded["q_high_observation_velocity_residual_h_min"]) {
        options.bounded_adaptive_rq.q_high_observation_velocity_residual_h_min =
          bounded["q_high_observation_velocity_residual_h_min"].as<double>();
      }
      if (bounded["velocity_evidence_time_tolerance_sec"]) {
        options.bounded_adaptive_rq.velocity_evidence_time_tolerance_sec =
          bounded["velocity_evidence_time_tolerance_sec"].as<double>();
      }
    }
  } catch (const YAML::Exception& exception) {
    std::cerr << "Failed to load KF-GINS bounded adaptive R/Q options from '" << yaml_path
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
    applyEarlyRecoveryBiasFeedbackConfig_();
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
    applyStateUpdateDebugEnabled_();
    applyEarlyRecoveryBiasFeedbackConfig_();
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
    // clamp std: keep a small positive floor for numerical safety while still allowing
    // tighter late-cruise GNSS velocity experiments when the caller explicitly requests them.
    Eigen::Vector3d std_clamped;
    std_clamped << std::max(0.03, std::abs(std_vel.x())),
                   std::max(0.03, std::abs(std_vel.y())),
                   std::max(0.08, std::abs(std_vel.z()));
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

  bool forceRollPitch(double t, double roll_deg, double pitch_deg,
                      double roll_pitch_std_deg) override
  {
    const double roll_rad = D2R(roll_deg);
    const double pitch_rad = D2R(pitch_deg);
    const double roll_pitch_std_rad = D2R(std::max(0.1, std::abs(roll_pitch_std_deg)));
    engine_->forceRollPitch(roll_rad, pitch_rad, roll_pitch_std_rad, t);
    last_time_ = t;
    fillState_(engine_->getNavState(), last_state_);
    return true;
  }

  bool reopenVerticalCovariance(double pos_std_m, double vel_std_mps,
                                double accbias_std_z_mps2) override
  {
    if (!engine_) {
      return false;
    }
    return engine_->reopenVerticalCovariance(pos_std_m, vel_std_mps, accbias_std_z_mps2);
  }

  bool reopenYawCovariance(double t, double yaw_std_deg,
                           const std::string& reason) override
  {
    if (!engine_) {
      return false;
    }
    const double yaw_std_rad = D2R(std::max(0.1, std::abs(yaw_std_deg)));
    const bool changed = engine_->reopenYawCovariance(yaw_std_rad, t, reason);
    if (changed) {
      last_time_ = t;
      fillState_(engine_->getNavState(), last_state_);
    }
    return changed;
  }

  bool reopenHorizontalPositionCovariance(double pos_std_h_m,
                                          double offdiag_corr_limit,
                                          const std::string& reason) override
  {
    if (!engine_) {
      return false;
    }
    return engine_->reopenHorizontalPositionCovariance(
      pos_std_h_m, offdiag_corr_limit, reason);
  }

  void setPropagationNoiseScale(double arw_q_scale, double vrw_q_scale,
                                double gyrbias_q_scale,
                                double accbias_q_scale) override
  {
    if (!engine_) {
      return;
    }
    engine_->setPropagationNoiseScale(
      arw_q_scale, vrw_q_scale, gyrbias_q_scale, accbias_q_scale);
  }

  void setAdaptiveRQProcessContext(bool valid, double score) override
  {
    if (!engine_) {
      return;
    }
    engine_->setAdaptiveRQProcessContext(valid, score);
  }

  void setAdaptiveRQSourceGate(
    bool allowed, double confidence, const std::string & reason) override
  {
    if (!engine_) {
      return;
    }
    engine_->setAdaptiveRQSourceGate(allowed, confidence, reason);
  }

  void configureEarlyRecoveryBiasFeedback(
    bool debug_enable, bool apply_enable, double history_sec,
    double min_armed_time_sec, double max_armed_time_sec,
    double ba_z_mean_max_mps2, double residual_u_mean_max_m,
    double core_gnss_u_mean_min_m, double dx_ba_z_sum_max_mps2,
    int min_history_rows, double negative_dx_scale) override
  {
    early_recovery_bias_feedback_debug_enable_ = debug_enable;
    early_recovery_bias_feedback_apply_enable_ = apply_enable;
    early_recovery_bias_feedback_history_sec_ = history_sec;
    early_recovery_bias_feedback_min_armed_time_sec_ = min_armed_time_sec;
    early_recovery_bias_feedback_max_armed_time_sec_ = max_armed_time_sec;
    early_recovery_bias_feedback_ba_z_mean_max_mps2_ = ba_z_mean_max_mps2;
    early_recovery_bias_feedback_residual_u_mean_max_m_ = residual_u_mean_max_m;
    early_recovery_bias_feedback_core_gnss_u_mean_min_m_ = core_gnss_u_mean_min_m;
    early_recovery_bias_feedback_dx_ba_z_sum_max_mps2_ = dx_ba_z_sum_max_mps2;
    early_recovery_bias_feedback_min_history_rows_ = min_history_rows;
    early_recovery_bias_feedback_negative_dx_scale_ = negative_dx_scale;
    applyEarlyRecoveryBiasFeedbackConfig_();
  }

  void setEarlyRecoveryBiasFeedbackContext(
    double armed_time_sec, double core_gnss_u_m) override
  {
    if (!engine_) {
      return;
    }
    engine_->setEarlyRecoveryBiasFeedbackContext(armed_time_sec, core_gnss_u_m);
  }

  KFCoreSnapshot snapshot() const override
  {
    KFCoreSnapshot snapshot;
    if (!engine_) {
      return snapshot;
    }
    const GIEngineSnapshot engine_snapshot = engine_->snapshot();
    snapshot.valid = engine_snapshot.valid;
    snapshot.time_sec = engine_snapshot.timestamp;
    fillState_(engine_snapshot.nav_state, snapshot.state);
    snapshot.state.sow = engine_snapshot.timestamp;
    snapshot.imu_error = convertImuError_(engine_snapshot.nav_state.imuerror);
    snapshot.covariance = engine_snapshot.covariance;
    snapshot.error_state = engine_snapshot.dx;
    snapshot.previous_imu = convertImuSample_(engine_snapshot.imupre);
    snapshot.current_imu = convertImuSample_(engine_snapshot.imucur);
    snapshot.adaptive_r = convertAdaptiveRMemory_(engine_snapshot.adaptive_r);
    snapshot.adaptive_rq = convertAdaptiveRQMemory_(engine_snapshot.adaptive_rq);
    return snapshot;
  }

  bool restore(const KFCoreSnapshot & snapshot,
               const KFCoreRestorePolicy & policy) override
  {
    if (!engine_ || !snapshot.valid) {
      return false;
    }

    GIEngineSnapshot engine_snapshot;
    engine_snapshot.valid = snapshot.valid;
    engine_snapshot.timestamp = snapshot.time_sec;
    engine_snapshot.nav_state.pos <<
      D2R(snapshot.state.lat_deg),
      D2R(snapshot.state.lon_deg),
      snapshot.state.h_m;
    engine_snapshot.nav_state.vel <<
      snapshot.state.vN,
      snapshot.state.vE,
      snapshot.state.vD;
    engine_snapshot.nav_state.euler <<
      D2R(snapshot.state.roll_deg),
      D2R(snapshot.state.pitch_deg),
      D2R(snapshot.state.yaw_deg);
    engine_snapshot.nav_state.imuerror = convertImuError_(snapshot.imu_error);
    engine_snapshot.covariance = snapshot.covariance;
    engine_snapshot.dx =
      (snapshot.error_state.rows() == 21 && snapshot.error_state.cols() == 1)
        ? snapshot.error_state
        : Eigen::VectorXd::Zero(21);
    engine_snapshot.imupre = convertImuSample_(snapshot.previous_imu);
    engine_snapshot.imucur = convertImuSample_(snapshot.current_imu);
    engine_snapshot.adaptive_r = convertAdaptiveRMemory_(snapshot.adaptive_r);
    engine_snapshot.adaptive_rq = convertAdaptiveRQMemory_(snapshot.adaptive_rq);

    GIEngineRestorePolicy engine_policy;
    engine_policy.copy_nominal_state = policy.copy_nominal_state;
    engine_policy.copy_covariance = policy.copy_covariance;
    engine_policy.copy_imu_error = policy.copy_imu_error;
    engine_policy.copy_imu_buffer = policy.copy_imu_buffer;
    engine_policy.copy_adaptive_r_memory = policy.copy_adaptive_r_memory;
    engine_policy.copy_adaptive_rq_memory = policy.copy_adaptive_rq_memory;
    engine_policy.reset_adaptive_r_memory = policy.reset_adaptive_r_memory;
    engine_policy.reset_adaptive_rq_memory = policy.reset_adaptive_rq_memory;
    engine_policy.covariance_inflation_factor =
      policy.covariance_inflation_factor;
    engine_policy.require_spd_covariance = policy.require_spd_covariance;
    engine_policy.reason = policy.reason;

    if (!engine_->restore(engine_snapshot, engine_policy)) {
      return false;
    }
    fillState_(engine_->getNavState(), last_state_);
    last_state_.sow = snapshot.time_sec;
    if (std::isfinite(snapshot.time_sec)) {
      last_time_ = snapshot.time_sec;
    }
    return true;
  }

  // ★ 新增：实现接口所需 reset（不直接调私有 initialize）
  bool reset(double lat_deg, double lon_deg, double h_m, double yaw_deg) override
  {
    // 更新 options_ 初值（位置、姿态），然后重建引擎
    const Vector3d reset_euler(
      D2R(cfg_.init_roll_deg),
      D2R(cfg_.init_pitch_deg),
      D2R(yaw_deg));
    Vector3d init_pos_blh(D2R(lat_deg), D2R(lon_deg), h_m);   // BLH(rad,rad,m)
    if (options_.antlever.norm() > 1e-9) {
      const Eigen::Matrix3d cbn = Rotation::euler2matrix(reset_euler);
      const Vector3d lever_n = cbn * options_.antlever;
      init_pos_blh -= Earth::DRi(init_pos_blh) * lever_n;
      std::cout << "KF-GINS reset compensated antenna lever: lever_n_ned_m="
                << lever_n.transpose() << std::endl;
    }

    options_.initstate.pos = init_pos_blh;
    options_.initstate.vel.setZero();
    options_.initstate.euler = reset_euler;

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
    applyStateUpdateDebugEnabled_();
    applyEarlyRecoveryBiasFeedbackConfig_();
    // 让 current() 在首次 IMU 推进前也能反映 reset 后的初值，避免上层读到空状态。
    last_state_.lat_deg = R2D(init_pos_blh[0]);
    last_state_.lon_deg = R2D(init_pos_blh[1]);
    last_state_.h_m = init_pos_blh[2];
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

  Eigen::MatrixXd covariance() const override
  {
    if (!engine_) {
      return Eigen::MatrixXd();
    }
    return engine_->getCovariance();
  }

  ObservationDebug lastObservationDebug() const override
  {
    ObservationDebug debug;
    if (!engine_) {
      return debug;
    }

    const ObservationDebugInfo & engine_debug = engine_->lastObservationDebug();
    debug.sequence = engine_debug.sequence;
    debug.valid = engine_debug.valid;
    debug.gnss_position_applied = engine_debug.gnss_position_applied;
    debug.gnss_velocity_applied = engine_debug.gnss_velocity_applied;
    debug.update_time_sec = engine_debug.update_time_sec;
    debug.update_mode = engine_debug.update_mode;
    debug.gnss_position_residual_neu_m = engine_debug.gnss_position_residual_neu_m;
    debug.gnss_position_std_neu_m = engine_debug.gnss_position_std_neu_m;
    debug.gnss_position_innovation_cov_neu_m2 =
      engine_debug.gnss_position_innovation_cov_neu_m2;
    debug.gnss_position_nis_h_2d = engine_debug.gnss_position_nis_h_2d;
    debug.gnss_position_nis_u_1d = engine_debug.gnss_position_nis_u_1d;
    debug.gnss_position_nis_3d = engine_debug.gnss_position_nis_3d;
    debug.gnss_position_gate_threshold_nis =
      engine_debug.gnss_position_gate_threshold_nis;
    debug.gnss_position_update_accepted =
      engine_debug.gnss_position_update_accepted;
    debug.gnss_position_update_rejected =
      engine_debug.gnss_position_update_rejected;
    debug.gnss_position_update_reason =
      engine_debug.gnss_position_update_reason;
    debug.gnss_velocity_residual_ned_mps = engine_debug.gnss_velocity_residual_ned_mps;
    debug.gnss_velocity_std_ned_mps = engine_debug.gnss_velocity_std_ned_mps;
    debug.gnss_position_adaptive_r =
      convertBoundedAdaptiveR_(engine_debug.gnss_position_adaptive_r);
    debug.gnss_velocity_adaptive_r =
      convertBoundedAdaptiveR_(engine_debug.gnss_velocity_adaptive_r);
    return debug;
  }

  StateUpdateDebug lastStateUpdateDebug() const override
  {
    StateUpdateDebug event;
    if (!engine_) {
      return event;
    }
    const StateUpdateDebugInfo & engine_event = engine_->lastStateUpdateDebug();
    event.sequence = engine_event.sequence;
    event.valid = engine_event.valid;
    event.applied = engine_event.applied;
    event.event_type = engine_event.event_type;
    event.reason = engine_event.reason;
    event.update_time_sec = engine_event.update_time_sec;
    event.update_mode = engine_event.update_mode;
    event.dx = engine_event.dx;
    event.pos_blh_before_rad_m = engine_event.pos_blh_before_rad_m;
    event.pos_blh_after_rad_m = engine_event.pos_blh_after_rad_m;
    event.vel_before_ned_mps = engine_event.vel_before_ned_mps;
    event.vel_after_ned_mps = engine_event.vel_after_ned_mps;
    event.euler_before_rad = engine_event.euler_before_rad;
    event.euler_after_rad = engine_event.euler_after_rad;
    event.gyrbias_before_radps = engine_event.gyrbias_before_radps;
    event.gyrbias_after_radps = engine_event.gyrbias_after_radps;
    event.accbias_before_mps2 = engine_event.accbias_before_mps2;
    event.accbias_after_mps2 = engine_event.accbias_after_mps2;
    event.covariance_before = engine_event.covariance_before;
    event.covariance_after = engine_event.covariance_after;
    event.kalman_gain = engine_event.kalman_gain;
    event.gnss_position_observation_valid =
      engine_event.gnss_position_observation_valid;
    event.observation_sequence = engine_event.observation_sequence;
    event.gnss_position_residual_neu_m =
      engine_event.gnss_position_residual_neu_m;
    event.gnss_position_std_neu_m =
      engine_event.gnss_position_std_neu_m;
    event.gnss_position_innovation_cov_neu_m2 =
      engine_event.gnss_position_innovation_cov_neu_m2;
    event.gnss_position_nis_h_2d = engine_event.gnss_position_nis_h_2d;
    event.gnss_position_nis_u_1d = engine_event.gnss_position_nis_u_1d;
    event.gnss_position_nis_3d = engine_event.gnss_position_nis_3d;
    event.gnss_position_gate_threshold_nis =
      engine_event.gnss_position_gate_threshold_nis;
    event.gnss_position_update_accepted =
      engine_event.gnss_position_update_accepted;
    event.gnss_position_update_rejected =
      engine_event.gnss_position_update_rejected;
    event.gnss_position_update_reason =
      engine_event.gnss_position_update_reason;
    event.gnss_position_adaptive_r =
      convertBoundedAdaptiveR_(engine_event.gnss_position_adaptive_r);
    event.gnss_velocity_adaptive_r =
      convertBoundedAdaptiveR_(engine_event.gnss_velocity_adaptive_r);
    event.early_recovery_bias_feedback =
      convertEarlyRecoveryBiasFeedback_(engine_event.early_recovery_bias_feedback);
    return event;
  }

  StateUpdateDebug lastGnssPositionStateUpdateDebug() const override
  {
    StateUpdateDebug event;
    if (!engine_) {
      return event;
    }
    const StateUpdateDebugInfo & engine_event = engine_->lastGnssPositionStateUpdateDebug();
    event.sequence = engine_event.sequence;
    event.valid = engine_event.valid;
    event.applied = engine_event.applied;
    event.event_type = engine_event.event_type;
    event.reason = engine_event.reason;
    event.update_time_sec = engine_event.update_time_sec;
    event.update_mode = engine_event.update_mode;
    event.dx = engine_event.dx;
    event.pos_blh_before_rad_m = engine_event.pos_blh_before_rad_m;
    event.pos_blh_after_rad_m = engine_event.pos_blh_after_rad_m;
    event.vel_before_ned_mps = engine_event.vel_before_ned_mps;
    event.vel_after_ned_mps = engine_event.vel_after_ned_mps;
    event.euler_before_rad = engine_event.euler_before_rad;
    event.euler_after_rad = engine_event.euler_after_rad;
    event.gyrbias_before_radps = engine_event.gyrbias_before_radps;
    event.gyrbias_after_radps = engine_event.gyrbias_after_radps;
    event.accbias_before_mps2 = engine_event.accbias_before_mps2;
    event.accbias_after_mps2 = engine_event.accbias_after_mps2;
    event.covariance_before = engine_event.covariance_before;
    event.covariance_after = engine_event.covariance_after;
    event.kalman_gain = engine_event.kalman_gain;
    event.gnss_position_observation_valid =
      engine_event.gnss_position_observation_valid;
    event.observation_sequence = engine_event.observation_sequence;
    event.gnss_position_residual_neu_m =
      engine_event.gnss_position_residual_neu_m;
    event.gnss_position_std_neu_m =
      engine_event.gnss_position_std_neu_m;
    event.gnss_position_innovation_cov_neu_m2 =
      engine_event.gnss_position_innovation_cov_neu_m2;
    event.gnss_position_nis_h_2d = engine_event.gnss_position_nis_h_2d;
    event.gnss_position_nis_u_1d = engine_event.gnss_position_nis_u_1d;
    event.gnss_position_nis_3d = engine_event.gnss_position_nis_3d;
    event.gnss_position_gate_threshold_nis =
      engine_event.gnss_position_gate_threshold_nis;
    event.gnss_position_update_accepted =
      engine_event.gnss_position_update_accepted;
    event.gnss_position_update_rejected =
      engine_event.gnss_position_update_rejected;
    event.gnss_position_update_reason =
      engine_event.gnss_position_update_reason;
    event.gnss_position_adaptive_r =
      convertBoundedAdaptiveR_(engine_event.gnss_position_adaptive_r);
    event.gnss_velocity_adaptive_r =
      convertBoundedAdaptiveR_(engine_event.gnss_velocity_adaptive_r);
    event.early_recovery_bias_feedback =
      convertEarlyRecoveryBiasFeedback_(engine_event.early_recovery_bias_feedback);
    return event;
  }

  void setStateUpdateDebugEnabled(bool enabled) override
  {
    state_update_debug_enabled_ = enabled;
    applyStateUpdateDebugEnabled_();
  }

  std::vector<StateUpdateDebug> stateUpdateDebugEvents() const override
  {
    std::vector<StateUpdateDebug> out;
    if (!engine_) {
      return out;
    }
    const std::vector<StateUpdateDebugInfo> & engine_events = engine_->stateUpdateDebugEvents();
    out.reserve(engine_events.size());
    for (const auto & engine_event : engine_events) {
      StateUpdateDebug event;
      event.sequence = engine_event.sequence;
      event.valid = engine_event.valid;
      event.applied = engine_event.applied;
      event.event_type = engine_event.event_type;
      event.reason = engine_event.reason;
      event.update_time_sec = engine_event.update_time_sec;
      event.update_mode = engine_event.update_mode;
      event.dx = engine_event.dx;
      event.pos_blh_before_rad_m = engine_event.pos_blh_before_rad_m;
      event.pos_blh_after_rad_m = engine_event.pos_blh_after_rad_m;
      event.vel_before_ned_mps = engine_event.vel_before_ned_mps;
      event.vel_after_ned_mps = engine_event.vel_after_ned_mps;
      event.euler_before_rad = engine_event.euler_before_rad;
      event.euler_after_rad = engine_event.euler_after_rad;
      event.gyrbias_before_radps = engine_event.gyrbias_before_radps;
      event.gyrbias_after_radps = engine_event.gyrbias_after_radps;
      event.accbias_before_mps2 = engine_event.accbias_before_mps2;
      event.accbias_after_mps2 = engine_event.accbias_after_mps2;
      event.covariance_before = engine_event.covariance_before;
      event.covariance_after = engine_event.covariance_after;
      event.kalman_gain = engine_event.kalman_gain;
      event.gnss_position_observation_valid =
        engine_event.gnss_position_observation_valid;
      event.observation_sequence = engine_event.observation_sequence;
      event.gnss_position_residual_neu_m =
        engine_event.gnss_position_residual_neu_m;
      event.gnss_position_std_neu_m =
        engine_event.gnss_position_std_neu_m;
      event.gnss_position_innovation_cov_neu_m2 =
        engine_event.gnss_position_innovation_cov_neu_m2;
      event.gnss_position_nis_h_2d = engine_event.gnss_position_nis_h_2d;
      event.gnss_position_nis_u_1d = engine_event.gnss_position_nis_u_1d;
      event.gnss_position_nis_3d = engine_event.gnss_position_nis_3d;
      event.gnss_position_gate_threshold_nis =
        engine_event.gnss_position_gate_threshold_nis;
      event.gnss_position_update_accepted =
        engine_event.gnss_position_update_accepted;
      event.gnss_position_update_rejected =
        engine_event.gnss_position_update_rejected;
      event.gnss_position_update_reason =
        engine_event.gnss_position_update_reason;
      event.gnss_position_adaptive_r =
        convertBoundedAdaptiveR_(engine_event.gnss_position_adaptive_r);
      event.gnss_velocity_adaptive_r =
        convertBoundedAdaptiveR_(engine_event.gnss_velocity_adaptive_r);
      event.early_recovery_bias_feedback =
        convertEarlyRecoveryBiasFeedback_(engine_event.early_recovery_bias_feedback);
      out.push_back(event);
    }
    return out;
  }

private:
  void applyStateUpdateDebugEnabled_()
  {
    if (engine_) {
      engine_->setStateUpdateDebugEnabled(state_update_debug_enabled_);
    }
  }

  void applyEarlyRecoveryBiasFeedbackConfig_()
  {
    if (engine_) {
      engine_->configureEarlyRecoveryBiasFeedback(
        early_recovery_bias_feedback_debug_enable_,
        early_recovery_bias_feedback_apply_enable_,
        early_recovery_bias_feedback_history_sec_,
        early_recovery_bias_feedback_min_armed_time_sec_,
        early_recovery_bias_feedback_max_armed_time_sec_,
        early_recovery_bias_feedback_ba_z_mean_max_mps2_,
        early_recovery_bias_feedback_residual_u_mean_max_m_,
        early_recovery_bias_feedback_core_gnss_u_mean_min_m_,
        early_recovery_bias_feedback_dx_ba_z_sum_max_mps2_,
        early_recovery_bias_feedback_min_history_rows_,
        early_recovery_bias_feedback_negative_dx_scale_);
    }
  }

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
  bool state_update_debug_enabled_{false};
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
