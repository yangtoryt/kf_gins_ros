#!/usr/bin/env python3
"""Autonomous multi-route IEKF tuning and validation loop.

This script implements the 2026-05-12 night plan:

* screen only fixed, non-gated IEKF parameters;
* use shortgen11 as a diagnostic route, not a generalization claim;
* require T1 development/control validation before any new holdout route;
* keep every candidate run reproducible through generated overlays and wrappers.

It intentionally does not edit PX4.  The PX4 tree is only used by the existing
SITL launch scripts.
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import io
import json
import math
import os
import re
import shutil
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence


WS_ROOT = Path("/home/yang/kf_gins_ws")
PX4_ROOT = Path("/home/yang/PX4-Autopilot")
PX4_ROOTFS_LOG_DIR = PX4_ROOT / "build/px4_sitl_default/rootfs/log"
SCRIPT = WS_ROOT / "src/kf_gins_ros2_native/scripts/autonomous_iekf_perf_loop.py"
OUT_DIR = WS_ROOT / "artifacts/manual/autonomous_iekf_perf_2026-05-12_night"
PLANS_DIR = WS_ROOT / "artifacts/manual/short_route_generalization_2026-05-07/plans"
MANUAL_SESSION = WS_ROOT / "src/kf_gins_ros2_native/scripts/manual_mainline_postflight_session.sh"
MISSION_SMOKE = WS_ROOT / "src/kf_gins_ros2_native/scripts/px4_mission_smoke.py"
GT_DIAG = WS_ROOT / "src/kf_gins_ros2_native/scripts/offline_groundtruth_convergence_diagnostic.py"
ULOG_GPS_GT_DIAG = WS_ROOT / "src/kf_gins_ros2_native/scripts/offline_ulog_gps_groundtruth_diagnostic.py"
BASE_KFGINS = WS_ROOT / "src/kf_gins_ros2_native/config/kfgins_sim_fixed.yaml"
BASE_CORE = WS_ROOT / "src/kf_gins_ros2_native/config/kf_gins_core_sim_no_sage.yaml"
GT_PROJECTION_MODE = "raw_wgs84_enu"
ULOG_TOPICS = [
    "vehicle_gps_position",
    "sensor_gps",
    "vehicle_local_position_groundtruth",
    "vehicle_global_position_groundtruth",
]

LEADERBOARD_FIELDS = [
    "timestamp",
    "candidate_id",
    "candidate_family",
    "stage",
    "route_id",
    "route_group",
    "run_label",
    "run_dir",
    "clean_run",
    "validity_status",
    "expected_success_seq",
    "success_seq",
    "mission_count",
    "simulator_poll_timeout",
    "required_files_ok",
    "groundtruth_summary",
    "mission_all_iekf_mean",
    "mission_all_ekf2_mean",
    "mission_all_delta",
    "main_40_180_iekf_mean",
    "main_40_180_ekf2_mean",
    "main_40_180_delta",
    "win_mission_all",
    "win_main_40_180",
    "win_140_180",
    "win_140_160",
    "win_160_180",
    "window_140_180_iekf_mean",
    "window_140_180_ekf2_mean",
    "window_140_180_delta",
    "window_140_160_iekf_mean",
    "window_140_160_ekf2_mean",
    "window_140_160_delta",
    "window_160_180_iekf_mean",
    "window_160_180_ekf2_mean",
    "window_160_180_delta",
    "rtl_all_iekf_mean",
    "rtl_all_ekf2_mean",
    "rtl_all_delta",
    "baseline_main_40_180_iekf_mean",
    "baseline_delta_main_40_180",
    "sync_residual_h_mean_m",
    "sync_dx_pos_h_mean_m",
    "sync_dx_over_residual_mean",
    "ulog_path",
    "status",
    "notes",
]

BASELINE_FIELDS = [
    "route_id",
    "source",
    "run_dir",
    "groundtruth_summary",
    "mission_all_iekf_mean",
    "mission_all_ekf2_mean",
    "main_40_180_iekf_mean",
    "main_40_180_ekf2_mean",
    "window_140_180_iekf_mean",
    "window_140_180_ekf2_mean",
    "window_140_160_iekf_mean",
    "window_140_160_ekf2_mean",
    "window_160_180_iekf_mean",
    "window_160_180_ekf2_mean",
]

REQUIRED_RUN_FILES = [
    "ekf_iekf_pairs.csv",
    "state_update_debug.csv",
    "gps_vs_pose.csv",
    "gps_vs_second_pose.csv",
]

DISABLED_MECHANISM_ENV = {
    "PX4_SAFE_COMPARE_PUBLISH_PX4_SPHERE_PROJECTION": "false",
    "PX4_SAFE_COMPARE_SEGMENT_TIMING_GATE_PROJECTION_ENABLE": "false",
    "PX4_SAFE_COMPARE_SEGMENT_TIMING_GATE_PROJECTION_APPLY_MISSION": "false",
    "PX4_SAFE_COMPARE_SEGMENT_TIMING_GATE_PROJECTION_APPLY_RTL": "false",
    "PX4_SAFE_COMPARE_SEGMENT_TIMING_GATE_PROJECTION_APPLY_OTHER": "false",
    "PX4_SAFE_COMPARE_ACCBIAS_Z_HISTORY_PROJECTION_ENABLE": "false",
    "PX4_SAFE_COMPARE_EARLY_RECOVERY_BIAS_FEEDBACK_DEBUG_ENABLE": "false",
    "PX4_SAFE_COMPARE_EARLY_RECOVERY_BIAS_FEEDBACK_APPLY_ENABLE": "false",
    "PX4_SAFE_COMPARE_GNSS_POSITION_RESPONSE_BOOST_ENABLE": "false",
    "PX4_SAFE_COMPARE_GNSS_POSITION_GAIN_RESPONSE_ENABLE": "false",
    "PX4_SAFE_COMPARE_TURN_RATE_PROPAGATION_NOISE_PROBE_ENABLE": "false",
    "PX4_SAFE_COMPARE_GNSS_VELOCITY_OUTWARD_DAMPING_ENABLE": "false",
    "PX4_SAFE_COMPARE_TURN_POSTTURN_NATIVE_VELOCITY_DEWEIGHT_ENABLE": "false",
    "PX4_SAFE_COMPARE_ADAPTIVE_GNSS_POS_WEIGHT_ENABLE": "false",
    "PX4_SAFE_COMPARE_HORIZONTAL_CONSISTENCY_SUPERVISOR_ENABLE": "false",
    "PX4_SAFE_COMPARE_MISSION_COV_HYGIENE_ENABLE": "false",
    "PX4_SAFE_COMPARE_MOTION_GNSS_POS_WEIGHT_ENABLE": "false",
    "PX4_SAFE_COMPARE_GNSS_POS_RECOVERY_WEIGHT_ENABLE": "false",
    "PX4_SAFE_COMPARE_ACCBIAS_Z_PROPAGATION_PROBE_ENABLE": "false",
    "PX4_SAFE_COMPARE_ACCBIAS_Z_PROPAGATION_PROBE_APPLY_NOISE_SCALE": "false",
    "PX4_SAFE_COMPARE_NATIVE_GNSS_VELOCITY_OUTLIER_GUARD_ENABLE": "false",
    "PX4_SAFE_COMPARE_NATIVE_GNSS_VELOCITY_LOW_SPEED_TURN_SOURCE_GUARD_ENABLE": "false",
}


@dataclass(frozen=True)
class Route:
    route_id: str
    stage: str
    group: str
    plan_name: str
    role: str
    holdout_rank: int | None = None

    @property
    def plan_path(self) -> Path:
        return PLANS_DIR / self.plan_name


@dataclass(frozen=True)
class Candidate:
    candidate_id: str
    family: str
    sim_gnss_std_h_m: float
    armed_cruise_gnss_pos_std_h_m: float
    gnss_vel_std_floor_h_mps: float
    native_gnss_speed_std_scale: float
    imunoise_vrw: float
    offline_rank: int
    expanded: bool = False
    sage_husa_enable: bool = False
    sage_husa_alpha: float = 0.95
    sage_husa_min_var_factor: float = 0.1
    enable_gnss_velocity_update: bool = True
    imunoise_arw: float = 0.24
    imunoise_gbstd: float = 50.0
    imunoise_abstd: float = 250.0
    publish_state_after_gnss_update: bool = False

    def env(self, kfgins_cfg: Path, core_cfg: Path) -> dict[str, str]:
        return {
            "PX4_SAFE_COMPARE_KF_GINS_PARAM_FILE": str(kfgins_cfg),
            "PX4_SAFE_COMPARE_KF_GINS_CORE_CONFIG_FILE": str(core_cfg),
            "PX4_SAFE_COMPARE_USE_SIM_GNSS_STD": "true",
            "PX4_SAFE_COMPARE_SIM_GNSS_STD_H_M": fmt_float(self.sim_gnss_std_h_m),
            "PX4_SAFE_COMPARE_SIM_GNSS_STD_U_M": "0.10",
            "PX4_SAFE_COMPARE_ARMED_CRUISE_GNSS_POS_OVERRIDE_ENABLE": "true",
            "PX4_SAFE_COMPARE_ARMED_CRUISE_GNSS_POS_STD_H_M": fmt_float(self.armed_cruise_gnss_pos_std_h_m),
            "PX4_SAFE_COMPARE_ARMED_CRUISE_GNSS_POS_STD_U_M": "0.08",
        }


ROUTES: dict[str, Route] = {
    "shortgen11": Route(
        "shortgen11",
        "T0",
        "diagnostic",
        "shortgen11_high_clearance_ladder_sitl_home.plan",
        "fast diagnostic only; not final generalization evidence",
    ),
    "shortgen01": Route(
        "shortgen01",
        "T1",
        "development/control",
        "shortgen01_s_bend_sitl_home.plan",
        "development/control route",
    ),
    "shortgen02": Route(
        "shortgen02",
        "T1",
        "development/control",
        "shortgen02_box_uturn_sitl_home.plan",
        "development/control route",
    ),
    "shortgen03": Route(
        "shortgen03",
        "T1",
        "development/control",
        "shortgen03_figure8_sitl_home.plan",
        "development/control route",
    ),
    "shortgen04": Route(
        "shortgen04",
        "T1",
        "known holdout/control",
        "shortgen04_ladder_sitl_home.plan",
        "known clean holdout; must not create a new local miss",
    ),
    "shortgen07": Route(
        "shortgen07",
        "T2",
        "new holdout",
        "shortgen07_safe_ladder_shift_sitl_home.plan",
        "new holdout pool: shifted ladder",
        1,
    ),
    "shortgen08": Route(
        "shortgen08",
        "T2",
        "new holdout",
        "shortgen08_safe_bowtie_sitl_home.plan",
        "new holdout pool: bowtie",
        2,
    ),
    "shortgen09": Route(
        "shortgen09",
        "T2",
        "new holdout",
        "shortgen09_safe_stair_return_sitl_home.plan",
        "new holdout pool: stair return",
        3,
    ),
    "shortgen10": Route(
        "shortgen10",
        "T2",
        "new holdout",
        "shortgen10_safe_ladder_subset_sitl_home.plan",
        "new holdout pool: ladder subset",
        4,
    ),
    "shortgen13": Route(
        "shortgen13",
        "T2",
        "new holdout",
        "shortgen13_high_clearance_cross_turn_sitl_home.plan",
        "new holdout pool: cross turn",
        5,
    ),
    "shortgen16": Route(
        "shortgen16",
        "T2",
        "new holdout reserve",
        "shortgen16_open_ne_bowtie_sitl_home.plan",
        "reserve holdout if a T2 route fails validity precheck",
        6,
    ),
    "shortgen17": Route(
        "shortgen17",
        "T3",
        "expanded clean holdout",
        "shortgen17_clean_straight_sitl_home.plan",
        "expanded clean holdout: straight line",
        7,
    ),
    "shortgen18": Route(
        "shortgen18",
        "T3",
        "expanded clean holdout",
        "shortgen18_clean_rectangle_sitl_home.plan",
        "expanded clean holdout: rectangle",
        8,
    ),
    "shortgen19": Route(
        "shortgen19",
        "T3",
        "expanded clean holdout",
        "shortgen19_clean_l_turn_sitl_home.plan",
        "expanded clean holdout: L-turn",
        9,
    ),
    "shortgen20": Route(
        "shortgen20",
        "T3",
        "expanded clean holdout",
        "shortgen20_clean_s_polyline_sitl_home.plan",
        "expanded clean holdout: S/polyline",
        10,
    ),
    "shortgen21": Route(
        "shortgen21",
        "T3",
        "expanded clean holdout",
        "shortgen21_clean_out_and_back_sitl_home.plan",
        "expanded clean holdout: out-and-back",
        11,
    ),
    "shortgen22": Route(
        "shortgen22",
        "GNSR-DEV0",
        "route-reset development",
        "shortgen22_gnsr_dev0_bowtie_offset_sitl_home.plan",
        "H-GNSR-v1 DEV0 route A: turn-rich offset bowtie",
        12,
    ),
    "shortgen23": Route(
        "shortgen23",
        "GNSR-DEV0",
        "route-reset development",
        "shortgen23_gnsr_dev0_straight_jog_negative_sitl_home.plan",
        "H-GNSR-v1 DEV0 route B: shortgen17-like straight-jog negative geometry",
        13,
    ),
    "shortgen24": Route(
        "shortgen24",
        "GMOC-DEV0",
        "route-reset development",
        "shortgen24_gmoc_dev0_turnrich_sitl_home.plan",
        "H-GMOC-v1 DEV0 route R1: turn-rich high-observability route",
        14,
    ),
    "shortgen25": Route(
        "shortgen25",
        "GMOC-DEV0",
        "route-reset development",
        "shortgen25_gmoc_dev0_lowobs_sitl_home.plan",
        "H-GMOC-v1 DEV0 route R2: low-observability straight-jog negative route",
        15,
    ),
    "shortgen26": Route(
        "shortgen26",
        "GMOC-DEV0",
        "route-reset development",
        "shortgen26_gmoc_dev0_mixed_sitl_home.plan",
        "H-GMOC-v1 DEV0 route R3: mixed straight/chicane route",
        16,
    ),
}

INITIAL_CANDIDATES: list[Candidate] = [
    Candidate("cand01_pos06_arm05_vel08_vrw24", "fixed_noise_grid", 0.06, 0.05, 0.08, 0.35, 0.24, 1),
    Candidate("cand02_pos06_arm05_vel08_vrw28", "fixed_noise_grid", 0.06, 0.05, 0.08, 0.35, 0.28, 2),
    Candidate("cand03_pos07_arm05_vel07_vrw24", "fixed_noise_grid", 0.07, 0.05, 0.07, 0.30, 0.24, 3),
    Candidate("cand04_pos08_arm06_vel08_vrw28", "fixed_noise_grid", 0.08, 0.06, 0.08, 0.30, 0.28, 4),
    Candidate("cand05_pos05_arm045_vel08_vrw24", "fixed_noise_grid", 0.05, 0.045, 0.08, 0.35, 0.24, 5),
]

EXPANDED_CANDIDATES: list[Candidate] = [
    Candidate("cand06_pos06_arm055_vel06_vrw32", "expanded_fixed_noise_grid", 0.06, 0.055, 0.06, 0.28, 0.32, 6, True),
    Candidate("cand07_pos07_arm06_vel06_vrw32", "expanded_fixed_noise_grid", 0.07, 0.06, 0.06, 0.28, 0.32, 7, True),
    Candidate("cand08_pos05_arm05_vel10_vrw28", "expanded_fixed_noise_grid", 0.05, 0.05, 0.10, 0.35, 0.28, 8, True),
    Candidate("cand09_pos08_arm055_vel06_vrw24", "expanded_fixed_noise_grid", 0.08, 0.055, 0.06, 0.25, 0.24, 9, True),
    Candidate("cand10_pos06_arm045_vel10_vrw32", "expanded_fixed_noise_grid", 0.06, 0.045, 0.10, 0.32, 0.32, 10, True),
]

RESCUE_CANDIDATES_ROUND2: list[Candidate] = [
    Candidate("cand11_pos04_arm04_vel08_vrw28", "rescue_fixed_noise_grid_r2", 0.04, 0.04, 0.08, 0.35, 0.28, 11, True),
    Candidate("cand12_pos05_arm04_vel06_vrw32", "rescue_fixed_noise_grid_r2", 0.05, 0.04, 0.06, 0.30, 0.32, 12, True),
    Candidate("cand13_pos07_arm045_vel05_vrw36", "rescue_fixed_noise_grid_r2", 0.07, 0.045, 0.05, 0.25, 0.36, 13, True),
    Candidate("cand14_pos09_arm06_vel06_vrw32", "rescue_fixed_noise_grid_r2", 0.09, 0.06, 0.06, 0.25, 0.32, 14, True),
    Candidate("cand15_pos06_arm05_vel05_vrw40", "rescue_fixed_noise_grid_r2", 0.06, 0.05, 0.05, 0.22, 0.40, 15, True),
]

RESCUE_CANDIDATES_ROUND3: list[Candidate] = [
    Candidate("cand16_pos04_arm035_vel10_vrw32", "rescue_fixed_noise_grid_r3", 0.04, 0.035, 0.10, 0.35, 0.32, 16, True),
    Candidate("cand17_pos07_arm055_vel04_vrw36", "rescue_fixed_noise_grid_r3", 0.07, 0.055, 0.04, 0.22, 0.36, 17, True),
    Candidate("cand18_pos05_arm045_vel05_vrw40", "rescue_fixed_noise_grid_r3", 0.05, 0.045, 0.05, 0.25, 0.40, 18, True),
    Candidate("cand19_pos08_arm05_vel04_vrw28", "rescue_fixed_noise_grid_r3", 0.08, 0.05, 0.04, 0.20, 0.28, 19, True),
    Candidate("cand20_pos06_arm035_vel08_vrw36", "rescue_fixed_noise_grid_r3", 0.06, 0.035, 0.08, 0.32, 0.36, 20, True),
]

SAGE_HUSA_CANDIDATES_ROUND4: list[Candidate] = [
    Candidate("cand21_sage_pos07_arm055_vel04_vrw36", "sage_husa_covariance_round", 0.07, 0.055, 0.04, 0.22, 0.36, 21, True, True, 0.95, 0.1),
    Candidate("cand22_sage_pos04_arm035_vel10_vrw32", "sage_husa_covariance_round", 0.04, 0.035, 0.10, 0.35, 0.32, 22, True, True, 0.95, 0.1),
    Candidate("cand23_sage_pos08_arm05_vel04_vrw28", "sage_husa_covariance_round", 0.08, 0.05, 0.04, 0.20, 0.28, 23, True, True, 0.95, 0.1),
    Candidate("cand24_sage_pos08_arm06_vel08_vrw28", "sage_husa_covariance_round", 0.08, 0.06, 0.08, 0.30, 0.28, 24, True, True, 0.95, 0.1),
    Candidate("cand25_sage_pos06_arm055_vel06_vrw32", "sage_husa_covariance_round", 0.06, 0.055, 0.06, 0.28, 0.32, 25, True, True, 0.95, 0.1),
]

SAGE_HUSA_CANDIDATES_ROUND5: list[Candidate] = [
    Candidate("cand26_sagefast_pos07_arm055_vel04_vrw36", "sage_husa_fast_covariance_round", 0.07, 0.055, 0.04, 0.22, 0.36, 26, True, True, 0.90, 0.1),
    Candidate("cand27_sagevf_pos07_arm055_vel04_vrw36", "sage_husa_fast_covariance_round", 0.07, 0.055, 0.04, 0.22, 0.36, 27, True, True, 0.80, 0.1),
    Candidate("cand28_sagefloor_pos07_arm055_vel04_vrw36", "sage_husa_fast_covariance_round", 0.07, 0.055, 0.04, 0.22, 0.36, 28, True, True, 0.90, 0.5),
    Candidate("cand29_sagefast_pos08_arm06_vel04_vrw36", "sage_husa_fast_covariance_round", 0.08, 0.06, 0.04, 0.22, 0.36, 29, True, True, 0.90, 0.1),
    Candidate("cand30_sagefast_pos07_arm055_vel04_vrw40", "sage_husa_fast_covariance_round", 0.07, 0.055, 0.04, 0.22, 0.40, 30, True, True, 0.90, 0.1),
]

BASELINE_PRESERVING_CANDIDATES_ROUND6: list[Candidate] = [
    Candidate("cand31_base_sage095", "baseline_preserving_covariance_round", 0.08, 0.06, 0.10, 0.40, 0.24, 31, True, True, 0.95, 0.1),
    Candidate("cand32_base_sage090", "baseline_preserving_covariance_round", 0.08, 0.06, 0.10, 0.40, 0.24, 32, True, True, 0.90, 0.1),
    Candidate("cand33_base_sage080", "baseline_preserving_covariance_round", 0.08, 0.06, 0.10, 0.40, 0.24, 33, True, True, 0.80, 0.1),
    Candidate("cand34_base_pos09_arm06_vel10_vrw24", "baseline_preserving_fixed_noise_round", 0.09, 0.06, 0.10, 0.40, 0.24, 34, True),
    Candidate("cand35_base_pos08_arm055_vel10_vrw24", "baseline_preserving_fixed_noise_round", 0.08, 0.055, 0.10, 0.40, 0.24, 35, True),
]

POST_VELFB_FIXED_CANDIDATES_ROUND7: list[Candidate] = [
    Candidate("cand36_velfb_pos05_arm04_vel08_vrw28", "post_velocity_feedback_fixed_grid", 0.05, 0.04, 0.08, 0.35, 0.28, 36, True),
    Candidate("cand37_velfb_pos055_arm045_vel08_vrw28", "post_velocity_feedback_fixed_grid", 0.055, 0.045, 0.08, 0.35, 0.28, 37, True),
    Candidate("cand38_velfb_pos06_arm045_vel08_vrw28", "post_velocity_feedback_fixed_grid", 0.06, 0.045, 0.08, 0.35, 0.28, 38, True),
    Candidate("cand39_velfb_pos065_arm05_vel08_vrw28", "post_velocity_feedback_fixed_grid", 0.065, 0.05, 0.08, 0.35, 0.28, 39, True),
    Candidate("cand40_velfb_pos08_arm055_vel08_vrw28", "post_velocity_feedback_fixed_grid", 0.08, 0.055, 0.08, 0.30, 0.28, 40, True),
]

VELOCITY_STRONG_FIXED_CANDIDATES_ROUND8: list[Candidate] = [
    Candidate("cand41_velstrong_pos08_arm05_vel02_scale015_vrw28", "velocity_strong_fixed_grid", 0.08, 0.05, 0.02, 0.15, 0.28, 41, True),
    Candidate("cand42_velstrong_pos07_arm05_vel02_scale015_vrw32", "velocity_strong_fixed_grid", 0.07, 0.05, 0.02, 0.15, 0.32, 42, True),
    Candidate("cand43_velstrong_pos06_arm05_vel03_scale02_vrw28", "velocity_strong_fixed_grid", 0.06, 0.05, 0.03, 0.20, 0.28, 43, True),
    Candidate("cand44_velstrong_pos08_arm055_vel03_scale015_vrw24", "velocity_strong_fixed_grid", 0.08, 0.055, 0.03, 0.15, 0.24, 44, True),
    Candidate("cand45_velstrong_pos06_arm045_vel03_scale02_vrw24", "velocity_strong_fixed_grid", 0.06, 0.045, 0.03, 0.20, 0.24, 45, True),
]

VELOCITY_OFF_DIAGNOSTIC_CANDIDATES_ROUND9: list[Candidate] = [
    Candidate("cand46_veloff_pos06_arm05_vel08_vrw28", "velocity_off_global_diagnostic", 0.06, 0.05, 0.08, 0.35, 0.28, 46, True, False, 0.95, 0.1, False),
    Candidate("cand47_veloff_pos08_arm055_vel06_vrw24", "velocity_off_global_diagnostic", 0.08, 0.055, 0.06, 0.25, 0.24, 47, True, False, 0.95, 0.1, False),
]

GLOBAL_GYRO_NOISE_CANDIDATES_ROUND10: list[Candidate] = [
    Candidate(
        "cand48_arw060_pos05_arm045_vel08_vrw24",
        "global_gyro_process_noise_round",
        0.05,
        0.045,
        0.08,
        0.35,
        0.24,
        48,
        True,
        imunoise_arw=0.60,
        imunoise_gbstd=100.0,
    ),
    Candidate(
        "cand49_arw080_sage_pos06_arm055_vel06_vrw32",
        "global_gyro_process_noise_round",
        0.06,
        0.055,
        0.06,
        0.28,
        0.32,
        49,
        True,
        True,
        0.95,
        0.1,
        imunoise_arw=0.80,
        imunoise_gbstd=150.0,
    ),
    Candidate(
        "cand50_arw060_pos08_arm055_vel06_vrw24",
        "global_gyro_process_noise_round",
        0.08,
        0.055,
        0.06,
        0.25,
        0.24,
        50,
        True,
        imunoise_arw=0.60,
        imunoise_gbstd=100.0,
    ),
]

GLOBAL_GYRO_NO_SAGE_CANDIDATES_ROUND11: list[Candidate] = [
    Candidate(
        candidate_id="cand51_arw080_pos05_arm045_vel08_vrw28",
        family="global_gyro_no_sage_split_round",
        sim_gnss_std_h_m=0.05,
        armed_cruise_gnss_pos_std_h_m=0.045,
        gnss_vel_std_floor_h_mps=0.08,
        native_gnss_speed_std_scale=0.35,
        imunoise_vrw=0.28,
        offline_rank=51,
        expanded=True,
        imunoise_arw=0.80,
        imunoise_gbstd=150.0,
    ),
    Candidate(
        candidate_id="cand52_arw080_pos045_arm04_vel08_vrw24",
        family="global_gyro_no_sage_split_round",
        sim_gnss_std_h_m=0.045,
        armed_cruise_gnss_pos_std_h_m=0.04,
        gnss_vel_std_floor_h_mps=0.08,
        native_gnss_speed_std_scale=0.35,
        imunoise_vrw=0.24,
        offline_rank=52,
        expanded=True,
        imunoise_arw=0.80,
        imunoise_gbstd=150.0,
    ),
    Candidate(
        candidate_id="cand53_arw070_pos05_arm045_vel06_vrw28",
        family="global_gyro_no_sage_split_round",
        sim_gnss_std_h_m=0.05,
        armed_cruise_gnss_pos_std_h_m=0.045,
        gnss_vel_std_floor_h_mps=0.06,
        native_gnss_speed_std_scale=0.28,
        imunoise_vrw=0.28,
        offline_rank=53,
        expanded=True,
        imunoise_arw=0.70,
        imunoise_gbstd=125.0,
    ),
]

GLOBAL_HIGH_VRW_CANDIDATES_ROUND12: list[Candidate] = [
    Candidate(
        candidate_id="cand54_vrw060_pos05_arm045_vel08",
        family="global_high_vrw_fixed_round",
        sim_gnss_std_h_m=0.05,
        armed_cruise_gnss_pos_std_h_m=0.045,
        gnss_vel_std_floor_h_mps=0.08,
        native_gnss_speed_std_scale=0.35,
        imunoise_vrw=0.60,
        offline_rank=54,
        expanded=True,
    ),
    Candidate(
        candidate_id="cand55_vrw080_pos06_arm05_vel08",
        family="global_high_vrw_fixed_round",
        sim_gnss_std_h_m=0.06,
        armed_cruise_gnss_pos_std_h_m=0.05,
        gnss_vel_std_floor_h_mps=0.08,
        native_gnss_speed_std_scale=0.35,
        imunoise_vrw=0.80,
        offline_rank=55,
        expanded=True,
    ),
    Candidate(
        candidate_id="cand56_vrw060_arw060_pos05_arm045_vel08",
        family="global_high_vrw_fixed_round",
        sim_gnss_std_h_m=0.05,
        armed_cruise_gnss_pos_std_h_m=0.045,
        gnss_vel_std_floor_h_mps=0.08,
        native_gnss_speed_std_scale=0.35,
        imunoise_vrw=0.60,
        offline_rank=56,
        expanded=True,
        imunoise_arw=0.60,
        imunoise_gbstd=100.0,
    ),
]

OUTPUT_FRESHNESS_CANDIDATES_ROUND13: list[Candidate] = [
    Candidate(
        candidate_id="cand57_pubafter_pos05_arm045_vel08_vrw24_arw060",
        family="output_freshness_global_diagnostic",
        sim_gnss_std_h_m=0.05,
        armed_cruise_gnss_pos_std_h_m=0.045,
        gnss_vel_std_floor_h_mps=0.08,
        native_gnss_speed_std_scale=0.35,
        imunoise_vrw=0.24,
        offline_rank=57,
        expanded=True,
        imunoise_arw=0.60,
        imunoise_gbstd=100.0,
        publish_state_after_gnss_update=True,
    ),
]

CANDIDATE_ROUNDS: list[list[Candidate]] = [
    INITIAL_CANDIDATES,
    EXPANDED_CANDIDATES,
    RESCUE_CANDIDATES_ROUND2,
    RESCUE_CANDIDATES_ROUND3,
    SAGE_HUSA_CANDIDATES_ROUND4,
    SAGE_HUSA_CANDIDATES_ROUND5,
    BASELINE_PRESERVING_CANDIDATES_ROUND6,
    POST_VELFB_FIXED_CANDIDATES_ROUND7,
    VELOCITY_STRONG_FIXED_CANDIDATES_ROUND8,
    VELOCITY_OFF_DIAGNOSTIC_CANDIDATES_ROUND9,
    GLOBAL_GYRO_NOISE_CANDIDATES_ROUND10,
    GLOBAL_GYRO_NO_SAGE_CANDIDATES_ROUND11,
    GLOBAL_HIGH_VRW_CANDIDATES_ROUND12,
    OUTPUT_FRESHNESS_CANDIDATES_ROUND13,
]

CANDIDATES: dict[str, Candidate] = {
    c.candidate_id: c
    for c in (
        INITIAL_CANDIDATES
        + EXPANDED_CANDIDATES
        + RESCUE_CANDIDATES_ROUND2
        + RESCUE_CANDIDATES_ROUND3
        + SAGE_HUSA_CANDIDATES_ROUND4
        + SAGE_HUSA_CANDIDATES_ROUND5
        + BASELINE_PRESERVING_CANDIDATES_ROUND6
        + POST_VELFB_FIXED_CANDIDATES_ROUND7
        + VELOCITY_STRONG_FIXED_CANDIDATES_ROUND8
        + VELOCITY_OFF_DIAGNOSTIC_CANDIDATES_ROUND9
        + GLOBAL_GYRO_NOISE_CANDIDATES_ROUND10
        + GLOBAL_GYRO_NO_SAGE_CANDIDATES_ROUND11
        + GLOBAL_HIGH_VRW_CANDIDATES_ROUND12
        + OUTPUT_FRESHNESS_CANDIDATES_ROUND13
    )
}


def fmt_float(value: float) -> str:
    text = f"{value:.6f}".rstrip("0").rstrip(".")
    return text if text else "0"


def now_stamp() -> str:
    return dt.datetime.now().strftime("%Y%m%d_%H%M%S")


def now_iso() -> str:
    return dt.datetime.now(dt.timezone.utc).astimezone().isoformat(timespec="seconds")


def write_run_state(out_dir: Path, **updates: object) -> None:
    path = out_dir / "run_state.json"
    state: dict[str, object] = {}
    if path.exists():
        try:
            loaded = json.loads(path.read_text(encoding="utf-8"))
            if isinstance(loaded, dict):
                state.update(loaded)
        except json.JSONDecodeError:
            state["previous_state_error"] = "invalid json was replaced"
    if "status" in updates and "failure" not in updates:
        state.pop("failure", None)
    state.update(updates)
    state["updated_at"] = now_iso()
    path.write_text(json.dumps(state, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def ensure_layout(out_dir: Path = OUT_DIR) -> None:
    for subdir in [
        out_dir,
        out_dir / "candidates",
        out_dir / "wrappers",
        out_dir / "reports",
        out_dir / "runs",
        out_dir / "prechecks",
    ]:
        subdir.mkdir(parents=True, exist_ok=True)
    ensure_csv_header(out_dir / "leaderboard.csv", LEADERBOARD_FIELDS)
    ensure_csv_header(out_dir / "baseline_reference.csv", BASELINE_FIELDS)
    ensure_csv_header(
        out_dir / "validity_prechecks.csv",
        ["timestamp", "route_id", "run_label", "run_dir", "valid", "success_seq", "expected_success_seq", "mission_count", "simulator_poll_timeout", "notes"],
    )
    if not (out_dir / "failed_runs.md").exists():
        (out_dir / "failed_runs.md").write_text(
            "# Failed Runs\n\n"
            "Rows are appended by `autonomous_iekf_perf_loop.py` when a run is invalid, incomplete, or fails acceptance.\n\n"
            "| time | candidate | route | stage | status | notes |\n"
            "| --- | --- | --- | --- | --- | --- |\n",
            encoding="utf-8",
        )
    if not (out_dir / "best_candidate.md").exists():
        (out_dir / "best_candidate.md").write_text(
            "# Best Candidate\n\n"
            "Status: `failed to beat EKF2 reliably`\n\n"
            "No candidate has completed the multi-route acceptance gate yet.\n",
            encoding="utf-8",
        )


def ensure_csv_header(path: Path, fields: Sequence[str]) -> None:
    if path.exists() and path.stat().st_size > 0:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        csv.DictWriter(f, fieldnames=list(fields)).writeheader()


def read_csv_dicts(path: Path) -> list[dict[str, str]]:
    if not path.exists() or path.stat().st_size == 0:
        return []
    text = path.read_text(encoding="utf-8", errors="ignore").replace("\x00", "")
    if not text.strip():
        return []
    return list(csv.DictReader(io.StringIO(text)))


def append_csv(path: Path, fields: Sequence[str], row: dict[str, object]) -> None:
    ensure_csv_header(path, fields)
    cleaned = {k: stringify(row.get(k, "")) for k in fields}
    with path.open("a", newline="", encoding="utf-8") as f:
        csv.DictWriter(f, fieldnames=list(fields)).writerow(cleaned)


def write_csv(path: Path, fields: Sequence[str], rows: Iterable[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(fields))
        writer.writeheader()
        for row in rows:
            writer.writerow({k: stringify(row.get(k, "")) for k in fields})


def stringify(value: object) -> str:
    if value is None:
        return ""
    if isinstance(value, bool):
        return "1" if value else "0"
    if isinstance(value, float):
        if math.isnan(value):
            return ""
        return f"{value:.12g}"
    return str(value)


def parse_float(value: object, default: float = math.nan) -> float:
    try:
        if value is None:
            return default
        text = str(value).strip()
        if not text:
            return default
        return float(text)
    except (TypeError, ValueError):
        return default


def parse_int(value: object, default: int | None = None) -> int | None:
    try:
        if value is None:
            return default
        text = str(value).strip()
        if not text:
            return default
        return int(float(text))
    except (TypeError, ValueError):
        return default


def replace_yaml_scalar(text: str, key: str, value: object) -> str:
    pattern = re.compile(rf"^(\s*){re.escape(key)}:\s*.*$", re.MULTILINE)
    rendered = "true" if value is True else "false" if value is False else str(value)
    replacement = rf"\1{key}: {rendered}"
    new_text, count = pattern.subn(replacement, text, count=1)
    if count:
        return new_text
    marker = "  ros__parameters:\n"
    if marker not in text:
        raise RuntimeError(f"cannot add {key}: ros__parameters marker missing")
    return text.replace(marker, marker + f"    {key}: {rendered}\n", 1)


def replace_core_noise_vector(text: str, key: str, value: float) -> str:
    rendered = f"[{fmt_float(value)}, {fmt_float(value)}, {fmt_float(value)}]"
    pattern = re.compile(rf"^(\s*){re.escape(key)}:\s*\[[^\]]*\]\s*$", re.MULTILINE)
    new_text, count = pattern.subn(rf"\1{key}: {rendered}", text, count=1)
    if not count:
        raise RuntimeError(f"cannot replace imunoise.{key} in core config")
    return new_text


def replace_core_vrw(text: str, vrw: float) -> str:
    return replace_core_noise_vector(text, "vrw", vrw)


def replace_sage_husa_options(text: str, candidate: Candidate) -> str:
    replacements = {
        "enable": candidate.sage_husa_enable,
        "alpha": fmt_float(candidate.sage_husa_alpha),
        "diag_only": True,
        "min_var_factor": fmt_float(candidate.sage_husa_min_var_factor),
    }
    for key, value in replacements.items():
        rendered = "true" if value is True else "false" if value is False else str(value)
        pattern = re.compile(
            rf"(^sage_husa:\n(?:(?:\s{{2,}}\S[^\n]*\n)*?))^(\s*){re.escape(key)}:\s*.*$",
            re.MULTILINE,
        )
        text, count = pattern.subn(rf"\1\2{key}: {rendered}", text, count=1)
        if not count:
            raise RuntimeError(f"cannot replace sage_husa.{key} in core config")
    return text


def candidate_dir(candidate: Candidate, out_dir: Path = OUT_DIR) -> Path:
    return out_dir / "candidates" / candidate.candidate_id


def write_candidate_files(candidate: Candidate, out_dir: Path = OUT_DIR) -> tuple[Path, Path]:
    cdir = candidate_dir(candidate, out_dir)
    cdir.mkdir(parents=True, exist_ok=True)
    kfgins = BASE_KFGINS.read_text(encoding="utf-8")
    kfgins = replace_yaml_scalar(kfgins, "sim_gnss_std_h_m", fmt_float(candidate.sim_gnss_std_h_m))
    kfgins = replace_yaml_scalar(kfgins, "armed_cruise_gnss_pos_std_h_m", fmt_float(candidate.armed_cruise_gnss_pos_std_h_m))
    kfgins = replace_yaml_scalar(kfgins, "enable_gnss_velocity_update", candidate.enable_gnss_velocity_update)
    kfgins = replace_yaml_scalar(kfgins, "gnss_vel_std_floor_h_mps", fmt_float(candidate.gnss_vel_std_floor_h_mps))
    kfgins = replace_yaml_scalar(kfgins, "native_gnss_speed_std_scale", fmt_float(candidate.native_gnss_speed_std_scale))
    for key in [
        "gnss_position_response_boost_enable",
        "gnss_position_gain_response_enable",
        "gnss_velocity_outward_damping_enable",
        "turn_postturn_native_velocity_deweight_enable",
        "adaptive_gnss_pos_weight_enable",
        "horizontal_consistency_supervisor_enable",
        "mission_cov_hygiene_enable",
        "motion_gnss_pos_weight_enable",
        "gnss_pos_recovery_weight_enable",
        "accbias_z_history_projection_enable",
        "native_gnss_velocity_outlier_guard_enable",
        "native_gnss_velocity_low_speed_turn_source_guard_enable",
    ]:
        kfgins = replace_yaml_scalar(kfgins, key, False)
    kfgins_path = cdir / "kfgins_overlay.yaml"
    kfgins_path.write_text(kfgins, encoding="utf-8")

    core = BASE_CORE.read_text(encoding="utf-8")
    core = replace_core_noise_vector(core, "arw", candidate.imunoise_arw)
    core = replace_core_noise_vector(core, "vrw", candidate.imunoise_vrw)
    core = replace_core_noise_vector(core, "gbstd", candidate.imunoise_gbstd)
    core = replace_core_noise_vector(core, "abstd", candidate.imunoise_abstd)
    core = replace_sage_husa_options(core, candidate)
    core_path = cdir / "kf_core_overlay.yaml"
    core_path.write_text(core, encoding="utf-8")

    manifest = (
        f"# {candidate.candidate_id}\n\n"
        f"- family: `{candidate.family}`\n"
        f"- offline_rank: `{candidate.offline_rank}`\n"
        f"- sim_gnss_std_h_m: `{candidate.sim_gnss_std_h_m}`\n"
        f"- armed_cruise_gnss_pos_std_h_m: `{candidate.armed_cruise_gnss_pos_std_h_m}`\n"
        f"- gnss_vel_std_floor_h_mps: `{candidate.gnss_vel_std_floor_h_mps}`\n"
        f"- native_gnss_speed_std_scale: `{candidate.native_gnss_speed_std_scale}`\n"
        f"- imunoise.arw: `{candidate.imunoise_arw}`\n"
        f"- imunoise.vrw: `{candidate.imunoise_vrw}`\n"
        f"- imunoise.gbstd: `{candidate.imunoise_gbstd}`\n"
        f"- imunoise.abstd: `{candidate.imunoise_abstd}`\n"
        f"- enable_gnss_velocity_update: `{candidate.enable_gnss_velocity_update}`\n"
        f"- publish_state_after_gnss_update: `{candidate.publish_state_after_gnss_update}`\n"
        f"- sage_husa.enable: `{candidate.sage_husa_enable}`\n"
        f"- sage_husa.alpha: `{candidate.sage_husa_alpha}`\n"
        f"- sage_husa.min_var_factor: `{candidate.sage_husa_min_var_factor}`\n"
        "\nDisabled by design: projection, age+lag, PGR/gain/boost selectors, response boost, "
        "route/phase selectors, low-speed source guard.\n"
    )
    (cdir / "candidate.md").write_text(manifest, encoding="utf-8")
    return kfgins_path, core_path


def write_wrapper(candidate: Candidate, route: Route, out_dir: Path = OUT_DIR) -> Path:
    wrapper_dir = out_dir / "wrappers" / candidate.candidate_id
    wrapper_dir.mkdir(parents=True, exist_ok=True)
    path = wrapper_dir / f"run_{route.route_id}.sh"
    path.write_text(
        "#!/usr/bin/env bash\n"
        "set -euo pipefail\n"
        f"cd {WS_ROOT}\n"
        f"exec python3 {SCRIPT} run-one --candidate {candidate.candidate_id} --route {route.route_id} \"$@\"\n",
        encoding="utf-8",
    )
    path.chmod(0o755)
    return path


def write_precheck_wrapper(route: Route, out_dir: Path = OUT_DIR) -> Path:
    wrapper_dir = out_dir / "wrappers" / "prechecks"
    wrapper_dir.mkdir(parents=True, exist_ok=True)
    path = wrapper_dir / f"precheck_{route.route_id}.sh"
    path.write_text(
        "#!/usr/bin/env bash\n"
        "set -euo pipefail\n"
        f"cd {WS_ROOT}\n"
        f"exec python3 {SCRIPT} precheck-route --route {route.route_id} \"$@\"\n",
        encoding="utf-8",
    )
    path.chmod(0o755)
    return path


def init_artifacts(out_dir: Path = OUT_DIR) -> None:
    ensure_layout(out_dir)
    candidate_rows: list[dict[str, object]] = []
    for candidate in sorted(CANDIDATES.values(), key=lambda c: c.offline_rank):
        write_candidate_files(candidate, out_dir)
        for route in ROUTES.values():
            if route.stage in {"T0", "T1", "T2"}:
                write_wrapper(candidate, route, out_dir)
        candidate_rows.append(
            {
                "candidate_id": candidate.candidate_id,
                "family": candidate.family,
                "offline_rank": candidate.offline_rank,
                "expanded": int(candidate.expanded),
                "sim_gnss_std_h_m": candidate.sim_gnss_std_h_m,
                "armed_cruise_gnss_pos_std_h_m": candidate.armed_cruise_gnss_pos_std_h_m,
                "gnss_vel_std_floor_h_mps": candidate.gnss_vel_std_floor_h_mps,
                "native_gnss_speed_std_scale": candidate.native_gnss_speed_std_scale,
                "imunoise_arw": candidate.imunoise_arw,
                "imunoise_vrw": candidate.imunoise_vrw,
                "imunoise_gbstd": candidate.imunoise_gbstd,
                "imunoise_abstd": candidate.imunoise_abstd,
                "publish_state_after_gnss_update": int(candidate.publish_state_after_gnss_update),
                "sage_husa_enable": int(candidate.sage_husa_enable),
                "sage_husa_alpha": candidate.sage_husa_alpha,
                "sage_husa_min_var_factor": candidate.sage_husa_min_var_factor,
            }
        )
    write_csv(
        out_dir / "candidate_grid.csv",
        [
            "candidate_id",
            "family",
            "offline_rank",
            "expanded",
            "sim_gnss_std_h_m",
            "armed_cruise_gnss_pos_std_h_m",
            "gnss_vel_std_floor_h_mps",
            "native_gnss_speed_std_scale",
            "imunoise_arw",
            "imunoise_vrw",
            "imunoise_gbstd",
            "imunoise_abstd",
            "publish_state_after_gnss_update",
            "sage_husa_enable",
            "sage_husa_alpha",
            "sage_husa_min_var_factor",
        ],
        candidate_rows,
    )
    for route in ROUTES.values():
        if route.stage == "T2":
            write_precheck_wrapper(route, out_dir)
    write_route_suite(out_dir)
    write_readme(out_dir)
    seed_baseline_reference(out_dir)
    refresh_best_candidate(out_dir)


def write_route_suite(out_dir: Path) -> None:
    lines = [
        "# Route Suite",
        "",
        "This file is generated by `autonomous_iekf_perf_loop.py init`.",
        "",
        "Rules:",
        "- `shortgen11` is T0 diagnostic only and is not a final generalization claim.",
        "- `shortgen01/02/03/04` are T1 development/control routes.",
        "- `shortgen05/06` are excluded route-validity failures.",
        "- T2 routes are not used for tuning; invalid prechecks are replaced by reserve routes.",
        "- `shortgen12/14/15` are not in the default T2 pool to respect the no-blind-run boundary.",
        "",
        "| route | stage | group | plan | role |",
        "| --- | --- | --- | --- | --- |",
    ]
    for route in sorted(ROUTES.values(), key=lambda r: (r.stage, r.holdout_rank or 0, r.route_id)):
        lines.append(f"| {route.route_id} | {route.stage} | {route.group} | `{route.plan_name}` | {route.role} |")
    (out_dir / "route_suite.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_readme(out_dir: Path) -> None:
    text = f"""# Autonomous IEKF Performance Loop

Created: {now_iso()}

Primary commands:

```bash
python3 {SCRIPT} init
python3 {SCRIPT} rebuild-leaderboard
python3 {SCRIPT} run-loop --time-budget-hours 10
```

Single-run wrappers are under `wrappers/<candidate>/run_<route>.sh`.
Candidate overlays are under `candidates/<candidate>/`.
`rebuild-leaderboard` first recovers ULogs from PX4 rootfs, exports the required topics, reruns groundtruth diagnostics, and rewrites `leaderboard.csv`.

The loop enforces:

- no PX4 source edits;
- no projection, age+lag, PGR/gain/boost, response_boost, route/phase selector, or low-speed source guard;
- T0 before T1 before T2;
- T2 validity precheck before candidate performance;
- final conclusion limited to `stable multi-route win`, `partial win, not stable`, or `failed to beat EKF2 reliably`.
"""
    (out_dir / "README.md").write_text(text, encoding="utf-8")


def seed_baseline_reference(out_dir: Path) -> None:
    rows: list[dict[str, object]] = []
    route_perf = WS_ROOT / (
        "artifacts/manual/short_route_generalization_2026-05-07/"
        "phs5_cross_route_phase_timing_diagnostic_2026-05-10/route_performance_summary.csv"
    )
    if route_perf.exists():
        for row in read_csv_dicts(route_perf):
            run_name = row.get("run", "")
            route_id = run_name.split("_", 1)[0]
            if route_id in {"shortgen01", "shortgen02", "shortgen03", "shortgen04"}:
                rows.append(
                    {
                        "route_id": route_id,
                        "source": "seed_phs5_cross_route",
                        "run_dir": row.get("run_dir", ""),
                        "groundtruth_summary": str(Path(row.get("run_dir", "")) / row.get("groundtruth_summary", "")),
                        "mission_all_iekf_mean": row.get("segment_mission_all_iekf_rmse", ""),
                        "mission_all_ekf2_mean": row.get("segment_mission_all_ekf2_rmse", ""),
                        "main_40_180_iekf_mean": row.get("segment_main_maneuver_40_180_iekf_rmse", ""),
                        "main_40_180_ekf2_mean": row.get("segment_main_maneuver_40_180_ekf2_rmse", ""),
                    }
                )
    shortgen11_summary = WS_ROOT / (
        "artifacts/manual/manual_shortgen11_high_clearance_ladder_sitl_home_headless_compare_core8_pairlogger_"
        "phs5_repeat2_cleancheck_20260510_155253/offline_groundtruth_convergence_diag_autogt/groundtruth_summary.csv"
    )
    if shortgen11_summary.exists():
        metrics = load_groundtruth_metrics(shortgen11_summary)
        rows.append(
            {
                "route_id": "shortgen11",
                "source": "seed_phs5_repeat2_cleancheck",
                "run_dir": str(shortgen11_summary.parents[1]),
                "groundtruth_summary": str(shortgen11_summary),
                **baseline_metric_fields(metrics),
            }
        )
    write_csv(out_dir / "baseline_reference.csv", BASELINE_FIELDS, rows)


def baseline_metric_fields(metrics: dict[str, dict[str, float]]) -> dict[str, object]:
    def value(window: str, estimator: str) -> float:
        return metrics.get(window, {}).get(estimator, math.nan)

    return {
        "mission_all_iekf_mean": value("segment_mission_all", "iekf_normalized"),
        "mission_all_ekf2_mean": value("segment_mission_all", "ekf2"),
        "main_40_180_iekf_mean": value("segment_main_maneuver_40_180", "iekf_normalized"),
        "main_40_180_ekf2_mean": value("segment_main_maneuver_40_180", "ekf2"),
        "window_140_180_iekf_mean": value("140-180", "iekf_normalized"),
        "window_140_180_ekf2_mean": value("140-180", "ekf2"),
        "window_140_160_iekf_mean": value("140-160", "iekf_normalized"),
        "window_140_160_ekf2_mean": value("140-160", "ekf2"),
        "window_160_180_iekf_mean": value("160-180", "iekf_normalized"),
        "window_160_180_ekf2_mean": value("160-180", "ekf2"),
    }


def summary_metrics(path: Path) -> dict[str, dict[str, float]]:
    metrics: dict[str, dict[str, float]] = {}
    for row in read_csv_dicts(path):
        window = row.get("window", "")
        estimator = row.get("estimator", "")
        if not window or not estimator:
            continue
        metrics.setdefault(window, {})[estimator] = parse_float(row.get("xy_mean_m"))
        metrics.setdefault(window + "__rmse", {})[estimator] = parse_float(row.get("xy_rmse_m"))
        metrics.setdefault(window + "__p95", {})[estimator] = parse_float(row.get("xy_p95_m"))
    return metrics


def load_groundtruth_metrics(summary_path: Path) -> dict[str, dict[str, float]]:
    metrics = summary_metrics(summary_path)
    joined = summary_path.with_name("groundtruth_joined.csv")
    if joined.exists():
        merge_joined_windows(metrics, joined, [(40.0, 180.0), (140.0, 180.0)])
    return metrics


def merge_joined_windows(metrics: dict[str, dict[str, float]], joined_path: Path, windows: Sequence[tuple[float, float]]) -> None:
    rows = read_csv_dicts(joined_path)
    for start, end in windows:
        label = f"{start:.0f}-{end:.0f}"
        selected = [
            row for row in rows
            if row.get("mavros_armed") in {"1", "true", "True"}
            and start <= parse_float(row.get("time_since_arm_sec")) < end
        ]
        if not selected:
            continue
        ekf2 = [parse_float(row.get("ekf2_error_xy_m")) for row in selected]
        iekf = [parse_float(row.get("iekf_error_xy_m")) for row in selected]
        metrics.setdefault(label, {})["ekf2"] = mean(ekf2)
        metrics.setdefault(label, {})["iekf_normalized"] = mean(iekf)


def row_delta_ok(row: dict[str, str], key: str, max_delta_m: float) -> bool:
    value = parse_float(row.get(key))
    return math.isfinite(value) and value <= max_delta_m


def candidate_passed_t0(row: dict[str, str]) -> bool:
    if row.get("clean_run") != "1":
        return False
    if row.get("status") != "complete":
        return False
    if row.get("win_main_40_180") != "1":
        return False
    wins = [row.get("win_mission_all"), row.get("win_main_40_180"), row.get("win_140_180")]
    if wins.count("1") < 2:
        return False
    return row_delta_ok(row, "window_140_160_delta", 0.08) and row_delta_ok(row, "window_160_180_delta", 0.08)


def row_is_complete(row: dict[str, str], candidate_id: str, route_id: str) -> bool:
    return row.get("candidate_id") == candidate_id and row.get("route_id") == route_id and row.get("status") == "complete"


def latest_row(candidate_id: str, route_id: str, out_dir: Path = OUT_DIR) -> dict[str, str] | None:
    rows = [
        row for row in read_csv_dicts(out_dir / "leaderboard.csv")
        if row.get("candidate_id") == candidate_id and row.get("route_id") == route_id
    ]
    return rows[-1] if rows else None


def route_baseline(route_id: str, out_dir: Path = OUT_DIR) -> dict[str, str] | None:
    rows = [row for row in read_csv_dicts(out_dir / "baseline_reference.csv") if row.get("route_id") == route_id]
    return rows[-1] if rows else None


def not_worse_than_baseline(row: dict[str, str], tolerance_m: float = 0.015, out_dir: Path = OUT_DIR) -> bool:
    baseline = route_baseline(row.get("route_id", ""), out_dir)
    if not baseline:
        return row.get("win_main_40_180") == "1"
    current = parse_float(row.get("main_40_180_iekf_mean"))
    base = parse_float(baseline.get("main_40_180_iekf_mean"))
    if not math.isfinite(current) or not math.isfinite(base):
        return False
    return current <= base + tolerance_m


def shortgen04_no_local_miss(row: dict[str, str], out_dir: Path = OUT_DIR) -> bool:
    if row.get("route_id") != "shortgen04":
        return True
    return row_delta_ok(row, "window_140_160_delta", 0.05) and row_delta_ok(row, "window_160_180_delta", 0.05)


def t1_route_direct_failed(row: dict[str, str], max_mission_delta_m: float = 0.10) -> bool:
    if row.get("stage") != "T1" or row.get("clean_run") != "1":
        return False
    value = parse_float(row.get("mission_all_delta"))
    return math.isfinite(value) and value > max_mission_delta_m


def t1_candidate_direct_failed(candidate_id: str, out_dir: Path = OUT_DIR) -> bool:
    for route_id in ["shortgen01", "shortgen02", "shortgen03", "shortgen04"]:
        row = latest_row(candidate_id, route_id, out_dir)
        if row and t1_route_direct_failed(row):
            return True
    return False


def t1_pass_count(candidate_id: str, out_dir: Path = OUT_DIR) -> tuple[int, int]:
    passed = 0
    total = 0
    direct_failed = False
    for route_id in ["shortgen01", "shortgen02", "shortgen03", "shortgen04"]:
        row = latest_row(candidate_id, route_id, out_dir)
        if not row or row.get("clean_run") != "1":
            continue
        total += 1
        if t1_route_direct_failed(row):
            direct_failed = True
            continue
        if not_worse_than_baseline(row, out_dir=out_dir) and shortgen04_no_local_miss(row, out_dir):
            passed += 1
    return (0, total) if direct_failed else (passed, total)


def t2_route_safe(row: dict[str, str]) -> bool:
    return row_delta_ok(row, "mission_all_delta", 0.12) and row_delta_ok(row, "rtl_all_delta", 0.12)


def t2_win_count(candidate_id: str, out_dir: Path = OUT_DIR) -> tuple[int, int]:
    wins = 0
    total = 0
    for route in sorted([r for r in ROUTES.values() if r.stage == "T2"], key=lambda r: r.holdout_rank or 99):
        row = latest_row(candidate_id, route.route_id, out_dir)
        if not row or row.get("clean_run") != "1":
            continue
        total += 1
        if row.get("win_main_40_180") == "1" and t2_route_safe(row):
            wins += 1
    return wins, total


def run_loop(args: argparse.Namespace) -> int:
    out_dir = Path(args.out_dir)
    init_artifacts(out_dir)
    rebuild_leaderboard(out_dir)
    deadline = time.monotonic() + max(0.1, args.time_budget_hours) * 3600.0

    def budget_left() -> bool:
        return time.monotonic() < deadline

    def current_t0_pass() -> list[str]:
        passed: list[str] = []
        for candidate in sorted(CANDIDATES.values(), key=lambda c: c.offline_rank):
            row = latest_row(candidate.candidate_id, "shortgen11", out_dir)
            if row and candidate_passed_t0(row):
                passed.append(candidate.candidate_id)
        return passed

    def run_t1_for_candidates(candidate_ids: Sequence[str]) -> list[str]:
        passed_candidates: list[str] = []
        for candidate_id in candidate_ids:
            write_run_state(out_dir, phase="T1", status="running", candidate=candidate_id, next_action="run shortgen01/02/03/04")
            for route_id in ["shortgen01", "shortgen02", "shortgen03", "shortgen04"]:
                if not budget_left():
                    break
                row = latest_row(candidate_id, route_id, out_dir)
                if not row or row.get("status") != "complete" or args.rerun:
                    try_run_one(candidate_id, route_id, out_dir, args.dry_run)
            passed, total = t1_pass_count(candidate_id, out_dir)
            if total >= 4 and passed >= 3:
                passed_candidates.append(candidate_id)
        return passed_candidates

    def run_t2_for_best(candidate_ids: Sequence[str]) -> None:
        if not candidate_ids or not budget_left():
            return
        best = rank_t1_candidates(candidate_ids, out_dir)[0]
        write_run_state(out_dir, phase="T2", status="running", candidate=best, next_action="precheck and run holdouts")
        valid_routes: list[str] = []
        for route in sorted([r for r in ROUTES.values() if r.stage == "T2"], key=lambda r: r.holdout_rank or 99):
            if len(valid_routes) >= 5 or not budget_left():
                break
            pre = latest_precheck(route.route_id, out_dir)
            if not pre or args.rerun:
                try_precheck_route(route.route_id, out_dir, args.dry_run)
                pre = latest_precheck(route.route_id, out_dir)
            if pre and pre.get("valid") == "1":
                valid_routes.append(route.route_id)
        for route_id in valid_routes[:5]:
            if not budget_left():
                break
            row = latest_row(best, route_id, out_dir)
            if not row or row.get("status") != "complete" or args.rerun:
                try_run_one(best, route_id, out_dir, args.dry_run)

    t0_pass: list[str] = current_t0_pass()
    t1_pass: list[str] = []
    max_rounds = min(max(1, args.max_candidate_rounds), len(CANDIDATE_ROUNDS))
    for round_index, candidates in enumerate(CANDIDATE_ROUNDS[:max_rounds], start=1):
        limit = args.max_t0_candidates if round_index == 1 else args.max_expanded_candidates
        write_run_state(out_dir, phase="T0", status="running", candidate_round=round_index, next_action="screen shortgen11 candidates")
        for candidate in candidates[:limit]:
            if not budget_left():
                break
            row = latest_row(candidate.candidate_id, "shortgen11", out_dir)
            if not row or row.get("status") != "complete" or args.rerun:
                try_run_one(candidate.candidate_id, "shortgen11", out_dir, args.dry_run)
                row = latest_row(candidate.candidate_id, "shortgen11", out_dir)
            if row and candidate_passed_t0(row):
                t0_pass.append(candidate.candidate_id)
        t0_pass = sorted(set(current_t0_pass()), key=lambda cid: CANDIDATES[cid].offline_rank)
        t1_pass = run_t1_for_candidates(t0_pass)
        if t1_pass or not budget_left():
            break
        write_run_state(out_dir, phase="T0", status="round_failed_t1" if t0_pass else "round_no_t0_pass", candidate_round=round_index, next_action="advance to next fixed-parameter round")

    run_t2_for_best(t1_pass)

    write_stage_summaries(out_dir)
    refresh_best_candidate(out_dir)
    write_run_state(out_dir, phase="supervisor", status="idle_or_budget_exhausted", t0_pass=t0_pass, t1_pass=t1_pass)
    return 0


def try_run_one(candidate_id: str, route_id: str, out_dir: Path, dry_run: bool) -> dict[str, object]:
    try:
        return run_one(candidate_id, route_id, out_dir, dry_run)
    except Exception as exc:  # noqa: BLE001 - the loop must record and continue.
        candidate = CANDIDATES[candidate_id]
        route = ROUTES[route_id]
        run_label = f"autoniekf_exception_{route.route_id}_{candidate.candidate_id}_{now_stamp()}"
        run_dir = out_dir / "runs" / run_label
        run_dir.mkdir(parents=True, exist_ok=True)
        (run_dir / "exception.txt").write_text(repr(exc) + "\n", encoding="utf-8")
        row = failed_row(candidate, route, run_label, run_dir, "exception", repr(exc))
        append_csv(out_dir / "leaderboard.csv", LEADERBOARD_FIELDS, row)
        append_failed(out_dir, row)
        write_stage_summaries(out_dir)
        refresh_best_candidate(out_dir)
        write_run_state(out_dir, phase=route.stage, status="exception", candidate=candidate_id, route=route_id, failure=repr(exc), run_dir=str(run_dir))
        return row


def try_precheck_route(route_id: str, out_dir: Path, dry_run: bool) -> dict[str, object]:
    try:
        return precheck_route(route_id, out_dir, dry_run)
    except Exception as exc:  # noqa: BLE001 - one invalid precheck must not stop T2.
        route = ROUTES[route_id]
        run_label = f"autoniekf_precheck_exception_{route.route_id}_{now_stamp()}"
        run_dir = out_dir / "prechecks" / run_label
        run_dir.mkdir(parents=True, exist_ok=True)
        (run_dir / "exception.txt").write_text(repr(exc) + "\n", encoding="utf-8")
        row = {
            "timestamp": now_iso(),
            "route_id": route.route_id,
            "run_label": run_label,
            "run_dir": str(run_dir),
            "valid": 0,
            "notes": repr(exc),
        }
        append_csv(out_dir / "validity_prechecks.csv", ["timestamp", "route_id", "run_label", "run_dir", "valid", "success_seq", "expected_success_seq", "mission_count", "simulator_poll_timeout", "notes"], row)
        write_run_state(out_dir, phase="T2-precheck", status="exception", route=route_id, failure=repr(exc), run_dir=str(run_dir))
        return row


def rank_t1_candidates(candidate_ids: Sequence[str], out_dir: Path) -> list[str]:
    def score(candidate_id: str) -> tuple[int, int, float]:
        passed, _ = t1_pass_count(candidate_id, out_dir)
        deltas = []
        for route_id in ["shortgen01", "shortgen02", "shortgen03", "shortgen04"]:
            row = latest_row(candidate_id, route_id, out_dir)
            if row:
                value = parse_float(row.get("main_40_180_delta"))
                if math.isfinite(value):
                    deltas.append(value)
        direct_failed = 1 if t1_candidate_direct_failed(candidate_id, out_dir) else 0
        return (direct_failed, -passed, sum(deltas) / len(deltas) if deltas else float("inf"))

    return sorted(candidate_ids, key=score)


def latest_precheck(route_id: str, out_dir: Path) -> dict[str, str] | None:
    rows = [row for row in read_csv_dicts(out_dir / "validity_prechecks.csv") if row.get("route_id") == route_id]
    return rows[-1] if rows else None


def rebuild_leaderboard(out_dir: Path = OUT_DIR, route_filter: str | None = None, candidate_filter: str | None = None) -> list[dict[str, object]]:
    ensure_layout(out_dir)
    write_run_state(out_dir, phase="rebuild-leaderboard", status="running", route_filter=route_filter or "", candidate_filter=candidate_filter or "")
    rows: list[dict[str, object]] = []
    run_dirs = sorted((out_dir / "runs").glob("autoniekf_*"))
    for run_dir in run_dirs:
        if not run_dir.is_dir():
            continue
        candidate, route, run_label = run_identity_from_dir(run_dir)
        if not candidate or not route:
            continue
        if not (run_dir / "mission_smoke.log").exists() and not (run_dir / "manual_summary.txt").exists():
            continue
        if candidate_filter and candidate.candidate_id != candidate_filter:
            continue
        if route_filter and route.route_id != route_filter:
            continue
        run_offline_groundtruth(run_label, run_dir)
        row = analyze_run(candidate, route, run_label, run_dir, out_dir)
        rows.append(row)
        write_per_route_report(out_dir, row)
    if route_filter or candidate_filter:
        def selected(row: dict[str, str]) -> bool:
            if route_filter and row.get("route_id") != route_filter:
                return False
            if candidate_filter and row.get("candidate_id") != candidate_filter:
                return False
            return True

        existing = [row for row in read_csv_dicts(out_dir / "leaderboard.csv") if not selected(row)]
        merged: list[dict[str, object]] = [*existing, *rows]
    else:
        merged = rows
    merged.sort(key=lambda row: (str(row.get("timestamp", "")), str(row.get("run_label", ""))))
    write_csv(out_dir / "leaderboard.csv", LEADERBOARD_FIELDS, merged)
    write_stage_summaries(out_dir)
    refresh_best_candidate(out_dir)
    write_run_state(out_dir, phase="rebuild-leaderboard", status="complete", rebuilt_rows=len(rows), total_rows=len(merged))
    return merged


def run_one(candidate_id: str, route_id: str, out_dir: Path = OUT_DIR, dry_run: bool = False) -> dict[str, object]:
    if candidate_id not in CANDIDATES:
        raise SystemExit(f"unknown candidate: {candidate_id}")
    if route_id not in ROUTES:
        raise SystemExit(f"unknown route: {route_id}")
    candidate = CANDIDATES[candidate_id]
    route = ROUTES[route_id]
    ensure_layout(out_dir)
    kfgins_cfg, core_cfg = write_candidate_files(candidate, out_dir)
    write_wrapper(candidate, route, out_dir)
    run_label = f"autoniekf_{route.route_id}_{candidate.candidate_id}_{now_stamp()}"
    run_dir = out_dir / "runs" / run_label
    run_dir.mkdir(parents=True, exist_ok=True)
    write_run_state(out_dir, phase=route.stage, status="running", candidate=candidate.candidate_id, route=route.route_id, run_dir=str(run_dir))
    shutil.copy2(route.plan_path, run_dir / route.plan_path.name)
    shutil.copy2(kfgins_cfg, run_dir / "kfgins_overlay.yaml")
    shutil.copy2(core_cfg, run_dir / "kf_core_overlay.yaml")
    (run_dir / "compare_meta.txt").write_text(
        f"run_label={run_label}\n"
        f"run_dir={run_dir}\n"
        f"candidate={candidate.candidate_id}\n"
        f"route={route.route_id}\n"
        f"plan={route.plan_path}\n"
        f"profile=autonomous fixed-noise IEKF performance loop\n"
        f"created_at={now_iso()}\n",
        encoding="utf-8",
    )

    if dry_run:
        row = failed_row(candidate, route, run_label, run_dir, "dry_run", "dry run only; no simulator launched")
        append_csv(out_dir / "leaderboard.csv", LEADERBOARD_FIELDS, row)
        append_failed(out_dir, row)
        write_stage_summaries(out_dir)
        write_run_state(out_dir, phase=route.stage, status="dry_run", candidate=candidate.candidate_id, route=route.route_id, run_dir=str(run_dir))
        return row

    env = run_env(run_dir, candidate, kfgins_cfg, core_cfg)
    try:
        run_session(route.plan_path, run_label, run_dir, env, compare_enabled=True)
    finally:
        stop_session(run_dir, env)
    run_offline_groundtruth(run_label, run_dir)
    row = analyze_run(candidate, route, run_label, run_dir, out_dir)
    append_csv(out_dir / "leaderboard.csv", LEADERBOARD_FIELDS, row)
    write_per_route_report(out_dir, row)
    if row.get("status") != "complete" or row.get("clean_run") != 1:
        append_failed(out_dir, row)
    write_stage_summaries(out_dir)
    refresh_best_candidate(out_dir)
    write_run_state(out_dir, phase=route.stage, status=str(row.get("status", "")), candidate=candidate.candidate_id, route=route.route_id, run_dir=str(run_dir))
    return row


def run_env(run_dir: Path, candidate: Candidate | None, kfgins_cfg: Path | None, core_cfg: Path | None) -> dict[str, str]:
    env = os.environ.copy()
    env.update(
        {
            "ROS_DOMAIN_ID": "0",
            "PX4_SAFE_PX4_WORLD": "none",
            "PX4_SAFE_HEADLESS": "1",
            "PX4_SAFE_NO_PXH": "1",
            "PX4_SAFE_START_QGC": "0",
            "PX4_SAFE_START_MONITOR": "1",
            "PX4_SAFE_START_COMPARE": "1",
            "PX4_SAFE_START_GPS_PROBES": "1",
            "PX4_SAFE_START_PAIR_LOGGER": "1",
            "PX4_SAFE_PAIR_LOGGER_RATE_HZ": "10",
            "PX4_SAFE_PAIR_LOGGER_SYNC_TOLERANCE_MS": "50",
            "PX4_SAFE_COMPARE_CORE_MAX_IMU_RATE_HZ": "8.0",
            "PX4_SAFE_COMPARE_EKF2_USE_INPUT_STAMP": "true",
            "PX4_SAFE_COMPARE_GNSS_RELAY_MODE": "px4_sensor_gps",
            "PX4_SAFE_COMPARE_ENABLE_GPS_DROPZONES": "false",
            "PX4_SAFE_COMPARE_INJECT_DROPZONE_GPS_TO_PX4": "false",
            "PX4_SAFE_COMPARE_PX4_GPS_INJECTION_MODE": "off",
            "PX4_SAFE_COMPARE_PX4_SET_PARAMS": "false",
            "PX4_SAFE_COMPARE_PUBLISH_STATE_AFTER_GNSS_UPDATE": (
                "true" if candidate and candidate.publish_state_after_gnss_update else "false"
            ),
            "PX4_SAFE_COMPARE_PUBLISH_STAMP_MODE": "core_fixed_offset",
            "PX4_SAFE_COMPARE_PUBLISH_CORE_STAMP_MAX_FUTURE_SEC": "0.0",
            "PX4_SAFE_COMPARE_PUBLISH_CORE_STAMP_OFFSET_BIAS_SEC": "-0.03",
            "PX4_SAFE_COMPARE_RAW_ODOM_DECIMATION": "50",
            "PX4_SAFE_COMPARE_PATH_PUBLISH_RATE_HZ": "5.0",
            "PX4_SAFE_COMPARE_POSE_DECIMATION": "5",
            "PX4_SAFE_COMPARE_MAX_PATH_POINTS": "20000",
            "PX4_SAFE_COMPARE_GNSS_UPDATE_DEBUG_CSV_PATH": str(run_dir / "gnss_update_debug.csv"),
            "PX4_SAFE_COMPARE_STATE_UPDATE_DEBUG_CSV_PATH": str(run_dir / "state_update_debug.csv"),
            "PX4_SAFE_COMPARE_STATE_UPDATE_DEBUG_MAX_RATE_HZ": "2.0",
            "PX4_SAFE_COMPARE_STATE_PUBLISH_DEBUG_CSV_PATH": str(run_dir / "state_publish_debug.csv"),
            "PX4_SAFE_COMPARE_GNSS_NIS_DEBUG_CSV_PATH": "__disabled__",
            "PX4_SAFE_COMPARE_HORIZONTAL_CONSISTENCY_DEBUG_CSV_PATH": "__disabled__",
            "PX4_SAFE_COMPARE_SEGMENT_TIMING_GATE_DEBUG_CSV_PATH": "__disabled__",
        }
    )
    env.update(DISABLED_MECHANISM_ENV)
    if candidate and kfgins_cfg and core_cfg:
        env.update(candidate.env(kfgins_cfg, core_cfg))
        env["PX4_SAFE_COMPARE_GNSS_VEL_STD_FLOOR_H_MPS"] = fmt_float(candidate.gnss_vel_std_floor_h_mps)
        env["PX4_SAFE_COMPARE_NATIVE_GNSS_SPEED_STD_SCALE"] = fmt_float(candidate.native_gnss_speed_std_scale)
    return env


def source_prefix() -> str:
    return (
        "set -euo pipefail; "
        f"cd {WS_ROOT}; "
        "set +u; "
        "source /opt/ros/${ROS_DISTRO:-humble}/setup.bash; "
        f"source {WS_ROOT}/install/setup.bash; "
        "set -u; "
    )


def run_bash(command: str, env: dict[str, str], stdout_path: Path, check: bool = True, timeout: float | None = None) -> subprocess.CompletedProcess[str]:
    stdout_path.parent.mkdir(parents=True, exist_ok=True)
    with stdout_path.open("w", encoding="utf-8") as f:
        return subprocess.run(
            ["bash", "-lc", source_prefix() + command],
            cwd=str(WS_ROOT),
            env=env,
            stdout=f,
            stderr=subprocess.STDOUT,
            text=True,
            check=check,
            timeout=timeout,
        )


def run_session(plan: Path, run_label: str, run_dir: Path, env: dict[str, str], compare_enabled: bool) -> None:
    if not compare_enabled:
        env = dict(env)
        env.update(
            {
                "PX4_SAFE_START_COMPARE": "0",
                "PX4_SAFE_START_GPS_PROBES": "0",
                "PX4_SAFE_START_PAIR_LOGGER": "0",
            }
        )
    run_bash(f"{MANUAL_SESSION} show-env {sh(run_dir)}", env, run_dir / "env.txt", check=False)
    run_bash(f"{MANUAL_SESSION} start {sh(run_dir)}", env, run_dir / "start_console.log", check=True)
    run_bash(
        f"python3 {MISSION_SMOKE} --plan-file {sh(plan)} --pre-arm-settle 8 --arm-retries 6 "
        "--mission-timeout 650 --post-mission-settle 45 --print-period 5",
        env,
        run_dir / "mission_smoke.log",
        check=False,
        timeout=780,
    )


def stop_session(run_dir: Path, env: dict[str, str]) -> None:
    run_bash(f"{MANUAL_SESSION} stop {sh(run_dir)}", env, run_dir / "stop.log", check=False, timeout=120)
    run_bash(f"{MANUAL_SESSION} summary {sh(run_dir)}", env, run_dir / "manual_summary.txt", check=False, timeout=120)
    run_bash(
        "pgrep -af 'PX4-Autopilot|px4|gz sim|gzserver|gzclient|MicroXRCEAgent|mavros|"
        "QGroundControl|rviz2|plotjuggler|kf_gins_node|ekf_iekf_pair_logger|px4_mission_smoke' || true",
        env,
        run_dir / "residual_process_check.txt",
        check=False,
        timeout=30,
    )


def sh(path: Path | str) -> str:
    text = str(path)
    return "'" + text.replace("'", "'\"'\"'") + "'"


def parse_run_meta(run_dir: Path) -> dict[str, str]:
    meta_path = run_dir / "compare_meta.txt"
    meta: dict[str, str] = {}
    if not meta_path.exists():
        return meta
    for line in meta_path.read_text(encoding="utf-8", errors="ignore").splitlines():
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        meta[key.strip()] = value.strip()
    return meta


def run_identity_from_dir(run_dir: Path) -> tuple[Candidate | None, Route | None, str]:
    meta = parse_run_meta(run_dir)
    run_label = meta.get("run_label") or run_dir.name
    candidate_id = meta.get("candidate", "")
    route_id = meta.get("route", "")
    if not candidate_id or not route_id:
        match = re.match(r"autoniekf_([^_]+)_(cand\d+_.+)_\d{8}_\d{6}$", run_dir.name)
        if match:
            route_id = route_id or match.group(1)
            candidate_id = candidate_id or match.group(2)
    return CANDIDATES.get(candidate_id), ROUTES.get(route_id), run_label


def px4_ulog_rel_from_log(run_dir: Path) -> str | None:
    px4_log = run_dir / "px4.log"
    if not px4_log.exists():
        return None
    text = px4_log.read_text(encoding="utf-8", errors="ignore")
    matches = re.findall(r"\./log/(\d{4}-\d{2}-\d{2}/[0-9_]+\.ulg)", text)
    return matches[-1] if matches else None


def locate_px4_ulog(run_dir: Path, copy_to_run_dir: bool = True) -> Path | None:
    existing = sorted(run_dir.glob("*.ulg"))
    if existing:
        (run_dir / "ulog_path.txt").write_text(f"run_dir_ulog={existing[0]}\n", encoding="utf-8")
        return existing[0]
    rel = px4_ulog_rel_from_log(run_dir)
    if not rel:
        (run_dir / "ulog_path.txt").write_text("status=missing px4.log ulog reference\n", encoding="utf-8")
        return None
    source = PX4_ROOTFS_LOG_DIR / rel
    if not source.exists():
        (run_dir / "ulog_path.txt").write_text(f"status=missing source\nsource={source}\n", encoding="utf-8")
        return None
    if not copy_to_run_dir:
        (run_dir / "ulog_path.txt").write_text(f"source={source}\n", encoding="utf-8")
        return source
    dest = run_dir / source.name
    if not dest.exists() or dest.stat().st_size != source.stat().st_size:
        shutil.copy2(source, dest)
    (run_dir / "ulog_path.txt").write_text(f"source={source}\nrun_dir_ulog={dest}\n", encoding="utf-8")
    return dest


def export_ulog_topics(run_dir: Path, ulog_path: Path | None = None) -> bool:
    ulog = ulog_path or locate_px4_ulog(run_dir)
    if not ulog:
        (run_dir / "ulog2csv.log").write_text("missing ULog; cannot export topics\n", encoding="utf-8")
        return False
    ulog2csv = shutil.which("ulog2csv") or str(Path.home() / ".local/bin/ulog2csv")
    if not Path(ulog2csv).exists() and shutil.which(ulog2csv) is None:
        (run_dir / "ulog2csv.log").write_text(f"ulog2csv not found: {ulog2csv}\n", encoding="utf-8")
        return False
    log_path = run_dir / "ulog2csv.log"
    with log_path.open("a", encoding="utf-8") as log:
        log.write(f"\n[{now_iso()}] ulog={ulog}\n")
        for topic in ULOG_TOPICS:
            if sorted(run_dir.glob(f"*_{topic}_0.csv")):
                log.write(f"skip existing {topic}\n")
                continue
            cmd = [ulog2csv, "-m", topic, "-o", str(run_dir), str(ulog)]
            log.write("$ " + " ".join(cmd) + "\n")
            proc = subprocess.run(cmd, cwd=str(WS_ROOT), stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, check=False)
            log.write(proc.stdout)
            log.write(f"exit={proc.returncode}\n")
    have_gps = find_first(run_dir, ["*_vehicle_gps_position_0.csv", "*_sensor_gps_0.csv"]) is not None
    have_local_gt = find_first(run_dir, ["*_vehicle_local_position_groundtruth_0.csv"]) is not None
    return have_gps and have_local_gt


def run_offline_groundtruth(run_label: str, run_dir: Path) -> bool:
    export_ulog_topics(run_dir)
    gps_csv = find_first(run_dir, ["*_vehicle_gps_position_0.csv", "*_sensor_gps_0.csv"])
    if not gps_csv:
        (run_dir / "offline_groundtruth_error.txt").write_text("missing vehicle_gps_position/sensor_gps exported CSV\n", encoding="utf-8")
        return False
    gps_gt_out = run_dir / "offline_ulog_gps_groundtruth_diag"
    gps_gt_cmd = f"python3 {ULOG_GPS_GT_DIAG} --ulog-dir {sh(run_dir)} --out-dir {sh(gps_gt_out)}"
    try:
        run_bash(gps_gt_cmd, os.environ.copy(), run_dir / "offline_ulog_gps_groundtruth_diag.log", check=True, timeout=240)
    except subprocess.CalledProcessError:
        (run_dir / "offline_groundtruth_error.txt").write_text("failed offline_ulog_gps_groundtruth_diagnostic.py\n", encoding="utf-8")
        return False
    fit_csv = gps_gt_out / "gps_groundtruth_fit.csv"
    if not fit_csv.exists():
        (run_dir / "offline_groundtruth_error.txt").write_text("missing gps_groundtruth_fit.csv\n", encoding="utf-8")
        return False
    out = run_dir / "offline_groundtruth_convergence_diag_autogt"
    cmd = (
        f"python3 {GT_DIAG} --run-label {sh(run_label)} --run-dir {sh(run_dir)} --ulog-dir {sh(run_dir)} "
        f"--fit-csv {sh(fit_csv)} --out-dir {sh(out)} --groundtruth-source auto "
        f"--projection-mode {GT_PROJECTION_MODE}"
    )
    try:
        run_bash(cmd, os.environ.copy(), run_dir / "offline_groundtruth_convergence_diag.log", check=True, timeout=240)
        error_path = run_dir / "offline_groundtruth_error.txt"
        if error_path.exists():
            error_path.unlink()
        return True
    except subprocess.CalledProcessError:
        return False


def find_first(root: Path, patterns: Sequence[str]) -> Path | None:
    for pattern in patterns:
        matches = sorted(root.glob(pattern))
        if matches:
            return matches[0]
    return None


def analyze_run(candidate: Candidate, route: Route, run_label: str, run_dir: Path, out_dir: Path) -> dict[str, object]:
    mission = parse_mission(run_dir)
    required = required_files_ok(run_dir)
    summary = run_dir / "offline_groundtruth_convergence_diag_autogt/groundtruth_summary.csv"
    metrics = load_groundtruth_metrics(summary) if summary.exists() else {}
    sync = sync_residual_summary(run_dir / "state_update_debug.csv")
    baseline = route_baseline(route.route_id, out_dir)
    baseline_main = parse_float(baseline.get("main_40_180_iekf_mean")) if baseline else math.nan

    def metric(window: str, estimator: str) -> float:
        return metrics.get(window, {}).get(estimator, math.nan)

    def win(window: str) -> bool:
        iekf = metric(window, "iekf_normalized")
        ekf2 = metric(window, "ekf2")
        return math.isfinite(iekf) and math.isfinite(ekf2) and iekf < ekf2

    mission_iekf = metric("segment_mission_all", "iekf_normalized")
    mission_ekf2 = metric("segment_mission_all", "ekf2")
    main_iekf = metric("segment_main_maneuver_40_180", "iekf_normalized")
    main_ekf2 = metric("segment_main_maneuver_40_180", "ekf2")
    win_140_180_iekf = metric("140-180", "iekf_normalized")
    win_140_180_ekf2 = metric("140-180", "ekf2")
    win_140_160_iekf = metric("140-160", "iekf_normalized")
    win_140_160_ekf2 = metric("140-160", "ekf2")
    win_160_180_iekf = metric("160-180", "iekf_normalized")
    win_160_180_ekf2 = metric("160-180", "ekf2")
    rtl_iekf = metric("300-360", "iekf_normalized")
    rtl_ekf2 = metric("300-360", "ekf2")
    ulog = find_first(run_dir, ["*.ulg"])
    clean = (
        mission["valid"]
        and not mission["poll_timeout"]
        and required
        and summary.exists()
        and math.isfinite(main_iekf)
        and math.isfinite(main_ekf2)
    )
    notes = []
    if not mission["valid"]:
        notes.append("mission did not reach expected seq")
    if mission["poll_timeout"]:
        notes.append("simulator poll timeout")
    if not required:
        notes.append("required CSV missing or empty")
    if not summary.exists():
        notes.append("groundtruth summary missing")
    status = "complete" if clean else "invalid_or_incomplete"
    row: dict[str, object] = {
        "timestamp": now_iso(),
        "candidate_id": candidate.candidate_id,
        "candidate_family": candidate.family,
        "stage": route.stage,
        "route_id": route.route_id,
        "route_group": route.group,
        "run_label": run_label,
        "run_dir": str(run_dir),
        "clean_run": clean,
        "validity_status": "valid" if mission["valid"] else "invalid",
        "expected_success_seq": mission["expected_success_seq"],
        "success_seq": mission["success_seq"],
        "mission_count": mission["mission_count"],
        "simulator_poll_timeout": mission["poll_timeout"],
        "required_files_ok": required,
        "groundtruth_summary": str(summary) if summary.exists() else "",
        "mission_all_iekf_mean": mission_iekf,
        "mission_all_ekf2_mean": mission_ekf2,
        "mission_all_delta": delta(mission_iekf, mission_ekf2),
        "main_40_180_iekf_mean": main_iekf,
        "main_40_180_ekf2_mean": main_ekf2,
        "main_40_180_delta": delta(main_iekf, main_ekf2),
        "win_mission_all": win("segment_mission_all"),
        "win_main_40_180": win("segment_main_maneuver_40_180"),
        "win_140_180": win("140-180"),
        "win_140_160": win("140-160"),
        "win_160_180": win("160-180"),
        "window_140_180_iekf_mean": win_140_180_iekf,
        "window_140_180_ekf2_mean": win_140_180_ekf2,
        "window_140_180_delta": delta(win_140_180_iekf, win_140_180_ekf2),
        "window_140_160_iekf_mean": win_140_160_iekf,
        "window_140_160_ekf2_mean": win_140_160_ekf2,
        "window_140_160_delta": delta(win_140_160_iekf, win_140_160_ekf2),
        "window_160_180_iekf_mean": win_160_180_iekf,
        "window_160_180_ekf2_mean": win_160_180_ekf2,
        "window_160_180_delta": delta(win_160_180_iekf, win_160_180_ekf2),
        "rtl_all_iekf_mean": rtl_iekf,
        "rtl_all_ekf2_mean": rtl_ekf2,
        "rtl_all_delta": delta(rtl_iekf, rtl_ekf2),
        "baseline_main_40_180_iekf_mean": baseline_main,
        "baseline_delta_main_40_180": delta(main_iekf, baseline_main),
        "sync_residual_h_mean_m": sync["residual_h_mean"],
        "sync_dx_pos_h_mean_m": sync["dx_pos_h_mean"],
        "sync_dx_over_residual_mean": sync["dx_over_residual_mean"],
        "ulog_path": str(ulog) if ulog else "",
        "status": status,
        "notes": "; ".join(notes),
    }
    return row


def failed_row(candidate: Candidate, route: Route, run_label: str, run_dir: Path, status: str, notes: str) -> dict[str, object]:
    return {
        "timestamp": now_iso(),
        "candidate_id": candidate.candidate_id,
        "candidate_family": candidate.family,
        "stage": route.stage,
        "route_id": route.route_id,
        "route_group": route.group,
        "run_label": run_label,
        "run_dir": str(run_dir),
        "clean_run": False,
        "validity_status": "not_run",
        "status": status,
        "notes": notes,
    }


def delta(a: float, b: float) -> float:
    if math.isfinite(a) and math.isfinite(b):
        return a - b
    return math.nan


def required_files_ok(run_dir: Path) -> bool:
    for name in REQUIRED_RUN_FILES:
        path = run_dir / name
        if not path.exists() or csv_rows(path) <= 0:
            return False
    return True


def csv_rows(path: Path) -> int:
    if not path.exists() or path.stat().st_size == 0:
        return 0
    with path.open(encoding="utf-8", errors="ignore") as f:
        return max(0, sum(1 for _ in f) - 1)


def parse_mission(run_dir: Path) -> dict[str, object]:
    text = ""
    for name in ["mission_smoke.log", "manual_summary.txt", "px4.log"]:
        path = run_dir / name
        if path.exists():
            text += "\n" + path.read_text(encoding="utf-8", errors="ignore")
    mission_count = parse_last_int(text, r"mission: current_seq=\d+ count=(\d+)")
    success_seq = parse_last_int(text, r"final last_reached=(\d+) success_seq=(\d+)", group=2)
    if success_seq is None:
        success_seq = parse_last_int(text, r"waypoint reached: seq=(\d+)")
    expected = mission_count - 2 if mission_count is not None and mission_count >= 2 else None
    poll_timeout = "simulator poll timeout" in text.lower() or "simulator_poll_timeout=1" in text
    valid = success_seq is not None and expected is not None and success_seq >= expected
    return {
        "mission_count": mission_count,
        "success_seq": success_seq,
        "expected_success_seq": expected,
        "poll_timeout": poll_timeout,
        "valid": valid,
    }


def parse_last_int(text: str, pattern: str, group: int = 1) -> int | None:
    matches = list(re.finditer(pattern, text))
    if not matches:
        return None
    try:
        return int(matches[-1].group(group))
    except (IndexError, ValueError):
        return None


def sync_residual_summary(path: Path) -> dict[str, float]:
    rows = read_csv_dicts(path)
    residuals: list[float] = []
    dxs: list[float] = []
    ratios: list[float] = []
    for row in rows:
        if row.get("event_type") != "gnss_position":
            continue
        if row.get("applied") not in {"1", "true", "True"}:
            continue
        armed_time = parse_float(row.get("armed_time_sec"))
        if not math.isfinite(armed_time) or armed_time < 40.0 or armed_time > 180.0:
            continue
        residual = parse_float(row.get("gnss_position_residual_h_m"))
        dx = parse_float(row.get("dx_pos_h_norm_m"))
        ratio = parse_float(row.get("dx_pos_h_over_residual_h"))
        if math.isfinite(residual):
            residuals.append(residual)
        if math.isfinite(dx):
            dxs.append(dx)
        if math.isfinite(ratio):
            ratios.append(ratio)
    return {
        "residual_h_mean": mean(residuals),
        "dx_pos_h_mean": mean(dxs),
        "dx_over_residual_mean": mean(ratios),
    }


def mean(values: Sequence[float]) -> float:
    finite = [v for v in values if math.isfinite(v)]
    return sum(finite) / len(finite) if finite else math.nan


def write_per_route_report(out_dir: Path, row: dict[str, object]) -> None:
    report = out_dir / "reports" / f"{row['candidate_id']}__{row['route_id']}.md"
    lines = [
        f"# {row['candidate_id']} on {row['route_id']}",
        "",
        f"- status: `{row.get('status', '')}`",
        f"- clean_run: `{row.get('clean_run', '')}`",
        f"- run_dir: `{row.get('run_dir', '')}`",
        f"- groundtruth_summary: `{row.get('groundtruth_summary', '')}`",
        f"- mission_all delta IEKF-EKF2 mean: `{stringify(row.get('mission_all_delta'))}`",
        f"- main_40_180 delta IEKF-EKF2 mean: `{stringify(row.get('main_40_180_delta'))}`",
        f"- sync update residual h mean: `{stringify(row.get('sync_residual_h_mean_m'))}`",
        f"- notes: {row.get('notes', '')}",
        "",
        "This is one route-level report.  It is not a multi-route claim by itself.",
    ]
    report.write_text("\n".join(lines) + "\n", encoding="utf-8")


ROUTE_SHAPES = {
    "shortgen01": "S-bend",
    "shortgen02": "box/U-turn",
    "shortgen03": "figure-eight",
    "shortgen04": "ladder",
    "shortgen07": "shifted ladder",
    "shortgen08": "bowtie",
    "shortgen09": "stair return",
    "shortgen10": "ladder subset",
    "shortgen11": "high-clearance ladder diagnostic",
    "shortgen13": "cross turn",
    "shortgen16": "NE bowtie reserve",
    "shortgen17": "straight line",
    "shortgen18": "rectangle",
    "shortgen19": "L-turn",
    "shortgen20": "S/polyline",
    "shortgen21": "out-and-back",
    "shortgen22": "GNSR DEV0 offset bowtie",
    "shortgen23": "GNSR DEV0 straight jog negative",
    "shortgen24": "GMOC DEV0 turn-rich",
    "shortgen25": "GMOC DEV0 low-observability",
    "shortgen26": "GMOC DEV0 mixed",
}


def t0_gate_note(row: dict[str, str]) -> str:
    if row.get("status") != "complete" or row.get("clean_run") != "1":
        return "fail: incomplete"
    if row.get("win_main_40_180") != "1":
        return "fail: 40-180 does not beat EKF2"
    wins = [row.get("win_mission_all"), row.get("win_main_40_180"), row.get("win_140_180")].count("1")
    if wins < 2:
        return "fail: fewer than 2/3 wins"
    if not row_delta_ok(row, "window_140_160_delta", 0.08):
        return "fail: 140-160 local degradation"
    if not row_delta_ok(row, "window_160_180_delta", 0.08):
        return "fail: 160-180 local degradation"
    return "pass"


def reset_failed_runs(out_dir: Path, rows: Sequence[dict[str, str]]) -> None:
    lines = [
        "# Failed Runs",
        "",
        "Regenerated from `leaderboard.csv`; ordinary candidate failures are evidence, not supervisor stop conditions.",
        "",
        "| time | candidate | route | stage | status | notes |",
        "| --- | --- | --- | --- | --- | --- |",
    ]
    for row in rows:
        failed = row.get("status") != "complete" or row.get("clean_run") != "1"
        if not failed:
            failed = row.get("route_id") == "shortgen11" and not candidate_passed_t0(row)
        if not failed:
            failed = row.get("stage") == "T1" and t1_route_direct_failed(row)
        if failed:
            lines.append(
                f"| {row.get('timestamp', '')} | {row.get('candidate_id', '')} | {row.get('route_id', '')} | "
                f"{row.get('stage', '')} | {row.get('status', '')} | {row.get('notes', '').replace('|', '/')} |"
            )
    (out_dir / "failed_runs.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_t0_summary(out_dir: Path, rows: Sequence[dict[str, str]]) -> None:
    lines = [
        "# T0 Summary",
        "",
        "Route: `shortgen11`. This is diagnostic screening only, not a generalization claim.",
        "",
        "| candidate | status | clean | mission_delta | 40-180_delta | 140-180_delta | 140-160_delta | 160-180_delta | gate | run |",
        "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |",
    ]
    for row in rows:
        if row.get("route_id") != "shortgen11":
            continue
        lines.append(
            f"| {row.get('candidate_id', '')} | {row.get('status', '')} | {row.get('clean_run', '')} | "
            f"{row.get('mission_all_delta', '')} | {row.get('main_40_180_delta', '')} | "
            f"{row.get('window_140_180_delta', '')} | {row.get('window_140_160_delta', '')} | "
            f"{row.get('window_160_180_delta', '')} | {t0_gate_note(row)} | `{row.get('run_label', '')}` |"
        )
    (out_dir / "t0_summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_t1_candidate_matrix(out_dir: Path, rows: Sequence[dict[str, str]]) -> None:
    fields = [
        "candidate_id",
        "route_id",
        "route_shape",
        "status",
        "clean_run",
        "mission_all_delta",
        "main_40_180_delta",
        "baseline_delta_main_40_180",
        "window_140_160_delta",
        "window_160_180_delta",
        "direct_failed_mission_all_gt_0p10",
        "not_worse_than_baseline",
        "shortgen04_no_local_miss",
        "run_label",
    ]
    out_rows: list[dict[str, object]] = []
    for row in rows:
        if row.get("route_id") not in {"shortgen01", "shortgen02", "shortgen03", "shortgen04"}:
            continue
        out_rows.append(
            {
                "candidate_id": row.get("candidate_id", ""),
                "route_id": row.get("route_id", ""),
                "route_shape": ROUTE_SHAPES.get(row.get("route_id", ""), ""),
                "status": row.get("status", ""),
                "clean_run": row.get("clean_run", ""),
                "mission_all_delta": row.get("mission_all_delta", ""),
                "main_40_180_delta": row.get("main_40_180_delta", ""),
                "baseline_delta_main_40_180": row.get("baseline_delta_main_40_180", ""),
                "window_140_160_delta": row.get("window_140_160_delta", ""),
                "window_160_180_delta": row.get("window_160_180_delta", ""),
                "direct_failed_mission_all_gt_0p10": int(t1_route_direct_failed(row)),
                "not_worse_than_baseline": int(not_worse_than_baseline(row, out_dir=out_dir)),
                "shortgen04_no_local_miss": int(shortgen04_no_local_miss(row, out_dir=out_dir)),
                "run_label": row.get("run_label", ""),
            }
        )
    write_csv(out_dir / "t1_candidate_matrix.csv", fields, out_rows)


def write_failure_atlas(out_dir: Path, rows: Sequence[dict[str, str]]) -> None:
    fields = [
        "candidate_id",
        "route_id",
        "route_shape",
        "stage",
        "clean_run",
        "mission_all_delta",
        "main_40_180_delta",
        "window_140_160_delta",
        "window_160_180_delta",
        "failure_reason",
        "run_label",
    ]
    out_rows: list[dict[str, object]] = []
    for row in rows:
        reason = ""
        if row.get("status") != "complete" or row.get("clean_run") != "1":
            reason = row.get("notes", "") or row.get("status", "")
        elif row.get("stage") == "T0" and not candidate_passed_t0(row):
            reason = t0_gate_note(row)
        elif row.get("stage") == "T1" and t1_route_direct_failed(row):
            reason = "T1 direct fail: mission_all_delta > 0.10 m"
        elif row.get("stage") == "T1" and not_worse_than_baseline(row, out_dir=out_dir) is False:
            reason = "worse than seeded baseline"
        elif row.get("route_id") == "shortgen04" and not shortgen04_no_local_miss(row, out_dir=out_dir):
            reason = "shortgen04 local miss"
        elif row.get("stage") == "T2" and not t2_route_safe(row):
            reason = "holdout mission/RTL degradation"
        if not reason:
            continue
        out_rows.append(
            {
                "candidate_id": row.get("candidate_id", ""),
                "route_id": row.get("route_id", ""),
                "route_shape": ROUTE_SHAPES.get(row.get("route_id", ""), ""),
                "stage": row.get("stage", ""),
                "clean_run": row.get("clean_run", ""),
                "mission_all_delta": row.get("mission_all_delta", ""),
                "main_40_180_delta": row.get("main_40_180_delta", ""),
                "window_140_160_delta": row.get("window_140_160_delta", ""),
                "window_160_180_delta": row.get("window_160_180_delta", ""),
                "failure_reason": reason.replace("|", "/"),
                "run_label": row.get("run_label", ""),
            }
        )
    write_csv(out_dir / "failure_atlas.csv", fields, out_rows)


def write_t2_holdout_summary(out_dir: Path, rows: Sequence[dict[str, str]]) -> None:
    lines = [
        "# T2 Holdout Summary",
        "",
        "T2 is verification only. Failed holdout evidence is preserved and does not feed tuning.",
        "",
        "| route | shape | candidate | status | clean | main_40_180_delta | mission_delta | rtl_delta | safe | run |",
        "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |",
    ]
    for row in rows:
        if row.get("stage") != "T2":
            continue
        lines.append(
            f"| {row.get('route_id', '')} | {ROUTE_SHAPES.get(row.get('route_id', ''), '')} | "
            f"{row.get('candidate_id', '')} | {row.get('status', '')} | {row.get('clean_run', '')} | "
            f"{row.get('main_40_180_delta', '')} | {row.get('mission_all_delta', '')} | {row.get('rtl_all_delta', '')} | "
            f"{int(t2_route_safe(row))} | `{row.get('run_label', '')}` |"
        )
    (out_dir / "t2_holdout_summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_stage_summaries(out_dir: Path) -> None:
    rows = read_csv_dicts(out_dir / "leaderboard.csv")
    write_t0_summary(out_dir, rows)
    write_t1_candidate_matrix(out_dir, rows)
    write_failure_atlas(out_dir, rows)
    write_t2_holdout_summary(out_dir, rows)
    reset_failed_runs(out_dir, rows)


def append_failed(out_dir: Path, row: dict[str, object]) -> None:
    path = out_dir / "failed_runs.md"
    with path.open("a", encoding="utf-8") as f:
        f.write(
            f"| {now_iso()} | {row.get('candidate_id', '')} | {row.get('route_id', '')} | "
            f"{row.get('stage', '')} | {row.get('status', '')} | {str(row.get('notes', '')).replace('|', '/')} |\n"
        )


def precheck_route(route_id: str, out_dir: Path = OUT_DIR, dry_run: bool = False) -> dict[str, object]:
    if route_id not in ROUTES:
        raise SystemExit(f"unknown route: {route_id}")
    route = ROUTES[route_id]
    ensure_layout(out_dir)
    write_precheck_wrapper(route, out_dir)
    run_label = f"autoniekf_precheck_{route.route_id}_{now_stamp()}"
    run_dir = out_dir / "prechecks" / run_label
    run_dir.mkdir(parents=True, exist_ok=True)
    write_run_state(out_dir, phase="T2-precheck", status="running", route=route.route_id, run_dir=str(run_dir))
    shutil.copy2(route.plan_path, run_dir / route.plan_path.name)
    if dry_run:
        row = {
            "timestamp": now_iso(),
            "route_id": route.route_id,
            "run_label": run_label,
            "run_dir": str(run_dir),
            "valid": 0,
            "notes": "dry run only; no simulator launched",
        }
        append_csv(out_dir / "validity_prechecks.csv", ["timestamp", "route_id", "run_label", "run_dir", "valid", "success_seq", "expected_success_seq", "mission_count", "simulator_poll_timeout", "notes"], row)
        write_run_state(out_dir, phase="T2-precheck", status="dry_run", route=route.route_id, run_dir=str(run_dir))
        return row
    env = run_env(run_dir, None, None, None)
    try:
        run_session(route.plan_path, run_label, run_dir, env, compare_enabled=False)
    finally:
        stop_session(run_dir, env)
    mission = parse_mission(run_dir)
    valid = mission["valid"] and not mission["poll_timeout"]
    row = {
        "timestamp": now_iso(),
        "route_id": route.route_id,
        "run_label": run_label,
        "run_dir": str(run_dir),
        "valid": valid,
        "success_seq": mission["success_seq"],
        "expected_success_seq": mission["expected_success_seq"],
        "mission_count": mission["mission_count"],
        "simulator_poll_timeout": mission["poll_timeout"],
        "notes": "" if valid else "invalid route precheck or simulator poll timeout",
    }
    append_csv(out_dir / "validity_prechecks.csv", ["timestamp", "route_id", "run_label", "run_dir", "valid", "success_seq", "expected_success_seq", "mission_count", "simulator_poll_timeout", "notes"], row)
    write_run_state(out_dir, phase="T2-precheck", status="valid" if valid else "invalid", route=route.route_id, run_dir=str(run_dir))
    return row


def refresh_best_candidate(out_dir: Path = OUT_DIR) -> None:
    rows = read_csv_dicts(out_dir / "leaderboard.csv")
    complete = [row for row in rows if row.get("status") == "complete" and row.get("clean_run") == "1"]
    candidates = sorted({row.get("candidate_id", "") for row in complete if row.get("candidate_id")})
    lines = ["# Best Candidate", ""]
    conclusion = "failed to beat EKF2 reliably"
    best = ""
    partial: list[str] = []
    for candidate_id in candidates:
        t0 = latest_row(candidate_id, "shortgen11", out_dir)
        t1_passed, t1_total = t1_pass_count(candidate_id, out_dir)
        t2_wins, t2_total = t2_win_count(candidate_id, out_dir)
        if t0 and candidate_passed_t0(t0) and t1_passed >= 3 and t1_total >= 4 and t2_wins >= 4 and t2_total >= 5:
            conclusion = "stable multi-route win"
            best = candidate_id
            break
        if t0 and candidate_passed_t0(t0) and (t1_passed > 0 or t2_wins > 0):
            partial.append(candidate_id)
    if not best and partial:
        conclusion = "partial win, not stable"
        best = rank_t1_candidates(partial, out_dir)[0]
    lines.append(f"Status: `{conclusion}`")
    lines.append("")
    if best:
        lines.append(f"Current best candidate: `{best}`")
        lines.append("")
        for route_id in ["shortgen11", "shortgen01", "shortgen02", "shortgen03", "shortgen04", "shortgen07", "shortgen08", "shortgen09", "shortgen10", "shortgen13", "shortgen16"]:
            row = latest_row(best, route_id, out_dir)
            if row:
                lines.append(
                    f"- {route_id}: clean={row.get('clean_run')} main_delta={row.get('main_40_180_delta')} "
                    f"win_main={row.get('win_main_40_180')} run=`{row.get('run_label')}`"
                )
    else:
        lines.append("No candidate has completed enough clean evidence to pass the T0/T1/T2 gates.")
    lines.extend(
        [
            "",
            "Evidence boundaries:",
            "- T0 shortgen11 is diagnostic only.",
            "- T1 shortgen01/02/03/04 is development/control and known-holdout protection.",
            "- T2 routes are new holdout validation only; they must not feed tuning.",
        ]
    )
    (out_dir / "best_candidate.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def print_status(out_dir: Path = OUT_DIR) -> None:
    rows = read_csv_dicts(out_dir / "leaderboard.csv")
    print(f"out_dir={out_dir}")
    print(f"leaderboard_rows={len(rows)}")
    if rows:
        last = rows[-1]
        print(
            "last="
            f"{last.get('candidate_id')} {last.get('route_id')} status={last.get('status')} "
            f"clean={last.get('clean_run')} main_delta={last.get('main_40_180_delta')}"
        )
    best = out_dir / "best_candidate.md"
    if best.exists():
        print(best.read_text(encoding="utf-8").splitlines()[2])


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out-dir", default=str(OUT_DIR), help="artifact root for the autonomous loop")
    sub = parser.add_subparsers(dest="cmd", required=True)

    sub.add_parser("init", help="create overlays, wrappers, and empty leaderboards")

    run_one_parser = sub.add_parser("run-one", help="run one candidate on one route")
    run_one_parser.add_argument("--candidate", required=True, choices=sorted(CANDIDATES))
    run_one_parser.add_argument("--route", required=True, choices=sorted(ROUTES))
    run_one_parser.add_argument("--dry-run", action="store_true")

    precheck_parser = sub.add_parser("precheck-route", help="run no-compare route validity precheck")
    precheck_parser.add_argument("--route", required=True, choices=sorted(ROUTES))
    precheck_parser.add_argument("--dry-run", action="store_true")

    loop_parser = sub.add_parser("run-loop", help="run T0/T1/T2 acceptance loop")
    loop_parser.add_argument("--time-budget-hours", type=float, default=10.0)
    loop_parser.add_argument("--max-t0-candidates", type=int, default=5)
    loop_parser.add_argument("--max-expanded-candidates", type=int, default=5)
    loop_parser.add_argument("--max-candidate-rounds", type=int, default=len(CANDIDATE_ROUNDS))
    loop_parser.add_argument("--rerun", action="store_true")
    loop_parser.add_argument("--dry-run", action="store_true")

    locate_parser = sub.add_parser("locate-px4-ulog", help="locate and copy the PX4 ULog referenced by a run's px4.log")
    locate_parser.add_argument("--run-dir", required=True)
    locate_parser.add_argument("--no-copy", action="store_true")

    export_parser = sub.add_parser("export-ulog-topics", help="export required ULog topics into an existing run_dir")
    export_parser.add_argument("--run-dir", required=True)

    rebuild_parser = sub.add_parser("rebuild-leaderboard", help="recover ULogs, rerun groundtruth diagnostics, and rewrite leaderboard")
    rebuild_parser.add_argument("--route", choices=sorted(ROUTES))
    rebuild_parser.add_argument("--candidate", choices=sorted(CANDIDATES))

    sub.add_parser("status", help="print concise loop status")

    args = parser.parse_args(argv)
    out_dir = Path(args.out_dir)

    if args.cmd == "init":
        init_artifacts(out_dir)
        print(f"initialized {out_dir}")
        return 0
    if args.cmd == "run-one":
        init_artifacts(out_dir)
        row = run_one(args.candidate, args.route, out_dir, args.dry_run)
        print(f"run_one status={row.get('status')} clean={row.get('clean_run')} run_dir={row.get('run_dir')}")
        return 0
    if args.cmd == "precheck-route":
        init_artifacts(out_dir)
        row = precheck_route(args.route, out_dir, args.dry_run)
        print(f"precheck valid={row.get('valid')} run_dir={row.get('run_dir')}")
        return 0
    if args.cmd == "run-loop":
        return run_loop(args)
    if args.cmd == "locate-px4-ulog":
        path = locate_px4_ulog(Path(args.run_dir), copy_to_run_dir=not args.no_copy)
        print(path if path else "NO_ULOG")
        return 0 if path else 2
    if args.cmd == "export-ulog-topics":
        ok = export_ulog_topics(Path(args.run_dir))
        print(f"export_ok={int(ok)}")
        return 0 if ok else 2
    if args.cmd == "rebuild-leaderboard":
        init_artifacts(out_dir)
        rows = rebuild_leaderboard(out_dir, route_filter=args.route, candidate_filter=args.candidate)
        print(f"rebuilt_rows={len(rows)} out_dir={out_dir}")
        return 0
    if args.cmd == "status":
        print_status(out_dir)
        return 0
    raise SystemExit(f"unsupported command: {args.cmd}")


if __name__ == "__main__":
    raise SystemExit(main())
