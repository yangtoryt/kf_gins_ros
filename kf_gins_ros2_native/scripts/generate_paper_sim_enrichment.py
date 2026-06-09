#!/usr/bin/env python3
"""Build an isolated paper simulation enrichment package.

The generator is offline-first: it reads the frozen/accepted cand19 clean-route
artifacts, writes new tables and figures under a separate artifact root, and
creates paper-only overlays plus wrappers for later ablation runs. It does not
start PX4/Gazebo/MAVROS and does not overwrite the original paper results.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import re
import stat
import textwrap
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


WS_ROOT = Path("/home/yang/kf_gins_ws")
CANDIDATE = "cand19_pos08_arm05_vel04_vrw28"
GT_FRAME = "global_groundtruth/raw_wgs84_enu"
DEFAULT_OUT = WS_ROOT / "artifacts/manual/paper_sim_enrichment_2026-05-16"
PLANS_DIR = WS_ROOT / "artifacts/manual/short_route_generalization_2026-05-07/plans"
BASE_CANDIDATE_DIR = (
    WS_ROOT
    / "artifacts/manual/cand19_expanded_holdout_20260514_1942/candidates"
    / CANDIDATE
)
FROZEN_T2 = (
    WS_ROOT
    / "artifacts/manual/frozen_cand19_t2_paper_package_20260514_193857"
    / "global_groundtruth_t2_cand19/runs"
)
LIVE_T2 = (
    WS_ROOT
    / "artifacts/manual/autonomous_iekf_perf_2026-05-12_night"
    / "reports/global_groundtruth_t2_cand19/runs"
)
LIVE_RUNS = WS_ROOT / "artifacts/manual/autonomous_iekf_perf_2026-05-12_night/runs"
EXPANDED_ROOT = WS_ROOT / "artifacts/manual/cand19_expanded_holdout_20260514_1942"
SCRIPT_DIR = WS_ROOT / "src/kf_gins_ros2_native/scripts"
MANUAL_SESSION = SCRIPT_DIR / "manual_mainline_postflight_session.sh"
MISSION_SMOKE = SCRIPT_DIR / "px4_mission_smoke.py"
ULOG_GPS_GT_DIAG = SCRIPT_DIR / "offline_ulog_gps_groundtruth_diagnostic.py"
GT_DIAG = SCRIPT_DIR / "offline_groundtruth_convergence_diagnostic.py"


@dataclass(frozen=True)
class RouteSpec:
    route: str
    stage: str
    paper_route: str
    route_type: str
    plan_name: str
    default_role: str
    source_group: str


ROUTES: list[RouteSpec] = [
    RouteSpec(
        "shortgen08",
        "T2",
        "Route 01",
        "bowtie",
        "shortgen08_safe_bowtie_sitl_home.plan",
        "clean holdout",
        "t2",
    ),
    RouteSpec(
        "shortgen09",
        "T2",
        "Route 02",
        "stair return",
        "shortgen09_safe_stair_return_sitl_home.plan",
        "T2 weak-positive route for ablation",
        "t2",
    ),
    RouteSpec(
        "shortgen10",
        "T2",
        "Route 03",
        "ladder subset",
        "shortgen10_safe_ladder_subset_sitl_home.plan",
        "clean holdout",
        "t2",
    ),
    RouteSpec(
        "shortgen13",
        "T2",
        "Route 04",
        "cross turn",
        "shortgen13_high_clearance_cross_turn_sitl_home.plan",
        "clean holdout",
        "t2",
    ),
    RouteSpec(
        "shortgen16",
        "T2",
        "Route 05",
        "open NE bowtie",
        "shortgen16_open_ne_bowtie_sitl_home.plan",
        "strong-positive route for ablation",
        "t2",
    ),
    RouteSpec(
        "shortgen17",
        "expanded",
        "Route 06",
        "straight line",
        "shortgen17_clean_straight_sitl_home.plan",
        "low-excitation straight-route sensitivity case",
        "expanded",
    ),
    RouteSpec(
        "shortgen18",
        "expanded",
        "Route 07",
        "rectangle",
        "shortgen18_clean_rectangle_sitl_home.plan",
        "expanded clean holdout",
        "expanded",
    ),
    RouteSpec(
        "shortgen19",
        "expanded",
        "Route 08",
        "L-turn",
        "shortgen19_clean_l_turn_sitl_home.plan",
        "expanded clean holdout",
        "expanded",
    ),
    RouteSpec(
        "shortgen20",
        "expanded",
        "Route 09",
        "S/polyline",
        "shortgen20_clean_s_polyline_sitl_home.plan",
        "expanded clean holdout",
        "expanded",
    ),
    RouteSpec(
        "shortgen21",
        "expanded",
        "Route 10",
        "out-and-back",
        "shortgen21_clean_out_and_back_sitl_home.plan",
        "expanded clean holdout",
        "expanded",
    ),
]


ABLATION_ROUTES = ["shortgen09", "shortgen16", "shortgen17"]
PRIMARY_ABLATION_VARIANTS = [
    "paper_cand19_full",
    "paper_cand19_posonly",
    "paper_cand19_single_iter",
]
OPTIONAL_VARIANTS = ["paper_cand19_iter3", "paper_cand19_cov2x"]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUT)
    return parser.parse_args()


def setup_matplotlib() -> None:
    plt.rcParams.update(
        {
            "figure.dpi": 150,
            "savefig.dpi": 300,
            "font.size": 9.0,
            "axes.titlesize": 10.0,
            "axes.labelsize": 9.0,
            "legend.fontsize": 8.0,
            "axes.grid": True,
            "grid.alpha": 0.22,
            "axes.spines.top": False,
            "axes.spines.right": False,
            "pdf.fonttype": 42,
            "ps.fonttype": 42,
        }
    )


def ensure_layout(out_dir: Path) -> None:
    for name in ["runs", "overlays", "tables", "figures", "wrappers"]:
        (out_dir / name).mkdir(parents=True, exist_ok=True)


def route_by_id(route_id: str) -> RouteSpec:
    for route in ROUTES:
        if route.route == route_id:
            return route
    raise KeyError(route_id)


def latest_matching(root: Path, pattern: str) -> Path | None:
    matches = sorted(root.glob(pattern))
    return matches[-1] if matches else None


def joined_dir(route: RouteSpec) -> Path | None:
    if route.source_group == "t2":
        frozen = FROZEN_T2 / route.route
        live = LIVE_T2 / route.route
        if (frozen / "groundtruth_joined.csv").exists():
            return frozen
        if (live / "groundtruth_joined.csv").exists():
            return live
        return None
    run = latest_matching(EXPANDED_ROOT / "runs", f"autoniekf_{route.route}_{CANDIDATE}_*")
    if run and (run / "offline_groundtruth_global_raw/groundtruth_joined.csv").exists():
        return run / "offline_groundtruth_global_raw"
    return None


def raw_run_dir(route: RouteSpec) -> Path | None:
    def best_run(root: Path, pattern: str) -> Path | None:
        matches = sorted(root.glob(pattern), reverse=True)
        for match in matches:
            if (match / "state_update_debug.csv").exists() and (match / "ekf_iekf_pairs.csv").exists():
                return match
        return matches[0] if matches else None

    if route.source_group == "t2":
        return best_run(LIVE_RUNS, f"autoniekf_{route.route}_{CANDIDATE}_*")
    return best_run(EXPANDED_ROOT / "runs", f"autoniekf_{route.route}_{CANDIDATE}_*")


def read_joined(route: RouteSpec) -> pd.DataFrame:
    root = joined_dir(route)
    if root is None:
        raise FileNotFoundError(f"missing groundtruth_joined.csv for {route.route}")
    df = pd.read_csv(root / "groundtruth_joined.csv")
    numeric_cols = [
        "time_since_arm_sec",
        "mavros_armed",
        "turning_now",
        "post_turn_context",
        "horizontal_speed_mps",
        "vertical_speed_mps",
        "gyro_deg_s",
        "source_yaw_rate_deg_s",
        "ekf2_error_xy_m",
        "iekf_error_xy_m",
        "ekf2_error_z_m",
        "iekf_error_z_m",
    ]
    for col in numeric_cols:
        if col in df.columns:
            df[col] = pd.to_numeric(df[col], errors="coerce")
    return df


def finite(series: pd.Series) -> pd.Series:
    return np.isfinite(pd.to_numeric(series, errors="coerce"))


def main_window(df: pd.DataFrame) -> pd.Series:
    armed = pd.to_numeric(df.get("mavros_armed", 0), errors="coerce").fillna(0) == 1
    t = pd.to_numeric(df.get("time_since_arm_sec", math.nan), errors="coerce")
    return armed & (t >= 40.0) & (t <= 180.0)


def metric_values(values: pd.Series) -> dict[str, float]:
    data = pd.to_numeric(values, errors="coerce")
    data = data[np.isfinite(data)]
    if data.empty:
        return {
            "rows": 0,
            "rmse": math.nan,
            "mae": math.nan,
            "p95": math.nan,
            "max": math.nan,
        }
    abs_data = data.abs()
    return {
        "rows": int(data.shape[0]),
        "rmse": float(np.sqrt(np.mean(np.square(data)))),
        "mae": float(abs_data.mean()),
        "p95": float(abs_data.quantile(0.95)),
        "max": float(abs_data.max()),
    }


def pct_improvement(ekf2: float, iekf: float) -> float:
    if not math.isfinite(ekf2) or ekf2 == 0.0 or not math.isfinite(iekf):
        return math.nan
    return 100.0 * (ekf2 - iekf) / ekf2


def first_finite(*values: pd.Series) -> pd.Series:
    if not values:
        return pd.Series(dtype=float)
    result = pd.to_numeric(values[0], errors="coerce").copy()
    for series in values[1:]:
        candidate = pd.to_numeric(series, errors="coerce")
        result = result.where(np.isfinite(result), candidate)
    return result


def classify_intensity(turn_ratio: float, yaw_p95: float) -> str:
    if not math.isfinite(turn_ratio):
        return "unknown"
    if turn_ratio < 0.10 and (not math.isfinite(yaw_p95) or yaw_p95 < 3.0):
        return "low"
    if turn_ratio < 0.35 and (not math.isfinite(yaw_p95) or yaw_p95 < 10.0):
        return "medium"
    return "high"


def route_metrics(route: RouteSpec) -> dict[str, object]:
    df = read_joined(route)
    mask = main_window(df)
    if "ekf2_error_xy_m" not in df.columns or "iekf_error_xy_m" not in df.columns:
        raise RuntimeError(f"{route.route} joined CSV lacks XY error columns")

    ekf2_xy = metric_values(df.loc[mask & finite(df["ekf2_error_xy_m"]), "ekf2_error_xy_m"])
    iekf_xy = metric_values(df.loc[mask & finite(df["iekf_error_xy_m"]), "iekf_error_xy_m"])

    has_z = "ekf2_error_z_m" in df.columns and "iekf_error_z_m" in df.columns
    if has_z:
        ekf2_z = metric_values(df.loc[mask & finite(df["ekf2_error_z_m"]), "ekf2_error_z_m"])
        iekf_z = metric_values(df.loc[mask & finite(df["iekf_error_z_m"]), "iekf_error_z_m"])
        ekf2_3d = np.sqrt(np.square(df["ekf2_error_xy_m"]) + np.square(df["ekf2_error_z_m"]))
        iekf_3d = np.sqrt(np.square(df["iekf_error_xy_m"]) + np.square(df["iekf_error_z_m"]))
        ekf2_3d_metrics = metric_values(ekf2_3d.loc[mask & finite(ekf2_3d)])
        iekf_3d_metrics = metric_values(iekf_3d.loc[mask & finite(iekf_3d)])
        z_status = "available_in_joined_csv"
    else:
        ekf2_z, iekf_z, ekf2_3d_metrics, iekf_3d_metrics, z_status = vertical_3d_from_pair(route, df, mask)

    turning = pd.to_numeric(df.get("turning_now", 0), errors="coerce").fillna(0) > 0.5
    post_turn = pd.to_numeric(df.get("post_turn_context", 0), errors="coerce").fillna(0) > 0.5
    turn_ratio = float((turning.loc[mask] | post_turn.loc[mask]).mean()) if mask.any() else math.nan
    yaw = first_finite(
        df.get("source_yaw_rate_deg_s", pd.Series(index=df.index, dtype=float)).abs(),
        df.get("gyro_deg_s", pd.Series(index=df.index, dtype=float)).abs(),
    )
    yaw_main = yaw.loc[mask & finite(yaw)]
    yaw_p95 = float(yaw_main.quantile(0.95)) if not yaw_main.empty else math.nan
    hspeed = pd.to_numeric(df.get("horizontal_speed_mps", pd.Series(index=df.index, dtype=float)), errors="coerce")
    hspeed_main = hspeed.loc[mask & finite(hspeed)]

    run_dir = raw_run_dir(route)
    eval_dir = joined_dir(route)
    return {
        "route": route.route,
        "paper_route": route.paper_route,
        "stage": route.stage,
        "route_type": route.route_type,
        "role": route.default_role,
        "candidate": CANDIDATE,
        "world": "empty.world",
        "vehicle": "Iris",
        "gnss_condition": "clean",
        "scoring_frame": GT_FRAME,
        "evaluation_window": "40-180 s after arm",
        "rows": int(min(ekf2_xy["rows"], iekf_xy["rows"])),
        "ekf2_xy_rmse_m": ekf2_xy["rmse"],
        "iekf_xy_rmse_m": iekf_xy["rmse"],
        "rmse_delta_m": iekf_xy["rmse"] - ekf2_xy["rmse"],
        "rmse_improvement_pct": pct_improvement(ekf2_xy["rmse"], iekf_xy["rmse"]),
        "ekf2_xy_mae_m": ekf2_xy["mae"],
        "iekf_xy_mae_m": iekf_xy["mae"],
        "ekf2_xy_p95_m": ekf2_xy["p95"],
        "iekf_xy_p95_m": iekf_xy["p95"],
        "ekf2_xy_max_m": ekf2_xy["max"],
        "iekf_xy_max_m": iekf_xy["max"],
        "ekf2_vertical_rmse_m": ekf2_z["rmse"],
        "iekf_vertical_rmse_m": iekf_z["rmse"],
        "vertical_rmse_delta_m": iekf_z["rmse"] - ekf2_z["rmse"],
        "ekf2_3d_rmse_m": ekf2_3d_metrics["rmse"],
        "iekf_3d_rmse_m": iekf_3d_metrics["rmse"],
        "rmse_3d_delta_m": iekf_3d_metrics["rmse"] - ekf2_3d_metrics["rmse"],
        "vertical_3d_status": z_status,
        "turn_ratio": turn_ratio,
        "yaw_rate_p95_deg_s": yaw_p95,
        "maneuver_intensity": classify_intensity(turn_ratio, yaw_p95),
        "horizontal_speed_mean_mps": float(hspeed_main.mean()) if not hspeed_main.empty else math.nan,
        "horizontal_speed_p95_mps": float(hspeed_main.quantile(0.95)) if not hspeed_main.empty else math.nan,
        "eval_dir": str(eval_dir) if eval_dir else "",
        "run_dir": str(run_dir) if run_dir else "",
    }


def empty_metric() -> dict[str, float]:
    return {
        "rows": 0,
        "rmse": math.nan,
        "mae": math.nan,
        "p95": math.nan,
        "max": math.nan,
    }


def vertical_3d_from_pair(
    route: RouteSpec, joined: pd.DataFrame, mask: pd.Series
) -> tuple[dict[str, float], dict[str, float], dict[str, float], dict[str, float], str]:
    run_dir = raw_run_dir(route)
    if not run_dir:
        return empty_metric(), empty_metric(), empty_metric(), empty_metric(), "missing_run_dir_for_pair_z"
    pair_path = run_dir / "ekf_iekf_pairs.csv"
    if not pair_path.exists():
        return empty_metric(), empty_metric(), empty_metric(), empty_metric(), "missing_pair_z_csv"
    needed = {"pair_ros_time_sec", "gt_z_m", "ekf2_error_xy_m", "iekf_error_xy_m"}
    if not needed.issubset(set(joined.columns)):
        return empty_metric(), empty_metric(), empty_metric(), empty_metric(), "missing_join_columns_for_pair_z"

    pair = pd.read_csv(pair_path)
    pair_cols = {"ros_time_sec", "ekf2_z_m"}
    if not pair_cols.issubset(set(pair.columns)):
        return empty_metric(), empty_metric(), empty_metric(), empty_metric(), "missing_pair_z_columns"
    if "iekf_aligned_z_m" not in pair.columns:
        if {"iekf_z_m", "align_offset_z_m"}.issubset(set(pair.columns)):
            pair["iekf_aligned_z_m"] = pd.to_numeric(pair["iekf_z_m"], errors="coerce") + pd.to_numeric(
                pair["align_offset_z_m"], errors="coerce"
            )
        else:
            return empty_metric(), empty_metric(), empty_metric(), empty_metric(), "missing_iekf_aligned_z"

    selected = joined.loc[mask, ["pair_ros_time_sec", "gt_z_m", "ekf2_error_xy_m", "iekf_error_xy_m"]].copy()
    selected["pair_ros_time_sec"] = pd.to_numeric(selected["pair_ros_time_sec"], errors="coerce")
    selected["gt_z_m"] = pd.to_numeric(selected["gt_z_m"], errors="coerce")
    selected = selected[np.isfinite(selected["pair_ros_time_sec"]) & np.isfinite(selected["gt_z_m"])]
    pair = pair[["ros_time_sec", "ekf2_z_m", "iekf_aligned_z_m"]].copy()
    for col in pair.columns:
        pair[col] = pd.to_numeric(pair[col], errors="coerce")
    pair = pair[np.isfinite(pair["ros_time_sec"])].sort_values("ros_time_sec")
    selected = selected.sort_values("pair_ros_time_sec")
    if selected.empty or pair.empty:
        return empty_metric(), empty_metric(), empty_metric(), empty_metric(), "empty_pair_z_merge"

    merged = pd.merge_asof(
        selected,
        pair,
        left_on="pair_ros_time_sec",
        right_on="ros_time_sec",
        direction="nearest",
        tolerance=0.05,
    )
    merged["ekf2_error_z_m"] = merged["ekf2_z_m"] - merged["gt_z_m"]
    merged["iekf_error_z_m"] = merged["iekf_aligned_z_m"] - merged["gt_z_m"]
    ekf2_z = metric_values(merged["ekf2_error_z_m"])
    iekf_z = metric_values(merged["iekf_error_z_m"])
    ekf2_3d = np.sqrt(np.square(merged["ekf2_error_xy_m"]) + np.square(merged["ekf2_error_z_m"]))
    iekf_3d = np.sqrt(np.square(merged["iekf_error_xy_m"]) + np.square(merged["iekf_error_z_m"]))
    ekf2_3d_metrics = metric_values(ekf2_3d)
    iekf_3d_metrics = metric_values(iekf_3d)
    if min(ekf2_z["rows"], iekf_z["rows"]) == 0:
        status = "pair_z_merge_no_finite_rows"
    else:
        status = "computed_from_pair_logger_z_nearest_0p05s"
    return ekf2_z, iekf_z, ekf2_3d_metrics, iekf_3d_metrics, status


def straight_turn_metrics(route: RouteSpec) -> list[dict[str, object]]:
    df = read_joined(route)
    mask = main_window(df)
    yaw = first_finite(
        df.get("source_yaw_rate_deg_s", pd.Series(index=df.index, dtype=float)).abs(),
        df.get("gyro_deg_s", pd.Series(index=df.index, dtype=float)).abs(),
    )
    turning = pd.to_numeric(df.get("turning_now", 0), errors="coerce").fillna(0) > 0.5
    post_turn = pd.to_numeric(df.get("post_turn_context", 0), errors="coerce").fillna(0) > 0.5
    speed = pd.to_numeric(df.get("horizontal_speed_mps", 0), errors="coerce").fillna(0)
    turn = mask & (turning | post_turn | (yaw >= 8.0))
    straight = mask & (~turn) & (speed >= 0.5)
    rows: list[dict[str, object]] = []
    for segment, seg_mask in [("straight", straight), ("turn", turn)]:
        ekf2 = metric_values(df.loc[seg_mask & finite(df["ekf2_error_xy_m"]), "ekf2_error_xy_m"])
        iekf = metric_values(df.loc[seg_mask & finite(df["iekf_error_xy_m"]), "iekf_error_xy_m"])
        rows.append(
            {
                "route": route.route,
                "paper_route": route.paper_route,
                "segment": segment,
                "rows": int(min(ekf2["rows"], iekf["rows"])),
                "ekf2_xy_rmse_m": ekf2["rmse"],
                "iekf_xy_rmse_m": iekf["rmse"],
                "rmse_delta_m": iekf["rmse"] - ekf2["rmse"],
                "rmse_improvement_pct": pct_improvement(ekf2["rmse"], iekf["rmse"]),
                "scoring_frame": GT_FRAME,
                "evaluation_window": "40-180 s after arm",
            }
        )
    return rows


def runtime_cost(route: RouteSpec) -> dict[str, object]:
    run_dir = raw_run_dir(route)
    row: dict[str, object] = {
        "route": route.route,
        "paper_route": route.paper_route,
        "candidate": CANDIDATE,
        "run_dir": str(run_dir) if run_dir else "",
        "state_update_debug_exists": 0,
        "ekf_iekf_pairs_exists": 0,
        "mission_smoke_log_exists": 0,
        "manual_summary_exists": 0,
        "mission_completed_marker": "unknown",
        "state_update_rows": math.nan,
        "gnss_position_event_count": math.nan,
        "gnss_position_accepted_count": math.nan,
        "gnss_velocity_event_count": math.nan,
        "gnss_velocity_applied_count": math.nan,
        "applied_event_count": math.nan,
        "mean_update_interval_sec": math.nan,
        "p95_update_interval_sec": math.nan,
        "core_cap_hz": 8.0,
    }
    if not run_dir:
        return row

    row["ekf_iekf_pairs_exists"] = int((run_dir / "ekf_iekf_pairs.csv").exists())
    row["mission_smoke_log_exists"] = int((run_dir / "mission_smoke.log").exists())
    row["manual_summary_exists"] = int((run_dir / "manual_summary.txt").exists())
    if (run_dir / "mission_smoke.log").exists():
        text = (run_dir / "mission_smoke.log").read_text(encoding="utf-8", errors="ignore")
        if re.search(r"mission (completed|success)|success_seq", text, flags=re.IGNORECASE):
            row["mission_completed_marker"] = "present"
        elif "timeout" in text.lower():
            row["mission_completed_marker"] = "timeout_marker_present"

    debug = run_dir / "state_update_debug.csv"
    if not debug.exists():
        return row
    row["state_update_debug_exists"] = 1
    df = pd.read_csv(debug)
    row["state_update_rows"] = int(df.shape[0])
    if "event_type" in df.columns:
        event = df["event_type"].astype(str)
        pos = event == "gnss_position"
        vel = event == "gnss_velocity"
        row["gnss_position_event_count"] = int(pos.sum())
        row["gnss_velocity_event_count"] = int(vel.sum())
        if "gnss_position_update_accepted" in df.columns:
            accepted = pd.to_numeric(df["gnss_position_update_accepted"], errors="coerce").fillna(0) > 0
            row["gnss_position_accepted_count"] = int((pos & accepted).sum())
    if "applied" in df.columns:
        applied = pd.to_numeric(df["applied"], errors="coerce").fillna(0) > 0
        row["applied_event_count"] = int(applied.sum())
        if "event_type" in df.columns:
            row["gnss_velocity_applied_count"] = int(((df["event_type"].astype(str) == "gnss_velocity") & applied).sum())
        time_col = None
        for candidate in ["ros_time_sec", "update_time_sec", "stamp_sec"]:
            if candidate in df.columns:
                time_col = candidate
                break
        if time_col:
            times = pd.to_numeric(df.loc[applied, time_col], errors="coerce")
            times = times[np.isfinite(times)].sort_values()
            if times.shape[0] > 2:
                dt = times.diff().dropna()
                dt = dt[(dt > 0.0) & np.isfinite(dt)]
                if not dt.empty:
                    row["mean_update_interval_sec"] = float(dt.mean())
                    row["p95_update_interval_sec"] = float(dt.quantile(0.95))
    return row


def write_csv(path: Path, rows: Iterable[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    rows = list(rows)
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    fields: list[str] = []
    seen: set[str] = set()
    for row in rows:
        for key in row:
            if key not in seen:
                fields.append(key)
                seen.add(key)
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def save_fig(fig: plt.Figure, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, bbox_inches="tight", pad_inches=0.08)
    plt.close(fig)


def plot_maneuver(metrics: pd.DataFrame, out_dir: Path) -> None:
    data = metrics.sort_values(["maneuver_intensity", "turn_ratio", "route"]).copy()
    x = np.arange(len(data))
    width = 0.38
    fig, ax = plt.subplots(figsize=(11.2, 4.3))
    ax.bar(x - width / 2, data["ekf2_xy_rmse_m"], width, label="PX4 EKF2", color="#2563eb")
    ax.bar(x + width / 2, data["iekf_xy_rmse_m"], width, label="Fixed-parameter IEKF", color="#dc2626")
    labels = [f"{r}\n{m}" for r, m in zip(data["route"], data["maneuver_intensity"])]
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=0)
    ax.set_ylabel("Horizontal RMSE / m")
    ax.set_title(f"Clean-route RMSE by maneuver intensity ({GT_FRAME}, 40-180 s)")
    ax.legend(frameon=True)
    save_fig(fig, out_dir / "figures/rmse_by_maneuver_intensity.png")

    fig, ax = plt.subplots(figsize=(7.2, 4.4))
    colors = {"low": "#64748b", "medium": "#0f766e", "high": "#b45309", "unknown": "#6b7280"}
    for _, row in data.iterrows():
        ax.scatter(
            row["turn_ratio"],
            row["rmse_improvement_pct"],
            s=70,
            color=colors.get(str(row["maneuver_intensity"]), "#6b7280"),
        )
        ax.text(row["turn_ratio"] + 0.006, row["rmse_improvement_pct"], row["route"], va="center", fontsize=8)
    ax.axhline(0.0, color="#111827", lw=0.8)
    ax.set_xlabel("Turn/post-turn ratio in evaluation window")
    ax.set_ylabel("IEKF RMSE improvement / %")
    ax.set_title("RMSE improvement versus route maneuver intensity")
    save_fig(fig, out_dir / "figures/rmse_improvement_vs_turn_ratio.png")


def plot_runtime(runtime: pd.DataFrame, out_dir: Path) -> None:
    data = runtime.sort_values("route").copy()
    x = np.arange(len(data))
    width = 0.38
    fig, ax = plt.subplots(figsize=(10.8, 4.0))
    ax.bar(x - width / 2, data["gnss_position_event_count"], width, label="GNSS position events", color="#2563eb")
    ax.bar(x + width / 2, data["gnss_velocity_event_count"], width, label="GNSS velocity events", color="#0f766e")
    ax.set_xticks(x)
    ax.set_xticklabels(data["route"], rotation=35, ha="right")
    ax.set_ylabel("Event count")
    ax.set_title("Runtime update-event counts from state_update_debug.csv")
    ax.legend(frameon=True)
    save_fig(fig, out_dir / "figures/runtime_cost.png")


def plot_ablation_placeholder(ablation: pd.DataFrame, out_dir: Path) -> None:
    data = ablation.copy()
    routes = ABLATION_ROUTES
    variants = PRIMARY_ABLATION_VARIANTS
    grid = np.zeros((len(variants), len(routes)))
    for i, variant in enumerate(variants):
        for j, route in enumerate(routes):
            status = data.loc[(data["route"] == route) & (data["variant"] == variant), "status"]
            grid[i, j] = 1.0 if not status.empty and status.iloc[0] == "complete" else 0.0
    fig, ax = plt.subplots(figsize=(7.0, 3.2))
    ax.imshow(grid, cmap="Greens", vmin=0.0, vmax=1.0, aspect="auto")
    ax.set_xticks(np.arange(len(routes)))
    ax.set_xticklabels(routes)
    ax.set_yticks(np.arange(len(variants)))
    ax.set_yticklabels([v.replace("paper_cand19_", "") for v in variants])
    for i in range(len(variants)):
        for j in range(len(routes)):
            ax.text(j, i, "pending", ha="center", va="center", color="#111827", fontsize=8)
    ax.set_title(f"Ablation matrix status ({GT_FRAME}, 40-180 s)")
    save_fig(fig, out_dir / "figures/ablation_rmse_bar.png")


def render_scalar(value: object) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    return str(value)


def replace_yaml_scalar(text: str, key: str, value: object) -> str:
    rendered = render_scalar(value)
    pattern = re.compile(rf"^(\s*{re.escape(key)}:\s*).*$", re.MULTILINE)
    new_text, count = pattern.subn(rf"\g<1>{rendered}", text, count=1)
    if not count:
        raise RuntimeError(f"cannot replace yaml key {key}")
    return new_text


def upsert_top_level_scalar(text: str, key: str, value: object, after_key: str = "filter_mode") -> str:
    rendered = render_scalar(value)
    pattern = re.compile(rf"^({re.escape(key)}:\s*).*$", re.MULTILINE)
    new_text, count = pattern.subn(rf"\g<1>{rendered}", text, count=1)
    if count:
        return new_text
    after = re.compile(rf"^({re.escape(after_key)}:\s*.*)$", re.MULTILINE)
    new_text, count = after.subn(rf"\1\n{key}: {rendered}", text, count=1)
    if not count:
        return f"{key}: {rendered}\n{text}"
    return new_text


def write_variant_overlays(out_dir: Path) -> list[dict[str, object]]:
    base_kfgins_path = BASE_CANDIDATE_DIR / "kfgins_overlay.yaml"
    base_core_path = BASE_CANDIDATE_DIR / "kf_core_overlay.yaml"
    if not base_kfgins_path.exists() or not base_core_path.exists():
        raise FileNotFoundError(f"missing base cand19 overlays under {BASE_CANDIDATE_DIR}")

    base_kfgins = base_kfgins_path.read_text(encoding="utf-8")
    base_core = base_core_path.read_text(encoding="utf-8")
    variants: list[dict[str, object]] = []

    def write_variant(name: str, kfgins: str, core: str, purpose: str) -> None:
        target = out_dir / "overlays" / name
        target.mkdir(parents=True, exist_ok=True)
        (target / "kfgins_overlay.yaml").write_text(kfgins, encoding="utf-8")
        (target / "kf_core_overlay.yaml").write_text(core, encoding="utf-8")
        (target / "variant.md").write_text(
            f"# {name}\n\n"
            f"- base: `{CANDIDATE}`\n"
            f"- purpose: {purpose}\n"
            f"- scoring frame for this package: `{GT_FRAME}`\n"
            "- scope: paper-only enrichment; original cand19 artifacts are not modified.\n",
            encoding="utf-8",
        )
        variants.append(
            {
                "variant": name,
                "purpose": purpose,
                "kfgins_overlay": str(target / "kfgins_overlay.yaml"),
                "kf_core_overlay": str(target / "kf_core_overlay.yaml"),
            }
        )

    write_variant(
        "paper_cand19_full",
        base_kfgins,
        base_core,
        "frozen full cand19 reference",
    )
    write_variant(
        "paper_cand19_posonly",
        replace_yaml_scalar(base_kfgins, "enable_gnss_velocity_update", False),
        base_core,
        "velocity-aiding ablation; all other cand19 parameters retained",
    )
    core_single = upsert_top_level_scalar(replace_yaml_scalar(base_core, "filter_mode", 0), "iekf_max_iterations", 1)
    write_variant(
        "paper_cand19_single_iter",
        base_kfgins,
        core_single,
        "single-iteration EKF-style update ablation",
    )
    core_iter3 = upsert_top_level_scalar(replace_yaml_scalar(base_core, "filter_mode", 1), "iekf_max_iterations", 3)
    write_variant(
        "paper_cand19_iter3",
        base_kfgins,
        core_iter3,
        "optional IEKF iteration-count sensitivity",
    )
    cov2x = base_kfgins
    for key, value in [
        ("sim_gnss_std_h_m", 0.16),
        ("sim_gnss_std_u_m", 0.20),
        ("armed_cruise_gnss_pos_std_h_m", 0.10),
        ("armed_cruise_gnss_pos_std_u_m", 0.16),
    ]:
        cov2x = replace_yaml_scalar(cov2x, key, value)
    write_variant(
        "paper_cand19_cov2x",
        cov2x,
        base_core,
        "optional measurement covariance sensitivity; not a GNSS-denial test",
    )
    write_csv(out_dir / "tables/variant_manifest.csv", variants)
    return variants


def sh(path: Path | str) -> str:
    text = str(path)
    return "'" + text.replace("'", "'\"'\"'") + "'"


def write_wrappers(out_dir: Path) -> None:
    wrappers = out_dir / "wrappers"
    run_one = wrappers / "run_one_enrichment.sh"
    run_one.write_text(
        textwrap.dedent(
            f"""\
            #!/usr/bin/env bash
            set -euo pipefail

            variant="${{1:?usage: $0 <variant> <route>}}"
            route="${{2:?usage: $0 <variant> <route>}}"
            ws={sh(WS_ROOT)}
            out_root={sh(out_dir)}
            plans_dir={sh(PLANS_DIR)}
            scripts_dir={sh(SCRIPT_DIR)}
            timestamp="$(date +%Y%m%d_%H%M%S)"
            run_label="paper_${{route}}_${{variant}}_${{timestamp}}"
            run_dir="${{out_root}}/runs/${{run_label}}"
            overlay_dir="${{out_root}}/overlays/${{variant}}"

            case "${{route}}" in
              shortgen08) plan="shortgen08_safe_bowtie_sitl_home.plan" ;;
              shortgen09) plan="shortgen09_safe_stair_return_sitl_home.plan" ;;
              shortgen10) plan="shortgen10_safe_ladder_subset_sitl_home.plan" ;;
              shortgen13) plan="shortgen13_high_clearance_cross_turn_sitl_home.plan" ;;
              shortgen16) plan="shortgen16_open_ne_bowtie_sitl_home.plan" ;;
              shortgen17) plan="shortgen17_clean_straight_sitl_home.plan" ;;
              shortgen18) plan="shortgen18_clean_rectangle_sitl_home.plan" ;;
              shortgen19) plan="shortgen19_clean_l_turn_sitl_home.plan" ;;
              shortgen20) plan="shortgen20_clean_s_polyline_sitl_home.plan" ;;
              shortgen21) plan="shortgen21_clean_out_and_back_sitl_home.plan" ;;
              *) echo "unknown route: ${{route}}" >&2; exit 2 ;;
            esac

            if [[ ! -f "${{overlay_dir}}/kfgins_overlay.yaml" || ! -f "${{overlay_dir}}/kf_core_overlay.yaml" ]]; then
              echo "missing overlay directory: ${{overlay_dir}}" >&2
              exit 2
            fi

            case "${{variant}}" in
              paper_cand19_cov2x)
                sim_gnss_std_h_m=0.16
                sim_gnss_std_u_m=0.20
                armed_cruise_gnss_pos_std_h_m=0.10
                armed_cruise_gnss_pos_std_u_m=0.16
                ;;
              *)
                sim_gnss_std_h_m=0.08
                sim_gnss_std_u_m=0.10
                armed_cruise_gnss_pos_std_h_m=0.05
                armed_cruise_gnss_pos_std_u_m=0.08
                ;;
            esac

            mkdir -p "${{run_dir}}"
            cp "${{overlay_dir}}/kfgins_overlay.yaml" "${{run_dir}}/kfgins_overlay.yaml"
            cp "${{overlay_dir}}/kf_core_overlay.yaml" "${{run_dir}}/kf_core_overlay.yaml"
            cp "${{plans_dir}}/${{plan}}" "${{run_dir}}/${{plan}}"
            cat > "${{run_dir}}/compare_meta.txt" <<EOF
            run_label=${{run_label}}
            run_dir=${{run_dir}}
            candidate={CANDIDATE}
            paper_variant=${{variant}}
            route=${{route}}
            plan=${{plans_dir}}/${{plan}}
            profile=paper_sim_enrichment_2026-05-16
            scoring_frame={GT_FRAME}
            created_at=$(date --iso-8601=seconds)
            EOF

            export ROS_DOMAIN_ID=0
            export PX4_SAFE_PX4_WORLD=none
            export PX4_SAFE_HEADLESS=1
            export PX4_SAFE_NO_PXH=1
            export PX4_SAFE_START_QGC=0
            export PX4_SAFE_START_MONITOR=1
            export PX4_SAFE_START_COMPARE=1
            export PX4_SAFE_START_GPS_PROBES=1
            export PX4_SAFE_START_PAIR_LOGGER=1
            export PX4_SAFE_PAIR_LOGGER_RATE_HZ=10
            export PX4_SAFE_PAIR_LOGGER_SYNC_TOLERANCE_MS=50
            export PX4_SAFE_COMPARE_CORE_MAX_IMU_RATE_HZ=8.0
            export PX4_SAFE_COMPARE_EKF2_USE_INPUT_STAMP=true
            export PX4_SAFE_COMPARE_GNSS_RELAY_MODE=px4_sensor_gps
            export PX4_SAFE_COMPARE_ENABLE_GPS_DROPZONES=false
            export PX4_SAFE_COMPARE_INJECT_DROPZONE_GPS_TO_PX4=false
            export PX4_SAFE_COMPARE_PX4_GPS_INJECTION_MODE=off
            export PX4_SAFE_COMPARE_PX4_SET_PARAMS=false
            export PX4_SAFE_COMPARE_KF_GINS_PARAM_FILE="${{overlay_dir}}/kfgins_overlay.yaml"
            export PX4_SAFE_COMPARE_KF_GINS_CORE_CONFIG_FILE="${{overlay_dir}}/kf_core_overlay.yaml"
            export PX4_SAFE_COMPARE_USE_SIM_GNSS_STD=true
            export PX4_SAFE_COMPARE_SIM_GNSS_STD_H_M="${{sim_gnss_std_h_m}}"
            export PX4_SAFE_COMPARE_SIM_GNSS_STD_U_M="${{sim_gnss_std_u_m}}"
            export PX4_SAFE_COMPARE_ARMED_CRUISE_GNSS_POS_OVERRIDE_ENABLE=true
            export PX4_SAFE_COMPARE_ARMED_CRUISE_GNSS_POS_OVERRIDE_APPLY_MISSION=true
            export PX4_SAFE_COMPARE_ARMED_CRUISE_GNSS_POS_OVERRIDE_APPLY_RTL=true
            export PX4_SAFE_COMPARE_ARMED_CRUISE_GNSS_POS_OVERRIDE_APPLY_OTHER=true
            export PX4_SAFE_COMPARE_ARMED_CRUISE_GNSS_POS_STD_H_M="${{armed_cruise_gnss_pos_std_h_m}}"
            export PX4_SAFE_COMPARE_ARMED_CRUISE_GNSS_POS_STD_U_M="${{armed_cruise_gnss_pos_std_u_m}}"
            export PX4_SAFE_COMPARE_PUBLISH_STATE_AFTER_GNSS_UPDATE=false
            export PX4_SAFE_COMPARE_PUBLISH_STAMP_MODE=core_fixed_offset
            export PX4_SAFE_COMPARE_PUBLISH_CORE_STAMP_MAX_FUTURE_SEC=0.0
            export PX4_SAFE_COMPARE_PUBLISH_CORE_STAMP_OFFSET_BIAS_SEC=-0.03
            export PX4_SAFE_COMPARE_RAW_ODOM_DECIMATION=50
            export PX4_SAFE_COMPARE_PATH_PUBLISH_RATE_HZ=5.0
            export PX4_SAFE_COMPARE_POSE_DECIMATION=5
            export PX4_SAFE_COMPARE_MAX_PATH_POINTS=20000
            export PX4_SAFE_COMPARE_GNSS_UPDATE_DEBUG_CSV_PATH="${{run_dir}}/gnss_update_debug.csv"
            export PX4_SAFE_COMPARE_STATE_UPDATE_DEBUG_CSV_PATH="${{run_dir}}/state_update_debug.csv"
            export PX4_SAFE_COMPARE_STATE_UPDATE_DEBUG_MAX_RATE_HZ=2.0
            export PX4_SAFE_COMPARE_STATE_PUBLISH_DEBUG_CSV_PATH="${{run_dir}}/state_publish_debug.csv"
            export PX4_SAFE_COMPARE_GNSS_NIS_DEBUG_CSV_PATH=__disabled__
            export PX4_SAFE_COMPARE_HORIZONTAL_CONSISTENCY_DEBUG_CSV_PATH=__disabled__
            export PX4_SAFE_COMPARE_SEGMENT_TIMING_GATE_DEBUG_CSV_PATH=__disabled__
            export PX4_SAFE_COMPARE_PUBLISH_PX4_SPHERE_PROJECTION=false
            export PX4_SAFE_COMPARE_SEGMENT_TIMING_GATE_PROJECTION_ENABLE=false
            export PX4_SAFE_COMPARE_SEGMENT_TIMING_GATE_PROJECTION_APPLY_MISSION=false
            export PX4_SAFE_COMPARE_SEGMENT_TIMING_GATE_PROJECTION_APPLY_RTL=false
            export PX4_SAFE_COMPARE_SEGMENT_TIMING_GATE_PROJECTION_APPLY_OTHER=false
            export PX4_SAFE_COMPARE_ACCBIAS_Z_HISTORY_PROJECTION_ENABLE=false
            export PX4_SAFE_COMPARE_EARLY_RECOVERY_BIAS_FEEDBACK_DEBUG_ENABLE=false
            export PX4_SAFE_COMPARE_EARLY_RECOVERY_BIAS_FEEDBACK_APPLY_ENABLE=false
            export PX4_SAFE_COMPARE_GNSS_POSITION_RESPONSE_BOOST_ENABLE=false
            export PX4_SAFE_COMPARE_GNSS_POSITION_GAIN_RESPONSE_ENABLE=false
            export PX4_SAFE_COMPARE_TURN_RATE_PROPAGATION_NOISE_PROBE_ENABLE=false
            export PX4_SAFE_COMPARE_GNSS_VELOCITY_OUTWARD_DAMPING_ENABLE=false
            export PX4_SAFE_COMPARE_TURN_POSTTURN_NATIVE_VELOCITY_DEWEIGHT_ENABLE=false
            export PX4_SAFE_COMPARE_ADAPTIVE_GNSS_POS_WEIGHT_ENABLE=false
            export PX4_SAFE_COMPARE_HORIZONTAL_CONSISTENCY_SUPERVISOR_ENABLE=false
            export PX4_SAFE_COMPARE_MISSION_COV_HYGIENE_ENABLE=false
            export PX4_SAFE_COMPARE_MOTION_GNSS_POS_WEIGHT_ENABLE=false
            export PX4_SAFE_COMPARE_GNSS_POS_RECOVERY_WEIGHT_ENABLE=false
            export PX4_SAFE_COMPARE_ACCBIAS_Z_PROPAGATION_PROBE_ENABLE=false
            export PX4_SAFE_COMPARE_ACCBIAS_Z_PROPAGATION_PROBE_APPLY_NOISE_SCALE=false
            export PX4_SAFE_COMPARE_NATIVE_GNSS_VELOCITY_OUTLIER_GUARD_ENABLE=false
            export PX4_SAFE_COMPARE_NATIVE_GNSS_VELOCITY_LOW_SPEED_TURN_SOURCE_GUARD_ENABLE=false

            cd "${{ws}}"
            set +u
            source /opt/ros/${{ROS_DISTRO:-humble}}/setup.bash
            source "${{ws}}/install/setup.bash"
            set -u

            cleanup() {{
              "${{scripts_dir}}/manual_mainline_postflight_session.sh" stop "${{run_dir}}" > "${{run_dir}}/stop.log" 2>&1 || true
              "${{scripts_dir}}/manual_mainline_postflight_session.sh" summary "${{run_dir}}" > "${{run_dir}}/manual_summary.txt" 2>&1 || true
              pgrep -af 'PX4-Autopilot|px4|gz sim|gzserver|gzclient|MicroXRCEAgent|mavros|kf_gins_node|ekf_iekf_pair_logger|px4_mission_smoke' > "${{run_dir}}/residual_process_check.txt" 2>&1 || true
            }}
            trap cleanup EXIT

            "${{scripts_dir}}/manual_mainline_postflight_session.sh" show-env "${{run_dir}}" > "${{run_dir}}/env.txt" 2>&1 || true
            "${{scripts_dir}}/manual_mainline_postflight_session.sh" start "${{run_dir}}" > "${{run_dir}}/start_console.log" 2>&1
            python3 "${{scripts_dir}}/px4_mission_smoke.py" --plan-file "${{plans_dir}}/${{plan}}" --pre-arm-settle 8 --arm-retries 6 --mission-timeout 650 --post-mission-settle 45 --print-period 5 > "${{run_dir}}/mission_smoke.log" 2>&1 || true
            cleanup

            echo "run_dir=${{run_dir}}"
            echo "Next: bash ${{out_root}}/wrappers/score_run_global_raw.sh ${{run_dir}}"
            """
        ),
        encoding="utf-8",
    )
    run_one.chmod(run_one.stat().st_mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)

    score_run = wrappers / "score_run_global_raw.sh"
    score_run.write_text(
        textwrap.dedent(
            f"""\
            #!/usr/bin/env bash
            set -euo pipefail

            run_dir="${{1:?usage: $0 <run_dir>}}"
            scripts_dir={sh(SCRIPT_DIR)}
            run_label="$(basename "${{run_dir}}")"
            cd {sh(WS_ROOT)}
            set +u
            source /opt/ros/${{ROS_DISTRO:-humble}}/setup.bash
            source {sh(WS_ROOT / "install/setup.bash")}
            set -u

            python3 {sh(SCRIPT_DIR / "autonomous_iekf_perf_loop.py")} export-ulog-topics --run-dir "${{run_dir}}"
            python3 {sh(ULOG_GPS_GT_DIAG)} --ulog-dir "${{run_dir}}" --out-dir "${{run_dir}}/offline_ulog_gps_groundtruth_diag" > "${{run_dir}}/offline_ulog_gps_groundtruth_diag.log" 2>&1
            fit_csv="${{run_dir}}/offline_ulog_gps_groundtruth_diag/gps_groundtruth_fit.csv"
            python3 {sh(GT_DIAG)} --run-label "${{run_label}}" --run-dir "${{run_dir}}" --ulog-dir "${{run_dir}}" --fit-csv "${{fit_csv}}" --out-dir "${{run_dir}}/offline_groundtruth_global_raw" --groundtruth-source global --projection-mode raw_wgs84_enu > "${{run_dir}}/offline_groundtruth_convergence_diag.log" 2>&1
            echo "${{run_dir}}/offline_groundtruth_global_raw/groundtruth_summary.csv"
            """
        ),
        encoding="utf-8",
    )
    score_run.chmod(score_run.stat().st_mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)

    matrix = wrappers / "run_ablation_matrix.sh"
    matrix.write_text(
        textwrap.dedent(
            f"""\
            #!/usr/bin/env bash
            set -euo pipefail
            out_root={sh(out_dir)}
            routes=(shortgen09 shortgen16 shortgen17)
            variants=(paper_cand19_full paper_cand19_posonly paper_cand19_single_iter)
            for route in "${{routes[@]}}"; do
              for variant in "${{variants[@]}}"; do
                bash "${{out_root}}/wrappers/run_one_enrichment.sh" "${{variant}}" "${{route}}"
              done
            done
            """
        ),
        encoding="utf-8",
    )
    matrix.chmod(matrix.stat().st_mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)

    dry = wrappers / "dry_run_matrix.sh"
    dry.write_text(
        textwrap.dedent(
            f"""\
            #!/usr/bin/env bash
            set -euo pipefail
            out_root={sh(out_dir)}
            for route in shortgen09 shortgen16 shortgen17; do
              for variant in paper_cand19_full paper_cand19_posonly paper_cand19_single_iter; do
                echo "bash ${{out_root}}/wrappers/run_one_enrichment.sh ${{variant}} ${{route}}"
              done
            done
            echo "# optional:"
            for route in shortgen09 shortgen16 shortgen17; do
              for variant in paper_cand19_iter3 paper_cand19_cov2x; do
                echo "# bash ${{out_root}}/wrappers/run_one_enrichment.sh ${{variant}} ${{route}}"
              done
            done
            """
        ),
        encoding="utf-8",
    )
    dry.chmod(dry.stat().st_mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)


def ablation_plan_rows(out_dir: Path) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    variants = PRIMARY_ABLATION_VARIANTS + OPTIONAL_VARIANTS
    for route_id in ABLATION_ROUTES:
        route = route_by_id(route_id)
        for variant in variants:
            optional = int(variant in OPTIONAL_VARIANTS)
            rows.append(
                {
                    "route": route.route,
                    "paper_route": route.paper_route,
                    "route_type": route.route_type,
                    "variant": variant,
                    "status": "pending_run",
                    "optional": optional,
                    "evaluation_window": "40-180 s after arm",
                    "scoring_frame": GT_FRAME,
                    "run_command": f"bash {out_dir}/wrappers/run_one_enrichment.sh {variant} {route.route}",
                    "score_command": "bash {out_dir}/wrappers/score_run_global_raw.sh <run_dir>".format(out_dir=out_dir),
                    "interpretation": variant_interpretation(variant),
                }
            )
    return rows


def variant_interpretation(variant: str) -> str:
    if variant == "paper_cand19_full":
        return "main fixed-parameter IEKF reference"
    if variant == "paper_cand19_posonly":
        return "isolates GNSS velocity aiding contribution"
    if variant == "paper_cand19_single_iter":
        return "isolates IEKF iteration contribution"
    if variant == "paper_cand19_iter3":
        return "optional IEKF iteration-count sensitivity"
    if variant == "paper_cand19_cov2x":
        return "optional measurement covariance sensitivity"
    return ""


def write_report(out_dir: Path, metrics: pd.DataFrame, runtime: pd.DataFrame) -> None:
    main = metrics.copy()
    wins = int((main["rmse_delta_m"] < 0.0).sum())
    total = int(main.shape[0])
    mean_improvement = float(main["rmse_improvement_pct"].mean())
    shortgen17 = main.loc[main["route"] == "shortgen17"].iloc[0] if (main["route"] == "shortgen17").any() else None
    z_statuses = sorted(set(str(v) for v in main["vertical_3d_status"].dropna().unique()))
    runtime_ok = int(((runtime["state_update_debug_exists"] == 1) & (runtime["ekf_iekf_pairs_exists"] == 1)).sum())

    text = [
        "# Paper Simulation Enrichment Package",
        "",
        f"- Artifact root: `{out_dir}`",
        f"- Main frozen candidate: `{CANDIDATE}`",
        "- Original cand19 results are read-only inputs; this package does not overwrite them.",
        f"- Scoring frame: `{GT_FRAME}`",
        "- World/vehicle/sensor scope: `empty.world`, Iris, clean GNSS.",
        "",
        "## Frozen Baseline Manifest",
        "",
        f"- Routes included: {total}",
        f"- IEKF wins by horizontal RMSE: {wins}/{total}",
        f"- Mean horizontal RMSE improvement: {mean_improvement:.2f}%",
        "- The accepted headline remains the prior paper result: T2 5/5 wins, combined 9/10 wins, mean RMSE improvement 44.38%.",
        "",
        "## New Offline Enrichment",
        "",
        "- `tables/baseline_manifest.csv`: frozen clean-route manifest and horizontal RMSE/MAE/p95/max.",
        "- `tables/vertical_3d_metrics.csv`: horizontal metrics plus vertical/3D metrics from pair-logger z joined to global groundtruth timestamps.",
        "- `tables/maneuver_intensity_summary.csv`: turn ratio, yaw-rate p95, and route intensity class.",
        "- `tables/straight_turn_aggregate.csv`: straight/turn horizontal RMSE slices from the main window.",
        "- `tables/runtime_cost_summary.csv`: state-update and GNSS-update event counts from existing logs.",
        "- `figures/rmse_by_maneuver_intensity.png`: route-level RMSE grouped by maneuver intensity.",
        "- `figures/rmse_improvement_vs_turn_ratio.png`: improvement versus turn/post-turn ratio.",
        "- `figures/runtime_cost.png`: runtime update-event count summary.",
        "",
        "## Vertical And 3D Note",
        "",
        f"- Current vertical/3D status values: `{', '.join(z_statuses)}`.",
        "- Current vertical/3D values are derived by nearest-time joining pair-logger z with global-groundtruth rows; rerun after new scoring if the joined CSV format changes.",
        "",
        "## Ablation Execution Plan",
        "",
        "- Main matrix: `shortgen09`, `shortgen16`, `shortgen17` x `paper_cand19_full`, `paper_cand19_posonly`, `paper_cand19_single_iter`.",
        "- Optional matrix: `paper_cand19_iter3` and `paper_cand19_cov2x` on the same three routes.",
        "- `paper_cand19_cov2x` is measurement covariance sensitivity, not degraded-GNSS robustness.",
        "",
        "Run commands:",
        "",
        "```bash",
        f"bash {out_dir}/wrappers/dry_run_matrix.sh",
        f"bash {out_dir}/wrappers/run_ablation_matrix.sh",
        "```",
        "",
        "After each new run, score it with:",
        "",
        "```bash",
        f"bash {out_dir}/wrappers/score_run_global_raw.sh <run_dir>",
        "```",
        "",
        "## Paper Wording",
        "",
        "- Prefer `Discussion and Applicability` or `Scope of Evaluation` instead of a long limitations block.",
        "- Suggested scope sentence: `The present evaluation focuses on clean-GNSS SITL and global-groundtruth scoring; broader degraded-GNSS and hardware validation are left for subsequent work.`",
        "- Do not put shortgen17 in the abstract. In discussion, describe it as a low-excitation straight-route sensitivity case.",
        "",
        "## Guardrails",
        "",
        "- Do not use `city_dense`, GPS denial/dropzones, online NIS gating, Sage-Husa adaptation, or local-groundtruth victory claims for this package.",
        "- Keep `rmse_delta_m = IEKF_RMSE - EKF2_RMSE`; negative values mean IEKF is better.",
        f"- Existing logs with both pair and state-update files available: {runtime_ok}/{total}.",
    ]
    if shortgen17 is not None:
        text.insert(
            text.index("## Ablation Execution Plan") - 1,
            f"- shortgen17 is labeled as `{shortgen17['role']}` and should not be framed as a headline failure.",
        )
    (out_dir / "report.md").write_text("\n".join(text) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    out_dir = args.out_dir.resolve()
    ensure_layout(out_dir)
    setup_matplotlib()

    metrics_rows = [route_metrics(route) for route in ROUTES]
    straight_rows: list[dict[str, object]] = []
    for route in ROUTES:
        straight_rows.extend(straight_turn_metrics(route))
    runtime_rows = [runtime_cost(route) for route in ROUTES]

    write_csv(out_dir / "tables/baseline_manifest.csv", metrics_rows)
    write_csv(out_dir / "tables/vertical_3d_metrics.csv", metrics_rows)
    maneuver_rows = [
        {
            key: row[key]
            for key in [
                "route",
                "paper_route",
                "route_type",
                "role",
                "turn_ratio",
                "yaw_rate_p95_deg_s",
                "maneuver_intensity",
                "horizontal_speed_mean_mps",
                "horizontal_speed_p95_mps",
                "ekf2_xy_rmse_m",
                "iekf_xy_rmse_m",
                "rmse_delta_m",
                "rmse_improvement_pct",
                "evaluation_window",
                "scoring_frame",
            ]
        }
        for row in metrics_rows
    ]
    write_csv(out_dir / "tables/maneuver_intensity_summary.csv", maneuver_rows)
    write_csv(out_dir / "tables/straight_turn_aggregate.csv", straight_rows)
    write_csv(out_dir / "tables/runtime_cost_summary.csv", runtime_rows)
    write_variant_overlays(out_dir)
    write_wrappers(out_dir)
    ablation_rows = ablation_plan_rows(out_dir)
    write_csv(out_dir / "tables/ablation_summary.csv", ablation_rows)

    metrics = pd.DataFrame(metrics_rows)
    runtime = pd.DataFrame(runtime_rows)
    ablation = pd.DataFrame(ablation_rows)
    plot_maneuver(metrics, out_dir)
    plot_runtime(runtime, out_dir)
    plot_ablation_placeholder(ablation, out_dir)
    write_report(out_dir, metrics, runtime)

    print(out_dir)
    print(out_dir / "report.md")
    print(out_dir / "tables/baseline_manifest.csv")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
