#!/usr/bin/env python3
"""Offline PHS5 time-semantics audit.

This script reads existing PHS5 logs only. It checks whether publish-stamp,
pair-logger, and groundtruth-join semantics can explain the remaining
shortgen11 failure without starting any simulator or estimator process.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence


BASE = Path("/home/yang/kf_gins_ws/artifacts/manual/short_route_generalization_2026-05-07")
DEFAULT_OUT = BASE / "phs5_time_semantics_audit_2026-05-11"


@dataclass(frozen=True)
class RunSpec:
    label: str
    role: str
    group: str
    path: Path
    mode_subdir: str
    source: str


RUNS = [
    RunSpec(
        "shortgen01_phs5c",
        "development/control",
        "positive",
        Path("/home/yang/kf_gins_ws/artifacts/manual/manual_shortgen01_s_bend_sitl_home_headless_compare_core8_pairlogger_phs5c_publish_core_stamp_bias_m003_20260510_003839"),
        "offline_groundtruth_projection_modes_phs_check",
        "local",
    ),
    RunSpec(
        "shortgen02_phs5b",
        "development/control",
        "positive",
        Path("/home/yang/kf_gins_ws/artifacts/manual/manual_shortgen02_box_uturn_sitl_home_headless_compare_core8_pairlogger_phs5b_publish_core_stamp_bias_m003_20260510_003144"),
        "offline_groundtruth_projection_modes_phs_check",
        "local",
    ),
    RunSpec(
        "shortgen03_phs5a",
        "development/control",
        "positive",
        Path("/home/yang/kf_gins_ws/artifacts/manual/manual_shortgen03_figure8_sitl_home_headless_compare_core8_pairlogger_phs5a_publish_core_stamp_bias_m003_20260510_002551"),
        "offline_groundtruth_projection_modes_phs_check",
        "local",
    ),
    RunSpec(
        "shortgen04_hld1a_phs5",
        "clean holdout positive with local miss",
        "positive",
        Path("/home/yang/kf_gins_ws/artifacts/manual/manual_shortgen04_ladder_sitl_home_headless_compare_core8_pairlogger_hld1a_phs5_publish_core_stamp_bias_m003_20260510_005528"),
        "offline_groundtruth_projection_modes_phs_check",
        "global",
    ),
    RunSpec(
        "shortgen04_futureclamp_global",
        "corrected protection rerun",
        "positive",
        Path("/home/yang/kf_gins_ws/artifacts/manual/manual_shortgen04_ladder_sitl_home_headless_compare_core8_pairlogger_phs5_coretrace_futureclamp_20260510_231317"),
        "offline_groundtruth_global_projection_modes",
        "global",
    ),
    RunSpec(
        "shortgen04_futureclamp_local",
        "same corrected run, local-source diagnostic",
        "diagnostic",
        Path("/home/yang/kf_gins_ws/artifacts/manual/manual_shortgen04_ladder_sitl_home_headless_compare_core8_pairlogger_phs5_coretrace_futureclamp_20260510_231317"),
        "offline_groundtruth_projection_modes",
        "local",
    ),
    RunSpec(
        "shortgen11_repeat2",
        "accepted clean negative",
        "negative",
        Path("/home/yang/kf_gins_ws/artifacts/manual/manual_shortgen11_high_clearance_ladder_sitl_home_headless_compare_core8_pairlogger_phs5_repeat2_cleancheck_20260510_155253"),
        "offline_groundtruth_projection_modes",
        "local",
    ),
    RunSpec(
        "shortgen11_coretrace",
        "timestamp-confounded enriched-trace diagnostic",
        "diagnostic",
        Path("/home/yang/kf_gins_ws/artifacts/manual/manual_shortgen11_high_clearance_ladder_sitl_home_headless_compare_core8_pairlogger_phs5_coretrace_20260510_223106"),
        "offline_groundtruth_projection_modes",
        "local",
    ),
]

WINDOWS = [
    ("main_40_180", 40.0, 180.0),
    ("60_100", 60.0, 100.0),
    ("100_120", 100.0, 120.0),
    ("120_140", 120.0, 140.0),
    ("140_160", 140.0, 160.0),
    ("160_180", 160.0, 180.0),
]

SHIFT_SECONDS = (-0.10, -0.08, -0.06, -0.04, -0.02, 0.0, 0.02, 0.04, 0.06, 0.08, 0.10)


def finite(value: object) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def to_float(value: object, default: float = math.nan) -> float:
    try:
        result = float(value)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return default
    return result if math.isfinite(result) else default


def fmt(value: object, digits: int = 4) -> str:
    val = to_float(value)
    return f"{val:.{digits}f}" if finite(val) else "nan"


def mean(values: Iterable[float]) -> float:
    vals = [value for value in values if finite(value)]
    return sum(vals) / len(vals) if vals else math.nan


def rmse(values: Iterable[float]) -> float:
    vals = [value for value in values if finite(value)]
    return math.sqrt(sum(value * value for value in vals) / len(vals)) if vals else math.nan


def percentile(values: Iterable[float], pct: float) -> float:
    vals = sorted(value for value in values if finite(value))
    if not vals:
        return math.nan
    if len(vals) == 1:
        return vals[0]
    k = (len(vals) - 1) * pct / 100.0
    lo = math.floor(k)
    hi = math.ceil(k)
    if lo == hi:
        return vals[lo]
    return vals[lo] * (hi - k) + vals[hi] * (k - lo)


def max_finite(values: Iterable[float]) -> float:
    vals = [value for value in values if finite(value)]
    return max(vals) if vals else math.nan


def min_finite(values: Iterable[float]) -> float:
    vals = [value for value in values if finite(value)]
    return min(vals) if vals else math.nan


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))


def write_csv(path: Path, rows: Sequence[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames: list[str] = []
    seen: set[str] = set()
    for row in rows:
        for key in row:
            if key not in seen:
                seen.add(key)
                fieldnames.append(key)
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def markdown_table(headers: Sequence[str], rows: Sequence[Sequence[object]]) -> str:
    lines = [
        "| " + " | ".join(headers) + " |",
        "| " + " | ".join("---" for _ in headers) + " |",
    ]
    lines.extend("| " + " | ".join(str(item) for item in row) + " |" for row in rows)
    return "\n".join(lines)


def nearest(
    rows: Sequence[dict[str, object]],
    times: Sequence[float],
    t: float,
    max_dt: float,
) -> tuple[dict[str, object] | None, float]:
    if not finite(t) or not rows:
        return None, math.nan
    i = bisect.bisect_left(times, t)
    candidates: list[tuple[float, dict[str, object]]] = []
    for j in (i - 1, i):
        if 0 <= j < len(rows):
            dt = abs(times[j] - t)
            candidates.append((dt, rows[j]))
    if not candidates:
        return None, math.nan
    dt, row = min(candidates, key=lambda item: item[0])
    return (row, dt) if dt <= max_dt else (None, dt)


def numeric_rows(path: Path) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for raw in read_csv(path):
        row: dict[str, object] = dict(raw)
        for key, value in raw.items():
            val = to_float(value)
            if finite(val):
                row[key] = val
        rows.append(row)
    return rows


def in_window(row: dict[str, object], start: float, end: float) -> bool:
    t = to_float(row.get("time_since_arm_sec"))
    return start <= t < end


def mode_dir(spec: RunSpec) -> Path:
    return spec.path / spec.mode_subdir


def load_run_rows(spec: RunSpec) -> list[dict[str, object]]:
    gt_path = mode_dir(spec) / "raw_wgs84_enu" / "groundtruth_joined.csv"
    pair_path = spec.path / "ekf_iekf_pairs.csv"
    state_path = spec.path / "state_publish_debug.csv"
    for path in (gt_path, pair_path, state_path):
        if not path.exists():
            raise FileNotFoundError(f"{spec.label}: missing {path}")

    gt_rows = numeric_rows(gt_path)
    pair_rows = numeric_rows(pair_path)
    state_rows = numeric_rows(state_path)
    pair_rows.sort(key=lambda row: to_float(row.get("ros_time_sec")))
    state_rows.sort(key=lambda row: to_float(row.get("ros_time_sec")))
    pair_times = [to_float(row.get("ros_time_sec")) for row in pair_rows]
    state_times = [to_float(row.get("ros_time_sec")) for row in state_rows]

    out: list[dict[str, object]] = []
    for gt in gt_rows:
        t_arm = to_float(gt.get("time_since_arm_sec"))
        if not (40.0 <= t_arm < 180.0):
            continue
        if to_float(gt.get("mavros_armed"), 0.0) < 0.5:
            continue
        pair_t = to_float(gt.get("pair_ros_time_sec"))
        pair, pair_dt = nearest(pair_rows, pair_times, pair_t, 0.08)
        state, state_dt = nearest(state_rows, state_times, pair_t, 0.16)

        selected_minus_now = to_float(state.get("publish_stamp_selected_minus_now_sec")) if state else math.nan
        state_ros = to_float(state.get("ros_time_sec")) if state else math.nan
        odom_stamp = to_float(state.get("odom_stamp_sec")) if state else math.nan
        core_time = to_float(state.get("last_core_time_sec")) if state else math.nan
        core_to_ros_offset = to_float(state.get("publish_core_to_ros_offset_sec")) if state else math.nan
        core_age = (
            state_ros - (core_time + core_to_ros_offset)
            if all(finite(value) for value in (state_ros, core_time, core_to_ros_offset))
            else math.nan
        )

        row = {
            "run": spec.label,
            "role": spec.role,
            "group": spec.group,
            "source": spec.source,
            "time_since_arm_sec": t_arm,
            "pair_ros_time_sec": pair_t,
            "stamp_sec": to_float(gt.get("stamp_sec")),
            "mavros_mode": gt.get("mavros_mode", ""),
            "gt_join_dt_sec": to_float(gt.get("gt_join_dt_sec")),
            "context_join_dt_sec": to_float(gt.get("context_join_dt_sec")),
            "turning_now": to_float(gt.get("turning_now")),
            "post_turn_context": to_float(gt.get("post_turn_context")),
            "armed_cruise_context": to_float(gt.get("armed_cruise_context")),
            "horizontal_speed_mps": to_float(gt.get("horizontal_speed_mps")),
            "gyro_deg_s": to_float(gt.get("gyro_deg_s")),
            "gt_x_m": to_float(gt.get("gt_x_m")),
            "gt_y_m": to_float(gt.get("gt_y_m")),
            "ekf2_x_m": to_float(gt.get("ekf2_aligned_x_m")),
            "ekf2_y_m": to_float(gt.get("ekf2_aligned_y_m")),
            "iekf_x_m": to_float(gt.get("iekf_normalized_aligned_x_m")),
            "iekf_y_m": to_float(gt.get("iekf_normalized_aligned_y_m")),
            "ekf2_error_m": to_float(gt.get("ekf2_error_xy_m")),
            "iekf_error_m": to_float(gt.get("iekf_error_xy_m")),
            "core_gnss_diff_h_m": to_float(gt.get("core_gnss_diff_h_m")),
            "pair_join_dt_sec": pair_dt,
            "pair_sync_dt_ms": to_float(pair.get("sync_dt_ms")) if pair else math.nan,
            "pair_ros_minus_iekf_stamp_sec": (
                pair_t - to_float(pair.get("iekf_stamp_sec")) if pair and finite(pair_t) else math.nan
            ),
            "pair_ros_minus_ekf2_stamp_sec": (
                pair_t - to_float(pair.get("ekf2_stamp_sec")) if pair and finite(pair_t) else math.nan
            ),
            "state_join_dt_sec": state_dt,
            "publish_stamp_selected_minus_now_sec": selected_minus_now,
            "stamp_lag_sec": -selected_minus_now if finite(selected_minus_now) else math.nan,
            "core_age_sec": core_age,
            "pair_ros_minus_odom_stamp_sec": pair_t - odom_stamp if finite(pair_t) and finite(odom_stamp) else math.nan,
            "iekf_stamp_minus_odom_stamp_sec": (
                to_float(pair.get("iekf_stamp_sec")) - odom_stamp if pair and finite(odom_stamp) else math.nan
            ),
            "gnss_source_age_sec": (
                state_ros - to_float(state.get("last_gnss_source_time_sec")) if state and finite(state_ros) else math.nan
            ),
            "state_core_gnss_diff_h_m": to_float(state.get("core_gnss_diff_h_m")) if state else math.nan,
        }
        out.append(row)
    out.sort(key=lambda row: to_float(row.get("time_since_arm_sec")))
    return out


def shifted_gt_error(row: dict[str, object], gt_row: dict[str, object], estimator: str) -> float:
    x = to_float(row.get(f"{estimator}_x_m"))
    y = to_float(row.get(f"{estimator}_y_m"))
    gt_x = to_float(gt_row.get("gt_x_m"))
    gt_y = to_float(gt_row.get("gt_y_m"))
    return math.hypot(x - gt_x, y - gt_y) if all(finite(v) for v in (x, y, gt_x, gt_y)) else math.nan


def build_shift_rows(rows_by_run: dict[str, list[dict[str, object]]]) -> list[dict[str, object]]:
    out: list[dict[str, object]] = []
    for run, rows in rows_by_run.items():
        rows = sorted(rows, key=lambda row: to_float(row.get("pair_ros_time_sec")))
        times = [to_float(row.get("pair_ros_time_sec")) for row in rows]
        for label, start, end in WINDOWS:
            subset = [row for row in rows if in_window(row, start, end)]
            if not subset:
                continue
            for shift in SHIFT_SECONDS:
                ekf2_errors: list[float] = []
                iekf_errors: list[float] = []
                used = 0
                for row in subset:
                    shifted_t = to_float(row.get("pair_ros_time_sec")) + shift
                    gt_row, dt = nearest(rows, times, shifted_t, 0.08)
                    if gt_row is None:
                        continue
                    used += 1
                    ekf2_errors.append(shifted_gt_error(row, gt_row, "ekf2"))
                    iekf_errors.append(shifted_gt_error(row, gt_row, "iekf"))
                out.append(
                    {
                        "run": run,
                        "window": label,
                        "shift_sec": shift,
                        "rows": used,
                        "ekf2_rmse_m": rmse(ekf2_errors),
                        "iekf_rmse_m": rmse(iekf_errors),
                        "iekf_minus_ekf2_rmse_m": rmse(iekf_errors) - rmse(ekf2_errors),
                    }
                )
    return out


def summarize_window(rows: Sequence[dict[str, object]], label: str, start: float, end: float) -> dict[str, object]:
    subset = [row for row in rows if in_window(row, start, end)]
    first = subset[0] if subset else {}

    def vals(field: str) -> list[float]:
        return [to_float(row.get(field)) for row in subset]

    return {
        "run": first.get("run", ""),
        "role": first.get("role", ""),
        "group": first.get("group", ""),
        "source": first.get("source", ""),
        "window": label,
        "rows": len(subset),
        "ekf2_rmse_m": rmse(vals("ekf2_error_m")),
        "iekf_rmse_m": rmse(vals("iekf_error_m")),
        "iekf_minus_ekf2_rmse_m": rmse(vals("iekf_error_m")) - rmse(vals("ekf2_error_m")),
        "gt_join_abs_p95_sec": percentile((abs(v) for v in vals("gt_join_dt_sec")), 95.0),
        "gt_join_mean_sec": mean(vals("gt_join_dt_sec")),
        "pair_sync_abs_p95_ms": percentile((abs(v) for v in vals("pair_sync_dt_ms")), 95.0),
        "pair_ros_minus_iekf_stamp_mean_sec": mean(vals("pair_ros_minus_iekf_stamp_sec")),
        "pair_ros_minus_iekf_stamp_p95_sec": percentile(vals("pair_ros_minus_iekf_stamp_sec"), 95.0),
        "state_join_abs_p95_sec": percentile((abs(v) for v in vals("state_join_dt_sec")), 95.0),
        "selected_minus_now_mean_sec": mean(vals("publish_stamp_selected_minus_now_sec")),
        "selected_minus_now_p95_sec": percentile(vals("publish_stamp_selected_minus_now_sec"), 95.0),
        "selected_minus_now_max_sec": max_finite(vals("publish_stamp_selected_minus_now_sec")),
        "future_stamp_frac": mean(1.0 if v > 1e-6 else 0.0 for v in vals("publish_stamp_selected_minus_now_sec")),
        "stamp_lag_mean_sec": mean(vals("stamp_lag_sec")),
        "core_age_mean_sec": mean(vals("core_age_sec")),
        "core_age_p95_sec": percentile(vals("core_age_sec"), 95.0),
        "pair_ros_minus_odom_stamp_mean_sec": mean(vals("pair_ros_minus_odom_stamp_sec")),
        "iekf_stamp_minus_odom_stamp_mean_sec": mean(vals("iekf_stamp_minus_odom_stamp_sec")),
        "gnss_source_age_mean_sec": mean(vals("gnss_source_age_sec")),
        "core_gnss_diff_h_mean_m": mean(vals("core_gnss_diff_h_m")),
    }


def best_shift_summary(shift_rows: Sequence[dict[str, object]]) -> list[dict[str, object]]:
    out: list[dict[str, object]] = []
    by_key: dict[tuple[str, str], list[dict[str, object]]] = {}
    for row in shift_rows:
        by_key.setdefault((str(row.get("run")), str(row.get("window"))), []).append(row)
    for (run, window), rows in by_key.items():
        base = next((row for row in rows if abs(to_float(row.get("shift_sec"))) < 1e-9), None)
        best_iekf = min(rows, key=lambda row: to_float(row.get("iekf_rmse_m"), math.inf))
        best_gap = min(rows, key=lambda row: to_float(row.get("iekf_minus_ekf2_rmse_m"), math.inf))
        out.append(
            {
                "run": run,
                "window": window,
                "base_rows": to_float(base.get("rows")) if base else math.nan,
                "base_iekf_rmse_m": to_float(base.get("iekf_rmse_m")) if base else math.nan,
                "base_ekf2_rmse_m": to_float(base.get("ekf2_rmse_m")) if base else math.nan,
                "base_gap_m": to_float(base.get("iekf_minus_ekf2_rmse_m")) if base else math.nan,
                "best_iekf_shift_sec": to_float(best_iekf.get("shift_sec")),
                "best_iekf_rmse_m": to_float(best_iekf.get("iekf_rmse_m")),
                "best_iekf_gain_m": (
                    to_float(base.get("iekf_rmse_m")) - to_float(best_iekf.get("iekf_rmse_m"))
                    if base else math.nan
                ),
                "best_gap_shift_sec": to_float(best_gap.get("shift_sec")),
                "best_gap_m": to_float(best_gap.get("iekf_minus_ekf2_rmse_m")),
                "best_gap_gain_m": (
                    to_float(base.get("iekf_minus_ekf2_rmse_m")) - to_float(best_gap.get("iekf_minus_ekf2_rmse_m"))
                    if base else math.nan
                ),
            }
        )
    out.sort(key=lambda row: (str(row.get("run")), str(row.get("window"))))
    return out


def build_shift_acceptance_rows(shift_rows: Sequence[dict[str, object]]) -> list[dict[str, object]]:
    positives = {
        "shortgen01_phs5c",
        "shortgen02_phs5b",
        "shortgen03_phs5a",
        "shortgen04_hld1a_phs5",
        "shortgen04_futureclamp_global",
    }
    accepted_negative = "shortgen11_repeat2"
    by_shift: dict[float, dict[tuple[str, str], dict[str, object]]] = {}
    base: dict[tuple[str, str], dict[str, object]] = {}
    for row in shift_rows:
        shift = to_float(row.get("shift_sec"))
        key = (str(row.get("run")), str(row.get("window")))
        by_shift.setdefault(shift, {})[key] = row
        if abs(shift) < 1e-9:
            base[key] = row

    out: list[dict[str, object]] = []
    for shift in sorted(by_shift):
        rows = by_shift[shift]
        positive_regs: list[float] = []
        positive_gaps: list[float] = []
        for run in positives:
            key = (run, "main_40_180")
            if key not in rows or key not in base:
                continue
            positive_regs.append(to_float(rows[key].get("iekf_rmse_m")) - to_float(base[key].get("iekf_rmse_m")))
            positive_gaps.append(to_float(rows[key].get("iekf_minus_ekf2_rmse_m")))
        neg_key = (accepted_negative, "140_160")
        neg = rows.get(neg_key)
        neg_base = base.get(neg_key)
        out.append(
            {
                "shift_sec": shift,
                "positive_main_max_regress_m": max_finite(positive_regs),
                "positive_main_max_gap_m": max_finite(positive_gaps),
                "positive_protected_regression_2cm": int(max_finite(positive_regs) <= 0.02),
                "negative_140_160_iekf_rmse_m": to_float(neg.get("iekf_rmse_m")) if neg else math.nan,
                "negative_140_160_iekf_gain_m": (
                    to_float(neg_base.get("iekf_rmse_m")) - to_float(neg.get("iekf_rmse_m"))
                    if neg and neg_base else math.nan
                ),
                "negative_140_160_gap_m": to_float(neg.get("iekf_minus_ekf2_rmse_m")) if neg else math.nan,
                "negative_repaired_2cm": int(to_float(neg.get("iekf_minus_ekf2_rmse_m")) <= 0.02) if neg else 0,
                "strict_protected_repair": int(
                    max_finite(positive_regs) <= 0.02
                    and neg is not None
                    and to_float(neg.get("iekf_minus_ekf2_rmse_m")) <= 0.02
                ),
            }
        )
    return out


def compact_window_table(rows: Sequence[dict[str, object]]) -> list[list[object]]:
    show = [
        row for row in rows
        if row.get("window") in {"main_40_180", "120_140", "140_160", "160_180"}
        and row.get("run") in {
            "shortgen04_hld1a_phs5",
            "shortgen04_futureclamp_global",
            "shortgen11_repeat2",
            "shortgen11_coretrace",
        }
    ]
    return [
        [
            row["run"],
            row["window"],
            fmt(row["iekf_minus_ekf2_rmse_m"]),
            fmt(row["gt_join_abs_p95_sec"], 3),
            fmt(row["pair_sync_abs_p95_ms"], 2),
            fmt(row["selected_minus_now_mean_sec"], 3),
            fmt(row["selected_minus_now_p95_sec"], 3),
            fmt(row["future_stamp_frac"], 3),
            fmt(row["core_age_mean_sec"], 3),
            fmt(row["pair_ros_minus_odom_stamp_mean_sec"], 3),
        ]
        for row in show
    ]


def compact_shift_table(rows: Sequence[dict[str, object]]) -> list[list[object]]:
    show = [
        row for row in rows
        if row.get("window") in {"main_40_180", "140_160", "160_180"}
        and row.get("run") in {
            "shortgen04_hld1a_phs5",
            "shortgen04_futureclamp_global",
            "shortgen11_repeat2",
            "shortgen11_coretrace",
        }
    ]
    return [
        [
            row["run"],
            row["window"],
            fmt(row["base_iekf_rmse_m"]),
            fmt(row["base_gap_m"]),
            fmt(row["best_iekf_shift_sec"], 2),
            fmt(row["best_iekf_rmse_m"]),
            fmt(row["best_iekf_gain_m"]),
            fmt(row["best_gap_shift_sec"], 2),
            fmt(row["best_gap_m"]),
        ]
        for row in show
    ]


def compact_acceptance_table(rows: Sequence[dict[str, object]]) -> list[list[object]]:
    return [
        [
            fmt(row["shift_sec"], 2),
            fmt(row["positive_main_max_regress_m"]),
            fmt(row["positive_main_max_gap_m"]),
            row["positive_protected_regression_2cm"],
            fmt(row["negative_140_160_iekf_rmse_m"]),
            fmt(row["negative_140_160_iekf_gain_m"]),
            fmt(row["negative_140_160_gap_m"]),
            row["negative_repaired_2cm"],
            row["strict_protected_repair"],
        ]
        for row in rows
    ]


def write_report(
    out_dir: Path,
    window_rows: Sequence[dict[str, object]],
    shift_summary: Sequence[dict[str, object]],
    shift_acceptance: Sequence[dict[str, object]],
) -> None:
    sg11_140 = next(
        row for row in shift_summary
        if row.get("run") == "shortgen11_repeat2" and row.get("window") == "140_160"
    )
    sg11_win = next(
        row for row in window_rows
        if row.get("run") == "shortgen11_repeat2" and row.get("window") == "140_160"
    )
    coretrace_main = next(
        row for row in window_rows
        if row.get("run") == "shortgen11_coretrace" and row.get("window") == "main_40_180"
    )
    repaired_shifts = [row for row in shift_acceptance if to_float(row.get("negative_repaired_2cm")) > 0.5]
    best_repaired = min(
        repaired_shifts,
        key=lambda row: to_float(row.get("positive_main_max_regress_m"), math.inf),
        default=None,
    )
    lines = [
        "# PHS5 time-semantics audit",
        "",
        "Date: 2026-05-11",
        "",
        "Offline-only audit over existing PHS5 logs. No flight, rebuild, PX4, Gazebo, MAVROS, QGC, RViz2, PlotJuggler, or estimator process was run.",
        "",
        "## Scope",
        "",
        "- Hypothesis: shortgen11 residual failure may be amplified by publish-selected stamp, pair logger sampling, and groundtruth join semantics rather than by another projection selector.",
        "- Accepted negative: shortgen11 repeat2. Diagnostic-only: shortgen11 coretrace, because its future-stamp policy differs.",
        "- Shift sensitivity uses already joined groundtruth rows as a local time series and rescoring offsets from -0.10 s to +0.10 s. This is a scoring/join diagnostic, not a proposed global timestamp shift.",
        "",
        "## Time Semantics Windows",
        "",
        markdown_table(
            [
                "run",
                "window",
                "IEKF-EKF2 RMSE",
                "GT |dt| p95",
                "pair sync p95 ms",
                "sel-now mean",
                "sel-now p95",
                "future frac",
                "core age mean",
                "pair-odom mean",
            ],
            compact_window_table(window_rows),
        ),
        "",
        "## Shift Sensitivity",
        "",
        markdown_table(
            [
                "run",
                "window",
                "base IEKF",
                "base gap",
                "best IEKF shift",
                "best IEKF",
                "IEKF gain",
                "best gap shift",
                "best gap",
            ],
            compact_shift_table(shift_summary),
        ),
        "",
        "## Cross-Route Shift Acceptance",
        "",
        markdown_table(
            [
                "shift",
                "positive max regress",
                "positive max gap",
                "protected",
                "sg11 140-160 IEKF",
                "sg11 IEKF gain",
                "sg11 gap",
                "sg11 repaired",
                "strict pass",
            ],
            compact_acceptance_table(shift_acceptance),
        ),
        "",
        "## Readout",
        "",
        f"- shortgen11 repeat2 `140_160` has selected-minus-now mean `{fmt(sg11_win['selected_minus_now_mean_sec'], 3)}` s and p95 `{fmt(sg11_win['selected_minus_now_p95_sec'], 3)}` s, so it is not future-stamped like the coretrace diagnostic.",
        f"- Its local GT-join shift sweep is sensitive but not protected: base IEKF RMSE `{fmt(sg11_140['base_iekf_rmse_m'])}` m, best IEKF RMSE `{fmt(sg11_140['best_iekf_rmse_m'])}` m at shift `{fmt(sg11_140['best_iekf_shift_sec'], 2)}` s, and best per-window IEKF-EKF2 gap `{fmt(sg11_140['best_gap_m'])}` m.",
        (
            f"- The least-damaging shift that repairs accepted shortgen11 repeat2 is `{fmt(best_repaired['shift_sec'], 2)}` s, but it regresses positive main windows by up to `{fmt(best_repaired['positive_main_max_regress_m'])}` m."
            if best_repaired is not None else
            "- No tested shift repairs accepted shortgen11 repeat2 even before protection checks."
        ),
        f"- shortgen11 coretrace remains timestamp-confounded: main-window future-stamp fraction `{fmt(coretrace_main['future_stamp_frac'], 3)}`, selected-minus-now mean `{fmt(coretrace_main['selected_minus_now_mean_sec'], 3)}` s.",
        "- Therefore the accepted shortgen11 repeat2 failure can be made to look better by a scoring-time shift, but not by a cross-route protected time-semantics correction.",
        "- This closes time-semantics as a deployable primary repair path. The next mechanism hypothesis should move inside estimator behavior, not output projection or scoring timestamp shifts.",
        "",
        "Generated files:",
        f"- `{out_dir / 'time_semantics_row_metrics.csv'}`",
        f"- `{out_dir / 'time_semantics_window_summary.csv'}`",
        f"- `{out_dir / 'time_shift_sensitivity.csv'}`",
        f"- `{out_dir / 'time_shift_best_summary.csv'}`",
        f"- `{out_dir / 'time_shift_acceptance_summary.csv'}`",
    ]
    (out_dir / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out-dir", default=str(DEFAULT_OUT))
    args = parser.parse_args()

    out_dir = Path(args.out_dir)
    row_metrics: list[dict[str, object]] = []
    rows_by_run: dict[str, list[dict[str, object]]] = {}
    for spec in RUNS:
        rows = load_run_rows(spec)
        row_metrics.extend(rows)
        rows_by_run[spec.label] = rows
    write_csv(out_dir / "time_semantics_row_metrics.csv", row_metrics)

    window_rows: list[dict[str, object]] = []
    for spec in RUNS:
        rows = rows_by_run[spec.label]
        for label, start, end in WINDOWS:
            window_rows.append(summarize_window(rows, label, start, end))
    write_csv(out_dir / "time_semantics_window_summary.csv", window_rows)

    shift_rows = build_shift_rows(rows_by_run)
    write_csv(out_dir / "time_shift_sensitivity.csv", shift_rows)
    shift_summary = best_shift_summary(shift_rows)
    write_csv(out_dir / "time_shift_best_summary.csv", shift_summary)
    shift_acceptance = build_shift_acceptance_rows(shift_rows)
    write_csv(out_dir / "time_shift_acceptance_summary.csv", shift_acceptance)
    write_report(out_dir, window_rows, shift_summary, shift_acceptance)
    print(f"wrote: {out_dir / 'report.md'}")


if __name__ == "__main__":
    main()
