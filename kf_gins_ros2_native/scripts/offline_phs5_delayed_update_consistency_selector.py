#!/usr/bin/env python3
"""Offline delayed GNSS update-consistency selector replay for PHS5.

This script scores candidate alpha-projection gates using only online-visible
state-update history available before each publish row. Groundtruth is used only
for offline scoring.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence

import offline_phs5_measurement_geometry_selector as mg


BASE = Path("/home/yang/kf_gins_ws/artifacts/manual/short_route_generalization_2026-05-07")
DEFAULT_IN = (
    BASE
    / "phs5_projection_proxy_augmented_after_timing_miss_2026-05-11"
    / "augmented_row_metrics.csv"
)
DEFAULT_OUT = BASE / "phs5_delayed_update_consistency_selector_2026-05-11"
ALPHA = 1.20


RUN_DIRS = {
    "shortgen01_phs5c": Path(
        "/home/yang/kf_gins_ws/artifacts/manual/"
        "manual_shortgen01_s_bend_sitl_home_headless_compare_core8_pairlogger_phs5c_publish_core_stamp_bias_m003_20260510_003839"
    ),
    "shortgen02_phs5b": Path(
        "/home/yang/kf_gins_ws/artifacts/manual/"
        "manual_shortgen02_box_uturn_sitl_home_headless_compare_core8_pairlogger_phs5b_publish_core_stamp_bias_m003_20260510_003144"
    ),
    "shortgen03_phs5a": Path(
        "/home/yang/kf_gins_ws/artifacts/manual/"
        "manual_shortgen03_figure8_sitl_home_headless_compare_core8_pairlogger_phs5a_publish_core_stamp_bias_m003_20260510_002551"
    ),
    "shortgen04_hld1a_phs5": Path(
        "/home/yang/kf_gins_ws/artifacts/manual/"
        "manual_shortgen04_ladder_sitl_home_headless_compare_core8_pairlogger_hld1a_phs5_publish_core_stamp_bias_m003_20260510_005528"
    ),
    "shortgen04_current_agelag_lag0": Path(
        "/home/yang/kf_gins_ws/artifacts/manual/"
        "manual_shortgen04_ladder_sitl_home_headless_compare_core8_pairlogger_phs5_agelagproj_alpha12_protection_20260511_145651"
    ),
    "shortgen04_current_lag0255": Path(
        "/home/yang/kf_gins_ws/artifacts/manual/"
        "manual_shortgen04_ladder_sitl_home_headless_compare_core8_pairlogger_phs5_lag0255_agelagproj_alpha12_protection_20260511_151939"
    ),
    "shortgen11_repeat2": Path(
        "/home/yang/kf_gins_ws/artifacts/manual/"
        "manual_shortgen11_high_clearance_ladder_sitl_home_headless_compare_core8_pairlogger_phs5_repeat2_cleancheck_20260510_155253"
    ),
    "shortgen11_current_agelag": Path(
        "/home/yang/kf_gins_ws/artifacts/manual/"
        "manual_shortgen11_high_clearance_ladder_sitl_home_headless_compare_core8_pairlogger_phs5_agelagproj_alpha12_20260511_144020"
    ),
}

POSITIVE_RUNS = {
    "shortgen01_phs5c",
    "shortgen02_phs5b",
    "shortgen03_phs5a",
    "shortgen04_hld1a_phs5",
    "shortgen04_current_agelag_lag0",
    "shortgen04_current_lag0255",
}
TARGET_RUNS = {"shortgen11_repeat2", "shortgen11_current_agelag"}
TARGET_WINDOWS = {"140_160", "160_180"}

WINDOWS = [
    ("main_40_180", 40.0, 180.0),
    ("60_100", 60.0, 100.0),
    ("100_120", 100.0, 120.0),
    ("120_140", 120.0, 140.0),
    ("140_160", 140.0, 160.0),
    ("160_180", 160.0, 180.0),
]

DELAYS_SEC = (0.0, 1.0, 2.0, 5.0)
HISTORY_SEC = (3.0, 5.0, 10.0, 20.0)


@dataclass(frozen=True)
class Condition:
    feature: str
    op: str
    threshold: float

    def name(self) -> str:
        suffix = f"{abs(self.threshold):.3f}".replace(".", "p")
        if self.threshold < 0.0:
            suffix = "m" + suffix
        return f"{self.feature}_{self.op}_{suffix}"

    def passes(self, row: dict[str, object]) -> bool:
        value = to_float(row.get(self.feature))
        if not finite(value):
            return False
        if self.op == "ge":
            return value >= self.threshold
        if self.op == "le":
            return value <= self.threshold
        raise ValueError(f"unknown op {self.op}")


@dataclass(frozen=True)
class Candidate:
    conditions: tuple[Condition, ...]

    def name(self) -> str:
        return "__".join(condition.name() for condition in self.conditions)

    def active(self, row: dict[str, object]) -> bool:
        return all(condition.passes(row) for condition in self.conditions)


def finite(value: object) -> bool:
    return mg.finite(value)


def to_float(value: object, default: float = math.nan) -> float:
    return mg.to_float(value, default)


def fmt(value: object, digits: int = 4) -> str:
    return mg.fmt(value, digits)


def mean(values: Iterable[float]) -> float:
    return mg.mean(values)


def rmse(values: Iterable[float]) -> float:
    return mg.rmse(values)


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


def markdown_table(headers: list[str], rows: list[list[object]]) -> str:
    return mg.markdown_table(headers, rows)


def norm(east: float, north: float) -> float:
    return math.hypot(east, north) if finite(east) and finite(north) else math.nan


def dot(a_e: float, a_n: float, b_e: float, b_n: float) -> float:
    if not all(finite(value) for value in (a_e, a_n, b_e, b_n)):
        return math.nan
    return a_e * b_e + a_n * b_n


def cosine(a_e: float, a_n: float, b_e: float, b_n: float) -> float:
    a_h = norm(a_e, a_n)
    b_h = norm(b_e, b_n)
    if not finite(a_h) or not finite(b_h) or a_h < 1e-9 or b_h < 1e-9:
        return math.nan
    return max(-1.0, min(1.0, dot(a_e, a_n, b_e, b_n) / (a_h * b_h)))


def reduction_m(vec_e: float, vec_n: float, delta_e: float, delta_n: float) -> float:
    before = norm(vec_e, vec_n)
    after = norm(vec_e - ALPHA * delta_e, vec_n - ALPHA * delta_n)
    if not finite(before) or not finite(after):
        return math.nan
    return before - after


def load_updates(run_label: str) -> tuple[list[dict[str, object]], list[float]]:
    run_dir = RUN_DIRS[run_label]
    path = run_dir / "state_update_debug.csv"
    rows: list[dict[str, object]] = []
    if not path.exists():
        return rows, []
    for raw in read_csv(path):
        if raw.get("event_type") != "gnss_position":
            continue
        if to_float(raw.get("applied"), 0.0) <= 0.5:
            continue
        t = to_float(raw.get("armed_time_sec"))
        if not finite(t):
            continue
        row = {
            "t": t,
            "res_e": to_float(raw.get("gnss_position_residual_e_m")),
            "res_n": to_float(raw.get("gnss_position_residual_n_m")),
            "res_h": to_float(raw.get("gnss_position_residual_h_m")),
            "dx_e": to_float(raw.get("dx_pos_e_m")),
            "dx_n": to_float(raw.get("dx_pos_n_m")),
            "dx_h": to_float(raw.get("dx_pos_h_norm_m")),
            "nis_h": to_float(raw.get("gnss_position_nis_h_2d")),
            "nis_3d": to_float(raw.get("gnss_position_nis_3d")),
            "dx_over_residual_h": to_float(raw.get("dx_pos_h_over_residual_h")),
            "gain_trace": to_float(raw.get("kalman_gain_horizontal_trace")),
            "gain_fro": to_float(raw.get("kalman_gain_horizontal_fro")),
            "accepted": to_float(raw.get("gnss_position_update_accepted")),
            "cov_pos_trace_before": (
                to_float(raw.get("cov_pos_n_before_m2")) + to_float(raw.get("cov_pos_e_before_m2"))
            ),
            "cov_pos_trace_after": (
                to_float(raw.get("cov_pos_n_after_m2")) + to_float(raw.get("cov_pos_e_after_m2"))
            ),
        }
        row["cov_pos_trace_shrink"] = (
            to_float(row["cov_pos_trace_before"]) - to_float(row["cov_pos_trace_after"])
        )
        rows.append(row)
    rows.sort(key=lambda row: to_float(row.get("t")))
    return rows, [to_float(row.get("t")) for row in rows]


def history_rows(
    rows: Sequence[dict[str, object]],
    times: Sequence[float],
    t: float,
    delay_sec: float,
    history_sec: float,
) -> list[dict[str, object]]:
    if not rows or not finite(t):
        return []
    cutoff = t - delay_sec
    start = cutoff - history_sec
    lo = bisect.bisect_left(times, start)
    hi = bisect.bisect_right(times, cutoff)
    return list(rows[lo:hi])


def add_history_features(
    row: dict[str, object],
    updates: Sequence[dict[str, object]],
    update_times: Sequence[float],
) -> None:
    t = to_float(row.get("t"))
    delta_e = to_float(row.get("px4x")) - to_float(row.get("rawx"))
    delta_n = to_float(row.get("px4y")) - to_float(row.get("rawy"))
    for delay in DELAYS_SEC:
        for history in HISTORY_SEC:
            prior = history_rows(updates, update_times, t, delay, history)
            prefix = f"d{delay:g}_w{history:g}"
            if not prior:
                for suffix in (
                    "update_count",
                    "res_h_mean",
                    "res_h_max",
                    "dx_h_mean",
                    "nis_h_mean",
                    "nis_h_max",
                    "dx_over_residual_h_mean",
                    "gain_trace_mean",
                    "cov_trace_shrink_mean",
                    "resid_proj_cos",
                    "dx_proj_cos",
                    "resid_reduction_m",
                    "dx_reduction_m",
                    "resid_reduction_frac",
                    "dx_reduction_frac",
                ):
                    row[f"{prefix}_{suffix}"] = math.nan if suffix != "update_count" else 0
                continue

            res_e_values = [to_float(item.get("res_e")) for item in prior]
            res_n_values = [to_float(item.get("res_n")) for item in prior]
            dx_e_values = [to_float(item.get("dx_e")) for item in prior]
            dx_n_values = [to_float(item.get("dx_n")) for item in prior]
            res_e_mean = mean(res_e_values)
            res_n_mean = mean(res_n_values)
            dx_e_mean = mean(dx_e_values)
            dx_n_mean = mean(dx_n_values)
            resid_reductions = [
                reduction_m(to_float(item.get("res_e")), to_float(item.get("res_n")), delta_e, delta_n)
                for item in prior
            ]
            dx_reductions = [
                reduction_m(to_float(item.get("dx_e")), to_float(item.get("dx_n")), delta_e, delta_n)
                for item in prior
            ]
            row[f"{prefix}_update_count"] = len(prior)
            row[f"{prefix}_res_h_mean"] = mean(to_float(item.get("res_h")) for item in prior)
            row[f"{prefix}_res_h_max"] = max(
                (to_float(item.get("res_h")) for item in prior if finite(item.get("res_h"))),
                default=math.nan,
            )
            row[f"{prefix}_dx_h_mean"] = mean(to_float(item.get("dx_h")) for item in prior)
            row[f"{prefix}_nis_h_mean"] = mean(to_float(item.get("nis_h")) for item in prior)
            row[f"{prefix}_nis_h_max"] = max(
                (to_float(item.get("nis_h")) for item in prior if finite(item.get("nis_h"))),
                default=math.nan,
            )
            row[f"{prefix}_dx_over_residual_h_mean"] = mean(
                to_float(item.get("dx_over_residual_h")) for item in prior
            )
            row[f"{prefix}_gain_trace_mean"] = mean(to_float(item.get("gain_trace")) for item in prior)
            row[f"{prefix}_cov_trace_shrink_mean"] = mean(
                to_float(item.get("cov_pos_trace_shrink")) for item in prior
            )
            row[f"{prefix}_resid_proj_cos"] = cosine(delta_e, delta_n, res_e_mean, res_n_mean)
            row[f"{prefix}_dx_proj_cos"] = cosine(delta_e, delta_n, dx_e_mean, dx_n_mean)
            row[f"{prefix}_resid_reduction_m"] = mean(resid_reductions)
            row[f"{prefix}_dx_reduction_m"] = mean(dx_reductions)
            row[f"{prefix}_resid_reduction_frac"] = mean(
                1.0 if reduction > 0.0 else 0.0 for reduction in resid_reductions if finite(reduction)
            )
            row[f"{prefix}_dx_reduction_frac"] = mean(
                1.0 if reduction > 0.0 else 0.0 for reduction in dx_reductions if finite(reduction)
            )


def load_rows(input_path: Path) -> list[dict[str, object]]:
    updates_by_run = {label: load_updates(label) for label in RUN_DIRS}
    rows: list[dict[str, object]] = []
    for raw in read_csv(input_path):
        run = raw["run"]
        if run not in RUN_DIRS:
            continue
        row: dict[str, object] = dict(raw)
        for key, value in raw.items():
            if key not in {"run", "group", "source"}:
                row[key] = to_float(value)
        rawx = to_float(row.get("rawx"))
        rawy = to_float(row.get("rawy"))
        px4x = to_float(row.get("px4x"))
        px4y = to_float(row.get("px4y"))
        gtx = to_float(row.get("gtx"))
        gty = to_float(row.get("gty"))
        alpha_x = rawx + ALPHA * (px4x - rawx)
        alpha_y = rawy + ALPHA * (px4y - rawy)
        row["alpha_x"] = alpha_x
        row["alpha_y"] = alpha_y
        row["alpha_error_m"] = norm(alpha_x - gtx, alpha_y - gty)
        row["alpha_minus_raw_error_m"] = to_float(row.get("alpha_error_m")) - to_float(
            row.get("raw_error_m")
        )
        row["alpha_minus_ekf2_error_m"] = to_float(row.get("alpha_error_m")) - to_float(
            row.get("ekf2_error_m")
        )
        row["projection_delta_e_m"] = px4x - rawx
        row["projection_delta_n_m"] = px4y - rawy
        updates, update_times = updates_by_run[run]
        add_history_features(row, updates, update_times)
        rows.append(row)
    return rows


WindowIndex = dict[tuple[str, str], list[dict[str, object]]]


def rows_in_window(rows: Sequence[dict[str, object]], run: str, start: float, end: float) -> list[dict[str, object]]:
    return [
        row
        for row in rows
        if row.get("run") == run and start <= to_float(row.get("t")) < end
    ]


def build_window_index(rows: Sequence[dict[str, object]]) -> WindowIndex:
    index: WindowIndex = {}
    for run in sorted(RUN_DIRS):
        run_rows = [row for row in rows if row.get("run") == run]
        for window, start, end in WINDOWS:
            index[(run, window)] = [
                row for row in run_rows if start <= to_float(row.get("t")) < end
            ]
    return index


def window_metric(
    window_index: WindowIndex,
    candidate: Candidate,
    run: str,
    window: str,
) -> dict[str, object]:
    subset = window_index[(run, window)]
    active_flags = [candidate.active(row) for row in subset]
    active_rows = [row for row, active in zip(subset, active_flags) if active]
    selected_errors = [
        to_float(row.get("alpha_error_m")) if active else to_float(row.get("raw_error_m"))
        for row, active in zip(subset, active_flags)
    ]
    raw = rmse(to_float(row.get("raw_error_m")) for row in subset)
    alpha = rmse(to_float(row.get("alpha_error_m")) for row in subset)
    selected = rmse(selected_errors)
    ekf2 = rmse(to_float(row.get("ekf2_error_m")) for row in subset)
    return {
        "candidate": candidate.name(),
        "run": run,
        "window": window,
        "rows": len(subset),
        "active_frac": len(active_rows) / len(subset) if subset else math.nan,
        "raw_rmse_m": raw,
        "alpha_all_rmse_m": alpha,
        "selected_rmse_m": selected,
        "ekf2_rmse_m": ekf2,
        "selected_minus_raw_m": selected - raw if finite(selected) and finite(raw) else math.nan,
        "selected_minus_ekf2_m": selected - ekf2 if finite(selected) and finite(ekf2) else math.nan,
        "alpha_all_minus_raw_m": alpha - raw if finite(alpha) and finite(raw) else math.nan,
    }


def make_candidates() -> list[Candidate]:
    candidates: list[Candidate] = []
    seen: set[str] = set()
    for delay in DELAYS_SEC:
        for history in HISTORY_SEC:
            prefix = f"d{delay:g}_w{history:g}"
            align_features = [
                f"{prefix}_resid_proj_cos",
                f"{prefix}_dx_proj_cos",
            ]
            reduction_features = [
                f"{prefix}_resid_reduction_m",
                f"{prefix}_dx_reduction_m",
            ]
            frac_features = [
                f"{prefix}_resid_reduction_frac",
                f"{prefix}_dx_reduction_frac",
            ]
            pressure_features = [
                f"{prefix}_res_h_mean",
                f"{prefix}_dx_h_mean",
                f"{prefix}_nis_h_mean",
            ]

            base_min_updates = Condition(f"{prefix}_update_count", "ge", 2.0)
            delta_min = [
                Condition("projection_delta_h_m", "ge", 0.15),
                Condition("projection_delta_h_m", "ge", 0.25),
                Condition("projection_delta_h_m", "ge", 0.35),
            ]
            for align_feature in align_features:
                for align_th in (0.50, 0.70, 0.85):
                    conds = (base_min_updates, Condition(align_feature, "ge", align_th))
                    add_candidate(candidates, seen, conds)
                    for delta_condition in delta_min:
                        add_candidate(candidates, seen, conds + (delta_condition,))
                    for reduction_feature in reduction_features:
                        for reduction_th in (0.03, 0.05):
                            add_candidate(
                                candidates,
                                seen,
                                conds + (Condition(reduction_feature, "ge", reduction_th),),
                            )

            for reduction_feature in reduction_features:
                for reduction_th in (0.03, 0.05, 0.10):
                    conds = (base_min_updates, Condition(reduction_feature, "ge", reduction_th))
                    add_candidate(candidates, seen, conds)
                    for delta_condition in delta_min:
                        add_candidate(candidates, seen, conds + (delta_condition,))

            for frac_feature in frac_features:
                for frac_th in (0.75, 0.90):
                    conds = (base_min_updates, Condition(frac_feature, "ge", frac_th))
                    matching_reduction = (
                        f"{prefix}_resid_reduction_m"
                        if "resid" in frac_feature
                        else f"{prefix}_dx_reduction_m"
                    )
                    for reduction_th in (0.03, 0.05):
                        add_candidate(
                            candidates,
                            seen,
                            conds + (Condition(matching_reduction, "ge", reduction_th),),
                        )
    return candidates


def add_candidate(candidates: list[Candidate], seen: set[str], conditions: tuple[Condition, ...]) -> None:
    candidate = Candidate(conditions)
    name = candidate.name()
    if name in seen:
        return
    seen.add(name)
    candidates.append(candidate)


def summarize_candidate(window_index: WindowIndex, candidate: Candidate) -> dict[str, object]:
    metrics: dict[tuple[str, str], dict[str, object]] = {}
    for run in sorted(POSITIVE_RUNS):
        metrics[(run, "main_40_180")] = window_metric(window_index, candidate, run, "main_40_180")
    for run in sorted(TARGET_RUNS):
        for window in sorted(TARGET_WINDOWS):
            metrics[(run, window)] = window_metric(window_index, candidate, run, window)

    positive_regressions = [
        to_float(metrics[(run, "main_40_180")].get("selected_minus_raw_m"))
        for run in POSITIVE_RUNS
        if finite(metrics[(run, "main_40_180")].get("selected_minus_raw_m"))
    ]
    positive_active = [
        to_float(metrics[(run, "main_40_180")].get("active_frac"))
        for run in POSITIVE_RUNS
        if finite(metrics[(run, "main_40_180")].get("active_frac"))
    ]
    repeat2_target = [metrics[("shortgen11_repeat2", window)] for window in TARGET_WINDOWS]
    current_target = [metrics[("shortgen11_current_agelag", window)] for window in TARGET_WINDOWS]
    repeat2_improve = [
        -to_float(item.get("selected_minus_raw_m"))
        for item in repeat2_target
        if finite(item.get("selected_minus_raw_m"))
    ]
    current_improve = [
        -to_float(item.get("selected_minus_raw_m"))
        for item in current_target
        if finite(item.get("selected_minus_raw_m"))
    ]
    repeat2_worst_ekf2 = max(
        (to_float(item.get("selected_minus_ekf2_m")) for item in repeat2_target if finite(item.get("selected_minus_ekf2_m"))),
        default=math.nan,
    )
    current_worst_ekf2 = max(
        (to_float(item.get("selected_minus_ekf2_m")) for item in current_target if finite(item.get("selected_minus_ekf2_m"))),
        default=math.nan,
    )
    current_min_active = min(
        (to_float(item.get("active_frac")) for item in current_target if finite(item.get("active_frac"))),
        default=math.nan,
    )
    repeat2_min_active = min(
        (to_float(item.get("active_frac")) for item in repeat2_target if finite(item.get("active_frac"))),
        default=math.nan,
    )
    positive_max_regress = max(positive_regressions, default=math.nan)
    repeat2_improve_sum = sum(improve for improve in repeat2_improve if improve > 0.0)
    current_improve_sum = sum(improve for improve in current_improve if improve > 0.0)
    strict = (
        finite(positive_max_regress)
        and positive_max_regress <= 0.01
        and repeat2_improve_sum >= 0.08
        and current_improve_sum >= 0.08
        and finite(repeat2_worst_ekf2)
        and repeat2_worst_ekf2 <= 0.03
        and finite(current_worst_ekf2)
        and current_worst_ekf2 <= 0.03
        and finite(repeat2_min_active)
        and repeat2_min_active >= 0.25
        and finite(current_min_active)
        and current_min_active >= 0.25
    )
    return {
        "candidate": candidate.name(),
        "conditions": len(candidate.conditions),
        "positive_max_regress_m": positive_max_regress,
        "positive_max_active_frac": max(positive_active, default=math.nan),
        "repeat2_target_improve_sum_m": repeat2_improve_sum,
        "repeat2_worst_candidate_minus_ekf2_m": repeat2_worst_ekf2,
        "repeat2_min_target_active_frac": repeat2_min_active,
        "current_shortgen11_target_improve_sum_m": current_improve_sum,
        "current_worst_candidate_minus_ekf2_m": current_worst_ekf2,
        "current_min_target_active_frac": current_min_active,
        "strict_delayed_update_pass": 1 if strict else 0,
    }


def sort_key(row: dict[str, object]) -> tuple[float, float, float, float, float, float]:
    strict = -to_float(row.get("strict_delayed_update_pass"), 0.0)
    pos_reg = to_float(row.get("positive_max_regress_m"), 1e9)
    repeat2_worst = to_float(row.get("repeat2_worst_candidate_minus_ekf2_m"), 1e9)
    current_worst = to_float(row.get("current_worst_candidate_minus_ekf2_m"), 1e9)
    target_improve = -(
        to_float(row.get("repeat2_target_improve_sum_m"), 0.0)
        + to_float(row.get("current_shortgen11_target_improve_sum_m"), 0.0)
    )
    current_active = -to_float(row.get("current_min_target_active_frac"), 0.0)
    return strict, pos_reg, repeat2_worst, current_worst, target_improve, current_active


def candidate_metrics_for_report(
    window_index: WindowIndex, candidates: Sequence[Candidate]
) -> list[dict[str, object]]:
    metric_rows: list[dict[str, object]] = []
    for candidate in candidates:
        for run in [
            "shortgen01_phs5c",
            "shortgen02_phs5b",
            "shortgen03_phs5a",
            "shortgen04_hld1a_phs5",
            "shortgen04_current_agelag_lag0",
            "shortgen04_current_lag0255",
            "shortgen11_repeat2",
            "shortgen11_current_agelag",
        ]:
            for window, _start, _end in WINDOWS:
                if window == "main_40_180" or (run in TARGET_RUNS and window in TARGET_WINDOWS):
                    metric_rows.append(window_metric(window_index, candidate, run, window))
    return metric_rows


def write_report(
    out_dir: Path,
    rows: Sequence[dict[str, object]],
    candidate_rows: Sequence[dict[str, object]],
    selected_metric_rows: Sequence[dict[str, object]],
    input_path: Path,
) -> None:
    strict_count = sum(to_float(row.get("strict_delayed_update_pass"), 0.0) > 0.5 for row in candidate_rows)
    protected_count = sum(
        finite(row.get("positive_max_regress_m")) and to_float(row.get("positive_max_regress_m")) <= 0.01
        for row in candidate_rows
    )
    protected_top = list(candidate_rows[:12])
    target_tradeoffs = sorted(
        candidate_rows,
        key=lambda row: -(
            to_float(row.get("repeat2_target_improve_sum_m"), 0.0)
            + to_float(row.get("current_shortgen11_target_improve_sum_m"), 0.0)
        ),
    )[:12]
    protected_table = [
        [
            row["candidate"],
            row["conditions"],
            fmt(row["positive_max_regress_m"]),
            fmt(row["repeat2_worst_candidate_minus_ekf2_m"]),
            fmt(row["repeat2_target_improve_sum_m"]),
            fmt(row["current_shortgen11_target_improve_sum_m"]),
            fmt(row["current_min_target_active_frac"], 3),
            row["strict_delayed_update_pass"],
        ]
        for row in protected_top
    ]
    tradeoff_table = [
        [
            row["candidate"],
            row["conditions"],
            fmt(row["positive_max_regress_m"]),
            fmt(row["repeat2_target_improve_sum_m"]),
            fmt(row["repeat2_worst_candidate_minus_ekf2_m"]),
            fmt(row["repeat2_min_target_active_frac"], 3),
            fmt(row["current_shortgen11_target_improve_sum_m"]),
            fmt(row["current_worst_candidate_minus_ekf2_m"]),
            fmt(row["current_min_target_active_frac"], 3),
        ]
        for row in target_tradeoffs
    ]
    metric_table = [
        [
            row["candidate"],
            row["run"],
            row["window"],
            fmt(row["active_frac"], 3),
            fmt(row["raw_rmse_m"]),
            fmt(row["selected_rmse_m"]),
            fmt(row["ekf2_rmse_m"]),
            fmt(row["selected_minus_raw_m"]),
            fmt(row["selected_minus_ekf2_m"]),
        ]
        for row in selected_metric_rows[:40]
    ]
    report = [
        "# PHS5 delayed GNSS update-consistency selector replay",
        "",
        "Date: 2026-05-11",
        "",
        "## Scope",
        "",
        "Offline-only replay over existing artifacts. Candidate activation uses only prior `state_update_debug.csv` GNSS position update history joined to the already exported augmented projection rows. Groundtruth is used only for scoring.",
        "",
        "No PX4/Gazebo/MAVROS/QGC/RViz2/PlotJuggler run is started by this script.",
        "",
        "## Inputs",
        "",
        f"- augmented rows: `{input_path}`",
        "- per-run `state_update_debug.csv` logs from the same eight runs",
        f"- alpha action: `raw + {ALPHA:.2f} * (PX4-sphere - raw)`",
        "",
        "Delayed feature families:",
        "",
        "- history windows: `3, 5, 10, 20 s`",
        "- delays: `0, 1, 2, 5 s` before the publish row",
        "- GNSS residual vector vs projection delta alignment",
        "- GNSS update displacement vector vs projection delta alignment",
        "- expected residual/update displacement reduction after alpha projection",
        "- residual, NIS, gain, and covariance-shrink pressure summaries",
        "",
        "## Candidate Summary",
        "",
        f"- rows scored: `{len(rows)}`",
        f"- candidates tested: `{len(candidate_rows)}`",
        f"- strict delayed-update pass count: `{strict_count}`",
        f"- protected candidate count (`positive_max_regress_m <= 0.01`): `{protected_count}`",
        "",
        "Protected-first view:",
        "",
        markdown_table(
            [
                "candidate",
                "conds",
                "pos_reg",
                "repeat2 worst-EKF2",
                "repeat2 improve",
                "current s11 improve",
                "current min active",
                "strict",
            ],
            protected_table,
        ),
        "",
        "Target-improvement tradeoff view:",
        "",
        markdown_table(
            [
                "candidate",
                "conds",
                "pos_reg",
                "repeat2 improve",
                "repeat2 worst-EKF2",
                "repeat2 min active",
                "current improve",
                "current worst-EKF2",
                "current min active",
            ],
            tradeoff_table,
        ),
        "",
        "## Selected Window Metrics",
        "",
        markdown_table(
            [
                "candidate",
                "run",
                "window",
                "active",
                "raw",
                "selected",
                "EKF2",
                "sel-raw",
                "sel-EKF2",
            ],
            metric_table,
        ),
        "",
        "## Readout",
        "",
    ]
    if strict_count:
        report.extend(
            [
                "At least one delayed update-consistency candidate passed the strict offline bar. This is still not online validation; the next step would be one existing-route opt-in mechanism diagnostic, not a new holdout.",
                "",
            ]
        )
    else:
        report.extend(
            [
                "No delayed update-consistency candidate passed the strict bar in this grid. The best protected candidates do not repair repeat2, while the target-improving candidates introduce positive/control regression and still leave at least one shortgen11 target window above EKF2.",
                "",
                "This weakens the hypothesis that a simple prior-GNSS-residual/update-consistency gate can solve the projection selector problem by itself.",
                "",
            ]
        )
    report.extend(
        [
            "Generated files:",
            "",
            f"- `{out_dir / 'row_metrics.csv'}`",
            f"- `{out_dir / 'candidate_summary.csv'}`",
            f"- `{out_dir / 'selected_window_metrics.csv'}`",
        ]
    )
    (out_dir / "report.md").write_text("\n".join(report) + "\n")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, default=DEFAULT_IN)
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUT)
    args = parser.parse_args()

    rows = load_rows(args.input)
    window_index = build_window_index(rows)
    candidates = make_candidates()
    candidate_rows = [summarize_candidate(window_index, candidate) for candidate in candidates]
    candidate_rows.sort(key=sort_key)
    target_top_rows = sorted(
        candidate_rows,
        key=lambda row: -(
            to_float(row.get("repeat2_target_improve_sum_m"), 0.0)
            + to_float(row.get("current_shortgen11_target_improve_sum_m"), 0.0)
        ),
    )[:20]
    top_names = [str(row["candidate"]) for row in candidate_rows[:20]]
    for row in target_top_rows:
        name = str(row["candidate"])
        if name not in top_names:
            top_names.append(name)
    top_rank = {name: idx for idx, name in enumerate(top_names)}
    selected_candidates = [candidate for candidate in candidates if candidate.name() in top_rank]
    selected_candidates.sort(key=lambda candidate: top_rank[candidate.name()])
    metric_rows = candidate_metrics_for_report(window_index, selected_candidates)

    args.out_dir.mkdir(parents=True, exist_ok=True)
    write_csv(args.out_dir / "row_metrics.csv", rows)
    write_csv(args.out_dir / "candidate_summary.csv", candidate_rows)
    write_csv(args.out_dir / "selected_window_metrics.csv", metric_rows)
    write_report(args.out_dir, rows, candidate_rows, metric_rows, args.input)

    strict_count = sum(to_float(row.get("strict_delayed_update_pass"), 0.0) > 0.5 for row in candidate_rows)
    print(f"rows={len(rows)} candidates={len(candidate_rows)} strict={strict_count}")
    print(args.out_dir / "report.md")


if __name__ == "__main__":
    main()
