#!/usr/bin/env python3
"""Offline finite-difference kinematic drift atlas for PHS5 routes."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from typing import Iterable, Sequence

import offline_phs5_measurement_geometry_selector as mg
import offline_phs5_selector_replay_after_lag_failure as replay


BASE = Path("/home/yang/kf_gins_ws/artifacts/manual/short_route_generalization_2026-05-07")
DEFAULT_WINDOWS = BASE / "phs5_update_propagation_timeline_2026-05-10" / "timeline_window_summary.csv"
DEFAULT_OUT = BASE / "phs5_kinematic_drift_atlas_2026-05-10"
TARGET_RUNS = {spec.label for spec in replay.RUNS if spec.group == "target"}
WINDOWS = [
    ("120_140", 120.0, 140.0),
    ("140_160", 140.0, 160.0),
    ("160_180", 160.0, 180.0),
    ("main_40_180", 40.0, 180.0),
]


def finite(value: object) -> bool:
    return mg.finite(value)


def to_float(value: object, default: float = math.nan) -> float:
    return mg.to_float(value, default)


def fmt(value: object, digits: int = 4) -> str:
    return mg.fmt(value, digits)


def mean(values: Iterable[float]) -> float:
    return mg.mean(values)


def rmse(values: Iterable[float]) -> float:
    vals = [value for value in values if finite(value)]
    return math.sqrt(sum(value * value for value in vals) / len(vals)) if vals else math.nan


def percentile(values: Iterable[float], p: float) -> float:
    vals = sorted(value for value in values if finite(value))
    if not vals:
        return math.nan
    rank = (len(vals) - 1) * p / 100.0
    lo = int(math.floor(rank))
    hi = int(math.ceil(rank))
    if lo == hi:
        return vals[lo]
    frac = rank - lo
    return vals[lo] * (1.0 - frac) + vals[hi] * frac


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
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


def norm(e: float, n: float) -> float:
    return math.hypot(e, n) if finite(e) and finite(n) else math.nan


def unit(e: float, n: float) -> tuple[float, float]:
    h = norm(e, n)
    if not finite(h) or h < 1e-9:
        return math.nan, math.nan
    return e / h, n / h


def dot(a_e: float, a_n: float, b_e: float, b_n: float) -> float:
    if not all(finite(value) for value in (a_e, a_n, b_e, b_n)):
        return math.nan
    return a_e * b_e + a_n * b_n


def cross(a_e: float, a_n: float, b_e: float, b_n: float) -> float:
    if not all(finite(value) for value in (a_e, a_n, b_e, b_n)):
        return math.nan
    return a_e * b_n - a_n * b_e


def window_label(t: float) -> str:
    for label, start, end in WINDOWS:
        if start <= t < end:
            return label
    return "other"


def load_bad_windows(path: Path) -> set[tuple[str, str]]:
    bad: set[tuple[str, str]] = set()
    for raw in read_csv(path):
        run = raw.get("run", "")
        window = raw.get("window", "")
        if run in TARGET_RUNS and window in {"140_160", "160_180"} and to_float(raw.get("iekf_minus_ekf2_rmse_m")) > 0.05:
            bad.add((run, window))
    bad.add(("shortgen11_repeat2", "120_140"))
    return bad


def load_joined(spec: replay.RunSpec) -> list[dict[str, object]]:
    path = mg.mode_dir(spec.path) / "raw_wgs84_enu" / "groundtruth_joined.csv"
    rows: list[dict[str, object]] = []
    for raw in read_csv(path):
        row: dict[str, object] = dict(raw)
        for key, value in raw.items():
            if key not in {"run", "mavros_mode"}:
                row[key] = to_float(value)
        row["run_label"] = spec.label
        row["group"] = spec.group
        rows.append(row)
    rows.sort(key=lambda row: to_float(row.get("pair_ros_time_sec")))
    return rows


def build_samples(bad_windows: set[tuple[str, str]]) -> list[dict[str, object]]:
    samples: list[dict[str, object]] = []
    for spec in replay.RUNS:
        rows = [row for row in load_joined(spec) if 35.0 <= to_float(row.get("time_since_arm_sec")) < 185.0]
        for current, nxt in zip(rows, rows[1:]):
            t = to_float(current.get("time_since_arm_sec"))
            dt = to_float(nxt.get("pair_ros_time_sec")) - to_float(current.get("pair_ros_time_sec"))
            if not finite(t) or not finite(dt) or not (0.02 <= dt <= 0.8):
                continue
            label = window_label(t)
            if label == "other":
                continue
            gt_e = to_float(current.get("gt_x_m"))
            gt_n = to_float(current.get("gt_y_m"))
            iekf_e = to_float(current.get("iekf_normalized_aligned_x_m"))
            iekf_n = to_float(current.get("iekf_normalized_aligned_y_m"))
            ekf2_e = to_float(current.get("ekf2_aligned_x_m"))
            ekf2_n = to_float(current.get("ekf2_aligned_y_m"))
            gt_ve = (to_float(nxt.get("gt_x_m")) - gt_e) / dt
            gt_vn = (to_float(nxt.get("gt_y_m")) - gt_n) / dt
            iekf_ve = (to_float(nxt.get("iekf_normalized_aligned_x_m")) - iekf_e) / dt
            iekf_vn = (to_float(nxt.get("iekf_normalized_aligned_y_m")) - iekf_n) / dt
            ekf2_ve = (to_float(nxt.get("ekf2_aligned_x_m")) - ekf2_e) / dt
            ekf2_vn = (to_float(nxt.get("ekf2_aligned_y_m")) - ekf2_n) / dt
            err_e = iekf_e - gt_e
            err_n = iekf_n - gt_n
            ekf2_err_e = ekf2_e - gt_e
            ekf2_err_n = ekf2_n - gt_n
            u_e, u_n = unit(err_e, err_n)
            eu_e, eu_n = unit(ekf2_err_e, ekf2_err_n)
            iekf_verr_e = iekf_ve - gt_ve
            iekf_verr_n = iekf_vn - gt_vn
            ekf2_verr_e = ekf2_ve - gt_ve
            ekf2_verr_n = ekf2_vn - gt_vn
            err_h = norm(err_e, err_n)
            ekf2_err_h = norm(ekf2_err_e, ekf2_err_n)
            next_err_h = to_float(nxt.get("iekf_error_xy_m"))
            next_ekf2_err_h = to_float(nxt.get("ekf2_error_xy_m"))
            radial_rate = (next_err_h - err_h) / dt if finite(next_err_h) and finite(err_h) else math.nan
            ekf2_radial_rate = (next_ekf2_err_h - ekf2_err_h) / dt if finite(next_ekf2_err_h) and finite(ekf2_err_h) else math.nan
            samples.append(
                {
                    "run": spec.label,
                    "group": spec.group,
                    "window": label,
                    "time_since_arm_sec": t,
                    "dt_sec": dt,
                    "target_focus": int((spec.label, label) in bad_windows),
                    "target_focus_outward": int((spec.label, label) in bad_windows and finite(radial_rate) and radial_rate > 0.0),
                    "positive_main": int(spec.group == "positive" and 40.0 <= t < 180.0),
                    "positive_main_outward": int(spec.group == "positive" and 40.0 <= t < 180.0 and finite(radial_rate) and radial_rate > 0.0),
                    "gt_speed_mps": norm(gt_ve, gt_vn),
                    "iekf_speed_mps": norm(iekf_ve, iekf_vn),
                    "ekf2_speed_mps": norm(ekf2_ve, ekf2_vn),
                    "iekf_error_h_m": err_h,
                    "ekf2_error_h_m": ekf2_err_h,
                    "iekf_minus_ekf2_error_m": err_h - ekf2_err_h,
                    "iekf_error_radial_rate_mps": radial_rate,
                    "ekf2_error_radial_rate_mps": ekf2_radial_rate,
                    "iekf_minus_ekf2_radial_rate_mps": radial_rate - ekf2_radial_rate if finite(radial_rate) and finite(ekf2_radial_rate) else math.nan,
                    "iekf_velocity_error_h_mps": norm(iekf_verr_e, iekf_verr_n),
                    "ekf2_velocity_error_h_mps": norm(ekf2_verr_e, ekf2_verr_n),
                    "iekf_velocity_error_radial_mps": dot(iekf_verr_e, iekf_verr_n, u_e, u_n),
                    "iekf_velocity_error_cross_mps": cross(iekf_verr_e, iekf_verr_n, u_e, u_n),
                    "ekf2_velocity_error_radial_mps": dot(ekf2_verr_e, ekf2_verr_n, eu_e, eu_n),
                    "ekf2_velocity_error_cross_mps": cross(ekf2_verr_e, ekf2_verr_n, eu_e, eu_n),
                    "iekf_minus_ekf2_velocity_error_h_mps": norm(iekf_verr_e, iekf_verr_n) - norm(ekf2_verr_e, ekf2_verr_n),
                    "turning_now": to_float(current.get("turning_now")),
                    "post_turn_context": to_float(current.get("post_turn_context")),
                    "armed_cruise_context": to_float(current.get("armed_cruise_context")),
                    "gyro_deg_s": abs(to_float(current.get("gyro_deg_s"))),
                    "source_yaw_rate_deg_s": abs(to_float(current.get("source_yaw_rate_deg_s"))),
                    "horizontal_speed_mps": to_float(current.get("horizontal_speed_mps")),
                }
            )
    return samples


def summarize(rows: Sequence[dict[str, object]], name: str) -> dict[str, object]:
    return {
        "group": name,
        "rows": len(rows),
        "iekf_error_mean_m": mean(to_float(row.get("iekf_error_h_m")) for row in rows),
        "ekf2_error_mean_m": mean(to_float(row.get("ekf2_error_h_m")) for row in rows),
        "iekf_minus_ekf2_mean_m": mean(to_float(row.get("iekf_minus_ekf2_error_m")) for row in rows),
        "iekf_radial_rate_mean_mps": mean(to_float(row.get("iekf_error_radial_rate_mps")) for row in rows),
        "iekf_radial_rate_p90_mps": percentile([to_float(row.get("iekf_error_radial_rate_mps")) for row in rows], 90.0),
        "iekf_radial_outward_frac": mean(1.0 if to_float(row.get("iekf_error_radial_rate_mps")) > 0.0 else 0.0 for row in rows),
        "ekf2_radial_rate_mean_mps": mean(to_float(row.get("ekf2_error_radial_rate_mps")) for row in rows),
        "iekf_minus_ekf2_radial_rate_mean_mps": mean(to_float(row.get("iekf_minus_ekf2_radial_rate_mps")) for row in rows),
        "iekf_velocity_error_h_mean_mps": mean(to_float(row.get("iekf_velocity_error_h_mps")) for row in rows),
        "ekf2_velocity_error_h_mean_mps": mean(to_float(row.get("ekf2_velocity_error_h_mps")) for row in rows),
        "iekf_velocity_radial_mean_mps": mean(to_float(row.get("iekf_velocity_error_radial_mps")) for row in rows),
        "gt_speed_mean_mps": mean(to_float(row.get("gt_speed_mps")) for row in rows),
        "gyro_mean_deg_s": mean(to_float(row.get("gyro_deg_s")) for row in rows),
        "turning_frac": mean(to_float(row.get("turning_now")) for row in rows),
        "post_turn_frac": mean(to_float(row.get("post_turn_context")) for row in rows),
        "armed_cruise_frac": mean(to_float(row.get("armed_cruise_context")) for row in rows),
    }


def build_summaries(samples: Sequence[dict[str, object]]) -> list[dict[str, object]]:
    groups = [
        ("target_focus", lambda row: to_float(row.get("target_focus")) > 0.5),
        ("target_focus_outward", lambda row: to_float(row.get("target_focus_outward")) > 0.5),
        ("positive_main", lambda row: to_float(row.get("positive_main")) > 0.5),
        ("positive_main_outward", lambda row: to_float(row.get("positive_main_outward")) > 0.5),
    ]
    rows = [summarize([row for row in samples if pred(row)], name) for name, pred in groups]
    for run in ["shortgen11_repeat2", "shortgen04_hld1a_phs5"]:
        for window in ["120_140", "140_160", "160_180"]:
            rows.append(summarize([row for row in samples if row.get("run") == run and row.get("window") == window], f"{run}_{window}"))
    return rows


def feature_separation(samples: Sequence[dict[str, object]]) -> list[dict[str, object]]:
    target = [row for row in samples if to_float(row.get("target_focus_outward")) > 0.5]
    positive = [row for row in samples if to_float(row.get("positive_main_outward")) > 0.5]
    features = [
        "gt_speed_mps",
        "gyro_deg_s",
        "turning_now",
        "post_turn_context",
        "armed_cruise_context",
        "iekf_error_h_m",
        "ekf2_error_h_m",
        "iekf_minus_ekf2_error_m",
        "iekf_error_radial_rate_mps",
        "ekf2_error_radial_rate_mps",
        "iekf_minus_ekf2_radial_rate_mps",
        "iekf_velocity_error_h_mps",
        "ekf2_velocity_error_h_mps",
        "iekf_velocity_error_radial_mps",
    ]
    out: list[dict[str, object]] = []
    for field in features:
        target_vals = [to_float(row.get(field)) for row in target]
        positive_vals = [to_float(row.get(field)) for row in positive]
        target_mean = mean(target_vals)
        positive_mean = mean(positive_vals)
        target_var = mean((value - target_mean) ** 2 for value in target_vals if finite(value) and finite(target_mean))
        positive_var = mean((value - positive_mean) ** 2 for value in positive_vals if finite(value) and finite(positive_mean))
        pooled = math.sqrt(target_var + positive_var) if finite(target_var) and finite(positive_var) else math.nan
        out.append(
            {
                "feature": field,
                "target_count": len([value for value in target_vals if finite(value)]),
                "positive_count": len([value for value in positive_vals if finite(value)]),
                "target_mean": target_mean,
                "positive_mean": positive_mean,
                "diff_target_minus_positive": target_mean - positive_mean,
                "separation_score": abs(target_mean - positive_mean) / pooled if finite(pooled) and pooled > 1e-9 else math.nan,
            }
        )
    out.sort(key=lambda row: to_float(row.get("separation_score"), -1.0), reverse=True)
    return out


def table_summary(rows: Sequence[dict[str, object]]) -> list[list[object]]:
    return [
        [
            row["group"],
            row["rows"],
            fmt(row["iekf_minus_ekf2_mean_m"]),
            fmt(row["iekf_radial_rate_mean_mps"]),
            fmt(row["ekf2_radial_rate_mean_mps"]),
            fmt(row["iekf_minus_ekf2_radial_rate_mean_mps"]),
            fmt(row["iekf_radial_outward_frac"]),
            fmt(row["iekf_velocity_error_h_mean_mps"]),
            fmt(row["ekf2_velocity_error_h_mean_mps"]),
            fmt(row["gt_speed_mean_mps"]),
            fmt(row["gyro_mean_deg_s"]),
        ]
        for row in rows
    ]


def table_sep(rows: Sequence[dict[str, object]]) -> list[list[object]]:
    return [
        [
            row["feature"],
            row["target_count"],
            row["positive_count"],
            fmt(row["target_mean"]),
            fmt(row["positive_mean"]),
            fmt(row["diff_target_minus_positive"]),
            fmt(row["separation_score"]),
        ]
        for row in rows
    ]


def write_report(out_dir: Path, summary_rows: list[dict[str, object]], sep_rows: list[dict[str, object]]) -> None:
    lines = [
        "# PHS5 kinematic drift atlas",
        "",
        "Date: 2026-05-10",
        "",
        "Offline finite-difference atlas from existing `groundtruth_joined.csv` rows. No flight stack, rebuild, or estimator process was started.",
        "",
        "## Summary",
        "",
        markdown_table(
            [
                "group/window",
                "rows",
                "IEKF-EKF2",
                "IEKF radial rate",
                "EKF2 radial rate",
                "radial diff",
                "outward frac",
                "IEKF vel err",
                "EKF2 vel err",
                "GT speed",
                "gyro",
            ],
            table_summary(summary_rows),
        ),
        "",
        "## Outward-Interval Feature Separation",
        "",
        markdown_table(
            ["feature", "target n", "positive n", "target mean", "positive mean", "diff", "score"],
            table_sep(sep_rows[:16]),
        ),
        "",
        "## Readout",
        "",
        "- `radial rate` is the finite-difference error-norm rate; positive means outward drift.",
        "- This atlas explains carry-in kinematics; it is not an online selector by itself.",
        "",
        "Generated files:",
        f"- `{out_dir / 'kinematic_drift_rows.csv'}`",
        f"- `{out_dir / 'kinematic_drift_summary.csv'}`",
        f"- `{out_dir / 'kinematic_drift_feature_separation.csv'}`",
    ]
    (out_dir / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--windows", default=str(DEFAULT_WINDOWS))
    parser.add_argument("--out-dir", default=str(DEFAULT_OUT))
    args = parser.parse_args()

    samples = build_samples(load_bad_windows(Path(args.windows)))
    summary_rows = build_summaries(samples)
    sep_rows = feature_separation(samples)
    out_dir = Path(args.out_dir)
    write_csv(out_dir / "kinematic_drift_rows.csv", samples)
    write_csv(out_dir / "kinematic_drift_summary.csv", summary_rows)
    write_csv(out_dir / "kinematic_drift_feature_separation.csv", sep_rows)
    write_report(out_dir, summary_rows, sep_rows)
    print(f"wrote: {out_dir / 'report.md'}")


if __name__ == "__main__":
    main()
