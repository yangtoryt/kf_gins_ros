#!/usr/bin/env python3
"""Diagnose shortgen11 accbias-z basin reproducibility across existing logs."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from typing import Iterable

import pandas as pd


BASE = Path("/home/yang/kf_gins_ws/artifacts/manual/short_route_generalization_2026-05-07")
DEFAULT_OUT = BASE / "shortgen11_accbias_basin_repro_2026-05-11"

RUN_DIRS = {
    "holdout": Path(
        "/home/yang/kf_gins_ws/artifacts/manual/"
        "manual_shortgen11_high_clearance_ladder_sitl_home_headless_compare_core8_pairlogger_phs5_holdout_20260510_145701"
    ),
    "repeat2": Path(
        "/home/yang/kf_gins_ws/artifacts/manual/"
        "manual_shortgen11_high_clearance_ladder_sitl_home_headless_compare_core8_pairlogger_phs5_repeat2_cleancheck_20260510_155253"
    ),
    "coretrace": Path(
        "/home/yang/kf_gins_ws/artifacts/manual/"
        "manual_shortgen11_high_clearance_ladder_sitl_home_headless_compare_core8_pairlogger_phs5_coretrace_20260510_223106"
    ),
    "accbiasz_diag": Path(
        "/home/yang/kf_gins_ws/artifacts/manual/"
        "manual_shortgen11_high_clearance_ladder_sitl_home_headless_compare_core8_pairlogger_phs5_accbiasz_diag_futureclamp_20260511_111411"
    ),
    "accbiasz_apply": Path(
        "/home/yang/kf_gins_ws/artifacts/manual/"
        "manual_shortgen11_high_clearance_ladder_sitl_home_headless_compare_core8_pairlogger_phs5_accbiasz_apply_futureclamp_20260511_112407"
    ),
}

WINDOWS = [
    ("0-40", 0.0, 40.0),
    ("40-80", 40.0, 80.0),
    ("80-100", 80.0, 100.0),
    ("100-120", 100.0, 120.0),
    ("120-140", 120.0, 140.0),
    ("140-160", 140.0, 160.0),
    ("160-180", 160.0, 180.0),
]


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
    value_f = to_float(value)
    return f"{value_f:.{digits}f}" if math.isfinite(value_f) else "nan"


def mean(values: Iterable[float]) -> float:
    vals = [value for value in values if finite(value)]
    return sum(vals) / len(vals) if vals else math.nan


def rmse(values: Iterable[float]) -> float:
    vals = [value for value in values if finite(value)]
    return math.sqrt(sum(value * value for value in vals) / len(vals)) if vals else math.nan


def markdown_table(headers: list[str], rows: list[list[object]]) -> str:
    lines = [
        "| " + " | ".join(headers) + " |",
        "| " + " | ".join("---" for _ in headers) + " |",
    ]
    lines.extend("| " + " | ".join(str(item) for item in row) + " |" for row in rows)
    return "\n".join(lines)


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


def numeric(frame: pd.DataFrame, column: str) -> pd.Series:
    if column not in frame:
        return pd.Series([math.nan] * len(frame), index=frame.index, dtype="float64")
    return pd.to_numeric(frame[column], errors="coerce")


def truthy(frame: pd.DataFrame, column: str) -> pd.Series:
    if column not in frame:
        return pd.Series([False] * len(frame), index=frame.index, dtype="bool")
    return frame[column].astype(str).str.strip().str.lower().isin({"1", "true", "yes"})


def load_gnss_debug(run_dir: Path) -> pd.DataFrame:
    path = run_dir / "gnss_update_debug.csv"
    if not path.exists():
        return pd.DataFrame()
    frame = pd.read_csv(path)
    if "update_time_sec" not in frame:
        return pd.DataFrame()
    out = pd.DataFrame()
    out["update_time_sec"] = numeric(frame, "update_time_sec")
    out["gnss_residual_h_m"] = (
        numeric(frame, "gnss_position_residual_n_m") ** 2
        + numeric(frame, "gnss_position_residual_e_m") ** 2
    ) ** 0.5
    out["gnss_residual_abs_u_m"] = numeric(frame, "gnss_position_residual_u_m").abs()
    out["last_position_residual_h_m"] = numeric(frame, "last_position_residual_h_m")
    out["core_gnss_diff_h_m"] = numeric(frame, "core_gnss_diff_h_m")
    out["core_gnss_abs_u_m"] = numeric(frame, "core_gnss_diff_u_m").abs()
    out["core_accbias_std_z_mps2"] = numeric(frame, "core_accbias_std_z_mps2")
    out = out.dropna(subset=["update_time_sec"]).sort_values("update_time_sec")
    return out


def load_state(run: str, run_dir: Path) -> pd.DataFrame:
    path = run_dir / "state_update_debug.csv"
    if not path.exists():
        return pd.DataFrame()
    frame = pd.read_csv(path)
    if "event_type" in frame:
        frame = frame[frame["event_type"] == "gnss_position"].copy()
    if "applied" in frame:
        frame = frame[numeric(frame, "applied") == 1].copy()
    frame["sequence"] = numeric(frame, "sequence").astype("Int64")
    frame["run"] = run
    frame["update_time_sec"] = numeric(frame, "update_time_sec")
    frame["t"] = numeric(frame, "armed_time_sec")
    frame["ba_z_before"] = numeric(frame, "accbias_z_before_mps2")
    frame["ba_z_after"] = numeric(frame, "accbias_z_after_mps2")
    frame["ba_z_update_delta"] = frame["ba_z_after"] - frame["ba_z_before"]
    frame["dx_ba_z"] = numeric(frame, "dx_ba_z_mps2")
    frame["dx_pos_h"] = numeric(frame, "dx_pos_h_norm_m")
    frame["dx_vel_h"] = numeric(frame, "dx_vel_h_norm_mps")
    frame["dx_yaw_deg"] = numeric(frame, "dx_phi_yaw_deg")
    frame["cov_pos_h_trace"] = numeric(frame, "cov_pos_n_before_m2") + numeric(frame, "cov_pos_e_before_m2")
    frame["cov_vel_h_trace"] = numeric(frame, "cov_vel_n_before_m2ps2") + numeric(frame, "cov_vel_e_before_m2ps2")
    frame["cov_ba_z"] = numeric(frame, "cov_ba_z_before_m2ps4")
    frame["cov_ba_trace"] = (
        numeric(frame, "cov_ba_x_before_m2ps4")
        + numeric(frame, "cov_ba_y_before_m2ps4")
        + numeric(frame, "cov_ba_z_before_m2ps4")
    )
    frame["hspeed"] = numeric(frame, "horizontal_speed_mps")
    frame["abs_vspeed"] = numeric(frame, "vertical_speed_mps").abs()
    frame["gyro_abs"] = numeric(frame, "gyro_deg_s").abs()
    frame["turning"] = truthy(frame, "turning_now")
    frame["post_turn"] = truthy(frame, "post_turn_context")
    frame["armed_cruise"] = truthy(frame, "armed_cruise_context")
    gnss = load_gnss_debug(run_dir)
    if not gnss.empty:
        frame = frame.dropna(subset=["update_time_sec"]).sort_values("update_time_sec")
        frame = pd.merge_asof(
            frame,
            gnss,
            on="update_time_sec",
            direction="nearest",
            tolerance=0.005,
        )
    return frame


def load_all_state() -> pd.DataFrame:
    frames = [load_state(run, run_dir) for run, run_dir in RUN_DIRS.items()]
    frames = [frame for frame in frames if not frame.empty]
    return pd.concat(frames, ignore_index=True) if frames else pd.DataFrame()


def load_groundtruth(run: str, run_dir: Path) -> pd.DataFrame:
    for rel in [
        "offline_groundtruth_convergence_diag_autogt/groundtruth_joined.csv",
        "offline_groundtruth_convergence_diag/groundtruth_joined.csv",
    ]:
        path = run_dir / rel
        if path.exists():
            frame = pd.read_csv(path)
            frame["run"] = run
            frame["t"] = numeric(frame, "time_since_arm_sec")
            return frame
    return pd.DataFrame()


def load_all_groundtruth() -> pd.DataFrame:
    frames = [load_groundtruth(run, run_dir) for run, run_dir in RUN_DIRS.items()]
    frames = [frame for frame in frames if not frame.empty]
    return pd.concat(frames, ignore_index=True) if frames else pd.DataFrame()


def basin_class(frac_le_018: float, frac_le_0205: float) -> str:
    if finite(frac_le_0205) and frac_le_0205 >= 0.40:
        return "deep"
    if finite(frac_le_018) and frac_le_018 >= 0.40:
        return "moderate"
    if finite(frac_le_018) and frac_le_018 < 0.10:
        return "clean"
    return "mixed"


def subset_window(frame: pd.DataFrame, start: float, end: float) -> pd.DataFrame:
    t = numeric(frame, "t")
    return frame[(t >= start) & (t < end)]


def window_state_summary(state: pd.DataFrame, groundtruth: pd.DataFrame) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for run in RUN_DIRS:
        run_state = state[state["run"] == run]
        run_gt = groundtruth[groundtruth["run"] == run]
        for label, start, end in WINDOWS:
            sub = subset_window(run_state, start, end)
            prior = subset_window(run_state, 40.0, start) if start > 40.0 else pd.DataFrame()
            gt = subset_window(run_gt, start, end)
            ba = numeric(sub, "ba_z_before")
            frac018 = float((ba <= -0.18).mean()) if len(ba) else math.nan
            frac0205 = float((ba <= -0.205).mean()) if len(ba) else math.nan
            prior_ba = numeric(prior, "ba_z_before") if not prior.empty else pd.Series(dtype="float64")
            ekf2_rmse = rmse(numeric(gt, "ekf2_error_xy_m"))
            iekf_rmse = rmse(numeric(gt, "iekf_error_xy_m"))
            rows.append(
                {
                    "run": run,
                    "window": label,
                    "rows": len(sub),
                    "ba_z_mean": float(ba.mean()) if len(ba) else math.nan,
                    "ba_z_min": float(ba.min()) if len(ba) else math.nan,
                    "ba_z_max": float(ba.max()) if len(ba) else math.nan,
                    "ba_z_after_mean": mean(numeric(sub, "ba_z_after")),
                    "ba_z_update_delta_mean": mean(numeric(sub, "ba_z_update_delta")),
                    "dx_ba_z_abs_mean": mean(numeric(sub, "dx_ba_z").abs()),
                    "dx_pos_h_mean": mean(numeric(sub, "dx_pos_h")),
                    "dx_pos_h_p95": float(numeric(sub, "dx_pos_h").quantile(0.95)) if len(sub) else math.nan,
                    "dx_vel_h_mean": mean(numeric(sub, "dx_vel_h")),
                    "cov_pos_h_trace_mean": mean(numeric(sub, "cov_pos_h_trace")),
                    "cov_vel_h_trace_mean": mean(numeric(sub, "cov_vel_h_trace")),
                    "cov_ba_z_mean": mean(numeric(sub, "cov_ba_z")),
                    "cov_ba_trace_mean": mean(numeric(sub, "cov_ba_trace")),
                    "gnss_residual_h_mean": mean(numeric(sub, "gnss_residual_h_m")),
                    "gnss_residual_abs_u_mean": mean(numeric(sub, "gnss_residual_abs_u_m")),
                    "last_position_residual_h_mean": mean(numeric(sub, "last_position_residual_h_m")),
                    "core_gnss_diff_h_mean": mean(numeric(sub, "core_gnss_diff_h_m")),
                    "core_gnss_abs_u_mean": mean(numeric(sub, "core_gnss_abs_u_m")),
                    "core_accbias_std_z_mean": mean(numeric(sub, "core_accbias_std_z_mps2")),
                    "hspeed_mean": mean(numeric(sub, "hspeed")),
                    "abs_vspeed_mean": mean(numeric(sub, "abs_vspeed")),
                    "gyro_abs_mean": mean(numeric(sub, "gyro_abs")),
                    "turning_frac": float(sub["turning"].mean()) if len(sub) else math.nan,
                    "post_turn_frac": float(sub["post_turn"].mean()) if len(sub) else math.nan,
                    "armed_cruise_frac": float(sub["armed_cruise"].mean()) if len(sub) else math.nan,
                    "ba_z_le_m0_18_frac": frac018,
                    "ba_z_le_m0_205_frac": frac0205,
                    "basin_class": basin_class(frac018, frac0205),
                    "prior_40_to_start_ba_z_mean": float(prior_ba.mean()) if len(prior_ba) else math.nan,
                    "prior_40_to_start_ba_z_min": float(prior_ba.min()) if len(prior_ba) else math.nan,
                    "prior_40_to_start_deep_frac": float((prior_ba <= -0.205).mean()) if len(prior_ba) else math.nan,
                    "ekf2_xy_rmse_m": ekf2_rmse,
                    "iekf_xy_rmse_m": iekf_rmse,
                    "iekf_minus_ekf2_rmse_m": iekf_rmse - ekf2_rmse if finite(ekf2_rmse) and finite(iekf_rmse) else math.nan,
                    "gt_rows": len(gt),
                }
            )
    return rows


def split_summary(window_rows: list[dict[str, object]]) -> list[dict[str, object]]:
    by_window: dict[str, list[dict[str, object]]] = {}
    for row in window_rows:
        by_window.setdefault(str(row["window"]), []).append(row)
    out: list[dict[str, object]] = []
    for label, _start, _end in WINDOWS:
        rows = by_window.get(label, [])
        vals = [to_float(row["ba_z_mean"]) for row in rows if finite(row["ba_z_mean"])]
        repeat2 = next((row for row in rows if row["run"] == "repeat2"), None)
        diag = next((row for row in rows if row["run"] == "accbiasz_diag"), None)
        holdout = next((row for row in rows if row["run"] == "holdout"), None)
        out.append(
            {
                "window": label,
                "run_count": len(vals),
                "ba_z_mean_min": min(vals) if vals else math.nan,
                "ba_z_mean_max": max(vals) if vals else math.nan,
                "ba_z_mean_spread": (max(vals) - min(vals)) if vals else math.nan,
                "repeat2_minus_diag_ba_z_mean": (
                    to_float(repeat2["ba_z_mean"]) - to_float(diag["ba_z_mean"])
                    if repeat2 is not None and diag is not None else math.nan
                ),
                "repeat2_minus_holdout_ba_z_mean": (
                    to_float(repeat2["ba_z_mean"]) - to_float(holdout["ba_z_mean"])
                    if repeat2 is not None and holdout is not None else math.nan
                ),
                "repeat2_class": repeat2["basin_class"] if repeat2 is not None else "",
                "diag_class": diag["basin_class"] if diag is not None else "",
                "holdout_class": holdout["basin_class"] if holdout is not None else "",
            }
        )
    return out


def first_window(rows: list[dict[str, object]], run: str, allowed: set[str]) -> str:
    for label, _start, _end in WINDOWS:
        row = next((item for item in rows if item["run"] == run and item["window"] == label), None)
        if row and row.get("basin_class") in allowed:
            return label
    return "none"


def transition_rows(window_rows: list[dict[str, object]]) -> list[dict[str, object]]:
    out: list[dict[str, object]] = []
    for run in RUN_DIRS:
        out.append(
            {
                "run": run,
                "first_moderate_or_deep": first_window(window_rows, run, {"moderate", "deep"}),
                "first_deep": first_window(window_rows, run, {"deep"}),
                "class_40_80": next((r["basin_class"] for r in window_rows if r["run"] == run and r["window"] == "40-80"), ""),
                "class_100_120": next((r["basin_class"] for r in window_rows if r["run"] == run and r["window"] == "100-120"), ""),
                "class_120_140": next((r["basin_class"] for r in window_rows if r["run"] == run and r["window"] == "120-140"), ""),
                "class_140_160": next((r["basin_class"] for r in window_rows if r["run"] == run and r["window"] == "140-160"), ""),
                "class_160_180": next((r["basin_class"] for r in window_rows if r["run"] == run and r["window"] == "160-180"), ""),
            }
        )
    return out


def table_main(rows: list[dict[str, object]]) -> list[list[object]]:
    keep_windows = {"40-80", "80-100", "100-120", "120-140", "140-160", "160-180"}
    return [
        [
            row["run"],
            row["window"],
            row["rows"],
            row["basin_class"],
            fmt(row["ba_z_mean"]),
            fmt(row["ba_z_le_m0_18_frac"]),
            fmt(row["ba_z_le_m0_205_frac"]),
            fmt(row["prior_40_to_start_ba_z_mean"]),
            fmt(row["gnss_residual_h_mean"]),
            fmt(row["dx_pos_h_mean"]),
            fmt(row["cov_ba_z_mean"]),
            fmt(row["iekf_minus_ekf2_rmse_m"]),
        ]
        for row in rows
        if row["window"] in keep_windows
    ]


def table_split(rows: list[dict[str, object]]) -> list[list[object]]:
    return [
        [
            row["window"],
            fmt(row["ba_z_mean_min"]),
            fmt(row["ba_z_mean_max"]),
            fmt(row["ba_z_mean_spread"]),
            fmt(row["repeat2_minus_diag_ba_z_mean"]),
            fmt(row["repeat2_minus_holdout_ba_z_mean"]),
            row["holdout_class"],
            row["repeat2_class"],
            row["diag_class"],
        ]
        for row in rows
    ]


def table_transition(rows: list[dict[str, object]]) -> list[list[object]]:
    return [
        [
            row["run"],
            row["first_moderate_or_deep"],
            row["first_deep"],
            row["class_40_80"],
            row["class_100_120"],
            row["class_120_140"],
            row["class_140_160"],
            row["class_160_180"],
        ]
        for row in rows
    ]


def write_report(
    out_dir: Path,
    window_rows: list[dict[str, object]],
    split_rows: list[dict[str, object]],
    transitions: list[dict[str, object]],
) -> None:
    lines = [
        "# shortgen11 accbias-z basin reproducibility diagnostic",
        "",
        "Date: 2026-05-11",
        "",
        "Offline-only diagnostic from existing shortgen11 logs. No PX4/Gazebo/MAVROS/QGC/RViz2/PlotJuggler process was started.",
        "",
        "Runs: `holdout`, `repeat2`, `coretrace`, `accbiasz_diag`, `accbiasz_apply`.",
        "",
        "Basin classes: `deep` means `frac(accbias_z <= -0.205) >= 0.40`; `moderate` means `frac(accbias_z <= -0.18) >= 0.40`; `clean` means `frac(accbias_z <= -0.18) < 0.10`.",
        "",
        "## Basin Timeline",
        "",
        markdown_table(
            [
                "run",
                "window",
                "rows",
                "class",
                "ba_z mean",
                "frac <= -0.18",
                "frac <= -0.205",
                "prior ba_z",
                "gnss resid h",
                "dx pos h",
                "cov ba_z",
                "IEKF-EKF2 RMSE",
            ],
            table_main(window_rows),
        ),
        "",
        "## Cross-run Split",
        "",
        markdown_table(
            [
                "window",
                "min ba_z",
                "max ba_z",
                "spread",
                "repeat2 - diag",
                "repeat2 - holdout",
                "holdout class",
                "repeat2 class",
                "diag class",
            ],
            table_split(split_rows),
        ),
        "",
        "## Transition Summary",
        "",
        markdown_table(
            [
                "run",
                "first moderate/deep",
                "first deep",
                "40-80",
                "100-120",
                "120-140",
                "140-160",
                "160-180",
            ],
            table_transition(transitions),
        ),
        "",
        "## Interpretation",
        "",
        "- The clean-vs-negative split is already visible by 40-80s: `holdout` is clean while `repeat2`, `coretrace`, `accbiasz_diag`, and `accbiasz_apply` are already moderate/deep.",
        "- The specific repeat2-vs-diagnostic threshold conflict is visible before the target failure window: `repeat2` relaxes around 100-140s while `accbiasz_diag` remains deep.",
        "- Current-window `accbias_z` is not sufficient as a mechanism gate: `accbiasz_diag` has a negative 160-180s basin but IEKF is not worse there, while `repeat2` fails at 140-160s with only a moderate instantaneous basin.",
        "- Prior negative-basin history is also insufficient by itself because the diagnostic run carries negative history into 160-180s without matching the repeat2 failure behavior.",
        "",
        "## Decision",
        "",
        "Do not run another accbias-z Q-scale mechanism flight yet. The next useful step is an offline event-history diagnostic that joins error direction, state correction direction, and bias-basin transitions. If that cannot separate repeat2 140-160 from diagnostic 160-180, keep accbias-z as diagnostic-only rather than an online action gate.",
        "",
    ]
    (out_dir / "report.md").write_text("\n".join(lines), encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUT)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)
    state = load_all_state()
    groundtruth = load_all_groundtruth()
    if state.empty:
        raise SystemExit("no state_update_debug rows found")
    window_rows = window_state_summary(state, groundtruth)
    split_rows = split_summary(window_rows)
    transitions = transition_rows(window_rows)
    write_csv(args.out_dir / "window_state_summary.csv", window_rows)
    write_csv(args.out_dir / "cross_run_split.csv", split_rows)
    write_csv(args.out_dir / "basin_transitions.csv", transitions)
    write_report(args.out_dir, window_rows, split_rows, transitions)
    print(f"wrote: {args.out_dir / 'window_state_summary.csv'}")
    print(f"wrote: {args.out_dir / 'cross_run_split.csv'}")
    print(f"wrote: {args.out_dir / 'basin_transitions.csv'}")
    print(f"wrote: {args.out_dir / 'report.md'}")


if __name__ == "__main__":
    main()
