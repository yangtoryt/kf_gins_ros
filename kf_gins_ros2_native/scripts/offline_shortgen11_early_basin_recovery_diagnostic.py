#!/usr/bin/env python3
"""Diagnose early shortgen11 accbias-z basin entry and recovery.

This script is offline-only. It compares existing shortgen11 logs to identify
which update family drives the early negative vertical accelerometer-bias basin,
and whether the clean run recovers through a different early update pattern.
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from typing import Iterable

import pandas as pd


BASE = Path("/home/yang/kf_gins_ws/artifacts/manual/short_route_generalization_2026-05-07")
DEFAULT_OUT = BASE / "shortgen11_early_basin_recovery_2026-05-11"

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

NEGATIVE_RUNS = {"repeat2", "coretrace", "accbiasz_diag", "accbiasz_apply"}

PROTECTION_RUN_DIRS = {
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
}

GATE_PROBE_RUN_DIRS = {
    **PROTECTION_RUN_DIRS,
    "shortgen11_holdout": RUN_DIRS["holdout"],
    "shortgen11_repeat2": RUN_DIRS["repeat2"],
    "shortgen11_coretrace": RUN_DIRS["coretrace"],
    "shortgen11_accbiasz_diag": RUN_DIRS["accbiasz_diag"],
    "shortgen11_accbiasz_apply": RUN_DIRS["accbiasz_apply"],
}

WINDOWS = [
    ("0-20", 0.0, 20.0),
    ("20-40", 20.0, 40.0),
    ("40-60", 40.0, 60.0),
    ("60-80", 60.0, 80.0),
    ("80-100", 80.0, 100.0),
    ("100-120", 100.0, 120.0),
    ("0-40", 0.0, 40.0),
    ("40-80", 40.0, 80.0),
    ("80-120", 80.0, 120.0),
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
    val = to_float(value)
    return f"{val:.{digits}f}" if finite(val) else "nan"


def mean(values: Iterable[float]) -> float:
    vals = [value for value in values if finite(value)]
    return sum(vals) / len(vals) if vals else math.nan


def numeric(frame: pd.DataFrame, column: str) -> pd.Series:
    if column not in frame:
        return pd.Series([math.nan] * len(frame), index=frame.index, dtype="float64")
    return pd.to_numeric(frame[column], errors="coerce")


def truthy(frame: pd.DataFrame, column: str) -> pd.Series:
    if column not in frame:
        return pd.Series([False] * len(frame), index=frame.index, dtype="bool")
    return frame[column].astype(str).str.strip().str.lower().isin({"1", "true", "yes"})


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


def load_gnss(run_dir: Path) -> pd.DataFrame:
    path = run_dir / "gnss_update_debug.csv"
    if not path.exists():
        return pd.DataFrame()
    frame = pd.read_csv(path)
    out = pd.DataFrame()
    out["update_time_sec"] = numeric(frame, "update_time_sec")
    out["gnss_residual_n_m"] = numeric(frame, "gnss_position_residual_n_m")
    out["gnss_residual_e_m"] = numeric(frame, "gnss_position_residual_e_m")
    out["gnss_residual_h_m"] = (
        out["gnss_residual_n_m"] * out["gnss_residual_n_m"]
        + out["gnss_residual_e_m"] * out["gnss_residual_e_m"]
    ) ** 0.5
    out["gnss_residual_u_m"] = numeric(frame, "gnss_position_residual_u_m")
    out["last_position_residual_h_m"] = numeric(frame, "last_position_residual_h_m")
    out["last_position_residual_u_m"] = numeric(frame, "last_position_residual_u_m")
    out["core_gnss_diff_h_m"] = numeric(frame, "core_gnss_diff_h_m")
    out["core_gnss_diff_u_m"] = numeric(frame, "core_gnss_diff_u_m")
    out["core_pos_std_d_m"] = numeric(frame, "core_pos_std_d_m")
    out["core_accbias_std_z_mps2"] = numeric(frame, "core_accbias_std_z_mps2")
    out = out.dropna(subset=["update_time_sec"]).sort_values("update_time_sec")
    return out


def load_events(run: str, run_dir: Path) -> pd.DataFrame:
    path = run_dir / "state_update_debug.csv"
    if not path.exists():
        return pd.DataFrame()
    frame = pd.read_csv(path)
    if "applied" in frame:
        frame = frame[numeric(frame, "applied") == 1].copy()
    frame["run"] = run
    frame["t"] = numeric(frame, "armed_time_sec")
    frame["update_time_sec"] = numeric(frame, "update_time_sec")
    frame["event_type"] = frame.get("event_type", "").astype(str)
    frame["ba_z_before"] = numeric(frame, "accbias_z_before_mps2")
    frame["ba_z_after"] = numeric(frame, "accbias_z_after_mps2")
    frame["dx_ba_z"] = numeric(frame, "dx_ba_z_mps2")
    frame["dx_pos_u"] = numeric(frame, "dx_pos_u_m")
    frame["dx_pos_h"] = numeric(frame, "dx_pos_h_norm_m")
    frame["dx_vel_d"] = numeric(frame, "dx_vel_d_mps")
    frame["dx_vel_h"] = numeric(frame, "dx_vel_h_norm_mps")
    frame["cov_ba_z"] = numeric(frame, "cov_ba_z_before_m2ps4")
    frame["cov_pos_u"] = numeric(frame, "cov_pos_u_before_m2")
    frame["cov_vel_d"] = numeric(frame, "cov_vel_d_before_m2ps2")
    frame["hspeed"] = numeric(frame, "horizontal_speed_mps")
    frame["vspeed"] = numeric(frame, "vertical_speed_mps")
    frame["gyro_abs"] = numeric(frame, "gyro_deg_s").abs()
    frame["turning"] = truthy(frame, "turning_now")
    frame["post_turn"] = truthy(frame, "post_turn_context")
    frame["armed_cruise"] = truthy(frame, "armed_cruise_context")
    frame = frame.dropna(subset=["t", "update_time_sec"]).sort_values("update_time_sec")

    gnss = load_gnss(run_dir)
    if not gnss.empty:
        frame = pd.merge_asof(
            frame,
            gnss,
            on="update_time_sec",
            direction="nearest",
            tolerance=0.005,
        )
    return frame


def load_events_for(run_dirs: dict[str, Path]) -> pd.DataFrame:
    frames = [load_events(run, run_dir) for run, run_dir in run_dirs.items()]
    frames = [frame for frame in frames if not frame.empty]
    return pd.concat(frames, ignore_index=True) if frames else pd.DataFrame()


def load_all_events() -> pd.DataFrame:
    return load_events_for(RUN_DIRS)


def subset(frame: pd.DataFrame, start: float, end: float) -> pd.DataFrame:
    t = numeric(frame, "t")
    return frame[(t >= start) & (t < end)]


def summarize_update_family(events: pd.DataFrame) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for run in RUN_DIRS:
        run_rows = subset(events[events["run"] == run], 0.0, 80.0)
        for event_type, group in run_rows.groupby("event_type", dropna=False):
            dx = numeric(group, "dx_ba_z")
            rows.append(
                {
                    "run": run,
                    "event_type": event_type,
                    "rows": len(group),
                    "dx_ba_z_sum": dx.sum(),
                    "dx_ba_z_mean": mean(dx),
                    "dx_ba_z_negative_frac": float((dx < 0.0).mean()) if len(dx) else math.nan,
                    "ba_z_first": to_float(group["ba_z_before"].iloc[0]) if len(group) else math.nan,
                    "ba_z_last": to_float(group["ba_z_after"].iloc[-1]) if len(group) else math.nan,
                }
            )
    return rows


def summarize_position_windows(events: pd.DataFrame) -> list[dict[str, object]]:
    pos = events[events["event_type"] == "gnss_position"].copy()
    rows: list[dict[str, object]] = []
    for run in RUN_DIRS:
        run_rows = pos[pos["run"] == run]
        for label, start, end in WINDOWS:
            sub = subset(run_rows, start, end)
            dx = numeric(sub, "dx_ba_z")
            ba = numeric(sub, "ba_z_before")
            res_u = numeric(sub, "gnss_residual_u_m")
            core_u = numeric(sub, "core_gnss_diff_u_m")
            rows.append(
                {
                    "run": run,
                    "window": label,
                    "rows": len(sub),
                    "ba_z_mean": mean(ba),
                    "ba_z_first": to_float(ba.iloc[0]) if len(ba) else math.nan,
                    "ba_z_last": to_float(ba.iloc[-1]) if len(ba) else math.nan,
                    "ba_z_le_m0_18_frac": float((ba <= -0.18).mean()) if len(ba) else math.nan,
                    "ba_z_le_m0_205_frac": float((ba <= -0.205).mean()) if len(ba) else math.nan,
                    "dx_ba_z_sum": dx.sum(),
                    "dx_ba_z_mean": mean(dx),
                    "dx_ba_z_neg_frac": float((dx < 0.0).mean()) if len(dx) else math.nan,
                    "gnss_residual_u_mean_m": mean(res_u),
                    "gnss_residual_u_abs_mean_m": mean(res_u.abs()),
                    "gnss_residual_h_mean_m": mean(numeric(sub, "gnss_residual_h_m")),
                    "core_gnss_u_mean_m": mean(core_u),
                    "dx_pos_u_mean_m": mean(numeric(sub, "dx_pos_u")),
                    "dx_pos_h_mean_m": mean(numeric(sub, "dx_pos_h")),
                    "abs_vspeed_mean_mps": mean(numeric(sub, "vspeed").abs()),
                    "hspeed_mean_mps": mean(numeric(sub, "hspeed")),
                    "cov_ba_z_mean": mean(numeric(sub, "cov_ba_z")),
                    "cov_pos_u_mean": mean(numeric(sub, "cov_pos_u")),
                    "post_turn_frac": float(sub["post_turn"].mean()) if len(sub) else math.nan,
                    "armed_cruise_frac": float(sub["armed_cruise"].mean()) if len(sub) else math.nan,
                    "turning_frac": float(sub["turning"].mean()) if len(sub) else math.nan,
                    "dx_resu_corr": dx.corr(res_u) if len(sub) > 2 else math.nan,
                }
            )
    return rows


def crossing_rows(events: pd.DataFrame) -> list[dict[str, object]]:
    pos = events[events["event_type"] == "gnss_position"].copy()
    out: list[dict[str, object]] = []
    for run in RUN_DIRS:
        run_rows = pos[pos["run"] == run].sort_values("t")
        for threshold in [-0.18, -0.205]:
            crossed = run_rows[numeric(run_rows, "ba_z_before") <= threshold]
            row = crossed.iloc[0] if len(crossed) else None
            out.append(
                {
                    "run": run,
                    "threshold": threshold,
                    "first_cross_t": to_float(row["t"]) if row is not None else math.nan,
                    "first_cross_ba_z": to_float(row["ba_z_before"]) if row is not None else math.nan,
                    "first_cross_dx_ba_z": to_float(row["dx_ba_z"]) if row is not None else math.nan,
                    "first_cross_residual_u_m": to_float(row.get("gnss_residual_u_m")) if row is not None else math.nan,
                    "first_cross_core_gnss_u_m": to_float(row.get("core_gnss_diff_u_m")) if row is not None else math.nan,
                    "first_cross_abs_vspeed_mps": abs(to_float(row.get("vspeed"))) if row is not None else math.nan,
                }
            )

        deep = run_rows[numeric(run_rows, "ba_z_before") <= -0.205]
        first_deep_t = to_float(deep.iloc[0]["t"]) if len(deep) else math.nan
        recovered = run_rows[
            (numeric(run_rows, "t") > first_deep_t)
            & (numeric(run_rows, "ba_z_before") > -0.18)
        ] if finite(first_deep_t) else pd.DataFrame()
        rec = recovered.iloc[0] if len(recovered) else None
        out.append(
            {
                "run": run,
                "threshold": "recover_gt_-0.18_after_deep",
                "first_cross_t": to_float(rec["t"]) if rec is not None else math.nan,
                "first_cross_ba_z": to_float(rec["ba_z_before"]) if rec is not None else math.nan,
                "first_cross_dx_ba_z": to_float(rec["dx_ba_z"]) if rec is not None else math.nan,
                "first_cross_residual_u_m": to_float(rec.get("gnss_residual_u_m")) if rec is not None else math.nan,
                "first_cross_core_gnss_u_m": to_float(rec.get("core_gnss_diff_u_m")) if rec is not None else math.nan,
                "first_cross_abs_vspeed_mps": abs(to_float(rec.get("vspeed"))) if rec is not None else math.nan,
            }
        )
    return out


def contrast_features(window_rows: list[dict[str, object]]) -> list[dict[str, object]]:
    features = [
        "dx_ba_z_sum",
        "dx_ba_z_neg_frac",
        "gnss_residual_u_mean_m",
        "gnss_residual_u_abs_mean_m",
        "gnss_residual_h_mean_m",
        "core_gnss_u_mean_m",
        "dx_pos_u_mean_m",
        "abs_vspeed_mean_mps",
        "cov_ba_z_mean",
    ]
    out: list[dict[str, object]] = []
    for window in ["0-40", "40-80", "80-120"]:
        selected = [row for row in window_rows if row["window"] == window]
        holdout = next((row for row in selected if row["run"] == "holdout"), None)
        neg = [row for row in selected if row["run"] in NEGATIVE_RUNS]
        if holdout is None or not neg:
            continue
        for feature in features:
            neg_vals = [to_float(row.get(feature)) for row in neg if finite(row.get(feature))]
            neg_mean = mean(neg_vals)
            holdout_val = to_float(holdout.get(feature))
            out.append(
                {
                    "window": window,
                    "feature": feature,
                    "holdout": holdout_val,
                    "negative_mean": neg_mean,
                    "negative_minus_holdout": neg_mean - holdout_val
                    if finite(neg_mean) and finite(holdout_val)
                    else math.nan,
                }
            )
    return out


def early_gate_probe_rows(events: pd.DataFrame) -> list[dict[str, object]]:
    pos = events[events["event_type"] == "gnss_position"].copy()
    rows: list[dict[str, object]] = []
    for run in GATE_PROBE_RUN_DIRS:
        run_rows = pos[pos["run"] == run]
        for label, start, end in [("0-40", 0.0, 40.0), ("40-80", 40.0, 80.0), ("80-120", 80.0, 120.0)]:
            sub = subset(run_rows, start, end)
            ba = numeric(sub, "ba_z_before")
            dx = numeric(sub, "dx_ba_z")
            res_u = numeric(sub, "gnss_residual_u_m")
            core_u = numeric(sub, "core_gnss_diff_u_m")
            ba_mean = mean(ba)
            dx_sum = dx.sum()
            res_u_mean = mean(res_u)
            core_u_mean = mean(core_u)
            # Diagnostic-only candidate: persistent negative basin during early
            # climb-to-cruise, with vertical residual sign still pushing the
            # position update toward negative accbias-z.
            gate_active = (
                label == "40-80"
                and finite(ba_mean)
                and ba_mean <= -0.18
                and finite(res_u_mean)
                and res_u_mean <= -0.02
                and finite(core_u_mean)
                and core_u_mean >= 0.02
                and finite(dx_sum)
                and dx_sum < 0.0
            )
            rows.append(
                {
                    "run": run,
                    "window": label,
                    "rows": len(sub),
                    "ba_z_mean": ba_mean,
                    "ba_z_first": to_float(ba.iloc[0]) if len(ba) else math.nan,
                    "ba_z_last": to_float(ba.iloc[-1]) if len(ba) else math.nan,
                    "dx_ba_z_sum": dx_sum,
                    "gnss_residual_u_mean_m": res_u_mean,
                    "core_gnss_u_mean_m": core_u_mean,
                    "early_recovery_gate_active": int(gate_active),
                }
            )
    return rows


def table_update_family(rows: list[dict[str, object]]) -> list[list[object]]:
    keep = {"gnss_position", "gnss_velocity", "heading"}
    return [
        [
            row["run"],
            row["event_type"],
            row["rows"],
            fmt(row["dx_ba_z_sum"]),
            fmt(row["dx_ba_z_mean"], 6),
            fmt(row["dx_ba_z_negative_frac"]),
            fmt(row["ba_z_last"]),
        ]
        for row in rows
        if row["event_type"] in keep
    ]


def table_windows(rows: list[dict[str, object]]) -> list[list[object]]:
    keep = {"0-40", "40-80", "80-120"}
    return [
        [
            row["run"],
            row["window"],
            row["rows"],
            fmt(row["ba_z_first"]),
            fmt(row["ba_z_last"]),
            fmt(row["dx_ba_z_sum"]),
            fmt(row["dx_ba_z_neg_frac"]),
            fmt(row["gnss_residual_u_mean_m"]),
            fmt(row["core_gnss_u_mean_m"]),
            fmt(row["abs_vspeed_mean_mps"]),
            fmt(row["cov_ba_z_mean"], 6),
        ]
        for row in rows
        if row["window"] in keep
    ]


def table_crossings(rows: list[dict[str, object]]) -> list[list[object]]:
    return [
        [
            row["run"],
            row["threshold"],
            fmt(row["first_cross_t"], 2),
            fmt(row["first_cross_ba_z"]),
            fmt(row["first_cross_dx_ba_z"], 6),
            fmt(row["first_cross_residual_u_m"]),
            fmt(row["first_cross_core_gnss_u_m"]),
            fmt(row["first_cross_abs_vspeed_mps"]),
        ]
        for row in rows
    ]


def table_contrast(rows: list[dict[str, object]]) -> list[list[object]]:
    priority = {
        ("0-40", "dx_ba_z_sum"),
        ("40-80", "dx_ba_z_sum"),
        ("40-80", "gnss_residual_u_mean_m"),
        ("40-80", "core_gnss_u_mean_m"),
        ("80-120", "dx_ba_z_sum"),
        ("80-120", "gnss_residual_u_mean_m"),
    }
    return [
        [
            row["window"],
            row["feature"],
            fmt(row["holdout"]),
            fmt(row["negative_mean"]),
            fmt(row["negative_minus_holdout"]),
        ]
        for row in rows
        if (row["window"], row["feature"]) in priority
    ]


def table_gate_probe(rows: list[dict[str, object]]) -> list[list[object]]:
    selected = [row for row in rows if row["window"] == "40-80"]
    return [
        [
            row["run"],
            row["rows"],
            fmt(row["ba_z_mean"]),
            fmt(row["dx_ba_z_sum"]),
            fmt(row["gnss_residual_u_mean_m"]),
            fmt(row["core_gnss_u_mean_m"]),
            row["early_recovery_gate_active"],
        ]
        for row in selected
    ]


def write_report(
    out_dir: Path,
    family_rows: list[dict[str, object]],
    window_rows: list[dict[str, object]],
    crossings: list[dict[str, object]],
    contrasts: list[dict[str, object]],
    gate_probe: list[dict[str, object]],
) -> None:
    lines = [
        "# shortgen11 early accbias-z basin recovery diagnostic",
        "",
        "Date: 2026-05-11",
        "",
        "Offline-only diagnostic from existing shortgen11 logs. It compares the clean holdout with repeat/coretrace/accbiasz reruns and does not run PX4, Gazebo, MAVROS, QGC, RViz2, PlotJuggler, or estimator code.",
        "",
        "## Update-Family Balance, 0-80s",
        "",
        markdown_table(
            ["run", "event", "rows", "sum dx_ba_z", "mean dx_ba_z", "neg frac", "last ba_z"],
            table_update_family(family_rows),
        ),
        "",
        "## Position-Update Windows",
        "",
        markdown_table(
            [
                "run",
                "window",
                "rows",
                "ba first",
                "ba last",
                "sum dx_ba_z",
                "neg frac",
                "res U",
                "core-GNSS U",
                "|vU|",
                "cov ba_z",
            ],
            table_windows(window_rows),
        ),
        "",
        "## Threshold Crossings",
        "",
        markdown_table(
            ["run", "event", "t", "ba_z", "dx_ba_z", "res U", "core-GNSS U", "|vU|"],
            table_crossings(crossings),
        ),
        "",
        "## Clean-vs-Negative Contrast",
        "",
        markdown_table(
            ["window", "feature", "holdout", "negative mean", "negative-holdout"],
            table_contrast(contrasts),
        ),
        "",
        "## Early Recovery Gate Probe",
        "",
        "Diagnostic candidate, not a mechanism: 40-80s mean `ba_z <= -0.18`, mean `gnss_residual_u <= -0.02`, mean `core_gnss_u >= 0.02`, and cumulative `dx_ba_z < 0`.",
        "",
        markdown_table(
            ["run", "rows", "ba_z", "sum dx_ba_z", "res U", "core-GNSS U", "active"],
            table_gate_probe(gate_probe),
        ),
        "",
        "## Interpretation",
        "",
        "- The early basin is driven almost entirely by `gnss_position` updates. In 0-80s, velocity and heading updates contribute little to `accbias_z`; the negative runs differ because position updates push `dx_ba_z` negative.",
        "- The clean holdout is not simply bias-free at takeoff: it starts with a moderate negative `accbias_z` in 0-40s, then recovers in 40-80s through positive cumulative position-update `dx_ba_z`.",
        "- The negative reruns share the opposite 40-80s pattern: vertical residual/core-GNSS-U sign stays in the direction that keeps position-update `dx_ba_z` negative or near zero, so the estimator remains in the negative basin before the later 120-160s failures.",
        "- This makes the likely failure mechanism earlier than the 140-160s target window: failed early vertical/bias recovery during the climb-to-cruise transition, not a late wrong-direction GNSS update.",
        "- The coarse early recovery gate is inactive on shortgen01/02/03/04 and on clean shortgen11 holdout in this replay, while active on the four negative shortgen11 reruns. That is a useful diagnostic signature, but it is still derived from a small rerun set.",
        "- A raw accbias threshold is still not a safe mechanism gate because recovery can happen after entering the basin, and the same basin can appear in windows where IEKF is already better.",
        "",
        "## Decision",
        "",
        "Do not run an online mechanism from this result yet. The next mechanism candidate should target early recovery-state handling, for example a diagnostic-only gate around 40-80s negative-basin persistence plus vertical residual/core-GNSS-U sign, then validate it offline against shortgen01/02/03/04 before any flight.",
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
    events = load_all_events()
    if events.empty:
        raise SystemExit("no state-update rows found")
    gate_events = load_events_for(GATE_PROBE_RUN_DIRS)
    family_rows = summarize_update_family(events)
    window_rows = summarize_position_windows(events)
    crossings = crossing_rows(events)
    contrasts = contrast_features(window_rows)
    gate_probe = early_gate_probe_rows(gate_events)
    write_csv(args.out_dir / "update_family_balance_0_80.csv", family_rows)
    write_csv(args.out_dir / "position_update_window_summary.csv", window_rows)
    write_csv(args.out_dir / "threshold_crossings.csv", crossings)
    write_csv(args.out_dir / "clean_vs_negative_contrast.csv", contrasts)
    write_csv(args.out_dir / "early_recovery_gate_probe.csv", gate_probe)
    write_report(args.out_dir, family_rows, window_rows, crossings, contrasts, gate_probe)
    print(f"wrote: {args.out_dir / 'update_family_balance_0_80.csv'}")
    print(f"wrote: {args.out_dir / 'position_update_window_summary.csv'}")
    print(f"wrote: {args.out_dir / 'threshold_crossings.csv'}")
    print(f"wrote: {args.out_dir / 'clean_vs_negative_contrast.csv'}")
    print(f"wrote: {args.out_dir / 'early_recovery_gate_probe.csv'}")
    print(f"wrote: {args.out_dir / 'report.md'}")


if __name__ == "__main__":
    main()
