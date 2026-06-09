#!/usr/bin/env python3
"""Diagnose whether event-history geometry can separate shortgen11 bad windows."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from typing import Iterable

import pandas as pd


BASE = Path("/home/yang/kf_gins_ws/artifacts/manual/short_route_generalization_2026-05-07")
DEFAULT_OUT = BASE / "shortgen11_event_history_selector_2026-05-11"

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

CONTRAST_WINDOWS = [
    ("holdout", "140-160"),
    ("repeat2", "140-160"),
    ("repeat2", "160-180"),
    ("coretrace", "140-160"),
    ("coretrace", "160-180"),
    ("accbiasz_diag", "140-160"),
    ("accbiasz_diag", "160-180"),
    ("accbiasz_apply", "140-160"),
    ("accbiasz_apply", "160-180"),
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


def numeric(frame: pd.DataFrame, column: str) -> pd.Series:
    if column not in frame:
        return pd.Series([math.nan] * len(frame), index=frame.index, dtype="float64")
    return pd.to_numeric(frame[column], errors="coerce")


def truthy(frame: pd.DataFrame, column: str) -> pd.Series:
    if column not in frame:
        return pd.Series([False] * len(frame), index=frame.index, dtype="bool")
    return frame[column].astype(str).str.strip().str.lower().isin({"1", "true", "yes"})


def norm2(x: pd.Series, y: pd.Series) -> pd.Series:
    return (x * x + y * y) ** 0.5


def cosine(ax: pd.Series, ay: pd.Series, bx: pd.Series, by: pd.Series) -> pd.Series:
    denom = norm2(ax, ay) * norm2(bx, by)
    return (ax * bx + ay * by) / denom.where(denom > 1e-9)


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
            return frame.dropna(subset=["t"]).sort_values("t")
    return pd.DataFrame()


def load_gnss(run_dir: Path) -> pd.DataFrame:
    path = run_dir / "gnss_update_debug.csv"
    if not path.exists():
        return pd.DataFrame()
    frame = pd.read_csv(path)
    out = pd.DataFrame()
    out["update_time_sec"] = numeric(frame, "update_time_sec")
    out["meas_resid_n_m"] = numeric(frame, "gnss_position_residual_n_m")
    out["meas_resid_e_m"] = numeric(frame, "gnss_position_residual_e_m")
    out["meas_resid_u_m"] = numeric(frame, "gnss_position_residual_u_m")
    out["last_position_residual_h_m"] = numeric(frame, "last_position_residual_h_m")
    out["last_position_residual_u_m"] = numeric(frame, "last_position_residual_u_m")
    out["core_gnss_diff_h_m"] = numeric(frame, "core_gnss_diff_h_m")
    out["core_gnss_diff_u_m"] = numeric(frame, "core_gnss_diff_u_m")
    out["core_accbias_std_z_mps2"] = numeric(frame, "core_accbias_std_z_mps2")
    out["phase_error_memory_recent_turnpost_active"] = truthy(
        frame, "phase_error_memory_recent_turnpost_active"
    )
    out["phase_error_memory_candidate_active"] = truthy(frame, "phase_error_memory_candidate_active")
    out = out.dropna(subset=["update_time_sec"]).sort_values("update_time_sec")
    return out


def load_events(run: str, run_dir: Path) -> pd.DataFrame:
    state_path = run_dir / "state_update_debug.csv"
    if not state_path.exists():
        return pd.DataFrame()
    state = pd.read_csv(state_path)
    if "event_type" in state:
        state = state[state["event_type"] == "gnss_position"].copy()
    if "applied" in state:
        state = state[numeric(state, "applied") == 1].copy()
    if state.empty:
        return pd.DataFrame()

    state["run"] = run
    state["t"] = numeric(state, "armed_time_sec")
    state["update_time_sec"] = numeric(state, "update_time_sec")
    state["dx_pos_n_m"] = numeric(state, "dx_pos_n_m")
    state["dx_pos_e_m"] = numeric(state, "dx_pos_e_m")
    state["dx_pos_h_m"] = norm2(state["dx_pos_n_m"], state["dx_pos_e_m"])
    state["dx_vel_h_mps"] = norm2(numeric(state, "dx_vel_n_mps"), numeric(state, "dx_vel_e_mps"))
    state["accbias_z_before_mps2"] = numeric(state, "accbias_z_before_mps2")
    state["accbias_z_after_mps2"] = numeric(state, "accbias_z_after_mps2")
    state["accbias_z_delta_mps2"] = state["accbias_z_after_mps2"] - state["accbias_z_before_mps2"]
    state["cov_ba_z_before_m2ps4"] = numeric(state, "cov_ba_z_before_m2ps4")
    state["horizontal_speed_mps"] = numeric(state, "horizontal_speed_mps")
    state["vertical_speed_abs_mps"] = numeric(state, "vertical_speed_mps").abs()
    state["gyro_abs_deg_s"] = numeric(state, "gyro_deg_s").abs()
    state["turning_now"] = truthy(state, "turning_now")
    state["post_turn_context"] = truthy(state, "post_turn_context")
    state["armed_cruise_context"] = truthy(state, "armed_cruise_context")

    state = state.dropna(subset=["update_time_sec", "t"]).sort_values("update_time_sec")
    gnss = load_gnss(run_dir)
    if not gnss.empty:
        state = pd.merge_asof(
            state,
            gnss,
            on="update_time_sec",
            direction="nearest",
            tolerance=0.005,
        )

    groundtruth = load_groundtruth(run, run_dir)
    if groundtruth.empty:
        return state

    gt_now = groundtruth[
        [
            "t",
            "iekf_error_x_m",
            "iekf_error_y_m",
            "iekf_error_xy_m",
            "ekf2_error_xy_m",
        ]
    ].dropna(subset=["t"]).sort_values("t")
    state = state.sort_values("t")
    state = pd.merge_asof(
        state,
        gt_now,
        on="t",
        direction="nearest",
        tolerance=0.30,
    )

    gt_future = gt_now.rename(
        columns={
            "t": "future_t",
            "iekf_error_xy_m": "iekf_error_xy_future2_m",
            "ekf2_error_xy_m": "ekf2_error_xy_future2_m",
        }
    )[["future_t", "iekf_error_xy_future2_m", "ekf2_error_xy_future2_m"]]
    state["future_t"] = state["t"] + 2.0
    state = pd.merge_asof(
        state.sort_values("future_t"),
        gt_future,
        on="future_t",
        direction="nearest",
        tolerance=0.35,
    ).sort_values("t")

    state["meas_resid_h_m"] = norm2(numeric(state, "meas_resid_n_m"), numeric(state, "meas_resid_e_m"))
    err_x = numeric(state, "iekf_error_x_m")
    err_y = numeric(state, "iekf_error_y_m")
    state["antierr_dx_cos"] = cosine(
        numeric(state, "dx_pos_n_m"),
        numeric(state, "dx_pos_e_m"),
        -err_x,
        -err_y,
    )
    state["antierr_resid_cos"] = cosine(
        numeric(state, "meas_resid_n_m"),
        numeric(state, "meas_resid_e_m"),
        -err_x,
        -err_y,
    )
    state["dx_resid_cos"] = cosine(
        numeric(state, "dx_pos_n_m"),
        numeric(state, "dx_pos_e_m"),
        numeric(state, "meas_resid_n_m"),
        numeric(state, "meas_resid_e_m"),
    )
    state["dx_over_resid_h"] = numeric(state, "dx_pos_h_m") / numeric(state, "meas_resid_h_m").where(
        numeric(state, "meas_resid_h_m") > 1e-9
    )
    state["dx_over_error_h"] = numeric(state, "dx_pos_h_m") / numeric(state, "iekf_error_xy_m").where(
        numeric(state, "iekf_error_xy_m") > 1e-9
    )
    state["iekf_error_delta_future2_m"] = (
        numeric(state, "iekf_error_xy_future2_m") - numeric(state, "iekf_error_xy_m")
    )
    state["ekf2_error_delta_future2_m"] = (
        numeric(state, "ekf2_error_xy_future2_m") - numeric(state, "ekf2_error_xy_m")
    )
    return state


def load_all_events() -> pd.DataFrame:
    frames = [load_events(run, run_dir) for run, run_dir in RUN_DIRS.items()]
    frames = [frame for frame in frames if not frame.empty]
    return pd.concat(frames, ignore_index=True) if frames else pd.DataFrame()


def subset_window(frame: pd.DataFrame, start: float, end: float) -> pd.DataFrame:
    t = numeric(frame, "t")
    return frame[(t >= start) & (t < end)]


def basin_class(frac_le_018: float, frac_le_0205: float) -> str:
    if finite(frac_le_0205) and frac_le_0205 >= 0.40:
        return "deep"
    if finite(frac_le_018) and frac_le_018 >= 0.40:
        return "moderate"
    if finite(frac_le_018) and frac_le_018 < 0.10:
        return "clean"
    return "mixed"


def geometry_class(mean_cos: float, bad_frac: float) -> str:
    if finite(mean_cos) and finite(bad_frac) and mean_cos >= 0.25 and bad_frac <= 0.20:
        return "anti_error"
    if finite(mean_cos) and finite(bad_frac) and (mean_cos <= -0.10 or bad_frac >= 0.40):
        return "with_error"
    return "mixed"


def summarize_windows(events: pd.DataFrame) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for run in RUN_DIRS:
        run_events = events[events["run"] == run]
        for label, start, end in WINDOWS:
            sub = subset_window(run_events, start, end)
            prior = subset_window(run_events, 40.0, start) if start > 40.0 else pd.DataFrame()
            ba = numeric(sub, "accbias_z_before_mps2")
            frac018 = float((ba <= -0.18).mean()) if len(ba) else math.nan
            frac0205 = float((ba <= -0.205).mean()) if len(ba) else math.nan
            anti_dx = numeric(sub, "antierr_dx_cos")
            bad_dx_frac = float((anti_dx < -0.25).mean()) if len(anti_dx.dropna()) else math.nan
            good_dx_frac = float((anti_dx > 0.25).mean()) if len(anti_dx.dropna()) else math.nan
            prior_ba = numeric(prior, "accbias_z_before_mps2") if not prior.empty else pd.Series(dtype="float64")
            ekf2_rmse = rmse(numeric(sub, "ekf2_error_xy_m"))
            iekf_rmse = rmse(numeric(sub, "iekf_error_xy_m"))
            anti_dx_mean = mean(anti_dx)
            rows.append(
                {
                    "run": run,
                    "window": label,
                    "rows": len(sub),
                    "iekf_minus_ekf2_rmse_m": iekf_rmse - ekf2_rmse if finite(iekf_rmse) and finite(ekf2_rmse) else math.nan,
                    "iekf_error_xy_mean_m": mean(numeric(sub, "iekf_error_xy_m")),
                    "iekf_error_delta_future2_mean_m": mean(numeric(sub, "iekf_error_delta_future2_m")),
                    "ba_z_mean": mean(ba),
                    "ba_z_le_m0_18_frac": frac018,
                    "ba_z_le_m0_205_frac": frac0205,
                    "basin_class": basin_class(frac018, frac0205),
                    "prior_40_to_start_ba_z_mean": mean(prior_ba),
                    "prior_40_to_start_deep_frac": float((prior_ba <= -0.205).mean()) if len(prior_ba) else math.nan,
                    "meas_resid_h_mean_m": mean(numeric(sub, "meas_resid_h_m")),
                    "dx_pos_h_mean_m": mean(numeric(sub, "dx_pos_h_m")),
                    "dx_over_resid_h_mean": mean(numeric(sub, "dx_over_resid_h")),
                    "dx_over_error_h_mean": mean(numeric(sub, "dx_over_error_h")),
                    "antierr_dx_cos_mean": anti_dx_mean,
                    "antierr_dx_cos_bad_frac": bad_dx_frac,
                    "antierr_dx_cos_good_frac": good_dx_frac,
                    "antierr_resid_cos_mean": mean(numeric(sub, "antierr_resid_cos")),
                    "dx_resid_cos_mean": mean(numeric(sub, "dx_resid_cos")),
                    "geometry_class": geometry_class(anti_dx_mean, bad_dx_frac),
                    "post_turn_frac": float(sub["post_turn_context"].mean()) if len(sub) else math.nan,
                    "armed_cruise_frac": float(sub["armed_cruise_context"].mean()) if len(sub) else math.nan,
                    "turning_frac": float(sub["turning_now"].mean()) if len(sub) else math.nan,
                    "gyro_abs_mean_deg_s": mean(numeric(sub, "gyro_abs_deg_s")),
                    "hspeed_mean_mps": mean(numeric(sub, "horizontal_speed_mps")),
                }
            )
    return rows


def contrast_rows(window_rows: list[dict[str, object]]) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for run, window in CONTRAST_WINDOWS:
        row = next((item for item in window_rows if item["run"] == run and item["window"] == window), None)
        if row:
            rows.append(row)
    return rows


def event_export_rows(events: pd.DataFrame) -> list[dict[str, object]]:
    keep = [
        "run",
        "t",
        "update_time_sec",
        "accbias_z_before_mps2",
        "accbias_z_delta_mps2",
        "meas_resid_h_m",
        "dx_pos_h_m",
        "dx_over_resid_h",
        "iekf_error_xy_m",
        "ekf2_error_xy_m",
        "iekf_error_delta_future2_m",
        "antierr_dx_cos",
        "antierr_resid_cos",
        "dx_resid_cos",
        "post_turn_context",
        "armed_cruise_context",
        "turning_now",
        "horizontal_speed_mps",
        "gyro_abs_deg_s",
    ]
    rows: list[dict[str, object]] = []
    for _, row in events[keep].iterrows():
        rows.append({key: row[key] for key in keep})
    return rows


def table_window(rows: list[dict[str, object]]) -> list[list[object]]:
    keep_windows = {"100-120", "120-140", "140-160", "160-180"}
    return [
        [
            row["run"],
            row["window"],
            row["rows"],
            fmt(row["iekf_minus_ekf2_rmse_m"]),
            row["basin_class"],
            fmt(row["ba_z_mean"]),
            row["geometry_class"],
            fmt(row["antierr_dx_cos_mean"]),
            fmt(row["antierr_dx_cos_bad_frac"]),
            fmt(row["dx_over_resid_h_mean"]),
            fmt(row["iekf_error_delta_future2_mean_m"]),
            fmt(row["post_turn_frac"]),
        ]
        for row in rows
        if row["window"] in keep_windows
    ]


def table_contrast(rows: list[dict[str, object]]) -> list[list[object]]:
    return [
        [
            row["run"],
            row["window"],
            fmt(row["iekf_minus_ekf2_rmse_m"]),
            row["basin_class"],
            fmt(row["ba_z_mean"]),
            row["geometry_class"],
            fmt(row["antierr_dx_cos_mean"]),
            fmt(row["antierr_dx_cos_bad_frac"]),
            fmt(row["antierr_resid_cos_mean"]),
            fmt(row["dx_resid_cos_mean"]),
            fmt(row["meas_resid_h_mean_m"]),
            fmt(row["dx_pos_h_mean_m"]),
            fmt(row["post_turn_frac"]),
            fmt(row["armed_cruise_frac"]),
        ]
        for row in rows
    ]


def write_report(out_dir: Path, window_rows: list[dict[str, object]], contrasts: list[dict[str, object]]) -> None:
    lines = [
        "# shortgen11 event-history selector diagnostic",
        "",
        "Date: 2026-05-11",
        "",
        "Offline-only diagnostic from existing shortgen11 logs. It joins GNSS update residuals to accepted state updates by `update_time_sec`, then joins groundtruth error by `armed_time_sec`.",
        "",
        "Definitions: `antierr dx cos` is positive when the accepted horizontal state correction points opposite the current IEKF groundtruth error; `bad frac` is the fraction with `antierr dx cos < -0.25`. These are offline labels because they use groundtruth error, not an online gate.",
        "",
        "## Window Geometry",
        "",
        markdown_table(
            [
                "run",
                "window",
                "rows",
                "IEKF-EKF2 RMSE",
                "basin",
                "ba_z",
                "geometry",
                "antierr dx cos",
                "bad frac",
                "dx/resid",
                "future2 dErr",
                "post turn",
            ],
            table_window(window_rows),
        ),
        "",
        "## Selector Contrast",
        "",
        markdown_table(
            [
                "run",
                "window",
                "IEKF-EKF2 RMSE",
                "basin",
                "ba_z",
                "geometry",
                "antierr dx cos",
                "bad frac",
                "antierr resid cos",
                "dx resid cos",
                "resid h",
                "dx h",
                "post turn",
                "cruise",
            ],
            table_contrast(contrasts),
        ),
        "",
        "## Interpretation",
        "",
        "- Accepted position corrections follow the GNSS residual almost exactly (`dx resid cos` is near 1.0), so residual-vs-correction direction does not add an independent selector.",
        "- `repeat2` 140-160s is a true failure window, but its accepted corrections point mostly opposite the current IEKF error (`antierr dx cos` around 0.46, bad fraction around 0.10). It is not an instantaneous wrong-direction GNSS correction problem.",
        "- `accbiasz_diag` 160-180s is protected by observed groundtruth performance, yet its online-observable basin and history are still negative/moderate. This is the key counterexample against a simple accbias-z history gate.",
        "- The opt-in apply run reduces the 140-160s miss but does not create a clean selector: 160-180s remains deep-basin and moves toward a small regression, consistent with the earlier scalar-Q action being too broad.",
        "- The current evidence points to a state-basin/recovery-timing issue rather than a single-frame measurement-geometry sign error. A safe mechanism needs segment-dependent timing or recovery-state logic, not only instantaneous residual geometry.",
        "",
        "## Decision",
        "",
        "Do not promote an online selector from this event-history pass. The next offline step should be cross-route selector scoring with features that are available online: bias basin/history, phase flags, motion context, publish/core age, and projection/timing mode labels. Groundtruth error should remain only the scoring label.",
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
        raise SystemExit("no accepted GNSS position state-update rows found")
    window_rows = summarize_windows(events)
    contrasts = contrast_rows(window_rows)
    write_csv(args.out_dir / "event_rows.csv", event_export_rows(events))
    write_csv(args.out_dir / "window_geometry_summary.csv", window_rows)
    write_csv(args.out_dir / "selector_contrast.csv", contrasts)
    write_report(args.out_dir, window_rows, contrasts)
    print(f"wrote: {args.out_dir / 'event_rows.csv'}")
    print(f"wrote: {args.out_dir / 'window_geometry_summary.csv'}")
    print(f"wrote: {args.out_dir / 'selector_contrast.csv'}")
    print(f"wrote: {args.out_dir / 'report.md'}")


if __name__ == "__main__":
    main()
