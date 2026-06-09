#!/usr/bin/env python3
"""Offline PHS5 segment-dependent projection/phase selector counterfactual.

The script uses only existing PHS5 artifacts.  It evaluates simple
online-observable selectors against the current hard constraint: improve
shortgen11 repeat2, especially 140-160 s, while preserving shortgen01/02/03/04
main-window performance.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Iterable


DEFAULT_OUT = (
    Path("/home/yang/kf_gins_ws/artifacts/manual/short_route_generalization_2026-05-07")
    / "phs5_segment_selector_repeat2_2026-05-10"
)


@dataclass(frozen=True)
class RunSpec:
    label: str
    role: str
    path: Path


RUNS = [
    RunSpec(
        "shortgen01_phs5c",
        "development positive",
        Path("/home/yang/kf_gins_ws/artifacts/manual/manual_shortgen01_s_bend_sitl_home_headless_compare_core8_pairlogger_phs5c_publish_core_stamp_bias_m003_20260510_003839"),
    ),
    RunSpec(
        "shortgen02_phs5b",
        "development/control positive",
        Path("/home/yang/kf_gins_ws/artifacts/manual/manual_shortgen02_box_uturn_sitl_home_headless_compare_core8_pairlogger_phs5b_publish_core_stamp_bias_m003_20260510_003144"),
    ),
    RunSpec(
        "shortgen03_phs5a",
        "development/control positive",
        Path("/home/yang/kf_gins_ws/artifacts/manual/manual_shortgen03_figure8_sitl_home_headless_compare_core8_pairlogger_phs5a_publish_core_stamp_bias_m003_20260510_002551"),
    ),
    RunSpec(
        "shortgen04_hld1a_phs5",
        "clean holdout positive with local miss",
        Path("/home/yang/kf_gins_ws/artifacts/manual/manual_shortgen04_ladder_sitl_home_headless_compare_core8_pairlogger_hld1a_phs5_publish_core_stamp_bias_m003_20260510_005528"),
    ),
    RunSpec(
        "shortgen11_repeat2",
        "clean negative/generalization warning",
        Path("/home/yang/kf_gins_ws/artifacts/manual/manual_shortgen11_high_clearance_ladder_sitl_home_headless_compare_core8_pairlogger_phs5_repeat2_cleancheck_20260510_155253"),
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
    lines = [
        "| " + " | ".join(headers) + " |",
        "| " + " | ".join("---" for _ in headers) + " |",
    ]
    lines.extend("| " + " | ".join(str(item) for item in row) + " |" for row in rows)
    return "\n".join(lines)


def nearest(
    rows: list[dict[str, object]],
    times: list[float],
    t: float,
    max_dt_sec: float,
) -> dict[str, object] | None:
    if not rows or not finite(t):
        return None
    idx = bisect.bisect_left(times, t)
    candidates: list[tuple[float, dict[str, object]]] = []
    for pos in (idx - 1, idx):
        if 0 <= pos < len(rows):
            candidates.append((abs(times[pos] - t), rows[pos]))
    if not candidates:
        return None
    dt, row = min(candidates, key=lambda item: item[0])
    return row if dt <= max_dt_sec else None


def mode_dir(run_dir: Path) -> Path:
    direct = run_dir / "offline_groundtruth_projection_modes"
    if direct.exists():
        return direct
    return run_dir / "offline_groundtruth_projection_modes_phs_check"


def load_rows(spec: RunSpec) -> list[dict[str, object]]:
    modes = mode_dir(spec.path)
    raw_rows = read_csv(modes / "raw_wgs84_enu" / "groundtruth_joined.csv")
    px4_rows: list[dict[str, object]] = read_csv(modes / "px4_sphere_anisotropic" / "groundtruth_joined.csv")
    px4_rows.sort(key=lambda row: to_float(row.get("pair_ros_time_sec")))
    px4_times = [to_float(row.get("pair_ros_time_sec")) for row in px4_rows]

    state_rows: list[dict[str, object]] = read_csv(spec.path / "state_publish_debug.csv")
    state_rows.sort(key=lambda row: to_float(row.get("ros_time_sec")))
    state_times = [to_float(row.get("ros_time_sec")) for row in state_rows]

    out: list[dict[str, object]] = []
    for raw in raw_rows:
        t_arm = to_float(raw.get("time_since_arm_sec"))
        if not (40.0 <= t_arm < 180.0):
            continue
        if to_float(raw.get("mavros_armed"), 0.0) < 0.5:
            continue
        pair_t = to_float(raw.get("pair_ros_time_sec"))
        px4 = nearest(px4_rows, px4_times, pair_t, 0.03)
        state = nearest(state_rows, state_times, pair_t, 0.08)
        if px4 is None or state is None:
            continue

        core_offset = to_float(state.get("publish_core_to_ros_offset_sec"), 0.0)
        state_ros = to_float(state.get("ros_time_sec"))
        core_ros = to_float(state.get("last_core_time_sec")) + core_offset
        gnss_source_age = state_ros - to_float(state.get("last_gnss_source_time_sec"))
        stamp_lag = -to_float(state.get("publish_stamp_selected_minus_now_sec"))

        out.append(
            {
                "run": spec.label,
                "role": spec.role,
                "time_since_arm_sec": t_arm,
                "mavros_mode": raw.get("mavros_mode", ""),
                "turning_now": to_float(raw.get("turning_now"), 0.0),
                "post_turn_context": to_float(raw.get("post_turn_context"), 0.0),
                "armed_cruise_context": to_float(raw.get("armed_cruise_context"), 0.0),
                "speed_mps": to_float(raw.get("horizontal_speed_mps")),
                "gyro_deg_s": to_float(raw.get("gyro_deg_s")),
                "raw_iekf_x_m": to_float(raw.get("iekf_normalized_aligned_x_m")),
                "raw_iekf_y_m": to_float(raw.get("iekf_normalized_aligned_y_m")),
                "px4_iekf_x_m": to_float(px4.get("iekf_normalized_aligned_x_m")),
                "px4_iekf_y_m": to_float(px4.get("iekf_normalized_aligned_y_m")),
                "gt_x_m": to_float(raw.get("gt_x_m")),
                "gt_y_m": to_float(raw.get("gt_y_m")),
                "raw_iekf_error_m": to_float(raw.get("iekf_error_xy_m")),
                "px4_iekf_error_m": to_float(px4.get("iekf_error_xy_m")),
                "ekf2_error_m": to_float(raw.get("ekf2_error_xy_m")),
                "stamp_lag_sec": stamp_lag,
                "core_age_sec": state_ros - core_ros,
                "gnss_source_age_sec": gnss_source_age,
                "state_core_gnss_diff_h_m": to_float(state.get("core_gnss_diff_h_m")),
                "core_velocity_e_mps": to_float(state.get("core_velocity_vE_mps")),
                "core_velocity_n_mps": to_float(state.get("core_velocity_vN_mps")),
            }
        )
    return out


@dataclass(frozen=True)
class Selector:
    name: str
    stamp_lag_min: float = -math.inf
    core_age_min: float = -math.inf
    gnss_source_age_max: float = math.inf
    core_gnss_min: float = -math.inf
    core_gnss_max: float = math.inf
    speed_min: float = -math.inf

    def active(self, row: dict[str, object]) -> bool:
        return (
            to_float(row.get("stamp_lag_sec")) >= self.stamp_lag_min
            and to_float(row.get("core_age_sec")) >= self.core_age_min
            and to_float(row.get("gnss_source_age_sec")) <= self.gnss_source_age_max
            and self.core_gnss_min <= to_float(row.get("state_core_gnss_diff_h_m")) <= self.core_gnss_max
            and to_float(row.get("speed_mps")) >= self.speed_min
        )


@dataclass(frozen=True)
class Candidate:
    name: str
    action: str
    selector: Selector
    tau_sec: float = 0.0


def projection_xy(row: dict[str, object], active: bool) -> tuple[float, float]:
    if active:
        return to_float(row.get("px4_iekf_x_m")), to_float(row.get("px4_iekf_y_m"))
    return to_float(row.get("raw_iekf_x_m")), to_float(row.get("raw_iekf_y_m"))


def deskew_xy(row: dict[str, object], active: bool, tau_sec: float) -> tuple[float, float]:
    x = to_float(row.get("raw_iekf_x_m"))
    y = to_float(row.get("raw_iekf_y_m"))
    if active:
        x -= tau_sec * to_float(row.get("core_velocity_e_mps"), 0.0)
        y -= tau_sec * to_float(row.get("core_velocity_n_mps"), 0.0)
    return x, y


def candidate_error(row: dict[str, object], candidate: Candidate) -> tuple[float, bool]:
    active = candidate.selector.active(row)
    if candidate.action == "projection_px4_when_active":
        x, y = projection_xy(row, active)
    elif candidate.action == "velocity_deskew_when_active":
        x, y = deskew_xy(row, active, candidate.tau_sec)
    else:
        raise ValueError(candidate.action)
    return math.hypot(x - to_float(row.get("gt_x_m")), y - to_float(row.get("gt_y_m"))), active


def in_window(row: dict[str, object], start: float, end: float) -> bool:
    t = to_float(row.get("time_since_arm_sec"))
    return start <= t < end


def summarize_subset(
    rows: list[dict[str, object]],
    candidate: Candidate | None,
    run_label: str,
    window_label: str,
    start: float,
    end: float,
) -> dict[str, object]:
    subset = [row for row in rows if row.get("run") == run_label and in_window(row, start, end)]
    if candidate is None:
        cf_errors = [to_float(row.get("raw_iekf_error_m")) for row in subset]
        active_frac = 0.0
    else:
        evaluated = [candidate_error(row, candidate) for row in subset]
        cf_errors = [item[0] for item in evaluated]
        active_frac = mean(1.0 if item[1] else 0.0 for item in evaluated)
    raw_errors = [to_float(row.get("raw_iekf_error_m")) for row in subset]
    px4_errors = [to_float(row.get("px4_iekf_error_m")) for row in subset]
    ekf2_errors = [to_float(row.get("ekf2_error_m")) for row in subset]
    return {
        "run": run_label,
        "window": window_label,
        "rows": len(subset),
        "active_frac": active_frac,
        "raw_iekf_rmse_m": rmse(raw_errors),
        "global_px4_rmse_m": rmse(px4_errors),
        "candidate_rmse_m": rmse(cf_errors),
        "ekf2_rmse_m": rmse(ekf2_errors),
        "candidate_minus_raw_m": rmse(cf_errors) - rmse(raw_errors),
        "candidate_minus_ekf2_m": rmse(cf_errors) - rmse(ekf2_errors),
    }


def build_candidates() -> list[Candidate]:
    candidates: list[Candidate] = [
        Candidate(
            "global_px4_projection",
            "projection_px4_when_active",
            Selector("always"),
        )
    ]
    for lag in (0.02, 0.025, 0.03, 0.035, 0.04, 0.045, 0.05):
        candidates.append(
            Candidate(
                f"proj_stamp_lag_ge_{lag:.3f}",
                "projection_px4_when_active",
                Selector(f"stamp_lag_ge_{lag:.3f}", stamp_lag_min=lag),
            )
        )
        for speed in (2.0, 3.0):
            candidates.append(
                Candidate(
                    f"proj_stamp_lag_ge_{lag:.3f}_speed_ge_{speed:.1f}",
                    "projection_px4_when_active",
                    Selector(f"stamp_lag_ge_{lag:.3f}_speed_ge_{speed:.1f}", stamp_lag_min=lag, speed_min=speed),
                )
            )
    for tau in (0.02, 0.04, 0.06, 0.08):
        for lag in (0.03, 0.035, 0.04, 0.05):
            candidates.append(
                Candidate(
                    f"deskew_tau_{tau:.2f}_stamp_lag_ge_{lag:.3f}",
                    "velocity_deskew_when_active",
                    Selector(f"stamp_lag_ge_{lag:.3f}", stamp_lag_min=lag),
                    tau_sec=tau,
                )
            )
    return candidates


def candidate_summary(rows: list[dict[str, object]], candidate: Candidate) -> dict[str, object]:
    by_run_window: dict[tuple[str, str], dict[str, object]] = {}
    for run in RUNS:
        for window_label, start, end in WINDOWS:
            by_run_window[(run.label, window_label)] = summarize_subset(rows, candidate, run.label, window_label, start, end)

    positives = [run.label for run in RUNS if run.label != "shortgen11_repeat2"]
    positive_regs = [
        to_float(by_run_window[(label, "main_40_180")]["candidate_minus_raw_m"])
        for label in positives
    ]
    positive_deltas = [
        to_float(by_run_window[(label, "main_40_180")]["candidate_minus_ekf2_m"])
        for label in positives
    ]
    sg11_main = by_run_window[("shortgen11_repeat2", "main_40_180")]
    sg11_140 = by_run_window[("shortgen11_repeat2", "140_160")]
    sg11_60100 = by_run_window[("shortgen11_repeat2", "60_100")]
    return {
        "candidate": candidate.name,
        "action": candidate.action,
        "selector": candidate.selector.name,
        "tau_sec": candidate.tau_sec,
        "sg11_main_raw_rmse_m": sg11_main["raw_iekf_rmse_m"],
        "sg11_main_candidate_rmse_m": sg11_main["candidate_rmse_m"],
        "sg11_main_ekf2_rmse_m": sg11_main["ekf2_rmse_m"],
        "sg11_main_improve_m": to_float(sg11_main["raw_iekf_rmse_m"]) - to_float(sg11_main["candidate_rmse_m"]),
        "sg11_main_candidate_minus_ekf2_m": sg11_main["candidate_minus_ekf2_m"],
        "sg11_140_160_raw_rmse_m": sg11_140["raw_iekf_rmse_m"],
        "sg11_140_160_candidate_rmse_m": sg11_140["candidate_rmse_m"],
        "sg11_140_160_ekf2_rmse_m": sg11_140["ekf2_rmse_m"],
        "sg11_140_160_improve_m": to_float(sg11_140["raw_iekf_rmse_m"]) - to_float(sg11_140["candidate_rmse_m"]),
        "sg11_140_160_candidate_minus_ekf2_m": sg11_140["candidate_minus_ekf2_m"],
        "sg11_140_160_active_frac": sg11_140["active_frac"],
        "sg11_60_100_candidate_rmse_m": sg11_60100["candidate_rmse_m"],
        "sg11_60_100_ekf2_rmse_m": sg11_60100["ekf2_rmse_m"],
        "positive_max_main_regress_m": max(positive_regs),
        "positive_mean_main_regress_m": mean(positive_regs),
        "positive_max_candidate_minus_ekf2_m": max(positive_deltas),
        "positive_all_protected": int(max(positive_regs) <= 0.02 and max(positive_deltas) < 0.02),
    }


def window_metrics(rows: list[dict[str, object]], candidate: Candidate) -> list[dict[str, object]]:
    out: list[dict[str, object]] = []
    for run in RUNS:
        for window_label, start, end in WINDOWS:
            row = summarize_subset(rows, candidate, run.label, window_label, start, end)
            row["candidate"] = candidate.name
            row["action"] = candidate.action
            out.append(row)
    return out


def choose_top(summary_rows: list[dict[str, object]], protected: bool) -> list[dict[str, object]]:
    rows = [
        row for row in summary_rows
        if (not protected or to_float(row.get("positive_all_protected"), 0.0) > 0.5)
    ]
    rows.sort(
        key=lambda row: (
            to_float(row.get("sg11_140_160_improve_m")),
            to_float(row.get("sg11_main_improve_m")),
        ),
        reverse=True,
    )
    return rows[:8]


def write_report(out_dir: Path, summary_rows: list[dict[str, object]], selected_metrics: list[dict[str, object]]) -> None:
    protected_top = choose_top(summary_rows, protected=True)
    relaxed_top = choose_top(summary_rows, protected=False)
    strict_best = protected_top[0] if protected_top else None
    loose_best = next(
        (
            row for row in relaxed_top
            if row["candidate"] != "global_px4_projection"
            and (strict_best is None or row["candidate"] != strict_best["candidate"])
        ),
        None,
    )
    global_px4 = next(row for row in summary_rows if row["candidate"] == "global_px4_projection")

    def row_table(rows: list[dict[str, object]]) -> list[list[object]]:
        return [
            [
                row["candidate"],
                row["action"],
                fmt(row["sg11_main_candidate_rmse_m"]),
                fmt(row["sg11_main_candidate_minus_ekf2_m"]),
                fmt(row["sg11_140_160_candidate_rmse_m"]),
                fmt(row["sg11_140_160_candidate_minus_ekf2_m"]),
                fmt(row["positive_max_main_regress_m"]),
                row["positive_all_protected"],
            ]
            for row in rows
        ]

    key_rows: list[dict[str, object]] = []
    seen_candidates: set[str] = set()
    for row in (strict_best, loose_best, global_px4):
        if row is None or str(row["candidate"]) in seen_candidates:
            continue
        seen_candidates.add(str(row["candidate"]))
        key_rows.append(row)

    display_metrics = [
        row for row in selected_metrics
        if (
            (row["run"] == "shortgen11_repeat2" and row["window"] in {"main_40_180", "60_100", "140_160", "160_180"})
            or (row["run"] != "shortgen11_repeat2" and row["window"] == "main_40_180")
        )
    ]
    selected_table = [
        [
            row["candidate"],
            row["run"],
            row["window"],
            fmt(row["active_frac"], 3),
            fmt(row["raw_iekf_rmse_m"]),
            fmt(row["candidate_rmse_m"]),
            fmt(row["ekf2_rmse_m"]),
            fmt(row["candidate_minus_raw_m"]),
            fmt(row["candidate_minus_ekf2_m"]),
        ]
        for row in display_metrics
    ]

    lines = [
        "# PHS5 segment-dependent selector counterfactual on repeat2",
        "",
        "Date: 2026-05-10",
        "",
        "Pure offline analysis using existing PHS5 logs only. No flight, rebuild, PX4, Gazebo, MAVROS, QGC, RViz2, PlotJuggler, or estimator code change was run.",
        "",
        "## Scope",
        "",
        "- Runs: shortgen01/02/03 PHS5 development/control, shortgen04 clean holdout positive, and shortgen11 repeat2 clean negative.",
        "- Actions tested: selector-controlled PX4-sphere projection scoring and selector-controlled backward output de-skew using core velocity.",
        "- Selector inputs are online-observable log fields: publish stamp lag, core age, GNSS source age, core/GNSS horizontal difference, speed, and context-independent thresholds.",
        "- The selector is not allowed to use route name, waypoint id, absolute hardcoded failure window, or groundtruth error.",
        "",
        "## Acceptance",
        "",
        "- Protect shortgen01/02/03/04 `main_40_180`: max regression <= 0.02 m and candidate still not worse than EKF2 by >= 0.02 m.",
        "- Reduce shortgen11 repeat2 `main_40_180` and specifically repair `140_160` enough to beat/tie EKF2.",
        "",
        "## Key Candidates",
        "",
        markdown_table(
            [
                "candidate",
                "action",
                "sg11 main RMSE",
                "sg11 main vs EKF2",
                "sg11 140-160 RMSE",
                "sg11 140-160 vs EKF2",
                "positive max regress",
                "protected",
            ],
            row_table(key_rows),
        ),
        "",
        "## Top Strict Candidates",
        "",
        markdown_table(
            [
                "candidate",
                "action",
                "sg11 main RMSE",
                "sg11 main vs EKF2",
                "sg11 140-160 RMSE",
                "sg11 140-160 vs EKF2",
                "positive max regress",
                "protected",
            ],
            row_table(protected_top[:6]),
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
                "raw IEKF",
                "candidate",
                "EKF2",
                "candidate-raw",
                "candidate-EKF2",
            ],
            selected_table,
        ),
        "",
        "## Readout",
        "",
    ]
    if strict_best is not None:
        lines.extend(
            [
                f"- Best strict candidate is `{strict_best['candidate']}`. It preserves the positive/control routes and improves shortgen11 repeat2 `main_40_180` from `{fmt(strict_best['sg11_main_raw_rmse_m'])}` to `{fmt(strict_best['sg11_main_candidate_rmse_m'])}` m.",
                f"- The same strict candidate does **not** repair the hard local failure: shortgen11 repeat2 `140_160` remains `{fmt(strict_best['sg11_140_160_candidate_rmse_m'])}` m vs EKF2 `{fmt(strict_best['sg11_140_160_ekf2_rmse_m'])}` m.",
            ]
        )
    lines.extend(
        [
            f"- Global PX4-sphere projection would repair shortgen11 `140_160` much more strongly, but its positive-route max main-window regression is `{fmt(global_px4['positive_max_main_regress_m'])}` m, so it remains rejected as a default.",
            "- Selector-controlled core-velocity de-skew did not beat the projection-lag candidates under the strict preservation gate; it still damages good shortgen11 windows when made strong enough to matter.",
            "- Current simple online-observable selectors are therefore insufficient for the full hard requirement. The next offline iteration needs a richer per-segment signal, likely using measurement-frame geometry and update direction rather than only stamp lag/core age.",
            "",
            "Generated files:",
            f"- `{out_dir / 'selector_row_metrics.csv'}`",
            f"- `{out_dir / 'candidate_summary.csv'}`",
            f"- `{out_dir / 'selected_window_metrics.csv'}`",
        ]
    )
    (out_dir / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out-dir", default=str(DEFAULT_OUT))
    args = parser.parse_args()

    out_dir = Path(args.out_dir)
    rows: list[dict[str, object]] = []
    for spec in RUNS:
        rows.extend(load_rows(spec))
    write_csv(out_dir / "selector_row_metrics.csv", rows)

    candidates = build_candidates()
    summary_rows = [candidate_summary(rows, candidate) for candidate in candidates]
    write_csv(out_dir / "candidate_summary.csv", summary_rows)

    protected_top = choose_top(summary_rows, protected=True)
    relaxed_top = choose_top(summary_rows, protected=False)
    loose_best = next(
        (
            row for row in relaxed_top
            if row["candidate"] != "global_px4_projection"
            and (not protected_top or row["candidate"] != protected_top[0]["candidate"])
        ),
        None,
    )
    selected_names = {"global_px4_projection", *(row["candidate"] for row in protected_top[:2])}
    if loose_best is not None:
        selected_names.add(str(loose_best["candidate"]))
    selected_candidates = [candidate for candidate in candidates if candidate.name in selected_names]
    metrics: list[dict[str, object]] = []
    for candidate in selected_candidates:
        metrics.extend(window_metrics(rows, candidate))
    write_csv(out_dir / "selected_window_metrics.csv", metrics)
    write_report(out_dir, summary_rows, metrics)
    print(f"wrote: {out_dir / 'report.md'}")


if __name__ == "__main__":
    main()
