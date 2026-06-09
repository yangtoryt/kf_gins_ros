#!/usr/bin/env python3
"""Diagnose IEKF publish-phase holds behind shortgen02 projected residuals."""

from __future__ import annotations

import argparse
import bisect
import csv
import math
from pathlib import Path

from offline_cross_route_failure_atlas import (
    finite,
    markdown_table,
    mean,
    percentile,
    to_float,
    write_csv,
)
from offline_shortgen02_projection_residual_diagnostic import (
    SUMMARY_WINDOWS,
    context_label,
    fmt,
    in_window,
    is_armed,
    load_projection_rows,
    nearest,
    norm2,
    read_csv,
)


HOLD_MIN_PUBLISH_DT_SEC = 0.08
HOLD_MAX_CORE_DT_SEC = 0.02
HOLD_MAX_POSITION_DELTA_M = 0.05
HOLD_MIN_SPEED_MPS = 1.0


def load_pairs(run_dir: Path) -> tuple[list[dict[str, object]], list[float]]:
    rows: list[dict[str, object]] = read_csv(run_dir / "ekf_iekf_pairs.csv")
    rows.sort(key=lambda row: to_float(row.get("ros_time_sec")))
    return rows, [to_float(row.get("ros_time_sec")) for row in rows]


def load_state_publish(run_dir: Path) -> tuple[list[dict[str, object]], list[float]]:
    rows: list[dict[str, object]] = read_csv(run_dir / "state_publish_debug.csv")
    rows.sort(key=lambda row: to_float(row.get("odom_stamp_sec")))
    return rows, [to_float(row.get("odom_stamp_sec")) for row in rows]


def nearest_state_by_stamp(
    rows: list[dict[str, object]],
    stamps: list[float],
    stamp: float,
) -> tuple[int | None, dict[str, object] | None, float]:
    if not rows or not finite(stamp):
        return None, None, math.nan
    idx = bisect.bisect_left(stamps, stamp)
    candidates: list[tuple[float, int, dict[str, object]]] = []
    for pos in (idx - 2, idx - 1, idx, idx + 1):
        if 0 <= pos < len(rows):
            candidates.append((abs(stamps[pos] - stamp), pos, rows[pos]))
    if not candidates:
        return None, None, math.nan
    dt, pos, row = min(candidates, key=lambda item: item[0])
    return (pos, row, dt) if dt <= 0.006 else (None, None, dt)


def previous_distinct_stamp_row(rows: list[dict[str, object]], index: int) -> dict[str, object] | None:
    if index <= 0:
        return None
    stamp = to_float(rows[index].get("odom_stamp_sec"))
    pos = index - 1
    while pos >= 0:
        prev_stamp = to_float(rows[pos].get("odom_stamp_sec"))
        if abs(prev_stamp - stamp) > 1e-9:
            return rows[pos]
        pos -= 1
    return None


def build_publish_phase_rows(run_dir: Path) -> list[dict[str, object]]:
    px4_rows, _ = load_projection_rows(run_dir, "px4_sphere_anisotropic")
    current_rows, current_times = load_projection_rows(run_dir, "gps_fit_all")
    pair_rows, pair_times = load_pairs(run_dir)
    state_rows, state_stamps = load_state_publish(run_dir)

    out: list[dict[str, object]] = []
    for px4 in px4_rows:
        if not is_armed(px4) or not in_window(px4, 120.0, 180.0):
            continue
        pair_ros_t = to_float(px4.get("pair_ros_time_sec"))
        current, current_dt = nearest(current_rows, current_times, pair_ros_t, 0.025)
        pair, pair_dt = nearest(pair_rows, pair_times, pair_ros_t, 0.025)
        if current is None or pair is None:
            continue
        state_index, state, state_join_dt = nearest_state_by_stamp(
            state_rows,
            state_stamps,
            to_float(pair.get("iekf_stamp_sec")),
        )
        if state is None or state_index is None:
            continue
        prev_state = previous_distinct_stamp_row(state_rows, state_index)
        if prev_state is None:
            continue

        publish_dt = to_float(state.get("ros_time_sec")) - to_float(prev_state.get("ros_time_sec"))
        core_dt = to_float(state.get("last_core_time_sec")) - to_float(prev_state.get("last_core_time_sec"))
        pos_delta_h = norm2(
            to_float(state.get("published_enu_e_m")) - to_float(prev_state.get("published_enu_e_m")),
            to_float(state.get("published_enu_n_m")) - to_float(prev_state.get("published_enu_n_m")),
        )
        speed = to_float(state.get("mavros_horizontal_speed_mps"))
        expected_motion_h = speed * publish_dt if finite(speed) and finite(publish_dt) else math.nan
        hold_like = (
            publish_dt >= HOLD_MIN_PUBLISH_DT_SEC
            and core_dt <= HOLD_MAX_CORE_DT_SEC
            and pos_delta_h <= HOLD_MAX_POSITION_DELTA_M
            and speed >= HOLD_MIN_SPEED_MPS
        )
        gnss_update_changed = (
            abs(to_float(state.get("last_gnss_update_time_sec")) - to_float(prev_state.get("last_gnss_update_time_sec")))
            > 0.01
        )

        row: dict[str, object] = {
            "time_since_arm_sec": px4.get("time_since_arm_sec"),
            "pair_ros_time_sec": pair_ros_t,
            "pair_ekf2_stamp_sec": pair.get("ekf2_stamp_sec"),
            "pair_iekf_stamp_sec": pair.get("iekf_stamp_sec"),
            "pair_join_dt_sec": pair_dt,
            "current_join_dt_sec": current_dt,
            "state_join_dt_sec": state_join_dt,
            "context": context_label(px4),
            "ekf2_error_h_m": px4.get("ekf2_error_xy_m"),
            "current_iekf_error_h_m": current.get("iekf_error_xy_m"),
            "px4_iekf_error_h_m": px4.get("iekf_error_xy_m"),
            "px4_iekf_error_x_m": px4.get("iekf_error_x_m"),
            "px4_iekf_error_y_m": px4.get("iekf_error_y_m"),
            "sync_dt_ms": pair.get("sync_dt_ms"),
            "pair_ros_minus_iekf_stamp_sec": pair_ros_t - to_float(pair.get("iekf_stamp_sec")),
            "iekf_stamp_minus_ekf2_stamp_sec": to_float(pair.get("iekf_stamp_sec")) - to_float(pair.get("ekf2_stamp_sec")),
            "state_ros_time_sec": state.get("ros_time_sec"),
            "state_odom_stamp_sec": state.get("odom_stamp_sec"),
            "state_last_core_time_sec": state.get("last_core_time_sec"),
            "state_last_gnss_update_time_sec": state.get("last_gnss_update_time_sec"),
            "state_prev_ros_time_sec": prev_state.get("ros_time_sec"),
            "state_prev_odom_stamp_sec": prev_state.get("odom_stamp_sec"),
            "state_prev_last_core_time_sec": prev_state.get("last_core_time_sec"),
            "state_prev_last_gnss_update_time_sec": prev_state.get("last_gnss_update_time_sec"),
            "state_publish_dt_sec": publish_dt,
            "state_core_dt_sec": core_dt,
            "state_position_delta_h_m": pos_delta_h,
            "state_expected_motion_h_m": expected_motion_h,
            "state_motion_deficit_h_m": expected_motion_h - pos_delta_h if finite(expected_motion_h) else math.nan,
            "state_horizontal_speed_mps": speed,
            "state_core_gnss_diff_h_m": state.get("core_gnss_diff_h_m"),
            "state_gnss_update_changed": int(gnss_update_changed),
            "publish_hold_like": int(hold_like),
        }
        out.append(row)
    out.sort(key=lambda row: to_float(row.get("time_since_arm_sec")))
    return out


def summarize_group(label: str, rows: list[dict[str, object]]) -> dict[str, object]:
    px4_vals = [to_float(row.get("px4_iekf_error_h_m")) for row in rows]
    ekf2_vals = [to_float(row.get("ekf2_error_h_m")) for row in rows]
    current_vals = [to_float(row.get("current_iekf_error_h_m")) for row in rows]
    return {
        "group": label,
        "rows": len(rows),
        "hold_like_rows": sum(1 for row in rows if to_float(row.get("publish_hold_like"), 0.0) > 0.5),
        "hold_like_frac": mean(1.0 if to_float(row.get("publish_hold_like"), 0.0) > 0.5 else 0.0 for row in rows),
        "ekf2_mean_m": mean(ekf2_vals),
        "current_iekf_mean_m": mean(current_vals),
        "px4_iekf_mean_m": mean(px4_vals),
        "px4_iekf_p95_m": percentile(px4_vals, 95.0),
        "px4_frac_over_0p3": mean(1.0 if item > 0.3 else 0.0 for item in px4_vals),
        "publish_dt_mean_sec": mean(to_float(row.get("state_publish_dt_sec")) for row in rows),
        "core_dt_mean_sec": mean(to_float(row.get("state_core_dt_sec")) for row in rows),
        "position_delta_mean_m": mean(to_float(row.get("state_position_delta_h_m")) for row in rows),
        "motion_deficit_mean_m": mean(to_float(row.get("state_motion_deficit_h_m")) for row in rows),
    }


def build_hold_summary(rows: list[dict[str, object]]) -> list[dict[str, object]]:
    hold_rows = [row for row in rows if to_float(row.get("publish_hold_like"), 0.0) > 0.5]
    nonhold_rows = [row for row in rows if to_float(row.get("publish_hold_like"), 0.0) <= 0.5]
    high_rows = [row for row in rows if to_float(row.get("px4_iekf_error_h_m")) > 0.3]
    return [
        summarize_group("all_120_180", rows),
        summarize_group("publish_hold_like", hold_rows),
        summarize_group("non_hold", nonhold_rows),
        summarize_group("px4_error_gt_0p3", high_rows),
        summarize_group("px4_error_gt_0p3_and_hold", [row for row in high_rows if row in hold_rows]),
    ]


def build_window_summary(rows: list[dict[str, object]]) -> list[dict[str, object]]:
    out: list[dict[str, object]] = []
    for label, start, end in SUMMARY_WINDOWS:
        subset = [row for row in rows if in_window(row, start, end)]
        out.append(summarize_group(label, subset))
    return out


def build_top_rows(rows: list[dict[str, object]], top_n: int) -> list[dict[str, object]]:
    return sorted(rows, key=lambda row: to_float(row.get("px4_iekf_error_h_m")), reverse=True)[:top_n]


def build_target_hold_rows(run_dir: Path, publish_rows: list[dict[str, object]]) -> list[dict[str, object]]:
    target_path = run_dir / "shortgen02_projection_residual_diag" / "original_target_high_rows_after_projection.csv"
    if not target_path.exists():
        return []
    sorted_publish = sorted(publish_rows, key=lambda row: to_float(row.get("time_since_arm_sec")))
    publish_times = [to_float(row.get("time_since_arm_sec")) for row in sorted_publish]
    out: list[dict[str, object]] = []
    for target in read_csv(target_path):
        publish, publish_dt = nearest(sorted_publish, publish_times, to_float(target.get("time_since_arm_sec")), 0.16)
        if publish is None:
            continue
        out.append({
            "time_since_arm_sec": target.get("time_since_arm_sec"),
            "context": target.get("context"),
            "ekf2_error_h_m": target.get("ekf2_error_h_m"),
            "current_iekf_error_h_m": target.get("current_iekf_error_h_m"),
            "px4_iekf_error_h_m": target.get("px4_iekf_error_h_m"),
            "publish_join_dt_sec": publish_dt,
            "publish_hold_like": publish.get("publish_hold_like"),
            "state_publish_dt_sec": publish.get("state_publish_dt_sec"),
            "state_core_dt_sec": publish.get("state_core_dt_sec"),
            "state_position_delta_h_m": publish.get("state_position_delta_h_m"),
            "state_motion_deficit_h_m": publish.get("state_motion_deficit_h_m"),
        })
    return out


def build_target_hold_summary(target_rows: list[dict[str, object]]) -> list[dict[str, object]]:
    if not target_rows:
        return []
    hold_rows = [row for row in target_rows if to_float(row.get("publish_hold_like"), 0.0) > 0.5]
    nonhold_rows = [row for row in target_rows if to_float(row.get("publish_hold_like"), 0.0) <= 0.5]
    return [
        summarize_group("target_high_all", target_rows),
        summarize_group("target_high_hold", hold_rows),
        summarize_group("target_high_non_hold", nonhold_rows),
    ]


def write_report(
    out_dir: Path,
    hold_summary: list[dict[str, object]],
    window_summary: list[dict[str, object]],
    top_rows: list[dict[str, object]],
    target_hold_rows: list[dict[str, object]],
    target_hold_summary: list[dict[str, object]],
) -> None:
    hold_table = [
        [
            row["group"],
            row["rows"],
            row["hold_like_rows"],
            fmt(row["hold_like_frac"], 3),
            fmt(row["ekf2_mean_m"]),
            fmt(row["current_iekf_mean_m"]),
            fmt(row["px4_iekf_mean_m"]),
            fmt(row["px4_iekf_p95_m"]),
            fmt(row["px4_frac_over_0p3"], 3),
            fmt(row["core_dt_mean_sec"], 4),
            fmt(row["position_delta_mean_m"]),
            fmt(row["motion_deficit_mean_m"]),
        ]
        for row in hold_summary
    ]
    window_table = [
        [
            row["group"],
            row["rows"],
            row["hold_like_rows"],
            fmt(row["hold_like_frac"], 3),
            fmt(row["ekf2_mean_m"]),
            fmt(row["px4_iekf_mean_m"]),
            fmt(row["px4_iekf_p95_m"]),
            fmt(row["px4_frac_over_0p3"], 3),
        ]
        for row in window_summary
    ]
    top_table = [
        [
            fmt(row["time_since_arm_sec"], 1),
            row["context"],
            fmt(row["ekf2_error_h_m"]),
            fmt(row["px4_iekf_error_h_m"]),
            row["publish_hold_like"],
            fmt(row["pair_ros_minus_iekf_stamp_sec"], 3),
            fmt(row["iekf_stamp_minus_ekf2_stamp_sec"], 3),
            fmt(row["state_publish_dt_sec"], 3),
            fmt(row["state_core_dt_sec"], 4),
            fmt(row["state_position_delta_h_m"]),
            fmt(row["state_expected_motion_h_m"]),
            fmt(row["state_motion_deficit_h_m"]),
        ]
        for row in top_rows[:20]
    ]
    target_summary_table = [
        [
            row["group"],
            row["rows"],
            row["hold_like_rows"],
            fmt(row["ekf2_mean_m"]),
            fmt(row["current_iekf_mean_m"]),
            fmt(row["px4_iekf_mean_m"]),
            fmt(row["px4_frac_over_0p3"], 3),
            fmt(row["motion_deficit_mean_m"]),
        ]
        for row in target_hold_summary
    ]
    target_table = [
        [
            fmt(row["time_since_arm_sec"], 1),
            row["context"],
            fmt(row["ekf2_error_h_m"]),
            fmt(row["px4_iekf_error_h_m"]),
            row["publish_hold_like"],
            fmt(row["state_publish_dt_sec"], 3),
            fmt(row["state_core_dt_sec"], 4),
            fmt(row["state_position_delta_h_m"]),
            fmt(row["state_motion_deficit_h_m"]),
        ]
        for row in target_hold_rows
    ]
    focus_rows = [
        row for row in top_rows
        if 151.0 <= to_float(row.get("time_since_arm_sec")) < 154.5
    ]
    focus_table = [
        [
            fmt(row["time_since_arm_sec"], 1),
            row["context"],
            fmt(row["px4_iekf_error_h_m"]),
            row["publish_hold_like"],
            fmt(row["state_publish_dt_sec"], 3),
            fmt(row["state_core_dt_sec"], 4),
            fmt(row["state_position_delta_h_m"]),
            fmt(row["state_motion_deficit_h_m"]),
            fmt(row["px4_iekf_error_x_m"]),
            fmt(row["px4_iekf_error_y_m"]),
        ]
        for row in sorted(focus_rows, key=lambda item: to_float(item.get("time_since_arm_sec")))
    ]

    hold_row = next(row for row in hold_summary if row["group"] == "publish_hold_like")
    nonhold_row = next(row for row in hold_summary if row["group"] == "non_hold")
    high_hold = next(row for row in hold_summary if row["group"] == "px4_error_gt_0p3_and_hold")
    high_all = next(row for row in hold_summary if row["group"] == "px4_error_gt_0p3")
    target_hold = next((row for row in target_hold_summary if row["group"] == "target_high_hold"), None)
    target_nonhold = next((row for row in target_hold_summary if row["group"] == "target_high_non_hold"), None)
    target_all = next((row for row in target_hold_summary if row["group"] == "target_high_all"), None)

    lines = [
        "# Shortgen02 Publish Phase Diagnostic",
        "",
        "This diagnostic joins projection-normalized pair rows back to the IEKF odom publish row selected by `iekf_stamp_sec`.",
        "",
        "A `publish_hold_like` row means the odom stamp advanced, but the internal core time and published ENU position barely moved while the vehicle was moving.",
        "",
        "## Hold Definition",
        "",
        markdown_table(
            ["condition", "value"],
            [
                ["publish dt", f">= {HOLD_MIN_PUBLISH_DT_SEC:.2f} s"],
                ["core dt", f"<= {HOLD_MAX_CORE_DT_SEC:.2f} s"],
                ["published horizontal delta", f"<= {HOLD_MAX_POSITION_DELTA_M:.2f} m"],
                ["horizontal speed", f">= {HOLD_MIN_SPEED_MPS:.1f} m/s"],
            ],
        ),
        "",
        "## Hold Summary",
        "",
        markdown_table(
            [
                "group",
                "rows",
                "hold_rows",
                "hold_frac",
                "ekf2_mean",
                "current_mean",
                "px4_mean",
                "px4_p95",
                "px4_frac_gt_0p3",
                "core_dt_mean",
                "pos_delta_mean",
                "motion_deficit_mean",
            ],
            hold_table,
        ),
        "",
        "## Window Summary",
        "",
        markdown_table(
            ["window", "rows", "hold_rows", "hold_frac", "ekf2_mean", "px4_mean", "px4_p95", "px4_frac_gt_0p3"],
            window_table,
        ),
        "",
        "## Largest PX4-Projected Errors",
        "",
        markdown_table(
            [
                "t_arm",
                "ctx",
                "ekf2",
                "px4",
                "hold",
                "pair_ros-iekf_stamp",
                "iekf-ekf2_stamp",
                "pub_dt",
                "core_dt",
                "pos_delta",
                "expected",
                "deficit",
            ],
            top_table,
        ),
        "",
        "## Original Target High Rows",
        "",
        markdown_table(
            ["group", "rows", "hold_rows", "ekf2_mean", "current_mean", "px4_mean", "px4_frac_gt_0p3", "deficit_mean"],
            target_summary_table,
        ),
        "",
        markdown_table(
            ["t_arm", "ctx", "ekf2", "px4", "hold", "pub_dt", "core_dt", "pos_delta", "deficit"],
            target_table,
        ),
        "",
        "## 151-154.5 s Focus Rows From Top Set",
        "",
        markdown_table(
            ["t_arm", "ctx", "px4", "hold", "pub_dt", "core_dt", "pos_delta", "deficit", "err_x", "err_y"],
            focus_table,
        ),
        "",
        "## Interpretation",
        "",
        f"- Hold-like rows are only `{int(hold_row['rows'])}` of the `120-180 s` rows, but their PX4-projected IEKF mean is `{fmt(hold_row['px4_iekf_mean_m'])}` m versus `{fmt(nonhold_row['px4_iekf_mean_m'])}` m for non-hold rows.",
        f"- Among rows with PX4-projected IEKF error above `0.3 m`, `{int(high_hold['rows'])}` of `{int(high_all['rows'])}` are hold-like.",
        (
            f"- In the original target high set, `{int(target_hold['rows'])}` of `{int(target_all['rows'])}` rows are hold-like; "
            f"hold-like target rows average `{fmt(target_hold['px4_iekf_mean_m'])}` m versus `{fmt(target_nonhold['px4_iekf_mean_m'])}` m for non-hold target rows."
            if target_hold and target_nonhold and target_all else
            "- Original target high-row hold summary was unavailable because the projection residual report was missing."
        ),
        "- Several largest residuals, including `153.0 s`, `153.4 s`, `174.6 s`, `177.8 s`, and `178.2 s`, are new-stamp/no-motion IEKF publish samples.",
        "- At `153.0 s` and `153.4 s`, the vehicle is moving about `4.5 m/s`, the publish dt is `0.1 s`, core dt is about `0.001 s`, and published position delta is effectively zero. The missing motion is the same order as the residual spike.",
        "- This points to an output publish/timestamp/propagation phase artifact layered on top of the projection issue. It is not another PMEM/PGR/TVD threshold problem.",
        "- Non-hold high rows still exist, so a publish-phase fix is not yet a complete keeper; the next offline check should counterfactually drop or retime hold-like samples and recompute the PX4-projected score before touching online estimator behavior.",
        "",
        "Generated files:",
        f"- `{out_dir / 'publish_phase_rows_120_180.csv'}`",
        f"- `{out_dir / 'publish_phase_hold_summary.csv'}`",
        f"- `{out_dir / 'publish_phase_window_summary.csv'}`",
        f"- `{out_dir / 'publish_phase_top_rows.csv'}`",
        f"- `{out_dir / 'publish_phase_target_high_rows.csv'}`",
        f"- `{out_dir / 'publish_phase_target_high_summary.csv'}`",
    ]
    (out_dir / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--out-dir")
    parser.add_argument("--top-n", type=int, default=25)
    args = parser.parse_args()

    run_dir = Path(args.run_dir)
    out_dir = Path(args.out_dir) if args.out_dir else run_dir / "shortgen02_publish_phase_diag"
    out_dir.mkdir(parents=True, exist_ok=True)

    rows = build_publish_phase_rows(run_dir)
    hold_summary = build_hold_summary(rows)
    window_summary = build_window_summary(rows)
    top_rows = build_top_rows(rows, args.top_n)
    target_hold_rows = build_target_hold_rows(run_dir, rows)
    target_hold_summary = build_target_hold_summary(target_hold_rows)

    write_csv(out_dir / "publish_phase_rows_120_180.csv", rows)
    write_csv(out_dir / "publish_phase_hold_summary.csv", hold_summary)
    write_csv(out_dir / "publish_phase_window_summary.csv", window_summary)
    write_csv(out_dir / "publish_phase_top_rows.csv", top_rows)
    write_csv(out_dir / "publish_phase_target_high_rows.csv", target_hold_rows)
    write_csv(out_dir / "publish_phase_target_high_summary.csv", target_hold_summary)
    write_report(out_dir, hold_summary, window_summary, top_rows, target_hold_rows, target_hold_summary)
    print(f"wrote: {out_dir / 'report.md'}")


if __name__ == "__main__":
    main()
