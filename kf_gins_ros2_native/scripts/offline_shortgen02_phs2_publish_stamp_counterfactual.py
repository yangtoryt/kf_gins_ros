#!/usr/bin/env python3
"""Score PHS2 published IEKF positions with a core-time-based odom stamp."""

from __future__ import annotations

import argparse
import bisect
import csv
import math
import statistics
from pathlib import Path
from typing import Callable, Iterable


WINDOWS = [
    ("armed_all", lambda row: to_float(row.get("mavros_armed")) > 0.5),
    ("segment_mission_all", lambda row: str(row.get("mavros_mode", "")).startswith("AUTO.MISSION")),
    ("segment_main_maneuver_40_180", lambda row: 40.0 <= to_float(row.get("time_since_arm_sec")) < 180.0),
    ("120-140", lambda row: 120.0 <= to_float(row.get("time_since_arm_sec")) < 140.0),
    ("140-160", lambda row: 140.0 <= to_float(row.get("time_since_arm_sec")) < 160.0),
    ("160-180", lambda row: 160.0 <= to_float(row.get("time_since_arm_sec")) < 180.0),
    ("120-180", lambda row: 120.0 <= to_float(row.get("time_since_arm_sec")) < 180.0),
    ("segment_rtl_all", lambda row: str(row.get("mavros_mode", "")) == "AUTO.RTL"),
]


def finite(value: float) -> bool:
    return math.isfinite(value)


def to_float(value: object, default: float = math.nan) -> float:
    try:
        result = float(value)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return default
    return result if finite(result) else default


def mean(values: Iterable[float]) -> float:
    vals = [item for item in values if finite(item)]
    return sum(vals) / len(vals) if vals else math.nan


def percentile(values: Iterable[float], pct: float) -> float:
    vals = sorted(item for item in values if finite(item))
    if not vals:
        return math.nan
    if len(vals) == 1:
        return vals[0]
    pos = (len(vals) - 1) * pct / 100.0
    lo = math.floor(pos)
    hi = math.ceil(pos)
    if lo == hi:
        return vals[lo]
    return vals[lo] * (hi - pos) + vals[hi] * (pos - lo)


def fmt(value: object, digits: int = 4) -> str:
    val = to_float(value)
    if not finite(val):
        return "nan"
    return f"{val:.{digits}f}"


def read_csv(path: Path) -> list[dict[str, str]]:
    if not path.exists() or path.stat().st_size == 0:
        return []
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


def find_groundtruth_csv(run_dir: Path) -> Path:
    matches = sorted((run_dir / "ulog_csv").glob("*_vehicle_local_position_groundtruth_0.csv"))
    if not matches:
        matches = sorted(run_dir.rglob("*_vehicle_local_position_groundtruth_0.csv"))
    if not matches:
        raise FileNotFoundError("missing vehicle_local_position_groundtruth CSV")
    return matches[0]


def load_groundtruth(run_dir: Path) -> tuple[list[dict[str, float]], list[float]]:
    rows: list[dict[str, float]] = []
    for row in read_csv(find_groundtruth_csv(run_dir)):
        t = to_float(row.get("timestamp")) * 1e-6
        n = to_float(row.get("x"))
        e = to_float(row.get("y"))
        d = to_float(row.get("z"))
        if all(finite(item) for item in (t, n, e, d)):
            rows.append({"timestamp_sec": t, "gt_x_m": e, "gt_y_m": n, "gt_z_m": -d})
    rows.sort(key=lambda row: row["timestamp_sec"])
    return rows, [row["timestamp_sec"] for row in rows]


def nearest_groundtruth(
    rows: list[dict[str, float]],
    times: list[float],
    stamp: float,
    max_dt: float,
) -> tuple[dict[str, float] | None, float]:
    if not finite(stamp):
        return None, math.nan
    idx = bisect.bisect_left(times, stamp)
    candidates: list[tuple[float, dict[str, float]]] = []
    for pos in (idx - 1, idx):
        if 0 <= pos < len(rows):
            candidates.append((abs(times[pos] - stamp), rows[pos]))
    if not candidates:
        return None, math.nan
    dt, row = min(candidates, key=lambda item: item[0])
    return (row, dt) if dt <= max_dt else (None, dt)


def infer_arm_stamp(run_dir: Path) -> float:
    joined = read_csv(run_dir / "offline_groundtruth_convergence_diag" / "groundtruth_joined.csv")
    offsets = [
        to_float(row.get("stamp_sec")) - to_float(row.get("time_since_arm_sec"))
        for row in joined
        if to_float(row.get("mavros_armed")) > 0.5
        and finite(to_float(row.get("stamp_sec")))
        and finite(to_float(row.get("time_since_arm_sec")))
    ]
    if not offsets:
        raise RuntimeError("cannot infer armed timestamp from groundtruth_joined.csv")
    return statistics.median(offsets)


def infer_core_to_ros_offset(
    publish_rows: list[dict[str, str]],
    arm_stamp: float,
    align_start: float,
    align_end: float,
) -> dict[str, float]:
    offsets: list[float] = []
    target_offsets: list[float] = []
    for row in publish_rows:
        odom_t = to_float(row.get("odom_stamp_sec"))
        core_t = to_float(row.get("last_core_time_sec"))
        if not finite(odom_t) or not finite(core_t):
            continue
        tsa = odom_t - arm_stamp
        offset = odom_t - core_t
        if align_start <= tsa <= align_end:
            offsets.append(offset)
        if 120.0 <= tsa < 180.0:
            target_offsets.append(offset)
    if not offsets:
        raise RuntimeError("cannot infer early core-to-ROS offset")
    return {
        "early_offset_median_sec": statistics.median(offsets),
        "early_offset_min_sec": min(offsets),
        "early_offset_max_sec": max(offsets),
        "early_offset_rows": len(offsets),
        "target_offset_mean_sec": mean(target_offsets),
        "target_offset_min_sec": min(target_offsets) if target_offsets else math.nan,
        "target_offset_max_sec": max(target_offsets) if target_offsets else math.nan,
        "target_offset_std_sec": statistics.pstdev(target_offsets) if len(target_offsets) > 1 else math.nan,
        "target_offset_rows": len(target_offsets),
    }


def build_publish_rows(
    run_dir: Path,
    mode: str,
    stamp_fn: Callable[[dict[str, str]], float],
    arm_stamp: float,
    max_join_dt: float,
) -> list[dict[str, object]]:
    gt_rows, gt_times = load_groundtruth(run_dir)
    rows: list[dict[str, object]] = []
    for row in read_csv(run_dir / "state_publish_debug.csv"):
        stamp = stamp_fn(row)
        gt, gt_dt = nearest_groundtruth(gt_rows, gt_times, stamp, max_join_dt)
        if gt is None:
            continue
        x = to_float(row.get("published_enu_e_m"))
        y = to_float(row.get("published_enu_n_m"))
        if not all(finite(item) for item in (stamp, x, y)):
            continue
        odom_stamp = to_float(row.get("odom_stamp_sec"))
        core_stamp = to_float(row.get("last_core_time_sec"))
        rows.append({
            "stamp_mode": mode,
            "stamp_sec": stamp,
            "odom_stamp_sec": odom_stamp,
            "last_core_time_sec": core_stamp,
            "odom_minus_selected_stamp_sec": odom_stamp - stamp if finite(odom_stamp) else math.nan,
            "time_since_arm_sec": stamp - arm_stamp,
            "mavros_armed": to_float(row.get("mavros_armed")),
            "mavros_mode": nearest_mode(run_dir, odom_stamp),
            "gt_join_dt_sec": gt_dt,
            "gt_x_m": gt["gt_x_m"],
            "gt_y_m": gt["gt_y_m"],
            "iekf_x_m": x,
            "iekf_y_m": y,
            "core_velocity_vN_mps": to_float(row.get("core_velocity_vN_mps")),
            "core_velocity_vE_mps": to_float(row.get("core_velocity_vE_mps")),
        })
    return rows


_MODE_CACHE: dict[Path, tuple[list[dict[str, str]], list[float]]] = {}


def nearest_mode(run_dir: Path, ros_time: float) -> str:
    if not finite(ros_time):
        return ""
    if run_dir not in _MODE_CACHE:
        joined = read_csv(run_dir / "offline_groundtruth_convergence_diag" / "groundtruth_joined.csv")
        joined.sort(key=lambda row: to_float(row.get("pair_ros_time_sec")))
        _MODE_CACHE[run_dir] = (joined, [to_float(row.get("pair_ros_time_sec")) for row in joined])
    rows, times = _MODE_CACHE[run_dir]
    idx = bisect.bisect_left(times, ros_time)
    candidates: list[tuple[float, dict[str, str]]] = []
    for pos in (idx - 1, idx):
        if 0 <= pos < len(rows):
            candidates.append((abs(times[pos] - ros_time), rows[pos]))
    if not candidates:
        return ""
    dt, row = min(candidates, key=lambda item: item[0])
    return str(row.get("mavros_mode", "")) if dt <= 0.2 else ""


def apply_alignment(rows: list[dict[str, object]], align_start: float, align_end: float) -> tuple[float, float]:
    dx = [
        to_float(row.get("gt_x_m")) - to_float(row.get("iekf_x_m"))
        for row in rows
        if align_start <= to_float(row.get("time_since_arm_sec")) <= align_end
        and to_float(row.get("mavros_armed")) > 0.5
    ]
    dy = [
        to_float(row.get("gt_y_m")) - to_float(row.get("iekf_y_m"))
        for row in rows
        if align_start <= to_float(row.get("time_since_arm_sec")) <= align_end
        and to_float(row.get("mavros_armed")) > 0.5
    ]
    off_x = mean(dx)
    off_y = mean(dy)
    if not finite(off_x):
        off_x = 0.0
    if not finite(off_y):
        off_y = 0.0
    for row in rows:
        x = to_float(row.get("iekf_x_m")) + off_x
        y = to_float(row.get("iekf_y_m")) + off_y
        gt_x = to_float(row.get("gt_x_m"))
        gt_y = to_float(row.get("gt_y_m"))
        ex = x - gt_x
        ey = y - gt_y
        row["iekf_aligned_x_m"] = x
        row["iekf_aligned_y_m"] = y
        row["iekf_error_x_m"] = ex
        row["iekf_error_y_m"] = ey
        row["iekf_error_xy_m"] = math.hypot(ex, ey) if finite(ex) and finite(ey) else math.nan
    return off_x, off_y


def summarize_errors(label: str, estimator: str, rows: list[dict[str, object]]) -> dict[str, object]:
    vals = [to_float(row.get(f"{estimator}_error_xy_m")) for row in rows]
    vals = [item for item in vals if finite(item)]
    return {
        "window": label,
        "estimator": estimator,
        "rows": len(vals),
        "xy_mean_m": mean(vals),
        "xy_p95_m": percentile(vals, 95.0),
        "xy_max_m": max(vals) if vals else math.nan,
        "frac_over_0p3": mean(1.0 if item > 0.3 else 0.0 for item in vals),
    }


def summarize_publish(mode: str, rows: list[dict[str, object]]) -> list[dict[str, object]]:
    out = []
    for label, predicate in WINDOWS:
        subset = [row for row in rows if predicate(row)]
        summary = summarize_errors(label, "iekf", subset)
        summary["stamp_mode"] = mode
        summary["stamp_shift_abs_mean_sec"] = mean(abs(to_float(row.get("odom_minus_selected_stamp_sec"))) for row in subset)
        summary["stamp_shift_abs_p95_sec"] = percentile(
            (abs(to_float(row.get("odom_minus_selected_stamp_sec"))) for row in subset), 95.0)
        out.append(summary)
    return out


def summarize_ekf2(run_dir: Path) -> list[dict[str, object]]:
    joined = read_csv(run_dir / "offline_groundtruth_convergence_diag" / "groundtruth_joined.csv")
    for row in joined:
        row["ekf2_error_xy_m"] = row.get("ekf2_error_xy_m", "")
    return [summarize_errors(label, "ekf2", [row for row in joined if predicate(row)]) for label, predicate in WINDOWS]


def write_report(
    out_dir: Path,
    offset_info: dict[str, float],
    summaries: list[dict[str, object]],
    offsets: dict[str, tuple[float, float]],
) -> None:
    by_window: dict[str, dict[str, dict[str, object]]] = {}
    for row in summaries:
        key = str(row.get("estimator"))
        if key == "iekf":
            key = str(row.get("stamp_mode"))
        by_window.setdefault(str(row.get("window")), {})[key] = row
    key_windows = ["armed_all", "segment_mission_all", "segment_main_maneuver_40_180", "120-140", "140-160", "160-180", "120-180", "segment_rtl_all"]
    rows = []
    for window in key_windows:
        ekf2 = by_window.get(window, {}).get("ekf2", {})
        old = by_window.get(window, {}).get("iekf_odom_stamp", {})
        corrected = by_window.get(window, {}).get("iekf_core_fixed_offset", {})
        rows.append([
            window,
            fmt(ekf2.get("xy_mean_m")),
            fmt(old.get("xy_mean_m")),
            fmt(corrected.get("xy_mean_m")),
            fmt(ekf2.get("xy_p95_m")),
            fmt(old.get("xy_p95_m")),
            fmt(corrected.get("xy_p95_m")),
            fmt(old.get("frac_over_0p3"), 3),
            fmt(corrected.get("frac_over_0p3"), 3),
            fmt(corrected.get("stamp_shift_abs_p95_sec"), 3),
        ])

    lines = [
        "# PHS2 Publish Stamp Counterfactual",
        "",
        "This report rescored the already published PHS2 IEKF positions with two timestamp choices:",
        "",
        "- `iekf_odom_stamp`: current `/kf_gins/odom` header stamp.",
        "- `iekf_core_fixed_offset`: `last_core_time_sec + median(odom_stamp_sec - last_core_time_sec)` from the early armed alignment window.",
        "",
        "No estimator state is changed here; this is an offline timestamp/frame diagnostic.",
        "",
        "## Core Time Offset",
        "",
        markdown_table(
            ["metric", "value"],
            [
                ["early rows", int(offset_info.get("early_offset_rows", 0))],
                ["early median offset sec", fmt(offset_info.get("early_offset_median_sec"), 6)],
                ["early offset range sec", f"{fmt(offset_info.get('early_offset_min_sec'), 6)}..{fmt(offset_info.get('early_offset_max_sec'), 6)}"],
                ["120-180 rows", int(offset_info.get("target_offset_rows", 0))],
                ["120-180 mean offset sec", fmt(offset_info.get("target_offset_mean_sec"), 6)],
                ["120-180 offset std sec", fmt(offset_info.get("target_offset_std_sec"), 6)],
                ["120-180 offset range sec", f"{fmt(offset_info.get('target_offset_min_sec'), 6)}..{fmt(offset_info.get('target_offset_max_sec'), 6)}"],
            ],
        ),
        "",
        "Alignment offsets applied per IEKF timestamp mode:",
        "",
        markdown_table(
            ["mode", "x offset", "y offset"],
            [[mode, fmt(pair[0]), fmt(pair[1])] for mode, pair in offsets.items()],
        ),
        "",
        "## Groundtruth Score",
        "",
        markdown_table(
            [
                "window", "EKF2 mean", "IEKF old mean", "IEKF core-stamp mean",
                "EKF2 p95", "IEKF old p95", "IEKF core-stamp p95",
                "old >0.3", "core-stamp >0.3", "stamp shift p95",
            ],
            rows,
        ),
        "",
        "Generated files:",
        f"- `{out_dir / 'publish_stamp_counterfactual_rows.csv'}`",
        f"- `{out_dir / 'publish_stamp_counterfactual_summary.csv'}`",
    ]
    (out_dir / "report.md").write_text("\n".join(lines) + "\n")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", required=True, type=Path)
    parser.add_argument("--out-dir", type=Path)
    parser.add_argument("--align-start", type=float, default=0.0)
    parser.add_argument("--align-end", type=float, default=20.0)
    parser.add_argument("--max-join-dt-sec", type=float, default=0.03)
    args = parser.parse_args()

    run_dir = args.run_dir
    out_dir = args.out_dir or run_dir / "shortgen02_phs2_publish_stamp_counterfactual"
    publish_rows = read_csv(run_dir / "state_publish_debug.csv")
    arm_stamp = infer_arm_stamp(run_dir)
    offset_info = infer_core_to_ros_offset(publish_rows, arm_stamp, args.align_start, args.align_end)
    core_to_ros_offset = offset_info["early_offset_median_sec"]

    mode_rows: list[dict[str, object]] = []
    offsets: dict[str, tuple[float, float]] = {}
    modes: list[tuple[str, Callable[[dict[str, str]], float]]] = [
        ("iekf_odom_stamp", lambda row: to_float(row.get("odom_stamp_sec"))),
        ("iekf_core_fixed_offset", lambda row: to_float(row.get("last_core_time_sec")) + core_to_ros_offset),
    ]
    summary_rows = summarize_ekf2(run_dir)
    for mode, stamp_fn in modes:
        rows = build_publish_rows(run_dir, mode, stamp_fn, arm_stamp, args.max_join_dt_sec)
        offsets[mode] = apply_alignment(rows, args.align_start, args.align_end)
        mode_rows.extend(rows)
        summary_rows.extend(summarize_publish(mode, rows))

    write_csv(out_dir / "publish_stamp_counterfactual_rows.csv", mode_rows)
    write_csv(out_dir / "publish_stamp_counterfactual_summary.csv", summary_rows)
    write_report(out_dir, offset_info, summary_rows, offsets)
    print(f"wrote {out_dir / 'report.md'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
