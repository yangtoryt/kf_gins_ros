#!/usr/bin/env python3
"""Compare PHS2 core-state growth with online published-frame growth."""

from __future__ import annotations

import argparse
import bisect
import csv
import math
from pathlib import Path
from typing import Iterable


WINDOWS = [
    ("120-140", 120.0, 140.0),
    ("140-160", 140.0, 160.0),
    ("160-180", 160.0, 180.0),
    ("120-180", 120.0, 180.0),
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


def norm2(x: float, y: float) -> float:
    return math.hypot(x, y) if finite(x) and finite(y) else math.nan


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


def context_label(row: dict[str, str]) -> str:
    if to_float(row.get("turning_now")) > 0.5:
        return "turning"
    if to_float(row.get("post_turn_context")) > 0.5:
        return "post_turn"
    if to_float(row.get("armed_cruise_context")) > 0.5:
        return "armed_cruise"
    if to_float(row.get("terminal_descent_context")) > 0.5:
        return "terminal_descent"
    return "none"


def nearest(rows: list[dict[str, str]], times: list[float], t: float, max_dt: float) -> tuple[dict[str, str] | None, float]:
    if not rows or not finite(t):
        return None, math.nan
    idx = bisect.bisect_left(times, t)
    candidates: list[tuple[float, dict[str, str]]] = []
    for pos in (idx - 1, idx):
        if 0 <= pos < len(rows):
            candidates.append((abs(times[pos] - t), rows[pos]))
    if not candidates:
        return None, math.nan
    dt, row = min(candidates, key=lambda item: item[0])
    return (row, dt) if dt <= max_dt else (None, dt)


def first_at_or_after(
    rows: list[dict[str, str]],
    times: list[float],
    t: float,
    max_dt: float,
) -> tuple[dict[str, str] | None, float]:
    if not rows or not finite(t):
        return None, math.nan
    idx = bisect.bisect_left(times, t)
    if idx >= len(rows):
        return None, math.nan
    dt = times[idx] - t
    return (rows[idx], dt) if dt <= max_dt else (None, dt)


def last_at_or_before(
    rows: list[dict[str, str]],
    times: list[float],
    t: float,
    max_dt: float,
) -> tuple[dict[str, str] | None, float]:
    if not rows or not finite(t):
        return None, math.nan
    idx = bisect.bisect_right(times, t) - 1
    if idx < 0:
        return None, math.nan
    dt = t - times[idx]
    return (rows[idx], dt) if dt <= max_dt else (None, dt)


def error_vec(row: dict[str, str], prefix: str) -> tuple[float, float]:
    return to_float(row.get(f"{prefix}_error_x_m")), to_float(row.get(f"{prefix}_error_y_m"))


def pair_change(start: dict[str, str] | None, end: dict[str, str] | None, prefix: str) -> tuple[float, float, float]:
    if start is None or end is None:
        return math.nan, math.nan, math.nan
    sx, sy = error_vec(start, prefix)
    ex, ey = error_vec(end, prefix)
    dx = ex - sx
    dy = ey - sy
    return dx, dy, norm2(dx, dy)


def build_rows(run_dir: Path) -> list[dict[str, object]]:
    pos_rows = read_csv(run_dir / "phase_error_memory_diag" / "position_update_rows_with_pmem.csv")
    pair_rows = read_csv(run_dir / "offline_groundtruth_convergence_diag" / "groundtruth_joined.csv")
    core_rows = read_csv(run_dir / "shortgen02_measurement_timing_projection_diag" / "propagation_compare_rows.csv")

    pos_rows.sort(key=lambda row: to_float(row.get("time_since_arm_sec")))
    pair_rows.sort(key=lambda row: to_float(row.get("pair_ros_time_sec")))
    core_rows.sort(key=lambda row: to_float(row.get("start_time_since_arm_sec")))

    pair_times = [to_float(row.get("pair_ros_time_sec")) for row in pair_rows]
    core_by_start = {
        round(to_float(row.get("start_time_since_arm_sec")), 3): row
        for row in core_rows
    }

    out: list[dict[str, object]] = []
    for current, nxt in zip(pos_rows, pos_rows[1:]):
        start = to_float(current.get("time_since_arm_sec"))
        end = to_float(nxt.get("time_since_arm_sec"))
        start_ros = to_float(current.get("ros_time_sec"))
        end_ros = to_float(nxt.get("ros_time_sec"))
        if not (120.0 <= start < 180.0) or not finite(end) or end <= start:
            continue

        nearest0, nearest0_dt = nearest(pair_rows, pair_times, start_ros, 0.15)
        nearest1, nearest1_dt = nearest(pair_rows, pair_times, end_ros, 0.15)

        # Use a small exclusion around the GNSS update timestamp so this sample
        # is less likely to include the update callback itself.
        after0, after0_dt = first_at_or_after(pair_rows, pair_times, start_ros + 0.02, 0.25)
        before1, before1_dt = last_at_or_before(pair_rows, pair_times, end_ros - 0.02, 0.25)

        published_near_x, published_near_y, published_near_h = pair_change(nearest0, nearest1, "iekf")
        ekf2_near_x, ekf2_near_y, ekf2_near_h = pair_change(nearest0, nearest1, "ekf2")
        published_prop_x, published_prop_y, published_prop_h = pair_change(after0, before1, "iekf")
        ekf2_prop_x, ekf2_prop_y, ekf2_prop_h = pair_change(after0, before1, "ekf2")

        core = core_by_start.get(round(start, 3), {})
        out.append({
            "start_time_since_arm_sec": start,
            "end_time_since_arm_sec": end,
            "dt_sec": end - start,
            "context": context_label(current),
            "nearest_start_dt_sec": nearest0_dt,
            "nearest_end_dt_sec": nearest1_dt,
            "after_start_dt_sec": after0_dt,
            "before_end_dt_sec": before1_dt,
            "core_iekf_growth_h_m": to_float(core.get("iekf_vector_growth_h_m")),
            "core_iekf_growth_x_m": to_float(core.get("iekf_vector_growth_x_m")),
            "core_iekf_growth_y_m": to_float(core.get("iekf_vector_growth_y_m")),
            "core_ekf2_change_h_m": to_float(core.get("ekf2_vector_change_h_m")),
            "published_nearest_iekf_change_h_m": published_near_h,
            "published_nearest_iekf_change_x_m": published_near_x,
            "published_nearest_iekf_change_y_m": published_near_y,
            "published_nearest_ekf2_change_h_m": ekf2_near_h,
            "published_prop_iekf_change_h_m": published_prop_h,
            "published_prop_iekf_change_x_m": published_prop_x,
            "published_prop_iekf_change_y_m": published_prop_y,
            "published_prop_ekf2_change_h_m": ekf2_prop_h,
            "core_minus_published_prop_h_m": to_float(core.get("iekf_vector_growth_h_m")) - published_prop_h,
            "published_prop_minus_ekf2_h_m": published_prop_h - ekf2_prop_h,
            "published_nearest_minus_ekf2_h_m": published_near_h - ekf2_near_h,
            "start_online_iekf_error_h_m": to_float(after0.get("iekf_error_xy_m")) if after0 else math.nan,
            "end_online_iekf_error_h_m": to_float(before1.get("iekf_error_xy_m")) if before1 else math.nan,
            "start_core_after_error_h_m": to_float(current.get("error_after_h_m")),
            "end_core_before_error_h_m": to_float(nxt.get("error_before_h_m")),
            "dx_pos_h_norm_m": to_float(current.get("dx_pos_h_norm_m")),
            "gnss_residual_h_m": to_float(current.get("gnss_residual_h_m")),
            "horizontal_speed_mps": to_float(current.get("horizontal_speed_mps")),
            "gyro_deg_s": to_float(current.get("gyro_deg_s")),
        })
    return out


def summarize(label: str, rows: list[dict[str, object]]) -> dict[str, object]:
    return {
        "sample": label,
        "rows": len(rows),
        "core_iekf_mean_m": mean(to_float(row.get("core_iekf_growth_h_m")) for row in rows),
        "published_prop_iekf_mean_m": mean(to_float(row.get("published_prop_iekf_change_h_m")) for row in rows),
        "published_nearest_iekf_mean_m": mean(to_float(row.get("published_nearest_iekf_change_h_m")) for row in rows),
        "published_prop_ekf2_mean_m": mean(to_float(row.get("published_prop_ekf2_change_h_m")) for row in rows),
        "core_minus_published_prop_mean_m": mean(to_float(row.get("core_minus_published_prop_h_m")) for row in rows),
        "published_prop_minus_ekf2_mean_m": mean(to_float(row.get("published_prop_minus_ekf2_h_m")) for row in rows),
        "published_prop_iekf_p95_m": percentile(
            (to_float(row.get("published_prop_iekf_change_h_m")) for row in rows), 95.0),
        "published_prop_gt_0p3_frac": mean(
            1.0 if to_float(row.get("published_prop_iekf_change_h_m")) > 0.3 else 0.0 for row in rows),
    }


def summary_rows(rows: list[dict[str, object]]) -> list[dict[str, object]]:
    out = []
    for label, start, end in WINDOWS:
        out.append(summarize(
            label,
            [row for row in rows if start <= to_float(row.get("start_time_since_arm_sec")) < end],
        ))
    out.append(summarize(
        "core_growth_gt_0p3",
        [row for row in rows if to_float(row.get("core_iekf_growth_h_m")) > 0.3],
    ))
    out.append(summarize(
        "published_prop_gt_0p3",
        [row for row in rows if to_float(row.get("published_prop_iekf_change_h_m")) > 0.3],
    ))
    return out


def write_report(out_dir: Path, rows: list[dict[str, object]], summaries: list[dict[str, object]]) -> None:
    largest_core_gap = sorted(
        rows,
        key=lambda row: to_float(row.get("core_minus_published_prop_h_m")),
        reverse=True,
    )[:20]
    largest_online = sorted(
        rows,
        key=lambda row: to_float(row.get("published_prop_iekf_change_h_m")),
        reverse=True,
    )[:20]

    lines = [
        "# PHS2 Online Growth Diagnostic",
        "",
        "This report checks whether the large between-update growth seen in the core/state-update diagnostic is also visible in the online published `/kf_gins/odom` frame used for PHS2 scoring.",
        "",
        "Definitions:",
        "",
        "- `core_iekf`: `state_update_debug` after-current-update to before-next-update, transformed by the offline GPS/groundtruth fit.",
        "- `published_prop`: first online pair row at least `0.02 s` after the current GNSS update to the last online pair row at least `0.02 s` before the next GNSS update.",
        "- `published_nearest`: nearest online pair rows to the two GNSS update timestamps; this is a timing sensitivity check, not the preferred pure-propagation readout.",
        "",
        "## Summary",
        "",
        markdown_table(
            [
                "sample", "rows", "core", "pub_prop", "pub_near", "ekf2_prop",
                "core-pub", "pub-ekf2", "pub_p95", "pub>0.3",
            ],
            [
                [
                    row.get("sample"),
                    row.get("rows"),
                    fmt(row.get("core_iekf_mean_m")),
                    fmt(row.get("published_prop_iekf_mean_m")),
                    fmt(row.get("published_nearest_iekf_mean_m")),
                    fmt(row.get("published_prop_ekf2_mean_m")),
                    fmt(row.get("core_minus_published_prop_mean_m")),
                    fmt(row.get("published_prop_minus_ekf2_mean_m")),
                    fmt(row.get("published_prop_iekf_p95_m")),
                    fmt(row.get("published_prop_gt_0p3_frac"), 3),
                ]
                for row in summaries
            ],
        ),
        "",
        "## Largest Core Minus Published Gaps",
        "",
        markdown_table(
            [
                "start", "end", "ctx", "core", "pub_prop", "pub_near",
                "ekf2_prop", "core-pub", "start_online", "end_online",
            ],
            [
                [
                    fmt(row.get("start_time_since_arm_sec"), 1),
                    fmt(row.get("end_time_since_arm_sec"), 1),
                    row.get("context"),
                    fmt(row.get("core_iekf_growth_h_m")),
                    fmt(row.get("published_prop_iekf_change_h_m")),
                    fmt(row.get("published_nearest_iekf_change_h_m")),
                    fmt(row.get("published_prop_ekf2_change_h_m")),
                    fmt(row.get("core_minus_published_prop_h_m")),
                    fmt(row.get("start_online_iekf_error_h_m")),
                    fmt(row.get("end_online_iekf_error_h_m")),
                ]
                for row in largest_core_gap
            ],
        ),
        "",
        "## Largest Published Propagation Changes",
        "",
        markdown_table(
            [
                "start", "end", "ctx", "core", "pub_prop", "ekf2_prop",
                "pub-ekf2", "start_online", "end_online",
            ],
            [
                [
                    fmt(row.get("start_time_since_arm_sec"), 1),
                    fmt(row.get("end_time_since_arm_sec"), 1),
                    row.get("context"),
                    fmt(row.get("core_iekf_growth_h_m")),
                    fmt(row.get("published_prop_iekf_change_h_m")),
                    fmt(row.get("published_prop_ekf2_change_h_m")),
                    fmt(row.get("published_prop_minus_ekf2_h_m")),
                    fmt(row.get("start_online_iekf_error_h_m")),
                    fmt(row.get("end_online_iekf_error_h_m")),
                ]
                for row in largest_online
            ],
        ),
        "",
        "Generated files:",
        f"- `{out_dir / 'online_growth_rows.csv'}`",
        f"- `{out_dir / 'online_growth_summary.csv'}`",
    ]
    (out_dir / "report.md").write_text("\n".join(lines) + "\n")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", required=True, type=Path)
    parser.add_argument("--out-dir", type=Path)
    args = parser.parse_args()

    run_dir = args.run_dir
    out_dir = args.out_dir or run_dir / "shortgen02_phs2_online_growth_diag"
    rows = build_rows(run_dir)
    summaries = summary_rows(rows)
    write_csv(out_dir / "online_growth_rows.csv", rows)
    write_csv(out_dir / "online_growth_summary.csv", summaries)
    write_report(out_dir, rows, summaries)
    print(f"wrote {out_dir / 'report.md'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
