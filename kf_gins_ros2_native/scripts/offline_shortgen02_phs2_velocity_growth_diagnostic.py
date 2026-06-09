#!/usr/bin/env python3
"""Compare PHS2 IEKF between-update error growth with native/core velocity mismatch."""

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


def norm2(x: float, y: float) -> float:
    return math.hypot(x, y) if finite(x) and finite(y) else math.nan


def dot(ax: float, ay: float, bx: float, by: float) -> float:
    return ax * bx + ay * by if all(finite(v) for v in (ax, ay, bx, by)) else math.nan


def cosine(ax: float, ay: float, bx: float, by: float) -> float:
    denom = norm2(ax, ay) * norm2(bx, by)
    if not finite(denom) or denom <= 1e-12:
        return math.nan
    return dot(ax, ay, bx, by) / denom


def load_sorted(path: Path, time_key: str) -> tuple[list[dict[str, str]], list[float]]:
    rows = read_csv(path)
    rows.sort(key=lambda row: to_float(row.get(time_key)))
    return rows, [to_float(row.get(time_key)) for row in rows]


def nearest(rows: list[dict[str, str]], times: list[float], t: float, max_dt_sec: float) -> tuple[dict[str, str] | None, float]:
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
    return (row, dt) if dt <= max_dt_sec else (None, dt)


def arm_ros_offset(pair_rows: list[dict[str, str]]) -> float:
    offsets = [
        to_float(row.get("pair_ros_time_sec")) - to_float(row.get("time_since_arm_sec"))
        for row in pair_rows
        if finite(to_float(row.get("pair_ros_time_sec")))
        and finite(to_float(row.get("time_since_arm_sec")))
        and to_float(row.get("mavros_armed"), 0.0) > 0.5
    ]
    offsets.sort()
    if not offsets:
        return math.nan
    return offsets[len(offsets) // 2]


def build_rows(run_dir: Path) -> list[dict[str, object]]:
    intervals = read_csv(run_dir / "shortgen02_measurement_timing_projection_diag" / "propagation_compare_rows.csv")
    pair_rows = read_csv(run_dir / "offline_groundtruth_convergence_diag" / "groundtruth_joined.csv")
    offset = arm_ros_offset(pair_rows)
    gnss_rows, gnss_times = load_sorted(run_dir / "gnss_update_debug.csv", "ros_time_sec")

    out: list[dict[str, object]] = []
    for row in intervals:
        start_t = to_float(row.get("start_time_since_arm_sec"))
        end_t = to_float(row.get("end_time_since_arm_sec"))
        dt_sec = to_float(row.get("dt_sec"))
        if not (120.0 <= start_t < 180.0) or not finite(offset) or not (dt_sec > 0.0):
            continue
        mid_ros = offset + 0.5 * (start_t + end_t)
        gnss, gnss_dt = nearest(gnss_rows, gnss_times, mid_ros, 0.35)
        if gnss is None:
            continue

        growth_x = to_float(row.get("iekf_vector_growth_x_m"))
        growth_y = to_float(row.get("iekf_vector_growth_y_m"))
        ekf2_x = to_float(row.get("ekf2_vector_change_x_m"))
        ekf2_y = to_float(row.get("ekf2_vector_change_y_m"))
        native_e = to_float(gnss.get("native_velocity_vE_mps"))
        native_n = to_float(gnss.get("native_velocity_vN_mps"))
        core_e = to_float(gnss.get("core_velocity_vE_mps"))
        core_n = to_float(gnss.get("core_velocity_vN_mps"))

        core_minus_native_e = (core_e - native_e) * dt_sec
        core_minus_native_n = (core_n - native_n) * dt_sec
        native_minus_core_e = -core_minus_native_e
        native_minus_core_n = -core_minus_native_n

        growth_h = norm2(growth_x, growth_y)
        cmn_h = norm2(core_minus_native_e, core_minus_native_n)
        nmc_h = norm2(native_minus_core_e, native_minus_core_n)
        cos_cmn = cosine(growth_x, growth_y, core_minus_native_e, core_minus_native_n)
        cos_nmc = cosine(growth_x, growth_y, native_minus_core_e, native_minus_core_n)
        proj_cmn = dot(growth_x, growth_y, core_minus_native_e, core_minus_native_n) / cmn_h if cmn_h > 1e-12 else math.nan
        proj_nmc = dot(growth_x, growth_y, native_minus_core_e, native_minus_core_n) / nmc_h if nmc_h > 1e-12 else math.nan

        out.append({
            "start_time_since_arm_sec": start_t,
            "end_time_since_arm_sec": end_t,
            "dt_sec": dt_sec,
            "context": row.get("start_context", ""),
            "gnss_join_dt_sec": gnss_dt,
            "iekf_vector_growth_x_m": growth_x,
            "iekf_vector_growth_y_m": growth_y,
            "iekf_vector_growth_h_m": growth_h,
            "ekf2_vector_change_h_m": norm2(ekf2_x, ekf2_y),
            "iekf_minus_ekf2_vector_change_m": to_float(row.get("iekf_minus_ekf2_vector_change_m")),
            "core_minus_native_e_dt_m": core_minus_native_e,
            "core_minus_native_n_dt_m": core_minus_native_n,
            "core_minus_native_dt_h_m": cmn_h,
            "native_minus_core_dt_h_m": nmc_h,
            "cos_growth_with_core_minus_native": cos_cmn,
            "cos_growth_with_native_minus_core": cos_nmc,
            "projected_growth_on_core_minus_native_m": proj_cmn,
            "projected_growth_on_native_minus_core_m": proj_nmc,
            "core_native_velocity_mismatch_h_mps": cmn_h / dt_sec,
            "horizontal_speed_mps": row.get("horizontal_speed_mps"),
            "gyro_deg_s": row.get("gyro_deg_s"),
        })
    return out


def summarize(label: str, rows: list[dict[str, object]]) -> dict[str, object]:
    return {
        "sample": label,
        "rows": len(rows),
        "iekf_growth_mean_m": mean(to_float(row.get("iekf_vector_growth_h_m")) for row in rows),
        "ekf2_growth_mean_m": mean(to_float(row.get("ekf2_vector_change_h_m")) for row in rows),
        "growth_delta_mean_m": mean(to_float(row.get("iekf_minus_ekf2_vector_change_m")) for row in rows),
        "velocity_mismatch_mean_mps": mean(to_float(row.get("core_native_velocity_mismatch_h_mps")) for row in rows),
        "cmn_dt_mean_m": mean(to_float(row.get("core_minus_native_dt_h_m")) for row in rows),
        "cos_core_minus_native_mean": mean(to_float(row.get("cos_growth_with_core_minus_native")) for row in rows),
        "cos_native_minus_core_mean": mean(to_float(row.get("cos_growth_with_native_minus_core")) for row in rows),
        "abs_cos_best_mean": mean(
            abs(to_float(row.get("cos_growth_with_core_minus_native"))) for row in rows),
        "same_direction_frac": mean(
            1.0 if to_float(row.get("cos_growth_with_core_minus_native")) > 0.5 else 0.0
            for row in rows),
        "opposite_direction_frac": mean(
            1.0 if to_float(row.get("cos_growth_with_core_minus_native")) < -0.5 else 0.0
            for row in rows),
    }


def summary_rows(rows: list[dict[str, object]]) -> list[dict[str, object]]:
    out = []
    for label, start, end in WINDOWS:
        sample = [
            row for row in rows
            if start <= to_float(row.get("start_time_since_arm_sec")) < end
        ]
        out.append(summarize(label, sample))
    out.append(summarize(
        "high_growth_delta_gt_0p1",
        [row for row in rows if to_float(row.get("iekf_minus_ekf2_vector_change_m")) > 0.10],
    ))
    out.append(summarize(
        "high_iekf_growth_gt_0p3",
        [row for row in rows if to_float(row.get("iekf_vector_growth_h_m")) > 0.30],
    ))
    return out


def write_report(out_dir: Path, rows: list[dict[str, object]], summaries: list[dict[str, object]]) -> None:
    largest = sorted(rows, key=lambda row: to_float(row.get("iekf_minus_ekf2_vector_change_m")), reverse=True)[:20]
    lines = [
        "# PHS2 Velocity Growth Diagnostic",
        "",
        "This report compares IEKF between-update error-vector growth with the local core/native velocity mismatch integrated over the same update interval.",
        "",
        "## Summary",
        "",
        markdown_table(
            [
                "sample", "rows", "iekf", "ekf2", "delta", "vdiff", "vdiff_dt",
                "cos_core-native", "cos_native-core", "same", "opposite",
            ],
            [
                [
                    row["sample"],
                    int(to_float(row.get("rows"), 0.0)),
                    fmt(row.get("iekf_growth_mean_m")),
                    fmt(row.get("ekf2_growth_mean_m")),
                    fmt(row.get("growth_delta_mean_m")),
                    fmt(row.get("velocity_mismatch_mean_mps")),
                    fmt(row.get("cmn_dt_mean_m")),
                    fmt(row.get("cos_core_minus_native_mean")),
                    fmt(row.get("cos_native_minus_core_mean")),
                    fmt(row.get("same_direction_frac"), 3),
                    fmt(row.get("opposite_direction_frac"), 3),
                ]
                for row in summaries
            ],
        ),
        "",
        "## Largest Growth-Delta Intervals",
        "",
        markdown_table(
            [
                "start", "end", "ctx", "iekf", "ekf2", "delta",
                "vdiff", "vdiff_dt", "cos_cmn", "cos_nmc",
            ],
            [
                [
                    fmt(row.get("start_time_since_arm_sec"), 1),
                    fmt(row.get("end_time_since_arm_sec"), 1),
                    row.get("context", ""),
                    fmt(row.get("iekf_vector_growth_h_m")),
                    fmt(row.get("ekf2_vector_change_h_m")),
                    fmt(row.get("iekf_minus_ekf2_vector_change_m")),
                    fmt(row.get("core_native_velocity_mismatch_h_mps")),
                    fmt(row.get("core_minus_native_dt_h_m")),
                    fmt(row.get("cos_growth_with_core_minus_native")),
                    fmt(row.get("cos_growth_with_native_minus_core")),
                ]
                for row in largest
            ],
        ),
        "",
        "## Interpretation",
        "",
        "- If velocity mismatch were the dominant direct propagation driver, the integrated mismatch vector would consistently align with IEKF error-vector growth.",
        "- A large growth delta with weak/unstable cosine means velocity mismatch magnitude alone is not enough; timing, frame, or state-transition behavior must be inspected.",
        "",
        "Generated files:",
        f"- `{out_dir / 'velocity_growth_rows.csv'}`",
        f"- `{out_dir / 'velocity_growth_summary.csv'}`",
    ]
    (out_dir / "report.md").write_text("\n".join(lines) + "\n")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", required=True, type=Path)
    parser.add_argument("--out-dir", type=Path)
    args = parser.parse_args()

    out_dir = args.out_dir or args.run_dir / "shortgen02_phs2_velocity_growth_diag"
    out_dir.mkdir(parents=True, exist_ok=True)
    rows = build_rows(args.run_dir)
    summaries = summary_rows(rows)
    write_csv(out_dir / "velocity_growth_rows.csv", rows)
    write_csv(out_dir / "velocity_growth_summary.csv", summaries)
    write_report(out_dir, rows, summaries)

    print(f"wrote: {out_dir / 'report.md'}")
    print(f"wrote: {out_dir / 'velocity_growth_rows.csv'}")
    print(f"wrote: {out_dir / 'velocity_growth_summary.csv'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
