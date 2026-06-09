#!/usr/bin/env python3
"""Summarize read-only phase-error memory diagnostics against groundtruth rows."""

from __future__ import annotations

import argparse
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
from offline_position_update_gain_diagnostic import build_rows


WINDOWS = [
    ("120-180", 120.0, 180.0),
    ("140-180", 140.0, 180.0),
    ("160-180", 160.0, 180.0),
]


def bool01(value: object) -> bool:
    return to_float(value, 0.0) > 0.5


def pct(value: float) -> str:
    if not finite(value):
        return "nan"
    return f"{100.0 * value:.1f}%"


def fmt(value: object, digits: int = 4) -> str:
    val = to_float(value)
    if not finite(val):
        return "nan"
    return f"{val:.{digits}f}"


def present_value(value: object) -> bool:
    if value is None:
        return False
    if finite(value):
        return True
    text = str(value).strip().lower()
    return text not in {"", "nan", "none"}


def reason_text(value: object) -> str:
    if not present_value(value):
        return "missing"
    return str(value)


def context_label(row: dict[str, object]) -> str:
    if bool01(row.get("turning_now")):
        return "turning"
    if bool01(row.get("post_turn_context")):
        return "post_turn"
    if bool01(row.get("armed_cruise_context")):
        return "armed_cruise"
    if bool01(row.get("terminal_descent_context")):
        return "terminal"
    return "none"


def in_window(row: dict[str, object], start: float, end: float) -> bool:
    t = to_float(row.get("time_since_arm_sec"))
    return finite(t) and start <= t < end


def coverage_row(
    label: str,
    rows: list[dict[str, object]],
    high_error_threshold_m: float,
) -> dict[str, object]:
    high = [
        row for row in rows
        if to_float(row.get("pair_iekf_error_h_m")) > high_error_threshold_m
    ]
    pressure = [row for row in rows if bool01(row.get("phase_error_memory_pressure_active"))]
    candidate = [row for row in rows if bool01(row.get("phase_error_memory_candidate_active"))]
    high_pressure = [
        row for row in high if bool01(row.get("phase_error_memory_pressure_active"))
    ]
    high_candidate = [
        row for row in high if bool01(row.get("phase_error_memory_candidate_active"))
    ]
    high_no_pressure = [
        row for row in high if not bool01(row.get("phase_error_memory_pressure_active"))
    ]
    high_no_candidate = [
        row for row in high if not bool01(row.get("phase_error_memory_candidate_active"))
    ]
    reasons: dict[str, int] = {}
    for row in high:
        reason = reason_text(row.get("phase_error_memory_reason", "missing"))
        reasons[reason] = reasons.get(reason, 0) + 1

    return {
        "window": label,
        "rows": len(rows),
        "high_error_threshold_m": high_error_threshold_m,
        "high_rows": len(high),
        "high_pressure_rows": len(high_pressure),
        "high_pressure_coverage": len(high_pressure) / len(high) if high else math.nan,
        "high_candidate_rows": len(high_candidate),
        "high_candidate_coverage": len(high_candidate) / len(high) if high else math.nan,
        "pressure_all_rows": len(pressure),
        "pressure_all_fraction": len(pressure) / len(rows) if rows else math.nan,
        "candidate_all_rows": len(candidate),
        "candidate_all_fraction": len(candidate) / len(rows) if rows else math.nan,
        "high_no_pressure_rows": len(high_no_pressure),
        "high_no_candidate_rows": len(high_no_candidate),
        "high_error_mean_m": mean(to_float(row.get("pair_iekf_error_h_m")) for row in high),
        "high_error_p95_m": percentile(
            (to_float(row.get("pair_iekf_error_h_m")) for row in high), 95.0),
        "top_high_reasons": ";".join(
            f"{reason}:{count}" for reason, count in sorted(
                reasons.items(), key=lambda item: (-item[1], item[0]))[:6]
        ),
    }


def select_examples(
    rows: list[dict[str, object]],
    high_error_threshold_m: float,
    require_miss: bool,
    limit: int,
) -> list[dict[str, object]]:
    high = [
        row for row in rows
        if to_float(row.get("pair_iekf_error_h_m")) > high_error_threshold_m
    ]
    if require_miss:
        high = [
            row for row in high
            if not bool01(row.get("phase_error_memory_candidate_active"))
        ]
    high.sort(key=lambda row: to_float(row.get("pair_iekf_error_h_m")), reverse=True)
    out: list[dict[str, object]] = []
    for row in high[:limit]:
        out.append({
            "sequence": row.get("sequence"),
            "time_since_arm_sec": row.get("time_since_arm_sec"),
            "context": context_label(row),
            "pair_iekf_error_h_m": row.get("pair_iekf_error_h_m"),
            "pair_ekf2_error_h_m": row.get("pair_ekf2_error_h_m"),
            "phase_error_memory_pressure_active": row.get(
                "phase_error_memory_pressure_active", math.nan),
            "phase_error_memory_candidate_active": row.get(
                "phase_error_memory_candidate_active", math.nan),
            "phase_error_memory_reason": reason_text(row.get("phase_error_memory_reason", "")),
            "phase_error_memory_recent_turnpost_age_sec": row.get(
                "phase_error_memory_recent_turnpost_age_sec", math.nan),
            "phase_error_memory_residual_h_m": row.get(
                "phase_error_memory_residual_h_m", math.nan),
            "phase_error_memory_dx_pos_h_m": row.get(
                "phase_error_memory_dx_pos_h_m", math.nan),
            "phase_error_memory_dx_over_residual_h": row.get(
                "phase_error_memory_dx_over_residual_h", math.nan),
            "phase_error_memory_pos_std_h_before_m": row.get(
                "phase_error_memory_pos_std_h_before_m", math.nan),
            "gnss_hnis_2d": row.get("gnss_hnis_2d", math.nan),
            "horizontal_speed_mps": row.get("horizontal_speed_mps", math.nan),
            "gyro_deg_s": row.get("gyro_deg_s", math.nan),
            "source_yaw_rate_deg_s": row.get("source_yaw_rate_deg_s", math.nan),
        })
    return out


def write_report(
    out_dir: Path,
    coverage: list[dict[str, object]],
    rows: list[dict[str, object]],
    high_error_threshold_m: float,
) -> None:
    pmem_columns_present = any(
        present_value(row.get("phase_error_memory_debug_enabled")) or
        present_value(row.get("phase_error_memory_reason"))
        for row in rows
    )
    debug_enabled_rows = [
        row for row in rows if bool01(row.get("phase_error_memory_debug_enabled"))
    ]
    pressure_rows = [
        row for row in rows if bool01(row.get("phase_error_memory_pressure_active"))
    ]
    candidate_rows = [
        row for row in rows if bool01(row.get("phase_error_memory_candidate_active"))
    ]

    table_rows = []
    for row in coverage:
        table_rows.append([
            row["window"],
            row["rows"],
            row["high_rows"],
            row["high_pressure_rows"],
            pct(to_float(row["high_pressure_coverage"])),
            row["high_candidate_rows"],
            pct(to_float(row["high_candidate_coverage"])),
            pct(to_float(row["pressure_all_fraction"])),
            pct(to_float(row["candidate_all_fraction"])),
        ])

    lines = [
        "# Phase Error Memory Diagnostic",
        "",
        f"position update rows: {len(rows)}",
        f"high-error threshold: {high_error_threshold_m:.3f} m",
        f"PMEM columns present: {'yes' if pmem_columns_present else 'no'}",
        f"PMEM debug-enabled rows: {len(debug_enabled_rows)}",
        f"PMEM pressure rows: {len(pressure_rows)}",
        f"PMEM candidate rows: {len(candidate_rows)}",
        "",
        markdown_table(
            [
                "window",
                "rows",
                "high_rows",
                "high_pressure",
                "pressure_cov",
                "high_candidate",
                "candidate_cov",
                "pressure_all",
                "candidate_all",
            ],
            table_rows,
        ),
        "",
        "## Interpretation",
        "",
    ]
    if not pmem_columns_present:
        lines.append("This run has no PMEM columns. Re-run with updated kf_gins_node and GNSS debug CSV enabled.")
    elif not debug_enabled_rows:
        lines.append("PMEM columns are present but debug was disabled; enable `PX4_SAFE_COMPARE_PHASE_ERROR_MEMORY_DEBUG_ENABLE=true` for validation.")
    else:
        lines.append("Use `pressure_active` as the pure weak-position-update label, and `candidate_active` as pressure plus recent turn/post memory.")
        lines.append("A keeper mechanism should not be inferred from this report alone; first require coverage on shortgen02 and then on new holdout routes.")

    lines.append("")
    lines.append("## Reason Counts")
    lines.append("")
    reason_counts: dict[str, int] = {}
    for row in rows:
        reason = reason_text(row.get("phase_error_memory_reason", "missing"))
        reason_counts[reason] = reason_counts.get(reason, 0) + 1
    lines.append(markdown_table(
        ["reason", "rows"],
        [[reason, count] for reason, count in sorted(
            reason_counts.items(), key=lambda item: (-item[1], item[0]))],
    ))

    lines.append("")
    lines.append("## Top High-Error Examples")
    lines.append("")
    examples = select_examples(rows, high_error_threshold_m, require_miss=False, limit=12)
    lines.append(markdown_table(
        [
            "t_arm",
            "ctx",
            "iekf_err",
            "candidate",
            "reason",
            "resid_h",
            "dx/resid",
            "age",
        ],
        [
            [
                fmt(row.get("time_since_arm_sec"), 2),
                row.get("context", ""),
                fmt(row.get("pair_iekf_error_h_m"), 3),
                int(to_float(row.get("phase_error_memory_candidate_active"), 0.0)),
                row.get("phase_error_memory_reason", ""),
                fmt(row.get("phase_error_memory_residual_h_m"), 3),
                fmt(row.get("phase_error_memory_dx_over_residual_h"), 3),
                fmt(row.get("phase_error_memory_recent_turnpost_age_sec"), 2),
            ]
            for row in examples
        ],
    ))

    (out_dir / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, default=None)
    parser.add_argument("--high-error-threshold-m", type=float, default=0.6)
    parser.add_argument("--examples-limit", type=int, default=50)
    args = parser.parse_args()

    out_dir = (
        args.out_dir if args.out_dir is not None
        else args.run_dir / "phase_error_memory_diag"
    )
    out_dir.mkdir(parents=True, exist_ok=True)

    rows = build_rows(args.run_dir)
    coverage = [
        coverage_row(label, [row for row in rows if in_window(row, start, end)],
                     args.high_error_threshold_m)
        for label, start, end in WINDOWS
    ]
    examples = select_examples(
        rows, args.high_error_threshold_m, require_miss=False,
        limit=max(0, args.examples_limit))
    misses = select_examples(
        rows, args.high_error_threshold_m, require_miss=True,
        limit=max(0, args.examples_limit))

    write_csv(out_dir / "position_update_rows_with_pmem.csv", rows)
    write_csv(out_dir / "coverage_by_window.csv", coverage)
    write_csv(out_dir / "high_error_examples.csv", examples)
    write_csv(out_dir / "high_error_candidate_misses.csv", misses)
    write_report(out_dir, coverage, rows, args.high_error_threshold_m)

    print(f"wrote: {out_dir / 'position_update_rows_with_pmem.csv'}")
    print(f"wrote: {out_dir / 'coverage_by_window.csv'}")
    print(f"wrote: {out_dir / 'high_error_examples.csv'}")
    print(f"wrote: {out_dir / 'high_error_candidate_misses.csv'}")
    print(f"wrote: {out_dir / 'report.md'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
