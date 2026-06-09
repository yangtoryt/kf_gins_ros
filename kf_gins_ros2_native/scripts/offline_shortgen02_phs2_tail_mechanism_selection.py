#!/usr/bin/env python3
"""Select likely mechanisms for the PHS2 already-projected shortgen02 tail."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from typing import Iterable


PHASE_WINDOWS = [
    ("120-140", 120.0, 140.0),
    ("140-160", 140.0, 160.0),
    ("160-180", 160.0, 180.0),
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


def phase_window(t_arm: float) -> str:
    for label, start, end in PHASE_WINDOWS:
        if start <= t_arm < end:
            return label
    return "other"


def bias_by(rows: list[dict[str, str]], key_fn) -> dict[str, tuple[float, float]]:
    groups: dict[str, list[dict[str, str]]] = {}
    for row in rows:
        key = key_fn(row)
        groups.setdefault(key, []).append(row)
    out: dict[str, tuple[float, float]] = {}
    for key, items in groups.items():
        out[key] = (
            mean(to_float(row.get("iekf_error_x_m")) for row in items),
            mean(to_float(row.get("iekf_error_y_m")) for row in items),
        )
    return out


def classify_rows(
    all_rows: list[dict[str, str]],
    high_rows: list[dict[str, str]],
    high_threshold_m: float,
) -> list[dict[str, object]]:
    phase_bias = bias_by(
        [
            row for row in all_rows
            if 120.0 <= to_float(row.get("time_since_arm_sec")) < 180.0
        ],
        lambda row: phase_window(to_float(row.get("time_since_arm_sec"))),
    )
    context_bias = bias_by(high_rows, lambda row: str(row.get("context", "none")))
    classified: list[dict[str, object]] = []
    for row in high_rows:
        t_arm = to_float(row.get("time_since_arm_sec"))
        phase = phase_window(t_arm)
        ctx = str(row.get("context", "none"))
        ex = to_float(row.get("iekf_error_x_m"))
        ey = to_float(row.get("iekf_error_y_m"))
        phase_bx, phase_by = phase_bias.get(phase, (math.nan, math.nan))
        ctx_bx, ctx_by = context_bias.get(ctx, (math.nan, math.nan))
        phase_local_error = norm2(ex - phase_bx, ey - phase_by)
        ctx_local_error = norm2(ex - ctx_bx, ey - ctx_by)

        gps_all = to_float(row.get("gps_fit_all_error_h_m"))
        gps_moving = to_float(row.get("gps_fit_moving_error_h_m"))
        moving_gain = gps_all - gps_moving if finite(gps_all) and finite(gps_moving) else math.nan
        gnss_meas = to_float(row.get("geometry_inferred_gnss_measurement_error_h_m"))
        growth_delta = to_float(row.get("interval_iekf_minus_ekf2_vector_change_m"))
        dx_over_resid = to_float(row.get("dx_over_residual_h"))
        gnss_resid = to_float(row.get("gnss_residual_h_m"))
        vdiff = to_float(row.get("core_native_velocity_mismatch_h_mps"))

        classified.append({
            **row,
            "phase_window": phase,
            "phase_bias_x_m": phase_bx,
            "phase_bias_y_m": phase_by,
            "phase_local_error_h_m": phase_local_error,
            "context_bias_x_m": ctx_bx,
            "context_bias_y_m": ctx_by,
            "context_local_error_h_m": ctx_local_error,
            "gps_moving_fit_gain_m": moving_gain,
            "flag_phase_bias_explains": int(phase_local_error <= high_threshold_m),
            "flag_context_bias_explains": int(ctx_local_error <= high_threshold_m),
            "flag_measurement_frame": int(
                (finite(moving_gain) and moving_gain >= 0.10)
                or (finite(gnss_meas) and gnss_meas >= 0.35)
            ),
            "flag_propagation_growth": int(finite(growth_delta) and growth_delta >= 0.10),
            "flag_update_underresponse": int(
                finite(dx_over_resid) and dx_over_resid <= 0.20
                and finite(gnss_resid) and gnss_resid >= 0.15
            ),
            "flag_velocity_mismatch": int(finite(vdiff) and vdiff >= 0.10),
        })
    return classified


def flag_summary(rows: list[dict[str, object]]) -> list[dict[str, object]]:
    flags = [
        "flag_phase_bias_explains",
        "flag_context_bias_explains",
        "flag_measurement_frame",
        "flag_propagation_growth",
        "flag_update_underresponse",
        "flag_velocity_mismatch",
    ]
    out = []
    for flag in flags:
        vals = [to_float(row.get(flag), 0.0) for row in rows]
        flagged = int(sum(1 for val in vals if val > 0.5))
        out.append({
            "mechanism_flag": flag.removeprefix("flag_"),
            "rows": len(rows),
            "flagged_rows": flagged,
            "coverage": flagged / len(rows) if rows else math.nan,
            "mean_iekf_error_m": mean(
                to_float(row.get("iekf_error_h_m")) for row in rows if to_float(row.get(flag), 0.0) > 0.5),
            "mean_ekf2_error_m": mean(
                to_float(row.get("ekf2_error_h_m")) for row in rows if to_float(row.get(flag), 0.0) > 0.5),
        })
    return out


def pair_overlap(rows: list[dict[str, object]]) -> list[dict[str, object]]:
    flags = [
        "flag_measurement_frame",
        "flag_propagation_growth",
        "flag_update_underresponse",
        "flag_velocity_mismatch",
    ]
    out: list[dict[str, object]] = []
    for left in flags:
        for right in flags:
            count = sum(
                1 for row in rows
                if to_float(row.get(left), 0.0) > 0.5 and to_float(row.get(right), 0.0) > 0.5
            )
            out.append({
                "left": left.removeprefix("flag_"),
                "right": right.removeprefix("flag_"),
                "overlap_rows": count,
                "overlap_frac": count / len(rows) if rows else math.nan,
            })
    return out


def context_summary(rows: list[dict[str, object]]) -> list[dict[str, object]]:
    groups: dict[str, list[dict[str, object]]] = {}
    for row in rows:
        groups.setdefault(str(row.get("context", "none")), []).append(row)
    out = []
    for ctx, items in sorted(groups.items()):
        out.append({
            "context": ctx,
            "rows": len(items),
            "iekf_mean_m": mean(to_float(row.get("iekf_error_h_m")) for row in items),
            "ekf2_mean_m": mean(to_float(row.get("ekf2_error_h_m")) for row in items),
            "measurement_frame_cov": mean(to_float(row.get("flag_measurement_frame"), 0.0) for row in items),
            "propagation_growth_cov": mean(to_float(row.get("flag_propagation_growth"), 0.0) for row in items),
            "update_underresponse_cov": mean(to_float(row.get("flag_update_underresponse"), 0.0) for row in items),
            "velocity_mismatch_cov": mean(to_float(row.get("flag_velocity_mismatch"), 0.0) for row in items),
        })
    return out


def numeric_summary(rows: list[dict[str, object]]) -> list[dict[str, object]]:
    metrics = [
        ("iekf_error_h_m", "IEKF high-row error"),
        ("ekf2_error_h_m", "EKF2 same-row error"),
        ("geometry_inferred_gnss_measurement_error_h_m", "GNSS measurement error"),
        ("gps_moving_fit_gain_m", "moving-fit gain"),
        ("interval_iekf_minus_ekf2_vector_change_m", "IEKF-EKF2 interval growth"),
        ("dx_over_residual_h", "update dx/residual"),
        ("core_native_velocity_mismatch_h_mps", "core/native velocity mismatch"),
        ("phase_local_error_h_m", "phase-local recentered IEKF error"),
        ("context_local_error_h_m", "context-local recentered IEKF error"),
    ]
    out = []
    for key, label in metrics:
        vals = [to_float(row.get(key)) for row in rows]
        out.append({
            "metric": key,
            "label": label,
            "mean": mean(vals),
            "p50": percentile(vals, 50.0),
            "p95": percentile(vals, 95.0),
        })
    return out


def write_report(
    out_dir: Path,
    classified: list[dict[str, object]],
    flags: list[dict[str, object]],
    contexts: list[dict[str, object]],
    numeric: list[dict[str, object]],
    overlaps: list[dict[str, object]],
    high_threshold_m: float,
) -> None:
    def row_flag(name: str) -> dict[str, object]:
        return next((row for row in flags if row["mechanism_flag"] == name), {})

    measurement = row_flag("measurement_frame")
    propagation = row_flag("propagation_growth")
    update = row_flag("update_underresponse")
    velocity = row_flag("velocity_mismatch")

    lines = [
        "# PHS2 Tail Mechanism Selection",
        "",
        "This report classifies the PHS2 already-projected online `/kf_gins/odom` high-tail rows.",
        f"High-tail threshold: `{high_threshold_m:.3f} m`.",
        "",
        "## Mechanism Coverage",
        "",
        markdown_table(
            ["mechanism", "rows", "coverage", "iekf_mean", "ekf2_mean"],
            [
                [
                    row["mechanism_flag"],
                    int(to_float(row.get("flagged_rows"), 0.0)),
                    fmt(row.get("coverage"), 3),
                    fmt(row.get("mean_iekf_error_m")),
                    fmt(row.get("mean_ekf2_error_m")),
                ]
                for row in flags
            ],
        ),
        "",
        "## Numeric Signals",
        "",
        markdown_table(
            ["metric", "mean", "p50", "p95"],
            [
                [row["label"], fmt(row.get("mean")), fmt(row.get("p50")), fmt(row.get("p95"))]
                for row in numeric
            ],
        ),
        "",
        "## Context Split",
        "",
        markdown_table(
            [
                "context", "rows", "iekf", "ekf2", "meas", "prop",
                "update", "vdiff",
            ],
            [
                [
                    row["context"],
                    int(to_float(row.get("rows"), 0.0)),
                    fmt(row.get("iekf_mean_m")),
                    fmt(row.get("ekf2_mean_m")),
                    fmt(row.get("measurement_frame_cov"), 3),
                    fmt(row.get("propagation_growth_cov"), 3),
                    fmt(row.get("update_underresponse_cov"), 3),
                    fmt(row.get("velocity_mismatch_cov"), 3),
                ]
                for row in contexts
            ],
        ),
        "",
        "## Largest Rows",
        "",
        markdown_table(
            [
                "t", "ctx", "iekf", "ekf2", "meas", "move_gain", "growth",
                "dx/res", "vdiff", "phase_local",
            ],
            [
                [
                    fmt(row.get("time_since_arm_sec"), 1),
                    row.get("context", ""),
                    fmt(row.get("iekf_error_h_m")),
                    fmt(row.get("ekf2_error_h_m")),
                    fmt(row.get("geometry_inferred_gnss_measurement_error_h_m")),
                    fmt(row.get("gps_moving_fit_gain_m")),
                    fmt(row.get("interval_iekf_minus_ekf2_vector_change_m")),
                    fmt(row.get("dx_over_residual_h")),
                    fmt(row.get("core_native_velocity_mismatch_h_mps")),
                    fmt(row.get("phase_local_error_h_m")),
                ]
                for row in sorted(
                    classified,
                    key=lambda item: to_float(item.get("iekf_error_h_m")),
                    reverse=True,
                )[:20]
            ],
        ),
        "",
        "## Recommendation",
        "",
        (
            f"- Measurement/frame evidence covers `{int(to_float(measurement.get('flagged_rows'), 0.0))}` "
            f"of `{len(classified)}` high rows; propagation-growth evidence covers "
            f"`{int(to_float(propagation.get('flagged_rows'), 0.0))}` rows."
        ),
        (
            f"- Update-underresponse evidence covers `{int(to_float(update.get('flagged_rows'), 0.0))}` rows, "
            "but the separate core-frame scale sweep showed that blind response scaling is too weak and worsens many rows."
        ),
        (
            f"- Velocity mismatch covers `{int(to_float(velocity.get('flagged_rows'), 0.0))}` rows, "
            "mostly as a contributor in post-turn/turning, not a standalone keeper."
        ),
        "- Next mechanism work should stay offline and target propagation/velocity-state error growth plus measurement-frame treatment; do not restart PGR/TVD/turn-gate tuning from this evidence.",
        "",
        "Generated files:",
        f"- `{out_dir / 'mechanism_classified_high_rows.csv'}`",
        f"- `{out_dir / 'mechanism_flag_summary.csv'}`",
        f"- `{out_dir / 'mechanism_context_summary.csv'}`",
        f"- `{out_dir / 'mechanism_numeric_summary.csv'}`",
        f"- `{out_dir / 'mechanism_overlap.csv'}`",
    ]
    (out_dir / "report.md").write_text("\n".join(lines) + "\n")
    write_csv(out_dir / "mechanism_overlap.csv", overlaps)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", required=True, type=Path)
    parser.add_argument("--out-dir", type=Path)
    parser.add_argument("--high-error-threshold-m", type=float, default=0.3)
    args = parser.parse_args()

    run_dir = args.run_dir
    out_dir = args.out_dir or run_dir / "shortgen02_phs2_tail_mechanism_selection"
    out_dir.mkdir(parents=True, exist_ok=True)

    all_rows = read_csv(run_dir / "offline_groundtruth_convergence_diag" / "groundtruth_joined.csv")
    all_rows = [
        row for row in all_rows
        if 120.0 <= to_float(row.get("time_since_arm_sec")) < 180.0
        and to_float(row.get("mavros_armed"), 0.0) > 0.5
    ]
    high_rows = read_csv(run_dir / "shortgen02_phs2_tail_diag" / "phs2_tail_high_rows.csv")
    classified = classify_rows(all_rows, high_rows, args.high_error_threshold_m)
    flags = flag_summary(classified)
    contexts = context_summary(classified)
    numeric = numeric_summary(classified)
    overlaps = pair_overlap(classified)

    write_csv(out_dir / "mechanism_classified_high_rows.csv", classified)
    write_csv(out_dir / "mechanism_flag_summary.csv", flags)
    write_csv(out_dir / "mechanism_context_summary.csv", contexts)
    write_csv(out_dir / "mechanism_numeric_summary.csv", numeric)
    write_report(out_dir, classified, flags, contexts, numeric, overlaps, args.high_error_threshold_m)

    print(f"wrote: {out_dir / 'report.md'}")
    print(f"wrote: {out_dir / 'mechanism_classified_high_rows.csv'}")
    print(f"wrote: {out_dir / 'mechanism_flag_summary.csv'}")
    print(f"wrote: {out_dir / 'mechanism_context_summary.csv'}")
    print(f"wrote: {out_dir / 'mechanism_numeric_summary.csv'}")
    print(f"wrote: {out_dir / 'mechanism_overlap.csv'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
