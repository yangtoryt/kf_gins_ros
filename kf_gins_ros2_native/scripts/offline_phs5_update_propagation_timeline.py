#!/usr/bin/env python3
"""Split PHS5 error evolution into accepted-update and propagation deltas."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from typing import Iterable, Sequence


BASE = Path("/home/yang/kf_gins_ws/artifacts/manual/short_route_generalization_2026-05-07")
DEFAULT_INPUT = BASE / "phs5_innovation_covariance_diagnostic_2026-05-10" / "innovation_update_rows.csv"
DEFAULT_OUT = BASE / "phs5_update_propagation_timeline_2026-05-10"
WINDOWS = [
    ("main_40_180", 40.0, 180.0),
    ("120_140", 120.0, 140.0),
    ("140_160", 140.0, 160.0),
    ("160_180", 160.0, 180.0),
    ("140_180", 140.0, 180.0),
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


def load_rows(path: Path) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for raw in read_csv(path):
        row: dict[str, object] = dict(raw)
        for key, value in raw.items():
            if key not in {"run", "role", "group", "mavros_mode", "publish_projection_action"}:
                row[key] = to_float(value)
        rows.append(row)
    rows.sort(key=lambda row: (str(row.get("run")), to_float(row.get("time_since_arm_sec"))))
    return rows


def in_window(t: float, start: float, end: float) -> bool:
    return finite(t) and start <= t < end


def run_groups(rows: Sequence[dict[str, object]]) -> dict[str, list[dict[str, object]]]:
    out: dict[str, list[dict[str, object]]] = {}
    for row in rows:
        out.setdefault(str(row.get("run")), []).append(row)
    for group in out.values():
        group.sort(key=lambda row: to_float(row.get("time_since_arm_sec")))
    return out


def propagation_intervals(rows: Sequence[dict[str, object]]) -> list[dict[str, object]]:
    intervals: list[dict[str, object]] = []
    for current, nxt in zip(rows, rows[1:]):
        t0 = to_float(current.get("time_since_arm_sec"))
        t1 = to_float(nxt.get("time_since_arm_sec"))
        if not finite(t0) or not finite(t1) or t1 <= t0:
            continue
        after = to_float(current.get("update_after_error_h_m"))
        next_before = to_float(nxt.get("update_before_error_h_m"))
        delta = next_before - after if finite(after) and finite(next_before) else math.nan
        intervals.append(
            {
                "run": current.get("run"),
                "group": current.get("group"),
                "start_sec": t0,
                "end_sec": t1,
                "dt_sec": t1 - t0,
                "propagation_delta_m": delta,
                "propagation_rate_mps": delta / (t1 - t0) if finite(delta) and t1 > t0 else math.nan,
                "start_after_error_m": after,
                "end_before_error_m": next_before,
            }
        )
    return intervals


def summarize_window(run: str, group: str, updates: Sequence[dict[str, object]], props: Sequence[dict[str, object]], label: str, start: float, end: float) -> dict[str, object]:
    up = [row for row in updates if in_window(to_float(row.get("time_since_arm_sec")), start, end)]
    pr = [row for row in props if in_window(to_float(row.get("start_sec")), start, end)]
    first = up[0] if up else None
    last = up[-1] if up else None
    update_sum = sum(to_float(row.get("update_error_delta_h_m")) for row in up if finite(row.get("update_error_delta_h_m")))
    prop_sum = sum(to_float(row.get("propagation_delta_m")) for row in pr if finite(row.get("propagation_delta_m")))
    return {
        "run": run,
        "group": group,
        "window": label,
        "update_rows": len(up),
        "propagation_intervals": len(pr),
        "window_first_after_error_m": to_float(first.get("update_after_error_h_m")) if first else math.nan,
        "window_last_after_error_m": to_float(last.get("update_after_error_h_m")) if last else math.nan,
        "window_after_error_change_m": (
            to_float(last.get("update_after_error_h_m")) - to_float(first.get("update_after_error_h_m"))
            if first and last else math.nan
        ),
        "iekf_rmse_m": rmse(to_float(row.get("iekf_error_m")) for row in up),
        "ekf2_rmse_m": rmse(to_float(row.get("ekf2_error_m")) for row in up),
        "iekf_minus_ekf2_rmse_m": rmse(to_float(row.get("iekf_error_m")) for row in up) - rmse(to_float(row.get("ekf2_error_m")) for row in up),
        "update_delta_sum_m": update_sum,
        "update_delta_mean_m": mean(to_float(row.get("update_error_delta_h_m")) for row in up),
        "update_worsen_frac": mean(1.0 if to_float(row.get("update_error_delta_h_m")) > 0.0 else 0.0 for row in up),
        "propagation_delta_sum_m": prop_sum,
        "propagation_delta_mean_m": mean(to_float(row.get("propagation_delta_m")) for row in pr),
        "propagation_worsen_frac": mean(1.0 if to_float(row.get("propagation_delta_m")) > 0.0 else 0.0 for row in pr),
        "update_plus_prop_sum_m": update_sum + prop_sum,
        "formal_hnis_mean": mean(to_float(row.get("formal_hnis_2d")) for row in up),
        "dx_over_residual_mean": mean(to_float(row.get("dx_over_residual_h")) for row in up),
    }


def build_summaries(rows: Sequence[dict[str, object]]) -> tuple[list[dict[str, object]], list[dict[str, object]]]:
    intervals_all: list[dict[str, object]] = []
    summaries: list[dict[str, object]] = []
    for run, updates in run_groups(rows).items():
        group = str(updates[0].get("group")) if updates else ""
        props = propagation_intervals(updates)
        intervals_all.extend(props)
        for label, start, end in WINDOWS:
            summaries.append(summarize_window(run, group, updates, props, label, start, end))
    return summaries, intervals_all


def group_summaries(window_rows: Sequence[dict[str, object]]) -> list[dict[str, object]]:
    out: list[dict[str, object]] = []
    for group in ("positive", "target"):
        for label, _start, _end in WINDOWS:
            rows = [
                row for row in window_rows
                if row.get("group") == group and row.get("window") == label and to_float(row.get("update_rows")) > 0.0
            ]
            weights = [to_float(row.get("update_rows")) for row in rows]

            def wmean(field: str) -> float:
                vals = [(to_float(row.get(field)), to_float(row.get("update_rows"))) for row in rows]
                vals = [(value, weight) for value, weight in vals if finite(value) and finite(weight) and weight > 0.0]
                total = sum(weight for _value, weight in vals)
                return sum(value * weight for value, weight in vals) / total if total > 0.0 else math.nan

            out.append(
                {
                    "group": group,
                    "window": label,
                    "runs": len(rows),
                    "update_rows": sum(weights),
                    "iekf_minus_ekf2_rmse_m": wmean("iekf_minus_ekf2_rmse_m"),
                    "first_after_error_m": wmean("window_first_after_error_m"),
                    "after_error_change_m": wmean("window_after_error_change_m"),
                    "update_delta_mean_m": wmean("update_delta_mean_m"),
                    "update_worsen_frac": wmean("update_worsen_frac"),
                    "propagation_delta_mean_m": wmean("propagation_delta_mean_m"),
                    "propagation_worsen_frac": wmean("propagation_worsen_frac"),
                    "update_plus_prop_sum_m": wmean("update_plus_prop_sum_m"),
                    "formal_hnis_mean": wmean("formal_hnis_mean"),
                    "dx_over_residual_mean": wmean("dx_over_residual_mean"),
                }
            )
    return out


def table(rows: Sequence[dict[str, object]], run_col: str = "run") -> list[list[object]]:
    return [
        [
            row[run_col],
            row["window"],
            int(to_float(row["update_rows"], 0.0)),
            fmt(row["iekf_minus_ekf2_rmse_m"]),
            fmt(row.get("window_first_after_error_m", row.get("first_after_error_m"))),
            fmt(row.get("window_after_error_change_m", row.get("after_error_change_m"))),
            fmt(row["update_delta_mean_m"]),
            fmt(row["update_worsen_frac"]),
            fmt(row["propagation_delta_mean_m"]),
            fmt(row["propagation_worsen_frac"]),
            fmt(row["formal_hnis_mean"]),
            fmt(row["dx_over_residual_mean"]),
        ]
        for row in rows
    ]


def write_report(out_dir: Path, window_rows: list[dict[str, object]], group_rows: list[dict[str, object]]) -> None:
    group_focus = [row for row in group_rows if row.get("window") in {"140_160", "160_180", "140_180"}]
    run_focus = [
        row for row in window_rows
        if row.get("window") in {"120_140", "140_160", "160_180"}
        and (row.get("group") == "target" or row.get("run") == "shortgen04_hld1a_phs5")
    ]
    headers = [
        "id",
        "window",
        "rows",
        "IEKF-EKF2",
        "first err",
        "err change",
        "upd mean",
        "upd worsen",
        "prop mean",
        "prop worsen",
        "HNIS",
        "dx/resid",
    ]
    lines = [
        "# PHS5 update/propagation timeline split",
        "",
        "Date: 2026-05-10",
        "",
        "Offline-only split built from `phs5_innovation_covariance_diagnostic_2026-05-10/innovation_update_rows.csv`. No flight stack, rebuild, or estimator process was started.",
        "",
        "## Group Focus",
        "",
        markdown_table(headers, table(group_focus, "group")),
        "",
        "## Target And Holdout Focus",
        "",
        markdown_table(headers, table(run_focus, "run")),
        "",
        "## Readout",
        "",
        "- `upd mean` is accepted GNSS position update after-error minus before-error; negative means the update reduces error.",
        "- `prop mean` is next-update before-error minus current-update after-error; positive means between-update propagation grows error.",
        "- `first err` and `err change` show whether the window starts already high and whether it grows across the window.",
        "",
        "Generated files:",
        f"- `{out_dir / 'timeline_window_summary.csv'}`",
        f"- `{out_dir / 'timeline_group_summary.csv'}`",
        f"- `{out_dir / 'timeline_propagation_intervals.csv'}`",
    ]
    (out_dir / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", default=str(DEFAULT_INPUT))
    parser.add_argument("--out-dir", default=str(DEFAULT_OUT))
    args = parser.parse_args()

    rows = load_rows(Path(args.input))
    window_rows, intervals = build_summaries(rows)
    group_rows = group_summaries(window_rows)
    out_dir = Path(args.out_dir)
    write_csv(out_dir / "timeline_window_summary.csv", window_rows)
    write_csv(out_dir / "timeline_group_summary.csv", group_rows)
    write_csv(out_dir / "timeline_propagation_intervals.csv", intervals)
    write_report(out_dir, window_rows, group_rows)
    print(f"wrote: {out_dir / 'report.md'}")


if __name__ == "__main__":
    main()
