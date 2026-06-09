#!/usr/bin/env python3
"""Offline oracle-vs-proxy gap diagnostic for PHS5 alpha projection.

Groundtruth is used only to label where alpha projection would have helped.
The diagnostic then checks whether existing online-visible proxy features can
separate those oracle-helpful segments from segments where projection is harmful.
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from typing import Iterable, Sequence

import offline_phs5_measurement_geometry_selector as mg


BASE = Path("/home/yang/kf_gins_ws/artifacts/manual/short_route_generalization_2026-05-07")
DEFAULT_IN = BASE / "phs5_delayed_update_consistency_selector_2026-05-11" / "row_metrics.csv"
DEFAULT_OUT = BASE / "phs5_projection_oracle_gap_diagnostic_2026-05-11"

WINDOWS = [
    ("main_40_180", 40.0, 180.0),
    ("60_100", 60.0, 100.0),
    ("100_120", 100.0, 120.0),
    ("120_140", 120.0, 140.0),
    ("140_160", 140.0, 160.0),
    ("160_180", 160.0, 180.0),
]
TARGET_RUNS = {"shortgen11_repeat2", "shortgen11_current_agelag"}
POSITIVE_RUNS = {
    "shortgen01_phs5c",
    "shortgen02_phs5b",
    "shortgen03_phs5a",
    "shortgen04_hld1a_phs5",
    "shortgen04_current_agelag_lag0",
    "shortgen04_current_lag0255",
}

ORACLE_IMPROVE_M = 0.03
ORACLE_EKF2_MARGIN_M = 0.03
HARMFUL_MARGIN_M = 0.03

PROXY_CANDIDATES = {
    "protected_resid_cos": [
        ("d0_w10_update_count", "ge", 2.0),
        ("d0_w10_resid_proj_cos", "ge", 0.50),
    ],
    "target_dx_cos": [
        ("d5_w10_update_count", "ge", 2.0),
        ("d5_w10_dx_proj_cos", "ge", 0.50),
    ],
}


def finite(value: object) -> bool:
    return mg.finite(value)


def to_float(value: object, default: float = math.nan) -> float:
    return mg.to_float(value, default)


def fmt(value: object, digits: int = 4) -> str:
    return mg.fmt(value, digits)


def mean(values: Iterable[float]) -> float:
    return mg.mean(values)


def rmse(values: Iterable[float]) -> float:
    return mg.rmse(values)


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))


def write_csv(path: Path, rows: Sequence[dict[str, object]]) -> None:
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
    return mg.markdown_table(headers, rows)


def load_rows(path: Path) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for raw in read_csv(path):
        row: dict[str, object] = dict(raw)
        for key, value in raw.items():
            if key not in {"run", "group", "source"}:
                row[key] = to_float(value)
        rows.append(row)
    return rows


def proxy_active(row: dict[str, object], conditions: Sequence[tuple[str, str, float]]) -> bool:
    for feature, op, threshold in conditions:
        value = to_float(row.get(feature))
        if not finite(value):
            return False
        if op == "ge" and value < threshold:
            return False
        if op == "le" and value > threshold:
            return False
    return True


def window_for_time(t: float) -> str:
    specific_first = [window for window in WINDOWS if window[0] != "main_40_180"]
    specific_first.append(("main_40_180", 40.0, 180.0))
    for name, start, end in specific_first:
        if start <= t < end:
            return name
    return "outside"


def segment_rows(rows: Sequence[dict[str, object]]) -> list[dict[str, object]]:
    groups: dict[tuple[str, int], list[dict[str, object]]] = {}
    for row in rows:
        run = str(row.get("run"))
        seg = int(to_float(row.get("segment_index"), -1))
        if seg < 0:
            continue
        groups.setdefault((run, seg), []).append(row)

    segments: list[dict[str, object]] = []
    for (run, seg), items in sorted(groups.items()):
        items.sort(key=lambda row: to_float(row.get("t")))
        start = min(to_float(row.get("t")) for row in items)
        end = max(to_float(row.get("t")) for row in items)
        raw = rmse(to_float(row.get("raw_error_m")) for row in items)
        alpha = rmse(to_float(row.get("alpha_error_m")) for row in items)
        ekf2 = rmse(to_float(row.get("ekf2_error_m")) for row in items)
        oracle_repair = (
            finite(raw)
            and finite(alpha)
            and finite(ekf2)
            and alpha <= raw - ORACLE_IMPROVE_M
            and alpha <= ekf2 + ORACLE_EKF2_MARGIN_M
        )
        harmful = finite(raw) and finite(alpha) and alpha >= raw + HARMFUL_MARGIN_M
        row: dict[str, object] = {
            "run": run,
            "segment_index": seg,
            "start_sec": start,
            "end_sec": end,
            "window": window_for_time(0.5 * (start + end)),
            "rows": len(items),
            "raw_rmse_m": raw,
            "alpha_rmse_m": alpha,
            "ekf2_rmse_m": ekf2,
            "alpha_minus_raw_m": alpha - raw if finite(alpha) and finite(raw) else math.nan,
            "alpha_minus_ekf2_m": alpha - ekf2 if finite(alpha) and finite(ekf2) else math.nan,
            "oracle_repair": 1 if oracle_repair else 0,
            "projection_harmful": 1 if harmful else 0,
            "target_run": 1 if run in TARGET_RUNS else 0,
            "positive_run": 1 if run in POSITIVE_RUNS else 0,
            "main_window": 1 if 40.0 <= 0.5 * (start + end) < 180.0 else 0,
            "target_window": 1 if run in TARGET_RUNS and window_for_time(0.5 * (start + end)) in {"140_160", "160_180"} else 0,
        }
        numeric_keys = sorted(
            {
                key
                for item in items
                for key in item.keys()
                if key not in {"run", "group", "source"}
                and key
                not in {
                    "rawx",
                    "rawy",
                    "px4x",
                    "px4y",
                    "gtx",
                    "gty",
                    "raw_error_m",
                    "alpha_error_m",
                    "ekf2_error_m",
                    "alpha_x",
                    "alpha_y",
                }
            }
        )
        for key in numeric_keys:
            row[f"{key}_mean"] = mean(to_float(item.get(key)) for item in items)
        for name, conditions in PROXY_CANDIDATES.items():
            active_frac = mean(1.0 if proxy_active(item, conditions) else 0.0 for item in items)
            row[f"{name}_active_frac"] = active_frac
            row[f"{name}_active"] = 1 if finite(active_frac) and active_frac >= 0.25 else 0
        segments.append(row)
    return segments


def selected_window_metrics(
    rows: Sequence[dict[str, object]],
    segments: Sequence[dict[str, object]],
) -> list[dict[str, object]]:
    segment_oracle = {
        (str(seg["run"]), int(to_float(seg["segment_index"]))): to_float(seg.get("oracle_repair"), 0.0) > 0.5
        for seg in segments
    }
    output: list[dict[str, object]] = []
    for run in sorted({str(row.get("run")) for row in rows}):
        run_rows = [row for row in rows if row.get("run") == run]
        for window, start, end in WINDOWS:
            subset = [row for row in run_rows if start <= to_float(row.get("t")) < end]
            if not subset:
                continue
            selected_errors = []
            for row in subset:
                seg_key = (run, int(to_float(row.get("segment_index"), -1)))
                use_alpha = segment_oracle.get(seg_key, False)
                selected_errors.append(
                    to_float(row.get("alpha_error_m")) if use_alpha else to_float(row.get("raw_error_m"))
                )
            raw = rmse(to_float(row.get("raw_error_m")) for row in subset)
            alpha = rmse(to_float(row.get("alpha_error_m")) for row in subset)
            selected = rmse(selected_errors)
            ekf2 = rmse(to_float(row.get("ekf2_error_m")) for row in subset)
            active_frac = mean(
                1.0
                if segment_oracle.get((run, int(to_float(row.get("segment_index"), -1))), False)
                else 0.0
                for row in subset
            )
            output.append(
                {
                    "run": run,
                    "window": window,
                    "rows": len(subset),
                    "oracle_active_frac": active_frac,
                    "raw_rmse_m": raw,
                    "alpha_all_rmse_m": alpha,
                    "oracle_selected_rmse_m": selected,
                    "ekf2_rmse_m": ekf2,
                    "oracle_minus_raw_m": selected - raw if finite(selected) and finite(raw) else math.nan,
                    "oracle_minus_ekf2_m": selected - ekf2 if finite(selected) and finite(ekf2) else math.nan,
                }
            )
    return output


def proxy_confusion(segments: Sequence[dict[str, object]]) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for name in PROXY_CANDIDATES:
        for scope_name, filt in [
            ("all", lambda seg: True),
            ("target_windows", lambda seg: to_float(seg.get("target_window"), 0.0) > 0.5),
            (
                "positive_main",
                lambda seg: to_float(seg.get("positive_run"), 0.0) > 0.5
                and to_float(seg.get("main_window"), 0.0) > 0.5,
            ),
        ]:
            scoped = [seg for seg in segments if filt(seg)]
            active = [seg for seg in scoped if to_float(seg.get(f"{name}_active"), 0.0) > 0.5]
            oracle = [seg for seg in scoped if to_float(seg.get("oracle_repair"), 0.0) > 0.5]
            harmful = [seg for seg in scoped if to_float(seg.get("projection_harmful"), 0.0) > 0.5]
            tp = [
                seg
                for seg in scoped
                if to_float(seg.get(f"{name}_active"), 0.0) > 0.5
                and to_float(seg.get("oracle_repair"), 0.0) > 0.5
            ]
            fn = [
                seg
                for seg in scoped
                if to_float(seg.get(f"{name}_active"), 0.0) <= 0.5
                and to_float(seg.get("oracle_repair"), 0.0) > 0.5
            ]
            fp_harm = [
                seg
                for seg in scoped
                if to_float(seg.get(f"{name}_active"), 0.0) > 0.5
                and to_float(seg.get("projection_harmful"), 0.0) > 0.5
            ]
            rows.append(
                {
                    "proxy": name,
                    "scope": scope_name,
                    "segments": len(scoped),
                    "active": len(active),
                    "oracle_repair": len(oracle),
                    "projection_harmful": len(harmful),
                    "tp": len(tp),
                    "fn": len(fn),
                    "fp_harmful": len(fp_harm),
                    "oracle_recall": len(tp) / len(oracle) if oracle else math.nan,
                    "harmful_active_frac": len(fp_harm) / len(harmful) if harmful else math.nan,
                }
            )
    return rows


def feature_contrast(segments: Sequence[dict[str, object]]) -> list[dict[str, object]]:
    repair = [
        seg
        for seg in segments
        if to_float(seg.get("target_window"), 0.0) > 0.5 and to_float(seg.get("oracle_repair"), 0.0) > 0.5
    ]
    bad = [
        seg
        for seg in segments
        if to_float(seg.get("positive_run"), 0.0) > 0.5 and to_float(seg.get("projection_harmful"), 0.0) > 0.5
    ]
    feature_names = sorted(
        key
        for key in segments[0].keys()
        if key.endswith("_mean")
        and not key.startswith(("raw_", "alpha_", "ekf2_"))
        and "error" not in key
        and "gt" not in key
    )
    rows: list[dict[str, object]] = []
    for feature in feature_names:
        repair_values = [to_float(seg.get(feature)) for seg in repair if finite(seg.get(feature))]
        bad_values = [to_float(seg.get(feature)) for seg in bad if finite(seg.get(feature))]
        if len(repair_values) < 2 or len(bad_values) < 2:
            continue
        repair_mean = mean(repair_values)
        bad_mean = mean(bad_values)
        rows.append(
            {
                "feature": feature,
                "target_oracle_repair_mean": repair_mean,
                "positive_harmful_mean": bad_mean,
                "mean_delta_repair_minus_bad": repair_mean - bad_mean,
                "repair_count": len(repair_values),
                "bad_count": len(bad_values),
            }
        )
    rows.sort(key=lambda row: abs(to_float(row.get("mean_delta_repair_minus_bad"))), reverse=True)
    return rows


def threshold_scan(segments: Sequence[dict[str, object]]) -> list[dict[str, object]]:
    repair = [
        seg
        for seg in segments
        if to_float(seg.get("target_window"), 0.0) > 0.5 and to_float(seg.get("oracle_repair"), 0.0) > 0.5
    ]
    bad = [
        seg
        for seg in segments
        if to_float(seg.get("positive_run"), 0.0) > 0.5 and to_float(seg.get("projection_harmful"), 0.0) > 0.5
    ]
    if not repair or not bad:
        return []
    feature_names = [
        key
        for key in segments[0].keys()
        if key.endswith("_mean")
        and not key.startswith(("raw_", "alpha_", "ekf2_"))
        and "error" not in key
        and "gt" not in key
    ]
    rows: list[dict[str, object]] = []
    for feature in feature_names:
        values = sorted({to_float(seg.get(feature)) for seg in repair + bad if finite(seg.get(feature))})
        if len(values) < 4:
            continue
        candidates = values[1:-1]
        if len(candidates) > 30:
            step = max(1, len(candidates) // 30)
            candidates = candidates[::step]
        for op in ("ge", "le"):
            best: dict[str, object] | None = None
            for threshold in candidates:
                if op == "ge":
                    repair_hit = sum(to_float(seg.get(feature)) >= threshold for seg in repair)
                    bad_hit = sum(to_float(seg.get(feature)) >= threshold for seg in bad)
                else:
                    repair_hit = sum(to_float(seg.get(feature)) <= threshold for seg in repair)
                    bad_hit = sum(to_float(seg.get(feature)) <= threshold for seg in bad)
                recall = repair_hit / len(repair)
                bad_active = bad_hit / len(bad)
                score = recall - bad_active
                row = {
                    "feature": feature,
                    "op": op,
                    "threshold": threshold,
                    "target_repair_recall": recall,
                    "positive_harmful_active": bad_active,
                    "score": score,
                    "repair_hit": repair_hit,
                    "repair_total": len(repair),
                    "bad_hit": bad_hit,
                    "bad_total": len(bad),
                }
                if best is None or score > to_float(best.get("score")):
                    best = row
            if best is not None:
                rows.append(best)
    rows.sort(key=lambda row: (to_float(row.get("score")), to_float(row.get("target_repair_recall"))), reverse=True)
    return rows


def write_report(
    out_dir: Path,
    segments: Sequence[dict[str, object]],
    oracle_metrics: Sequence[dict[str, object]],
    confusion: Sequence[dict[str, object]],
    contrast: Sequence[dict[str, object]],
    thresholds: Sequence[dict[str, object]],
    input_path: Path,
) -> None:
    oracle_count = sum(to_float(seg.get("oracle_repair"), 0.0) > 0.5 for seg in segments)
    harmful_count = sum(to_float(seg.get("projection_harmful"), 0.0) > 0.5 for seg in segments)
    target_oracle = sum(
        to_float(seg.get("oracle_repair"), 0.0) > 0.5 and to_float(seg.get("target_window"), 0.0) > 0.5
        for seg in segments
    )
    target_metrics = [
        row
        for row in oracle_metrics
        if row["run"] in TARGET_RUNS and row["window"] in {"140_160", "160_180", "main_40_180"}
    ]
    target_metric_table = [
        [
            row["run"],
            row["window"],
            fmt(row["oracle_active_frac"], 3),
            fmt(row["raw_rmse_m"]),
            fmt(row["alpha_all_rmse_m"]),
            fmt(row["oracle_selected_rmse_m"]),
            fmt(row["ekf2_rmse_m"]),
            fmt(row["oracle_minus_raw_m"]),
            fmt(row["oracle_minus_ekf2_m"]),
        ]
        for row in target_metrics
    ]
    confusion_table = [
        [
            row["proxy"],
            row["scope"],
            row["segments"],
            row["active"],
            row["oracle_repair"],
            row["projection_harmful"],
            row["tp"],
            row["fn"],
            row["fp_harmful"],
            fmt(row["oracle_recall"], 3),
            fmt(row["harmful_active_frac"], 3),
        ]
        for row in confusion
    ]
    contrast_table = [
        [
            row["feature"],
            fmt(row["target_oracle_repair_mean"]),
            fmt(row["positive_harmful_mean"]),
            fmt(row["mean_delta_repair_minus_bad"]),
        ]
        for row in contrast[:15]
    ]
    threshold_table = [
        [
            row["feature"],
            row["op"],
            fmt(row["threshold"]),
            fmt(row["target_repair_recall"], 3),
            fmt(row["positive_harmful_active"], 3),
            fmt(row["score"], 3),
            f"{int(to_float(row['repair_hit']))}/{int(to_float(row['repair_total']))}",
            f"{int(to_float(row['bad_hit']))}/{int(to_float(row['bad_total']))}",
        ]
        for row in thresholds[:15]
    ]
    nontiming_thresholds = [
        row
        for row in thresholds
        if not any(
            marker in str(row.get("feature"))
            for marker in (
                "gnss_source_age",
                "stamp_lag",
                "t_mean",
                "segment_index",
                "update_count",
            )
        )
    ][:15]
    nontiming_threshold_table = [
        [
            row["feature"],
            row["op"],
            fmt(row["threshold"]),
            fmt(row["target_repair_recall"], 3),
            fmt(row["positive_harmful_active"], 3),
            fmt(row["score"], 3),
            f"{int(to_float(row['repair_hit']))}/{int(to_float(row['repair_total']))}",
            f"{int(to_float(row['bad_hit']))}/{int(to_float(row['bad_total']))}",
        ]
        for row in nontiming_thresholds
    ]
    report = [
        "# PHS5 projection oracle-gap diagnostic",
        "",
        "Date: 2026-05-11",
        "",
        "## Scope",
        "",
        "Offline-only diagnostic. It labels 10 s segments using groundtruth to estimate where alpha projection would have helped, then compares that oracle label to online-visible proxy features already present in the delayed update-consistency replay table.",
        "",
        f"- input rows: `{input_path}`",
        f"- oracle repair label: `alpha_rmse <= raw_rmse - {ORACLE_IMPROVE_M:.2f} m` and `alpha_rmse <= EKF2_rmse + {ORACLE_EKF2_MARGIN_M:.2f} m`",
        f"- harmful label: `alpha_rmse >= raw_rmse + {HARMFUL_MARGIN_M:.2f} m`",
        "",
        "## Segment Counts",
        "",
        f"- segments: `{len(segments)}`",
        f"- oracle repair segments: `{oracle_count}`",
        f"- projection harmful segments: `{harmful_count}`",
        f"- target-window oracle repair segments: `{target_oracle}`",
        "",
        "## Oracle Upper Bound",
        "",
        markdown_table(
            [
                "run",
                "window",
                "oracle active",
                "raw",
                "alpha all",
                "oracle selected",
                "EKF2",
                "oracle-raw",
                "oracle-EKF2",
            ],
            target_metric_table,
        ),
        "",
        "## Proxy Confusion",
        "",
        markdown_table(
            [
                "proxy",
                "scope",
                "segments",
                "active",
                "oracle",
                "harmful",
                "tp",
                "fn",
                "fp harmful",
                "recall",
                "harm active",
            ],
            confusion_table,
        ),
        "",
        "## Feature Contrast",
        "",
        "Target oracle-repair segments are compared with positive/control segments where projection is harmful.",
        "",
        markdown_table(
            ["feature", "target oracle mean", "positive harmful mean", "delta"],
            contrast_table,
        ),
        "",
        "## Best Single-Feature Thresholds",
        "",
        "This table is intentionally shown first because it exposes why the old timing proxies looked tempting offline. Timing and phase-like fields in this table are not accepted as deployable selectors after the online misses.",
        "",
        markdown_table(
            ["feature", "op", "threshold", "target recall", "bad active", "score", "repair hit", "bad hit"],
            threshold_table,
        ),
        "",
        "## Best Non-Timing Thresholds",
        "",
        markdown_table(
            ["feature", "op", "threshold", "target recall", "bad active", "score", "repair hit", "bad hit"],
            nontiming_threshold_table,
        ),
        "",
        "## Readout",
        "",
        "The oracle upper bound confirms that alpha projection can repair the shortgen11 target windows if the trigger were known. The tested online-visible proxies fail for two different reasons: the protected residual-cos proxy avoids positive harm but misses most target-window oracle repairs; the target dx-cos proxy catches more target repair but activates on many positive/control segments where alpha projection is harmful.",
        "",
        "A one-dimensional threshold scan rediscovers timing/phase proxies (`gnss_source_age_sec`, `stamp_lag_sec`, segment time/index), which are already rejected by the online shortgen04 misses. After excluding timing/phase-like fields, the best thresholds either recall only part of the target repairs or still activate on many positive harmful segments. This points to missing online information rather than a small threshold tuning issue.",
        "",
        "Generated files:",
        "",
        f"- `{out_dir / 'segment_oracle_metrics.csv'}`",
        f"- `{out_dir / 'oracle_window_metrics.csv'}`",
        f"- `{out_dir / 'proxy_confusion.csv'}`",
        f"- `{out_dir / 'feature_contrast.csv'}`",
        f"- `{out_dir / 'single_feature_threshold_scan.csv'}`",
    ]
    (out_dir / "report.md").write_text("\n".join(report) + "\n")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, default=DEFAULT_IN)
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUT)
    args = parser.parse_args()

    rows = load_rows(args.input)
    segments = segment_rows(rows)
    oracle_metrics = selected_window_metrics(rows, segments)
    confusion = proxy_confusion(segments)
    contrast = feature_contrast(segments)
    thresholds = threshold_scan(segments)

    args.out_dir.mkdir(parents=True, exist_ok=True)
    write_csv(args.out_dir / "segment_oracle_metrics.csv", segments)
    write_csv(args.out_dir / "oracle_window_metrics.csv", oracle_metrics)
    write_csv(args.out_dir / "proxy_confusion.csv", confusion)
    write_csv(args.out_dir / "feature_contrast.csv", contrast)
    write_csv(args.out_dir / "single_feature_threshold_scan.csv", thresholds)
    write_report(args.out_dir, segments, oracle_metrics, confusion, contrast, thresholds, args.input)
    print(f"segments={len(segments)} oracle={sum(to_float(s.get('oracle_repair'), 0.0) > 0.5 for s in segments)}")
    print(args.out_dir / "report.md")


if __name__ == "__main__":
    main()
