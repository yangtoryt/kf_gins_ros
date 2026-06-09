#!/usr/bin/env python3
"""Scan causal early-recovery gates from existing short-route logs.

This converts the non-causal 40-80s recovery signature into trailing-window
signals that an online diagnostic hook could observe without looking ahead.
It is offline-only and does not run the simulator or estimator.
"""

from __future__ import annotations

import argparse
import itertools
import math
from dataclasses import dataclass
from pathlib import Path

import pandas as pd

from offline_shortgen11_early_basin_recovery_diagnostic import (
    GATE_PROBE_RUN_DIRS,
    finite,
    fmt,
    load_events_for,
    markdown_table,
    mean,
    numeric,
    to_float,
    write_csv,
)
from offline_shortgen11_gnss_position_vertical_path_diagnostic import position_events


BASE = Path("/home/yang/kf_gins_ws/artifacts/manual/short_route_generalization_2026-05-07")
DEFAULT_OUT = BASE / "shortgen11_early_recovery_causal_gate_scan_2026-05-11"

NEGATIVE_RUNS = {
    "shortgen11_repeat2",
    "shortgen11_coretrace",
    "shortgen11_accbiasz_diag",
    "shortgen11_accbiasz_apply",
}
PROTECTION_RUNS = [run for run in GATE_PROBE_RUN_DIRS if run not in NEGATIVE_RUNS]


@dataclass(frozen=True)
class GateConfig:
    history_sec: float
    ba_z_mean_max: float
    res_u_mean_max: float
    core_u_mean_min: float
    dx_ba_z_sum_max: float
    t_min: float = 35.0
    t_max: float = 95.0
    min_history_rows: int = 5

    def name(self) -> str:
        return (
            f"h{self.history_sec:g}_ba{self.ba_z_mean_max:g}_"
            f"res{self.res_u_mean_max:g}_core{self.core_u_mean_min:g}_"
            f"dx{self.dx_ba_z_sum_max:g}_t{self.t_min:g}_{self.t_max:g}"
        )


DEFAULT_CANDIDATE = GateConfig(
    history_sec=10.0,
    ba_z_mean_max=-0.18,
    res_u_mean_max=-0.02,
    core_u_mean_min=0.02,
    dx_ba_z_sum_max=0.0,
)


def history_feature_frame(pos: pd.DataFrame, history_sec: float) -> pd.DataFrame:
    rows: list[dict[str, object]] = []
    for run in GATE_PROBE_RUN_DIRS:
        run_rows = pos[pos["run"] == run].sort_values("t").copy()
        times = numeric(run_rows, "t")
        for idx, row in run_rows.iterrows():
            t = to_float(row.get("t"))
            history = run_rows[(times >= t - history_sec) & (times <= t)]
            rows.append(
                {
                    "run": run,
                    "t": t,
                    "dx_ba_z": to_float(row.get("dx_ba_z")),
                    "history_rows": len(history),
                    "history_ba_z_mean": mean(numeric(history, "ba_z_before")),
                    "history_res_u_mean": mean(numeric(history, "gnss_residual_u_m")),
                    "history_core_u_mean": mean(numeric(history, "core_gnss_diff_u_m")),
                    "history_dx_ba_z_sum": float(numeric(history, "dx_ba_z").sum())
                    if len(history)
                    else math.nan,
                }
            )
    return pd.DataFrame(rows)


def build_history_features(pos: pd.DataFrame) -> dict[float, pd.DataFrame]:
    return {history: history_feature_frame(pos, history) for history in [5.0, 10.0, 15.0, 20.0, 25.0, 30.0]}


def active_rows_for_config(features: pd.DataFrame, run: str, cfg: GateConfig) -> pd.DataFrame:
    run_rows = features[features["run"] == run].copy()
    return run_rows[
        (numeric(run_rows, "t") >= cfg.t_min)
        & (numeric(run_rows, "t") <= cfg.t_max)
        & (numeric(run_rows, "history_rows") >= cfg.min_history_rows)
        & (numeric(run_rows, "history_ba_z_mean") <= cfg.ba_z_mean_max)
        & (numeric(run_rows, "history_res_u_mean") <= cfg.res_u_mean_max)
        & (numeric(run_rows, "history_core_u_mean") >= cfg.core_u_mean_min)
        & (numeric(run_rows, "history_dx_ba_z_sum") < cfg.dx_ba_z_sum_max)
    ]


def summarize_config(
    history_features: dict[float, pd.DataFrame],
    cfg: GateConfig,
) -> tuple[dict[str, object], list[dict[str, object]]]:
    detail_rows: list[dict[str, object]] = []
    first_times: list[float] = []
    last_times: list[float] = []
    negative_covered = 0
    protection_active = 0
    total_active_events = 0
    total_active_negative_dx = 0.0

    features = history_features[cfg.history_sec]
    for run in GATE_PROBE_RUN_DIRS:
        active_rows = active_rows_for_config(features, run, cfg)
        active_count = len(active_rows)
        total_active_events += active_count
        active_dx = numeric(active_rows, "dx_ba_z")
        active_neg_dx_sum = float(active_dx[active_dx < 0.0].sum()) if active_count else 0.0
        total_active_negative_dx += active_neg_dx_sum
        first_active_t = to_float(active_rows["t"].iloc[0]) if active_count else math.nan
        last_active_t = to_float(active_rows["t"].iloc[-1]) if active_count else math.nan
        if run in NEGATIVE_RUNS and active_count:
            negative_covered += 1
            first_times.append(first_active_t)
            last_times.append(last_active_t)
        if run in PROTECTION_RUNS and active_count:
            protection_active += 1
        detail_rows.append(
            {
                "config": cfg.name(),
                "run": run,
                "active_events": active_count,
                "first_active_t": first_active_t,
                "last_active_t": last_active_t,
                "active_negative_dx_sum": active_neg_dx_sum,
                "active_positive_dx_sum": float(active_dx[active_dx > 0.0].sum()) if active_count else 0.0,
                "active_dx_sum": float(active_dx.sum()) if active_count else 0.0,
            }
        )

    summary = {
        "config": cfg.name(),
        "history_sec": cfg.history_sec,
        "ba_z_mean_max": cfg.ba_z_mean_max,
        "res_u_mean_max": cfg.res_u_mean_max,
        "core_u_mean_min": cfg.core_u_mean_min,
        "dx_ba_z_sum_max": cfg.dx_ba_z_sum_max,
        "t_min": cfg.t_min,
        "t_max": cfg.t_max,
        "negative_runs_covered": negative_covered,
        "protection_runs_active": protection_active,
        "strict_pass": int(negative_covered == len(NEGATIVE_RUNS) and protection_active == 0),
        "mean_first_active_t_negative": mean(first_times),
        "max_first_active_t_negative": max(first_times) if first_times else math.nan,
        "mean_last_active_t_negative": mean(last_times),
        "total_active_events": total_active_events,
        "total_active_negative_dx_sum": total_active_negative_dx,
    }
    return summary, detail_rows


def grid_configs() -> list[GateConfig]:
    configs: list[GateConfig] = []
    for history, ba_thr, res_thr, core_thr, dx_thr in itertools.product(
        [5.0, 10.0, 15.0, 20.0, 25.0, 30.0],
        [-0.16, -0.17, -0.18, -0.19, -0.20, -0.205],
        [-0.01, -0.02, -0.03, -0.04, -0.05],
        [0.01, 0.02, 0.03, 0.04, 0.05],
        [0.0, -0.005, -0.01, -0.02],
    ):
        configs.append(
            GateConfig(
                history_sec=history,
                ba_z_mean_max=ba_thr,
                res_u_mean_max=res_thr,
                core_u_mean_min=core_thr,
                dx_ba_z_sum_max=dx_thr,
            )
        )
    return configs


def scan(pos: pd.DataFrame) -> tuple[list[dict[str, object]], list[dict[str, object]], list[dict[str, object]]]:
    summaries: list[dict[str, object]] = []
    default_detail: list[dict[str, object]] = []
    top_detail: list[dict[str, object]] = []
    history_features = build_history_features(pos)

    default_summary, default_detail = summarize_config(history_features, DEFAULT_CANDIDATE)
    summaries.append(default_summary)

    for cfg in grid_configs():
        if cfg == DEFAULT_CANDIDATE:
            continue
        summary, _ = summarize_config(history_features, cfg)
        summaries.append(summary)

    strict = [row for row in summaries if int(row["strict_pass"]) == 1]
    strict_sorted = sorted(
        strict,
        key=lambda row: (
            to_float(row["max_first_active_t_negative"], math.inf),
            to_float(row["history_sec"], math.inf),
            abs(to_float(row["ba_z_mean_max"], 0.0) + 0.18),
            abs(to_float(row["res_u_mean_max"], 0.0) + 0.02),
            abs(to_float(row["core_u_mean_min"], 0.0) - 0.02),
        ),
    )
    top_configs = []
    for row in strict_sorted[:10]:
        top_configs.append(
            GateConfig(
                history_sec=to_float(row["history_sec"]),
                ba_z_mean_max=to_float(row["ba_z_mean_max"]),
                res_u_mean_max=to_float(row["res_u_mean_max"]),
                core_u_mean_min=to_float(row["core_u_mean_min"]),
                dx_ba_z_sum_max=to_float(row["dx_ba_z_sum_max"]),
            )
        )
    for cfg in top_configs:
        _, detail = summarize_config(history_features, cfg)
        top_detail.extend(detail)
    return summaries, default_detail, top_detail


def table_top_strict(rows: list[dict[str, object]]) -> list[list[object]]:
    strict = [row for row in rows if int(row["strict_pass"]) == 1]
    strict_sorted = sorted(
        strict,
        key=lambda row: (
            to_float(row["max_first_active_t_negative"], math.inf),
            to_float(row["history_sec"], math.inf),
        ),
    )
    return [
        [
            row["history_sec"],
            row["ba_z_mean_max"],
            row["res_u_mean_max"],
            row["core_u_mean_min"],
            row["dx_ba_z_sum_max"],
            row["negative_runs_covered"],
            row["protection_runs_active"],
            fmt(row["mean_first_active_t_negative"], 2),
            fmt(row["max_first_active_t_negative"], 2),
            row["total_active_events"],
            fmt(row["total_active_negative_dx_sum"]),
        ]
        for row in strict_sorted[:12]
    ]


def table_detail(rows: list[dict[str, object]]) -> list[list[object]]:
    return [
        [
            row["run"],
            row["active_events"],
            fmt(row["first_active_t"], 2),
            fmt(row["last_active_t"], 2),
            fmt(row["active_dx_sum"]),
            fmt(row["active_negative_dx_sum"]),
            fmt(row["active_positive_dx_sum"]),
        ]
        for row in rows
        if row["config"] == DEFAULT_CANDIDATE.name()
    ]


def write_report(
    out_dir: Path,
    summaries: list[dict[str, object]],
    default_detail: list[dict[str, object]],
    top_detail: list[dict[str, object]],
) -> None:
    strict_count = sum(1 for row in summaries if int(row["strict_pass"]) == 1)
    lines = [
        "# shortgen11 early recovery causal gate scan",
        "",
        "Date: 2026-05-11",
        "",
        "Offline-only scan from existing logs. This tests trailing-window gates that could be observed online without using future 40-80s summary statistics.",
        "",
        f"Total candidates scanned: {len(summaries)}",
        f"Strict candidates: {strict_count}",
        "",
        "Strict means active on all four shortgen11 negative reruns and inactive on shortgen01/02/03/04 plus clean shortgen11 holdout.",
        "",
        "## Top Strict Candidates",
        "",
        markdown_table(
            [
                "hist",
                "ba max",
                "res max",
                "core min",
                "dx max",
                "neg covered",
                "prot active",
                "mean first",
                "max first",
                "active events",
                "active neg dx",
            ],
            table_top_strict(summaries),
        ),
        "",
        "## Default Interpretable Candidate",
        "",
        "`history_sec=10`, `35s <= armed_time <= 95s`, trailing mean `ba_z <= -0.18`, trailing mean `gnss_residual_u <= -0.02`, trailing mean `core_gnss_u >= 0.02`, trailing cumulative `dx_ba_z < 0`.",
        "",
        markdown_table(
            ["run", "events", "first t", "last t", "active dx", "active neg dx", "active pos dx"],
            table_detail(default_detail),
        ),
        "",
        "## Interpretation",
        "",
        "- The early recovery signature is not only a hindsight 40-80s window effect. A causal trailing-window gate can detect the negative reruns between roughly 35-84s while staying inactive on the current protection set.",
        "- The default interpretable 10s gate activates early enough to be a plausible recovery-state diagnostic, before the later 120-160s miss.",
        "- This still should not be treated as a validated online mechanism. It uses an armed-time window and a small protection set; the next step is to convert it into a default-off logged counterfactual or action hook and verify activation/output against one opt-in diagnostic run before any holdout expansion.",
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
    events = load_events_for(GATE_PROBE_RUN_DIRS)
    if events.empty:
        raise SystemExit("no state-update rows found")
    pos = position_events(events)
    summaries, default_detail, top_detail = scan(pos)
    write_csv(args.out_dir / "causal_gate_scan_summary.csv", summaries)
    write_csv(args.out_dir / "causal_gate_default_detail.csv", default_detail)
    write_csv(args.out_dir / "causal_gate_top_detail.csv", top_detail)
    write_report(args.out_dir, summaries, default_detail, top_detail)
    print(f"wrote: {args.out_dir / 'causal_gate_scan_summary.csv'}")
    print(f"wrote: {args.out_dir / 'causal_gate_default_detail.csv'}")
    print(f"wrote: {args.out_dir / 'causal_gate_top_detail.csv'}")
    print(f"wrote: {args.out_dir / 'report.md'}")


if __name__ == "__main__":
    main()
