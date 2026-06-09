#!/usr/bin/env python3
"""Diagnose the early GNSS-position vertical update path.

Offline-only diagnostic. It uses existing state-update and GNSS-update debug
CSVs to inspect whether 40-80s vertical residual/core-GNSS-U sign explains
the vertical accelerometer-bias update direction, and whether a narrow
recovery-state action space is worth testing before any online hook.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import Iterable

import pandas as pd

from offline_shortgen11_early_basin_recovery_diagnostic import (
    GATE_PROBE_RUN_DIRS,
    RUN_DIRS,
    finite,
    fmt,
    load_events_for,
    markdown_table,
    mean,
    numeric,
    to_float,
    write_csv,
)


BASE = Path("/home/yang/kf_gins_ws/artifacts/manual/short_route_generalization_2026-05-07")
DEFAULT_OUT = BASE / "shortgen11_gnss_position_vertical_path_2026-05-11"

WINDOWS = [
    ("0-40", 0.0, 40.0),
    ("40-80", 40.0, 80.0),
    ("80-120", 80.0, 120.0),
]


def subset(frame: pd.DataFrame, start: float, end: float) -> pd.DataFrame:
    t = numeric(frame, "t")
    return frame[(t >= start) & (t < end)]


def sign_nonzero(series: pd.Series, eps: float = 1e-9) -> pd.Series:
    values = pd.to_numeric(series, errors="coerce")
    out = pd.Series([0] * len(values), index=values.index, dtype="int64")
    out[values > eps] = 1
    out[values < -eps] = -1
    return out


def frac(mask: pd.Series) -> float:
    return float(mask.mean()) if len(mask) else math.nan


def zero_slope(x: pd.Series, y: pd.Series) -> float:
    xx = pd.to_numeric(x, errors="coerce")
    yy = pd.to_numeric(y, errors="coerce")
    valid = xx.notna() & yy.notna()
    xx = xx[valid]
    yy = yy[valid]
    denom = float((xx * xx).sum())
    if denom <= 0.0:
        return math.nan
    return float((xx * yy).sum() / denom)


def ordinary_slope(x: pd.Series, y: pd.Series) -> float:
    xx = pd.to_numeric(x, errors="coerce")
    yy = pd.to_numeric(y, errors="coerce")
    valid = xx.notna() & yy.notna()
    xx = xx[valid]
    yy = yy[valid]
    if len(xx) < 2:
        return math.nan
    var = float(((xx - xx.mean()) ** 2).sum())
    if var <= 0.0:
        return math.nan
    return float(((xx - xx.mean()) * (yy - yy.mean())).sum() / var)


def corr(x: pd.Series, y: pd.Series) -> float:
    xx = pd.to_numeric(x, errors="coerce")
    yy = pd.to_numeric(y, errors="coerce")
    valid = xx.notna() & yy.notna()
    if int(valid.sum()) < 3:
        return math.nan
    return float(xx[valid].corr(yy[valid]))


def split_sum(values: pd.Series, selector: pd.Series) -> float:
    vals = pd.to_numeric(values, errors="coerce")
    return float(vals[selector].sum()) if len(vals) else math.nan


def recovery_gate_active(sub: pd.DataFrame) -> bool:
    ba_mean = mean(numeric(sub, "ba_z_before"))
    dx_sum = float(numeric(sub, "dx_ba_z").sum()) if len(sub) else math.nan
    res_u_mean = mean(numeric(sub, "gnss_residual_u_m"))
    core_u_mean = mean(numeric(sub, "core_gnss_diff_u_m"))
    return (
        finite(ba_mean)
        and ba_mean <= -0.18
        and finite(res_u_mean)
        and res_u_mean <= -0.02
        and finite(core_u_mean)
        and core_u_mean >= 0.02
        and finite(dx_sum)
        and dx_sum < 0.0
    )


def position_events(events: pd.DataFrame) -> pd.DataFrame:
    out = events[events["event_type"] == "gnss_position"].copy()
    out["res_u_sign"] = sign_nonzero(numeric(out, "gnss_residual_u_m"))
    out["core_u_sign"] = sign_nonzero(numeric(out, "core_gnss_diff_u_m"))
    out["dx_ba_z_sign"] = sign_nonzero(numeric(out, "dx_ba_z"))
    out["dx_pos_u_sign"] = sign_nonzero(numeric(out, "dx_pos_u"))
    return out


def path_summary_rows(events: pd.DataFrame) -> list[dict[str, object]]:
    pos = position_events(events)
    rows: list[dict[str, object]] = []
    for run in GATE_PROBE_RUN_DIRS:
        run_rows = pos[pos["run"] == run]
        for window, start, end in WINDOWS:
            sub = subset(run_rows, start, end)
            res_u = numeric(sub, "gnss_residual_u_m")
            core_u = numeric(sub, "core_gnss_diff_u_m")
            dx_ba = numeric(sub, "dx_ba_z")
            dx_pos_u = numeric(sub, "dx_pos_u")
            res_sign = sign_nonzero(res_u)
            core_sign = sign_nonzero(core_u)
            dx_sign = sign_nonzero(dx_ba)
            dx_pos_sign = sign_nonzero(dx_pos_u)
            valid_res_dx = (res_sign != 0) & (dx_sign != 0)
            valid_res_core = (res_sign != 0) & (core_sign != 0)
            valid_res_pos = (res_sign != 0) & (dx_pos_sign != 0)
            rows.append(
                {
                    "run": run,
                    "window": window,
                    "rows": len(sub),
                    "ba_z_first": to_float(numeric(sub, "ba_z_before").iloc[0]) if len(sub) else math.nan,
                    "ba_z_last": to_float(numeric(sub, "ba_z_after").iloc[-1]) if len(sub) else math.nan,
                    "dx_ba_z_sum": float(dx_ba.sum()) if len(sub) else math.nan,
                    "dx_ba_z_mean": mean(dx_ba),
                    "res_u_mean_m": mean(res_u),
                    "core_gnss_u_mean_m": mean(core_u),
                    "res_u_negative_frac": frac(res_u < 0.0),
                    "res_core_opposite_frac": frac((res_sign * core_sign < 0)[valid_res_core])
                    if int(valid_res_core.sum())
                    else math.nan,
                    "dx_res_same_sign_frac": frac((dx_sign * res_sign > 0)[valid_res_dx])
                    if int(valid_res_dx.sum())
                    else math.nan,
                    "dx_core_opposite_sign_frac": frac((dx_sign * core_sign < 0)[valid_res_dx & (core_sign != 0)])
                    if int((valid_res_dx & (core_sign != 0)).sum())
                    else math.nan,
                    "dx_pos_res_same_sign_frac": frac((dx_pos_sign * res_sign > 0)[valid_res_pos])
                    if int(valid_res_pos.sum())
                    else math.nan,
                    "dx_per_res_zero_slope": zero_slope(res_u, dx_ba),
                    "dx_per_res_ordinary_slope": ordinary_slope(res_u, dx_ba),
                    "dx_res_corr": corr(res_u, dx_ba),
                    "dx_when_res_negative": split_sum(dx_ba, res_u < 0.0),
                    "dx_when_res_positive": split_sum(dx_ba, res_u > 0.0),
                    "gate_active": int(window == "40-80" and recovery_gate_active(sub)),
                }
            )
    return rows


def event_sign_bin_rows(events: pd.DataFrame) -> list[dict[str, object]]:
    pos = position_events(events)
    rows: list[dict[str, object]] = []
    for run in GATE_PROBE_RUN_DIRS:
        sub = subset(pos[pos["run"] == run], 40.0, 80.0)
        res_u = numeric(sub, "gnss_residual_u_m")
        for label, selector in [
            ("res_u_negative", res_u < 0.0),
            ("res_u_positive", res_u > 0.0),
        ]:
            part = sub[selector].copy()
            dx = numeric(part, "dx_ba_z")
            rows.append(
                {
                    "run": run,
                    "bin": label,
                    "rows": len(part),
                    "res_u_mean_m": mean(numeric(part, "gnss_residual_u_m")),
                    "core_gnss_u_mean_m": mean(numeric(part, "core_gnss_diff_u_m")),
                    "dx_ba_z_sum": float(dx.sum()) if len(part) else math.nan,
                    "dx_ba_z_mean": mean(dx),
                    "dx_ba_z_negative_frac": frac(dx < 0.0),
                }
            )
    return rows


def proxy_action_rows(events: pd.DataFrame) -> list[dict[str, object]]:
    pos = position_events(events)
    rows: list[dict[str, object]] = []
    for run in GATE_PROBE_RUN_DIRS:
        sub = subset(pos[pos["run"] == run], 40.0, 80.0)
        dx = numeric(sub, "dx_ba_z")
        neg_dx = dx[dx < 0.0]
        actual_sum = float(dx.sum()) if len(sub) else math.nan
        neg_sum = float(neg_dx.sum()) if len(sub) else math.nan
        pos_sum = float(dx[dx > 0.0].sum()) if len(sub) else math.nan
        ba_first = to_float(numeric(sub, "ba_z_before").iloc[0]) if len(sub) else math.nan
        ba_last = to_float(numeric(sub, "ba_z_after").iloc[-1]) if len(sub) else math.nan
        gate = recovery_gate_active(sub)
        clip_shift = -neg_sum if gate and finite(neg_sum) else 0.0
        half_shift = -0.5 * neg_sum if gate and finite(neg_sum) else 0.0
        scale25_shift = -0.75 * neg_sum if gate and finite(neg_sum) else 0.0
        rows.append(
            {
                "run": run,
                "rows": len(sub),
                "gate_active": int(gate),
                "ba_z_first": ba_first,
                "ba_z_last_actual": ba_last,
                "dx_ba_z_sum_actual": actual_sum,
                "dx_ba_z_negative_sum": neg_sum,
                "dx_ba_z_positive_sum": pos_sum,
                "ba_z_last_if_negative_dx_halved": ba_last + half_shift
                if finite(ba_last)
                else math.nan,
                "ba_z_last_if_negative_dx_scaled_25pct": ba_last + scale25_shift
                if finite(ba_last)
                else math.nan,
                "ba_z_last_if_negative_dx_clipped": ba_last + clip_shift
                if finite(ba_last)
                else math.nan,
                "clip_margin_to_m0_18": ba_last + clip_shift + 0.18
                if finite(ba_last)
                else math.nan,
            }
        )
    return rows


def compact_40_80_rows(rows: list[dict[str, object]]) -> list[dict[str, object]]:
    return [row for row in rows if row["window"] == "40-80"]


def table_path_40_80(rows: list[dict[str, object]]) -> list[list[object]]:
    return [
        [
            row["run"],
            row["rows"],
            fmt(row["ba_z_first"]),
            fmt(row["ba_z_last"]),
            fmt(row["res_u_mean_m"]),
            fmt(row["core_gnss_u_mean_m"]),
            fmt(row["dx_ba_z_sum"]),
            fmt(row["res_u_negative_frac"]),
            fmt(row["res_core_opposite_frac"]),
            fmt(row["dx_res_same_sign_frac"]),
            fmt(row["dx_per_res_zero_slope"], 6),
            fmt(row["dx_res_corr"]),
            row["gate_active"],
        ]
        for row in compact_40_80_rows(rows)
    ]


def table_bins(rows: list[dict[str, object]]) -> list[list[object]]:
    return [
        [
            row["run"],
            row["bin"],
            row["rows"],
            fmt(row["res_u_mean_m"]),
            fmt(row["core_gnss_u_mean_m"]),
            fmt(row["dx_ba_z_sum"]),
            fmt(row["dx_ba_z_mean"], 6),
            fmt(row["dx_ba_z_negative_frac"]),
        ]
        for row in rows
    ]


def table_proxy(rows: list[dict[str, object]]) -> list[list[object]]:
    return [
        [
            row["run"],
            row["gate_active"],
            fmt(row["ba_z_first"]),
            fmt(row["ba_z_last_actual"]),
            fmt(row["dx_ba_z_sum_actual"]),
            fmt(row["dx_ba_z_negative_sum"]),
            fmt(row["ba_z_last_if_negative_dx_halved"]),
            fmt(row["ba_z_last_if_negative_dx_scaled_25pct"]),
            fmt(row["ba_z_last_if_negative_dx_clipped"]),
            fmt(row["clip_margin_to_m0_18"]),
        ]
        for row in rows
    ]


def write_report(
    out_dir: Path,
    path_rows: list[dict[str, object]],
    bin_rows: list[dict[str, object]],
    proxy_rows: list[dict[str, object]],
) -> None:
    lines = [
        "# shortgen11 GNSS-position vertical path diagnostic",
        "",
        "Date: 2026-05-11",
        "",
        "Offline-only diagnostic from existing state-update and GNSS-update logs. No simulator or estimator run was started.",
        "",
        "## 40-80s Path Summary",
        "",
        markdown_table(
            [
                "run",
                "rows",
                "ba first",
                "ba last",
                "res U",
                "core-GNSS U",
                "sum dx_ba_z",
                "res U neg frac",
                "res/core opp frac",
                "dx/res same frac",
                "dx per res slope",
                "dx-res corr",
                "gate",
            ],
            table_path_40_80(path_rows),
        ),
        "",
        "## 40-80s Residual-Sign Split",
        "",
        markdown_table(
            [
                "run",
                "bin",
                "rows",
                "res U",
                "core-GNSS U",
                "sum dx_ba_z",
                "mean dx_ba_z",
                "dx neg frac",
            ],
            table_bins(bin_rows),
        ),
        "",
        "## Narrow Action-Space Proxy",
        "",
        "This is not a closed-loop counterfactual. It only estimates how much room exists if an online mechanism, active only under the early recovery gate, reduced negative `gnss_position` contributions to `accbias_z` in 40-80s.",
        "",
        markdown_table(
            [
                "run",
                "gate",
                "ba first",
                "ba actual",
                "actual dx",
                "neg dx",
                "half neg",
                "25pct neg",
                "clip neg",
                "clip margin",
            ],
            table_proxy(proxy_rows),
        ),
        "",
        "## Interpretation",
        "",
        "- In 40-80s, `gnss_residual_u` and `core_gnss_diff_u` are almost exactly opposite signs, so either one can describe the same vertical disagreement direction.",
        "- The sign of `dx_ba_z` follows `gnss_residual_u` for most updates. The shortgen11 negative reruns are not receiving random bias kicks; their GNSS-position vertical residual direction persistently drives `accbias_z` deeper or keeps it deep.",
        "- A mechanism that only reduces negative `dx_ba_z` under the coarse early recovery gate would be inactive on shortgen01/02/03/04 and clean shortgen11 holdout in these logs.",
        "- The proxy also shows a limitation: clipping negative bias increments can move the 40-80s endpoint toward recovery, but it does not prove mission-level performance because the real filter would change subsequent residuals and covariances.",
        "",
        "## Decision",
        "",
        "The next candidate should be a default-off early recovery diagnostic/action experiment around GNSS-position vertical bias feedback, not an output projection. Before online flight, first inspect or implement a logged counterfactual that can distinguish deweighting harmful vertical bias feedback from merely masking output error.",
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
    path_rows = path_summary_rows(events)
    bin_rows = event_sign_bin_rows(events)
    proxy_rows = proxy_action_rows(events)
    write_csv(args.out_dir / "vertical_path_window_summary.csv", path_rows)
    write_csv(args.out_dir / "vertical_path_residual_sign_split_40_80.csv", bin_rows)
    write_csv(args.out_dir / "vertical_bias_feedback_action_proxy_40_80.csv", proxy_rows)
    write_report(args.out_dir, path_rows, bin_rows, proxy_rows)
    print(f"wrote: {args.out_dir / 'vertical_path_window_summary.csv'}")
    print(f"wrote: {args.out_dir / 'vertical_path_residual_sign_split_40_80.csv'}")
    print(f"wrote: {args.out_dir / 'vertical_bias_feedback_action_proxy_40_80.csv'}")
    print(f"wrote: {args.out_dir / 'report.md'}")


if __name__ == "__main__":
    main()
