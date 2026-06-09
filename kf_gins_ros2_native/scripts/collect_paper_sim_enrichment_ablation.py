#!/usr/bin/env python3
"""Collect scored paper-simulation ablation runs into a table and figure.

This is intentionally artifact-local. It reads only runs under
paper_sim_enrichment_2026-05-16 and ignores invalidated attempts.
"""

from __future__ import annotations

import argparse
import csv
import re
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pandas as pd


WS_ROOT = Path("/home/yang/kf_gins_ws")
DEFAULT_OUT = WS_ROOT / "artifacts/manual/paper_sim_enrichment_2026-05-16"
ROUTES = ["shortgen09", "shortgen16", "shortgen17"]
VARIANTS = ["paper_cand19_full", "paper_cand19_posonly", "paper_cand19_single_iter"]
VARIANT_LABEL = {
    "paper_cand19_full": "Full",
    "paper_cand19_posonly": "Position-only",
    "paper_cand19_single_iter": "Single iteration",
}
WINDOW = "segment_main_maneuver_40_180"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUT)
    return parser.parse_args()


def read_meta(run_dir: Path) -> dict[str, str]:
    meta: dict[str, str] = {}
    path = run_dir / "compare_meta.txt"
    if not path.exists():
        return meta
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            meta[key.strip()] = value.strip()
    return meta


def mission_success(run_dir: Path) -> tuple[int | None, int | None]:
    path = run_dir / "mission_smoke.log"
    if not path.exists():
        return None, None
    text = path.read_text(encoding="utf-8", errors="replace")
    match = re.search(r"final last_reached=(\d+) success_seq=(\d+)", text)
    if not match:
        return None, None
    return int(match.group(1)), int(match.group(2))


def latest_valid_run(out_dir: Path, route: str, variant: str) -> Path:
    pattern = f"paper_{route}_{variant}_*"
    candidates = sorted((out_dir / "runs").glob(pattern), reverse=True)
    for run_dir in candidates:
        if (run_dir / "INVALID_PARAM_MISMATCH.txt").exists():
            continue
        if (run_dir / "offline_groundtruth_global_raw/groundtruth_summary.csv").exists():
            return run_dir
    raise FileNotFoundError(f"no valid scored run found for {route} {variant}")


def summary_rows(run_dir: Path) -> dict[str, dict[str, str]]:
    path = run_dir / "offline_groundtruth_global_raw/groundtruth_summary.csv"
    rows: dict[str, dict[str, str]] = {}
    with path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            if row.get("window") == WINDOW:
                rows[row["estimator"]] = row
    if "ekf2" not in rows or "iekf_normalized" not in rows:
        raise ValueError(f"missing {WINDOW} ekf2/iekf rows in {path}")
    return rows


def f(row: dict[str, str], key: str) -> float:
    return float(row[key])


def build_table(out_dir: Path) -> pd.DataFrame:
    records: list[dict[str, object]] = []
    for route in ROUTES:
        for variant in VARIANTS:
            run_dir = latest_valid_run(out_dir, route, variant)
            rows = summary_rows(run_dir)
            ekf2 = rows["ekf2"]
            iekf = rows["iekf_normalized"]
            last_reached, success_seq = mission_success(run_dir)
            ekf2_rmse = f(ekf2, "xy_rmse_m")
            iekf_rmse = f(iekf, "xy_rmse_m")
            records.append(
                {
                    "route": route,
                    "variant": variant,
                    "variant_label": VARIANT_LABEL[variant],
                    "window": WINDOW,
                    "scoring_frame": "global_groundtruth/raw_wgs84_enu",
                    "run_dir": str(run_dir),
                    "mission_last_reached": last_reached,
                    "mission_success_seq": success_seq,
                    "rows_ekf2": int(float(ekf2["rows"])),
                    "rows_iekf": int(float(iekf["rows"])),
                    "ekf2_xy_rmse_m": ekf2_rmse,
                    "iekf_xy_rmse_m": iekf_rmse,
                    "rmse_delta_m": iekf_rmse - ekf2_rmse,
                    "rmse_improvement_pct": (ekf2_rmse - iekf_rmse) / ekf2_rmse * 100.0,
                    "ekf2_xy_mean_m": f(ekf2, "xy_mean_m"),
                    "iekf_xy_mean_m": f(iekf, "xy_mean_m"),
                    "ekf2_xy_p95_m": f(ekf2, "xy_p95_m"),
                    "iekf_xy_p95_m": f(iekf, "xy_p95_m"),
                    "ekf2_xy_max_m": f(ekf2, "xy_max_m"),
                    "iekf_xy_max_m": f(iekf, "xy_max_m"),
                }
            )
    return pd.DataFrame.from_records(records)


def plot_ablation(df: pd.DataFrame, out_path: Path) -> None:
    plt.rcParams.update(
        {
            "figure.dpi": 150,
            "savefig.dpi": 300,
            "font.size": 9,
            "axes.spines.top": False,
            "axes.spines.right": False,
            "axes.grid": True,
            "grid.alpha": 0.25,
        }
    )
    fig, ax = plt.subplots(figsize=(7.2, 3.8))
    routes = ROUTES
    variants = VARIANTS
    width = 0.22
    x = range(len(routes))
    for offset, variant in enumerate(variants):
        subset = df[df["variant"] == variant].set_index("route").loc[routes]
        xs = [value + (offset - 1) * width for value in x]
        ax.bar(xs, subset["iekf_xy_rmse_m"], width=width, label=VARIANT_LABEL[variant])

    ekf2 = (
        df[df["variant"] == "paper_cand19_full"]
        .set_index("route")
        .loc[routes]["ekf2_xy_rmse_m"]
        .to_list()
    )
    ax.scatter(list(x), ekf2, marker="x", color="black", label="EKF2 baseline", zorder=4)
    ax.set_xticks(list(x))
    ax.set_xticklabels(routes)
    ax.set_ylabel("Horizontal RMSE (m)")
    ax.set_title("Ablation on global-groundtruth/raw-wgs84-enu scoring")
    ax.legend(ncol=2, frameon=False)
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


def write_report(df: pd.DataFrame, out_path: Path) -> None:
    full = df[df["variant"] == "paper_cand19_full"]
    lines = [
        "# Paper Simulation Enrichment Ablation Runs",
        "",
        f"- Window: `{WINDOW}`.",
        "- Scoring frame: `global_groundtruth/raw_wgs84_enu`.",
        "- `rmse_delta_m = IEKF_RMSE - EKF2_RMSE`; negative means IEKF is better.",
        f"- Full cand19 wins {int((full['rmse_delta_m'] < 0).sum())}/{len(full)} ablation routes.",
        "",
        "## Main RMSE Table",
        "",
        "| route | variant | EKF2 RMSE (m) | IEKF RMSE (m) | delta (m) | improvement |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for row in df.itertuples(index=False):
        lines.append(
            f"| {row.route} | {row.variant_label} | {row.ekf2_xy_rmse_m:.3f} | "
            f"{row.iekf_xy_rmse_m:.3f} | {row.rmse_delta_m:.3f} | "
            f"{row.rmse_improvement_pct:.1f}% |"
        )
    lines.extend(
        [
            "",
            "## Interpretation Notes",
            "",
            "- GNSS velocity aiding is helpful on the maneuvering routes but not monotonic on the low-excitation straight route.",
            "- Single-iteration runs remain competitive in this matrix, so the paper should present them as a computational-cost ablation rather than claiming that more IEKF iterations always reduce RMSE.",
        ]
    )
    out_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    args = parse_args()
    out_dir: Path = args.out_dir
    tables = out_dir / "tables"
    figures = out_dir / "figures"
    tables.mkdir(parents=True, exist_ok=True)
    figures.mkdir(parents=True, exist_ok=True)
    df = build_table(out_dir)
    df.to_csv(tables / "ablation_summary.csv", index=False)
    plot_ablation(df, figures / "ablation_rmse_bar.png")
    write_report(df, out_dir / "ablation_runs_report.md")
    print(tables / "ablation_summary.csv")
    print(figures / "ablation_rmse_bar.png")
    print(out_dir / "ablation_runs_report.md")


if __name__ == "__main__":
    main()
