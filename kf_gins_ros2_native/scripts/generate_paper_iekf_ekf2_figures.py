#!/usr/bin/env python3
"""Generate offline paper figures for IEKF/EKF2 and runtime-boundary results."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Iterable

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


GROUNDTRUTH_COLUMNS = {
    "run",
    "time_since_arm_sec",
    "mavros_armed",
    "mavros_mode",
    "gt_x_m",
    "gt_y_m",
    "ekf2_aligned_x_m",
    "ekf2_aligned_y_m",
    "iekf_normalized_aligned_x_m",
    "iekf_normalized_aligned_y_m",
    "ekf2_error_xy_m",
    "iekf_error_xy_m",
}


def parse_cli() -> argparse.Namespace:
    workspace = Path(__file__).resolve().parents[3]
    manual_root = workspace / "artifacts" / "manual"
    default_out = workspace / "artifacts" / "paper_figures" / "2026-05-06"

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--manual124",
        type=Path,
        default=manual_root / "offline_groundtruth_convergence_manual124",
        help="Directory containing manual124 groundtruth_joined.csv and groundtruth_summary.csv.",
    )
    parser.add_argument(
        "--manual125",
        type=Path,
        default=manual_root / "offline_groundtruth_convergence_manual125",
        help="Directory containing manual125 groundtruth_joined.csv and groundtruth_summary.csv.",
    )
    parser.add_argument(
        "--plan",
        type=Path,
        default=manual_root / "complex_sparse_nocollision_turn_uturn.plan",
        help="QGroundControl plan for the complex route.",
    )
    parser.add_argument(
        "--a7d",
        type=Path,
        default=manual_root
        / "manual_mainline_A7d_sparse_nocollision_headless_no_qgc_compare_core8hz_complex_mission_load_iso_guarded",
        help="A7d artifact directory. Used for source references in figure_index.md.",
    )
    parser.add_argument(
        "--a7h",
        type=Path,
        default=manual_root
        / "manual_mainline_A7h_sparse_nocollision_headless_no_qgc_compare_core8p125hz_complex_mission_load_iso_guarded",
        help="A7h artifact directory. Used for source references in figure_index.md.",
    )
    parser.add_argument(
        "--out",
        type=Path,
        default=default_out,
        help="Output directory for figures, tables, and figure_index.md.",
    )
    return parser.parse_args()


def setup_style() -> None:
    plt.rcParams.update(
        {
            "figure.dpi": 160,
            "savefig.dpi": 320,
            "font.size": 12,
            "axes.titlesize": 13,
            "axes.labelsize": 12,
            "legend.fontsize": 10,
            "axes.grid": True,
            "grid.alpha": 0.25,
            "axes.spines.top": False,
            "axes.spines.right": False,
            "pdf.fonttype": 42,
            "ps.fonttype": 42,
        }
    )


def read_groundtruth_dir(directory: Path) -> tuple[pd.DataFrame, pd.DataFrame]:
    joined_path = directory / "groundtruth_joined.csv"
    summary_path = directory / "groundtruth_summary.csv"
    joined = pd.read_csv(joined_path)
    summary = pd.read_csv(summary_path)

    missing = GROUNDTRUTH_COLUMNS - set(joined.columns)
    if missing:
        raise ValueError(f"{joined_path} missing columns: {', '.join(sorted(missing))}")
    if joined.empty:
        raise ValueError(f"{joined_path} has no rows")
    if summary.empty:
        raise ValueError(f"{summary_path} has no rows")
    return joined, summary


def flight_rows(df: pd.DataFrame) -> pd.DataFrame:
    mask = (df["time_since_arm_sec"] >= 0.0) & (df["mavros_armed"] == 1)
    filtered = df.loc[mask].sort_values("time_since_arm_sec").copy()
    if filtered.empty:
        raise ValueError("No armed rows found in groundtruth data")
    return filtered.reset_index(drop=True)


def save_figure(fig: plt.Figure, out_dir: Path, stem: str) -> list[Path]:
    paths = [out_dir / f"{stem}.png", out_dir / f"{stem}.pdf", out_dir / f"{stem}.svg"]
    for path in paths:
        fig.savefig(path, bbox_inches="tight", pad_inches=0.08)
    plt.close(fig)
    return paths


def dataframe_to_markdown(df: pd.DataFrame) -> str:
    text_df = df.copy()
    for col in text_df.columns:
        text_df[col] = text_df[col].map(lambda value: "" if pd.isna(value) else str(value))
    columns = list(text_df.columns)
    rows = [columns] + text_df.values.tolist()
    widths = [max(len(str(row[i])) for row in rows) for i in range(len(columns))]

    def fmt(row: Iterable[str]) -> str:
        return "| " + " | ".join(str(cell).ljust(widths[i]) for i, cell in enumerate(row)) + " |"

    sep = "| " + " | ".join("-" * widths[i] for i in range(len(columns))) + " |"
    lines = [fmt(columns), sep]
    for row in text_df.values.tolist():
        lines.append(fmt(row))
    return "\n".join(lines) + "\n"


def make_trajectory_figure(run_name: str, df: pd.DataFrame, out_dir: Path) -> list[Path]:
    data = flight_rows(df)
    fig, ax = plt.subplots(figsize=(11.2, 4.2), constrained_layout=False)

    ax.plot(data["gt_x_m"], data["gt_y_m"], color="#111827", lw=2.3, label="Ground truth")
    ax.plot(data["ekf2_aligned_x_m"], data["ekf2_aligned_y_m"], color="#2563eb", lw=1.7, label="PX4 EKF2")
    ax.plot(
        data["iekf_normalized_aligned_x_m"],
        data["iekf_normalized_aligned_y_m"],
        color="#dc2626",
        lw=1.6,
        label="IEKF normalized",
    )
    ax.scatter(data["gt_x_m"].iloc[0], data["gt_y_m"].iloc[0], color="#16a34a", s=55, zorder=4, label="Start")
    ax.scatter(data["gt_x_m"].iloc[-1], data["gt_y_m"].iloc[-1], color="#f59e0b", s=55, zorder=4, label="End")
    # The main mission route is nearly straight. Expanding the lateral axis makes
    # the estimator separation readable; the numeric axis still preserves units.
    y_values = pd.concat(
        [
            data["gt_y_m"],
            data["ekf2_aligned_y_m"],
            data["iekf_normalized_aligned_y_m"],
        ],
        ignore_index=True,
    )
    y_mid = 0.5 * (float(y_values.min()) + float(y_values.max()))
    y_half = max(2.0, 0.65 * (float(y_values.max()) - float(y_values.min())))
    ax.set_ylim(y_mid - y_half, y_mid + y_half)
    ax.set_xlabel("Local x / m")
    ax.set_ylabel("Local y / m")
    ax.set_title(f"{run_name} trajectory comparison")
    ax.text(
        0.99,
        0.04,
        "lateral scale expanded",
        transform=ax.transAxes,
        ha="right",
        va="bottom",
        fontsize=9,
        color="#475569",
    )
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.20), ncol=5, frameon=True)
    fig.subplots_adjust(left=0.08, right=0.98, top=0.84, bottom=0.30)
    return save_figure(fig, out_dir, f"{run_name}_trajectory_xy")


def make_error_timeseries(run_name: str, df: pd.DataFrame, out_dir: Path) -> list[Path]:
    data = flight_rows(df)
    fig, ax = plt.subplots(figsize=(10.8, 4.8), constrained_layout=True)

    ax.plot(data["time_since_arm_sec"], data["ekf2_error_xy_m"], color="#2563eb", lw=1.5, label="PX4 EKF2")
    ax.plot(
        data["time_since_arm_sec"],
        data["iekf_error_xy_m"],
        color="#dc2626",
        lw=1.5,
        label="IEKF normalized",
    )
    ax.axvspan(40, 120, color="#64748b", alpha=0.14, label="main maneuver 40-120 s")
    ax.axvspan(300, 360, color="#f97316", alpha=0.10, label="landing/RTL tail 300-360 s")
    ax.set_xlabel("Time since arm / s")
    ax.set_ylabel("Horizontal position error / m")
    ax.set_title(f"{run_name} horizontal error time series")
    ax.legend(loc="upper left", ncol=2, frameon=True)
    return save_figure(fig, out_dir, f"{run_name}_error_timeseries")


def make_landing_tail_figure(runs: dict[str, pd.DataFrame], out_dir: Path) -> list[Path]:
    fig, axes = plt.subplots(len(runs), 1, figsize=(10.8, 6.2), sharex=True, constrained_layout=True)
    if len(runs) == 1:
        axes = [axes]

    for ax, (run_name, df) in zip(axes, runs.items()):
        data = flight_rows(df)
        zoom = data[(data["time_since_arm_sec"] >= 250) & (data["time_since_arm_sec"] <= 365)]
        ax.plot(zoom["time_since_arm_sec"], zoom["ekf2_error_xy_m"], color="#2563eb", lw=1.5, label="PX4 EKF2")
        ax.plot(zoom["time_since_arm_sec"], zoom["iekf_error_xy_m"], color="#dc2626", lw=1.5, label="IEKF normalized")
        ax.axvspan(300, 360, color="#f97316", alpha=0.12, label="landing/RTL tail")
        ax.set_ylabel("Error / m")
        ax.set_title(f"{run_name} terminal-tail limitation window")
        ax.legend(loc="upper left", ncol=3, frameon=True)
    axes[-1].set_xlabel("Time since arm / s")
    return save_figure(fig, out_dir, "manual124_125_landing_tail_limitation")


def selected_metric_table(summaries: dict[str, pd.DataFrame]) -> pd.DataFrame:
    selected_windows = [
        "segment_mission_all",
        "segment_main_maneuver_40_120",
        "segment_main_maneuver_40_180",
        "segment_pre_rtl_250_300",
        "segment_landing_tail_300_360",
    ]
    rows = []
    for run_name, summary in summaries.items():
        for window in selected_windows:
            ekf2 = summary[(summary["window"] == window) & (summary["estimator"] == "ekf2")]
            iekf = summary[(summary["window"] == window) & (summary["estimator"] == "iekf_normalized")]
            if ekf2.empty or iekf.empty:
                continue
            ekf2_row = ekf2.iloc[0]
            iekf_row = iekf.iloc[0]
            rmse_reduction = 1.0 - float(iekf_row["xy_rmse_m"]) / float(ekf2_row["xy_rmse_m"])
            mean_reduction = 1.0 - float(iekf_row["xy_mean_m"]) / float(ekf2_row["xy_mean_m"])
            rows.append(
                {
                    "run": run_name,
                    "window": window.replace("segment_", ""),
                    "ekf2_rmse_m": round(float(ekf2_row["xy_rmse_m"]), 4),
                    "iekf_rmse_m": round(float(iekf_row["xy_rmse_m"]), 4),
                    "rmse_reduction_pct": round(100.0 * rmse_reduction, 1),
                    "ekf2_mean_m": round(float(ekf2_row["xy_mean_m"]), 4),
                    "iekf_mean_m": round(float(iekf_row["xy_mean_m"]), 4),
                    "mean_reduction_pct": round(100.0 * mean_reduction, 1),
                    "ekf2_p95_m": round(float(ekf2_row["xy_p95_m"]), 4),
                    "iekf_p95_m": round(float(iekf_row["xy_p95_m"]), 4),
                }
            )
    return pd.DataFrame(rows)


def make_metric_bar_figure(metric_df: pd.DataFrame, out_dir: Path) -> list[Path]:
    windows = ["mission_all", "main_maneuver_40_120", "pre_rtl_250_300", "landing_tail_300_360"]
    labels = ["Mission all", "Main 40-120 s", "Pre-RTL 250-300 s", "Tail 300-360 s"]
    runs = ["manual124", "manual125"]

    fig, axes = plt.subplots(1, 2, figsize=(13.5, 4.8), sharey=True, constrained_layout=True)
    for ax, run_name in zip(axes, runs):
        run_df = metric_df[metric_df["run"] == run_name].set_index("window")
        x = np.arange(len(windows))
        width = 0.34
        ax.bar(x - width / 2, [run_df.loc[w, "ekf2_rmse_m"] for w in windows], width, color="#2563eb", label="PX4 EKF2")
        ax.bar(
            x + width / 2,
            [run_df.loc[w, "iekf_rmse_m"] for w in windows],
            width,
            color="#dc2626",
            label="IEKF normalized",
        )
        ax.set_xticks(x)
        ax.set_xticklabels(labels, rotation=18, ha="right")
        ax.set_title(f"{run_name} RMSE by segment")
        ax.set_ylabel("Horizontal RMSE / m")
        ax.legend(frameon=True)
    return save_figure(fig, out_dir, "manual124_125_segment_rmse_bars")


def parse_plan_waypoints(plan_path: Path) -> pd.DataFrame:
    plan = json.loads(plan_path.read_text(encoding="utf-8"))
    rows = []
    for item in plan.get("mission", {}).get("items", []):
        params = item.get("params") or []
        command = item.get("command")
        frame = item.get("frame")
        if len(params) < 7:
            continue
        lat, lon, alt = params[4], params[5], params[6]
        if lat is None or lon is None:
            continue
        if command not in {16, 22} or frame != 3:
            continue
        if abs(float(lat)) < 1.0 or abs(float(lon)) < 1.0:
            continue
        rows.append(
            {
                "doJumpId": item.get("doJumpId"),
                "command": command,
                "lat": float(lat),
                "lon": float(lon),
                "alt": float(alt) if alt is not None else np.nan,
            }
        )
    if not rows:
        raise ValueError(f"No georeferenced mission items found in {plan_path}")
    df = pd.DataFrame(rows)
    lat0 = math.radians(float(df["lat"].iloc[0]))
    lon0 = math.radians(float(df["lon"].iloc[0]))
    radius = 6378137.0
    df["x_m"] = (np.radians(df["lon"]) - lon0) * math.cos(lat0) * radius
    df["y_m"] = (np.radians(df["lat"]) - lat0) * radius
    return df


def make_plan_route_figure(plan_path: Path, out_dir: Path) -> tuple[list[Path], pd.DataFrame]:
    waypoints = parse_plan_waypoints(plan_path)
    fig, ax = plt.subplots(figsize=(8.6, 7.2), constrained_layout=False)
    ax.plot(waypoints["x_m"], waypoints["y_m"], color="#f59e0b", lw=2.2, marker="o", ms=5.5)
    for _, row in waypoints.iterrows():
        label = f"{int(row['doJumpId'])}"
        if int(row["command"]) == 22:
            label += " T"
        ax.annotate(
            label,
            (row["x_m"], row["y_m"]),
            xytext=(6, 6),
            textcoords="offset points",
            fontsize=9,
            bbox={"boxstyle": "round,pad=0.18", "fc": "white", "ec": "none", "alpha": 0.8},
        )
    ax.scatter(waypoints["x_m"].iloc[0], waypoints["y_m"].iloc[0], color="#16a34a", s=70, zorder=4, label="Takeoff")
    ax.scatter(waypoints["x_m"].iloc[-1], waypoints["y_m"].iloc[-1], color="#7c3aed", s=70, zorder=4, label="Last georef item")
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("Local east / m")
    ax.set_ylabel("Local north / m")
    ax.set_title("Complex QGC route geometry")
    ax.ticklabel_format(style="plain", useOffset=False)
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.10), ncol=2, frameon=True)
    fig.subplots_adjust(left=0.10, right=0.98, top=0.90, bottom=0.18)
    return save_figure(fig, out_dir, "complex_qgc_route_plan"), waypoints


def runtime_boundary_table() -> pd.DataFrame:
    return pd.DataFrame(
        [
            {"run": "A7d", "core_cap_hz": 8.0, "verdict": "clean", "flight_phase_event": "none"},
            {
                "run": "A7h",
                "core_cap_hz": 8.125,
                "verdict": "not clean",
                "flight_phase_event": "poll timeout at 10:16; Time jump at 10:22",
            },
            {
                "run": "A7g",
                "core_cap_hz": 8.25,
                "verdict": "not clean",
                "flight_phase_event": "poll timeout and Time jump during AUTO.MISSION",
            },
            {
                "run": "A7f",
                "core_cap_hz": 8.5,
                "verdict": "not clean",
                "flight_phase_event": "poll timeout and Time jump around mission 7/8",
            },
            {
                "run": "A7e",
                "core_cap_hz": 9.0,
                "verdict": "not clean",
                "flight_phase_event": "poll timeout shortly after takeoff",
            },
            {
                "run": "A7",
                "core_cap_hz": 9.57,
                "verdict": "not clean",
                "flight_phase_event": "flight-phase Time jump at 14:23",
            },
        ]
    )


def make_runtime_boundary_figure(boundary: pd.DataFrame, out_dir: Path) -> list[Path]:
    fig, ax = plt.subplots(figsize=(8.6, 3.8), constrained_layout=True)
    clean = boundary[boundary["verdict"] == "clean"]
    bad = boundary[boundary["verdict"] != "clean"]
    ax.scatter(clean["core_cap_hz"], np.ones(len(clean)), s=90, color="#16a34a", label="clean")
    ax.scatter(bad["core_cap_hz"], np.zeros(len(bad)), s=90, color="#dc2626", label="not clean")
    for _, row in boundary.iterrows():
        y = 1.0 if row["verdict"] == "clean" else 0.0
        ax.annotate(row["run"], (row["core_cap_hz"], y), xytext=(0, 10), textcoords="offset points", ha="center")
    ax.axvline(8.0, color="#111827", lw=1.2, ls="--", label="conservative cap 8.0 Hz")
    ax.set_xlim(7.85, 9.75)
    ax.set_yticks([0, 1])
    ax.set_yticklabels(["not clean", "clean"])
    ax.set_xlabel("Core processing cap / Hz")
    ax.set_title("Complex long-route runtime boundary")
    ax.legend(loc="center right", frameon=True)
    return save_figure(fig, out_dir, "complex_qgc_core_runtime_boundary")


def make_runtime_event_timeline(out_dir: Path) -> list[Path]:
    events = pd.DataFrame(
        [
            {"run": "A7d clean", "minute": 1 + 26 / 60, "label": "takeoff", "kind": "normal", "tx": 2.2, "ty": 1.30},
            {"run": "A7d clean", "minute": 19 + 17 / 60, "label": "RTL", "kind": "normal", "tx": 18.2, "ty": 1.30},
            {"run": "A7d clean", "minute": 20 + 22 / 60, "label": "disarm", "kind": "normal", "tx": 20.0, "ty": 0.74},
            {"run": "A7h not clean", "minute": 1 + 15 / 60, "label": "takeoff", "kind": "normal", "tx": 2.2, "ty": -0.30},
            {
                "run": "A7h not clean",
                "minute": 10 + 16 / 60,
                "label": "poll timeout",
                "kind": "failure",
                "tx": 8.6,
                "ty": 0.42,
            },
            {
                "run": "A7h not clean",
                "minute": 10 + 22 / 60,
                "label": "Time jump",
                "kind": "failure",
                "tx": 10.65,
                "ty": 0.62,
            },
            {"run": "A7h not clean", "minute": 10 + 32 / 60, "label": "RTL cmd", "kind": "normal", "tx": 12.2, "ty": 0.34},
            {"run": "A7h not clean", "minute": 13 + 5 / 60, "label": "disarm", "kind": "normal", "tx": 13.4, "ty": -0.30},
        ]
    )
    y_map = {"A7d clean": 1, "A7h not clean": 0}

    fig, ax = plt.subplots(figsize=(12.4, 4.8), constrained_layout=False)
    ax.axvspan(10 + 16 / 60, 10 + 32 / 60, color="#fee2e2", alpha=0.6, zorder=0, label="A7h failure window")
    for run_name, y in y_map.items():
        run_events = events[events["run"] == run_name]
        lane_color = "#bbf7d0" if run_name == "A7d clean" else "#fecaca"
        ax.hlines(y, 0, run_events["minute"].max(), color=lane_color, lw=7, alpha=0.65, zorder=1)
        for _, row in run_events.iterrows():
            color = "#dc2626" if row["kind"] == "failure" else "#2563eb"
            ax.scatter(row["minute"], y, color=color, s=80, zorder=4)
            bbox = {
                "boxstyle": "round,pad=0.22",
                "fc": "#fff1f2" if row["kind"] == "failure" else "white",
                "ec": "#dc2626" if row["kind"] == "failure" else "#cbd5e1",
                "alpha": 0.96,
            }
            ax.annotate(
                row["label"],
                (row["minute"], y),
                xytext=(row["tx"], row["ty"]),
                textcoords="data",
                ha="center",
                va="center",
                fontsize=9,
                bbox=bbox,
                arrowprops={
                    "arrowstyle": "-",
                    "color": "#64748b" if row["kind"] == "normal" else "#dc2626",
                    "lw": 1.0,
                    "shrinkA": 2,
                    "shrinkB": 3,
                },
                zorder=5,
            )
    ax.set_yticks(list(y_map.values()))
    ax.set_yticklabels(list(y_map.keys()))
    ax.set_xlabel("ULog time / min")
    ax.set_title("A7d versus A7h runtime event timeline")
    ax.set_xlim(-0.4, 21.0)
    ax.set_ylim(-0.55, 1.55)
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.18), frameon=True)
    fig.subplots_adjust(left=0.12, right=0.98, top=0.86, bottom=0.28)
    return save_figure(fig, out_dir, "a7d_a7h_runtime_event_timeline")


def write_tables(metric_df: pd.DataFrame, boundary_df: pd.DataFrame, waypoint_df: pd.DataFrame, out_dir: Path) -> list[Path]:
    outputs = []
    tables = {
        "table_manual124_125_selected_metrics": metric_df,
        "table_complex_qgc_runtime_boundary": boundary_df,
        "table_complex_qgc_plan_waypoints": waypoint_df[
            ["doJumpId", "command", "lat", "lon", "alt", "x_m", "y_m"]
        ].round({"lat": 8, "lon": 8, "alt": 2, "x_m": 2, "y_m": 2}),
    }
    for stem, df in tables.items():
        csv_path = out_dir / f"{stem}.csv"
        md_path = out_dir / f"{stem}.md"
        df.to_csv(csv_path, index=False)
        md_path.write_text(dataframe_to_markdown(df), encoding="utf-8")
        outputs.extend([csv_path, md_path])
    return outputs


def write_figure_index(out_dir: Path, args: argparse.Namespace, generated: list[Path]) -> Path:
    pngs = [p for p in generated if p.suffix == ".png"]
    lines = [
        "# Paper Figure Index 2026-05-06",
        "",
        "Generated offline. No simulator, QGC, RViz2, PlotJuggler, or PX4 process was started.",
        "",
        "## Data Sources",
        "",
        f"- manual124: `{args.manual124}`",
        f"- manual125: `{args.manual125}`",
        f"- complex QGC plan: `{args.plan}`",
        f"- A7d clean runtime anchor: `{args.a7d}`",
        f"- A7h reject runtime anchor: `{args.a7h}`",
        "",
        "## Figures",
        "",
    ]
    descriptions = {
        "manual124_trajectory_xy.png": "manual124 trajectory against ULog ground truth; lateral scale is expanded for readability.",
        "manual125_trajectory_xy.png": "manual125 trajectory against ULog ground truth; lateral scale is expanded for readability.",
        "manual124_error_timeseries.png": "manual124 EKF2 versus projection-normalized IEKF horizontal error over time.",
        "manual125_error_timeseries.png": "manual125 EKF2 versus projection-normalized IEKF horizontal error over time.",
        "manual124_125_segment_rmse_bars.png": "Selected segment RMSE comparison; primary main-maneuver claim comes from 40-120 s.",
        "manual124_125_landing_tail_limitation.png": "Terminal tail limitation window; do not claim IEKF is better in all phases.",
        "complex_qgc_route_plan.png": "Offline QGC plan geometry for the complex route.",
        "complex_qgc_core_runtime_boundary.png": "Complex-route core cap boundary; runtime cleanliness only, not performance.",
        "a7d_a7h_runtime_event_timeline.png": "A7d clean versus A7h not-clean event timeline from summaries/ULog messages.",
    }
    for path in sorted(pngs):
        note = descriptions.get(path.name, "")
        lines.append(f"- `{path.name}`: {note}")
    lines.extend(
        [
            "",
            "## Tables",
            "",
            "- `table_manual124_125_selected_metrics.csv/.md`: selected RMSE/mean/p95 values and reductions.",
            "- `table_complex_qgc_runtime_boundary.csv/.md`: A7 runtime clean/not-clean boundary.",
            "- `table_complex_qgc_plan_waypoints.csv/.md`: parsed QGC georeferenced mission items.",
            "",
            "## Writing Boundaries",
            "",
            "- manual124/125 can support the main IEKF-vs-EKF2 performance claim under corrected comparison.",
            "- A7d/A7h support only runtime-boundary claims. They are not IEKF performance samples.",
            "- RViz2/PlotJuggler screenshots are optional appendix material; core paper figures should come from offline data.",
        ]
    )
    path = out_dir / "figure_index.md"
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path


def main() -> int:
    args = parse_cli()
    args.out.mkdir(parents=True, exist_ok=True)
    setup_style()

    m124, s124 = read_groundtruth_dir(args.manual124)
    m125, s125 = read_groundtruth_dir(args.manual125)
    runs = {"manual124": m124, "manual125": m125}
    summaries = {"manual124": s124, "manual125": s125}

    generated: list[Path] = []
    for run_name, df in runs.items():
        generated.extend(make_trajectory_figure(run_name, df, args.out))
        generated.extend(make_error_timeseries(run_name, df, args.out))

    generated.extend(make_landing_tail_figure(runs, args.out))
    metric_df = selected_metric_table(summaries)
    generated.extend(make_metric_bar_figure(metric_df, args.out))

    route_paths, waypoint_df = make_plan_route_figure(args.plan, args.out)
    generated.extend(route_paths)

    boundary_df = runtime_boundary_table()
    generated.extend(make_runtime_boundary_figure(boundary_df, args.out))
    generated.extend(make_runtime_event_timeline(args.out))
    generated.extend(write_tables(metric_df, boundary_df, waypoint_df, args.out))
    generated.append(write_figure_index(args.out, args, generated))

    for path in generated:
        print(f"generated: {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
