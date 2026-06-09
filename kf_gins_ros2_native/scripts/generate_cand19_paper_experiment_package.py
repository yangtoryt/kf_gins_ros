#!/usr/bin/env python3
"""Generate paper-ready cand19 experiment tables, figures, and chapter drafts.

This script is intentionally offline-only. It reads existing groundtruth and
debug CSV artifacts, then writes a reproducible paper evidence package. It does
not start PX4/Gazebo/MAVROS and does not modify estimator code or parameters.
"""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


CANDIDATE = "cand19_pos08_arm05_vel04_vrw28"
CHI2_2D_95 = 5.991464547


@dataclass(frozen=True)
class EvidenceRun:
    stage: str
    route: str
    label: str
    eval_dir: Path
    raw_dir: Path | None
    paper_route: str = ""
    role: str = ""


def workspace_root() -> Path:
    return Path(__file__).resolve().parents[3]


def parse_cli() -> argparse.Namespace:
    ws = workspace_root()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=ws / "artifacts" / "manual" / "paper_experiment_package_20260514",
        help="Output directory for generated CSV, PNG, README, and chapter drafts.",
    )
    parser.add_argument(
        "--expanded-root",
        type=Path,
        default=None,
        help="Optional expanded holdout run root containing runs/*/offline_groundtruth_global_raw.",
    )
    return parser.parse_args()


def setup_style() -> None:
    plt.rcParams.update(
        {
            "figure.dpi": 150,
            "savefig.dpi": 320,
            "font.size": 9.5,
            "axes.titlesize": 10.5,
            "axes.labelsize": 9.5,
            "legend.fontsize": 8.5,
            "axes.grid": True,
            "grid.alpha": 0.22,
            "axes.spines.top": False,
            "axes.spines.right": False,
            "pdf.fonttype": 42,
            "ps.fonttype": 42,
        }
    )


def latest_matching(root: Path, pattern: str) -> Path | None:
    matches = sorted(root.glob(pattern))
    return matches[-1] if matches else None


def build_runs(ws: Path) -> list[EvidenceRun]:
    base = ws / "artifacts" / "manual" / "autonomous_iekf_perf_2026-05-12_night"
    audit_runs = base / "reports" / "global_groundtruth_audit_20260513" / "runs"
    t2_runs = base / "reports" / "global_groundtruth_t2_cand19" / "runs"
    raw_root = base / "runs"
    repeat_root = ws / "artifacts" / "manual" / "cand19_repeatability_20260514" / "runs"

    def official(stage: str, route: str, paper_route: str = "", role: str = "") -> EvidenceRun:
        if stage == "T2":
            eval_dir = t2_runs / route
        else:
            eval_dir = audit_runs / f"{route}__{CANDIDATE}"
        raw_dir = latest_matching(raw_root, f"autoniekf_{route}_{CANDIDATE}_*")
        return EvidenceRun(stage, route, route, eval_dir, raw_dir, paper_route, role)

    runs = [
        official("T0", "shortgen11", role="selection diagnostic"),
        official("T1", "shortgen01", role="selection/generalization"),
        official("T1", "shortgen02", role="selection/generalization"),
        official("T1", "shortgen03", role="selection/generalization"),
        official("T1", "shortgen04", role="selection/generalization"),
        official("T2", "shortgen08", "Route 01", "clean holdout"),
        official("T2", "shortgen09", "Route 02", "clean holdout"),
        official("T2", "shortgen10", "Route 03", "clean holdout"),
        official("T2", "shortgen13", "Route 04", "clean holdout"),
        official("T2", "shortgen16", "Route 05", "clean holdout"),
    ]

    repeat_map = {
        "shortgen09": "autoniekf_shortgen09_cand19_pos08_arm05_vel04_vrw28_20260514_184823",
        "shortgen13": "autoniekf_shortgen13_cand19_pos08_arm05_vel04_vrw28_20260514_185406",
        "shortgen16": "autoniekf_shortgen16_cand19_pos08_arm05_vel04_vrw28_20260514_190000",
    }
    for route, dirname in repeat_map.items():
        raw = repeat_root / dirname
        runs.append(
            EvidenceRun(
                "repeatability",
                route,
                f"{route}_repeat",
                raw / "offline_groundtruth_global_raw",
                raw,
                role="representative rerun",
            )
        )
    return runs


EXPANDED_ROUTE_INFO = {
    "shortgen17": ("Expanded 01", "straight line"),
    "shortgen18": ("Expanded 02", "rectangle"),
    "shortgen19": ("Expanded 03", "L-turn"),
    "shortgen20": ("Expanded 04", "S/polyline"),
    "shortgen21": ("Expanded 05", "out-and-back"),
}


def build_expanded_runs(expanded_root: Path | None) -> list[EvidenceRun]:
    if expanded_root is None:
        return []
    root = expanded_root.resolve()
    runs_dir = root / "runs"
    if not runs_dir.exists():
        return []
    expanded: list[EvidenceRun] = []
    for route, (paper_route, role) in EXPANDED_ROUTE_INFO.items():
        matches = sorted(runs_dir.glob(f"autoniekf_{route}_{CANDIDATE}_*"))
        usable = [m for m in matches if (m / "offline_groundtruth_global_raw" / "groundtruth_joined.csv").exists()]
        if not usable:
            continue
        raw = usable[-1]
        expanded.append(
            EvidenceRun(
                "expanded",
                route,
                route,
                raw / "offline_groundtruth_global_raw",
                raw,
                paper_route,
                role,
            )
        )
    return expanded


def require_file(path: Path) -> None:
    if not path.exists():
        raise FileNotFoundError(path)


def read_joined(run: EvidenceRun) -> pd.DataFrame:
    path = run.eval_dir / "groundtruth_joined.csv"
    require_file(path)
    df = pd.read_csv(path)
    for col in [
        "time_since_arm_sec",
        "mavros_armed",
        "turning_now",
        "horizontal_speed_mps",
        "gyro_deg_s",
        "source_yaw_rate_deg_s",
        "gt_x_m",
        "gt_y_m",
        "ekf2_aligned_x_m",
        "ekf2_aligned_y_m",
        "iekf_normalized_aligned_x_m",
        "iekf_normalized_aligned_y_m",
        "ekf2_error_xy_m",
        "iekf_error_xy_m",
    ]:
        if col in df.columns:
            df[col] = pd.to_numeric(df[col], errors="coerce")
    return df


def finite_mask(series: pd.Series) -> pd.Series:
    return np.isfinite(pd.to_numeric(series, errors="coerce"))


def segment_masks(df: pd.DataFrame) -> dict[str, pd.Series]:
    armed = (df["mavros_armed"] == 1) & (df["time_since_arm_sec"] >= 0.0)
    mode = df.get("mavros_mode", pd.Series("", index=df.index)).astype(str)
    t = df["time_since_arm_sec"]
    main = armed & (t >= 40.0) & (t <= 180.0)
    mission = armed & mode.str.contains("AUTO.MISSION", na=False)
    rtl = armed & mode.str.contains("AUTO.RTL", na=False)
    gyro = pd.to_numeric(df.get("gyro_deg_s", 0.0), errors="coerce").abs()
    yaw_rate = pd.to_numeric(df.get("source_yaw_rate_deg_s", 0.0), errors="coerce").abs()
    turning_now = pd.to_numeric(df.get("turning_now", 0.0), errors="coerce").fillna(0.0) > 0.5
    turn = main & (turning_now | (gyro >= 8.0) | (yaw_rate >= 8.0))
    speed = pd.to_numeric(df.get("horizontal_speed_mps", 0.0), errors="coerce")
    straight = main & (~turn) & (speed >= 0.5)
    return {
        "armed_all": armed,
        "mission_all": mission,
        "main_40_180": main,
        "rtl_all": rtl,
        "straight_main_40_180": straight,
        "turn_main_40_180": turn,
    }


def metric_values(errors: pd.Series) -> dict[str, float]:
    err = pd.to_numeric(errors, errors="coerce")
    err = err[np.isfinite(err)]
    if err.empty:
        return {"rows": 0, "rmse_m": math.nan, "mae_m": math.nan, "max_m": math.nan}
    return {
        "rows": int(err.shape[0]),
        "rmse_m": float(np.sqrt(np.mean(np.square(err)))),
        "mae_m": float(np.mean(np.abs(err))),
        "max_m": float(np.max(np.abs(err))),
    }


def pct_improvement(baseline: float, candidate: float) -> float:
    if not math.isfinite(baseline) or baseline == 0.0 or not math.isfinite(candidate):
        return math.nan
    return 100.0 * (baseline - candidate) / baseline


def compute_segment_metrics(runs: list[EvidenceRun]) -> tuple[pd.DataFrame, dict[str, pd.DataFrame]]:
    rows: list[dict[str, object]] = []
    joined_by_label: dict[str, pd.DataFrame] = {}
    for run in runs:
        df = read_joined(run)
        joined_by_label[run.label] = df
        masks = segment_masks(df)
        for segment, mask in masks.items():
            ekf2 = metric_values(df.loc[mask & finite_mask(df["ekf2_error_xy_m"]), "ekf2_error_xy_m"])
            iekf = metric_values(df.loc[mask & finite_mask(df["iekf_error_xy_m"]), "iekf_error_xy_m"])
            rows.append(
                {
                    "stage": run.stage,
                    "paper_route": run.paper_route,
                    "route": run.route,
                    "label": run.label,
                    "role": run.role,
                    "segment": segment,
                    "rows": min(ekf2["rows"], iekf["rows"]),
                    "ekf2_rmse_m": ekf2["rmse_m"],
                    "iekf_rmse_m": iekf["rmse_m"],
                    "rmse_delta_m": iekf["rmse_m"] - ekf2["rmse_m"],
                    "rmse_improvement_pct": pct_improvement(ekf2["rmse_m"], iekf["rmse_m"]),
                    "ekf2_mae_m": ekf2["mae_m"],
                    "iekf_mae_m": iekf["mae_m"],
                    "mae_delta_m": iekf["mae_m"] - ekf2["mae_m"],
                    "mae_improvement_pct": pct_improvement(ekf2["mae_m"], iekf["mae_m"]),
                    "ekf2_max_m": ekf2["max_m"],
                    "iekf_max_m": iekf["max_m"],
                    "max_delta_m": iekf["max_m"] - ekf2["max_m"],
                    "max_improvement_pct": pct_improvement(ekf2["max_m"], iekf["max_m"]),
                }
            )
    return pd.DataFrame(rows), joined_by_label


def read_nis(run: EvidenceRun) -> pd.DataFrame:
    if run.raw_dir is None:
        return pd.DataFrame()
    path = run.raw_dir / "state_update_debug.csv"
    if not path.exists():
        return pd.DataFrame()
    df = pd.read_csv(path)
    if "event_type" in df.columns:
        df = df.loc[df["event_type"].astype(str) == "gnss_position"].copy()
    for col in [
        "armed_time_sec",
        "ros_time_sec",
        "gnss_position_nis_h_2d",
        "gnss_position_gate_threshold_nis",
        "gnss_position_update_accepted",
        "mavros_armed",
    ]:
        if col in df.columns:
            df[col] = pd.to_numeric(df[col], errors="coerce")
    if "armed_time_sec" not in df.columns or df["armed_time_sec"].isna().all():
        if "ros_time_sec" in df.columns and not df["ros_time_sec"].isna().all():
            df["armed_time_sec"] = df["ros_time_sec"] - float(df["ros_time_sec"].min())
        else:
            df["armed_time_sec"] = np.arange(len(df), dtype=float)
    df = df.loc[np.isfinite(df["gnss_position_nis_h_2d"])].copy()
    df = df.loc[df["armed_time_sec"] >= 0.0].copy()
    df["stage"] = run.stage
    df["route"] = run.route
    df["label"] = run.label
    df["paper_route"] = run.paper_route
    df["chi2_2d_95"] = CHI2_2D_95
    df["exceeds_chi2_2d_95"] = (df["gnss_position_nis_h_2d"] > CHI2_2D_95).astype(int)
    return df


def compute_nis_metrics(runs: list[EvidenceRun]) -> tuple[pd.DataFrame, pd.DataFrame]:
    frames = [read_nis(run) for run in runs]
    series = pd.concat([f for f in frames if not f.empty], ignore_index=True) if any(not f.empty for f in frames) else pd.DataFrame()
    if series.empty:
        return pd.DataFrame(), pd.DataFrame()
    rows: list[dict[str, object]] = []
    for (stage, route, label, paper_route), group in series.groupby(["stage", "route", "label", "paper_route"], dropna=False):
        accepted = pd.to_numeric(group.get("gnss_position_update_accepted", pd.Series(np.nan, index=group.index)), errors="coerce")
        rows.append(
            {
                "stage": stage,
                "paper_route": paper_route,
                "route": route,
                "label": label,
                "rows": int(group.shape[0]),
                "chi2_2d_95": CHI2_2D_95,
                "exceed_count": int(group["exceeds_chi2_2d_95"].sum()),
                "exceedance_ratio": float(group["exceeds_chi2_2d_95"].mean()),
                "hnis_mean": float(group["gnss_position_nis_h_2d"].mean()),
                "hnis_p95": float(group["gnss_position_nis_h_2d"].quantile(0.95)),
                "hnis_max": float(group["gnss_position_nis_h_2d"].max()),
                "accepted_ratio": float(accepted.mean()) if accepted.notna().any() else math.nan,
            }
        )
    return pd.DataFrame(rows), series


def aggregate_metrics(metrics: pd.DataFrame) -> pd.DataFrame:
    rows: list[dict[str, object]] = []
    for (stage, segment), group in metrics.groupby(["stage", "segment"]):
        rows.append(
            {
                "stage": stage,
                "segment": segment,
                "routes": int(group.shape[0]),
                "rmse_win_count": int((group["rmse_delta_m"] < 0.0).sum()),
                "ekf2_rmse_mean_m": float(group["ekf2_rmse_m"].mean()),
                "iekf_rmse_mean_m": float(group["iekf_rmse_m"].mean()),
                "rmse_delta_mean_m": float(group["rmse_delta_m"].mean()),
                "rmse_improvement_mean_pct": float(group["rmse_improvement_pct"].mean()),
                "mae_improvement_mean_pct": float(group["mae_improvement_pct"].mean()),
                "max_improvement_mean_pct": float(group["max_improvement_pct"].mean()),
            }
        )
    return pd.DataFrame(rows).sort_values(["stage", "segment"])


def save_csv(df: pd.DataFrame, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    df.to_csv(path, index=False, quoting=csv.QUOTE_MINIMAL)


def save_fig(fig: plt.Figure, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, bbox_inches="tight", pad_inches=0.08)
    plt.close(fig)


def t2_labels(metrics: pd.DataFrame) -> list[str]:
    subset = metrics.loc[(metrics["stage"] == "T2") & (metrics["segment"] == "main_40_180")].copy()
    subset = subset.sort_values("paper_route")
    return subset["label"].tolist()


def stage_labels(metrics: pd.DataFrame, stage: str) -> list[str]:
    subset = metrics.loc[(metrics["stage"] == stage) & (metrics["segment"] == "main_40_180")].copy()
    subset = subset.sort_values("paper_route")
    return subset["label"].tolist()


def make_t2_trajectory_grid(joined: dict[str, pd.DataFrame], labels: list[str], out: Path) -> None:
    fig, axes = plt.subplots(2, 3, figsize=(12.5, 7.0))
    axes = axes.ravel()
    for ax, label in zip(axes, labels):
        data = joined[label]
        mask = segment_masks(data)["mission_all"]
        d = data.loc[mask].copy()
        ax.plot(d["gt_x_m"], d["gt_y_m"], color="#111827", lw=1.8, label="Ground truth")
        ax.plot(d["ekf2_aligned_x_m"], d["ekf2_aligned_y_m"], color="#2563eb", lw=1.1, label="PX4 EKF2")
        ax.plot(d["iekf_normalized_aligned_x_m"], d["iekf_normalized_aligned_y_m"], color="#dc2626", lw=1.1, label="IEKF")
        ax.scatter(d["gt_x_m"].iloc[0], d["gt_y_m"].iloc[0], s=14, color="#16a34a")
        ax.set_title(label)
        ax.set_xlabel("x / m")
        ax.set_ylabel("y / m")
        ax.axis("equal")
    for ax in axes[len(labels) :]:
        ax.axis("off")
    handles, labels_ = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels_, loc="lower center", ncol=3, frameon=True)
    fig.suptitle("T2 holdout trajectory comparison")
    fig.subplots_adjust(bottom=0.12, top=0.90, hspace=0.35, wspace=0.28)
    save_fig(fig, out / "figures" / "t2_trajectory_comparison.png")


def make_t2_error_grid(joined: dict[str, pd.DataFrame], labels: list[str], out: Path) -> None:
    fig, axes = plt.subplots(5, 1, figsize=(11.5, 10.0), sharex=False)
    for ax, label in zip(axes, labels):
        data = joined[label]
        mask = segment_masks(data)["mission_all"] | segment_masks(data)["rtl_all"]
        d = data.loc[mask].copy()
        ax.plot(d["time_since_arm_sec"], d["ekf2_error_xy_m"], color="#2563eb", lw=1.0, label="PX4 EKF2")
        ax.plot(d["time_since_arm_sec"], d["iekf_error_xy_m"], color="#dc2626", lw=1.0, label="IEKF")
        ax.axvspan(40, 180, color="#64748b", alpha=0.10)
        ax.set_title(label)
        ax.set_ylabel("XY error / m")
    axes[-1].set_xlabel("Time since arm / s")
    handles, labels_ = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels_, loc="lower center", ncol=2, frameon=True)
    fig.suptitle("T2 holdout horizontal error time series")
    fig.subplots_adjust(bottom=0.08, top=0.94, hspace=0.62)
    save_fig(fig, out / "figures" / "t2_error_timeseries.png")


def make_metric_bars(metrics: pd.DataFrame, out: Path) -> None:
    data = metrics.loc[(metrics["stage"] == "T2") & (metrics["segment"] == "main_40_180")].copy()
    data = data.sort_values("paper_route")
    x = np.arange(len(data))
    width = 0.36
    fig, axes = plt.subplots(1, 3, figsize=(13.2, 4.0), sharex=True)
    specs = [
        ("rmse", "RMSE / m"),
        ("mae", "MAE / m"),
        ("max", "Max error / m"),
    ]
    for ax, (prefix, ylabel) in zip(axes, specs):
        ax.bar(x - width / 2, data[f"ekf2_{prefix}_m"], width, label="PX4 EKF2", color="#2563eb")
        ax.bar(x + width / 2, data[f"iekf_{prefix}_m"], width, label="IEKF", color="#dc2626")
        ax.set_ylabel(ylabel)
        ax.set_xticks(x)
        ax.set_xticklabels(data["paper_route"].fillna(data["route"]), rotation=25, ha="right")
        ax.set_title(prefix.upper())
    axes[0].legend(frameon=True)
    fig.suptitle("T2 main-window error metrics")
    fig.subplots_adjust(top=0.82, bottom=0.22, wspace=0.28)
    save_fig(fig, out / "figures" / "t2_rmse_mae_max_bars.png")

    fig, ax = plt.subplots(figsize=(8.4, 3.6))
    ax.bar(data["paper_route"].fillna(data["route"]), data["rmse_improvement_pct"], color="#0f766e")
    ax.axhline(0.0, color="#111827", lw=0.8)
    ax.set_ylabel("RMSE improvement / %")
    ax.set_title("T2 main-window IEKF improvement over PX4 EKF2")
    save_fig(fig, out / "figures" / "t2_rmse_improvement_percent.png")


def make_straight_turn_bars(metrics: pd.DataFrame, out: Path) -> None:
    data = metrics.loc[
        (metrics["stage"] == "T2") & (metrics["segment"].isin(["straight_main_40_180", "turn_main_40_180"]))
    ].copy()
    order = ["straight_main_40_180", "turn_main_40_180"]
    agg = data.groupby("segment")[["ekf2_rmse_m", "iekf_rmse_m", "rmse_improvement_pct"]].mean().reindex(order)
    labels = ["Straight", "Turn"]
    x = np.arange(len(labels))
    width = 0.36
    fig, ax = plt.subplots(figsize=(7.2, 4.0))
    ax.bar(x - width / 2, agg["ekf2_rmse_m"], width, label="PX4 EKF2", color="#2563eb")
    ax.bar(x + width / 2, agg["iekf_rmse_m"], width, label="IEKF", color="#dc2626")
    for i, pct in enumerate(agg["rmse_improvement_pct"]):
        if math.isfinite(float(pct)):
            ax.text(i, max(agg.iloc[i]["ekf2_rmse_m"], agg.iloc[i]["iekf_rmse_m"]) + 0.015, f"{pct:.1f}%", ha="center")
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.set_ylabel("Mean route RMSE / m")
    ax.set_title("T2 straight vs turn main-window error")
    ax.legend(frameon=True)
    save_fig(fig, out / "figures" / "t2_straight_turn_rmse.png")


def make_nis_grid(nis_series: pd.DataFrame, labels: list[str], out: Path) -> None:
    data = nis_series.loc[nis_series["label"].isin(labels)].copy()
    if data.empty:
        return
    fig, axes = plt.subplots(len(labels), 1, figsize=(11.2, 9.8), sharex=False)
    for ax, label in zip(axes, labels):
        d = data.loc[data["label"] == label].sort_values("armed_time_sec")
        ax.plot(d["armed_time_sec"], d["gnss_position_nis_h_2d"], color="#7c3aed", lw=0.8)
        ax.axhline(CHI2_2D_95, color="#dc2626", ls="--", lw=1.0, label="95% chi-square (2D)")
        ax.set_title(label)
        ax.set_ylabel("hNIS")
    axes[-1].set_xlabel("Armed time / s")
    axes[0].legend(loc="upper right", frameon=True)
    fig.suptitle("Offline GNSS position NIS diagnostic")
    fig.subplots_adjust(top=0.94, hspace=0.62)
    save_fig(fig, out / "figures" / "t2_offline_nis_curves.png")


def make_stage_trajectory_grid(joined: dict[str, pd.DataFrame], labels: list[str], out: Path, prefix: str, title: str) -> None:
    if not labels:
        return
    cols = min(3, len(labels))
    rows = int(math.ceil(len(labels) / cols))
    fig, axes = plt.subplots(rows, cols, figsize=(4.2 * cols, 3.4 * rows), squeeze=False)
    flat = axes.ravel()
    for ax, label in zip(flat, labels):
        data = joined[label]
        masks = segment_masks(data)
        d = data.loc[masks["mission_all"] | masks["rtl_all"]].copy()
        ax.plot(d["gt_x_m"], d["gt_y_m"], color="#111827", lw=1.7, label="Ground truth")
        ax.plot(d["ekf2_aligned_x_m"], d["ekf2_aligned_y_m"], color="#2563eb", lw=1.0, label="PX4 EKF2")
        ax.plot(d["iekf_normalized_aligned_x_m"], d["iekf_normalized_aligned_y_m"], color="#dc2626", lw=1.0, label="IEKF")
        ax.set_title(label)
        ax.set_xlabel("x / m")
        ax.set_ylabel("y / m")
        ax.axis("equal")
    for ax in flat[len(labels) :]:
        ax.axis("off")
    handles, labels_ = flat[0].get_legend_handles_labels()
    fig.legend(handles, labels_, loc="lower center", ncol=3, frameon=True)
    fig.suptitle(title)
    fig.subplots_adjust(bottom=0.13, top=0.88, hspace=0.35, wspace=0.28)
    save_fig(fig, out / "figures" / f"{prefix}_trajectory_comparison.png")


def make_stage_error_grid(joined: dict[str, pd.DataFrame], labels: list[str], out: Path, prefix: str, title: str) -> None:
    if not labels:
        return
    fig, axes = plt.subplots(len(labels), 1, figsize=(11.2, 2.1 * len(labels)), sharex=False)
    if len(labels) == 1:
        axes = [axes]
    for ax, label in zip(axes, labels):
        data = joined[label]
        masks = segment_masks(data)
        d = data.loc[masks["mission_all"] | masks["rtl_all"]].copy()
        ax.plot(d["time_since_arm_sec"], d["ekf2_error_xy_m"], color="#2563eb", lw=1.0, label="PX4 EKF2")
        ax.plot(d["time_since_arm_sec"], d["iekf_error_xy_m"], color="#dc2626", lw=1.0, label="IEKF")
        ax.axvspan(40, 180, color="#64748b", alpha=0.10)
        ax.set_title(label)
        ax.set_ylabel("XY error / m")
    axes[-1].set_xlabel("Time since arm / s")
    handles, labels_ = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels_, loc="lower center", ncol=2, frameon=True)
    fig.suptitle(title)
    fig.subplots_adjust(bottom=0.08, top=0.93, hspace=0.66)
    save_fig(fig, out / "figures" / f"{prefix}_error_timeseries.png")


def make_stage_metric_bars(metrics: pd.DataFrame, out: Path, stage: str, prefix: str, title: str) -> None:
    data = metrics.loc[(metrics["stage"] == stage) & (metrics["segment"] == "main_40_180")].copy()
    if data.empty:
        return
    data = data.sort_values("paper_route")
    x = np.arange(len(data))
    width = 0.36
    fig, axes = plt.subplots(1, 3, figsize=(13.2, 4.0), sharex=True)
    specs = [("rmse", "RMSE / m"), ("mae", "MAE / m"), ("max", "Max error / m")]
    for ax, (prefix_metric, ylabel) in zip(axes, specs):
        ax.bar(x - width / 2, data[f"ekf2_{prefix_metric}_m"], width, label="PX4 EKF2", color="#2563eb")
        ax.bar(x + width / 2, data[f"iekf_{prefix_metric}_m"], width, label="IEKF", color="#dc2626")
        ax.set_ylabel(ylabel)
        ax.set_xticks(x)
        ax.set_xticklabels(data["paper_route"].fillna(data["route"]), rotation=25, ha="right")
        ax.set_title(prefix_metric.upper())
    axes[0].legend(frameon=True)
    fig.suptitle(title)
    fig.subplots_adjust(top=0.82, bottom=0.24, wspace=0.28)
    save_fig(fig, out / "figures" / f"{prefix}_rmse_mae_max_bars.png")


def make_stage_straight_turn_bars(metrics: pd.DataFrame, out: Path, stage: str, prefix: str, title: str) -> None:
    data = metrics.loc[
        (metrics["stage"] == stage) & (metrics["segment"].isin(["straight_main_40_180", "turn_main_40_180"]))
    ].copy()
    if data.empty:
        return
    order = ["straight_main_40_180", "turn_main_40_180"]
    agg = data.groupby("segment")[["ekf2_rmse_m", "iekf_rmse_m", "rmse_improvement_pct"]].mean().reindex(order)
    labels = ["Straight", "Turn"]
    x = np.arange(len(labels))
    width = 0.36
    fig, ax = plt.subplots(figsize=(7.2, 4.0))
    ax.bar(x - width / 2, agg["ekf2_rmse_m"], width, label="PX4 EKF2", color="#2563eb")
    ax.bar(x + width / 2, agg["iekf_rmse_m"], width, label="IEKF", color="#dc2626")
    for i, pct in enumerate(agg["rmse_improvement_pct"]):
        if math.isfinite(float(pct)):
            ax.text(i, max(agg.iloc[i]["ekf2_rmse_m"], agg.iloc[i]["iekf_rmse_m"]) + 0.015, f"{pct:.1f}%", ha="center")
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.set_ylabel("Mean route RMSE / m")
    ax.set_title(title)
    ax.legend(frameon=True)
    save_fig(fig, out / "figures" / f"{prefix}_straight_turn_rmse.png")


def make_stage_nis_grid(nis_series: pd.DataFrame, labels: list[str], out: Path, prefix: str, title: str) -> None:
    data = nis_series.loc[nis_series["label"].isin(labels)].copy() if not nis_series.empty else pd.DataFrame()
    if data.empty:
        return
    fig, axes = plt.subplots(len(labels), 1, figsize=(11.2, 2.0 * len(labels)), sharex=False)
    if len(labels) == 1:
        axes = [axes]
    for ax, label in zip(axes, labels):
        d = data.loc[data["label"] == label].sort_values("armed_time_sec")
        ax.plot(d["armed_time_sec"], d["gnss_position_nis_h_2d"], color="#7c3aed", lw=0.8)
        ax.axhline(CHI2_2D_95, color="#dc2626", ls="--", lw=1.0, label="95% chi-square (2D)")
        ax.set_title(label)
        ax.set_ylabel("hNIS")
    axes[-1].set_xlabel("Armed time / s")
    axes[0].legend(loc="upper right", frameon=True)
    fig.suptitle(title)
    fig.subplots_adjust(top=0.92, hspace=0.66)
    save_fig(fig, out / "figures" / f"{prefix}_offline_nis_curves.png")


def write_switch_inventory(out: Path) -> pd.DataFrame:
    rows = [
        {
            "item": "PX4 EKF2 baseline",
            "status": "available and used",
            "evidence": "EKF2 aligned trajectory/error columns in every groundtruth_joined.csv",
            "paper_use": "primary baseline",
        },
        {
            "item": "IEKF position plus GNSS velocity",
            "status": "available and used",
            "evidence": "cand19 has enable_gnss_velocity_update=true in generated overlays",
            "paper_use": "primary fixed-parameter IEKF result",
        },
        {
            "item": "IEKF position-only",
            "status": "switch exists, not rerun in this package",
            "evidence": "enable_gnss_velocity_update can be set false; existing cand46/cand47 are velocity-off diagnostics, not frozen cand19 T2 holdout",
            "paper_use": "future controlled ablation unless rerun as cand19-position-only",
        },
        {
            "item": "single EKF / one-pass IEKF",
            "status": "no direct runtime switch found",
            "evidence": "no iteration-count or one-pass update parameter found in current kf_node/config search",
            "paper_use": "future work; do not claim",
        },
        {
            "item": "IEKF iteration count sensitivity",
            "status": "no direct runtime switch found",
            "evidence": "no 1/2/3/5 iteration parameter found in current kf_node/config search",
            "paper_use": "future work; do not claim",
        },
        {
            "item": "mild GNSS position noise scale",
            "status": "no clean input-noise injection switch found",
            "evidence": "available knobs tune estimator covariance/dropzones, not a simple clean/1.5x/2x measurement-noise injection protocol",
            "paper_use": "future work unless an explicit measurement-noise relay is added",
        },
        {
            "item": "single-frame GNSS position outlier",
            "status": "no clean position-outlier injection switch found",
            "evidence": "native velocity outlier guard exists but is not a GNSS position outlier injector",
            "paper_use": "future work; do not claim",
        },
        {
            "item": "offline NIS diagnostic",
            "status": "available and used",
            "evidence": "state_update_debug.csv contains gnss_position_nis_h_2d",
            "paper_use": "diagnostic only, not online control",
        },
    ]
    df = pd.DataFrame(rows)
    save_csv(df, out / "tables" / "switch_inventory.csv")
    return df


def md_table(df: pd.DataFrame, columns: list[str], n: int | None = None) -> str:
    work = df.loc[:, columns].copy()
    if n is not None:
        work = work.head(n)
    for col in work.columns:
        if pd.api.types.is_float_dtype(work[col]):
            work[col] = work[col].map(lambda x: "" if pd.isna(x) else f"{x:.4f}")
        else:
            work[col] = work[col].fillna("").astype(str)
    widths = [max(len(col), *(len(str(v)) for v in work[col])) for col in work.columns]
    header = "| " + " | ".join(col.ljust(widths[i]) for i, col in enumerate(work.columns)) + " |"
    sep = "| " + " | ".join("-" * widths[i] for i in range(len(widths))) + " |"
    lines = [header, sep]
    for _, row in work.iterrows():
        lines.append("| " + " | ".join(str(row[col]).ljust(widths[i]) for i, col in enumerate(work.columns)) + " |")
    return "\n".join(lines)


def route_lookup(metrics: pd.DataFrame, stage: str, route: str, segment: str) -> pd.Series | None:
    rows = metrics.loc[(metrics["stage"] == stage) & (metrics["route"] == route) & (metrics["segment"] == segment)]
    if rows.empty:
        return None
    return rows.iloc[0]


def nis_lookup(nis: pd.DataFrame, stage: str, route: str) -> pd.Series | None:
    if nis.empty:
        return None
    rows = nis.loc[(nis["stage"] == stage) & (nis["route"] == route)]
    if rows.empty:
        return None
    return rows.iloc[0]


def value_or_nan(row: pd.Series | None, col: str) -> float:
    if row is None or col not in row.index:
        return math.nan
    val = row[col]
    try:
        return float(val)
    except (TypeError, ValueError):
        return math.nan


def text_or_empty(row: pd.Series | None, col: str) -> str:
    if row is None or col not in row.index or pd.isna(row[col]):
        return ""
    return str(row[col])


def combined_order_key(row: pd.Series) -> tuple[int, str]:
    stage_order = {"T2": 0, "expanded": 1}
    return (stage_order.get(str(row["stage"]), 9), str(row.get("paper_route", "")))


def build_combined_outputs(metrics: pd.DataFrame, nis: pd.DataFrame) -> dict[str, pd.DataFrame]:
    main = metrics.loc[
        metrics["stage"].isin(["T2", "expanded"]) & (metrics["segment"] == "main_40_180")
    ].copy()
    if main.empty:
        empty = pd.DataFrame()
        return {
            "combined_routes": empty,
            "combined_summary": empty,
            "straight_turn_summary": empty,
            "rtl_boundary_cases": empty,
            "nis_exceedance_summary": empty,
        }

    main = main.sort_values(["stage", "paper_route"], key=lambda col: col.map(str))
    main = main.sort_values(by=["stage", "paper_route"], key=lambda col: col.map(str))
    ordered_rows = sorted([row for _, row in main.iterrows()], key=combined_order_key)
    rows: list[dict[str, object]] = []
    for main_row in ordered_rows:
        stage = str(main_row["stage"])
        route = str(main_row["route"])
        mission = route_lookup(metrics, stage, route, "mission_all")
        rtl = route_lookup(metrics, stage, route, "rtl_all")
        straight = route_lookup(metrics, stage, route, "straight_main_40_180")
        turn = route_lookup(metrics, stage, route, "turn_main_40_180")
        nis_row = nis_lookup(nis, stage, route)
        notes: list[str] = []
        rtl_delta = value_or_nan(rtl, "rmse_delta_m")
        if math.isfinite(rtl_delta) and rtl_delta > 0.0:
            notes.append("RTL boundary case: IEKF RMSE is higher than EKF2")
        if route == "shortgen17":
            notes.append("valid negative result; retained as route-sensitivity limitation")
        rows.append(
            {
                "set": "original_t2_holdout" if stage == "T2" else "expanded_holdout",
                "paper_route": text_or_empty(main_row, "paper_route"),
                "route": route,
                "role": text_or_empty(main_row, "role"),
                "main_window": "40-180 s after arming",
                "main_win": bool(value_or_nan(main_row, "rmse_delta_m") < 0.0),
                "mission_ekf2_rmse_m": value_or_nan(mission, "ekf2_rmse_m"),
                "mission_iekf_rmse_m": value_or_nan(mission, "iekf_rmse_m"),
                "mission_rmse_delta_m": value_or_nan(mission, "rmse_delta_m"),
                "mission_rmse_improvement_pct": value_or_nan(mission, "rmse_improvement_pct"),
                "mission_ekf2_mae_m": value_or_nan(mission, "ekf2_mae_m"),
                "mission_iekf_mae_m": value_or_nan(mission, "iekf_mae_m"),
                "mission_mae_improvement_pct": value_or_nan(mission, "mae_improvement_pct"),
                "mission_ekf2_max_m": value_or_nan(mission, "ekf2_max_m"),
                "mission_iekf_max_m": value_or_nan(mission, "iekf_max_m"),
                "mission_max_improvement_pct": value_or_nan(mission, "max_improvement_pct"),
                "main_rmse_delta_m": value_or_nan(main_row, "rmse_delta_m"),
                "rtl_ekf2_rmse_m": value_or_nan(rtl, "ekf2_rmse_m"),
                "rtl_iekf_rmse_m": value_or_nan(rtl, "iekf_rmse_m"),
                "rtl_rmse_delta_m": rtl_delta,
                "rtl_rmse_improvement_pct": value_or_nan(rtl, "rmse_improvement_pct"),
                "rtl_ekf2_mae_m": value_or_nan(rtl, "ekf2_mae_m"),
                "rtl_iekf_mae_m": value_or_nan(rtl, "iekf_mae_m"),
                "rtl_mae_improvement_pct": value_or_nan(rtl, "mae_improvement_pct"),
                "rtl_ekf2_max_m": value_or_nan(rtl, "ekf2_max_m"),
                "rtl_iekf_max_m": value_or_nan(rtl, "iekf_max_m"),
                "rtl_max_improvement_pct": value_or_nan(rtl, "max_improvement_pct"),
                "main_ekf2_rmse_m": value_or_nan(main_row, "ekf2_rmse_m"),
                "main_iekf_rmse_m": value_or_nan(main_row, "iekf_rmse_m"),
                "main_rmse_improvement_pct": value_or_nan(main_row, "rmse_improvement_pct"),
                "main_ekf2_mae_m": value_or_nan(main_row, "ekf2_mae_m"),
                "main_iekf_mae_m": value_or_nan(main_row, "iekf_mae_m"),
                "main_mae_improvement_pct": value_or_nan(main_row, "mae_improvement_pct"),
                "main_ekf2_max_m": value_or_nan(main_row, "ekf2_max_m"),
                "main_iekf_max_m": value_or_nan(main_row, "iekf_max_m"),
                "main_max_improvement_pct": value_or_nan(main_row, "max_improvement_pct"),
                "straight_ekf2_rmse_m": value_or_nan(straight, "ekf2_rmse_m"),
                "straight_iekf_rmse_m": value_or_nan(straight, "iekf_rmse_m"),
                "straight_rmse_improvement_pct": value_or_nan(straight, "rmse_improvement_pct"),
                "turn_ekf2_rmse_m": value_or_nan(turn, "ekf2_rmse_m"),
                "turn_iekf_rmse_m": value_or_nan(turn, "iekf_rmse_m"),
                "turn_rmse_improvement_pct": value_or_nan(turn, "rmse_improvement_pct"),
                "nis_exceedance_ratio": value_or_nan(nis_row, "exceedance_ratio"),
                "nis_hnis_mean": value_or_nan(nis_row, "hnis_mean"),
                "nis_hnis_p95": value_or_nan(nis_row, "hnis_p95"),
                "nis_hnis_max": value_or_nan(nis_row, "hnis_max"),
                "notes": "; ".join(notes),
            }
        )
    combined_routes = pd.DataFrame(rows)

    improvements = combined_routes["main_rmse_improvement_pct"].dropna()
    ekf2_rmse = combined_routes["main_ekf2_rmse_m"].dropna()
    iekf_rmse = combined_routes["main_iekf_rmse_m"].dropna()
    nis_ratios = combined_routes["nis_exceedance_ratio"].dropna()
    total = int(combined_routes.shape[0])
    wins = int(combined_routes["main_win"].sum())
    summary_rows = [
        {
            "metric": "delta_sign_definition",
            "value": "rmse_delta_m = IEKF_RMSE - EKF2_RMSE; negative favors IEKF",
            "unit": "",
            "definition": "Improvement percentage is (EKF2_RMSE - IEKF_RMSE) / EKF2_RMSE * 100; positive favors IEKF.",
        },
        {"metric": "main_window_win_count", "value": wins, "unit": "routes", "definition": "Number of routes with main_rmse_delta_m < 0."},
        {"metric": "main_window_total_routes", "value": total, "unit": "routes", "definition": "Original T2 plus expanded holdout routes."},
        {"metric": "main_window_win_fraction", "value": f"{wins}/{total}", "unit": "", "definition": "Combined short-route clean holdout win count."},
        {"metric": "mean_ekf2_rmse", "value": float(ekf2_rmse.mean()), "unit": "m", "definition": "Mean route-level EKF2 RMSE in the main window."},
        {"metric": "mean_iekf_rmse", "value": float(iekf_rmse.mean()), "unit": "m", "definition": "Mean route-level IEKF RMSE in the main window."},
        {"metric": "mean_improvement", "value": float(improvements.mean()), "unit": "%", "definition": "Mean route-level RMSE improvement percentage."},
        {"metric": "median_improvement", "value": float(improvements.median()), "unit": "%", "definition": "Median route-level RMSE improvement percentage."},
        {"metric": "std_improvement", "value": float(improvements.std()), "unit": "%", "definition": "Sample standard deviation of route-level RMSE improvement percentage."},
        {"metric": "min_improvement", "value": float(improvements.min()), "unit": "%", "definition": "Worst route-level RMSE improvement percentage; negative means IEKF was worse."},
        {"metric": "max_improvement", "value": float(improvements.max()), "unit": "%", "definition": "Best route-level RMSE improvement percentage."},
        {
            "metric": "rtl_boundary_case_count",
            "value": int((combined_routes["rtl_rmse_delta_m"] > 0.0).sum()),
            "unit": "routes",
            "definition": "RTL segments where IEKF RMSE is higher than EKF2.",
        },
        {"metric": "nis_exceedance_mean", "value": float(nis_ratios.mean()), "unit": "ratio", "definition": "Mean offline horizontal NIS exceedance ratio against 95% chi-square threshold."},
        {"metric": "nis_exceedance_median", "value": float(nis_ratios.median()), "unit": "ratio", "definition": "Median offline NIS exceedance ratio."},
        {"metric": "nis_exceedance_std", "value": float(nis_ratios.std()), "unit": "ratio", "definition": "Sample standard deviation of offline NIS exceedance ratio."},
        {"metric": "nis_exceedance_min", "value": float(nis_ratios.min()), "unit": "ratio", "definition": "Minimum offline NIS exceedance ratio."},
        {"metric": "nis_exceedance_max", "value": float(nis_ratios.max()), "unit": "ratio", "definition": "Maximum offline NIS exceedance ratio."},
    ]
    combined_summary = pd.DataFrame(summary_rows)

    st_rows: list[dict[str, object]] = []
    for segment, name in [("straight_main_40_180", "straight"), ("turn_main_40_180", "turn")]:
        group = metrics.loc[metrics["stage"].isin(["T2", "expanded"]) & (metrics["segment"] == segment)].copy()
        st_rows.append(
            {
                "segment": name,
                "routes": int(group.shape[0]),
                "rows_mean": float(group["rows"].mean()),
                "win_count": int((group["rmse_delta_m"] < 0.0).sum()),
                "ekf2_rmse_mean_m": float(group["ekf2_rmse_m"].mean()),
                "iekf_rmse_mean_m": float(group["iekf_rmse_m"].mean()),
                "rmse_delta_mean_m": float(group["rmse_delta_m"].mean()),
                "rmse_improvement_mean_pct": float(group["rmse_improvement_pct"].mean()),
            }
        )
    straight_turn_summary = pd.DataFrame(st_rows)
    rtl_boundary_cases = combined_routes.loc[combined_routes["rtl_rmse_delta_m"] > 0.0].copy()
    nis_exceedance_summary = pd.DataFrame(
        [
            {
                "count": int(nis_ratios.shape[0]),
                "mean": float(nis_ratios.mean()),
                "std": float(nis_ratios.std()),
                "min": float(nis_ratios.min()),
                "q25": float(nis_ratios.quantile(0.25)),
                "median": float(nis_ratios.median()),
                "q75": float(nis_ratios.quantile(0.75)),
                "max": float(nis_ratios.max()),
                "chi2_2d_95": CHI2_2D_95,
            }
        ]
    )
    return {
        "combined_routes": combined_routes,
        "combined_summary": combined_summary,
        "straight_turn_summary": straight_turn_summary,
        "rtl_boundary_cases": rtl_boundary_cases,
        "nis_exceedance_summary": nis_exceedance_summary,
    }


def write_combined_tables(out: Path, combined: dict[str, pd.DataFrame]) -> None:
    save_csv(combined["combined_routes"], out / "tables" / "combined_holdout_metrics.csv")
    save_csv(combined["combined_summary"], out / "tables" / "combined_summary.csv")
    save_csv(combined["straight_turn_summary"], out / "tables" / "combined_straight_turn_summary.csv")
    save_csv(combined["rtl_boundary_cases"], out / "tables" / "combined_rtl_boundary_cases.csv")
    save_csv(combined["nis_exceedance_summary"], out / "tables" / "combined_nis_exceedance_summary.csv")


def summary_value(combined: dict[str, pd.DataFrame], metric: str) -> object:
    summary = combined.get("combined_summary", pd.DataFrame())
    rows = summary.loc[summary["metric"] == metric] if not summary.empty else pd.DataFrame()
    if rows.empty:
        return math.nan
    return rows.iloc[0]["value"]


def summary_float(combined: dict[str, pd.DataFrame], metric: str) -> float:
    val = summary_value(combined, metric)
    try:
        return float(val)
    except (TypeError, ValueError):
        return math.nan


def write_readme(
    out: Path,
    metrics: pd.DataFrame,
    aggregate: pd.DataFrame,
    nis: pd.DataFrame,
    combined: dict[str, pd.DataFrame],
) -> None:
    t2_main = metrics.loc[(metrics["stage"] == "T2") & (metrics["segment"] == "main_40_180")].sort_values("paper_route")
    agg_t2_main = aggregate.loc[(aggregate["stage"] == "T2") & (aggregate["segment"] == "main_40_180")].iloc[0]
    expanded_main = metrics.loc[(metrics["stage"] == "expanded") & (metrics["segment"] == "main_40_180")].sort_values("paper_route")
    combined_routes = combined.get("combined_routes", pd.DataFrame())
    combined_st = combined.get("straight_turn_summary", pd.DataFrame())
    combined_rtl = combined.get("rtl_boundary_cases", pd.DataFrame())
    combined_nis = combined.get("nis_exceedance_summary", pd.DataFrame())
    combined_win_fraction = summary_value(combined, "main_window_win_fraction")
    mean_ekf2 = summary_float(combined, "mean_ekf2_rmse")
    mean_iekf = summary_float(combined, "mean_iekf_rmse")
    mean_improvement = summary_float(combined, "mean_improvement")
    median_improvement = summary_float(combined, "median_improvement")
    std_improvement = summary_float(combined, "std_improvement")
    min_improvement = summary_float(combined, "min_improvement")
    max_improvement = summary_float(combined, "max_improvement")
    text = f"""# Cand19 Paper Experiment Package

Generated offline from existing artifacts. No PX4/Gazebo/MAVROS flight stack is started by this package.

Candidate: `{CANDIDATE}`

Main claim boundary: PX4 Gazebo Classic, `empty.world` through `PX4_SAFE_PX4_WORLD=none`, Iris vehicle, clean GNSS, fixed-parameter IEKF, `global_groundtruth + raw_wgs84_enu`.

Delta sign definition: `rmse_delta_m = IEKF_RMSE - EKF2_RMSE`; negative values mean IEKF has lower error. `rmse_improvement_pct = (EKF2_RMSE - IEKF_RMSE) / EKF2_RMSE * 100`; positive values mean IEKF improves over EKF2.

## Reproduce

```bash
python3 src/kf_gins_ros2_native/scripts/generate_cand19_paper_experiment_package.py \\
  --expanded-root artifacts/manual/cand19_expanded_holdout_20260514_1942 \\
  --out-dir artifacts/manual/paper_experiment_package_20260514_expanded
```

## Tables

- `tables/route_segment_metrics.csv`: RMSE, MAE, max error, and improvement percentage for T0/T1/T2/repeatability runs.
- `tables/t2_holdout_metrics.csv`: compact T2 clean holdout table.
- `tables/expanded_holdout_metrics.csv`: optional expanded clean holdout table when `--expanded-root` is provided.
- `tables/combined_holdout_metrics.csv`: original 5-route T2 plus expanded `shortgen17-21` clean holdout table, including mission/main/RTL RMSE, MAE, maximum error, deltas, and improvement percentages.
- `tables/combined_summary.csv`: win count, mean/median/std/min/max improvement, NIS summary, and delta definitions.
- `tables/combined_straight_turn_summary.csv`: combined straight/turn aggregate.
- `tables/combined_rtl_boundary_cases.csv`: RTL segments where IEKF RMSE is higher than EKF2.
- `tables/combined_nis_exceedance_summary.csv`: combined offline hNIS exceedance-ratio summary.
- `tables/aggregate_summary.csv`: stage/segment aggregate statistics.
- `tables/nis_summary.csv`: offline hNIS exceedance ratio against the 95% 2D chi-square threshold.
- `tables/switch_inventory.csv`: checked experiment knobs and why unsupported items are future work.

## Figures

- `figures/t2_trajectory_comparison.png`
- `figures/t2_error_timeseries.png`
- `figures/t2_rmse_mae_max_bars.png`
- `figures/t2_rmse_improvement_percent.png`
- `figures/t2_straight_turn_rmse.png`
- `figures/t2_offline_nis_curves.png`
- `figures/expanded_*`: optional expanded holdout figures when expanded runs are present.

## T2 Main Result

T2 main-window wins: `{int((t2_main["rmse_delta_m"] < 0).sum())}/{len(t2_main)}`.
Mean T2 main-window RMSE improvement: `{agg_t2_main["rmse_improvement_mean_pct"]:.2f}%`.

{md_table(t2_main, ["paper_route", "route", "ekf2_rmse_m", "iekf_rmse_m", "rmse_delta_m", "rmse_improvement_pct"])}

## Expanded Holdout

Expanded holdout results are supplementary and do not replace the frozen 5-route T2 main result.

"""
    if not expanded_main.empty:
        wins = int((expanded_main["rmse_delta_m"] < 0).sum())
        text += f"Expanded main-window wins: `{wins}/{len(expanded_main)}`.\n\n"
        text += md_table(
            expanded_main,
            ["paper_route", "route", "role", "ekf2_rmse_m", "iekf_rmse_m", "rmse_delta_m", "rmse_improvement_pct"],
        )
        text += "\n"
    else:
        text += "No expanded holdout runs were included in this package.\n"

    text += f"""
## Combined 10-Route Holdout

The combined table joins the frozen 5-route T2 holdout with the supplementary expanded clean routes `shortgen17-21`. It is used to report route sensitivity, not to retune the estimator.

Combined main-window wins: `{combined_win_fraction}`.
Mean EKF2 RMSE: `{mean_ekf2:.4f} m`; mean IEKF RMSE: `{mean_iekf:.4f} m`.
Mean/median RMSE improvement: `{mean_improvement:.2f}%` / `{median_improvement:.2f}%`.
Std/min/max RMSE improvement: `{std_improvement:.2f}%` / `{min_improvement:.2f}%` / `{max_improvement:.2f}%`.

{md_table(combined_routes, ["set", "paper_route", "route", "role", "main_ekf2_rmse_m", "main_iekf_rmse_m", "main_rmse_delta_m", "main_rmse_improvement_pct", "nis_exceedance_ratio", "notes"])}

### Straight/Turn Aggregate

{md_table(combined_st, ["segment", "routes", "win_count", "ekf2_rmse_mean_m", "iekf_rmse_mean_m", "rmse_delta_mean_m", "rmse_improvement_mean_pct"])}

### RTL Boundary Cases

"""
    if not combined_rtl.empty:
        text += md_table(
            combined_rtl,
            ["set", "paper_route", "route", "rtl_rmse_delta_m", "main_rmse_delta_m", "main_rmse_improvement_pct", "notes"],
        )
        text += "\n"
    else:
        text += "No RTL boundary cases were detected.\n"

    if not combined_nis.empty:
        text += "\n### Combined NIS Exceedance Summary\n\n"
        text += md_table(combined_nis, ["count", "mean", "std", "min", "q25", "median", "q75", "max", "chi2_2d_95"])
        text += "\n"

    text += """
## Offline NIS

NIS is used only as an offline diagnostic here. It is not used for online rejection, weighting, or control in the reported cand19 result.

"""
    if not nis.empty:
        t2_nis = nis.loc[nis["stage"] == "T2"].sort_values("paper_route")
        text += md_table(t2_nis, ["paper_route", "route", "rows", "exceedance_ratio", "hnis_mean", "hnis_p95", "hnis_max"])
        text += "\n"
    (out / "README.md").write_text(text, encoding="utf-8")


def write_chapter_drafts(
    out: Path,
    metrics: pd.DataFrame,
    aggregate: pd.DataFrame,
    nis: pd.DataFrame,
    combined: dict[str, pd.DataFrame],
) -> None:
    drafts = out / "drafts"
    drafts.mkdir(parents=True, exist_ok=True)
    t2_main = metrics.loc[(metrics["stage"] == "T2") & (metrics["segment"] == "main_40_180")].sort_values("paper_route")
    t2_mission = metrics.loc[(metrics["stage"] == "T2") & (metrics["segment"] == "mission_all")].sort_values("paper_route")
    t2_turn = metrics.loc[(metrics["stage"] == "T2") & (metrics["segment"] == "turn_main_40_180")]
    t2_straight = metrics.loc[(metrics["stage"] == "T2") & (metrics["segment"] == "straight_main_40_180")]
    repeat_main = metrics.loc[(metrics["stage"] == "repeatability") & (metrics["segment"] == "main_40_180")].sort_values("route")
    expanded_main = metrics.loc[(metrics["stage"] == "expanded") & (metrics["segment"] == "main_40_180")].sort_values("paper_route")
    agg_t2_main = aggregate.loc[(aggregate["stage"] == "T2") & (aggregate["segment"] == "main_40_180")].iloc[0]
    expanded_win_text = "not run"
    if not expanded_main.empty:
        expanded_win_text = f"{int((expanded_main['rmse_delta_m'] < 0).sum())}/{len(expanded_main)}"
    combined_routes = combined.get("combined_routes", pd.DataFrame())
    combined_st = combined.get("straight_turn_summary", pd.DataFrame())
    combined_rtl = combined.get("rtl_boundary_cases", pd.DataFrame())
    combined_nis = combined.get("nis_exceedance_summary", pd.DataFrame())
    combined_win_fraction = summary_value(combined, "main_window_win_fraction")
    combined_wins = summary_value(combined, "main_window_win_count")
    combined_total = summary_value(combined, "main_window_total_routes")
    combined_mean_ekf2 = summary_float(combined, "mean_ekf2_rmse")
    combined_mean_iekf = summary_float(combined, "mean_iekf_rmse")
    combined_mean_improvement = summary_float(combined, "mean_improvement")
    combined_median_improvement = summary_float(combined, "median_improvement")
    combined_std_improvement = summary_float(combined, "std_improvement")
    combined_min_improvement = summary_float(combined, "min_improvement")
    combined_max_improvement = summary_float(combined, "max_improvement")

    chapter4 = f"""# Chapter 4. Experimental Validation

## 4.1 Experimental Platform

The experiments were conducted in PX4 SITL with Gazebo Classic. The vehicle model was the PX4 Iris quadrotor. The reported holdout results use `PX4_SAFE_PX4_WORLD=none`, which resolves to the empty Gazebo world, and therefore isolate estimator behavior from dense-scene collision or occlusion effects. GNSS measurements were clean nominal simulation measurements. No GPS denial zones, dropzone injection, online NIS rejection, online NIS weighting, or Sage-Husa covariance adaptation are included in the main result.

The evaluated estimator is a fixed-parameter GNSS/INS IEKF configuration, `cand19_pos08_arm05_vel04_vrw28`. The comparison baseline is PX4 EKF2, evaluated against the same global-groundtruth reference after common early-window alignment. The main evaluation branch is `global_groundtruth + raw_wgs84_enu`. Local-groundtruth evaluation is retained as a limitation and is not used to support the main performance claim.

## 4.2 Route Protocol and Metrics

The clean holdout set contains five short routes. In the paper route map, Route 01 corresponds to `shortgen08`, Route 02 to `shortgen09`, Route 03 to `shortgen10`, Route 04 to `shortgen13`, and Route 05 to `shortgen16`. Route `shortgen07` is excluded from the clean holdout set because it was used as failure-development evidence. Routes `shortgen05` and `shortgen06` are excluded because they failed route validity checks.

For each route, horizontal position error is computed with respect to the global-groundtruth reference. The reported metrics are RMSE, MAE, maximum horizontal error, and percentage improvement of IEKF over EKF2. The main maneuver window is 40-180 s after arming. Mission and RTL segments are also reported. To analyze maneuver sensitivity, the main window is split into straight and turning subsets using the logged turning flag and an 8 deg/s yaw-rate threshold.

## 4.3 Clean Holdout Results

The IEKF wins all five clean T2 holdout routes in the main maneuver window. The mean T2 main-window RMSE improvement is {agg_t2_main["rmse_improvement_mean_pct"]:.2f}%.

{md_table(t2_main, ["paper_route", "route", "ekf2_rmse_m", "iekf_rmse_m", "rmse_delta_m", "rmse_improvement_pct"])}

Mission-level results show the same direction of improvement:

{md_table(t2_mission, ["paper_route", "route", "ekf2_rmse_m", "iekf_rmse_m", "rmse_delta_m", "rmse_improvement_pct"])}

The route-level figures generated with this chapter show the same conclusion from three views: trajectory overlay, horizontal error time series, and RMSE/MAE/maximum-error bar charts. The weakest clean holdout margin is Route 02 (`shortgen09`), where the main-window delta remains negative and therefore still favors the IEKF.

## 4.4 Straight and Turning Segment Analysis

The straight/turn split is computed offline from the same logged data, not by manually selecting visual intervals. A sample is classified as turning if the logged turning flag is active or the absolute yaw-rate proxy exceeds 8 deg/s. Otherwise, main-window samples with horizontal speed above 0.5 m/s are classified as straight.

{md_table(pd.concat([t2_straight, t2_turn]).sort_values(["segment", "paper_route"]), ["segment", "paper_route", "route", "rows", "ekf2_rmse_m", "iekf_rmse_m", "rmse_delta_m", "rmse_improvement_pct"])}

This analysis indicates whether the improvement is concentrated in straight tracking, turning maneuvers, or both. It should be interpreted as a trajectory-derived diagnostic, because the current experiment does not include a separate hand-labeled turn dataset.

## 4.5 Repeatability Check

Three representative routes were rerun without retuning the candidate: `shortgen09`, `shortgen13`, and `shortgen16`. The repeated runs preserve the global-groundtruth IEKF win in the main window.

{md_table(repeat_main, ["route", "label", "ekf2_rmse_m", "iekf_rmse_m", "rmse_delta_m", "rmse_improvement_pct"])}

This supports the interpretation that the T2 result is not a single-run artifact. The `shortgen09` repeat required an offline artifact-path repair after a relative output directory split, but the mission itself completed successfully and was rescored with forced global and local groundtruth.

## 4.6 Expanded Holdout Check

After freezing the five-route T2 result, five additional clean short routes were generated to probe external route sensitivity: a straight line, a rectangle, an L-turn, an S/polyline, and an out-and-back route. These runs are supplementary. They are not used to retune the candidate and do not replace the frozen T2 holdout result.

"""
    if not expanded_main.empty:
        chapter4 += md_table(
            expanded_main,
            ["paper_route", "route", "role", "ekf2_rmse_m", "iekf_rmse_m", "rmse_delta_m", "rmse_improvement_pct"],
        )
        chapter4 += (
            "\n\nThe expanded check is mixed by design: it records both additional wins and failures. "
            "The straight-line route `shortgen17` is a valid negative result: its main-window delta is positive, so IEKF is worse than EKF2 on that route. "
            "It is retained as a route-sensitivity limitation rather than removed from the evidence package.\n"
        )
    else:
        chapter4 += "No expanded holdout runs were included in this package.\n"

    chapter4 += f"""
## 4.7 Combined Holdout Summary

For paper reporting, the frozen T2 holdout and the supplementary expanded holdout can be summarized together as a 10-route clean short-route set. This combined table must be interpreted with a clear sign convention: `rmse_delta_m = IEKF_RMSE - EKF2_RMSE`, so negative values favor IEKF, while positive values favor EKF2. The improvement percentage is `(EKF2_RMSE - IEKF_RMSE) / EKF2_RMSE * 100`, so positive values favor IEKF.

Across the combined set, IEKF wins {combined_win_fraction} main windows. Mean main-window RMSE is {combined_mean_ekf2:.4f} m for EKF2 and {combined_mean_iekf:.4f} m for IEKF. The mean and median RMSE improvements are {combined_mean_improvement:.2f}% and {combined_median_improvement:.2f}%, respectively. The route-level improvement distribution has standard deviation {combined_std_improvement:.2f}%, minimum {combined_min_improvement:.2f}%, and maximum {combined_max_improvement:.2f}%.

{md_table(combined_routes, ["set", "paper_route", "route", "role", "main_ekf2_rmse_m", "main_iekf_rmse_m", "main_rmse_delta_m", "main_rmse_improvement_pct", "nis_exceedance_ratio", "notes"])}

The combined straight/turn aggregate is:

{md_table(combined_st, ["segment", "routes", "win_count", "ekf2_rmse_mean_m", "iekf_rmse_mean_m", "rmse_delta_mean_m", "rmse_improvement_mean_pct"])}

RTL boundary cases are retained because they identify where the IEKF advantage weakens near return-to-land behavior:

"""
    if not combined_rtl.empty:
        chapter4 += md_table(
            combined_rtl,
            ["set", "paper_route", "route", "rtl_rmse_delta_m", "main_rmse_delta_m", "main_rmse_improvement_pct", "notes"],
        )
        chapter4 += "\n"
    else:
        chapter4 += "No RTL boundary cases were detected.\n"

    chapter4 += """
## 4.8 Offline NIS Diagnostic

The GNSS position horizontal NIS is reported only as an offline diagnostic. The 95% chi-square threshold for two horizontal dimensions is 5.991. The estimator does not use this diagnostic for online rejection or adaptive weighting in the reported result.

"""
    if not nis.empty:
        chapter4 += md_table(nis.loc[nis["stage"] == "T2"].sort_values("paper_route"), ["paper_route", "route", "rows", "exceedance_ratio", "hnis_mean", "hnis_p95", "hnis_max"])
        chapter4 += "\n"
    chapter4 += """
The NIS curves are useful for discussing consistency and occasional high-innovation intervals, but they should not be presented as evidence of an online robust controller.

## 4.9 Scope of Validity

The experimental claim is deliberately limited. The result supports a stable multi-route IEKF advantage over PX4 EKF2 under `global_groundtruth + raw_wgs84_enu` in clean short-route SITL experiments. It does not prove superiority under every groundtruth definition. In particular, the local-groundtruth evaluation branch remains a negative boundary result and should be discussed as a frame/evaluation limitation rather than hidden.

The experiment also does not claim performance in complex GNSS denial. Mild or severe GNSS degradation, single-frame position outliers, online NIS gating, Sage-Husa adaptation, and iteration-count sensitivity require additional controlled experiments before they can be reported as results.
"""

    chapter5 = f"""# Chapter 5. Conclusions and Future Work

## 5.1 Conclusions

This study evaluated a fixed-parameter GNSS/INS IEKF against PX4 EKF2 in PX4 Gazebo Classic SITL. Under the clean GNSS, empty-world, Iris-vehicle, global-groundtruth evaluation protocol, the selected IEKF candidate `cand19_pos08_arm05_vel04_vrw28` achieved lower horizontal position error than EKF2 on all five clean T2 holdout routes. In the main 40-180 s maneuver window, the IEKF won {int((t2_main["rmse_delta_m"] < 0).sum())}/{len(t2_main)} routes, with a mean RMSE improvement of {agg_t2_main["rmse_improvement_mean_pct"]:.2f}%.

The additional repeatability runs on `shortgen09`, `shortgen13`, and `shortgen16` preserved the same global-groundtruth advantage without retuning. A further expanded clean-route check produced {expanded_win_text} main-window wins. Combining the frozen T2 set and the expanded set gives {combined_win_fraction} main-window wins, mean EKF2 RMSE of {combined_mean_ekf2:.4f} m, mean IEKF RMSE of {combined_mean_iekf:.4f} m, mean improvement of {combined_mean_improvement:.2f}%, and median improvement of {combined_median_improvement:.2f}%. This combined summary records external route sensitivity rather than hiding it. In particular, `shortgen17` is retained as a valid negative result and should be presented as a limitation. The straight/turn split and offline NIS diagnostic add interpretability to the result: they show where the estimator improves the trajectory and how the GNSS innovation behaves relative to a standard 95% chi-square reference. These diagnostics are offline analyses and are not part of an online rejection or adaptation mechanism.

The most important limitation is the groundtruth-frame boundary. The current evidence supports the global-groundtruth/raw-WGS84-ENU branch, while local-groundtruth evaluation remains unresolved and can reverse the route-level conclusion. Therefore, the thesis should state the estimator's validated operating and evaluation conditions precisely instead of claiming universal superiority.

## 5.2 Future Work

First, the local/global groundtruth discrepancy should be resolved before expanding the claim to other reference definitions. This requires a frame audit and possibly a revised evaluation protocol, not merely additional route runs.

Second, the paper can be extended with controlled ablation experiments. The current code exposes GNSS velocity aiding switches, so a future controlled experiment can compare the fixed IEKF with and without GNSS velocity aiding under the same frozen candidate parameters. However, the present package does not include a full cand19 position-only T2 rerun, and it should not be claimed as completed.

Third, iteration-count sensitivity is a natural next experiment, but the current code search did not identify a direct runtime parameter for selecting 1/2/3/5 IEKF iterations. Adding such an experiment would require estimator or configuration support and should be treated as future work rather than inferred from the current data.

Fourth, Sage-Husa covariance adaptation and online NIS-based rejection or weighting should be positioned as candidate extensions. They are not part of the reported cand19 result. Future experiments should introduce them through a predefined protocol and evaluate them on separate clean and mildly degraded holdout routes.

Finally, GNSS degradation experiments should start with mild, reproducible conditions, such as explicit measurement-noise scaling and single-frame outlier injection. The current validated result does not include GPS denial zones or complex urban GNSS outages, so those scenarios should not be described as completed experimental evidence.
"""

    front_matter = f"""# Title, Abstract, and Contributions Draft

## Title

Fixed-Parameter GNSS/INS Iterated EKF Evaluation Against PX4 EKF2 in Clean UAV SITL Short-Route Flight

## Abstract

This paper evaluates a fixed-parameter GNSS/INS iterated extended Kalman filter (IEKF) against the PX4 EKF2 baseline in PX4 Gazebo Classic SITL. The experiments use the Iris quadrotor model, an empty Gazebo world, and clean nominal GNSS measurements. The main result is intentionally limited to the `global_groundtruth + raw_wgs84_enu` evaluation branch and does not claim GPS-denial or complex urban-GNSS robustness. A five-route clean holdout set is used after candidate selection, and three representative reruns are used to check repeatability. On the five clean holdout routes, the IEKF reduces main-window horizontal-position RMSE relative to EKF2 on 5/5 routes, with a mean RMSE improvement of {agg_t2_main["rmse_improvement_mean_pct"]:.2f}%. A supplementary expanded route check is reported without retuning. Combining the frozen and expanded clean routes gives {combined_win_fraction} main-window wins, mean EKF2 RMSE of {combined_mean_ekf2:.4f} m, mean IEKF RMSE of {combined_mean_iekf:.4f} m, and mean/median RMSE improvements of {combined_mean_improvement:.2f}%/{combined_median_improvement:.2f}%. The straight-line expanded route `shortgen17` is retained as a valid negative result. Additional straight/turn segmentation and offline GNSS-position NIS analysis are reported as diagnostics. The paper concludes that the fixed IEKF configuration provides a reproducible clean-GNSS short-route advantage under the global-groundtruth protocol, while local-groundtruth evaluation, GNSS degradation, online NIS control, Sage-Husa adaptation, and iteration-count sensitivity remain future work.

## Introduction Contributions

1. A reproducible estimator-to-estimator comparison protocol is established for PX4 EKF2 and a fixed-parameter GNSS/INS IEKF in PX4 Gazebo Classic, using Iris, `empty.world`, clean GNSS, and a frozen global-groundtruth evaluation branch.
2. A five-route clean holdout result is reported without using the holdout routes for tuning; the selected IEKF candidate wins all five main maneuver windows against PX4 EKF2.
3. Repeatability and route-sensitivity evidence is added through representative reruns and a supplementary expanded route set; `shortgen17` is retained as a valid negative result rather than removed.
4. Straight/turn error decomposition and offline GNSS-position NIS curves are provided to interpret when the IEKF improves the trajectory and when innovation consistency is weak.
5. The paper explicitly separates completed results from future extensions: Sage-Husa adaptation, online NIS rejection or weighting, GNSS degradation, position-only ablation, and IEKF iteration-count sensitivity are not claimed as completed experiments.
"""

    limitations = f"""# Limitations and Future Work Draft

The main limitation is the evaluation-frame boundary. The current result is a stable win under `global_groundtruth + raw_wgs84_enu`, while local-groundtruth evaluation remains unresolved and may reverse route-level conclusions. This limitation should be presented directly.

The second limitation is environmental scope. The experiments use clean GNSS, `empty.world`, and the Iris vehicle model. They do not validate performance in GPS-denial zones, urban multipath, or noisy real-world GNSS. The expanded `shortgen17` straight-line run is also retained as a valid negative clean-route result, showing that the IEKF advantage is not universal across every short route.

The third limitation is algorithmic scope. The reported cand19 result is fixed-parameter IEKF with GNSS velocity aiding. It is not an adaptive Sage-Husa estimator, does not use online NIS rejection or weighting, and does not include a completed IEKF iteration-count sensitivity study.

Future work should first resolve the local/global groundtruth discrepancy, then add predefined controlled ablations: position-only IEKF, GNSS velocity-aided IEKF, mild GNSS noise scaling, single-frame position outliers, and explicit IEKF iteration-count settings if the estimator exposes or adds a safe runtime parameter.
"""

    figure_captions = f"""# Figure Captions

**Figure 4-1. Frozen T2 clean-holdout trajectory comparison.** Ground-truth, PX4 EKF2, and fixed-parameter IEKF trajectories are overlaid for the five frozen T2 routes. The routes are evaluated in PX4 Gazebo Classic with Iris, `empty.world`, and clean GNSS.

**Figure 4-2. Frozen T2 horizontal error time series.** Horizontal position error of PX4 EKF2 and IEKF is shown over time for each T2 route. The shaded interval marks the main 40-180 s maneuver window used for the primary RMSE, MAE, and maximum-error statistics.

**Figure 4-3. Frozen T2 main-window RMSE, MAE, and maximum error.** Bar charts compare EKF2 and IEKF on the five frozen T2 routes. Lower values indicate better horizontal-position accuracy.

**Figure 4-4. Frozen T2 RMSE improvement percentage.** Improvement is defined as `(EKF2_RMSE - IEKF_RMSE) / EKF2_RMSE * 100`; positive values indicate lower IEKF RMSE.

**Figure 4-5. Frozen T2 straight/turn RMSE decomposition.** Main-window samples are classified as turning using the logged turning flag or an 8 deg/s yaw-rate threshold, and otherwise as straight when horizontal speed exceeds 0.5 m/s.

**Figure 4-6. Frozen T2 offline GNSS-position NIS.** Horizontal GNSS-position NIS is plotted against the 95% chi-square threshold for two dimensions, 5.991. This is an offline diagnostic only and is not used for online rejection, weighting, or control.

**Figure 4-7. Expanded clean-holdout trajectory comparison.** The supplementary expanded routes `shortgen17-21` probe straight-line, rectangular, L-turn, S/polyline, and out-and-back route sensitivity without retuning the estimator.

**Figure 4-8. Expanded clean-holdout horizontal error time series.** Error traces for the expanded routes show both additional wins and the valid negative result on `shortgen17`; failed or unfavorable routes are retained rather than removed.

**Figure 4-9. Expanded clean-holdout main-window RMSE, MAE, and maximum error.** The expanded set is used as a route-sensitivity check and does not replace the frozen T2 holdout claim.

**Figure 4-10. Expanded clean-holdout straight/turn RMSE decomposition.** The same offline straight/turn rule is applied to the expanded routes to keep the analysis comparable with the frozen T2 routes.

**Figure 4-11. Expanded clean-holdout offline GNSS-position NIS.** NIS curves are diagnostic consistency plots only; they do not demonstrate an online robust GNSS rejection mechanism.
"""

    claims_checklist = f"""# Final Paper Claims Checklist

## Allowed Claims

- The main experiment is limited to PX4 Gazebo Classic, Iris, `empty.world`, clean GNSS, fixed-parameter IEKF, GNSS velocity aiding, and `global_groundtruth + raw_wgs84_enu`.
- The primary frozen T2 holdout has 5/5 IEKF main-window wins against PX4 EKF2, with mean RMSE improvement {agg_t2_main["rmse_improvement_mean_pct"]:.2f}%.
- The combined frozen-plus-expanded clean short-route set has {combined_win_fraction} main-window wins, mean EKF2 RMSE {combined_mean_ekf2:.4f} m, mean IEKF RMSE {combined_mean_iekf:.4f} m, mean improvement {combined_mean_improvement:.2f}%, and median improvement {combined_median_improvement:.2f}%.
- Delta sign is fixed as `rmse_delta_m = IEKF_RMSE - EKF2_RMSE`; negative values favor IEKF. Improvement percentage is positive when IEKF is better.
- `shortgen17` is a valid negative result and must remain in the combined evidence package.
- Offline NIS curves and exceedance ratios may be used as diagnostics only.

## Disallowed Claims

- Do not claim GNSS-denied navigation, urban multipath robustness, or validated GPS-dropzone behavior.
- Do not claim Sage-Husa adaptation is part of the reported result.
- Do not claim online NIS rejection, weighting, or control is validated.
- Do not claim local-groundtruth evaluation also proves IEKF superiority.
- Do not claim position-only IEKF, single-pass EKF, or 1/2/3/5 iteration sensitivity results unless new controlled experiments are later added.

## Required Wording Guardrails

- Use "fixed-parameter GNSS/INS IEKF with GNSS velocity aiding" for cand19.
- Use "clean short-route SITL evaluation" instead of "GNSS-denied navigation".
- Use "supplementary expanded route-sensitivity check" for `shortgen17-21`.
- Mention `shortgen17` explicitly as a limitation when reporting the combined 10-route result.
- Mention that NIS is offline diagnostic evidence, not online estimator logic.
"""

    final_paste_ready = "\n\n".join(
        [
            front_matter,
            chapter4,
            chapter5,
            limitations,
            figure_captions,
            claims_checklist,
        ]
    )

    (drafts / "title_abstract_contributions_en.md").write_text(front_matter, encoding="utf-8")
    (drafts / "limitations_future_work_en.md").write_text(limitations, encoding="utf-8")
    (drafts / "chapter4_experiments_en.md").write_text(chapter4, encoding="utf-8")
    (drafts / "chapter5_conclusions_en.md").write_text(chapter5, encoding="utf-8")
    (drafts / "figure_captions_en.md").write_text(figure_captions, encoding="utf-8")
    (drafts / "final_paper_claims_checklist.md").write_text(claims_checklist, encoding="utf-8")
    (drafts / "final_paper_paste_ready_en.md").write_text(final_paste_ready, encoding="utf-8")


def main() -> None:
    args = parse_cli()
    out = args.out_dir.resolve()
    setup_style()
    runs = build_runs(workspace_root()) + build_expanded_runs(args.expanded_root)
    metrics, joined = compute_segment_metrics(runs)
    nis_summary, nis_series = compute_nis_metrics(runs)
    aggregate = aggregate_metrics(metrics)
    switch_inventory = write_switch_inventory(out)
    combined = build_combined_outputs(metrics, nis_summary)

    save_csv(metrics, out / "tables" / "route_segment_metrics.csv")
    save_csv(metrics.loc[metrics["stage"] == "T2"].copy(), out / "tables" / "t2_holdout_metrics.csv")
    save_csv(metrics.loc[metrics["stage"] == "repeatability"].copy(), out / "tables" / "repeatability_metrics.csv")
    save_csv(metrics.loc[metrics["stage"] == "expanded"].copy(), out / "tables" / "expanded_holdout_metrics.csv")
    save_csv(aggregate, out / "tables" / "aggregate_summary.csv")
    save_csv(nis_summary, out / "tables" / "nis_summary.csv")
    write_combined_tables(out, combined)
    if not nis_series.empty:
        keep_cols = [
            "stage",
            "paper_route",
            "route",
            "label",
            "armed_time_sec",
            "gnss_position_nis_h_2d",
            "chi2_2d_95",
            "exceeds_chi2_2d_95",
            "gnss_position_update_accepted",
        ]
        save_csv(nis_series[[c for c in keep_cols if c in nis_series.columns]], out / "tables" / "nis_timeseries.csv")

    labels = t2_labels(metrics)
    make_t2_trajectory_grid(joined, labels, out)
    make_t2_error_grid(joined, labels, out)
    make_metric_bars(metrics, out)
    make_straight_turn_bars(metrics, out)
    make_nis_grid(nis_series, labels, out)
    expanded_labels = stage_labels(metrics, "expanded")
    make_stage_trajectory_grid(joined, expanded_labels, out, "expanded", "Expanded clean holdout trajectory comparison")
    make_stage_error_grid(joined, expanded_labels, out, "expanded", "Expanded clean holdout horizontal error time series")
    make_stage_metric_bars(metrics, out, "expanded", "expanded", "Expanded clean holdout main-window error metrics")
    make_stage_straight_turn_bars(metrics, out, "expanded", "expanded", "Expanded clean holdout straight vs turn error")
    make_stage_nis_grid(nis_series, expanded_labels, out, "expanded", "Expanded clean holdout offline GNSS position NIS")
    write_readme(out, metrics, aggregate, nis_summary, combined)
    write_chapter_drafts(out, metrics, aggregate, nis_summary, combined)

    print(f"wrote package: {out}")
    print(f"runs: {len(runs)}")
    print(f"metric rows: {len(metrics)}")
    print(f"nis rows: {0 if nis_summary.empty else len(nis_summary)}")
    print(f"switch inventory rows: {len(switch_inventory)}")


if __name__ == "__main__":
    main()
