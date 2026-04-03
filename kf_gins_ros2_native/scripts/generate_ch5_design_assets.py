#!/usr/bin/env python3
"""Generate stable Chapter 5 design figures for the thesis."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from textwrap import fill

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle, FancyBboxPatch


def parse_args() -> argparse.Namespace:
    repo_root = Path(__file__).resolve().parents[1]
    return argparse.ArgumentParser(description=__doc__).parse_args(
        []
    )


def parse_cli() -> argparse.Namespace:
    repo_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        type=Path,
        default=repo_root / "config" / "ch5_paper_assets.json",
        help="Path to Chapter 5 paper asset config JSON.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=repo_root / "paper_outputs" / "ch5",
        help="Directory for generated figures.",
    )
    return parser.parse_args()


def load_config(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def setup_style() -> None:
    plt.rcParams.update(
        {
            "figure.dpi": 160,
            "savefig.dpi": 220,
            "font.size": 10,
            "axes.titlesize": 12,
            "axes.labelsize": 10,
            "legend.fontsize": 9,
            "xtick.labelsize": 9,
            "ytick.labelsize": 9,
            "axes.grid": True,
            "grid.alpha": 0.25,
        }
    )


def add_box(ax, x, y, w, h, title, body, facecolor, edgecolor="#243b53") -> None:
    box = FancyBboxPatch(
        (x, y),
        w,
        h,
        boxstyle="round,pad=0.02,rounding_size=0.03",
        linewidth=1.6,
        edgecolor=edgecolor,
        facecolor=facecolor,
    )
    ax.add_patch(box)
    ax.text(x + w / 2, y + h * 0.64, title, ha="center", va="center", weight="bold")
    ax.text(x + w / 2, y + h * 0.32, fill(body, 24), ha="center", va="center")


def add_arrow(ax, x0, y0, x1, y1, label="") -> None:
    ax.annotate(
        "",
        xy=(x1, y1),
        xytext=(x0, y0),
        arrowprops=dict(arrowstyle="->", lw=1.5, color="#243b53"),
    )
    if label:
        ax.text((x0 + x1) / 2, (y0 + y1) / 2 + 0.04, label, ha="center", va="center")


def draw_platform_diagram(output_dir: Path) -> Path:
    fig, ax = plt.subplots(figsize=(14, 4.6))
    ax.set_xlim(0.0, 1.0)
    ax.set_ylim(0.0, 1.0)
    ax.axis("off")

    boxes = [
        (0.03, 0.32, 0.16, 0.34, "PX4 SITL", "Controller, mission logic, actuator commands", "#d9eaf7"),
        (0.23, 0.32, 0.16, 0.34, "Gazebo", "Vehicle dynamics, environment, sensor truth", "#dff3e4"),
        (0.43, 0.32, 0.16, 0.34, "ROS 2 Bridge", "MAVROS relay, GNSS outage injection, time sync", "#fdf1d6"),
        (0.63, 0.32, 0.16, 0.34, "Filter Bank", "PX4 EKF2, KF-GINS EKF, IEKF, A-IEKF", "#f9dce1"),
        (0.83, 0.32, 0.14, 0.34, "Evaluator", "Bag recording, metrics, offline analysis", "#e6dcf5"),
    ]

    for x, y, w, h, title, body, face in boxes:
        add_box(ax, x, y, w, h, title, body, face)

    add_arrow(ax, 0.19, 0.49, 0.23, 0.49, "actuation")
    add_arrow(ax, 0.39, 0.49, 0.43, 0.49, "IMU / GNSS / pose")
    add_arrow(ax, 0.59, 0.49, 0.63, 0.49, "odometry")
    add_arrow(ax, 0.79, 0.49, 0.83, 0.49, "metrics / logs")

    ax.text(0.5, 0.9, "Figure 5-1  PX4-Gazebo-ROS2 integrated validation platform", ha="center", fontsize=13, weight="bold")
    ax.text(0.5, 0.14, "The figure is independent of filter tuning details and remains valid while the estimation code evolves.", ha="center")

    output = output_dir / "fig5_1_platform_diagram.png"
    fig.savefig(output, bbox_inches="tight")
    plt.close(fig)
    return output


def draw_trajectory_schematics(output_dir: Path) -> Path:
    fig, axes = plt.subplots(1, 3, figsize=(14, 4.6), constrained_layout=True)
    titles = ["Line trajectory", "Circular trajectory", "Agile trajectory"]

    for ax, title in zip(axes, titles):
        ax.set_aspect("equal")
        ax.set_xlim(0, 100)
        ax.set_ylim(0, 100)
        ax.set_xlabel("x / m")
        ax.set_ylabel("y / m")
        ax.set_title(title)
        outage = Circle((62, 52), 14, facecolor="#ffb4a2", edgecolor="#b00020", alpha=0.35)
        ax.add_patch(outage)
        ax.text(62, 70, "denied zone", ha="center", color="#7f1d1d")

    x = np.linspace(8, 92, 240)
    y = np.linspace(18, 82, 240)
    axes[0].plot(x, y, color="#1d4ed8", lw=2.2)
    axes[0].scatter([x[0], x[-1]], [y[0], y[-1]], color=["#059669", "#b91c1c"], s=45, zorder=3)
    axes[0].text(x[0], y[0] - 7, "start", ha="center")
    axes[0].text(x[-1], y[-1] + 6, "end", ha="center")

    theta = np.linspace(0, 2 * np.pi, 320)
    axes[1].plot(50 + 28 * np.cos(theta), 50 + 28 * np.sin(theta), color="#7c3aed", lw=2.2)
    axes[1].scatter([78], [50], color="#059669", s=45, zorder=3)
    axes[1].text(79, 45, "start", ha="left")

    t = np.linspace(0, 1, 420)
    agile_x = 8 + 84 * t
    agile_y = 50 + 22 * np.sin(2.8 * np.pi * t) + 12 * np.sin(7.2 * np.pi * t)
    axes[2].plot(agile_x, agile_y, color="#ea580c", lw=2.2)
    axes[2].scatter([agile_x[0], agile_x[-1]], [agile_y[0], agile_y[-1]], color=["#059669", "#b91c1c"], s=45, zorder=3)
    axes[2].text(agile_x[0], agile_y[0] - 8, "start", ha="center")
    axes[2].text(agile_x[-1], agile_y[-1] + 6, "end", ha="center")

    fig.suptitle("Figure 5-5  Planned trajectory families used for Chapter 5 experiments", fontsize=13, weight="bold")
    output = output_dir / "fig5_5_trajectory_schematics.png"
    fig.savefig(output, bbox_inches="tight")
    plt.close(fig)
    return output


def draw_metric_schematic(output_dir: Path) -> Path:
    ts = 20.0
    te = 40.0
    threshold = 1.0
    t = np.linspace(0.0, 80.0, 800)
    err = np.empty_like(t)

    before = t < ts
    outage = (t >= ts) & (t <= te)
    recovery = t > te

    err[before] = 0.18 + 0.03 * np.sin(0.4 * t[before])
    err[outage] = 0.28 + 0.065 * (t[outage] - ts) + 0.09 * np.sin(0.55 * (t[outage] - ts))
    err[recovery] = 0.22 + 1.32 * np.exp(-(t[recovery] - te) / 7.5) + 0.04 * np.sin(0.7 * (t[recovery] - te))

    outage_idx = np.where(outage)[0]
    emax_idx = outage_idx[np.argmax(err[outage_idx])]
    emax_t = float(t[emax_idx])
    emax_v = float(err[emax_idx])

    recovery_idx = np.where((t > te) & (err <= threshold))[0]
    trec_t = float(t[recovery_idx[0]]) if recovery_idx.size else float(t[-1])

    fig, ax = plt.subplots(figsize=(10.8, 4.8))
    ax.plot(t, err, color="#1d4ed8", lw=2.2, label="position error")
    ax.axvspan(ts, te, color="#fecaca", alpha=0.5, label="GNSS denied interval")
    ax.axhline(threshold, color="#b45309", ls="--", lw=1.5, label="recovery threshold")
    ax.axvline(ts, color="#374151", ls=":", lw=1.2)
    ax.axvline(te, color="#374151", ls=":", lw=1.2)
    ax.scatter([emax_t], [emax_v], color="#b91c1c", s=50, zorder=3)
    ax.scatter([trec_t], [threshold], color="#059669", s=50, zorder=3)

    ax.annotate("t_s", xy=(ts, 0.05), xytext=(ts - 3, 0.36), arrowprops=dict(arrowstyle="->", lw=1.0))
    ax.annotate("t_e", xy=(te, 0.05), xytext=(te - 3, 0.36), arrowprops=dict(arrowstyle="->", lw=1.0))
    ax.annotate("e_max", xy=(emax_t, emax_v), xytext=(emax_t - 9, emax_v + 0.35), arrowprops=dict(arrowstyle="->", lw=1.0))
    ax.annotate("T_rec", xy=(trec_t, threshold), xytext=(trec_t + 4, threshold + 0.32), arrowprops=dict(arrowstyle="->", lw=1.0))

    ax.set_xlim(0, 80)
    ax.set_ylim(0, max(1.85, emax_v + 0.25))
    ax.set_xlabel("time / s")
    ax.set_ylabel("position error / m")
    ax.set_title("Figure 5-6  Schematic definition of outage-stage evaluation metrics")
    ax.legend(loc="upper right")

    output = output_dir / "fig5_6_outage_metric_schematic.png"
    fig.savefig(output, bbox_inches="tight")
    plt.close(fig)
    return output


def main() -> int:
    args = parse_cli()
    load_config(args.config)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    setup_style()

    outputs = [
        draw_platform_diagram(args.output_dir),
        draw_trajectory_schematics(args.output_dir),
        draw_metric_schematic(args.output_dir),
    ]
    for output in outputs:
        print(f"generated: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
