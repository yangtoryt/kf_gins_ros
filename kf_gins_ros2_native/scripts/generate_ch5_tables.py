#!/usr/bin/env python3
"""Generate Chapter 5 table templates from a single JSON config."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import pandas as pd


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
        help="Directory for generated tables.",
    )
    return parser.parse_args()


def load_config(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def dataframe_to_markdown(df: pd.DataFrame) -> str:
    columns = list(df.columns)
    rows = [columns] + [[str(row[col]) for col in columns] for _, row in df.iterrows()]
    widths = [max(len(str(row[i])) for row in rows) for i in range(len(columns))]

    def fmt_row(row: list[str]) -> str:
        return "| " + " | ".join(str(cell).ljust(widths[i]) for i, cell in enumerate(row)) + " |"

    sep = "| " + " | ".join("-" * widths[i] for i in range(len(columns))) + " |"
    lines = [fmt_row(columns), sep]
    for _, row in df.iterrows():
        lines.append(fmt_row([str(row[col]) for col in columns]))
    return "\n".join(lines) + "\n"


def write_table(df: pd.DataFrame, stem: str, output_dir: Path) -> None:
    csv_path = output_dir / f"{stem}.csv"
    md_path = output_dir / f"{stem}.md"
    df.to_csv(csv_path, index=False, encoding="utf-8-sig")
    md_path.write_text(dataframe_to_markdown(df), encoding="utf-8")
    print(f"generated: {csv_path}")
    print(f"generated: {md_path}")


def build_scene_table(config: dict) -> pd.DataFrame:
    return pd.DataFrame(
        [
            {
                "场景名称": item["scene_name"],
                "世界类型": item["world_type"],
                "建筑密度": item["building_density"],
                "拒止区设置": item["denied_zone_setup"],
                "拒止区数量": item["denied_zone_count"],
                "主要用途": item["purpose"],
            }
            for item in config["scenes"]
        ]
    )


def build_algorithm_table(config: dict) -> pd.DataFrame:
    return pd.DataFrame(
        [
            {
                "算法名称": item["algorithm_name"],
                "是否迭代更新": item["iterative_update"],
                "是否自适应R": item["adaptive_r"],
                "是否有边界保护": item["bounded_r"],
                "说明": item["description"],
            }
            for item in config["algorithms"]
        ]
    )


def build_trajectory_table(config: dict) -> pd.DataFrame:
    return pd.DataFrame(
        [
            {
                "轨迹名称": item["trajectory_name"],
                "平均速度(m/s)": item["avg_speed_mps"],
                "最大速度(m/s)": item["max_speed_mps"],
                "最大转弯率(deg/s)": item["max_turn_rate_dps"],
                "是否穿越拒止区": item["crosses_denied_zone"],
                "主要目的": item["purpose"],
            }
            for item in config["trajectories"]
        ]
    )


def build_metric_table(config: dict) -> pd.DataFrame:
    return pd.DataFrame(
        [
            {
                "指标名称": item["metric_name"],
                "符号": item["symbol"],
                "反映维度": item["dimension"],
                "使用场景": item["usage"],
            }
            for item in config["metrics"]
        ]
    )


def main() -> int:
    args = parse_cli()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    config = load_config(args.config)

    write_table(build_scene_table(config), "table5_1_scene_config", args.output_dir)
    write_table(build_algorithm_table(config), "table5_2_algorithm_ablation", args.output_dir)
    write_table(build_trajectory_table(config), "table5_3_trajectory_params", args.output_dir)
    write_table(build_metric_table(config), "table5_5_metric_definitions", args.output_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
