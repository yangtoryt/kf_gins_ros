#!/usr/bin/env python3
"""Offline feasibility check for non-projection GNSS correction gain.

This is not a closed-loop estimator replay.  It asks a narrower question:
given the accepted GNSS residual/update vectors already logged in PHS5, could
an extra first-order correction along latest residual/dx/unresolved direction
repair shortgen11 140-180s without damaging the positive routes?
"""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence


BASE = Path("/home/yang/kf_gins_ws/artifacts/manual/short_route_generalization_2026-05-07")
DEFAULT_INPUT = BASE / "phs5_history_proxy_replay_2026-05-10" / "history_proxy_row_observables.csv"
DEFAULT_OUT = BASE / "phs5_correction_gain_feasibility_2026-05-10"

POSITIVE_RUNS = [
    "shortgen01_phs5c",
    "shortgen02_phs5b",
    "shortgen03_phs5a",
    "shortgen04_hld1a_phs5",
]
TARGET_RUNS = [
    "shortgen11_repeat2",
    "shortgen11_seg2lagdiag_174924",
    "shortgen11_seg2lagproj025_182101",
    "shortgen11_seg2lagproj025_183529",
]
WINDOWS = {
    "main_40_180": (40.0, 180.0),
    "140_160": (140.0, 160.0),
    "160_180": (160.0, 180.0),
}


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


def norm(east: float, north: float) -> float:
    return math.hypot(east, north) if finite(east) and finite(north) else math.nan


def vec_from_along_cross(along: float, cross: float, unit_e: float, unit_n: float) -> tuple[float, float]:
    if not all(finite(value) for value in (along, cross, unit_e, unit_n)):
        return math.nan, math.nan
    return along * unit_e + cross * unit_n, along * unit_n - cross * unit_e


def load_rows(path: Path) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for raw in read_csv(path):
        row: dict[str, object] = dict(raw)
        for key in raw:
            if key not in {"run", "role", "mavros_mode", "publish_projection_action", "group"}:
                row[key] = to_float(raw.get(key))
        speed = norm(to_float(row.get("core_velocity_e_mps")), to_float(row.get("core_velocity_n_mps")))
        if finite(speed) and speed > 0.25:
            unit_e = to_float(row.get("core_velocity_e_mps")) / speed
            unit_n = to_float(row.get("core_velocity_n_mps")) / speed
        else:
            unit_e = math.nan
            unit_n = math.nan
        res_e, res_n = vec_from_along_cross(
            to_float(row.get("latest_gnss_residual_along_velocity_m")),
            to_float(row.get("latest_gnss_residual_cross_velocity_m")),
            unit_e,
            unit_n,
        )
        dx_e, dx_n = vec_from_along_cross(
            to_float(row.get("latest_dx_pos_along_velocity_m")),
            to_float(row.get("latest_dx_pos_cross_velocity_m")),
            unit_e,
            unit_n,
        )
        row["latest_residual_e_m"] = res_e
        row["latest_residual_n_m"] = res_n
        row["latest_dx_e_m"] = dx_e
        row["latest_dx_n_m"] = dx_n
        row["latest_unresolved_e_m"] = res_e - dx_e if finite(res_e) and finite(dx_e) else math.nan
        row["latest_unresolved_n_m"] = res_n - dx_n if finite(res_n) and finite(dx_n) else math.nan
        rows.append(row)
    rows.sort(key=lambda row: (str(row.get("run")), to_float(row.get("time_since_arm_sec"))))
    return rows


@dataclass(frozen=True)
class Gate:
    name: str
    field: str | None = None
    op: str = "ge"
    threshold: float = math.nan
    second_field: str | None = None
    second_op: str = "ge"
    second_threshold: float = math.nan

    def active(self, row: dict[str, object]) -> bool:
        if self.field is None:
            return True
        value = to_float(row.get(self.field))
        if not finite(value):
            return False
        if self.op == "ge":
            ok = value >= self.threshold
        elif self.op == "le":
            ok = value <= self.threshold
        else:
            raise ValueError(self.op)
        if not ok or self.second_field is None:
            return ok
        value2 = to_float(row.get(self.second_field))
        if not finite(value2):
            return False
        if self.second_op == "ge":
            return value2 >= self.second_threshold
        if self.second_op == "le":
            return value2 <= self.second_threshold
        raise ValueError(self.second_op)


@dataclass(frozen=True)
class Candidate:
    direction: str
    scale: float
    gate: Gate

    @property
    def name(self) -> str:
        return f"{self.direction}_scale_{self.scale:.2f}_{self.gate.name}"

    def action(self, row: dict[str, object]) -> tuple[float, float]:
        if not self.gate.active(row):
            return 0.0, 0.0
        if self.direction == "residual":
            east = to_float(row.get("latest_residual_e_m"))
            north = to_float(row.get("latest_residual_n_m"))
        elif self.direction == "dx":
            east = to_float(row.get("latest_dx_e_m"))
            north = to_float(row.get("latest_dx_n_m"))
        elif self.direction == "unresolved":
            east = to_float(row.get("latest_unresolved_e_m"))
            north = to_float(row.get("latest_unresolved_n_m"))
        else:
            raise ValueError(self.direction)
        if not finite(east) or not finite(north):
            return 0.0, 0.0
        return self.scale * east, self.scale * north


def build_candidates() -> list[Candidate]:
    gates = [
        Gate("always"),
        Gate("ewma_unresolved20_ge_0p05", "ewma_unresolved_20s_h_m", "ge", 0.05),
        Gate("ewma_unresolved20_ge_0p08", "ewma_unresolved_20s_h_m", "ge", 0.08),
        Gate("hist_resmean30_ge_0p08", "hist_residual_mean_30s_h_m", "ge", 0.08),
        Gate("hist_resmean30_ge_0p10", "hist_residual_mean_30s_h_m", "ge", 0.10),
        Gate("weakgain30_ge_0p45", "hist_weak_gain_frac_30s", "ge", 0.45),
        Gate(
            "resmean30_ge_0p08_gain30_le_0p30",
            "hist_residual_mean_30s_h_m",
            "ge",
            0.08,
            "hist_dx_over_residual_mean_30s",
            "le",
            0.30,
        ),
        Gate(
            "ewmaunres20_ge_0p05_gain30_le_0p30",
            "ewma_unresolved_20s_h_m",
            "ge",
            0.05,
            "hist_dx_over_residual_mean_30s",
            "le",
            0.30,
        ),
    ]
    scales = (-0.50, -0.25, 0.25, 0.50, 0.75, 1.00, 1.50, 2.00)
    return [Candidate(direction, scale, gate) for direction in ("residual", "dx", "unresolved") for scale in scales for gate in gates]


def in_window(row: dict[str, object], window: str) -> bool:
    start, end = WINDOWS[window]
    t = to_float(row.get("time_since_arm_sec"))
    return start <= t < end


def aggregate(rows: Sequence[dict[str, object]], candidate: Candidate) -> dict[str, object]:
    if not rows:
        return {
            "rows": 0,
            "active_frac": math.nan,
            "raw_rmse_m": math.nan,
            "candidate_rmse_m": math.nan,
            "ekf2_rmse_m": math.nan,
            "candidate_minus_raw_m": math.nan,
            "candidate_minus_ekf2_m": math.nan,
        }
    raw_sse = 0.0
    cand_sse = 0.0
    ekf2_sse = 0.0
    active = 0
    for row in rows:
        raw_x = to_float(row.get("raw_iekf_x_m"))
        raw_y = to_float(row.get("raw_iekf_y_m"))
        gt_x = to_float(row.get("gt_x_m"))
        gt_y = to_float(row.get("gt_y_m"))
        act_e, act_n = candidate.action(row)
        if abs(act_e) > 0.0 or abs(act_n) > 0.0:
            active += 1
        raw_err = norm(raw_x - gt_x, raw_y - gt_y)
        cand_err = norm(raw_x + act_e - gt_x, raw_y + act_n - gt_y)
        ekf2_err = to_float(row.get("ekf2_error_m"))
        raw_sse += raw_err * raw_err
        cand_sse += cand_err * cand_err
        ekf2_sse += ekf2_err * ekf2_err
    n = len(rows)
    raw_rmse = math.sqrt(raw_sse / n)
    cand_rmse = math.sqrt(cand_sse / n)
    ekf2_rmse = math.sqrt(ekf2_sse / n)
    return {
        "rows": n,
        "active_frac": active / n,
        "raw_rmse_m": raw_rmse,
        "candidate_rmse_m": cand_rmse,
        "ekf2_rmse_m": ekf2_rmse,
        "candidate_minus_raw_m": cand_rmse - raw_rmse,
        "candidate_minus_ekf2_m": cand_rmse - ekf2_rmse,
    }


def score(rows: Sequence[dict[str, object]], candidate: Candidate) -> dict[str, object]:
    pos_main = [
        aggregate([row for row in rows if row.get("run") == run and in_window(row, "main_40_180")], candidate)
        for run in POSITIVE_RUNS
    ]
    target_main = [
        aggregate([row for row in rows if row.get("run") == run and in_window(row, "main_40_180")], candidate)
        for run in TARGET_RUNS
    ]
    target_140 = [
        aggregate([row for row in rows if row.get("run") == run and in_window(row, "140_160")], candidate)
        for run in TARGET_RUNS
    ]
    target_160 = [
        aggregate([row for row in rows if row.get("run") == run and in_window(row, "160_180")], candidate)
        for run in TARGET_RUNS
    ]
    pos_reg = [to_float(row.get("candidate_minus_raw_m")) for row in pos_main]
    pos_gap = [to_float(row.get("candidate_minus_ekf2_m")) for row in pos_main]
    target_140_gap = [to_float(row.get("candidate_minus_ekf2_m")) for row in target_140]
    target_160_gap = [to_float(row.get("candidate_minus_ekf2_m")) for row in target_160]
    protected = max(pos_reg) <= 0.02 and max(pos_gap) < 0.02
    return {
        "candidate": candidate.name,
        "direction": candidate.direction,
        "scale": candidate.scale,
        "gate": candidate.gate.name,
        "positive_max_main_regress_m": max(pos_reg),
        "positive_max_candidate_minus_ekf2_m": max(pos_gap),
        "positive_all_protected": int(protected),
        "target_worst_main_candidate_minus_ekf2_m": max(to_float(row.get("candidate_minus_ekf2_m")) for row in target_main),
        "target_worst_140_160_candidate_minus_ekf2_m": max(target_140_gap),
        "target_worst_160_180_candidate_minus_ekf2_m": max(target_160_gap),
        "target_mean_140_160_improve_m": mean(
            to_float(row.get("raw_rmse_m")) - to_float(row.get("candidate_rmse_m")) for row in target_140
        ),
        "target_mean_160_180_improve_m": mean(
            to_float(row.get("raw_rmse_m")) - to_float(row.get("candidate_rmse_m")) for row in target_160
        ),
        "target_140_160_pass": int(max(target_140_gap) <= 0.02),
        "target_160_180_pass": int(max(target_160_gap) <= 0.02),
        "strict_pass": int(protected and max(target_140_gap) <= 0.02 and max(target_160_gap) <= 0.02),
    }


def sort_key(row: dict[str, object]) -> tuple[float, float, float, float]:
    return (
        0.0 if to_float(row.get("positive_all_protected")) > 0.5 else 1.0,
        to_float(row.get("target_worst_140_160_candidate_minus_ekf2_m")),
        to_float(row.get("target_worst_160_180_candidate_minus_ekf2_m")),
        to_float(row.get("positive_max_main_regress_m")),
    )


def candidate_table(rows: Sequence[dict[str, object]]) -> list[list[object]]:
    return [
        [
            row["direction"],
            fmt(row["scale"], 2),
            row["gate"],
            row["positive_all_protected"],
            fmt(row["positive_max_main_regress_m"]),
            fmt(row["target_worst_140_160_candidate_minus_ekf2_m"]),
            fmt(row["target_worst_160_180_candidate_minus_ekf2_m"]),
            fmt(row["target_mean_140_160_improve_m"]),
            row["strict_pass"],
        ]
        for row in rows
    ]


def write_report(out_dir: Path, rows: list[dict[str, object]]) -> None:
    strict = [row for row in rows if to_float(row.get("strict_pass")) > 0.5]
    protected = [row for row in rows if to_float(row.get("positive_all_protected")) > 0.5]
    strict.sort(key=sort_key)
    protected.sort(key=sort_key)
    relaxed = sorted(
        rows,
        key=lambda row: (
            to_float(row.get("target_worst_140_160_candidate_minus_ekf2_m")),
            to_float(row.get("target_worst_160_180_candidate_minus_ekf2_m")),
            to_float(row.get("positive_max_main_regress_m")),
        ),
    )
    lines = [
        "# PHS5 correction-gain feasibility",
        "",
        "Date: 2026-05-10",
        "",
        "Offline first-order feasibility check from existing PHS5 logs. It does not prove closed-loop behavior and starts no flight stack.",
        "",
        f"candidates tested: `{len(rows)}`",
        f"strict-pass count: `{len(strict)}`",
        f"positive-protected count: `{len(protected)}`",
        "",
        "## Top Protected Candidates",
        "",
        markdown_table(
            ["direction", "scale", "gate", "protected", "pos regress", "worst 140-EKF2", "worst 160-EKF2", "mean 140 improve", "strict"],
            candidate_table(protected[:12]),
        ),
        "",
        "## Top Relaxed Candidates",
        "",
        markdown_table(
            ["direction", "scale", "gate", "protected", "pos regress", "worst 140-EKF2", "worst 160-EKF2", "mean 140 improve", "strict"],
            candidate_table(relaxed[:12]),
        ),
        "",
        "## Readout",
        "",
    ]
    if strict:
        best = strict[0]
        lines.append(f"- A strict first-order correction candidate exists: `{best['candidate']}`.")
    else:
        lines.append("- No tested first-order correction-gain candidate passes both positive protection and shortgen11 target repair.")
    lines.extend(
        [
            "- This is a feasibility screen only; any positive result would still need a real offline closed-loop replay or opt-in mechanism run.",
            "",
            "Generated files:",
            f"- `{out_dir / 'correction_gain_candidate_summary.csv'}`",
        ]
    )
    (out_dir / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", default=str(DEFAULT_INPUT))
    parser.add_argument("--out-dir", default=str(DEFAULT_OUT))
    args = parser.parse_args()

    rows = load_rows(Path(args.input))
    out_dir = Path(args.out_dir)
    scored = [score(rows, candidate) for candidate in build_candidates()]
    write_csv(out_dir / "correction_gain_candidate_summary.csv", scored)
    write_report(out_dir, scored)
    print(f"wrote: {out_dir / 'report.md'}")


if __name__ == "__main__":
    main()
