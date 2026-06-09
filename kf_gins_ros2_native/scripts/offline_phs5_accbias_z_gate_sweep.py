#!/usr/bin/env python3
"""Offline sweep for accbias-z propagation-probe gates.

The sweep estimates selector coverage and Q-exposure from existing
state-update logs. It does not replay the estimator or claim a performance
counterfactual.
"""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import pandas as pd


BASE = Path("/home/yang/kf_gins_ws/artifacts/manual/short_route_generalization_2026-05-07")
DEFAULT_OUT = BASE / "phs5_accbias_z_gate_sweep_2026-05-11"

RUN_DIRS = {
    "shortgen01_phs5c": Path(
        "/home/yang/kf_gins_ws/artifacts/manual/"
        "manual_shortgen01_s_bend_sitl_home_headless_compare_core8_pairlogger_phs5c_publish_core_stamp_bias_m003_20260510_003839"
    ),
    "shortgen02_phs5b": Path(
        "/home/yang/kf_gins_ws/artifacts/manual/"
        "manual_shortgen02_box_uturn_sitl_home_headless_compare_core8_pairlogger_phs5b_publish_core_stamp_bias_m003_20260510_003144"
    ),
    "shortgen03_phs5a": Path(
        "/home/yang/kf_gins_ws/artifacts/manual/"
        "manual_shortgen03_figure8_sitl_home_headless_compare_core8_pairlogger_phs5a_publish_core_stamp_bias_m003_20260510_002551"
    ),
    "shortgen04_hld1a_phs5": Path(
        "/home/yang/kf_gins_ws/artifacts/manual/"
        "manual_shortgen04_ladder_sitl_home_headless_compare_core8_pairlogger_hld1a_phs5_publish_core_stamp_bias_m003_20260510_005528"
    ),
    "shortgen11_repeat2": Path(
        "/home/yang/kf_gins_ws/artifacts/manual/"
        "manual_shortgen11_high_clearance_ladder_sitl_home_headless_compare_core8_pairlogger_phs5_repeat2_cleancheck_20260510_155253"
    ),
    "shortgen11_accbiasz_diag": Path(
        "/home/yang/kf_gins_ws/artifacts/manual/"
        "manual_shortgen11_high_clearance_ladder_sitl_home_headless_compare_core8_pairlogger_phs5_accbiasz_diag_futureclamp_20260511_111411"
    ),
    "shortgen11_accbiasz_apply": Path(
        "/home/yang/kf_gins_ws/artifacts/manual/"
        "manual_shortgen11_high_clearance_ladder_sitl_home_headless_compare_core8_pairlogger_phs5_accbiasz_apply_futureclamp_20260511_112407"
    ),
    "shortgen04_futureclamp": Path(
        "/home/yang/kf_gins_ws/artifacts/manual/"
        "manual_shortgen04_ladder_sitl_home_headless_compare_core8_pairlogger_phs5_coretrace_futureclamp_20260510_231317"
    ),
}

FORMAL_PROTECTION_RUNS = {
    "shortgen01_phs5c",
    "shortgen02_phs5b",
    "shortgen03_phs5a",
    "shortgen04_hld1a_phs5",
}
FORMAL_TARGET_RUN = "shortgen11_repeat2"
ONLINE_DIAGNOSTIC_RUN = "shortgen11_accbiasz_diag"

WINDOWS = {
    "40-180": (40.0, 180.0),
    "80-100": (80.0, 100.0),
    "100-120": (100.0, 120.0),
    "120-140": (120.0, 140.0),
    "140-160": (140.0, 160.0),
    "160-180": (160.0, 180.0),
    "120-160": (120.0, 160.0),
    "120-180": (120.0, 180.0),
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
    value_f = to_float(value)
    return f"{value_f:.{digits}f}" if math.isfinite(value_f) else "nan"


def mean(values: Iterable[float]) -> float:
    vals = [value for value in values if finite(value)]
    return sum(vals) / len(vals) if vals else math.nan


def markdown_table(headers: list[str], rows: list[list[object]]) -> str:
    lines = [
        "| " + " | ".join(headers) + " |",
        "| " + " | ".join("---" for _ in headers) + " |",
    ]
    lines.extend("| " + " | ".join(str(item) for item in row) + " |" for row in rows)
    return "\n".join(lines)


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


def numeric(frame: pd.DataFrame, column: str) -> pd.Series:
    if column not in frame:
        return pd.Series([math.nan] * len(frame), index=frame.index, dtype="float64")
    return pd.to_numeric(frame[column], errors="coerce")


def truthy_series(frame: pd.DataFrame, column: str) -> pd.Series:
    if column not in frame:
        return pd.Series([False] * len(frame), index=frame.index, dtype="bool")
    return frame[column].astype(str).str.strip().str.lower().isin({"1", "true", "yes"})


def load_state_updates(run_dirs: dict[str, Path]) -> pd.DataFrame:
    frames: list[pd.DataFrame] = []
    for run, run_dir in run_dirs.items():
        path = run_dir / "state_update_debug.csv"
        if not path.exists():
            continue
        frame = pd.read_csv(path)
        if "event_type" in frame:
            frame = frame[frame["event_type"] == "gnss_position"].copy()
        if "applied" in frame:
            frame = frame[pd.to_numeric(frame["applied"], errors="coerce") == 1].copy()
        frame["run"] = run
        frame["time_since_arm_sec"] = numeric(frame, "armed_time_sec")
        frame["accbias_z_mps2"] = numeric(frame, "accbias_z_before_mps2")
        frame["horizontal_speed_mps"] = numeric(frame, "horizontal_speed_mps")
        frame["vertical_speed_mps"] = numeric(frame, "vertical_speed_mps")
        frame["gyro_deg_s"] = numeric(frame, "gyro_deg_s").abs()
        frame["mavros_armed_bool"] = truthy_series(frame, "mavros_armed")
        frame["mission_bool"] = frame.get("mavros_mode", "").astype(str).eq("AUTO.MISSION")
        frame["turning_bool"] = truthy_series(frame, "turning_now")
        frame["post_turn_bool"] = truthy_series(frame, "post_turn_context")
        frame["armed_cruise_bool"] = truthy_series(frame, "armed_cruise_context")
        frames.append(frame)
    if not frames:
        return pd.DataFrame()
    return pd.concat(frames, ignore_index=True)


@dataclass(frozen=True)
class Candidate:
    trigger_mps2: float
    full_mps2: float
    min_hspeed_mps: float
    max_abs_vspeed_mps: float
    context_gate: str
    vrw_q_scale_max: float
    accbias_q_scale_max: float

    @property
    def name(self) -> str:
        return (
            f"trig{self.trigger_mps2:.3f}_full{self.full_mps2:.3f}_"
            f"h{self.min_hspeed_mps:.1f}_v{self.max_abs_vspeed_mps:.1f}_"
            f"{self.context_gate}_vrw{self.vrw_q_scale_max:.1f}_ba{self.accbias_q_scale_max:.1f}"
        )


def context_mask(rows: pd.DataFrame, gate: str) -> pd.Series:
    base = pd.Series([True] * len(rows), index=rows.index)
    if gate == "all":
        return base
    if gate == "cruise_only":
        return truthy_series(rows, "armed_cruise_bool")
    if gate == "not_turning":
        return ~truthy_series(rows, "turning_bool")
    if gate == "not_turn_or_post":
        return (~truthy_series(rows, "turning_bool")) & (~truthy_series(rows, "post_turn_bool"))
    if gate == "cruise_or_post":
        return truthy_series(rows, "armed_cruise_bool") | truthy_series(rows, "post_turn_bool")
    raise ValueError(gate)


def ramp_score(values: pd.Series, trigger_mps2: float, full_mps2: float) -> pd.Series:
    trigger_mag = abs(trigger_mps2)
    full_mag = max(trigger_mag + 1e-9, abs(full_mps2))
    negative_bias_mag = -pd.to_numeric(values, errors="coerce")
    score = (negative_bias_mag - trigger_mag) / (full_mag - trigger_mag)
    return score.clip(lower=0.0, upper=1.0).fillna(0.0)


def candidate_frame(rows: pd.DataFrame, candidate: Candidate) -> pd.DataFrame:
    out = rows.copy()
    phase_ok = out["mavros_armed_bool"] & out["mission_bool"]
    motion_ok = (
        (out["horizontal_speed_mps"] >= candidate.min_hspeed_mps)
        & (out["vertical_speed_mps"].abs() <= candidate.max_abs_vspeed_mps)
    )
    ctx_ok = context_mask(out, candidate.context_gate)
    score = ramp_score(out["accbias_z_mps2"], candidate.trigger_mps2, candidate.full_mps2)
    active = phase_ok & motion_ok & ctx_ok & (score > 1e-6)
    out["gate_active"] = active.astype(float)
    out["gate_score"] = score.where(active, 0.0)
    out["vrw_q_scale"] = 1.0 + out["gate_score"] * (candidate.vrw_q_scale_max - 1.0)
    out["accbias_q_scale"] = 1.0 + out["gate_score"] * (candidate.accbias_q_scale_max - 1.0)
    out["q_exposure"] = (out["vrw_q_scale"] - 1.0) + (out["accbias_q_scale"] - 1.0)
    return out


def window_rows(rows: pd.DataFrame, run: str, window: str) -> pd.DataFrame:
    start, end = WINDOWS[window]
    subset = rows[rows["run"] == run]
    t = pd.to_numeric(subset["time_since_arm_sec"], errors="coerce")
    return subset[(t >= start) & (t < end)]


def summarize_subset(rows: pd.DataFrame) -> dict[str, float]:
    if rows.empty:
        return {
            "rows": 0,
            "active_frac": math.nan,
            "score_mean_all": math.nan,
            "score_mean_active": math.nan,
            "q_exposure_mean": math.nan,
            "q_exposure_p95": math.nan,
            "accbias_z_mean": math.nan,
        }
    active = rows["gate_active"] > 0.5
    exposure = pd.to_numeric(rows["q_exposure"], errors="coerce")
    return {
        "rows": float(len(rows)),
        "active_frac": float(active.mean()),
        "score_mean_all": float(pd.to_numeric(rows["gate_score"], errors="coerce").mean()),
        "score_mean_active": float(pd.to_numeric(rows.loc[active, "gate_score"], errors="coerce").mean()) if active.any() else 0.0,
        "q_exposure_mean": float(exposure.mean()),
        "q_exposure_p95": float(exposure.quantile(0.95)),
        "accbias_z_mean": float(pd.to_numeric(rows["accbias_z_mps2"], errors="coerce").mean()),
    }


def build_candidates() -> list[Candidate]:
    triggers = [-0.18, -0.20, -0.205, -0.21, -0.215, -0.22]
    full_offsets = [0.03, 0.04, 0.06]
    hmins = [3.0, 4.0, 4.5]
    vmaxs = [0.3, 0.5, 1.0]
    context_gates = ["all", "not_turning", "cruise_only", "cruise_or_post"]
    scale_pairs = [(2.0, 3.0), (3.0, 6.0)]
    out: list[Candidate] = []
    for trigger in triggers:
        for offset in full_offsets:
            full = trigger - offset
            for hmin in hmins:
                for vmax in vmaxs:
                    for gate in context_gates:
                        for vrw, ba in scale_pairs:
                            out.append(Candidate(trigger, full, hmin, vmax, gate, vrw, ba))
    return out


def score_candidate(rows: pd.DataFrame, candidate: Candidate) -> dict[str, object]:
    framed = candidate_frame(rows, candidate)

    target_140 = summarize_subset(window_rows(framed, FORMAL_TARGET_RUN, "140-160"))
    target_120_160 = summarize_subset(window_rows(framed, FORMAL_TARGET_RUN, "120-160"))
    target_100_120 = summarize_subset(window_rows(framed, FORMAL_TARGET_RUN, "100-120"))
    target_160_180 = summarize_subset(window_rows(framed, FORMAL_TARGET_RUN, "160-180"))
    online_diag_140 = summarize_subset(window_rows(framed, ONLINE_DIAGNOSTIC_RUN, "140-160"))
    online_diag_160 = summarize_subset(window_rows(framed, ONLINE_DIAGNOSTIC_RUN, "160-180"))

    protection_summaries = {
        run: summarize_subset(window_rows(framed, run, "40-180"))
        for run in FORMAL_PROTECTION_RUNS
    }
    protection_active_vals = [
        to_float(summary["active_frac"])
        for summary in protection_summaries.values()
        if finite(summary["active_frac"])
    ]
    protection_exposure_vals = [
        to_float(summary["q_exposure_mean"])
        for summary in protection_summaries.values()
        if finite(summary["q_exposure_mean"])
    ]
    protection_max_active = max(protection_active_vals) if protection_active_vals else math.nan
    protection_mean_active = mean(protection_active_vals)
    protection_max_exposure = max(protection_exposure_vals) if protection_exposure_vals else math.nan

    # This is a selector-risk score, not a performance prediction.
    objective = (
        2.5 * target_140["active_frac"]
        + 1.2 * online_diag_140["active_frac"]
        + 1.0 * target_140["score_mean_all"]
        - 2.0 * target_100_120["active_frac"]
        - 2.5 * target_160_180["active_frac"]
        - 3.0 * online_diag_160["active_frac"]
        - 3.0 * protection_max_active
        - 0.45 * protection_max_exposure
    )
    strict = (
        finite(target_140["active_frac"])
        and target_140["active_frac"] >= 0.40
        and finite(online_diag_140["active_frac"])
        and online_diag_140["active_frac"] >= 0.45
        and finite(target_100_120["active_frac"])
        and target_100_120["active_frac"] <= 0.35
        and finite(target_160_180["active_frac"])
        and target_160_180["active_frac"] <= 0.15
        and finite(online_diag_160["active_frac"])
        and online_diag_160["active_frac"] <= 0.20
        and finite(protection_max_active)
        and protection_max_active <= 0.10
    )
    return {
        "candidate": candidate.name,
        "trigger_mps2": candidate.trigger_mps2,
        "full_mps2": candidate.full_mps2,
        "min_hspeed_mps": candidate.min_hspeed_mps,
        "max_abs_vspeed_mps": candidate.max_abs_vspeed_mps,
        "context_gate": candidate.context_gate,
        "vrw_q_scale_max": candidate.vrw_q_scale_max,
        "accbias_q_scale_max": candidate.accbias_q_scale_max,
        "objective": objective,
        "strict": int(strict),
        "target_140_active": target_140["active_frac"],
        "target_140_score": target_140["score_mean_all"],
        "target_120_160_active": target_120_160["active_frac"],
        "target_100_120_active": target_100_120["active_frac"],
        "target_160_180_active": target_160_180["active_frac"],
        "online_diag_140_active": online_diag_140["active_frac"],
        "online_diag_160_180_active": online_diag_160["active_frac"],
        "protection_mean_active": protection_mean_active,
        "protection_max_active": protection_max_active,
        "protection_max_q_exposure": protection_max_exposure,
        **{f"{run}_40_180_active": summary["active_frac"] for run, summary in protection_summaries.items()},
    }


def score_candidates(rows: pd.DataFrame) -> list[dict[str, object]]:
    scored = [score_candidate(rows, candidate) for candidate in build_candidates()]
    scored.sort(
        key=lambda row: (
            -to_float(row.get("strict"), 0.0),
            -to_float(row.get("objective"), -1e9),
            -to_float(row.get("target_140_active"), -1.0),
            to_float(row.get("protection_max_active"), 1.0),
        )
    )
    return scored


def per_run_windows(rows: pd.DataFrame, candidate: Candidate) -> list[dict[str, object]]:
    framed = candidate_frame(rows, candidate)
    out: list[dict[str, object]] = []
    for run in RUN_DIRS:
        for window in ["40-180", "80-100", "100-120", "120-140", "140-160", "160-180"]:
            summary = summarize_subset(window_rows(framed, run, window))
            out.append(
                {
                    "run": run,
                    "window": window,
                    "rows": int(summary["rows"]),
                    "active_frac": summary["active_frac"],
                    "score_mean_all": summary["score_mean_all"],
                    "score_mean_active": summary["score_mean_active"],
                    "q_exposure_mean": summary["q_exposure_mean"],
                    "q_exposure_p95": summary["q_exposure_p95"],
                    "accbias_z_mean": summary["accbias_z_mean"],
                }
            )
    return out


def table_score(rows: list[dict[str, object]], limit: int = 16) -> list[list[object]]:
    keys = [
        "candidate",
        "objective",
        "strict",
        "target_140_active",
        "target_100_120_active",
        "target_160_180_active",
        "online_diag_140_active",
        "online_diag_160_180_active",
        "protection_max_active",
        "protection_mean_active",
    ]
    return [[fmt(row[key]) if key not in {"candidate", "strict"} else row[key] for key in keys] for row in rows[:limit]]


def filtered_score_table(rows: list[dict[str, object]], mode: str, limit: int = 12) -> list[list[object]]:
    if mode == "formal":
        selected = [
            row for row in rows
            if to_float(row["target_140_active"]) >= 0.40
            and to_float(row["protection_max_active"]) <= 0.10
        ]
    elif mode == "online_clean":
        selected = [
            row for row in rows
            if to_float(row["online_diag_140_active"]) >= 0.45
            and to_float(row["online_diag_160_180_active"]) <= 0.20
            and to_float(row["protection_max_active"]) <= 0.10
        ]
    else:
        raise ValueError(mode)
    selected.sort(
        key=lambda row: (
            -to_float(row["objective"], -1e9),
            -to_float(row["target_140_active"], -1.0),
            to_float(row["online_diag_160_180_active"], 1.0),
        )
    )
    return table_score(selected, limit)


def table_windows(rows: list[dict[str, object]], limit_runs: set[str]) -> list[list[object]]:
    selected = [
        row for row in rows
        if row["run"] in limit_runs and row["window"] in {"40-180", "100-120", "120-140", "140-160", "160-180"}
    ]
    return [
        [
            row["run"],
            row["window"],
            row["rows"],
            fmt(row["active_frac"]),
            fmt(row["score_mean_all"]),
            fmt(row["q_exposure_mean"]),
            fmt(row["accbias_z_mean"]),
        ]
        for row in selected
    ]


def write_report(out_dir: Path, rows: pd.DataFrame, scored: list[dict[str, object]], best_windows: list[dict[str, object]]) -> None:
    strict = [row for row in scored if to_float(row.get("strict")) > 0.5]
    best = scored[0]
    report = [
        "# PHS5 accbias_z gate sweep",
        "",
        "Date: 2026-05-11",
        "",
        "This is an offline selector/Q-exposure sweep over existing `state_update_debug.csv` logs. It does not replay the estimator and does not claim a performance counterfactual.",
        "",
        "Formal target: `shortgen11_repeat2` 140-160s. Spill checks: `shortgen11_repeat2` 100-120s and 160-180s. Protection checks: shortgen01/02/03/04 PHS5 40-180s. Online diagnostic/apply runs are included only as sanity checks.",
        "",
        f"Loaded rows: `{len(rows)}`. Scored candidates: `{len(scored)}`. Strict candidates: `{len(strict)}`.",
        "",
        "## Top candidates",
        "",
        markdown_table(
            [
                "candidate",
                "objective",
                "strict",
                "target140",
                "target100-120",
                "target160-180",
                "diag140",
                "diag160-180",
                "protect max",
                "protect mean",
            ],
            table_score(scored, 18),
        ),
        "",
        "## Conflict checks",
        "",
        "Formal-preserving candidates keep `shortgen11_repeat2` 140-160s coverage and protect shortgen01/02/03/04. Online-clean candidates keep the new diagnostic 140-160s coverage while suppressing its 160-180s spill. A safe next online candidate should appear in both tables.",
        "",
        "### Formal-preserving candidates",
        "",
        markdown_table(
            [
                "candidate",
                "objective",
                "strict",
                "target140",
                "target100-120",
                "target160-180",
                "diag140",
                "diag160-180",
                "protect max",
                "protect mean",
            ],
            filtered_score_table(scored, "formal", 12),
        ),
        "",
        "### Online-clean candidates",
        "",
        markdown_table(
            [
                "candidate",
                "objective",
                "strict",
                "target140",
                "target100-120",
                "target160-180",
                "diag140",
                "diag160-180",
                "protect max",
                "protect mean",
            ],
            filtered_score_table(scored, "online_clean", 12),
        ),
        "",
        "## Best-candidate windows",
        "",
        f"Best candidate: `{best['candidate']}`.",
        "",
        markdown_table(
            ["run", "window", "rows", "active", "score mean", "q exposure", "ba_z mean"],
            table_windows(
                best_windows,
                FORMAL_PROTECTION_RUNS | {FORMAL_TARGET_RUN, ONLINE_DIAGNOSTIC_RUN, "shortgen11_accbiasz_apply", "shortgen04_futureclamp"},
            ),
        ),
        "",
        "## Verdict",
        "",
    ]
    if strict:
        report.extend(
            [
                "At least one offline selector candidate satisfies the coarse constraints. This is enough to justify one more opt-in online mechanism run after implementing any missing gate parameters.",
                "",
                "Recommended default next action: inspect the strict candidates and implement only the missing gate parameters they require. Do not promote the current `-0.18/-0.24` mechanism.",
                "",
            ]
        )
    else:
        report.extend(
            [
                "No candidate simultaneously preserves the target window, suppresses shortgen11 spill, and protects shortgen01/02/03/04 under the current coarse sweep constraints.",
                "",
                "The conflict is visible in the two filtered tables: formal-preserving candidates require low thresholds around `-0.18` and still spill in the new online diagnostic 160-180s window, while online-clean candidates around `-0.205/-0.210` miss the formal shortgen11 repeat2 140-160s state signature.",
                "",
                "Recommended default next action: do not run another mechanism flight yet. First diagnose why the formal repeat2 and new diagnostic shortgen11 logs disagree in the `accbias_z_before_mps2` state signature around 140-160s.",
                "",
            ]
        )
    (out_dir / "report.md").write_text("\n".join(report), encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUT)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)
    rows = load_state_updates(RUN_DIRS)
    if rows.empty:
        raise SystemExit("no state_update_debug rows found")
    scored = score_candidates(rows)
    best = scored[0]
    best_candidate = Candidate(
        trigger_mps2=to_float(best["trigger_mps2"]),
        full_mps2=to_float(best["full_mps2"]),
        min_hspeed_mps=to_float(best["min_hspeed_mps"]),
        max_abs_vspeed_mps=to_float(best["max_abs_vspeed_mps"]),
        context_gate=str(best["context_gate"]),
        vrw_q_scale_max=to_float(best["vrw_q_scale_max"]),
        accbias_q_scale_max=to_float(best["accbias_q_scale_max"]),
    )
    best_windows = per_run_windows(rows, best_candidate)
    write_csv(args.out_dir / "candidate_scores.csv", scored)
    write_csv(args.out_dir / "best_candidate_windows.csv", best_windows)
    write_report(args.out_dir, rows, scored, best_windows)
    print(f"wrote: {args.out_dir / 'candidate_scores.csv'}")
    print(f"wrote: {args.out_dir / 'best_candidate_windows.csv'}")
    print(f"wrote: {args.out_dir / 'report.md'}")


if __name__ == "__main__":
    main()
