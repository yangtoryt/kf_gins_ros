#!/usr/bin/env python3
"""Offline PHS5 projection replay with update-history proxies.

The row-level proxy atlas only used the latest accepted GNSS/update values
already joined onto each pair row.  This script reads the full GNSS and state
update debug streams and reconstructs short history memories before each pair:
windowed sums/means, EWMA vectors, and weak-correction pressure vectors.

It remains offline-only.  It starts no flight stack and changes no estimator
behavior.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence

import offline_phs5_measurement_geometry_selector as mg
import offline_phs5_selector_replay_after_lag_failure as replay


BASE = Path("/home/yang/kf_gins_ws/artifacts/manual/short_route_generalization_2026-05-07")
DEFAULT_OUT = BASE / "phs5_history_proxy_replay_2026-05-10"
SEGMENT_LENGTHS = (2.0, 5.0, 10.0)
HISTORY_WINDOWS = (5.0, 10.0, 20.0, 30.0)
EWMA_TAUS = (5.0, 10.0, 20.0)

POSITIVE_RUNS = replay.POSITIVE_RUNS
TARGET_RUNS = replay.TARGET_RUNS
WINDOWS = {
    "main_40_180": (40.0, 180.0),
    "140_160": (140.0, 160.0),
    "160_180": (160.0, 180.0),
}


def finite(value: object) -> bool:
    return mg.finite(value)


def to_float(value: object, default: float = math.nan) -> float:
    return mg.to_float(value, default)


def fmt(value: object, digits: int = 4) -> str:
    return mg.fmt(value, digits)


def mean(values: Iterable[float]) -> float:
    return mg.mean(values)


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
    return mg.markdown_table(headers, rows)


def norm(east: float, north: float) -> float:
    return math.hypot(east, north) if finite(east) and finite(north) else math.nan


def angle_deg(east: float, north: float) -> float:
    if not finite(east) or not finite(north) or (abs(east) < 1e-9 and abs(north) < 1e-9):
        return math.nan
    return math.degrees(math.atan2(east, north))


def angle_diff_deg(a: float, b: float) -> float:
    if not finite(a) or not finite(b):
        return math.nan
    return (a - b + 180.0) % 360.0 - 180.0


def abs_angle_diff_deg(a: float, b: float) -> float:
    diff = angle_diff_deg(a, b)
    return abs(diff) if finite(diff) else math.nan


def cosine_between(a_e: float, a_n: float, b_e: float, b_n: float) -> float:
    a_len = norm(a_e, a_n)
    b_len = norm(b_e, b_n)
    if not finite(a_len) or not finite(b_len) or a_len < 1e-9 or b_len < 1e-9:
        return math.nan
    return (a_e * b_e + a_n * b_n) / (a_len * b_len)


def safe_div(num: float, den: float) -> float:
    if not finite(num) or not finite(den) or abs(den) < 1e-9:
        return math.nan
    return num / den


def vec_sum(items: Iterable[tuple[float, float]]) -> tuple[float, float]:
    vals = [(east, north) for east, north in items if finite(east) and finite(north)]
    if not vals:
        return math.nan, math.nan
    return sum(east for east, _north in vals), sum(north for _east, north in vals)


def vec_mean(items: Iterable[tuple[float, float]]) -> tuple[float, float]:
    vals = [(east, north) for east, north in items if finite(east) and finite(north)]
    if not vals:
        return math.nan, math.nan
    return sum(east for east, _north in vals) / len(vals), sum(north for _east, north in vals) / len(vals)


def add_vecs(*vectors: tuple[float, float]) -> tuple[float, float]:
    vals = [(east, north) for east, north in vectors if finite(east) and finite(north)]
    if not vals:
        return math.nan, math.nan
    return sum(east for east, _north in vals), sum(north for _east, north in vals)


@dataclass(frozen=True)
class UpdateEvent:
    time_sec: float
    residual_e: float
    residual_n: float
    residual_h: float
    dx_e: float
    dx_n: float
    dx_h: float
    dx_over_residual: float
    pos_std_h: float
    context: str


def load_update_events(run_dir: Path) -> tuple[list[UpdateEvent], list[float]]:
    gnss_rows = [
        row
        for row in read_csv(run_dir / "gnss_update_debug.csv")
        if to_float(row.get("gnss_position_applied"), 0.0) > 0.5
    ]
    state_rows = [
        row
        for row in read_csv(run_dir / "state_update_debug.csv")
        if row.get("event_type") == "gnss_position" and to_float(row.get("applied"), 0.0) > 0.5
    ]
    state_rows.sort(key=lambda row: to_float(row.get("ros_time_sec")))
    state_times = [to_float(row.get("ros_time_sec")) for row in state_rows]

    events: list[UpdateEvent] = []
    for gnss in gnss_rows:
        t = to_float(gnss.get("ros_time_sec"))
        if not finite(t):
            continue
        state, age = mg.latest_before(state_rows, state_times, t + 0.05, 0.25)
        if state is None:
            continue
        residual_e = to_float(gnss.get("gnss_position_residual_e_m"))
        residual_n = to_float(gnss.get("gnss_position_residual_n_m"))
        dx_e = to_float(state.get("dx_pos_e_m"))
        dx_n = to_float(state.get("dx_pos_n_m"))
        residual_h = norm(residual_e, residual_n)
        dx_h = norm(dx_e, dx_n)
        std_n = to_float(gnss.get("gnss_position_std_n_m"))
        std_e = to_float(gnss.get("gnss_position_std_e_m"))
        pos_std_h = math.hypot(std_n, std_e) if finite(std_n) and finite(std_e) else math.nan
        context = "armed_cruise"
        if to_float(state.get("turning_now"), 0.0) > 0.5:
            context = "turning"
        elif to_float(state.get("post_turn_context"), 0.0) > 0.5:
            context = "post_turn"
        elif to_float(state.get("armed_cruise_context"), 0.0) < 0.5:
            context = "other"
        _ = age
        events.append(
            UpdateEvent(
                time_sec=t,
                residual_e=residual_e,
                residual_n=residual_n,
                residual_h=residual_h,
                dx_e=dx_e,
                dx_n=dx_n,
                dx_h=dx_h,
                dx_over_residual=safe_div(dx_h, residual_h),
                pos_std_h=pos_std_h,
                context=context,
            )
        )
    events.sort(key=lambda event: event.time_sec)
    return events, [event.time_sec for event in events]


def event_slice(events: Sequence[UpdateEvent], times: Sequence[float], t: float, window_sec: float) -> list[UpdateEvent]:
    lo = bisect.bisect_left(times, t - window_sec)
    hi = bisect.bisect_right(times, t)
    return list(events[lo:hi])


def ewma_vec(events: Sequence[UpdateEvent], t: float, tau_sec: float, field: str) -> tuple[float, float]:
    weighted: list[tuple[float, float]] = []
    weights: list[float] = []
    for event in events:
        age = t - event.time_sec
        if age < -1e-6 or age > 4.0 * tau_sec:
            continue
        weight = math.exp(-max(age, 0.0) / tau_sec)
        if field == "residual":
            east, north = event.residual_e, event.residual_n
        elif field == "dx":
            east, north = event.dx_e, event.dx_n
        elif field == "unresolved":
            east, north = event.residual_e - event.dx_e, event.residual_n - event.dx_n
        elif field == "weak_residual":
            gain = event.dx_over_residual if finite(event.dx_over_residual) else 0.0
            pressure = max(0.0, min(1.0, 1.0 - gain))
            east, north = event.residual_e * pressure, event.residual_n * pressure
        else:
            raise ValueError(field)
        if finite(east) and finite(north):
            weighted.append((east * weight, north * weight))
            weights.append(weight)
    if not weights:
        return math.nan, math.nan
    east, north = vec_sum(weighted)
    total = sum(weights)
    return east / total, north / total


def window_vec(events: Sequence[UpdateEvent], field: str, reducer: str) -> tuple[float, float]:
    vectors: list[tuple[float, float]] = []
    for event in events:
        if field == "residual":
            vectors.append((event.residual_e, event.residual_n))
        elif field == "dx":
            vectors.append((event.dx_e, event.dx_n))
        elif field == "unresolved":
            vectors.append((event.residual_e - event.dx_e, event.residual_n - event.dx_n))
        elif field == "weak_residual":
            gain = event.dx_over_residual if finite(event.dx_over_residual) else 0.0
            pressure = max(0.0, min(1.0, 1.0 - gain))
            vectors.append((event.residual_e * pressure, event.residual_n * pressure))
        elif field == "std_weighted_residual":
            weight = 1.0 / max(to_float(event.pos_std_h, 1.0), 0.05) ** 2
            vectors.append((event.residual_e * weight, event.residual_n * weight))
        else:
            raise ValueError(field)
    if reducer == "sum":
        return vec_sum(vectors)
    if reducer == "mean":
        return vec_mean(vectors)
    raise ValueError(reducer)


def add_proxy_features(
    row: dict[str, object],
    prefix: str,
    proxy_e: float,
    proxy_n: float,
    proj_e: float,
    proj_n: float,
    raw_err_e: float,
    raw_err_n: float,
) -> None:
    proxy_heading = angle_deg(proxy_e, proxy_n)
    neg_heading = angle_deg(-proxy_e, -proxy_n)
    proj_heading = angle_deg(proj_e, proj_n)
    raw_heading = angle_deg(raw_err_e, raw_err_n)
    row[f"{prefix}_h_m"] = norm(proxy_e, proxy_n)
    row[f"{prefix}_vs_projection_abs_deg"] = abs_angle_diff_deg(proxy_heading, proj_heading)
    row[f"neg_{prefix}_vs_projection_abs_deg"] = abs_angle_diff_deg(neg_heading, proj_heading)
    row[f"{prefix}_dot_projection_cos"] = cosine_between(proxy_e, proxy_n, proj_e, proj_n)
    row[f"neg_{prefix}_dot_projection_cos"] = cosine_between(-proxy_e, -proxy_n, proj_e, proj_n)
    row[f"{prefix}_vs_oracle_raw_error_abs_deg"] = abs_angle_diff_deg(proxy_heading, raw_heading)


def build_row_observables() -> list[dict[str, object]]:
    out: list[dict[str, object]] = []
    for spec in replay.RUNS:
        rows = mg.load_rows(mg.RunSpec(spec.label, spec.role, spec.path))
        events, times = load_update_events(spec.path)
        for row in rows:
            t = to_float(row.get("pair_ros_time_sec"))
            if not finite(t):
                continue
            raw_x = to_float(row.get("raw_iekf_x_m"))
            raw_y = to_float(row.get("raw_iekf_y_m"))
            gt_x = to_float(row.get("gt_x_m"))
            gt_y = to_float(row.get("gt_y_m"))
            px4_x = to_float(row.get("px4_iekf_x_m"))
            px4_y = to_float(row.get("px4_iekf_y_m"))
            proj_e = px4_x - raw_x
            proj_n = px4_y - raw_y
            raw_err_e = raw_x - gt_x
            raw_err_n = raw_y - gt_y

            row["group"] = spec.group
            row["history_event_count_total"] = len(events)
            row["oracle_raw_error_h_m"] = norm(raw_err_e, raw_err_n)
            row["projection_delta_h_m"] = norm(proj_e, proj_n)
            row["oracle_raw_error_vs_projection_abs_deg"] = abs_angle_diff_deg(
                angle_deg(raw_err_e, raw_err_n), angle_deg(proj_e, proj_n)
            )
            row["projection_dot_oracle_raw_error_cos"] = cosine_between(proj_e, proj_n, raw_err_e, raw_err_n)

            for window_sec in HISTORY_WINDOWS:
                hist = event_slice(events, times, t, window_sec)
                suffix = f"{int(window_sec)}s"
                row[f"hist_count_{suffix}"] = len(hist)
                row[f"hist_residual_h_mean_{suffix}"] = mean(event.residual_h for event in hist)
                row[f"hist_dx_h_mean_{suffix}"] = mean(event.dx_h for event in hist)
                row[f"hist_dx_over_residual_mean_{suffix}"] = mean(event.dx_over_residual for event in hist)
                row[f"hist_weak_gain_frac_{suffix}"] = mean(
                    1.0 if finite(event.dx_over_residual) and event.dx_over_residual < 0.25 else 0.0
                    for event in hist
                )
                row[f"hist_post_turn_frac_{suffix}"] = mean(
                    1.0 if event.context in {"turning", "post_turn"} else 0.0 for event in hist
                )
                for field in ("residual", "dx", "unresolved", "weak_residual", "std_weighted_residual"):
                    for reducer in ("sum", "mean"):
                        prefix = f"hist_{field}_{reducer}_{suffix}"
                        east, north = window_vec(hist, field, reducer)
                        add_proxy_features(row, prefix, east, north, proj_e, proj_n, raw_err_e, raw_err_n)
                residual_sum = window_vec(hist, "residual", "sum")
                dx_sum = window_vec(hist, "dx", "sum")
                combo_e, combo_n = add_vecs(residual_sum, dx_sum)
                add_proxy_features(row, f"hist_residual_plus_dx_sum_{suffix}", combo_e, combo_n, proj_e, proj_n, raw_err_e, raw_err_n)

            for tau_sec in EWMA_TAUS:
                suffix = f"{int(tau_sec)}s"
                for field in ("residual", "dx", "unresolved", "weak_residual"):
                    prefix = f"ewma_{field}_{suffix}"
                    east, north = ewma_vec(events, t, tau_sec, field)
                    add_proxy_features(row, prefix, east, north, proj_e, proj_n, raw_err_e, raw_err_n)
            out.append(row)
    out.sort(key=lambda row: (str(row.get("run")), to_float(row.get("time_since_arm_sec"))))
    return out


def segment_index(t: float, segment_sec: float) -> int:
    return int(math.floor((t - 40.0) / segment_sec))


def segment_rows(rows: Sequence[dict[str, object]], segment_sec: float) -> list[dict[str, object]]:
    grouped: dict[tuple[str, int], list[dict[str, object]]] = {}
    for row in rows:
        t = to_float(row.get("time_since_arm_sec"))
        if 40.0 <= t < 180.0:
            grouped.setdefault((str(row.get("run")), segment_index(t, segment_sec)), []).append(row)

    out: list[dict[str, object]] = []
    for (run, index), items in sorted(grouped.items()):
        if len(items) < 2:
            continue
        start = 40.0 + index * segment_sec
        n = len(items)
        raw_sse = sum(to_float(row.get("raw_iekf_error_m")) ** 2 for row in items)
        px4_sse = sum(to_float(row.get("px4_iekf_error_m")) ** 2 for row in items)
        ekf2_sse = sum(to_float(row.get("ekf2_error_m")) ** 2 for row in items)
        segment: dict[str, object] = {
            "run": run,
            "group": "positive" if run in POSITIVE_RUNS else "target",
            "segment_sec": segment_sec,
            "segment_index": index,
            "start_sec": start,
            "end_sec": start + segment_sec,
            "rows": n,
            "raw_rmse_m": math.sqrt(raw_sse / n),
            "px4_rmse_m": math.sqrt(px4_sse / n),
            "ekf2_rmse_m": math.sqrt(ekf2_sse / n),
            "turning_now_mean": mean(to_float(row.get("turning_now")) for row in items),
            "post_turn_context_mean": mean(to_float(row.get("post_turn_context")) for row in items),
            "armed_cruise_context_mean": mean(to_float(row.get("armed_cruise_context")) for row in items),
            "stamp_lag_sec_mean": mean(to_float(row.get("stamp_lag_sec")) for row in items),
            "state_core_gnss_diff_h_m_mean": mean(to_float(row.get("state_core_gnss_diff_h_m")) for row in items),
            "core_gnss_along_velocity_m_mean": mean(to_float(row.get("core_gnss_along_velocity_m")) for row in items),
            "projection_delta_cross_velocity_m_mean": mean(
                to_float(row.get("projection_delta_cross_velocity_m")) for row in items
            ),
            "projection_delta_h_m_mean": mean(to_float(row.get("projection_delta_h_m")) for row in items),
            "oracle_raw_error_vs_projection_abs_deg": mean(
                to_float(row.get("oracle_raw_error_vs_projection_abs_deg")) for row in items
            ),
            "projection_dot_oracle_raw_error_cos": mean(
                to_float(row.get("projection_dot_oracle_raw_error_cos")) for row in items
            ),
        }
        segment["px4_minus_raw_rmse_m"] = to_float(segment["px4_rmse_m"]) - to_float(segment["raw_rmse_m"])
        segment["px4_minus_ekf2_rmse_m"] = to_float(segment["px4_rmse_m"]) - to_float(segment["ekf2_rmse_m"])
        segment["raw_minus_ekf2_rmse_m"] = to_float(segment["raw_rmse_m"]) - to_float(segment["ekf2_rmse_m"])
        segment["projection_benefit_rmse_m"] = to_float(segment["raw_rmse_m"]) - to_float(segment["px4_rmse_m"])
        segment["projection_class"] = (
            "help" if to_float(segment["projection_benefit_rmse_m"]) > 0.02
            else "harm" if to_float(segment["projection_benefit_rmse_m"]) < -0.02
            else "neutral"
        )

        for key in items[0]:
            if (
                key.startswith("hist_")
                or key.startswith("ewma_")
                or key.startswith("neg_hist_")
                or key.startswith("neg_ewma_")
            ):
                segment[f"{key}_mean"] = mean(to_float(row.get(key)) for row in items)
        out.append(segment)
    return out


@dataclass(frozen=True)
class Condition:
    field: str
    op: str
    threshold: float

    def matches(self, row: dict[str, object]) -> bool:
        value = to_float(row.get(self.field))
        if not finite(value):
            return False
        if self.op == "ge":
            return value >= self.threshold
        if self.op == "le":
            return value <= self.threshold
        raise ValueError(self.op)

    def label(self) -> str:
        return f"{self.field}_{self.op}_{self.threshold:.3f}"


@dataclass(frozen=True)
class Candidate:
    name: str
    family: str
    conditions: tuple[Condition, ...]

    def active(self, row: dict[str, object]) -> bool:
        return all(condition.matches(row) for condition in self.conditions)


def make_candidate(family: str, conditions: Sequence[Condition]) -> Candidate:
    return Candidate("__".join(condition.label() for condition in conditions), family, tuple(conditions))


def vector_prefixes() -> list[str]:
    prefixes: list[str] = []
    for window_sec in HISTORY_WINDOWS:
        suffix = f"{int(window_sec)}s"
        for field in ("residual", "dx", "unresolved", "weak_residual"):
            prefixes.append(f"hist_{field}_sum_{suffix}")
        prefixes.append(f"hist_residual_mean_{suffix}")
        prefixes.append(f"hist_residual_plus_dx_sum_{suffix}")
    for tau_sec in (10.0, 20.0):
        suffix = f"{int(tau_sec)}s"
        for field in ("residual", "unresolved", "weak_residual"):
            prefixes.append(f"ewma_{field}_{suffix}")
    return prefixes


def build_candidates() -> list[Candidate]:
    candidates: list[Candidate] = []
    angle_thresholds = (30.0, 60.0, 120.0, 150.0)
    mag_thresholds = (0.05, 0.10, 0.20, 0.40)
    global_guards = [
        Condition("armed_cruise_context_mean", "ge", 0.50),
        Condition("turning_now_mean", "le", 0.25),
        Condition("stamp_lag_sec_mean", "ge", 0.015),
        Condition("state_core_gnss_diff_h_m_mean", "le", 1.05),
        Condition("projection_delta_cross_velocity_m_mean", "le", 0.02),
    ]
    window_guards: dict[str, list[Condition]] = {}
    for window_sec in HISTORY_WINDOWS:
        suffix = f"{int(window_sec)}s"
        window_guards[suffix] = [
            Condition(f"hist_count_{suffix}_mean", "ge", max(2.0, window_sec * 0.5)),
            Condition(f"hist_residual_h_mean_{suffix}_mean", "ge", 0.10),
            Condition(f"hist_dx_over_residual_mean_{suffix}_mean", "le", 0.25),
            Condition(f"hist_weak_gain_frac_{suffix}_mean", "ge", 0.25),
        ]
    for prefix in vector_prefixes():
        suffix_guards: list[Condition] = []
        for suffix, guards_for_suffix in window_guards.items():
            if prefix.endswith(f"_{suffix}"):
                suffix_guards = guards_for_suffix
                break
        guards = global_guards + suffix_guards
        angle_conditions: list[Condition] = []
        for threshold in angle_thresholds:
            angle_conditions.append(Condition(f"{prefix}_vs_projection_abs_deg_mean", "le", threshold))
            angle_conditions.append(Condition(f"{prefix}_vs_projection_abs_deg_mean", "ge", threshold))
            angle_conditions.append(Condition(f"neg_{prefix}_vs_projection_abs_deg_mean", "le", threshold))
            angle_conditions.append(Condition(f"neg_{prefix}_vs_projection_abs_deg_mean", "ge", threshold))
        mag_conditions = [Condition(f"{prefix}_h_m_mean", "ge", value) for value in mag_thresholds]
        for condition in angle_conditions + mag_conditions:
            candidates.append(make_candidate(prefix, (condition,)))
        for angle in angle_conditions:
            for guard in guards:
                candidates.append(make_candidate(f"{prefix}_x_guard", (angle, guard)))
    unique: dict[str, Candidate] = {}
    for candidate in candidates:
        unique[f"{candidate.family}:{candidate.name}"] = candidate
    return list(unique.values())


def segment_in_window(segment: dict[str, object], window: str) -> bool:
    start, end = WINDOWS[window]
    return to_float(segment.get("start_sec")) < end and to_float(segment.get("end_sec")) > start


def select_segments(segments: Sequence[dict[str, object]], runs: Sequence[str], window: str) -> list[dict[str, object]]:
    run_set = set(runs)
    return [row for row in segments if row.get("run") in run_set and segment_in_window(row, window)]


def aggregate(segments: Sequence[dict[str, object]], candidate: Candidate) -> dict[str, object]:
    rows = sum(int(to_float(row.get("rows"), 0.0)) for row in segments)
    if rows <= 0:
        return {
            "rows": 0,
            "active_frac": math.nan,
            "raw_rmse_m": math.nan,
            "px4_rmse_m": math.nan,
            "ekf2_rmse_m": math.nan,
            "candidate_rmse_m": math.nan,
            "candidate_minus_raw_m": math.nan,
            "candidate_minus_ekf2_m": math.nan,
        }
    active_rows = 0
    totals = {"raw": 0.0, "px4": 0.0, "ekf2": 0.0, "candidate": 0.0}
    for row in segments:
        n = int(to_float(row.get("rows"), 0.0))
        raw = to_float(row.get("raw_rmse_m"))
        px4 = to_float(row.get("px4_rmse_m"))
        ekf2 = to_float(row.get("ekf2_rmse_m"))
        active = candidate.active(row)
        chosen = px4 if active else raw
        active_rows += n if active else 0
        totals["raw"] += raw * raw * n
        totals["px4"] += px4 * px4 * n
        totals["ekf2"] += ekf2 * ekf2 * n
        totals["candidate"] += chosen * chosen * n
    raw_rmse = math.sqrt(totals["raw"] / rows)
    px4_rmse = math.sqrt(totals["px4"] / rows)
    ekf2_rmse = math.sqrt(totals["ekf2"] / rows)
    candidate_rmse = math.sqrt(totals["candidate"] / rows)
    return {
        "rows": rows,
        "active_frac": active_rows / rows,
        "raw_rmse_m": raw_rmse,
        "px4_rmse_m": px4_rmse,
        "ekf2_rmse_m": ekf2_rmse,
        "candidate_rmse_m": candidate_rmse,
        "candidate_minus_raw_m": candidate_rmse - raw_rmse,
        "candidate_minus_ekf2_m": candidate_rmse - ekf2_rmse,
    }


def score_candidate(segments: Sequence[dict[str, object]], candidate: Candidate) -> dict[str, object]:
    pos = [aggregate(select_segments(segments, [run], "main_40_180"), candidate) for run in POSITIVE_RUNS]
    target_main = [aggregate(select_segments(segments, [run], "main_40_180"), candidate) for run in TARGET_RUNS]
    target_140 = [aggregate(select_segments(segments, [run], "140_160"), candidate) for run in TARGET_RUNS]
    target_160 = [aggregate(select_segments(segments, [run], "160_180"), candidate) for run in TARGET_RUNS]
    pos_reg = [to_float(row.get("candidate_minus_raw_m")) for row in pos]
    pos_gap = [to_float(row.get("candidate_minus_ekf2_m")) for row in pos]
    target_140_gap = [to_float(row.get("candidate_minus_ekf2_m")) for row in target_140]
    target_160_gap = [to_float(row.get("candidate_minus_ekf2_m")) for row in target_160]
    protected = max(pos_reg) <= 0.02 and max(pos_gap) < 0.02
    return {
        "candidate": candidate.name,
        "family": candidate.family,
        "condition_count": len(candidate.conditions),
        "positive_max_main_regress_m": max(pos_reg),
        "positive_max_candidate_minus_ekf2_m": max(pos_gap),
        "positive_all_protected": int(protected),
        "target_worst_main_candidate_minus_ekf2_m": max(to_float(row.get("candidate_minus_ekf2_m")) for row in target_main),
        "target_worst_140_160_candidate_minus_ekf2_m": max(target_140_gap),
        "target_worst_160_180_candidate_minus_ekf2_m": max(target_160_gap),
        "target_mean_140_160_improve_m": mean(
            to_float(row.get("raw_rmse_m")) - to_float(row.get("candidate_rmse_m")) for row in target_140
        ),
        "target_140_160_pass": int(max(target_140_gap) <= 0.02),
        "target_160_180_pass": int(max(target_160_gap) <= 0.02),
        "strict_pass": int(protected and max(target_140_gap) <= 0.02 and max(target_160_gap) <= 0.02),
    }


def sort_key(row: dict[str, object]) -> tuple[float, float, float, float, float]:
    return (
        0.0 if to_float(row.get("positive_all_protected"), 0.0) > 0.5 else 1.0,
        to_float(row.get("target_worst_140_160_candidate_minus_ekf2_m")),
        to_float(row.get("target_worst_160_180_candidate_minus_ekf2_m")),
        to_float(row.get("positive_max_main_regress_m")),
        -to_float(row.get("target_mean_140_160_improve_m")),
    )


def feature_separation(segments: Sequence[dict[str, object]]) -> list[dict[str, object]]:
    target_help = [
        row for row in segments
        if row.get("run") in TARGET_RUNS
        and 140.0 <= to_float(row.get("start_sec")) < 180.0
        and to_float(row.get("projection_benefit_rmse_m")) > 0.02
    ]
    positive_harm = [
        row for row in segments
        if row.get("run") in POSITIVE_RUNS
        and to_float(row.get("projection_benefit_rmse_m")) < -0.02
    ]
    fields = [
        "oracle_raw_error_vs_projection_abs_deg",
        "projection_dot_oracle_raw_error_cos",
        "stamp_lag_sec_mean",
        "state_core_gnss_diff_h_m_mean",
        "core_gnss_along_velocity_m_mean",
        "projection_delta_cross_velocity_m_mean",
    ]
    for window_sec in HISTORY_WINDOWS:
        suffix = f"{int(window_sec)}s"
        fields.extend(
            [
                f"hist_count_{suffix}_mean",
                f"hist_residual_h_mean_{suffix}_mean",
                f"hist_dx_h_mean_{suffix}_mean",
                f"hist_dx_over_residual_mean_{suffix}_mean",
                f"hist_weak_gain_frac_{suffix}_mean",
            ]
        )
    for prefix in vector_prefixes():
        fields.extend(
            [
                f"{prefix}_h_m_mean",
                f"{prefix}_vs_projection_abs_deg_mean",
                f"neg_{prefix}_vs_projection_abs_deg_mean",
                f"{prefix}_dot_projection_cos_mean",
                f"neg_{prefix}_dot_projection_cos_mean",
                f"{prefix}_vs_oracle_raw_error_abs_deg_mean",
            ]
        )
    out: list[dict[str, object]] = []
    for field in fields:
        help_values = [to_float(row.get(field)) for row in target_help]
        harm_values = [to_float(row.get(field)) for row in positive_harm]
        help_mean = mean(help_values)
        harm_mean = mean(harm_values)
        help_var = mean((value - help_mean) ** 2 for value in help_values if finite(value) and finite(help_mean))
        harm_var = mean((value - harm_mean) ** 2 for value in harm_values if finite(value) and finite(harm_mean))
        pooled = math.sqrt(help_var + harm_var) if finite(help_var) and finite(harm_var) else math.nan
        out.append(
            {
                "feature": field,
                "target_help_count": len([value for value in help_values if finite(value)]),
                "positive_harm_count": len([value for value in harm_values if finite(value)]),
                "target_help_mean": help_mean,
                "positive_harm_mean": harm_mean,
                "mean_diff_target_help_minus_positive_harm": help_mean - harm_mean,
                "separation_score": abs(help_mean - harm_mean) / pooled if finite(pooled) and pooled > 1e-9 else math.nan,
            }
        )
    out.sort(key=lambda row: to_float(row.get("separation_score")), reverse=True)
    return out


def table_candidate(rows: Sequence[dict[str, object]]) -> list[list[object]]:
    return [
        [
            fmt(row["segment_sec"], 1),
            row["family"],
            row["candidate"],
            row["positive_all_protected"],
            fmt(row["positive_max_main_regress_m"]),
            fmt(row["target_worst_140_160_candidate_minus_ekf2_m"]),
            fmt(row["target_worst_160_180_candidate_minus_ekf2_m"]),
            fmt(row["target_mean_140_160_improve_m"]),
            row["strict_pass"],
        ]
        for row in rows
    ]


def write_report(out_dir: Path, separation_rows: list[dict[str, object]], candidate_rows: list[dict[str, object]]) -> None:
    strict = [row for row in candidate_rows if to_float(row.get("strict_pass"), 0.0) > 0.5]
    strict.sort(key=sort_key)
    protected_top = [row for row in candidate_rows if to_float(row.get("positive_all_protected"), 0.0) > 0.5]
    protected_top.sort(key=sort_key)
    relaxed_top = sorted(
        candidate_rows,
        key=lambda row: (
            to_float(row.get("target_worst_140_160_candidate_minus_ekf2_m")),
            to_float(row.get("target_worst_160_180_candidate_minus_ekf2_m")),
            to_float(row.get("positive_max_main_regress_m")),
        ),
    )
    sep_table = [
        [
            row["feature"],
            row["target_help_count"],
            row["positive_harm_count"],
            fmt(row["target_help_mean"]),
            fmt(row["positive_harm_mean"]),
            fmt(row["mean_diff_target_help_minus_positive_harm"]),
            fmt(row["separation_score"]),
        ]
        for row in separation_rows[:18]
    ]
    lines = [
        "# PHS5 update-history proxy replay",
        "",
        "Date: 2026-05-10",
        "",
        "Offline-only replay from existing PHS5 logs. No flight stack, rebuild, PX4, Gazebo, MAVROS, QGC, RViz2, PlotJuggler, or estimator process was started.",
        "",
        "## Feature Separation",
        "",
        markdown_table(
            ["feature", "target help n", "positive harm n", "target help mean", "positive harm mean", "diff", "score"],
            sep_table,
        ),
        "",
        "## Top Protected History Selectors",
        "",
        markdown_table(
            [
                "seg",
                "family",
                "candidate",
                "protected",
                "pos regress",
                "worst 140-EKF2",
                "worst 160-EKF2",
                "mean 140 improve",
                "strict",
            ],
            table_candidate(protected_top[:14]),
        ),
        "",
        "## Top Relaxed History Selectors",
        "",
        markdown_table(
            [
                "seg",
                "family",
                "candidate",
                "protected",
                "pos regress",
                "worst 140-EKF2",
                "worst 160-EKF2",
                "mean 140 improve",
                "strict",
            ],
            table_candidate(relaxed_top[:14]),
        ),
        "",
        "## Readout",
        "",
    ]
    if strict:
        best = strict[0]
        lines.append(
            f"- A strict-pass history proxy exists: `{best['candidate']}` at `{fmt(best['segment_sec'], 1)}` s."
        )
    else:
        lines.append(
            "- No tested update-history proxy passes both positive protection and all shortgen11 140-180 target repair."
        )
    lines.extend(
        [
            "- This test uses full accepted GNSS/state-update history, so a failure here is stronger than the prior latest-row proxy failure.",
            "",
            "Generated files:",
            f"- `{out_dir / 'history_proxy_row_observables.csv'}`",
            f"- `{out_dir / 'history_proxy_segment_observables.csv'}`",
            f"- `{out_dir / 'history_proxy_feature_separation.csv'}`",
            f"- `{out_dir / 'history_proxy_candidate_summary.csv'}`",
        ]
    )
    (out_dir / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out-dir", default=str(DEFAULT_OUT))
    args = parser.parse_args()

    out_dir = Path(args.out_dir)
    row_observables = build_row_observables()
    candidates = build_candidates()

    all_segments: list[dict[str, object]] = []
    candidate_rows: list[dict[str, object]] = []
    for segment_sec in SEGMENT_LENGTHS:
        segments = segment_rows(row_observables, segment_sec)
        all_segments.extend(segments)
        for candidate in candidates:
            row = score_candidate(segments, candidate)
            row["segment_sec"] = segment_sec
            candidate_rows.append(row)

    separation_rows = feature_separation([row for row in all_segments if to_float(row.get("segment_sec")) == 2.0])
    write_csv(out_dir / "history_proxy_row_observables.csv", row_observables)
    write_csv(out_dir / "history_proxy_segment_observables.csv", all_segments)
    write_csv(out_dir / "history_proxy_feature_separation.csv", separation_rows)
    write_csv(out_dir / "history_proxy_candidate_summary.csv", candidate_rows)
    write_report(out_dir, separation_rows, candidate_rows)
    print(f"wrote: {out_dir / 'report.md'}")


if __name__ == "__main__":
    main()
