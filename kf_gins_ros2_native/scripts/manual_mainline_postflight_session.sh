#!/usr/bin/env bash

set -euo pipefail

CMD="${1:-}"
RUN_REF="${2:-manual_mainline_15_postflight_idle}"

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd -- "${SCRIPT_DIR}/../../.." && pwd)"
BRINGUP_SCRIPT="${SCRIPT_DIR}/px4_interactive_safe_bringup.sh"
DEFAULT_RUN_NAME="manual_mainline_15_postflight_idle"

usage() {
  cat <<EOF
usage: $(basename "$0") {start|status|stop|summary|show-env} [run_name|/abs/run_dir]

Defaults:
  run_name: ${DEFAULT_RUN_NAME}
  run_dir:  ${WS_ROOT}/artifacts/manual/${DEFAULT_RUN_NAME}
EOF
}

resolve_run_dir() {
  local run_ref="${1:-${DEFAULT_RUN_NAME}}"

  if [[ -z "${run_ref}" ]]; then
    run_ref="${DEFAULT_RUN_NAME}"
  fi

  if [[ "${run_ref}" == /* ]]; then
    printf '%s\n' "${run_ref}"
  else
    printf '%s\n' "${WS_ROOT}/artifacts/manual/${run_ref}"
  fi
}

csv_rows() {
  local path="$1"

  if [[ ! -s "${path}" ]]; then
    printf '0\n'
    return
  fi

  local total_lines
  total_lines="$(wc -l < "${path}")"
  if (( total_lines > 0 )); then
    printf '%s\n' "$((total_lines - 1))"
  else
    printf '0\n'
  fi
}

show_env() {
  cat <<EOF
RUN_DIR=${RUN_DIR}
ROS_DOMAIN_ID=${ROS_DOMAIN_ID}
PX4_SAFE_PX4_WORLD=${PX4_SAFE_PX4_WORLD}
PX4_SAFE_START_MAVROS=${PX4_SAFE_START_MAVROS}
PX4_SAFE_START_QGC=${PX4_SAFE_START_QGC}
PX4_SAFE_START_MONITOR=${PX4_SAFE_START_MONITOR}
PX4_SAFE_START_COMPARE=${PX4_SAFE_START_COMPARE}
PX4_SAFE_START_GPS_PROBES=${PX4_SAFE_START_GPS_PROBES}
PX4_SAFE_START_PLOTJUGGLER=${PX4_SAFE_START_PLOTJUGGLER}
PX4_SAFE_HEADLESS=${PX4_SAFE_HEADLESS}
PX4_SAFE_NO_PXH=${PX4_SAFE_NO_PXH}
PX4_SAFE_MAVROS_PROFILE=${PX4_SAFE_MAVROS_PROFILE}
PX4_SAFE_GCS_LINK_PROFILE=${PX4_SAFE_GCS_LINK_PROFILE}
PX4_SAFE_GCS_RATE_BPS=${PX4_SAFE_GCS_RATE_BPS}
PX4_SAFE_GCS_POSITION_RATE_HZ=${PX4_SAFE_GCS_POSITION_RATE_HZ}
PX4_SAFE_GCS_ATTITUDE_RATE_HZ=${PX4_SAFE_GCS_ATTITUDE_RATE_HZ}
PX4_SAFE_GCS_TARGET_RATE_HZ=${PX4_SAFE_GCS_TARGET_RATE_HZ}
PX4_SAFE_GCS_SERVO_RATE_HZ=${PX4_SAFE_GCS_SERVO_RATE_HZ}
PX4_SAFE_GCS_RC_RATE_HZ=${PX4_SAFE_GCS_RC_RATE_HZ}
PX4_SAFE_GCS_FLOW_RATE_HZ=${PX4_SAFE_GCS_FLOW_RATE_HZ}
PX4_SAFE_COMPARE_CSV_PATH=${PX4_SAFE_COMPARE_CSV_PATH}
PX4_SAFE_COMPARE_GNSS_UPDATE_DEBUG_CSV_PATH=${PX4_SAFE_COMPARE_GNSS_UPDATE_DEBUG_CSV_PATH}
PX4_SAFE_COMPARE_HEADING_UPDATE_DEBUG_CSV_PATH=${PX4_SAFE_COMPARE_HEADING_UPDATE_DEBUG_CSV_PATH}
PX4_SAFE_COMPARE_STATE_PUBLISH_DEBUG_CSV_PATH=${PX4_SAFE_COMPARE_STATE_PUBLISH_DEBUG_CSV_PATH}
PX4_SAFE_GPS_PROBE_CSV_PATH=${PX4_SAFE_GPS_PROBE_CSV_PATH}
PX4_SAFE_GPS_PROBE_SECOND_CSV_PATH=${PX4_SAFE_GPS_PROBE_SECOND_CSV_PATH}
PX4_SAFE_GPS_PROBE_PAIR_AFTER_NEWER_POSE=${PX4_SAFE_GPS_PROBE_PAIR_AFTER_NEWER_POSE}
PX4_SAFE_GPS_PROBE_MAX_PAIR_WAIT_SEC=${PX4_SAFE_GPS_PROBE_MAX_PAIR_WAIT_SEC}
EOF
}

summarize_run() {
  local run_dir="$1"
  local gnss_csv="${run_dir}/gnss_update_debug.csv"
  local heading_csv="${run_dir}/heading_update_debug.csv"
  local state_publish_csv="${run_dir}/state_publish_debug.csv"
  local compare_csv="${run_dir}/comparison_metrics.csv"
  local gps_pose_csv="${run_dir}/gps_vs_pose.csv"
  local gps_raw_csv="${run_dir}/gps_vs_second_pose.csv"

  printf 'run_dir=%s\n' "${run_dir}"
  printf 'comparison_metrics_csv_present=%s\n' "$([[ -s "${compare_csv}" ]] && echo 1 || echo 0)"
  printf 'gnss_update_debug_csv_present=%s\n' "$([[ -s "${gnss_csv}" ]] && echo 1 || echo 0)"
  printf 'heading_update_debug_csv_present=%s\n' "$([[ -s "${heading_csv}" ]] && echo 1 || echo 0)"
  printf 'state_publish_debug_csv_present=%s\n' "$([[ -s "${state_publish_csv}" ]] && echo 1 || echo 0)"
  printf 'gps_vs_pose_csv_present=%s\n' "$([[ -s "${gps_pose_csv}" ]] && echo 1 || echo 0)"
  printf 'gps_vs_second_pose_csv_present=%s\n' "$([[ -s "${gps_raw_csv}" ]] && echo 1 || echo 0)"
  printf 'comparison_metrics_rows=%s\n' "$(csv_rows "${compare_csv}")"
  printf 'heading_update_debug_rows=%s\n' "$(csv_rows "${heading_csv}")"
  printf 'state_publish_debug_rows=%s\n' "$(csv_rows "${state_publish_csv}")"
  printf 'gps_vs_pose_rows=%s\n' "$(csv_rows "${gps_pose_csv}")"
  printf 'gps_vs_second_pose_rows=%s\n' "$(csv_rows "${gps_raw_csv}")"

  if [[ -s "${gps_raw_csv}" ]]; then
    awk -F, '
      function get(name,    idx) {
        idx = header[name]
        if (idx == 0) {
          return ""
        }
        return $(idx)
      }

      function finite(value) {
        return value != "" && value != "nan" && value != "-nan" && value != "inf" && value != "-inf"
      }

      NR == 1 {
        for (i = 1; i <= NF; ++i) {
          header[$i] = i
        }
        has_pair_diag = header["pose_waited_for_newer_stamp"] != 0
        next
      }

      has_pair_diag {
        rows += 1
        if ((get("pose_waited_for_newer_stamp") + 0) != 0) {
          waited_rows += 1
        }
        if (finite(get("pending_pair_age_sec"))) {
          pending_age_sum += get("pending_pair_age_sec")
          pending_age_count += 1
        }
        if (finite(get("same_stamp_pose_count"))) {
          same_stamp_count_sum += get("same_stamp_pose_count")
          same_stamp_count_count += 1
        }
        if (finite(get("pose_buffer_latest_stamp_sec")) && finite(get("gps_stamp_sec"))) {
          latest_minus_gps_sum += get("pose_buffer_latest_stamp_sec") - get("gps_stamp_sec")
          latest_minus_gps_count += 1
        }
      }

      END {
        if (has_pair_diag) {
          print "gps_vs_second_pose_pair_diag_rows=" (rows + 0)
          print "gps_vs_second_pose_pair_waited_for_newer_stamp_rows=" (waited_rows + 0)
          if (pending_age_count > 0) {
            printf "gps_vs_second_pose_pending_pair_age_mean_sec=%.4f\n", pending_age_sum / pending_age_count
          } else {
            print "gps_vs_second_pose_pending_pair_age_mean_sec="
          }
          if (same_stamp_count_count > 0) {
            printf "gps_vs_second_pose_same_stamp_pose_count_mean=%.2f\n", same_stamp_count_sum / same_stamp_count_count
          } else {
            print "gps_vs_second_pose_same_stamp_pose_count_mean="
          }
          if (latest_minus_gps_count > 0) {
            printf "gps_vs_second_pose_latest_pose_minus_gps_mean_sec=%.4f\n", latest_minus_gps_sum / latest_minus_gps_count
          } else {
            print "gps_vs_second_pose_latest_pose_minus_gps_mean_sec="
          }
        }
      }
    ' "${gps_raw_csv}"
  fi

  if [[ -s "${state_publish_csv}" ]]; then
    awk -F, '
      function get(name,    idx) {
        idx = header[name]
        if (idx == 0) {
          return ""
        }
        return $(idx)
      }

      function finite(value) {
        return value != "" && value != "nan" && value != "-nan" && value != "inf" && value != "-inf"
      }

      function add(name, value) {
        if (finite(value)) {
          sum[name] += value
          count[name] += 1
        }
      }

      NR == 1 {
        for (i = 1; i <= NF; ++i) {
          header[$i] = i
        }
        next
      }

      {
        rows += 1
        if ((get("mavros_armed") + 0) != 0) {
          armed_rows += 1
          add("armed_core_gnss_diff_h_m", get("core_gnss_diff_h_m"))
        }
        if ((get("odom_uses_gnss_pose") + 0) != 0) {
          odom_uses_gnss_pose_rows += 1
        }
        add("core_time_minus_ros_sec", get("core_time_minus_ros_sec"))
        add("gnss_update_time_minus_ros_sec", get("gnss_update_time_minus_ros_sec"))
        add("gnss_source_time_minus_ros_sec", get("gnss_source_time_minus_ros_sec"))
      }

      END {
        print "state_publish_debug_rows_seen=" (rows + 0)
        print "state_publish_debug_armed_rows=" (armed_rows + 0)
        print "state_publish_debug_odom_uses_gnss_pose_rows=" (odom_uses_gnss_pose_rows + 0)
        if (count["core_time_minus_ros_sec"] > 0) {
          printf "state_publish_debug_core_time_minus_ros_mean_sec=%.6f\n", sum["core_time_minus_ros_sec"] / count["core_time_minus_ros_sec"]
        } else {
          print "state_publish_debug_core_time_minus_ros_mean_sec="
        }
        if (count["gnss_update_time_minus_ros_sec"] > 0) {
          printf "state_publish_debug_gnss_update_time_minus_ros_mean_sec=%.6f\n", sum["gnss_update_time_minus_ros_sec"] / count["gnss_update_time_minus_ros_sec"]
        } else {
          print "state_publish_debug_gnss_update_time_minus_ros_mean_sec="
        }
        if (count["gnss_source_time_minus_ros_sec"] > 0) {
          printf "state_publish_debug_gnss_source_time_minus_ros_mean_sec=%.6f\n", sum["gnss_source_time_minus_ros_sec"] / count["gnss_source_time_minus_ros_sec"]
        } else {
          print "state_publish_debug_gnss_source_time_minus_ros_mean_sec="
        }
        if (count["armed_core_gnss_diff_h_m"] > 0) {
          printf "state_publish_debug_armed_core_gnss_diff_h_mean_m=%.4f\n", sum["armed_core_gnss_diff_h_m"] / count["armed_core_gnss_diff_h_m"]
        } else {
          print "state_publish_debug_armed_core_gnss_diff_h_mean_m="
        }
      }
    ' "${state_publish_csv}"
  fi

  if [[ -s "${heading_csv}" ]]; then
    awk -F, '
      function get(name,    idx) {
        idx = header[name]
        if (idx == 0) {
          return ""
        }
        return $(idx)
      }

      function truthy(value) {
        return value != "" && value != "nan" && value != "-nan" && (value + 0) != 0
      }

      NR == 1 {
        for (i = 1; i <= NF; ++i) {
          header[$i] = i
        }
        next
      }

      {
        event_type = get("event_type")
        heading_mode = get("heading_mode")
        if (event_type == "ingest") {
          ingest_rows += 1
          if (heading_mode == "post_turn_cruise_track") {
            post_turn_cruise_track_ingest_rows += 1
          }
          if (heading_mode == "armed_cruise_track") {
            armed_cruise_track_ingest_rows += 1
            if (truthy(get("post_turn_window_active_before"))) {
              armed_cruise_track_window_before_rows += 1
            }
            if (truthy(get("post_turn_followthrough_active_before"))) {
              armed_cruise_track_followthrough_before_rows += 1
            }
            if (truthy(get("post_turn_context_active_before"))) {
              armed_cruise_track_context_before_rows += 1
            }
          }
          if (truthy(get("used_post_turn_cruise_track"))) {
            used_post_turn_cruise_track_rows += 1
          }
          if (truthy(get("post_turn_low_hspeed_cluster_ok"))) {
            post_turn_low_hspeed_cluster_rows += 1
          }
          if (truthy(get("used_armed_cruise_track"))) {
            used_armed_cruise_track_rows += 1
          }
          if (truthy(get("heading_update_underreacted"))) {
            heading_update_underreacted_rows += 1
          }
        } else if (event_type == "underreaction_force_relock") {
          underreaction_force_relock_rows += 1
        }
      }

      END {
        print "heading_update_debug_ingest_rows=" (ingest_rows + 0)
        print "heading_update_debug_post_turn_cruise_track_ingest_rows=" (post_turn_cruise_track_ingest_rows + 0)
        print "heading_update_debug_armed_cruise_track_ingest_rows=" (armed_cruise_track_ingest_rows + 0)
        print "heading_update_debug_armed_cruise_track_window_before_rows=" (armed_cruise_track_window_before_rows + 0)
        print "heading_update_debug_armed_cruise_track_followthrough_before_rows=" (armed_cruise_track_followthrough_before_rows + 0)
        print "heading_update_debug_armed_cruise_track_context_before_rows=" (armed_cruise_track_context_before_rows + 0)
        print "heading_update_debug_used_post_turn_cruise_track_rows=" (used_post_turn_cruise_track_rows + 0)
        print "heading_update_debug_post_turn_low_hspeed_cluster_rows=" (post_turn_low_hspeed_cluster_rows + 0)
        print "heading_update_debug_used_armed_cruise_track_rows=" (used_armed_cruise_track_rows + 0)
        print "heading_update_debug_heading_update_underreacted_rows=" (heading_update_underreacted_rows + 0)
        print "heading_update_debug_underreaction_force_relock_rows=" (underreaction_force_relock_rows + 0)
      }
    ' "${heading_csv}"
  fi

  if [[ ! -s "${gnss_csv}" ]]; then
    printf 'summary_error=missing_gnss_update_debug_csv\n'
    return 1
  fi

  awk -F, '
    function get(name,    idx) {
      idx = header[name]
      if (idx == 0) {
        return ""
      }
      return $(idx)
    }

    function is_nan(value) {
      return value == "" || value == "nan" || value == "-nan" || value == "NaN" || value == "-NaN"
    }

    function abs_num(value) {
      return (value < 0.0) ? -value : value
    }

    NR == 1 {
      for (i = 1; i <= NF; ++i) {
        header[$i] = i
      }
      have_postflight_active = ("post_flight_vertical_cov_reopen_active" in header)
      have_postflight_applied = ("post_flight_vertical_cov_reopen_applied" in header)
      next
    }

    {
      armed_raw = get("mavros_armed")
      ros_raw = get("ros_time_sec")
      seq_raw = get("sequence")

      if (is_nan(armed_raw) || is_nan(ros_raw) || is_nan(seq_raw)) {
        next
      }

      armed = armed_raw + 0
      ros = ros_raw + 0
      seq = seq_raw + 0

      if (!have_prev_armed) {
        prev_armed = armed
        have_prev_armed = 1
      }

      if (armed != prev_armed) {
        if (!have_arm_transition && prev_armed == 0 && armed == 1) {
          have_arm_transition = 1
          arm_transition_seq = seq
          arm_transition_ros = ros
        }
        if (!have_disarm_transition && prev_armed == 1 && armed == 0) {
          have_disarm_transition = 1
          disarm_transition_seq = seq
          disarm_transition_ros = ros
        }
        prev_armed = armed
      }

      if (have_disarm_transition && ros >= disarm_transition_ros) {
        postflight_rows += 1
        last_postflight_seq = seq
        last_postflight_ros = ros

        core_gnss_diff_u = get("core_gnss_diff_u_m")
        if (!is_nan(core_gnss_diff_u)) {
          postflight_core_gnss_diff_u_sum += abs_num(core_gnss_diff_u + 0.0)
          postflight_core_gnss_diff_u_count += 1
        }

        last_pos_residual_u = get("last_position_residual_u_m")
        if (!is_nan(last_pos_residual_u)) {
          postflight_last_pos_residual_u_sum += abs_num(last_pos_residual_u + 0.0)
          postflight_last_pos_residual_u_count += 1
        }

        core_pos_std_d = get("core_pos_std_d_m")
        if (!is_nan(core_pos_std_d)) {
          postflight_core_pos_std_d_sum += core_pos_std_d + 0.0
          postflight_core_pos_std_d_count += 1
        }

        vertical_active = get("vertical_cov_reopen_active")
        if (!is_nan(vertical_active) && (vertical_active + 0) != 0) {
          combined_vertical_active_rows += 1
        }

        vertical_applied = get("vertical_cov_reopen_applied")
        if (!is_nan(vertical_applied) && (vertical_applied + 0) != 0) {
          combined_vertical_applied_rows += 1
        }

        if (have_postflight_active) {
          postflight_active = get("post_flight_vertical_cov_reopen_active")
          if (!is_nan(postflight_active) && (postflight_active + 0) != 0) {
            postflight_active_rows += 1
          }
        }

        if (have_postflight_applied) {
          postflight_applied = get("post_flight_vertical_cov_reopen_applied")
          if (!is_nan(postflight_applied) && (postflight_applied + 0) != 0) {
            postflight_applied_rows += 1
          }
        }
      }
    }

    END {
      print "gnss_update_debug_rows=" NR - 1
      print "armed_transition_present=" (have_arm_transition ? 1 : 0)
      if (have_arm_transition) {
        printf("armed_transition_seq=%d\n", arm_transition_seq)
        printf("armed_transition_ros_time_sec=%.3f\n", arm_transition_ros)
      } else {
        print "armed_transition_seq="
        print "armed_transition_ros_time_sec="
      }

      print "disarm_transition_present=" (have_disarm_transition ? 1 : 0)
      if (have_disarm_transition) {
        printf("disarm_transition_seq=%d\n", disarm_transition_seq)
        printf("disarm_transition_ros_time_sec=%.3f\n", disarm_transition_ros)
      } else {
        print "disarm_transition_seq="
        print "disarm_transition_ros_time_sec="
      }

      print "postflight_rows=" (postflight_rows + 0)
      if (postflight_rows > 0) {
        printf("last_postflight_seq=%d\n", last_postflight_seq)
        printf("last_postflight_ros_time_sec=%.3f\n", last_postflight_ros)
      } else {
        print "last_postflight_seq="
        print "last_postflight_ros_time_sec="
      }

      print "post_flight_vertical_cov_reopen_columns_present=" ((have_postflight_active && have_postflight_applied) ? 1 : 0)
      print "post_flight_vertical_cov_reopen_active_rows=" (postflight_active_rows + 0)
      print "post_flight_vertical_cov_reopen_applied_rows=" (postflight_applied_rows + 0)
      print "combined_vertical_cov_reopen_active_rows=" (combined_vertical_active_rows + 0)
      print "combined_vertical_cov_reopen_applied_rows=" (combined_vertical_applied_rows + 0)

      if (postflight_core_gnss_diff_u_count > 0) {
        printf("postflight_mean_abs_core_gnss_diff_u_m=%.4f\n", postflight_core_gnss_diff_u_sum / postflight_core_gnss_diff_u_count)
      } else {
        print "postflight_mean_abs_core_gnss_diff_u_m="
      }

      if (postflight_last_pos_residual_u_count > 0) {
        printf("postflight_mean_abs_last_pos_residual_u_m=%.4f\n", postflight_last_pos_residual_u_sum / postflight_last_pos_residual_u_count)
      } else {
        print "postflight_mean_abs_last_pos_residual_u_m="
      }

      if (postflight_core_pos_std_d_count > 0) {
        printf("postflight_mean_core_pos_std_d_m=%.4f\n", postflight_core_pos_std_d_sum / postflight_core_pos_std_d_count)
      } else {
        print "postflight_mean_core_pos_std_d_m="
      }

      print "manual14_baseline_postflight_mean_abs_core_gnss_diff_u_m=0.2517"
      print "manual14_baseline_postflight_mean_abs_last_pos_residual_u_m=0.2581"
      print "manual14_baseline_postflight_mean_core_pos_std_d_m=0.0380"
    }
  ' "${gnss_csv}"

  if [[ -s "${gps_raw_csv}" ]]; then
    python3 - "${gnss_csv}" "${gps_raw_csv}" <<'PY'
import bisect
import csv
import math
import sys


def as_float(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return math.nan


def truthy(value):
    return str(value).strip().lower() in ("1", "true", "yes")


def mean(values):
    vals = [v for v in values if math.isfinite(v)]
    if not vals:
        return math.nan
    return sum(vals) / len(vals)


def nearest(rows, times, target):
    index = bisect.bisect_left(times, target)
    candidates = []
    if index < len(rows):
        candidates.append((abs(times[index] - target), rows[index]))
    if index > 0:
        candidates.append((abs(times[index - 1] - target), rows[index - 1]))
    if not candidates:
        return None, math.nan
    return min(candidates, key=lambda item: item[0])


gnss_path, gps_raw_path = sys.argv[1], sys.argv[2]
with open(gnss_path, newline="") as handle:
    gnss_rows = list(csv.DictReader(handle))
gnss_rows.sort(key=lambda row: as_float(row.get("ros_time_sec")))
gnss_times = [as_float(row.get("ros_time_sec")) for row in gnss_rows]

groups = {
    "armed": lambda g: truthy(g.get("mavros_armed")),
    "turning": lambda g: truthy(g.get("mavros_armed")) and truthy(g.get("turning_now")),
    "post_only": lambda g: (
        truthy(g.get("mavros_armed"))
        and truthy(g.get("post_turn_context"))
        and not truthy(g.get("turning_now"))
    ),
    "turn_or_post": lambda g: (
        truthy(g.get("mavros_armed"))
        and (truthy(g.get("turning_now")) or truthy(g.get("post_turn_context")))
    ),
    "cruise": lambda g: truthy(g.get("mavros_armed")) and truthy(g.get("armed_cruise_context")),
    "low_speed": lambda g: (
        truthy(g.get("mavros_armed"))
        and math.isfinite(as_float(g.get("horizontal_speed_mps")))
        and as_float(g.get("horizontal_speed_mps")) < 1.0
    ),
    "high_vertical": lambda g: (
        truthy(g.get("mavros_armed"))
        and math.isfinite(as_float(g.get("vertical_speed_mps")))
        and abs(as_float(g.get("vertical_speed_mps"))) > 0.6
    ),
}
stats = {
    name: {
        "rows": 0,
        "local_xy": [],
        "local_abs_z": [],
        "core_gnss_diff_h": [],
        "core_gnss_abs_u": [],
        "gnss_residual_xy": [],
        "gnss_residual_local6_xy": [],
        "raw_probe_residual_lag_sec": [],
    }
    for name in groups
}
join_dts = []
residual_window = []

with open(gps_raw_path, newline="") as handle:
    for row in csv.DictReader(handle):
        t = as_float(row.get("gps_stamp_sec"))
        if not math.isfinite(t):
            t = as_float(row.get("pose_stamp_sec"))
        if not math.isfinite(t):
            t = as_float(row.get("ros_time_sec"))
        if not math.isfinite(t):
            continue
        dt, match = nearest(gnss_rows, gnss_times, t)
        if match is None:
            continue
        if math.isfinite(dt):
            join_dts.append(dt)
        residual_e_m = as_float(match.get("gnss_position_residual_e_m"))
        residual_n_m = as_float(match.get("gnss_position_residual_n_m"))
        residual_xy_m = math.nan
        residual_local6_xy_m = math.nan
        if math.isfinite(residual_e_m) and math.isfinite(residual_n_m):
            residual_xy_m = math.hypot(residual_e_m, residual_n_m)
            residual_window.append((t, residual_e_m, residual_n_m))
            while len(residual_window) > 1 and (t - residual_window[0][0]) > 6.0:
                residual_window.pop(0)
            anchor = residual_window[0]
            residual_local6_xy_m = math.hypot(
                residual_e_m - anchor[1],
                residual_n_m - anchor[2],
            )
        raw_probe_residual_lag_sec = math.nan
        probe_error_e_m = as_float(row.get("error_x_m"))
        probe_error_n_m = as_float(row.get("error_y_m"))
        velocity_e_mps = as_float(match.get("native_velocity_vE_mps"))
        velocity_n_mps = as_float(match.get("native_velocity_vN_mps"))
        speed_sq_mps2 = velocity_e_mps * velocity_e_mps + velocity_n_mps * velocity_n_mps
        if (
            math.isfinite(probe_error_e_m)
            and math.isfinite(probe_error_n_m)
            and math.isfinite(residual_e_m)
            and math.isfinite(residual_n_m)
            and math.isfinite(speed_sq_mps2)
            and speed_sq_mps2 > 1.0
        ):
            delta_e_m = probe_error_e_m - residual_e_m
            delta_n_m = probe_error_n_m - residual_n_m
            raw_probe_residual_lag_sec = -(
                delta_e_m * velocity_e_mps + delta_n_m * velocity_n_mps
            ) / speed_sq_mps2
        for name, predicate in groups.items():
            if not predicate(match):
                continue
            stats[name]["rows"] += 1
            stats[name]["local_xy"].append(as_float(row.get("local_xy_error_m")))
            stats[name]["local_abs_z"].append(abs(as_float(row.get("local_error_z_m"))))
            stats[name]["core_gnss_diff_h"].append(as_float(match.get("core_gnss_diff_h_m")))
            stats[name]["core_gnss_abs_u"].append(abs(as_float(match.get("core_gnss_diff_u_m"))))
            stats[name]["gnss_residual_xy"].append(residual_xy_m)
            stats[name]["gnss_residual_local6_xy"].append(residual_local6_xy_m)
            stats[name]["raw_probe_residual_lag_sec"].append(raw_probe_residual_lag_sec)

if join_dts:
    print(f"phase_metric_join_max_dt_sec={max(join_dts):.4f}")
else:
    print("phase_metric_join_max_dt_sec=")

for name in ("armed", "turning", "post_only", "turn_or_post", "cruise", "low_speed", "high_vertical"):
    item = stats[name]
    print(f"phase_metric_{name}_rows={item['rows']}")
    for key, values in (
        ("local_xy_mean_m", item["local_xy"]),
        ("local_abs_z_mean_m", item["local_abs_z"]),
        ("core_gnss_diff_h_mean_m", item["core_gnss_diff_h"]),
        ("core_gnss_abs_u_mean_m", item["core_gnss_abs_u"]),
        ("gnss_residual_xy_mean_m", item["gnss_residual_xy"]),
        ("gnss_residual_local6_xy_mean_m", item["gnss_residual_local6_xy"]),
        ("raw_probe_residual_lag_mean_sec", item["raw_probe_residual_lag_sec"]),
    ):
        value = mean(values)
        if math.isfinite(value):
            print(f"phase_metric_{name}_{key}={value:.4f}")
        else:
            print(f"phase_metric_{name}_{key}=")
PY
  fi
}

RUN_DIR="$(resolve_run_dir "${RUN_REF}")"

export RUN_DIR
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export PX4_SAFE_PX4_WORLD="${PX4_SAFE_PX4_WORLD:-none}"
export PX4_SAFE_START_MAVROS="${PX4_SAFE_START_MAVROS:-1}"
export PX4_SAFE_START_QGC="${PX4_SAFE_START_QGC:-1}"
export PX4_SAFE_START_MONITOR="${PX4_SAFE_START_MONITOR:-0}"
export PX4_SAFE_START_COMPARE="${PX4_SAFE_START_COMPARE:-1}"
export PX4_SAFE_START_GPS_PROBES="${PX4_SAFE_START_GPS_PROBES:-1}"
export PX4_SAFE_START_PLOTJUGGLER="${PX4_SAFE_START_PLOTJUGGLER:-0}"
export PX4_SAFE_HEADLESS="${PX4_SAFE_HEADLESS:-1}"
export PX4_SAFE_NO_PXH="${PX4_SAFE_NO_PXH:-1}"
export PX4_SAFE_MAVROS_PROFILE="${PX4_SAFE_MAVROS_PROFILE:-runtime_minimal}"
export PX4_SAFE_GCS_LINK_PROFILE="${PX4_SAFE_GCS_LINK_PROFILE:-light}"
export PX4_SAFE_GCS_RATE_BPS="${PX4_SAFE_GCS_RATE_BPS:-500000}"
export PX4_SAFE_GCS_POSITION_RATE_HZ="${PX4_SAFE_GCS_POSITION_RATE_HZ:-6}"
export PX4_SAFE_GCS_ATTITUDE_RATE_HZ="${PX4_SAFE_GCS_ATTITUDE_RATE_HZ:-8}"
export PX4_SAFE_GCS_TARGET_RATE_HZ="${PX4_SAFE_GCS_TARGET_RATE_HZ:-5}"
export PX4_SAFE_GCS_SERVO_RATE_HZ="${PX4_SAFE_GCS_SERVO_RATE_HZ:-0}"
export PX4_SAFE_GCS_RC_RATE_HZ="${PX4_SAFE_GCS_RC_RATE_HZ:-0}"
export PX4_SAFE_GCS_FLOW_RATE_HZ="${PX4_SAFE_GCS_FLOW_RATE_HZ:-0}"
export PX4_SAFE_COMPARE_CSV_PATH="${PX4_SAFE_COMPARE_CSV_PATH:-${RUN_DIR}/comparison_metrics.csv}"
export PX4_SAFE_COMPARE_GNSS_UPDATE_DEBUG_CSV_PATH="${PX4_SAFE_COMPARE_GNSS_UPDATE_DEBUG_CSV_PATH:-${RUN_DIR}/gnss_update_debug.csv}"
export PX4_SAFE_COMPARE_HEADING_UPDATE_DEBUG_CSV_PATH="${PX4_SAFE_COMPARE_HEADING_UPDATE_DEBUG_CSV_PATH:-${RUN_DIR}/heading_update_debug.csv}"
export PX4_SAFE_COMPARE_STATE_PUBLISH_DEBUG_CSV_PATH="${PX4_SAFE_COMPARE_STATE_PUBLISH_DEBUG_CSV_PATH:-${RUN_DIR}/state_publish_debug.csv}"
export PX4_SAFE_GPS_PROBE_CSV_PATH="${PX4_SAFE_GPS_PROBE_CSV_PATH:-${RUN_DIR}/gps_vs_pose.csv}"
export PX4_SAFE_GPS_PROBE_SECOND_CSV_PATH="${PX4_SAFE_GPS_PROBE_SECOND_CSV_PATH:-${RUN_DIR}/gps_vs_second_pose.csv}"
export PX4_SAFE_GPS_PROBE_PAIR_AFTER_NEWER_POSE="${PX4_SAFE_GPS_PROBE_PAIR_AFTER_NEWER_POSE:-true}"
export PX4_SAFE_GPS_PROBE_MAX_PAIR_WAIT_SEC="${PX4_SAFE_GPS_PROBE_MAX_PAIR_WAIT_SEC:-0.25}"

case "${CMD}" in
  start)
    mkdir -p "${RUN_DIR}"
    printf '[manual-postflight] run_dir=%s\n' "${RUN_DIR}"
    printf '[manual-postflight] keep the session idle for 30-60s after landing/disarm before stop\n'
    exec "${BRINGUP_SCRIPT}" start "${RUN_DIR}"
    ;;
  status)
    exec "${BRINGUP_SCRIPT}" status "${RUN_DIR}"
    ;;
  stop)
    exec "${BRINGUP_SCRIPT}" stop "${RUN_DIR}"
    ;;
  summary)
    summarize_run "${RUN_DIR}"
    ;;
  show-env)
    show_env
    ;;
  *)
    usage >&2
    exit 1
    ;;
esac
