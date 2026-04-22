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
PX4_SAFE_GPS_PROBE_CSV_PATH=${PX4_SAFE_GPS_PROBE_CSV_PATH}
PX4_SAFE_GPS_PROBE_SECOND_CSV_PATH=${PX4_SAFE_GPS_PROBE_SECOND_CSV_PATH}
EOF
}

summarize_run() {
  local run_dir="$1"
  local gnss_csv="${run_dir}/gnss_update_debug.csv"
  local compare_csv="${run_dir}/comparison_metrics.csv"
  local gps_pose_csv="${run_dir}/gps_vs_pose.csv"
  local gps_raw_csv="${run_dir}/gps_vs_second_pose.csv"

  printf 'run_dir=%s\n' "${run_dir}"
  printf 'comparison_metrics_csv_present=%s\n' "$([[ -s "${compare_csv}" ]] && echo 1 || echo 0)"
  printf 'gnss_update_debug_csv_present=%s\n' "$([[ -s "${gnss_csv}" ]] && echo 1 || echo 0)"
  printf 'gps_vs_pose_csv_present=%s\n' "$([[ -s "${gps_pose_csv}" ]] && echo 1 || echo 0)"
  printf 'gps_vs_second_pose_csv_present=%s\n' "$([[ -s "${gps_raw_csv}" ]] && echo 1 || echo 0)"
  printf 'comparison_metrics_rows=%s\n' "$(csv_rows "${compare_csv}")"
  printf 'gps_vs_pose_rows=%s\n' "$(csv_rows "${gps_pose_csv}")"
  printf 'gps_vs_second_pose_rows=%s\n' "$(csv_rows "${gps_raw_csv}")"

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
export PX4_SAFE_GPS_PROBE_CSV_PATH="${PX4_SAFE_GPS_PROBE_CSV_PATH:-${RUN_DIR}/gps_vs_pose.csv}"
export PX4_SAFE_GPS_PROBE_SECOND_CSV_PATH="${PX4_SAFE_GPS_PROBE_SECOND_CSV_PATH:-${RUN_DIR}/gps_vs_second_pose.csv}"

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
