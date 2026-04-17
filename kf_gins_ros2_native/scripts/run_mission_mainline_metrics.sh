#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WS_ROOT="$(cd "${PKG_ROOT}/../.." && pwd)"

RUN_DIR=""
BUILD_PACKAGE=1
GAP_PROBE_PROFILE="directdds_relay_chain"
GAP_PROBE_START_AFTER_REACHED="1"
GAP_PROBE_START_DELAY="5"
GAP_PROBE_WINDOW="25"
EXIT_AFTER_GAP_PROBE=1
MISSION_TIMEOUT_SEC="320"
STATUS_WINDOW_START_SEC="145"
STATUS_WINDOW_END_SEC="151"
GPS_TOPIC="/gps/fix"
GPS_VS_POSE_TOPIC="/ekf2/pose_odom"
GPS_VS_SECOND_POSE_TOPIC="/kf_gins/odom_raw"
SMOKE_PRINT_PERIOD_SEC="10"
ROS_DOMAIN_ID_VALUE="${ROS_DOMAIN_ID:-0}"
RAW_GPS_LITE=0
ENABLE_GNSS_UPDATE_DEBUG_CSV=1
GPS_VS_POSE_TOPIC_SET=0
GPS_VS_SECOND_POSE_TOPIC_SET=0
GNSS_DEBUG_CSV_SET=0
declare -a EXTRA_LAUNCH_ARGS=()
declare -a EXTRA_SMOKE_ARGS=()

usage() {
  cat <<'EOF'
usage: run_mission_mainline_metrics.sh [run_dir] [options]

options:
  --skip-build
  --full-mission
  --raw-gps-lite
  --enable-gnss-debug-csv
  --disable-gnss-debug-csv
  --gap-probe-profile <name>
  --gap-probe-start-after-reached <seq>
  --gap-probe-start-delay <sec>
  --gap-probe-window <sec>
  --mission-timeout <sec>
  --status-window-start <sec>
  --status-window-end <sec>
  --gps-topic <topic>
  --gps-vs-pose-topic <topic|none>
  --gps-vs-second-pose-topic <topic|none>
  --launch-arg <name:=value>
  --smoke-arg <arg>
EOF
}

while (($# > 0)); do
  case "$1" in
    --skip-build)
      BUILD_PACKAGE=0
      shift
      ;;
    --full-mission)
      EXIT_AFTER_GAP_PROBE=0
      shift
      ;;
    --raw-gps-lite)
      RAW_GPS_LITE=1
      shift
      ;;
    --enable-gnss-debug-csv)
      ENABLE_GNSS_UPDATE_DEBUG_CSV=1
      GNSS_DEBUG_CSV_SET=1
      shift
      ;;
    --disable-gnss-debug-csv)
      ENABLE_GNSS_UPDATE_DEBUG_CSV=0
      GNSS_DEBUG_CSV_SET=1
      shift
      ;;
    --gap-probe-profile)
      GAP_PROBE_PROFILE="$2"
      shift 2
      ;;
    --gap-probe-start-after-reached)
      GAP_PROBE_START_AFTER_REACHED="$2"
      shift 2
      ;;
    --gap-probe-start-delay)
      GAP_PROBE_START_DELAY="$2"
      shift 2
      ;;
    --gap-probe-window)
      GAP_PROBE_WINDOW="$2"
      shift 2
      ;;
    --mission-timeout)
      MISSION_TIMEOUT_SEC="$2"
      shift 2
      ;;
    --status-window-start)
      STATUS_WINDOW_START_SEC="$2"
      shift 2
      ;;
    --status-window-end)
      STATUS_WINDOW_END_SEC="$2"
      shift 2
      ;;
    --gps-topic)
      GPS_TOPIC="$2"
      shift 2
      ;;
    --gps-vs-pose-topic)
      if [[ "$2" == "none" ]]; then
        GPS_VS_POSE_TOPIC=""
      else
        GPS_VS_POSE_TOPIC="$2"
      fi
      GPS_VS_POSE_TOPIC_SET=1
      shift 2
      ;;
    --gps-vs-second-pose-topic)
      if [[ "$2" == "none" ]]; then
        GPS_VS_SECOND_POSE_TOPIC=""
      else
        GPS_VS_SECOND_POSE_TOPIC="$2"
      fi
      GPS_VS_SECOND_POSE_TOPIC_SET=1
      shift 2
      ;;
    --launch-arg)
      EXTRA_LAUNCH_ARGS+=("$2")
      shift 2
      ;;
    --smoke-arg)
      EXTRA_SMOKE_ARGS+=("$2")
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      if [[ -z "${RUN_DIR}" ]]; then
        RUN_DIR="$1"
        shift
      else
        echo "unknown argument: $1" >&2
        usage >&2
        exit 1
      fi
      ;;
  esac
done

if [[ "${RAW_GPS_LITE}" == "1" ]]; then
  if [[ "${GPS_VS_POSE_TOPIC_SET}" == "0" ]]; then
    GPS_VS_POSE_TOPIC=""
  fi
  if [[ "${GPS_VS_SECOND_POSE_TOPIC_SET}" == "0" ]]; then
    GPS_VS_SECOND_POSE_TOPIC="/kf_gins/odom_raw"
  fi
  if [[ "${GNSS_DEBUG_CSV_SET}" == "0" ]]; then
    ENABLE_GNSS_UPDATE_DEBUG_CSV=0
  fi
  EXTRA_LAUNCH_ARGS+=(
    "enable_real_time_comparison:=false"
    "publish_ekf2_state:=false"
    "publish_iekf_state:=false"
    "publish_aligned_iekf_state:=false"
    "enable_ekf2_path:=false"
    "enable_iekf_path:=false"
    "enable_gt_path:=false"
    "enable_ekf2_relay:=false"
    "enable_iekf_aligned_path:=false"
  )
fi

if [[ -z "${RUN_DIR}" ]]; then
  RUN_DIR="${WS_ROOT}/artifacts/runs/px4_mission_safe_$(date +%Y%m%d_%H%M%S)"
elif [[ "${RUN_DIR}" != /* ]]; then
  RUN_DIR="${WS_ROOT}/artifacts/runs/${RUN_DIR}"
fi

mkdir -p "${RUN_DIR}" "${RUN_DIR}/ros_logs"

BRINGUP_SCRIPT="${PKG_ROOT}/scripts/px4_interactive_safe_bringup.sh"
PROBE_SCRIPT="${PKG_ROOT}/scripts/gps_vs_pose_probe.py"

BUILD_LOG="${RUN_DIR}/build.log"
BRINGUP_LOG="${RUN_DIR}/bringup_start.log"
COMPARE_LOG="${RUN_DIR}/compare.log"
SMOKE_LOG="${RUN_DIR}/smoke.log"
STATUS_FILE="${RUN_DIR}/status_after_smoke.txt"
COMPARISON_CSV="${RUN_DIR}/comparison_metrics.csv"
GNSS_UPDATE_CSV="${RUN_DIR}/gnss_update_debug.csv"
GPS_VS_POSE_CSV="${RUN_DIR}/gps_vs_pose.csv"
GPS_VS_SECOND_CSV="${RUN_DIR}/gps_vs_second_pose.csv"

COMPARE_PID=""
GPS_PROBE_PID=""
GPS_SECOND_PROBE_PID=""
SMOKE_RC=0

log() {
  echo "[run-mission-mainline] $*"
}

source_ros() {
  set +u
  source /opt/ros/humble/setup.bash
  if [[ -f "${WS_ROOT}/install/setup.bash" ]]; then
    source "${WS_ROOT}/install/setup.bash"
  fi
  set -u
}

kill_session_pid() {
  local pid="${1:-}"
  if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
    kill -- -"${pid}" 2>/dev/null || kill "${pid}" 2>/dev/null || true
    sleep 1
    if kill -0 "${pid}" 2>/dev/null; then
      kill -KILL -- -"${pid}" 2>/dev/null || kill -KILL "${pid}" 2>/dev/null || true
      sleep 1
    fi
    wait "${pid}" 2>/dev/null || true
  fi
}

cleanup() {
  local rc=$?
  set +e
  kill_session_pid "${GPS_SECOND_PROBE_PID}"
  kill_session_pid "${GPS_PROBE_PID}"
  kill_session_pid "${COMPARE_PID}"
  "${BRINGUP_SCRIPT}" stop "${RUN_DIR}" >/dev/null 2>&1 || true
  exit "${rc}"
}
trap cleanup EXIT

run_bash_with_ros() {
  local log_file="$1"
  shift
  local -a cmd=("$@")
  local cmd_str
  printf -v cmd_str '%q ' "${cmd[@]}"
  setsid bash -lc "
    set -euo pipefail
    set +u
    source /opt/ros/humble/setup.bash
    if [[ -f '${WS_ROOT}/install/setup.bash' ]]; then
      source '${WS_ROOT}/install/setup.bash'
    fi
    set -u
    export ROS_DOMAIN_ID='${ROS_DOMAIN_ID_VALUE}'
    export ROS_LOG_DIR='${RUN_DIR}/ros_logs'
    unset ROS_LOCALHOST_ONLY
    exec ${cmd_str}
  " > "${log_file}" 2>&1 < /dev/null &
  echo $!
}

write_status_summary() {
  python3 - "${STATUS_FILE}" "${RUN_DIR}" "${COMPARISON_CSV}" "${GNSS_UPDATE_CSV}" "${GPS_VS_POSE_CSV}" "${GPS_VS_SECOND_CSV}" "${STATUS_WINDOW_START_SEC}" "${STATUS_WINDOW_END_SEC}" "${SMOKE_RC}" <<'PY'
import csv
import math
import os
import re
import sys

status_path, run_dir, compare_csv, gnss_csv, gps_csv, gps_second_csv, window_start, window_end, smoke_rc = sys.argv[1:]
window_start = float(window_start)
window_end = float(window_end)
smoke_rc = int(smoke_rc)

px4_log = os.path.join(run_dir, "px4.log")
mavros_log = os.path.join(run_dir, "mavros.log")
smoke_log = os.path.join(run_dir, "smoke.log")

def count_fixed(needle, path):
    if not os.path.exists(path):
        return 0
    count = 0
    with open(path, "r", errors="ignore") as f:
        for line in f:
            count += line.count(needle)
    return count

def load_rows(path):
    if not os.path.exists(path) or os.path.getsize(path) == 0:
        return []
    with open(path, "r", newline="") as f:
        return list(csv.DictReader(f))

def parse_float(row, key):
    try:
        value = float(row.get(key, "nan"))
    except (TypeError, ValueError):
        return float("nan")
    return value

def finite_max(values):
    vals = [v for v in values if math.isfinite(v)]
    return max(vals) if vals else float("nan")

def finite_mean(values):
    vals = [v for v in values if math.isfinite(v)]
    return sum(vals) / len(vals) if vals else float("nan")

def finite_std(values):
    vals = [v for v in values if math.isfinite(v)]
    if not vals:
        return float("nan")
    mean = sum(vals) / len(vals)
    return math.sqrt(sum((v - mean) * (v - mean) for v in vals) / len(vals))

def finite_abs_max(values):
    vals = [abs(v) for v in values if math.isfinite(v)]
    return max(vals) if vals else float("nan")

def finite_last(rows, key):
    for row in reversed(rows):
        value = parse_float(row, key)
        if math.isfinite(value):
            return value
    return float("nan")

def rebased_xy_stats(rows):
    if not rows:
        return {
            "max": float("nan"),
            "last": float("nan"),
            "mean": float("nan"),
            "std": float("nan"),
        }
    first_ex = parse_float(rows[0], "error_x_m")
    first_ey = parse_float(rows[0], "error_y_m")
    if not math.isfinite(first_ex) or not math.isfinite(first_ey):
        return {
            "max": float("nan"),
            "last": float("nan"),
            "mean": float("nan"),
            "std": float("nan"),
        }
    vals = []
    for row in rows:
        ex = parse_float(row, "error_x_m")
        ey = parse_float(row, "error_y_m")
        if math.isfinite(ex) and math.isfinite(ey):
            vals.append(math.hypot(ex - first_ex, ey - first_ey))
    return {
        "max": finite_max(vals),
        "last": vals[-1] if vals else float("nan"),
        "mean": finite_mean(vals),
        "std": finite_std(vals),
    }

compare_rows = load_rows(compare_csv)
gnss_rows = load_rows(gnss_csv)
gps_rows = load_rows(gps_csv)
gps_second_rows = load_rows(gps_second_csv)

def choose_time_key(rows):
    for key in ("ros_time_sec", "elapsed_sec"):
        for row in rows:
            if math.isfinite(parse_float(row, key)):
                return key
    return None

def select_window(rows, start_sec, end_sec):
    time_key = choose_time_key(rows)
    if time_key is None:
        return [], None
    return [
        row for row in rows
        if start_sec <= parse_float(row, time_key) <= end_sec
    ], time_key

def max_time(rows):
    time_key = choose_time_key(rows)
    if time_key is None:
        return float("nan"), None
    return finite_max(parse_float(row, time_key) for row in rows), time_key

requested_width_sec = max(0.0, window_end - window_start)
effective_window_start = window_start
effective_window_end = window_end
window_mode = "requested"

requested_compare_window, compare_time_key = select_window(compare_rows, window_start, window_end)
if not requested_compare_window:
    fallback_end = float("nan")
    for candidate_rows in (compare_rows, gnss_rows, gps_second_rows, gps_rows):
        candidate_end, _ = max_time(candidate_rows)
        if math.isfinite(candidate_end):
            fallback_end = candidate_end
            break
    if math.isfinite(fallback_end):
        effective_window_end = fallback_end
        effective_window_start = fallback_end - requested_width_sec
        window_mode = "tail_fallback"

compare_window, compare_time_key = select_window(compare_rows, effective_window_start, effective_window_end)
gnss_window, gnss_time_key = select_window(gnss_rows, effective_window_start, effective_window_end)
gps_window, gps_time_key = select_window(gps_rows, effective_window_start, effective_window_end)
gps_second_window, gps_second_time_key = select_window(gps_second_rows, effective_window_start, effective_window_end)
gps_window_rebased = rebased_xy_stats(gps_window)
gps_second_window_rebased = rebased_xy_stats(gps_second_window)

rawdiff_values = []
for row in compare_rows:
    raw_available = parse_float(row, "raw_available")
    pos = parse_float(row, "position_error_norm_m")
    raw = parse_float(row, "raw_position_error_norm_m")
    if raw_available >= 0.5 and math.isfinite(pos) and math.isfinite(raw):
        rawdiff_values.append(abs(pos - raw))

gnss_residual_h = []
gnss_residual_3d = []
for row in gnss_window:
    rn = parse_float(row, "gnss_position_residual_n_m")
    res_e = parse_float(row, "gnss_position_residual_e_m")
    ru = parse_float(row, "gnss_position_residual_u_m")
    if math.isfinite(rn) and math.isfinite(res_e):
        gnss_residual_h.append(math.hypot(rn, res_e))
    if math.isfinite(rn) and math.isfinite(res_e) and math.isfinite(ru):
        gnss_residual_3d.append(math.sqrt(rn * rn + res_e * res_e + ru * ru))

gap_total = 0
gap_topics = 0
gap_good_topics = 0
gap_samples_positive = 1
gap_pattern = re.compile(r"gap-probe summary: topic=(.+?) samples=(\d+) gaps=(\d+)")
if os.path.exists(smoke_log):
    with open(smoke_log, "r", errors="ignore") as f:
        for line in f:
            match = gap_pattern.search(line)
            if not match:
                continue
            gap_topics += 1
            samples = int(match.group(2))
            gaps = int(match.group(3))
            gap_total += gaps
            if samples > 0:
                gap_good_topics += 1
            else:
                gap_samples_positive = 0

out = {
    "run_dir": run_dir,
    "smoke_rc": smoke_rc,
    "requested_window_start_sec": window_start,
    "requested_window_end_sec": window_end,
    "effective_window_start_sec": effective_window_start,
    "effective_window_end_sec": effective_window_end,
    "window_mode": window_mode,
    "compare_time_key": compare_time_key or "",
    "gnss_time_key": gnss_time_key or "",
    "gps_vs_pose_time_key": gps_time_key or "",
    "gps_vs_second_time_key": gps_second_time_key or "",
    "compare_csv_present": int(os.path.exists(compare_csv) and os.path.getsize(compare_csv) > 0),
    "gnss_update_debug_csv_present": int(os.path.exists(gnss_csv) and os.path.getsize(gnss_csv) > 0),
    "gps_vs_pose_csv_present": int(os.path.exists(gps_csv) and os.path.getsize(gps_csv) > 0),
    "gps_vs_second_csv_present": int(os.path.exists(gps_second_csv) and os.path.getsize(gps_second_csv) > 0),
    "simulator_poll_timeout": count_fixed("ERROR [simulator_mavlink] poll timeout", px4_log),
    "px4_timesync_err": count_fixed("ERROR [timesync]", px4_log),
    "mavros_timesync_err": count_fixed("TM: Time jump detected. Resetting time synchroniser.", mavros_log),
    "gap_probe_topics": gap_topics,
    "gap_probe_good_topics": gap_good_topics,
    "gap_probe_samples_positive": gap_samples_positive if gap_topics > 0 else 0,
    "gap_probe_gaps_total": gap_total,
    "compare_rows": len(compare_rows),
    "compare_ekf2_init": int(any(parse_float(row, "ekf2_initialized") >= 0.5 for row in compare_rows)),
    "compare_iekf_init": int(any(parse_float(row, "iekf_initialized") >= 0.5 for row in compare_rows)),
    "compare_align_init": int(len(compare_rows) > 0),
    "fallback_active_any": int(any(parse_float(row, "fallback_active") >= 0.5 for row in compare_rows)),
    "raw_available_any": int(any(parse_float(row, "raw_available") >= 0.5 for row in compare_rows)),
    "rawdiff_max_abs_m": finite_max(rawdiff_values),
    "max_pos_m": finite_max(parse_float(row, "position_error_norm_m") for row in compare_rows),
    "last_pos_m": finite_last(compare_rows, "position_error_norm_m"),
    "window_compare_samples": len(compare_window),
    "window_max_pos_m": finite_max(parse_float(row, "position_error_norm_m") for row in compare_window),
    "window_last_pos_m": finite_last(compare_window, "position_error_norm_m"),
    "window_max_abs_pos_x_m": finite_max(abs(parse_float(row, "position_error_x_m")) for row in compare_window),
    "window_max_abs_raw_pos_x_m": finite_max(abs(parse_float(row, "raw_position_error_x_m")) for row in compare_window),
    "gps_vs_pose_window_samples": len(gps_window),
    "gps_vs_pose_window_max_xy_m": finite_max(parse_float(row, "xy_error_m") for row in gps_window),
    "gps_vs_pose_window_last_xy_m": finite_last(gps_window, "xy_error_m"),
    "gps_vs_pose_window_mean_xy_m": finite_mean(parse_float(row, "xy_error_m") for row in gps_window),
    "gps_vs_pose_window_std_xy_m": finite_std(parse_float(row, "xy_error_m") for row in gps_window),
    "gps_vs_pose_window_max_abs_pair_dt_ms": finite_abs_max(parse_float(row, "pair_dt_ms") for row in gps_window),
    "gps_vs_pose_window_rebased_max_xy_m": gps_window_rebased["max"],
    "gps_vs_pose_window_rebased_last_xy_m": gps_window_rebased["last"],
    "gps_vs_pose_window_rebased_mean_xy_m": gps_window_rebased["mean"],
    "gps_vs_pose_window_rebased_std_xy_m": gps_window_rebased["std"],
    "gps_vs_second_window_samples": len(gps_second_window),
    "gps_vs_second_window_max_xy_m": finite_max(parse_float(row, "xy_error_m") for row in gps_second_window),
    "gps_vs_second_window_last_xy_m": finite_last(gps_second_window, "xy_error_m"),
    "gps_vs_second_window_mean_xy_m": finite_mean(parse_float(row, "xy_error_m") for row in gps_second_window),
    "gps_vs_second_window_std_xy_m": finite_std(parse_float(row, "xy_error_m") for row in gps_second_window),
    "gps_vs_second_window_max_abs_pair_dt_ms": finite_abs_max(parse_float(row, "pair_dt_ms") for row in gps_second_window),
    "gps_vs_second_window_rebased_max_xy_m": gps_second_window_rebased["max"],
    "gps_vs_second_window_rebased_last_xy_m": gps_second_window_rebased["last"],
    "gps_vs_second_window_rebased_mean_xy_m": gps_second_window_rebased["mean"],
    "gps_vs_second_window_rebased_std_xy_m": gps_second_window_rebased["std"],
    "gnss_window_samples": len(gnss_window),
    "gnss_window_max_pos_residual_h_m": finite_max(gnss_residual_h),
    "gnss_window_last_pos_residual_h_m": gnss_residual_h[-1] if gnss_residual_h else float("nan"),
    "gnss_window_mean_pos_residual_h_m": finite_mean(gnss_residual_h),
    "gnss_window_std_pos_residual_h_m": finite_std(gnss_residual_h),
    "gnss_window_max_pos_residual_3d_m": finite_max(gnss_residual_3d),
    "gnss_window_last_pos_residual_3d_m": gnss_residual_3d[-1] if gnss_residual_3d else float("nan"),
    "gnss_window_mean_pos_residual_3d_m": finite_mean(gnss_residual_3d),
    "gnss_window_std_pos_residual_3d_m": finite_std(gnss_residual_3d),
}

with open(status_path, "w") as f:
    for key, value in out.items():
        f.write(f"{key}={value}\n")
PY
}

log "run_dir=${RUN_DIR}"

if [[ "${BUILD_PACKAGE}" == "1" ]]; then
  log "building kf_gins_ros2_native"
  (
    cd "${WS_ROOT}"
    source_ros
    colcon build --packages-select kf_gins_ros2_native --symlink-install
  ) > "${BUILD_LOG}" 2>&1
fi

log "stopping any old session bound to this run_dir"
"${BRINGUP_SCRIPT}" stop "${RUN_DIR}" >/dev/null 2>&1 || true

log "starting PX4 + MAVROS runtime"
export PX4_SAFE_START_MAVROS=1
export PX4_SAFE_START_QGC=0
export PX4_SAFE_START_MONITOR=0
export PX4_SAFE_START_COMPARE=0
export PX4_SAFE_START_PLOTJUGGLER=0
export PX4_SAFE_MAVROS_PROFILE=runtime_minimal
export PX4_SAFE_HEADLESS=1
export PX4_SAFE_GCS_LINK_PROFILE=light
export PX4_SAFE_GCS_RATE_BPS=400000
export PX4_SAFE_GCS_POSITION_RATE_HZ=5
export PX4_SAFE_GCS_ATTITUDE_RATE_HZ=8
export PX4_SAFE_GCS_TARGET_RATE_HZ=2
export PX4_SAFE_GCS_SERVO_RATE_HZ=0
export PX4_SAFE_GCS_RC_RATE_HZ=0
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID_VALUE}"
"${BRINGUP_SCRIPT}" start "${RUN_DIR}" > "${BRINGUP_LOG}" 2>&1

log "starting compare stack with persistent CSV outputs"
declare -a COMPARE_CMD=(
  ros2 launch kf_gins_ros2_native compare_ekf_iekf.launch.py
  use_sim_time:=true
  record_bag:=false
  start_rviz:=false
  enable_real_time_comparison:=true
  imu_source:=px4_sensor_combined
  gnss_relay_mode:=px4_sensor_gps
  gnss_source:=navsatfix
  enable_px4_aux_state_relay:=true
  ekf2_input_mode:=px4_vehicle_odometry
  heading_source:=mavros_imu
  mavros_heading_topic:=/px4_aux/imu/data
  speed_source:=mavros_local_velocity
  mavros_local_velocity_topic:=/px4_aux/local_position/velocity_local
  comparison_csv_path:="${COMPARISON_CSV}"
)
if [[ "${ENABLE_GNSS_UPDATE_DEBUG_CSV}" == "1" ]]; then
  COMPARE_CMD+=("gnss_update_debug_csv_path:=${GNSS_UPDATE_CSV}")
fi
for item in "${EXTRA_LAUNCH_ARGS[@]}"; do
  COMPARE_CMD+=("${item}")
done
COMPARE_PID="$(run_bash_with_ros "${COMPARE_LOG}" "${COMPARE_CMD[@]}")"

sleep 10

log "starting GPS-vs-pose probes"
if [[ -n "${GPS_VS_POSE_TOPIC}" ]]; then
  declare -a GPS_PROBE_CMD=(
    ros2 run kf_gins_ros2_native gps_vs_pose_probe.py
    --ros-args
    -p use_sim_time:=true
    -p gps_topic:="${GPS_TOPIC}"
    -p odom_topic:="${GPS_VS_POSE_TOPIC}"
    -p csv_path:="${GPS_VS_POSE_CSV}"
    -p status_print_period_sec:=10.0
  )
  GPS_PROBE_PID="$(run_bash_with_ros "${RUN_DIR}/gps_vs_pose.log" "${GPS_PROBE_CMD[@]}")"
fi

if [[ -n "${GPS_VS_SECOND_POSE_TOPIC}" ]]; then
  declare -a GPS_SECOND_PROBE_CMD=(
    ros2 run kf_gins_ros2_native gps_vs_pose_probe.py
    --ros-args
    -p use_sim_time:=true
    -p gps_topic:="${GPS_TOPIC}"
    -p odom_topic:="${GPS_VS_SECOND_POSE_TOPIC}"
    -p csv_path:="${GPS_VS_SECOND_CSV}"
    -p status_print_period_sec:=10.0
  )
  GPS_SECOND_PROBE_PID="$(run_bash_with_ros "${RUN_DIR}/gps_vs_second_pose.log" "${GPS_SECOND_PROBE_CMD[@]}")"
fi

sleep 5

log "running mission smoke"
declare -a SMOKE_CMD=(
  ros2 run kf_gins_ros2_native px4_mission_smoke.py
  --gap-probe-profile "${GAP_PROBE_PROFILE}"
  --gap-probe-start-after-reached "${GAP_PROBE_START_AFTER_REACHED}"
  --gap-probe-start-delay "${GAP_PROBE_START_DELAY}"
  --gap-probe-window "${GAP_PROBE_WINDOW}"
  --mission-timeout "${MISSION_TIMEOUT_SEC}"
  --print-period "${SMOKE_PRINT_PERIOD_SEC}"
  --skip-pull-mission-on-exit
)
if [[ "${EXIT_AFTER_GAP_PROBE}" == "1" ]]; then
  SMOKE_CMD+=(--exit-after-gap-probe)
fi
for item in "${EXTRA_SMOKE_ARGS[@]}"; do
  SMOKE_CMD+=("${item}")
done

set +e
(
  source_ros
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID_VALUE}"
  export ROS_LOG_DIR="${RUN_DIR}/ros_logs"
  unset ROS_LOCALHOST_ONLY
  "${SMOKE_CMD[@]}"
) > "${SMOKE_LOG}" 2>&1
SMOKE_RC=$?
set -e

kill_session_pid "${GPS_SECOND_PROBE_PID}"
GPS_SECOND_PROBE_PID=""
kill_session_pid "${GPS_PROBE_PID}"
GPS_PROBE_PID=""
kill_session_pid "${COMPARE_PID}"
COMPARE_PID=""

sleep 2

write_status_summary

log "status written to ${STATUS_FILE}"
cat "${STATUS_FILE}"

if [[ "${SMOKE_RC}" -ne 0 ]]; then
  log "smoke returned rc=${SMOKE_RC}"
  exit "${SMOKE_RC}"
fi
