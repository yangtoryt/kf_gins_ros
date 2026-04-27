#!/usr/bin/env bash

set -euo pipefail

CMD="${1:-start}"
RUN_DIR="${2:-/tmp/px4_interactive_safe_bringup}"

WS_ROOT="${HOME}/kf_gins_ws"
PX4_ROOT="${HOME}/PX4-Autopilot"
AGENT_BIN="${WS_ROOT}/tools/microxrce_install/bin/MicroXRCEAgent"
AGENT_LIB="${WS_ROOT}/tools/microxrce_install/lib"
PX4_HEADLESS="${PX4_SAFE_HEADLESS:-}"

ROS_DOMAIN_ID_VALUE="${ROS_DOMAIN_ID:-0}"
AGENT_PORT="${PX4_SAFE_AGENT_PORT:-8888}"
PX4_WORLD="${PX4_SAFE_PX4_WORLD:-city_dense}"
PX4_READY_TIMEOUT_SEC="${PX4_SAFE_PX4_READY_TIMEOUT_SEC:-60}"
PX4_POST_READY_SETTLE_SEC="${PX4_SAFE_PX4_POST_READY_SETTLE_SEC:-12}"
MAVROS_CONNECT_TIMEOUT_SEC="${PX4_SAFE_MAVROS_CONNECT_TIMEOUT_SEC:-30}"
QGC_DELAY_AFTER_MAVROS_SEC="${PX4_SAFE_QGC_DELAY_AFTER_MAVROS_SEC:-8}"
COMPARE_DELAY_AFTER_QGC_SEC="${PX4_SAFE_COMPARE_DELAY_AFTER_QGC_SEC:-8}"
PLOTJUGGLER_DELAY_AFTER_COMPARE_SEC="${PX4_SAFE_PLOTJUGGLER_DELAY_AFTER_COMPARE_SEC:-5}"
PX4_SAFE_NO_PXH="${PX4_SAFE_NO_PXH:-1}"

START_MAVROS="${PX4_SAFE_START_MAVROS:-1}"
START_QGC="${PX4_SAFE_START_QGC:-0}"
START_MONITOR="${PX4_SAFE_START_MONITOR:-1}"
START_COMPARE="${PX4_SAFE_START_COMPARE:-0}"
START_PLOTJUGGLER="${PX4_SAFE_START_PLOTJUGGLER:-0}"
START_GPS_PROBES="${PX4_SAFE_START_GPS_PROBES:-0}"

PX4_GCS_LINK_PROFILE="${PX4_SAFE_GCS_LINK_PROFILE:-auto}"
PX4_GCS_RATE_BPS="${PX4_SAFE_GCS_RATE_BPS:-800000}"
PX4_GCS_POSITION_RATE_HZ="${PX4_SAFE_GCS_POSITION_RATE_HZ:-10}"
PX4_GCS_ATTITUDE_RATE_HZ="${PX4_SAFE_GCS_ATTITUDE_RATE_HZ:-12}"
PX4_GCS_TARGET_RATE_HZ="${PX4_SAFE_GCS_TARGET_RATE_HZ:-5}"
PX4_GCS_SERVO_RATE_HZ="${PX4_SAFE_GCS_SERVO_RATE_HZ:-0}"
PX4_GCS_RC_RATE_HZ="${PX4_SAFE_GCS_RC_RATE_HZ:-0}"
PX4_GCS_FLOW_RATE_HZ="${PX4_SAFE_GCS_FLOW_RATE_HZ:-0}"
PX4_SAFE_MAVROS_DISABLE_GUIDED_TARGET="${PX4_SAFE_MAVROS_DISABLE_GUIDED_TARGET:-1}"
PX4_SAFE_MAVROS_PROFILE="${PX4_SAFE_MAVROS_PROFILE:-default}"
PX4_SAFE_MAVROS_CONFIG_YAML="${PX4_SAFE_MAVROS_CONFIG_YAML:-}"
PX4_SAFE_MAVROS_PLUGINLIST_YAML="${PX4_SAFE_MAVROS_PLUGINLIST_YAML:-}"
PX4_SAFE_RESET_MISSION_CURRENT="${PX4_SAFE_RESET_MISSION_CURRENT:-1}"
PX4_SAFE_RESET_MISSION_SEQ="${PX4_SAFE_RESET_MISSION_SEQ:-0}"
PX4_SAFE_MISSION_PREFLIGHT_TIMEOUT_SEC="${PX4_SAFE_MISSION_PREFLIGHT_TIMEOUT_SEC:-10}"

MAVROS_FCU_URL="${PX4_SAFE_MAVROS_FCU_URL:-udp://:14540@127.0.0.1:14557}"
QGC_BIN="${PX4_SAFE_QGC_BIN:-${HOME}/QGroundControl-x86_64.AppImage}"
COMPARE_IMU_SOURCE="${PX4_SAFE_COMPARE_IMU_SOURCE:-px4_sensor_combined}"
COMPARE_RECORD_BAG="${PX4_SAFE_COMPARE_RECORD_BAG:-false}"
COMPARE_START_RVIZ="${PX4_SAFE_COMPARE_START_RVIZ:-false}"
COMPARE_ENABLE_REAL_TIME="${PX4_SAFE_COMPARE_ENABLE_REAL_TIME:-true}"
COMPARE_SENSOR_COMBINED_TOPIC="${PX4_SAFE_COMPARE_SENSOR_COMBINED_TOPIC:-/fmu/out/sensor_combined}"
# Keep interactive compare aligned with the automated minimal-mainline runtime unless
# the caller explicitly overrides these env vars.
COMPARE_GNSS_RELAY_MODE="${PX4_SAFE_COMPARE_GNSS_RELAY_MODE:-px4_sensor_gps}"
COMPARE_GNSS_SOURCE="${PX4_SAFE_COMPARE_GNSS_SOURCE:-navsatfix}"
COMPARE_ENABLE_PX4_AUX_STATE_RELAY="${PX4_SAFE_COMPARE_ENABLE_PX4_AUX_STATE_RELAY:-true}"
COMPARE_EKF2_INPUT_MODE="${PX4_SAFE_COMPARE_EKF2_INPUT_MODE:-px4_vehicle_odometry}"
COMPARE_HEADING_SOURCE="${PX4_SAFE_COMPARE_HEADING_SOURCE:-mavros_imu}"
COMPARE_MAVROS_HEADING_TOPIC="${PX4_SAFE_COMPARE_MAVROS_HEADING_TOPIC:-/px4_aux/imu/data}"
COMPARE_SPEED_SOURCE="${PX4_SAFE_COMPARE_SPEED_SOURCE:-mavros_local_velocity}"
COMPARE_MAVROS_LOCAL_VELOCITY_TOPIC="${PX4_SAFE_COMPARE_MAVROS_LOCAL_VELOCITY_TOPIC:-/px4_aux/local_position/velocity_local}"
COMPARE_PUBLISH_EKF2_STATE="${PX4_SAFE_COMPARE_PUBLISH_EKF2_STATE:-false}"
COMPARE_PUBLISH_IEKF_STATE="${PX4_SAFE_COMPARE_PUBLISH_IEKF_STATE:-false}"
COMPARE_PUBLISH_ALIGNED_IEKF_STATE="${PX4_SAFE_COMPARE_PUBLISH_ALIGNED_IEKF_STATE:-false}"
COMPARE_PUBLISH_NAMED_METRICS="${PX4_SAFE_COMPARE_PUBLISH_NAMED_METRICS:-false}"
COMPARE_PUBLISH_LIVE_METRICS="${PX4_SAFE_COMPARE_PUBLISH_LIVE_METRICS:-false}"
COMPARE_METRICS_PUBLISH_RATE="${PX4_SAFE_COMPARE_METRICS_PUBLISH_RATE:-20}"
COMPARE_METRICS_LOG_PERIOD_SEC="${PX4_SAFE_COMPARE_METRICS_LOG_PERIOD_SEC:-15.0}"
COMPARE_EKF2_RELAY_PUBLISH_POSE="${PX4_SAFE_COMPARE_EKF2_RELAY_PUBLISH_POSE:-false}"
COMPARE_SUBSCRIBE_EKF2_POSE="${PX4_SAFE_COMPARE_SUBSCRIBE_EKF2_POSE:-false}"
COMPARE_ENABLE_EKF2_PATH="${PX4_SAFE_COMPARE_ENABLE_EKF2_PATH:-false}"
COMPARE_ENABLE_IEKF_PATH="${PX4_SAFE_COMPARE_ENABLE_IEKF_PATH:-false}"
COMPARE_ENABLE_GT_PATH="${PX4_SAFE_COMPARE_ENABLE_GT_PATH:-false}"
COMPARE_ENABLE_IEKF_ALIGNED_PATH="${PX4_SAFE_COMPARE_ENABLE_IEKF_ALIGNED_PATH:-false}"
COMPARE_CSV_PATH="${PX4_SAFE_COMPARE_CSV_PATH:-${RUN_DIR}/comparison_metrics.csv}"
COMPARE_GNSS_UPDATE_DEBUG_CSV_PATH="${PX4_SAFE_COMPARE_GNSS_UPDATE_DEBUG_CSV_PATH:-${RUN_DIR}/gnss_update_debug.csv}"
COMPARE_HEADING_UPDATE_DEBUG_CSV_PATH="${PX4_SAFE_COMPARE_HEADING_UPDATE_DEBUG_CSV_PATH:-${RUN_DIR}/heading_update_debug.csv}"
COMPARE_STATE_PUBLISH_DEBUG_CSV_PATH="${PX4_SAFE_COMPARE_STATE_PUBLISH_DEBUG_CSV_PATH:-${RUN_DIR}/state_publish_debug.csv}"
GPS_PROBE_GPS_TOPIC="${PX4_SAFE_GPS_PROBE_GPS_TOPIC:-/gps/fix}"
GPS_PROBE_ODOM_TOPIC="${PX4_SAFE_GPS_PROBE_ODOM_TOPIC:-/ekf2/pose_odom}"
GPS_PROBE_SECOND_ODOM_TOPIC="${PX4_SAFE_GPS_PROBE_SECOND_ODOM_TOPIC:-/kf_gins/odom_raw}"
GPS_PROBE_CSV_PATH="${PX4_SAFE_GPS_PROBE_CSV_PATH:-${RUN_DIR}/gps_vs_pose.csv}"
GPS_PROBE_SECOND_CSV_PATH="${PX4_SAFE_GPS_PROBE_SECOND_CSV_PATH:-${RUN_DIR}/gps_vs_second_pose.csv}"
GPS_PROBE_STATUS_PRINT_PERIOD_SEC="${PX4_SAFE_GPS_PROBE_STATUS_PRINT_PERIOD_SEC:-10.0}"
GPS_PROBE_GLOBAL_ALIGNMENT_HOLDOFF_SEC="${PX4_SAFE_GPS_PROBE_GLOBAL_ALIGNMENT_HOLDOFF_SEC:-0.0}"
GPS_PROBE_GLOBAL_ALIGNMENT_MIN_PAIRS="${PX4_SAFE_GPS_PROBE_GLOBAL_ALIGNMENT_MIN_PAIRS:-1}"
GPS_PROBE_LOCAL_REANCHOR_WINDOW_SEC="${PX4_SAFE_GPS_PROBE_LOCAL_REANCHOR_WINDOW_SEC:-6.0}"
GPS_PROBE_PAIR_AFTER_NEWER_POSE="${PX4_SAFE_GPS_PROBE_PAIR_AFTER_NEWER_POSE:-true}"
GPS_PROBE_MAX_PAIR_WAIT_SEC="${PX4_SAFE_GPS_PROBE_MAX_PAIR_WAIT_SEC:-0.25}"
GPS_PROBE_DELAY_AFTER_COMPARE_SEC="${PX4_SAFE_GPS_PROBE_DELAY_AFTER_COMPARE_SEC:-3}"

AGENT_LOG="${RUN_DIR}/agent.log"
PX4_LOG="${RUN_DIR}/px4.log"
MAVROS_LOG="${RUN_DIR}/mavros.log"
QGC_LOG="${RUN_DIR}/qgc.log"
MONITOR_LOG="${RUN_DIR}/status_monitor.log"
COMPARE_LOG="${RUN_DIR}/compare_rviz.log"
PLOTJUGGLER_LOG="${RUN_DIR}/plotjuggler.log"
MISSION_PREFLIGHT_LOG="${RUN_DIR}/mission_preflight.log"
GPS_VS_POSE_LOG="${RUN_DIR}/gps_vs_pose.log"
GPS_VS_SECOND_POSE_LOG="${RUN_DIR}/gps_vs_second_pose.log"
ROS_LOG_DIR_SAFE="${RUN_DIR}/ros_logs"
PID_FILE="${RUN_DIR}/pids.env"
PX4_LOG_SANITIZED="${RUN_DIR}/px4_sanitized.log"
PX4_RC_MAVLINK_OVERRIDE="${RUN_DIR}/px4-rc.mavlink"
MAVROS_PLUGINLIST_OVERRIDE="${RUN_DIR}/mavros_px4_pluginlists.yaml"
MAVROS_CONFIG_OVERRIDE="${RUN_DIR}/mavros_px4_config.yaml"
KFGINS_CONFIG_DIR="${WS_ROOT}/src/kf_gins_ros2_native/config"
KFGINS_MAVROS_RUNTIME_MINIMAL_PLUGINLIST="${KFGINS_CONFIG_DIR}/mavros_px4_runtime_minimal_pluginlist.yaml"
KFGINS_MAVROS_RUNTIME_MINIMAL_CONFIG="${KFGINS_CONFIG_DIR}/mavros_px4_runtime_minimal_config.yaml"

AGENT_PATTERN="MicroXRCEAgent udp4 -p ${AGENT_PORT}"
PX4_PATTERN="${PX4_ROOT}/build/px4_sitl_default/bin/px4"
PX4_SITL_RUN_PATTERN="${PX4_ROOT}/Tools/simulation/gazebo-classic/sitl_run.sh"
GZSERVER_PATTERN="gzserver( |$)"
GZCLIENT_PATTERN="gzclient( |$)"
MAVROS_PATTERN="mavros_node|ros2 launch mavros px4.launch"
MONITOR_PATTERN="px4_flight_status_monitor.py"
QGC_PATTERN="QGroundControl-x86_64\\.AppImage|/tmp/\\.mount_.*QGroundControl"
COMPARE_PATTERN="compare_ekf_iekf\\.launch\\.py|kf_gins_node|real_time_comparison\\.py|ekf2_state_relay\\.py|ekf2_path_publisher\\.py|iekf_path_publisher\\.py|iekf_aligned_path_publisher\\.py|rviz2"
PLOTJUGGLER_PATTERN="plotjuggler"
GPS_PROBE_PATTERN="gps_vs_pose_probe.py"

mkdir -p "${RUN_DIR}" "${ROS_LOG_DIR_SAFE}"

log() {
  echo "[px4-safe-bringup] $*"
}

resolve_px4_gcs_link_profile() {
  case "${PX4_GCS_LINK_PROFILE}" in
    auto)
      if [[ "${START_QGC}" == "1" ]]; then
        echo "light"
      else
        echo "upstream"
      fi
      ;;
    upstream|light)
      echo "${PX4_GCS_LINK_PROFILE}"
      ;;
    *)
      log "unsupported PX4_SAFE_GCS_LINK_PROFILE=${PX4_GCS_LINK_PROFILE} (expected: auto|upstream|light)"
      exit 1
      ;;
  esac
}

prepare_px4_rc_mavlink_override() {
  local profile
  profile="$(resolve_px4_gcs_link_profile)"
  rm -f "${PX4_RC_MAVLINK_OVERRIDE}"

  if [[ "${profile}" == "upstream" ]]; then
    log "using upstream PX4 GCS MAVLink profile"
    return
  fi

  cat > "${PX4_RC_MAVLINK_OVERRIDE}" <<'EOF'
#!/bin/sh
# Generated by px4_interactive_safe_bringup.sh to reduce GCS/QGC load on SITL.
# Uses environment overrides exported by the bringup shell.

udp_offboard_port_local=$((14580+px4_instance))
udp_offboard_port_remote=$((14540+px4_instance))
[ "$px4_instance" -gt 9 ] && udp_offboard_port_remote=14549
udp_onboard_payload_port_local=$((14280+px4_instance))
udp_onboard_payload_port_remote=$((14030+px4_instance))
udp_onboard_gimbal_port_local=$((13030+px4_instance))
udp_onboard_gimbal_port_remote=$((13280+px4_instance))
udp_gcs_port_local=$((18570+px4_instance))

gcs_rate=${PX4_SAFE_GCS_RATE_BPS:-1000000}
gcs_position_rate=${PX4_SAFE_GCS_POSITION_RATE_HZ:-15}
gcs_attitude_rate=${PX4_SAFE_GCS_ATTITUDE_RATE_HZ:-20}
gcs_target_rate=${PX4_SAFE_GCS_TARGET_RATE_HZ:-10}
gcs_servo_rate=${PX4_SAFE_GCS_SERVO_RATE_HZ:-5}
gcs_rc_rate=${PX4_SAFE_GCS_RC_RATE_HZ:-2}
gcs_flow_rate=${PX4_SAFE_GCS_FLOW_RATE_HZ:-0}

# GCS link
mavlink start -x -u $udp_gcs_port_local -r $gcs_rate -f
mavlink stream -r $gcs_target_rate -s POSITION_TARGET_LOCAL_NED -u $udp_gcs_port_local
mavlink stream -r $gcs_position_rate -s LOCAL_POSITION_NED -u $udp_gcs_port_local
mavlink stream -r $gcs_position_rate -s GLOBAL_POSITION_INT -u $udp_gcs_port_local
mavlink stream -r $gcs_attitude_rate -s ATTITUDE -u $udp_gcs_port_local
mavlink stream -r $gcs_position_rate -s ATTITUDE_QUATERNION -u $udp_gcs_port_local
mavlink stream -r $gcs_target_rate -s ATTITUDE_TARGET -u $udp_gcs_port_local
mavlink stream -r $gcs_servo_rate -s SERVO_OUTPUT_RAW_0 -u $udp_gcs_port_local
mavlink stream -r $gcs_rc_rate -s RC_CHANNELS -u $udp_gcs_port_local

if [ "$gcs_flow_rate" -gt 0 ]; then
	mavlink stream -r $gcs_flow_rate -s OPTICAL_FLOW_RAD -u $udp_gcs_port_local
fi

# API/Offboard link
mavlink start -x -u $udp_offboard_port_local -r 4000000 -f -m onboard -o $udp_offboard_port_remote

# Onboard link to camera
mavlink start -x -u $udp_onboard_payload_port_local -r 4000 -f -m onboard -o $udp_onboard_payload_port_remote

# Onboard link to gimbal
mavlink start -x -u $udp_onboard_gimbal_port_local -r 400000 -m gimbal -o $udp_onboard_gimbal_port_remote

if [ "$PX4_SIMULATOR" = "sihsim" ]; then
	udp_sihsim_port_local=$((19450+px4_instance))
	udp_sihsim_port_remote=$((19410+px4_instance))
	mavlink start -x -u $udp_sihsim_port_local -r 400000 -m custom -o $udp_sihsim_port_remote
	mavlink stream -r 200 -s HIL_ACTUATOR_CONTROLS -u  $udp_sihsim_port_local
	mavlink stream -r 25 -s HIL_STATE_QUATERNION -u  $udp_sihsim_port_local
fi
EOF

  chmod +x "${PX4_RC_MAVLINK_OVERRIDE}"
  log "using light PX4 GCS MAVLink profile: rate=${PX4_GCS_RATE_BPS}B/s pos=${PX4_GCS_POSITION_RATE_HZ}Hz att=${PX4_GCS_ATTITUDE_RATE_HZ}Hz target=${PX4_GCS_TARGET_RATE_HZ}Hz servo=${PX4_GCS_SERVO_RATE_HZ}Hz rc=${PX4_GCS_RC_RATE_HZ}Hz flow=${PX4_GCS_FLOW_RATE_HZ}Hz"
}

prepare_mavros_pluginlist_override() {
  rm -f "${MAVROS_PLUGINLIST_OVERRIDE}"

  if [[ -n "${PX4_SAFE_MAVROS_PLUGINLIST_YAML}" ]]; then
    cp "${PX4_SAFE_MAVROS_PLUGINLIST_YAML}" "${MAVROS_PLUGINLIST_OVERRIDE}"
    log "using explicit MAVROS plugin list override: ${PX4_SAFE_MAVROS_PLUGINLIST_YAML}"
    return
  fi

  case "${PX4_SAFE_MAVROS_PROFILE}" in
    default)
      ;;
    runtime_minimal)
      cp "${KFGINS_MAVROS_RUNTIME_MINIMAL_PLUGINLIST}" "${MAVROS_PLUGINLIST_OVERRIDE}"
      log "using MAVROS runtime_minimal plugin allowlist"
      return
      ;;
    *)
      log "unsupported PX4_SAFE_MAVROS_PROFILE=${PX4_SAFE_MAVROS_PROFILE} (expected: default|runtime_minimal)"
      exit 1
      ;;
  esac

  if [[ "${PX4_SAFE_MAVROS_DISABLE_GUIDED_TARGET}" != "1" ]]; then
    log "using upstream MAVROS plugin list"
    return
  fi

  cat > "${MAVROS_PLUGINLIST_OVERRIDE}" <<'EOF'
/**:
  ros__parameters:
    plugin_denylist:
      # common

      # extras
      - image_pub
      - vibration
      - distance_sensor
      - rangefinder
      - wheel_odometry
      - guided_target
EOF

  log "using MAVROS plugin denylist override: guided_target disabled"
}

prepare_mavros_config_override() {
  rm -f "${MAVROS_CONFIG_OVERRIDE}"

  if [[ -n "${PX4_SAFE_MAVROS_CONFIG_YAML}" ]]; then
    cp "${PX4_SAFE_MAVROS_CONFIG_YAML}" "${MAVROS_CONFIG_OVERRIDE}"
    log "using explicit MAVROS config override: ${PX4_SAFE_MAVROS_CONFIG_YAML}"
    return
  fi

  case "${PX4_SAFE_MAVROS_PROFILE}" in
    default)
      ;;
    runtime_minimal)
      cp "${KFGINS_MAVROS_RUNTIME_MINIMAL_CONFIG}" "${MAVROS_CONFIG_OVERRIDE}"
      log "using MAVROS runtime_minimal config override"
      ;;
    *)
      log "unsupported PX4_SAFE_MAVROS_PROFILE=${PX4_SAFE_MAVROS_PROFILE} (expected: default|runtime_minimal)"
      exit 1
      ;;
  esac
}

save_pids() {
  cat > "${PID_FILE}" <<EOF
AGENT_PID="${AGENT_PID:-}"
PX4_PID="${PX4_PID:-}"
MAVROS_PID="${MAVROS_PID:-}"
QGC_PID="${QGC_PID:-}"
MONITOR_PID="${MONITOR_PID:-}"
COMPARE_PID="${COMPARE_PID:-}"
PLOTJUGGLER_PID="${PLOTJUGGLER_PID:-}"
GPS_POSE_PID="${GPS_POSE_PID:-}"
GPS_SECOND_POSE_PID="${GPS_SECOND_POSE_PID:-}"
RUN_DIR="${RUN_DIR}"
EOF
}

load_pids() {
  if [[ -f "${PID_FILE}" ]]; then
    # shellcheck disable=SC1090
    source "${PID_FILE}"
  fi
}

is_running() {
  local pid="${1:-}"
  [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null
}

first_matching_pid() {
  local pattern="$1"
  pgrep -f -- "${pattern}" 2>/dev/null | head -n 1 || true
}

collect_pid_tree() {
  local root_pid="$1"
  local child_pid=""

  [[ -n "${root_pid}" ]] || return 0
  is_running "${root_pid}" || return 0
  printf '%s\n' "${root_pid}"

  while IFS= read -r child_pid; do
    [[ -n "${child_pid}" ]] || continue
    collect_pid_tree "${child_pid}"
  done < <(pgrep -P "${root_pid}" 2>/dev/null || true)
}

build_current_session_ignore_pids() {
  local -A seen_pids=()
  local root_pid=""
  local tree_pid=""

  CURRENT_SESSION_IGNORE_PIDS=()

  for root_pid in \
    "${AGENT_PID:-}" \
    "${PX4_PID:-}" \
    "${MAVROS_PID:-}" \
    "${QGC_PID:-}" \
    "${MONITOR_PID:-}" \
    "${COMPARE_PID:-}" \
    "${PLOTJUGGLER_PID:-}" \
    "${GPS_POSE_PID:-}" \
    "${GPS_SECOND_POSE_PID:-}"; do
    [[ -n "${root_pid}" ]] || continue
    while IFS= read -r tree_pid; do
      [[ -n "${tree_pid}" ]] || continue
      if [[ -n "${seen_pids[${tree_pid}]:-}" ]]; then
        continue
      fi
      seen_pids["${tree_pid}"]=1
      CURRENT_SESSION_IGNORE_PIDS+=("${tree_pid}")
    done < <(collect_pid_tree "${root_pid}")
  done
}

hydrate_current_core_pids_from_runtime() {
  local agent_pid_current=""
  local px4_pid_current=""
  local mavros_pid_current=""

  agent_pid_current="$(first_matching_pid "${AGENT_PATTERN}")"
  px4_pid_current="$(first_matching_pid "${PX4_PATTERN}|${PX4_SITL_RUN_PATTERN}")"
  mavros_pid_current="$(first_matching_pid "${MAVROS_PATTERN}")"

  if [[ -z "${agent_pid_current}" || -z "${px4_pid_current}" || -z "${mavros_pid_current}" ]]; then
    return 1
  fi

  AGENT_PID="${agent_pid_current}"
  PX4_PID="${px4_pid_current}"
  MAVROS_PID="${mavros_pid_current}"
  save_pids
  return 0
}

tracked_run_dir_for_pid() {
  local pid="$1"
  local file=""
  local run_dir=""

  while IFS= read -r file; do
    [[ -f "${file}" ]] || continue
    if grep -Fq "=\"${pid}\"" "${file}"; then
      run_dir="$(sed -n 's/^RUN_DIR=\"\(.*\)\"$/\1/p' "${file}")"
      if [[ -n "${run_dir}" ]]; then
        printf '%s\n' "${run_dir}"
        return 0
      fi
    fi
  done < <(find "${WS_ROOT}/artifacts" -path '*/pids.env' -type f 2>/dev/null)

  return 1
}

detect_global_runtime_conflicts() {
  local -a ignore_pids=("$@")
  local -A seen_pids=()
  local ignore_pid=""
  local label=""
  local pattern=""
  local line=""
  local pid=""
  local cmd=""
  local run_dir=""

  GLOBAL_RUNTIME_CONFLICT_COUNT=0
  GLOBAL_RUNTIME_CONFLICT_LINES=()

  pid_is_ignored() {
    local candidate="$1"
    for ignore_pid in "${ignore_pids[@]}"; do
      if [[ -n "${ignore_pid}" && "${candidate}" == "${ignore_pid}" ]]; then
        return 0
      fi
    done
    return 1
  }

  append_pattern_conflicts() {
    label="$1"
    pattern="$2"
    while IFS= read -r line; do
      [[ -z "${line}" ]] && continue
      pid="${line%% *}"
      cmd="${line#* }"
      if pid_is_ignored "${pid}"; then
        continue
      fi
      if [[ -n "${seen_pids[${pid}]:-}" ]]; then
        continue
      fi
      seen_pids["${pid}"]=1
      run_dir="$(tracked_run_dir_for_pid "${pid}" || true)"
      if [[ -n "${run_dir}" ]]; then
        GLOBAL_RUNTIME_CONFLICT_LINES+=("${label}: pid=${pid} run_dir=${run_dir} cmd=${cmd}")
      else
        GLOBAL_RUNTIME_CONFLICT_LINES+=("${label}: pid=${pid} cmd=${cmd}")
      fi
      GLOBAL_RUNTIME_CONFLICT_COUNT=$((GLOBAL_RUNTIME_CONFLICT_COUNT + 1))
    done < <(pgrep -af -- "${pattern}" 2>/dev/null || true)
  }

  append_pattern_conflicts "agent" "${AGENT_PATTERN}"
  append_pattern_conflicts "px4" "${PX4_PATTERN}|${PX4_SITL_RUN_PATTERN}"
  append_pattern_conflicts "gzserver" "${GZSERVER_PATTERN}"
  append_pattern_conflicts "gzclient" "${GZCLIENT_PATTERN}"
  append_pattern_conflicts "mavros" "${MAVROS_PATTERN}"
  append_pattern_conflicts "monitor" "${MONITOR_PATTERN}"
  append_pattern_conflicts "compare" "${COMPARE_PATTERN}"
  append_pattern_conflicts "plotjuggler" "${PLOTJUGGLER_PATTERN}"
  append_pattern_conflicts "gps-probe" "${GPS_PROBE_PATTERN}"
}

require_no_global_runtime_conflicts() {
  local action="$1"
  shift

  detect_global_runtime_conflicts "$@"
  if (( GLOBAL_RUNTIME_CONFLICT_COUNT == 0 )); then
    return 0
  fi

  log "${action} blocked: detected residual runtime from another session"
  local item=""
  for item in "${GLOBAL_RUNTIME_CONFLICT_LINES[@]}"; do
    printf '  - %s\n' "${item}"
  done
  log "stop the old session first, then retry"
  exit 1
}

require_current_core_runtime() {
  local action="$1"
  local timeout_sec="${2:-15}"
  local elapsed=0

  while (( elapsed <= timeout_sec )); do
    load_pids
    if is_running "${AGENT_PID:-}" && is_running "${PX4_PID:-}" && is_running "${MAVROS_PID:-}"; then
      return 0
    fi
    if hydrate_current_core_pids_from_runtime; then
      return 0
    fi
    sleep 1
    elapsed=$((elapsed + 1))
  done

  log "${action} blocked: no active PX4+MAVROS runtime under ${RUN_DIR}; run 'start' first"
  exit 1
}

kill_if_running() {
  local pid="${1:-}"
  if is_running "${pid}"; then
    kill -- -"${pid}" 2>/dev/null || kill "${pid}" 2>/dev/null || true
    sleep 1
    if is_running "${pid}"; then
      kill -KILL -- -"${pid}" 2>/dev/null || kill -KILL "${pid}" 2>/dev/null || true
      sleep 1
    fi
    wait "${pid}" 2>/dev/null || true
  fi
}

kill_pattern_if_running() {
  local pattern="$1"
  local pids
  pids="$(pgrep -f -- "${pattern}" 2>/dev/null || true)"
  if [[ -z "${pids}" ]]; then
    return
  fi
  while read -r pid; do
    [[ -z "${pid}" ]] && continue
    kill_if_running "${pid}"
  done <<< "${pids}"
}

count_fixed() {
  local needle="$1"
  local file="$2"
  if [[ ! -f "${file}" ]]; then
    echo 0
    return
  fi
  (grep -F -o -- "${needle}" "${file}" || true) | wc -l
}

wait_for_log_fixed() {
  local needle="$1"
  local file="$2"
  local timeout_sec="$3"
  local label="$4"
  local elapsed=0
  while (( elapsed < timeout_sec )); do
    if [[ -f "${file}" ]] && grep -Fq -- "${needle}" "${file}"; then
      log "${label} observed after ${elapsed}s"
      return 0
    fi
    sleep 1
    elapsed=$((elapsed + 1))
  done
  log "${label} not observed within ${timeout_sec}s"
  return 1
}

source_ros() {
  set +u
  source /opt/ros/humble/setup.bash
  if [[ -f "${WS_ROOT}/install/setup.bash" ]]; then
    source "${WS_ROOT}/install/setup.bash"
  fi
  set -u
}

start_agent() {
  log "starting MicroXRCEAgent..."
  setsid env \
    LD_LIBRARY_PATH="${AGENT_LIB}:${LD_LIBRARY_PATH:-}" \
    "${AGENT_BIN}" udp4 -p "${AGENT_PORT}" > "${AGENT_LOG}" 2>&1 < /dev/null &
  AGENT_PID=$!
  save_pids
  sleep 2
}

start_px4_gui() {
  prepare_px4_rc_mavlink_override
  log "starting PX4 + Gazebo GUI (no_pxh=${PX4_SAFE_NO_PXH})..."
  setsid bash -lc "
    set -euo pipefail
    set +u
    source /opt/ros/humble/setup.bash
    set -u
    cd '${PX4_ROOT}'
    unset LIBGL_ALWAYS_SOFTWARE
    unset QT_OPENGL
    export QT_QPA_PLATFORM=xcb
    export QT_X11_NO_MITSHM=1
    export ROS_DOMAIN_ID='${ROS_DOMAIN_ID_VALUE}'
    export ROS_LOG_DIR='${ROS_LOG_DIR_SAFE}'
    export PX4_UXRCE_DDS_PORT='${AGENT_PORT}'
    export PX4_SAFE_GCS_RATE_BPS='${PX4_GCS_RATE_BPS}'
    export PX4_SAFE_GCS_POSITION_RATE_HZ='${PX4_GCS_POSITION_RATE_HZ}'
    export PX4_SAFE_GCS_ATTITUDE_RATE_HZ='${PX4_GCS_ATTITUDE_RATE_HZ}'
    export PX4_SAFE_GCS_TARGET_RATE_HZ='${PX4_GCS_TARGET_RATE_HZ}'
    export PX4_SAFE_GCS_SERVO_RATE_HZ='${PX4_GCS_SERVO_RATE_HZ}'
    export PX4_SAFE_GCS_RC_RATE_HZ='${PX4_GCS_RC_RATE_HZ}'
    export PX4_SAFE_GCS_FLOW_RATE_HZ='${PX4_GCS_FLOW_RATE_HZ}'
    if [[ '${PX4_SAFE_NO_PXH}' == '1' ]]; then
      export NO_PXH=1
    else
      unset NO_PXH
    fi
    unset PX4_UXRCE_DDS_NS
    if [[ -n '${PX4_HEADLESS}' ]]; then
      export HEADLESS='${PX4_HEADLESS}'
    fi
    export PX4_SITL_WORLD='${PX4_WORLD}'
    export PATH='${RUN_DIR}':\"\$PATH\"
    # Keep PX4 logging raw at runtime.
    # Per-line tr/sed/grep sanitization was consuming significant CPU and could
    # inject ~1s callback stalls into mission-time ROS consumers.
    exec make px4_sitl gazebo-classic_iris
  " > "${PX4_LOG}" 2>&1 < /dev/null &
  PX4_PID=$!
  save_pids
}

start_mavros() {
  log "starting MAVROS..."
  local pluginlists_yaml
  local config_yaml
  pluginlists_yaml="/opt/ros/humble/share/mavros/launch/px4_pluginlists.yaml"
  config_yaml="/opt/ros/humble/share/mavros/launch/px4_config.yaml"
  if [[ -f "${MAVROS_PLUGINLIST_OVERRIDE}" ]]; then
    pluginlists_yaml="${MAVROS_PLUGINLIST_OVERRIDE}"
  fi
  if [[ -f "${MAVROS_CONFIG_OVERRIDE}" ]]; then
    config_yaml="${MAVROS_CONFIG_OVERRIDE}"
  fi
  setsid bash -lc "
    set -euo pipefail
    set +u
    source /opt/ros/humble/setup.bash
    if [[ -f '${WS_ROOT}/install/setup.bash' ]]; then
      source '${WS_ROOT}/install/setup.bash'
    fi
    set -u
    export ROS_DOMAIN_ID='${ROS_DOMAIN_ID_VALUE}'
    export ROS_LOG_DIR='${ROS_LOG_DIR_SAFE}'
    unset ROS_LOCALHOST_ONLY
    exec ros2 launch mavros node.launch \
      'fcu_url:=${MAVROS_FCU_URL}' \
      'gcs_url:=\"\"' \
      'tgt_system:=1' \
      'tgt_component:=1' \
      'pluginlists_yaml:=${pluginlists_yaml}' \
      'config_yaml:=${config_yaml}' \
      'log_output:=screen' \
      'fcu_protocol:=v2.0' \
      'respawn_mavros:=false' \
      'namespace:=mavros'
  " > "${MAVROS_LOG}" 2>&1 < /dev/null &
  MAVROS_PID=$!
  save_pids
}

start_monitor() {
  log "starting flight status monitor..."
  setsid bash -lc "
    set -euo pipefail
    set +u
    source /opt/ros/humble/setup.bash
    if [[ -f '${WS_ROOT}/install/setup.bash' ]]; then
      source '${WS_ROOT}/install/setup.bash'
    fi
    set -u
    export ROS_DOMAIN_ID='${ROS_DOMAIN_ID_VALUE}'
    export ROS_LOG_DIR='${ROS_LOG_DIR_SAFE}'
    unset ROS_LOCALHOST_ONLY
    exec ros2 run kf_gins_ros2_native px4_flight_status_monitor.py --ros-args -p print_hz:=2.0
  " > "${MONITOR_LOG}" 2>&1 < /dev/null &
  MONITOR_PID=$!
  save_pids
}

run_mission_preflight_reset() {
  if [[ "${PX4_SAFE_RESET_MISSION_CURRENT}" != "1" ]]; then
    log "mission current reset disabled (PX4_SAFE_RESET_MISSION_CURRENT=${PX4_SAFE_RESET_MISSION_CURRENT})"
    return 0
  fi

  local current_seq=""
  local mission_count=""
  local current_command=""
  local target_seq="${PX4_SAFE_RESET_MISSION_SEQ}"

  log "checking PX4 stored mission state via MAVROS..."
  if ! (
    source_ros
    export ROS_DOMAIN_ID="${ROS_DOMAIN_ID_VALUE}"
    export ROS_LOG_DIR="${ROS_LOG_DIR_SAFE}"
    unset ROS_LOCALHOST_ONLY
    ros2 run kf_gins_ros2_native px4_mission_preflight.py \
      check \
      --mavros-ns /mavros \
      --timeout "${PX4_SAFE_MISSION_PREFLIGHT_TIMEOUT_SEC}"
  ) > "${MISSION_PREFLIGHT_LOG}" 2>&1; then
    log "mission preflight check failed; see ${MISSION_PREFLIGHT_LOG}"
    return 0
  fi

  current_seq="$(sed -n 's/.*mission: current_seq=\([0-9-]\+\) count=.*/\1/p' "${MISSION_PREFLIGHT_LOG}" | tail -n 1)"
  mission_count="$(sed -n 's/.*mission: current_seq=[0-9-]\+ count=\([0-9]\+\).*/\1/p' "${MISSION_PREFLIGHT_LOG}" | tail -n 1)"
  current_command="$(sed -n 's/.*current waypoint: seq=[0-9-]\+ command=\([0-9]\+\) frame=.*/\1/p' "${MISSION_PREFLIGHT_LOG}" | tail -n 1)"

  log "mission preflight: current_seq=${current_seq:-unknown} count=${mission_count:-unknown} command=${current_command:-unknown}"

  if [[ -z "${mission_count}" || "${mission_count}" == "0" ]]; then
    log "no stored mission available via MAVROS; skipping mission current reset"
    return 0
  fi

  if [[ "${current_seq}" == "${target_seq}" && "${current_command}" != "20" ]]; then
    log "mission current item already at seq=${target_seq}"
    return 0
  fi

  log "resetting mission current item to seq=${target_seq}"
  if (
    source_ros
    export ROS_DOMAIN_ID="${ROS_DOMAIN_ID_VALUE}"
    export ROS_LOG_DIR="${ROS_LOG_DIR_SAFE}"
    unset ROS_LOCALHOST_ONLY
    ros2 run kf_gins_ros2_native px4_mission_preflight.py \
      set-current \
      --seq "${target_seq}" \
      --mavros-ns /mavros \
      --timeout "${PX4_SAFE_MISSION_PREFLIGHT_TIMEOUT_SEC}"
  ) >> "${MISSION_PREFLIGHT_LOG}" 2>&1; then
    log "mission current reset complete"
  else
    log "mission current reset failed; see ${MISSION_PREFLIGHT_LOG}"
  fi
}

start_qgc() {
  log "starting QGC..."
  setsid "${QGC_BIN}" > "${QGC_LOG}" 2>&1 < /dev/null &
  QGC_PID=$!
  save_pids
}

start_compare() {
  log "starting compare stack (rviz=${COMPARE_START_RVIZ}, ekf2_state=${COMPARE_PUBLISH_EKF2_STATE}, iekf_state=${COMPARE_PUBLISH_IEKF_STATE}, aligned_state=${COMPARE_PUBLISH_ALIGNED_IEKF_STATE}, ekf2_path=${COMPARE_ENABLE_EKF2_PATH}, iekf_path=${COMPARE_ENABLE_IEKF_PATH}, gt_path=${COMPARE_ENABLE_GT_PATH}, aligned_path=${COMPARE_ENABLE_IEKF_ALIGNED_PATH})..."
  setsid bash -lc "
    set -euo pipefail
    set +u
    source /opt/ros/humble/setup.bash
    if [[ -f '${WS_ROOT}/install/setup.bash' ]]; then
      source '${WS_ROOT}/install/setup.bash'
    fi
    set -u
    export ROS_DOMAIN_ID='${ROS_DOMAIN_ID_VALUE}'
    export ROS_LOG_DIR='${ROS_LOG_DIR_SAFE}'
    unset ROS_LOCALHOST_ONLY
    exec ros2 launch kf_gins_ros2_native compare_ekf_iekf.launch.py \
      use_sim_time:=true \
      record_bag:='${COMPARE_RECORD_BAG}' \
      start_rviz:='${COMPARE_START_RVIZ}' \
      enable_real_time_comparison:='${COMPARE_ENABLE_REAL_TIME}' \
      imu_source:='${COMPARE_IMU_SOURCE}' \
      px4_sensor_combined_topic:='${COMPARE_SENSOR_COMBINED_TOPIC}' \
      gnss_relay_mode:='${COMPARE_GNSS_RELAY_MODE}' \
      gnss_source:='${COMPARE_GNSS_SOURCE}' \
      enable_px4_aux_state_relay:='${COMPARE_ENABLE_PX4_AUX_STATE_RELAY}' \
      ekf2_input_mode:='${COMPARE_EKF2_INPUT_MODE}' \
      heading_source:='${COMPARE_HEADING_SOURCE}' \
      mavros_heading_topic:='${COMPARE_MAVROS_HEADING_TOPIC}' \
      speed_source:='${COMPARE_SPEED_SOURCE}' \
      mavros_local_velocity_topic:='${COMPARE_MAVROS_LOCAL_VELOCITY_TOPIC}' \
      publish_ekf2_state:='${COMPARE_PUBLISH_EKF2_STATE}' \
      publish_iekf_state:='${COMPARE_PUBLISH_IEKF_STATE}' \
      publish_aligned_iekf_state:='${COMPARE_PUBLISH_ALIGNED_IEKF_STATE}' \
      comparison_publish_named_metrics:='${COMPARE_PUBLISH_NAMED_METRICS}' \
      comparison_publish_live_metrics:='${COMPARE_PUBLISH_LIVE_METRICS}' \
      comparison_metrics_publish_rate:='${COMPARE_METRICS_PUBLISH_RATE}' \
      comparison_metrics_log_period_sec:='${COMPARE_METRICS_LOG_PERIOD_SEC}' \
      ekf2_relay_publish_pose:='${COMPARE_EKF2_RELAY_PUBLISH_POSE}' \
      comparison_subscribe_ekf2_pose:='${COMPARE_SUBSCRIBE_EKF2_POSE}' \
      enable_ekf2_path:='${COMPARE_ENABLE_EKF2_PATH}' \
      enable_iekf_path:='${COMPARE_ENABLE_IEKF_PATH}' \
      enable_gt_path:='${COMPARE_ENABLE_GT_PATH}' \
      enable_iekf_aligned_path:='${COMPARE_ENABLE_IEKF_ALIGNED_PATH}' \
      comparison_csv_path:='${COMPARE_CSV_PATH}' \
      gnss_update_debug_csv_path:='${COMPARE_GNSS_UPDATE_DEBUG_CSV_PATH}' \
      heading_update_debug_csv_path:='${COMPARE_HEADING_UPDATE_DEBUG_CSV_PATH}' \
      state_publish_debug_csv_path:='${COMPARE_STATE_PUBLISH_DEBUG_CSV_PATH}'
  " > "${COMPARE_LOG}" 2>&1 < /dev/null &
  COMPARE_PID=$!
  save_pids
}

start_plotjuggler() {
  log "starting PlotJuggler..."
  setsid bash -lc "
    set -euo pipefail
    set +u
    source /opt/ros/humble/setup.bash
    set -u
    export ROS_DOMAIN_ID='${ROS_DOMAIN_ID_VALUE}'
    export ROS_LOG_DIR='${ROS_LOG_DIR_SAFE}'
    export PLOTJUGGLER_PLUGIN_PATH=/opt/ros/humble/lib/plotjuggler_ros:/opt/ros/humble/lib/plotjuggler
    unset ROS_LOCALHOST_ONLY
    exec ros2 run plotjuggler plotjuggler
  " > "${PLOTJUGGLER_LOG}" 2>&1 < /dev/null &
  PLOTJUGGLER_PID=$!
  save_pids
}

start_single_gps_probe() {
  local odom_topic="$1"
  local csv_path="$2"
  local log_path="$3"
  local pid_var_name="$4"
  local pid=""

  [[ -n "${odom_topic}" ]] || return 0

  log "starting GPS probe for ${odom_topic} -> ${csv_path}"
  setsid bash -lc "
    set -euo pipefail
    set +u
    source /opt/ros/humble/setup.bash
    if [[ -f '${WS_ROOT}/install/setup.bash' ]]; then
      source '${WS_ROOT}/install/setup.bash'
    fi
    set -u
    export ROS_DOMAIN_ID='${ROS_DOMAIN_ID_VALUE}'
    export ROS_LOG_DIR='${ROS_LOG_DIR_SAFE}'
    unset ROS_LOCALHOST_ONLY
    exec ros2 run kf_gins_ros2_native gps_vs_pose_probe.py \
      --ros-args \
      -p use_sim_time:=true \
      -p gps_topic:='${GPS_PROBE_GPS_TOPIC}' \
      -p odom_topic:='${odom_topic}' \
      -p csv_path:='${csv_path}' \
      -p status_print_period_sec:='${GPS_PROBE_STATUS_PRINT_PERIOD_SEC}' \
      -p global_alignment_holdoff_sec:='${GPS_PROBE_GLOBAL_ALIGNMENT_HOLDOFF_SEC}' \
      -p global_alignment_min_pairs:='${GPS_PROBE_GLOBAL_ALIGNMENT_MIN_PAIRS}' \
      -p local_reanchor_window_sec:='${GPS_PROBE_LOCAL_REANCHOR_WINDOW_SEC}' \
      -p pair_after_newer_pose:='${GPS_PROBE_PAIR_AFTER_NEWER_POSE}' \
      -p max_pair_wait_sec:='${GPS_PROBE_MAX_PAIR_WAIT_SEC}'
  " > "${log_path}" 2>&1 < /dev/null &
  pid=$!
  printf -v "${pid_var_name}" '%s' "${pid}"
  save_pids
}

start_gps_probes() {
  start_single_gps_probe "${GPS_PROBE_ODOM_TOPIC}" "${GPS_PROBE_CSV_PATH}" "${GPS_VS_POSE_LOG}" GPS_POSE_PID
  start_single_gps_probe "${GPS_PROBE_SECOND_ODOM_TOPIC}" "${GPS_PROBE_SECOND_CSV_PATH}" "${GPS_VS_SECOND_POSE_LOG}" GPS_SECOND_POSE_PID
}

show_status() {
  load_pids

  local agent_pid_current px4_pid_current mavros_pid_current qgc_pid_current monitor_pid_current compare_pid_current plotjuggler_pid_current
  local gps_pose_state gps_second_pose_state
  agent_pid_current="$(first_matching_pid "${AGENT_PATTERN}")"
  px4_pid_current="$(first_matching_pid "${PX4_PATTERN}|${PX4_SITL_RUN_PATTERN}|${GZSERVER_PATTERN}|${GZCLIENT_PATTERN}")"
  mavros_pid_current="$(first_matching_pid "${MAVROS_PATTERN}")"
  qgc_pid_current="$(first_matching_pid "${QGC_PATTERN}")"
  monitor_pid_current="$(first_matching_pid "${MONITOR_PATTERN}")"
  compare_pid_current="$(first_matching_pid "${COMPARE_PATTERN}")"
  plotjuggler_pid_current="$(first_matching_pid "${PLOTJUGGLER_PATTERN}")"
  if [[ -n "${GPS_POSE_PID:-}" ]] && is_running "${GPS_POSE_PID}"; then
    gps_pose_state="running"
  else
    gps_pose_state="stopped"
  fi
  if [[ -n "${GPS_SECOND_POSE_PID:-}" ]] && is_running "${GPS_SECOND_POSE_PID}"; then
    gps_second_pose_state="running"
  else
    gps_second_pose_state="stopped"
  fi

  echo
  log "run_dir=${RUN_DIR}"
  printf '  agent:       %s pid=%s\n'  "$([[ -n "${agent_pid_current}" ]] && echo running || echo stopped)"       "${agent_pid_current}"
  printf '  px4:         %s pid=%s\n'  "$([[ -n "${px4_pid_current}" ]] && echo running || echo stopped)"         "${px4_pid_current}"
  printf '  mavros:      %s pid=%s\n'  "$([[ -n "${mavros_pid_current}" ]] && echo running || echo stopped)"      "${mavros_pid_current}"
  printf '  qgc:         %s pid=%s\n'  "$([[ -n "${qgc_pid_current}" ]] && echo running || echo stopped)"         "${qgc_pid_current}"
  printf '  monitor:     %s pid=%s\n'  "$([[ -n "${monitor_pid_current}" ]] && echo running || echo stopped)"     "${monitor_pid_current}"
  printf '  compare:     %s pid=%s\n'  "$([[ -n "${compare_pid_current}" ]] && echo running || echo stopped)"     "${compare_pid_current}"
  printf '  plotjuggler: %s pid=%s\n'  "$([[ -n "${plotjuggler_pid_current}" ]] && echo running || echo stopped)" "${plotjuggler_pid_current}"
  printf '  gps_vs_pose: %s pid=%s\n'  "${gps_pose_state}" "${GPS_POSE_PID:-}"
  printf '  gps_vs_raw:  %s pid=%s\n'  "${gps_second_pose_state}" "${GPS_SECOND_POSE_PID:-}"

  echo
  log "summary"
  echo "  ready_for_takeoff=$(count_fixed 'Ready for takeoff!' "${PX4_LOG}")"
  echo "  simulator_poll_timeout=$(count_fixed 'ERROR [simulator_mavlink] poll timeout' "${PX4_LOG}")"
  echo "  px4_timesync_warn=$(count_fixed 'WARN  [timesync] RTT too high' "${PX4_LOG}")"
  echo "  px4_timesync_err=$(count_fixed 'ERROR [timesync]' "${PX4_LOG}")"
  echo "  mavros_connected=$(count_fixed 'CON: Got HEARTBEAT, connected. FCU: PX4 Autopilot' "${MAVROS_LOG}")"
  echo "  mavros_timesync_warn=$(count_fixed 'TM: RTT too high for timesync:' "${MAVROS_LOG}")"
  echo "  mavros_timesync_err=$(count_fixed 'TM: Time jump detected. Resetting time synchroniser.' "${MAVROS_LOG}")"
  echo "  mavros_guided_no_origin=$(count_fixed 'PositionTargetGlobal failed because no origin' "${MAVROS_LOG}")"
  echo "  compare_started=$(count_fixed '[INFO] [launch]: All log files can be found below' "${COMPARE_LOG}")"
  echo "  comparison_topics_seen=$(count_fixed '/comparison/' "${COMPARE_LOG}")"
  echo "  state_publish_debug_csv_present=$([[ -s "${COMPARE_STATE_PUBLISH_DEBUG_CSV_PATH}" ]] && echo 1 || echo 0)"
  echo "  gps_vs_pose_csv_present=$([[ -s "${GPS_PROBE_CSV_PATH}" ]] && echo 1 || echo 0)"
  echo "  gps_vs_second_csv_present=$([[ -s "${GPS_PROBE_SECOND_CSV_PATH}" ]] && echo 1 || echo 0)"

  echo
  log "logs"
  echo "  ${AGENT_LOG}"
  echo "  ${PX4_LOG}"
  echo "  ${MAVROS_LOG}"
  echo "  ${QGC_LOG}"
  echo "  ${MONITOR_LOG}"
  echo "  ${COMPARE_LOG}"
  echo "  ${PLOTJUGGLER_LOG}"
  echo "  ${MISSION_PREFLIGHT_LOG}"
  echo "  ${GPS_VS_POSE_LOG}"
  echo "  ${GPS_VS_SECOND_POSE_LOG}"
}

start_all() {
  if [[ -f "${PID_FILE}" ]]; then
    load_pids
    if is_running "${PX4_PID:-}" || is_running "${MAVROS_PID:-}" || is_running "${QGC_PID:-}" || is_running "${AGENT_PID:-}"; then
      log "existing session appears to be running under ${RUN_DIR}; use 'status' or 'stop' first"
      exit 1
    fi
  fi

  require_no_global_runtime_conflicts "start"

  : > "${AGENT_LOG}"
  : > "${PX4_LOG}"
  : > "${MAVROS_LOG}"
  : > "${QGC_LOG}"
  : > "${MONITOR_LOG}"
  : > "${COMPARE_LOG}"
  : > "${PLOTJUGGLER_LOG}"
  : > "${MISSION_PREFLIGHT_LOG}"
  : > "${GPS_VS_POSE_LOG}"
  : > "${GPS_VS_SECOND_POSE_LOG}"

  prepare_mavros_pluginlist_override
  prepare_mavros_config_override
  start_agent
  start_px4_gui

  wait_for_log_fixed "Ready for takeoff!" "${PX4_LOG}" "${PX4_READY_TIMEOUT_SEC}" "PX4 ready for takeoff"
  log "settling PX4/Gazebo GUI for ${PX4_POST_READY_SETTLE_SEC}s before external clients..."
  sleep "${PX4_POST_READY_SETTLE_SEC}"

  if [[ "${START_MAVROS}" == "1" ]]; then
    start_mavros
    wait_for_log_fixed "CON: Got HEARTBEAT, connected. FCU: PX4 Autopilot" "${MAVROS_LOG}" "${MAVROS_CONNECT_TIMEOUT_SEC}" "MAVROS heartbeat"
    run_mission_preflight_reset
  fi

  if [[ "${START_MONITOR}" == "1" ]]; then
    start_monitor
  fi

  if [[ "${START_QGC}" == "1" ]]; then
    log "waiting ${QGC_DELAY_AFTER_MAVROS_SEC}s before QGC..."
    sleep "${QGC_DELAY_AFTER_MAVROS_SEC}"
    start_qgc
  fi

  if [[ "${START_COMPARE}" == "1" ]]; then
    log "waiting ${COMPARE_DELAY_AFTER_QGC_SEC}s before compare/RViz..."
    sleep "${COMPARE_DELAY_AFTER_QGC_SEC}"
    start_compare
  fi

  if [[ "${START_GPS_PROBES}" == "1" ]]; then
    log "waiting ${GPS_PROBE_DELAY_AFTER_COMPARE_SEC}s before GPS probes..."
    sleep "${GPS_PROBE_DELAY_AFTER_COMPARE_SEC}"
    start_gps_probes
  fi

  if [[ "${START_PLOTJUGGLER}" == "1" ]]; then
    log "waiting ${PLOTJUGGLER_DELAY_AFTER_COMPARE_SEC}s before PlotJuggler..."
    sleep "${PLOTJUGGLER_DELAY_AFTER_COMPARE_SEC}"
    start_plotjuggler
  fi

  show_status
}

start_optional_client() {
  local client="$1"
  load_pids

  case "${client}" in
    qgc)
      require_current_core_runtime "start-qgc"
      build_current_session_ignore_pids
      require_no_global_runtime_conflicts "start-qgc" \
        "${CURRENT_SESSION_IGNORE_PIDS[@]}"
      if [[ -n "$(first_matching_pid "${QGC_PATTERN}")" ]]; then
        log "QGC is already running"
      else
        start_qgc
      fi
      ;;
    compare)
      require_current_core_runtime "start-compare"
      build_current_session_ignore_pids
      require_no_global_runtime_conflicts "start-compare" \
        "${CURRENT_SESSION_IGNORE_PIDS[@]}"
      if [[ -n "$(first_matching_pid "${COMPARE_PATTERN}")" ]]; then
        log "compare/RViz is already running"
      else
        start_compare
      fi
      ;;
    plotjuggler)
      require_current_core_runtime "start-plotjuggler"
      build_current_session_ignore_pids
      require_no_global_runtime_conflicts "start-plotjuggler" \
        "${CURRENT_SESSION_IGNORE_PIDS[@]}"
      if [[ -n "$(first_matching_pid "${PLOTJUGGLER_PATTERN}")" ]]; then
        log "PlotJuggler is already running"
      else
        start_plotjuggler
      fi
      ;;
    gps-probes)
      require_current_core_runtime "start-gps-probes"
      build_current_session_ignore_pids
      require_no_global_runtime_conflicts "start-gps-probes" \
        "${CURRENT_SESSION_IGNORE_PIDS[@]}"
      if { [[ -n "${GPS_POSE_PID:-}" ]] && is_running "${GPS_POSE_PID:-}"; } || { [[ -n "${GPS_SECOND_POSE_PID:-}" ]] && is_running "${GPS_SECOND_POSE_PID:-}"; }; then
        log "GPS probes are already running"
      else
        start_gps_probes
      fi
      ;;
    *)
      log "unknown optional client: ${client}"
      exit 1
      ;;
  esac

  show_status
}

stop_all() {
  load_pids
  log "stopping session under ${RUN_DIR}"
  kill_if_running "${GPS_SECOND_POSE_PID:-}"
  kill_if_running "${GPS_POSE_PID:-}"
  kill_if_running "${MONITOR_PID:-}"
  kill_if_running "${PLOTJUGGLER_PID:-}"
  kill_if_running "${COMPARE_PID:-}"
  kill_if_running "${QGC_PID:-}"
  kill_if_running "${MAVROS_PID:-}"
  kill_if_running "${PX4_PID:-}"
  kill_if_running "${AGENT_PID:-}"
  kill_pattern_if_running "${MONITOR_PATTERN}"
  kill_pattern_if_running "${PLOTJUGGLER_PATTERN}"
  kill_pattern_if_running "${COMPARE_PATTERN}"
  kill_pattern_if_running "${QGC_PATTERN}"
  kill_pattern_if_running "${GPS_PROBE_PATTERN}"
  kill_pattern_if_running "${MAVROS_PATTERN}"
  kill_pattern_if_running "${GZCLIENT_PATTERN}"
  kill_pattern_if_running "${GZSERVER_PATTERN}"
  kill_pattern_if_running "${PX4_PATTERN}"
  kill_pattern_if_running "${PX4_SITL_RUN_PATTERN}"
  kill_pattern_if_running "${AGENT_PATTERN}"
  rm -f "${PID_FILE}"
  show_status || true
}

case "${CMD}" in
  start)
    start_all
    ;;
  start-qgc)
    start_optional_client qgc
    ;;
  start-compare)
    start_optional_client compare
    ;;
  start-plotjuggler)
    start_optional_client plotjuggler
    ;;
  start-gps-probes)
    start_optional_client gps-probes
    ;;
  stop)
    stop_all
    ;;
  status)
    show_status
    ;;
  *)
    echo "usage: $0 {start|start-qgc|start-compare|start-plotjuggler|start-gps-probes|stop|status} [run_dir]" >&2
    exit 1
    ;;
esac
