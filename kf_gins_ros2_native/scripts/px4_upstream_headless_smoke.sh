#!/usr/bin/env bash

set -euo pipefail

WS_ROOT="${HOME}/kf_gins_ws"
PX4_ROOT="${HOME}/PX4-Autopilot"
AGENT_BIN="${WS_ROOT}/tools/microxrce_install/bin/MicroXRCEAgent"
AGENT_LIB="${WS_ROOT}/tools/microxrce_install/lib"

READY_WAIT_SEC="${1:-20}"
HZ_SAMPLE_SEC="${2:-10}"
STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="${3:-/tmp/px4_upstream_headless_smoke_${STAMP}}"
SMOKE_MODE="${PX4_SMOKE_MODE:-headless}"
ENABLE_MAVROS="${PX4_SMOKE_ENABLE_MAVROS:-0}"
ENABLE_QGC="${PX4_SMOKE_ENABLE_QGC:-0}"
EXTERNAL_START_DELAY_SEC="${PX4_SMOKE_EXTERNAL_START_DELAY_SEC:-5}"
MAVROS_FCU_URL="${PX4_SMOKE_MAVROS_FCU_URL:-udp://:14540@127.0.0.1:14557}"
PX4_SMOKE_MAVROS_PROFILE="${PX4_SMOKE_MAVROS_PROFILE:-default}"
PX4_SMOKE_MAVROS_CONFIG_YAML="${PX4_SMOKE_MAVROS_CONFIG_YAML:-}"
PX4_SMOKE_MAVROS_PLUGINLIST_YAML="${PX4_SMOKE_MAVROS_PLUGINLIST_YAML:-}"
QGC_BIN="${PX4_SMOKE_QGC_BIN:-${HOME}/QGroundControl-x86_64.AppImage}"
EXTERNAL_RUNTIME_SEC="${PX4_SMOKE_EXTERNAL_RUNTIME_SEC:-25}"

AGENT_LOG="${LOG_DIR}/agent.log"
PX4_LOG="${LOG_DIR}/px4_sitl.log"
HZ_LOG="${LOG_DIR}/sensor_combined_hz.log"
ROS_LOG_DIR_SMOKE="${LOG_DIR}/ros_logs"
PX4_LOG_SANITIZED="${LOG_DIR}/px4_sitl_sanitized.log"
MAVROS_LOG="${LOG_DIR}/mavros.log"
QGC_LOG="${LOG_DIR}/qgc.log"
KFGINS_CONFIG_DIR="${WS_ROOT}/src/kf_gins_ros2_native/config"
KFGINS_MAVROS_RUNTIME_MINIMAL_PLUGINLIST="${KFGINS_CONFIG_DIR}/mavros_px4_runtime_minimal_pluginlist.yaml"
KFGINS_MAVROS_RUNTIME_MINIMAL_CONFIG="${KFGINS_CONFIG_DIR}/mavros_px4_runtime_minimal_config.yaml"
MAVROS_PLUGINLIST_YAML_EFFECTIVE="/opt/ros/humble/share/mavros/launch/px4_pluginlists.yaml"
MAVROS_CONFIG_YAML_EFFECTIVE="/opt/ros/humble/share/mavros/launch/px4_config.yaml"

mkdir -p "${LOG_DIR}"
mkdir -p "${ROS_LOG_DIR_SMOKE}"

PX4_PID=""
AGENT_PID=""
MAVROS_PID=""
QGC_PID=""

cleanup() {
  set +e
  if [[ -n "${PX4_PID}" ]]; then
    kill "${PX4_PID}" 2>/dev/null || true
    wait "${PX4_PID}" 2>/dev/null || true
    PX4_PID=""
  fi
  if [[ -n "${AGENT_PID}" ]]; then
    kill "${AGENT_PID}" 2>/dev/null || true
    wait "${AGENT_PID}" 2>/dev/null || true
    AGENT_PID=""
  fi
  if [[ -n "${MAVROS_PID}" ]]; then
    kill "${MAVROS_PID}" 2>/dev/null || true
    wait "${MAVROS_PID}" 2>/dev/null || true
    MAVROS_PID=""
  fi
  if [[ -n "${QGC_PID}" ]]; then
    kill "${QGC_PID}" 2>/dev/null || true
    wait "${QGC_PID}" 2>/dev/null || true
    QGC_PID=""
  fi
}

count_matches() {
  local pattern="$1"
  local file="$2"
  local count
  if [[ ! -f "${file}" ]]; then
    echo 0
    return
  fi
  count="$( (grep -F -o -- "${pattern}" "${file}" || true) | wc -l )"
  echo "${count}"
}

source_setup() {
  set +u
  source /opt/ros/humble/setup.bash
  if [[ -f "${WS_ROOT}/install/setup.bash" ]]; then
    source "${WS_ROOT}/install/setup.bash"
  fi
  set -u
}

trap cleanup EXIT INT TERM

echo "[px4-upstream-smoke] log_dir=${LOG_DIR}"
echo "[px4-upstream-smoke] mode=${SMOKE_MODE} ready_wait_sec=${READY_WAIT_SEC} hz_sample_sec=${HZ_SAMPLE_SEC}"
echo "[px4-upstream-smoke] enable_mavros=${ENABLE_MAVROS} enable_qgc=${ENABLE_QGC} external_start_delay_sec=${EXTERNAL_START_DELAY_SEC}"

source_setup

export LD_LIBRARY_PATH="${AGENT_LIB}:${LD_LIBRARY_PATH:-}"
export ROS_DOMAIN_ID=0
export ROS_LOG_DIR="${ROS_LOG_DIR_SMOKE}"
unset ROS_LOCALHOST_ONLY

if [[ ! -x "${AGENT_BIN}" ]]; then
  echo "[px4-upstream-smoke] missing agent binary: ${AGENT_BIN}" >&2
  exit 1
fi

if [[ ! -d "${PX4_ROOT}" ]]; then
  echo "[px4-upstream-smoke] missing PX4 root: ${PX4_ROOT}" >&2
  exit 1
fi

case "${PX4_SMOKE_MAVROS_PROFILE}" in
  default)
    ;;
  runtime_minimal)
    MAVROS_PLUGINLIST_YAML_EFFECTIVE="${KFGINS_MAVROS_RUNTIME_MINIMAL_PLUGINLIST}"
    MAVROS_CONFIG_YAML_EFFECTIVE="${KFGINS_MAVROS_RUNTIME_MINIMAL_CONFIG}"
    ;;
  *)
    echo "[px4-upstream-smoke] unsupported PX4_SMOKE_MAVROS_PROFILE=${PX4_SMOKE_MAVROS_PROFILE}" >&2
    exit 1
    ;;
esac

if [[ -n "${PX4_SMOKE_MAVROS_PLUGINLIST_YAML}" ]]; then
  MAVROS_PLUGINLIST_YAML_EFFECTIVE="${PX4_SMOKE_MAVROS_PLUGINLIST_YAML}"
fi
if [[ -n "${PX4_SMOKE_MAVROS_CONFIG_YAML}" ]]; then
  MAVROS_CONFIG_YAML_EFFECTIVE="${PX4_SMOKE_MAVROS_CONFIG_YAML}"
fi

echo "[px4-upstream-smoke] starting agent..."
"${AGENT_BIN}" udp4 -p 8888 >"${AGENT_LOG}" 2>&1 &
AGENT_PID=$!
sleep 2

echo "[px4-upstream-smoke] starting ${SMOKE_MODE^^} PX4/Gazebo..."
(
  set -euo pipefail
  set +u
  source /opt/ros/humble/setup.bash
  set -u
  cd "${PX4_ROOT}"
  export ROS_DOMAIN_ID=0
  export ROS_LOG_DIR="${ROS_LOG_DIR_SMOKE}"
  export PX4_UXRCE_DDS_PORT=8888
  unset PX4_UXRCE_DDS_NS
  export PX4_SITL_WORLD=city_dense
  case "${SMOKE_MODE}" in
    headless)
      HEADLESS=1 make px4_sitl gazebo-classic_iris >"${PX4_LOG}" 2>&1
      ;;
    gui)
      unset HEADLESS
      make px4_sitl gazebo-classic_iris >"${PX4_LOG}" 2>&1
      ;;
    *)
      echo "[px4-upstream-smoke] unsupported PX4_SMOKE_MODE=${SMOKE_MODE}" >&2
      exit 1
      ;;
  esac
) &
PX4_PID=$!

if [[ "${ENABLE_MAVROS}" == "1" ]]; then
  echo "[px4-upstream-smoke] scheduling MAVROS after ${EXTERNAL_START_DELAY_SEC}s..."
  (
    sleep "${EXTERNAL_START_DELAY_SEC}"
    set +u
    source /opt/ros/humble/setup.bash
    if [[ -f "${WS_ROOT}/install/setup.bash" ]]; then
      source "${WS_ROOT}/install/setup.bash"
    fi
    set -u
    export ROS_DOMAIN_ID=0
    export ROS_LOG_DIR="${ROS_LOG_DIR_SMOKE}"
    unset ROS_LOCALHOST_ONLY
    timeout "${EXTERNAL_RUNTIME_SEC}" ros2 launch mavros node.launch \
      "fcu_url:=${MAVROS_FCU_URL}" \
      'gcs_url:=""' \
      'tgt_system:=1' \
      'tgt_component:=1' \
      "pluginlists_yaml:=${MAVROS_PLUGINLIST_YAML_EFFECTIVE}" \
      "config_yaml:=${MAVROS_CONFIG_YAML_EFFECTIVE}" \
      'log_output:=screen' \
      'fcu_protocol:=v2.0' \
      'respawn_mavros:=false' \
      'namespace:=mavros' >"${MAVROS_LOG}" 2>&1
  ) &
  MAVROS_PID=$!
fi

if [[ "${ENABLE_QGC}" == "1" ]]; then
  echo "[px4-upstream-smoke] scheduling QGC after ${EXTERNAL_START_DELAY_SEC}s..."
  (
    sleep "${EXTERNAL_START_DELAY_SEC}"
    timeout "${EXTERNAL_RUNTIME_SEC}" "${QGC_BIN}" >"${QGC_LOG}" 2>&1
  ) &
  QGC_PID=$!
fi

echo "[px4-upstream-smoke] waiting ${READY_WAIT_SEC}s for PX4/Gazebo..."
sleep "${READY_WAIT_SEC}"

echo "[px4-upstream-smoke] sampling /fmu/out/sensor_combined for ${HZ_SAMPLE_SEC}s..."
timeout "${HZ_SAMPLE_SEC}" bash -lc "
  set +u
  source /opt/ros/humble/setup.bash
  if [[ -f \"${WS_ROOT}/install/setup.bash\" ]]; then
    source \"${WS_ROOT}/install/setup.bash\"
  fi
  set -u
  export ROS_DOMAIN_ID=0
  export ROS_LOG_DIR="${ROS_LOG_DIR_SMOKE}"
  unset ROS_LOCALHOST_ONLY
  ros2 topic echo --qos-profile sensor_data --field timestamp /fmu/out/sensor_combined
" >"${HZ_LOG}" 2>&1 || true

cleanup

tr '\r' '\n' < "${PX4_LOG}" | sed -E 's/\x1B\[[0-9;]*[[:alpha:]]//g' > "${PX4_LOG_SANITIZED}" || cp "${PX4_LOG}" "${PX4_LOG_SANITIZED}"

px4_timeout_count="$(count_matches 'ERROR [simulator_mavlink] poll timeout' "${PX4_LOG_SANITIZED}")"
px4_timesync_warn_count="$(count_matches 'WARN  [timesync] RTT too high' "${PX4_LOG_SANITIZED}")"
px4_timesync_err_count="$(count_matches 'ERROR [timesync]' "${PX4_LOG_SANITIZED}")"
px4_ready_count="$(count_matches 'Ready for takeoff!' "${PX4_LOG_SANITIZED}")"
sensor_combined_samples="$(grep -E '^[0-9]+$' "${HZ_LOG}" 2>/dev/null | wc -l)"
sensor_combined_rate_hz="$(awk -v samples="${sensor_combined_samples}" -v secs="${HZ_SAMPLE_SEC}" 'BEGIN { if (secs > 0) printf "%.3f", samples / secs; else print "0.000" }')"
mavros_timesync_warn_count="$(count_matches 'TM: RTT too high for timesync:' "${MAVROS_LOG}")"
mavros_connected_count="$(count_matches 'CON: Got HEARTBEAT, connected. FCU: PX4 Autopilot' "${MAVROS_LOG}")"

echo
echo "[px4-upstream-smoke] summary"
echo "  ready_for_takeoff=${px4_ready_count}"
echo "  simulator_poll_timeout=${px4_timeout_count}"
echo "  timesync_warn=${px4_timesync_warn_count}"
echo "  timesync_err=${px4_timesync_err_count}"
echo "  sensor_combined_samples=${sensor_combined_samples}"
echo "  sensor_combined_rate_hz_approx=${sensor_combined_rate_hz}"
echo "  mavros_connected=${mavros_connected_count}"
echo "  mavros_timesync_warn=${mavros_timesync_warn_count}"

echo
echo "[px4-upstream-smoke] key PX4 lines"
rg -o "Ready for takeoff!|ERROR \\[simulator_mavlink\\] poll timeout [0-9, ]+|WARN  \\[timesync\\] RTT too high [0-9]+ ms|ERROR \\[timesync\\][^[:cntrl:]]*" "${PX4_LOG_SANITIZED}" | head -n 80 || true

echo
echo "[px4-upstream-smoke] sensor_combined sample tail"
tail -n 10 "${HZ_LOG}" || true

if [[ -f "${MAVROS_LOG}" ]]; then
  echo
  echo "[px4-upstream-smoke] MAVROS key lines"
  rg -o "CON: Got HEARTBEAT, connected\\. FCU: PX4 Autopilot|TM: RTT too high for timesync: [0-9.]+ ms\\." "${MAVROS_LOG}" | head -n 40 || true
fi

if [[ -f "${QGC_LOG}" ]]; then
  echo
  echo "[px4-upstream-smoke] QGC log tail"
  tail -n 20 "${QGC_LOG}" || true
fi

echo
echo "[px4-upstream-smoke] logs saved under ${LOG_DIR}"
