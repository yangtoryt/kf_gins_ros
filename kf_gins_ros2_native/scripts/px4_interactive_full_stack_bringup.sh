#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Flight-safe default: keep the timing-sensitive mission test path small.
# Start QGC/RViz/PlotJuggler explicitly after the PX4+MAVROS baseline is stable:
#   ros2 run kf_gins_ros2_native px4_interactive_safe_bringup.sh start-qgc /tmp/px4_full_run
#   ros2 run kf_gins_ros2_native px4_interactive_safe_bringup.sh start-compare /tmp/px4_full_run
#   ros2 run kf_gins_ros2_native px4_interactive_safe_bringup.sh start-plotjuggler /tmp/px4_full_run
export PX4_SAFE_START_MAVROS="${PX4_SAFE_START_MAVROS:-1}"
export PX4_SAFE_START_QGC="${PX4_SAFE_START_QGC:-0}"
export PX4_SAFE_START_MONITOR="${PX4_SAFE_START_MONITOR:-1}"
export PX4_SAFE_START_COMPARE="${PX4_SAFE_START_COMPARE:-0}"
export PX4_SAFE_START_PLOTJUGGLER="${PX4_SAFE_START_PLOTJUGGLER:-0}"

exec "${SCRIPT_DIR}/px4_interactive_safe_bringup.sh" "$@"
