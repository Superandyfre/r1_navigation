#!/bin/bash

source "$(dirname "$0")/run_once.sh"
single_instance_or_exit "starterbutton-navigation-high-sampling-neupan"

source ~/.rpprc

# Humble's Python extensions are built for system Python 3.10.  The button
# must not inherit an interactive Conda Python when starting ROS executables.
export PATH=/usr/bin:/bin:$PATH

# Starterbutton runs outside the interactive GNOME session, so it does not
# inherit the Xwayland connection variables required by the RViz node in the
# launch file. Recover them from the active local Xwayland process.
if [[ -z "${DISPLAY:-}" ]]; then
  xwayland_args="$(pgrep -a -u "$(id -u)" Xwayland 2>/dev/null | head -n 1 || true)"
  xwayland_display="$(sed -nE 's#.*Xwayland ([^ ]+).*#\1#p' <<<"${xwayland_args}")"
  xwayland_auth="$(sed -nE 's#.* -auth ([^ ]+).*#\1#p' <<<"${xwayland_args}")"
  if [[ -n "${xwayland_display}" && -r "${xwayland_auth}" ]]; then
    export DISPLAY="${xwayland_display}"
    export XAUTHORITY="${xwayland_auth}"
  fi
fi

left_arm_ip="${RPP_LEFT_ARM_IP:-192.168.11.60}"
right_arm_ip="${RPP_RIGHT_ARM_IP:-192.168.11.61}"
arm_local_ip="${RPP_ARM_LOCAL_IP:-192.168.11.11}"
# UI 7 is the production dual-LiDAR / live-arm-footprint profile. Operators
# may still override these values explicitly for a diagnostic session.
scan_source="${RPP_NEUPAN_SCAN_SOURCE:-dual}"
mesh_self_filter="${RPP_NEUPAN_USE_MESH_SELF_FILTER:-true}"

if [[ "${scan_source}" != "livox_only" && "${scan_source}" != "dual" ]]; then
  echo "RPP_NEUPAN_SCAN_SOURCE must be livox_only or dual" >&2
  exit 1
fi

if [[ "${mesh_self_filter}" != "false" && "${mesh_self_filter}" != "true" ]]; then
  echo "RPP_NEUPAN_USE_MESH_SELF_FILTER must be true or false" >&2
  exit 1
fi

pointcloud_prefix="$(ros2 pkg prefix pointcloud_to_laserscan 2>/dev/null || true)"
if [[ ! -x "$pointcloud_prefix/lib/pointcloud_to_laserscan/pointcloud_to_laserscan_node" ]]; then
  echo "Missing pointcloud_to_laserscan in the workspace install prefix" >&2
  exit 1
fi

# Keep ROS launch in its own session while binding that session to this script.
# The helper closes inherited descriptors and receives SIGTERM if this script's
# terminal is closed, so neither a stale lock nor a partial navigation graph
# can survive the starterbutton process.
launch_pid=""
launch_finished=false

terminate_launch_group() {
  local pgid=""
  local target=""
  local signal=""
  local attempts=""

  [[ -z "${launch_pid}" || "${launch_finished}" == true ]] && return
  pgid="$(ps -o pgid= -p "${launch_pid}" 2>/dev/null | tr -d ' ')"
  if [[ "${pgid}" == "${launch_pid}" ]]; then
    target="-${pgid}"
  else
    # Do not risk signalling this script's own process group if setsid failed.
    target="${launch_pid}"
  fi

  for signal in INT TERM KILL; do
    kill -"${signal}" -- "${target}" 2>/dev/null || break
    for attempts in {1..20}; do
      if [[ "${target}" == -* ]]; then
        kill -0 -- "${target}" 2>/dev/null || break 2
      elif ! kill -0 "${launch_pid}" 2>/dev/null; then
        break 2
      fi
      sleep 0.25
    done
  done
}

on_exit_signal() {
  terminate_launch_group
  exit 130
}

trap on_exit_signal HUP INT TERM

/usr/bin/python3 "$(dirname "$0")/launch_with_parent_death.py" \
  ros2 launch navigation_2d local_and_nav_high_sampling_controller.launch.py \
  local_controller_mode:=neupan \
  scan_source:="${scan_source}" \
  use_mesh_self_filter:="${mesh_self_filter}" \
  calibration_record:=/home/rpp/rpp_ws/config/r1_hw1_calibration.json \
  start_arm_drivers:=false \
  left_robot_ip:="${left_arm_ip}" \
  right_robot_ip:="${right_arm_ip}" \
  local_ip:="${arm_local_ip}" 9>&- &
launch_pid=$!

wait "${launch_pid}"
launch_status=$?
launch_finished=true
exit "${launch_status}"
