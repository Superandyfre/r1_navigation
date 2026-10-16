#!/bin/bash
set -eo pipefail

navigation_repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Conda libraries conflict with Humble's BehaviorTree.CPP link dependencies.
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_SHLVL
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH
export PATH=/usr/bin:/bin:/opt/ros/humble/bin

source /opt/ros/humble/setup.bash
source /home/rpp/rpp_ws/install/local_setup.bash

colcon --log-base "${navigation_repo}/log" build \
  --base-paths "${navigation_repo}/packages" \
  --build-base "${navigation_repo}/build" \
  --install-base "${navigation_repo}/install" \
  --symlink-install --cmake-clean-cache \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
