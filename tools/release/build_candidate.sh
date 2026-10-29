#!/bin/bash
# Build-only. Never source this candidate into a running robot or launch nodes.
set -eo pipefail
navigation_repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
if [[ $# != 1 || "$1" != /* ]]; then
  echo "Usage: bash build_candidate.sh /absolute/new-candidate-directory" >&2
  exit 2
fi
candidate_root="$1"
if [[ -e "${candidate_root}" ]]; then
  echo "Refusing to reuse existing candidate directory: ${candidate_root}" >&2
  exit 2
fi
if [[ -n "$(git -C "${navigation_repo}" status --porcelain --untracked-files=normal)" ]]; then
  echo "Refusing candidate build from a dirty source tree" >&2
  exit 2
fi
mkdir -p "${candidate_root}"
git -C "${navigation_repo}" rev-parse HEAD > "${candidate_root}/source-sha.txt"
# Explicit isolated output roots; no symlink-install, so installed artifacts do
# not follow subsequent source/build edits in the development tree.
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_SHLVL
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH
export PATH=/usr/bin:/bin:/opt/ros/humble/bin
source /opt/ros/humble/setup.bash
source /home/rpp/rpp_ws/install/local_setup.bash
colcon --log-base "${candidate_root}/log" build \
  --base-paths "${navigation_repo}/packages" \
  --build-base "${candidate_root}/build" \
  --install-base "${candidate_root}/install" \
  --parallel-workers 2 \
  --packages-select r1_neupan_runtime nav2_controller navigation_neupan_bridge \
    r1_collision_guard r1_neupan_integration navigation_2d r1_lidar_preprocess \
  --allow-overriding nav2_controller navigation_neupan_bridge r1_collision_guard \
    r1_neupan_integration navigation_2d r1_lidar_preprocess \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DPython3_EXECUTABLE=/usr/bin/python3 \
    -Dfmt_DIR=/usr/lib/x86_64-linux-gnu/cmake/fmt
# A changed source snapshot during the build makes the candidate ambiguous.
[[ "$(git -C "${navigation_repo}" rev-parse HEAD)" == "$(cat "${candidate_root}/source-sha.txt")" ]]
[[ -z "$(git -C "${navigation_repo}" status --porcelain --untracked-files=normal)" ]]
echo "BUILD ONLY: candidate requires inventory, compatibility, acceptance and operator approval."
