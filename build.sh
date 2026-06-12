#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")"
if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "ROS_DISTRO is not set. Source your ROS 2 setup first, for example: source /opt/ros/<distro>/setup.bash" >&2
  exit 1
fi

if [[ "${ROS_VERSION:-}" != "2" ]]; then
  echo "ROS_VERSION is not 2. Source a ROS 2 setup first, for example: source /opt/ros/<distro>/setup.bash" >&2
  exit 1
fi

missing_ros2_pkgs=()
for pkg in pcl_ros pcl_conversions; do
  if ! ros2 pkg prefix "${pkg}" >/dev/null 2>&1; then
    missing_ros2_pkgs+=("${pkg}")
  fi
done

if ((${#missing_ros2_pkgs[@]})); then
  echo "Missing ROS 2 package(s): ${missing_ros2_pkgs[*]}" >&2
  echo "For ${ROS_DISTRO}, install them with:" >&2
  printf '  sudo apt install' >&2
  for pkg in "${missing_ros2_pkgs[@]}"; do
    printf ' ros-%s-%s' "${ROS_DISTRO}" "${pkg//_/-}" >&2
  done
  printf '\n' >&2
  exit 1
fi

if [[ ":${CMAKE_PREFIX_PATH:-}:" == *":/opt/ros/noetic:"* && "${ROS_DISTRO}" != "noetic" ]]; then
  echo "Warning: CMAKE_PREFIX_PATH still contains /opt/ros/noetic. Use a clean terminal or unset ROS 1 paths before building ROS 2." >&2
fi

src/fast_lio_sam_sc_qn2/scripts/prepare_livox_ros_driver2.sh
colcon build --symlink-install \
  --base-paths \
    src/livox_ros_driver2 \
    src/fast_lio_sam_sc_qn2 \
    src/web_mapping \
    src/fast_lio_sam_sc_qn2/third_party/FAST_LIO \
  "$@" \
  --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS="${ROS_DISTRO}"
