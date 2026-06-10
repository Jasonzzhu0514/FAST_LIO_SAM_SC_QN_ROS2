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

src/fast_lio_sam_sc_qn2/scripts/prepare_livox_ros_driver2.sh
colcon build --symlink-install --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS="${ROS_DISTRO}"
