#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
WS_ROOT="$(cd "${PKG_DIR}/../.." && pwd)"
DRIVER_DIR="${WS_ROOT}/src/livox_ros_driver2"

if [[ ! -f "${DRIVER_DIR}/package_ROS2.xml" ]]; then
  echo "livox_ros_driver2 is missing. Initialize submodules first: git submodule update --init --recursive" >&2
  exit 1
fi

cp "${DRIVER_DIR}/package_ROS2.xml" "${DRIVER_DIR}/package.xml"
echo "livox_ros_driver2 ROS 2 package.xml is ready at ${DRIVER_DIR}/package.xml"
