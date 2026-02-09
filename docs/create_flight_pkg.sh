#!/bin/bash

# 自动在当前仓库下创建一个 ros2_ws/src/flight_core 包
# 你可以参考/修改这个脚本，在真正的 ROS2 工作空间里运行。

set -e

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS2_WS="${ROOT_DIR}/ros2_ws"
PKG_NAME="flight_core"

echo "Root: ${ROOT_DIR}"
echo "ROS2 ws (demo): ${ROS2_WS}"

mkdir -p "${ROS2_WS}/src"
cd "${ROS2_WS}/src"

if [ -d "${PKG_NAME}" ]; then
  echo "Package ${PKG_NAME} already exists under ${ROS2_WS}/src, skip creating."
  exit 0
fi

mkdir -p ${PKG_NAME}/{action,msg,include/${PKG_NAME},src}

echo "Created skeleton for ${PKG_NAME} (actions/msg/include/src)."

