#!/usr/bin/env bash
set -euo pipefail

WS_DIR="/home/linkerhand/RMUL2026/rm26_ros2_ws"
cd "${WS_DIR}"

# ROS setup 脚本在部分环境会访问未定义变量，这里临时关闭 -u 避免中断
set +u
source /opt/ros/foxy/setup.bash
source "${WS_DIR}/install/setup.bash"
set -u

exec /usr/bin/python3 "${WS_DIR}/robot_monitor_web/app.py"
