#!/usr/bin/env bash
set -euo pipefail

SERVICE_NAME="rm26-start-all.service"
WS_DIR="/home/linkerhand/RMUL2026/rm26_ros2_ws"
SRC_SERVICE_FILE="${WS_DIR}/${SERVICE_NAME}"
DST_SERVICE_FILE="/etc/systemd/system/${SERVICE_NAME}"

if [[ ! -f "${SRC_SERVICE_FILE}" ]]; then
  echo "未找到服务文件: ${SRC_SERVICE_FILE}"
  exit 1
fi

for f in start_all.py stop_all.py setup.sh; do
  if [[ ! -x "${WS_DIR}/${f}" ]]; then
    chmod +x "${WS_DIR}/${f}"
  fi
done

sudo cp "${SRC_SERVICE_FILE}" "${DST_SERVICE_FILE}"
sudo systemctl daemon-reload
sudo systemctl enable --now "${SERVICE_NAME}"
sudo systemctl status "${SERVICE_NAME}" --no-pager
