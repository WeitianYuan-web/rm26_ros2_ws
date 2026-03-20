#!/usr/bin/env bash
set -euo pipefail

SERVICE_NAME="rm26-start-all.service"
DST_SERVICE_FILE="/etc/systemd/system/${SERVICE_NAME}"

if systemctl list-unit-files | awk '{print $1}' | rg -x "${SERVICE_NAME}" >/dev/null 2>&1; then
  sudo systemctl disable --now "${SERVICE_NAME}" || true
fi

if [[ -f "${DST_SERVICE_FILE}" ]]; then
  sudo rm -f "${DST_SERVICE_FILE}"
fi

sudo systemctl daemon-reload

echo "已取消开机自启并停止 ${SERVICE_NAME}。"
echo "当前启用状态："
systemctl is-enabled "${SERVICE_NAME}" 2>/dev/null || echo "disabled"
echo "当前运行状态："
systemctl is-active "${SERVICE_NAME}" 2>/dev/null || echo "inactive"
