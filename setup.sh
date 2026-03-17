#!/bin/bash
################################################################
## 一键配置脚本
## 功能：配置 CAN0/CAN1/CAN2 接口 + USB 串口权限
################################################################

set -e

SUDO_PASS="linkerhand"

run_sudo() {
  echo "$SUDO_PASS" | sudo -S -p '' "$@"
}

configure_can() {
  local can_if="$1"
  local bitrate="$2"
  local can_status

  can_status="$(ip -o link show "$can_if" 2>/dev/null || true)"

  if [[ "$can_status" == *"state UP"* ]]; then
    echo "  -> $can_if 已在工作，跳过配置"
    return 0
  fi

  run_sudo ip link set "$can_if" down 2>/dev/null || true
  run_sudo ip link set "$can_if" type can bitrate "$bitrate"
  run_sudo ip link set "$can_if" up
  run_sudo ip link set "$can_if" txqueuelen 100
  echo "  -> $can_if 配置完成"
}

echo "========================================="
echo "  一键配置 CAN 接口 & USB 串口权限"
echo "========================================="

# ---- 加载 CAN 内核模块 ----
echo "[1/5] 加载 CAN 内核模块..."
run_sudo modprobe can
run_sudo modprobe can_raw
run_sudo modprobe can_dev
echo "  -> CAN 内核模块加载完成"

# ---- 配置 CAN0 ----
echo "[2/5] 配置 CAN0..."
configure_can can0 1000000

# ---- 配置 CAN1 ----
echo "[3/5] 配置 CAN1..."
configure_can can1 1000000

# ---- 配置 CAN2 (供弹电机) ----
echo "[4/5] 配置 CAN2..."
configure_can can2 1000000

# ---- 配置 USB 串口权限 ----
echo "[5/5] 配置 USB 串口权限..."
run_sudo chmod 666 /dev/ttyUSB0
run_sudo chmod 666 /dev/ttyUSB1
echo "  -> USB 串口权限配置完成"

# ---- 验证 ----
echo ""
echo "========================================="
echo "  配置结果"
echo "========================================="
echo "CAN0 状态:"
ip link show can0 | head -2
echo ""
echo "CAN1 状态:"
ip link show can1 | head -2
echo ""
echo "CAN2 状态:"
ip link show can2 | head -2
echo ""
echo "USB 串口权限:"
ls -l /dev/ttyUSB0 /dev/ttyUSB1 2>/dev/null || echo "  警告: 部分 USB 设备未找到"
echo ""
echo "========================================="
echo "  所有配置完成!"
echo "========================================="
