#!/bin/bash
################################################################
## 一键配置脚本
## 功能：配置 CAN0/CAN1/CAN2 接口 + USB 串口权限
################################################################

set -e

echo "========================================="
echo "  一键配置 CAN 接口 & USB 串口权限"
echo "========================================="

# ---- 加载 CAN 内核模块 ----
echo "[1/5] 加载 CAN 内核模块..."
sudo modprobe can
sudo modprobe can_raw
sudo modprobe can_dev
echo "  -> CAN 内核模块加载完成"

# ---- 配置 CAN0 ----
echo "[2/5] 配置 CAN0..."
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
sudo ip link set can0 txqueuelen 100
echo "  -> CAN0 配置完成"

# ---- 配置 CAN1 ----
echo "[3/5] 配置 CAN1..."
sudo ip link set can1 type can bitrate 1000000
sudo ip link set can1 up
sudo ip link set can1 txqueuelen 100
echo "  -> CAN1 配置完成"

# ---- 配置 CAN2 (供弹电机) ----
echo "[4/5] 配置 CAN2..."
sudo ip link set can2 type can bitrate 1000000
sudo ip link set can2 up
sudo ip link set can2 txqueuelen 100
echo "  -> CAN2 配置完成"

# ---- 配置 USB 串口权限 ----
echo "[5/5] 配置 USB 串口权限..."
sudo chmod 666 /dev/ttyUSB0
sudo chmod 666 /dev/ttyUSB1
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
