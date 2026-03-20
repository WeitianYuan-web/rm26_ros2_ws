#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""!
@file stop_all.py
@brief RM26 一键关闭脚本
@details 优先停止 start_all.py 主控进程（触发其优雅清理逻辑），再对残留的 ROS2 节点执行分阶段兜底关闭。
你现在直接用：

cd ~/RMUL2026/rm26_ros2_ws
./stop_all.py
如果你还想“重启后也不再自动启动”，再执行一次：

sudo systemctl disable rm26-start-all.service
后续需要恢复开机自启时再：

sudo systemctl enable --now rm26-start-all.service
"""

import os
import signal
import subprocess
import sys
import time

## systemd 服务名（由 enable_start_all_autostart.sh 安装）
SERVICE_NAME = "rm26-start-all.service"

## 需要关闭的目标进程匹配模式（pgrep -f）
TARGET_PATTERNS = [
    r"vt_remote_control.*vt_remote_node",
    r"rm_gimbal_control.*gimbal_control_node",
    r"rm_if_chassis_control.*chassis\.launch\.py",
    r"ammo_booster_control.*ammo_booster_node",
    r"rm26_auto_aim.*auto_aim\.launch\.py",
    r"hk_camera_node",
    r"armor_detector_node",
    r"auto_aim_node",
]

## start_all 主脚本匹配模式
START_ALL_PATTERNS = [
    r"(^|/)start_all\.py($| )",
    r"python3 .*start_all\.py",
]

"""!
@brief 执行 systemctl 命令
@param args systemctl 参数列表
@param use_sudo 是否使用 sudo 调用
@return tuple[bool, str] (是否成功, 输出文本)
"""
def run_systemctl(args, use_sudo=False):
    cmd = ["systemctl"] + args
    if use_sudo:
        cmd = ["sudo"] + cmd
    result = subprocess.run(cmd, check=False, text=True, capture_output=True)
    output = (result.stdout or "") + (result.stderr or "")
    return (result.returncode == 0, output.strip())


"""!
@brief 关闭自启动服务（避免被后台自动拉起）
@details 优先检测服务状态；若服务存在且正在运行，尝试停止该服务。
"""
def stop_autostart_service():
    ok, _ = run_systemctl(["list-unit-files", SERVICE_NAME], use_sudo=False)
    if not ok:
        print(f"[INFO] 未检测到 systemd 服务 {SERVICE_NAME}，跳过服务停止。")
        return

    active, _ = run_systemctl(["is-active", SERVICE_NAME], use_sudo=False)
    if not active:
        print(f"[INFO] 服务 {SERVICE_NAME} 当前未运行。")
        return

    print(f"[INFO] 检测到服务 {SERVICE_NAME} 正在运行，尝试停止...")
    stopped, out = run_systemctl(["stop", SERVICE_NAME], use_sudo=True)
    if stopped:
        print(f"[INFO] 服务 {SERVICE_NAME} 已停止。")
    else:
        print(f"[WARNING] 停止服务失败，可能需要手动执行: sudo systemctl stop {SERVICE_NAME}")
        if out:
            print(f"[WARNING] 详细信息: {out}")


"""!
@brief 按模式查找进程 PID
@param pattern pgrep -f 使用的正则模式
@return list[int] 进程 PID 列表
@details 自动排除当前 stop_all.py 自身 PID，避免误杀当前脚本进程。
"""
def get_pids_by_pattern(pattern):
    try:
        result = subprocess.run(
            ["pgrep", "-f", pattern],
            check=False,
            text=True,
            capture_output=True,
        )
    except FileNotFoundError:
        print("[ERROR] 系统缺少 pgrep 命令，无法继续。")
        sys.exit(1)

    if result.returncode != 0 or not result.stdout.strip():
        return []

    current_pid = os.getpid()
    pids = []
    for line in result.stdout.strip().splitlines():
        try:
            pid = int(line.strip())
            if pid != current_pid:
                pids.append(pid)
        except ValueError:
            continue
    return sorted(set(pids))


"""!
@brief 尝试向目标 PID 发送信号
@param pid 目标进程 PID
@param sig 要发送的信号
@return bool 发送是否成功
"""
def safe_kill(pid, sig):
    try:
        os.kill(pid, sig)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        print(f"[WARNING] 无权限向 PID={pid} 发送信号 {sig}.")
        return False


"""!
@brief 检查 PID 是否仍存活
@param pid 进程 PID
@return bool 进程是否存在
"""
def is_pid_alive(pid):
    try:
        os.kill(pid, 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return True


"""!
@brief 对一组 PID 执行分阶段关闭
@param pids 目标 PID 列表
@details 依次发送 SIGINT、SIGTERM、SIGKILL，并在每阶段等待短暂时间。
"""
def kill_with_escalation(pids):
    if not pids:
        return

    print(f"[INFO] 发现残留进程 {len(pids)} 个，开始分阶段关闭...")

    for sig, wait_sec, sig_name in [
        (signal.SIGINT, 1.5, "SIGINT"),
        (signal.SIGTERM, 1.5, "SIGTERM"),
        (signal.SIGKILL, 0.0, "SIGKILL"),
    ]:
        alive = [pid for pid in pids if is_pid_alive(pid)]
        if not alive:
            break
        print(f"[INFO] 发送 {sig_name} 到: {alive}")
        for pid in alive:
            safe_kill(pid, sig)
        if wait_sec > 0:
            time.sleep(wait_sec)

    remaining = [pid for pid in pids if is_pid_alive(pid)]
    if remaining:
        print(f"[WARNING] 以下进程仍存活，请手动检查: {remaining}")
    else:
        print("[INFO] 残留进程已全部关闭。")


"""!
@brief 主函数
@details 先关闭 start_all.py，再清理相关节点残留进程。
"""
def main():
    print("=======================================")
    print("      RM26 一键关闭脚本 (Python版)     ")
    print("=======================================")

    # 0) 先停 systemd 服务，避免进程被自动重启
    stop_autostart_service()

    # 1) 优先关闭 start_all.py，触发其优雅清理流程
    start_all_pids = []
    for pattern in START_ALL_PATTERNS:
        start_all_pids.extend(get_pids_by_pattern(pattern))
    start_all_pids = sorted(set(start_all_pids))

    if start_all_pids:
        print(f"[INFO] 检测到 start_all.py 进程: {start_all_pids}")
        for pid in start_all_pids:
            safe_kill(pid, signal.SIGINT)
        time.sleep(2.0)
    else:
        print("[INFO] 未检测到 start_all.py 进程，直接执行兜底关闭。")

    # 2) 兜底关闭可能残留的节点进程
    fallback_pids = []
    for pattern in TARGET_PATTERNS:
        fallback_pids.extend(get_pids_by_pattern(pattern))
    fallback_pids = sorted(set(fallback_pids))

    if fallback_pids:
        kill_with_escalation(fallback_pids)
    else:
        print("[INFO] 未发现需要兜底关闭的节点进程。")

    print("[INFO] 一键关闭流程结束。")


if __name__ == "__main__":
    main()
