#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""!
@file start_all.py
@brief RM26 一键启动脚本
@details 该脚本用于一键启动所有相关节点，并在启动前执行 setup.sh。根据 `启动说明.md` 编写，将各个节点放在独立的进程中运行。
"""

import subprocess
import time
import signal
import sys
import os
import threading

## 预先 source 环境的命令
SOURCE_CMD = (
    "source /opt/ros/foxy/setup.bash"
    " && source install/setup.bash"
    " && export LD_LIBRARY_PATH=/opt/MVS/lib/aarch64:${LD_LIBRARY_PATH:-}"
)

## 要执行的节点及其启动命令列表
# 按照建议的启动顺序排列
NODE_COMMANDS = [
    (
        "vt_remote_node",
        f"{SOURCE_CMD} && ros2 run vt_remote_control vt_remote_node --ros-args -p serial_port:=/dev/ttyUSB0 -p baud_rate:=921600"
    ),
    (
        "gimbal_control_node",
        f"{SOURCE_CMD} && ros2 run rm_gimbal_control gimbal_control_node"
    ),
    (
        "chassis_launch",
        f"{SOURCE_CMD} && ros2 launch rm_if_chassis_control chassis.launch.py serial_port:=/dev/ttyUSB1 baud_rate:=460800"
    ),
    (
        "ammo_booster_node",
        f"{SOURCE_CMD} && ros2 run ammo_booster_control ammo_booster_node --ros-args -p can_interface:=can2"
    ),
    (
        "auto_aim_launch",
        f"{SOURCE_CMD} && ros2 launch rm26_auto_aim auto_aim.launch.py"
    )
]

## 保存所有启动子进程的列表
processes = []
## 保存日志转发线程，便于结束时回收
log_threads = []

## ANSI 颜色常量
ANSI_RESET = "\033[0m"
ANSI_COLORS = [
    "\033[38;5;39m",   # 蓝
    "\033[38;5;208m",  # 橙
    "\033[38;5;46m",   # 绿
    "\033[38;5;201m",  # 洋红
    "\033[38;5;51m",   # 青
    "\033[38;5;226m",  # 黄
]
ANSI_ERR = "\033[38;5;196m"
ANSI_WARN = "\033[38;5;226m"

## 多线程打印锁，避免不同节点日志交叉导致难以阅读
print_lock = threading.Lock()

"""!
@brief 带颜色转发子进程日志
@param name 节点名称
@param stream 子进程输出流（stdout/stderr）
@param color 该节点对应颜色
@param is_stderr 是否为错误输出流
@details 使用单独线程逐行读取并打印，保证每个节点在同一终端中拥有稳定颜色和标签前缀
"""
def forward_stream(name, stream, color, is_stderr=False):
    tag = "ERR" if is_stderr else "OUT"
    prefix = f"[{name}/{tag}]"
    try:
        for line in iter(stream.readline, ""):
            log_color = get_log_color(line, color)
            with print_lock:
                print(f"{log_color}{prefix} {line.rstrip()}{ANSI_RESET}", flush=True)
    finally:
        stream.close()

"""!
@brief 根据日志级别选择颜色
@param line 单行日志文本
@param default_color 节点默认颜色
@return str ANSI 颜色码
@details 按日志内容关键字匹配：ERROR/FATAL 为红色，WARN 为黄色，其余沿用节点默认颜色
"""
def get_log_color(line, default_color):
    if ("[ERROR]" in line) or ("[FATAL]" in line):
        return ANSI_ERR
    if "[WARN]" in line:
        return ANSI_WARN
    return default_color

"""!
@brief 信号清理函数
@param args 接收的信号参数
@details 捕捉 Ctrl+C (SIGINT) 或 SIGTERM 等信号，通过进程组安全地关闭所有关联的 ROS 2 进程
"""
def cleanup(*args):
    print("\n[INFO] 接收到终止信号，正在关闭所有节点...")
    for name, p in processes:
        print(f"[INFO] 正在终止 {name} 进程组 (PID: {p.pid})...")
        try:
            # 向整个进程组发送 SIGINT，允许节点执行析构或清理工作
            os.killpg(os.getpgid(p.pid), signal.SIGINT)
        except Exception as e:
            print(f"[WARNING] 终止 {name} 时出错: {e}")
            
    # 等待进程完全退出
    for name, p in processes:
        try:
            p.wait(timeout=5)
        except subprocess.TimeoutExpired:
            print(f"[WARNING] {name} 无法正常退出，尝试强制杀死 (SIGKILL)...")
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGKILL)
            except Exception:
                pass

    # 等待日志线程退出，避免残留输出
    for t in log_threads:
        t.join(timeout=1)

    print("[INFO] 所有节点已关闭。")
    sys.exit(0)

"""!
@brief 主函数
@details 依次执行 setup.sh 和所有预设的 ROS 2 启动命令，最后挂起等待退出信号
"""
def main():
    # 注册信号处理器（支持 Ctrl+C 和 kill 命令）
    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)

    print("=======================================")
    print("      RM26 一键启动脚本 (Python版)     ")
    print("=======================================")

    # 1. 执行 ./setup.sh
    setup_script = "./setup.sh"
    if os.path.exists(setup_script):
        print(f"\n[INFO] 正在执行: {setup_script}")
        try:
            subprocess.run(["/bin/bash", setup_script], check=True)
            print("[INFO] setup.sh 执行完成。")
        except subprocess.CalledProcessError as e:
            print(f"[ERROR] setup.sh 执行失败，退出。错误信息: {e}")
            sys.exit(1)
    else:
        print(f"[WARNING] 未找到 {setup_script} 文件，跳过执行。请确保您在 rm26_ros2_ws 目录下运行此脚本。")

    time.sleep(1)

    # 2. 依次启动各个节点进程
    print("\n[INFO] 开始启动 ROS 2 节点...")
    for idx, (name, cmd) in enumerate(NODE_COMMANDS):
        color = ANSI_COLORS[idx % len(ANSI_COLORS)]
        print(f"{color} -> 正在启动: {name}{ANSI_RESET}")
        # preexec_fn=os.setsid 创建新的进程组，以便干净地杀掉该命令衍生的所有子进程
        p = subprocess.Popen(
            cmd,
            shell=True,
            executable='/bin/bash',
            preexec_fn=os.setsid,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1
        )
        processes.append((name, p))

        # 每个节点单独线程转发 stdout/stderr，实现彩色区分
        t_out = threading.Thread(target=forward_stream, args=(name, p.stdout, color, False), daemon=True)
        t_err = threading.Thread(target=forward_stream, args=(name, p.stderr, color, True), daemon=True)
        t_out.start()
        t_err.start()
        log_threads.extend([t_out, t_err])

        time.sleep(2)  # 等待前一个节点初始化完成再启动下一个，确保启动顺序和系统稳定性

    print("\n=======================================")
    print("  所有节点已启动！按 Ctrl+C 可一键安全退出 ")
    print("=======================================\n")

    # 保持主进程运行并等待信号
    while True:
        try:
            time.sleep(1)
        except KeyboardInterrupt:
            # 这里的捕捉为了兼容某些环境，实际清理工作由信号处理器 cleanup 完成
            pass

if __name__ == "__main__":
    main()
