# CAN2 弹仓电机控制测试工具 文档

## 1. 概述

`ammo_booster_control_can_motor_test.py` 是一个基于 **python-can** 库的交互式 CAN 电机控制测试脚本，用于通过 CAN 总线与 **CtrBoard-H7** MCU 通信，实现对弹仓拨弹电机的速度控制，并实时接收和显示电机反馈数据。

### 功能特性

- 通过 CAN 总线发送速度指令（RPM）给 MCU
- 实时接收并显示双电机的转速与电流反馈
- 支持交互式键盘控制（数字设速 / 加减速 / 急停 / 退出）
- 支持多种 CAN 适配器（slcan、pcan、socketcan、canable 等）
- 多线程架构：发送、接收、显示互不阻塞

---

## 2. 依赖

| 依赖项 | 版本要求 | 说明 |
|--------|---------|------|
| Python | >= 3.6 | 标准库 `argparse`, `struct`, `threading`, `time`, `sys` |
| python-can | >= 4.0 | CAN 总线通信库 |

### 安装依赖

```bash
pip install python-can
```

如果使用 **SocketCAN**（Linux），还需要确保系统已正确配置 CAN 接口：

```bash
# 启用 CAN 接口（以 can2 为例，波特率 1Mbps）
sudo ip link set can2 type can bitrate 1000000
sudo ip link set can2 up

# 查看 CAN 接口状态
ip link show can2
```

---

## 3. 使用方法

### 命令行参数

| 参数 | 缩写 | 默认值 | 说明 |
|------|------|--------|------|
| `--interface` | `-i` | `slcan` | CAN 接口类型 |
| `--channel` | `-c` | `COM3` | CAN 通道（串口号或设备名） |
| `--bitrate` | `-b` | `1000000` | CAN 波特率 (bps) |

### 启动示例

```bash
# 默认: slcan 适配器, COM3, 1Mbps
python ammo_booster_control_can_motor_test.py

# Linux SocketCAN (Jetson 等嵌入式平台)
python ammo_booster_control_can_motor_test.py -i socketcan -c can2

# 指定串口 (Windows/Linux)
python ammo_booster_control_can_motor_test.py -i slcan -c COM5

# PCAN 适配器
python ammo_booster_control_can_motor_test.py -i pcan -c PCAN_USBBUS1
```

### 交互命令

启动后进入交互模式，支持以下命令：

| 命令 | 说明 | 示例 |
|------|------|------|
| `<数字>` | 设置目标速度 (RPM) | `1000`, `-500`, `0` |
| `s` | 急停电机（速度设为 0） | `s` |
| `+` | 速度 +100 RPM（上限 9000） | `+` |
| `-` | 速度 -100 RPM（下限 -9000） | `-` |
| `q` | 发送停止指令并退出程序 | `q` |
| `Ctrl+C` | 强制退出（同样会发送停止指令） | — |

---

## 4. CAN 通信协议

### 4.1 总线参数

| 参数 | 值 |
|------|-----|
| 波特率 | 1,000,000 bps (1 Mbps) |
| 帧类型 | 标准帧 (11-bit ID) |
| 字节序 | 大端序 (Big-Endian) |

### 4.2 PC → MCU 速度指令帧

| 字段 | 值 |
|------|-----|
| CAN ID | `0x100` |
| DLC | 2 字节 |

**数据段定义：**

| 字节偏移 | 长度 | 类型 | 说明 |
|----------|------|------|------|
| 0 - 1 | 2 | int16 (大端) | 目标速度 (RPM), 范围: -32768 ~ +32767 |

### 4.3 MCU → PC 电机反馈帧

| 字段 | 值 |
|------|-----|
| CAN ID | `0x101` |
| DLC | 8 字节 |

**数据段定义：**

| 字节偏移 | 长度 | 类型 | 说明 |
|----------|------|------|------|
| 0 - 1 | 2 | int16 (大端) | 电机 0 实际速度 (RPM) |
| 2 - 3 | 2 | int16 (大端) | 电机 0 电流/转矩 |
| 4 - 5 | 2 | int16 (大端) | 电机 1 实际速度 (RPM) |
| 6 - 7 | 2 | int16 (大端) | 电机 1 电流/转矩 |

### 4.4 通信时序

```
PC (本脚本)                         MCU (CtrBoard-H7)
    |                                       |
    |  ---- [0x100] 速度指令 (2B) ---->     |
    |                                       |
    |  <--- [0x101] 反馈数据 (8B) -----     |  (MCU 周期性回传)
    |                                       |
```

---

## 5. 软件架构

### 5.1 线程模型

脚本启动后创建 **3 个线程**，各司其职：

```
┌──────────────────────────────────────────┐
│              主线程 (Main)                │
│  interactive_mode()                      │
│  - 读取键盘输入                           │
│  - 调用 send_speed_command() 发送指令     │
└──────────────┬───────────────────────────┘
               │ stop_event
    ┌──────────┴──────────┐
    ▼                     ▼
┌────────────────┐  ┌─────────────────┐
│  接收线程       │  │  显示线程        │
│  receive_thread │  │  display_thread  │
│  (daemon)       │  │  (daemon)        │
│                 │  │                  │
│  bus.recv()     │  │  每200ms刷新     │
│  → feedback     │  │  读取 feedback   │
│    .update()    │  │  → 控制台输出    │
└────────────────┘  └─────────────────┘
```

| 线程 | 函数 | 类型 | 职责 |
|------|------|------|------|
| 主线程 | `interactive_mode()` | 前台 | 读取用户输入，发送速度指令 |
| 接收线程 | `receive_thread()` | 守护 (daemon) | 持续接收 CAN 反馈帧，更新 `MotorFeedback` |
| 显示线程 | `display_thread()` | 守护 (daemon) | 每 200ms 刷新终端状态行 |

### 5.2 核心类与函数

#### `MotorFeedback` 类

线程安全的电机反馈数据容器。

| 方法 | 说明 |
|------|------|
| `update(data: bytes)` | 解析 8 字节 CAN 数据，更新内部状态（加锁） |
| `get() -> tuple` | 返回当前所有反馈数据的快照（加锁） |

**内部字段：**

| 字段 | 类型 | 说明 |
|------|------|------|
| `motor0_speed` | int | 电机 0 转速 (RPM) |
| `motor0_torque` | int | 电机 0 电流/转矩 |
| `motor1_speed` | int | 电机 1 转速 (RPM) |
| `motor1_torque` | int | 电机 1 电流/转矩 |
| `update_count` | int | 累计接收帧数 |
| `last_update` | float | 最后一次更新的时间戳 |

#### `send_speed_command(bus, speed_rpm)`

将速度指令打包为 CAN 帧并发送。速度值会被裁剪到 int16 范围（-32768 ~ +32767）。

#### `receive_thread(bus, feedback, stop_event)`

后台循环接收 CAN 帧，过滤 ID 为 `0x101` 的反馈帧，调用 `feedback.update()` 解析。

#### `display_thread(feedback, stop_event)`

每 200ms 读取一次反馈数据，以 `\r` 覆写方式在终端实时刷新电机状态信息（转速、电流、帧数、接收频率）。

---

## 6. 安全机制

| 场景 | 处理方式 |
|------|---------|
| 用户按 `q` 退出 | 先发送速度 0 指令，再关闭 CAN 接口 |
| 用户按 `Ctrl+C` | 捕获 `KeyboardInterrupt`，发送速度 0 指令后退出 |
| 速度范围保护 | 交互模式下限制 ±9000 RPM，协议层裁剪到 int16 |
| CAN 发送失败 | 捕获异常并打印错误信息，不中断程序 |

---

## 7. 故障排查

### 7.1 `Network is down [Error Code 100]`

**现象：**

```
OSError: [Errno 100] Network is down
can.exceptions.CanOperationError: Error receiving: Network is down [Error Code 100]
```

**原因：** CAN 接口未启用或已掉线（常见于 SocketCAN）。

**解决方法：**

```bash
# 1. 检查 CAN 接口状态
ip link show can2

# 2. 如果状态为 DOWN，重新启用
sudo ip link set can2 down
sudo ip link set can2 type can bitrate 1000000
sudo ip link set can2 up

# 3. 如果频繁掉线，检查 bus-off 恢复
ip -details -statistics link show can2
# 查看 "bus-off" 计数，如果持续增长，检查线缆和终端电阻

# 4. 启用自动恢复 (可选)
sudo ip link set can2 type can restart-ms 100
```

### 7.2 无法打开 CAN 接口

**可能原因及排查步骤：**

1. **CAN 适配器未连接** — 检查 USB/物理连接
2. **通道名错误** — 确认设备名（`can0` / `can2` / `COM3` 等）
3. **驱动未安装** — 确认内核模块已加载（`lsmod | grep can`）
4. **python-can 未安装** — 执行 `pip install python-can`

### 7.3 接收不到反馈数据（帧数始终为 0）

1. 确认 MCU 已上电且固件正常运行
2. 确认 CAN 波特率与 MCU 端一致（默认 1 Mbps）
3. 使用 `candump can2` 检查是否有数据到达
4. 检查 CAN 总线终端电阻（120Ω）是否正确接入
5. 确认 MCU 发送的 CAN ID 为 `0x101`

### 7.4 发送指令无效（电机不转）

1. 确认 MCU 端固件已启用 CAN 接收并处理 `0x100` ID
2. 使用 `cansend can2 100#01F4`（十六进制，对应 500 RPM）手动测试
3. 检查电机电源和驱动板接线

---

## 8. 平台适配

| 平台 | 适配器类型 (`-i`) | 通道 (`-c`) | 备注 |
|------|------------------|-------------|------|
| Jetson (Linux) | `socketcan` | `can0` / `can2` | 需 `ip link set` 配置 |
| PC (Windows) | `slcan` | `COM3` / `COM5` | USB-CAN 适配器 |
| PC (Linux) | `slcan` | `/dev/ttyUSB0` | USB-CAN 适配器 |
| PCAN | `pcan` | `PCAN_USBBUS1` | PEAK PCAN-USB |

---

## 9. 文件信息

| 项目 | 值 |
|------|-----|
| 文件名 | `ammo_booster_control_can_motor_test.py` |
| 路径 | `RMUL2026/rm26_ros2_ws/scrip/` |
| 编码 | UTF-8 |
| 协议版本 | CAN 2.0A (标准帧) |
| 默认上电速度 | 500 RPM |
