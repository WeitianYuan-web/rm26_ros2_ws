# rm_if_chassis_control

底盘运动控制 ROS2 功能包。订阅图传遥控器通道数据，将摇杆量映射为底盘三轴速度，通过 RS485 串口以 **200Hz** 频率发送控制帧到 STM32 MCU，并实时接收里程计/速度反馈帧。

## 环境要求

| 项目 | 版本 |
|------|------|
| 平台 | NVIDIA Jetson (aarch64) |
| 系统 | Ubuntu 20.04 |
| ROS2 | Foxy Fitzroy |
| C++ | C++17 |

## 编译

```bash
cd ~/RMUL2026/rm26_ros2_ws
source /opt/ros/foxy/setup.bash
colcon build --packages-select rm_if_chassis_control
```

## 运行

```bash
source ~/RMUL2026/rm26_ros2_ws/install/setup.bash

# 默认参数启动
ros2 run rm_if_chassis_control chassis_control_node

# 指定串口和最大速度
ros2 run rm_if_chassis_control chassis_control_node \
  --ros-args -p serial_port:=/dev/ttyUSB1 -p max_vx:=2.0 -p max_vw:=5.0
```

## 通道映射

订阅 `/vt_remote/channels` 话题（由 `vt_remote_control` 发布），数据格式为 `[ch0, ch1, ch2, ch3, wheel]`。

| 通道 | 索引 | 控制轴 | 说明 |
|------|------|--------|------|
| ch2 | 2 | vx | x 方向线速度（前后） |
| ch3 | 3 | vy | y 方向线速度（左右） |
| wheel | 4 | vw | 角速度（逆时针为正） |

**通道值域：** 364 ~ 1684，中值 1024

**映射公式：**

```
offset = raw - 1024
if |offset| <= deadzone:
    velocity = 0
else:
    velocity = (offset ∓ deadzone) / (660 - deadzone) × max_vel
```

## ROS 接口

### 订阅话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/vt_remote/channels` | `std_msgs/Int16MultiArray` | 遥控器通道 [ch0, ch1, ch2, ch3, wheel] |

### 发布话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/chassis/feedback` | `std_msgs/Float32MultiArray` | MCU 反馈 [x, y, θ, vx, vy, vw, feed_rpm] |

### 参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `serial_port` | string | `/dev/ttyUSB0` | 串口设备路径 |
| `baud_rate` | int | `921600` | 波特率 |
| `max_vx` | double | `1.0` | 最大 x 速度 (m/s) |
| `max_vy` | double | `1.0` | 最大 y 速度 (m/s) |
| `max_vw` | double | `3.0` | 最大角速度 (rad/s) |
| `deadzone` | int | `20` | 摇杆死区（原始值单位） |

## 串口通信协议

### 通信参数

| 参数 | 值 |
|------|-----|
| 接口 | RS485（通过 USB 转串口） |
| 波特率 | 921600 bps |
| 数据位 | 8 |
| 停止位 | 1 |
| 校验 | 无 |
| 字节序 | 小端序 (Little-Endian) |

### PC → MCU 控制帧 (17 字节, 200Hz)

```
Byte:  [0]    [1  2  3  4]  [5  6  7  8]  [9  10 11 12]  [13 14 15 16]
       0xAA   vx (float)    vy (float)     vw (float)     feed_rpm (float)
       ────   ───────────   ───────────    ────────────   ───────────────
       帧头   x方向速度      y方向速度      角速度          供弹转速
```

### MCU → PC 反馈帧 (30 字节)

```
Byte:  [0]    [1-4]  [5-8]  [9-12]  [13-16] [17-20] [21-24] [25-28] [29]
       0x55   x      y      theta   vx      vy      vw      feed    0xAA
       ────   ─────  ─────  ──────  ──────  ──────  ──────  ──────  ────
       帧头   位置x  位置y  航向角  速度x   速度y   角速度  供弹    帧尾
```

## 安全机制

| 机制 | 说明 |
|------|------|
| 遥控超时保护 | 500ms 未收到通道数据自动发送零速 |
| 反馈超时检测 | 3 秒无反馈帧打印告警日志 |
| 串口自动重连 | 串口断开后约每秒自动尝试重连 |
| 安全退出 | 节点销毁时连续发送 3 帧零速停止指令 |

## 性能优化

| 优化项 | 说明 |
|--------|------|
| 轻量订阅回调 | 回调仅加锁更新缓存速度值，不做串口 I/O |
| 非阻塞串口 | `O_NONBLOCK` + `VMIN=0, VTIME=0`，读写永不阻塞主线程 |
| 预分配缓冲区 | 控制帧、反馈消息、接收 buffer 在构造时一次性分配 |
| 收发合并 | 200Hz 定时器中先发送后读取，避免额外定时器开销 |
| 帧头帧尾联合校验 | 同时验证 `0x55` 和 `0xAA` 位置关系，减少误帧头 |
| 缓冲区溢出保护 | 超过 512 字节自动截断，防止内存增长 |

## 架构

```
/vt_remote/channels (Int16MultiArray)
        │
        ▼
┌─────────────────────────────────┐
│     ChassisControlNode          │
│                                 │
│  channelsCallback()             │ ← 订阅回调（仅更新速度缓存）
│    ↓ mutex                      │
│  controlTimerCallback() @200Hz  │ ← 定时器回调
│    ├─ sendControlFrame()        │    发送 17B 控制帧 → MCU
│    ├─ readFeedback()            │    读取 30B 反馈帧 ← MCU
│    └─ checkFeedbackTimeout()    │    超时检测
│                                 │
└──────────┬──────────────────────┘
           │
           ▼
  /chassis/feedback (Float32MultiArray)
```
