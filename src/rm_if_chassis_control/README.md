# rm_if_chassis_control

底盘控制与串口通讯 ROS2 功能包（双节点架构）：

- `chassis_control_node`：融合图传遥控、键鼠、云台 yaw 信息，生成底盘控制指令
- `chassis_serial_node`：通过 RS485 与 STM32 MCU 通讯，发送控制帧并发布反馈帧

控制与通讯主循环频率均为 **200Hz**。

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

## 启动方式

```bash
source ~/RMUL2026/rm26_ros2_ws/install/setup.bash
```

### 方式 1：使用 launch（推荐）

```bash
ros2 launch rm_if_chassis_control chassis.launch.py
```

`chassis.launch.py` 默认参数：

- `chassis_serial_node`: `serial_port=/dev/ttyUSB1`, `baud_rate=460800`
- `chassis_control_node`: `max_linear_accel=4.0`, `max_angular_accel=8.0`, `max_linear_vel=1.5`, `max_angular_vel=4.0`, `min_linear_vel=1.0`, `fire_control_source=remote`, `input_priority_timeout=0.3`, `deadzone=2`, `gimbal_yaw_zero_offset=0.0`

可通过 launch 参数覆盖关键模式与串口配置：

```bash
ros2 launch rm_if_chassis_control chassis.launch.py \
  control_mode:=mouse \
  serial_port:=/dev/ttyUSB1 \
  baud_rate:=460800
```

### 方式 2：分别启动节点

```bash
# 节点 1：控制层
ros2 run rm_if_chassis_control chassis_control_node --ros-args \
  -p max_linear_vel:=1.8 -p max_angular_vel:=5.0 -p fire_control_source:=hybrid

# 节点 2：串口层
ros2 run rm_if_chassis_control chassis_serial_node --ros-args \
  -p serial_port:=/dev/ttyUSB0 -p baud_rate:=460800
```

## 节点与接口

### 1) `chassis_control_node`

功能：把输入设备数据转换为底盘指令并发布到 `/chassis/command`。

**订阅话题**

| 话题 | 类型 | 说明 |
|------|------|------|
| `/vt_remote/channels` | `std_msgs/Int16MultiArray` | 遥控通道 `[ch0, ch1, ch2, ch3, wheel]` |
| `/vt_remote/keyboard` | `std_msgs/UInt16` | 键盘位图（W/S/A/D/Q/E） |
| `/vt_remote/mouse` | `std_msgs/Int16MultiArray` | 鼠标数据，含左右键状态 |
| `/vt_remote/key_toggles` | `std_msgs/Int16MultiArray` | 切换量 `[pause, fn_left, fn_right, trigger]` |
| `motor1/multi_turn_position` | `std_msgs/Float32` | 云台 yaw 电机多圈角（电机侧） |

**发布话题**

| 话题 | 类型 | 说明 |
|------|------|------|
| `/chassis/command` | `std_msgs/Float32MultiArray` | `[max_linear_accel, max_angular_accel, max_linear_vel, max_angular_vel, vx, vy, vw, feed_rpm]` |

**参数**

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `max_linear_accel` | double | `4.0` | 最大线加速度 (m/s^2) |
| `max_angular_accel` | double | `8.0` | 最大角加速度 (rad/s^2) |
| `max_linear_vel` | double | `1.5` | 最大线速度 (m/s) |
| `max_angular_vel` | double | `4.0` | 最大角速度 (rad/s) |
| `min_linear_vel` | double | `1.0` | 非零线速度的最小幅值 (m/s) |
| `deadzone` | int | `2` | 遥控通道死区（原始值） |
| `gimbal_yaw_zero_offset` | double | `0.0` | yaw 零点偏移 (rad) |
| `fire_control_source` | string | `hybrid` | 发射输入源：`mouse` / `remote` / `hybrid` |
| `input_priority_timeout` | double | `0.3` | hybrid 下鼠标优先窗口 (s) |

### 2) `chassis_serial_node`

功能：把 `/chassis/command` 转成串口控制帧下发 MCU，并解析反馈帧。

**订阅话题**

| 话题 | 类型 | 说明 |
|------|------|------|
| `/chassis/command` | `std_msgs/Float32MultiArray` | 控制层输出命令 |

**发布话题**

| 话题 | 类型 | 说明 |
|------|------|------|
| `/chassis/feedback` | `std_msgs/Float32MultiArray` | 反馈数组（12 个 float） |
| `/chassis/gyro_z` | `std_msgs/Float32` | 陀螺仪 z 轴角速度 |

**参数**

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `serial_port` | string | `/dev/ttyUSB0` | 串口设备路径 |
| `baud_rate` | int | `460800` | 波特率（支持 9600/115200/460800/921600） |

## 输入映射与控制逻辑

### 遥控通道映射

- 通道值域：`364 ~ 1684`，中心值：`1024`
- 映射关系：
  - `ch2(index=2)` -> `vx`
  - `ch3(index=3)` -> `vy`（程序内取负号）
  - `wheel(index=4)` -> `vw`

```text
offset = raw - 1024
if abs(offset) <= deadzone:
    vel = 0
else:
    vel = (offset - sign(offset)*deadzone) / (660 - deadzone) * max_vel
```

### 键盘优先

- 键盘位图中 W/S/A/D/Q/E 对应平移与旋转
- 当 WASD/QE 任意有效时，覆盖遥控通道速度输入
- 斜向运动自动按 `1/sqrt(2)` 归一化，避免对角速度过大

### 供弹速度逻辑（`feed_rpm`）

- `fire_control_source=remote`：仅看 `/vt_remote/key_toggles`
- `fire_control_source=mouse`：仅看鼠标左右键
- `fire_control_source=hybrid`：默认遥控，鼠标最近切换在 `input_priority_timeout` 内可抢占

遥控模式下：

- `fn_right=0` -> 低速模式，`feed_rpm=0`
- `fn_right=1` -> 高速模式，`fn_left` 变化时在 `-3200` 与 `1200` 间切换

鼠标模式下：

- 左键未按 -> `feed_rpm=0`
- 左键按下 + 右键未按 -> `feed_rpm=-3200`
- 左键按下 + 右键持续按住 >= 0.2s -> `feed_rpm=1200`

## 串口通信协议

### 链路参数

| 参数 | 值 |
|------|-----|
| 物理层 | RS485（USB 转串口） |
| 波特率 | 默认 460800 bps（可参数化） |
| 数据位 | 8 |
| 停止位 | 1 |
| 校验位 | 无 |
| 字节序 | Little-Endian |

### PC -> MCU 控制帧（35B, 200Hz）

```text
[0]      : 0xAA
[1..32]  : 8 x float32
           max_linear_accel, max_angular_accel, max_linear_vel, max_angular_vel,
           vx, vy, vw, feed_rpm
[33..34] : CRC16-CCITT (对 [0..32] 计算)
```

### MCU -> PC 反馈帧（52B）

```text
[0]      : 0x55
[1..48]  : 12 x float32
           x, y, theta, vx, vy, vw, wheel1_v, wheel2_v, wheel3_v, wheel4_v, feed_rpm, gyro_z
[49..50] : CRC16-CCITT (对 [0..48] 计算)
[51]     : 0xAA
```

## 安全机制

| 机制 | 说明 |
|------|------|
| 控制输入超时 | 控制层 0.5s 无输入时输出零速 |
| 指令超时保护 | 串口层 0.5s 无新命令时仅保留限幅参数，速度/供弹清零 |
| 反馈超时检测 | 3s 无有效反馈帧打印告警 |
| 串口自动重连 | 串口掉线后定时尝试重连 |
| 安全退出 | 节点析构前连续发送 3 帧停止命令 |

## 快速自检

```bash
# 查看核心话题是否存在
ros2 topic list | grep chassis

# 查看控制层输出（8 floats）
ros2 topic echo /chassis/command --once

# 查看反馈层输出
ros2 topic echo /chassis/feedback --once
ros2 topic echo /chassis/gyro_z --once
```

## 架构示意

```text
/vt_remote/channels + /vt_remote/keyboard + /vt_remote/mouse + /vt_remote/key_toggles + motor1/multi_turn_position
                                           |
                                           v
                              chassis_control_node @200Hz
                                           |
                                           v
                                  /chassis/command
                                           |
                                           v
                               chassis_serial_node @200Hz
                                  |                  |
                                  v                  v
                          /chassis/feedback     /chassis/gyro_z
```
