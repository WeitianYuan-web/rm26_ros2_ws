# rm_gimbal_control — 云台控制功能包

基于 RobStride 电机的云台 Pitch/Yaw 双轴控制 ROS2 功能包，通过图传遥控器信号驱动两个电机实现云台运动控制。

> 电机驱动部分基于 DR.MuShibo 开发的 RobStride 电机控制代码，在此表示感谢。

## 目录结构

```
rm_gimbal_control/
├── CMakeLists.txt
├── package.xml
├── README.md
├── include/
│   └── rm_gimbal_control/
│       └── motor_cfg.h          # RobStride 电机驱动库（CAN 通信）
└── src/
    ├── gimbal_control_node.cpp  # 云台控制主节点
    ├── motor_cfg.cpp            # 电机驱动实现
    ├── motor_position_reader.cpp # 电机位置读取工具（独立程序，未编入本包）
    └── main.cpp                  # 电机控制示例代码（未编入本包）
```

## 依赖

- ROS2（请根据自己的版本替换 `humble`）

```shell
sudo apt-get install can-utils
sudo apt-get install ros-humble-can-msgs
```

- 系统依赖：Linux SocketCAN 支持

## CAN 接口配置

### 加载 CAN 内核模块

```shell
sudo modprobe can
sudo modprobe can_raw
sudo modprobe can_dev
```

### 配置 CAN0（电机1 默认接口）

```shell
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
sudo ip link set can0 txqueuelen 100
```

### 配置 CAN1（电机2 默认接口）

```shell
sudo ip link set can1 type can bitrate 1000000
sudo ip link set can1 up
sudo ip link set can1 txqueuelen 100
```

### 查看 CAN 接口状态

```shell
ip link show can0
ip link show can1
```

### 监控 CAN 总线（调试用）

```shell
candump can0
candump can1
```

## 编译

在工作空间根目录运行：

```shell
colcon build --packages-select rm_gimbal_control
source install/setup.bash
```

## 节点说明

### gimbal_control_node

云台控制主节点，控制两个 RobStride 电机（Pitch / Yaw），使用 CSP（Cyclic Synchronous Position）位置模式。

- **电机1（速度积分模式）**：遥控器摇杆映射为速度，通过高频定时器（默认 200Hz）对速度积分得到目标位置，电机可连续旋转，无位置限幅。适用于 Yaw 轴。
- **电机2（位置映射模式）**：遥控器通道值直接线性映射到目标位置范围，有最大/最小位置限制。适用于 Pitch 轴。

#### 运行

```shell
# 使用默认参数
ros2 run rm_gimbal_control gimbal_control_node

# 使用自定义参数
ros2 run rm_gimbal_control gimbal_control_node --ros-args \
  -p motor1_can_interface:=can0 \
  -p motor1_id:=0x02 \
  -p motor1_max_velocity:=6.0 \
  -p motor1_channel_index:=0 \
  -p motor1_control_rate:=200 \
  -p motor2_can_interface:=can1 \
  -p motor2_id:=0x01 \
  -p motor2_min_position:=-0.5 \
  -p motor2_max_position:=0.5 \
  -p motor2_channel_index:=1
```

#### 参数列表

**电机1 参数（速度积分模式 / Yaw 轴）**

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `motor1_can_interface` | string | `can0` | 电机1 CAN 接口 |
| `motor1_id` | int | `0x02` | 电机1 ID |
| `motor1_max_velocity` | double | `6.0` | 遥控器映射最大速度 (rad/s) |
| `motor1_channel_index` | int | `0` | 遥控器通道索引 (ch0) |
| `motor1_control_rate` | int | `200` | 控制环频率 (Hz) |
| `motor1_control_speed` | double | `6.0` | CSP 位置跟踪速度 (rad/s) |
| `motor1_control_acceleration` | double | `4.0` | CSP 加速度 |

**电机2 参数（位置映射模式 / Pitch 轴）**

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `motor2_can_interface` | string | `can1` | 电机2 CAN 接口 |
| `motor2_id` | int | `0x01` | 电机2 ID |
| `motor2_min_position` | double | `-0.5` | 最小目标位置 (rad) |
| `motor2_max_position` | double | `0.5` | 最大目标位置 (rad) |
| `motor2_channel_index` | int | `1` | 遥控器通道索引 (ch1) |
| `motor2_control_speed` | double | `1.0` | CSP 位置跟踪速度 (rad/s) |
| `motor2_control_acceleration` | double | `1.0` | CSP 加速度 |

**通用参数**

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `master_id` | int | `0xFF` | CAN 主机 ID |
| `actuator_type` | int | `5` | 电机类型 (0-6，对应 RS00-RS06) |
| `rc_min_value` | int | `364` | 遥控器通道最小值 |
| `rc_max_value` | int | `1684` | 遥控器通道最大值 |
| `rc_mid_value` | int | `1024` | 遥控器通道中值 |
| `deadzone` | int | `5` | 遥控器数值死区 |

#### 话题

**订阅**

| 话题名 | 消息类型 | 说明 |
|--------|----------|------|
| `/vt_remote/channels` | `std_msgs/msg/Int16MultiArray` | 图传遥控器通道数据 |

**发布**

| 话题名 | 消息类型 | 说明 |
|--------|----------|------|
| `motor1/target_position` | `std_msgs/msg/Float32` | 电机1 当前目标位置 (rad) |
| `motor2/target_position` | `std_msgs/msg/Float32` | 电机2 当前目标位置 (rad) |

## 控制逻辑

### 电机1 — 速度积分模式

```
遥控器 ch[0] → 映射为速度 [-max_vel, +max_vel]
                         ↓
              高频定时器 (200Hz) 积分: position += velocity × dt
                         ↓
              CSP 位置指令下发给电机
```

1. 遥控器通道值通过 `map_rc_to_velocity()` 映射为 `[-motor1_max_velocity, +motor1_max_velocity]`
2. 中值（1024）附近存在死区（±5），死区内速度为 0
3. 高频定时器以 200Hz 频率运行，对速度做积分得到目标位置
4. 电机可连续旋转，无位置限幅

### 电机2 — 位置映射模式

```
遥控器 ch[1] → 线性映射到 [min_position, max_position]
                         ↓
              CSP 位置指令下发给电机
```

1. 遥控器通道值通过 `map_rc_to_pos()` 线性映射到 `[motor2_min_position, motor2_max_position]`
2. 中值附近有死区处理，死区内映射到位置范围中点
3. 在 RC 回调中直接下发位置指令

## 使用说明

1. 配置并启动 CAN 接口（can0 和 can1）
2. 确保图传遥控器接收节点已启动并发布 `/vt_remote/channels` 话题
3. 启动云台控制节点：`ros2 run rm_gimbal_control gimbal_control_node`
4. 节点启动后会自动将电机切换到 CSP 位置模式并使能
5. 通过遥控器 ch0 控制电机1（Yaw），ch1 控制电机2（Pitch）
6. 节点退出时会自动停止（失能）电机

### 查看运行状态

```shell
# 查看电机1目标位置
ros2 topic echo motor1/target_position

# 查看电机2目标位置
ros2 topic echo motor2/target_position

# 查看所有话题
ros2 topic list
```

## 支持的电机类型

| actuator_type | 型号 | 最大位置 (rad) | 最大速度 (rad/s) | 最大扭矩 (Nm) |
|:---:|:---:|:---:|:---:|:---:|
| 0 | RS00 | 4π | 50 | 17 |
| 1 | RS01 | 4π | 44 | 17 |
| 2 | RS02 | 4π | 44 | 17 |
| 3 | RS03 | 4π | 50 | 60 |
| 4 | RS04 | 4π | 15 | 120 |
| 5 | RS05 | 4π | 33 | 17 |
| 6 | RS06 | 4π | 20 | 60 |

## 故障排查

### CAN 接口错误 "Network is down"

1. 检查 CAN 接口是否存在：`ip link show can0` / `ip link show can1`
2. 确认 CAN 接口已启动（状态应为 UP）
3. 按照上述步骤重新配置 CAN 接口

### 电机无响应

1. 确认电机已上电
2. 检查 CAN 线缆连接是否正常
3. 确认电机 ID 是否正确（电机1 默认 0x02，电机2 默认 0x01）
4. 使用 `candump can0` / `candump can1` 监控 CAN 总线，查看是否有数据
5. 检查波特率是否正确（1000000）

### 电机初始化失败

- 节点使用异步线程初始化电机，启动后日志中会打印初始化结果
- 如果初始化失败，检查上述 CAN 接口和电机连接
- 节点不会因为电机初始化失败而退出，但对应电机不会被控制

### 遥控器无反应

1. 确认图传遥控器接收节点正在运行
2. 检查 `/vt_remote/channels` 话题是否有数据：`ros2 topic echo /vt_remote/channels`
3. 确认通道索引配置正确（默认 ch0 = 电机1，ch1 = 电机2）
4. 检查 `rc_min_value` / `rc_max_value` / `rc_mid_value` 是否与实际遥控器范围匹配
