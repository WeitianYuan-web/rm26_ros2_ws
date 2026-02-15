# ROS package for RobStride motor control
This routine was reposted by RobStride Dynamics from DR.MuShibo. Sincere gratitude goes to DR.MuShibo for their development and sharing.

### USB2CAN Hardware:Canable
- canable (cantact clone): http://canable.io/ (STM32F042C6)
- 灵足的串口转CAN模块只适用于灵足的上位机，Ubuntu上使用需要额外的canable模块。

## Dependency:
- 注意自己的ros2版本号，自行修改
```shell
sudo apt-get install can-utils
sudo apt-get install ros-humble-can-msgs
sudo apt-get install ros-humble-socketcan-bridge
```

## CAN 接口配置

### 加载 CAN 内核模块
```shell
sudo modprobe can
sudo modprobe can_raw
sudo modprobe can_dev
```

### 配置 CAN0
```shell
sudo ip link set can0 type can bitrate 1000000 
sudo ip link set can0 up
sudo ip link set can0 txqueuelen 100
```

### 配置 CAN1
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
# 监控 CAN0
candump can0

# 监控 CAN1
candump can1
```

## 编译和运行

### 编译
在工作空间中运行如下命令: 
```shell
cd ~/RM_
colcon build --packages-select rs_motor_ros2
source install/setup.bash  # 或 setup.zsh
```

### 运行示例程序
```shell
ros2 run rs_motor_ros2 rs_motor_ros2
```

### 运行电机位置读取节点

**用于测试电机最大最小位置范围**

```shell
# 使用默认参数（CAN1, 电机ID=0x01, 主机ID=0xFF, RS05）
ros2 run rs_motor_ros2 motor_position_reader

# 使用自定义参数
ros2 run rs_motor_ros2 motor_position_reader --ros-args \
  -p can_interface:=can0 \
  -p motor_id:=0x01 \
  -p master_id:=0xFF \
  -p actuator_type:=5 \
  -p publish_rate:=50.0
```

#### 参数说明
- `can_interface`: CAN 接口名称（默认：can1）
- `motor_id`: 电机 ID（默认：0x01）
- `master_id`: 主机 ID（默认：0xFF）
- `actuator_type`: 电机类型（0-6，对应 RS00-RS06，默认：5 即 RS05）
- `publish_rate`: 发布频率 Hz（默认：50.0）

#### 查看电机位置话题
```shell
# 查看位置数据（弧度）
ros2 topic echo /motor/position

# 查看关节状态（包含位置、速度、力矩）
ros2 topic echo /motor/joint_state
```

### 运行遥控器电机控制节点

**使用遥控器 ch3 控制电机位置**

```shell
# 使用默认参数（CAN1, 电机ID=0x01, 位置范围 1.18~2.0 rad）
ros2 run rs_motor_ros2 motor_rc_control

# 使用自定义参数
ros2 run rs_motor_ros2 motor_rc_control --ros-args \
  -p can_interface:=can1 \
  -p motor_id:=0x01 \
  -p master_id:=0xFF \
  -p actuator_type:=5 \
  -p min_position:=1.18 \
  -p max_position:=2.0 \
  -p control_speed:=1.0 \
  -p control_acceleration:=0.5 \
  -p channel_min:=172 \
  -p channel_max:=1811
```

#### 参数说明
- `can_interface`: CAN 接口名称（默认：can1）
- `motor_id`: 电机 ID（默认：0x01）
- `master_id`: 主机 ID（默认：0xFF）
- `actuator_type`: 电机类型（0-6，对应 RS00-RS06，默认：5 即 RS05）
- `min_position`: 最小位置 rad（默认：1.18）
- `max_position`: 最大位置 rad（默认：2.0）
- `control_speed`: 控制速度 rad/s（默认：1.0）
- `control_acceleration`: 加速度（默认：0.5）
- `channel_min`: 遥控器通道最小值（默认：172）
- `channel_max`: 遥控器通道最大值（默认：1811）

#### 查看目标位置话题
```shell
# 查看遥控器映射的目标位置
ros2 topic echo /motor/target_position
```

#### 使用说明
1. 确保遥控器接收器节点正在运行并发布 `/rc/channels` 话题
2. 启动电机控制节点
3. 移动遥控器 ch3（通常是左摇杆或滑块）
4. 电机会在 1.18 ~ 2.0 rad 范围内跟随遥控器位置
5. 遥控器最小值（172）对应最小位置（1.18 rad）
6. 遥控器最大值（1811）对应最大位置（2.0 rad）

## 故障排查

### CAN 接口错误 "Network is down"
1. 检查 CAN 接口是否存在：`ip link show can0` 或 `ip link show can1`
2. 确认 CAN 接口已启动（状态应为 UP）
3. 按照上述步骤重新配置 CAN 接口

### 电机无响应
1. 确认电机已上电
2. 检查 CAN 线缆连接是否正常
3. 确认电机 ID 是否正确（默认为 0x01）
4. 使用 `candump can1` 监控 CAN 总线，查看是否有数据
5. 检查波特率是否正确（1000000）
