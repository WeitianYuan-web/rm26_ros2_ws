# RM26 ROS2 Workspace

RMUL 2026 赛季 RoboMaster 机器人 ROS2 工作空间。

## 环境要求

| 项目 | 版本 |
|------|------|
| 平台 | NVIDIA Jetson (aarch64) |
| 系统 | Ubuntu 20.04 |
| ROS2 | Foxy Fitzroy |
| C++ | C++17 |

## 功能包一览

| 功能包 | 节点 | 说明 |
|--------|------|------|
| `rm_auto_aim` | `auto_aim_node` | 自动瞄准（目标检测与跟踪） |
| `rm_if_chassis_control` | `chassis_control_node` | 底盘运动控制 |
| `rm_gimbal_control` | `gimbal_control_node` | 云台 Pitch/Yaw 控制 |
| `rm_serial_cm` | `serial_cm_node` | 与下位机串口通信 |
| `ammo_booster_control` | `ammo_booster_node` | 弹仓与拨弹机构控制 |
| `vt_remote_control` | `vt_remote_node` | 图传遥控器数据解析 |

## 编译

```bash
cd ~/RMUL2026/rm26_ros2_ws
source /opt/ros/foxy/setup.bash
colcon build
```

编译单个功能包：

```bash
colcon build --packages-select <包名>
```

## 运行

```bash
source ~/RMUL2026/rm26_ros2_ws/install/setup.bash

# 示例：启动图传遥控器节点
ros2 run vt_remote_control vt_remote_node

# 指定串口参数
ros2 run vt_remote_control vt_remote_node --ros-args -p serial_port:=/dev/ttyUSB0
```

## 目录结构

```
rm26_ros2_ws/
├── src/                        # ROS2 功能包源码
│   ├── rm_auto_aim/            # 自动瞄准
│   ├── rm_if_chassis_control/  # 底盘控制
│   ├── rm_gimbal_control/      # 云台控制
│   ├── rm_serial_cm/           # 串口通信
│   ├── ammo_booster_control/   # 弹仓拨弹控制
│   └── vt_remote_control/      # 图传遥控器
├── scrip/                      # 测试脚本
├── build/                      # 编译产物（gitignore）
├── install/                    # 安装产物（gitignore）
└── log/                        # 编译日志（gitignore）
```

## 话题列表（已实现）

### vt_remote_control

| 话题 | 类型 | 说明 |
|------|------|------|
| `/vt_remote/channels` | `std_msgs/Int16MultiArray` | 摇杆通道 [ch0, ch1, ch2, ch3, wheel] |
| `/vt_remote/mouse` | `std_msgs/Int16MultiArray` | 鼠标 [x, y, z, left, right, middle] |
| `/vt_remote/keyboard` | `std_msgs/UInt16` | 键盘 16 位按键状态 |
| `/vt_remote/switches` | `std_msgs/Int16MultiArray` | 开关 [mode, pause, fn_left, fn_right, trigger] |
