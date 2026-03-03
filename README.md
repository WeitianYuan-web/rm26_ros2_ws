# RM26 ROS2 Workspace

RMUL 2026 赛季 RoboMaster 机器人 ROS2 工作空间（Jetson 端）。

---

## 1. 环境要求

| 项目 | 版本/说明 |
|------|------|
| 平台 | NVIDIA Jetson (aarch64) |
| 系统 | Ubuntu 20.04 |
| ROS2 | Foxy Fitzroy |
| 构建工具 | colcon |
| C++ 标准 | C++17 |

---

## 2. 工作空间功能包

| 功能包 | 可执行节点 | 说明 |
|--------|------|------|
| `rm26_auto_aim` | `auto_aim_node`、`hk_camera_node` | 自动瞄准逻辑与海康相机采集 |
| `rm_if_chassis_control` | `chassis_control_node` | 底盘速度控制与串口反馈 |
| `rm_gimbal_control` | `gimbal_control_node` | 云台 Pitch/Yaw 控制 |
| `rm_serial_cm` | `serial_cm_node` | 串口通信基础节点（占位） |
| `ammo_booster_control` | `ammo_booster_node` | 弹仓/拨弹电机 CAN 控制 |
| `vt_remote_control` | `vt_remote_node` | 图传遥控器串口数据解析 |

---

## 3. 首次配置

### 3.1 加载 ROS2 环境

```bash
source /opt/ros/foxy/setup.bash
```

建议写入 `~/.bashrc`：

```bash
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
```

### 3.2 安装依赖（推荐）

```bash
cd ~/RMUL2026/rm26_ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

> `rm26_auto_aim` 的 `hk_camera_node` 依赖海康 MVS SDK（默认路径 `/opt/MVS`）和 OpenCV；请先确认开发机已安装。

### 3.3 运行硬件初始化脚本

该脚本会完成：
- `can0/can1/can2` 配置（1Mbps）
- `ttyUSB0/ttyUSB1` 权限设置

```bash
cd ~/RMUL2026/rm26_ros2_ws
chmod +x setup.sh
./setup.sh
```

---

## 4. 编译

### 4.1 全量编译

```bash
cd ~/RMUL2026/rm26_ros2_ws
source /opt/ros/foxy/setup.bash
colcon build --symlink-install
```

### 4.2 单包编译

```bash
cd ~/RMUL2026/rm26_ros2_ws
source /opt/ros/foxy/setup.bash
colcon build --packages-select <包名> --symlink-install
```

编译完成后加载工作空间：

```bash
source ~/RMUL2026/rm26_ros2_ws/install/setup.bash
```

---

## 5. 运行示例

### 5.1 图传遥控器节点

```bash
source ~/RMUL2026/rm26_ros2_ws/install/setup.bash
ros2 run vt_remote_control vt_remote_node
```

常用参数示例：

```bash
ros2 run vt_remote_control vt_remote_node --ros-args \
  -p serial_port:=/dev/ttyUSB1 \
  -p baud_rate:=921600
```

### 5.2 底盘控制节点

```bash
source ~/RMUL2026/rm26_ros2_ws/install/setup.bash
ros2 run rm_if_chassis_control chassis_control_node
```

串口参数示例：

```bash
ros2 run rm_if_chassis_control chassis_control_node --ros-args \
  -p serial_port:=/dev/ttyUSB0 \
  -p baud_rate:=921600
```

### 5.3 弹仓拨弹节点

```bash
source ~/RMUL2026/rm26_ros2_ws/install/setup.bash
ros2 run ammo_booster_control ammo_booster_node --ros-args -p can_interface:=can2
```

### 5.4 云台控制节点

```bash
source ~/RMUL2026/rm26_ros2_ws/install/setup.bash
ros2 run rm_gimbal_control gimbal_control_node
```

---

## 6. 推荐启动顺序（实机）

1. `./setup.sh`（CAN 与串口权限）
2. `source install/setup.bash`
3. 启动 `vt_remote_node`（输入源）
4. 启动 `chassis_control_node`、`ammo_booster_node`、`gimbal_control_node`
5. 按需启动 `hk_camera_node` / `auto_aim_node`

---

## 7. 关键话题速查

### 7.1 `vt_remote_control` 发布

| 话题 | 类型 | 说明 |
|------|------|------|
| `/vt_remote/channels` | `std_msgs/Int16MultiArray` | 摇杆通道 `[ch0, ch1, ch2, ch3, wheel]` |
| `/vt_remote/mouse` | `std_msgs/Int16MultiArray` | 鼠标 `[x, y, z, left, right, middle]` |
| `/vt_remote/mouse_toggles` | `std_msgs/Int16MultiArray` | 鼠标三键切换状态 |
| `/vt_remote/keyboard` | `std_msgs/UInt16` | 键盘 16 位位图 |
| `/vt_remote/keyboard_toggles` | `std_msgs/UInt16` | 键盘按键切换位图 |
| `/vt_remote/keyboard_readable` | `std_msgs/String` | 可读键盘状态字符串 |
| `/vt_remote/switches` | `std_msgs/Int16MultiArray` | `[mode, pause, fn_left, fn_right, trigger]` |
| `/vt_remote/key_toggles` | `std_msgs/Int16MultiArray` | `[pause, fn_left, fn_right, trigger]` 切换状态 |

### 7.2 其他核心话题

| 话题 | 来源节点 | 类型 | 说明 |
|------|------|------|------|
| `/chassis/feedback` | `chassis_control_node` | `std_msgs/Float32MultiArray` | 底盘反馈（位姿、速度、轮速、供弹速度等） |
| `/chassis/gyro_z` | `chassis_control_node` | `std_msgs/Float32` | 底盘角速度反馈 |
| `/ammo_booster/feedback` | `ammo_booster_node` | `std_msgs/Int16MultiArray` | 拨弹电机反馈 |
| `/gimbal/gyro_z` | `ammo_booster_node` | `std_msgs/Float64` | 陀螺仪 Z 轴角速度 |
| `/image_raw` | `hk_camera_node` | `sensor_msgs/Image` | 相机图像流 |
| `/camera_info` | `hk_camera_node` | `sensor_msgs/CameraInfo` | 相机内参 |

---

## 8. 常用调试命令

```bash
# 查看当前节点
ros2 node list

# 查看当前话题
ros2 topic list

# 监听遥控器通道
ros2 topic echo /vt_remote/channels

# 查看底盘反馈频率
ros2 topic hz /chassis/feedback
```

---

## 9. 目录结构

```text
rm26_ros2_ws/
├── src/                        # ROS2 功能包源码
│   ├── rm26_auto_aim/
│   ├── rm_if_chassis_control/
│   ├── rm_gimbal_control/
│   ├── rm_serial_cm/
│   ├── ammo_booster_control/
│   └── vt_remote_control/
├── setup.sh                    # 硬件接口初始化脚本（CAN/串口权限）
├── build/                      # 编译产物（建议忽略）
├── install/                    # 安装产物（建议忽略）
└── log/                        # 编译日志（建议忽略）
```

---

## 10. 常见问题

### Q1: 节点启动后提示串口打开失败

- 先确认设备存在：`ls /dev/ttyUSB*`
- 重新执行：`./setup.sh`
- 或手动添加用户到串口组后重登：`sudo usermod -aG dialout $USER`

### Q2: CAN 无法发送/接收

- 确认接口状态：`ip -details link show can0 can1 can2`
- 若接口不存在，先检查驱动/设备树，再执行 `./setup.sh`

### Q3: `hk_camera_node` 无法启动

- 检查 MVS SDK 路径是否为 `/opt/MVS`
- 检查相机连接与网口配置
- 重新编译 `rm26_auto_aim` 并确认链接库可用
