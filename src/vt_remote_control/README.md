# vt_remote_control

## 简介

图传遥控功能包。`vt_remote_node` 通过串口接收 VTM Receiver 数据帧并发布标准 ROS2 话题，供底盘、拨弹等节点使用。

## 主要节点

- `vt_remote_node`

## 话题接口

### 发布话题

| 话题 | 类型 | 数据格式 | 说明 |
|------|------|----------|------|
| `/vt_remote/channels` | `std_msgs/Int16MultiArray` | `[ch0, ch1, ch2, ch3, wheel]` | 摇杆通道原始值 |
| `/vt_remote/mouse` | `std_msgs/Int16MultiArray` | `[mouse_x, mouse_y, mouse_z, left, right, middle]` | 鼠标数据 |
| `/vt_remote/keyboard` | `std_msgs/UInt16` | `16bit` 按键位图 | 键盘按键状态 |
| `/vt_remote/switches` | `std_msgs/Int16MultiArray` | `[mode, pause, fn_left, fn_right, trigger]` | 原始开关/按键电平 |
| `/vt_remote/key_toggles` | `std_msgs/Int16MultiArray` | `[pause, fn_left, fn_right, trigger]` | 按键切换状态（按下上升沿翻转） |

## 参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `serial_port` | string | `/dev/ttyUSB1` | 串口设备路径 |
| `baud_rate` | int | `921600` | 串口波特率 |

## 依赖

- `rclcpp`
- `std_msgs`

## 构建

```bash
cd ~/RMUL2026/rm26_ros2_ws
colcon build --packages-select vt_remote_control
source install/setup.bash
```

## 运行示例

```bash
ros2 run vt_remote_control vt_remote_node

# 指定串口
ros2 run vt_remote_control vt_remote_node --ros-args -p serial_port:=/dev/ttyUSB0
```
