# rm_serial_cm

## 简介

串口通信功能包，用于串口数据收发与通信流程管理。

## 主要节点

- `serial_cm_node`

## 依赖

- `rclcpp`
- `std_msgs`

## 构建

```bash
cd ~/RMUL2026/rm26_ros2_ws
colcon build --packages-select rm_serial_cm
source install/setup.bash
```

## 运行示例

```bash
ros2 run rm_serial_cm serial_cm_node
```
