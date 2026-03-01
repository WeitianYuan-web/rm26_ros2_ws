# rm26_auto_aim

## 简介

自动瞄准功能包，包含海康威视相机图像采集与自动瞄准相关节点。

## 主要节点

- `auto_aim_node`
- `hk_camera_node`

## 依赖

- `rclcpp`
- `std_msgs`
- `sensor_msgs`
- `cv_bridge`
- `OpenCV`

## 构建

```bash
cd ~/RMUL2026/rm26_ros2_ws
colcon build --packages-select rm26_auto_aim
source install/setup.bash
```

## 运行示例

```bash
ros2 run rm26_auto_aim auto_aim_node
ros2 run rm26_auto_aim hk_camera_node
```
