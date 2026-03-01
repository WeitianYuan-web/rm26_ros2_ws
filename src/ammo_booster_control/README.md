# ammo_booster_control

## 简介

弹仓拨弹控制功能包。`ammo_booster_node` 订阅图传按键切换状态，根据 `fn_right` 切换状态控制拨弹电机低速/高速模式，并通过 SocketCAN 周期发送速度指令。

## 主要节点

- `ammo_booster_node`

## 话题接口

### 订阅话题

| 话题 | 类型 | 数据格式 | 说明 |
|------|------|----------|------|
| `/vt_remote/key_toggles` | `std_msgs/Int16MultiArray` | `[pause, fn_left, fn_right, trigger]` | 使用 `fn_right` 切换状态决定高低速模式 |

## 参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `can_interface` | string | `can2` | CAN 设备名 |
| `low_speed_rpm` | int | `500` | 低速模式目标转速 |
| `high_speed_rpm` | int | `4500` | 高速模式目标转速 |
| `send_interval_ms` | int | `50` | 速度指令发送周期 |

## 依赖

- `rclcpp`
- `std_msgs`

## 构建

```bash
cd ~/RMUL2026/rm26_ros2_ws
colcon build --packages-select ammo_booster_control
source install/setup.bash
```

## 运行示例

```bash
ros2 run ammo_booster_control ammo_booster_node

# 指定 CAN 口与转速参数
ros2 run ammo_booster_control ammo_booster_node \
  --ros-args -p can_interface:=can2 -p low_speed_rpm:=500 -p high_speed_rpm:=4500
```
