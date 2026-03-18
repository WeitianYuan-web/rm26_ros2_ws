# rm26_auto_aim

## 简介

自动瞄准功能包，包含海康威视相机采集、TensorRT 装甲板检测与自瞄控制。

## 主要节点

- `hk_camera_node`：海康威视相机图像采集，发布 `/image_raw` 与 `/camera_info`
- `armor_detector_node`：基于 TensorRT 推理识别红/蓝装甲板，发布 `/detector/armors`
- `auto_aim_node`：订阅 `/detector/armors`，执行自瞄控制并发布 `/gimbal/cmd`

## 主要话题

- 输入
  - `/image_raw` (`sensor_msgs/msg/Image`)
  - `/camera_info` (`sensor_msgs/msg/CameraInfo`)
- 输出
  - `/detector/armors` (`auto_aim_interfaces/msg/Armors`)
  - `/detector/result_img` (`sensor_msgs/msg/Image`，可视化调试)
  - `/gimbal/cmd` (`std_msgs/msg/Float64MultiArray`)

## 依赖

- `rclcpp`
- `std_msgs`
- `sensor_msgs`
- `cv_bridge`
- `OpenCV`
- `auto_aim_interfaces`
- TensorRT (`nvinfer`)
- CUDA Runtime (`cudart`)

## 模型文件

默认模型路径：

- `src/rm26_auto_aim/model/yolo26n_rm_500.engine`

如果需要更换模型，可在 launch 或命令行中覆盖 `engine_path` 参数。

## 构建

```bash
cd ~/RMUL2026/rm26_ros2_ws
colcon build --packages-select rm26_auto_aim
source install/setup.bash
```

## 运行方式

### 方式一：一键启动（推荐）

```bash
cd ~/RMUL2026/rm26_ros2_ws
source install/setup.bash
ros2 launch rm26_auto_aim auto_aim.launch.py
```

可通过 launch 参数直接切换控制模式与模型路径：

```bash
ros2 launch rm26_auto_aim auto_aim.launch.py \
  gimbal_control_source:=mouse \
  engine_path:=/home/linkerhand/RMUL2026/rm26_ros2_ws/src/rm26_auto_aim/model/yolo26n_rm_500.engine
```

`auto_aim.launch.py` 默认 `gimbal_control_source=remote`。

### 方式二：分节点启动

```bash
# 终端1：相机
ros2 run rm26_auto_aim hk_camera_node

# 终端2：检测
ros2 run rm26_auto_aim armor_detector_node   --ros-args -p engine_path:=/home/linkerhand/RMUL2026/rm26_ros2_ws/src/rm26_auto_aim/model/yolo26n_rm_500.engine

# 终端3：自瞄控制
ros2 run rm26_auto_aim auto_aim_node
```

## 常用调试命令

```bash
# 检测结果频率
ros2 topic hz /detector/armors

# 查看检测输出
ros2 topic echo /detector/armors

# 图像可视化
rqt_image_view
```

## 追踪参数调参指南（按现象排查）

当前 `auto_aim_node` 的追踪链路为：  
`目标锁定 -> KF 预测/更新 -> 弹道补偿 -> 增量式 PI`。

日志中可重点看两类信息：

- `[追踪] ... | TRACK`：有匹配目标，使用测量更新
- `[追踪] ... | PREDICT`：目标暂时丢失，仅靠预测外推

如果 `PREDICT` 占比很高，优先优化检测与匹配参数；如果 `TRACK` 正常但云台抖动，优先调 PI/KF。

### 1) 核心参数分组

- **PI 控制**
  - `track_yaw_kp`, `track_yaw_ki`
  - `track_pitch_kp`, `track_pitch_ki`
- **KF 滤波**
  - `kf_q_angle`, `kf_q_velocity`
  - `kf_r_yaw`, `kf_r_pitch`
- **目标锁定**
  - `target_match_threshold`
  - `target_lost_timeout`
- **补偿**
  - `track_yaw_offset`, `track_pitch_offset`
  - `bullet_velocity`, `gravity`

### 2) 典型现象 -> 调参方向

- **现象A：进入追踪后“会跟上但发抖”**
  - 先降 `track_yaw_kp`（每次降 15%~25%）
  - 再降 `track_yaw_ki`（每次降 20%~40%）
  - 仍抖动则增大 `kf_r_yaw`（每次 +20%），或减小 `kf_q_velocity`（每次 -20%）

- **现象B：跟踪很稳但明显“跟不上”**
  - 先增 `track_yaw_kp`（每次 +10%~20%）
  - 再小幅增 `track_yaw_ki`（每次 +10%~20%）
  - 若响应仍慢，可增 `kf_q_velocity`（允许速度估计更快变化）

- **现象C：偶发“跑飞”或突然大幅甩头**
  - 降 `track_yaw_kp` / `track_yaw_ki`
  - 减小 `target_match_threshold`（例如 `0.30 -> 0.22`），防止跳到错误目标
  - 缩短 `target_lost_timeout`（例如 `0.30 -> 0.18`），减少纯预测时间

- **现象D：目标短暂遮挡就立刻丢失，频繁重锁**
  - 适当增大 `target_lost_timeout`（例如 `0.30 -> 0.40`）
  - 适当增大 `target_match_threshold`（例如 `0.30 -> 0.35`），但不要过大

- **现象E：Pitch 有系统性偏高/偏低**
  - 优先调 `track_pitch_offset`
  - 距离越远越偏：优先检查 `bullet_velocity`
  - 全距离都偏：优先检查安装角和 `track_pitch_offset`

### 3) 推荐调参顺序（实战）

1. **先关掉追踪激进性**：把 `track_yaw_kp/ki` 调到保守值，确保不跑飞  
2. **调匹配稳定性**：看 `TRACK/PREDICT` 比例，调 `target_match_threshold` + `target_lost_timeout`  
3. **调 KF 平滑度**：抖动多就增 `kf_r_yaw`、降 `kf_q_velocity`；响应慢则反向微调  
4. **最后提性能**：在不抖的前提下逐步加 `track_yaw_kp`，再微调 `ki`

### 4) 建议起步参数（当前版本）

以下是当前 launch 默认值，可作为“起步点”：

- `track_yaw_kp=0.3`, `track_yaw_ki=0.05`
- `track_pitch_kp=-0.3`, `track_pitch_ki=-0.05`
- `kf_q_angle=0.01`, `kf_q_velocity=0.1`
- `kf_r_yaw=0.005`, `kf_r_pitch=0.005`
- `target_match_threshold=0.3`, `target_lost_timeout=0.3`

### 5) 命令行快速覆写示例

```bash
ros2 launch rm26_auto_aim auto_aim.launch.py \
  gimbal_control_source:=remote \
  track_yaw_kp:=0.25 \
  track_yaw_ki:=0.03 \
  kf_q_velocity:=0.06 \
  kf_r_yaw:=0.008 \
  target_match_threshold:=0.24 \
  target_lost_timeout:=0.20
```

建议一次只改 1~2 个参数，每次记录：

- 是否跑飞
- 是否抖动
- `TRACK/PREDICT` 比例
- 命中体感（近/中/远）
