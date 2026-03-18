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
