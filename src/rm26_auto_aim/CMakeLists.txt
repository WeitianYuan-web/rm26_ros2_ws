cmake_minimum_required(VERSION 3.5)
project(rm26_auto_aim)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(auto_aim_interfaces REQUIRED)
message(STATUS "OpenCV version: ${OpenCV_VERSION}")

# ==================== Jetson GPU 加速 (可选) ====================
# 检查 OpenCV 是否包含 CUDA 模块 (cudaimgproc)
set(HK_USE_CUDA OFF)

# 方式1: 检查 find_package 找到的 OpenCV 是否包含 cudaimgproc 库
find_library(OPENCV_CUDA_IMGPROC_LIB opencv_cudaimgproc
  PATHS ${OpenCV_INSTALL_PATH}/lib
        /usr/local/lib
        /usr/local/lib/aarch64-linux-gnu
        /usr/lib/aarch64-linux-gnu)
if(OPENCV_CUDA_IMGPROC_LIB)
  set(HK_USE_CUDA ON)
  message(STATUS "[GPU] 找到 OpenCV CUDA 模块: ${OPENCV_CUDA_IMGPROC_LIB}")
endif()

if(HK_USE_CUDA)
  message(STATUS "[GPU] CUDA GPU 加速: 已启用")
else()
  message(STATUS "[GPU] CUDA GPU 加速: 未启用 (OpenCV 缺少 cudaimgproc 模块)")
  message(STATUS "[GPU] 提示: 安装 cuda-toolkit-11-4 并重编译 OpenCV with CUDA 可启用 GPU 加速")
endif()

# ==================== 海康威视 MVS SDK ====================
set(MVS_SDK_DIR "/opt/MVS")
set(MVS_INCLUDE_DIR "${MVS_SDK_DIR}/include")
set(MVS_LIB_DIR "${MVS_SDK_DIR}/lib/aarch64")

# ==================== auto_aim_node ====================
add_executable(auto_aim_node src/auto_aim_node.cpp)
ament_target_dependencies(auto_aim_node rclcpp std_msgs auto_aim_interfaces)

# ==================== hk_camera_node ====================
add_executable(hk_camera_node src/hk_camera_node.cpp)
target_include_directories(hk_camera_node PRIVATE ${MVS_INCLUDE_DIR})
target_link_directories(hk_camera_node PRIVATE ${MVS_LIB_DIR})
target_link_libraries(hk_camera_node MvCameraControl)
ament_target_dependencies(hk_camera_node
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)

# 如果有 CUDA 支持，添加编译定义和额外链接
if(HK_USE_CUDA)
  target_compile_definitions(hk_camera_node PRIVATE USE_CUDA)
  if(OPENCV_CUDA_IMGPROC_LIB)
    target_link_libraries(hk_camera_node ${OPENCV_CUDA_IMGPROC_LIB})
    # 同时链接 opencv_cudaarithm (cudaimgproc 依赖)
    find_library(OPENCV_CUDA_ARITHM_LIB opencv_cudaarithm
      PATHS /usr/local/lib /usr/local/lib/aarch64-linux-gnu
      NO_DEFAULT_PATH)
    if(OPENCV_CUDA_ARITHM_LIB)
      target_link_libraries(hk_camera_node ${OPENCV_CUDA_ARITHM_LIB})
    endif()
  endif()
endif()

# 安装目标
install(TARGETS
  auto_aim_node
  hk_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 launch 文件（如有）
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
