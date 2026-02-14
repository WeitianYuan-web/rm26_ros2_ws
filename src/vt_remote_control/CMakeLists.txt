cmake_minimum_required(VERSION 3.5)
project(vt_remote_control)

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

# 头文件包含路径
include_directories(include)

# 添加可执行文件
add_executable(vt_remote_node src/vt_remote_node.cpp)
ament_target_dependencies(vt_remote_node rclcpp std_msgs)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include
)

# 安装目标
install(TARGETS
  vt_remote_node
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
