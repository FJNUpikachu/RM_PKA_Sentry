cmake_minimum_required(VERSION 3.8)
project(rm_serial_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# ── ROS 2 依赖 ────────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rm_interfaces REQUIRED)
find_package(rm_utils REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ── 内嵌 wjwwood/serial 源码（无需系统安装，无需 find_package(serial)） ────────
# 头文件位于 include/serial/
# 源文件位于 src/serial/
set(SERIAL_SRCS
  src/serial/serial.cc
  src/serial/impl/unix.cc
  src/serial/impl/list_ports/list_ports_linux.cc
)

# ── 共享库 ────────────────────────────────────────────────────────────────────
add_library(${PROJECT_NAME}_lib SHARED
  src/uart_node.cpp
  src/uart_node_tool.cpp
  src/uart_protocol.cpp
  ${SERIAL_SRCS}
)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  # 同一个 include/ 目录同时包含 serial/ 和 rm_serial_driver/ 两个子目录
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Linux 下 serial 库依赖 rt（POSIX 实时扩展）和 pthread
if(UNIX AND NOT APPLE)
  target_link_libraries(${PROJECT_NAME}_lib rt pthread)
endif()

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  rclcpp_components
  rm_interfaces
  rm_utils
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# ── 组件注册 ──────────────────────────────────────────────────────────────────
rclcpp_components_register_nodes(${PROJECT_NAME}_lib
  "pka::serial_driver::UARTNode"
)

# ── 安装 ──────────────────────────────────────────────────────────────────────
install(
  TARGETS ${PROJECT_NAME}_lib
  EXPORT  export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY include/
  DESTINATION include
)

# ── 导出 ──────────────────────────────────────────────────────────────────────
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)
ament_export_dependencies(
  rclcpp
  rclcpp_components
  rm_interfaces
  rm_utils
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

ament_package()