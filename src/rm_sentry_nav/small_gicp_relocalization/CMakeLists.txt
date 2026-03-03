cmake_minimum_required(VERSION 3.16)
project(small_gicp_relocalization)

## Use C++17 (必须使用 C++17 以兼容 small_gicp)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Export compile commands
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################
find_package(ament_cmake_auto REQUIRED)
find_package(OpenMP REQUIRED)

# 1. 查找 PCL (本地安装必须显式查找)
find_package(PCL REQUIRED COMPONENTS common io kdtree search registration filters)

# 2. 查找本地安装的 small_gicp
find_package(small_gicp REQUIRED)

# 3. 查找本地安装的 ndt_omp
# 注意：koide3/ndt_omp 安装后的 CMake 包名通常是 ndt_omp
find_package(ndt_omp REQUIRED)

ament_auto_find_build_dependencies()

###########
## Build ##
###########

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/small_gicp_relocalization.cpp
)

# 包含目录
target_include_directories(${PROJECT_NAME} PUBLIC
  ${PCL_INCLUDE_DIRS}
  # small_gicp 和 ndt_omp 的 include 路径通常会自动通过下面的 target_link 传递
  # 如果编译报错找不到头文件，检查你的库是否安装到了系统路径 (/usr/local/include)
)

# 链接库
target_link_libraries(${PROJECT_NAME} 
  OpenMP::OpenMP_CXX
  small_gicp::small_gicp  # small_gicp 导出的标准 target
  ndt_omp                 # ndt_omp 导出的 target (通常叫 ndt_omp)
  ${PCL_LIBRARIES}
)

# 注册组件
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN small_gicp_relocalization::SmallGicpRelocalizationNode
  EXECUTABLE ${PROJECT_NAME}_node
)

#############
## Testing ##
#############
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

#############
## Install ##
#############
ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
  launch
)
