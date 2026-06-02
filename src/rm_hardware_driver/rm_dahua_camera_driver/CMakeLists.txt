cmake_minimum_required(VERSION 3.8)
project(rm_dahua_camera_driver)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## 启用O3优化 
add_definitions(-O3)

find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(OpenCV REQUIRED)
ament_auto_find_build_dependencies()

###########
## Build ##
###########

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

target_include_directories(${PROJECT_NAME} PUBLIC include)
target_include_directories(${PROJECT_NAME} PUBLIC include ${OpenCV_INCLUDE_DIRS})
target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
target_link_directories(${PROJECT_NAME} PUBLIC /opt/HuarayTech/MVviewer/lib)


target_link_libraries(${PROJECT_NAME} 
    ImageConvert 
    MVSDK 
    MVSDKGuiQt 
    RecordVideo
    VideoRender 
    avcodec 
    avfilter 
    avformat 
    avutil 
    log4cpp 
    postproc 
    swresample 
    swscale
    x264
)

install(TARGETS 
${PROJECT_NAME} 
  DESTINATION lib/${PROJECT_NAME})

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN pka::dahua_camera::DahuaCameraNode
  EXECUTABLE ${PROJECT_NAME}_node
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

endif()

#############
## Install ##
#############

ament_auto_package(
  INSTALL_TO_SHARE
)