cmake_minimum_required(VERSION 3.8)
project(rm_bringup)

find_package(ament_cmake_auto REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
  scripts
)

# Install helper scripts as runnable executables: `ros2 run rm_bringup <script>`
install(
  PROGRAMS
    scripts/bag_record_all.sh
    scripts/bag_play_all.sh
    scripts/bag_to_mp4.py
  DESTINATION lib/${PROJECT_NAME}
)
