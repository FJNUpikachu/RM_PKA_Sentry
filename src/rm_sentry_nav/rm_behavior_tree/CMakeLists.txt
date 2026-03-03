cmake_minimum_required(VERSION 3.10)
project(rm_behavior_tree)

## Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

###########
## Build ##
###########

set(PACKAGE_INCLUDE_DEPENDS
    rm_interfaces
    rclcpp
    std_msgs
    behaviortree_cpp
    behaviortree_ros2
)

# 定义宏函数 add_target_dependencies
function(add_target_dependencies target)
    target_include_directories(${target} PRIVATE $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>)
    ament_target_dependencies(${target} ${PACKAGE_INCLUDE_DEPENDS})
    target_compile_definitions(${target} PRIVATE BT_PLUGIN_EXPORT)
endfunction()

ament_auto_add_executable(rm_behavior_tree 
    src/rm_behavior_tree.cpp
)
# Controllers
ament_auto_add_library(decision_switch SHARED
    plugins/control/decision_switch.cpp
)
add_target_dependencies(decision_switch)

# Decorators
ament_auto_add_library(rate_controller SHARED 
    plugins/decorator/rate_controller.cpp
)
add_target_dependencies(rate_controller)

# Conditions
ament_auto_add_library(is_game_time SHARED 
    plugins/condition/is_game_time.cpp
)
add_target_dependencies(is_game_time)

ament_auto_add_library(is_status_ok SHARED 
    plugins/condition/is_status_ok.cpp
)
add_target_dependencies(is_status_ok)

ament_auto_add_library(is_attacked SHARED 
    plugins/condition/is_attacked.cpp
)
add_target_dependencies(is_attacked)

ament_auto_add_library(is_friend_ok SHARED 
    plugins/condition/is_friend_ok.cpp
)
add_target_dependencies(is_friend_ok)

ament_auto_add_library(is_outpost_ok SHARED 
    plugins/condition/is_outpost_ok.cpp
)
add_target_dependencies(is_outpost_ok)

ament_auto_add_library(is_detect_enemy SHARED 
    plugins/condition/is_detect_enemy.cpp
)
add_target_dependencies(is_detect_enemy)

ament_auto_add_library(is_rfid_detected SHARED 
    plugins/condition/is_rfid_detected.cpp
)
add_target_dependencies(is_rfid_detected)

ament_auto_add_library(is_rfid_not_detected SHARED 
    plugins/condition/is_rfid_not_detected.cpp
)
add_target_dependencies(is_rfid_not_detected)

ament_auto_add_library(is_status_best SHARED 
    plugins/condition/is_status_best.cpp
)
add_target_dependencies(is_status_best)

ament_auto_add_library(is_status_middle SHARED 
    plugins/condition/is_status_middle.cpp
)
add_target_dependencies(is_status_middle)

ament_auto_add_library(is_status_bad SHARED 
    plugins/condition/is_status_bad.cpp
)
add_target_dependencies(is_status_bad)

# Actions
ament_auto_add_library(send_goal SHARED 
    plugins/action/send_goal.cpp
)
add_target_dependencies(send_goal)

ament_auto_add_library(sub_all_robot_hp SHARED 
    plugins/action/sub_all_robot_hp.cpp
)
add_target_dependencies(sub_all_robot_hp)

ament_auto_add_library(sub_robot_status SHARED 
    plugins/action/sub_robot_status.cpp
)
add_target_dependencies(sub_robot_status)

ament_auto_add_library(sub_game_status SHARED 
    plugins/action/sub_game_status.cpp
)
add_target_dependencies(sub_game_status)

ament_auto_add_library(sub_armors SHARED 
    plugins/action/sub_armors.cpp
)
add_target_dependencies(sub_armors)

ament_auto_add_library(sub_global_costmap SHARED 
    plugins/action/sub_global_costmap.cpp
)
add_target_dependencies(sub_global_costmap)

ament_auto_add_library(sub_target SHARED 
    plugins/action/sub_target.cpp
)
add_target_dependencies(sub_target)

ament_auto_add_library(sub_decision_num SHARED
    plugins/action/sub_decision_num.cpp
)
add_target_dependencies(sub_decision_num)

ament_auto_add_library(get_current_location SHARED 
    plugins/action/get_current_location.cpp
)
add_target_dependencies(get_current_location)

ament_auto_add_library(move_around SHARED 
    plugins/action/move_around.cpp
)
add_target_dependencies(move_around)

ament_auto_add_library(robot_control SHARED 
    plugins/action/robot_control.cpp
)
add_target_dependencies(robot_control)

ament_auto_add_library(print_message SHARED 
    plugins/action/print_message.cpp
)
add_target_dependencies(print_message)

ament_auto_add_library(can_fire SHARED 
    plugins/action/can_fire.cpp
)
add_target_dependencies(can_fire)

ament_auto_add_library(check_goal_reached SHARED 
    plugins/action/check_goal_reached.cpp
)
add_target_dependencies(check_goal_reached)

ament_auto_add_library(sub_rfid_status SHARED 
    plugins/action/sub_rfid_status.cpp
)
add_target_dependencies(sub_rfid_status)

ament_auto_add_library(pub_nav2_goal SHARED 
    plugins/action/pub_nav2_goal.cpp
)
add_target_dependencies(pub_nav2_goal)

ament_auto_add_library(send_nav2_goal SHARED 
    plugins/action/send_nav2_goal.cpp
)
add_target_dependencies(send_nav2_goal)

ament_auto_add_library(calculate_attack_pose SHARED 
    plugins/action/calculate_attack_pose.cpp
)
add_target_dependencies(calculate_attack_pose)


ament_auto_package(
    USE_SCOPED_HEADER_INSTALL_DIR
    INSTALL_TO_SHARE
    launch
    config
)