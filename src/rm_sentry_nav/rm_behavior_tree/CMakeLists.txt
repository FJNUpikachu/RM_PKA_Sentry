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

ament_auto_add_library(is_bullet_rest_ok SHARED 
    plugins/condition/is_bullet_rest_ok.cpp
)
add_target_dependencies(is_bullet_rest_ok)

ament_auto_add_library(is_outpost_ok SHARED 
    plugins/condition/is_outpost_ok.cpp
)
add_target_dependencies(is_outpost_ok)

ament_auto_add_library(is_rfid_detected SHARED 
    plugins/condition/is_rfid_detected.cpp
)
add_target_dependencies(is_rfid_detected)

ament_auto_add_library(is_fortress_detected SHARED 
    plugins/condition/is_fortress_detected.cpp
)
add_target_dependencies(is_fortress_detected)

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

ament_auto_add_library(can_outpost_revive SHARED 
    plugins/condition/can_outpost_revive.cpp
)
add_target_dependencies(can_outpost_revive)

ament_auto_add_library(is_battle_status SHARED 
    plugins/condition/is_battle_status.cpp
)
add_target_dependencies(is_battle_status)

ament_auto_add_library(is_arrived SHARED 
    plugins/condition/is_arrived.cpp
)
add_target_dependencies(is_arrived)

# Actions
ament_auto_add_library(send_goal SHARED 
    plugins/action/send_goal.cpp
)
add_target_dependencies(send_goal)

ament_auto_add_library(pose_switch SHARED 
    plugins/action/pose_switch.cpp
)
add_target_dependencies(pose_switch)

ament_auto_add_library(mark_arrived SHARED 
    plugins/action/mark_arrived.cpp
)
add_target_dependencies(mark_arrived)

ament_auto_add_library(sub_current_hp SHARED 
    plugins/action/sub_current_hp.cpp
)
add_target_dependencies(sub_current_hp)

ament_auto_add_library(sub_game_progress SHARED 
    plugins/action/sub_game_progress.cpp
)
add_target_dependencies(sub_game_progress)

ament_auto_add_library(sub_armors SHARED 
    plugins/action/sub_armors.cpp
)
add_target_dependencies(sub_armors)

ament_auto_add_library(sub_global_costmap SHARED 
    plugins/action/sub_global_costmap.cpp
)
add_target_dependencies(sub_global_costmap)

ament_auto_add_library(sub_is_fortress SHARED 
    plugins/action/sub_is_fortress.cpp
)
add_target_dependencies(sub_is_fortress)

ament_auto_add_library(sub_outpost_hp SHARED 
    plugins/action/sub_outpost_hp.cpp
)
add_target_dependencies(sub_outpost_hp)

ament_auto_add_library(print_message SHARED 
    plugins/action/print_message.cpp
)
add_target_dependencies(print_message)


ament_auto_add_library(sub_is_supply SHARED 
    plugins/action/sub_is_supply.cpp
)
add_target_dependencies(sub_is_supply)

ament_auto_add_library(sub_bullet_rest SHARED 
    plugins/action/sub_bullet_rest.cpp
)
add_target_dependencies(sub_bullet_rest)

ament_auto_add_library(sub_battling SHARED 
    plugins/action/sub_battling.cpp
)
add_target_dependencies(sub_battling)


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

ament_auto_add_library(move_around SHARED 
    plugins/action/move_around.cpp
)
add_target_dependencies(move_around)



ament_auto_package(
    USE_SCOPED_HEADER_INSTALL_DIR
    INSTALL_TO_SHARE
    launch
    config
)