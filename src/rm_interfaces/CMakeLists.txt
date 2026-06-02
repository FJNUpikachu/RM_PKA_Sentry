cmake_minimum_required(VERSION 3.8)
project(rm_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# code compile
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Armor.msg"
  "msg/Armors.msg"
  "msg/GimbalCmd.msg"
  "msg/SentrySerialReceiveData.msg"
  "msg/SentrySerialSendData.msg"
  "msg/InfantrySerialReceiveData.msg"
  "msg/InfantrySerialSendData.msg"
  "msg/Measurement.msg"
  "msg/Target.msg"
  "msg/DebugLight.msg"
  "msg/DebugLights.msg"
  "msg/DebugArmor.msg"
  "msg/DebugArmors.msg"
  "msg/DecisionNum.msg"
  "msg/NavigationReceive.msg"
  "msg/IsAttacked.msg"
  "msg/AllRobotHP.msg"
  "msg/RfidStatus.msg"
  "msg/NavigationSend.msg"
  "msg/OutpostTarget.msg"
  "msg/OutpostMeasurement.msg"

  "msg/CurrentHP.msg"
  "msg/GameProgress.msg"
  "msg/BulletRest.msg"
  "msg/IsBattling.msg"
  "msg/IsFortress.msg"
  "msg/IsSupply.msg"
  "msg/PoseStatus.msg"
  "msg/OutpostHP.msg"

  "srv/SetMode.srv"
  DEPENDENCIES
    std_msgs
    geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
