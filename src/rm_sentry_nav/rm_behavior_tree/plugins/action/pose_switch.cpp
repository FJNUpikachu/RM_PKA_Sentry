#include "rm_behavior_tree/plugins/action/pose_switch.hpp"
#include <iostream>
#include "rm_interfaces/msg/pose_status.hpp" // 【新增头文件】

namespace rm_behavior_tree
{

PoseSwitch::PoseSwitch(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: BT::SyncActionNode(name, conf), node_(params.nh)
{
  std::string topic_name = "PoseStatus"; // 【修改默认话题名】
  getInput("topic_name", topic_name);

  // 【修改发布者类型】
  publisher_ = node_->create_publisher<rm_interfaces::msg::PoseStatus>(topic_name, 10);
}

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::PoseSwitch, "PoseSwitch");

BT::NodeStatus PoseSwitch::tick()
{
  int defend = 0;
  int attack = 0;
  int run = 0;

  getInput("Defend", defend);
  getInput("Attack", attack);
  getInput("Run", run);

  uint8_t current_pose = 0;
  if (defend == 1) { current_pose = 2; } 
  else if (attack == 1) { current_pose = 1; } 
  else if (run == 1) { current_pose = 3; }

  // 【修改消息发送逻辑】
  rm_interfaces::msg::PoseStatus msg;
  msg.pose_status = current_pose;
  publisher_->publish(msg);

  // RCLCPP_INFO(node_->get_logger(), "PoseSwitch sent pose_status: %d", current_pose);
  // std::cout << "[Direct Output] pose_status value: " << static_cast<int>(current_pose) << std::endl;

  setOutput("pose_status_out", static_cast<int>(current_pose));

  return BT::NodeStatus::SUCCESS;
}
}