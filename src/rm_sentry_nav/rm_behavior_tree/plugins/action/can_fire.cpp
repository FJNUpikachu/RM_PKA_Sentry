#include "rm_behavior_tree/plugins/action/can_fire.hpp"

namespace rm_behavior_tree
{

CanFireAction::CanFireAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: RosTopicPubNode<rm_interfaces::msg::NavigationSend>(name,conf,params),Node("CanFire")
{
  // 创建发布者，发布到 navigation_send 话题
  publisher_navigation_send = this->create_publisher<rm_interfaces::msg::NavigationSend>("navigation_send", 10);
  auto msg = std::make_shared<rm_interfaces::msg::NavigationSend>();
}

bool CanFireAction::setMessage(rm_interfaces::msg::NavigationSend & msg)
{
  getInput("friction_wheel_scan", msg.friction_wheel_scan);

  // 发布消息到 navigation_send 话题
  publisher_navigation_send->publish(msg);

  return true;
}
}

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::CanFireAction, "CanFire");  