#include "rm_behavior_tree/plugins/action/sub_battling.hpp"

namespace rm_behavior_tree
{

SubBattlingAction::SubBattlingAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: BT::RosTopicSubNode<rm_interfaces::msg::IsBattling>(name, conf, params)
{
}

BT::NodeStatus SubBattlingAction::onTick(
  const std::shared_ptr<rm_interfaces::msg::IsBattling> & last_msg)
{
  if (last_msg)  // empty if no new message received, since the last tick
  {
    RCLCPP_DEBUG(
      logger(), "[%s] new message, is_battling: %s", name().c_str(),
      std::to_string(last_msg->is_battling).c_str());
    setOutput("is_battling", *last_msg);
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::SubBattlingAction, "SubBattling");