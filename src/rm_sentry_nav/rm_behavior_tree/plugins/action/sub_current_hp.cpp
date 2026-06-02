#include "rm_behavior_tree/plugins/action/sub_current_hp.hpp"

namespace rm_behavior_tree
{

SubCurrentHPAction::SubCurrentHPAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: BT::RosTopicSubNode<rm_interfaces::msg::CurrentHP>(name, conf, params)
{
}

BT::NodeStatus SubCurrentHPAction::onTick(
  const std::shared_ptr<rm_interfaces::msg::CurrentHP> & last_msg)
{
  if (last_msg)  // empty if no new message received, since the last tick
  {
    RCLCPP_DEBUG(
      logger(), "[%s] new message", name().c_str());
    setOutput("current_hp", *last_msg);
  }
  return BT::NodeStatus::SUCCESS;
}


}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::SubCurrentHPAction, "SubCurrentHP");