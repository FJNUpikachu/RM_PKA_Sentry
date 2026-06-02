#include "rm_behavior_tree/plugins/action/sub_is_supply.hpp"

namespace rm_behavior_tree
{

SubIsSupplyAction::SubIsSupplyAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: BT::RosTopicSubNode<rm_interfaces::msg::IsSupply>(name, conf, params)
{
}

BT::NodeStatus SubIsSupplyAction::onTick(
  const std::shared_ptr<rm_interfaces::msg::IsSupply> & last_msg)
{
  if (last_msg)  // empty if no new message received, since the last tick
  {
    RCLCPP_DEBUG(
      logger(), "[%s] new message", name().c_str());
    setOutput("is_supply", *last_msg);
  }
  return BT::NodeStatus::SUCCESS;
}


}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::SubIsSupplyAction, "SubIsSupply");