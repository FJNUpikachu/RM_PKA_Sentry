#include "rm_behavior_tree/plugins/action/sub_rfid_status.hpp"

namespace rm_behavior_tree
{

SubRfidStatusAction::SubRfidStatusAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: BT::RosTopicSubNode<rm_interfaces::msg::RfidStatus>(name, conf, params)
{
}

BT::NodeStatus SubRfidStatusAction::onTick(
  const std::shared_ptr<rm_interfaces::msg::RfidStatus> & last_msg)
{
  if (last_msg)  // empty if no new message received, since the last tick
  {
    RCLCPP_DEBUG(
      logger(), "[%s] new message", name().c_str());
    setOutput("rfid_status", *last_msg);
  }
  return BT::NodeStatus::SUCCESS;
}


}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::SubRfidStatusAction, "SubRfidStatus");