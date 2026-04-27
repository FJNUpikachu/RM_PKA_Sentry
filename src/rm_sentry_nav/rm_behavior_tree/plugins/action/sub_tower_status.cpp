#include "rm_behavior_tree/plugins/action/sub_tower_status.hpp"

namespace rm_behavior_tree
{

SubTowerStatusAction::SubTowerStatusAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: BT::RosTopicSubNode<rm_interfaces::msg::TowerStatus>(name, conf, params)
{
}

BT::NodeStatus SubTowerStatusAction::onTick(
  const std::shared_ptr<rm_interfaces::msg::TowerStatus> & last_msg)
{
  if (last_msg)  // empty if no new message received, since the last tick
  {
    RCLCPP_DEBUG(
      logger(), "[%s] new message, base_hp: %s, outpost_hp: %s", name().c_str(),
      std::to_string(last_msg->base_hp).c_str(),
      std::to_string(last_msg->outpost_hp).c_str());
      
    setOutput("tower_status", *last_msg);
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::SubTowerStatusAction, "SubTowerStatus");