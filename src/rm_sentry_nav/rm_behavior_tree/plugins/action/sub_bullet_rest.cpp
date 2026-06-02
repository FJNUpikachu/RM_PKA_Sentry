#include "rm_behavior_tree/plugins/action/sub_bullet_rest.hpp"

namespace rm_behavior_tree
{

SubBulletRestAction::SubBulletRestAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: BT::RosTopicSubNode<rm_interfaces::msg::BulletRest>(name, conf, params)
{
}

BT::NodeStatus SubBulletRestAction::onTick(
  const std::shared_ptr<rm_interfaces::msg::BulletRest> & last_msg)
{
  if (last_msg)  // empty if no new message received, since the last tick
  {
    RCLCPP_DEBUG(
      logger(), "[%s] new message, bullet_rest: %s", name().c_str(),
      // std::to_string(last_msg->base_hp).c_str(),
      std::to_string(last_msg->bullet_rest).c_str());
      
    setOutput("bullet_rest", *last_msg);
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::SubBulletRestAction, "SubBulletRest");