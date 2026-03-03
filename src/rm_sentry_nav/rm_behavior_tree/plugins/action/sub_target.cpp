#include "rm_behavior_tree/plugins/action/sub_target.hpp"

namespace rm_behavior_tree
{

SubTargetAction::SubTargetAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: BT::RosTopicSubNode<rm_interfaces::msg::Target>(name, conf, params)
{
}

BT::NodeStatus SubTargetAction::onTick(
  const std::shared_ptr<rm_interfaces::msg::Target> & last_msg)
{
  if (last_msg)  // 收到视觉发来的目标坐标时，写入黑板
  {
    setOutput("target", *last_msg);
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::SubTargetAction, "SubTarget");