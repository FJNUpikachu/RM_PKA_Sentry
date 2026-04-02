#include "rm_behavior_tree/plugins/action/mark_arrived.hpp"
#include <iostream>

namespace rm_behavior_tree
{

MarkArrivedAction::MarkArrivedAction(
  const std::string & name, const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
}

BT::NodeStatus MarkArrivedAction::tick()
{
  setOutput("nav_arrived", true);
  std::cout << "[MarkArrived] 导航完成，机器人已停在终点。" << std::endl;
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::MarkArrivedAction>("MarkArrived");
}