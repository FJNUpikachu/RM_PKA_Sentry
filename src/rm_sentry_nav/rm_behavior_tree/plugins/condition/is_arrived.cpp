#include "rm_behavior_tree/plugins/condition/is_arrived.hpp"
#include <iostream>

namespace rm_behavior_tree
{

IsArrivedCondition::IsArrivedCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(
    name, std::bind(&IsArrivedCondition::checkArrived, this), config)
{
}

BT::NodeStatus IsArrivedCondition::checkArrived()
{
  auto val = getInput<bool>("nav_arrived");

  // 未设置或为 false → 尚未到达
  if (!val || !(*val)) {
    return BT::NodeStatus::FAILURE;
  }

  std::cout << "[IsArrived] 已到达终点，保持停止状态。" << std::endl;
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::IsArrivedCondition>("IsArrived");
}