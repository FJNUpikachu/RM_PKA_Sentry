#include "rm_behavior_tree/plugins/condition/is_outpost_ok.hpp"

namespace rm_behavior_tree
{

IsOutpostOkCondition::IsOutpostOkCondition(const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsOutpostOkCondition::checkOutpostOk, this), config)
{
}

BT::NodeStatus IsOutpostOkCondition::checkOutpostOk()
{
  auto msg = getInput<rm_interfaces::msg::OutpostHP>("message");

  if (!msg) {
    return BT::NodeStatus::FAILURE;
  }

  if ((*msg).outpost_hp > 0) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

}  // namespace rm_behavior_tree

// 注册到 BehaviorTree 节点工厂中
#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::IsOutpostOkCondition>("IsOutpostOk");
}