#include "rm_behavior_tree/plugins/condition/is_battle_status.hpp"
#include <rm_interfaces/msg/detail/is_battling__struct.hpp> // 确保该头文件/消息类型包含 is_battling 字段

namespace rm_behavior_tree
{

IsBattleStatusCondition::IsBattleStatusCondition(const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsBattleStatusCondition::checkBattleStatus, this), config)
{
}

BT::NodeStatus IsBattleStatusCondition::checkBattleStatus()
{
  auto msg = getInput<rm_interfaces::msg::IsBattling>("message");

  if (!msg) {
    return BT::NodeStatus::FAILURE;
  }

  // 新逻辑：当 is_battling 值为 1 时返回 SUCCESS，其他情况返回 FAILURE
  if (msg->is_battling == 1) {
    return BT::NodeStatus::SUCCESS;
  } else {
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::IsBattleStatusCondition>("IsBattleStatus");
}