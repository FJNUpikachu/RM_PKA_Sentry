#ifndef RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_BATTLE_STATUS_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_BATTLE_STATUS_HPP_

#include "behaviortree_cpp/condition_node.h"
#include "rm_interfaces/msg/is_battling.hpp"

namespace rm_behavior_tree
{
/**
 * @brief Condition节点，用于判断机器人是否处于战斗状态
 * * 该节点从输入端口获取机器人状态消息，并根据 is_battling 标志判断战斗状态。
 * 如果 is_battling 值为 1，返回 SUCCESS（成功）；否则返回 FAILURE（失败）。
 * @param[in] message 机器人状态消息
 */
class IsBattleStatusCondition : public BT::SimpleConditionNode
{
public:
  IsBattleStatusCondition(const std::string & name, const BT::NodeConfig & config);

  // 检查战斗状态的回调函数
  BT::NodeStatus checkBattleStatus();

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<rm_interfaces::msg::IsBattling>("message")
    };
  }
};
}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_BATTLE_STATUS_HPP_