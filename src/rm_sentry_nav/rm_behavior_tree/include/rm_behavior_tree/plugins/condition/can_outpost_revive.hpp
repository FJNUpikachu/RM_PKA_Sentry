#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__CAN_OUTPOST_REVIVE_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__CAN_OUTPOST_REVIVE_HPP_

#include "behaviortree_cpp/condition_node.h"

#include "rm_interfaces/msg/tower_status.hpp" 
#include "rm_interfaces/msg/game_status.hpp" 

namespace rm_behavior_tree
{
/**
 * @brief Condition节点，用于判断前哨站(Outpost)是否满足复活条件
 */
class CanOutpostReviveAction : public BT::SimpleConditionNode
{
public:
  CanOutpostReviveAction(const std::string & name, const BT::NodeConfig & config);

  BT::NodeStatus checkCanRevive();

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<rm_interfaces::msg::TowerStatus>("tower_message"),
      BT::InputPort<rm_interfaces::msg::GameStatus>("game_message")
    };
  }

private:
  // 增加记忆状态：用于判断前哨站是否发生过复活
  int last_outpost_hp_;
  int recovery_count_; // 记录已经成功复活的次数
};
}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__CAN_OUTPOST_REVIVE_HPP_