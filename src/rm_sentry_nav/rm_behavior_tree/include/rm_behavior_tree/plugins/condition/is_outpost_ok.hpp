#ifndef RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_OUTPOST_OK_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_OUTPOST_OK_HPP_

#include "behaviortree_cpp/condition_node.h"
#include "rm_interfaces/msg/outpost_hp.hpp"

namespace rm_behavior_tree
{
/**
 * @brief Condition节点，用于判断前哨站(Outpost)血量是否正常
 * 该节点从输入端口获取 `rm_interfaces::msg::OutpostHP` 消息，并判断血量是否大于0。
 * 如果血量大于0返回 SUCCESS，否则返回 FAILURE。
 * @param[in] message 状态话题消息
 */
class IsOutpostOkCondition : public BT::SimpleConditionNode
{
public:
  IsOutpostOkCondition(const std::string & name, const BT::NodeConfig & config);

  BT::NodeStatus checkOutpostOk();

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<rm_interfaces::msg::OutpostHP>("message")
    };
  }
};
}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_OUTPOST_OK_HPP_