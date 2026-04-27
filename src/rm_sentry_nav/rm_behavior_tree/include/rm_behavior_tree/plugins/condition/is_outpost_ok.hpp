#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__IS_OUTPOST_OK_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__IS_OUTPOST_OK_HPP_

#include "behaviortree_cpp/condition_node.h"
// 请根据实际情况确认前哨站状态是包含在 RobotStatus 还是其他自定义消息（如 GameStatus）中
#include "rm_interfaces/msg/tower_status.hpp" 
#include "rm_interfaces/msg/navigation_receive.hpp"

namespace rm_behavior_tree
{
/**
 * @brief Condition节点，用于判断前哨站(Outpost)状态是否正常
 * * 该节点从输入端口获取前哨站/比赛状态消息和血量阈值，并根据条件判断前哨站状态是否正常。
 * 如果前哨站状态不正常（例如血量低于阈值或已被击毁），返回失败(FAILURE)；否则返回成功(SUCCESS)。
 * @param[in] message 状态话题消息
 */
class IsOutpostOkAction : public BT::SimpleConditionNode
{
public:
  IsOutpostOkAction(const std::string & name, const BT::NodeConfig & config);

  // 检查前哨站状态的核心函数
  BT::NodeStatus checkOutpostStatus();

  static BT::PortsList providedPorts()
  {
    return {
      // 接收状态消息
      BT::InputPort<rm_interfaces::msg::TowerStatus>("message")
    };
  }
};
}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__IS_OUTPOST_OK_HPP_