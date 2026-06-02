#ifndef RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_BULLET_REST_OK_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_BULLET_REST_OK_HPP_

#include "behaviortree_cpp/condition_node.h"
// 请根据实际情况确认发弹量状态是包含在 RobotStatus 还是其他自定义消息（如 GameStatus）中
#include "rm_interfaces/msg/bullet_rest.hpp" 
#include "rm_interfaces/msg/navigation_receive.hpp"

namespace rm_behavior_tree
{
/**
 * @brief Condition节点，用于判断前哨站(BulletRest)状态是否正常
 * * 该节点从输入端口获取前哨站/比赛状态消息和血量阈值，并根据条件判断发弹量状态是否正常。
 * 如果发弹量状态不正常（例如血量低于阈值或已被击毁），返回失败(FAILURE)；否则返回成功(SUCCESS)。
 * @param[in] message 状态话题消息
 */
class IsBulletRestOkCondition : public BT::SimpleConditionNode
{
public:
  IsBulletRestOkCondition(const std::string & name, const BT::NodeConfig & config);

  // 检查发弹量状态的核心函数
  BT::NodeStatus checkBulletRestOk();

  static BT::PortsList providedPorts()
  {
    return {
      // 接收状态消息
      BT::InputPort<rm_interfaces::msg::BulletRest>("message")
    };
  }
};
}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_BULLET_REST_OK_HPP_
// 判断发弹量是否大于一定的值