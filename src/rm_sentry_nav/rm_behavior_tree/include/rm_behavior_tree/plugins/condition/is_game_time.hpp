#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__IS_GAME_TIME_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__IS_GAME_TIME_HPP_

#include "behaviortree_cpp/condition_node.h"
#include "rm_interfaces/msg/navigation_receive.hpp"
#include "rm_interfaces/msg/game_status.hpp"

namespace rm_behavior_tree
{

/**
 * @brief condition节点，用于判断比赛阶段与剩余时间是否符合预期
  * {0, "未开始比赛"}, {1, "准备阶段"}, {2, "十五秒裁判系统自检阶段"},
  * {3, "五秒倒计时"}, {4, "比赛开始"}, {5, "比赛结算中"}
 * @param[in] message 比赛状态话题id
 * @param[in] game_progress 期望的比赛阶段
 * @param[in] lower_remain_time 期望的剩余时间下限
 * @param[in] higher_remain_time 期望的剩余时间上限
 */
class IsGameTimeCondition : public BT::SimpleConditionNode
{
public:
  IsGameTimeCondition(const std::string & name, const BT::NodeConfig & config);

  BT::NodeStatus checkGameStart();

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<rm_interfaces::msg::GameStatus>("message"),
      BT::InputPort<int>("game_progress")};
  }
};
}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__IS_GAME_TIME_HPP_