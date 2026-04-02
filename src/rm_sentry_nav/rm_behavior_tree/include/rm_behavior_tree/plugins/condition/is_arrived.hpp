#pragma once
#include "behaviortree_cpp/behavior_tree.h"

namespace rm_behavior_tree
{

/**
 * @brief IsArrivedCondition
 * 检查黑板变量 {nav_arrived} 是否为 true。
 * - true  → SUCCESS（已到达终点，保持停止）
 * - false / 未设置 → FAILURE（尚未到达，继续导航）
 *
 * 文件放置路径：
 *   rm_behavior_tree/include/rm_behavior_tree/plugins/condition/is_arrived.hpp
 */
class IsArrivedCondition : public BT::SimpleConditionNode
{
public:
  IsArrivedCondition(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<bool>("nav_arrived", false, "导航到达标志（由 MarkArrived 写入）")
    };
  }

private:
  BT::NodeStatus checkArrived();
};

}  // namespace rm_behavior_tree