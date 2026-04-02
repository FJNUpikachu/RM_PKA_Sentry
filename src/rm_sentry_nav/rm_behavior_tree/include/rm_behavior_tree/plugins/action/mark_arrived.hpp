#pragma once
#include "behaviortree_cpp/behavior_tree.h"

namespace rm_behavior_tree
{

/**
 * @brief MarkArrivedAction
 * 将黑板变量 {nav_arrived} 置为 true，表示机器人已到达终点。
 * 紧接在每条路径最后一个 SendGoal 之后调用。
 * 节点本身始终返回 SUCCESS。
 *
 * 文件放置路径：
 *   rm_behavior_tree/include/rm_behavior_tree/plugins/action/mark_arrived.hpp
 */
class MarkArrivedAction : public BT::SyncActionNode
{
public:
  MarkArrivedAction(const std::string & name, const BT::NodeConfig & config);

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::OutputPort<bool>("nav_arrived", "{nav_arrived}", "写入到达标志")
    };
  }
};

}  // namespace rm_behavior_tree