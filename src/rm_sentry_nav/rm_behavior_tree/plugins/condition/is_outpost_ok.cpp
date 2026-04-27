#include "rm_behavior_tree/plugins/condition/is_outpost_ok.hpp"
#include <rm_interfaces/msg/detail/tower_status__struct.hpp>

namespace rm_behavior_tree
{

IsOutpostOkAction::IsOutpostOkAction(const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsOutpostOkAction::checkOutpostStatus, this), config)
{
}

BT::NodeStatus IsOutpostOkAction::checkOutpostStatus()
{
  // 获取输入端口绑定的状态消息
  auto msg = getInput<rm_interfaces::msg::TowerStatus>("message");

  // 如果未获取到消息，视为失败
  if (!msg) {
    return BT::NodeStatus::FAILURE;
  }

  // 判断条件：前哨站血量大于0时，返回SUCCESS，否则返回FAILURE
  if (msg->outpost_hp > 0) {
    return BT::NodeStatus::SUCCESS;
  } else {
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace rm_behavior_tree

// 注册到 BehaviorTree 节点工厂中
#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::IsOutpostOkAction>("IsOutpostOk");
}