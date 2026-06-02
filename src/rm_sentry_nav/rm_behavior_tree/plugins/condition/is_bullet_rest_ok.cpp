#include "rm_behavior_tree/plugins/condition/is_bullet_rest_ok.hpp"
#include <rm_interfaces/msg/detail/bullet_rest__struct.hpp>

namespace rm_behavior_tree
{

IsBulletRestOkCondition::IsBulletRestOkCondition(const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsBulletRestOkCondition::checkBulletRestOk, this), config)
{
}

BT::NodeStatus IsBulletRestOkCondition::checkBulletRestOk()
{
  // 获取输入端口绑定的状态消息
  auto msg = getInput<rm_interfaces::msg::BulletRest>("message");

  // 如果未获取到消息，视为失败
  if (!msg) {
    return BT::NodeStatus::FAILURE;
  }

  // 判断条件：前哨站血量大于0时，返回SUCCESS，否则返回FAILURE
  if (msg->bullet_rest > 50) {
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
  factory.registerNodeType<rm_behavior_tree::IsBulletRestOkCondition>("IsBulletRestOk");
}