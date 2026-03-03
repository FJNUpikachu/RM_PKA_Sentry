#include "rm_behavior_tree/plugins/condition/is_status_middle.hpp"
#include <rm_interfaces/msg/detail/robot_status__struct.hpp>

namespace rm_behavior_tree
{

IsStatusMiddleAction::IsStatusMiddleAction(const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsStatusMiddleAction::checkRobotStatus, this), config)
{
}

BT::NodeStatus IsStatusMiddleAction::checkRobotStatus()
{
  auto msg = getInput<rm_interfaces::msg::RobotStatus>("message");

  if (!msg) {
    return BT::NodeStatus::FAILURE;
  }

  // 新逻辑：200 < current_hp < 400 时返回 SUCCESS，否则 FAILURE
  if (msg->current_hp > 200 && msg->current_hp < 400) {
    return BT::NodeStatus::SUCCESS;
  } else {
    return BT::NodeStatus::FAILURE;
  }


}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::IsStatusMiddleAction>("IsStatusMiddle");
}




//联盟赛前修改
// #include "rm_behavior_tree/plugins/condition/is_status_ok.hpp"

// namespace rm_behavior_tree
// {

// IsStatusOKAction::IsStatusOKAction(const std::string & name, const BT::NodeConfig & config)
// : BT::SimpleConditionNode(name, std::bind(&IsStatusOKAction::checkRobotStatus, this), config)
// {
// }

// BT::NodeStatus IsStatusOKAction::checkRobotStatus()
// {
//   int hp_threshold, heat_threshold;
//   auto msg = getInput<rm_interfaces::msg::NavigationReceive>("message");
//   getInput("hp_threshold", hp_threshold);
//   getInput("heat_threshold", heat_threshold);

//   if (!msg) {
//     // throw BT::RuntimeError("missing required input [game_status]: ", msg.error());
//     // std::cout << "missing required input [game_status]" << '\n';
//     return BT::NodeStatus::FAILURE;
//   }

//   if (msg->current_hp < hp_threshold || msg->shooter_heat > heat_threshold) {
//     // std::cout << "血量/热量达到预警值" << '\n';
//     return BT::NodeStatus::FAILURE;
//   } else {
//     // std::cout << "血量/热量正常" << '\n';
//     return BT::NodeStatus::SUCCESS;
//   }
// }

// }  // namespace rm_behavior_tree

// #include "behaviortree_cpp/bt_factory.h"
// BT_REGISTER_NODES(factory)
// {
//   factory.registerNodeType<rm_behavior_tree::IsStatusOKAction>("IsStatusOK");
// }
