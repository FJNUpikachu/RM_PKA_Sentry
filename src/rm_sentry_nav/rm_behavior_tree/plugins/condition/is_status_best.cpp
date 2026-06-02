#include "rm_behavior_tree/plugins/condition/is_status_best.hpp"
#include <rm_interfaces/msg/detail/current_hp__struct.hpp>

namespace rm_behavior_tree
{

IsStatusBestCondition::IsStatusBestCondition(const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsStatusBestCondition::checkRobotStatus, this), config)
{
}

BT::NodeStatus IsStatusBestCondition::checkRobotStatus()
{
  auto msg = getInput<rm_interfaces::msg::CurrentHP>("message");

  if (!msg) {
    return BT::NodeStatus::FAILURE;
  }

  // 新逻辑：仅当 current_hp == 400 时返回 SUCCESS，其他情况返回 FAILURE
  if (msg->current_hp == 400) {
    return BT::NodeStatus::SUCCESS;
  } else {
    return BT::NodeStatus::FAILURE;
  }


}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::IsStatusBestCondition>("IsStatusBest");
}




//联盟赛前修改
// #include "rm_behavior_tree/plugins/condition/is_status_ok.hpp"

// namespace rm_behavior_tree
// {

// IsStatusOkCondition::IsStatusOkCondition(const std::string & name, const BT::NodeConfig & config)
// : BT::SimpleConditionNode(name, std::bind(&IsStatusOkCondition::checkRobotStatus, this), config)
// {
// }

// BT::NodeStatus IsStatusOkCondition::checkRobotStatus()
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
//   factory.registerNodeType<rm_behavior_tree::IsStatusOkCondition>("IsStatusOK");
// }
