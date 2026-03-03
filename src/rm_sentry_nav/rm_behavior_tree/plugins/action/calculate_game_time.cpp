#include "rm_behavior_tree/plugins/condition/calculate_game_time.hpp"

namespace rm_behavior_tree
{

CalculateGameTimeCondition::CalculateGameTimeCondition(const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&CalculateGameTimeCondition::checkGameStart, this), config)
{
}

BT::NodeStatus CalculateGameTimeCondition::checkGameStart()
{
  int lower_remain_time, higher_remain_time;
  auto msg = getInput<rm_interfaces::msg::GameStatus>("message");
  getInput("lower_remain_time", lower_remain_time);
  getInput("higher_remain_time", higher_remain_time);
  if (!msg) {
    // std::cout << "missing required input [game_status]" << '\n';
    return BT::NodeStatus::FAILURE;
  }

  if (
    msg->stage_remain_time >= lower_remain_time &&
    msg->stage_remain_time <= higher_remain_time) {
    return BT::NodeStatus::SUCCESS;
  } else {
    return BT::NodeStatus::FAILURE;
  }
}
}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::CalculateGameTimeCondition>("CalculateGameTime");
}