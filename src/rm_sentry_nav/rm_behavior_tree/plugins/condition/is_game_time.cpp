//初版

#include "rm_behavior_tree/plugins/condition/is_game_time.hpp"

namespace rm_behavior_tree
{

IsGameTimeCondition::IsGameTimeCondition(const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsGameTimeCondition::checkGameStart, this), config)
{
}

BT::NodeStatus IsGameTimeCondition::checkGameStart()
{
  int game_progress;
  auto msg = getInput<rm_interfaces::msg::GameStatus>("message");
  getInput("game_progress", game_progress);
  if (!msg) {
    // std::cout << "missing required input [game_status]" << '\n';
    return BT::NodeStatus::FAILURE;
  }

  if (
    msg->game_progress == game_progress) {
    return BT::NodeStatus::SUCCESS;
  } else {
    return BT::NodeStatus::FAILURE;
  }
}
}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::IsGameTimeCondition>("IsGameTime");
}


//改动版  效果：game_progress达到一次4后，节点状态一直返回成功

// #include "rm_behavior_tree/plugins/condition/is_game_time.hpp"

// namespace rm_behavior_tree
// {

// class IsGameTimeCondition : public BT::ConditionNode
// {
// public:
//   IsGameTimeCondition(const std::string & name, const BT::NodeConfig & config)
//   : BT::ConditionNode(name, config)
//   {
//     // 从端口读取目标进度值（保持原配置的输入端口名）
//     if (!getInput("game_progress", target_game_progress_)) {
//       throw BT::RuntimeError("Missing required input [game_progress]");
//     }
//   }

//   static BT::PortsList providedPorts()
//   {
//     return {
//       BT::InputPort<int>("game_progress", "目标比赛进度值"),
//       BT::InputPort<rm_interfaces::msg::GameStatus>("message", "比赛状态消息")
//     };
//   }

//   virtual BT::NodeStatus tick() override
//   {
//     // 如果已经满足条件，保持永久成功
//     if (game_progress_reached_) {
//       return BT::NodeStatus::SUCCESS;
//     }

//     // 获取当前比赛状态（保持原配置的输入端口名）
//     auto msg = getInput<rm_interfaces::msg::GameStatus>("message");
//     if (!msg) {
//       return BT::NodeStatus::FAILURE;
//     }

//     // 检测进度值变化
//     if (msg->game_progress == target_game_progress_) {
//       game_progress_reached_ = true;
//       return BT::NodeStatus::SUCCESS;
//     }
//     return BT::NodeStatus::FAILURE;
//   }

// private:
//   bool game_progress_reached_ = false;
//   int target_game_progress_;
// };

// }  // namespace rm_behavior_tree

// #include "behaviortree_cpp/bt_factory.h"
// BT_REGISTER_NODES(factory)
// {
//   factory.registerNodeType<rm_behavior_tree::IsGameTimeCondition>("IsGameTime");
// }