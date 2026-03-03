//修改qos前
#include "rm_behavior_tree/plugins/action/sub_game_status.hpp"

namespace rm_behavior_tree
{

SubGameStatusAction::SubGameStatusAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: BT::RosTopicSubNode<rm_interfaces::msg::GameStatus>(name, conf, params)
{
}

BT::NodeStatus SubGameStatusAction::onTick(
  const std::shared_ptr<rm_interfaces::msg::GameStatus> & last_msg)
{
  if (last_msg)  // empty if no new message received, since the last tick
  {
    RCLCPP_DEBUG(
      logger(), "[%s] new message, game_progress: %s", name().c_str(),
      std::to_string(last_msg->game_progress).c_str());
    setOutput("game_status", *last_msg);
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::SubGameStatusAction, "SubGameStatus");



// #include "rm_behavior_tree/plugins/action/sub_game_status.hpp"

// namespace rm_behavior_tree
// {

// SubGameStatusAction::SubGameStatusAction(
//   const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
// : BT::RosTopicSubNode<rm_interfaces::msg::GameStatus>(
//     name,
//     conf,
//     // 关键修改：通过 params 传递 QoS 配置
//     {
//       params.ros_node,  // 原始 node 参数
//       params.topic_name,  // 原始 topic 名
//       // 显式设置 QoS
//       rclcpp::QoS(rclcpp::KeepLast(10)).best_effort()  // <-- QoS 配置
//     }) 
// {
// }

// BT::NodeStatus SubGameStatusAction::onTick(
//   const std::shared_ptr<rm_interfaces::msg::GameStatus> & last_msg)
// {
//   if (last_msg)
//   {
//     RCLCPP_DEBUG(
//       logger(), "[%s] new message, game_progress: %s", name().c_str(),
//       std::to_string(last_msg->game_progress).c_str());
//     setOutput("game_status", *last_msg);
//   }
//   return BT::NodeStatus::SUCCESS;
// }

// }  // namespace rm_behavior_tree

// #include "behaviortree_ros2/plugins.hpp"
// CreateRosNodePlugin(rm_behavior_tree::SubGameStatusAction, "SubGameStatus");