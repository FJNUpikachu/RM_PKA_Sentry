#include "rm_behavior_tree/plugins/condition/is_attacked.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rm_behavior_tree
{

IsAttakedAction::IsAttakedAction(const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsAttakedAction::checkRobotAttacked, this), config),
  msg_received_(false) // 初始化消息接收标志
{
  // 创建一个 ROS 2 节点
  node_ = rclcpp::Node::make_shared("is_attacked_node");
  
  // 初始化订阅者
  subscriber_ = node_->create_subscription<rm_interfaces::msg::IsAttacked>(
    "serial/receive", 
    rclcpp::SensorDataQoS(),
    std::bind(&IsAttakedAction::messageCallback, this, std::placeholders::_1)
  );
}

BT::NodeStatus IsAttakedAction::checkRobotAttacked()
{
  // 检查是否接收到消息
  if (!msg_received_) {
    return BT::NodeStatus::FAILURE; // 未接收到消息，返回失败
  }

  // 根据最后接收到的消息的 is_attacked 值返回状态
  return (last_msg_.is_attacked == 1) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

void IsAttakedAction::messageCallback(const rm_interfaces::msg::IsAttacked::SharedPtr msg)
{
  last_msg_ = *msg; // 更新最后接收到的消息
  msg_received_ = true; // 标记消息已接收
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::IsAttakedAction>("IsAttaked");
}






// #include "rm_behavior_tree/plugins/condition/is_attacked.hpp"

// namespace rm_behavior_tree
// {

// IsAttakedAction::IsAttakedAction(const std::string & name, const BT::NodeConfig & config)
// : BT::SimpleConditionNode(name, std::bind(&IsAttakedAction::checkRobotAttacked, this), config)
// {
// }

// BT::NodeStatus IsAttakedAction::checkRobotAttacked()
// {
//   auto msg = getInput<rm_interfaces::msg::SerialReceiveData>("message");

//   if (!msg) {
//     // std::cout << "missing required input [game_status]" << '\n';
//     return BT::NodeStatus::FAILURE;
//   }

//   if (msg->is_attacked) {
//     // 机器人受到攻击
//     return BT::NodeStatus::SUCCESS;
//   }

//   return BT::NodeStatus::FAILURE;
// }

// }  // namespace rm_behavior_tree

// #include "behaviortree_cpp/bt_factory.h"
// BT_REGISTER_NODES(factory)
// {
//   factory.registerNodeType<rm_behavior_tree::IsAttakedAction>("IsAttaked");
// }
