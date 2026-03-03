#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__IS_ATTACKED_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__IS_ATTACKED_HPP_

#include "behaviortree_cpp/condition_node.h"
#include "rm_interfaces/msg/is_attacked.hpp"
#include "rclcpp/rclcpp.hpp" // 添加 ROS 2 头文件

namespace rm_behavior_tree
{

/**
 * @brief condition节点，用于判断机器人是否被攻击掉血
 * @param[in] message 机器人状态话题id
 */
class IsAttakedAction : public BT::SimpleConditionNode
{
public:
  IsAttakedAction(const std::string & name, const BT::NodeConfig & config);

  BT::NodeStatus checkRobotAttacked();

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<rm_interfaces::msg::IsAttacked>("message")};
  }

private:
  void messageCallback(const rm_interfaces::msg::IsAttacked::SharedPtr msg); // 消息回调函数

  rclcpp::Node::SharedPtr node_; // ROS 2 节点
  rclcpp::Subscription<rm_interfaces::msg::IsAttacked>::SharedPtr subscriber_; // 订阅者
  rm_interfaces::msg::IsAttacked last_msg_; // 存储最后接收到的消息
  bool msg_received_; // 标记是否接收到消息
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__IS_ATTACKED_HPP_





// #ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__IS_ATTACKED_HPP_
// #define RM_BEHAVIOR_TREE__PLUGINS__ACTION__IS_ATTACKED_HPP_

// #include "behaviortree_cpp/condition_node.h"
// #include "rm_interfaces/msg/serial_receive_data.hpp"

// namespace rm_behavior_tree
// {

// /**
//  * @brief condition节点，用于判断机器人是否被攻击掉血
//  * @param[in] message 机器人状态话题id
//  */
// class IsAttakedAction : public BT::SimpleConditionNode
// {
// public:
//   IsAttakedAction(const std::string & name, const BT::NodeConfig & config);

//   // BT::NodeStatus checkGameStart(BT::TreeNode & self_node)
//   BT::NodeStatus checkRobotAttacked();

//   static BT::PortsList providedPorts()
//   {
//     return {BT::InputPor t<rm_interfaces::msg::SerialReceiveData>("message")};
//   }
// };
// }  // namespace rm_behavior_tree

// #endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__IS_ATTACKED_HPP_