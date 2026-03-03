#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__IS_STATUS_BEST_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__IS_STATUS_BEST_HPP_

#include "behaviortree_cpp/condition_node.h"
#include "rm_interfaces/msg/robot_status.hpp"
#include "rm_interfaces/msg/navigation_receive.hpp"

namespace rm_behavior_tree
{
/**
 * @brief Action节点，用于判断机器人状态是否正常
 * 
 * 该节点从输入端口获取机器人状态消息、血量阈值和热量阈值，并根据条件判断机器人状态是否正常。
 * 如果机器人状态不正常（血量低于阈值或热量高于阈值），返回失败；否则返回成功。
 * @param[in] message 机器人状态话题id
 * @param[in] hp_threshold 最低血量阈值（哨兵最大血量400）
 */
class IsStatusBestAction : public BT::SimpleConditionNode
{
public:
  IsStatusBestAction(const std::string & name, const BT::NodeConfig & config);

  // BT::NodeStatus checkGameStart(BT::TreeNode & self_node)
  BT::NodeStatus checkRobotStatus();

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<rm_interfaces::msg::RobotStatus>("message")};
  }
};
}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__IS_STATUS_OK_HPP_




//联盟赛前的节点
// #ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__IS_STATUS_OK_HPP_
// #define RM_BEHAVIOR_TREE__PLUGINS__ACTION__IS_STATUS_OK_HPP_

// #include "behaviortree_cpp/condition_node.h"
// // #include "rm_interfaces/msg/robot_status.hpp"
// #include "rm_interfaces/msg/navigation_receive.hpp"

// namespace rm_behavior_tree
// {
// /**
//  * @brief Action节点，用于判断机器人状态是否正常
//  * 
//  * 该节点从输入端口获取机器人状态消息、血量阈值和热量阈值，并根据条件判断机器人状态是否正常。
//  * 如果机器人状态不正常（血量低于阈值或热量高于阈值），返回失败；否则返回成功。
//  * @param[in] message 机器人状态话题id
//  * @param[in] hp_threshold 最低血量阈值（哨兵最大血量600）
//  * @param[in] heat_threshold 最大热量阈值（哨兵最高热量400）
//  */
// class IsStatusOKAction : public BT::SimpleConditionNode
// {
// public:
//   IsStatusOKAction(const std::string & name, const BT::NodeConfig & config);

//   // BT::NodeStatus checkGameStart(BT::TreeNode & self_node)
//   BT::NodeStatus checkRobotStatus();

//   static BT::PortsList providedPorts()
//   {
//     return {
//       BT::InputPort<rm_interfaces::msg::NavigationReceive>("message"),
//       BT::InputPort<int>("hp_threshold"), BT::InputPort<int>("heat_threshold")};
//   }
// };
// }  // namespace rm_behavior_tree

// #endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__IS_STATUS_OK_HPP_