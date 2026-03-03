#include "rm_behavior_tree/plugins/action/send_nav2_goal.hpp"

namespace rm_behavior_tree
{
SendNav2Goal::SendNav2Goal(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: BT::RosActionNode<nav2_msgs::action::NavigateToPose>(name, conf, params)
{
}

bool SendNav2Goal::setGoal(RosActionNode::Goal & goal)
{
  auto goal_pose = getInput<geometry_msgs::msg::PoseStamped>("goal");
  if (!goal_pose) {
    RCLCPP_ERROR(node_->get_logger(), "SendNav2Goal: 无法从黑板获取 goal_pose");
    return false;
  }
  goal.pose = goal_pose.value();
  return true;
}

void SendNav2Goal::onHalt()
{
  RCLCPP_INFO(node_->get_logger(), "SendNav2Goal 被打断 (Halted).");
}

BT::NodeStatus SendNav2Goal::onResultReceived(const RosActionNode::WrappedResult & wr)
{
  if (wr.code == rclcpp_action::ResultCode::SUCCEEDED) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus SendNav2Goal::onFailure(BT::ActionNodeErrorCode error)
{
  RCLCPP_ERROR(node_->get_logger(), "SendNav2Goal 失败，错误码: %d", error);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus SendNav2Goal::onFeedback(
  const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
{
  // 关键所在：只要底盘还在移动，就返回 RUNNING，阻止行为树疯狂发新目标！
  return BT::NodeStatus::RUNNING;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::SendNav2Goal, "SendNav2Goal");