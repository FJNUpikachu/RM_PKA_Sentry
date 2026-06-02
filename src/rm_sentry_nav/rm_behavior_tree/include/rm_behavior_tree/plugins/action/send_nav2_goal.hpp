#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__SEND_NAV2_GOAL_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__SEND_NAV2_GOAL_HPP_

#include "behaviortree_ros2/bt_action_node.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

namespace rm_behavior_tree
{
class SendNav2Goal : public BT::RosActionNode<nav2_msgs::action::NavigateToPose>
{
public:
  SendNav2Goal(
    const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params);

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal")
    });
  }

  bool setGoal(RosActionNode::Goal & goal) override;
  void onHalt() override;
  BT::NodeStatus onResultReceived(const RosActionNode::WrappedResult & wr) override;
  virtual BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;
  virtual BT::NodeStatus onFeedback(
    const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback) override;
};
}  // namespace rm_behavior_tree

#endif