#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__CAN_FIRE_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__CAN_FIRE_HPP_

#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include "rm_interfaces/msg/navigation_send.hpp"

namespace rm_behavior_tree
{

class CanFireAction : public BT::RosTopicPubNode<rm_interfaces::msg::NavigationSend>, rclcpp::Node
{
public:
  CanFireAction(
    const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params);

  bool setMessage(rm_interfaces::msg::NavigationSend & msg)override; 

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<bool>("friction_wheel_scan")};
  }

  // BT::NodeStatus onStart() override;

  // BT::NodeStatus onRunning() override;

  // void onHalted() override;

private:
  rclcpp::Publisher<rm_interfaces::msg::NavigationSend>::SharedPtr publisher_navigation_send;
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__CAN_FIRE_HPP_