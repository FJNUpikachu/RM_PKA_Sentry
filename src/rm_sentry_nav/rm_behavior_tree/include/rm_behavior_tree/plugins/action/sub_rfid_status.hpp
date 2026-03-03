#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_RFID_STATUS_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_RFID_STATUS_HPP_

#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "rm_interfaces/msg/navigation_receive.hpp"
#include "rm_interfaces/msg/rfid_status.hpp"

namespace rm_behavior_tree
{
class SubRfidStatusAction : public BT::RosTopicSubNode<rm_interfaces::msg::RfidStatus>
{
public:
  SubRfidStatusAction(
    const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("topic_name"),
      BT::OutputPort<rm_interfaces::msg::RfidStatus>("rfid_status")};
  }

  BT::NodeStatus onTick(
    const std::shared_ptr<rm_interfaces::msg::RfidStatus> & last_msg) override;
};
}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_RFID_STATUS_HPP_