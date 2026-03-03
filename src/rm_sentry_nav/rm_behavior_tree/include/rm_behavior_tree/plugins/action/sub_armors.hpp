#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_ARMORS_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_ARMORS_HPP_

#include "rm_interfaces/msg/navigation_receive.hpp"
#include "behaviortree_ros2/bt_topic_sub_node.hpp"

namespace rm_behavior_tree
{
class SubArmorsAction : public BT::RosTopicSubNode<rm_interfaces::msg::NavigationReceive>
{
public:
  SubArmorsAction(
    const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("topic_name"),
      BT::OutputPort<rm_interfaces::msg::NavigationReceive>("armors")};
  }

  BT::NodeStatus onTick(
    const std::shared_ptr<rm_interfaces::msg::NavigationReceive> & last_msg) override;
};
}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_ARMORS_HPP_