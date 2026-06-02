#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_GAME_PROGRESS_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_GAME_PROGRESS_HPP_

#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "rm_interfaces/msg/navigation_receive.hpp"
#include "rm_interfaces/msg/game_progress.hpp"

namespace rm_behavior_tree
{
class SubGameProgressAction : public BT::RosTopicSubNode<rm_interfaces::msg::GameProgress>
{
public:
  SubGameProgressAction(
    const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("topic_name"),
      BT::OutputPort<rm_interfaces::msg::GameProgress>("game_progress")};
  }

  BT::NodeStatus onTick(
    const std::shared_ptr<rm_interfaces::msg::GameProgress> & last_msg) override;
};
}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_GAME_PROGRESS_HPP_