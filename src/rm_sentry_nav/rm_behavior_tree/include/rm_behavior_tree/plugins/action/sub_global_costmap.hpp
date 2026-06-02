// #ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_GLOBAL_COSTMAP_HPP_
// #define RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_GLOBAL_COSTMAP_HPP_

// #include "behaviortree_ros2/bt_topic_sub_node.hpp"
// #include "rm_interfaces/msg/navigation_receive.hpp"
// #include "rm_interfaces/msg/game_status.hpp"
// #include "nav_msgs/msg/occupancy_grid.hpp"

// namespace rm_behavior_tree
// {
// class SubGlobalCostmapAction : public BT::RosTopicSubNode<nav_msgs/msg/occupancy_grid.hpp>
// {
// public:
//   SubGlobalCostmapAction(
//     const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params);

//   static BT::PortsList providedPorts()
//   {
//     return {
//       BT::InputPort<std::string>("topic_name"),
//       BT::OutputPort<nav_msgs/msg/occupancy_grid.hpp>("global_costmap")};
//   }

//   BT::NodeStatus onTick(
//     const std::shared_ptr<nav_msgs/msg/occupancy_grid.hpp> & last_msg) override;
// };
// }  // namespace rm_behavior_tree

// #endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_GAME_STATUS_HPP_
#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_GLOBAL_COSTMAP_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_GLOBAL_COSTMAP_HPP_

#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace rm_behavior_tree
{
class SubGlobalCostmapAction : public BT::RosTopicSubNode<nav_msgs::msg::OccupancyGrid>
{
public:
  SubGlobalCostmapAction(
    const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("topic_name"),
      BT::OutputPort<nav_msgs::msg::OccupancyGrid>("global_costmap")};
  }

  BT::NodeStatus onTick(
    const std::shared_ptr<nav_msgs::msg::OccupancyGrid> & last_msg) override;
};
}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_GLOBAL_COSTMAP_HPP_