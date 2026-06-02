// #include "rm_behavior_tree/plugins/action/sub_global_costmap.hpp"
// #include "nav_msgs/msg/occupancy_grid.hpp"

// namespace rm_behavior_tree
// {

// SubGlobalCostmapAction::SubGlobalCostmapAction(
//   const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
// : BT::RosTopicSubNode<nav_msgs/msg/occupancy_grid.hpp>(name, conf, params)
// {
// }

// BT::NodeStatus SubGlobalCostmapAction::onTick(
//   const std::shared_ptr<nav_msgs/msg/occupancy_grid.hpp> & last_msg)
// {
//   if (last_msg)  // empty if no new message received, since the last tick
//   {

//   }
//   return BT::NodeStatus::SUCCESS;
// }

// }  // namespace rm_behavior_tree

// #include "behaviortree_ros2/plugins.hpp"
// CreateRosNodePlugin(rm_behavior_tree::SubGlobalCostmapAction, "SubGlobalCostmap");
#include "rm_behavior_tree/plugins/action/sub_global_costmap.hpp"

namespace rm_behavior_tree
{

SubGlobalCostmapAction::SubGlobalCostmapAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: BT::RosTopicSubNode<nav_msgs::msg::OccupancyGrid>(name, conf, params)
{
}

BT::NodeStatus SubGlobalCostmapAction::onTick(
  const std::shared_ptr<nav_msgs::msg::OccupancyGrid> & last_msg)
{
  if (last_msg)  // 收到新地图时，更新到黑板上
  {
    setOutput("global_costmap", *last_msg);
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::SubGlobalCostmapAction, "SubGlobalCostmap");