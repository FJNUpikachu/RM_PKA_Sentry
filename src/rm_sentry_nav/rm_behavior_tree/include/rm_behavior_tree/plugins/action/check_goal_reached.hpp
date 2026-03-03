#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__CHECK_GOAL_REACHED_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__CHECK_GOAL_REACHED_HPP_

#include "behaviortree_cpp/condition_node.h"
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace rm_behavior_tree
{

/**
 * @brief Condition节点，用于判断目标是否已达到
 * @param[in] current_location 当前位置信息
 */
class CheckGoalReached : public BT::SimpleConditionNode
{
public:
    CheckGoalReached(const std::string & name, const BT::NodeConfig & config);

    BT::NodeStatus checkGoalReached();

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<geometry_msgs::msg::TransformStamped>("is_goal_reached")};
    }
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__CHECK_GOAL_REACHED_HPP_