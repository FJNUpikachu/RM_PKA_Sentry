#include "rm_behavior_tree/plugins/action/check_goal_reached.hpp"
#include <rclcpp/logging.hpp>
#include <cmath>

namespace rm_behavior_tree
{

CheckGoalReached::CheckGoalReached(const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&CheckGoalReached::checkGoalReached, this), config)
{
}

BT::NodeStatus CheckGoalReached::checkGoalReached()
{
    geometry_msgs::msg::TransformStamped current_location;

    // 获取输入端口 current_location
    if (!getInput<geometry_msgs::msg::TransformStamped>("current_location", current_location)) {
        RCLCPP_ERROR(rclcpp::get_logger("CheckGoalReached"), "Missing required input: current_location");
        return BT::NodeStatus::FAILURE;
    }

    // 设置目标位置为 (0, 10.8, 0) 和旋转 (0, 0, 0, 1)
    const double goal_x = 0.0;  // 目标位置的 x 坐标
    const double goal_y = 10.8; // 目标位置的 y 坐标
    const double goal_z = 0.0;  // 目标位置的 z 坐标

    // 计算当前位置与目标位置的距离
    double distance = std::sqrt(
        std::pow(current_location.transform.translation.x - goal_x, 2) +
        std::pow(current_location.transform.translation.y - goal_y, 2) +
        std::pow(current_location.transform.translation.z - goal_z, 2));

    // 判断目标是否达到
    const double threshold = 0.1;  // 允许的误差范围

    if (distance < threshold) {
        RCLCPP_INFO(rclcpp::get_logger("CheckGoalReached"), "Goal reached! Distance: %f", distance);
        return BT::NodeStatus::SUCCESS;  // 返回成功
    } else {
        RCLCPP_INFO(rclcpp::get_logger("CheckGoalReached"), "Goal not reached. Distance: %f", distance);
        return BT::NodeStatus::FAILURE;  // 返回失败
    }
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<rm_behavior_tree::CheckGoalReached>("CheckGoalReached");
}