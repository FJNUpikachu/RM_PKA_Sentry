#include "rm_behavior_tree/plugins/action/robot_control.hpp"

namespace rm_behavior_tree
{

RobotControlAction::RobotControlAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: RosTopicPubNode<rm_interfaces::msg::NavigationSend>(name, conf, params)
{
}

bool RobotControlAction::setMessage(rm_interfaces::msg::NavigationSend & msg)
{
  getInput("stop_gimbal_scan", msg.stop_gimbal_scan);
  getInput("chassis_spin_vel", msg.chassis_spin_vel);

  // std::cout << "stop_gimbal_scan: " << msg.stop_gimbal_scan << '\n';
  // std::cout << "chassis_spin_vel: " << msg.chassis_spin_vel << '\n';
    // std::cout << "stop_gimbal_scan: " << msg.stop_gimbal_scan << '\n';
      // std::cout << helloworld<< '\n';

  return true;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::RobotControlAction, "RobotControl");