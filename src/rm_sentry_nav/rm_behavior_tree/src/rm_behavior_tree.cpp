#include "rm_behavior_tree/rm_behavior_tree.h"

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "behaviortree_cpp/utils/shared_library.h"
#include "behaviortree_ros2/plugins.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  BT::BehaviorTreeFactory factory;

  std::string bt_xml_path;
  auto node = std::make_shared<rclcpp::Node>("rm_behavior_tree");
  node->declare_parameter<std::string>(
    "style", "./rm_decision_ws/rm_behavior_tree/rm_behavior_tree.xml");
  node->get_parameter_or<std::string>(
    "style", bt_xml_path, "./rm_decision_ws/rm_behavior_tree/config/attack_left.xml");

  std::cout << "Start RM_Behavior_Tree" << '\n';
  RCLCPP_INFO(node->get_logger(), "Load bt_xml: \e[1;42m %s \e[0m", bt_xml_path.c_str());

  BT::RosNodeParams params_update_msg;
  params_update_msg.nh = std::make_shared<rclcpp::Node>("update_msg");

  BT::RosNodeParams params_robot_control;
  params_robot_control.nh = std::make_shared<rclcpp::Node>("robot_control");
  params_robot_control.default_port_value = "robot_control";

  BT::RosNodeParams params_send_goal;
  params_send_goal.nh = std::make_shared<rclcpp::Node>("send_goal");
  params_send_goal.default_port_value = "goal_pose";

    BT::RosNodeParams params_pub_nav2_goal;
  params_pub_nav2_goal.nh = std::make_shared<rclcpp::Node>("pub_nav2_goal");
  params_pub_nav2_goal.default_port_value = "goal_pose";

  BT::RosNodeParams params_can_fire;
  params_can_fire.nh = std::make_shared<rclcpp::Node>("can_fire");
  params_can_fire.default_port_value = "can_fire";

  BT::RosNodeParams params_calculate_attack_pose;
  params_calculate_attack_pose.nh = std::make_shared<rclcpp::Node>("calculate_attack_pose");
  params_calculate_attack_pose.default_port_value = "calculate_attack_pose";

  BT::RosNodeParams params_send_nav2_goal;
  params_send_nav2_goal.nh = std::make_shared<rclcpp::Node>("send_nav2_goal");
  params_send_nav2_goal.default_port_value = "navigate_to_pose";

  // clang-format off
  const std::vector<std::string> msg_update_plugin_libs = {
    "sub_all_robot_hp",
    "sub_robot_status",
    "sub_game_status",
    "sub_armors",
    "sub_decision_num",
    "sub_rfid_status",
    "sub_global_costmap",
    "sub_target",
  };

  const std::vector<std::string> bt_plugin_libs = {
    "rate_controller",
    "decision_switch",
    "is_game_time",
    "is_status_ok",
    "is_detect_enemy",
    "is_attacked",
    "is_friend_ok",
    "is_outpost_ok",
    "is_rfid_detected",
    "is_rfid_not_detected",
    "get_current_location",
    "move_around",
    "print_message",
    "check_goal_reached",
    "is_status_best",
    "is_status_middle",
    "is_status_bad",
  };
  // clang-format on

  for (const auto & p : msg_update_plugin_libs) {
    RegisterRosNode(factory, BT::SharedLibrary::getOSName(p), params_update_msg);
  }

  for (const auto & p : bt_plugin_libs) {
    factory.registerFromPlugin(BT::SharedLibrary::getOSName(p));
  }

  RegisterRosNode(factory, BT::SharedLibrary::getOSName("send_goal"), params_send_goal);

  RegisterRosNode(factory, BT::SharedLibrary::getOSName("pub_nav2_goal"), params_pub_nav2_goal);

  RegisterRosNode(factory, BT::SharedLibrary::getOSName("robot_control"), params_robot_control);

  RegisterRosNode(factory, BT::SharedLibrary::getOSName("can_fire"), params_can_fire);

  RegisterRosNode(factory, BT::SharedLibrary::getOSName("calculate_attack_pose"), params_calculate_attack_pose);

  RegisterRosNode(factory, BT::SharedLibrary::getOSName("send_nav2_goal"), params_send_nav2_goal);

  auto tree = factory.createTreeFromFile(bt_xml_path);

  // Connect the Groot2Publisher. This will allow Groot2 to get the tree and poll status updates.
  const unsigned port = 1667;
  BT::Groot2Publisher publisher(tree, port);

  while (rclcpp::ok()) {
    tree.tickWhileRunning(std::chrono::milliseconds(10));
  }

  rclcpp::shutdown();
  return 0;
}