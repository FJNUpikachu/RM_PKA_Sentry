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
  std::vector<rclcpp::Node::SharedPtr> ros_node_handles;
  node->declare_parameter<std::string>(
    "style", "./rm_decision_ws/rm_behavior_tree/rm_behavior_tree.xml");
  node->get_parameter_or<std::string>(
    "style", bt_xml_path, "./rm_decision_ws/rm_behavior_tree/config/attack_left.xml");

  std::cout << "Start RM_Behavior_Tree" << '\n';
  RCLCPP_INFO(node->get_logger(), "Load bt_xml: \e[1;42m %s \e[0m", bt_xml_path.c_str());

  BT::RosNodeParams params_update_msg;
  ros_node_handles.push_back(std::make_shared<rclcpp::Node>("update_msg"));
  params_update_msg.nh = ros_node_handles.back();

  BT::RosNodeParams params_send_goal;
  ros_node_handles.push_back(std::make_shared<rclcpp::Node>("send_goal"));
  params_send_goal.nh = ros_node_handles.back();
  params_send_goal.default_port_value = "goal_pose";

    BT::RosNodeParams params_pub_nav2_goal;
  ros_node_handles.push_back(std::make_shared<rclcpp::Node>("pub_nav2_goal"));
  params_pub_nav2_goal.nh = ros_node_handles.back();
  params_pub_nav2_goal.default_port_value = "goal_pose";

  BT::RosNodeParams params_calculate_attack_pose;
  ros_node_handles.push_back(std::make_shared<rclcpp::Node>("calculate_attack_pose"));
  params_calculate_attack_pose.nh = ros_node_handles.back();
  params_calculate_attack_pose.default_port_value = "calculate_attack_pose";

  BT::RosNodeParams params_send_nav2_goal;
  ros_node_handles.push_back(std::make_shared<rclcpp::Node>("send_nav2_goal"));
  params_send_nav2_goal.nh = ros_node_handles.back();
  params_send_nav2_goal.default_port_value = "navigate_to_pose";

  // ★ 新增：为 PoseSwitch 创建并传入 ROS Node 句柄
  BT::RosNodeParams params_pose_switch;
  ros_node_handles.push_back(std::make_shared<rclcpp::Node>("pose_switch_node"));
  params_pose_switch.nh = ros_node_handles.back();


  // clang-format off
  const std::vector<std::string> msg_update_plugin_libs = {
    "sub_current_hp",
    "sub_game_progress",
    "sub_armors",
    "sub_is_supply",
    "sub_bullet_rest",
    "sub_global_costmap",
    "sub_is_fortress",
    "sub_battling",  // ★ 新增：订阅是否处于战斗状态
    "sub_outpost_hp",  // ★ 新增：订阅前哨站血量
  };

  const std::vector<std::string> bt_plugin_libs = {
    "rate_controller",
    "is_game_time",
    "is_status_ok",
    "is_attacked",
    "is_bullet_rest_ok",
    "can_outpost_revive",
    "is_fortress_detected",
    "is_battle_status",
    "is_rfid_detected",
    "is_rfid_not_detected",
    "move_around",
    "print_message",
    "is_status_best",
    "is_status_middle",
    "is_status_bad",
    "is_outpost_ok",  // ★ 新增：检查前哨站是否正常
    "is_arrived",    // ★ 新增：检查是否已到达终点
    "mark_arrived",  // ★ 新增：将到达标志写入黑板
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

  RegisterRosNode(factory, BT::SharedLibrary::getOSName("calculate_attack_pose"), params_calculate_attack_pose);

  RegisterRosNode(factory, BT::SharedLibrary::getOSName("send_nav2_goal"), params_send_nav2_goal);

  RegisterRosNode(factory, BT::SharedLibrary::getOSName("pose_switch"), params_pose_switch);

  auto tree = factory.createTreeFromFile(bt_xml_path);

  // Connect the Groot2Publisher. This will allow Groot2 to get the tree and poll status updates.
  const unsigned port = 1667;
  BT::Groot2Publisher publisher(tree, port);

  while (rclcpp::ok()) {
    try {
      tree.tickWhileRunning(std::chrono::milliseconds(10));
    } catch (const BT::NodeExecutionError & e) {
      RCLCPP_ERROR(node->get_logger(), "Behavior tree node execution error: %s", e.what());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(node->get_logger(), "Behavior tree exception: %s", e.what());
    } catch (...) {
      RCLCPP_ERROR(node->get_logger(), "Unknown exception in behavior tree tick");
    }
  }

  rclcpp::shutdown();
  return 0;
}