#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__POSE_SWITCH_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__POSE_SWITCH_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_ros2/plugins.hpp"

// 【新增/修改】：引入你的自定义姿态状态消息头文件
#include "rm_interfaces/msg/pose_status.hpp"

namespace rm_behavior_tree
{

class PoseSwitch : public BT::SyncActionNode
{
public:
  PoseSwitch(
    const std::string & name, 
    const BT::NodeConfig & conf, 
    const BT::RosNodeParams & params);

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("topic_name"),
      BT::InputPort<int>("Defend"),
      BT::InputPort<int>("Attack"),
      BT::InputPort<int>("Run"),
      // 声明输出到黑板的端口
      BT::OutputPort<int>("pose_status_out")
    };
  }

private:
  rclcpp::Node::SharedPtr node_;
  
  // 【修改】：使用自定义消息 rm_interfaces::msg::PoseStatus 的发布者
  rclcpp::Publisher<rm_interfaces::msg::PoseStatus>::SharedPtr publisher_;
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__POSE_SWITCH_HPP_