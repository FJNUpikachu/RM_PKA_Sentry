#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_TOWER_STATUS_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_TOWER_STATUS_HPP_

#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "rm_interfaces/msg/navigation_receive.hpp"
// 引入新的消息接口头文件
#include "rm_interfaces/msg/tower_status.hpp"

namespace rm_behavior_tree
{
// 继承的模板类型改为 TowerStatus
class SubTowerStatusAction : public BT::RosTopicSubNode<rm_interfaces::msg::TowerStatus>
{
public:
  SubTowerStatusAction(
    const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("topic_name"),
      // 定义输出黑板的端口名称和类型
      BT::OutputPort<rm_interfaces::msg::TowerStatus>("tower_status")};
  }

  BT::NodeStatus onTick(
    const std::shared_ptr<rm_interfaces::msg::TowerStatus> & last_msg) override;
};
}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_TOWER_STATUS_HPP_