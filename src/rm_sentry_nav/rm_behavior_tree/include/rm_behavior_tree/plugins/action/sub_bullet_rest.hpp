#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_BULLET_REST_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_BULLET_REST_HPP_

#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "rm_interfaces/msg/navigation_receive.hpp"
// 引入新的消息接口头文件
#include "rm_interfaces/msg/bullet_rest.hpp"

namespace rm_behavior_tree
{
// 继承的模板类型改为 TowerStatus
class SubBulletRestAction : public BT::RosTopicSubNode<rm_interfaces::msg::BulletRest>
{
public:
  SubBulletRestAction(
    const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("topic_name"),
      // 定义输出黑板的端口名称和类型
      BT::OutputPort<rm_interfaces::msg::BulletRest>("bullet_rest")};
  }

  BT::NodeStatus onTick(
    const std::shared_ptr<rm_interfaces::msg::BulletRest> & last_msg) override;
};
}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_BULLET_REST_HPP_
// 这个是允许发弹量