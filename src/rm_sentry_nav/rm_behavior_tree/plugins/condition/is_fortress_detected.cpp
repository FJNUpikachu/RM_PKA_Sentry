#include "rm_behavior_tree/plugins/condition/is_fortress_detected.hpp"

namespace rm_behavior_tree
{

IsFortressDetectedCondition::IsFortressDetectedCondition(const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsFortressDetectedCondition::checkFortressDetected, this), config)
{
}

BT::NodeStatus IsFortressDetectedCondition::checkFortressDetected()
{
  // bool friendly_outpost_gain_point;
  auto msg = getInput<rm_interfaces::msg::IsFortress>("message");
  if (!msg) {
    return BT::NodeStatus::FAILURE;
  }

  // getInput("friendly_outpost_gain_point", friendly_outpost_gain_point);

  if (
    (msg->is_fortress == true) ) {
    return BT::NodeStatus::SUCCESS;
  // } else if(msg->friendly_outpost_gain_point == false){
  }else{
    return BT::NodeStatus::FAILURE;
  }
}



// BT::PortsList IsFortressDetectedCondition::providedPorts()
// {
//   return {
//     BT::InputPort<bool>(
//       "friendly_outpost_gain_point", false, "己方前哨站增益点"),
//   };
// }



}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::IsFortressDetectedCondition>("IsFortressDetected");
}