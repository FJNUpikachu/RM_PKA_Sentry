#include "rm_behavior_tree/plugins/condition/is_rfid_detected.hpp"

namespace rm_behavior_tree
{

IsRfidDetectedCondition::IsRfidDetectedCondition(const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsRfidDetectedCondition::checkRfidStatus, this), config)
{
}

BT::NodeStatus IsRfidDetectedCondition::checkRfidStatus()
{
  // bool friendly_supply_zone_non_exchange;
  auto msg = getInput<rm_interfaces::msg::RfidStatus>("message");
  if (!msg) {
    return BT::NodeStatus::FAILURE;
  }

  // getInput("friendly_supply_zone_non_exchange", friendly_supply_zone_non_exchange);

  if (
    (msg->friendly_supply_zone_non_exchange == true) ) {
    return BT::NodeStatus::SUCCESS;
  // } else if(msg->friendly_supply_zone_non_exchange == false){
  }else{
    return BT::NodeStatus::FAILURE;
  }
}



// BT::PortsList IsRfidDetectedCondition::providedPorts()
// {
//   return {
//     BT::InputPort<bool>(
//       "friendly_supply_zone_non_exchange", false, "己方与兑换区不重叠的补给区 / RMUL 补给区"),
//   };
// }



}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::IsRfidDetectedCondition>("IsRfidDetected");
}
