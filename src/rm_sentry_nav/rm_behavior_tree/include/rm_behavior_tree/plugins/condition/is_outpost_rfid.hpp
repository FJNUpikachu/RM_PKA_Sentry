#ifndef RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_OUTPOST_RFID_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_OUTPOST_RFID_HPP_

#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "rm_interfaces/msg/game_status.hpp"
#include "rm_interfaces/msg/rfid_status.hpp"
#include "rm_interfaces/msg/robot_status.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rm_behavior_tree
{
/**
 * @brief A BT::ConditionNode that get GameStatus from port and
 * returns SUCCESS when current game status and remain time is expected
 */
class IsOutpostRfidCondition : public BT::SimpleConditionNode
{
public:
  IsOutpostRfidCondition(const std::string & name, const BT::NodeConfig & config);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
    BT::NodeStatus checkRfidStatus();

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<rm_interfaces::msg::RfidStatus>("message"),
      // BT::InputPort<bool>("friendly_outpost_gain_point")
      };
  }

// private:
//   /**
//    * @brief Tick function for game status ports
//    */
//   BT::NodeStatus checkRfidStatus();

//   rclcpp::Logger logger_ = rclcpp::get_logger("IsOutpostRfidCondition");

};
}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_OUTPOST_RFID_HPP_