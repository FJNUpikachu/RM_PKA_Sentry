#ifndef RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_FORTRESS_DETECTED_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_FORTRESS_DETECTED_HPP_

#include <string>

#include "behaviortree_cpp/condition_node.h"
// #include "rm_interfaces/msg/game_status.hpp"
#include "rm_interfaces/msg/is_fortress.hpp"
// #include "rm_interfaces/msg/robot_status.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rm_behavior_tree
{
/**
 * @brief A BT::ConditionNode that get GameStatus from port and
 * returns SUCCESS when current game status and remain time is expected
 */
class IsFortressDetectedCondition : public BT::SimpleConditionNode
{
public:
  IsFortressDetectedCondition(const std::string & name, const BT::NodeConfig & config);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
    BT::NodeStatus checkFortressDetected();

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<rm_interfaces::msg::IsFortress>("message"),
      // BT::InputPort<bool>("friendly_outpost_gain_point")
      };
  }

// private:
//   /**
//    * @brief Tick function for game status ports
//    */
//   BT::NodeStatus checkFortressDetected();

//   rclcpp::Logger logger_ = rclcpp::get_logger("IsFortressDetectedCondition");

};
}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_FORTRESS_DETECTED_HPP_
// 判断是否识别到堡垒rfid