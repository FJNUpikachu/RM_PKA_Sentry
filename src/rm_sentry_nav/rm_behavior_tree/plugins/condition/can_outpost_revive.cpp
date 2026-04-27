#include "rm_behavior_tree/plugins/condition/can_outpost_revive.hpp"

namespace rm_behavior_tree
{

// 构造函数中初始化：初始上一次血量设为-1（代表未获取），复活次数设为0
CanOutpostReviveAction::CanOutpostReviveAction(const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&CanOutpostReviveAction::checkCanRevive, this), config),
  last_outpost_hp_(-1), 
  recovery_count_(0)
{
}

BT::NodeStatus CanOutpostReviveAction::checkCanRevive()
{
  auto tower_msg = getInput<rm_interfaces::msg::TowerStatus>("tower_message");
  auto game_msg = getInput<rm_interfaces::msg::GameStatus>("game_message");

  if (!tower_msg || !game_msg) {
    return BT::NodeStatus::FAILURE;
  }

  // 获取当前的前哨站和基地血量
  int current_outpost_hp = tower_msg->outpost_hp;
  int current_base_hp = tower_msg->base_hp; // 请根据实际情况确认 base_hp 是否在 tower_msg 中

  // === 核心逻辑 1：检测前哨站复活事件 (0 -> 750) ===
  // 如果是第一次收到消息，先记录初始血量
  if (last_outpost_hp_ == -1) {
    last_outpost_hp_ = current_outpost_hp;
  } 
  else {
    // 边缘检测：只有当上一次血量为0，且当前血量恢复到750及以上时，判定为发生了一次复活
    if (last_outpost_hp_ == 0 && current_outpost_hp >= 750) {
      recovery_count_ += 1;
    }
    // 更新上一次的血量记录
    last_outpost_hp_ = current_outpost_hp;
  }

  // === 核心逻辑 2：赛程与 Flag 判断 ===
  if (game_msg->stage_remain_time == 0) {
    return BT::NodeStatus::FAILURE;
  }

  if (game_msg->stage_remain_time == 1) {
    int flag = 0;
    
    // 计算基地掉血累计的初始 flag (每掉1000血，flag+1)
    if (current_base_hp <= 5000) {
      flag += (5000 - current_base_hp) / 1000;
    }

    // 减去已经消耗掉的复活次数
    flag -= recovery_count_;

    // 只有当 flag 依然大于 0 时，才允许再次复活
    if (flag > 0) {
      return BT::NodeStatus::SUCCESS;
    } else {
      return BT::NodeStatus::FAILURE;
    }
  }

  return BT::NodeStatus::FAILURE;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::CanOutpostReviveAction>("CanOutpostRevive");
}