#!/usr/bin/env bash
set -euo pipefail

# Fixed-state publisher for RMUC_attack.xml
# 用途：周期发布固定状态的 ROS2 话题消息，用于行为树测试
# 用法:
#   ./test_tree_attack.sh [rate_hz]
#   RFID_OUTPOST=1 IS_BATTLING=0 ./test_tree_attack.sh 20    # 进攻模式，20 Hz
#   RFID_OUTPOST=0 IS_BATTLING=1 ./test_tree_attack.sh 10    # 防御模式，10 Hz
#   RFID_OUTPOST=0 IS_BATTLING=0 ./test_tree_attack.sh 5     # 移动模式，5 Hz

# 发布频率（Hz），默认 20 Hz（即每秒 20 次），与之前的 0.05s 间隔等价
RATE_HZ=${1:-20}

# 游戏状态消息参数
GAME_PROGRESS=${GAME_PROGRESS:-4}           # 游戏进度（4=比赛中）

# 机器人状态消息参数
CURRENT_HP=${CURRENT_HP:-400}       # 当前血量
IS_BATTLING=${IS_BATTLING:-0}       # 是否在战斗（0=否, 1=是，控制防御姿态）

# RFID 状态消息参数（控制姿态切换）
RFID_SUPPLY=${RFID_SUPPLY:-1}       # 补给区RFID（暂未使用）
RFID_OUTPOST=${RFID_OUTPOST:-0}     # 堡垒RFID（1=检测到，触发进攻姿态）

TOWER_OUTPOST_HP=${TOWER_OUTPOST_HP:-70} # 前哨战血量

BULLET_REST=${BULLET_REST:-60} # 允许发弹量

echo "game_progress=${GAME_PROGRESS}, hp=${CURRENT_HP}, battling=${IS_BATTLING}, rfid_supply=${RFID_SUPPLY}, rfid_outpost=${RFID_OUTPOST}, outpost_hp=${TOWER_OUTPOST_HP}"
echo "Publishing all topics in parallel at ${RATE_HZ} Hz"
echo "姿态映射: RFID_OUTPOST=1 -> Attack(2), IS_BATTLING=1 -> Defend(1), else -> Run(3)"

trap 'echo "Stopping..."; kill 0; exit 0' SIGINT SIGTERM

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 command not found. Source your workspace (install/setup.bash) and retry." >&2
  exit 2
fi

# 使用 -r 选项在每个后台进程中独立以固定频率发布
ros2 topic pub -r "${RATE_HZ}" /game_progress rm_interfaces/msg/GameProgress \
  "{game_progress: ${GAME_PROGRESS}}" &
PID1=$!

ros2 topic pub -r "${RATE_HZ}" /current_hp rm_interfaces/msg/CurrentHP \
  "{current_hp: ${CURRENT_HP}}" &
PID2=$!

# 发布补给检测结果，类型为 IsSupply，topic 名与 BT XML 中的 topic_name 保持一致
ros2 topic pub -r "${RATE_HZ}" /is_supply rm_interfaces/msg/IsSupply \
  "{is_supply: ${RFID_SUPPLY}}" &
PID3=$!

ros2 topic pub -r "${RATE_HZ}" /bullet_rest rm_interfaces/msg/BulletRest \
  "{bullet_rest: ${BULLET_REST}}" &
PID4=$!

# 独立话题发布是否处于战斗状态，避免与 BulletRest 混用同一 topic
ros2 topic pub -r "${RATE_HZ}" /is_battling rm_interfaces/msg/IsBattling \
  "{is_battling: ${IS_BATTLING}}" &
PID5=$!

# 独立话题发布堡垒 RFID 检测结果
ros2 topic pub -r "${RATE_HZ}" /is_fortress rm_interfaces/msg/IsFortress \
  "{is_fortress: ${RFID_OUTPOST}}" &
PID6=$!

ros2 topic pub -r "${RATE_HZ}" /outpost_hp rm_interfaces/msg/OutpostHP \
  "{outpost_hp: ${TOWER_OUTPOST_HP}}" &
PID7=$!

# 等待任意一个后台进程结束（或者被信号打断）
wait

# sudo apt install ros-humble-generate-parameter-library
