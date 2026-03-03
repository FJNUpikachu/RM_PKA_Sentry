#!/bin/bash
set -e  # 任何命令执行失败时立即退出脚本，避免无效执行
# 2. 加载ROS 2 Humble环境（检查环境是否存在）
echo "=== 加载ROS 2 Humble环境 ==="
ROS2_SETUP_FILE="/opt/ros/humble/setup.bash"
if [ ! -f "$ROS2_SETUP_FILE" ]; then
    echo "错误：未找到ROS 2 Humble环境文件 $ROS2_SETUP_FILE"
    echo "请确认已安装ROS 2 Humble，或修改脚本中的ROS 2版本路径"
    exit 1
fi
source "$ROS2_SETUP_FILE"

# 3. 验证ros2命令是否可用（核心检查）
if ! command -v ros2 &> /dev/null; then
    echo "错误：ROS 2环境加载失败，ros2命令未找到！"
    exit 1
fi

# 5. 加载本地编译的环境
echo "=== 加载本地编译环境 ==="
source install/setup.bash

# 6. 启动launch文件（如果启动失败也会退出）
echo "=== 启动rm_bringup.launch.py ==="

ros2 launch pb2025_nav_bringup rm_sentry_reality_launch.py \
world:=test_new \
slam:=True \
use_composition:=False \
mapping:=False \
use_robot_state_pub:=True \
use_respawn:=True \
style:=RMUL3
