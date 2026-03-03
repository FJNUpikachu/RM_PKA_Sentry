#!/bin/bash
set -e  # 任何命令执行失败时立即退出脚本，避免无效执行

source install/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml