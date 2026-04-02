# 从下位机接收的串口数据 —— 哨兵（Sentry）协议（32字节包）
#
# 帧头(1) + mode(1) + roll(4) + pitch(4) + yaw(4)
# + chassis_imu_yaw_offset(4) + game_progress(2)
# + friendly_supply_zone_non_exchange(1) + current_hp(2)
# + reserved(7) + 校验(1) + 帧尾(1) = 32字节

std_msgs/Header header
uint8   mode
float32 roll
float32 pitch
float32 yaw
float32 chassis_imu_yaw_offset              # 底盘正方向与 IMU 初始上电位置的偏航角偏差（弧度）
uint16  game_progress                        # 当前比赛阶段
uint8   friendly_supply_zone_non_exchange    # 己方与兑换区不重叠的补给区/RMUL 补给区
uint16  current_hp                           # 血量
