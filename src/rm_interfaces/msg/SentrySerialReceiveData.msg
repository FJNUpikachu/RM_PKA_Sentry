std_msgs/Header header
uint8   mode
float32 roll
float32 pitch
float32 yaw
float32 chassis_imu_yaw_offset   # 底盘正方向与 IMU 初始上电位置的偏航角偏差（弧度）
uint16  game_progress            # 当前比赛阶段
uint16  current_hp               # 当前血量
uint16  bullet_rest              # 允许发弹量
uint8   is_battling              # 是否脱战
uint8   is_fortress              # 是否刷到堡垒RFID
uint8   is_supply                # 是否刷到基地补给区RFID
uint16  outpost_hp               # 己方前哨站血量