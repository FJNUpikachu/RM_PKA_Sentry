# 发送给下位机的串口数据 —— 哨兵（Sentry）协议（32字节包）
#
# 帧头(1) + fire_advice(1) + pitch(4) + yaw(4) + distance(4)
# + linear_x(4) + linear_y(4) + angular_z(4)
# + reserved(4) + 校验(1) + 帧尾(1) = 32字节

bool     fire_advice    # 开火指令
float32  pitch          # 云台 pitch 角度（弧度）
float32  yaw            # 云台 yaw   角度（弧度）
float32  distance       # 目标距离（米）
float32  linear_x       # 底盘线速度 x（m/s）
float32  linear_y       # 底盘线速度 y（m/s）
float32  angular_z      # 底盘角速度 z（rad/s）
