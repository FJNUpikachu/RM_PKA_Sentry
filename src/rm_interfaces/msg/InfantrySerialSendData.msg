# 发送给下位机的串口数据 —— 步兵（Infantry）协议（16字节包）
#
# 帧头(1) + fire_advice(1) + pitch(4) + yaw(4) + distance(4)
# + 校验(1) + 帧尾(1) = 16字节

bool     fire_advice    # 开火指令
float32  pitch          # 云台 pitch 角度（弧度）
float32  yaw            # 云台 yaw   角度（弧度）
float32  distance       # 目标距离（米）
