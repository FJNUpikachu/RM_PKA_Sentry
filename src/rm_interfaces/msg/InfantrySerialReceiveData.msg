# 从下位机接收的串口数据 —— 步兵（Infantry）协议（16字节包）
#
# 帧头(1) + mode(1) + roll(4) + pitch(4) + yaw(4)
# + 校验(1) + 帧尾(1) = 16字节

std_msgs/Header header
uint8   mode
float32 roll
float32 pitch
float32 yaw
