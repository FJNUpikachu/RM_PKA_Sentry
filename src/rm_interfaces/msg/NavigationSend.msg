# 机器人开火控制命令
bool friction_wheel_scan # 是否开启摩擦轮 0->停止状态 1->开启状态
# 机器人控制命令（云台，底盘）
bool stop_gimbal_scan # 是否停止云台扫描 0->扫描状态 1->停止扫描
float32 chassis_spin_vel # 底盘小陀螺旋转速度 (rad/s)