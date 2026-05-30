# 高层运动指令（竞赛管理器→运动桥接节点）
# High-level motion command: Competition Manager -> Motion Bridge

# ===== 运动模式枚举 =====
uint8 MODE_STOP             = 0   # 停止
uint8 MODE_STAND_UP         = 1   # 起身站立
uint8 MODE_LIE_DOWN         = 2   # 趴下
uint8 MODE_WALK_FORWARD     = 3   # 前进
uint8 MODE_WALK_BACKWARD    = 4   # 后退
uint8 MODE_TURN_LEFT        = 5   # 原地左转
uint8 MODE_TURN_RIGHT       = 6   # 原地右转
uint8 MODE_STRAFE_LEFT      = 7   # 横移左
uint8 MODE_STRAFE_RIGHT     = 8   # 横移右
uint8 MODE_VELOCITY_CTRL    = 9   # 速度闭环控制（使用vel_x/y/yaw字段）
uint8 MODE_LOW_WALK         = 10  # 低姿走（限高杆下通过）
uint8 MODE_JUMP_DOWN        = 11  # 跳下动作（独木桥跳下）
uint8 MODE_TROT             = 12  # 小跑步态
uint8 MODE_BALANCE_STAND    = 13  # 平衡站立
uint8 MODE_RECOVERY_STAND   = 14  # 恢复站立（跌倒后）
uint8 MODE_HIT_FORWARD      = 15  # 向前撞击（击球/推倒可乐）
uint8 MODE_KICK_FORWARD     = 16  # 向前踢（踢足球）

std_msgs/Header header

uint8   mode            # 运动模式 (MODE_*)

# 速度控制参数（MODE_VELOCITY_CTRL 时使用）
float32 vel_x           # 前进速度 m/s [−1.5, 1.5]
float32 vel_y           # 侧移速度 m/s [−0.8, 0.8]
float32 vel_yaw         # 转向角速度 rad/s [−2.0, 2.0]
float32 body_height     # 机身高度 m（0=默认）
float32 step_height     # 抬脚高度 m（0=默认）

# 目标朝向（MODE_TURN 时使用）
float32 target_yaw_rad  # 目标偏航角（弧度，绝对值）

# 持续时间
float32 duration_s      # 执行持续时间（秒，0=保持直到下一条命令）

# 指令序列号（用于反馈匹配）
uint32  seq_id
