# 机器人当前状态（聚合自LCM状态估计器）
# Robot current state, aggregated from LCM state_estimator_lcmt

std_msgs/Header header

# ===== 位姿 =====
float32[3] position         # 世界坐标系位置 [x, y, z]（米）
float32[3] velocity_world   # 世界坐标系速度 [vx, vy, vz]（m/s）
float32[3] velocity_body    # 机体坐标系速度
float32[3] rpy              # 欧拉角 [roll, pitch, yaw]（弧度）
float32[4] quaternion       # 四元数 [w, x, y, z]
float32[3] omega_body       # 角速度（机体系）

# ===== 足端接触 =====
float32[4] contact_estimate # 四足接触估计 [FL, FR, RL, RR] [0,1]

# ===== 运动状态 =====
int32  fsm_mode             # 底层FSM当前模式（对应FsmStateName枚举）
int32  gait_id              # 当前步态ID
bool   is_fallen            # 是否跌倒
bool   is_recovering        # 是否在自恢复中

# ===== 系统状态 =====
float32 battery_percent     # 电量百分比
bool    motor_error         # 电机是否有错误
int64   timestamp_us        # 时间戳（微秒）
