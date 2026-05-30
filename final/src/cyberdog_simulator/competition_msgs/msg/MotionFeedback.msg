# 运动执行反馈（运动桥接节点→竞赛管理器）
# Motion execution feedback: Motion Bridge -> Competition Manager

# ===== 执行状态枚举 =====
uint8 EXEC_IDLE         = 0   # 空闲
uint8 EXEC_RUNNING      = 1   # 执行中
uint8 EXEC_COMPLETED    = 2   # 完成
uint8 EXEC_FAILED       = 3   # 失败（如跌倒）
uint8 EXEC_TIMEOUT      = 4   # 超时

std_msgs/Header header

uint32  seq_id          # 对应MotionCommand的seq_id
uint8   status          # 执行状态 (EXEC_*)
float32 progress        # 执行进度 [0.0, 1.0]
string  message         # 状态描述
