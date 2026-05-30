# 竞赛全局状态
# Competition global state message

# ===== 竞赛阶段枚举 =====
uint8 STAGE_IDLE        = 0   # 待机
uint8 STAGE_1           = 1   # 石径探路
uint8 STAGE_2           = 2   # 荒野寻珠
uint8 STAGE_3           = 3   # 曲道冲锋
uint8 STAGE_4           = 4   # 深隧寻珍
uint8 STAGE_5           = 5   # 孤梁稳渡
uint8 STAGE_6           = 6   # 撷金建功
uint8 STAGE_FINISHED    = 7   # 比赛结束
uint8 STAGE_ERROR       = 8   # 错误/恢复中

# ===== 赛段子状态枚举 =====
uint8 SUB_INIT          = 0
uint8 SUB_RUNNING       = 1
uint8 SUB_INTERACTING   = 2
uint8 SUB_AVOIDING      = 3
uint8 SUB_EXITING       = 4
uint8 SUB_RECOVERY      = 5

# ===== 字段 =====
std_msgs/Header header

uint8  stage              # 当前赛段 (STAGE_*)
uint8  sub_state          # 赛段内子状态 (SUB_*)
uint8  tasks_completed    # 当前赛段已完成任务数
uint8  tasks_total        # 当前赛段总任务数

float32 elapsed_time_s    # 比赛累计耗时（秒）
float32 stage_time_s      # 当前赛段耗时（秒）

int32  score              # 当前得分（估算）
int32  penalty            # 当前扣分（估算）

string status_message     # 人类可读状态描述
