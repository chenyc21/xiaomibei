# 检测到的单个目标物体
# Single detected object message

# ===== 目标类型枚举 =====
uint8 TYPE_UNKNOWN          = 0
uint8 TYPE_ORANGE_BALL      = 1   # 橙色小球（赛段二、四）
uint8 TYPE_BLUE_BALL        = 2   # 浅蓝色小球（非目标）
uint8 TYPE_SOCCER           = 3   # 足球（赛段四、六）
uint8 TYPE_COLA_BOTTLE      = 4   # 大桶可乐（赛段四）
uint8 TYPE_HEIGHT_BAR       = 5   # 限高杆（赛段四）
uint8 TYPE_BLOCK_OBSTACLE   = 6   # 无法跨越障碍（赛段四）
uint8 TYPE_GOAL_NET         = 7   # 球门（赛段四）
uint8 TYPE_BALANCE_BEAM     = 8   # 独木桥（赛段四→五）
uint8 TYPE_STONE_PLATE      = 9   # 石板（赛段一）
uint8 TYPE_BOUNDARY_LINE    = 10  # 赛道边沿黄线
uint8 TYPE_DASHED_LINE      = 11  # 虚线（赛段分隔线）
uint8 TYPE_FINISH_CIRCLE    = 12  # 终点圆圈（赛段六）

# ===== 目标状态枚举 =====
uint8 STATUS_VISIBLE        = 0   # 可见未交互
uint8 STATUS_TARGETED       = 1   # 已锁定（正在接近）
uint8 STATUS_INTERACTED     = 2   # 已完成交互
uint8 STATUS_MISSED         = 3   # 丢失跟踪

# ===== 字段 =====
std_msgs/Header header

uint8   object_type     # 目标类型 (TYPE_*)
uint8   object_status   # 目标状态 (STATUS_*)
uint32  object_id       # 目标唯一ID（同类型内）

# 图像坐标（像素）
float32 bbox_x          # 边框中心 x
float32 bbox_y          # 边框中心 y
float32 bbox_w          # 边框宽度
float32 bbox_h          # 边框高度
float32 confidence      # 检测置信度 [0.0, 1.0]

# 相对机器人的估计位置（机体坐标系，米）
float32 rel_x           # 前方距离
float32 rel_y           # 侧向偏移（左正右负）
float32 rel_z           # 高度
float32 distance        # 直线距离

# 目标物体附加属性
bool    is_target       # 是否为当前赛段目标
bool    is_obstacle     # 是否为障碍物
