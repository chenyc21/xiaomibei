# 赛道边界信息
# Track boundary information from boundary detector

std_msgs/Header header

# ===== 边界检测结果 =====
bool   left_boundary_detected   # 左侧黄线是否可见
bool   right_boundary_detected  # 右侧黄线是否可见

float32 left_offset_px     # 左边界相对图像中心的像素偏移（正=在左）
float32 right_offset_px    # 右边界相对图像中心的像素偏移（正=在右）
float32 center_offset_px   # 机器人偏离赛道中心的像素偏移（正=偏右）
float32 center_offset_m    # 机器人偏离赛道中心的估计距离（米）

float32 heading_error_rad  # 机器人朝向偏离赛道方向的角度（弧度）

# ===== 前方道路状态 =====
bool   turn_detected        # 前方是否有弯道
float32 turn_angle_deg      # 估计弯道角度（度，正=左弯，负=右弯）
float32 straight_distance_m # 前方可行直线距离估计（米）

# ===== 虚线检测 =====
bool   dashed_line_detected    # 是否检测到赛段分界虚线
float32 dashed_line_distance_m # 到虚线的距离（米）

# ===== 石板检测（赛段一专用）=====
bool   stone_plate_visible  # 是否有石板可见
float32 stone_plate_offset_px # 石板中心偏移像素
