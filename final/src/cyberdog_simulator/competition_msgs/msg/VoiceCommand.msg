# 语音播报指令（竞赛管理器→语音节点）
# Voice TTS command

# ===== 预定义播报内容（赛规要求） =====
uint8 ANNOUNCE_NONE             = 0
uint8 ANNOUNCE_COLA             = 1   # "识别到可乐瓶"
uint8 ANNOUNCE_ORANGE_BALL      = 2   # "识别到橙色小球"
uint8 ANNOUNCE_SOCCER           = 3   # "识别到足球"
uint8 ANNOUNCE_HEIGHT_BAR       = 4   # "识别到限高杆"
uint8 ANNOUNCE_IMPASSABLE       = 5   # "识别到无法跨越障碍"
uint8 ANNOUNCE_CUSTOM           = 99  # 自定义文本（使用text字段）

std_msgs/Header header

uint8   announce_type   # 播报类型（ANNOUNCE_*）
string  text            # 自定义播报文本（announce_type=ANNOUNCE_CUSTOM时有效）
float32 volume          # 音量 [0.0, 1.0]，默认1.0
uint32  seq_id
