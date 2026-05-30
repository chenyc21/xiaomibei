from .state import State
from .basic_state import *


class Stage1_StonePath(State):
    """
    第一赛段：石径探路
    规则：起点趴下，站起计时，后腿离开弯道虚线结束。

    策略（2026 最终鲁棒版）：
    1. 站起稳定
    2. LiDAR + 视觉双重感知穿越石板路，增加鲁棒的出口防抖检测
    3. 到达出口后，执行反馈驱动的 90 度左转（看到第二赛段入口即停）
    4. 进入第二赛段并对齐中线
    """
    def __init__(self):
        super().__init__("Stage1_StonePath")
        self.basic_states = [
            # 1. 站起
            Standing_Up(3),

            # 2. 跨越石板路（带帧级防抖出口拦截）
            # 增加自适应：如果提前看到出口，会自动缩短补位
            Walking_Forward_LidarEnhanced(duration=150, motion_id=16),

            # 3. 拦截成功后站立稳定
            Standing(1.0),

            # 4. 空间预留：微量后退，防止转弯时后腿/侧边扫到黄色边界线
            # 增加后退时长（0.8s -> 1.5s），确保转弯半径完全在赛道内
            Walking_Backward(1.5),
            Standing(0.5),

            # 5. 关键衔接：视觉反馈旋转（转向第二赛段入口）
            Spin_To_Next_Stage(duration=12), 
            
            # 6. 精确对齐对微调
            Standing(0.5),
            Spin_by_line(4), 

            # 7. 直行冲刺，完全跨过第一赛段边界并对齐
            # 缩短时长（8s -> 5s），防止冲出第二赛段安全区
            Walking_Forward_Robust(duration=5, motion_id=5),

            # 8. 稳定并准备寻珠
            Standing(1)
        ]
