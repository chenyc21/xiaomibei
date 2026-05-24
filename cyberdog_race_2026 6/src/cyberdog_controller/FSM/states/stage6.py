from .state import State
from .basic_state import *


class Stage6_KickAndFinish(State):
    """
    第六赛段：撷金建功

    规则：
    - 从独木桥跳下后进入本赛段
    - 将足球从出口位置踢出
    - 来到终点位置，四条腿足底必须在圈内
    - 趴下，比赛结束
    - 摔倒或碰撞独木桥不扣分
    - 小球从独木桥上飞出后不允许放回

    策略：
    1. 落地稳定（从独木桥跳下后）
    2. 搜索足球（黑白配色）
    3. 接近足球并对准
    4. 前冲踢球，将球踢出出口
    5. 导航至终点圆圈
    6. 精确对位（四腿在圈内）
    7. 趴下结束
    """
    def __init__(self):
        super().__init__("Stage6_KickAndFinish")
        self.basic_states = [
            # 1. 落地后稳定站立
            Standing(2),

            # 2. 搜索足球（旋转扫描+视觉+LiDAR）
            Search_Football(duration=15),

            # 3. 稳定一下
            Standing(0.5),

            # 4. 接近足球（视觉引导对准+前进）
            Approach_Football(duration=20),

            # 5. 稳定准备踢球
            Standing(0.5),

            # 6. 全力前冲踢球
            Kick_Football(duration=4),

            # 7. 踢球后稳定
            Standing(1),

            # 8. 导航至终点（视觉搜索终点圆圈）
            Navigate_To_Finish(duration=25),

            # 9. 精确对位（确保四腿在圈内）
            Align_In_Circle(duration=12),

            # 10. 最终站稳
            Standing(1),

            # 11. 趴下，比赛结束
            Laying(3),
        ]
