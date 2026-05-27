from .state import State
from .basic_state import *


class Stage6_Final(State):
    """
    第六赛段：撷金建功 (纯时间序列盲走版)
    """

    def __init__(self):
        super().__init__("Stage6_Final")

        self.basic_states = [
            Standing(2.0),

            # 走到足球所在的位置
            Walking_Forward(2.0),
            Standing(0.5),
            Shift_Right(1.5),
            Standing(0.5),

            # 将足球推出去
            Walking_Forward_Slow(3.5),
            Standing(1.0),

            # 走向蓝色终点圈
            Spin_Right(1.5),
            Standing(0.5),
            Walking_Forward(4.0),
            Standing(1.0),

            # 趴下结算
            Laying(5.0)
        ]