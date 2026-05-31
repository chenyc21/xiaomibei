from .state import State
from .basic_state import *

class Stage5_BridgeCrossing(State):
    """
    第五赛段：孤梁稳渡

    赛题要求：
    1. 全程为连续独木桥，设备需要在独木桥上行走；
    2. 穿过独木桥上的虚线后可以跳下；
    3. 必须四个足底都越过虚线后才可以跳下；
    4. 跳下时不允许身体撞击独木桥。
    """

    def __init__(self):
        super().__init__("Stage5_BridgeCrossing")
        self.basic_states = [
            # 仿真启动后底层控制器通常处于 Off/Passive，先恢复站立再发走路指令。
            Recovery_Stand(5.0),

            Standing(1.0),

            # 第一段入口动作临时跳过：直接从桥上开始调试后续独木桥。
            # Walking_Forward_Climb(duration=5.0),
            # Bridge_Mid_Jump(duration=1.8),
            # Recovery_Stand(2.5),
            # Standing(0.5),
            # Walking_Forward(duration=21.5),
            # Bridge_Turn_Left(duration=7.3),
            # Standing(0.3),

            # 第一段 400cm 倾斜独木桥。
            Slope_Bridge_Forward(duration=51.5),

            Bridge_Turn_Right(duration=10.3),
            Standing(0.3),

            # 第二段 400cm 倾斜独木桥。
            #Slope_Bridge_Forward(duration=49),

            #Bridge_Turn_Right(duration=10.1),
            #Standing(0.3),

            # 第三段倾斜独木桥。
            Slope_Bridge_Forward(duration=45.5),

            Bridge_Turn_Right(duration=10.3),
            Standing(0.3),

            # 第四段 400cm 倾斜独木桥。
            Slope_Bridge_Forward(duration=51.5),

            Bridge_Turn_Right(duration=13.3),
            Standing(0.3),

            # 第五段倾斜独木桥；走完后右转，面向跳下方向。
            Walking_Forward(duration=8.5),
            Bridge_Turn_Right(duration=8.3),
            Standing(0.6),

            # 使用 Jump3D 下台阶动作跳下，避免用普通行走蹭到桥体。
            Bridge_Jump_Down(duration=2.0),

            # 落地后稳定
            Recovery_Stand(3.0),
            Standing(1.0),
        ]
