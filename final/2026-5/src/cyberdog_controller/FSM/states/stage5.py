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

            # 第一段：从第五赛段入口起步，完成中段跳跃并接入连续独木桥。
            # 起点 (3.12, 7.35) → 桥中段虚线 (3.12, ~11.95)，全程沿 +Y 直行。
            # 中段跳跃前先用原始 high-step 步态爬上桥。
            # 跳跃落地、Recovery_Stand 后做一次性对位：先小步幅横移把 x 拉回 3.12，
            # 再小步幅旋转对准 +Y，然后开环正常前进，避免闭环干扰。
            # 注：Jump3D→Recovery 转换条件已在 fsm_state_jump_3d.cpp 放宽为只判 data_end_。
            Walking_Forward_Slow(duration=5.0),
            Bridge_Mid_Jump(duration=2.0),
            Standing(0.4),
            Recovery_Stand(4.0),
            Standing(1.0),
            Stage5_Align_To_3p12(target_x=3.10),
            Standing(0.5),
            Walking_Forward(duration=16.5),
            Bridge_Turn_Left(duration=8.0),
            Standing(0.3),

            # 第一段 400cm 倾斜独木桥。
            Slope_Bridge_Forward(duration=50.0),

            Bridge_Turn_Right(duration=9.1),
            Standing(0.3),

            # 第二段 400cm 倾斜独木桥。
            Slope_Bridge_Forward(duration=51.5),

            Bridge_Turn_Right(duration=9.1),
            Standing(0.3),

            # 第三段倾斜独木桥。
            Slope_Bridge_Forward(duration=38.0),

            Bridge_Turn_Right(duration=9.1),
            Standing(0.3),

            # 第四段 400cm 倾斜独木桥。
            Slope_Bridge_Forward(duration=51.5),

            Bridge_Turn_Right(duration=9.1),
            Standing(0.3),

            # 第五段倾斜独木桥；走完后右转，面向跳下方向。
            Slope_Bridge_Forward(duration=25.0),

            Bridge_Turn_Right(duration=9.1),
            Standing(0.6),

            # 使用 Jump3D 下台阶动作跳下，避免用普通行走蹭到桥体。
            Bridge_Jump_Down(duration=2.0),

            # 落地后稳定
            Recovery_Stand(3.0),
            Standing(1.0),
        ]
