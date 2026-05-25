from .state import State
from .basic_state import *

class Stage5_BridgeCrossing(State):
    """
    第五赛段：孤梁稳渡

    半闭环策略：
    1. 站稳，默认第四赛段已将机身对准独木桥入口；
    2. 桥上初段低速居中；
    3. 稳定慢速前进，看到虚线后停止；
    4. 连续确认虚线，未确认则使用保守时间 fallback；
    5. 继续补偿一小段，近似保证四足都越过虚线；
    6. 短距离下桥，落地站稳。
    """

    def __init__(self):
        super().__init__("Stage5_BridgeCrossing")
        self.basic_states = [
            # 仿真启动后底层控制器通常处于 Off/Passive，先恢复站立再发走路指令。
            Recovery_Stand(5.0),

            Standing(1.0),

            # 上桥入口/台阶段：持续高抬腿向前；卡住时继续爬，不后退重试。
            Bridge_Entry_Climb(duration=9.5, strong_duration=5.0),

            # BridgeAlign: 桥上初段低速居中，视觉失效时保守慢走。
            Bridge_Center_Forward(duration=4.0, stop_on_line=False),

            # BridgeTraverse: 稳定慢走，识别到虚线后不再继续冲下桥。
            Bridge_Center_Forward(duration=11.0, stop_on_line=True),

            # BridgeLineConfirm: 连续确认虚线；检测不到则走保守 fallback。
            Bridge_Line_Confirm(max_duration=4.0, required_frames=3),

            # 四足越过虚线补偿段，防止机身过线但后足未过线。
            Walking_Forward_Slow(2.2),

            Standing(0.4),

            # BridgeDismount: 低风险短距离下桥，避免激烈跳跃撞桥。
            Bridge_Dismount(duration=1.4),

            # 落地后稳定
            Standing(2.0),
        ]
