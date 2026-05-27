from .state import State
from .basic_state import *


class Stage6_Final(State):
    """
    第六赛段：撷金建功 (视觉+传感器引导版)
    
    流程:
    1. 等待传感器就绪
    2. 搜索足球（扫描）
    3. 接近足球
    4. 快速冲撞足球（可连续撞击）
    5. 扫描回到终点（固定位置）
    6. 进入终点圆（50cm直径）
    7. 趴下结算
    """

    def __init__(self):
        super().__init__("Stage6_Final")

        self.basic_states = [
            # 1. 等待传感器就绪（摄像头+LiDAR）
            Wait_For_Sensors(5.0),
            Standing(1.0),
            
            # 2. 搜索足球 - 原地旋转扫描，用摄像头确认黑白球
            Search_Football(30.0),
            Standing(1.0),
            
            # 3. 接近足球 - 视觉引导，慢速前进接近
            Approach_Football(25.0),
            Standing(1.0),
            
            # 4. 快速冲撞足球 - 加速冲撞，可能需要多次撞击
            Rush_And_Kick_Football(8.0),
            Standing(2.0),
            
            # 5. 扫描并返回终点 - LiDAR/摄像头找终点标记，走回去
            Scan_And_Return_To_Finish(20.0),
            Standing(1.0),
            
            # 6. 精确进入终点圆 - 调整位置，确保四只脚在50cm圆内
            Precise_Enter_Finish_Circle(10.0),
            Standing(1.0),
            
            # 7. 趴下结算
            Laying(5.0)
        ]
