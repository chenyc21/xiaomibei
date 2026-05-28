from .state import State
from .basic_state import *


class Stage6_Final(State):
    """
    第六赛段：撷金建功 (基于绝对位置的导航版)
    
    赛道坐标系：
    - 足球位置：左上角（距左边50cm，距顶部50cm）
    - 终点位置：右下角（50cm × 50cm正方形区域）
    
    流程:
    1. 等待传感器就绪
    2. 搜索足球（扫描）
    3. 接近足球
    4. 快速冲撞足球（连续撞击踢飞）
    5. 原地转身180° + 前进（绝对位置导航）
    6. 利用LiDAR+黄色边沿定位终点区（进入50cm正方形区域）
    7. 趴下结算
    """

    def __init__(self):
        super().__init__("Stage6_Final")

        self.basic_states = [
            # 1. 等待传感器就绪（摄像头+LiDAR）
            Wait_For_Sensors(5.0),
            Standing(1.0),
            
            # 2. 搜索足球 - 原地旋转扫描，用摄像头确认黑白球
            Search_Football(35.0),
            Standing(1.0),
            
            # 3. 接近足球 - 视觉引导，慢速前进接近
            Approach_Football(30.0),
            Standing(1.0),
            
            # 4. 快速冲撞足球 - 全速撞击，可连续多次，改进的踢飞检测
            Rush_And_Kick_Football(12.0),
            Standing(2.0),
            
            # 5. 基于绝对位置导航返回终点 - 转身180°+前进
            Navigate_To_Finish_By_Position(18.0),
            Standing(1.0),
            
            # 6. 精确进入终点正方形区域 - 用LiDAR+黄色边沿校准居中
            Enter_Finish_Circle_Precise(15.0),
            Standing(1.0),
            
            # 7. 趴下结算
            Laying(5.0)
        ]
