from .state import State
from .basic_state import *
import time
from ...camera.path_scanner import path_scanner_main

class From_Charging_To_Section_A(State):
    def __init__(self):
        super().__init__("From_Charging_To_Section_A")
         # 初始化扫描器
        self.basic_states = []
        
    def build_states(self):
        """动态构建状态链"""
        self.basic_states = [
            # Spin_by_line(6),
            # Spin_Right(0.1),
            Walking_Forward_by_px(8,48),
            Standing(1),
            Spin_Right(5.82),
            Standing(1),
            Walking_Forward(3),
            Standing(1),
            # Spin_by_line(6),
            # Standing(1),
        ]
        
    
    def execute(self):
        print(f"Executing {self.name}")
        self.build_states()
        super().execute()
        print(f"Finished executing {self.name}")

class From_Section_A_To_Charging(State):
    def __init__(self):
        super().__init__("From_Section_A_To_Charging")
        self.basic_states = [
            Walking_Forward(2.5),
            Standing(1),
            Spin_Left(5.85),
            Standing(1),   
            Walking_Forward(3.5),            
        ]
        