from .state import State
from .basic_state import *
from ...camera.path_scanner import path_scanner_main
from ...camera.arrow_scanner import arrow_scanner_main

class From_Section_A_To_S_Route(State):
    def __init__(self):
        super().__init__("From_Section_A_To_S_Route")
        self._check_method = None
        self.basic_states = []

    def build_states(self, check_method):
        """动态构建状态链"""
        self.basic_states = [
            # Spin_by_line(6),
            # Standing(1),
            Walking_Forward_by_distance_z(7,0),
            Standing(1),
            Walking_Forward_Slow(2.5),
            Standing(1),
            Spin_Right(5.12),
            Standing(2),
            # Shift_Left_by_bottom(8, 69),
            # Spin_by_line(6),
            # Standing(1),
            Walking_Forward_by_rightbound(10,80),
            # Walking_Forward(2.5),
            Standing(3),
            # Shift_Left_by_bottom(8, 60),
            # Standing(5),
            # Shift_Right_by_bottom(8, 70),
            # Standing(5),
        ]

    @property
    def check_method(self):
        """动态获取检测方法"""
        if self._check_method is None:
            self._check_method = self._init_check_method()
        return self._check_method

    def _init_check_method(self):
        """初始化检测方法并构建状态链"""
        # 获取检测方法（可复用已有实例）
        method = path_scanner_main
        
        # 构建状态链
        self.build_states(method)
        return method
    
    def execute(self):
        print(f"Executing {self.name}")
        _ = self.check_method
        super().execute()
        print(f"Finished executing {self.name}")

class From_S_Route_To_Section_A(State):
    def __init__(self):
        super().__init__("From_S_Route_To_Section_A")
        self.basic_states = [
            # Walking_Forward(1),
            Standing(1),
            Spin_by_line(6),
            Standing(1),
            Spin_Left(5.9),
            Standing(2),
            Walking_Forward(4),
            Spin_by_line(6),
            Standing(3),
        ]
        

class S_Route(State):
    def __init__(self):
        super().__init__("S_Route")
        self.basic_states = []

    def build_states(self):
        """动态构建状态链"""
        self.basic_states = [
            Walking_Left_Turn(8),
            # Standing(1),
            Walking_Forward_Slow(0.03),
            # Walking_Forward_by_bottom(10,12),
            # Walking_Forward(0.1),
            # Standing(3),
            Walking_Right_Turn(9.96),
            # Standing(1),
            # Walking_Forward_by_bottom(10,12),
            # Walking_Forward(0.1),
            # Standing(1),
            Walking_Left_Turn(8.18),
            # Standing(1),
            # Walking_Forward(0.01),
            Walking_Right_Turn(4.02),
            Standing(1),
            # Spin_Left(1.85),
            # # Standing(5),
            # # Spin_by_line(6),
            # Standing(1),
            # Walking_Forward(2),
            # Standing(1),
        ]   


    
    def execute(self):
        print(f"Executing {self.name}")
        self.build_states()
        super().execute()
        print(f"Finished executing {self.name}")


class Reverse_S_Route(State):
    def __init__(self):
        super().__init__("Reverse_S_Route")
        self.basic_states = [
            Walking_Left_Turn(4),
            # Standing(3),
            # Walking_Forward_by_bottom(10,90),
            # Walking_Forward(0.1),
            # Standing(3),
            Walking_Right_Turn(8.1),
            # Walking_Forward_Slow(0.15),
            # Standing(1),
            Walking_Left_Turn(9.8),
            Walking_Forward_Slow(0.2),
            # Standing(3),
            Walking_Right_Turn(8),

            Walking_Forward(1)
            
        ]


class From_S_Route_To_Choose_Route(State):
    def __init__(self):
        super().__init__("From_S_Route_To_Choose_Route")
        self.arrow_result = None
        
    def execute(self):
        print(f"Executing {self.name}")
        super().execute()
        self.arrwo_result = arrow_scanner_main()
        print(f"Finished executing {self.name}")


class From_Choose_Route_To_S_Route(State):
    def __init__(self):
        super().__init__("From_Choose_Route_To_S_Route")


class Choose_Route(State):
    def __init__(self):
        super().__init__("Choose_Route")

