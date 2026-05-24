from .state import State
from .basic_state import *
from ...camera import qr_scanner_main
from ...camera.path_scanner import path_scanner_main

class Section_A(State):
    def __init__(self):
        super().__init__("Section_A")
        self.qr_result = None
        
    def execute(self):
        print(f"Executing {self.name}")
        super().execute()
        self.qr_result = qr_scanner_main()
        print(f"Finished executing {self.name}")



class From_Section_A_To_Section_A1(State):
    def __init__(self):
        super().__init__("From_Section_A_To_Section_A1")
        self.basic_states = [Walking_Forward(4), 
                             Standing(1), 
                             Spin_Left(5.85),
                             Standing(1),
                             Walking_Forward(4),
                             Standing(1), 
                             Spin_Left(5.65),
                             Standing(0.1),
                             Spin_by_line(6),
                             Standing(1),
                             Walking_Forward(4.5),
                             Standing(1)]

class From_Section_A_To_Section_A2(State):
    def __init__(self):
        super().__init__("From_Section_A_To_Section_A2")
        self.basic_states = []

    def build_states(self):
        """动态构建状态链"""
        self.basic_states = [
                            Walking_Forward(4),
                            Standing(1), 
                            Spin_Right(5.85),
                            Standing(1),
                            Spin_by_line(6),
                            Standing(1),
                            Walking_Forward(4.4),
                            Standing(1), 
                            Spin_Right(5.86),
                            Standing(1),
                            Spin_by_line(6),
                            Standing(1),
                            Walking_Forward(4),
                            Standing(1),
                            Spin_by_line(5),
                            Standing(1),]


    def execute(self):
        print(f"Executing {self.name}")
        if not self.basic_states:
            self.build_states()
        super().execute()
        print(f"Finished executing {self.name}")


class Section_A_Rotation(State):
    def __init__(self):
        super().__init__("Section_A_Rotation")
        self.basic_states = [Laying(5), Standing(4)]


class From_Section_A1_To_Section_A(State):
    def __init__(self):
        super().__init__("From_Section_A1_To_Section_A")
        self.basic_states = [Walking_Backward(4),
                             Standing(1), 
                             Shift_Left(3),
                             Standing(1),
                             Walking_Forward(5),
                             Standing(1)]


class From_Section_A2_To_Section_A(State):
    def __init__(self):
        super().__init__("From_Section_A2_To_Section_A")
        self.basic_states = [Walking_Backward(4.1),
                            #  Standing(1), 
                             Shift_Right(3),
                             Standing(1),
                             Shift_by_angle(5),
                            #  Shift_Left(0.2),
                             Walking_Forward(5),
                            #  Standing(1),
                            #  Spin_by_line(6),
                            #  Standing(1),
                             Standing(1)]