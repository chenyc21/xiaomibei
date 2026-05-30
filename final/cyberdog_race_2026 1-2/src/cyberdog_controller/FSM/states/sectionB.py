from .state import State
from .basic_state import *
from camera import qr_scanner_main

class Section_B_Scan(State):
    def __init__(self):
        super().__init__("Section_B_scan")
        self.qr_result = None
        
    def execute(self):
        print(f"Executing {self.name}")
        super().execute()
        self.qr_result = qr_scanner_main()
        print(f"Finished executing {self.name}")

class Section_B(State):
    def __init__(self):
        super().__init__("Section_B")
        self.basic_states = [
            Standing(1)
        ]


class Section_B_Rotation(State):
    def __init__(self):
        super().__init__("Section_B_Rotation")
        self.basic_states = [
            Laying(6), 
            Standing(3),
            
        ]


class From_Section_B_To_Section_B1(State):
    def __init__(self):
        super().__init__("From_Section_B_To_Section_B1")
        self.basic_states = [
            Shift_Left(3.3),
            Standing(1),
            Walking_Forward(3.5),
            Standing(1),
        ]


class From_Section_B_To_Section_B2(State):
    def __init__(self):
        super().__init__("From_Section_B_To_Section_B2")
        self.basic_states = [
            Shift_Right(3.3),
            Standing(1),
            Spin_by_line(6),
            Standing(1),
            Walking_Forward(3.5),
            Standing(1),
        ]


class From_Section_B1_To_Section_B(State):
    def __init__(self):
        super().__init__("From_Section_B1_To_Section_B")
        self.basic_states = [
            Walking_Backward_by_distance(5,30),
            Standing(1),
            Spin_by_line(6),
            Standing(1),
            Shift_Right(3.3),
            Standing(1),
        ]


class From_Section_B2_To_Section_B(State):
    def __init__(self):
        super().__init__("From_Section_B2_To_Section_B")
        self.basic_states = [
            Walking_Backward_by_distance(5,30),
            Standing(1),
            Spin_by_line(6),
            Standing(1),
            Shift_Left(3.3),
            Standing(1),
        ]

