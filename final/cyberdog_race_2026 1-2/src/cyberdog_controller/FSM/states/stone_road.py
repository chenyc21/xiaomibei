from .state import State
from .basic_state import *

class From_S_To_Stone(State):
    def __init__(self):
        super().__init__("From_S_To_Stone")
        self.basic_states = [
            # Walking_Forward_by_distance_z(7, 0),
            Walking_Forward(2.5),
            Standing(1),
            Spin_Left(4.5),
            Walking_Forward_Slow(0.2),
            Spin_by_line(6),
            Standing(1),
            Walking_Forward_by_distance_z(7, 0),
            Walking_Forward_Slow(1),
            Standing(1),
            Spin_Right(4.7),
        ]


class From_Stone_To_S(State):
    def __init__(self):
        super().__init__("From_Stone_To_S")
        self.basic_states = [
            # Standing(1),
            Walking_Stone_by_distance(50, 30),
            Walking_Forward_Slow(1),
            Standing(1),
            Shift_Left(2),
            Standing(1),
            # Shift_by_angle(3),
            # Shift_Left(0.5),
            # Standing(1),
            # Walking_Forward_by_distance(4, 58),
            Walking_Forward(1.75),
            Standing(1),
            Spin_Right(1.2)
        ]


class Stone(State):
    def __init__(self):
        super().__init__("Stone")
        self.basic_states = [
            Shift_by_angle(5),
            # Standing(1),
            Walking_Forward(27),
            Standing(1),
        ]


class Between_Limit_Height_and_Stone(State):
    def __init__(self):
        super().__init__("Between_Limit_Height_and_Stone")
        self.basic_states = [
            Shift_by_angle(5),
            Walking_by_black_bar(10),
        ]


class Limit_Height(State):
    def __init__(self):
        super().__init__("Limit_Height")
        self.basic_states = [
            Walking_High_Limiter(10),
            Standing(1),
        ]


class From_Limit_Height_To_Section_B(State):
    def __init__(self):
        super().__init__("From_Limit_Height_To_Section_B")


class From_Section_B_To_Limit_Height(State):
    def __init__(self):
        super().__init__("From_Section_B_To_Limit_Height")
        self.basic_states = [
            Spin_Left(12.6), 
            Standing(1),
            Shift_Right(3.3), 
            Standing(1),
            Shift_by_angle(5),
            Standing(1),
            Walking_Forward(6),
            Standing(1),
            # # Spin_by_black_bar(5),
            # # Standing(1),
            # # Shift_by_angle(5),
            # Standing(1),
            Walking_Forward(2),
            Standing(1),
        ]
