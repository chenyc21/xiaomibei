from .state import State
from .basic_state import *

class From_S_To_Slope(State):
    def __init__(self):
        super().__init__("From_S_To_Slope")
        self.basic_states = [
            # # Shift_by_angle(5),
            # # Standing(1),
            # Shift_Right(3.9), 
            # Standing(1),
            # # Walking_Forward(0.5),
            # Walking_Forward_by_distance_z(7, 0),
            Walking_Forward(2.5),
            Standing(1),
            Spin_Right(4.5),
            Walking_Forward_Slow(1),
            Spin_by_line(6),
            Standing(1),
            Walking_Forward_by_distance_z(7, 0),
            Walking_Forward_Slow(1),
            Standing(1),
            Spin_Left(5),
            Walking_Forward_Slow(1),
        ]


class From_Slope_To_S(State):
    def __init__(self):
        super().__init__("From_Slope_To_S")


class Slope(State):
    def __init__(self):
        super().__init__("Slope")
        self.basic_states = [
            Standing(1),
            Shift_by_slope(8),
            Standing(1),
            # Spin_by_slope(8),
            Walking_Slope_by_distance(60,20),
            Walking_Slope(7),
            Standing(1), 
            # Shift_Left(0.1),
            Shift_by_angle(1),
            # Walking_Forward(1),
            # Standing(1),
            # Spin_by_angle(6),
            Walking_Slope(22),
            Standing(1),
            Walking_Forward(1),
            Shift_by_angle(5),
            Standing(1),
        ]


class Between_Yellow_Light_and_Slope(State):
    def __init__(self):
        super().__init__("Between_Yellow_Light_and_Slope")
        self.basic_states = []


class Yellow_Light(State):
    def __init__(self):
        super().__init__("Yellow_Light")
        self.yellow_light = None
        self.basic_states = [
            Standing(5),
            Walking_Forward(10),
        ]

    def execute(self):
        print(f"Executing {self.name}")
        # self.yellow_light = yellow_light_main()
        super().execute()
        print(f"Finished executing {self.name}")


class From_Yellow_Light_To_Section_B(State):
    def __init__(self):
        super().__init__("From_Yellow_Light_To_Section_B")
        self.basic_states = [
            Walking_Forward(10),
            Standing(1),
            # Shift_by_angle(5),
            # Walking_Forward_by_distance(5, 45),
            # Walking_Forward(0.5),
            Standing(1),
            Walking_Backward_by_distance(5, 30),
            Standing(1),
            Spin_by_line(6),
            # Walking_Forward(1),
            Standing(1),
            Shift_Left(3.28),
            Standing(1),
        ]


class From_Section_B_To_Yellow_Light(State):
    def __init__(self):
        super().__init__("From_Section_B_To_Yellow_Light")
