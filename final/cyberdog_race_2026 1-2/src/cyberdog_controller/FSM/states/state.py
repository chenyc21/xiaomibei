from abc import ABC
from .basic_state import *

class State(ABC):
    def __init__(self, name):
        self.name = name
        self.basic_states = [Standing(0)]

    def execute(self):
        print(f"Executing {self.name}")
        for state in self.basic_states:
            state.execute()
        print(f"Finished executing {self.name}")

    
class Inital(State):
    def __init__(self):
        super().__init__("Inital")
        self.name = "Inital"
        self.basic_states = [Standing(3)]

class DebugStand(State):
    def __init__(self):
        super().__init__("Standing")
        self.name = "Standing"
        self.basic_states = [Standing(1000000)]

class Rotation(State):
    def __init__(self):
        super().__init__("Rotation")
        self.name = "Rotation"
        self.basic_states = [Standing(0), Spin_Right(100000)]


class Finish(State):
    def __init__(self):
        super().__init__("Inital")
        self.name = "Inital"
        self.basic_states = [Laying(4)]
