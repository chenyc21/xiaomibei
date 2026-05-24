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


class Finish(State):
    def __init__(self):
        super().__init__("Finish")
        self.basic_states = [Laying(4)]
