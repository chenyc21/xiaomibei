from .states import *

class FSM:
    def __init__(self):
        self.states = [
            Stage6_Final(),  # 注意这里改成了我们写好的盲走类
        ]
        self.current_state = None
        self.current_state_index = 0

    def execute(self):
        print("=" * 60)
        print("FSM Starting... Stage 6: 撷金建功 (盲走模式)")
        print("=" * 60)

        while self.current_state_index < len(self.states):
            self.current_state = self.states[self.current_state_index]
            print(f"\n>>> Transitioning to State: {self.current_state.name}")
            self.current_state.execute()
            print(f"<<< Finished State: {self.current_state.name}")
            self.current_state_index += 1

        print("\nFSM Completed. 第六赛段完成，比赛结束！")

def main():
    fsm = FSM()
    fsm.execute()