from .states import *


class FSM:
    def __init__(self):
        self.states = [
            Stage5_BridgeCrossing(),
        ]
        self.current_state = None
        self.current_state_index = 0

    def execute(self):
        print("=" * 60)
        print("FSM Starting... Stage 5: 孤梁稳渡")
        print("=" * 60)

        from ..utils.ros2_manager import unpause_physics
        unpause_physics(timeout=3.0)

        from ..camera.path_scanner import check_camera_ready
        camera_ok = check_camera_ready(timeout=3.0)
        if not camera_ok:
            print("=" * 60)
            print("警告: 摄像头未就绪！第五赛段将使用保守时间 fallback。")
            print("=" * 60)

        while self.current_state_index < len(self.states):
            self.current_state = self.states[self.current_state_index]
            print(f"\n>>> Transitioning to State: {self.current_state.name}")
            self.current_state.execute()
            print(f"<<< Finished State: {self.current_state.name}")
            self.current_state_index += 1

        print("\nFSM Completed. 第五赛段完成。")


def main():
    fsm = FSM()
    fsm.execute()
