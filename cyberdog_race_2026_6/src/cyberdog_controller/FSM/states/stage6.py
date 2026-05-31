from .state import State
from .basic_state import *
import time


def run_motion(locomotion, motion_id, duration, name="", dt=0.02):
    print(f"[Stage6] {name}: motion_id={motion_id}, duration={duration}")
    start = time.time()
    while time.time() - start < duration:
        locomotion.set_motion(motion_id)
        time.sleep(dt)
    locomotion.set_motion(1)
    time.sleep(0.25)


class Force_Close_To_Ball(Basic_State):
    """
    强制贴球：
    不用黄边保护，避免误判导致远距离停下。
    """

    def __init__(self, final_push_time=12.0):
        super().__init__()
        self.name = "Force Close To Ball"
        self.final_push_time = final_push_time

    def execute(self):
        print("Executing Force Close To Ball: 盲贴球")
        run_motion(self.locomotion, 1, 0.5, "Stand Before Close")
        run_motion(self.locomotion, 5, self.final_push_time, "Final Close Push")
        run_motion(self.locomotion, 1, 0.5, "Stand After Close")


class Rush_Ball(Basic_State):
    """
    角度踢球版：
    先轻微右转，让球不要卡在身体正中间；
    然后用高速中低身位短踢 motion_id=27 给球冲量。
    """

    def __init__(self, rush_times=6, rush_duration=1.1):
        super().__init__()
        self.name = "Angle Kick Ball"
        self.rush_times = rush_times
        self.rush_duration = rush_duration

    def execute(self):
        print("Executing Angle Kick Ball: 轻微右转后高速短踢")

        run_motion(self.locomotion, 1, 0.8, "Stand Before Angle Kick")

        # 关键：先轻微右转3秒，避免正面顶球被反弹
        # motion_id=10 是右转；如果方向反了改成 9
        run_motion(self.locomotion, 10, 3.0, "Slight Right Turn 3s Before Kick")

        run_motion(self.locomotion, 1, 0.5, "Stand After Slight Turn")

        # 再补一点距离，确保接触到球
        run_motion(self.locomotion, 5, 2.0, "Final Reach Ball After Turn")

        # 高速中低身位短踢，别长时间顶，避免跨球/卡球
        for i in range(self.rush_times):
            print(f"Angle Kick Ball: 第 {i + 1}/{self.rush_times} 次高速短踢")
            run_motion(self.locomotion, 27, self.rush_duration, "Fast Medium Low Kick")
            run_motion(self.locomotion, 1, 0.15, "Micro Stand")

        # 踢完后后退，给转身留空间
        run_motion(self.locomotion, 6, 3.0, "Back Away From Ball Before Return")
        run_motion(self.locomotion, 1, 1.0, "Stand After Kick")

        print("Angle Kick Ball: 完成")


class Force_Turn_Back_And_Return(Basic_State):
    """
    强制折返：
    右转58秒，回走50秒。
    """

    def __init__(self, turn_time=55.0, walk_time=50.0):
        super().__init__()
        self.name = "Force Turn Back And Return"
        self.turn_motion = 10
        self.turn_time = turn_time
        self.walk_time = walk_time

    def execute(self):
        print("Executing Force Turn Back And Return")

        run_motion(self.locomotion, 1, 2.0, "Stand Before Turn")
        run_motion(self.locomotion, self.turn_motion, self.turn_time, "Turn Back")
        run_motion(self.locomotion, 1, 1.0, "Stand After Turn")

        run_motion(self.locomotion, 5, self.walk_time, "Walk Back To Finish")

        run_motion(self.locomotion, 1, 1.5, "Final Standing")


class Stage6_Final(State):
    """
    第六赛段：
    找球 -> 靠近球 -> 盲贴球12秒 -> 角度高速踢球 -> 折返 -> 趴下
    """

    def __init__(self):
        super().__init__("Stage6_Final")

        self.basic_states = [
            Standing(2.0),

            Search_Football(duration=20),
            Standing(0.5),

            Approach_Football(duration=25),
            Standing(0.5),

            Force_Close_To_Ball(final_push_time=12.0),
            Standing(0.5),

            Rush_Ball(rush_times=6, rush_duration=1.1),
            Standing(0.8),

            Force_Turn_Back_And_Return(turn_time=55.0, walk_time=50.0),

            Standing(1.0),
            Laying(3.0),
        ]

    def execute(self):
        print("\nExecuting Stage6_Final: Close 12s -> Angle Kick -> Return")
        for state in self.basic_states:
            print(f"\n>>> Executing {state.name}")
            state.execute()
            print(f"<<< Finished {state.name}")
