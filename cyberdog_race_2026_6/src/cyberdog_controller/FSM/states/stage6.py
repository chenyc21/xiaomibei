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
    不用黄边保护，避免黄边误判导致远距离停下。
    先让 Approach_Football 完成视觉靠近，再盲贴 12 秒。
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
    角度高速踢球版：
    先用 12s 盲走贴近球；
    然后轻微右转 3s；
    再前进 3s 重新贴球；
    最后用 motion_id=27 高速中低身位踢球。
    """

    def __init__(self, rush_times=6, rush_duration=2.0):
        super().__init__()
        self.name = "Angle Fast Kick Ball"
        self.rush_times = rush_times
        self.rush_duration = rush_duration

    def execute(self):
        print("Executing Angle Fast Kick Ball: 右转角度 + 高速踢球")

        run_motion(self.locomotion, 1, 0.8, "Stand Before Kick")

        # 轻微右转 3 秒，改变接触角度，避免正面被球顶回来
        # 如果方向反了，把 10 改成 9
        run_motion(self.locomotion, 10, 3.0, "Slight Right Turn 3s")

        run_motion(self.locomotion, 1, 0.4, "Stand After Slight Turn")

        # 右转后再向前贴球，避免转完离球更远
        run_motion(self.locomotion, 5, 3.0, "Reach Ball After Turn")

        # 高速中低身位踢球
        for i in range(self.rush_times):
            print(f"Angle Fast Kick: 第 {i + 1}/{self.rush_times} 次")
            run_motion(self.locomotion, 27, self.rush_duration, "Fast Kick Forward")
            run_motion(self.locomotion, 1, 0.15, "Micro Stand")

        # 后退脱离球，避免折返转身被球卡住
        run_motion(self.locomotion, 6, 3.0, "Back Away From Ball")
        run_motion(self.locomotion, 1, 1.0, "Stand After Kick")

        print("Angle Fast Kick Ball: 完成")


class Force_Turn_Back_And_Return(Basic_State):
    """
    强制折返：
    右转 55 秒，回走 50 秒。
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
    找球 -> 靠近球 -> 盲贴球 12 秒 -> 角度高速踢球 -> 折返 -> 趴下。
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

            Rush_Ball(rush_times=6, rush_duration=2.0),
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
