from .state import State
from .basic_state import *


class Hit_Ball_With_Shake(Basic_State):
    """
    撞击小球并验证晃动（2026赛题规则2.a要求）
    需要有明显晃动才算成功撞击
    """
    def __init__(self, duration=20):
        super().__init__()
        self.name = "Hit Ball With Shake Verification"
        self.duration = duration
        self.hit_threshold = 15
        self.shake_threshold = 5

    def execute(self):
        from ...camera.ball_scanner import ball_scanner_main
        from ...camera.path_scanner import check_camera_ready

        if not check_camera_ready(timeout=1.0):
            print(f"{self.name}: 摄像头不可用，无法寻找小球。跳过此步骤。")
            return

        print(f"Executing {self.name}")
        self._ros2_manager.init()
        sim_clock = self._ros2_manager.get_clock()

        try:
            _start_time = sim_clock.get_sim_time()
            wait_count = 0
            while _start_time is None:
                if wait_count % 5 == 0:
                    print(f"Waiting for /clock topic... (Attempt {wait_count})")
                time.sleep(1.0)
                _start_time = sim_clock.get_sim_time()
                wait_count += 1

            start_time = _start_time.nanosec / 1e9 + _start_time.sec
            hit_count = 0
            shake_detected = False
            no_vision_count = 0

            while True:
                _current_time = sim_clock.get_sim_time()
                current_time = _current_time.nanosec / 1e9 + _current_time.sec
                if current_time - start_time >= self.duration:
                    print("Hit ball duration reached.")
                    break

                try:
                    # 增加稳定性：使用 1.0s timeout，目标保持为默认（橙色）
                    data = ball_scanner_main(timeout=1.0)
                    if not data:
                        no_vision_count += 1
                        if no_vision_count < 3:
                            time.sleep(0.3)
                            continue
                        else:
                            print("Orange ball vision lost, spinning to search...")
                            self.locomotion.set_motion(11)
                            time.sleep(0.5)
                            no_vision_count = 0
                            continue
                    no_vision_count = 0

                    if data.get('target_detected'):
                        offset = data.get('center_offset', 0)
                        dist = data.get('distance', 100)

                        if dist < self.hit_threshold:
                            hit_count += 1
                            print(f"Orange ball hit detected! Count: {hit_count}")

                            self.locomotion.set_motion(5)
                            time.sleep(0.5)

                            prev_offset = offset
                            for _ in range(3):
                                shake_data = ball_scanner_main(timeout=0.4)
                                if shake_data and shake_data.get('target_detected'):
                                    new_offset = shake_data.get('center_offset', 0)
                                    if abs(new_offset - prev_offset) > self.shake_threshold:
                                        shake_detected = True
                                        print("Shake detected after hit!")
                                        break
                                    prev_offset = new_offset

                            if shake_detected:
                                self.locomotion.set_motion(6)
                                time.sleep(1.0)
                                break
                            else:
                                self.locomotion.set_motion(5)
                                time.sleep(0.3)

                        if offset > 15:
                            self.locomotion.set_motion(12)
                            time.sleep(0.15)
                        elif offset < -15:
                            self.locomotion.set_motion(11)
                            time.sleep(0.15)
                        else:
                            self.locomotion.set_motion(5)
                            time.sleep(0.1)
                    else:
                        self.locomotion.set_motion(11)
                        time.sleep(0.3)
                except Exception as e:
                    print(f"Ball detection error: {str(e)}")
                    self.locomotion.set_motion(5)
                    time.sleep(0.2)
        finally:
            self._ros2_manager.shutdown()


class Stage2_BeadHunt(State):
    """
    第二赛段：荒野寻珠
    修改版：确保撞击 4 个球后通过明确的导航序列前往出口
    """
    def __init__(self):
        super().__init__("Stage2_BeadHunt")
        self.basic_states = [
            # 1. 进入赛段稳定
            Standing(1),

            # 2. 任务：寻珠（4次循环）
            # 每轮循环：扫描 -> 撞击 -> 验证
            # 2026 规则要求：撞击橙色小球
            Spin_to_Find_Ball(15), Hit_Ball_With_Shake(25),
            Spin_to_Find_Ball(12), Hit_Ball_With_Shake(20),
            Spin_to_Find_Ball(12), Hit_Ball_With_Shake(20),
            Spin_to_Find_Ball(12), Hit_Ball_With_Shake(20),

            # 3. 衔接与转向：准备前往出口（出口在赛段左前方/左上角）
            Standing(1.5),
            # 执行一个确定性的大角度左转，脱离球阵，指向出口大致方向
            Spin_Left(4.5), 
            Standing(0.5),

            # 4. 冲刺与对齐：迈向出口
            # 利用 Walking_Forward_Robust 的自动寻迹/纠偏能力
            Walking_Forward_Robust(duration=10, motion_id=16), 
            
            # 5. 最后穿越：确认看到出口边界并清场
            Spin_To_Next_Stage(duration=5), 
            Walking_Forward_Robust(duration=6, motion_id=5),

            Standing(1)
        ]
