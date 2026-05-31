from abc import ABC
import time
from ...locomotion import LocomotionController
from ...utils.ros2_manager import ros2_manager
from ...camera.path_scanner import *


class Basic_State(ABC):
    def __init__(self):
        self.name = "Basic_State"
        self.motion_id = 0
        self.duration = -1
        self.locomotion = LocomotionController()
        self._ros2_manager = ros2_manager

    def execute(self):
        self._ros2_manager.init()
        sim_clock = self._ros2_manager.get_clock()

        try:
            print(f"Executing {self.name} with motion ID {self.motion_id} for duration {self.duration}")
            _start_time = sim_clock.get_sim_time()
            wait_count = 0
            while _start_time is None:
                if wait_count % 5 == 0:
                    print(f"Waiting for /clock topic... (Attempt {wait_count})")
                time.sleep(1.0)
                _start_time = sim_clock.get_sim_time()
                wait_count += 1

            start_time = _start_time.nanosec / 1e9 + _start_time.sec
            current_time = start_time

            while True:
                self.locomotion.set_motion(self.motion_id)
                time.sleep(0.01)
                _current_time = sim_clock.get_sim_time()
                if _current_time:
                    current_time = _current_time.nanosec / 1e9 + _current_time.sec
                if current_time - start_time >= self.duration:
                    break

            print(f"Finished executing {self.name} in {current_time - start_time:.2f} seconds")
        finally:
            self._ros2_manager.shutdown()


class Standing(Basic_State):
    def __init__(self, duration=0):
        super().__init__()
        self.name = "Standing"
        self.motion_id = 1
        self.duration = duration


class Laying(Basic_State):
    def __init__(self, duration=0):
        super().__init__()
        self.name = "Laying"
        self.motion_id = 3
        self.duration = duration


class Walking_Forward(Basic_State):
    def __init__(self, duration=0):
        super().__init__()
        self.name = "Walking Forward"
        self.motion_id = 5
        self.duration = duration


class Walking_Forward_Slow(Basic_State):
    def __init__(self, duration=0):
        super().__init__()
        self.name = "Walking Forward Slow"
        self.motion_id = 16
        self.duration = duration


class Walking_Backward(Basic_State):
    def __init__(self, duration=0):
        super().__init__()
        self.name = "Walking Backward"
        self.motion_id = 6
        self.duration = duration


class Spin_Left(Basic_State):
    def __init__(self, duration=0):
        super().__init__()
        self.name = "Spin Left"
        self.motion_id = 9
        self.duration = duration


class Spin_Right(Basic_State):
    def __init__(self, duration=0):
        super().__init__()
        self.name = "Spin Right"
        self.motion_id = 10
        self.duration = duration


class Shift_Right(Basic_State):
    def __init__(self, duration=0):
        super().__init__()
        self.name = "Shift Right"
        self.motion_id = 14
        self.duration = duration


class Shift_Left(Basic_State):
    def __init__(self, duration=0):
        super().__init__()
        self.name = "Shift Left"
        self.motion_id = 13
        self.duration = duration


class Wait_For_Sensors(Basic_State):
    def __init__(self, duration=5):
        super().__init__()
        self.name = "Wait For Sensors"
        self.duration = duration

    def execute(self):
        print(f"Executing {self.name}: 等待传感器就绪...")
        self._ros2_manager.init()
        sim_clock = self._ros2_manager.get_clock()

        try:
            _start_time = sim_clock.get_sim_time()
            while _start_time is None:
                time.sleep(1.0)
                _start_time = sim_clock.get_sim_time()
            start_time = _start_time.nanosec / 1e9 + _start_time.sec

            camera_ready = False
            lidar_ready = False

            while True:
                _current_time = sim_clock.get_sim_time()
                current_time = _current_time.nanosec / 1e9 + _current_time.sec
                if current_time - start_time >= self.duration:
                    break

                self.locomotion.set_motion(1)

                if not camera_ready:
                    if check_camera_ready(timeout=1.0):
                        camera_ready = True
                        print("  摄像头就绪")

                if not lidar_ready:
                    lidar_data = lidar_scanner_main(timeout=1.0)
                    if lidar_data.get('scan_available'):
                        lidar_ready = True
                        print("  LiDAR就绪")

                if camera_ready and lidar_ready:
                    print("  所有传感器就绪!")
                    break

                time.sleep(0.5)

        finally:
            self._ros2_manager.shutdown()


class Search_Football(Basic_State):
    """
    搜索足球：
    - 原地左转搜索
    - 摄像头连续确认 3 次才算锁定
    """

    def __init__(self, duration=25):
        super().__init__()
        self.name = "Search Football"
        self.duration = duration

    def execute(self):
        print(f"Executing {self.name}: 旋转搜索足球（视觉为主）...")
        self._ros2_manager.init()
        sim_clock = self._ros2_manager.get_clock()

        try:
            _start_time = sim_clock.get_sim_time()
            while _start_time is None:
                time.sleep(1.0)
                _start_time = sim_clock.get_sim_time()
            start_time = _start_time.nanosec / 1e9 + _start_time.sec

            found = False
            confirm_count = 0

            while True:
                _current_time = sim_clock.get_sim_time()
                current_time = _current_time.nanosec / 1e9 + _current_time.sec

                if current_time - start_time >= self.duration:
                    print(f"{self.name}: 搜索超时")
                    break

                # 原地左转一小段
                self.locomotion.set_motion(9)
                time.sleep(0.25)

                # 停稳拍照
                self.locomotion.set_motion(1)
                time.sleep(0.30)

                try:
                    cam_data = football_scanner_main(timeout=0.8)
                    if cam_data and cam_data.get('football_detected'):
                        confirm_count += 1
                        offset = cam_data.get('center_offset', 0)
                        dist_px = cam_data.get('distance', 0)
                        area = cam_data.get('area', 0)
                        print(
                            f"{self.name}: 视觉检测到足球! "
                            f"offset={offset}, dist_px={dist_px}, area={area}, "
                            f"确认={confirm_count}/3"
                        )
                        if confirm_count >= 3:
                            print(f"{self.name}: 足球确认锁定!")
                            found = True
                            break
                    else:
                        confirm_count = 0

                except Exception as e:
                    print(f"{self.name}: 异常: {str(e)}")

            if found:
                self.locomotion.set_motion(1)
                time.sleep(0.4)

        finally:
            self._ros2_manager.shutdown()


class Approach_Football(Basic_State):
    """
    接近足球：
    - 先对正
    - 再前进
    - 不再使用 area>2000 直接停下的老逻辑
    - 只有“更近 + 更居中 + 连续确认”才结束
    """

    def __init__(self, duration=18):
        super().__init__()
        self.name = "Approach Football"
        self.duration = duration

    def execute(self):
        print(f"Executing {self.name}: 视觉引导接近足球...")
        self._ros2_manager.init()
        sim_clock = self._ros2_manager.get_clock()

        try:
            _start_time = sim_clock.get_sim_time()
            while _start_time is None:
                time.sleep(1.0)
                _start_time = sim_clock.get_sim_time()
            start_time = _start_time.nanosec / 1e9 + _start_time.sec

            no_ball_count = 0
            close_confirm = 0

            while True:
                _current_time = sim_clock.get_sim_time()
                current_time = _current_time.nanosec / 1e9 + _current_time.sec

                if current_time - start_time >= self.duration:
                    print(f"{self.name}: 超时")
                    break

                try:
                    cam_data = football_scanner_main(timeout=0.6)

                    if cam_data and cam_data.get('football_detected'):
                        no_ball_count = 0
                        offset = int(cam_data.get('center_offset', 0))
                        dist_px = cam_data.get('distance', 0)
                        area = int(cam_data.get('area', 0))

                        print(f"{self.name}: offset={offset}, dist_px={dist_px}, area={area}, close={close_confirm}/3")

                        # 更严格的结束条件：既靠近又居中，还要连着确认几次
                        if abs(offset) < 12 and area > 1800:
                            close_confirm += 1
                            self.locomotion.set_motion(1)
                            time.sleep(0.20)
                            if close_confirm >= 3:
                                print(f"{self.name}: 已到贴球准备位✓ area={area}")
                                break
                            continue
                        else:
                            close_confirm = 0

                        # 没对正先调头
                        if offset > 15:
                            self.locomotion.set_motion(12)
                            time.sleep(0.08)
                        elif offset < -15:
                            self.locomotion.set_motion(11)
                            time.sleep(0.08)
                        else:
                            # 对正后用正常前进，小步靠近
                            self.locomotion.set_motion(5)
                            time.sleep(0.10)

                    else:
                        no_ball_count += 1
                        print(f"{self.name}: 暂时丢球 {no_ball_count}")

                        if no_ball_count > 8:
                            print(f"{self.name}: 丢球较久，原地搜索")
                            self.locomotion.set_motion(11)
                            time.sleep(0.18)
                            no_ball_count = 0
                        else:
                            self.locomotion.set_motion(1)
                            time.sleep(0.08)

                except Exception as e:
                    print(f"{self.name}: 异常: {str(e)}")
                    time.sleep(0.15)

            self.locomotion.set_motion(1)
            time.sleep(0.3)

        finally:
            self._ros2_manager.shutdown()
