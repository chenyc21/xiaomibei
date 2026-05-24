from abc import ABC
import time
from ...locomotion import LocomotionController
import threading
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


class Shift_Left(Basic_State):
    def __init__(self, duration=0):
        super().__init__()
        self.name = "Shift Left"
        self.motion_id = 13
        self.duration = duration


class Shift_Right(Basic_State):
    def __init__(self, duration=0):
        super().__init__()
        self.name = "Shift Right"
        self.motion_id = 14
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


class Walking_Forward_Kick(Basic_State):
    """
    快速前冲踢球：使用较高速度前进撞击足球
    """
    def __init__(self, duration=3):
        super().__init__()
        self.name = "Walking Forward Kick"
        self.motion_id = 5
        self.duration = duration


class Search_Football(Basic_State):
    """
    搜索足球：旋转扫描寻找黑白足球
    结合LiDAR和视觉，检测到足球后停止
    """
    def __init__(self, duration=20):
        super().__init__()
        self.name = "Search Football"
        self.duration = duration

    def execute(self):
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

            found = False
            while True:
                _current_time = sim_clock.get_sim_time()
                current_time = _current_time.nanosec / 1e9 + _current_time.sec
                if current_time - start_time >= self.duration:
                    print(f"{self.name}: 超时未找到足球，继续前进")
                    break

                try:
                    data = football_scanner_main(timeout=1.0)
                    if data and data.get('football_detected'):
                        print(f"{self.name}: 检测到足球! offset={data.get('center_offset')}, dist={data.get('distance')}")
                        found = True
                        break
                    else:
                        self.locomotion.set_motion(9)
                        time.sleep(0.1)
                except Exception as e:
                    print(f"{self.name}: 检测异常: {str(e)}")
                    self.locomotion.set_motion(9)
                    time.sleep(0.1)

            if not found:
                print(f"{self.name}: 使用LiDAR辅助搜索...")
                try:
                    lidar_data = lidar_scanner_main(timeout=1.5)
                    if lidar_data.get('scan_available'):
                        front_dist = lidar_data.get('front_distance', float('inf'))
                        if front_dist < 1.5:
                            print(f"{self.name}: LiDAR检测到前方物体 dist={front_dist:.2f}m")
                except Exception:
                    pass

        finally:
            self._ros2_manager.shutdown()


class Approach_Football(Basic_State):
    """
    接近足球：视觉引导对准足球并前进靠近
    """
    def __init__(self, duration=25):
        super().__init__()
        self.name = "Approach Football"
        self.duration = duration
        self.align_threshold = 20
        self.kick_distance = 25

    def execute(self):
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

            no_detect_count = 0
            while True:
                _current_time = sim_clock.get_sim_time()
                current_time = _current_time.nanosec / 1e9 + _current_time.sec
                if current_time - start_time >= self.duration:
                    print(f"{self.name}: 超时")
                    break

                try:
                    data = football_scanner_main(timeout=1.0)
                    if data and data.get('football_detected'):
                        no_detect_count = 0
                        offset = data.get('center_offset', 0)
                        dist = data.get('distance', 100)

                        if dist < self.kick_distance:
                            print(f"{self.name}: 足球已就位，准备踢球! dist={dist}")
                            break

                        if offset > self.align_threshold:
                            self.locomotion.set_motion(12)
                        elif offset < -self.align_threshold:
                            self.locomotion.set_motion(11)
                        else:
                            self.locomotion.set_motion(5)
                        time.sleep(0.05)
                    else:
                        no_detect_count += 1
                        if no_detect_count > 10:
                            self.locomotion.set_motion(5)
                        else:
                            self.locomotion.set_motion(9)
                        time.sleep(0.1)
                except Exception as e:
                    print(f"{self.name}: 异常: {str(e)}")
                    self.locomotion.set_motion(5)
                    time.sleep(0.1)
        finally:
            self._ros2_manager.shutdown()


class Kick_Football(Basic_State):
    """
    踢球动作：快速前冲将足球踢出出口
    """
    def __init__(self, duration=5):
        super().__init__()
        self.name = "Kick Football"
        self.duration = duration

    def execute(self):
        print(f"Executing {self.name}: 全力前冲踢球!")
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

            while True:
                _current_time = sim_clock.get_sim_time()
                current_time = _current_time.nanosec / 1e9 + _current_time.sec
                if current_time - start_time >= self.duration:
                    break

                self.locomotion.set_motion(5)
                time.sleep(0.02)

                try:
                    data = football_scanner_main(timeout=0.3)
                    if data and data.get('football_detected'):
                        offset = data.get('center_offset', 0)
                        if offset > 25:
                            self.locomotion.set_motion(12)
                            time.sleep(0.05)
                        elif offset < -25:
                            self.locomotion.set_motion(11)
                            time.sleep(0.05)
                except Exception:
                    pass

            print(f"{self.name}: 踢球动作完成")
        finally:
            self._ros2_manager.shutdown()


class Navigate_To_Finish(Basic_State):
    """
    导航至终点：使用LiDAR和视觉找到终点圆圈并走过去
    终点在踢球出口方向的前方，直行为主
    """
    def __init__(self, duration=30):
        super().__init__()
        self.name = "Navigate To Finish"
        self.duration = duration
        self.steering_threshold = 10

    def execute(self):
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

            circle_found = False
            while True:
                _current_time = sim_clock.get_sim_time()
                current_time = _current_time.nanosec / 1e9 + _current_time.sec
                if current_time - start_time >= self.duration:
                    print(f"{self.name}: 导航超时，在当前位置停下")
                    break

                try:
                    data = finish_circle_scanner_main(timeout=0.8)
                    if data and data.get('circle_detected'):
                        circle_found = True
                        offset = data.get('center_offset', 0)
                        dist = data.get('distance', 100)

                        if dist < 30:
                            print(f"{self.name}: 到达终点圆圈上方!")
                            break

                        if offset > self.steering_threshold:
                            self.locomotion.set_motion(12)
                        elif offset < -self.steering_threshold:
                            self.locomotion.set_motion(11)
                        else:
                            self.locomotion.set_motion(16)
                        time.sleep(0.05)
                    else:
                        self.locomotion.set_motion(5)
                        time.sleep(0.05)
                except Exception as e:
                    print(f"{self.name}: 异常: {str(e)}")
                    self.locomotion.set_motion(5)
                    time.sleep(0.1)
        finally:
            self._ros2_manager.shutdown()


class Align_In_Circle(Basic_State):
    """
    精确对位：在终点圆圈内微调位置，确保四条腿都在圈内
    使用视觉反馈微调，最终停稳
    """
    def __init__(self, duration=15):
        super().__init__()
        self.name = "Align In Circle"
        self.duration = duration
        self.fine_threshold = 8

    def execute(self):
        print(f"Executing {self.name}: 精确对位中...")
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

            aligned_count = 0
            while True:
                _current_time = sim_clock.get_sim_time()
                current_time = _current_time.nanosec / 1e9 + _current_time.sec
                if current_time - start_time >= self.duration:
                    print(f"{self.name}: 对位超时，直接趴下")
                    break

                try:
                    data = finish_circle_scanner_main(timeout=0.8)
                    if data and data.get('circle_detected'):
                        offset = data.get('center_offset', 0)
                        dist = data.get('distance', 100)

                        if abs(offset) < self.fine_threshold and dist < 20:
                            aligned_count += 1
                            if aligned_count >= 5:
                                print(f"{self.name}: 对位完成! offset={offset}, dist={dist}")
                                break
                            self.locomotion.set_motion(1)
                        else:
                            aligned_count = 0
                            if dist >= 20:
                                self.locomotion.set_motion(16)
                            elif offset > self.fine_threshold:
                                self.locomotion.set_motion(14)
                            elif offset < -self.fine_threshold:
                                self.locomotion.set_motion(13)
                        time.sleep(0.05)
                    else:
                        self.locomotion.set_motion(1)
                        time.sleep(0.2)
                        aligned_count += 1
                        if aligned_count >= 10:
                            print(f"{self.name}: 未检测到圆圈，假定已在位")
                            break
                except Exception as e:
                    print(f"{self.name}: 异常: {str(e)}")
                    time.sleep(0.1)
        finally:
            self._ros2_manager.shutdown()


class Walking_Forward_LidarGuided(Basic_State):
    """
    LiDAR引导前进：避开独木桥和障碍物，安全前进
    """
    def __init__(self, duration=20):
        super().__init__()
        self.name = "Walking Forward LiDAR Guided"
        self.duration = duration

    def execute(self):
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

            while True:
                _current_time = sim_clock.get_sim_time()
                current_time = _current_time.nanosec / 1e9 + _current_time.sec
                if current_time - start_time >= self.duration:
                    break

                try:
                    lidar_data = lidar_scanner_main(timeout=1.0)
                    if lidar_data.get('scan_available'):
                        front_dist = lidar_data.get('front_distance', float('inf'))
                        left_dist = lidar_data.get('left_distance', float('inf'))
                        right_dist = lidar_data.get('right_distance', float('inf'))

                        if front_dist < 0.3:
                            if left_dist > right_dist:
                                self.locomotion.set_motion(13)
                            else:
                                self.locomotion.set_motion(14)
                            time.sleep(0.2)
                        else:
                            self.locomotion.set_motion(5)
                            time.sleep(0.05)
                    else:
                        self.locomotion.set_motion(5)
                        time.sleep(0.05)
                except Exception:
                    self.locomotion.set_motion(5)
                    time.sleep(0.1)
        finally:
            self._ros2_manager.shutdown()
