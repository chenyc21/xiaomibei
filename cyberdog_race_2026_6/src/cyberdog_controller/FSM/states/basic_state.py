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


class Wait_For_Sensors(Basic_State):
    """等待传感器就绪"""
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
                        print(f"  摄像头就绪")

                if not lidar_ready:
                    lidar_data = lidar_scanner_main(timeout=1.0)
                    if lidar_data.get('scan_available'):
                        lidar_ready = True
                        print(f"  LiDAR就绪")

                if camera_ready and lidar_ready:
                    print(f"  所有传感器就绪!")
                    break
                time.sleep(0.5)
        finally:
            self._ros2_manager.shutdown()


class Search_Football(Basic_State):
    """
    搜索足球：原地慢速旋转，用摄像头识别黑白球
    摄像头确认是球才算找到（不仅仅靠LiDAR）
    """
    def __init__(self, duration=30):
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

                # 原地左转（motion_id=9，原地旋转）
                self.locomotion.set_motion(9)
                time.sleep(0.3)
                # 停下让画面稳定再拍照
                self.locomotion.set_motion(1)
                time.sleep(0.5)

                try:
                    # 用摄像头找黑白球
                    cam_data = football_scanner_main(timeout=1.0)
                    if cam_data and cam_data.get('football_detected'):
                        confirm_count += 1
                        offset = cam_data.get('center_offset', 0)
                        dist_px = cam_data.get('distance', 0)
                        print(f"{self.name}: 视觉检测到足球! offset={offset}, dist_px={dist_px}, 确认={confirm_count}/3")
                        if confirm_count >= 3:
                            print(f"{self.name}: 足球确认锁定!")
                            found = True
                            break
                    else:
                        # 没看到球，不重置计数（可能只是转过头了）
                        pass

                except Exception as e:
                    print(f"{self.name}: 异常: {str(e)}")

            if found:
                self.locomotion.set_motion(1)
                time.sleep(0.5)
        finally:
            self._ros2_manager.shutdown()


class Approach_Football(Basic_State):
    """
    接近足球：视觉对准 + 慢速前进
    用摄像头持续跟踪球的位置，对准后前进
    """
    def __init__(self, duration=25):
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

            while True:
                _current_time = sim_clock.get_sim_time()
                current_time = _current_time.nanosec / 1e9 + _current_time.sec
                if current_time - start_time >= self.duration:
                    print(f"{self.name}: 超时")
                    break

                try:
                    cam_data = football_scanner_main(timeout=0.8)

                    if cam_data and cam_data.get('football_detected'):
                        no_ball_count = 0
                        offset = cam_data.get('center_offset', 0)
                        dist_px = cam_data.get('distance', 0)
                        area = cam_data.get('area', 0)

                        # 球非常近了（面积大于2000），准备绕位
                        if area > 2000:
                            print(f"{self.name}: 球已进入冲撞范围! area={area}")
                            break

                        # 对准球：微调方向
                        if offset > 15:
                            self.locomotion.set_motion(12)  # 微右转
                            time.sleep(0.1)
                        elif offset < -15:
                            self.locomotion.set_motion(11)  # 微左转
                            time.sleep(0.1)
                        else:
                            # 对准了，慢速前进
                            self.locomotion.set_motion(16)
                            time.sleep(0.15)
                    else:
                        no_ball_count += 1
                        if no_ball_count > 15:
                            # 丢球太久，原地旋转找回
                            print(f"{self.name}: 丢失球，旋转搜索...")
                            self.locomotion.set_motion(11)
                            time.sleep(0.3)
                            no_ball_count = 0
                        else:
                            # 短暂丢失，继续慢速前进
                            self.locomotion.set_motion(16)
                            time.sleep(0.1)

                except Exception as e:
                    print(f"{self.name}: 异常: {str(e)}")
                    time.sleep(0.2)
        finally:
            self._ros2_manager.shutdown()


class Position_Behind_Ball(Basic_State):
    """
    绕到球的左上方：先左移，再前进一点
    目的是让机器人处于球和出口（右下）之间
    这样推球时球就会朝出口方向走
    """
    def __init__(self, duration=8):
        super().__init__()
        self.name = "Position Behind Ball"
        self.duration = duration

    def execute(self):
        print(f"Executing {self.name}: 绕到球的左上方...")
        self._ros2_manager.init()
        sim_clock = self._ros2_manager.get_clock()

        try:
            _start_time = sim_clock.get_sim_time()
            while _start_time is None:
                time.sleep(1.0)
                _start_time = sim_clock.get_sim_time()
            start_time = _start_time.nanosec / 1e9 + _start_time.sec

            # 第一阶段：左移2秒（绕过球）
            phase_start = start_time
            while True:
                _current_time = sim_clock.get_sim_time()
                current_time = _current_time.nanosec / 1e9 + _current_time.sec
                if current_time - phase_start >= 2.5:
                    break
                self.locomotion.set_motion(13)  # 左平移
                time.sleep(0.05)

            # 第二阶段：前进2秒（走到球的后方）
            phase_start2 = current_time
            while True:
                _current_time = sim_clock.get_sim_time()
                current_time = _current_time.nanosec / 1e9 + _current_time.sec
                if current_time - phase_start2 >= 2.0:
                    break
                self.locomotion.set_motion(16)  # 慢速前进
                time.sleep(0.05)

            # 站稳
            self.locomotion.set_motion(1)
            time.sleep(1.0)
            print(f"{self.name}: 已绕到球后方")
        finally:
            self._ros2_manager.shutdown()


class Position_For_Push(Basic_State):
    """
    定位推球：绕到球的左上方，然后转向右下（出口方向）
    用视觉反馈确认球的位置
    """
    def __init__(self, duration=10):
        super().__init__()
        self.name = "Position For Push"
        self.duration = duration

    def execute(self):
        print(f"Executing {self.name}: 绕位+转向出口方向...")
        self._ros2_manager.init()
        sim_clock = self._ros2_manager.get_clock()

        try:
            _start_time = sim_clock.get_sim_time()
            while _start_time is None:
                time.sleep(1.0)
                _start_time = sim_clock.get_sim_time()
            start_time = _start_time.nanosec / 1e9 + _start_time.sec

            # 第一阶段：左移绕过球（2秒）
            print(f"  阶段1: 左移绕过球")
            phase_start = start_time
            while True:
                _current_time = sim_clock.get_sim_time()
                current_time = _current_time.nanosec / 1e9 + _current_time.sec
                if current_time - phase_start >= 2.0:
                    break
                self.locomotion.set_motion(13)
                time.sleep(0.05)

            # 第二阶段：前进超过球（1.5秒）
            print(f"  阶段2: 前进超过球")
            phase_start = current_time
            while True:
                _current_time = sim_clock.get_sim_time()
                current_time = _current_time.nanosec / 1e9 + _current_time.sec
                if current_time - phase_start >= 1.5:
                    break
                self.locomotion.set_motion(16)
                time.sleep(0.05)

            # 第三阶段：右转面向出口方向（2.5秒）
            print(f"  阶段3: 右转面向出口")
            phase_start = current_time
            while True:
                _current_time = sim_clock.get_sim_time()
                current_time = _current_time.nanosec / 1e9 + _current_time.sec
                if current_time - phase_start >= 2.5:
                    break
                self.locomotion.set_motion(10)  # 原地右转
                time.sleep(0.05)

            # 第四阶段：确认球在视野中
            print(f"  阶段4: 确认球在视野中")
            self.locomotion.set_motion(1)
            time.sleep(0.5)

            # 微调：如果球不在视野正前方，旋转对准
            for _ in range(10):
                cam_data = football_scanner_main(timeout=0.8)
                if cam_data and cam_data.get('football_detected'):
                    offset = cam_data.get('center_offset', 0)
                    if abs(offset) < 30:
                        print(f"  球已在正前方! offset={offset}")
                        break
                    elif offset > 0:
                        self.locomotion.set_motion(10)
                        time.sleep(0.2)
                    else:
                        self.locomotion.set_motion(9)
                        time.sleep(0.2)
                    self.locomotion.set_motion(1)
                    time.sleep(0.3)
                else:
                    # 找不到球，继续右转搜索
                    self.locomotion.set_motion(10)
                    time.sleep(0.3)
                    self.locomotion.set_motion(1)
                    time.sleep(0.3)

            self.locomotion.set_motion(1)
            time.sleep(0.5)
            print(f"{self.name}: 定位完成，准备推球")
        finally:
            self._ros2_manager.shutdown()


class Dribble_To_Exit(Basic_State):
    """
    带球推向出口：持续前进推球，用视觉跟踪球
    球偏左就微左转追，球偏右就微右转追
    """
    def __init__(self, duration=15):
        super().__init__()
        self.name = "Dribble To Exit"
        self.duration = duration

    def execute(self):
        print(f"Executing {self.name}: 带球推向出口!")
        self._ros2_manager.init()
        sim_clock = self._ros2_manager.get_clock()

        try:
            _start_time = sim_clock.get_sim_time()
            while _start_time is None:
                time.sleep(1.0)
                _start_time = sim_clock.get_sim_time()
            start_time = _start_time.nanosec / 1e9 + _start_time.sec

            lost_count = 0

            while True:
                _current_time = sim_clock.get_sim_time()
                current_time = _current_time.nanosec / 1e9 + _current_time.sec
                if current_time - start_time >= self.duration:
                    break

                try:
                    cam_data = football_scanner_main(timeout=0.5)

                    if cam_data and cam_data.get('football_detected'):
                        lost_count = 0
                        offset = cam_data.get('center_offset', 0)
                        area = cam_data.get('area', 0)

                        # 球还在视野中，边推边调方向
                        if offset > 25:
                            # 球偏右，微右转追球
                            self.locomotion.set_motion(12)
                            time.sleep(0.08)
                        elif offset < -25:
                            # 球偏左，微左转追球
                            self.locomotion.set_motion(11)
                            time.sleep(0.08)
                        else:
                            # 球在正前方，全速推
                            self.locomotion.set_motion(5)
                            time.sleep(0.05)
                    else:
                        lost_count += 1
                        if lost_count > 50:
                            # 球丢了很久，可能已经推出去了
                            print(f"{self.name}: 球已离开视野，可能已踢出!")
                            break
                        # 短暂丢失，继续前进（球可能被推到脚下看不到）
                        self.locomotion.set_motion(5)
                        time.sleep(0.1)

                except Exception as e:
                    self.locomotion.set_motion(5)
                    time.sleep(0.1)

            self.locomotion.set_motion(1)
            time.sleep(1.0)
            print(f"{self.name}: 推球完成")
        finally:
            self._ros2_manager.shutdown()


class Kick_Football(Basic_State):
    """
    踢球：正常速度前进推球，持续推动
    边推边用视觉跟踪确保方向不偏
    """
    def __init__(self, duration=5):
        super().__init__()
        self.name = "Kick Football"
        self.duration = duration

    def execute(self):
        print(f"Executing {self.name}: 前冲踢球!")
        self._ros2_manager.init()
        sim_clock = self._ros2_manager.get_clock()

        try:
            _start_time = sim_clock.get_sim_time()
            while _start_time is None:
                time.sleep(1.0)
                _start_time = sim_clock.get_sim_time()
            start_time = _start_time.nanosec / 1e9 + _start_time.sec

            while True:
                _current_time = sim_clock.get_sim_time()
                current_time = _current_time.nanosec / 1e9 + _current_time.sec
                if current_time - start_time >= self.duration:
                    break
                # 正常速度前进推球
                self.locomotion.set_motion(5)
                time.sleep(0.02)

            self.locomotion.set_motion(1)
            time.sleep(1.0)
            print(f"{self.name}: 踢球完成")
        finally:
            self._ros2_manager.shutdown()


class Navigate_To_Finish(Basic_State):
    """
    导航至终点：踢完球后前进到终点区域
    """
    def __init__(self, duration=10):
        super().__init__()
        self.name = "Navigate To Finish"
        self.duration = duration

    def execute(self):
        print(f"Executing {self.name}: 走向终点...")
        self._ros2_manager.init()
        sim_clock = self._ros2_manager.get_clock()

        try:
            _start_time = sim_clock.get_sim_time()
            while _start_time is None:
                time.sleep(1.0)
                _start_time = sim_clock.get_sim_time()
            start_time = _start_time.nanosec / 1e9 + _start_time.sec

            while True:
                _current_time = sim_clock.get_sim_time()
                current_time = _current_time.nanosec / 1e9 + _current_time.sec
                if current_time - start_time >= self.duration:
                    break
                self.locomotion.set_motion(16)
                time.sleep(0.05)

            self.locomotion.set_motion(1)
            time.sleep(0.5)
            print(f"{self.name}: 到达终点区域")
        finally:
            self._ros2_manager.shutdown()
