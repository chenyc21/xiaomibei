from abc import ABC
import time
from ...locomotion import LocomotionController
import threading


class _LazyROS2Manager:
    def __getattr__(self, name):
        from ...utils.ros2_manager import ros2_manager
        return getattr(ros2_manager, name)


def bridge_scanner_main(timeout=5.0):
    from ...camera.path_scanner import bridge_scanner_main as scanner
    return scanner(timeout=timeout)


def lidar_scanner_main(timeout=2.0):
    from ...camera.path_scanner import lidar_scanner_main as scanner
    return scanner(timeout=timeout)


def _bridge_line_is_close(data, min_y_ratio=0.58, min_confidence=0.25):
    if not data.get('line_detected'):
        return False

    confidence = data.get('line_confidence', 0.0) or 0.0
    if confidence < min_confidence:
        return False

    line_y = data.get('line_y')
    image_height = data.get('image_height')
    if line_y is None:
        return False
    if image_height:
        return line_y >= image_height * min_y_ratio

    return True


class Basic_State(ABC):
    def __init__(self):
        self.name = "Basic_State"
        self.motion_id = 0
        self.duration = -1
        self._locomotion = None
        self._ros2_manager = _LazyROS2Manager()
        self._use_wall_time = False

    @property
    def locomotion(self):
        if self._locomotion is None:
            self._locomotion = LocomotionController()
        return self._locomotion

    def execute(self):
        self._ros2_manager.init()
        sim_clock = self._ros2_manager.get_clock()

        try:
            print(f"Executing {self.name} with motion ID {self.motion_id} for duration {self.duration}")
            start_time = self._wait_for_start_time(sim_clock)
            current_time = start_time

            while True:
                self.locomotion.set_motion(self.motion_id)
                time.sleep(0.01)
                current_time = self._get_current_time(sim_clock, current_time)
                if current_time - start_time >= self.duration:
                    break
            print(f"Finished executing {self.name} in {current_time - start_time:.2f} seconds")
        finally:
            self._ros2_manager.shutdown()

    def _wait_for_start_time(self, sim_clock, max_wait_seconds=8.0):
        _start_time = sim_clock.get_sim_time()
        wait_count = 0
        wait_started = time.monotonic()
        while _start_time is None:
            if wait_count % 5 == 0:
                print(f"Waiting for /clock topic... (Attempt {wait_count})")
            if time.monotonic() - wait_started >= max_wait_seconds:
                self._use_wall_time = True
                print("未收到 /clock，切换到本机时间 fallback。请确认 Gazebo 已取消暂停。")
                return time.monotonic()
            time.sleep(1.0)
            _start_time = sim_clock.get_sim_time()
            wait_count += 1
        self._use_wall_time = False
        return _start_time.nanosec / 1e9 + _start_time.sec

    def _get_current_time(self, sim_clock, fallback_time):
        if self._use_wall_time:
            return time.monotonic()
        _current_time = sim_clock.get_sim_time()
        if _current_time:
            return _current_time.nanosec / 1e9 + _current_time.sec
        return fallback_time


class Standing(Basic_State):
    def __init__(self, duration=0):
        super().__init__()
        self.name = "Standing"
        self.motion_id = 1
        self.duration = duration


class Recovery_Stand(Basic_State):
    def __init__(self, duration=5.0):
        super().__init__()
        self.name = "Recovery Stand"
        self.motion_id = 2
        self.duration = duration
        self.trigger_duration = 0.25

    def execute(self):
        self._ros2_manager.init()
        sim_clock = self._ros2_manager.get_clock()

        try:
            print(
                f"Executing {self.name}: trigger recovery then hold locomotion stand "
                f"for duration {self.duration}"
            )
            start_time = self._wait_for_start_time(sim_clock)
            current_time = start_time

            while True:
                elapsed = current_time - start_time
                if elapsed >= self.duration:
                    break

                if elapsed < self.trigger_duration:
                    self.locomotion.set_motion(self.motion_id)
                else:
                    self.locomotion.set_motion(1)
                time.sleep(0.02)
                current_time = self._get_current_time(sim_clock, current_time)

            self.locomotion.set_motion(1)
            print(f"Finished executing {self.name} in {current_time - start_time:.2f} seconds")
        finally:
            self._ros2_manager.shutdown()


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
        self.name = "Walking Forward Slow High Step"
        self.motion_id = 16
        self.duration = duration


class Slope_Bridge_Forward(Basic_State):
    def __init__(self, duration=0):
        super().__init__()
        self.name = "Slope Bridge Forward"
        self.motion_id = 25
        self.duration = duration


class Walking_Forward_Slope(Slope_Bridge_Forward):
    def __init__(self, duration=0):
        super().__init__(duration)
        self.name = "Walking Forward Slope Bridge"


class Walking_Forward_Climb(Basic_State):
    def __init__(self, duration=0):
        super().__init__()
        self.name = "Bridge Entry High Step"
        self.motion_id = 17
        self.duration = duration


class Walking_Forward_Stable_Climb(Basic_State):
    def __init__(self, duration=0):
        super().__init__()
        self.name = "Walking Forward Stable Climb"
        self.motion_id = 18
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


class Bridge_Turn_Left(Basic_State):
    def __init__(self, duration=7.9):
        super().__init__()
        self.name = "Bridge Stable Turn Left"
        self.motion_id = 23
        self.duration = duration


class Bridge_Turn_Right(Basic_State):
    def __init__(self, duration=7.9):
        super().__init__()
        self.name = "Bridge Stable Turn Right"
        self.motion_id = 24
        self.duration = duration


class Slope_Bridge_Turn_Left(Basic_State):
    def __init__(self, duration=8.5):
        super().__init__()
        self.name = "Slope Bridge Turn Left"
        self.motion_id = 26
        self.duration = duration


class Slope_Bridge_Turn_Right(Basic_State):
    def __init__(self, duration=8.5):
        super().__init__()
        self.name = "Slope Bridge Turn Right"
        self.motion_id = 27
        self.duration = duration


class Bridge_Entry_Climb(Basic_State):
    """
    上桥入口慢速进入：第五赛段要求全程在独木桥上行走，入口段不使用快速冲刺。
    """
    def __init__(self, duration=9.5):
        super().__init__()
        self.name = "Bridge Entry High Step"
        self.duration = duration

    def execute(self):
        print(f"Executing {self.name} for duration {self.duration}")
        self._ros2_manager.init()
        sim_clock = self._ros2_manager.get_clock()

        try:
            start_time = self._wait_for_start_time(sim_clock)
            current_time = start_time
            while True:
                current_time = self._get_current_time(sim_clock, current_time)
                elapsed = current_time - start_time
                if elapsed >= self.duration:
                    break

                self.locomotion.set_motion(17)
                time.sleep(0.02)

            print(f"Finished executing {self.name} in {elapsed:.2f} seconds")
        finally:
            self._ros2_manager.shutdown()


class Bridge_Center_Forward(Basic_State):
    """
    独木桥居中慢行：视觉可用时根据桥面中心微调，视觉失效时保守慢走。
    """
    def __init__(self, duration=10.0, stop_on_line=False, line_min_y_ratio=0.58):
        super().__init__()
        self.name = "Bridge Center Forward"
        self.duration = duration
        self.stop_on_line = stop_on_line
        self.line_min_y_ratio = line_min_y_ratio
        self.align_threshold = 18
        self.strong_align_threshold = 45
        self.line_confirm_frames = 2

    def execute(self):
        print(f"Executing {self.name} for duration {self.duration}")
        self._ros2_manager.init()
        sim_clock = self._ros2_manager.get_clock()

        try:
            start_time = self._wait_for_start_time(sim_clock)
            current_time = start_time
            line_seen_count = 0

            while True:
                current_time = self._get_current_time(sim_clock, current_time)
                if current_time - start_time >= self.duration:
                    print(f"{self.name}: 到达保守时长，结束本段")
                    break

                try:
                    data = bridge_scanner_main(timeout=0.4)
                except Exception as e:
                    print(f"{self.name}: 视觉异常，保守慢走: {str(e)}")
                    data = {}

                if self.stop_on_line and _bridge_line_is_close(data, self.line_min_y_ratio):
                    line_seen_count += 1
                    print(
                        f"{self.name}: 检测到近处虚线 "
                        f"count={line_seen_count}, y={data.get('line_y')}, "
                        f"conf={data.get('line_confidence', 0.0):.2f}"
                    )
                    if line_seen_count >= self.line_confirm_frames:
                        self.locomotion.set_motion(1)
                        print(f"{self.name}: 已接近虚线，停止前进等待越线确认")
                        break
                    self.locomotion.set_motion(1)
                    time.sleep(0.05)
                    continue
                elif self.stop_on_line and data.get('line_detected'):
                    line_seen_count = 0
                    print(
                        f"{self.name}: 虚线仍偏远，继续慢走 "
                        f"y={data.get('line_y')}, conf={data.get('line_confidence', 0.0):.2f}"
                    )
                else:
                    line_seen_count = 0

                if data.get('bridge_detected'):
                    offset = data.get('center_offset', 0)
                    confidence = data.get('bridge_confidence', 0.0)
                    if offset > self.strong_align_threshold:
                        self.locomotion.set_motion(14)
                        time.sleep(0.12)
                    elif offset < -self.strong_align_threshold:
                        self.locomotion.set_motion(13)
                        time.sleep(0.12)
                    elif offset > self.align_threshold:
                        self.locomotion.set_motion(14)
                        time.sleep(0.06)
                    elif offset < -self.align_threshold:
                        self.locomotion.set_motion(13)
                        time.sleep(0.06)
                    else:
                        self.locomotion.set_motion(16)
                        time.sleep(0.05)
                    print(f"{self.name}: offset={offset}, conf={confidence:.2f}")
                else:
                    self.locomotion.set_motion(16)
                    time.sleep(0.08)
        finally:
            self._ros2_manager.shutdown()


class Bridge_Line_Confirm(Basic_State):
    """
    虚线确认：连续看到虚线才允许进入越线补偿；看不到则等待到保守超时。
    """
    def __init__(self, max_duration=4.0, required_frames=3, line_min_y_ratio=0.58):
        super().__init__()
        self.name = "Bridge Line Confirm"
        self.duration = max_duration
        self.required_frames = required_frames
        self.line_min_y_ratio = line_min_y_ratio

    def execute(self):
        print(f"Executing {self.name}")
        self._ros2_manager.init()
        sim_clock = self._ros2_manager.get_clock()

        try:
            start_time = self._wait_for_start_time(sim_clock)
            current_time = start_time
            confirmed_frames = 0

            while True:
                current_time = self._get_current_time(sim_clock, current_time)
                if current_time - start_time >= self.duration:
                    print(f"{self.name}: 未稳定识别虚线，采用保守时长 fallback")
                    break

                try:
                    data = bridge_scanner_main(timeout=0.5)
                except Exception as e:
                    print(f"{self.name}: 检测异常: {str(e)}")
                    data = {}

                if _bridge_line_is_close(data, self.line_min_y_ratio):
                    confirmed_frames += 1
                    self.locomotion.set_motion(1)
                    print(
                        f"{self.name}: 虚线确认帧 {confirmed_frames}/{self.required_frames}, "
                        f"y={data.get('line_y')}, conf={data.get('line_confidence', 0.0):.2f}"
                    )
                    if confirmed_frames >= self.required_frames:
                        print(f"{self.name}: 虚线确认完成")
                        break
                else:
                    confirmed_frames = 0
                    self.locomotion.set_motion(16)
                    if data.get('line_detected'):
                        print(
                            f"{self.name}: 虚线未到近处，继续慢走 "
                            f"y={data.get('line_y')}, conf={data.get('line_confidence', 0.0):.2f}"
                        )
                    time.sleep(0.08)
        finally:
            self._ros2_manager.shutdown()


class Bridge_Jump_Down(Basic_State):
    """
    赛题第五赛段要求：四个足底都越过独木桥虚线后，从桥上跳下。
    """
    def __init__(self, duration=2.0):
        super().__init__()
        self.name = "Bridge Jump Down"
        self.duration = duration
        self.motion_id = 22

    def execute(self):
        print(f"Executing {self.name}: 已确认四足越线补偿，开始跳下")
        self._ros2_manager.init()
        sim_clock = self._ros2_manager.get_clock()

        try:
            start_time = self._wait_for_start_time(sim_clock)
            current_time = start_time
            while True:
                current_time = self._get_current_time(sim_clock, current_time)
                if current_time - start_time >= self.duration:
                    break
                self.locomotion.set_motion(self.motion_id)
                time.sleep(0.03)
            print(f"{self.name}: 跳下动作已发出")
        finally:
            self._ros2_manager.shutdown()


class Bridge_Mid_Jump(Basic_State):
    """
    第五赛段中段跳跃：先给 Jump3D 足够起跳时间，再进入恢复站立。
    """
    def __init__(self, duration=1.8):
        super().__init__()
        self.name = "Bridge Mid Jump"
        self.duration = duration
        self.motion_id = 22
        self.trigger_duration = 0.25

    def execute(self):
        print(f"Executing {self.name}: 短触发 Jump3D，落地后切回 locomotion 站立")
        self._ros2_manager.init()
        sim_clock = self._ros2_manager.get_clock()

        try:
            start_time = self._wait_for_start_time(sim_clock)
            current_time = start_time
            while True:
                current_time = self._get_current_time(sim_clock, current_time)
                if current_time - start_time >= self.duration:
                    break
                if current_time - start_time < self.trigger_duration:
                    self.locomotion.set_motion(self.motion_id)
                else:
                    self.locomotion.set_motion(1)
                time.sleep(0.03)
            self.locomotion.set_motion(1)
            print(f"{self.name}: 中段跳跃动作已发出")
        finally:
            self._ros2_manager.shutdown()
