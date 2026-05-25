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


class Walking_Forward_Climb(Basic_State):
    def __init__(self, duration=0):
        super().__init__()
        self.name = "Walking Forward Climb"
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


class Bridge_Entry_Climb(Basic_State):
    """
    上桥入口连续爬台：卡住时保持高抬腿向前，不切后退或停站重试。
    """
    def __init__(self, duration=9.5, strong_duration=5.0, pulse_interval=1.2, pulse_duration=0.5):
        super().__init__()
        self.name = "Bridge Entry Continuous Climb"
        self.duration = duration
        self.strong_duration = strong_duration
        self.pulse_interval = pulse_interval
        self.pulse_duration = pulse_duration

    def execute(self):
        print(f"Executing {self.name} for duration {self.duration}")
        self._ros2_manager.init()
        sim_clock = self._ros2_manager.get_clock()

        try:
            start_time = self._wait_for_start_time(sim_clock)
            while True:
                current_time = self._get_current_time(sim_clock, start_time)
                elapsed = current_time - start_time
                if elapsed >= self.duration:
                    break

                if elapsed < self.strong_duration:
                    self.locomotion.set_motion(17)
                else:
                    pulse_time = (elapsed - self.strong_duration) % self.pulse_interval
                    if pulse_time < self.pulse_duration:
                        self.locomotion.set_motion(17)
                    else:
                        self.locomotion.set_motion(18)
                time.sleep(0.02)

            print(f"Finished executing {self.name} in {elapsed:.2f} seconds")
        finally:
            self._ros2_manager.shutdown()


class Bridge_Center_Forward(Basic_State):
    """
    独木桥居中慢行：视觉可用时根据桥面中心微调，视觉失效时保守慢走。
    """
    def __init__(self, duration=10.0, stop_on_line=False):
        super().__init__()
        self.name = "Bridge Center Forward"
        self.duration = duration
        self.stop_on_line = stop_on_line
        self.align_threshold = 18
        self.strong_align_threshold = 45
        self.line_confirm_frames = 2

    def execute(self):
        print(f"Executing {self.name} for duration {self.duration}")
        self._ros2_manager.init()
        sim_clock = self._ros2_manager.get_clock()

        try:
            start_time = self._wait_for_start_time(sim_clock)
            line_seen_count = 0

            while True:
                current_time = self._get_current_time(sim_clock, start_time)
                if current_time - start_time >= self.duration:
                    print(f"{self.name}: 到达保守时长，结束本段")
                    break

                try:
                    data = bridge_scanner_main(timeout=0.4)
                except Exception as e:
                    print(f"{self.name}: 视觉异常，保守慢走: {str(e)}")
                    data = {}

                if self.stop_on_line and data.get('line_detected'):
                    line_seen_count += 1
                    print(
                        f"{self.name}: 检测到虚线 "
                        f"count={line_seen_count}, y={data.get('line_y')}, "
                        f"conf={data.get('line_confidence', 0.0):.2f}"
                    )
                    if line_seen_count >= self.line_confirm_frames:
                        self.locomotion.set_motion(1)
                        print(f"{self.name}: 已接近虚线，停止前进等待越线确认")
                        break
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
    def __init__(self, max_duration=4.0, required_frames=3):
        super().__init__()
        self.name = "Bridge Line Confirm"
        self.duration = max_duration
        self.required_frames = required_frames

    def execute(self):
        print(f"Executing {self.name}")
        self._ros2_manager.init()
        sim_clock = self._ros2_manager.get_clock()

        try:
            start_time = self._wait_for_start_time(sim_clock)
            confirmed_frames = 0

            while True:
                current_time = self._get_current_time(sim_clock, start_time)
                if current_time - start_time >= self.duration:
                    print(f"{self.name}: 未稳定识别虚线，采用保守时长 fallback")
                    break

                try:
                    data = bridge_scanner_main(timeout=0.5)
                except Exception as e:
                    print(f"{self.name}: 检测异常: {str(e)}")
                    data = {}

                if data.get('line_detected') and data.get('line_confidence', 0.0) >= 0.25:
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
                    time.sleep(0.08)
        finally:
            self._ros2_manager.shutdown()


class Bridge_Dismount(Basic_State):
    """
    下桥：确认越线后短距离低风险前进，避免激烈跳跃导致身体撞桥。
    """
    def __init__(self, duration=1.4):
        super().__init__()
        self.name = "Bridge Dismount"
        self.duration = duration

    def execute(self):
        print(f"Executing {self.name}: 已确认越线，开始下桥")
        self._ros2_manager.init()
        sim_clock = self._ros2_manager.get_clock()

        try:
            start_time = self._wait_for_start_time(sim_clock)
            while True:
                current_time = self._get_current_time(sim_clock, start_time)
                if current_time - start_time >= self.duration:
                    break
                self.locomotion.set_motion(5)
                time.sleep(0.03)
            self.locomotion.set_motion(1)
            print(f"{self.name}: 下桥动作完成")
        finally:
            self._ros2_manager.shutdown()

