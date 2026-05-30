from abc import ABC
import copy
import math
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
    # duration 默认按 Gazebo /clock (sim 秒) 计时：
    # 机器人 vel_des 单位是 m/sim秒，且 Recovery/Stand/Jump 等动作内部也按 sim 时间推进，
    # 用 sim 时钟才能让"5 秒站立 / 11 秒前进"在不同 RTF 的机器上跑出一致的物理结果。
    # 仅在 /clock 一直拿不到时才会自动 fallback 到墙钟，避免完全卡死。
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
        if self._use_wall_time:
            return time.monotonic()
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


class Walking_Forward_X_Correct(Basic_State):
    """沿 +Y 方向直行 + 闭环修正横向位置 (x) 与朝向 (yaw)。

    使用 base_motion_id 对应的 motion 表项作为模板（保留 step_height/pos_z/gait 等），
    每个控制周期根据 /gazebo/model_states 真值改写 vel_des = [base_vx, vy_corr, yaw_rate]，
    通过 LocomotionController.set_override 覆盖 50 Hz 的发布线程。

    仅用于第五赛段第一段直行 (3.12, 7.35) → (3.12, 11.95)，避免横漂掉桥。
    """
    KP_X = 0.4              # x 偏差 (m) → 横向 vy (m/s)
    KP_YAW = 0.6            # yaw 偏差 (rad) → yaw_rate (rad/s)
    VY_LIMIT = 0.06
    YAW_RATE_LIMIT = 0.15
    # 死区：小于此值时不修正，避免微小误差让机器人持续扭动而爬不上台阶。
    X_DEADZONE = 0.03       # 3 cm
    YAW_DEADZONE = 0.10     # 约 5.7°
    YAW_TARGET = math.pi / 2.0  # 朝 +Y

    def __init__(self, duration=0, base_motion_id=5, target_x=3.12, y_stop=None):
        super().__init__()
        self.name = f"Walking Forward X-Correct (base={base_motion_id})"
        self.motion_id = base_motion_id
        self.duration = duration
        self.target_x = target_x
        self.y_stop = y_stop

    def execute(self):
        self._ros2_manager.init()
        sim_clock = self._ros2_manager.get_clock()
        pose_listener = self._ros2_manager.get_pose_listener()
        base_step = self.locomotion.get_motion_step(self.motion_id)
        base_vx = float(base_step["vel_des"][0])
        override = copy.deepcopy(base_step)

        try:
            print(f"Executing {self.name} duration={self.duration} "
                  f"target_x={self.target_x} y_stop={self.y_stop}")
            start_time = self._wait_for_start_time(sim_clock)
            current_time = start_time
            log_t = 0.0

            while True:
                pose = pose_listener.get_pose() if pose_listener else None
                if pose is not None:
                    px, py, yaw = pose
                    e_x = px - self.target_x
                    if abs(e_x) <= self.X_DEADZONE:
                        vy_corr = 0.0
                    else:
                        vy_corr = max(-self.VY_LIMIT,
                                      min(self.VY_LIMIT, -self.KP_X * e_x))
                    yaw_err = self.YAW_TARGET - yaw
                    yaw_err = math.atan2(math.sin(yaw_err), math.cos(yaw_err))
                    if abs(yaw_err) <= self.YAW_DEADZONE:
                        yaw_rate = 0.0
                    else:
                        yaw_rate = max(-self.YAW_RATE_LIMIT,
                                       min(self.YAW_RATE_LIMIT, self.KP_YAW * yaw_err))
                    override["vel_des"] = [base_vx, vy_corr, yaw_rate]
                    if current_time - log_t >= 0.5:
                        print(f"[X-Correct] x={px:.3f} y={py:.3f} yaw={yaw:+.3f} "
                              f"e_x={e_x:+.3f} vy={vy_corr:+.3f} wz={yaw_rate:+.3f}")
                        log_t = current_time
                    if self.y_stop is not None and py >= self.y_stop:
                        print(f"[X-Correct] y={py:.3f} >= y_stop={self.y_stop}, "
                              "提前结束。")
                        break
                else:
                    override["vel_des"] = [base_vx, 0.0, 0.0]

                self.locomotion.set_override(override)
                time.sleep(0.02)
                current_time = self._get_current_time(sim_clock, current_time)
                if current_time - start_time >= self.duration:
                    break

            print(f"Finished {self.name} in {current_time - start_time:.2f} s")
        finally:
            self.locomotion.clear_override()
            self._ros2_manager.shutdown()


class Stage5_Align_To_3p12(Basic_State):
    """一次性对位状态：先小步幅横移把 x 拉到 target_x，再小幅原地转向对准 +Y。

    动作序列：
      1. 读 /gazebo/model_states 真值。
      2. x > target_x + DZ_X → Shift_Right (motion 14, vy<0)；
         x < target_x - DZ_X → Shift_Left  (motion 13, vy>0)；直到进入死区或超时。
      3. yaw 误差 → Spin_Left/Right (motion 9/10) 直到进入死区或超时。
      4. 结束时切回 Standing (motion 1)。

    完成后由后续 Walking_Forward 开环正常前进，不再叠加闭环。
    """
    DZ_X = 0.015           # 1.5 cm 死区
    DZ_YAW = 0.02          # ≈1.15° 死区
    YAW_TARGET = math.pi / 2.0
    POLL_DT = 0.05
    TIMEOUT_X = 12.0
    TIMEOUT_YAW = 8.0

    def __init__(self, target_x=3.12):
        super().__init__()
        self.name = f"Stage5_Align_To_X={target_x}"
        self.motion_id = 1   # 默认占位 (Standing)
        self.duration = 0.0
        self.target_x = target_x

    def _wrap(self, a):
        return math.atan2(math.sin(a), math.cos(a))

    def _run_until(self, sim_clock, pose_listener, motion_id,
                   predicate, timeout, label):
        """在 sim 时间内持续发送 motion_id，直到 predicate(pose) 为真或超时。"""
        start_time = self._get_current_time(sim_clock, 0.0)
        if start_time == 0.0:
            start_time = self._wait_for_start_time(sim_clock)
        current_time = start_time
        log_t = 0.0
        while True:
            self.locomotion.set_motion(motion_id)
            time.sleep(self.POLL_DT)
            current_time = self._get_current_time(sim_clock, current_time)
            pose = pose_listener.get_pose()
            if pose is not None:
                if current_time - log_t >= 0.5:
                    px, py, yaw = pose
                    print(f"[Align/{label}] x={px:.3f} y={py:.3f} yaw={yaw:+.3f}")
                    log_t = current_time
                if predicate(pose):
                    print(f"[Align/{label}] 到位，t={current_time - start_time:.2f} s")
                    return True
            if current_time - start_time >= timeout:
                print(f"[Align/{label}] 超时 ({timeout:.1f} s)，进入下一阶段。")
                return False

    def execute(self):
        self._ros2_manager.init()
        sim_clock = self._ros2_manager.get_clock()
        pose_listener = self._ros2_manager.get_pose_listener()

        try:
            print(f"Executing {self.name}")
            # 等到拿到一次真值
            wait_start = self._wait_for_start_time(sim_clock)
            current_time = wait_start
            pose = None
            while pose is None:
                pose = pose_listener.get_pose()
                if pose is not None:
                    break
                time.sleep(0.05)
                current_time = self._get_current_time(sim_clock, current_time)
                if current_time - wait_start >= 3.0:
                    print("[Align] 3 s 未收到 /gazebo/model_states，跳过对位。")
                    return

            px, py, yaw = pose
            e_x = px - self.target_x
            print(f"[Align] 起始 pose x={px:.3f} y={py:.3f} yaw={yaw:+.3f} e_x={e_x:+.3f}")

            # ── 阶段 1: 横移修 x ──
            if abs(e_x) > self.DZ_X:
                # x 偏大 → 向 -x 横移；motion 14 vy<0；motion 13 vy>0。
                # 全局 x 与机体 y 关系：机器人朝 +Y，机体 +y = 全局 -x，机体 -y = 全局 +x。
                # 所以全局 x 太大 (e_x>0)，需要机体 +y → motion 13 (Shift_Left, vy=+0.09)。
                if e_x > 0:
                    motion_id = 13
                    pred = lambda p: p[0] - self.target_x <= self.DZ_X * 0.5
                    label = "Shift_Left(x↓)"
                else:
                    motion_id = 14
                    pred = lambda p: self.target_x - p[0] <= self.DZ_X * 0.5
                    label = "Shift_Right(x↑)"
                self._run_until(sim_clock, pose_listener, motion_id, pred,
                                self.TIMEOUT_X, label)
                self.locomotion.set_motion(1)
                time.sleep(0.3)
            else:
                print(f"[Align] |e_x|={abs(e_x):.3f} 已在死区，跳过横移。")

            # ── 阶段 2: 旋转对准 +Y ──
            pose = pose_listener.get_pose() or pose
            yaw_err = self._wrap(self.YAW_TARGET - pose[2])
            print(f"[Align] 横移后 yaw={pose[2]:+.3f} yaw_err={yaw_err:+.3f}")
            if abs(yaw_err) > self.DZ_YAW:
                # yaw_err > 0 → 需要 yaw 增大 → 左转 (Spin_Left, motion 9)
                if yaw_err > 0:
                    motion_id = 9
                    pred = lambda p: self._wrap(self.YAW_TARGET - p[2]) <= self.DZ_YAW * 0.5
                    label = "Spin_Left(yaw↑)"
                else:
                    motion_id = 10
                    pred = lambda p: -self._wrap(self.YAW_TARGET - p[2]) <= self.DZ_YAW * 0.5
                    label = "Spin_Right(yaw↓)"
                self._run_until(sim_clock, pose_listener, motion_id, pred,
                                self.TIMEOUT_YAW, label)
                self.locomotion.set_motion(1)
                time.sleep(0.3)
            else:
                print(f"[Align] |yaw_err|={abs(yaw_err):.3f} 已在死区，跳过旋转。")

            final_pose = pose_listener.get_pose()
            if final_pose is not None:
                print(f"[Align] 完成 pose x={final_pose[0]:.3f} y={final_pose[1]:.3f} "
                      f"yaw={final_pose[2]:+.3f}")
        finally:
            self._ros2_manager.shutdown()


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
        self.name = "Bridge Entry Slow Walk"
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

                self.locomotion.set_motion(16)
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
    第五赛段中段跳跃：使用 Jump3D (motion 22, kJumpDownStair)。
    Jump3D→RecoveryStand 的 transition 门槛已在 fsm_state_jump_3d.cpp 放宽为只判
    data_end_（轨迹播完），不再依赖 touch_down_/height_good_for_trans_，
    桥上即使腿悬空、姿态偏也不会卡死，后续 Recovery_Stand 一定能进。
    """
    def __init__(self, duration=2.0):
        super().__init__()
        self.name = "Bridge Mid Jump"
        self.duration = duration
        self.motion_id = 22

    def execute(self):
        print(f"Executing {self.name}: 中段 Jump3D (kJumpDownStair) 跳跃")
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
            print(f"{self.name}: 中段跳跃动作已发出")
        finally:
            self._ros2_manager.shutdown()
