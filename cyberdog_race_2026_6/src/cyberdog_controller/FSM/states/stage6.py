from .state import State
from .basic_state import *
from ...camera.path_scanner import (
    football_scanner_main,
    yellow_edge_scanner_main,
    finish_circle_scanner_main,
)
import time


# ==================== 通用辅助函数 ====================

def _wait_for_sim_time(sim_clock):
    """等待仿真时钟可用，返回起始秒数"""
    _t = sim_clock.get_sim_time()
    while _t is None:
        time.sleep(0.2)
        _t = sim_clock.get_sim_time()
    return _t.nanosec / 1e9 + _t.sec


def _sim_now(sim_clock):
    """当前仿真秒数"""
    _t = sim_clock.get_sim_time()
    if _t is None:
        return None
    return _t.nanosec / 1e9 + _t.sec


def _run_motion_for(sim_clock, locomotion, motion_id, duration, step=0.05):
    """在仿真时间下持续执行某个 motion_id"""
    phase_start = _sim_now(sim_clock)
    if phase_start is None:
        phase_start = _wait_for_sim_time(sim_clock)

    while True:
        now = _sim_now(sim_clock)
        if now is None:
            time.sleep(step)
            continue
        if now - phase_start >= duration:
            break
        locomotion.set_motion(motion_id)
        time.sleep(step)

    locomotion.set_motion(1)
    time.sleep(0.10)


# ==================== 顶层赛段 ====================

class Stage6_Final(State):
    """
    第六赛段：撷金建功（最终射门版）
    """

    def __init__(self):
        super().__init__("Stage6_Final")

        self.basic_states = [
            Standing(2.0),

            Search_Football(25.0),
            Standing(0.8),

            Approach_Football(18.0),
            Standing(0.8),

            Align_To_Ball(duration=6.0, align_threshold=16, min_area=900),
            Standing(0.6),

            Reposition_For_Shot(duration=10.0),
            Standing(0.8),

            Search_Football(8.0),
            Standing(0.5),

            Align_To_Ball(duration=5.0, align_threshold=10, min_area=700),
            Standing(0.5),

            Rush_And_Kick_Football(duration=10.0, max_kicks=3),
            Standing(0.8),

            Return_To_Finish(duration=10.0),
            Standing(0.8),

            Laying(5.0)
        ]

    def execute(self):
        for state in self.basic_states:
            print(f"\n{'=' * 70}")
            print(f">>> Executing: {state.name}")
            print(f"{'=' * 70}")
            state.execute()
            print(f"<<< Finished: {state.name}")
            print(f"{'=' * 70}\n")


# ==================== 贴球对齐 ====================

class Align_To_Ball(Basic_State):
    """
    对球精调：
    - 球尽量居中
    - 球面积达到最小阈值才算贴近
    - 连续确认多帧才结束
    """

    def __init__(self, duration=6.0, align_threshold=15, min_area=700):
        super().__init__()
        self.name = "Align To Ball"
        self.duration = duration
        self.align_threshold = align_threshold
        self.min_area = min_area

    def execute(self):
        print(
            f"Executing {self.name}: 对球精调 "
            f"(threshold={self.align_threshold}, min_area={self.min_area})..."
        )
        self._ros2_manager.init()
        sim_clock = self._ros2_manager.get_clock()

        try:
            start_time = _wait_for_sim_time(sim_clock)
            confirm_count = 0
            need_confirm = 3
            lost_count = 0

            while True:
                now = _sim_now(sim_clock)
                if now is None:
                    time.sleep(0.1)
                    continue

                if now - start_time >= self.duration:
                    print(f"{self.name}: 超时结束")
                    break

                try:
                    cam_data = football_scanner_main(timeout=0.5)

                    if cam_data and cam_data.get("football_detected"):
                        lost_count = 0
                        offset = int(cam_data.get("center_offset", 0))
                        area = int(cam_data.get("area", 0))

                        print(
                            f"{self.name}: offset={offset}, area={area}, "
                            f"confirm={confirm_count}/{need_confirm}"
                        )

                        if abs(offset) > self.align_threshold:
                            if offset > 0:
                                self.locomotion.set_motion(12)
                            else:
                                self.locomotion.set_motion(11)
                            time.sleep(0.10)
                            confirm_count = 0
                            continue

                        if area < self.min_area:
                            print(f"{self.name}: 球还不够近，轻推贴近")
                            _run_motion_for(sim_clock, self.locomotion, 5, 0.08)
                            confirm_count = 0
                            continue

                        self.locomotion.set_motion(1)
                        time.sleep(0.18)
                        confirm_count += 1

                        if confirm_count >= need_confirm:
                            print(f"{self.name}: 对球完成✓")
                            break

                    else:
                        lost_count += 1
                        print(f"{self.name}: 暂时看不到球 lost={lost_count}")
                        _run_motion_for(sim_clock, self.locomotion, 11, 0.10)

                        if lost_count >= 5:
                            print(f"{self.name}: 连续丢球，结束对齐")
                            break

                except Exception as e:
                    print(f"{self.name}: 异常 {str(e)}")
                    time.sleep(0.15)

            self.locomotion.set_motion(1)
            time.sleep(0.30)

        finally:
            self._ros2_manager.shutdown()


# ==================== 绕位到射门位 ====================

class Reposition_For_Shot(Basic_State):
    """
    绕位：
    左45 -> 前进 -> 左90 -> 前进 -> 左45
    """

    def __init__(self, duration=10.0):
        super().__init__()
        self.name = "Reposition For Shot"
        self.duration = duration

        self.turn_45_time = 0.95
        self.turn_90_time = 1.90
        self.forward_short_time = 0.65

    def execute(self):
        print(f"Executing {self.name}: 绕位到射门位...")
        self._ros2_manager.init()
        sim_clock = self._ros2_manager.get_clock()

        try:
            start_time = _wait_for_sim_time(sim_clock)

            def _timeout():
                now = _sim_now(sim_clock)
                return now is not None and (now - start_time >= self.duration)

            actions = [
                (9, self.turn_45_time, "左转45°"),
                (5, self.forward_short_time, "前进短距离①"),
                (9, self.turn_90_time, "左转90°"),
                (5, self.forward_short_time, "前进短距离②"),
                (9, self.turn_45_time, "左转45°"),
            ]

            for motion_id, dur, msg in actions:
                if _timeout():
                    print(f"{self.name}: 超时，提前结束")
                    break
                print(f"{self.name}: {msg}")
                _run_motion_for(sim_clock, self.locomotion, motion_id, dur)

            self.locomotion.set_motion(1)
            time.sleep(0.40)
            print(f"{self.name}: 绕位完成✓")

        finally:
            self._ros2_manager.shutdown()


# ==================== 射门 / 冲撞 ====================

class Rush_And_Kick_Football(Basic_State):
    """
    射门逻辑：
    - 只有球“居中且够近”时才记作一次 kick
    - 最多 3 次
    - 至少踢过一次后，若连续丢球，则认为球已被打走
    """

    def __init__(self, duration=10.0, max_kicks=3):
        super().__init__()
        self.name = "Rush And Kick Football"
        self.duration = duration
        self.max_kicks = max_kicks

        self.align_threshold = 10
        self.shoot_area_threshold = 700
        self.approach_time = 0.10
        self.kick_burst_time = 1.00

    def execute(self):
        print(f"Executing {self.name}: 准备射门...")
        self._ros2_manager.init()
        sim_clock = self._ros2_manager.get_clock()

        try:
            start_time = _wait_for_sim_time(sim_clock)
            kick_count = 0
            lost_count = 0

            while True:
                now = _sim_now(sim_clock)
                if now is None:
                    time.sleep(0.1)
                    continue

                if now - start_time >= self.duration:
                    print(f"{self.name}: 时间到")
                    break

                if kick_count >= self.max_kicks:
                    print(f"{self.name}: 已完成 {kick_count} 次冲撞")
                    break

                try:
                    cam_data = football_scanner_main(timeout=0.5)

                    if cam_data and cam_data.get("football_detected"):
                        lost_count = 0
                        offset = int(cam_data.get("center_offset", 0))
                        area = int(cam_data.get("area", 0))

                        print(
                            f"{self.name}: offset={offset}, area={area}, "
                            f"kick={kick_count}/{self.max_kicks}"
                        )

                        if abs(offset) > self.align_threshold:
                            if offset > 0:
                                self.locomotion.set_motion(12)
                            else:
                                self.locomotion.set_motion(11)
                            time.sleep(0.10)
                            continue

                        if area < self.shoot_area_threshold:
                            print(f"{self.name}: 球还不够近，继续贴近")
                            _run_motion_for(sim_clock, self.locomotion, 5, self.approach_time)
                            continue

                        kick_count += 1
                        print(f"{self.name}: >>> KICK {kick_count} <<<")
                        _run_motion_for(sim_clock, self.locomotion, 5, self.kick_burst_time)
                        self.locomotion.set_motion(1)
                        time.sleep(0.25)

                    else:
                        lost_count += 1
                        print(f"{self.name}: 看不到球 lost={lost_count}")

                        if kick_count >= 1 and lost_count >= 4:
                            print(f"{self.name}: 已踢中并连续丢球，结束射门")
                            break

                        _run_motion_for(sim_clock, self.locomotion, 11, 0.15)

                except Exception as e:
                    print(f"{self.name}: 异常 {str(e)}")
                    time.sleep(0.15)

            self.locomotion.set_motion(1)
            time.sleep(0.30)
            print(f"{self.name}: 射门完成✓")

        finally:
            self._ros2_manager.shutdown()


# ==================== 回终点 ====================

class Return_To_Finish(Basic_State):
    """
    回终点：
    - 优先找终点圈
    - 找不到则使用黄边缺口
    - 连续检测到“黄边缺失 / 开口区域”后，再前进一段进入终点
    """

    def __init__(self, duration=10.0):
        super().__init__()
        self.name = "Return To Finish"
        self.duration = duration

    def execute(self):
        print(f"Executing {self.name}: 正在归航...")
        self._ros2_manager.init()
        sim_clock = self._ros2_manager.get_clock()

        try:
            start_time = _wait_for_sim_time(sim_clock)
            opening_seen_count = 0
            no_yellow_count = 0
            circle_seen_count = 0

            while True:
                now = _sim_now(sim_clock)
                if now is None:
                    time.sleep(0.1)
                    continue

                if now - start_time >= self.duration:
                    print(f"{self.name}: 时间到，停止归航")
                    break

                # ---------- A. 优先找终点圈 ----------
                try:
                    circle_data = finish_circle_scanner_main(timeout=0.5)
                except Exception:
                    circle_data = {}

                if circle_data and circle_data.get("circle_detected"):
                    offset = int(circle_data.get("center_offset", 0))
                    area = int(circle_data.get("area", 0))
                    dist = int(circle_data.get("distance", 999))

                    print(f"{self.name}: 终点圈 offset={offset}, area={area}, dist={dist}")

                    opening_seen_count = 0
                    no_yellow_count = 0

                    if abs(offset) > 18:
                        if offset > 0:
                            self.locomotion.set_motion(12)
                        else:
                            self.locomotion.set_motion(11)
                        time.sleep(0.10)
                    else:
                        _run_motion_for(sim_clock, self.locomotion, 5, 0.20)

                    if area > 2200 or dist < 28:
                        circle_seen_count += 1
                    else:
                        circle_seen_count = 0

                    if circle_seen_count >= 2:
                        print(f"{self.name}: 已接近终点圈，最后进入")
                        _run_motion_for(sim_clock, self.locomotion, 5, 0.80)
                        break

                    continue

                # ---------- B. 黄边缺口兜底 ----------
                try:
                    yellow_data = yellow_edge_scanner_main(timeout=0.5)
                except Exception:
                    yellow_data = {}

                if yellow_data and yellow_data.get("yellow_detected"):
                    left_edge = yellow_data.get("left_yellow_edge", None)
                    right_edge = yellow_data.get("right_yellow_edge", None)
                    track_offset = int(yellow_data.get("track_center_offset", 0))
                    track_width = int(yellow_data.get("track_width", 0))

                    print(
                        f"{self.name}: yellow "
                        f"left={left_edge}, right={right_edge}, "
                        f"offset={track_offset}, width={track_width}"
                    )

                    no_yellow_count = 0

                    if left_edge is None or right_edge is None or track_width < 70:
                        opening_seen_count += 1
                        print(f"{self.name}: 检测到终点开口 {opening_seen_count}/3")
                        _run_motion_for(sim_clock, self.locomotion, 5, 0.20)

                        if opening_seen_count >= 3:
                            print(f"{self.name}: 终点开口确认，最后前进进入")
                            _run_motion_for(sim_clock, self.locomotion, 5, 0.80)
                            break
                    else:
                        opening_seen_count = 0

                        if abs(track_offset) > 20:
                            if track_offset > 0:
                                self.locomotion.set_motion(12)
                            else:
                                self.locomotion.set_motion(11)
                            time.sleep(0.10)
                        else:
                            _run_motion_for(sim_clock, self.locomotion, 5, 0.18)

                else:
                    no_yellow_count += 1
                    print(f"{self.name}: 未检测到黄边 {no_yellow_count}/4")
                    _run_motion_for(sim_clock, self.locomotion, 5, 0.20)

                    if no_yellow_count >= 4:
                        print(f"{self.name}: 连续无黄边，认为进入终点区域")
                        _run_motion_for(sim_clock, self.locomotion, 5, 0.70)
                        break

            self.locomotion.set_motion(1)
            time.sleep(0.40)
            print(f"{self.name}: 归航完成✓")

        finally:
            self._ros2_manager.shutdown()
