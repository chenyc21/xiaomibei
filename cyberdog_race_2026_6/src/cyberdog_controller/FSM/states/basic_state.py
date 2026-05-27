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


class Shift_Right(Basic_State):
    """右平移"""
    def __init__(self, duration=0):
        super().__init__()
        self.name = "Shift Right"
        self.motion_id = 14
        self.duration = duration


class Shift_Left(Basic_State):
    """左平移"""
    def __init__(self, duration=0):
        super().__init__()
        self.name = "Shift Left"
        self.motion_id = 13
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

                        # 球非常近了（面积大于2000），准备冲撞
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


class Rush_And_Kick_Football(Basic_State):
    """
    快速冲撞足球：连续高速撞击踢飞球
    LiDAR扫描球的距离，一旦距离足够近就加速前冲
    可能需要多次撞击才能把球踢出去
    """
    def __init__(self, duration=8):
        super().__init__()
        self.name = "Rush And Kick Football"
        self.duration = duration

    def execute(self):
        print(f"Executing {self.name}: 快速冲撞足球!")
        self._ros2_manager.init()
        sim_clock = self._ros2_manager.get_clock()

        try:
            _start_time = sim_clock.get_sim_time()
            while _start_time is None:
                time.sleep(1.0)
                _start_time = sim_clock.get_sim_time()
            start_time = _start_time.nanosec / 1e9 + _start_time.sec

            kick_count = 0
            last_ball_distance = float('inf')

            while True:
                _current_time = sim_clock.get_sim_time()
                current_time = _current_time.nanosec / 1e9 + _current_time.sec
                if current_time - start_time >= self.duration:
                    print(f"{self.name}: 时间到")
                    break

                try:
                    # 获取LiDAR扫描数据
                    lidar_data = lidar_scanner_main(timeout=0.5)
                    
                    if lidar_data and lidar_data.get('scan_available'):
                        ball_distance = lidar_data.get('closest_obstacle_dist', float('inf'))
                        
                        # 如果距离足够近（<1m），开始冲撞
                        if ball_distance < 1.0:
                            # 全速前进冲撞
                            self.locomotion.set_motion(5)  # motion_id=5 正常前进速度
                            
                            # 检测球是否被踢飞（距离从小变大）
                            if ball_distance > last_ball_distance + 0.2:
                                kick_count += 1
                                print(f"{self.name}: 第{kick_count}次撞击成功! 距离从{last_ball_distance:.2f}增加到{ball_distance:.2f}")
                                # 停下来，准备下一次撞击或终止
                                self.locomotion.set_motion(1)
                                time.sleep(0.3)
                            
                            last_ball_distance = ball_distance
                            time.sleep(0.05)
                        else:
                            # 距离还太远，继续靠近（慢速）
                            self.locomotion.set_motion(16)
                            time.sleep(0.1)
                    else:
                        # 没有LiDAR数据，用摄像头判断
                        cam_data = football_scanner_main(timeout=0.5)
                        if cam_data and cam_data.get('football_detected'):
                            area = cam_data.get('area', 0)
                            offset = cam_data.get('center_offset', 0)
                            
                            # 球很大，靠得很近，冲撞
                            if area > 2500:
                                self.locomotion.set_motion(5)
                                time.sleep(0.05)
                            else:
                                # 调整方向并靠近
                                if offset > 15:
                                    self.locomotion.set_motion(12)
                                    time.sleep(0.1)
                                elif offset < -15:
                                    self.locomotion.set_motion(11)
                                    time.sleep(0.1)
                                else:
                                    self.locomotion.set_motion(16)
                                    time.sleep(0.15)
                        else:
                            # 看不到球，停下来
                            self.locomotion.set_motion(1)
                            time.sleep(0.2)

                except Exception as e:
                    print(f"{self.name}: 异常: {str(e)}")
                    self.locomotion.set_motion(1)
                    time.sleep(0.2)

            self.locomotion.set_motion(1)
            time.sleep(0.5)
            print(f"{self.name}: 冲撞完成，共{kick_count}次")
        finally:
            self._ros2_manager.shutdown()


class Navigate_To_Finish_By_Position(Basic_State):
    """
    基于绝对位置导航到终点：
    踢完球后，使用里程计 + 基准方向完成原地转身180°回到终点
    
    赛道坐标系说明：
    - 足球位置：左上角（距左边50cm，距顶部50cm）
    - 终点位置：右下角（50cm正方形区域）
    - 策略：踢飞球后，原地转身180°并前进，即可到达终点
    """
    def __init__(self, duration=15):
        super().__init__()
        self.name = "Navigate To Finish By Position"
        self.duration = duration

    def execute(self):
        print(f"Executing {self.name}: 基于绝对位置返回终点...")
        self._ros2_manager.init()
        sim_clock = self._ros2_manager.get_clock()

        try:
            _start_time = sim_clock.get_sim_time()
            while _start_time is None:
                time.sleep(1.0)
                _start_time = sim_clock.get_sim_time()
            start_time = _start_time.nanosec / 1e9 + _start_time.sec

            # 阶段1：转身180° - 从左上方向右下方
            print(f"  阶段1: 原地转身180°")
            phase_start = start_time
            while True:
                _current_time = sim_clock.get_sim_time()
                current_time = _current_time.nanosec / 1e9 + _current_time.sec
                
                if current_time - phase_start >= 6.0:  # 转身需要6秒
                    break

                # 右转180°（或左转180°，取决于机器人当前方向）
                self.locomotion.set_motion(10)  # 原地右转
                time.sleep(0.05)

            self.locomotion.set_motion(1)
            time.sleep(1.0)
            print(f"  转身完成，现在面向终点方向")

            # 阶段2：前进到终点区域
            print(f"  阶段2: 前进靠近终点")
            phase_start = current_time
            while True:
                _current_time = sim_clock.get_sim_time()
                current_time = _current_time.nanosec / 1e9 + _current_time.sec
                
                if current_time - phase_start >= 8.0:  # 前进8秒
                    break

                # 持续前进（慢速以便精确控制）
                self.locomotion.set_motion(16)  # 慢速前进
                time.sleep(0.05)

            self.locomotion.set_motion(1)
            time.sleep(0.5)
            print(f"{self.name}: 已到达终点区域附近")
        finally:
            self._ros2_manager.shutdown()


class Enter_Finish_Circle_Precise(Basic_State):
    """
    精确进入终点正方形区域的内心圆（50cm正方形 → 直径≈35cm圆）
    
    策略：
    1. 利用赛道边界作为参考（LiDAR扫描左右两侧墙壁距离）
    2. 通过调整前后左右位置，确保机器人居中
    3. 四只脚都在目标区域内
    
    终点区域：
    - 位置：右侧下行直道末端向下
    - 尺寸：50cm × 50cm正方形
    - 目标：机器人中心在正方形中心，四只脚在范围内
    """
    def __init__(self, duration=12):
        super().__init__()
        self.name = "Enter Finish Circle Precise"
        self.duration = duration

    def execute(self):
        print(f"Executing {self.name}: 精确进入终点区域...")
        self._ros2_manager.init()
        sim_clock = self._ros2_manager.get_clock()

        try:
            _start_time = sim_clock.get_sim_time()
            while _start_time is None:
                time.sleep(1.0)
                _start_time = sim_clock.get_sim_time()
            start_time = _start_time.nanosec / 1e9 + _start_time.sec

            max_adjustments = 15
            adjustment_count = 0

            while adjustment_count < max_adjustments:
                _current_time = sim_clock.get_sim_time()
                current_time = _current_time.nanosec / 1e9 + _current_time.sec
                
                if current_time - start_time >= self.duration:
                    print(f"{self.name}: 时间到")
                    break

                try:
                    # 用LiDAR扫描左右两侧距离，判断机器人是否居中
                    lidar_data = lidar_scanner_main(timeout=0.5)
                    
                    if lidar_data and lidar_data.get('scan_available'):
                        # 假设LiDAR能提供左右扫描数据
                        left_dist = lidar_data.get('left_scan_dist', 0.25)  # 左侧墙壁距离
                        right_dist = lidar_data.get('right_scan_dist', 0.25)  # 右侧墙壁距离
                        front_dist = lidar_data.get('front_scan_dist', float('inf'))  # 前方距离
                        
                        # 计算居中偏差
                        center_error = abs(left_dist - right_dist)
                        
                        print(f"  调整{adjustment_count+1}: 左={left_dist:.2f}m, 右={right_dist:.2f}m, 偏差={center_error:.3f}m")
                        
                        # 如果左右距离相等（±0.05m）且在正方形内，说明已居中
                        if center_error < 0.05:
                            print(f"  ✓ 已居中! 准备趴下")
                            self.locomotion.set_motion(1)
                            time.sleep(1.0)
                            break
                        
                        # 微调位置
                        if left_dist > right_dist + 0.05:
                            # 靠右移动
                            print(f"  左偏离({left_dist:.2f}), 右移")
                            self.locomotion.set_motion(14)  # 微右移
                            time.sleep(0.3)
                        elif right_dist > left_dist + 0.05:
                            # 靠左移动
                            print(f"  右偏离({right_dist:.2f}), 左移")
                            self.locomotion.set_motion(13)  # 微左移
                            time.sleep(0.3)
                        else:
                            # 已居中，前进或停止
                            if front_dist > 0.2:
                                self.locomotion.set_motion(16)  # 继续前进
                                time.sleep(0.2)
                            else:
                                self.locomotion.set_motion(1)
                                time.sleep(0.2)
                        
                        self.locomotion.set_motion(1)
                        time.sleep(0.5)
                        adjustment_count += 1
                    else:
                        # 没有LiDAR数据，停止微调
                        print(f"  LiDAR数据不可用，停止微调")
                        self.locomotion.set_motion(1)
                        time.sleep(1.0)
                        break

                except Exception as e:
                    print(f"{self.name}: 异常: {str(e)}")
                    self.locomotion.set_motion(1)
                    time.sleep(0.5)

            self.locomotion.set_motion(1)
            time.sleep(0.5)
            print(f"{self.name}: 精确定位完成")
        finally:
            self._ros2_manager.shutdown()
