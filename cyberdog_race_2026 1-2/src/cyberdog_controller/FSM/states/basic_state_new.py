from .state import State
from .basic_state import *
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import time

class Walking_Forward_LidarEnhanced(Basic_State):
    """
    LiDAR + 视觉双重感知的前进状态（石板路专用）
    2026 最终自适应版本：引入路径宽度验证与帧级障碍持久化
    """
    def __init__(self, duration=120, motion_id=16):
        super().__init__()
        self.name = "Walking Forward LiDAR Enhanced"
        self.duration = duration
        self.default_motion = motion_id      
        self.high_step_motion = 22            
        self.extreme_step_motion = 25         
        self.recovery_back_motion = 24        
        self.recovery_forward_motion = 25     

        self.steering_threshold = 15
        self.safe_margin = 35
        self.critical_margin = 5  # 极低灵敏度

        self.stuck_check_interval = 5.0
        self.stuck_dist_threshold = 3
        self.last_dist = -1
        self.last_check_time = 0
        self.stuck_count = 0

        self._lidar_cache = None
        self._lidar_cache_time = 0
        self._lidar_cooldown = 0.5

        self.obstacle_count = 0
        self.obstacle_persistence = 0   # 帧级持久化
        self.MIN_PATH_WIDTH = 75        # 真实路径宽度阈值
        self.RECOVERY_SIDE_STEP = 0.12  

    def _get_lidar_data(self):
        now = time.time()
        if self._lidar_cache is not None and (now - self._lidar_cache_time) < self._lidar_cooldown:
            return self._lidar_cache
        try:
            from ...camera.path_scanner import lidar_scanner_main
            data = lidar_scanner_main(timeout=1.5)
            if data and data.get('scan_available', False):
                self._lidar_cache = data
                self._lidar_cache_time = now
                return data
        except Exception:
            pass
        return self._lidar_cache if self._lidar_cache else {'scan_available': False}

    def execute(self):
        print(f"Executing {self.name} with Adaptive Fusion (Width-Discerning)")
        from ...camera.path_scanner import check_camera_ready, path_scanner_main, CenterOffsetScanner
        camera_ok = check_camera_ready(timeout=1.0)
        self._ros2_manager.init()
        sim_clock = self._ros2_manager.get_clock()

        try:
            _start_time = sim_clock.get_sim_time()
            wait_count = 0
            while _start_time is None:
                time.sleep(1.0)
                _start_time = sim_clock.get_sim_time()
            start_time = _start_time.nanosec / 1e9 + _start_time.sec
            self.last_check_time = start_time

            while True:
                _current_time = sim_clock.get_sim_time()
                current_time = _current_time.nanosec / 1e9 + _current_time.sec
                if current_time - start_time >= self.duration: break

                try:
                    # 1. LiDAR 持久化
                    lidar_data = self._get_lidar_data()
                    front_dist = lidar_data.get('front_distance', float('inf'))
                    front_blocked = lidar_data.get('front_blocked', False)
                    stone_warning = (0.15 <= front_dist < 0.35)
                    
                    if front_blocked or stone_warning:
                        self.obstacle_persistence = 25
                    else:
                        self.obstacle_persistence = max(0, self.obstacle_persistence - 1)
                    
                    on_stone_zone = (self.obstacle_persistence > 0)

                    # 2. 视觉与宽度验证
                    if not camera_ok:
                        self.locomotion.set_motion(self.high_step_motion if on_stone_zone else self.default_motion)
                        time.sleep(0.1)
                        continue

                    data = path_scanner_main(CenterOffsetScanner, timeout=1.0)
                    left_edge = data.get('left_edge') if data else None
                    right_edge = data.get('right_edge') if data else None
                    path_width = (right_edge - left_edge) if (left_edge is not None and right_edge is not None) else 0
                    has_real_track = (path_width > self.MIN_PATH_WIDTH)

                    if left_edge is None and right_edge is None:
                        self.locomotion.set_motion(self.high_step_motion if on_stone_zone else self.default_motion)
                        time.sleep(0.1)
                        continue

                    offset = data.get('center_offset', 0)
                    dist = data.get('center_distance', 0)
                    center_x = 160

                    # 3. 紧急避障拦截
                    is_too_close = (left_edge is not None and (center_x - left_edge) < self.critical_margin) or \
                                   (right_edge is not None and (right_edge - center_x) < self.critical_margin)

                    if is_too_close:
                        if on_stone_zone or not has_real_track:
                            print(f"{self.name}: 拦截误判(Width:{path_width}) -> 纠偏侧移")
                            self.locomotion.set_motion(14 if left_edge is not None and (center_x - left_edge) < self.critical_margin else 13)
                            time.sleep(self.RECOVERY_SIDE_STEP)
                            self.locomotion.set_motion(25 if on_stone_zone else self.default_motion)
                            time.sleep(0.1)
                        else:
                            print(f"{self.name}: 真实边界 -> 极小后退")
                            self.locomotion.set_motion(self.recovery_back_motion)
                            time.sleep(0.25)
                        continue

                    # 4. 正常跨越
                    if front_blocked or stone_warning:
                        self.obstacle_count += 1
                        if self.obstacle_count >= 2:
                            print(f"LiDAR: 前方石块 {front_dist:.2f}m -> 跨越！")
                            self.locomotion.set_motion(self.extreme_step_motion if front_blocked else self.high_step_motion)
                            time.sleep(0.6)
                            self.obstacle_count = 0
                            continue
                    else:
                        self.obstacle_count = 0
                        if offset > self.steering_threshold: self.locomotion.set_motion(12); time.sleep(0.08)
                        elif offset < -self.steering_threshold: self.locomotion.set_motion(11); time.sleep(0.08)
                        else: self.locomotion.set_motion(self.default_motion); time.sleep(0.05)

                    if current_time - self.last_check_time > self.stuck_check_interval:
                        self.last_dist = dist
                        self.last_check_time = current_time

                except Exception as e:
                    time.sleep(0.1)
        finally:
            self._ros2_manager.shutdown()
