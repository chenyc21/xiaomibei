from .state import State
from .basic_state import *


class Stage6_Final(State):
    """
    第六赛段：撷金建功 (基于绝对位置 Pose 的精确版)
    
    核心坐标（世界坐标系）：
    - 球初始位置：(0.277102, 14.838659) ← 贴球状态
    - 球一侧参考点：(0.989358, 14.804397) ← 侧向移位目标（贴球状态右侧约0.71m）
    - 起始位置：(2.356542, 13.434867)
    - 推球后位置：(2.34, 13.19) ← 已进入终点圈近处
    - 终点圆心：约 (2.47, 12.97) ← 需要从坐标推球到此
    - 最终位置：(3.01, 12.94) ← 终点圈中心，趴下
    
    流程：
    1. 初始站稳
    2. 搜索足球（旋转扫描）
    3. 接近足球（视觉引导）
    4. 移位到球的一侧（绝对坐标：X≈0.99, Y≈14.80）
    5. 面向球门方向（yaw ≈ -0.51rad，约-29°）
    6. 冲撞踢球（连续推进）
    7. 回到起始点附近（2.34, 13.19）
    8. 进入终点圆形区域中心（3.01, 12.94）
    9. 趴下结算
    """

    def __init__(self):
        super().__init__("Stage6_Final")

        self.basic_states = [
            # 1. 初始站稳
            Standing(2.0),
            
            # 2. 搜索足球（旋转扫描，找到白球）
            Search_Football(30.0),
            Standing(1.0),
            
            # 3. 接近足球（视觉引导，慢速前进）
            Approach_Football_Advanced(25.0),
            Standing(1.0),
            
            # 4. 移位到球的一侧（绝对坐标导航）
            # 从球位置(0.277, 14.839) → 侧向移位点(0.989, 14.804)
            # 需要向前移约0.71m，然后调整方向面向球门（yaw ≈ -0.51rad）
            Move_To_Ball_Side(15.0),
            Standing(1.0),
            
            # 5. 推球进终点（连续前进推动，直到到达起始点附近）
            Push_Ball_To_Finish(15.0),
            Standing(1.0),
            
            # 6. 精确进入终点圆形区域（从(2.34, 13.19) → (3.01, 12.94)）
            Enter_Finish_Precise(10.0),
            Standing(1.0),
            
            # 7. 趴下结算
            Laying(5.0)
        ]

    def execute(self):
        """执行所有子状态"""
        for state in self.basic_states:
            print(f"\n>>> Executing: {state.name}")
            state.execute()
            print(f"<<< Finished: {state.name}")


# ==================== 新增高级状态类 ====================

class Approach_Football_Advanced(Basic_State):
    """
    接近足球（高级版）：视觉对准 + 距离检测 + 自动停止
    当距离足够近时（area > 2000）自动停止，不等待超时
    """
    def __init__(self, duration=25):
        super().__init__()
        self.name = "Approach Football Advanced"
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
            approach_complete = False

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
                        area = cam_data.get('area', 0)

                        print(f"{self.name}: 视觉检测 offset={offset}, area={area}")

                        # 球非常近了（面积大于2000），自动停止
                        if area > 2000:
                            print(f"{self.name}: 球已进入接近范围! area={area}，停止接近")
                            self.locomotion.set_motion(1)
                            approach_complete = True
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
                            self.locomotion.set_motion(16)  # 慢速前进
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

            if approach_complete:
                self.locomotion.set_motion(1)
                time.sleep(0.5)
                print(f"{self.name}: 接近完成")
        finally:
            self._ros2_manager.shutdown()


class Move_To_Ball_Side(Basic_State):
    """
    移位到球的一侧：
    目标：从球位置 (0.277, 14.839) 移到侧向位置 (0.989, 14.804)
    
    策略：
    1. 向前移约0.71m（从0.277→0.989）
    2. 调整Y和yaw，使机器狗面向球门方向（yaw ≈ -0.51rad）
    """
    def __init__(self, duration=15):
        super().__init__()
        self.name = "Move To Ball Side"
        self.duration = duration

    def execute(self):
        print(f"Executing {self.name}: 移位到球的一侧...")
        self._ros2_manager.init()
        sim_clock = self._ros2_manager.get_clock()

        try:
            _start_time = sim_clock.get_sim_time()
            while _start_time is None:
                time.sleep(1.0)
                _start_time = sim_clock.get_sim_time()
            start_time = _start_time.nanosec / 1e9 + _start_time.sec

            # 阶段1：先直行向前约0.7m（从x=0.277→0.989）
            print(f"  阶段1: 向前移约0.7m，从贴球状态到球的一侧")
            phase_start = start_time
            phase_duration = 5.0  # 预计5秒走0.7m（慢速）
            
            while True:
                _current_time = sim_clock.get_sim_time()
                current_time = _current_time.nanosec / 1e9 + _current_time.sec
                
                if current_time - phase_start >= phase_duration:
                    break

                # 慢速前进
                self.locomotion.set_motion(16)
                time.sleep(0.05)

            self.locomotion.set_motion(1)
            time.sleep(0.5)
            print(f"  阶段1 完成: 已移位向前")

            # 阶段2：调整方向面向球门（yaw ≈ -0.51rad，约-29°）
            print(f"  阶段2: 调整朝向，面向球门方向（yaw ≈ -0.51rad）")
            phase_start = current_time
            phase_duration = 3.0  # 转向预计3秒
            
            while True:
                _current_time = sim_clock.get_sim_time()
                current_time = _current_time.nanosec / 1e9 + _current_time.sec
                
                if current_time - phase_start >= phase_duration:
                    break

                # 缓慢右转（从背对球门转向朝向球门）
                self.locomotion.set_motion(10)  # 原地右转
                time.sleep(0.05)

            self.locomotion.set_motion(1)
            time.sleep(0.5)
            print(f"  阶段2 完成: 已调整朝向面向球门")
            print(f"{self.name}: 移位完成，准备推球")

        finally:
            self._ros2_manager.shutdown()


class Push_Ball_To_Finish(Basic_State):
    """
    推球进终点：
    从球侧位置 (0.989, 14.804) 推球到起始点附近 (2.34, 13.19)
    
    策略：
    1. 全速或中速前进，推动足球向终点方向
    2. 通过LiDAR或时间控制停止（约推进1.5m）
    3. 最终应该到达 (2.34, 13.19)
    """
    def __init__(self, duration=15):
        super().__init__()
        self.name = "Push Ball To Finish"
        self.duration = duration

    def execute(self):
        print(f"Executing {self.name}: 推球进终点...")
        self._ros2_manager.init()
        sim_clock = self._ros2_manager.get_clock()

        try:
            _start_time = sim_clock.get_sim_time()
            while _start_time is None:
                time.sleep(1.0)
                _start_time = sim_clock.get_sim_time()
            start_time = _start_time.nanosec / 1e9 + _start_time.sec

            push_distance_estimate = 0.0
            push_count = 0

            while True:
                _current_time = sim_clock.get_sim_time()
                current_time = _current_time.nanosec / 1e9 + _current_time.sec
                if current_time - start_time >= self.duration:
                    print(f"{self.name}: 推球时间到")
                    break

                try:
                    # 中速前进推球（motion_id=16 或 5）
                    # 使用慢速以保证推球稳定
                    self.locomotion.set_motion(16)
                    time.sleep(0.1)
                    push_distance_estimate += 0.15 * 0.1  # 粗估距离

                    # 每1秒检查一次LiDAR，如果没有障碍了说明球已推出
                    if int(current_time - start_time) % 1 == 0:
                        try:
                            lidar_data = lidar_scanner_main(timeout=0.5)
                            if lidar_data and lidar_data.get('scan_available'):
                                front_dist = lidar_data.get('front_distance', float('inf'))
                                print(f"{self.name}: 前方距离 {front_dist:.2f}m, 推进距离约 {push_distance_estimate:.2f}m")
                                
                                # 如果前方1.5m内都没有障碍，可能球已被推出，停止
                                if front_dist > 1.5:
                                    push_count += 1
                                    if push_count > 2:  # 连续3次都没有障碍，停止
                                        print(f"{self.name}: 球已推出，停止推进")
                                        break
                        except:
                            pass

                except Exception as e:
                    print(f"{self.name}: 异常: {str(e)}")
                    time.sleep(0.2)

            self.locomotion.set_motion(1)
            time.sleep(0.5)
            print(f"{self.name}: 推球完成，共推进约 {push_distance_estimate:.2f}m")

        finally:
            self._ros2_manager.shutdown()


class Enter_Finish_Precise(Basic_State):
    """
    精确进入终点圆形区域：
    从推球后位置 (2.34, 13.19) 移到终点圆心 (3.01, 12.94)
    
    策略：
    1. 继续向前进约0.67m（从x=2.34→3.01）
    2. 左移约0.25m（从y=13.19→12.94），保持直线推进即可
    3. 检查LiDAR左右距离是否对称（可选）
    """
    def __init__(self, duration=10):
        super().__init__()
        self.name = "Enter Finish Precise"
        self.duration = duration

    def execute(self):
        print(f"Executing {self.name}: 精确进入终点圆形区域...")
        self._ros2_manager.init()
        sim_clock = self._ros2_manager.get_clock()

        try:
            _start_time = sim_clock.get_sim_time()
            while _start_time is None:
                time.sleep(1.0)
                _start_time = sim_clock.get_sim_time()
            start_time = _start_time.nanosec / 1e9 + _start_time.sec

            # 阶段1：前进到终点圆心 (3.01, 12.94)
            # 从 (2.34, 13.19) 前进约0.67m + 左移0.25m
            print(f"  阶段1: 前进并微调方向，进入终点圆形中心")
            phase_start = start_time
            phase_duration = 6.0  # 预计6秒完成微调和前进
            
            adjustment_count = 0

            while True:
                _current_time = sim_clock.get_sim_time()
                current_time = _current_time.nanosec / 1e9 + _current_time.sec
                
                if current_time - phase_start >= phase_duration:
                    break

                try:
                    # 每2秒尝试一次LiDAR左右对称调整
                    if int(current_time - phase_start) % 2 == 0 and adjustment_count < 2:
                        lidar_data = lidar_scanner_main(timeout=0.5)
                        if lidar_data and lidar_data.get('scan_available'):
                            left_dist = lidar_data.get('left_distance', float('inf'))
                            right_dist = lidar_data.get('right_distance', float('inf'))
                            
                            print(f"  调整检查: 左={left_dist:.2f}m, 右={right_dist:.2f}m")
                            
                            # 如果左右不对称，微调
                            if left_dist < right_dist - 0.05:
                                print(f"  左侧更近，微右移")
                                self.locomotion.set_motion(14)  # 微右移
                                time.sleep(0.2)
                            elif right_dist < left_dist - 0.05:
                                print(f"  右侧更近，微左移")
                                self.locomotion.set_motion(13)  # 微左移
                                time.sleep(0.2)
                            else:
                                print(f"  已居中，前进")
                                self.locomotion.set_motion(16)  # 慢速前进
                                time.sleep(0.2)
                            
                            adjustment_count += 1
                        else:
                            # LiDAR不可用，直接前进
                            self.locomotion.set_motion(16)
                            time.sleep(0.1)
                    else:
                        # 默认前进
                        self.locomotion.set_motion(16)
                        time.sleep(0.1)

                except Exception as e:
                    print(f"  调整异常: {str(e)}")
                    self.locomotion.set_motion(16)
                    time.sleep(0.1)

            self.locomotion.set_motion(1)
            time.sleep(0.5)
            print(f"  阶段1 完成: 已进入终点圆形区域")
            print(f"{self.name}: 精确定位完成")

        finally:
            self._ros2_manager.shutdown()
