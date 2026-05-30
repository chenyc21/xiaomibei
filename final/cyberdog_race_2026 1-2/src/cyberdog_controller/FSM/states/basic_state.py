from abc import ABC
import time
from ...locomotion import LocomotionController
import threading
from ...utils.ros2_manager import ros2_manager
from ...camera.path_scanner import *

# 2026 衔接控制全局变量
global_turn_direction = 'left' 

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
            
            # 等待仿真时钟
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
                time.sleep(0.01)  # 发布频率
                
                _current_time = sim_clock.get_sim_time()
                if _current_time:
                    current_time = _current_time.nanosec / 1e9 + _current_time.sec
                
                if current_time - start_time >= self.duration:
                    break
            print(f"Finished executing {self.name} in {current_time - start_time:.2f} seconds")
        finally:
            self._ros2_manager.shutdown()



class Standing_Up(Basic_State):
    """
    从趴下状态站起（2026赛题起点要求）
    """
    def __init__(self, duration=3):
        super().__init__()
        self.name = "Standing Up"
        self.motion_id = 1 # 站立模式
        self.duration = duration


class Standing(Basic_State):
    """
    站立状态
    """
    def __init__(self, duration=0):
        super().__init__()
        self.name = "Standing"
        self.motion_id = 1
        self.duration = duration


class Laying(Basic_State):
    """
    趴下状态（2026赛题起点要求）
    """
    def __init__(self, duration=0):
        super().__init__()
        self.name = "Laying"
        self.motion_id = 3
        self.duration = duration


class Walking_to_Ball(Basic_State):
    """
    向小球移动并撞击
    """
    def __init__(self, duration=15, target_color='orange'):
        super().__init__()
        self.name = f"Walking to {target_color} ball"
        self.duration = duration
        self.target_color = target_color
        self.hit_threshold = 15 # 距离阈值

    def execute(self):
        from ...camera.ball_scanner import ball_scanner_main
        from ...camera.path_scanner import check_camera_ready

        if not check_camera_ready(timeout=1.0):
            print(f"{self.name}: 摄像头不可用，跳过。")
            return

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
                    data = ball_scanner_main(timeout=1.0, target_color=self.target_color)
                    if data and data.get('target_detected'):
                        offset = data.get('center_offset', 0)
                        dist = data.get('distance', 100)

                        if dist < self.hit_threshold:
                            print(f"Target {self.target_color} ball hit!")
                            self.locomotion.set_motion(5)
                            time.sleep(1.0)
                            break

                        if offset > 15:
                            self.locomotion.set_motion(12)
                        elif offset < -15:
                            self.locomotion.set_motion(11)
                        else:
                            self.locomotion.set_motion(5)
                    else:
                        self.locomotion.set_motion(11)
                except Exception as e:
                    print(f"Ball detection error: {str(e)}")
                    break
        finally:
            self._ros2_manager.shutdown()

class Spin_to_Find_Ball(Basic_State):
    """
    旋转寻找小球
    """
    def __init__(self, duration=10, target_color='orange'):
        super().__init__()
        self.name = f"Spin to find {target_color} ball"
        self.duration = duration
        self.target_color = target_color

    def execute(self):
        from ...camera.ball_scanner import ball_scanner_main
        from ...camera.path_scanner import check_camera_ready

        if not check_camera_ready(timeout=1.0):
            print(f"{self.name}: 摄像头不可用，无法搜索小球。跳过此步骤。")
            return

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
                    data = ball_scanner_main(timeout=1.0, target_color=self.target_color)
                    if data and data.get('target_detected'):
                        print(f"{self.target_color.capitalize()} ball spotted!")
                        break
                    else:
                        self.locomotion.set_motion(11) # 左旋寻找
                except Exception as e:
                    print(f"Ball search error: {str(e)}")
                    break
        finally:
            self._ros2_manager.shutdown()


class Walking_Forward(Basic_State):
    """
    前进
    """
    def __init__(self, duration=0):
        super().__init__()
        self.name = "Walking Forward"
        self.motion_id = 5
        self.duration = duration

class Walking_Forward_Slow(Basic_State):
    """
    前进，很慢很慢，正常速度1/2
    """
    def __init__(self, duration=0):
        super().__init__()
        self.name = "Walking Forward"
        self.motion_id = 16
        self.duration = duration

class Walking_Forward_by_px(Basic_State):
    '''
    动态的根据目前视野最远点的距离，来判断停下的时间，传入参数是停下时最远点距离。
    @param duration: 最大值，达不到要求时作用
    @param px: 距离最远点的距离
    '''
    def __init__(self, duration, px):
        super().__init__()
        self.name = "Walking Forward by px"
        self.duration = duration
        self.px = px
        self.flag = 0
        

    def execute(self):
        print(f"Executing {self.name} with px threshold: {self.px}")
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
            current_time = start_time
            while True:
                time.sleep(0.01)
                _current_time = sim_clock.get_sim_time()
                current_time = _current_time.nanosec / 1e9 + _current_time.sec
                    
                if current_time - start_time >= self.duration:
                    break
                try:
                    data = path_scanner_main(FarDistanceScanner, timeout=1.0)  # 动态调用
                    distance = data.get('far_distance', 0)
                    
                    if distance > self.px:
                        self.locomotion.set_motion(5)
                        time.sleep(0.01)
                    elif self.flag == 1:
                        print(f"Finished executing {self.name} in {current_time - start_time:.2f} seconds")
                        break

                    if distance < self.px:
                        self.flag = 1
                except Exception as e:
                    print(f"检测异常: {str(e)}")
                    break
        finally:
            self._ros2_manager.shutdown()


class Walking_Forward_by_rightbound(Basic_State):
    '''
    前进，检测右边界，前进至右边界小于设定值。这个也是曲线入口专用，
    根据视野内右边界与赛道的距离判断,
    注意不要和下面的Walking_Forward_by_bottom混淆，
    这个是根据整个赛道判断的，下面的是根据底边相接赛道处判断的
    '''
    def __init__(self, duration,right_bound):
        super().__init__()
        self.name = "Walking Forward by rightbound"
        self.duration = duration  
        self.flag = 0
        self.right_bound = right_bound

    def execute(self):
        print(f"Executing {self.name} with distance threshold: {self.duration}")
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
            current_time = start_time
            while True:
                time.sleep(0.01)
                _current_time = sim_clock.get_sim_time()
                current_time = _current_time.nanosec / 1e9 + _current_time.sec
                if current_time - start_time >= self.duration:
                    break
                try:
                    data = path_scanner_main(RightBoundaryScanner, timeout=1.0)  # 动态调用
                    right_bound= data.get('right_boundary', 0)
                    if self.right_bound <= right_bound:
                        self.locomotion.set_motion(5)
                        time.sleep(0.01)
                    else : 
                        print(f"Finished executing {self.name} in {current_time - start_time:.2f} seconds")
                        break
                except Exception as e:
                    print(f"检测异常: {str(e)}")
                    break
        
        finally:
            self._ros2_manager.shutdown()

class Walking_Stone_by_distance(Basic_State):
    '''
    限高杆->石板路的石板路
    '''
    def __init__(self, duration,distance):
        super().__init__()
        self.name = "Walking Forward by distance"
        self.duration = duration  
        self.flag = 0
        self.distance = distance

    def execute(self):
        print(f"Executing {self.name} with distance threshold: {self.duration}")
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
            current_time = start_time
            while True:
                time.sleep(0.01)
                _current_time = sim_clock.get_sim_time()
                current_time = _current_time.nanosec / 1e9 + _current_time.sec
                if current_time - start_time >= self.duration:
                    break
                try:
                    data = path_scanner_main(CenterOffsetScanner, timeout=1.0)  # 动态调用
                    distance= data.get('center_distance', 0)
                    if distance <= self.distance :
                        self.locomotion.set_motion(16)
                        time.sleep(0.01)
                    else : 
                        print(f"Finished executing {self.name} in {current_time - start_time:.2f} seconds")
                        break
                except Exception as e:
                    print(f"检测异常: {str(e)}")
                    break
        
        finally:
            self._ros2_manager.shutdown()

class Walking_Forward_by_distance_z(Basic_State):
    '''
    使用离得最近的边线的距离，
    @param duration: 最大值，达不到要求时作用
    @param distance: 距离最近赛道的距离
    '''
    def __init__(self, duration,distance):
        super().__init__()
        self.name = "Walking Forward by distance"
        self.duration = duration  
        self.flag = 0
        self.distance = distance

    def execute(self):
        print(f"Executing {self.name} with distance threshold: {self.duration}")
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
            current_time = start_time
            while True:
                time.sleep(0.01)
                _current_time = sim_clock.get_sim_time()
                current_time = _current_time.nanosec / 1e9 + _current_time.sec
                if current_time - start_time >= self.duration:
                    break
                try:
                    data = path_scanner_main(CenterOffsetScanner, timeout=1.0)  # 动态调用
                    distance= data.get('center_distance', 0)
                    if  self.distance < distance:
                        self.locomotion.set_motion(5)
                        time.sleep(0.01)
                    elif distance == 0 : 
                        print(f"Finished executing {self.name} in {current_time - start_time:.2f} seconds")
                        break
                    else : 
                        print(f"Finished executing {self.name} in {current_time - start_time:.2f} seconds")
                        break
                except Exception as e:
                    print(f"检测异常: {str(e)}")
                    break
        
        finally:
            self._ros2_manager.shutdown()


class Walking_Slope_by_distance(Basic_State):
    '''
    走斜坡，动态调整走坡时间
    '''
    def __init__(self, duration,distance):
        super().__init__()
        self.name = "Walking Forward by distance"
        self.duration = duration  
        self.flag = 0
        self.distance = distance

    def execute(self):
        print(f"Executing {self.name} with distance threshold: {self.duration}")
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
            current_time = start_time
            while True:
                time.sleep(0.01)
                _current_time = sim_clock.get_sim_time()
                current_time = _current_time.nanosec / 1e9 + _current_time.sec
                if current_time - start_time >= self.duration:
                    break
                try:
                    data = path_scanner_main(CenterOffsetScanner, timeout=1.0)  # 动态调用
                    distance= data.get('center_distance', 0)
                    offset = data.get('center_offset', 0)
                    if distance <= self.distance or distance == None:
                        self.locomotion.set_motion(19)
                        time.sleep(0.01)
                        self.flag = 0
                    elif self.flag == 1: 
                        print(f"Finished executing {self.name} in {current_time - start_time:.2f} seconds")
                        break
                    if distance > self.distance and offset != 0 and offset < 50 and offset > -50:
                        self.flag = 1
                except Exception as e:
                    print(f"检测异常: {str(e)}")
                    break
        
        finally:
            self._ros2_manager.shutdown()



class Spin_by_line(Basic_State):
    '''
    原地旋转，根据眼前横向（小于10度）线条判断，然后决定旋转方向，顺、逆旋转至横线小于1.5度。
    如果摄像头不可用，快速退出（不做大角度旋转），由外层 Stage 的 Spin_Left 负责实际转弯。
    '''
    def __init__(self, duration):
        super().__init__()
        self.name = "Spin by line"
        self.duration = duration
        self.flag = 0

    def execute(self):
        # 先检查摄像头是否可用
        from ...camera.path_scanner import check_camera_ready
        camera_ok = check_camera_ready(timeout=1.5)

        if not camera_ok:
            print(f"{self.name}: 摄像头不可用，跳过视觉对齐（转弯由外层 Spin_Left 负责）。")
            return

        print(f"Executing {self.name} with distance threshold: {self.duration}")
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
            current_time = start_time

            no_line_count = 0
            while True:
                time.sleep(0.02)
                _current_time = sim_clock.get_sim_time()
                current_time = _current_time.nanosec / 1e9 + _current_time.sec
                if current_time - start_time >= self.duration:
                    print(f"{self.name}: 视觉对齐超时（{self.duration}s），停止搜索。")
                    break
                try:
                    data = path_scanner_main(HorizontalLineScanner, timeout=1.0)
                    is_horizontal_line = data.get('is_horizontal_line', -1) if data else -1

                    if is_horizontal_line == 2:
                        self.locomotion.set_motion(12)
                        no_line_count = 0
                    elif is_horizontal_line == 1:
                        self.locomotion.set_motion(11)
                        no_line_count = 0
                    elif is_horizontal_line == 3:
                        print(f"水平线已对齐！完成 {self.name} in {current_time - start_time:.2f}s")
                        break
                    else:
                        no_line_count += 1
                        if no_line_count <= 3:
                            print("Spinning... searching for horizontal line.")
                        self.locomotion.set_motion(11)
                except Exception as e:
                    print(f"检测异常: {str(e)}")
                    break
        finally:
            self._ros2_manager.shutdown()


class Walking_by_black_bar(Basic_State):
    '''
    石板路->限高杆，根据限高杆走
    '''
    def __init__(self, duration):
        super().__init__()
        self.name = "Spin by black bar"
        self.duration = duration
        self.flag = 0

    def execute(self):
        print(f"Executing {self.name} with distance threshold: {self.duration}")
        
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
            current_time = start_time

            while True:
                time.sleep(0.01)
                _current_time = sim_clock.get_sim_time()
                current_time = _current_time.nanosec / 1e9 + _current_time.sec
                if current_time - start_time >= self.duration:
                    break
                try:
                    data = path_scanner_main(BlackBarScanner, timeout=1.0)  # 动态调用
                    is_horizontal_line= data.get('is_horizontal_bar', 0)
                    if is_horizontal_line == -1:
                        print(f"Finished executing {self.name} in {current_time - start_time:.2f} seconds")
                        break
                    else :
                        self.locomotion.set_motion(5)
                    
                except Exception as e:
                    print(f"检测异常: {str(e)}")
                    break
        finally:
            sim_clock.destroy_node()
            self._ros2_manager.shutdown()   

class Walking_Backward(Basic_State):
    """
    后退
    """
    def __init__(self, duration=0):
        super().__init__()
        self.name = "Walking Backward"
        self.motion_id = 6
        self.duration = duration


class Walking_Backward_by_distance(Basic_State):
    '''
    后退，根据目前视野最远点的距离，来判断停下的时间，传入参数是停下时最远点距离。
    '''
    def __init__(self, duration,distance):
        super().__init__()
        self.name = "Walking Forward by distance"
        self.duration = duration  
        self.flag = 0
        self.distance = distance

    def execute(self):
        print(f"Executing {self.name} with distance threshold: {self.duration}")
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
            current_time = start_time
            while True:
                time.sleep(0.01)
                _current_time = sim_clock.get_sim_time()
                current_time = _current_time.nanosec / 1e9 + _current_time.sec
                if current_time - start_time >= self.duration:
                    break
                try:
                    data = path_scanner_main(CenterOffsetScanner, timeout=1.0)  # 动态调用
                    distance= data.get('center_distance', 0)
                    offset = data.get('center_offset', 0)
                    if distance <= self.distance or distance == None:
                        self.locomotion.set_motion(6)
                        time.sleep(0.01)
                        self.flag = 0
                    elif self.flag == 1: 
                        print(f"Finished executing {self.name} in {current_time - start_time:.2f} seconds")
                        break
                    if distance > self.distance and offset != 0 and offset < 50 and offset > -50:
                        self.flag = 1
                except Exception as e:
                    print(f"检测异常: {str(e)}")
                    break
        
        finally:
            self._ros2_manager.shutdown()

class Shift_Left(Basic_State):
    """
    向左平移，非前进左转
    """
    def __init__(self, duration=0):
        super().__init__()
        self.name = "Shift Left"
        self.motion_id = 7
        self.duration = duration
    
class Shift_Left_by_bottom(Basic_State):
    def __init__(self, duration , distance ):
        """
        向左平移，非前进左转，目前是曲线入口专用，根据视野内底边赛道与左边界的距离判断
        """
        super().__init__()
        self.name = "Shift Left by bottom"
        self.motion_id = 7
        self.duration = duration
        self.distance = distance

    def execute(self):
        print(f"Executing {self.name} with distance threshold: {self.distance}")
        
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
            current_time = start_time
            while True:
                time.sleep(0.01)
                _current_time = sim_clock.get_sim_time()
                current_time = _current_time.nanosec / 1e9 + _current_time.sec
                if current_time - start_time >= self.duration:
                    break
                try:
                    data = path_scanner_main(BottomLeftScanner  , timeout=1.0)  # 动态调用
                    distance = data.get('left_bottom_distance', 0)
                    if distance <= self.distance and distance != 0:
                        self.locomotion.set_motion(13)
                        time.sleep(0.01)
                    else : 
                        print(f"Finished executing {self.name} in {current_time - start_time:.2f} seconds")
                        break
                except Exception as e:
                    print(f"检测异常: {str(e)}")
                break
        finally:
            self._ros2_manager.shutdown()
            
class Shift_Right_by_bottom(Basic_State):
    def __init__(self, duration , distance):
        """
        向右平移，非前进右转，目前是曲线入口专用，根据视野内底边赛道与右边界的距离判断
        """
        super().__init__()
        self.name = "Shift Right by bottom"
        self.motion_id = 8
        self.duration = duration
        self.distance = distance

    def execute(self):
        print(f"Executing {self.name} with distance threshold: {self.distance}")
       
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
            current_time = start_time
            while True:
                time.sleep(0.01)
                _current_time = sim_clock.get_sim_time()
                current_time = _current_time.nanosec / 1e9 + _current_time.sec
                if current_time - start_time >= self.duration:
                    break
                try:
                    data = path_scanner_main(BottomLeftScanner  , timeout=1.0)  # 动态调用
                    distance = data.get('left_bottom_distance', 0)
                    if distance >= self.distance:
                        self.locomotion.set_motion(8)
                    else: 
                        print(f"Finished executing {self.name} in {current_time - start_time:.2f} seconds")
                        break
                except Exception as e:
                    print(f"检测异常: {str(e)}")
                    break
        finally:
            self._ros2_manager.shutdown()

class Shift_Right(Basic_State):
    """
    向右平移，非前进右转
    """
    def __init__(self, duration=0):
        super().__init__()
        self.name = "Shift Right"
        self.motion_id = 8
        self.duration = duration

class Shift_by_angle(Basic_State):
    '''
    有调整的原地向右转
    '''
    def __init__(self, duration):
        super().__init__()
        self.name = "Shift by angle"
        self.motion_id = 7
        self.duration = duration
        self.flag = 0

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
            current_time = start_time
            while True:
                time.sleep(0.01)
                _current_time = sim_clock.get_sim_time()
                current_time = _current_time.nanosec / 1e9 + _current_time.sec
                if current_time - start_time >= self.duration:
                    break
                try:
                    data = path_scanner_main(CenterOffsetScanner, timeout=1.0)  # 动态调用
                    distance_to_center = data.get('center_offset', 0)
                    if distance_to_center <= 3 and distance_to_center >= -3:
                        print(f"Finished executing {self.name} in {current_time - start_time:.2f} seconds")
                        break
                    if distance_to_center < -3 :
                        self.locomotion.set_motion(13)
                        time.sleep(0.01)
                    if distance_to_center > 3:
                        self.locomotion.set_motion(14)
                        time.sleep(0.01)
                except Exception as e:
                    print(f"检测异常: {str(e)}")
                    break
        finally:
            self._ros2_manager.shutdown()

class Spin_Left(Basic_State):
    """
    原地向左转
    """
    def __init__(self, duration=0):
        super().__init__()
        self.name = "Spin Left"
        self.motion_id = 9
        self.duration = duration


class Spin_Right(Basic_State):
    """
    原地向右转
    """
    def __init__(self, duration=0):
        super().__init__()
        self.name = "Spin Right"
        self.motion_id = 10
        self.duration = duration

class Spin_by_angle(Basic_State):
    '''
    有调整的原地向右转
    '''
    def __init__(self, duration):
        super().__init__()
        self.name = "Spin Right by angle"
        self.motion_id = 10
        self.duration = duration
        self.flag = 0

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
            current_time = start_time
            while True:
                time.sleep(0.01)
                _current_time = sim_clock.get_sim_time()
                current_time = _current_time.nanosec / 1e9 + _current_time.sec
                if current_time - start_time >= self.duration:
                    break
                try:
                    data = path_scanner_main(CenterOffsetScanner, timeout=1.0)  # 动态调用
                    distance_to_center = data.get('center_offset', 0)

                    if distance_to_center <= 5 and distance_to_center >= -5:
                        print(f"Finished executing {self.name} in {current_time - start_time:.2f} seconds")
                        break
                    if distance_to_center <= -4 :
                        self.locomotion.set_motion(11)
                        time.sleep(0.01)
                    if distance_to_center >= 4:
                        self.locomotion.set_motion(12)
                except Exception as e:
                    print(f"检测异常: {str(e)}")
                    break
        finally:
            self._ros2_manager.shutdown()

class Spin_by_slope(Basic_State):
    '''
    坡底使用，旋转调整
    '''
    def __init__(self, duration):
        super().__init__()
        self.name = "Spin Slope by angle"
        self.motion_id = 10
        self.duration = duration
        self.flag = 0

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
            current_time = start_time
            while True:
                time.sleep(0.01)
                _current_time = sim_clock.get_sim_time()
                current_time = _current_time.nanosec / 1e9 + _current_time.sec
                if current_time - start_time >= self.duration:
                    break
                try:
                    data = path_scanner_main(SlopeScanner, timeout=1.0)  # 动态调用
                    distance_to_center = data.get('center_offset', 0)

                    if distance_to_center < 9 and distance_to_center > -9:
                        print(f"Finished executing {self.name} in {current_time - start_time:.2f} seconds")
                        break
                    if distance_to_center <= -8 :
                        self.locomotion.set_motion(11)
                        time.sleep(0.01)
                    if distance_to_center >= 8:
                        self.locomotion.set_motion(12)
                except Exception as e:
                    print(f"检测异常: {str(e)}")
                    break
        finally:
            self._ros2_manager.shutdown()

class Shift_by_slope(Basic_State):
    '''
    坡底使用，平移调整
    '''
    def __init__(self, duration):
        super().__init__()
        self.name = "Shift Slope by angle"
        self.motion_id = 10
        self.duration = duration
        self.flag = 0

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
            current_time = start_time
            while True:
                time.sleep(0.01)
                _current_time = sim_clock.get_sim_time()
                current_time = _current_time.nanosec / 1e9 + _current_time.sec
                if current_time - start_time >= self.duration:
                    break
                try:
                    data = path_scanner_main(SlopeScanner, timeout=1.0)  # 动态调用
                    distance_to_center = data.get('center_offset', 0)

                    if distance_to_center < 12 and distance_to_center > -12:
                        print(f"Finished executing {self.name} in {current_time - start_time:.2f} seconds")
                        break
                    if distance_to_center <= -13 :
                        self.locomotion.set_motion(13)
                        time.sleep(0.01)
                    if distance_to_center >= 13:
                        self.locomotion.set_motion(14)
                except Exception as e:
                    print(f"检测异常: {str(e)}")
                    break
        finally:
            self._ros2_manager.shutdown()

class Walking_Forward_Robust(Basic_State):
    """
    更健壮的前进状态：
    1. 增加黄色边界线避障（防止踩线/越线）
    2. 增加卡死检测（冗余逻辑）
    3. 增加自动脱困（后退/侧移）
    4. 无视觉数据时也能可靠前进
    """
    def __init__(self, duration=30, motion_id=16, target_distance=0):
        super().__init__()
        self.name = "Walking Forward Robust"
        self.duration = duration
        self.motion_id = motion_id
        self.target_distance = target_distance

        # 边界控制参数
        self.safe_margin = 45
        self.critical_margin = 35  # 增加紧急纠正阈值，严防死守
        self.steering_threshold = 8 # 降低阈值，精细纠偏防止踩线

        # 卡死检测参数
        self.stuck_check_interval = 8.0 # 给予极高容错，适应颠簸
        self.stuck_dist_threshold = 3
        self.last_dist = -1
        self.last_check_time = 0
        self.stuck_count = 0
        self.ball_confirm_count = 0 # 衔接检测防抖

        # 无视觉时的盲走参数
        self.blind_walk_start = 0
        self.last_pos_x = 0  # 用于基于位置的卡死检测（无视觉时）

    def execute(self):
        print(f"Executing {self.name} with motion_id {self.motion_id}")

        # 先检查摄像头状态
        from ...camera.path_scanner import check_camera_ready
        camera_ok = check_camera_ready(timeout=1.0)

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
            self.last_check_time = start_time
            self.blind_walk_start = start_time
            had_vision = False
            blind_msg_count = 0

            while True:
                _current_time = sim_clock.get_sim_time()
                current_time = _current_time.nanosec / 1e9 + _current_time.sec

                if current_time - start_time >= self.duration:
                    print("Duration reached, stopping.")
                    break

                try:
                    if not camera_ok:
                        # 摄像头不可用：纯盲走模式，使用预编程时序
                        self.locomotion.set_motion(self.motion_id)
                        time.sleep(0.1)
                        if current_time - self.last_check_time > self.stuck_check_interval:
                            self.last_check_time = current_time
                        continue

                    # === 2026 衔接增强：非阻塞高优蓝球检测 ===
                    try:
                        from ...camera.ball_scanner import ball_scanner_main
                        ball_data = ball_scanner_main(timeout=0.15, target_color='blue')
                        if ball_data:
                            if ball_data.get('target_detected'):
                                ball_dist = ball_data.get('distance', 100)
                                # 放宽至 160，尽早捕捉
                                if ball_dist < 160: 
                                    self.ball_confirm_count += 1
                                    if self.ball_confirm_count >= 2:
                                        print(f"{self.name}: 提前捕捉到前方蓝球 (PixelDist:{ball_dist})，切入第二赛段！")
                                        break
                                else:
                                    self.ball_confirm_count = max(0, self.ball_confirm_count - 1)
                            else:
                                self.ball_confirm_count = 0
                    except Exception:
                        pass

                    data = path_scanner_main(CenterOffsetScanner, timeout=0.2) # 降低路径扫描 timeout

                    # 检查是否识别到赛道
                    left_edge = data.get('left_edge') if data else None
                    right_edge = data.get('right_edge') if data else None
                    has_track = (left_edge is not None) or (right_edge is not None)

                    if not has_track:
                        if had_vision:
                            print(f"{self.name}: 视觉信号丢失，立即停止并等待恢复...")
                            self.locomotion.set_motion(1) # 发现看不清路，原地站立不动
                            time.sleep(0.5)
                        else:
                            self.locomotion.set_motion(1) # 持续看不清路，保持站立
                            time.sleep(0.2)
                            blind_msg_count += 1
                            if blind_msg_count % 30 == 0:
                                print(f"{self.name}: 正在等待视觉信号... (已停止 {current_time - self.blind_walk_start:.1f}s)")
                        
                        if current_time - self.last_check_time > self.stuck_check_interval:
                            self.last_check_time = current_time
                        continue

                    # === 有视觉数据 ===
                    if not had_vision:
                        print(f"{self.name}: 视觉已恢复！开始赛道跟踪。")
                    had_vision = True

                    offset = data.get('center_offset', 0)
                    dist = data.get('center_distance', 0)

                    center_x = 160

                    # 1. 黄色边界线避障逻辑（2026 增强版：侧移替代后退）
                    boundary_steer = 0

                    if (left_edge is not None and (center_x - left_edge) < self.critical_margin) or \
                       (right_edge is not None and (right_edge - center_x) < self.critical_margin):
                        print(f"{self.name}: 靠近边界，执行纠正侧移...")
                        if left_edge is not None and (center_x - left_edge) < self.critical_margin:
                            self.locomotion.set_motion(14) # 右移
                        else:
                            self.locomotion.set_motion(13) # 左移
                        time.sleep(0.2)
                        continue

                    if left_edge is not None and (center_x - left_edge) < self.safe_margin:
                        boundary_steer = 12
                    elif right_edge is not None and (right_edge - center_x) < self.safe_margin:
                        boundary_steer = 11

                    # 2. 卡死检测逻辑 (增强版)
                    if current_time - self.last_check_time > self.stuck_check_interval:
                        # 检查是否有位置进度或视觉距离变化
                        is_stuck = False
                        if self.last_dist != -1:
                            dist_change = abs(dist - self.last_dist)
                            if dist_change < self.stuck_dist_threshold:
                                is_stuck = True
                        
                        if is_stuck:
                            self.stuck_count += 1
                            print(f"Stuck detected! Count: {self.stuck_count}/2")
                            if self.stuck_count >= 2:
                                print("Auto-recovery: side-shift wiggle + ultra-high step...")
                                # 1. 侧移摇摆脱困（避免后退到后面石头上）
                                self.locomotion.set_motion(13)  # 左移
                                time.sleep(0.12)
                                self.locomotion.set_motion(14)  # 右移
                                time.sleep(0.12)
                                # 2. 极小后退（仅0.12s，一丁点距离）
                                self.locomotion.set_motion(24)
                                time.sleep(0.12)
                                # 3. 极限高抬腿前冲跨越（motion_id=25, step_height=1.00）
                                self.locomotion.set_motion(25)
                                time.sleep(0.5)
                                # 4. 恢复默认前进
                                self.stuck_count = 0
                                print("Recovery done, resuming forward.")
                        else:
                            self.stuck_count = 0
                        
                        self.last_dist = dist
                        self.last_check_time = current_time

                    # 3. 正常行走与纠偏
                    if boundary_steer != 0:
                        self.locomotion.set_motion(boundary_steer)
                        time.sleep(0.1)
                    elif offset > self.steering_threshold:
                        self.locomotion.set_motion(12)
                        time.sleep(0.1)
                    elif offset < -self.steering_threshold:
                        self.locomotion.set_motion(11)
                        time.sleep(0.1)
                    else:
                        self.locomotion.set_motion(self.motion_id)
                        time.sleep(0.05)

                    if self.target_distance > 0 and dist >= self.target_distance:
                        print(f"Target distance {self.target_distance} reached.")
                        break

                except Exception as e:
                    print(f"Detection error in Robust Walk: {str(e)}")
                    self.locomotion.set_motion(self.motion_id)
                    time.sleep(0.1)

        finally:
            self._ros2_manager.shutdown()

class Walking_Left_Turn(Basic_State):
    """
    前进左转（曲线）
    """
    def __init__(self, duration=0):
        super().__init__()
        self.name = "Walking Left Turn"
        self.motion_id = 18
        self.duration = duration


class Walking_Right_Turn(Basic_State):
    """
    前进右转（曲线）
    """
    def __init__(self, duration=0):
        super().__init__()
        self.name = "Walking Right Turn"
        self.motion_id = 15
        self.duration = duration


class Walking_Slope(Basic_State):
    """
    上坡下坡
    """
    def __init__(self, duration=0):
        super().__init__()
        self.name = "Walking Slope"
        self.motion_id = 19
        self.duration = duration


class Walking_High_Limiter(Basic_State):
    """
    限高杆行走
    """
    def __init__(self, duration=0):
        super().__init__()
        self.name = "Walking High Limiter"
        self.motion_id = 20
        self.duration = duration


class Spin_To_Next_Stage(Basic_State):
    """
    衔接转弯：向目标方向旋转直到看见第二赛段的直道
    """
    def __init__(self, duration=8):
        super().__init__()
        self.name = "Spin to Next Stage"
        self.duration = duration
        self.MIN_PATH_WIDTH = 80 # 判定进入赛道的宽度

    def execute(self):
        global global_turn_direction
        self.motion_id = 9 if global_turn_direction == 'left' else 10 # 9左转, 10右转
        print(f"Executing {self.name} turning {global_turn_direction} with feedback control...")
        from ...camera.path_scanner import path_scanner_main, CenterOffsetScanner
        
        self._ros2_manager.init()
        sim_clock = self._ros2_manager.get_clock()
        try:
            _start_time = sim_clock.get_sim_time()
            while _start_time is None:
                time.sleep(0.5)
                _start_time = sim_clock.get_sim_time()
            start_time = _start_time.nanosec / 1e9 + _start_time.sec
            
            align_count = 0
            while True:
                _current_time = sim_clock.get_sim_time()
                current_time = _current_time.nanosec / 1e9 + _current_time.sec
                if current_time - start_time >= self.duration: 
                    print(f"{self.name}: 旋转达到预设时间。")
                    break

                # 执行旋转
                self.locomotion.set_motion(self.motion_id)

                # 实时检测是否看到新赛道并严防踩线（降低 timeout 保证旋转平滑度）
                data = path_scanner_main(CenterOffsetScanner, timeout=0.2)
                if data:
                    le = data.get('left_edge')
                    re = data.get('right_edge')
                    center_x = 160
                    
                    # 转向踩线保护逻辑（针对转弯特性放宽阈值 35->20，防止误触导致转向不彻底）
                    if (le is not None and (center_x - le) < 20) or \
                       (re is not None and (re - center_x) < 20):
                        print(f"{self.name}: 转向中靠近边界，执行紧急规避侧移...")
                        # le 靠近则右移(14)，re 靠近则左移(13)
                        self.locomotion.set_motion(14 if (le is not None and (center_x - le) < 20) else 13)
                        time.sleep(0.3)
                        continue

                    # 鲁棒性增强：放宽对齐标准（宽度 60, 深度 50），确保能更快锁定新赛道
                    if le is not None and re is not None:
                        width = re - le
                        dist = data.get('center_distance', 0)
                        if width > 60 and dist > 50:
                            align_count += 1
                            if align_count >= 5: 
                                print(f"{self.name}: 成功对齐下一赛段(Width:{width}, Depth:{dist})。")
                                break
                    else:
                        align_count = 0
                else:
                    align_count = 0
                time.sleep(0.01)
        finally:
            self._ros2_manager.shutdown()


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
        self.steering_threshold = 8  # 降低阈值，精细纠偏防止踩线
        self.safe_margin = 45
        self.critical_margin = 35    # 增加紧急纠正阈值，严防死守

        # 卡死检测参数
        self.stuck_check_interval = 5.0
        self.stuck_dist_threshold = 3
        self.last_dist = -1
        self.last_check_time = 0
        self.stuck_count = 0
        self.ball_confirm_count = 0  # 衔接防抖

        self._lidar_cache = None
        self._lidar_cache_time = 0
        self._lidar_cooldown = 0.5

        self.obstacle_count = 0
        self.obstacle_persistence = 0   # 帧级持久化
        self.MIN_PATH_WIDTH = 75        # 真实路径宽度阈值
        self.RECOVERY_SIDE_STEP = 0.12  
        self.exit_confidence = 0        # 出口检测置信度计数（帧数）
        self.exit_confirmed = False     # 确认出口标志
        self.exit_forward_timer = 0     # 确认出口后的直行计时器

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
                    # 统一阈值：优先使用 scanner 原始标志，或者对齐 0.22 下限
                    stone_warning = lidar_data.get('stone_warning', (0.22 <= front_dist < 0.35))
                    
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

                    # === 2026 衔接增强：非阻塞高优蓝球检测 ===
                    try:
                        from ...camera.ball_scanner import ball_scanner_main
                        ball_data = ball_scanner_main(timeout=0.15, target_color='blue')
                        if ball_data:
                            if ball_data.get('target_detected'):
                                ball_dist = ball_data.get('distance', 100)
                                # 距离阈值放宽至 160，尽早发现蓝球切入第二赛段
                                if ball_dist < 160 and not on_stone_zone: 
                                    self.ball_confirm_count += 1
                                    if self.ball_confirm_count >= 2: # 降低连续确认帧数
                                        print(f"{self.name}: 提前捕捉到第二赛段蓝球 (PixelDist:{ball_dist})，切入第二赛段逻辑！")
                                        if not self.exit_confirmed:
                                            self.exit_confirmed = True
                                            self.exit_forward_timer = current_time
                                            break # 强制跳出循环
                                else:
                                    self.ball_confirm_count = max(0, self.ball_confirm_count - 1)
                            else:
                                self.ball_confirm_count = 0
                    except Exception:
                        pass

                    data = path_scanner_main(CenterOffsetScanner, timeout=0.2) # 将路径扫描 timeout 降至 0.2
                    left_edge = data.get('left_edge') if data else None
                    right_edge = data.get('right_edge') if data else None
                    dist = data.get('center_distance', 0)
                    path_width = (right_edge - left_edge) if (left_edge is not None and right_edge is not None) else 0
                    has_real_track = (path_width > self.MIN_PATH_WIDTH)

                    if left_edge is None and right_edge is None:
                        # 衔接增强：如果在补位中丢线是正常的；否则尝试慢速找回
                        if self.exit_confirmed:
                            self.locomotion.set_motion(5) # 补位期间用平地行走
                        else:
                            # 没确认出口却丢了线，尝试慢速左转找回（大多数弯道是左转）
                            self.locomotion.set_motion(11)
                        time.sleep(0.1)
                        continue

                    offset = data.get('center_offset', 0)
                    center_x = 160

                    # === 3. 出口检测与衔接 (核心逻辑：判定方向 + 开启补位) ===
                    if not self.exit_confirmed:
                        is_at_exit_frame = False
                        global global_turn_direction
                        # 2026 最终严谨版：只有在非石头区（LiDAR清空）且运行足够长时间后，才允许确认出口
                        # 这能防止在石板路上因姿态颠簸导致的误判
                        if not on_stone_zone and (current_time - start_time > 15.0):
                            # 哪边没有边界就往哪边转
                            if left_edge is None and right_edge is not None:
                                is_at_exit_frame = True
                                global_turn_direction = 'left'
                            elif right_edge is None and left_edge is not None:
                                is_at_exit_frame = True
                                global_turn_direction = 'right'
                        
                        if is_at_exit_frame:
                            self.exit_confidence += 1
                            if self.exit_confidence >= 10: # 提高置信度阈值
                                print(f"{self.name}: 出口确认(Direction:{global_turn_direction}) -> 启动 1.6s 衔接补位")
                                self.exit_confirmed = True
                                self.exit_forward_timer = current_time
                        else:
                            self.exit_confidence = max(0, self.exit_confidence - 1)

                    # === 4. 补位结束判定 ===
                    if self.exit_confirmed:
                        # 衔接优化：缩短补位时间（1.6s -> 1.0s），防止狗头顶到 T 字路口对面边界
                        if (current_time - self.exit_forward_timer >= 1.0) or front_blocked:
                            print(f"{self.name}: 衔接区到达，准备转向。")
                            break

                    # === 5. 紧急避障 (严防死守，哪怕在出口确认期也要保护另一侧边界) ===
                    is_too_close_left = (left_edge is not None and (center_x - left_edge) < self.critical_margin)
                    is_too_close_right = (right_edge is not None and (right_edge - center_x) < self.critical_margin)

                    if is_too_close_left or is_too_close_right:
                        # 只有在强障碍区（持久化计数高）且无明确轨道时才忽略
                        if on_stone_zone and self.obstacle_persistence > 15 and not has_real_track:
                            pass 
                        else:
                            # 真实边界纠偏：平地、过渡段或明确边界
                            print(f"{self.name}: 边界预警 -> 执行纠偏(L:{left_edge}, R:{right_edge}, Width:{path_width})")
                            if is_too_close_left:
                                self.locomotion.set_motion(14) # 右偏
                            else:
                                self.locomotion.set_motion(13) # 左偏
                            time.sleep(0.12)
                            continue

                    # === 6. 正常跨越与闭环纠偏 ===
                    # [DEBUG]
                    if not hasattr(self, '_dbg_n'): self._dbg_n = 0
                    self._dbg_n += 1
                    if self._dbg_n % 10 == 0:
                        print(f"[DBG] front_dist={front_dist:.2f} blocked={front_blocked} warn={stone_warning} persist={self.obstacle_persistence} on_stone={on_stone_zone}")
                    if front_blocked or stone_warning:
                        self.obstacle_count += 1
                        if self.obstacle_count >= 2:
                            print(f"LiDAR: 前方障碍 -> 跨越！ motion={'25' if front_blocked else '22'}")
                            self.locomotion.set_motion(self.extreme_step_motion if front_blocked else self.high_step_motion)
                            time.sleep(0.6)
                            self.obstacle_count = 0
                            continue
                    else:
                        self.obstacle_count = 0
                        # 2026 衔接增强：非石头区强制使用 motion 5 (0.15m/s) 以获得更好的稳定性
                        active_default = 5 if not on_stone_zone else self.default_motion
                        
                        if offset > self.steering_threshold: self.locomotion.set_motion(12); time.sleep(0.08)
                        elif offset < -self.steering_threshold: self.locomotion.set_motion(11); time.sleep(0.08)
                        else: self.locomotion.set_motion(active_default); time.sleep(0.05)

                    if current_time - self.last_check_time > self.stuck_check_interval:
                        # 检查是否有位置进度或视觉距离变化
                        is_stuck = False
                        if self.last_dist != -1:
                            dist_change = abs(dist - self.last_dist)
                            # 在石头区放宽判定，非石头区严判定
                            threshold = self.stuck_dist_threshold if on_stone_zone else 1
                            if dist_change < threshold:
                                is_stuck = True
                        
                        if is_stuck:
                            self.stuck_count += 1
                            print(f"{self.name}: Stuck detected! Count: {self.stuck_count}/3")
                            if self.stuck_count >= 3:
                                print(f"{self.name}: 执行石板路专项脱困 - 高抬腿冲击...")
                                # 1. 极小后退（仅 0.2s，拉开一点距离重新冲刺）
                                self.locomotion.set_motion(24)
                                time.sleep(0.2)
                                # 2. 极限高抬腿前冲跨越（motion_id=25）
                                self.locomotion.set_motion(25)
                                time.sleep(0.8)
                                self.stuck_count = 0
                        else:
                            self.stuck_count = 0
                        
                        self.last_dist = dist
                        self.last_check_time = current_time

                except Exception as e:
                    time.sleep(0.1)
        finally:
            self._ros2_manager.shutdown()
