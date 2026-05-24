import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import time

# ===== 基础类 =====
class PathScannerBase(Node):
    def __init__(self, node_name, done_callback):
        super().__init__(node_name)
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)

        # 订阅主要相机话题（与去年比赛一致）
        self.sub = self.create_subscription(Image, '/rgb_camera/image_raw', self.image_callback, qos)

        # 激光雷达备选
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, qos)
        self.latest_lidar = None

        self.bridge = CvBridge()
        self.done_callback = done_callback
        self.center_x = 0
        self.img_height = 0

        # 预处理参数 (2026 最终加强版：极大化黄色感光，适应低分辨率)
        self.lower_yellow = np.array([15, 30, 40])
        self.upper_yellow = np.array([50, 255, 255])
        self.kernel_open = np.ones((2,2), np.uint8)
        self.kernel_close = np.ones((5,5), np.uint8)

        # 帧计数器（调试用）
        self.frame_count = 0

    def lidar_callback(self, msg):
        self.latest_lidar = msg

    def image_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.img_height, self.center_x = cv_img.shape[0], cv_img.shape[1]//2

            if self.frame_count == 0:
                self.get_logger().info(f"FIRST FRAME received! size={cv_img.shape}")
            self.frame_count += 1

            # 预处理流水线
            hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel_close)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel_open)

            result = self.specialized_detect(cv_img, mask)
            self.done_callback(result)

        except Exception as e:
            self.get_logger().error(f"Processing error: {str(e)}")

    def specialized_detect(self, img, mask):
        raise NotImplementedError()

# ===== 专用检测器 =====
class FarDistanceScanner(PathScannerBase):
    """最远点距离检测"""
    def __init__(self, done_callback=lambda x: None):
        super().__init__("far_distance_scanner", done_callback)
    def specialized_detect(self, img, mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours: return {'far_distance': None}

        all_points = np.vstack([c.reshape(-1,2) for c in contours])
        min_y = np.min(all_points[:,1]) if all_points.size > 0 else self.img_height
        return {'far_distance': self.img_height - min_y}

class BottomLeftScanner(PathScannerBase):
    """底部左边界检测"""
    def __init__(self, done_callback=lambda x: None):
        super().__init__("bottom_left_scanner", done_callback)
    def specialized_detect(self, img, mask):
        img_height = mask.shape[0]
        bottom_row = img_height - 1
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        leftmost_x = None
        for contour in contours:
            for point in contour[:,0,:]:
                x, y = point
                if y == bottom_row:
                    leftmost_x = x if (leftmost_x is None or x < leftmost_x) else leftmost_x
        return {'left_bottom_distance': leftmost_x}

class SlopeScanner(PathScannerBase):
    def __init__(self, done_callback=lambda x: None):
        super().__init__("slope_scanner", done_callback)

    def specialized_detect(self, img, msk):
        gray_low=70
        gray_high=120
        blank_threshold=0.03
        min_ramp_height=20
        if len(img.shape) == 2:
            gray = img
        else:
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        h, w = gray.shape
        center_x = w // 2

        mask = np.zeros((h, w), dtype=np.uint8)
        mask[(gray >= gray_low) & (gray <= gray_high)] = 255

        ramp_top_y = None
        ramp_bottom_y = h - 1

        for y in range(0, h):
            row = mask[y, :]
            non_zero_count = np.count_nonzero(row)
            non_zero_ratio = non_zero_count / w
            if non_zero_ratio > blank_threshold:
                ramp_top_y = y
                break

        if ramp_top_y is None or ramp_top_y < min_ramp_height:
            return {
                'ramp_top_line': None,
                'ramp_top_mid': (center_x, 0),
                'center_offset': 0,
                'ramp_height': 0
            }

        for y in range(ramp_top_y, h):
            row = mask[y, :]
            non_zero_count = np.count_nonzero(row)
            if non_zero_count / w < 0.2:
                ramp_bottom_y = y
                break

        top_row = mask[ramp_top_y, :]
        gray_pixels = np.where(top_row > 0)[0]
        if gray_pixels.size == 0:
            left_edge = 0
            right_edge = w - 1
        else:
            left_edge = gray_pixels.min()
            right_edge = gray_pixels.max()

        mid_x = (left_edge + right_edge) // 2
        offset = mid_x - center_x
        ramp_height = ramp_bottom_y - ramp_top_y

        return {
            'ramp_top_line': (ramp_top_y, (left_edge, right_edge)),
            'ramp_top_mid': (mid_x, ramp_top_y),
            'center_offset': offset,
            'ramp_height': ramp_height
        }

class RightBoundaryScanner(PathScannerBase):
    """右边界距离检测"""
    def __init__(self, done_callback=lambda x: None):
        super().__init__("right_boundary_scanner", done_callback)
    def specialized_detect(self,img, mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours: return {'right_boundary': None}

        all_points = np.vstack([c.reshape(-1,2) for c in contours])
        right_points = all_points[all_points[:,0] > self.center_x]
        if right_points.size == 0: return {'right_boundary': None}

        min_x = np.min(right_points[:,0])
        return {'right_boundary': mask.shape[1] - min_x}

class CenterOffsetScanner(PathScannerBase):
    """中心线偏移检测器（2026 增强版：多行扫描 + 动态权重）"""
    def __init__(self, done_callback=lambda x: None):
        super().__init__("center_offset_scanner", done_callback)

    def specialized_detect(self, img, mask):
        h, w = mask.shape
        self.center_x = w // 2
        
        # 多行扫描（上、中、下），增加稳定性
        scan_rows = [h * 3 // 4, h * 5 // 6, h - 10]
        all_offsets = []
        left_edges = []
        right_edges = []

        for scan_y in scan_rows:
            scan_line = mask[scan_y, :]
            white_pixels = np.where(scan_line == 255)[0]

            lp = white_pixels[white_pixels < self.center_x]
            rp = white_pixels[white_pixels > self.center_x]

            le = lp.max() if lp.size > 0 else None
            re = rp.min() if rp.size > 0 else None
            
            if le is not None and re is not None:
                all_offsets.append((le + re) // 2 - self.center_x)
                left_edges.append(le)
                right_edges.append(re)
            elif lp.size >= 2:
                # 仅看到左边界：假设路径中心在左边界右侧 80 像素处
                le = lp.max()
                all_offsets.append((le + 80) - self.center_x)
                left_edges.append(le)
            elif rp.size >= 2:
                # 仅看到右边界：假设路径中心在右边界左侧 80 像素处
                re = rp.min()
                all_offsets.append((re - 80) - self.center_x)
                right_edges.append(re)

        if all_offsets:
            offset = int(np.mean(all_offsets))
            left_edge = int(np.mean(left_edges)) if left_edges else None
            right_edge = int(np.mean(right_edges)) if right_edges else None
        else:
            offset, left_edge, right_edge = 0, None, None

        # 检测中心线交点
        center_intersection, center_dist = self._find_center_end(mask)

        # 视觉检测辅助：如果行扫描全部失败，尝试使用最大轮廓的中点
        if left_edge is None and right_edge is None:
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                largest = max(contours, key=cv2.contourArea)
                if cv2.contourArea(largest) > 100:
                    M = cv2.moments(largest)
                    if M["m00"] > 0:
                        mid_x = int(M["m10"] / M["m00"])
                        offset = mid_x - self.center_x
                        # 找到有效轮廓，设置 has_track 标志需要的边缘数据
                        left_edge = mid_x - 10
                        right_edge = mid_x + 10

        return {
            'mid_point': (self.center_x + offset, h * 5 // 6),
            'center_offset': offset,
            'left_edge': left_edge,
            'right_edge': right_edge,
            'center_intersection': center_intersection,
            'center_distance': center_dist
        }

    def _find_center_end(self, mask):
        """中心线终点检测"""
        center_line = mask[:, self.center_x]
        white_pixels = np.where(center_line == 255)[0]
        if white_pixels.size == 0:
            return (self.center_x, mask.shape[0]-1), 0
        return (self.center_x, white_pixels[-1]), mask.shape[0]-1 - white_pixels[-1]


class HorizontalLineScanner(PathScannerBase):
    """水平线检测器（用于弯道对齐）"""
    def __init__(self, done_callback=lambda x: None):
        super().__init__("horizontal_line_scanner", done_callback)

    def specialized_detect(self, img, mask):
        try:
            if mask is None or mask.size == 0:
                return {'is_horizontal_line': -1, 'angle': None, 'points': None}

            if len(mask.shape) == 3:
                mask = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
            mask = mask.astype(np.uint8)
            img_h, img_w = mask.shape

            min_length = img_w * 0.20 # 降低阈值以更容易识别弯道线
            max_angle = 15
            bottom_margin = 10

            kernel = np.ones((5,5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            best_segment = None
            best_angle = 90
            max_length = 0

            for cnt in contours:
                epsilon = 0.02 * cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt, epsilon, True)

                for i in range(len(approx)):
                    pt1 = approx[i][0]
                    pt2 = approx[(i+1)%len(approx)][0]
                    x1, y1 = pt1
                    x2, y2 = pt2

                    dx = x2 - x1
                    dy = y2 - y1
                    length = np.hypot(dx, dy)
                    angle = np.degrees(np.arctan2(dy, dx))

                    if angle > 90: angle -= 180
                    elif angle < -90: angle += 180

                    if (abs(angle) > max_angle or length < min_length or max(y1, y2) > img_h - bottom_margin):
                        continue

                    if abs(angle) < abs(best_angle) or (abs(angle) == abs(best_angle) and length > max_length):
                        best_angle = angle
                        max_length = length
                        best_segment = (x1, y1, x2, y2)

            if best_segment:
                code = 3 if abs(best_angle) < 1.5 else (2 if best_angle > 0 else 1)
                return {
                    'is_horizontal_line': code,
                    'angle': float(best_angle),
                    'points': best_segment
                }
            return {'is_horizontal_line': -1}
        except Exception:
            return {'is_horizontal_line': -1}


class BlackBarScanner(PathScannerBase):
    """黑色限高杆识别"""
    def __init__(self, done_callback=lambda x: None):
        super().__init__("black_bar_scanner", done_callback)
        self.lower_black = np.array([0, 0, 0])
        self.upper_black = np.array([180, 255, 60])

    def image_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.img_height, self.center_x = cv_img.shape[0], cv_img.shape[1]//2

            hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, self.lower_black, self.upper_black)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel_open)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel_close)

            result = self.specialized_detect(cv_img, mask)
            self.done_callback(result)
        except Exception as e:
            self.get_logger().error(f"Processing error: {str(e)}")

    def specialized_detect(self, img, mask):
        try:
            if mask is None or mask.size == 0:
                return {'is_horizontal_bar': -1}

            hough_params = {
                'rho': 1,
                'theta': np.pi/180,
                'threshold': 40,
                'minLineLength': int(mask.shape[1]*0.2),
                'maxLineGap': 15
            }

            lines = cv2.HoughLinesP(mask, **hough_params)
            if lines is None:
                return {'is_horizontal_bar': -1}

            candidates = []
            for line in lines:
                x1, y1, x2, y2 = line[0]
                if x1 > x2: x1, x2 = x2, x1; y1, y2 = y2, y1
                dx = x2 - x1
                dy = y2 - y1
                angle = np.degrees(np.arctan2(dy, dx))
                if not (-10 <= angle <= 10): continue
                y_avg = (y1 + y2) / 2
                candidates.append({'angle': angle, 'y_avg': y_avg})

            if not candidates: return {'is_horizontal_bar': -1}

            best = max(candidates, key=lambda x: x['y_avg'])
            angle = best['angle']

            if abs(angle) < 1.1: return {'is_horizontal_bar': 3, 'angle': angle}
            elif angle >= 1.1: return {'is_horizontal_bar': 2, 'angle': angle}
            else: return {'is_horizontal_bar': 1, 'angle': angle}

        except Exception as e:
            self.get_logger().error(f"黑色限高杆检测异常: {str(e)}")
            return {'is_horizontal_bar': -1}


class CenterIntersectionScanner(PathScannerBase):
    """中心线交点检测器"""
    def __init__(self, done_callback=lambda x: None):
        super().__init__("center_intersection_scanner", done_callback)
        self.cluster_threshold = 5
        self.min_cluster_size = 3

    def specialized_detect(self, img, mask):
        if len(mask.shape) == 3:
            mask = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
        img_h, img_w = mask.shape
        center_x = img_w // 2

        result = {
            'center_intersection': (center_x, img_h-1),
            'center_distance': 0
        }

        center_line = mask[:, center_x]
        white_pixels = np.where(center_line == 255)[0]
        if white_pixels.size == 0:
            return result

        clusters = []
        current_cluster = []
        for y in reversed(np.sort(white_pixels)):
            if not current_cluster:
                current_cluster.append(y)
            else:
                if current_cluster[-1] - y <= self.cluster_threshold:
                    current_cluster.append(y)
                else:
                    if len(current_cluster) >= self.min_cluster_size:
                        clusters.append(current_cluster)
                    current_cluster = [y]
        if current_cluster:
            clusters.append(current_cluster)

        for cluster in clusters:
            if len(cluster) >= self.min_cluster_size:
                cluster_y = int(np.mean(cluster))
                result['center_intersection'] = (center_x, cluster_y)
                result['center_distance'] = img_h - 1 - cluster_y
                break

        return result


# ===== 球检测器 =====
class BallScanner(PathScannerBase):
    """2026 寻珠任务专用检测器 (优化版)"""
    def __init__(self, done_callback=lambda x: None, target_color='orange'):
        super().__init__("ball_scanner", done_callback)
        # 颜色范围定义
        self.target_color = target_color
        
        # 橙色范围优化
        self.lower_orange = np.array([0, 100, 100])
        self.upper_orange = np.array([25, 255, 255])
        
        # 蓝色范围（2026赛题规则：进入第二赛段后的首选目标）
        self.lower_blue = np.array([100, 150, 100])
        self.upper_blue = np.array([130, 255, 255])

    def image_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.img_height, self.center_x = cv_img.shape[0], cv_img.shape[1]//2
            hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)

            if self.target_color == 'blue':
                mask = cv2.inRange(hsv, self.lower_blue, self.upper_blue)
            else:
                mask = cv2.inRange(hsv, self.lower_orange, self.upper_orange)
                
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel_open)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel_close)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if contours:
                best_cnt = max(contours, key=cv2.contourArea)
                if cv2.contourArea(best_cnt) > 80: # 稍微降低面积阈值，增加远距离识别
                    M = cv2.moments(best_cnt)
                    if M["m00"] > 0:
                        cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])

                        self.done_callback({
                            'target_detected': True,
                            'center_offset': cX - self.center_x,
                            'distance': self.img_height - cY,
                            'area': cv2.contourArea(best_cnt),
                            'color': self.target_color
                        })
                        return

            self.done_callback({'target_detected': False})
        except Exception as e:
            self.get_logger().error(f"Ball Scanner Error: {str(e)}")


# ===== 摄像头健康检查 =====
_camera_checked = False
_camera_available = False


def check_camera_ready(timeout=3.0):
    """
    检查 /rgb_camera/image_raw 话题是否有发布者（即摄像头是否已配置并工作）。
    返回 True 表示摄像头可用，False 表示不可用。
    结果会缓存，避免重复检查。
    """
    global _camera_checked, _camera_available
    if _camera_checked:
        return _camera_available

    try:
        from ..utils.ros2_manager import ros2_manager
        ros2_manager.init()
    except Exception:
        pass

    result_event = threading.Event()

    class _CameraCheckNode(Node):
        def __init__(self):
            super().__init__('camera_check_node')
            qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.sub = self.create_subscription(Image, '/rgb_camera/image_raw', self._cb, qos)
            self._got_frame = False

        def _cb(self, msg):
            if not self._got_frame:
                self._got_frame = True
                result_event.set()

    checker = _CameraCheckNode()
    executor_check = rclpy.executors.SingleThreadedExecutor()
    executor_check.add_node(checker)

    start = time.time()
    while not result_event.is_set() and (time.time() - start) < timeout:
        executor_check.spin_once(timeout_sec=0.1)

    executor_check.remove_node(checker)
    checker.destroy_node()

    _camera_available = checker._got_frame
    _camera_checked = True

    if _camera_available:
        print("[Camera] /rgb_camera/image_raw 话题正常，视觉系统可用。")
    else:
        print("[Camera] 警告: 未收到 /rgb_camera/image_raw 图像！")
        print("[Camera] 请确认 gazebo.xacro 中已添加 RGB 摄像头传感器插件。")
        print("[Camera] 修复方法: 将 gazebo_with_camera.xacro 替换到容器内对应位置后重启仿真。")
        print("[Camera] 当前将在无视觉模式下运行（仅靠预编程动作）。")

    return _camera_available


def reset_camera_check():
    """重置摄像头检查缓存（仿真重启后调用）"""
    global _camera_checked, _camera_available
    _camera_checked = False
    _camera_available = False


# ===== 主函数（与去年一致的 SingleThreadedExecutor + spin_once 模式）=====
_vision_fail_count = 0
_VISION_FAIL_SUPPRESS = 3  # 前几次失败时打印，之后抑制输出


def path_scanner_main(scanner_type: type, timeout=5.0, **kwargs):
    """
    参数:
        scanner_type: 检测器类（必须继承自PathScannerBase）
        timeout: 超时时间（秒）
        **kwargs: 传递给扫描器构造函数的额外参数

    使用与去年比赛完全一致的可靠模式：
    创建新节点 → SingleThreadedExecutor → spin_once() 循环 → 回调触发 → 返回结果
    """
    global _vision_fail_count

    result = {}
    event = threading.Event()

    def on_detection(data):
        result.update(data)
        event.set()

    # 确保 rclpy 已初始化（通过 ROS2Manager）
    from ..utils.ros2_manager import ros2_manager
    ros2_manager.init()

    # 创建全新的扫描器节点和专属 executor（与去年一致）
    scanner = scanner_type(done_callback=on_detection, **kwargs)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(scanner)

    try:
        start_time = time.time()
        while not event.is_set() and (time.time() - start_time) < timeout:
            executor.spin_once(timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(scanner)
        scanner.destroy_node()

    if result:
        _vision_fail_count = 0
        return result
    else:
        _vision_fail_count += 1
        if _vision_fail_count <= _VISION_FAIL_SUPPRESS:
            print("未识别到路径信息。")
        elif _vision_fail_count == _VISION_FAIL_SUPPRESS + 1:
            print("未识别到路径信息。（后续相同提示已抑制）")
        # 如果摄像头一直没数据，建议检查
        if _vision_fail_count == 10:
            check_camera_ready(timeout=1.0)
        return {}


def ball_scanner_main(timeout=5.0, target_color='orange'):
    return path_scanner_main(BallScanner, timeout=timeout, target_color=target_color)


# ===== LiDAR 障碍物扫描器 =====
class LidarObstacleScanner(Node):
    """
    LiDAR 障碍物检测器（独立节点，仅订阅 /scan）
    用于石板路等场景的障碍物感知与避障。
    """
    def __init__(self, done_callback=lambda x: None):
        super().__init__("lidar_obstacle_scanner")
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, qos)
        self.done_callback = done_callback
        self.latest_scan = None
        self.scan_count = 0

    def lidar_callback(self, msg):
        self.latest_scan = msg
        self.scan_count += 1
        # 收到第一帧后立即回调
        if self.scan_count == 1:
            result = self.analyze_scan(msg)
            self.done_callback(result)

    def analyze_scan(self, scan):
        """
        分析 LaserScan 数据，返回前方障碍物信息。

        返回:
            front_blocked: 正前方是否被阻挡 (True/False)
            front_distance: 正前方最近距离 (m)
            left_distance: 左前方最小距离 (m)
            right_distance: 右前方最小距离 (m)
            min_distance: 前方180°范围内最小距离 (m)
            min_angle: 最小距离对应的角度 (rad)
            clear_path: 前方是否有至少0.5m的畅通路径
        """
        if scan is None or len(scan.ranges) == 0:
            return {
                'front_blocked': False,
                'stone_warning': False,
                'front_distance': float('inf'),
                'left_distance': float('inf'),
                'right_distance': float('inf'),
                'min_distance': float('inf'),
                'min_angle': 0.0,
                'clear_path': True,
                'scan_available': False
            }

        ranges = np.array(scan.ranges)
        # 过滤无效值 (inf, nan, 负值)
        valid_mask = np.isfinite(ranges) & (ranges > 0.01) & (ranges < scan.range_max)
        if not np.any(valid_mask):
            return {
                'front_blocked': False,
                'stone_warning': False,
                'front_distance': float('inf'),
                'left_distance': float('inf'),
                'right_distance': float('inf'),
                'min_distance': float('inf'),
                'min_angle': 0.0,
                'clear_path': True,
                'scan_available': False
            }

        angles = np.linspace(scan.angle_min, scan.angle_max, len(ranges))

        # 前方扇区定义 (角度偏移: 正=左, 负=右, 0=正前)
        front_range = (-0.3, 0.3)    # 正前方 ±17°
        left_range = (0.2, 1.2)      # 左前方 11°~69°
        right_range = (-1.2, -0.2)   # 右前方 -69°~-11°

        def min_in_range(a_min, a_max):
            idx = np.where(valid_mask & (angles >= a_min) & (angles <= a_max))[0]
            if len(idx) == 0:
                return float('inf')
            return float(np.min(ranges[idx]))

        front_dist = min_in_range(*front_range)
        left_dist = min_in_range(*left_range)
        right_dist = min_in_range(*right_range)

        # 全局最小距离
        valid_ranges = ranges[valid_mask]
        valid_angles = angles[valid_mask]
        min_idx = np.argmin(valid_ranges)
        min_dist = float(valid_ranges[min_idx])
        min_angle = float(valid_angles[min_idx])

        # 判定（针对石板路石头优化阈值）
        front_blocked = front_dist < 0.22     # 正前方22cm内有障碍→极限高抬腿
        stone_warning = 0.22 <= front_dist < 0.35  # 中距离石头预警→超高抬腿
        clear_path = front_dist >= 0.40       # 至少40cm畅通

        return {
            'front_blocked': front_blocked,
            'stone_warning': stone_warning,
            'front_distance': front_dist,
            'left_distance': left_dist,
            'right_distance': right_dist,
            'min_distance': min_dist,
            'min_angle': min_angle,
            'clear_path': clear_path,
            'scan_available': True
        }


def lidar_scanner_main(timeout=2.0):
    """
    调用 LiDAR 障碍物扫描器，返回前方障碍物分析结果。

    参数:
        timeout: 超时时间（秒）

    返回:
        dict: 障碍物分析结果（同 LidarObstacleScanner.analyze_scan 返回值）
    """
    result = {}
    event = threading.Event()

    def on_detection(data):
        result.update(data)
        event.set()

    from ..utils.ros2_manager import ros2_manager
    ros2_manager.init()

    scanner = LidarObstacleScanner(done_callback=on_detection)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(scanner)

    try:
        start_time = time.time()
        while not event.is_set() and (time.time() - start_time) < timeout:
            executor.spin_once(timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(scanner)
        scanner.destroy_node()

    if not result:
        return {
            'front_blocked': False,
            'stone_warning': False,
            'front_distance': float('inf'),
            'left_distance': float('inf'),
            'right_distance': float('inf'),
            'min_distance': float('inf'),
            'min_angle': 0.0,
            'clear_path': True,
            'scan_available': False
        }

    return result


if __name__ == '__main__':
    # 先检查摄像头
    check_camera_ready()
    result = path_scanner_main(CenterOffsetScanner, timeout=5.0)
    if result:
        print("Path scanner result:", result)
    else:
        print("No path detected.")
