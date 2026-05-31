import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import time


class PathScannerBase(Node):
    def __init__(self, node_name, done_callback):
        super().__init__(node_name)
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(Image, '/rgb_camera/image_raw', self.image_callback, qos)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, qos)
        self.latest_lidar = None
        self.bridge = CvBridge()
        self.done_callback = done_callback
        self.center_x = 0
        self.img_height = 0
        self.lower_yellow = np.array([15, 30, 40])
        self.upper_yellow = np.array([50, 255, 255])
        self.kernel_open = np.ones((2, 2), np.uint8)
        self.kernel_close = np.ones((5, 5), np.uint8)
        self.frame_count = 0

    def lidar_callback(self, msg):
        self.latest_lidar = msg

    def image_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.img_height, self.center_x = cv_img.shape[0], cv_img.shape[1] // 2
            if self.frame_count == 0:
                self.get_logger().info(f"FIRST FRAME received! size={cv_img.shape}")
            self.frame_count += 1
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


class CenterOffsetScanner(PathScannerBase):
    def __init__(self, done_callback=lambda x: None):
        super().__init__("center_offset_scanner", done_callback)

    def specialized_detect(self, img, mask):
        h, w = mask.shape
        self.center_x = w // 2
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
                le = lp.max()
                all_offsets.append((le + 80) - self.center_x)
                left_edges.append(le)
            elif rp.size >= 2:
                re = rp.min()
                all_offsets.append((re - 80) - self.center_x)
                right_edges.append(re)

        if all_offsets:
            offset = int(np.mean(all_offsets))
            left_edge = int(np.mean(left_edges)) if left_edges else None
            right_edge = int(np.mean(right_edges)) if right_edges else None
        else:
            offset, left_edge, right_edge = 0, None, None

        center_intersection, center_dist = self._find_center_end(mask)

        if left_edge is None and right_edge is None:
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                largest = max(contours, key=cv2.contourArea)
                if cv2.contourArea(largest) > 100:
                    M = cv2.moments(largest)
                    if M["m00"] > 0:
                        mid_x = int(M["m10"] / M["m00"])
                        offset = mid_x - self.center_x
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
        center_line = mask[:, self.center_x]
        white_pixels = np.where(center_line == 255)[0]
        if white_pixels.size == 0:
            return (self.center_x, mask.shape[0] - 1), 0
        return (self.center_x, white_pixels[-1]), mask.shape[0] - 1 - white_pixels[-1]


class BridgeScanner(PathScannerBase):
    """
    独木桥检测器：估计桥面中心偏移，并检测桥面虚线。
    第一版只依赖前视 RGB，失败时返回 bridge_detected=False 让动作层走保守 fallback。
    """
    def __init__(self, done_callback=lambda x: None):
        super().__init__("bridge_scanner", done_callback)
        self.lower_bridge_yellow = np.array([12, 35, 35])
        self.upper_bridge_yellow = np.array([55, 255, 255])
        self.lower_bridge_brown = np.array([5, 25, 25])
        self.upper_bridge_brown = np.array([30, 220, 190])
        self.lower_line_white = np.array([0, 0, 160])
        self.upper_line_white = np.array([180, 80, 255])

    def image_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.img_height, self.center_x = cv_img.shape[0], cv_img.shape[1] // 2
            self.frame_count += 1
            result = self.specialized_detect(cv_img, None)
            self.done_callback(result)
        except Exception as e:
            self.get_logger().error(f"Bridge Scanner Error: {str(e)}")

    def specialized_detect(self, img, mask):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        h, w = img.shape[:2]
        self.center_x = w // 2

        yellow_mask = cv2.inRange(hsv, self.lower_bridge_yellow, self.upper_bridge_yellow)
        brown_mask = cv2.inRange(hsv, self.lower_bridge_brown, self.upper_bridge_brown)
        bridge_mask = yellow_mask | brown_mask
        bridge_mask = cv2.morphologyEx(bridge_mask, cv2.MORPH_CLOSE, self.kernel_close)
        bridge_mask = cv2.morphologyEx(bridge_mask, cv2.MORPH_OPEN, self.kernel_open)

        offset, left_edge, right_edge, confidence = self._detect_bridge_center(bridge_mask)
        line_detected, line_y, line_confidence = self._detect_bridge_line(hsv, h, w)

        return {
            'image_width': w,
            'image_height': h,
            'bridge_detected': confidence > 0.0,
            'center_offset': offset,
            'left_edge': left_edge,
            'right_edge': right_edge,
            'bridge_confidence': confidence,
            'line_detected': line_detected,
            'line_y': line_y,
            'line_confidence': line_confidence,
            'line_distance': h - line_y if line_detected else None
        }

    def _detect_bridge_center(self, bridge_mask):
        h, w = bridge_mask.shape
        scan_rows = [h * 2 // 3, h * 3 // 4, h * 5 // 6]
        centers = []
        left_edges = []
        right_edges = []

        for scan_y in scan_rows:
            row = bridge_mask[scan_y, :]
            pixels = np.where(row == 255)[0]
            if pixels.size < max(8, w // 40):
                continue

            segments = self._continuous_segments(pixels)
            if not segments:
                continue
            left, right = max(segments, key=lambda item: item[1] - item[0])
            if right - left < max(8, w // 30):
                continue

            centers.append((left + right) // 2)
            left_edges.append(left)
            right_edges.append(right)

        if not centers:
            return 0, None, None, 0.0

        center = int(np.mean(centers))
        left_edge = int(np.mean(left_edges))
        right_edge = int(np.mean(right_edges))
        avg_width = max(1, int(np.mean(np.array(right_edges) - np.array(left_edges))))
        confidence = min(1.0, len(centers) / len(scan_rows) * avg_width / max(1, w * 0.35))
        return center - self.center_x, left_edge, right_edge, confidence

    def _detect_bridge_line(self, hsv, h, w):
        white_mask = cv2.inRange(hsv, self.lower_line_white, self.upper_line_white)
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, np.ones((3, 9), np.uint8))
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, np.ones((2, 4), np.uint8))

        search_top = h // 3
        row_counts = np.sum(white_mask[search_top:, :] == 255, axis=1)
        min_pixels = max(18, w // 8)
        candidate_rows = np.where(row_counts >= min_pixels)[0]
        if candidate_rows.size == 0:
            return False, None, 0.0

        best_local_y = int(candidate_rows[np.argmax(row_counts[candidate_rows])])
        line_y = search_top + best_local_y
        row = white_mask[line_y, :]
        pixels = np.where(row == 255)[0]
        segments = self._continuous_segments(pixels)
        if not segments:
            return False, None, 0.0

        total_line_width = sum(right - left + 1 for left, right in segments if right - left >= 4)
        confidence = min(1.0, total_line_width / max(1, w * 0.45))
        return confidence >= 0.25, line_y, confidence

    def _continuous_segments(self, pixels):
        if pixels.size == 0:
            return []

        splits = np.where(np.diff(pixels) > 1)[0] + 1
        groups = np.split(pixels, splits)
        return [(int(group[0]), int(group[-1])) for group in groups if group.size > 0]


class LidarObstacleScanner(Node):
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
        if self.scan_count == 1:
            result = self.analyze_scan(msg)
            self.done_callback(result)

    def analyze_scan(self, scan):
        if scan is None or len(scan.ranges) == 0:
            return {
                'front_blocked': False,
                'front_distance': float('inf'),
                'left_distance': float('inf'),
                'right_distance': float('inf'),
                'min_distance': float('inf'),
                'min_angle': 0.0,
                'clear_path': True,
                'scan_available': False
            }

        ranges = np.array(scan.ranges)
        valid_mask = np.isfinite(ranges) & (ranges > 0.01) & (ranges < scan.range_max)
        if not np.any(valid_mask):
            return {
                'front_blocked': False,
                'front_distance': float('inf'),
                'left_distance': float('inf'),
                'right_distance': float('inf'),
                'min_distance': float('inf'),
                'min_angle': 0.0,
                'clear_path': True,
                'scan_available': False
            }

        angles = np.linspace(scan.angle_min, scan.angle_max, len(ranges))

        def min_in_range(a_min, a_max):
            idx = np.where(valid_mask & (angles >= a_min) & (angles <= a_max))[0]
            if len(idx) == 0:
                return float('inf')
            return float(np.min(ranges[idx]))

        front_dist = min_in_range(-0.3, 0.3)
        left_dist = min_in_range(0.2, 1.2)
        right_dist = min_in_range(-1.2, -0.2)

        valid_ranges = ranges[valid_mask]
        valid_angles = angles[valid_mask]
        min_idx = np.argmin(valid_ranges)
        min_dist = float(valid_ranges[min_idx])
        min_angle = float(valid_angles[min_idx])

        front_blocked = front_dist < 0.25
        clear_path = front_dist >= 0.40

        return {
            'front_blocked': front_blocked,
            'front_distance': front_dist,
            'left_distance': left_dist,
            'right_distance': right_dist,
            'min_distance': min_dist,
            'min_angle': min_angle,
            'clear_path': clear_path,
            'scan_available': True
        }


_camera_checked = False
_camera_available = False


def check_camera_ready(timeout=3.0):
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

    return _camera_available


def reset_camera_check():
    global _camera_checked, _camera_available
    _camera_checked = False
    _camera_available = False


_vision_fail_count = 0
_VISION_FAIL_SUPPRESS = 3


def path_scanner_main(scanner_type: type, timeout=5.0, **kwargs):
    global _vision_fail_count

    result = {}
    event = threading.Event()

    def on_detection(data):
        result.update(data)
        event.set()

    from ..utils.ros2_manager import ros2_manager
    ros2_manager.init()

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
        return {}


def bridge_scanner_main(timeout=5.0):
    return path_scanner_main(BridgeScanner, timeout=timeout)


def lidar_scanner_main(timeout=2.0):
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
            'front_distance': float('inf'),
            'left_distance': float('inf'),
            'right_distance': float('inf'),
            'min_distance': float('inf'),
            'min_angle': 0.0,
            'clear_path': True,
            'scan_available': False
        }
    return result
