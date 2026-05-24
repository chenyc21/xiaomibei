"""
足球检测器 - Stage 6 第六赛段专用
检测黑白足球（青少年4号足球）
黑白配色特征：由黑色和白色的五边形组成
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import time


class FootballScanner(Node):
    """
    足球检测器：识别黑白足球的位置和距离
    使用对比度检测和形状识别
    """
    def __init__(self, done_callback=lambda x: None):
        super().__init__("football_scanner")
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        
        self.sub = self.create_subscription(Image, '/rgb_camera/image_raw', self.image_callback, qos)
        self.bridge = CvBridge()
        self.done_callback = done_callback
        self.center_x = 0
        self.img_height = 0
        self.frame_count = 0
        
        # 黑白足球颜色范围
        # 黑色范围
        self.lower_black = np.array([0, 0, 0])
        self.upper_black = np.array([180, 100, 50])
        
        # 白色范围
        self.lower_white = np.array([0, 0, 150])
        self.upper_white = np.array([180, 50, 255])

    def image_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.img_height, img_width = cv_img.shape[:2]
            self.center_x = img_width // 2
            
            if self.frame_count == 0:
                self.get_logger().info(f"Football Scanner: First frame received! size={cv_img.shape}")
            self.frame_count += 1
            
            # 转换为HSV色彩空间
            hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
            
            # 提取黑色和白色区域
            mask_black = cv2.inRange(hsv, self.lower_black, self.upper_black)
            mask_white = cv2.inRange(hsv, self.lower_white, self.upper_white)
            
            # 合并黑白掩码（足球由黑白块组成）
            mask_combined = cv2.bitwise_or(mask_black, mask_white)
            
            # 形态学操作：闭运算（填充孔洞）→ 开运算（去除噪声）
            kernel_close = np.ones((5, 5), np.uint8)
            kernel_open = np.ones((3, 3), np.uint8)
            mask_combined = cv2.morphologyEx(mask_combined, cv2.MORPH_CLOSE, kernel_close)
            mask_combined = cv2.morphologyEx(mask_combined, cv2.MORPH_OPEN, kernel_open)
            
            # 轮廓检测
            contours, _ = cv2.findContours(mask_combined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if not contours:
                self.done_callback({'football_detected': False})
                return
            
            # 找到最大的轮廓（最有可能是足球）
            largest_cnt = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_cnt)
            
            # 面积阈值：足球占据一定比例的像素
            if area < 50:
                self.done_callback({'football_detected': False})
                return
            
            # 计算轮廓的中心和圆度
            M = cv2.moments(largest_cnt)
            if M["m00"] == 0:
                self.done_callback({'football_detected': False})
                return
            
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            
            # 计算轮廓的圆度（圆形度越接近1，越接近圆）
            perimeter = cv2.arcLength(largest_cnt, True)
            if perimeter == 0:
                self.done_callback({'football_detected': False})
                return
            
            circularity = 4 * np.pi * area / (perimeter ** 2)
            
            # 圆度检验：足球应该相对圆形（0.6以上）
            if circularity < 0.4:
                self.done_callback({'football_detected': False})
                return
            
            # 计算到摄像头中心的偏移
            center_offset = cX - self.center_x
            
            # 计算距离（基于像素高度 y 坐标）
            distance = self.img_height - cY
            
            self.done_callback({
                'football_detected': True,
                'center_offset': center_offset,
                'distance': distance,
                'area': area,
                'circularity': circularity,
                'position': (cX, cY)
            })
            
        except Exception as e:
            self.get_logger().error(f"Football Scanner Error: {str(e)}")
            self.done_callback({'football_detected': False})


def football_scanner_main(timeout=2.0):
    """
    主函数：检测足球
    
    参数:
        timeout: 超时时间（秒）
    
    返回:
        dict: {
            'football_detected': bool,
            'center_offset': int,      # 像素偏移（正=右，负=左）
            'distance': int,            # 像素距离（越小越近）
            'area': float,              # 轮廓面积
            'circularity': float,       # 圆度 (0-1)
            'position': (x, y)          # 足球中心位置
        }
    """
    result = {}
    event = threading.Event()
    
    def on_detection(data):
        result.update(data)
        event.set()
    
    # 初始化 ROS2
    try:
        from ..utils.ros2_manager import ros2_manager
        ros2_manager.init()
    except Exception:
        pass
    
    # 创建足球扫描器
    scanner = FootballScanner(done_callback=on_detection)
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
        return result
    else:
        return {'football_detected': False}


__all__ = ['FootballScanner', 'football_scanner_main']
