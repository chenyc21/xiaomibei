"""
终点圆圈检测器 - Stage 6 第六赛段专用
检测终点区域的圆圈（用于精确对位）
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


class FinishCircleScanner(Node):
    """
    终点圆圈检测器：识别终点区域圆圈
    根据竞赛规则，终点是一个标记区域（通常是特定颜色的圆圈或方框）
    这里实现基于颜色和形状识别的终点检测
    """
    def __init__(self, done_callback=lambda x: None, target_color='red'):
        super().__init__("finish_circle_scanner")
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        
        self.sub = self.create_subscription(Image, '/rgb_camera/image_raw', self.image_callback, qos)
        self.bridge = CvBridge()
        self.done_callback = done_callback
        self.center_x = 0
        self.img_height = 0
        self.frame_count = 0
        self.target_color = target_color
        
        # 颜色范围定义（HSV）
        # 红色范围（终点标记常见为红色）
        self.lower_red1 = np.array([0, 100, 100])
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([170, 100, 100])
        self.upper_red2 = np.array([180, 255, 255])
        
        # 黄色范围（备选：赛道边界线）
        self.lower_yellow = np.array([15, 100, 100])
        self.upper_yellow = np.array([35, 255, 255])
        
        # 绿色范围（备选）
        self.lower_green = np.array([40, 100, 100])
        self.upper_green = np.array([80, 255, 255])

    def image_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.img_height, img_width = cv_img.shape[:2]
            self.center_x = img_width // 2
            
            if self.frame_count == 0:
                self.get_logger().info(f"Finish Circle Scanner: First frame received! size={cv_img.shape}")
            self.frame_count += 1
            
            # 转换为HSV色彩空间
            hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
            
            # 根据目标颜色选择掩码
            if self.target_color == 'red':
                mask1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
                mask2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
                mask = cv2.bitwise_or(mask1, mask2)
            elif self.target_color == 'yellow':
                mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
            elif self.target_color == 'green':
                mask = cv2.inRange(hsv, self.lower_green, self.upper_green)
            else:
                mask = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
            
            # 形态学操作
            kernel_close = np.ones((7, 7), np.uint8)
            kernel_open = np.ones((3, 3), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_close)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_open)
            
            # 轮廓检测
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if not contours:
                self.done_callback({'circle_detected': False})
                return
            
            # 找到最大的轮廓（最有可能是终点圆圈）
            largest_cnt = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_cnt)
            
            # 面积阈值：终点圆圈应该足够大
            if area < 100:
                self.done_callback({'circle_detected': False})
                return
            
            # 计算轮廓的中心
            M = cv2.moments(largest_cnt)
            if M["m00"] == 0:
                self.done_callback({'circle_detected': False})
                return
            
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            
            # 计算轮廓的圆度（判定是否接近圆形）
            perimeter = cv2.arcLength(largest_cnt, True)
            if perimeter == 0:
                self.done_callback({'circle_detected': False})
                return
            
            circularity = 4 * np.pi * area / (perimeter ** 2)
            
            # 计算到摄像头中心的偏移
            center_offset = cX - self.center_x
            
            # 计算距离（基于像素高度 y 坐标）
            distance = self.img_height - cY
            
            # 如果圆度满足要求或面积足够大，则判定为检测到圆圈
            if circularity > 0.3 or area > 500:
                self.done_callback({
                    'circle_detected': True,
                    'center_offset': center_offset,
                    'distance': distance,
                    'area': area,
                    'circularity': circularity,
                    'position': (cX, cY)
                })
            else:
                # 即使不完全圆形，如果足够大也认为是圆圈
                if area > 200:
                    self.done_callback({
                        'circle_detected': True,
                        'center_offset': center_offset,
                        'distance': distance,
                        'area': area,
                        'circularity': circularity,
                        'position': (cX, cY)
                    })
                else:
                    self.done_callback({'circle_detected': False})
            
        except Exception as e:
            self.get_logger().error(f"Finish Circle Scanner Error: {str(e)}")
            self.done_callback({'circle_detected': False})


def finish_circle_scanner_main(timeout=2.0, target_color='red'):
    """
    主函数：检测终点圆圈
    
    参数:
        timeout: 超时时间（秒）
        target_color: 目标颜色 ('red', 'yellow', 'green')
    
    返回:
        dict: {
            'circle_detected': bool,
            'center_offset': int,      # 像素偏移（正=右，负=左）
            'distance': int,            # 像素距离（越小越近）
            'area': float,              # 轮廓面积
            'circularity': float,       # 圆度 (0-1)
            'position': (x, y)          # 圆圈中心位置
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
    
    # 创建终点圆圈扫描器
    scanner = FinishCircleScanner(done_callback=on_detection, target_color=target_color)
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
        return {'circle_detected': False}


__all__ = ['FinishCircleScanner', 'finish_circle_scanner_main']
