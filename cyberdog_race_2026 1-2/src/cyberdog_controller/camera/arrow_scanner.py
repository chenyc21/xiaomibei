import rclpy 
import time
import sys
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os

class ArrowScanner(Node):
    def __init__(self, done_callback):
        super().__init__('arrow_scanner')
        qos_profile = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.subscription = self.create_subscription(
            Image,
            '/rgb_camera/image_raw',
            self.scan_arrow,
            qos_profile
        )
        self.bridge = CvBridge()
        self.done_callback = done_callback  # 回调通知 main
        self.get_logger().info('Arrow Scanner Node has been started.')

    def scan_arrow(self, msg):
        try:
            # 转换ROS图像消息为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 提取左半部分图像 (宽度的一半)
            height, width = cv_image.shape[:2]
            left_half = cv_image[:, :width]
            
            # 转换到HSV颜色空间以便更好地检测绿色
            hsv = cv2.cvtColor(left_half, cv2.COLOR_BGR2HSV)
            
            # 定义绿色的HSV范围 (调整这些值以适应实际光照条件)
            lower_green = np.array([40, 40, 40])
            upper_green = np.array([80, 255, 255])
            
            # 创建绿色掩码
            mask = cv2.inRange(hsv, lower_green, upper_green)
            
            # 形态学操作去除噪声
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            
            # 分析垂直扫描线
            scan_results = []
            prev_length = 0
            
            # 从左到右扫描图像
            for col in range(mask.shape[1]):
                column = mask[:, col]
                
                # 找到绿色区域的连续线段
                in_segment = False
                current_length = 0
                max_length = 0
                
                for pixel in column:
                    if pixel == 255:  # 绿色像素
                        current_length += 1
                        if not in_segment:
                            in_segment = True
                    else:
                        if in_segment:
                            if current_length > max_length:
                                max_length = current_length
                            current_length = 0
                        in_segment = False
                
                # 检查最后的线段
                if in_segment and current_length > max_length:
                    max_length = current_length
                if max_length > 0:
                    scan_results.append(max_length)
            
            # 分析扫描结果
            significant_changes_index = 0
            CHANGE_THRESHOLD = 1  # 长度变化的阈值
            print(f"lengths: {len(scan_results)},scan_results: {scan_results}")
            for i in range(1, len(scan_results)):
                # 检测连续两列之间的显著变化
                if abs(scan_results[i] - scan_results[i-1]) > CHANGE_THRESHOLD:
                    CHANGE_THRESHOLD = abs(scan_results[i] - scan_results[i-1])
                    print(f"Significant change detected between columns {i-1} and {i}: {scan_results[i-1]} -> {scan_results[i]}")   
                    significant_changes_index = i
            
            # 判断箭头方向
            if significant_changes_index > len(scan_results)//2:  # 检测到多个显著变化
                arrow_direction = "slope"
            else:
                arrow_direction = "stone"
            
            # 调用回调函数并设置完成标志
            self.get_logger().info(f"Detected arrow direction: {arrow_direction}")
            self.done_callback(arrow_direction)
            self.detected = True
        
        except Exception as e:
            self.get_logger().error(f"Error in scan_arrow: {str(e)}")
        

def arrow_scanner_main(args = None):
    rclpy.init(args=args)
    arrow_result = {}

    from threading import Event
    done_event = Event()

    def on_done(data):
        arrow_result["data"] = data
        done_event.set()

    node = ArrowScanner(done_callback=on_done)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    try:
        start_time = time.time()
        while rclpy.ok() and not done_event.is_set():
            executor.spin_once(timeout_sec=0.1)
            if time.time() - start_time > 100:
                node.get_logger().info("Arrow Detected")
                break
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down by keyboard...')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

    # 获取箭头数据
    if "data" in arrow_result:
        print("Data detected from Arrow Scanner: ", arrow_result["data"])
        return arrow_result["data"]
    else:
        print("No data detected from Arrow Scanner!!!")
        return None

if __name__ == "__main__":
    result = arrow_scanner_main(lambda x: print(f"Arrow Scanner Result: {x}"))
    if result:
        print("Arrow Scanner executed successfully.")
