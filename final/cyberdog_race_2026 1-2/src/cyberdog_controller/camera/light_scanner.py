import rclpy 
import time
import sys
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
# from pyzbar.pyzbar import decode
import os

class LightScanner(Node):
    def __init__(self, done_callback):
        super().__init__('light_scanner')
        qos_profile = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.subscription = self.create_subscription(
            Image,
            '/rgb_camera/image_raw',
            self.scan_light,
            qos_profile
        )
        self.bridge = CvBridge()
        self.done_callback = done_callback  # 回调通知 main
        self.get_logger().info('Light Scanner Node has been started.')

    def scan_light(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            height, width = cv_image.shape[:2]

            # === 1. 找背景板 ===
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            lower_green = np.array([20, 100, 100])  
            upper_green = np.array([30, 255, 255])
            mask_green = cv2.inRange(hsv, lower_green, upper_green)
            mask_green[height // 2:, :] = 0 
            kernel = np.ones((5, 5), np.uint8)
            mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_CLOSE, kernel)
            contours, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            board_contour = None
            for c in sorted(contours, key=cv2.contourArea, reverse=True):
                x, y, w, h = cv2.boundingRect(c)
                if (w/h > 1.2) and (w > 20) and (h > 20):
                    board_contour = c
                    break
            x, y, w, h = cv2.boundingRect(board_contour)
            board_roi = cv_image[y:y+h, x:x+w]

            # === 2. 检测黄色灯光 ===
            hsv_roi = cv2.cvtColor(board_roi, cv2.COLOR_BGR2HSV)
            lower_yellow = np.array([25, 150, 150])  
            upper_yellow = np.array([30, 255, 255])
            mask_yellow = cv2.inRange(hsv_roi, lower_yellow, upper_yellow)
            kernel = np.ones((3,3), np.uint8)
            mask_yellow = cv2.morphologyEx(mask_yellow, cv2.MORPH_OPEN, kernel)
            light_contours, _ = cv2.findContours(mask_yellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            light_detected = False
            light_rects = []  
            light_area = 0 
            for cnt in light_contours:
                area = cv2.contourArea(cnt)
                if area > 5000: 
                    x_light, y_light, w_light, h_light = cv2.boundingRect(cnt)
                    light_rects.append((x_light, y_light, w_light, h_light))
                    light_area = w_light * h_light
                    light_detected = True
            status = "Yellow Light On" if light_detected else "Light Off"

            # === 3. 可视化结果 ===
            output_img = cv_image.copy()
            cv2.rectangle(output_img, (x, y), (x+w, y+h), (0, 255, 0), 2)
            for rect in light_rects:
                x_light, y_light, w_light, h_light = rect
                cv2.rectangle(output_img, 
                            (x+x_light, y+y_light), 
                            (x+x_light+w_light, y+y_light+h_light),
                            (0, 255, 255), 2) 
            cv2.putText(output_img, f"{status}", (x, y-10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            save_path = os.path.join(os.getcwd(), f"detection_{timestamp}.jpg")
            cv2.imwrite(save_path, output_img)
            self.done_callback({
                "status": status,
                "light_area": light_area,
            })
            self.get_logger().info(f"status: {status},light_area: {light_area}")

        except Exception as e:
            self.get_logger().error(f"Detection error: {str(e)}")
            self.done_callback({"status": "Error"})
            
def yellow_light_main(args=None):
    rclpy.init(args=args)
    light_result = {}

    # 用于通知主线程退出
    from threading import Event
    done_event = Event()

    def on_done(data):
        light_result["data"] = data
        done_event.set()  # 通知主线程

    node = LightScanner(done_callback=on_done)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    try:
        # 用自定义循环替代 rclpy.spin
        start_time = time.time()
        while rclpy.ok() and not done_event.is_set():
            executor.spin_once(timeout_sec=0.1)
            if time.time() - start_time > 100:
                node.get_logger().info("Light Detected")
                break
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down by keyboard...')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

    # 获取光线数据
    if "data" in light_result:
        print("Data detected from Light Scanner: ", light_result["data"])
        return light_result["data"]
    else:
        print("No data detected!!!")
        return None
    
if __name__ == '__main__':
    rclpy.init()
    def test_callback(data):
        print("检测结果:", data) 
    node = LightScanner(done_callback=test_callback)
    if len(sys.argv) > 1:
        img_path = sys.argv[1]
        if not os.path.exists(img_path):
            print(f"Image file {img_path} not found!")
            sys.exit(1)
        cv_image = cv2.imread(img_path)
        if cv_image is None:
            print("Failed to load image.")
            sys.exit(1)
        bridge = CvBridge()
        ros_img_msg = bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        node.scan_light(ros_img_msg)
    else:
        print("Usage: python light_scanner.py <image_path>")
    node.destroy_node()
    rclpy.shutdown()
