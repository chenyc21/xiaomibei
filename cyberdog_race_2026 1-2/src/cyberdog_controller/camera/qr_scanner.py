import rclpy 
import time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
# from pyzbar.pyzbar import decode
import os
import easyocr

class QRScanner(Node):
    def __init__(self, done_callback, scanner="pyzbar"):
        super().__init__('qr_scanner')
        qos_profile = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)

        if scanner == "wechat":
            self.scanner_method = self.scan_QR_code_wechat
        elif scanner == "pyzbar":
            self.scanner_method = self.scan_QR_code
        else:
            raise ValueError("No such scanner method!")

        self.subscription = self.create_subscription(
            Image,
            '/rgb_camera/image_raw',
            self.scanner_method,
            qos_profile
        )
        self.bridge = CvBridge()
        self.done_callback = done_callback  # 回调通知 main
        self.get_logger().info('Image Subscriber Node has been started.')

    def scan_QR_code(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            cv_image = cv_image[0:100, 0:300]
            cv_image = cv2.resize(cv_image, None, fx=2.5, fy=2.5, interpolation=cv2.INTER_CUBIC)

            kernel_sharpening = np.array([
                [-1, -1, -1],
                [-1,  9, -1],
                [-1, -1, -1]
            ])
            cv_image = cv2.filter2D(cv_image, -1, kernel_sharpening)

            cv_image = cv2.convertScaleAbs(cv_image, alpha=3, beta=50)

            # 可视化
            cv2.imshow("QR Code Scanner", cv_image)
            cv2.waitKey(1)

            decoded_objects = decode(cv_image)
            if not decoded_objects:
                self.get_logger().info("No QR code detected.")
                return

            for obj in decoded_objects:
                qr_data = obj.data.decode('utf-8')
                self.get_logger().info(f"QR Code Detected: {qr_data}")

                # 成功识别后执行回调，传递数据并关闭节点
                self.done_callback(qr_data)
                return

        except Exception as e:
            self.get_logger().error(f"Failed to process image for QR code: {e}")

    def scan_QR_code_wechat(self, msg):
        current_dir = os.path.dirname(os.path.abspath(__file__))
        model_dir = os.path.join(current_dir, "qr_detector")

        try:
            detector = cv2.wechat_qrcode_WeChatQRCode(
                os.path.join(model_dir, "detect.prototxt"),
                os.path.join(model_dir, "detect.caffemodel"),
                os.path.join(model_dir, "sr.prototxt"),
                os.path.join(model_dir, "sr.caffemodel")
            )
        except Exception as e:
            self.get_logger().error("Failed to initialize Wechat Detector!")
            print(e)
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')

            cv2.imshow("QR Code Scanner", cv_image)
            cv2.waitKey(1)

            res, _ = detector.detectAndDecode(cv_image)

            if not res:
                self.get_logger().info("No QR code detected.")
                return

            for obj in res:
                self.get_logger().info(f"QR Code Detected: {res}")
                self.done_callback(res)
                return

        except Exception as e:
            self.get_logger().error(f"Failed to process image for QR code: {e}")


class OCRScanner(Node):
    def __init__(self, done_callback):
        super().__init__('ocr_scanner')
        self.subscription = self.create_subscription(
            Image,
            '/rgb_camera/image_raw',
            self.scan_ocr,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        self.bridge = CvBridge()
        self.done_callback = done_callback  # 回调通知 main
        self.reader = easyocr.Reader(['en'])
        self.get_logger().info('OCR Scanner Node has been started.')

    def scan_ocr(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            results = self.reader.readtext(cv_image)

            if not results:
                self.get_logger().info("No text detected.")
                return

            for (bbox, text, prob) in results:
                if prob > 0.5:  # 置信度阈值
                    self.get_logger().info(f"Detected text: {text}")
                    self.done_callback(text)
                    return

        except Exception as e:
            self.get_logger().error(f"Failed to process image for OCR: {e}")

        

# main 函数
def qr_scanner_main(args=None):
    rclpy.init(args=args)
    qr_result = {}

    # 用于通知主线程退出
    from threading import Event
    done_event = Event()

    def on_done(data):
        qr_result["data"] = data
        done_event.set()  # 通知主线程

    node = QRScanner(done_callback=on_done, scanner="wechat")
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    try:
        # 用自定义循环替代 rclpy.spin
        start_time = time.time()
        while rclpy.ok() and not done_event.is_set():
            executor.spin_once(timeout_sec=1)
            if time.time() - start_time > 10:
                node.get_logger().info("QR Code Detected")
                break
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down by keyboard...')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

    # 获取二维码数据
    if "data" in qr_result:
        print("Data detected from QR Code: ", qr_result["data"])
        return qr_result["data"][0]
    
    # 无二维码，则需要OCR识别
    ocr_node = OCRScanner(done_callback=on_done)
    ocr_executor = rclpy.executors.SingleThreadedExecutor()
    ocr_executor.add_node(ocr_node)

    try:
        start_time = time.time()
        while rclpy.ok() and not done_event.is_set():
            ocr_executor.spin_once(timeout_sec=1)
            if time.time() - start_time > 10:
                ocr_node.get_logger().info("No QR Code Detected, trying OCR")
                break
    except KeyboardInterrupt:
        ocr_node.get_logger().info('Shutting down OCR Scanner by keyboard...')
    finally:
        ocr_node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

    if "data" in qr_result:
        print("Data detected from OCR: ", qr_result["data"])
        return qr_result["data"][0]

    print("No data detected!!!")
    return None

if __name__ == '__main__':
    result = qr_scanner_main()
    if result:
        # 在这里使用 QR 码内容
        print("Result pass to main func: ", result)
