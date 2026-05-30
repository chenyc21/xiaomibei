#!/usr/bin/env python3
"""验证 RGB 摄像头是否正常发布图像"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
import time


class CameraChecker(Node):
    def __init__(self):
        super().__init__('camera_checker')
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(Image, '/rgb_camera/image_raw', self.callback, qos)
        self.frame_count = 0
        self.frame_time = None

    def callback(self, msg):
        if self.frame_count == 0:
            self.frame_time = time.time()
            self.get_logger().info(
                f"FIRST FRAME! size={msg.width}x{msg.height}, "
                f"encoding={msg.encoding}, frame_id={msg.header.frame_id}"
            )
        self.frame_count += 1


def main():
    rclpy.init()
    node = CameraChecker()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    print("等待摄像头图像 /rgb_camera/image_raw ... (超时 15 秒)")
    start = time.time()
    while node.frame_count == 0 and (time.time() - start) < 15:
        executor.spin_once(timeout_sec=0.5)

    if node.frame_count > 0:
        elapsed = time.time() - node.frame_time
        fps = node.frame_count / elapsed if elapsed > 0 else 0
        print(f"✓ 摄像头工作正常! {node.frame_count} 帧 / {elapsed:.1f}s = {fps:.1f} FPS")
    else:
        print("✗ 未收到任何摄像头图像。请确认:")
        print("  1. gazebo.xacro 中已添加摄像头传感器插件")
        print("  2. 仿真环境已正确启动 (ros2 launch cyberdog_gazebo race_gazebo.launch.py)")
        print("  3. 仿真未暂停 (取消 paused:=true)")

    executor.remove_node(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
