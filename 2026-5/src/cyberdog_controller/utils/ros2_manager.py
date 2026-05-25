import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from std_srvs.srv import Empty
import subprocess
import threading
import time

class SimulationClock(Node):
    _instance = None
    _lock = threading.Lock()

    def __new__(cls):
        with cls._lock:
            if cls._instance is None:
                cls._instance = super().__new__(cls)
                cls._instance._initialized = False
        return cls._instance

    def __init__(self):
        if not self._initialized:
            super().__init__('simulation_clock_listener')
            self._sim_time = None
            self._clock_lock = threading.Lock()
            self._sub = self.create_subscription(Clock, '/clock', self._clock_callback, 10)
            self._initialized = True

    def _clock_callback(self, msg):
        with self._clock_lock:
            self._sim_time = msg.clock

    def get_sim_time(self):
        with self._clock_lock:
            return self._sim_time

class ROS2Manager:
    _instance = None
    _lock = threading.Lock()

    def __new__(cls):
        with cls._lock:
            if cls._instance is None:
                cls._instance = super().__new__(cls)
                cls._instance._initialized = False
        return cls._instance

    def __init__(self):
        if not self._initialized:
            self._initialized = True
            self._init_lock = threading.Lock()
            self._executor = None
            self._thread = None
            self._sim_clock = None
            self._is_running = False

    def init(self):
        with self._init_lock:
            if not self._is_running:
                print("Initializing Global ROS2 Manager...")
                try:
                    if not rclpy.ok():
                        rclpy.init()
                except Exception as e:
                    print(f"rclpy.init error: {e}")

                self._executor = rclpy.executors.MultiThreadedExecutor()
                self._sim_clock = SimulationClock()
                self._executor.add_node(self._sim_clock)

                self._thread = threading.Thread(target=self._executor.spin, daemon=True)
                self._thread.start()
                self._is_running = True
                print("ROS2 Global Manager started.")

    def get_clock(self):
        if not self._sim_clock:
            self.init()
        return self._sim_clock

    def get_executor(self):
        if not self._executor:
            self.init()
        return self._executor

    def shutdown(self):
        pass

ros2_manager = ROS2Manager()


def unpause_physics(timeout=2.0):
    """Best-effort Gazebo unpause helper used by single-stage test scripts."""
    _unpause_gazebo_world()
    try:
        if not rclpy.ok():
            rclpy.init()
    except Exception as exc:
        print(f"[ROS2] rclpy.init failed before unpause: {exc}")
        return False

    node = rclpy.create_node("gazebo_unpause_client")
    try:
        client = node.create_client(Empty, "/unpause_physics")
        if not client.wait_for_service(timeout_sec=timeout):
            print("[ROS2] /unpause_physics service not available, continuing.")
            return False

        future = client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        if future.done() and future.result() is not None:
            print("[ROS2] Gazebo physics unpaused.")
            return True

        print("[ROS2] /unpause_physics call timed out, continuing.")
        return False
    except Exception as exc:
        print(f"[ROS2] Failed to unpause Gazebo physics: {exc}")
        return False
    finally:
        node.destroy_node()


def _unpause_gazebo_world():
    try:
        subprocess.run(
            ["gz", "world", "-w", "earth", "-p", "0"],
            check=False,
            timeout=2.0,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except Exception:
        pass
