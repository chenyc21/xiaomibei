import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
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
    """
    Global ROS2 Manager (Singleton)
    Manages the lifecycle of rclpy, a global MultiThreadedExecutor, 
    and the SimulationClock.
    """
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
        # Do not shutdown rclpy to keep background vision threads alive
        pass

# Global instance for easy access
ros2_manager = ROS2Manager()
