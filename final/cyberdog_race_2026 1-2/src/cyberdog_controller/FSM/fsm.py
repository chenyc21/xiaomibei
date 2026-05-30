from .states import *

class InitialLying(State):
    """
    初始趴下状态（2026赛题规则1.a要求）
    确保机器狗在起点处于趴下状态
    """
    def __init__(self):
        super().__init__("InitialLying")
        self.basic_states = [
            Laying(3),  # 确保趴下状态
            Standing(1)  # 短暂站立确认
        ]


class FSM:
    def __init__(self):
        # 2026 赛题赛程：初始趴下 -> 第一赛段 -> 第二赛段
        self.states = [
            InitialLying(),  # 确保起点趴下状态
            Stage1_StonePath(),
            Stage2_BeadHunt(),
            Finish()
        ]
        self.current_state = None
        self.current_state_index = 0

    def execute(self):
        print("=" * 60)
        print("FSM Starting... 2026 Competition Mode (荒野寻宝)")
        print("=" * 60)

        # 启动时检查摄像头状态
        from ..camera.path_scanner import check_camera_ready
        camera_ok = check_camera_ready(timeout=3.0)
        if not camera_ok:
            print("=" * 60)
            print("警告: 摄像头未就绪！将使用纯预编程动作（无视觉反馈）。")
            print("请确认仿真环境中的 gazebo.xacro 已包含 RGB 摄像头传感器插件。")
            print("修复方法：")
            print("  1. 退出仿真 (Ctrl+C)")
            print("  2. 运行: bash /path/to/fix_camera_inside_docker.sh")
            print("  3. 重新启动仿真")
            print("  4. 重新运行本程序")
            print("=" * 60)

        while self.current_state_index < len(self.states):
            self.current_state = self.states[self.current_state_index]
            print(f"\n>>> Transitioning to State: {self.current_state.name}")
            self.current_state.execute()

            print(f"<<< Finished State: {self.current_state.name}")
            self.current_state_index += 1
        print("\nFSM Completed. Mission accomplished.")

if __name__ == "__main__":
    # 此处假设环境已准备好（如 ROS2 init 等在 basic_state 中处理）
    fsm = FSM()
    fsm.execute()
