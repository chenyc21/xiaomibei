import os
import sys
import time
import copy
import math
import threading
from .robot_control_cmd_lcmt import robot_control_cmd_lcmt
from .file_send_lcmt import file_send_lcmt

robot_cmd = {
    'mode': 0, 'gait_id': 0, 'contact': 0, 'life_count': 0,
    'vel_des': [0.0, 0.0, 0.0],
    'rpy_des': [0.0, 0.0, 0.0],
    'pos_des': [0.0, 0.0, 0.0],
    'acc_des': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    'ctrl_point': [0.0, 0.0, 0.0],
    'foot_pose': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    'step_height': [0.0, 0.0],
    'value': 0, 'duration': 0
}


class MyController:
    def __init__(self, steps, lcm_cmd):
        self.steps = steps
        self.num = 1
        self.msg = robot_control_cmd_lcmt()
        self.lc = lcm_cmd
        self.running = True
        self.lock = threading.Lock()

    def update_and_publish(self):
        with self.lock:
            step = self.steps["step"][self.num]
            self.msg.mode = step["mode"]
            self.msg.gait_id = step["gait_id"]
            self.msg.contact = step["contact"]
            self.msg.value = step["value"]
            self.msg.duration = step["duration"]
            self.msg.life_count = (self.msg.life_count + 1) % 128

            for i in range(3):
                self.msg.vel_des[i] = step["vel_des"][i]
                self.msg.rpy_des[i] = step["rpy_des"][i]
                self.msg.pos_des[i] = step["pos_des"][i]
                self.msg.acc_des[i] = step["acc_des"][i]
                self.msg.acc_des[i + 3] = step["acc_des"][i + 3]
                self.msg.foot_pose[i] = step["foot_pose"][i]
                self.msg.foot_pose[i + 3] = step["foot_pose"][i + 3]
                self.msg.ctrl_point[i] = step["ctrl_point"][i]

            for i in range(2):
                self.msg.step_height[i] = step['step_height'][i]
            self.lc.publish("robot_control_cmd", self.msg.encode())

    def run(self):
        while self.running:
            self.update_and_publish()
            time.sleep(0.02)

    def set_num(self, motion_id):
        with self.lock:
            self.num = motion_id


def singleton(cls):
    instances = {}
    def wrapper(*args, **kwargs):
        if cls not in instances:
            instances[cls] = cls(*args, **kwargs)
        return instances[cls]
    return wrapper


def _ensure_python_lcm_path():
    version = f"python{sys.version_info.major}.{sys.version_info.minor}"
    candidates = [
        f"/usr/local/lib/{version}/site-packages",
        "/home/lcm/build/python",
    ]
    for path in candidates:
        if os.path.exists(os.path.join(path, "lcm")) and path not in sys.path:
            sys.path.append(path)


def _cmd(
    mode=0,
    gait_id=0,
    vel=None,
    rpy=None,
    pos=None,
    step_height=None,
    value=0,
    contact=None,
    acc=None,
    duration=0,
):
    cmd = copy.deepcopy(robot_cmd)
    cmd["mode"] = mode
    cmd["gait_id"] = gait_id
    cmd["contact"] = 15 if contact is None and mode == 11 else (contact or 0)
    cmd["vel_des"] = vel or [0.0, 0.0, 0.0]
    cmd["rpy_des"] = rpy or [0.0, 0.0, 0.0]
    cmd["pos_des"] = pos or [0.0, 0.0, 0.24]
    cmd["acc_des"] = acc or [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    cmd["step_height"] = step_height or [0.035, 0.035]
    cmd["value"] = value
    cmd["duration"] = duration
    return cmd


@singleton
class LocomotionController:
    def __init__(self):
        _ensure_python_lcm_path()
        import lcm

        self.lcm_cmd = lcm.LCM("udpm://239.255.76.67:7671?ttl=255")
        self.lcm_usergait = lcm.LCM("udpm://239.255.76.67:7671?ttl=255")
        self.usergait_msg = file_send_lcmt()
        self.cmd_msg = robot_control_cmd_lcmt()
        self.steps = None
        self.my_ctrl = None
        self.ctrl_thread = None
        self.locomotion_dir = os.path.dirname(os.path.abspath(__file__))
        self.generated_params_path = os.path.join(self.locomotion_dir, "Gait_Params_moonwalk_full.toml")

        if not self._try_load_legacy_gait_files():
            self.steps = self._build_builtin_motion_table()
        self.start_control_thread()

    def _find_gait_file(self, filename):
        workspace_root = os.path.abspath(os.path.join(self.locomotion_dir, "..", "..", "..", ".."))
        env_root = os.environ.get("CYBERDOG_SIM_ROOT")
        candidates = [
            os.path.join(self.locomotion_dir, filename),
            os.path.join(workspace_root, "src", "cyberdog_controller", "locomotion", filename),
        ]
        if env_root:
            candidates.extend([
                os.path.join(env_root, "src", "cyberdog_controller", "locomotion", filename),
                os.path.join(env_root, "cyberdog_controller", "locomotion", filename),
            ])
        candidates.append(os.path.join("/home/cyberdog_sim/src/cyberdog_controller/locomotion", filename))

        for path in candidates:
            if os.path.exists(path):
                return path
        raise FileNotFoundError(f"未找到步态文件 {filename}，已搜索: {candidates}")

    def _try_load_legacy_gait_files(self):
        try:
            import toml
            self.toml = toml
            self._initialize_gait_files()
            self._load_user_gait_list()
            print("[Locomotion] 已加载 legacy Gait_Params/Usergait_List 步态文件。")
            return True
        except Exception as e:
            print(f"[Locomotion] legacy 步态文件不可用，使用内置运动表: {e}")
            return False

    def _build_builtin_motion_table(self):
        """
        cyberdog_sim:v2026 中没有旧版 Gait_Params_moonwalk.toml。
        这里直接使用官方 robot_control_cmd_lcmt 高层接口：
        mode=11 是 locomotion，gait_id=1/6/27 分别用于站立/行走/慢步。
        """
        steps = [_cmd()]
        for motion_id in range(1, 26):
            steps.append(_cmd())

        steps[1] = _cmd(mode=11, gait_id=1, vel=[0.0, 0.0, 0.0])
        steps[2] = _cmd(mode=12, gait_id=0, vel=[0.0, 0.0, 0.0])
        steps[3] = _cmd(mode=7, gait_id=0, pos=[0.0, 0.0, 0.0])
        steps[5] = _cmd(mode=11, gait_id=6, vel=[0.20, 0.0, 0.0])
        steps[6] = _cmd(mode=11, gait_id=6, vel=[-0.12, 0.0, 0.0])
        steps[9] = _cmd(mode=11, gait_id=27, vel=[0.0, 0.0, 0.35])
        steps[10] = _cmd(mode=11, gait_id=27, vel=[0.0, 0.0, -0.35])
        steps[11] = _cmd(mode=11, gait_id=6, vel=[0.0, -0.10, 0.0])
        steps[12] = _cmd(mode=11, gait_id=6, vel=[0.0, 0.10, 0.0])
        steps[13] = _cmd(mode=11, gait_id=6, vel=[0.0, 0.10, 0.0])
        steps[14] = _cmd(mode=11, gait_id=6, vel=[0.0, -0.10, 0.0])
        # 参考赛段工程的 motion_id=16：高抬腿慢走，专门处理台阶/桥入口卡脚。
        steps[16] = _cmd(
            mode=11,
            gait_id=27,
            vel=[0.08, 0.0, 0.0],
            pos=[0.0, 0.0, 0.30],
            step_height=[0.50, 0.50],
            value=3,
        )
        steps[17] = _cmd(
            mode=11,
            gait_id=27,
            vel=[0.055, 0.0, 0.0],
            pos=[0.0, 0.0, 0.34],
            step_height=[0.90, 0.90],
            value=3,
        )
        steps[18] = _cmd(
            mode=11,
            gait_id=27,
            vel=[0.055, 0.0, 0.0],
            pos=[0.0, 0.0, 0.34],
            step_height=[0.85, 0.85],
            value=3,
        )
        return {"step": steps}

    def _initialize_gait_files(self):
        gait_params_path = self._find_gait_file("Gait_Params_moonwalk.toml")
        gait_def_path = self._find_gait_file("Gait_Def_moonwalk.toml")
        self.steps = self.toml.load(gait_params_path)
        full_steps = {'step': [robot_cmd]}
        k = 0
        for i in self.steps['step']:
            cmd = copy.deepcopy(robot_cmd)
            cmd['duration'] = i['duration']
            if i['type'] == 'usergait':
                cmd['mode'] = 11
                cmd['gait_id'] = 110
                cmd['vel_des'] = i['body_vel_des']
                cmd['rpy_des'] = i['body_pos_des'][0:3]
                cmd['pos_des'] = i['body_pos_des'][3:6]
                cmd['foot_pose'][0:2] = i['landing_pos_des'][0:2]
                cmd['foot_pose'][2:4] = i['landing_pos_des'][3:5]
                cmd['foot_pose'][4:6] = i['landing_pos_des'][6:8]
                cmd['ctrl_point'][0:2] = i['landing_pos_des'][9:11]
                cmd['step_height'][0] = math.ceil(i['step_height'][0] * 1e3) + math.ceil(i['step_height'][1] * 1e3) * 1e3
                cmd['step_height'][1] = math.ceil(i['step_height'][2] * 1e3) + math.ceil(i['step_height'][3] * 1e3) * 1e3
                cmd['acc_des'] = i['weight']
                cmd['value'] = i['use_mpc_traj']
                cmd['contact'] = math.floor(i['landing_gain'] * 1e1)
                cmd['ctrl_point'][2] = i['mu']
            if k == 0:
                full_steps['step'] = [cmd]
            else:
                full_steps['step'].append(cmd)
            k += 1

        with open(self.generated_params_path, 'w') as f:
            f.write("# Gait Params\n")
            f.writelines(self.toml.dumps(full_steps))

        with open(gait_def_path, 'r') as f:
            self.usergait_msg.data = f.read()
            self.lcm_usergait.publish("user_gait_file", self.usergait_msg.encode())
            time.sleep(0.5)

        with open(self.generated_params_path, 'r') as f:
            self.usergait_msg.data = f.read()
            self.lcm_usergait.publish("user_gait_file", self.usergait_msg.encode())
            time.sleep(0.1)

    def _load_user_gait_list(self):
        usergait_list_path = self._find_gait_file("Usergait_List.toml")
        with open(usergait_list_path, 'r') as f:
            self.steps = self.toml.load(f)

    def start_control_thread(self):
        self.my_ctrl = MyController(self.steps, self.lcm_cmd)
        self.ctrl_thread = threading.Thread(target=self.my_ctrl.run)
        self.ctrl_thread.daemon = True
        self.ctrl_thread.start()

    def set_motion(self, motion_id):
        if 0 <= motion_id < len(self.steps["step"]):
            self.my_ctrl.set_num(motion_id)
        else:
            print(f"无效 motion_id: {motion_id}，有效范围 0~{len(self.steps['step']) - 1}")

    def stop(self):
        if self.my_ctrl:
            self.my_ctrl.running = False
            self.ctrl_thread.join()
