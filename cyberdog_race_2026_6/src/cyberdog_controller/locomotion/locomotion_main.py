import lcm
import sys
import time
import toml
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

@singleton
class LocomotionController:
    def __init__(self):
        self.lcm_cmd = lcm.LCM("udpm://239.255.76.67:7671?ttl=255")
        self.lcm_usergait = lcm.LCM("udpm://239.255.76.67:7671?ttl=255")
        self.usergait_msg = file_send_lcmt()
        self.cmd_msg = robot_control_cmd_lcmt()
        self.steps = None
        self.my_ctrl = None
        self.ctrl_thread = None

        self._initialize_gait_files()
        self._load_user_gait_list()
        self.start_control_thread()

    def _initialize_gait_files(self):
        self.steps = toml.load("/home/cyberdog_sim/src/cyberdog_controller/locomotion/Gait_Params_moonwalk.toml")
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

        with open("/home/cyberdog_sim/src/cyberdog_controller/locomotion/Gait_Params_moonwalk_full.toml", 'w') as f:
            f.write("# Gait Params\n")
            f.writelines(toml.dumps(full_steps))

        with open("/home/cyberdog_sim/src/cyberdog_controller/locomotion/Gait_Def_moonwalk.toml", 'r') as f:
            self.usergait_msg.data = f.read()
            self.lcm_usergait.publish("user_gait_file", self.usergait_msg.encode())
            time.sleep(0.5)

        with open("/home/cyberdog_sim/src/cyberdog_controller/locomotion/Gait_Params_moonwalk_full.toml", 'r') as f:
            self.usergait_msg.data = f.read()
            self.lcm_usergait.publish("user_gait_file", self.usergait_msg.encode())
            time.sleep(0.1)

    def _load_user_gait_list(self):
        with open("/home/cyberdog_sim/src/cyberdog_controller/locomotion/Usergait_List.toml", 'r') as f:
            self.steps = toml.load(f)

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
