'''
This demo show the communication interface of MR813 motion control board based on Lcm
- robot_control_cmd_lcmt.py
- file_send_lcmt.py
- Gait_Def_moonwalk.toml
- Gait_Params_moonwalk.toml
- Usergait_List.toml
'''
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
    'mode':0, 'gait_id':0, 'contact':0, 'life_count':0,
    'vel_des':[0.0, 0.0, 0.0],
    'rpy_des':[0.0, 0.0, 0.0],
    'pos_des':[0.0, 0.0, 0.0],
    'acc_des':[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    'ctrl_point':[0.0, 0.0, 0.0],
    'foot_pose':[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    'step_height':[0.0, 0.0],
    'value':0,  'duration':0
    }

class MyController:
    def __init__(self, steps, lcm_cmd):
        self.steps = steps
        self.num = 1
        self.msg = robot_control_cmd_lcmt()
        self.lc = lcm_cmd
        self.running = True

    def update_and_publish(self):
        step = self.steps["step"][self.num]
        self.msg.mode = step["mode"]
        self.msg.gait_id = step["gait_id"]
        self.msg.contact = step["contact"]
        self.msg.value = step["value"]
        self.msg.duration = step["duration"]
        for i in range(3):
            self.msg.vel_des[i] = step["vel_des"][i]
            self.msg.rpy_des[i] = step["rpy_des"][i]
            self.msg.pos_des[i] = step["pos_des"][i]
            self.msg.acc_des[i] = step["acc_des"][i]
            self.msg.acc_des[i+3] = step["acc_des"][i+3]
            self.msg.foot_pose[i] = step["foot_pose"][i]
            self.msg.foot_pose[i+3] = step["foot_pose"][i+3]
            self.msg.ctrl_point[i] = step["ctrl_point"][i]
        for i in range(2):
            self.msg.step_height[i] = step['step_height'][i]
        self.lc.publish("robot_control_cmd", self.msg.encode())

    def run(self):
        while self.running:
            self.update_and_publish()
            time.sleep(0.2)


def locomotion_main(keyboard, motion_id=0, duration=0):
    lcm_cmd = lcm.LCM("udpm://239.255.76.67:7671?ttl=255")
    lcm_usergait = lcm.LCM("udpm://239.255.76.67:7671?ttl=255")
    usergait_msg = file_send_lcmt()
    cmd_msg = robot_control_cmd_lcmt()
    steps = toml.load("/home/cyberdog_sim/src/cyberdog_controller/locomotion/Gait_Params_moonwalk.toml")
    full_steps = {'step':[robot_cmd]}
    k =0
    for i in steps['step']:
        cmd = copy.deepcopy(robot_cmd)
        cmd['duration'] = i['duration']
        if i['type'] == 'usergait':                
            cmd['mode'] = 11 # LOCOMOTION
            cmd['gait_id'] = 110 # USERGAIT
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
            cmd['ctrl_point'][2] =  i['mu']
        if k == 0:
            full_steps['step'] = [cmd]
        else:
            full_steps['step'].append(cmd)
        k=k+1
    f = open("Gait_Params_moonwalk_full.toml", 'w')
    f.write("# Gait Params\n")
    f.writelines(toml.dumps(full_steps))
    f.close()

    file_obj_gait_def = open("/home/cyberdog_sim/src/cyberdog_controller/locomotion/Gait_Def_moonwalk.toml",'r')
    file_obj_gait_params = open("/home/cyberdog_sim/src/cyberdog_controller/locomotion/Gait_Params_moonwalk_full.toml",'r')
    usergait_msg.data = file_obj_gait_def.read()
    lcm_usergait.publish("user_gait_file",usergait_msg.encode())
    time.sleep(0.5)
    usergait_msg.data = file_obj_gait_params.read()
    lcm_usergait.publish("user_gait_file",usergait_msg.encode())
    time.sleep(0.1)
    file_obj_gait_def.close()
    file_obj_gait_params.close()

    user_gait_list = open("/home/cyberdog_sim/src/cyberdog_controller/locomotion/Usergait_List.toml",'r')
    steps = toml.load(user_gait_list)
    user_gait_list.close()
        
    my_ctrl = MyController(steps, lcm_cmd)
    ctrl_thread = threading.Thread(target=my_ctrl.run)
    ctrl_thread.start()

    last_command = None
    if keyboard:
        try:
            while True:
                num = input("请输入你的指令编号 (输入 -1 退出): ")
                if num.strip() == "-1":
                    my_ctrl.running = False
                    ctrl_thread.join()
                    break
                elif num.strip() == "":
                    if last_command is not None:
                        my_ctrl.num = last_command
                else:
                    num = int(num)
                    if 0 <= num < len(steps["step"]):
                        my_ctrl.num = num
                        my_ctrl.msg.life_count += 1
                        last_command = num
                    else:
                        print(f"无效编号：{num}，有效范围为 0 ~ {len(steps['step']) - 1}")
        
        except KeyboardInterrupt:
            my_ctrl.msg.mode = 7
            my_ctrl.msg.gait_id = 0
            my_ctrl.msg.duration = 0
            my_ctrl.msg.life_count += 1
            lcm_cmd.publish("robot_control_cmd", my_ctrl.msg.encode())
            my_ctrl.running = False
            ctrl_thread.join()
        sys.exit()

if __name__ == '__main__':
    locomotion_main(True)