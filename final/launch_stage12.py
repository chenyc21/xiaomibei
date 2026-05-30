#!/usr/bin/env python3
"""
launch_stage12.py — 赛段 1-2「荒野寻宝」前置仿真启动脚本

为 my_workspace/cyberdog_race_2026 1-2 的 FSM 提供运行环境：
  · 启动 Gazebo (race.world, lidar=on)
  · 启动 cyberdog_control（控制器停留在 use_rc=1 / RC 模式，
    这样 LCM channel `robot_control_cmd` 可被该 FSM 直接驱动）
  · /unpause_physics，使 /clock 正常推进
  · **不**传送，**不**启动 competition_motion_bridge / camera_perception
  · 启动后阻塞，按 Ctrl+C 退出

在另一终端运行 FSM:
  cd /home/cyberdog_sim
  source install/setup.bash
  cd "my_workspace/cyberdog_race_2026 1-2"
  python3 -m src.cyberdog_controller.FSM.fsm
"""

import os
import subprocess
import time

BASE_DIR      = "/home/cyberdog_sim"
ROS_SETUP     = "/opt/ros/galactic/setup.bash"
INSTALL_SETUP = os.path.join(BASE_DIR, "install/setup.bash")


def src(cmd):
    return f"source {ROS_SETUP} && source {INSTALL_SETUP} && {cmd}"


def run(cmd, check=True, timeout=None):
    print(f"[RUN] {cmd[:110]}")
    return subprocess.run(cmd, shell=True, cwd=BASE_DIR,
                          executable="/bin/bash", check=check, timeout=timeout)


def wait_service(name, timeout_s=60):
    print(f"  等待服务 {name} （≤{timeout_s}s）...", flush=True)
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        r = subprocess.run(
            src(f"ros2 service list 2>/dev/null | grep -qF '{name}'"),
            shell=True, executable="/bin/bash", capture_output=True)
        if r.returncode == 0:
            print(f"  ✓ {name} 就绪")
            return True
        time.sleep(1.0)
    print(f"  ✗ 超时：{name}")
    return False


def check_topic(name, timeout_s=10):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        r = subprocess.run(
            src(f"ros2 topic list 2>/dev/null | grep -qx '{name}'"),
            shell=True, executable="/bin/bash", capture_output=True)
        if r.returncode == 0:
            return True
        time.sleep(0.5)
    return False


def step_gazebo():
    print("\n" + "=" * 55 + "\n  [1/3] 启动 Gazebo (race.world, lidar=on)\n" + "=" * 55)
    subprocess.run("killall -9 gzserver gzclient 2>/dev/null",
                   shell=True, check=False)
    time.sleep(1.5)
    proc = subprocess.Popen(
        src("ros2 launch cyberdog_gazebo race_gazebo.launch.py "
            "wname:=race use_lidar:=true gui:=false"),
        shell=True, cwd=BASE_DIR, executable="/bin/bash",
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    print(f"  Gazebo PID: {proc.pid}")
    wait_service("/spawn_entity",            timeout_s=90)
    wait_service("/gazebo/set_entity_state", timeout_s=20)
    time.sleep(2)
    return proc


def step_unpause():
    print("\n  → unpause /unpause_physics ...", flush=True)
    run(src("ros2 service call /unpause_physics std_srvs/srv/Empty '{}'"),
        check=False, timeout=10)


def step_control():
    print("\n" + "=" * 55 + "\n  [2/3] 启动 cyberdog_control\n" + "=" * 55)
    log_path = os.path.join(BASE_DIR, "my_workspace", "cyberdog_control.log")
    log_f = open(log_path, "w")
    proc = subprocess.Popen(
        src("ros2 launch cyberdog_gazebo cyberdog_control_launch.py"),
        shell=True, cwd=BASE_DIR, executable="/bin/bash",
        stdout=log_f, stderr=subprocess.STDOUT)
    print(f"  control PID: {proc.pid}  log: {log_path}")
    print("  等待 6s 控制器握手 ...")
    time.sleep(6)
    return proc


def step_visual():
    # 队友 launchsim.py 里启动了 cyberdog_visual：robot_state_publisher + joint_state_publisher (+ rviz2)
    # 用于发布 URDF 的 TF（LiDAR/摄像头 frame 需要）。rviz2 没有 DISPLAY 时会失败但不影响 TF 发布
    print("\n" + "=" * 55 + "\n  [3/4] 启动 cyberdog_visual\n" + "=" * 55)
    log_path = os.path.join(BASE_DIR, "my_workspace", "cyberdog_visual.log")
    log_f = open(log_path, "w")
    proc = subprocess.Popen(
        src("ros2 launch cyberdog_visual cyberdog_visual.launch.py use_lidar:=true"),
        shell=True, cwd=BASE_DIR, executable="/bin/bash",
        stdout=log_f, stderr=subprocess.STDOUT)
    print(f"  visual PID: {proc.pid}  log: {log_path}")
    time.sleep(3)
    return proc


def health_check():
    print("\n" + "=" * 55 + "\n  关键话题检查\n" + "=" * 55)
    for t in ["/clock", "/rgb_camera/image_raw", "/scan", "/gazebo/model_states"]:
        ok = check_topic(t, timeout_s=6)
        print(f"  {'✓' if ok else '✗'} {t}")


def print_tips():
    print("\n" + "=" * 55)
    print("  ✓ 仿真已就绪。请在另一终端运行 FSM：")
    print("    cd /home/cyberdog_sim")
    print("    source install/setup.bash")
    print('    cd "my_workspace/cyberdog_race_2026 1-2"')
    print("    python3 -m src.cyberdog_controller.FSM.fsm")
    print("")
    print("  本进程 Ctrl+C 退出时会清理 Gazebo / control")
    print("=" * 55 + "\n")


def main():
    procs = []
    try:
        procs.append(step_gazebo())
        step_unpause()
        procs.append(step_control())
        procs.append(step_visual())
        health_check()
        print_tips()
        while True:
            time.sleep(60)
    except KeyboardInterrupt:
        print("\n[INFO] 用户中断，清理...")
    finally:
        for p in procs:
            try:
                p.terminate()
            except Exception:
                pass
        subprocess.run("killall -9 gzserver gzclient 2>/dev/null",
                       shell=True, check=False)
        print("[INFO] 清理完成")


if __name__ == "__main__":
    main()
