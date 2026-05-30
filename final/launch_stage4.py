#!/usr/bin/env python3
"""
launch_stage4.py — 赛段四「深隧寻珍」专用测试脚本

封装 launch_stage34.py 的赛段4场景，并附带：
  · 一键启动 Gazebo + control + camera_perception + motion_bridge + manager
  · 启动后自动检查关键 ROS2 话题是否在线（/image_rgb /scan /perception/objects ...）
  · 打印感知/桥接日志路径，便于 tail -f 调试
  · 可选 --inspect-only：只跑感知/检查话题，不启动 FSM（用于排查感知）
  · Ctrl+C 时自动清理 gzserver / gzclient 及子进程

用法：
  python3 my_workspace/launch_stage4.py                # 完整跑赛段4
  python3 my_workspace/launch_stage4.py --no-build     # 跳过 colcon build
  python3 my_workspace/launch_stage4.py --no-teleport  # 不传送（用于真实出生测试）
  python3 my_workspace/launch_stage4.py --inspect-only # 只起 Gazebo+感知，不跑 FSM
"""

import argparse
import math
import os
import subprocess
import sys
import time

BASE_DIR      = "/home/cyberdog_sim"
MY_WS         = os.path.join(BASE_DIR, "my_workspace")
ROS_SETUP     = "/opt/ros/galactic/setup.bash"
INSTALL_SETUP = os.path.join(BASE_DIR, "install/setup.bash")

# 赛段4出生点：race.world 中独木桥北端 (3.13, 5.91)，桥北口入口 ≈ (3.15, 7.60)
# 官方出生在桥下游 (3.1, 6.6) 朝 +Y，前方 1m 即独木桥
SPAWN_FULL = {"x": 3.1, "y": 6.60, "z": 0.30, "yaw_deg": 90.0, "desc": "深隧寻珍入口"}
# C3-only 快捷测试：瞬移到走廃3 南端 (2.10, 7.10)，朝 +Y，直接走 kAdvanceC3
SPAWN_C3   = {"x": 2.10, "y": 7.10, "z": 0.30, "yaw_deg": 90.0, "desc": "走廃3入口 (C3-only)"}
# bridge-only 快捷测试：直接传送到独木桥南口前 (3.15, 7.20)，朝 +Y，起身后一步入桥
SPAWN_BRIDGE = {"x": 3.15, "y": 7.20, "z": 0.30, "yaw_deg": 90.0, "desc": "独木桥南口 (bridge-only)"}
# 连跑赛段3+4：从赛段3入口出发
SPAWN_S3   = {"x": -0.3, "y": 4.60, "z": 0.20, "yaw_deg": 90.0, "desc": "曲道冲锋入口 (Stage3→4)"}
SPAWN = SPAWN_FULL  # 运行时 main() 会根据 --c3-only / --from-stage3 覆盖

# 关键话题列表（启动后做存在性检查）
TOPICS_REQUIRED = [
    "/image_rgb",                # Gazebo 相机
    "/scan",                     # 激光雷达
    "/gazebo/model_states",      # GT pose
    "/competition/state",        # FSM 状态
    "/perception/objects",       # 感知输出
    "/perception/boundary",      # 边界
    "/competition/motion_cmd",   # FSM → 桥
]

# ──────────────────────────────────────────────────────────────────
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

def check_topic(name, timeout_s=8):
    """轻量话题存在性检查：grep ros2 topic list。"""
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        r = subprocess.run(
            src(f"ros2 topic list 2>/dev/null | grep -qx '{name}'"),
            shell=True, executable="/bin/bash", capture_output=True)
        if r.returncode == 0:
            return True
        time.sleep(0.5)
    return False

# ──────────────────────────────────────────────────────────────────
def step_build():
    print("\n" + "="*55 + "\n  [1/7] 编译竞赛包\n" + "="*55)
    run(src("colcon build --merge-install --symlink-install "
            "--packages-select competition_msgs competition_manager"))

def step_gazebo():
    print("\n" + "="*55 + "\n  [2/7] 启动 Gazebo (race.world, lidar=on)\n" + "="*55)
    subprocess.run("killall -9 gzserver gzclient 2>/dev/null",
                   shell=True, check=False)
    time.sleep(1.5)
    proc = subprocess.Popen(
        src("ros2 launch cyberdog_gazebo race_gazebo.launch.py "
            "wname:=race use_lidar:=true"),
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
    print("\n" + "="*55 + "\n  [3/7] 启动 cyberdog_control\n" + "="*55)
    log_path = os.path.join(MY_WS, "cyberdog_control.log")
    log_f = open(log_path, "w")
    proc = subprocess.Popen(
        src("ros2 launch cyberdog_gazebo cyberdog_control_launch.py"),
        shell=True, cwd=BASE_DIR, executable="/bin/bash",
        stdout=log_f, stderr=subprocess.STDOUT)
    print(f"  control PID: {proc.pid}  log: {log_path}")
    print(f"  control PID: {proc.pid}（等待 6s 控制器握手）")
    time.sleep(6)
    return proc

def step_teleport():
    print("\n" + "="*55 +
          f"\n  [4/7] Teleport → ({SPAWN['x']},{SPAWN['y']}) yaw={SPAWN['yaw_deg']}°\n" +
          "="*55)
    yaw = math.radians(SPAWN["yaw_deg"])
    qz, qw = math.sin(yaw / 2), math.cos(yaw / 2)
    pose = (
        f"'{{state: {{name: \"robot\", "
        f"pose: {{position: {{x: {SPAWN['x']}, y: {SPAWN['y']}, z: {SPAWN['z']}}}, "
        f"orientation: {{x: 0.0, y: 0.0, z: {qz:.4f}, w: {qw:.4f}}}}}, "
        f"reference_frame: \"world\"}}}}'"
    )
    r = run(src("ros2 service call /gazebo/set_entity_state "
                "gazebo_msgs/srv/SetEntityState " + pose),
            check=False, timeout=10)
    print("  ✓ Teleport 成功" if r.returncode == 0 else "  ⚠ Teleport 失败")

def spawn_log_proc(script, log_name):
    full = os.path.join(MY_WS, script)
    if not os.path.exists(full):
        print(f"  ✗ 找不到 {full}")
        return None
    log_path = os.path.join(MY_WS, log_name)
    log_f = open(log_path, "w")
    proc = subprocess.Popen(
        src(f"python3 -u {full}"),
        shell=True, cwd=BASE_DIR, executable="/bin/bash",
        stdout=log_f, stderr=subprocess.STDOUT)
    print(f"  PID {proc.pid}  日志: tail -f {log_path}")
    return proc

def step_perception():
    print("\n" + "="*55 + "\n  [5/7] 启动 camera_perception_node\n" + "="*55)
    return spawn_log_proc("camera_perception_node.py", "camera_perception.log")

def step_bridge():
    print("\n" + "="*55 + "\n  [6/7] 启动 motion_bridge (ROS2→LCM)\n" + "="*55)
    proc = spawn_log_proc("competition_motion_bridge.py", "motion_bridge.log")
    if proc: time.sleep(2)  # 等 use_rc=0 切换
    return proc

def step_manager(c3_only=False, start_stage=4, stop_after_stage=4, bridge_only=False):
    tag = f" [start={start_stage} stop_after={stop_after_stage}]"
    if c3_only: tag += " [C3-ONLY]"
    if bridge_only: tag += " [BRIDGE-ONLY]"
    print("\n" + "="*55 + "\n  [7/7] 启动 competition_manager" + tag + "\n" + "="*55)
    log_path = os.path.join(MY_WS, "competition_manager.log")
    log_f = open(log_path, "w")
    env = os.environ.copy()
    if c3_only:
        env["STAGE4_C3_ONLY"] = "1"
        print("  环境变量 STAGE4_C3_ONLY=1 → 起身后直接跳 kAdvanceC3")
    if bridge_only:
        env["STAGE4_BRIDGE_ONLY"] = "1"
        print("  环境变量 STAGE4_BRIDGE_ONLY=1 → 起身后直接跳 kAdvanceToBridgeTop")
    proc = subprocess.Popen(
        src("ros2 run competition_manager competition_manager_node "
            f"--ros-args -p start_stage:={start_stage} -p stop_after_stage:={stop_after_stage}"),
        shell=True, cwd=BASE_DIR, executable="/bin/bash",
        stdout=log_f, stderr=subprocess.STDOUT, env=env)
    print(f"  manager PID: {proc.pid}  日志: tail -f {log_path}")
    return proc

def topic_health_check():
    print("\n" + "="*55 + "\n  话题在线性检查\n" + "="*55)
    ok_all = True
    for t in TOPICS_REQUIRED:
        ok = check_topic(t, timeout_s=6)
        flag = "✓" if ok else "✗"
        print(f"  {flag} {t}")
        if not ok: ok_all = False
    if not ok_all:
        print("\n  ⚠ 部分话题未上线 — 请检查感知/Gazebo 日志")
    return ok_all

def print_runtime_tips():
    cp = os.path.join(MY_WS, "camera_perception.log")
    mb = os.path.join(MY_WS, "motion_bridge.log")
    mg = os.path.join(MY_WS, "competition_manager.log")
    print(f"\n{'='*55}")
    print("  ✓ 赛段4 已启动。常用监控命令：")
    print(f"    tail -f {cp}")
    print(f"    tail -f {mb}")
    print(f"    tail -f {mg}")
    print( "    ros2 topic echo /perception/objects --once --no-arr")
    print( "    ros2 topic hz   /image_rgb")
    print( "    ros2 topic hz   /scan")
    print( "    ros2 topic echo /competition/state")
    print( "    ros2 topic echo /competition/motion_cmd")
    print( "  Ctrl+C 结束")
    print(f"{'='*55}\n")

# ──────────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser(description="赛段四专用测试启动脚本")
    ap.add_argument("--no-build",     action="store_true")
    ap.add_argument("--no-teleport",  action="store_true")
    ap.add_argument("--c3-only",      action="store_true",
                    help="瞬移到走廃3入口 (2.10,7.10) 并跳过廃1/2，直接测试走廃3作业")
    ap.add_argument("--bridge-only",  action="store_true",
                    help="瞬移到独木桥南口 (3.15,7.20) yaw=90°，仅测试上桥停稳")
    ap.add_argument("--from-stage3",  action="store_true",
                    help="从赛段3入口出发，连跑赛段3+4 (start_stage=3, stop_after_stage=4)")
    ap.add_argument("--inspect-only", action="store_true",
                    help="只起 Gazebo+control+感知+检查话题，不跑 FSM")
    args = ap.parse_args()

    if args.from_stage3 and args.c3_only:
        print("[ERROR] --from-stage3 与 --c3-only 不可同时使用", file=sys.stderr)
        sys.exit(2)
    if args.bridge_only and (args.c3_only or args.from_stage3):
        print("[ERROR] --bridge-only 不能与 --c3-only / --from-stage3 同时使用", file=sys.stderr)
        sys.exit(2)

    # 根据 开关 选择出生点
    global SPAWN
    if args.bridge_only:
        SPAWN = SPAWN_BRIDGE
    elif args.c3_only:
        SPAWN = SPAWN_C3
    elif args.from_stage3:
        SPAWN = SPAWN_S3
    else:
        SPAWN = SPAWN_FULL
    print(f"  出生点: {SPAWN['desc']} → ({SPAWN['x']}, {SPAWN['y']}) yaw={SPAWN['yaw_deg']}°")

    procs = []
    try:
        if not args.no_build:
            step_build()
        procs.append(step_gazebo())
        step_unpause()
        procs.append(step_control())
        if not args.no_teleport:
            step_teleport()
        if (p := step_perception()): procs.append(p)
        if (p := step_bridge()):     procs.append(p)

        topic_health_check()

        if args.inspect_only:
            print("\n  [--inspect-only] 已跳过 FSM；按 Ctrl+C 退出")
            while True:
                time.sleep(60)

        start_stage = 3 if args.from_stage3 else 4
        stop_after  = 4
        mgr = step_manager(c3_only=args.c3_only,
                           start_stage=start_stage,
                           stop_after_stage=stop_after,
                           bridge_only=args.bridge_only); procs.append(mgr)
        print_runtime_tips()
        mgr.wait()

    except KeyboardInterrupt:
        print("\n[INFO] 用户中断，清理...")
    except Exception as e:
        print(f"\n[ERROR] {e}", file=sys.stderr)
        import traceback; traceback.print_exc()
    finally:
        for p in procs:
            try: p.terminate()
            except Exception: pass
        subprocess.run("killall -9 gzserver gzclient 2>/dev/null",
                       shell=True, check=False)
        print("[INFO] 清理完成")

if __name__ == "__main__":
    main()
