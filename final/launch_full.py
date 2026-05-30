#!/usr/bin/env python3
"""
launch_full.py — 赛段 1→4 串联仿真启动脚本

复用 launch_stage12.py / launch_stage4.py 的组件，但把两套 FSM 串起来：

  阶段 A（赛段 1-2）：
    Gazebo + cyberdog_control(use_rc=1) + cyberdog_visual
    + python3 -m cyberdog_controller.FSM.fsm     ← 阻塞直到进程退出
                                                     （直发 LCM robot_control_cmd）

  阶段 B（赛段 3-4）：
    保持 Gazebo + control 不重启
    (可选) Teleport 到赛段3入口 (-0.3, 4.60) yaw=90°
    + camera_perception_node + competition_motion_bridge(use_rc=0)
    + competition_manager_node  --ros-args  -p start_stage:=3 -p stop_after_stage:=4

用法：
  python3 my_workspace/launch_full.py                  # 全流程：1→2→teleport→3→4
  python3 my_workspace/launch_full.py --no-teleport    # 1-2 结束后不传送，原地接 3
  python3 my_workspace/launch_full.py --skip-stage12   # 跳过 1-2，等价于 launch_stage4.py --from-stage3
  python3 my_workspace/launch_full.py --stage12-timeout 600  # 1-2 FSM 最长 600s

Ctrl+C 安全清理 gzserver/gzclient 与所有子进程。
"""

import argparse
import math
import os
import signal
import subprocess
import sys
import time

BASE_DIR      = "/home/cyberdog_sim"
MY_WS         = os.path.join(BASE_DIR, "my_workspace")
ROS_SETUP     = "/opt/ros/galactic/setup.bash"
INSTALL_SETUP = os.path.join(BASE_DIR, "install/setup.bash")

# 赛段3入口（来自 launch_stage4.py SPAWN_S3）
SPAWN_S3 = {"x": -0.3, "y": 4.60, "z": 0.20, "yaw_deg": 90.0}


def src(cmd: str) -> str:
    return f"source {ROS_SETUP} && source {INSTALL_SETUP} && {cmd}"


def run(cmd: str, check: bool = True, timeout=None):
    print(f"[RUN] {cmd[:120]}")
    return subprocess.run(cmd, shell=True, cwd=BASE_DIR,
                          executable="/bin/bash", check=check, timeout=timeout)


def wait_service(name: str, timeout_s: int = 60) -> bool:
    print(f"  等待服务 {name} (≤{timeout_s}s) ...", flush=True)
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


def check_topic(name: str, timeout_s: int = 8) -> bool:
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
def step_gazebo():
    print("\n" + "=" * 60 + "\n  [启动] Gazebo (race.world, lidar=on, gui=off)\n" + "=" * 60)
    subprocess.run("killall -9 gzserver gzclient 2>/dev/null",
                   shell=True, check=False)
    time.sleep(1.5)
    proc = subprocess.Popen(
        src("ros2 launch cyberdog_gazebo race_gazebo.launch.py "
            "wname:=race use_lidar:=true gui:=false"),
        shell=True, cwd=BASE_DIR, executable="/bin/bash",
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid)
    print(f"  Gazebo PID: {proc.pid}")
    wait_service("/spawn_entity",            timeout_s=90)
    wait_service("/gazebo/set_entity_state", timeout_s=20)
    time.sleep(2)
    return proc


def step_unpause():
    print("\n  → /unpause_physics ...", flush=True)
    run(src("ros2 service call /unpause_physics std_srvs/srv/Empty '{}'"),
        check=False, timeout=10)


def spawn_logged(cmd_inner: str, log_name: str, setsid: bool = True):
    log_path = os.path.join(MY_WS, log_name)
    log_f = open(log_path, "w")
    kw = {"preexec_fn": os.setsid} if setsid else {}
    proc = subprocess.Popen(
        src(cmd_inner),
        shell=True, cwd=BASE_DIR, executable="/bin/bash",
        stdout=log_f, stderr=subprocess.STDOUT, **kw)
    print(f"  PID {proc.pid}  log: tail -f {log_path}")
    return proc


def step_control():
    print("\n" + "=" * 60 + "\n  [启动] cyberdog_control\n" + "=" * 60)
    proc = spawn_logged("ros2 launch cyberdog_gazebo cyberdog_control_launch.py",
                        "cyberdog_control.log")
    print("  等待 6s 控制器握手 ...")
    time.sleep(6)
    return proc


def step_visual():
    print("\n" + "=" * 60 + "\n  [启动] cyberdog_visual (TF publisher)\n" + "=" * 60)
    proc = spawn_logged("ros2 launch cyberdog_visual cyberdog_visual.launch.py use_lidar:=true",
                        "cyberdog_visual.log")
    time.sleep(3)
    return proc


# ── Stage 1-2 ──────────────────────────────────────────────────────
def run_stage12_fsm(timeout_s: int) -> int:
    """阻塞执行 cyberdog_controller.FSM.fsm，返回退出码"""
    print("\n" + "=" * 60 +
          "\n  [Stage 1-2] python3 -m cyberdog_controller.FSM.fsm\n" + "=" * 60)
    log_path = os.path.join(MY_WS, "stage12_fsm.log")
    print(f"  log: tail -f {log_path}")
    print(f"  最长等待 {timeout_s}s（FSM 进程退出即认为 1-2 完成）")
    with open(log_path, "w") as log_f:
        proc = subprocess.Popen(
            src("python3 -u -m cyberdog_controller.FSM.fsm"),
            shell=True, cwd=BASE_DIR, executable="/bin/bash",
            stdout=log_f, stderr=subprocess.STDOUT, preexec_fn=os.setsid)
        try:
            rc = proc.wait(timeout=timeout_s)
        except subprocess.TimeoutExpired:
            print(f"  ✗ Stage12 FSM 超时 {timeout_s}s，强制结束")
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            time.sleep(2)
            try: os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except Exception: pass
            return -1
    print(f"  Stage12 FSM 退出码 = {rc}")
    return rc


# ── Stage 3-4 ──────────────────────────────────────────────────────
def step_teleport_to_stage3():
    s = SPAWN_S3
    print("\n" + "=" * 60 +
          f"\n  [Teleport] → 赛段3入口 ({s['x']},{s['y']}) yaw={s['yaw_deg']}°\n" + "=" * 60)
    yaw = math.radians(s["yaw_deg"])
    qz, qw = math.sin(yaw / 2), math.cos(yaw / 2)
    pose = (
        f"'{{state: {{name: \"robot\", "
        f"pose: {{position: {{x: {s['x']}, y: {s['y']}, z: {s['z']}}}, "
        f"orientation: {{x: 0.0, y: 0.0, z: {qz:.4f}, w: {qw:.4f}}}}}, "
        f"reference_frame: \"world\"}}}}'"
    )
    r = run(src("ros2 service call /gazebo/set_entity_state "
                "gazebo_msgs/srv/SetEntityState " + pose),
            check=False, timeout=10)
    print("  ✓ Teleport 成功" if r.returncode == 0 else "  ⚠ Teleport 失败")
    time.sleep(2)


def step_perception():
    print("\n" + "=" * 60 + "\n  [启动] camera_perception_node\n" + "=" * 60)
    return spawn_logged(f"python3 -u {MY_WS}/camera_perception_node.py",
                        "camera_perception.log")


def step_bridge():
    print("\n" + "=" * 60 + "\n  [启动] competition_motion_bridge (ROS2→LCM, use_rc=0)\n" + "=" * 60)
    proc = spawn_logged(f"python3 -u {MY_WS}/competition_motion_bridge.py",
                        "motion_bridge.log")
    time.sleep(2)
    return proc


def step_manager(start_stage: int = 3, stop_after: int = 4):
    print("\n" + "=" * 60 +
          f"\n  [启动] competition_manager_node [start={start_stage} stop_after={stop_after}]\n" +
          "=" * 60)
    return spawn_logged(
        "ros2 run competition_manager competition_manager_node "
        f"--ros-args -p start_stage:={start_stage} -p stop_after_stage:={stop_after}",
        "competition_manager.log")


def topic_health_check(topics):
    print("\n" + "=" * 60 + "\n  话题在线性检查\n" + "=" * 60)
    ok_all = True
    for t in topics:
        ok = check_topic(t, timeout_s=6)
        print(f"  {'✓' if ok else '✗'} {t}")
        if not ok: ok_all = False
    return ok_all


# ──────────────────────────────────────────────────────────────────
def kill_proc(p):
    if p is None or p.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(p.pid), signal.SIGTERM)
    except Exception:
        try: p.terminate()
        except Exception: pass


def main():
    ap = argparse.ArgumentParser(description="赛段 1→4 串联仿真启动脚本")
    ap.add_argument("--skip-stage12",   action="store_true",
                    help="跳过赛段 1-2，直接从赛段3开始（等同 launch_stage4.py --from-stage3）")
    ap.add_argument("--no-teleport",    action="store_true",
                    help="1-2 结束后不传送，原地进入 Stage3 FSM")
    ap.add_argument("--stage12-timeout", type=int, default=900,
                    help="Stage12 FSM 最长等待秒数（默认 900s）")
    args = ap.parse_args()

    procs = []
    try:
        # ── 共享层 ─────────────────────────────────────────
        procs.append(step_gazebo())
        step_unpause()
        procs.append(step_control())
        visual_proc = step_visual()
        procs.append(visual_proc)

        # ── 阶段 A: 赛段 1-2 ───────────────────────────────
        if not args.skip_stage12:
            topic_health_check(["/clock", "/scan", "/gazebo/model_states"])
            rc = run_stage12_fsm(args.stage12_timeout)
            if rc != 0:
                print(f"\n[WARN] Stage12 FSM 异常退出 (rc={rc})，仍继续进入 Stage3-4")

            # visual 在 3-4 阶段不需要，关掉避免与后续 TF 冲突（可注释保留）
            print("\n  [清理] 停止 cyberdog_visual（3-4 阶段不再使用）")
            kill_proc(visual_proc)
            procs.remove(visual_proc)
            time.sleep(1)

            if not args.no_teleport:
                step_teleport_to_stage3()

        # ── 阶段 B: 赛段 3-4 ───────────────────────────────
        procs.append(step_perception())
        procs.append(step_bridge())
        mgr = step_manager(start_stage=3, stop_after=4)
        procs.append(mgr)

        topic_health_check([
            "/image_rgb", "/scan", "/gazebo/model_states",
            "/competition/state", "/perception/objects",
            "/perception/boundary", "/competition/motion_cmd",
        ])

        print("\n" + "=" * 60)
        print("  ✓ 赛段 3-4 已启动，常用监控命令：")
        for f in ("camera_perception.log", "motion_bridge.log", "competition_manager.log"):
            print(f"    tail -f {os.path.join(MY_WS, f)}")
        print("    ros2 topic echo /competition/state")
        print("  Ctrl+C 结束")
        print("=" * 60 + "\n")

        mgr.wait()

    except KeyboardInterrupt:
        print("\n[INFO] 用户中断，清理 ...")
    except Exception as e:
        print(f"\n[ERROR] {e}", file=sys.stderr)
        import traceback; traceback.print_exc()
    finally:
        for p in procs:
            kill_proc(p)
        subprocess.run("killall -9 gzserver gzclient 2>/dev/null",
                       shell=True, check=False)
        print("[INFO] 清理完成")


if __name__ == "__main__":
    main()
