#!/usr/bin/env python3
"""
launch_stage34.py  v3  —  赛段三/四仿真快速启动脚本
用法：
    python3 my_workspace/launch_stage34.py [--stage 3|4] [--no-build] [--no-teleport]

启动顺序：
  1. 编译竞赛包（可跳过）
  2. 启动 Gazebo（world 文件已内置 gazebo_ros_state 插件）
  3. Unpause 物理仿真
  4. 启动 cyberdog_control（unpause 之后启动，避免 ESTOP）
  5. 【可选】Teleport 机器人到赛段起始位置（服务不可用时自动跳过）
  6. 启动 mock_perception_node.py
  7. 启动 competition_manager（指定 start_stage）
  8. 发送 /competition/start 信号
"""

import subprocess
import sys
import time
import os
import argparse
import math

BASE_DIR      = "/home/cyberdog_sim"
MY_WS         = os.path.join(BASE_DIR, "my_workspace")
ROS_SETUP     = "/opt/ros/galactic/setup.bash"
INSTALL_SETUP = os.path.join(BASE_DIR, "install/setup.bash")

# 赛道坐标系（race_gazebo.launch.py 实测 + STL 相对比例推算）：
# 机器人默认出生点 (0, 0, 0.6) = 赛段1起点；赛道沿 world +Y 方向延伸
# 赛段3入口由 Gazebo 实测得 (x≈0, y≈4.45)；赛段3长≈3.72m
# 赛道中线 world_x ≈ 0；机器人朝 +Y 行进（yaw=90°）
STAGE_SPAWN = {
    3: {"x":  -0.3,  "y": 4.60,  "z": 0.2, "yaw": 90.0, "desc": "曲道冲锋入口"},
    # 赛段4官方出生点：(3.1, 6.6) 朝 +Y。前方 1m 处是独木桥(3.15, 7.6)，需绕开。
    4: {"x":   3.1, "y": 6.60, "z": 0.3, "yaw": 90.0, "desc": "深隧寻珍入口"},
}

def source_cmd(cmd):
    return f"source {ROS_SETUP} && source {INSTALL_SETUP} && {cmd}"

def run(cmd, check=True, timeout=None, cwd=BASE_DIR):
    print(f"[RUN] {cmd[:110]}")
    return subprocess.run(cmd, shell=True, cwd=cwd, executable="/bin/bash",
                          check=check, timeout=timeout)

def wait_for_service(service_name, timeout_s=60):
    print(f"  等待服务 {service_name}（最多 {timeout_s:.0f}s）...", flush=True)
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        ret = subprocess.run(
            source_cmd(f"ros2 service list 2>/dev/null | grep -qF '{service_name}'"),
            shell=True, executable="/bin/bash", capture_output=True)
        if ret.returncode == 0:
            print(f"  ✓ {service_name} 已就绪")
            return True
        time.sleep(1.0)
    print(f"  ✗ 超时：{service_name} 未就绪（继续后续步骤）")
    return False

def step_build():
    print("\n" + "="*55 + "\n  Step 1: 增量编译竞赛包\n" + "="*55)
    run(source_cmd(
        "colcon build --merge-install --symlink-install "
        "--packages-select competition_msgs competition_manager"
    ), cwd=BASE_DIR)

def step_gazebo(wname):
    print("\n" + "="*55 + f"\n  Step 2: 启动 Gazebo（{wname}.world）\n" + "="*55)
    subprocess.run("killall -9 gzserver gzclient 2>/dev/null", shell=True, check=False)
    time.sleep(1)
    cmd = source_cmd(f"ros2 launch cyberdog_gazebo race_gazebo.launch.py wname:={wname} use_lidar:=true")
    proc = subprocess.Popen(cmd, shell=True, cwd=BASE_DIR, executable="/bin/bash",
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    print(f"  Gazebo PID: {proc.pid}")
    wait_for_service("/spawn_entity", timeout_s=90)
    # set_entity_state 由 world 文件中的 gazebo_ros_state 插件提供
    wait_for_service("/gazebo/set_entity_state", timeout_s=20)
    time.sleep(2)
    return proc

def step_unpause():
    print("\n  → Unpause 物理仿真...", flush=True)
    run(source_cmd("ros2 service call /unpause_physics std_srvs/srv/Empty '{}'"),
        check=False, timeout=10)

def step_locomotion():
    print("\n" + "="*55 + "\n  Step 3: 启动 cyberdog_control\n" + "="*55)
    cmd = source_cmd("ros2 launch cyberdog_gazebo cyberdog_control_launch.py")
    proc = subprocess.Popen(cmd, shell=True, cwd=BASE_DIR, executable="/bin/bash",
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    print(f"  control PID: {proc.pid}，等待 6s 让控制器建立连接...")
    time.sleep(6)
    return proc

def step_teleport(stage):
    sp = STAGE_SPAWN[stage]
    print("\n" + "="*55 +
          f"\n  Step 4: Teleport → 赛段{stage} {sp['desc']} ({sp['x']},{sp['y']})\n" +
          "="*55)
    ret = subprocess.run(
        source_cmd("ros2 service list 2>/dev/null | grep -qF '/gazebo/set_entity_state'"),
        shell=True, executable="/bin/bash", capture_output=True, timeout=5)
    if ret.returncode != 0:
        print("  ⚠ /gazebo/set_entity_state 不可用，跳过 Teleport")
        print("    机器人将从默认出生点运行，mock 感知数据仍正常")
        return False
    yaw_rad = math.radians(sp["yaw"])
    qz, qw = math.sin(yaw_rad / 2), math.cos(yaw_rad / 2)
    pose_str = (
        f"'{{state: {{name: \"robot\", "
        f"pose: {{position: {{x: {sp['x']}, y: {sp['y']}, z: {sp['z']}}}, "
        f"orientation: {{x: 0.0, y: 0.0, z: {qz:.4f}, w: {qw:.4f}}}}}, "
        f"reference_frame: \"world\"}}}}'"
    )
    ret = run(source_cmd(
        f"ros2 service call /gazebo/set_entity_state "
        f"gazebo_msgs/srv/SetEntityState {pose_str}"),
        check=False, timeout=10)
    if ret.returncode == 0:
        print("  ✓ Teleport 成功")
        return True
    print(f"  ⚠ Teleport 失败（rc={ret.returncode}），继续执行")
    return False

def step_mock_perception(stage):
    """启动感知节点：赛段4使用真实相机感知 (camera_perception_node)，
    其它赛段沿用 mock_perception_node。"""
    use_camera = (stage == 4)
    name = "camera_perception" if use_camera else "mock_perception"
    print("\n" + "="*55 + f"\n  Step 5: 启动 {name}_node\n" + "="*55)
    script_name = "camera_perception_node.py" if use_camera else "mock_perception_node.py"
    script = os.path.join(MY_WS, script_name)
    if not os.path.exists(script):
        print(f"  ✗ 找不到 {script}，跳过感知")
        return None
    log_path = os.path.join(MY_WS, f"{name}.log")
    log_f = open(log_path, "w")
    proc = subprocess.Popen(
        source_cmd(f"python3 -u {script}"),
        shell=True, cwd=BASE_DIR, executable="/bin/bash",
        stdout=log_f, stderr=subprocess.STDOUT)
    print(f"  {name} PID: {proc.pid}  (日志: tail -f {log_path})")
    time.sleep(1)
    return proc

def step_motion_bridge():
    print("\n" + "="*55 + "\n  Step 5b: 启动 motion bridge（ROS2→LCM）\n" + "="*55)
    script = os.path.join(MY_WS, "competition_motion_bridge.py")
    if not os.path.exists(script):
        print(f"  ✗ 找不到 {script}，跳过 bridge")
        return None
    log_path = os.path.join(MY_WS, "motion_bridge.log")
    log_f = open(log_path, "w")
    proc = subprocess.Popen(
        source_cmd(f"python3 -u {script}"),
        shell=True, cwd=BASE_DIR, executable="/bin/bash",
        stdout=log_f, stderr=subprocess.STDOUT)
    print(f"  motion_bridge PID: {proc.pid}，等待 2s 让 gamepad 模式切换生效（日志: tail -f {log_path}）...")
    time.sleep(2)
    return proc

def step_competition_manager(start_stage, stop_after_stage=None):
    if stop_after_stage is None:
        stop_after_stage = start_stage  # 默认：只跑指定赛段
    print("\n" + "="*55 +
          f"\n  Step 6: 启动 competition_manager（start={start_stage}, stop_after={stop_after_stage}）\n" +
          "="*55)
    cmd = source_cmd(
        f"ros2 run competition_manager competition_manager_node "
        f"--ros-args -p start_stage:={start_stage} -p stop_after_stage:={stop_after_stage}")
    proc = subprocess.Popen(cmd, shell=True, cwd=BASE_DIR, executable="/bin/bash")
    print(f"  competition_manager PID: {proc.pid}")
    return proc

def step_send_start():
    print("\n  → 发送 /competition/start ...")
    time.sleep(2)
    run(source_cmd(
        "ros2 topic pub --once /competition/start std_msgs/msg/Bool '{data: true}'"),
        check=False, timeout=8)

def main():
    parser = argparse.ArgumentParser(description="赛段三/四仿真快速启动 v3")
    parser.add_argument("--stage",       type=int, choices=[3, 4], default=3)
    parser.add_argument("--no-build",    action="store_true", help="跳过编译")
    parser.add_argument("--no-teleport", action="store_true", help="跳过 Teleport")
    parser.add_argument("--stop-after",  type=int, default=None,
                        help="完成第N赛段后停止（默认=start_stage，即只跑一个赛段）")
    parser.add_argument("--world",       default="race",
                        help="Gazebo world（不含 .world，默认 race；race2025 也已可用）")
    args = parser.parse_args()

    procs = []
    try:
        if not args.no_build:
            step_build()

        procs.append(step_gazebo(args.world))
        step_unpause()
        procs.append(step_locomotion())

        if not args.no_teleport:
            step_teleport(args.stage)
        else:
            print("\n  [--no-teleport] 已跳过")

        mp = step_mock_perception(args.stage)
        if mp:
            procs.append(mp)

        # motion bridge：先于 competition_manager 启动（确保 use_rc=0 先发送）
        mb = step_motion_bridge()
        if mb:
            procs.append(mb)

        mgr = step_competition_manager(args.stage, args.stop_after)
        procs.append(mgr)
        # 注意：FSM 已由 start_stage 参数自动启动，无需再发 /competition/start
        # （发送该信号只用于真实比赛的裁判系统，避免重置赛段）

        print(f"\n{'='*55}")
        print(f"  ✓ 仿真已启动！赛段 {args.stage} 正在运行")
        print(f"  监控状态:  ros2 topic echo /competition/state")
        print(f"  监控运动:  ros2 topic echo /competition/motion_cmd")
        print(f"  监控感知:  ros2 topic echo /perception/boundary")
        print(f"  按 Ctrl+C 结束所有进程")
        print(f"{'='*55}\n")
        mgr.wait()

    except KeyboardInterrupt:
        print("\n[INFO] 用户中断，清理进程...")
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
