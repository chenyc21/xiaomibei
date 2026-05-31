#!/usr/bin/env bash
set -euo pipefail

SIM_ROOT="${SIM_ROOT:-/home/cyberdog_sim}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/galactic/setup.bash}"
SIM_SETUP="${SIM_SETUP:-$SIM_ROOT/install/setup.bash}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="${LOG_DIR:-/tmp/cyberdog_stage5_logs}"
KEEP_GAZEBO_AFTER_RUN="${KEEP_GAZEBO_AFTER_RUN:-1}"
AUTO_DOCKER="${AUTO_DOCKER:-1}"
DOCKER_IMAGE="${DOCKER_IMAGE:-cyberdog_sim:v2026}"
GAZEBO_PID=""
VISUAL_PID=""
CONTROL_PID=""

mkdir -p "$LOG_DIR"

print_env_help() {
    cat <<EOF

环境没有准备好，仿真还不能启动。

需要能找到这两个 setup 文件：
  ROS_SETUP=$ROS_SETUP
  SIM_SETUP=$SIM_SETUP

常见解决方式：
  1. 进入 cyberdog 仿真 Docker/容器后再运行本脚本。
  2. 如果 ROS 或仿真目录不在默认位置，显式指定路径，例如：
     ROS_SETUP=/opt/ros/galactic/setup.bash SIM_ROOT=/home/cyberdog_sim ./run_stage5_one_terminal.sh
     或：
     SIM_SETUP=/path/to/cyberdog_sim/install/setup.bash ./run_stage5_one_terminal.sh

EOF
}

run_inside_docker_if_available() {
    if [[ "$AUTO_DOCKER" != "1" || "${IN_CYBERDOG_DOCKER:-0}" == "1" ]]; then
        return 1
    fi

    if ! command -v docker >/dev/null 2>&1; then
        return 1
    fi

    if ! docker image inspect "$DOCKER_IMAGE" >/dev/null 2>&1; then
        return 1
    fi

    echo "当前 shell 没有完整 ROS/cyberdog 环境，自动进入 Docker 镜像 $DOCKER_IMAGE 运行..."

    local docker_tty=()
    if [[ -t 0 && -t 1 ]]; then
        docker_tty=(-it)
    else
        docker_tty=(-i)
    fi

    docker run --rm "${docker_tty[@]}" \
        --privileged \
        --network host \
        -e DISPLAY="${DISPLAY:-:0}" \
        -e QT_X11_NO_MITSHM=1 \
        -e IN_CYBERDOG_DOCKER=1 \
        -e KEEP_GAZEBO_AFTER_RUN="$KEEP_GAZEBO_AFTER_RUN" \
        -e LOG_DIR="$LOG_DIR" \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v "$REPO_ROOT:$REPO_ROOT" \
        -w "$REPO_ROOT" \
        "$DOCKER_IMAGE" \
        bash -lc "./$(basename "$0")"
    exit $?
}

load_environment() {
    set +u

    if [[ -f "$ROS_SETUP" ]]; then
        source "$ROS_SETUP"
    elif command -v ros2 >/dev/null 2>&1; then
        echo "没有找到 ROS_SETUP=$ROS_SETUP，但当前 shell 已经有 ros2，继续使用现有 ROS 环境。"
    else
        for candidate in /opt/ros/*/setup.bash; do
            if [[ -f "$candidate" ]]; then
                echo "没有找到 ROS_SETUP=$ROS_SETUP，自动使用 $candidate"
                source "$candidate"
                break
            fi
        done
    fi

    if ! command -v ros2 >/dev/null 2>&1; then
        if run_inside_docker_if_available; then
            exit 0
        fi
        print_env_help
        exit 1
    fi

    if [[ -f "$SIM_SETUP" ]]; then
        source "$SIM_SETUP"
    else
        if run_inside_docker_if_available; then
            exit 0
        fi
        print_env_help
        exit 1
    fi

    set -u
}

load_environment

echo "日志目录: $LOG_DIR"

stop_existing_sim() {
    echo "清理旧的 Gazebo/control/Stage5 残留进程..."
    pkill -TERM -f "python3 run_stage5.py" 2>/dev/null || true
    pkill -TERM -f "python3 spawn_stage5.py" 2>/dev/null || true
    pkill -TERM -f "cyberdog_control m s" 2>/dev/null || true
    pkill -TERM -f "ros2 launch cyberdog_gazebo cyberdog_control_launch.py" 2>/dev/null || true
    pkill -TERM -f "ros2 launch cyberdog_visual cyberdog_visual.launch.py" 2>/dev/null || true
    pkill -TERM -f "ros2 launch cyberdog_gazebo race_gazebo.launch.py" 2>/dev/null || true
    pkill -TERM -f "gzserver .*race.world" 2>/dev/null || true
    pkill -TERM -f "gzclient" 2>/dev/null || true
    sleep 3
}

start_group() {
    local log_file="$1"
    shift
    setsid "$@" >"$log_file" 2>&1 &
    echo "$!"
}

stop_existing_sim

echo "启动 Gazebo..."
cd "$SIM_ROOT"
GAZEBO_PID="$(start_group "$LOG_DIR/gazebo.log" ros2 launch cyberdog_gazebo race_gazebo.launch.py paused:=False)"

cleanup() {
    echo
    echo "正在停止本脚本启动的后台进程..."
    for pid in "$CONTROL_PID" "$VISUAL_PID" "$GAZEBO_PID"; do
        if [[ -n "$pid" ]]; then
            kill -- "-$pid" 2>/dev/null || kill "$pid" 2>/dev/null || true
        fi
    done
}
trap cleanup EXIT INT TERM

echo "等待 Gazebo 服务..."
for _ in $(seq 1 45); do
    if ros2 service list 2>/dev/null | grep -q '^/spawn_entity$'; then
        break
    fi
    sleep 1
done

if ! ros2 service list 2>/dev/null | grep -q '^/spawn_entity$'; then
    echo "Gazebo 没有启动成功，请看 $LOG_DIR/gazebo.log"
    exit 1
fi

echo "取消 Gazebo 暂停..."
gz world -w earth -p 0 >/dev/null 2>&1 || true
ros2 service call /unpause_physics std_srvs/srv/Empty "{}" >/dev/null 2>&1 || true

cd "$REPO_ROOT"
echo "移动到第五赛段桥上开始位置..."
if ! python3 spawn_stage5.py; then
    echo
    echo "普通传送失败，尝试删除并重生 robot 到第五赛段桥上开始位置..."
    if ! python3 spawn_stage5.py --respawn; then
        echo
        echo "移动到第五赛段桥上开始位置失败。Gazebo 会保持打开，方便你观察现场。"
        echo "Gazebo 日志: $LOG_DIR/gazebo.log"
        echo "按 Ctrl+C 退出并清理后台进程。"
        while true; do
            sleep 3600
        done
    fi
fi
sleep 3

echo "启动运动控制..."
cd "$SIM_ROOT"
CONTROL_PID="$(start_group "$LOG_DIR/control.log" ros2 launch cyberdog_gazebo cyberdog_control_launch.py)"
sleep 8

echo "再次取消 Gazebo 暂停..."
gz world -w earth -p 0 >/dev/null 2>&1 || true
ros2 service call /unpause_physics std_srvs/srv/Empty "{}" >/dev/null 2>&1 || true

cd "$REPO_ROOT"
echo "开始运行第五赛段..."
python3 -u run_stage5.py

if [[ "$KEEP_GAZEBO_AFTER_RUN" == "1" ]]; then
    echo
    echo "第五赛段已完成，Gazebo 会保持打开，方便你用鼠标观察。"
    echo "按 Ctrl+C 退出并清理后台进程。"
    while true; do
        sleep 3600
    done
fi
