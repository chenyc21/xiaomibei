#!/usr/bin/env bash
# =============================================================================
# start_container.sh
# 启动 cyberdog_sim:v2026 仿真开发容器
#
# 前提：已通过 bash docker/load_image.sh 导入镜像
#
# 用法：
#   bash docker/start_container.sh          # 前台交互（直接进入容器 shell）
#   bash docker/start_container.sh --detach # 后台运行（推荐配合 VS Code Attach）
#   bash docker/start_container.sh --stop   # 停止并删除容器
# =============================================================================
set -euo pipefail

# ── 可配置参数 ────────────────────────────────────────────────
IMAGE_NAME="cyberdog_sim:v2026"
CONTAINER_NAME="cyberdog_sim_dev"

# 宿主机：本仓库根目录（自动检测，无需手动修改）
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# 容器内路径：
#   SIM_HOME  = 镜像内预置的仿真环境（含 src/cyberdog_simulator 等原装代码）
#   WS_MOUNT  = 我们的竞赛开发代码挂载点（VS Code 打开此目录）
SIM_HOME="/home/cyberdog_sim"
WS_MOUNT="${SIM_HOME}/my_workspace"

# ── 解析参数 ──────────────────────────────────────────────────
DETACH=false
STOP=false

for arg in "$@"; do
  case $arg in
    --detach) DETACH=true ;;
    --stop)   STOP=true ;;
    *)
      echo "未知参数: $arg"
      echo "用法: $0 [--detach] [--stop]"
      exit 1
      ;;
  esac
done

# ── 停止并删除容器 ────────────────────────────────────────────
if $STOP; then
  echo "[INFO] 停止并删除容器: $CONTAINER_NAME"
  sudo docker stop "$CONTAINER_NAME" 2>/dev/null || true
  sudo docker rm   "$CONTAINER_NAME" 2>/dev/null || true
  echo "[INFO] 已清理容器。"
  exit 0
fi

# ── 检查镜像是否存在 ──────────────────────────────────────────
if ! sudo docker image inspect "$IMAGE_NAME" &>/dev/null; then
  echo "[ERROR] 未找到镜像: $IMAGE_NAME"
  echo ""
  echo "        请先导入镜像："
  echo "          bash docker/load_image.sh"
  echo ""
  echo "        当前可用的 cyberdog 相关镜像："
  sudo docker images | grep -E "cyberdog|REPOSITORY" || sudo docker images | head -5
  exit 1
fi

# ── 若容器已在运行，直接 exec 进入 ───────────────────────────
if sudo docker inspect -f '{{.State.Running}}' "$CONTAINER_NAME" 2>/dev/null | grep -q "true"; then
  echo "[INFO] 容器 $CONTAINER_NAME 已在运行。"
  if ! $DETACH; then
    echo "[INFO] 直接进入容器 shell..."
    sudo docker exec -it "$CONTAINER_NAME" /bin/bash
  fi
  exit 0
fi

# ── 清理已停止的同名容器 ─────────────────────────────────────
sudo docker rm "$CONTAINER_NAME" 2>/dev/null || true

# ── X11 授权（Gazebo GUI 显示）────────────────────────────────
xhost +local:docker 2>/dev/null \
  || echo "[WARN] xhost 不可用，Gazebo GUI 可能无法显示（纯 WSL CLI 环境正常）"

# ── 确定运行标志和启动命令 ────────────────────────────────────
# --detach 模式：容器内没有终端，必须用 sleep infinity 保活，
#               否则 /bin/bash 发现没有 TTY 立刻退出（Exit 0）
# 前台模式：-it 分配伪终端，/bin/bash 会等待用户输入
if $DETACH; then
  RUN_FLAGS="-d"
  RUN_CMD="sleep infinity"
  echo "[INFO] 后台模式启动容器（配合 VS Code Attach）..."
else
  RUN_FLAGS="-it"
  RUN_CMD="/bin/bash"
  echo "[INFO] 前台交互模式启动容器..."
fi

# ── 启动容器 ──────────────────────────────────────────────────
#   --shm-size        Gazebo 物理仿真共享内存需求
#   --privileged      LCM 多播 + Gazebo 渲染权限
#   --network=host    LCM 使用 UDP 多播，必须与宿主机共享网络栈
#   -e DISPLAY        Gazebo / rviz2 GUI 显示
#   -v X11            X11 socket 转发，支持 GUI
#   -v REPO→WS_MOUNT  将本仓库竞赛代码挂载进容器（VS Code 打开此目录）
sudo docker run $RUN_FLAGS \
  --name "$CONTAINER_NAME" \
  --shm-size="1g" \
  --privileged=true \
  --network=host \
  -e DISPLAY="${DISPLAY:-:0}" \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "${REPO_ROOT}:${WS_MOUNT}" \
  "$IMAGE_NAME" \
  $RUN_CMD

# ── 后台模式提示 ─────────────────────────────────────────────
if $DETACH; then
  echo ""
  echo "======================================================"
  echo " 容器已在后台运行: $CONTAINER_NAME"
  echo " 镜像:  $IMAGE_NAME"
  echo " 挂载:  $REPO_ROOT"
  echo "    →   $WS_MOUNT (容器内)"
  echo "======================================================"
  echo ""
  echo " ▶  VS Code 附加到容器（3步）："
  echo "    1. Ctrl+Shift+P → Dev Containers: Attach to Running Container"
  echo "    2. 选择: $CONTAINER_NAME"
  echo "    3. File → Open Folder → $WS_MOUNT"
  echo ""
  echo " ▶  手动进入容器终端："
  echo "    sudo docker exec -it $CONTAINER_NAME /bin/bash"
  echo ""
  echo " ▶  容器内启动仿真（进入终端后执行）："
  echo "    cd $SIM_HOME"
  echo "    python3 src/cyberdog_simulator/cyberdog_gazebo/script/launchsim.py"
  echo ""
  echo " ▶  停止容器："
  echo "    bash docker/start_container.sh --stop"
  echo "======================================================"
fi
