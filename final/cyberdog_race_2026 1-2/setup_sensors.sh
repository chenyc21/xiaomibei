#!/bin/bash
# ============================================================
# 一键注入 摄像头 + LiDAR 传感器到 2026 仿真容器
#
# 用法（从宿主机执行）：
#   bash setup_sensors.sh <容器名>
#
# 示例：
#   bash setup_sensors.sh cyberdog_race
# ============================================================
set -e

if [ -z "$1" ]; then
    echo "用法: bash setup_sensors.sh <容器名>"
    echo "示例: bash setup_sensors.sh cyberdog_race"
    exit 1
fi

CONTAINER="$1"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "=== 2026 仿真传感器一键部署 (摄像头 + LiDAR) ==="
echo "目标容器: $CONTAINER"
echo ""

# 检查容器是否存在且运行
if ! docker ps | grep -q "$CONTAINER"; then
    echo "错误: 容器 '$CONTAINER' 未运行"
    echo "请先启动容器: docker start $CONTAINER"
    exit 1
fi

# 复制注入脚本到容器
echo "[1/3] 复制注入脚本到容器 ..."
docker cp "$SCRIPT_DIR/inject_sensors.py" "$CONTAINER:/home/cyberdog_sim/"
echo "  完成"

# 在容器内执行注入
echo ""
echo "[2/3] 在容器内执行传感器注入 ..."
docker exec "$CONTAINER" python3 /home/cyberdog_sim/inject_sensors.py

# 验证
echo ""
echo "[3/3] 验证 ..."
echo ""
echo "下一步："
echo "  1. 杀死旧仿真并重启（在容器内执行）："
echo "     docker exec $CONTAINER bash -c 'killall -9 gazebo gzserver gzclient 2>/dev/null; ros2 launch cyberdog_gazebo race_gazebo.launch.py'"
echo ""
echo "  2. 验证传感器（在容器内另一个终端）："
echo "     docker exec $CONTAINER bash -c 'source /opt/ros/foxy/setup.bash; ros2 topic list | grep -E \"scan|camera\"'"
echo ""
echo "  3. 运行状态机："
echo "     docker exec -it $CONTAINER bash"
echo "     cd /home/cyberdog_sim"
echo "     source install/setup.bash"
echo "     python3 -m cyberdog_controller.FSM.fsm"
