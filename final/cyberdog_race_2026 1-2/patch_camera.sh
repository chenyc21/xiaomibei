#!/bin/bash
# ============================================================
# 安全修补 2026 仿真容器：注入 RGB 摄像头传感器插件
#
# 从宿主机一键修复：将修复脚本复制到容器内执行注入操作。
# 不替换整个 gazebo.xacro 文件，只在 </robot> 前插入摄像头块。
#
# 使用方法: bash patch_camera.sh [container_name]
# ============================================================

CONTAINER=${1:-cyberdog_sim}
SRC_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "=== 安全修补 2026 仿真容器 — 注入 RGB 摄像头传感器 ==="

# 检查容器是否在运行
if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER}$"; then
    echo "容器 '${CONTAINER}' 未运行。"
    echo "请先启动容器，然后重新运行此脚本。"
    exit 1
fi

# 检查修复脚本是否存在
if [ ! -f "${SRC_DIR}/safe_fix_camera.sh" ]; then
    echo "错误: 未找到 ${SRC_DIR}/safe_fix_camera.sh"
    exit 1
fi

echo "[1/3] 将修复脚本复制到容器 ..."
docker cp "${SRC_DIR}/safe_fix_camera.sh" "${CONTAINER}":/home/cyberdog_sim/safe_fix_camera.sh

echo "[2/3] 在容器内执行安全修复 ..."
docker exec "${CONTAINER}" bash -c "
    cd /home/cyberdog_sim && bash safe_fix_camera.sh
"

echo ""
echo "[3/3] 完成！"
echo ""
echo "下一步（在容器内执行）："
echo "  killall -9 gazebo gzserver gzclient 2>/dev/null"
echo "  ros2 launch cyberdog_gazebo race_gazebo.launch.py"
echo ""
echo "验证摄像头（在另一个终端）："
echo "  docker exec -it ${CONTAINER} bash"
echo "  python3 /home/cyberdog_sim/test_camera.py"
