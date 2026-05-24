#!/bin/bash
# ============================================================
# 恢复 2026 仿真镜像原始 gazebo.xacro
#
# 如果之前用 2025 版本的 gazebo.xacro 替换了 2026 版本，
# 导致仿真卡死/机器狗悬浮，运行此脚本恢复原始文件。
#
# 使用方法（在 Docker 容器内执行）：
#   docker exec -it <容器名> bash
#   cd /home/cyberdog_sim
#   bash restore_original.sh
# ============================================================

set -e

SRC_XACRO="/home/cyberdog_sim/src/cyberdog_simulator/cyberdog_robot/cyberdog_description/xacro/gazebo.xacro"
INSTALL_XACRO="/home/cyberdog_sim/install/share/cyberdog_description/xacro/gazebo.xacro"

echo "=== 恢复 2026 原始 gazebo.xacro ==="
echo ""

restored=0

for f in "${SRC_XACRO}" "${INSTALL_XACRO}"; do
    if [ -f "${f}.bak" ]; then
        cp "${f}.bak" "${f}"
        echo "  已恢复: ${f}.bak -> ${f}"
        restored=1
    elif [ -f "${f}.pre_camera_fix.bak" ]; then
        cp "${f}.pre_camera_fix.bak" "${f}"
        echo "  已恢复: ${f}.pre_camera_fix.bak -> ${f}"
        restored=1
    else
        echo "  未找到备份: ${f} 的 .bak 或 .pre_camera_fix.bak"
    fi
done

if [ "$restored" -eq 0 ]; then
    echo ""
    echo "没有找到任何备份文件。"
    echo "如果文件已被损坏，请从镜像重新导入："
    echo "  1. 停止容器: docker stop <容器名>"
    echo "  2. 删除容器: docker rm <容器名>"
    echo "  3. 重新导入镜像: docker load < cyberdog_race2026.tar"
    echo "  4. 重新创建容器: docker run ..."
else
    echo ""
    echo "=== 恢复完成 ==="
    echo "下一步："
    echo "  1. 杀死旧仿真: killall -9 gazebo gzserver gzclient 2>/dev/null"
    echo "  2. 重启仿真:   ros2 launch cyberdog_gazebo race_gazebo.launch.py"
    echo ""
    echo "仿真应该恢复正常（但没有摄像头）。"
    echo "之后请运行 safe_fix_camera.sh 安全添加摄像头插件。"
fi
