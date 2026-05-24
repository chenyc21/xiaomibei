#!/bin/bash
# ============================================================
# 在 Docker 容器内部运行此脚本以修复 RGB 摄像头传感器缺失问题
#
# 问题：2026 仿真镜像的 gazebo.xacro 没有配置 RGB 摄像头传感器插件，
#       导致 /rgb_camera/image_raw 话题无数据，所有视觉检测超时失败。
#
# 修复方式：在原始 gazebo.xacro 的 </robot> 标签前注入摄像头插件块，
#           不替换整个文件，保留 2026 镜像自带的所有运动控制插件。
#
# 使用方法（在容器内）：
#   cd /home/cyberdog_sim
#   bash fix_camera_inside_docker.sh
#
# 执行后需要重启仿真：
#   killall -9 gazebo gzserver gzclient 2>/dev/null
#   ros2 launch cyberdog_gazebo race_gazebo.launch.py
# ============================================================

set -e

echo "=== 2026 仿真摄像头传感器修复 ==="
echo ""

# 直接调用 safe_fix_camera.sh
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
if [ -f "${SCRIPT_DIR}/safe_fix_camera.sh" ]; then
    bash "${SCRIPT_DIR}/safe_fix_camera.sh"
else
    echo "错误: 未找到 safe_fix_camera.sh"
    echo "请确保 safe_fix_camera.sh 与此脚本在同一目录下。"
    exit 1
fi
