#!/bin/bash
# ============================================================
# 安全修复 2026 仿真 RGB 摄像头传感器缺失问题
#
# 策略：在原始 gazebo.xacro 的 </robot> 标签前注入摄像头插件块，
#       不替换整个文件，保留 2026 镜像自带的所有运动控制插件。
#
# 使用方法（在 Docker 容器内执行）：
#   docker exec -it <容器名> bash
#   cd /home/cyberdog_sim
#   bash safe_fix_camera.sh
#
# 修复后需要重启仿真：
#   killall -9 gazebo gzserver gzclient 2>/dev/null
#   ros2 launch cyberdog_gazebo race_gazebo.launch.py
# ============================================================

set -e

SRC_XACRO="/home/cyberdog_sim/src/cyberdog_simulator/cyberdog_robot/cyberdog_description/xacro/gazebo.xacro"
INSTALL_XACRO="/home/cyberdog_sim/install/share/cyberdog_description/xacro/gazebo.xacro"

echo "=== 2026 仿真 RGB 摄像头安全修复 ==="
echo ""

# ---------- 第1步：检查原始文件是否已被替换 ----------
echo "[1/4] 检查原始文件状态 ..."

for f in "${SRC_XACRO}" "${INSTALL_XACRO}"; do
    if [ -f "$f" ]; then
        if ! grep -q 'liblegged_plugin.so' "$f" 2>/dev/null; then
            echo ""
            echo "错误: $f 已被替换为不完整的版本（缺少运动控制插件）。"
            echo ""
            echo "请先恢复原始文件："
            echo "  方法1: 运行 restore_original.sh（如果有 .bak 备份）"
            echo "  方法2: 重新导入镜像："
            echo "    docker stop <容器名>"
            echo "    docker rm <容器名>"
            echo "    docker load < cyberdog_race2026.tar"
            echo "    docker run -it ..."
            echo ""
            echo "恢复原始文件后，重新运行此脚本。"
            exit 1
        fi
        echo "  $(basename $f): OK (运动控制插件存在)"
    else
        echo "  错误: $f 不存在"
        exit 1
    fi
done

# ---------- 第2步：检查是否已有摄像头插件 ----------
echo ""
echo "[2/4] 检查摄像头插件状态 ..."

if grep -q 'libgazebo_ros_camera.so' "${INSTALL_XACRO}" 2>/dev/null; then
    echo "  摄像头传感器插件已存在，无需修复。"
    echo ""
    echo "如果仿真仍然异常，请先恢复原始文件后重新运行此脚本。"
    exit 0
fi

echo "  摄像头插件不存在，开始注入 ..."

# ---------- 第3步：注入摄像头插件 ----------
echo ""
echo "[3/4] 在 </robot> 标签前注入摄像头插件块 ..."

for f in "${SRC_XACRO}" "${INSTALL_XACRO}"; do
    if [ -f "$f" ]; then
        # 备份
        if [ ! -f "$f.bak" ]; then
            cp "$f" "$f.bak"
            echo "  已备份: $f -> $f.bak"
        fi

        # 使用 Python 注入（heredoc 避免 shell 转义问题）
        XACRO_FILE="$f" python3 << 'PYTHON_EOF'
import sys
import os

filepath = os.environ.get('XACRO_FILE', '')

camera_block = """
    <!-- RGB Camera sensor plugin (移植自 2025 赛题，2026 仿真缺失此传感器) -->
    <gazebo reference="RGB_camera_link">
        <sensor type="camera" name="rgb camera">
            <always_on>true</always_on>
            <update_rate>15.0</update_rate>
            <camera name="rgb_camera">
                <horizontal_fov>1.46608</horizontal_fov>
                <image>
                    <width>320</width>
                    <height>180</height>
                    <format>R8G8B8</format>
                </image>
                <distortion>
                    <k1>0.0</k1>
                    <k2>0.0</k2>
                    <k3>0.0</k3>
                    <p1>0.0</p1>
                    <p2>0.0</p2>
                    <center> 0.5 0.5 </center>
                </distortion>
            </camera>
            <plugin name="rgb_camera_plugin" filename="libgazebo_ros_camera.so">
                <ros>
                    <remapping>~/image_raw:=image_raw</remapping>
                    <remapping>~/camera_info:=camera_info</remapping>
                </ros>
                <camera_name>rgb_camera</camera_name>
                <frame_name>RGB_camera_link</frame_name>
                <hack_baseline>0.2</hack_baseline>
            </plugin>
        </sensor>
    </gazebo>
"""

with open(filepath, 'r') as fh:
    content = fh.read()

if '</robot>' in content:
    content = content.replace('</robot>', camera_block + '\n</robot>')
    with open(filepath, 'w') as fh:
        fh.write(content)
    print(f'  已注入: {filepath}')
else:
    print(f'  错误: 未找到 </robot> 标签')
    sys.exit(1)
PYTHON_EOF
    fi
done

# ---------- 第4步：验证 ----------
echo ""
echo "[4/4] 验证修复结果 ..."

errors=0

for f in "${SRC_XACRO}" "${INSTALL_XACRO}"; do
    if [ -f "$f" ]; then
        fname=$(basename "$f")

        if grep -q 'libgazebo_ros_camera.so' "$f"; then
            echo "  $fname: 摄像头插件已注入"
        else
            echo "  错误: $fname 摄像头插件未注入"
            errors=$((errors + 1))
        fi

        if grep -q 'liblegged_plugin.so' "$f"; then
            echo "  $fname: 运动控制插件已保留"
        else
            echo "  警告: $fname 运动控制插件缺失"
            errors=$((errors + 1))
        fi

        if grep -q 'libreal_time_control.so' "$f"; then
            echo "  $fname: 实时控制插件已保留"
        else
            echo "  警告: $fname 实时控制插件缺失"
            errors=$((errors + 1))
        fi
    fi
done

if [ $errors -gt 0 ]; then
    echo ""
    echo "=== 修复有错误，请检查 ==="
    exit 1
fi

echo ""
echo "=== 修复完成 ==="
echo ""
echo "下一步操作："
echo "  1. 杀死旧仿真:  killall -9 gazebo gzserver gzclient 2>/dev/null"
echo "  2. 重启仿真:    ros2 launch cyberdog_gazebo race_gazebo.launch.py"
echo "  3. 验证摄像头:  python3 test_camera.py"
echo ""
echo "摄像头将发布到: /rgb_camera/image_raw (320x180, 15 FPS, ROS2 topic)"
echo "蓝光亮起 = 摄像头工作正常"
