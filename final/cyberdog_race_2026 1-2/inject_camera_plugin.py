#!/usr/bin/env python3
"""
在 2026 原始 gazebo.xacro 的 </robot> 标签前注入 RGB 摄像头传感器插件。
不替换整个文件，只插入摄像头块，保留 2026 镜像自带的所有运动控制插件。

用法（在 Docker 容器内执行）：
    python3 inject_camera_plugin.py

修复后需要重启仿真：
    killall -9 gazebo gzserver gzclient 2>/dev/null
    ros2 launch cyberdog_gazebo race_gazebo.launch.py
"""
import sys
import os
import shutil

# 两个需要修补的文件路径
XACRO_PATHS = [
    "/home/cyberdog_sim/src/cyberdog_simulator/cyberdog_robot/cyberdog_description/xacro/gazebo.xacro",
    "/home/cyberdog_sim/install/share/cyberdog_description/xacro/gazebo.xacro",
]

# RGB 摄像头传感器插件 XML 块（移植自 2025 赛题，适配 2026 link 名称）
# 注意：使用 RGB_camera_link 作为 reference，这是 robot.xacro 中定义的摄像头 link
CAMERA_PLUGIN_XML = """
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


def inject_camera(xacro_path):
    """在 gazebo.xacro 的 </robot> 标签前注入摄像头插件块。"""
    if not os.path.exists(xacro_path):
        print(f"  跳过: 文件不存在 {xacro_path}")
        return False

    with open(xacro_path, 'r', encoding='utf-8') as f:
        content = f.read()

    # 检查是否已有摄像头插件
    if 'libgazebo_ros_camera.so' in content:
        print(f"  跳过: 摄像头插件已存在 {xacro_path}")
        return True

    # 检查是否有 </robot> 标签
    if '</robot>' not in content:
        print(f"  错误: 未找到 </robot> 标签 {xacro_path}")
        return False

    # 备份原始文件
    backup_path = xacro_path + ".bak"
    if not os.path.exists(backup_path):
        shutil.copy2(xacro_path, backup_path)
        print(f"  已备份: {xacro_path} -> {backup_path}")

    # 在 </robot> 前插入摄像头块
    new_content = content.replace('</robot>', CAMERA_PLUGIN_XML + '\n</robot>')

    with open(xacro_path, 'w', encoding='utf-8') as f:
        f.write(new_content)

    print(f"  已注入: {xacro_path}")
    return True


def main():
    print("=" * 60)
    print("2026 仿真 RGB 摄像头传感器注入工具")
    print("=" * 60)
    print()
    print("策略: 在原始 gazebo.xacro 的 </robot> 标签前插入摄像头插件块")
    print("      不替换整个文件，保留 2026 镜像自带的所有运动控制插件")
    print()

    success = True
    for path in XACRO_PATHS:
        print(f"处理: {path}")
        if not inject_camera(path):
            success = False
        print()

    if success:
        # 验证
        print("验证修复结果:")
        for path in XACRO_PATHS:
            if os.path.exists(path):
                with open(path, 'r', encoding='utf-8') as f:
                    content = f.read()

                has_camera = 'libgazebo_ros_camera.so' in content
                has_legged = 'liblegged_plugin.so' in content
                has_rt_control = 'libreal_time_control.so' in content
                has_foot_contact = 'libfoot_contact_plugin.so' in content

                print(f"  {os.path.basename(path)}:")
                print(f"    摄像头插件: {'OK' if has_camera else '缺失'}")
                print(f"    运动控制插件: {'OK' if has_legged else '缺失'}")
                print(f"    实时控制插件: {'OK' if has_rt_control else '缺失'}")
                print(f"    足端接触插件: {'OK' if has_foot_contact else '缺失'}")

        print()
        print("=" * 60)
        print("注入完成！")
        print("=" * 60)
        print()
        print("下一步操作：")
        print("  1. 杀死旧仿真: killall -9 gazebo gzserver gzclient 2>/dev/null")
        print("  2. 重启仿真:   ros2 launch cyberdog_gazebo race_gazebo.launch.py")
        print("  3. 验证摄像头: python3 test_camera.py")
        print()
        print("摄像头将发布到: /rgb_camera/image_raw (320x180, 15 FPS)")
        print("蓝光亮起 = 摄像头工作正常")
    else:
        print()
        print("注入失败，请检查错误信息。")
        sys.exit(1)


if __name__ == '__main__':
    main()
