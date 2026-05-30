#!/usr/bin/env python3
"""
诊断脚本：检查 2026 仿真镜像中的摄像头 link 名称。

在 Docker 容器内运行：
    python3 check_camera_link.py

这会检查 robot.xacro 和 gazebo.xacro，告诉你应该用哪个 link 名称。
"""
import os
import re

# 可能的 xacro 文件路径
SEARCH_PATHS = [
    "/home/cyberdog_sim/src/cyberdog_simulator/cyberdog_robot/cyberdog_description/xacro/",
    "/home/cyberdog_sim/install/share/cyberdog_description/xacro/",
]


def find_camera_links(content, filename):
    """在 xacro 文件中查找摄像头相关的 link。"""
    # 查找所有 link 定义
    links = re.findall(r'<link\s+name=["\']([^"\']*(?:camera|Camera|D435|d435|RGB|rgb)[^"\']*)["\']', content)
    # 查找所有包含 camera/Camera/D435 的行
    camera_lines = []
    for i, line in enumerate(content.split('\n'), 1):
        if re.search(r'camera|Camera|D435|d435|RGB|rgb', line, re.IGNORECASE):
            camera_lines.append((i, line.strip()))

    return links, camera_lines


def main():
    print("=" * 60)
    print("2026 仿真摄像头 link 诊断")
    print("=" * 60)
    print()

    all_links = set()

    for search_path in SEARCH_PATHS:
        if not os.path.exists(search_path):
            print(f"路径不存在: {search_path}")
            continue

        print(f"扫描目录: {search_path}")
        print()

        for fname in os.listdir(search_path):
            if not fname.endswith('.xacro'):
                continue

            fpath = os.path.join(search_path, fname)
            with open(fpath, 'r', encoding='utf-8') as f:
                content = f.read()

            links, camera_lines = find_camera_links(content, fname)

            if links or camera_lines:
                print(f"  文件: {fname}")

                if links:
                    print(f"    摄像头相关 link 定义:")
                    for link in links:
                        print(f"      - {link}")
                        all_links.add(link)

                if camera_lines:
                    print(f"    摄像头相关行:")
                    for line_no, line_text in camera_lines[:10]:  # 只显示前10行
                        print(f"      L{line_no}: {line_text[:100]}")

                print()

    print("=" * 60)
    print("诊断结果:")
    print("=" * 60)
    print()

    if all_links:
        print(f"找到的摄像头 link 名称: {', '.join(sorted(all_links))}")
        print()
        print("请确认哪个是正确的摄像头 link 名称，然后在 inject_camera_plugin.py 中修改")
        print("CAMERA_PLUGIN_XML 中的 reference 和 frame_name。")
    else:
        print("未找到摄像头相关 link。可能需要检查 robot.xacro 文件。")
        print()
        print("请运行以下命令查看所有 link：")
        print("  grep -n 'link name' /home/cyberdog_sim/src/cyberdog_simulator/cyberdog_robot/cyberdog_description/xacro/robot.xacro")

    print()
    print("=" * 60)
    print("检查 gazebo.xacro 中是否已有摄像头配置:")
    print("=" * 60)
    print()

    for search_path in SEARCH_PATHS:
        gazebo_path = os.path.join(search_path, "gazebo.xacro")
        if not os.path.exists(gazebo_path):
            continue

        with open(gazebo_path, 'r', encoding='utf-8') as f:
            content = f.read()

        if 'libgazebo_ros_camera.so' in content:
            print(f"  {gazebo_path}: 已有摄像头插件")
        else:
            print(f"  {gazebo_path}: 缺少摄像头插件 (需要注入)")

        if 'liblegged_plugin.so' in content:
            print(f"    运动控制插件: 存在")
        else:
            print(f"    运动控制插件: 缺失!")


if __name__ == '__main__':
    main()
