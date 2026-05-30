#!/usr/bin/env python3
"""
在 2026 原始 gazebo.xacro 的 </robot> 标签前注入摄像头 + LiDAR 传感器插件。
不替换整个文件，只插入传感器块，保留 2026 镜像自带的所有运动控制插件。

用法（在 Docker 容器内执行）：
    python3 inject_sensors.py

修复后需要重启仿真：
    killall -9 gazebo gzserver gzclient 2>/dev/null
    ros2 launch cyberdog_gazebo race_gazebo.launch.py
"""
import sys
import os
import shutil

XACRO_PATHS = [
    "/home/cyberdog_sim/src/cyberdog_simulator/cyberdog_robot/cyberdog_description/xacro/gazebo.xacro",
    "/home/cyberdog_sim/install/share/cyberdog_description/xacro/gazebo.xacro",
]

# RGB 摄像头传感器插件
CAMERA_PLUGIN_XML = """
    <!-- RGB Camera sensor plugin (2026 仿真缺失) -->
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

# LiDAR 激光雷达传感器插件（正前方180度，12m范围，发布到 /scan topic）
LIDAR_PLUGIN_XML = """
    <!-- LiDAR laser scan sensor (2026 赛题石板路障碍物检测) -->
    <gazebo reference="lidar_link">
        <sensor name="realsense" type="ray">
            <always_on>true</always_on>
            <visualize>true</visualize>
            <pose>0.0 0 0.0 0 0 0</pose>
            <update_rate>5</update_rate>
            <ray>
            <scan>
                <horizontal>
                <samples>180</samples>
                <resolution>1.000000</resolution>
                <min_angle>-1.5700</min_angle>
                <max_angle>1.5700</max_angle>
                </horizontal>
            </scan>
            <range>
                <min>0.01</min>
                <max>12.00</max>
                <resolution>0.015000</resolution>
            </range>
            <noise>
                <type>gaussian</type>
                <mean>0.0</mean>
                <stddev>0.01</stddev>
            </noise>
            </ray>
            <plugin name="cyberdog_laserscan" filename="libgazebo_ros_ray_sensor.so">
            <ros>
                <remapping>~/out:=scan</remapping>
            </ros>
            <output_type>sensor_msgs/LaserScan</output_type>
            <frame_name>lidar_link</frame_name>
            </plugin>
        </sensor>
    </gazebo>
"""


def inject_sensors(xacro_path):
    """在 gazebo.xacro 的 </robot> 标签前注入传感器插件块。"""
    if not os.path.exists(xacro_path):
        print(f"  跳过: 文件不存在 {xacro_path}")
        return {'camera': False, 'lidar': False}

    with open(xacro_path, 'r', encoding='utf-8') as f:
        content = f.read()

    if '</robot>' not in content:
        print(f"  错误: 未找到 </robot> 标签 {xacro_path}")
        return {'camera': False, 'lidar': False}

    # 备份原始文件
    backup_path = xacro_path + ".bak"
    if not os.path.exists(backup_path):
        shutil.copy2(xacro_path, backup_path)
        print(f"  已备份: {os.path.basename(xacro_path)} -> {os.path.basename(backup_path)}")

    has_camera = 'libgazebo_ros_camera.so' in content
    has_lidar = 'libgazebo_ros_ray_sensor.so' in content

    # 按需注入
    insert_block = ""
    if not has_camera:
        insert_block += CAMERA_PLUGIN_XML
        print(f"  注入: 摄像头传感器")
    else:
        print(f"  跳过: 摄像头传感器已存在")

    if not has_lidar:
        insert_block += LIDAR_PLUGIN_XML
        print(f"  注入: LiDAR 激光雷达传感器")
    else:
        print(f"  跳过: LiDAR 传感器已存在")

    if insert_block:
        new_content = content.replace('</robot>', insert_block + '\n</robot>')
        with open(xacro_path, 'w', encoding='utf-8') as f:
            f.write(new_content)
        print(f"  已保存: {os.path.basename(xacro_path)}")
    else:
        print(f"  无需修改")

    return {'camera': not has_camera or has_camera, 'lidar': not has_lidar or has_lidar}


def main():
    print("=" * 60)
    print("2026 仿真传感器注入工具 (摄像头 + LiDAR)")
    print("=" * 60)
    print()
    print("策略: 在原始 gazebo.xacro 的 </robot> 标签前插入传感器插件块")
    print("      不替换整个文件，保留 2026 镜像自带的所有运动控制插件")
    print()

    results = {}
    for path in XACRO_PATHS:
        print(f"处理: {path}")
        results[path] = inject_sensors(path)
        print()

    # 验证
    print("-" * 60)
    print("验证:")
    for path in XACRO_PATHS:
        if os.path.exists(path):
            with open(path, 'r', encoding='utf-8') as f:
                content = f.read()

            name = os.path.basename(path)
            checks = {
                '运动控制 (legged)': 'liblegged_plugin.so',
                '实时控制 (rt_control)': 'libreal_time_control.so',
                '足端接触 (foot_contact)': 'libfoot_contact_plugin.so',
                '摄像头 (camera)': 'libgazebo_ros_camera.so',
                '激光雷达 (LiDAR)': 'libgazebo_ros_ray_sensor.so',
            }
            print(f"  {name}:")
            for label, keyword in checks.items():
                status = 'OK' if keyword in content else '缺失'
                print(f"    {label}: {status}")

    print()
    print("=" * 60)
    print("注入完成！")
    print("=" * 60)
    print()
    print("下一步操作 (在容器内执行)：")
    print("  1. 杀死旧仿真:")
    print("     killall -9 gazebo gzserver gzclient 2>/dev/null")
    print()
    print("  2. 重启仿真:")
    print("     ros2 launch cyberdog_gazebo race_gazebo.launch.py")
    print()
    print("  3. 验证传感器 (在容器内开另一个终端):")
    print("     # 检查 topic")
    print("     ros2 topic list | grep -E 'scan|camera'")
    print("     预期输出: /scan 和 /rgb_camera/image_raw")
    print()
    print("     # 检查 LiDAR 数据")
    print("     ros2 topic echo /scan --once")
    print()
    print("  4. 运行状态机:")
    print("     cd /home/cyberdog_sim")
    print("     source install/setup.bash")
    print("     python3 -m cyberdog_controller.FSM.fsm")
    print()
    print("话题映射:")
    print("  摄像头:  /rgb_camera/image_raw  (sensor_msgs/Image, 320x180)")
    print("  LiDAR:   /scan                 (sensor_msgs/LaserScan, 180 samples, 12m)")
    print()
    print("LiDAR 启用后，机器狗将自动:")
    print("  - 检测前方石头并切换超高/极限抬腿步态跨越")
    print("  - 卡死时侧移摇摆脱困（极小后退，不会退到后面石头上）")


if __name__ == '__main__':
    main()
