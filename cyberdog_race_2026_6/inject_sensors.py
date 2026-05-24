#!/usr/bin/env python3
"""
在 2026 原始 gazebo.xacro 的 </robot> 标签前注入摄像头 + LiDAR 传感器插件。
复制自组长第1-2赛段代码，第六赛段独立测试时使用。

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

LIDAR_PLUGIN_XML = """
    <!-- LiDAR laser scan sensor -->
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
    if not os.path.exists(xacro_path):
        print(f"  跳过: 文件不存在 {xacro_path}")
        return

    with open(xacro_path, 'r', encoding='utf-8') as f:
        content = f.read()

    if '</robot>' not in content:
        print(f"  错误: 未找到 </robot> 标签 {xacro_path}")
        return

    backup_path = xacro_path + ".bak"
    if not os.path.exists(backup_path):
        shutil.copy2(xacro_path, backup_path)
        print(f"  已备份: {os.path.basename(xacro_path)} -> {os.path.basename(backup_path)}")

    has_camera = 'libgazebo_ros_camera.so' in content
    has_lidar = 'libgazebo_ros_ray_sensor.so' in content

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


def main():
    print("=" * 60)
    print("2026 仿真传感器注入工具 (摄像头 + LiDAR)")
    print("=" * 60)

    for path in XACRO_PATHS:
        print(f"处理: {path}")
        inject_sensors(path)
        print()

    print("注入完成！请重启仿真:")
    print("  killall -9 gazebo gzserver gzclient 2>/dev/null")
    print("  ros2 launch cyberdog_gazebo race_gazebo.launch.py")
    print()
    print("验证传感器:")
    print("  ros2 topic list | grep -E 'scan|camera'")
    print("  预期: /scan 和 /rgb_camera/image_raw")


if __name__ == '__main__':
    main()
