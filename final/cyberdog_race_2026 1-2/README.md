# 2026 小米杯四足机器人大赛 - 荒野寻宝 (第一、二赛段)

本项目严格遵循 2026 年赛题要求，基于 2025 年参考代码进行架构对齐与逻辑重构。

## 重要：2026 仿真缺少摄像头和 LiDAR 传感器

2026 年仿真镜像 (`cyberdog_race2026.tar`) 中的 `gazebo.xacro` **没有配置 RGB 摄像头和 LiDAR 激光雷达传感器插件**，导致：
- `/rgb_camera/image_raw` 话题无数据 → 视觉检测全部失败
- `/scan` 话题无数据 → 无法用 LiDAR 检测石头障碍物

## 一键修复（推荐）

### 方法A：从宿主机执行

```bash
cd /home/chenyc/桌面/cyberdog_race_2026
bash setup_sensors.sh <容器名>
```

### 方法B：在容器内手动执行

```bash
# 将修复脚本复制到容器
docker cp safe_fix_camera.sh <容器名>:/home/cyberdog_sim/
docker cp test_camera.py <容器名>:/home/cyberdog_sim/

# 进入容器
docker exec -it <容器名> bash

# 在容器内运行安全修复脚本
cd /home/cyberdog_sim
bash safe_fix_camera.sh

# 杀死旧仿真并重启
killall -9 gazebo gzserver gzclient 2>/dev/null
ros2 launch cyberdog_gazebo race_gazebo.launch.py
```

### 修复后验证

```bash
# 在容器内验证传感器
docker exec -it <容器名> bash

# 检查 topic 是否存在
ros2 topic list | grep -E 'scan|camera'
# 预期输出:
#   /scan
#   /rgb_camera/image_raw

# 检查 LiDAR 数据（应看到 range 数据）
ros2 topic echo /scan --once

# 检查摄像头
python3 test_camera.py
# 预期输出: ✓ 摄像头工作正常! 15 帧 / 1.0s = 15.0 FPS
```

### 运行状态机

```bash
cd /home/cyberdog_sim
source install/setup.bash
python3 -m cyberdog_controller.FSM.fsm
```

启动时 FSM 会自动检测摄像头状态：
- ✓ 摄像头正常 → 视觉导航模式（精确跟踪赛道、检测横线、寻找小球）
- ✗ 摄像头不可用 → 预编程动作模式（定时盲走、预编程转弯）

## 常见问题

### Q: 仿真卡死 / 机器狗悬浮不落地

**原因：** 使用了不完整的 `gazebo_with_camera.xacro` 整体替换了 2026 版本的 `gazebo.xacro`。2026 镜像的运动控制插件配置与 2025 版本不兼容。

**解决：**
```bash
# 在容器内恢复原始文件（如果之前替换过）
cd /home/cyberdog_sim
bash restore_original.sh

# 然后用安全方式注入摄像头（只在 </robot> 前插入摄像头块）
bash safe_fix_camera.sh

# 重启仿真
killall -9 gazebo gzserver gzclient 2>/dev/null
ros2 launch cyberdog_gazebo race_gazebo.launch.py
```

**如果仍然卡死：** 完全重置容器（见下方"如何完全重置"）。

### Q: 摄像头验证脚本显示"未收到任何摄像头图像"

检查：
1. 是否重启了仿真？（修改 xacro 后必须重启 Gazebo）
2. 运行 `ros2 topic list | grep camera` 确认话题存在
3. 运行 `ros2 topic hz /rgb_camera/image_raw` 确认有数据发布
4. 运行 `python3 check_camera_link.py` 检查摄像头 link 名称是否正确

### Q: 如何完全重置

```bash
# 停止容器
docker stop <容器名>
docker rm <容器名>

# 重新导入镜像
docker load < cyberdog_race2026.tar

# 重新创建容器并运行
```

### Q: D435_camera_link 出现两次

如果 `robot.xacro` 中定义的摄像头 link 名称不是 `RGB_camera_link`，需要修改 `safe_fix_camera.sh` 和 `inject_camera_plugin.py` 中的 `reference` 和 `frame_name`。

运行诊断脚本确认正确的 link 名称：
```bash
python3 check_camera_link.py
```

## 修复原理

`safe_fix_camera.sh` 在原始 `gazebo.xacro` 的 `</robot>` 标签前注入摄像头传感器插件块，**不替换整个文件**，保留 2026 镜像自带的所有运动控制插件。

摄像头参数（与 2025 赛题一致）：
- 分辨率：320x180
- 帧率：15 FPS
- 话题：`/rgb_camera/image_raw`
- 蓝光亮起 = 摄像头工作正常

### 为什么之前的修复方式会卡死

之前的 `safe_fix_camera.sh` 直接用 `gazebo_with_camera.xacro`（基于 2025 版本）整体替换 `gazebo.xacro`。问题在于：
1. `gazebo_with_camera.xacro` 只有 288 行，缺少 2026 镜像中的关键运动控制配置
2. 2026 镜像的 `liblegged_plugin.so` 和 `libreal_time_control.so` 需要特定的参数配置
3. 整体替换导致运动控制插件参数丢失，机器狗无法正常站立

新方式只注入摄像头插件块，不触动任何原有配置。

## 项目架构

- **locomotion/**: 核心运动控制。`robot_control_cmd_lcmt.py` (LCM 指令), `Usergait_List.toml` (步态参数库)
- **camera/**: 视觉检测。
  - `path_scanner.py`: 黄色赛道边线检测 + 摄像头健康检查
  - `ball_scanner.py`: 橙色小球检测
- **FSM/**: 有限状态机。
  - `fsm.py`: 赛段调度器，启动时自动检查摄像头
  - `states/stage1.py`: Stage1 石径探路
  - `states/stage2.py`: Stage2 荒野寻珠

## 编译与运行

```bash
colcon build --packages-select cyberdog_controller
source install/setup.bash
python3 -m cyberdog_controller.FSM.fsm
```
