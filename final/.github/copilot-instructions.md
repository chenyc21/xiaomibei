# CyberDog Sim — Copilot 工作规范

## 项目背景
2026年全国大学生计算机系统能力大赛（小米杯）"荒野寻宝"赛道。
CyberDog 四足机器人自主完成6关障碍赛。
技术栈：ROS2 Galactic + Gazebo 11 + LCM，运行在 Docker 容器 `cyberdog_sim:v2026` 内。

---

## ⚠️ 关键路径规则（违反会导致代码丢失）

| 路径 | 说明 |
|------|------|
| `/home/cyberdog_sim/my_workspace/` | **唯一安全写代码区域**，bind mount 到宿主机 |
| `/home/cyberdog_sim/` 根目录 | 镜像内预置环境，容器重建后恢复原样 |
| `/tmp/`、`/root/` | 容器删除后永久丢失 |

**所有新建文件、修改代码必须在 `/home/cyberdog_sim/my_workspace/` 内。**
等价于宿主机路径 `/home/ll15501582006/cyberdog_recovery/cyberdog_sim/`。

---

## 目录结构（容器内）

```
/home/cyberdog_sim/                    ← 镜像预置仿真环境（只读对待）
├── src/cyberdog_simulator/            ← 官方仿真代码
├── src/cyberdog_locomotion/           ← 官方运动控制代码
├── install/                           ← 预编译结果
└── my_workspace/                      ← ⭐ bind mount，开发代码写这里
    └── src/
        ├── cyberdog_locomotion/       ← 若需修改运动控制，在此覆盖
        ├── cyberdog_simulator/
        │   ├── competition_msgs/      ← ROS2 消息定义
        │   ├── competition_manager/   ← 全局状态机（6关）
        │   ├── competition_perception/← 视觉感知节点
        │   ├── competition_motion_bridge/ ← ROS2↔LCM 桥接
        │   └── competition_voice/    ← 语音播报节点
        └── ...
```

---

## 构建规范

```bash
# colcon build 必须在 /home/cyberdog_sim 执行（不是 my_workspace）
cd /home/cyberdog_sim
source /opt/ros/galactic/setup.bash
colcon build --merge-install --symlink-install \
  --packages-select competition_msgs competition_manager \
    competition_perception competition_motion_bridge competition_voice
```

## 仿真启动

```bash
# 必须从 /home/cyberdog_sim 启动（不是 my_workspace）
cd /home/cyberdog_sim
source /opt/ros/galactic/setup.bash
source install/setup.bash
python3 src/cyberdog_simulator/cyberdog_gazebo/script/launchsim.py
```

---

## 通信协议

- **ROS2 Topics**（上层逻辑）：`/competition/state`, `/motion/command`, `/perception/objects`, `/perception/boundary`, `/robot/state`
- **LCM Channels**（底层运动控制）：`robot_control_cmd`, `state_estimator`, `gamepad_lcm`
- LCM 需要 `--network=host` 和 UDP 多播

## 竞赛阶段
1. 石头路（视觉循线）→ 2. 打球（YOLO检测+HIT动作）→ 3. 弯道（PD边界控制）→ 4. 深洞隧道（探索+目标检索）→ 5. 平衡木（低速慢走+跳下）→ 6. 踢球（kick动作→终点趴下）
