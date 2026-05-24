#!/usr/bin/env python3
"""
competition_motion_bridge.py — 将 /competition/motion_cmd (ROS2) 桥接到 LCM gamepad_lcmt
参考去年代码 (main.py)：mode=12→b=1(RecoveryStand), mode=11→y=1(Locomotion)
速度映射：
  leftStickAnalog[1]  = vel_x    (直接 m/s，clamped to ±2.0)
  leftStickAnalog[0]  = -vel_y   (注意符号，来自 command_interface.cpp:521)
  rightStickAnalog[0] = -vel_yaw (注意符号，来自 command_interface.cpp:522)
"""

import sys
import os
import time
import threading

# LCM Python 绑定路径
sys.path.insert(0, '/usr/local/lib/python3.8/site-packages')
# gamepad_lcmt Python 类型路径
LCM_TYPES_DIR = '/home/cyberdog_sim/src/cyberdog_locomotion/common/lcm_type/lcm'
sys.path.insert(0, LCM_TYPES_DIR)

import lcm
from gamepad_lcmt import gamepad_lcmt
from state_estimator_lcmt import state_estimator_lcmt

import rclpy
from rclpy.node import Node
from competition_msgs.msg import MotionCommand, RobotState
from cyberdog_msg.msg import YamlParam

# ─── 常量 ────────────────────────────────────────────────────────
MODE_STOP         = MotionCommand.MODE_STOP
MODE_STAND_UP     = MotionCommand.MODE_STAND_UP
MODE_LIE_DOWN     = MotionCommand.MODE_LIE_DOWN
MODE_VELOCITY_CTRL= MotionCommand.MODE_VELOCITY_CTRL
MODE_LOW_WALK     = MotionCommand.MODE_LOW_WALK
MODE_JUMP_DOWN    = MotionCommand.MODE_JUMP_DOWN
MODE_HIT_FORWARD  = MotionCommand.MODE_HIT_FORWARD
MODE_KICK_FORWARD = MotionCommand.MODE_KICK_FORWARD

# RecoveryStand 按下后保持发送的时长（避免控制器未接收到）
STAND_BUTTON_HOLD_SEC = 3.0
# 发布 gamepad_lcmt 的频率
PUBLISH_HZ = 50.0
PUBLISH_DT = 1.0 / PUBLISH_HZ

class MotionBridge(Node):
    def __init__(self):
        super().__init__('competition_motion_bridge')

        # LCM 实例：发布和订阅分离，避免线程安全问题
        self._lcm_pub = lcm.LCM()   # 仅用于 publish gamepad（主线程）
        self._lcm_sub = lcm.LCM()   # 仅用于 subscribe state_estimator（子线程）

        # 当前 gamepad 状态（50Hz 循环不断发送）
        self._gp = gamepad_lcmt()
        self._gp_lock = threading.Lock()

        # 状态跟踪
        self._in_loco_mode = False
        self._stand_button_until = 0.0   # 时间戳，之前一直发 b=1

        # ── 初始化：切换为 gamepad 控制模式 ──────────────────────────
        self._yaml_pub = self.create_publisher(YamlParam, 'yaml_parameter', 10)
        self.get_logger().info('[Bridge] 等待 1s 让 yaml_parameter 订阅者就绪...')
        time.sleep(1.0)
        self._send_use_rc_0()

        # ── 发布 /robot/state ─────────────────────────────────────
        self._robot_state_pub = self.create_publisher(RobotState, '/robot/state', 10)
        # 缓存最新状态（由 LCM 子线程写入，由 ROS2 定时器线程读取发布）
        self._pending_state: RobotState | None = None
        self._state_lock = threading.Lock()

        # ── 订阅运动指令 ───────────────────────────────────────────
        self._cmd_sub = self.create_subscription(
            MotionCommand, '/competition/motion_cmd',
            self._on_motion_cmd, 10)

        # ── 50Hz 发布定时器 ────────────────────────────────────────
        self._timer = self.create_timer(PUBLISH_DT, self._publish_gamepad)

        # ── LCM 订阅线程（接收状态估计器）────────────────────────
        self._lcm_sub.subscribe('state_estimator', self._on_state_estimator)
        self._lcm_thread = threading.Thread(target=self._lcm_loop, daemon=True)
        self._lcm_thread.start()

        self.get_logger().info('[Bridge] 就绪，监听 /competition/motion_cmd + state_estimator LCM')

    def _lcm_loop(self):
        while True:
            try:
                self._lcm_sub.handle_timeout(100)  # 100ms timeout
            except Exception as e:
                self.get_logger().warn(f'[Bridge] LCM handle 异常: {e}')

    def _on_state_estimator(self, channel, data):
        # 此回调运行在 LCM 子线程，不可直接调用 ROS2 API。
        # 只做数据转换并写入缓存；由 50Hz 定时器在 ROS2 线程统一发布。
        try:
            msg = state_estimator_lcmt.decode(data)
        except Exception:
            return  # 解码失败静默丢弃，不调用 get_logger()（非线程安全）
        rs = RobotState()
        # 不在子线程调用 get_clock()，不填 header.stamp
        rs.position        = [msg.p[0], msg.p[1], msg.p[2]]
        rs.velocity_world  = [msg.vWorld[0], msg.vWorld[1], msg.vWorld[2]]
        rs.velocity_body   = [msg.vBody[0], msg.vBody[1], msg.vBody[2]]
        rs.rpy             = [msg.rpy[0], msg.rpy[1], msg.rpy[2]]
        rs.quaternion      = [msg.quat[0], msg.quat[1], msg.quat[2], msg.quat[3]]
        rs.omega_body      = [msg.omegaBody[0], msg.omegaBody[1], msg.omegaBody[2]]
        rs.contact_estimate= [float(msg.contactEstimate[i]) for i in range(4)]
        rs.is_fallen = (abs(msg.rpy[0]) > 1.2 or abs(msg.rpy[1]) > 1.2)
        with self._state_lock:
            self._pending_state = rs

    def _send_use_rc_0(self):
        """切换到 gamepad 模式（use_rc=0）"""
        msg = YamlParam()
        msg.name     = 'use_rc'
        msg.kind     = 2   # kS64
        msg.s64_value = 0
        msg.is_user  = 0
        self._yaml_pub.publish(msg)
        self.get_logger().info('[Bridge] 已发送 use_rc=0 (gamepad 模式)')
        time.sleep(0.2)

    def _on_motion_cmd(self, cmd: MotionCommand):
        now = time.time()
        with self._gp_lock:
            gp = self._gp
            if cmd.mode == MODE_STAND_UP or cmd.mode == MODE_LIE_DOWN:
                # RecoveryStand：按住 b 键 3 秒
                gp.b = 1
                gp.y = 0
                gp.leftStickAnalog[0] = 0.0
                gp.leftStickAnalog[1] = 0.0
                gp.rightStickAnalog[0] = 0.0
                gp.rightStickAnalog[1] = 0.0
                gp.leftTriggerAnalog  = 0.0
                self._in_loco_mode   = False
                self._stand_button_until = now + STAND_BUTTON_HOLD_SEC
                self.get_logger().info('[Bridge] MODE_STAND_UP → b=1')

            elif cmd.mode == MODE_VELOCITY_CTRL or cmd.mode == MODE_LOW_WALK:
                vx   = float(cmd.vel_x)
                vy   = float(cmd.vel_y)
                vyaw = float(cmd.vel_yaw)
                # 参考 command_interface.cpp 520-522
                gp.leftStickAnalog[1]  =  vx
                gp.leftStickAnalog[0]  = -vy
                gp.rightStickAnalog[0] = -vyaw
                # body_height 透传 (leftTriggerAnalog 作为绝对高度覆盖，单位 m)
                # 0 表示不覆盖（用控制器默认 0.32m）
                bh = float(cmd.body_height)
                if cmd.mode == MODE_LOW_WALK and bh > 0.05:
                    gp.leftTriggerAnalog = max(0.06, min(0.40, bh))
                    # 低姿过限高杆时同时俯首：command_interface.cpp 把
                    # rightStickAnalog[1] 映射为 rpy_des[1]*0.4，正值=低头。
                    # 1.0 → pitch +0.4 rad ≈ 23°，让头/雷达柱下沉 ~10cm
                    gp.rightStickAnalog[1] = 1.0
                elif cmd.mode == MODE_VELOCITY_CTRL and bh > 0.05:
                    # 速度控制下允许抬高机身（如上独木桥前抬高重心）
                    gp.leftTriggerAnalog = max(0.06, min(0.40, bh))
                    gp.rightStickAnalog[1] = 0.0
                else:
                    gp.leftTriggerAnalog = 0.0
                    gp.rightStickAnalog[1] = 0.0
                # step_height 透传 (rightTriggerAnalog 作为抬脚高度覆盖, 单位 m)
                # command_interface.cpp 在 loco 模式下读取此字段并覆盖默认 step_height (0.06m)
                sh = float(cmd.step_height)
                if sh > 0.04:
                    gp.rightTriggerAnalog = max(0.04, min(0.15, sh))
                else:
                    gp.rightTriggerAnalog = 0.0
                if not self._in_loco_mode:
                    gp.y = 1                   # 触发一次 locomotion 模式切换
                    self._in_loco_mode = True
                    self.get_logger().info(
                        f'[Bridge] 切换 Locomotion，vel=({vx:.2f},{vy:.2f},{vyaw:.2f}) body_h={gp.leftTriggerAnalog:.2f}')
                else:
                    gp.y = 0
                    # 低姿状态变化时补一条日志
                    if abs(gp.leftTriggerAnalog - getattr(self, '_last_bh_log', 0.0)) > 0.02:
                        self._last_bh_log = gp.leftTriggerAnalog
                        self.get_logger().info(
                            f'[Bridge] body_h 变更 → {gp.leftTriggerAnalog:.2f}m '
                            f'(mode={"LOW_WALK" if cmd.mode == MODE_LOW_WALK else "VEL"})')

            elif cmd.mode == MODE_STOP:
                # 仅清零速度不够：locomotion 控制器仍在 trot 步态里 vel=0 原地踏步。
                # 这里同时按住 b 触发 RecoveryStand，让控制器切出 trot 真正停下。
                gp.leftStickAnalog[0]  = 0.0
                gp.leftStickAnalog[1]  = 0.0
                gp.rightStickAnalog[0] = 0.0
                gp.rightStickAnalog[1] = 0.0
                gp.leftTriggerAnalog   = 0.0
                gp.rightTriggerAnalog  = 0.0
                gp.y = 0
                gp.b = 1
                self._in_loco_mode = False
                self._stand_button_until = now + STAND_BUTTON_HOLD_SEC
                self.get_logger().info('[Bridge] MODE_STOP → 速度清零 + b=1 (RecoveryStand)')

            elif cmd.mode == MODE_JUMP_DOWN:
                # 跳下：给一个较大前向冲量后停止
                gp.leftStickAnalog[1] = 0.5
                gp.y = 0

            elif cmd.mode in (MODE_HIT_FORWARD, MODE_KICK_FORWARD):
                # 前击/踢球：短暂前推
                gp.leftStickAnalog[1] = 0.4
                gp.y = 0

    def _publish_gamepad(self):
        """50Hz 持续发布 gamepad_lcmt，防止运控超时停止；同时转发缓存的 RobotState。"""
        now = time.time()
        with self._gp_lock:
            gp = self._gp
            # 超过保持时间后松开 b/y 按钮
            if gp.b == 1 and now > self._stand_button_until:
                gp.b = 0
        self._lcm_pub.publish('gamepad_lcmt', self._gp.encode())

        # 在 ROS2 执行器线程中发布最新状态（线程安全）
        with self._state_lock:
            pending = self._pending_state
            self._pending_state = None
        if pending is not None:
            self._robot_state_pub.publish(pending)


def main():
    rclpy.init()
    node = MotionBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
