#!/usr/bin/env python3
"""
mock_perception_node.py — 赛段三/四 Mock 感知节点

当真实 camera/YOLO 未就绪时，模拟发布：
  - /perception/boundary  (BoundaryInfo)
  - /perception/objects   (DetectedObjectArray)

赛段三：持续发布直道/弯道边界（随时间产生一个 S 型弯道模拟）
赛段四：按时间序列依次"出现"各目标物体
"""

import rclpy
from rclpy.node import Node
import math
import time

from competition_msgs.msg import (
    BoundaryInfo, DetectedObjectArray, DetectedObject, CompetitionState
)
from std_msgs.msg import Header
from gazebo_msgs.msg import ModelStates
from rclpy.qos import qos_profile_sensor_data


class MockPerceptionNode(Node):
    def __init__(self):
        super().__init__('mock_perception')

        self.boundary_pub = self.create_publisher(BoundaryInfo,       '/perception/boundary', 10)
        self.objects_pub  = self.create_publisher(DetectedObjectArray, '/perception/objects',  10)

        self.state_sub = self.create_subscription(
            CompetitionState, '/competition/state',
            self._on_state, 10)

        # 订阅 Gazebo 真值，用机器人位姿驱动 stage4 目标显隐
        self.model_sub = self.create_subscription(
            ModelStates, '/gazebo/model_states',
            self._on_model_states, qos_profile_sensor_data)
        self._robot_x = 0.0
        self._robot_y = 0.0
        self._robot_yaw = math.pi / 2  # 初始朝 +Y

        self.timer = self.create_timer(0.05, self._tick)  # 20 Hz

        self.current_stage   = 3   # 默认赛段三
        self.stage_start_t   = time.time()
        self.obj_seq         = 0

        # 赛段四已交互记录（每个目标只交互一次）
        self._stage4_interacted = {
            DetectedObject.TYPE_COLA_BOTTLE: False,
            DetectedObject.TYPE_ORANGE_BALL: False,
            DetectedObject.TYPE_SOCCER:      False,
        }

        # 赛段四物体世界坐标（race.world 实测/用户提供）
        # 字段：(类型, world_x, world_y, world_z, is_target, is_obstacle)
        # 三个目标位置已确认：
        #   可乐  (-0.10, 11.10, 0.17)
        #   橙球  ( 0.95, 11.10, 0.30)
        #   足球  ( 2.10, 10.80, 0.10)
        # 独木桥起点 (3.15, 7.60, 0.05)
        # 注：race.world 没有限高杆/挡板模型，故此处也不再虚构。
        self._stage4_objects = [
            (DetectedObject.TYPE_COLA_BOTTLE,  -0.10, 11.10, 0.17, True,  False),
            (DetectedObject.TYPE_ORANGE_BALL,   0.95, 11.10, 0.30, True,  False),
            (DetectedObject.TYPE_SOCCER,        2.10, 10.80, 0.10, True,  False),
            (DetectedObject.TYPE_BALANCE_BEAM,  3.15,  7.60, 0.05, False, False),
        ]

        self.get_logger().info('[MockPerception] 启动，默认发布赛段3边界数据')

    def _on_state(self, msg: CompetitionState):
        if msg.stage != self.current_stage:
            self.get_logger().info(f'[MockPerception] 赛段切换: {self.current_stage} → {msg.stage}')
            self.current_stage = msg.stage
            self.stage_start_t = time.time()

    def _on_model_states(self, msg: ModelStates):
        try:
            idx = msg.name.index('robot')
        except ValueError:
            return
        p = msg.pose[idx].position
        q = msg.pose[idx].orientation
        self._robot_x = p.x
        self._robot_y = p.y
        # yaw from quaternion
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._robot_yaw = math.atan2(siny, cosy)

    def _tick(self):
        if self.current_stage == 3:
            self._publish_stage3()
        elif self.current_stage == 4:
            self._publish_stage4()
        else:
            # 其他赛段发布空数据
            self._publish_empty()

    # ── 赛段三：弯道边界模拟 ──────────────────────────────────────────────
    def _publish_stage3(self):
        elapsed = time.time() - self.stage_start_t

        msg = BoundaryInfo()
        msg.header = self._header()
        msg.left_boundary_detected  = True
        msg.right_boundary_detected = True

        # 时序与 stage3_curved_track.cpp 对齐：
        # 0-3s   kStandUp  3-6s kEntry  6-13s kCurveLeft
        # 13-15s kMidStraight  15-22s kCurveRight  22s+ kExitStraight
        # 出口虚线在约 27s 开始出现（机器人走完 S 弯进入出口直道 ~5s 后）
        if elapsed < 27.0:
            msg.center_offset_m  = 0.0
            msg.heading_error_rad = 0.0
            msg.turn_detected    = False
            msg.straight_distance_m = 3.0
        else:
            # 出口虚线出现并逐渐接近
            msg.center_offset_m       = 0.0
            msg.heading_error_rad     = 0.0
            msg.turn_detected         = False
            msg.dashed_line_detected  = True
            msg.dashed_line_distance_m = max(0.0, 1.5 - (elapsed - 27.0) * 0.2)

        self.boundary_pub.publish(msg)
        self.objects_pub.publish(self._empty_objects())

    # ── 赛段四：基于机器人世界坐标真值发布目标 ──────────────────────────
    def _publish_stage4(self):
        objects = DetectedObjectArray()
        objects.header = self._header()

        boundary = BoundaryInfo()
        boundary.header = self._header()
        boundary.left_boundary_detected  = True
        boundary.right_boundary_detected = True
        boundary.center_offset_m  = 0.0
        boundary.heading_error_rad = 0.0

        # ── 视场参数 ──
        # 机器人前向最大检出距离 3m，侧向 ±1.0m（机体系，rel_y 左正右负）
        FOV_FRONT_MAX = 3.0
        FOV_SIDE_MAX  = 1.0
        APPROACH_HIT  = 0.45  # 距离 < 此值时标记为 INTERACTED

        cyaw = math.cos(self._robot_yaw)
        syaw = math.sin(self._robot_yaw)

        for oid, (otype, wx, wy, wz, is_target, is_obs) in enumerate(
                self._stage4_objects):
            # 世界 → 机体系
            dx = wx - self._robot_x
            dy = wy - self._robot_y
            rel_x =  cyaw * dx + syaw * dy   # 前向
            rel_y = -syaw * dx + cyaw * dy   # 左正右负
            rel_z =  wz
            dist  = math.hypot(rel_x, rel_y)

            if rel_x <= 0.0 or rel_x > FOV_FRONT_MAX:
                continue
            if abs(rel_y) > FOV_SIDE_MAX:
                continue

            # 标记交互完成
            if is_target and dist < APPROACH_HIT:
                self._stage4_interacted[otype] = True
            status = (DetectedObject.STATUS_INTERACTED
                      if is_target and self._stage4_interacted.get(otype, False)
                      else DetectedObject.STATUS_VISIBLE)

            o = DetectedObject()
            o.header        = self._header()
            o.object_type   = otype
            o.object_status = status
            o.object_id     = oid
            o.confidence    = 0.92
            o.rel_x         = rel_x
            o.rel_y         = rel_y
            o.rel_z         = rel_z
            o.distance      = dist
            o.bbox_x        = 320.0
            o.bbox_y        = 240.0
            o.bbox_w        = 60.0
            o.bbox_h        = 80.0
            o.is_target     = is_target
            o.is_obstacle   = is_obs
            objects.objects.append(o)

        self.boundary_pub.publish(boundary)
        self.objects_pub.publish(objects)

    def _header(self):
        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        h.frame_id = 'base_link'
        return h

    def _empty_objects(self):
        msg = DetectedObjectArray()
        msg.header = self._header()
        return msg

    def _publish_empty(self):
        self.boundary_pub.publish(BoundaryInfo(header=self._header()))
        self.objects_pub.publish(DetectedObjectArray(header=self._header()))


def main():
    rclpy.init()
    node = MockPerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
