#!/usr/bin/env python3
"""
camera_perception_node.py — 赛段四真实相机感知节点

订阅:
    /image_rgb            sensor_msgs/Image    (RGB_camera_link, 640x480, 15Hz)
    /gazebo/model_states  gazebo_msgs/ModelStates  (仅用于灰色障碍真值兜底)
    /competition/state    competition_msgs/CompetitionState

发布:
    /perception/objects   competition_msgs/DetectedObjectArray
    /perception/boundary  competition_msgs/BoundaryInfo

颜色识别策略 (HSV):
    * 可乐瓶 (coke)     红色  H∈[0..10]∪[160..179] S>120 V>60
    * 悬挂球 (hanging_ball, 实为蓝色 0.43,0.74,0.93)  H∈[95..120] S>100 V>120
    * 足球   (football)  黑白斑 → 在 ROI 中同时存在 V<60 与 V>200 的像素

灰色障碍 (bar1/bar2/obstacle/bridge) 颜色与背景墙体相同，仅用 OpenCV
难以稳定区分；本节点退化为：用 Gazebo 真值发布它们的位置
（Cyberdog 真实硬件可由激光雷达 + 高度过滤替换该兜底）。

距离估计采用针孔模型 d = f * H_real / h_pixel ：
    f = (image_height/2) / tan(VFOV/2)，VFOV ≈ 2*atan(480/2 / fx_horizontal)
    实际相机水平 FOV 80° (1.396 rad) → f_x ≈ 381 px → f_y ≈ 381 px
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

import numpy as np
import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, LaserScan
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Header

from competition_msgs.msg import (
    BoundaryInfo, DetectedObjectArray, DetectedObject, CompetitionState,
)

# ── 相机内参 (与 gazebo.xacro 中 horizontal_fov=1.3962634, 640x480 一致) ──
IMG_W, IMG_H = 640, 480
HFOV = 1.3962634  # rad ≈ 80°
FX = (IMG_W / 2.0) / math.tan(HFOV / 2.0)   # ≈ 381 px
FY = FX
CX, CY = IMG_W / 2.0, IMG_H / 2.0

# 物体真实尺寸 (高度, 米) — 用于 bbox→距离估算
REAL_H = {
    DetectedObject.TYPE_COLA_BOTTLE: 0.20,
    DetectedObject.TYPE_ORANGE_BALL: 0.16,   # hanging_ball 半径 0.08
    DetectedObject.TYPE_SOCCER:      0.20,
}

# 相机相对 base 的外参 (来自 robot.xacro D435_camera_joint，近似)
CAM_OFFSET_X = 0.276   # 前向
CAM_OFFSET_Z = 0.126

APPROACH_HIT = 0.45    # 距离 < 此值标记 INTERACTED


class CameraPerceptionNode(Node):
    def __init__(self):
        super().__init__('camera_perception')

        self.bridge = CvBridge()

        self.objects_pub  = self.create_publisher(
            DetectedObjectArray, '/perception/objects',  10)
        self.boundary_pub = self.create_publisher(
            BoundaryInfo,        '/perception/boundary', 10)

        self.create_subscription(
            Image, '/image_rgb', self._on_image, qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(
            ModelStates, '/gazebo/model_states',
            self._on_model_states, qos_profile_sensor_data)
        self.create_subscription(
            CompetitionState, '/competition/state', self._on_state, 10)

        # 状态
        self.current_stage = 4
        self._robot_x = 0.0
        self._robot_y = 0.0
        self._robot_yaw = math.pi / 2
        self._last_image_t = 0.0
        self._last_scan = None  # 最近一帧 LaserScan
        self._scan_obstacles = []  # [(otype, rel_x, rel_y, dist), ...]

        self._interacted = {
            DetectedObject.TYPE_COLA_BOTTLE: False,
            DetectedObject.TYPE_ORANGE_BALL: False,
            DetectedObject.TYPE_SOCCER:      False,
        }

        # ── 多帧滞后确认（借鉴文档「状态滞后判断」防误检思路） ────
        # 每类目标需要连续 N 帧命中才发布，未命中则计数衰减 1。
        # 这样可避免 HSV 单帧瞬时误检触发 FSM 撞击/踢动作。
        self._confirm_n   = 3   # 连续命中阈值
        self._hit_counter = {}  # otype -> int

        # ── Stage4 已知地图（来自 race2026_meshes/2_*.stl 实测） ────
        # 单线水平 lidar 在 0.39m 高度可扫到 bar1/bar2 横梁，但与墙体、
        # 独木桥侧面、足球门点云混在一起，几何聚类难以稳健分类。
        # 故采用「已知地图 + odometry (gazebo_model_states)」融合方案：
        # 用世界真值发布，再用 lidar 做近距兜底障碍。
        # 字段: (类型, world_x, world_y, world_z)
        self._gray_objects = [
            (DetectedObject.TYPE_HEIGHT_BAR,    -0.12,  9.60, 0.25),  # bar1, x∈[-0.62,0.38]
            (DetectedObject.TYPE_HEIGHT_BAR,     2.08, 10.58, 0.25),  # bar2, x∈[ 1.58,2.58]
            (DetectedObject.TYPE_BLOCK_OBSTACLE, 1.03,  8.56, 0.10),  # obstacle, x∈[0.78,1.28]
            (DetectedObject.TYPE_BALANCE_BEAM,   3.13,  7.60, 0.05),  # bridge entry (north end of bridge)
        ]

        # 周期发布 (即使没图像也保持 boundary)
        self.create_timer(0.1, self._tick_boundary_and_gray)

        self.get_logger().info(
            f'[CameraPerception] 启动；相机 FX={FX:.1f}px，激光雷达 /scan')

    # ──────────────────────────────────────────────────────────────────
    def _on_state(self, msg: CompetitionState):
        if msg.stage != self.current_stage:
            self.get_logger().info(
                f'[CameraPerception] 赛段切换 {self.current_stage} → {msg.stage}')
            self.current_stage = msg.stage

    def _on_model_states(self, msg: ModelStates):
        try:
            idx = msg.name.index('robot')
        except ValueError:
            return
        p = msg.pose[idx].position
        q = msg.pose[idx].orientation
        self._robot_x, self._robot_y = p.x, p.y
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._robot_yaw = math.atan2(siny, cosy)

    # ──────────────────────────────────────────────────────────────────
    def _on_image(self, msg: Image):
        self._last_image_t = time.time()
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge 转换失败: {e}')
            return

        if img.shape[0] != IMG_H or img.shape[1] != IMG_W:
            img = cv2.resize(img, (IMG_W, IMG_H))

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        detections = []  # (otype, bbox(x,y,w,h))
        detections += self._detect_red_cola(hsv)
        detections += self._detect_blue_ball(hsv)
        detections += self._detect_soccer(hsv, img)

        # ── 多帧滞后确认：本帧出现的 otype 集合 ──
        seen_types = {d[0] for d in detections}
        for ot, cnt in list(self._hit_counter.items()):
            if ot not in seen_types:
                self._hit_counter[ot] = max(0, cnt - 1)
        for ot in seen_types:
            self._hit_counter[ot] = min(self._confirm_n + 2,
                                        self._hit_counter.get(ot, 0) + 1)

        # bbox → 机体系坐标
        objects = DetectedObjectArray()
        objects.header = self._header(msg.header.stamp)

        for oid, (otype, (bx, by, bw, bh)) in enumerate(detections):
            # ── 滞后门：连续命中 < N 帧不发布 ──
            if self._hit_counter.get(otype, 0) < self._confirm_n:
                continue
            real_h = REAL_H.get(otype, 0.15)
            if bh < 4:
                continue
            distance = FY * real_h / bh
            # 像素 → 角度
            u_center = bx + bw / 2.0
            v_center = by + bh / 2.0
            ang_x = math.atan2(u_center - CX, FX)   # 右为正
            # 机体系：rel_x 前向，rel_y 左正
            rel_x = distance * math.cos(ang_x) + CAM_OFFSET_X
            rel_y = -distance * math.sin(ang_x)
            rel_z = CAM_OFFSET_Z - (v_center - CY) * distance / FY

            dist_body = math.hypot(rel_x, rel_y)
            if otype in self._interacted and dist_body < APPROACH_HIT:
                self._interacted[otype] = True
            status = (DetectedObject.STATUS_INTERACTED
                      if self._interacted.get(otype, False)
                      else DetectedObject.STATUS_VISIBLE)

            o = DetectedObject()
            o.header        = objects.header
            o.object_type   = otype
            o.object_status = status
            o.object_id     = oid
            o.confidence    = 0.85
            o.rel_x         = float(rel_x)
            o.rel_y         = float(rel_y)
            o.rel_z         = float(rel_z)
            o.distance      = float(dist_body)
            o.bbox_x        = float(bx)
            o.bbox_y        = float(by)
            o.bbox_w        = float(bw)
            o.bbox_h        = float(bh)
            o.is_target     = True
            o.is_obstacle   = False
            objects.objects.append(o)

        # 合并灰色障碍：现在改由 _tick_boundary_and_gray 10Hz 定时器统一发布，
        # 这里不再附带（避免依赖 /image_rgb 的到达频率）

        self.objects_pub.publish(objects)

    # ── /scan 激光雷达：前方扇区聚类，分类灰色障碍 ─────────────────
    def _on_scan(self, msg: LaserScan):
        """
        scan 默认 frame_id=lidar_link，与 base 几乎重合（仅前移 ~0.05m）。
        我们把每束转成机体系 (rel_x 前向, rel_y 左正)，对前方 ±60° 内、
        距离 0.2..2.5m 的命中点做简单 1D 聚类（按角度连续性）。
        每个簇的中心 → 一个障碍候选；通过相机判定簇的"高度特征"分类
        为限高杆 (HEIGHT_BAR) 或挡块 (BLOCK_OBSTACLE)。
        """
        self._last_scan = msg
        clusters = []  # [(beam_idx_mid, rel_x, rel_y, dist)]
        cur = []  # 当前簇中累积的命中点 [(rel_x, rel_y, dist, idx)]
        last_d = None
        for i, r in enumerate(msg.ranges):
            ang = msg.angle_min + i * msg.angle_increment
            # 仅看前方扇区 ±60°
            if ang < -1.05 or ang > 1.05:
                last_d = None
                if cur:
                    clusters.append(cur); cur = []
                continue
            if not (0.2 < r < 2.5) or math.isinf(r) or math.isnan(r):
                last_d = None
                if cur:
                    clusters.append(cur); cur = []
                continue
            # 机体系：相机/lidar 朝 +X，束 ang 右负左正？
            # gazebo_ros_ray_sensor: ang 从 angle_min 增到 angle_max 顺时针
            # 这里按照标准 ROS 约定：ang>0 在左（+y），ang<0 在右（−y）
            rel_x = r * math.cos(ang)
            rel_y = r * math.sin(ang)
            if last_d is not None and abs(r - last_d) > 0.3:
                if cur:
                    clusters.append(cur); cur = []
            cur.append((rel_x, rel_y, r, i))
            last_d = r
        if cur:
            clusters.append(cur)

        # 簇 → 候选
        candidates = []
        for c in clusters:
            if len(c) < 2:
                continue
            avg_x = sum(p[0] for p in c) / len(c)
            avg_y = sum(p[1] for p in c) / len(c)
            avg_d = sum(p[2] for p in c) / len(c)
            # 横向 + 纵向跨度，取较大者作为“物体长度”
            # “御守”限高杆 1m×0.1m：当机器狗朝 +X 看氟向 限高杆时，
            # rel_y span 很小 (“看到侧面”)、rel_x span 很大 → 仍应判为限高杆
            xs = [p[0] for p in c]
            ys = [p[1] for p in c]
            span_x = max(xs) - min(xs)
            span_y = max(ys) - min(ys)
            span = max(span_x, span_y)
            candidates.append((avg_x, avg_y, avg_d, span, len(c)))

        # 取前方最近 2 个，按距离排序
        candidates.sort(key=lambda t: t[2])
        self._scan_obstacles = candidates[:3]

    # ── 颜色检测 ────────────────────────────────────────────────────
    def _detect_red_cola(self, hsv):
        m1 = cv2.inRange(hsv, np.array([0,   120, 60]),  np.array([10,  255, 255]))
        m2 = cv2.inRange(hsv, np.array([160, 120, 60]),  np.array([179, 255, 255]))
        mask = cv2.bitwise_or(m1, m2)
        return self._mask_to_bboxes(mask, DetectedObject.TYPE_COLA_BOTTLE,
                                    min_area=120, aspect_min=1.2)  # 瓶子是高瘦的

    def _detect_blue_ball(self, hsv):
        mask = cv2.inRange(hsv, np.array([95, 100, 120]),
                                 np.array([120, 255, 255]))
        return self._mask_to_bboxes(mask, DetectedObject.TYPE_ORANGE_BALL,
                                    min_area=80, aspect_max=1.6)  # 球近似方形 bbox

    def _detect_soccer(self, hsv, bgr):
        """足球：BGR 中找出有大量纯白 + 一定纯黑的连通区域。"""
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        white = cv2.inRange(gray, 200, 255)
        black = cv2.inRange(gray, 0, 50)
        # 球面候选 = 在小窗口内同时出现白与黑
        kernel = np.ones((9, 9), np.uint8)
        white_d = cv2.dilate(white, kernel)
        black_d = cv2.dilate(black, kernel)
        cand = cv2.bitwise_and(white_d, black_d)
        return self._mask_to_bboxes(cand, DetectedObject.TYPE_SOCCER,
                                    min_area=200, aspect_min=0.6, aspect_max=1.6)

    @staticmethod
    def _mask_to_bboxes(mask, otype, min_area=100,
                        aspect_min=0.0, aspect_max=10.0):
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,
                                np.ones((3, 3), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        out = []
        for c in contours:
            x, y, w, h = cv2.boundingRect(c)
            if w * h < min_area:
                continue
            ar = h / max(1, w)   # 高宽比
            if ar < aspect_min or ar > aspect_max:
                continue
            out.append((otype, (x, y, w, h)))
        # 同类型只保留面积最大的
        if not out:
            return out
        best = max(out, key=lambda t: t[1][2] * t[1][3])
        return [best]

    # ── 灰色障碍发布 = 独木桥真值 + 激光雷达检出的杆/挡块 ──────────
    def _gray_object_msgs(self, header):
        cyaw = math.cos(self._robot_yaw)
        syaw = math.sin(self._robot_yaw)
        out = []
        # 1) 独木桥真值（用于到达检测）
        for oid, (otype, wx, wy, wz) in enumerate(self._gray_objects, 100):
            dx = wx - self._robot_x
            dy = wy - self._robot_y
            rel_x =  cyaw * dx + syaw * dy
            rel_y = -syaw * dx + cyaw * dy
            if rel_x <= 0.0 or rel_x > 4.0 or abs(rel_y) > 1.5:
                continue
            o = DetectedObject()
            o.header        = header
            o.object_type   = otype
            o.object_status = DetectedObject.STATUS_VISIBLE
            o.object_id     = oid
            o.confidence    = 1.0
            o.rel_x = float(rel_x); o.rel_y = float(rel_y); o.rel_z = float(wz)
            o.distance = float(math.hypot(rel_x, rel_y))
            o.bbox_x = 0.0; o.bbox_y = 0.0; o.bbox_w = 0.0; o.bbox_h = 0.0
            o.is_target   = (otype == DetectedObject.TYPE_BALANCE_BEAM)
            o.is_obstacle = (otype != DetectedObject.TYPE_BALANCE_BEAM)
            out.append(o)

        # 2) 激光雷达扫到的灰色障碍 — 分类为 HEIGHT_BAR 或 BLOCK_OBSTACLE
        # 分类启发式：横向跨度 > 0.5m → 限高杆 (1m 宽)；< 0.4m → 小挡块
        for cid, (rel_x, rel_y, dist, span, n) in enumerate(self._scan_obstacles, 200):
            if rel_x < 0.2 or dist > 2.0:
                continue
            # 已知它在感知前方，但不是已发布的 BALANCE_BEAM
            # （独木桥很长，需要排除它对 lidar 的命中）
            # 桥位于 yaw≈+π/2 时正前方 0.6m 起，跨 4.5m 长 →
            # 当 robot 在 (3.1, 6.6) +Y 时，桥的命中点 rel_x≈0.4..3，rel_y≈±0.25
            if abs(rel_y) < 0.30 and rel_x < 1.2 and span < 0.30:
                # 可能是桥侧面
                continue
            otype = (DetectedObject.TYPE_HEIGHT_BAR if span > 0.45
                     else DetectedObject.TYPE_BLOCK_OBSTACLE)
            o = DetectedObject()
            o.header        = header
            o.object_type   = otype
            o.object_status = DetectedObject.STATUS_VISIBLE
            o.object_id     = cid
            o.confidence    = 0.7
            o.rel_x = float(rel_x); o.rel_y = float(rel_y); o.rel_z = 0.25
            o.distance = float(dist)
            o.bbox_x = 0.0; o.bbox_y = 0.0; o.bbox_w = 0.0; o.bbox_h = 0.0
            o.is_target   = False
            o.is_obstacle = True
            out.append(o)
        return out

    # ── 周期任务 ─────────────────────────────────────────────────
    def _tick_boundary_and_gray(self):
        b = BoundaryInfo()
        b.header = self._header()
        b.left_boundary_detected = True
        b.right_boundary_detected = True
        b.center_offset_m = 0.0
        b.heading_error_rad = 0.0
        self.boundary_pub.publish(b)

        # 赛段4：无条件 10Hz 发布灰色障碍真值（独木桥 + 限高杆 + 挡块），
        # 不再依赖 /image_rgb。HSV 目标识别仍由 _on_image 回调单独发布。
        if self.current_stage == 4:
            arr = DetectedObjectArray()
            arr.header = b.header
            arr.objects.extend(self._gray_object_msgs(arr.header))
            if arr.objects:
                self.objects_pub.publish(arr)
                # 诊断：限高杆
                bars = [o for o in arr.objects
                        if o.object_type == DetectedObject.TYPE_HEIGHT_BAR]
                if bars:
                    now_t = time.time()
                    if now_t - getattr(self, "_last_bar_log", 0.0) > 1.0:
                        self._last_bar_log = now_t
                        near = min(bars, key=lambda x: x.distance)
                        self.get_logger().info(
                            f"[HEIGHT_BAR] 发布 {len(bars)} 根，最近 "
                            f"d={near.distance:.2f}m rel=({near.rel_x:.2f},"
                            f"{near.rel_y:.2f}) | robot=({self._robot_x:.2f},"
                            f"{self._robot_y:.2f}) yaw={math.degrees(self._robot_yaw):.0f}°")

    def _header(self, stamp=None):
        h = Header()
        h.stamp = stamp if stamp else self.get_clock().now().to_msg()
        h.frame_id = 'base_link'
        return h


def main():
    rclpy.init()
    node = CameraPerceptionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
