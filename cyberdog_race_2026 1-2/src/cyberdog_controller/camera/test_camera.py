import rclpy
import os
import datetime
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        qos_profile = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self.subscription = self.create_subscription(
            Image,
            '/rgb_camera/image_raw',
            self.image_callback,
            qos_profile
        )
        self.bridge = CvBridge()
        self.img_height = 0  # 添加图像高度存储

    def cluster_x(self, x_coords, threshold=10):
        """横向坐标聚类"""
        if x_coords.size == 0: return None
        sorted_x = np.sort(x_coords)
        clusters = []
        current = [sorted_x[0]]

        for x in sorted_x[1:]:
            if x - current[-1] <= threshold:
                current.append(x)
            else:
                clusters.append(current)
                current = [x]
        clusters.append(current)

        largest = max(clusters, key=len)
        return int(np.mean(largest))

    def detect_extreme_points(self, mask):
        """检测最远和最近点"""
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours: 
            return None, None, None, None

        all_points = np.vstack(contours)
        if all_points.size == 0:
            return None, None, None, None

        # 最远点（y最小）
        min_y = np.min(all_points[:,0,1])
        far_points = all_points[all_points[:,0,1] == min_y]
        far_x = self.cluster_x(far_points[:,0,0], 15)
        farthest = (far_x, min_y) if far_x else None
        far_dist = self.img_height - min_y

        # 最近点（y最大）
        max_y = np.max(all_points[:,0,1])
        near_points = all_points[all_points[:,0,1] == max_y]
        near_x = self.cluster_x(near_points[:,0,0], 15)
        nearest = (near_x, max_y) if near_x else None
        near_dist = self.img_height - max_y

        return farthest, far_dist, nearest, near_dist

    def detect_boundary_distances(self, mask, img_width):
        """检测各半区到对应边界的最远点"""
        center_x = img_width // 2
        
        # 获取并重塑轮廓点结构（关键修复）
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return (None, None), (None, None)

        # 将轮廓点转换为(N,2)结构
        all_points = np.vstack([c.reshape(-1,2) for c in contours])
        
        # 添加有效性过滤
        valid_mask = (all_points[:,0] > 1) & (all_points[:,0] < img_width-1)
        filtered_points = all_points[valid_mask]

        if filtered_points.size == 0:
            return (None, None), (None, None)

        # 分割半区
        left_mask = filtered_points[:,0] <= center_x
        right_mask = filtered_points[:,0] >= center_x

        # 获取左右半区点
        left_points = filtered_points[left_mask] if np.any(left_mask) else None
        right_points = filtered_points[right_mask] if np.any(right_mask) else None

        # 计算最远点（带空值保护）
        left_far = left_points[np.argmax(left_points[:,0])] if left_points is not None else None
        right_far = right_points[np.argmin(right_points[:,0])] if right_points is not None else None

        # 计算距离
        left_dist = left_far[0] if left_far is not None else None
        right_dist = (img_width - right_far[0]) if right_far is not None else None

        return (left_dist, right_dist), (tuple(left_far) if left_far else None, 
                                        tuple(right_far) if right_far else None)

    def detect_center_intersection(self, mask, img_width, img_height, threshold=10):
        """检测中间基准线与赛道的交点，并返回到底线的距离"""
        center_x = img_width // 2
        # 收集中间线上所有属于赛道的y坐标
        y_coords = []
        for y in range(img_height):
            if mask[y, center_x] == 255:
                y_coords.append(y)
        if not y_coords:
            return None, None

        # 按降序排列以便优先处理下方点
        y_coords_sorted = sorted(y_coords, reverse=True)
        clusters = []
        current_cluster = [y_coords_sorted[0]]

        # 聚类处理
        for y in y_coords_sorted[1:]:
            if current_cluster[-1] - y <= threshold:
                current_cluster.append(y)
            else:
                clusters.append(current_cluster)
                current_cluster = [y]
        clusters.append(current_cluster)  # 添加最后一个聚类

        # 寻找最下方的聚类（平均y最大）
        max_avg_y = -1
        best_cluster = None
        for cluster in clusters:
            avg_y = np.mean(cluster)
            if avg_y > max_avg_y:
                max_avg_y = avg_y
                best_cluster = cluster

        # 计算交点坐标和距离
        cluster_y = int(round(np.mean(best_cluster)))
        distance = (img_height - 1) - cluster_y
        return (center_x, cluster_y), distance

    def detect_black_bar(self, img):
        """检测黑色限高杆，返回检测结果字典"""
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower_black = np.array([0, 0, 0])
        upper_black = np.array([180, 255, 60])
        mask = cv2.inRange(hsv, lower_black, upper_black)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # 霍夫变换检测横线
        hough_params = {
            'rho': 1,
            'theta': np.pi/180,
            'threshold': 40,
            'minLineLength': int(mask.shape[1]*0.2),
            'maxLineGap': 15
        }
        lines = cv2.HoughLinesP(mask, **hough_params)
        if lines is None:
            return {'is_horizontal_bar': -1, 'angle': None, 'points': None}

        candidates = []
        for line in lines:
            x1, y1, x2, y2 = line[0]
            if x1 > x2:
                x1, x2 = x2, x1
                y1, y2 = y2, y1
            dx = x2 - x1
            dy = y2 - y1
            angle = np.degrees(np.arctan2(dy, dx))
            if not (-10 <= angle <= 10):
                continue
            y_avg = (y1 + y2) / 2
            candidates.append({'angle': angle, 'points': (x1, y1, x2, y2), 'y_avg': y_avg})

        if not candidates:
            return {'is_horizontal_bar': -1, 'angle': None, 'points': None}

        best = max(candidates, key=lambda x: x['y_avg'])
        angle = best['angle']
        points = best['points']

        if abs(angle) < 1.1:
            return {'is_horizontal_bar': 3, 'angle': angle, 'points': points}
        elif angle >= 1.1:
            return {'is_horizontal_bar': 2, 'angle': angle, 'points': points}
        else:
            return {'is_horizontal_bar': 1, 'angle': angle, 'points': points}

    
    def specialized_detect(self, mask):
        """
        基于轮廓线段的横向检测（高效直接版）
        """
        try:
            # 输入校验
            if mask is None or mask.size == 0:
                return {'is_horizontal_line': -1, 'angle': None, 'points': None}

            # 预处理
            if len(mask.shape) == 3:
                mask = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
            mask = mask.astype(np.uint8)
            img_h, img_w = mask.shape

            # 调试图像
            debug_img = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

            # 关键参数
            min_length = img_w * 0.25       # 线段最短长度
            max_angle = 15                  # 最大允许角度
            bottom_margin = 10              # 底边避让距离（像素）

            # 形态学处理
            kernel = np.ones((15,3), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            # 轮廓提取
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(debug_img, contours, -1, (0,255,0), 1)  # 绘制所有轮廓

            best_segment = None
            best_angle = 90  # 初始化为最大角度
            max_length = 0

            for cnt in contours:
                # 多边形近似（减少点数）
                epsilon = 0.01 * cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt, epsilon, True)

                # 遍历所有线段
                for i in range(len(approx)):
                    # 获取线段端点
                    pt1 = approx[i][0]
                    pt2 = approx[(i+1)%len(approx)][0]
                    x1, y1 = pt1
                    x2, y2 = pt2

                    # 计算线段特征
                    dx = x2 - x1
                    dy = y2 - y1
                    length = np.hypot(dx, dy)
                    angle = np.degrees(np.arctan2(dy, dx))

                    # 角度标准化到-90~90度
                    if angle > 90:
                        angle -= 180
                    elif angle < -90:
                        angle += 180

                    # 筛选条件
                    if (abs(angle) > max_angle or
                        length < min_length or
                        max(y1, y2) > img_h - bottom_margin):
                        continue

                    # 更新最佳线段（优先角度，其次长度）
                    if abs(angle) < abs(best_angle) or (abs(angle) == abs(best_angle) and length > max_length):
                        best_angle = angle
                        max_length = length
                        best_segment = (x1, y1, x2, y2)

            # 结果处理
            if best_segment:
                x1, y1, x2, y2 = best_segment
                # 绘制最终结果
                cv2.line(debug_img, (x1,y1), (x2,y2), (0,0,255), 3)
                cv2.putText(debug_img, f"{best_angle:.1f}deg", (x1,y1-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)

                # 分类结果
                code = 3 if abs(best_angle) < 1 else (2 if best_angle > 0 else 1)

                cv2.imshow("Debug", debug_img)
                cv2.waitKey(1)

                return {
                    'is_horizontal_line': code,
                    'angle': float(best_angle),
                    'points': best_segment
                }
            else:
                cv2.imshow("Debug", debug_img)
                cv2.waitKey(1)
                return {'is_horizontal_line': -1, 'angle': None, 'points': None}

        except Exception as e:
            self.get_logger().error(f"检测异常: {str(e)}")
            return {'is_horizontal_line': -1, 'angle': None, 'points': None}


    def detect_bottom_left_distance(self, mask, img_width):
        """检测与底线相接的赛道部分的最左侧点
        返回值:
            distance: 到左边界的距离(像素)
            point: 最左侧点坐标(x,y)
        """
        # 获取图像高度
        img_height = mask.shape[0]
        
        # 提取底线相接的点 (y坐标等于图像高度-1)
        bottom_row = img_height - 1
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # 收集所有与底线相接的点
        bottom_points = []
        for contour in contours:
            for point in contour:
                x, y = point[0]
                if y == bottom_row:
                    bottom_points.append([x, y])
        
        # 如果没有找到与底线相接的点，返回None
        if not bottom_points:
            return None, None
        
        # 转换为numpy数组以便处理
        bottom_points = np.array(bottom_points)
        
        # 找到最左侧的点
        leftmost_idx = np.argmin(bottom_points[:, 0])
        leftmost_point = bottom_points[leftmost_idx]
        
        # 计算到左边界的距离
        distance = leftmost_point[0]
        
        return distance, tuple(leftmost_point)

    def detect_track_points(self, mask):
        """简化版中心偏移检测"""
        if len(mask.shape) == 3:
            mask = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
        h, w = mask.shape
        self.center_x = w // 2
        self.scan_y = h * 5 //6 # 扫描线位置（可调）

        # 步骤1：在固定Y轴扫描赛道边界
        scan_line = mask[self.scan_y, :]
        white_pixels = np.where(scan_line == 255)[0]

        # 步骤2：分离左右边界点
        left_points = white_pixels[white_pixels < self.center_x]
        right_points = white_pixels[white_pixels > self.center_x]

        # 步骤3：取最近的左右边界
        left_edge = left_points.max() if left_points.size > 0 else None
        right_edge = right_points.min() if right_points.size > 0 else None

        # 步骤4：计算前视中点
        if left_edge and right_edge:
            mid = (left_edge + right_edge) // 2
            offset = mid - self.center_x
            # print(f"Left Edge: {left_edge}, Right Edge: {right_edge}, Mid: {mid}, Offset: {offset}")
        else:
            mid, offset = self.center_x, 0

        # 步骤5：检测中心线交点（保持原有逻辑）
        center_intersection, center_dist = self._find_center_end(mask)
        # print(f"Center Intersection: {center_intersection}, Distance: {center_dist}")

        return {
            'front_mid': (mid, self.scan_y),
            'offset': offset,
            'left_edge': left_edge,
            'right_edge': right_edge,
            'center_intersection': center_intersection,
            'center_distance': center_dist
        }

    def _find_center_end(self, mask):
        """中心线终点检测"""
        center_line = mask[:, self.center_x]
        white_pixels = np.where(center_line == 255)[0]
        if white_pixels.size == 0:
            return (self.center_x, mask.shape[0]-1), 0
        return (self.center_x, white_pixels[-1]), mask.shape[0]-1 - white_pixels[-1]



    def visualize_black_bar(self, img, detect_result):
        """可视化限高杆检测结果"""
        vis_img = img.copy()
        points = detect_result.get('points')
        angle = detect_result.get('angle')
        if points is not None:
            x1, y1, x2, y2 = points
            cv2.line(vis_img, (x1, y1), (x2, y2), (0, 255, 0), 3)
            text = f"Bar Angle: {angle:.2f} deg"
            cv2.putText(vis_img, text, (min(x1, x2)+10, min(y1, y2)-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        return vis_img


    def visualize_track_points(self, img, result):
        """可视化赛道关键点（包含中心线和距离显示）"""
        vis_img = img.copy()
        h, w = vis_img.shape[:2]
        
        # # 绘制中心线（红色虚线）
        # center_x = w // 2
        # cv2.line(vis_img, 
        #         (center_x, 0), 
        #         (center_x, h), 
        #         color=(0, 0, 255),  # BGR格式红色
        #         thickness=1, 
        #         lineType=cv2.LINE_AA)
        
        # # 获取中心距离数据
        # center_dist = result.get('center_distance', 0)
        # intersection = result.get('center_intersection', (center_x, h-1))
        
        # # 在中心线底部显示距离信息
        # text = f"Center Dist: {center_dist}px"
        # text_scale = 0.4  # 字体缩放系数
        # text_thickness = 2  # 字体粗细
        
        # # 计算文本尺寸
        # (text_w, text_h), _ = cv2.getTextSize(text, 
        #                                     cv2.FONT_HERSHEY_SIMPLEX,
        #                                     text_scale, 
        #                                     text_thickness)
        
        # # 文本位置：左下角坐标
        # text_x = center_x - text_w // 2
        # text_y = h - 10  # 距离底部10像素
        
        # # 绘制文字背景（增强可读性）
        # cv2.rectangle(vis_img, 
        #             (text_x-5, text_y-text_h-5),
        #             (text_x + text_w +5, text_y+5), 
        #             color=(40, 40, 40),  # 深灰色背景
        #             thickness=-1)  # 填充
        
        # # 绘制文字
        # cv2.putText(vis_img, text,
        #         org=(text_x, text_y),
        #         fontFace=cv2.FONT_HERSHEY_SIMPLEX,
        #         fontScale=text_scale,
        #         color=(200, 200, 250),  # 浅蓝色文字
        #         thickness=text_thickness,
        #         lineType=cv2.LINE_AA)

        # # 在交点位置添加距离标注（可选）
        # if center_dist > 0:
        #     point_text = f"{center_dist}"
        #     (pt_w, pt_h), _ = cv2.getTextSize(point_text, 
        #                                     cv2.FONT_HERSHEY_SIMPLEX, 
        #                                     0.6, 1)
        #     cv2.putText(vis_img, point_text,
        #             (intersection[0] - pt_w//2, intersection[1] - 15),
        #             cv2.FONT_HERSHEY_SIMPLEX, 0.6,
        #             (255, 255, 255), 1, cv2.LINE_AA)
        
        # # 绘制扫描线
        # cv2.line(vis_img, (0, self.scan_y), (vis_img.shape[1], self.scan_y),
        #          (100, 100, 255), 1)

        # # 绘制左右边界点
        # if result['left_edge'] is not None:
        #     x = int(result['left_edge'])
        #     cv2.circle(vis_img, (x, self.scan_y), 5, (255,0,0), -1)
        # if result['right_edge'] is not None:
        #     x = int(result['right_edge'])
        #     cv2.circle(vis_img, (x, self.scan_y), 5, (0,255,0), -1)

        # # 绘制前视中点
        # cv2.circle(vis_img, result['front_mid'], 8, (0,255,255), -1)

        # # 绘制中心交点
        # cx, cy = result['center_intersection']
        # cv2.drawMarker(vis_img, (cx, cy), (255,0,255), markerSize=15, thickness=2)
        
        return vis_img



    def visualize_horizontal_line(self, img, detect_result):
        """
        只显示当前检测到的横线及其角度
        """
        vis_img = img.copy()
        points = detect_result.get('points')
        angle = detect_result.get('angle')
        if points is not None:
            x1, y1, x2, y2 = points
            cv2.line(vis_img, (x1, y1), (x2, y2), (0, 0, 255), 3)
            text = f"Angle: {angle:.2f} deg"
            cv2.putText(vis_img, text, (min(x1, x2)+10, min(y1, y2)-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        return vis_img

    def visualize_points(self, img, farthest, far_dist, nearest, near_dist):
        """可视化检测点"""
        # 绘制最远点（红色）
        if farthest:
            cv2.circle(img, farthest, 8, (0,0,255), -1)
            cv2.putText(img, f"Far: {far_dist}px", 
                       (farthest[0]-50, farthest[1]-20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)
        
        # 绘制最近点（绿色）
        if nearest:
            cv2.circle(img, nearest, 8, (0,255,0), -1)
            cv2.putText(img, f"Near: {near_dist}px",
                       (nearest[0]-50, nearest[1]-20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
        return img

    def visualize_boundaries(self, img, distances, points):
        """可视化边界距离检测结果"""
        vis_img = img.copy()
        left_dist, right_dist = distances
        left_point, right_point = points
        
        # 绘制中间分割线
        center_x = img.shape[1] // 2
        # cv2.line(vis_img, (center_x, 0), (center_x, img.shape[0]), (200,200,200), 1)

        # 绘制左边界点
        if left_point is not None:
            cv2.circle(vis_img, tuple(left_point), 8, (255,0,0), -1)  # 蓝色
            cv2.putText(vis_img, f"L: {left_dist}px", 
                    (left_point[0]+10, left_point[1]-15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,0,0), 2)

        # 绘制右边界点
        if right_point is not None:
            cv2.circle(vis_img, tuple(right_point), 8, (0,255,255), -1)  # 黄色
            cv2.putText(vis_img, f"R: {right_dist}px",
                    (right_point[0]-120, right_point[1]-15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)

        return vis_img

    def visualize_bottom_left(self, img, distance, point):
        """可视化底部最左侧点"""
        vis_img = img.copy()
        if point is not None:
            # 绘制底线
            img_height = img.shape[0]
            cv2.line(vis_img, (0, img_height-1), (img.shape[1], img_height-1), (200,200,200), 1)
            
            # 绘制检测点
            cv2.circle(vis_img, point, 8, (255,0,255), -1)  # 紫色
            cv2.putText(vis_img, f"Bottom Left: {distance}px",
                    (point[0]+10, point[1]-15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255,0,255), 2)
        return vis_img

    def visualize_center_intersection(self, img, point, distance):
        """可视化中间交点"""
        vis_img = img.copy()
        if point is not None:
            cv2.circle(vis_img, point, 8, (255, 0, 0), -1)  # 蓝色点
            cv2.putText(vis_img, f"Center: {distance}px",
                        (point[0] + 10, point[1] - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 2)
        return vis_img

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            img_height, img_width = cv_image.shape[:2]
            self.img_height = img_height  # 更新图像高度
            self.center_x = img_width // 2  # 更新中心x坐标
            # 颜色阈值处理
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            lower_yellow = np.array([20, 100, 100])
            upper_yellow = np.array([30, 255, 255])
            mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

            # 形态学处理
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7,7))
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)  # 先闭运算填充空洞
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

            # 检测关键点
            # farthest, far_dist, nearest, near_dist = self.detect_extreme_points(mask)
            # center_point, center_dist = self.detect_center_intersection(mask, img_width, img_height)
            track_points = self.detect_track_points(mask)
            # left_dist, left_point = self.detect_bottom_left_distance(mask, img_width)
            # detect_result = self.specialized_detect(mask)
            # # 可视化结果
            # bar_result = self.detect_black_bar(cv_image)
        
            result_img = cv_image.copy()
            # result_img = self.visualize_points(result_img, farthest, far_dist, nearest, near_dist)
            # boundary_dists, boundary_points = self.detect_boundary_distances(mask, img_width)
            # result_img = self.visualize_boundaries(cv_image, boundary_dists, boundary_points)
            # result_img = self.visualize_bottom_left(cv_image, left_dist, left_point)
            # result_img = self.visualize_center_intersection(result_img, center_point, center_dist)
            result_img = self.visualize_track_points(result_img, track_points)
            #  显示结果
            # result_img = self.visualize_horizontal_line(result_img, detect_result)
            # result_img = self.visualize_black_bar(result_img, bar_result)
            cv2.imshow("Extreme Points Detection", result_img)
            # 创建保存目录（如果不存在）
            save_dir = "/home/cyberdog_sim/src/cyberdog_controller/camera/saved_images"
            os.makedirs(save_dir, exist_ok=True)  # 确保目录存在

            key = cv2.waitKey(1) & 0xFF  # 获取按键值

            # 检查是否按下回车键（Enter键的ASCII码是13）
            if key == 13:
                # 生成时间戳文件名
                timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
                filename = f"extreme_points_{timestamp}.png"
                save_path = os.path.join(save_dir, filename)
                
                # 保存图像
                cv2.imwrite(save_path, result_img)
                print(f"图像已保存至: {save_path}")
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Image processing error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()