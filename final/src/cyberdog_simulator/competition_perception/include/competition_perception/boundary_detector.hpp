#pragma once
#include <opencv2/opencv.hpp>
#include "competition_msgs/msg/boundary_info.hpp"

namespace competition_perception {

/// 赛道边界检测器
/// 使用HSV颜色过滤检测黄色边沿线（RGB: 255,255,0）
/// 同时检测虚线（赛段分界）和弯道
class BoundaryDetector {
public:
    BoundaryDetector();
    ~BoundaryDetector() = default;

    /// 分析图像，返回边界信息
    competition_msgs::msg::BoundaryInfo detect(const cv::Mat& bgr_image);

    /// 设置相机-地面投影参数（用于像素→米换算）
    void set_camera_params(float height_m, float pitch_deg, float fx, float fy);

private:
    // ── 黄色边沿线检测（HSV阈值过滤）
    cv::Mat threshold_yellow(const cv::Mat& hsv) const;

    // ── 从二值图中提取左右边界轮廓
    void extract_boundaries(const cv::Mat& binary,
                             float& left_x, float& right_x,
                             bool& left_found, bool& right_found) const;

    // ── 弯道检测（使用霍夫直线检测）
    bool detect_turn(const cv::Mat& binary, float& turn_angle_deg) const;

    // ── 虚线检测（统计间断性黑白交替）
    bool detect_dashed_line(const cv::Mat& binary, float& distance_m) const;

    // ── 像素偏移 → 物理距离（米）
    float pixel_to_meter(float pixel_offset) const;

    // HSV阈值（黄色边沿线）
    cv::Scalar hsv_lower_yellow_{20, 100, 100};
    cv::Scalar hsv_upper_yellow_{35, 255, 255};

    // 相机参数
    float camera_height_m_  = 0.45f;  // 相机离地高度
    float camera_pitch_deg_ = 15.0f;  // 相机俯仰角
    float fx_ = 600.0f;
    float fy_ = 600.0f;

    // 感兴趣区域（只分析图像下半部分）
    float roi_top_ratio_ = 0.4f;  // 从40%行开始分析
};

}  // namespace competition_perception
