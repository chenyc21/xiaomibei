#pragma once
#include <memory>
#include <opencv2/opencv.hpp>
#include "competition_msgs/msg/detected_object_array.hpp"
#include "competition_msgs/msg/competition_state.hpp"

namespace competition_perception {

/// 目标检测器
/// 基于 YOLOv8 或轻量级检测模型，识别竞赛相关目标
class ObjectDetector {
public:
    ObjectDetector();
    ~ObjectDetector() = default;

    /// 初始化模型（加载ONNX/TensorRT模型）
    /// @param model_path  ONNX/TRT模型路径
    /// @param conf_thresh 置信度阈值
    bool init(const std::string& model_path, float conf_thresh = 0.5f);

    /// 在图像上执行检测
    /// @param image   输入BGR图像
    /// @param stage   当前赛段（用于过滤不相关类别）
    /// @return 检测到的目标列表
    competition_msgs::msg::DetectedObjectArray detect(
        const cv::Mat& image, uint8_t stage);

    /// 估计目标的相对位置（基于已知目标尺寸+针孔相机模型）
    /// @param bbox_h_px   目标高度像素
    /// @param real_h_m    目标真实高度（米）
    /// @param focal_len_px 焦距（像素）
    float estimate_distance(float bbox_h_px, float real_h_m, float focal_len_px) const;

private:
    struct Detection {
        int   class_id;
        float confidence;
        cv::Rect2f bbox;
    };

    std::vector<Detection> run_inference(const cv::Mat& image);

    // 类别ID → DetectedObject.TYPE_* 映射表
    uint8_t class_id_to_type(int class_id) const;

    float conf_thresh_ = 0.5f;
    bool  initialized_ = false;

    // 相机内参（在launch时通过参数服务器配置）
    float fx_ = 600.0f;  // 焦距x（像素）
    float fy_ = 600.0f;  // 焦距y（像素）
    float cx_ = 320.0f;  // 主点x
    float cy_ = 240.0f;  // 主点y

    // 各类别真实尺寸（高度，米）
    static constexpr float kOrangeBallH  = 0.20f;  // 直径20cm
    static constexpr float kSoccerH      = 0.20f;
    static constexpr float kColaH        = 0.33f;  // 约33cm
    static constexpr float kHeightBarH   = 0.70f;  // 限高杆底距地40cm，杆长110cm
};

}  // namespace competition_perception
