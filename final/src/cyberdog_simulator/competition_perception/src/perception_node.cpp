/// perception_node.cpp — 感知节点骨架实现
///
/// 功能：
///   1. 订阅相机图像
///   2. 订阅当前竞赛状态（用于按赛段过滤检测类别）
///   3. 调用 ObjectDetector 和 BoundaryDetector
///   4. 发布 DetectedObjectArray 和 BoundaryInfo

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

#include "competition_perception/object_detector.hpp"
#include "competition_perception/boundary_detector.hpp"
#include "competition_msgs/msg/detected_object_array.hpp"
#include "competition_msgs/msg/boundary_info.hpp"
#include "competition_msgs/msg/competition_state.hpp"

namespace competition_perception {

class PerceptionNode : public rclcpp::Node {
public:
    explicit PerceptionNode(const rclcpp::NodeOptions& options)
        : Node("competition_perception", options)
    {
        // ── 参数 ─────────────────────────────────────────────────
        declare_parameter("model_path",    "/opt/competition/models/objects.onnx");
        declare_parameter("conf_thresh",   0.5);
        declare_parameter("camera_height", 0.45);
        declare_parameter("camera_pitch",  15.0);
        declare_parameter("fx", 600.0);
        declare_parameter("fy", 600.0);

        const std::string model_path  = get_parameter("model_path").as_string();
        const float conf_thresh       = get_parameter("conf_thresh").as_double();
        const float cam_h             = get_parameter("camera_height").as_double();
        const float cam_p             = get_parameter("camera_pitch").as_double();
        const float fx                = get_parameter("fx").as_double();
        const float fy                = get_parameter("fy").as_double();

        // ── 初始化检测器 ─────────────────────────────────────────
        if (!detector_.init(model_path, conf_thresh)) {
            RCLCPP_ERROR(get_logger(), "Failed to load detection model: %s", model_path.c_str());
        }
        boundary_detector_.set_camera_params(cam_h, cam_p, fx, fy);

        // ── 发布者 ───────────────────────────────────────────────
        objects_pub_  = create_publisher<competition_msgs::msg::DetectedObjectArray>(
            "/perception/objects", 10);
        boundary_pub_ = create_publisher<competition_msgs::msg::BoundaryInfo>(
            "/perception/boundary", 10);

        // ── 订阅者 ───────────────────────────────────────────────
        image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10,
            [this](sensor_msgs::msg::Image::SharedPtr msg) { on_image(msg); });

        state_sub_ = create_subscription<competition_msgs::msg::CompetitionState>(
            "/competition/state", 10,
            [this](competition_msgs::msg::CompetitionState::SharedPtr msg) {
                current_stage_ = msg->stage;
            });

        RCLCPP_INFO(get_logger(), "PerceptionNode started.");
    }

private:
    void on_image(sensor_msgs::msg::Image::SharedPtr msg) {
        cv::Mat bgr;
        try {
            bgr = cv_bridge::toCvShare(msg, "bgr8")->image;
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_WARN(get_logger(), "cv_bridge error: %s", e.what());
            return;
        }

        // 目标检测
        auto objects = detector_.detect(bgr, current_stage_);
        objects.header = msg->header;
        objects_pub_->publish(objects);

        // 边界检测
        auto boundary = boundary_detector_.detect(bgr);
        boundary.header = msg->header;
        boundary_pub_->publish(boundary);
    }

    ObjectDetector   detector_;
    BoundaryDetector boundary_detector_;

    uint8_t current_stage_ = 0;

    rclcpp::Publisher<competition_msgs::msg::DetectedObjectArray>::SharedPtr objects_pub_;
    rclcpp::Publisher<competition_msgs::msg::BoundaryInfo>::SharedPtr        boundary_pub_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr                 image_sub_;
    rclcpp::Subscription<competition_msgs::msg::CompetitionState>::SharedPtr state_sub_;
};

}  // namespace competition_perception

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto node = std::make_shared<competition_perception::PerceptionNode>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
