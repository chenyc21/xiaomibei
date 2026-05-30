/// competition_manager_node.cpp
/// 竞赛管理器主节点 — 骨架实现
///
/// 功能：
///   1. 订阅感知层输出（目标、边界、机器人状态、运动反馈）
///   2. 驱动全局状态机（GlobalStateMachine）以 50Hz 运行
///   3. 发布竞赛状态、运动指令、语音指令

#include <chrono>
#include <memory>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <algorithm>
#include <gazebo_msgs/msg/model_states.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "competition_manager/global_state_machine.hpp"
#include "competition_msgs/msg/competition_state.hpp"
#include "competition_msgs/msg/detected_object_array.hpp"
#include "competition_msgs/msg/boundary_info.hpp"
#include "competition_msgs/msg/robot_state.hpp"
#include "competition_msgs/msg/motion_command.hpp"
#include "competition_msgs/msg/motion_feedback.hpp"
#include "competition_msgs/msg/voice_command.hpp"

using namespace std::chrono_literals;

namespace competition_manager {

class CompetitionManagerNode : public rclcpp::Node {
public:
    explicit CompetitionManagerNode(const rclcpp::NodeOptions& options)
        : Node("competition_manager", options)
    {
        // ── 发布者 ──────────────────────────────────────────────────
        state_pub_  = create_publisher<competition_msgs::msg::CompetitionState>(
            "/competition/state", 10);
        motion_pub_ = create_publisher<competition_msgs::msg::MotionCommand>(
            "/competition/motion_cmd", 10);
        voice_pub_  = create_publisher<competition_msgs::msg::VoiceCommand>(
            "/competition/voice_cmd", 10);

        // ── 订阅者 ──────────────────────────────────────────────────
        objects_sub_ = create_subscription<competition_msgs::msg::DetectedObjectArray>(
            "/perception/objects", 10,
            [this](competition_msgs::msg::DetectedObjectArray::SharedPtr msg) {
                fsm_->on_objects_update(msg);
            });

        boundary_sub_ = create_subscription<competition_msgs::msg::BoundaryInfo>(
            "/perception/boundary", 10,
            [this](competition_msgs::msg::BoundaryInfo::SharedPtr msg) {
                fsm_->on_boundary_update(msg);
            });

        robot_state_sub_ = create_subscription<competition_msgs::msg::RobotState>(
            "/robot/state", 10,
            [this](competition_msgs::msg::RobotState::SharedPtr msg) {
                fsm_->on_robot_state_update(msg);
            });

        // ── Gazebo 真值订阅（绕开 LCM 桥）──────────────────────────
        // race_gazebo.launch.py 已加载 libgazebo_ros_state.so，这里读取地面真值
        // 转换成 RobotState 注入 FSM；保证 latest_robot_state 总是有数据
        declare_parameter("robot_model_name", std::string("robot"));
        robot_model_name_ = get_parameter("robot_model_name").as_string();
        model_states_sub_ = create_subscription<gazebo_msgs::msg::ModelStates>(
            "/gazebo/model_states", rclcpp::SensorDataQoS(),
            std::bind(&CompetitionManagerNode::on_model_states, this,
                      std::placeholders::_1));

        motion_feedback_sub_ = create_subscription<competition_msgs::msg::MotionFeedback>(
            "/competition/motion_feedback", 10,
            [this](competition_msgs::msg::MotionFeedback::SharedPtr msg) {
                fsm_->on_motion_feedback(msg);
            });

        // ── 初始化状态机 ────────────────────────────────────────────
        fsm_ = std::make_unique<GlobalStateMachine>(this);

        // ── 注入发布函数 ────────────────────────────────────────────
        fsm_->inject_motion_publisher(
            [this](competition_msgs::msg::MotionCommand cmd) {
                motion_pub_->publish(cmd);
            });
        fsm_->inject_voice_publisher(
            [this](competition_msgs::msg::VoiceCommand cmd) {
                voice_pub_->publish(cmd);
            });
        fsm_->inject_state_publisher(
            [this](competition_msgs::msg::CompetitionState st) {
                state_pub_->publish(st);
            });

        // ── start_stage 参数：支持从任意赛段开始（调试用）────────────
        declare_parameter("start_stage", 1);
        int start_stage = get_parameter("start_stage").as_int();
        start_stage = std::clamp(start_stage, 1, 6);
        from_stage_ = static_cast<CompetitionStage>(start_stage);
        RCLCPP_INFO(get_logger(), "start_stage = %d", start_stage);

        // ── stop_after_stage 参数：完成此赛段后停止（默认 6 = 跑完全程）
        declare_parameter("stop_after_stage", 6);
        int stop_after = get_parameter("stop_after_stage").as_int();
        stop_after = std::clamp(stop_after, start_stage, 6);
        fsm_->set_stop_after(static_cast<CompetitionStage>(stop_after));
        RCLCPP_INFO(get_logger(), "stop_after_stage = %d", stop_after);

        fsm_->start(from_stage_);

        // ── 订阅外部开始信号（用于真实比赛裁判开始指令）────────────────
        // 注意：只有在 FSM 未运行时才响应，防止重复触发重置赛段
        start_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/competition/start", 10,
            [this](std_msgs::msg::Bool::SharedPtr msg) {
                if (msg->data && !fsm_->is_running()) {
                    RCLCPP_INFO(get_logger(), "[Start Signal] Received, starting from stage %d",
                                static_cast<int>(from_stage_));
                    fsm_->start(from_stage_);
                }
            });

        // ── 50Hz 定时器驱动状态机 ───────────────────────────────────
        timer_ = create_wall_timer(20ms, [this]() {
            fsm_->tick();
        });

        RCLCPP_INFO(get_logger(), "CompetitionManagerNode started.");
    }

private:
    // ── /gazebo/model_states 回调：转换为 RobotState 注入 FSM ───────────
    // 世界系 twist 通过当前 yaw 反旋转得到 body 系线速度；
    // 角速度对近似直立机体直接取 z 分量即可。
    void on_model_states(gazebo_msgs::msg::ModelStates::SharedPtr msg)
    {
        int idx = -1;
        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (msg->name[i] == robot_model_name_) { idx = static_cast<int>(i); break; }
        }
        if (idx < 0) {
            if (!warned_missing_model_) {
                RCLCPP_WARN(get_logger(),
                    "[GT] /gazebo/model_states 中找不到模型 '%s'。当前模型列表大小=%zu",
                    robot_model_name_.c_str(), msg->name.size());
                warned_missing_model_ = true;
            }
            return;
        }

        const auto& p = msg->pose[idx];
        const auto& t = msg->twist[idx];

        // 四元数 → RPY
        tf2::Quaternion q(p.orientation.x, p.orientation.y,
                          p.orientation.z, p.orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        // 世界系 → 机体系（仅绕 z 反旋转）
        const float cy = std::cos(static_cast<float>(yaw));
        const float sy = std::sin(static_cast<float>(yaw));
        const float vwx = static_cast<float>(t.linear.x);
        const float vwy = static_cast<float>(t.linear.y);
        const float vbx =  cy * vwx + sy * vwy;
        const float vby = -sy * vwx + cy * vwy;

        auto rs = std::make_shared<competition_msgs::msg::RobotState>();
        rs->header.stamp = now();
        rs->position       = {static_cast<float>(p.position.x),
                              static_cast<float>(p.position.y),
                              static_cast<float>(p.position.z)};
        rs->velocity_world = {vwx, vwy, static_cast<float>(t.linear.z)};
        rs->velocity_body  = {vbx, vby, static_cast<float>(t.linear.z)};
        rs->rpy            = {static_cast<float>(roll),
                              static_cast<float>(pitch),
                              static_cast<float>(yaw)};
        rs->quaternion     = {static_cast<float>(p.orientation.x),
                              static_cast<float>(p.orientation.y),
                              static_cast<float>(p.orientation.z),
                              static_cast<float>(p.orientation.w)};
        rs->omega_body     = {static_cast<float>(t.angular.x),
                              static_cast<float>(t.angular.y),
                              static_cast<float>(t.angular.z)};
        rs->is_fallen = (std::fabs(roll) > 1.2 || std::fabs(pitch) > 1.2);

        if (!got_first_gt_) {
            got_first_gt_ = true;
            RCLCPP_INFO(get_logger(),
                "[GT] /gazebo/model_states 已就绪 — 注入 RobotState（model='%s' idx=%d）",
                robot_model_name_.c_str(), idx);
        }
        fsm_->on_robot_state_update(rs);
    }

    std::unique_ptr<GlobalStateMachine> fsm_;

    rclcpp::Publisher<competition_msgs::msg::CompetitionState>::SharedPtr state_pub_;
    rclcpp::Publisher<competition_msgs::msg::MotionCommand>::SharedPtr    motion_pub_;
    rclcpp::Publisher<competition_msgs::msg::VoiceCommand>::SharedPtr     voice_pub_;

    rclcpp::Subscription<competition_msgs::msg::DetectedObjectArray>::SharedPtr objects_sub_;
    rclcpp::Subscription<competition_msgs::msg::BoundaryInfo>::SharedPtr        boundary_sub_;
    rclcpp::Subscription<competition_msgs::msg::RobotState>::SharedPtr          robot_state_sub_;
    rclcpp::Subscription<competition_msgs::msg::MotionFeedback>::SharedPtr      motion_feedback_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr                          start_sub_;
    rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr                model_states_sub_;
    std::string                                                                    robot_model_name_;
    bool                                                                           warned_missing_model_{false};
    bool                                                                           got_first_gt_{false};
    CompetitionStage                                                               from_stage_{CompetitionStage::kStage1};

    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace competition_manager

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto node = std::make_shared<competition_manager::CompetitionManagerNode>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
