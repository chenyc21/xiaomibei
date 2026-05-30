/// motion_bridge_node.cpp — 运动控制桥接节点骨架
///
/// 功能：
///   1. 订阅高层 MotionCommand（来自竞赛管理器）
///   2. 将高层指令翻译为：
///      a) LCM robot_control_cmd_lcmt → 发送给 cyberdog_locomotion
///      b) ROS2 YamlParam → 切换底层控制模式
///   3. 订阅 LCM state_estimator_lcmt 聚合为 RobotState 发布
///   4. 订阅 LCM motion_control_response_lcmt 翻译为 MotionFeedback 发布

#include <rclcpp/rclcpp.hpp>
#include <lcm/lcm-cpp.hpp>
#include <thread>
#include <atomic>

#include "competition_msgs/msg/motion_command.hpp"
#include "competition_msgs/msg/motion_feedback.hpp"
#include "competition_msgs/msg/robot_state.hpp"
#include "cyberdog_msg/msg/yaml_param.hpp"

// LCM 生成的类型头文件（来自 cyberdog_locomotion）
#include "robot_control_cmd_lcmt.hpp"
#include "state_estimator_lcmt.hpp"
#include "motion_control_response_lcmt.hpp"
#include "competition_cmd_lcmt.hpp"

using namespace std::chrono_literals;
using MotionCmd = competition_msgs::msg::MotionCommand;

namespace competition_motion_bridge {

// 高层模式 → 底层FSM模式映射
// 参考 cyberdog_locomotion 的 FsmStateName 枚举
static constexpr int kFsmLocomotion    = 4;
static constexpr int kFsmBalanceStand  = 6;
static constexpr int kFsmRecoveryStand = 12;
static constexpr int kFsmPassive       = 0;

class MotionBridgeNode : public rclcpp::Node {
public:
    explicit MotionBridgeNode(const rclcpp::NodeOptions& options)
        : Node("competition_motion_bridge", options)
        , lcm_running_(true)
    {
        // ── 发布者 ─────────────────────────────────────────────
        robot_state_pub_ = create_publisher<competition_msgs::msg::RobotState>(
            "/robot/state", 10);
        feedback_pub_ = create_publisher<competition_msgs::msg::MotionFeedback>(
            "/competition/motion_feedback", 10);
        yaml_pub_ = create_publisher<cyberdog_msg::msg::YamlParam>(
            "/yaml_parameter", 10);

        // ── 订阅者 ─────────────────────────────────────────────
        motion_cmd_sub_ = create_subscription<MotionCmd>(
            "/competition/motion_cmd", 10,
            [this](MotionCmd::SharedPtr msg) { on_motion_command(msg); });

        // ── 初始化 LCM ─────────────────────────────────────────
        if (!lcm_.good()) {
            RCLCPP_ERROR(get_logger(), "LCM initialization failed!");
            return;
        }

        // 订阅 LCM 状态估计和运动响应
        lcm_.subscribe("state_estimator", &MotionBridgeNode::on_lcm_state_estimator, this);
        lcm_.subscribe("motion_control_response", &MotionBridgeNode::on_lcm_motion_response, this);

        // LCM 在独立线程中轮询
        lcm_thread_ = std::thread([this]() {
            while (lcm_running_ && lcm_.good()) {
                lcm_.handleTimeout(10);  // 10ms超时
            }
        });

        // 切换底层控制模式：使用gamepad模式（不使用遥控器）
        switch_to_gamepad_mode();

        RCLCPP_INFO(get_logger(), "MotionBridgeNode started.");
    }

    ~MotionBridgeNode() {
        lcm_running_ = false;
        if (lcm_thread_.joinable()) lcm_thread_.join();
    }

private:
    // ── 高层指令处理 ──────────────────────────────────────────
    void on_motion_command(MotionCmd::SharedPtr msg) {
        robot_control_cmd_lcmt lcm_cmd{};
        lcm_cmd.life_count++;

        switch (msg->mode) {
        case MotionCmd::MODE_STOP:
            lcm_cmd.mode    = kFsmPassive;
            lcm_cmd.gait_id = 0;
            break;

        case MotionCmd::MODE_STAND_UP:
        case MotionCmd::MODE_RECOVERY_STAND:
            lcm_cmd.mode    = kFsmRecoveryStand;
            lcm_cmd.gait_id = 0;
            break;

        case MotionCmd::MODE_LIE_DOWN:
            lcm_cmd.mode    = kFsmPassive;
            lcm_cmd.gait_id = 0;
            break;

        case MotionCmd::MODE_VELOCITY_CTRL:
        case MotionCmd::MODE_WALK_FORWARD:
        case MotionCmd::MODE_WALK_BACKWARD:
        case MotionCmd::MODE_TURN_LEFT:
        case MotionCmd::MODE_TURN_RIGHT:
        case MotionCmd::MODE_STRAFE_LEFT:
        case MotionCmd::MODE_STRAFE_RIGHT:
        case MotionCmd::MODE_TROT:
            lcm_cmd.mode         = kFsmLocomotion;
            lcm_cmd.gait_id      = 0;  // Trot
            lcm_cmd.vel_des[0]   = msg->vel_x;
            lcm_cmd.vel_des[1]   = msg->vel_y;
            lcm_cmd.vel_des[2]   = msg->vel_yaw;
            lcm_cmd.step_height[0] = (msg->step_height > 0) ? msg->step_height : 0.08f;
            lcm_cmd.step_height[1] = lcm_cmd.step_height[0];
            if (msg->body_height > 0) {
                lcm_cmd.pos_des[2] = msg->body_height;
            }
            break;

        case MotionCmd::MODE_LOW_WALK:
            lcm_cmd.mode         = kFsmLocomotion;
            lcm_cmd.gait_id      = 0;
            lcm_cmd.vel_des[0]   = msg->vel_x;
            lcm_cmd.vel_des[1]   = msg->vel_y;
            lcm_cmd.vel_des[2]   = msg->vel_yaw;
            lcm_cmd.pos_des[2]   = 0.15f;  // 极低机身通过限高杆
            break;

        case MotionCmd::MODE_BALANCE_STAND:
            lcm_cmd.mode    = kFsmBalanceStand;
            lcm_cmd.gait_id = 0;
            break;

        case MotionCmd::MODE_JUMP_DOWN:
            // TODO: 调用专用跳下动作（motion list）
            lcm_cmd.mode    = kFsmLocomotion;
            lcm_cmd.gait_id = 6;  // 跳跃步态ID（待确认）
            break;

        case MotionCmd::MODE_HIT_FORWARD:
        case MotionCmd::MODE_KICK_FORWARD:
            // 高速短距前冲
            lcm_cmd.mode       = kFsmLocomotion;
            lcm_cmd.gait_id    = 0;
            lcm_cmd.vel_des[0] = (msg->mode == MotionCmd::MODE_KICK_FORWARD) ? 1.2f : 0.8f;
            lcm_cmd.duration   = 500;  // 500ms
            break;

        default:
            RCLCPP_WARN(get_logger(), "Unknown motion mode: %d", msg->mode);
            return;
        }

        lcm_cmd.duration = static_cast<int32_t>(msg->duration_s * 1000.0f);
        lcm_.publish("robot_control_cmd", &lcm_cmd);

        last_cmd_seq_ = msg->seq_id;
        publish_feedback(msg->seq_id,
            competition_msgs::msg::MotionFeedback::EXEC_RUNNING);
    }

    // ── LCM 回调 ──────────────────────────────────────────────
    void on_lcm_state_estimator(const lcm::ReceiveBuffer*, const std::string&,
                                 const state_estimator_lcmt* msg) {
        competition_msgs::msg::RobotState rs;
        rs.header.stamp = now();

        for (int i = 0; i < 3; i++) {
            rs.position[i]        = msg->p[i];
            rs.velocity_world[i]  = msg->vWorld[i];
            rs.velocity_body[i]   = msg->vBody[i];
            rs.rpy[i]             = msg->rpy[i];
            rs.omega_body[i]      = msg->omegaBody[i];
        }
        for (int i = 0; i < 4; i++) {
            rs.quaternion[i]       = msg->quat[i];
            rs.contact_estimate[i] = msg->contactEstimate[i];
        }
        rs.timestamp_us = msg->timestamp;

        // 跌倒判断：pitch或roll超过阈值
        rs.is_fallen = (std::abs(msg->rpy[0]) > 1.2f || std::abs(msg->rpy[1]) > 1.2f);

        robot_state_pub_->publish(rs);
    }

    void on_lcm_motion_response(const lcm::ReceiveBuffer*, const std::string&,
                                 const motion_control_response_lcmt* msg) {
        (void)msg;
        // TODO: 根据响应中的 order_process_bar 更新反馈进度
        publish_feedback(last_cmd_seq_,
            competition_msgs::msg::MotionFeedback::EXEC_RUNNING,
            msg->order_process_bar / 100.0f);
    }

    void publish_feedback(uint32_t seq_id, uint8_t status, float progress = 0.0f) {
        competition_msgs::msg::MotionFeedback fb;
        fb.header.stamp = now();
        fb.seq_id   = seq_id;
        fb.status   = status;
        fb.progress = progress;
        feedback_pub_->publish(fb);
    }

    void switch_to_gamepad_mode() {
        cyberdog_msg::msg::YamlParam param;
        param.name      = "use_rc";
        param.kind      = 2;  // kS64
        param.s64_value = 0;
        param.is_user   = 0;
        yaml_pub_->publish(param);
        RCLCPP_INFO(get_logger(), "Switched to gamepad (LCM) control mode.");
    }

    lcm::LCM              lcm_;
    std::thread           lcm_thread_;
    std::atomic<bool>     lcm_running_;
    uint32_t              last_cmd_seq_ = 0;

    rclcpp::Publisher<competition_msgs::msg::RobotState>::SharedPtr    robot_state_pub_;
    rclcpp::Publisher<competition_msgs::msg::MotionFeedback>::SharedPtr feedback_pub_;
    rclcpp::Publisher<cyberdog_msg::msg::YamlParam>::SharedPtr          yaml_pub_;

    rclcpp::Subscription<MotionCmd>::SharedPtr motion_cmd_sub_;
};

}  // namespace competition_motion_bridge

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto node = std::make_shared<competition_motion_bridge::MotionBridgeNode>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
