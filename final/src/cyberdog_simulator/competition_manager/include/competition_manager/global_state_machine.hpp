#pragma once

#include <memory>
#include <functional>
#include <string>
#include <unordered_map>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include "competition_msgs/msg/competition_state.hpp"
#include "competition_msgs/msg/detected_object_array.hpp"
#include "competition_msgs/msg/boundary_info.hpp"
#include "competition_msgs/msg/robot_state.hpp"
#include "competition_msgs/msg/motion_command.hpp"
#include "competition_msgs/msg/motion_feedback.hpp"
#include "competition_msgs/msg/voice_command.hpp"

namespace competition_manager {

// ─────────────────────────────────────────────
// 枚举定义
// ─────────────────────────────────────────────
enum class CompetitionStage : uint8_t {
    kIdle       = 0,
    kStage1     = 1,   // 石径探路
    kStage2     = 2,   // 荒野寻珠
    kStage3     = 3,   // 曲道冲锋
    kStage4     = 4,   // 深隧寻珍
    kStage5     = 5,   // 孤梁稳渡
    kStage6     = 6,   // 撷金建功
    kFinished   = 7,
    kError      = 8,
};

enum class SubState : uint8_t {
    kInit       = 0,
    kRunning    = 1,
    kInteracting= 2,
    kAvoiding   = 3,
    kExiting    = 4,
    kRecovery   = 5,
};

// ─────────────────────────────────────────────
// 前向声明
// ─────────────────────────────────────────────
class StageHandlerBase;

// ─────────────────────────────────────────────
// 全局状态机上下文（各赛段共享）
// ─────────────────────────────────────────────
struct FSMContext {
    // 最新感知数据
    competition_msgs::msg::DetectedObjectArray::SharedPtr latest_objects;
    competition_msgs::msg::BoundaryInfo::SharedPtr        latest_boundary;
    competition_msgs::msg::RobotState::SharedPtr          latest_robot_state;
    competition_msgs::msg::MotionFeedback::SharedPtr      latest_motion_feedback;

    // 比赛计时
    std::chrono::steady_clock::time_point race_start_time;
    std::chrono::steady_clock::time_point stage_start_time;

    // 得分记录（估算）
    int score   = 0;
    int penalty = 0;

    // 发布接口（由FSM注入给赛段处理器使用）
    std::function<void(competition_msgs::msg::MotionCommand)> publish_motion;
    std::function<void(competition_msgs::msg::VoiceCommand)>  publish_voice;
};

// ─────────────────────────────────────────────
// 全局状态机主类
// ─────────────────────────────────────────────
class GlobalStateMachine {
public:
    explicit GlobalStateMachine(rclcpp::Node* node);
    ~GlobalStateMachine() = default;

    /// 主循环 tick，由节点定时器调用（建议 50Hz）
    void tick();

    /// 外部触发：比赛开始（从赛段一）
    void start();

    /// 外部触发：从指定赛段开始（调试用）
    void start(CompetitionStage from_stage);

    /// 外部触发：紧急停止
    void emergency_stop();

    // ── 发布函数注入（由节点层调用）────────────────────────────────────
    void inject_motion_publisher(
        std::function<void(competition_msgs::msg::MotionCommand)> fn) {
        ctx_.publish_motion = std::move(fn);
    }
    void inject_voice_publisher(
        std::function<void(competition_msgs::msg::VoiceCommand)> fn) {
        ctx_.publish_voice = std::move(fn);
    }
    void inject_state_publisher(
        std::function<void(competition_msgs::msg::CompetitionState)> fn) {
        publish_state_fn_ = std::move(fn);
    }

    /// 获取当前赛段
    CompetitionStage current_stage() const { return current_stage_; }
    bool is_running() const { return current_stage_ != CompetitionStage::kIdle; }

    /// 设置停止赛段（调试用：完成该赛段后不继续下一赛段）
    void set_stop_after(CompetitionStage s) { stop_after_stage_ = s; }

    /// 获取当前子状态
    SubState current_sub_state() const { return current_sub_state_; }

    /// 注入感知数据（由订阅回调调用）
    void on_objects_update(competition_msgs::msg::DetectedObjectArray::SharedPtr msg);
    void on_boundary_update(competition_msgs::msg::BoundaryInfo::SharedPtr msg);
    void on_robot_state_update(competition_msgs::msg::RobotState::SharedPtr msg);
    void on_motion_feedback(competition_msgs::msg::MotionFeedback::SharedPtr msg);

private:
    /// 赛段转换
    void transition_to(CompetitionStage next_stage);

    /// 子状态转换
    void set_sub_state(SubState sub);

    /// 构建并发布 CompetitionState
    void publish_state();

    /// 注册所有赛段处理器
    void register_stage_handlers();

    rclcpp::Node* node_;
    FSMContext    ctx_;

    CompetitionStage current_stage_     = CompetitionStage::kIdle;
    SubState         current_sub_state_ = SubState::kInit;
    CompetitionStage stop_after_stage_  = CompetitionStage::kStage6;  // 完成此赛段后停止

    std::function<void(competition_msgs::msg::CompetitionState)> publish_state_fn_;

    std::unordered_map<CompetitionStage,
                       std::shared_ptr<StageHandlerBase>> stage_handlers_;

    uint32_t motion_seq_id_ = 0;
};

}  // namespace competition_manager
