/// global_state_machine.cpp — 全局状态机实现骨架

#include "competition_manager/global_state_machine.hpp"
#include "competition_manager/stages/stage1_stone_path.hpp"
#include "competition_manager/stages/stage2_ball_hunt.hpp"
#include "competition_manager/stages/stage3_curved_track.hpp"
#include "competition_manager/stages/stage4_deep_tunnel.hpp"
#include "competition_manager/stages/stage5_balance_beam.hpp"
#include "competition_manager/stages/stage6_final_goal.hpp"

namespace competition_manager {

GlobalStateMachine::GlobalStateMachine(rclcpp::Node* node)
    : node_(node)
{
    // 初始化发布者（由调用方节点传入实际publisher）
    // 这里通过注入函数的方式解耦
    register_stage_handlers();
}

void GlobalStateMachine::register_stage_handlers() {
    stage_handlers_[CompetitionStage::kStage1] =
        std::make_shared<Stage1StonePath>(node_, &ctx_);
    stage_handlers_[CompetitionStage::kStage2] =
        std::make_shared<Stage2BallHunt>(node_, &ctx_);
    stage_handlers_[CompetitionStage::kStage3] =
        std::make_shared<Stage3CurvedTrack>(node_, &ctx_);
    stage_handlers_[CompetitionStage::kStage4] =
        std::make_shared<Stage4DeepTunnel>(node_, &ctx_);
    stage_handlers_[CompetitionStage::kStage5] =
        std::make_shared<Stage5BalanceBeam>(node_, &ctx_);
    stage_handlers_[CompetitionStage::kStage6] =
        std::make_shared<Stage6FinalGoal>(node_, &ctx_);
}

void GlobalStateMachine::start() {
    start(CompetitionStage::kStage1);
}

void GlobalStateMachine::start(CompetitionStage from_stage) {
    RCLCPP_INFO(node_->get_logger(), "[FSM] Competition STARTED from stage %d",
        static_cast<int>(from_stage));
    ctx_.race_start_time = std::chrono::steady_clock::now();
    transition_to(from_stage);
}

void GlobalStateMachine::emergency_stop() {
    RCLCPP_WARN(node_->get_logger(), "[FSM] EMERGENCY STOP");
    competition_msgs::msg::MotionCommand stop_cmd;
    stop_cmd.mode = competition_msgs::msg::MotionCommand::MODE_STOP;
    if (ctx_.publish_motion) ctx_.publish_motion(stop_cmd);
    current_stage_ = CompetitionStage::kError;
    publish_state();
}

void GlobalStateMachine::tick() {
    auto it = stage_handlers_.find(current_stage_);
    if (it == stage_handlers_.end()) {
        // kIdle / kFinished / kError — 无活跃处理器
        return;
    }

    bool stage_done = it->second->tick();

    if (stage_done) {
        if (current_stage_ == stop_after_stage_) {
            RCLCPP_INFO(node_->get_logger(),
                "[FSM] stop_after_stage=%d 已完成，停止比赛",
                static_cast<int>(stop_after_stage_));
            transition_to(CompetitionStage::kFinished);
        } else {
            auto next = static_cast<CompetitionStage>(
                static_cast<uint8_t>(current_stage_) + 1);
            transition_to(next);
        }
    }

    publish_state();
}

void GlobalStateMachine::transition_to(CompetitionStage next) {
    // 退出当前赛段
    auto cur_it = stage_handlers_.find(current_stage_);
    if (cur_it != stage_handlers_.end()) {
        cur_it->second->on_exit();
    }

    RCLCPP_INFO(node_->get_logger(),
        "[FSM] Stage transition: %d → %d",
        static_cast<int>(current_stage_),
        static_cast<int>(next));

    current_stage_    = next;
    current_sub_state_ = SubState::kInit;
    ctx_.stage_start_time = std::chrono::steady_clock::now();

    // 进入新赛段
    auto next_it = stage_handlers_.find(current_stage_);
    if (next_it != stage_handlers_.end()) {
        next_it->second->on_enter();
    }
}

void GlobalStateMachine::publish_state() {
    competition_msgs::msg::CompetitionState state_msg;
    state_msg.header.stamp     = node_->now();
    state_msg.stage            = static_cast<uint8_t>(current_stage_);
    state_msg.sub_state        = static_cast<uint8_t>(current_sub_state_);
    state_msg.score            = ctx_.score;
    state_msg.penalty          = ctx_.penalty;

    auto now = std::chrono::steady_clock::now();
    state_msg.elapsed_time_s = std::chrono::duration<float>(
        now - ctx_.race_start_time).count();
    state_msg.stage_time_s  = std::chrono::duration<float>(
        now - ctx_.stage_start_time).count();

    // NOTE: 需要通过注入发布者来发布，此处留待节点层完成
    if (publish_state_fn_) publish_state_fn_(state_msg);
}

// ── 感知数据注入 ─────────────────────────────────────────────
void GlobalStateMachine::on_objects_update(
    competition_msgs::msg::DetectedObjectArray::SharedPtr msg) {
    ctx_.latest_objects = msg;
}

void GlobalStateMachine::on_boundary_update(
    competition_msgs::msg::BoundaryInfo::SharedPtr msg) {
    ctx_.latest_boundary = msg;
}

void GlobalStateMachine::on_robot_state_update(
    competition_msgs::msg::RobotState::SharedPtr msg) {
    ctx_.latest_robot_state = msg;
}

void GlobalStateMachine::on_motion_feedback(
    competition_msgs::msg::MotionFeedback::SharedPtr msg) {
    ctx_.latest_motion_feedback = msg;
}

}  // namespace competition_manager
