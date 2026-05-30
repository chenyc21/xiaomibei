/// stage1_stone_path.cpp — 赛段一：石径探路（编译存根）
/// 完整实现待补充；当前提供最小可编译骨架

#include "competition_manager/stages/stage1_stone_path.hpp"

namespace competition_manager {

void Stage1StonePath::on_enter()
{
    RCLCPP_INFO(node_->get_logger(), "[Stage1] Enter — 石径探路");
    internal_state_ = InternalState::kStandUp;
    cmd_seq_        = 1000;
    cmd_stand_up(cmd_seq_++);
}

void Stage1StonePath::on_exit()
{
    RCLCPP_INFO(node_->get_logger(), "[Stage1] Exit");
    cmd_stop(cmd_seq_++);
}

bool Stage1StonePath::tick()
{
    if (is_robot_fallen()) { cmd_stop(cmd_seq_++); return false; }

    switch (internal_state_) {
    case InternalState::kStandUp:
        internal_state_ = InternalState::kFollowPath;
        break;

    case InternalState::kFollowPath:
        boundary_follow_control();
        if (ctx_->latest_boundary && ctx_->latest_boundary->turn_detected)
            internal_state_ = InternalState::kDetectTurn;
        break;

    case InternalState::kDetectTurn:
        internal_state_ = InternalState::kExecuteTurn;
        break;

    case InternalState::kExecuteTurn: {
        float angle = estimate_turn_angle();
        float yaw_rate = (angle > 0) ? 0.8f : -0.8f;
        cmd_velocity(0.2f, 0.0f, yaw_rate, 0.0f, 0.0f, cmd_seq_++);
        if (ctx_->latest_boundary && !ctx_->latest_boundary->turn_detected)
            internal_state_ = InternalState::kWaitExit;
        break;
    }

    case InternalState::kWaitExit:
        if (ctx_->latest_boundary && !ctx_->latest_boundary->dashed_line_detected) {
            return true;
        }
        cmd_velocity(0.2f, 0.0f, 0.0f, 0.0f, 0.0f, cmd_seq_++);
        break;
    }
    return false;
}

void Stage1StonePath::boundary_follow_control()
{
    float vx = 0.4f, vyaw = 0.0f;
    if (ctx_->latest_boundary) {
        vyaw = -0.8f * ctx_->latest_boundary->center_offset_m;
        vyaw = std::clamp(vyaw, -1.2f, 1.2f);
    }
    cmd_velocity(vx, 0.0f, vyaw, 0.0f, 0.0f, cmd_seq_++);
}

float Stage1StonePath::estimate_turn_angle() const
{
    if (ctx_->latest_boundary) return ctx_->latest_boundary->turn_angle_deg;
    return 0.0f;
}

}  // namespace competition_manager
