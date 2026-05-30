/// stage2_ball_hunt.cpp — 赛段二：荒野寻珠（编译存根）

#include "competition_manager/stages/stage2_ball_hunt.hpp"

namespace competition_manager {

using DO = competition_msgs::msg::DetectedObject;

void Stage2BallHunt::on_enter()
{
    RCLCPP_INFO(node_->get_logger(), "[Stage2] Enter — 荒野寻珠");
    internal_state_ = InternalState::kScanArea;
    cmd_seq_        = 2000;
    completed_balls_.clear();
    target_ball_id_ = 0;
    cmd_stand_up(cmd_seq_++);
}

void Stage2BallHunt::on_exit()
{
    RCLCPP_INFO(node_->get_logger(), "[Stage2] Exit");
    cmd_stop(cmd_seq_++);
}

bool Stage2BallHunt::tick()
{
    if (is_robot_fallen()) { cmd_stop(cmd_seq_++); return false; }

    switch (internal_state_) {
    case InternalState::kScanArea:
        infer_orange_ball_layout();
        internal_state_ = InternalState::kSelectTarget;
        break;

    case InternalState::kSelectTarget:
        if (!select_next_target()) {
            // 全部完成
            internal_state_ = InternalState::kNavigateExit;
        } else {
            internal_state_ = InternalState::kApproach;
        }
        break;

    case InternalState::kApproach: {
        if (!ctx_->latest_objects) break;
        for (const auto& obj : ctx_->latest_objects->objects) {
            if (obj.object_type == DO::TYPE_ORANGE_BALL &&
                obj.object_id == target_ball_id_)
            {
                if (obj.distance < 0.6f) {
                    internal_state_ = InternalState::kHit;
                } else {
                    float vy = -obj.rel_y * 0.5f;
                    vy = std::clamp(vy, -0.3f, 0.3f);
                    cmd_velocity(0.3f, vy, 0.0f, 0.0f, 0.0f, cmd_seq_++);
                }
                break;
            }
        }
        break;
    }

    case InternalState::kHit:
        cmd_hit_forward(cmd_seq_++);
        internal_state_ = InternalState::kConfirmHit;
        break;

    case InternalState::kConfirmHit:
        if (detect_ball_oscillation(target_ball_id_)) {
            completed_balls_.insert(target_ball_id_);
            internal_state_ = InternalState::kSelectTarget;
        }
        break;

    case InternalState::kNavigateExit:
        cmd_velocity(0.3f, 0.0f, 0.0f, 0.0f, 0.0f, cmd_seq_++);
        if (ctx_->latest_boundary && ctx_->latest_boundary->dashed_line_detected)
            internal_state_ = InternalState::kWaitExit;
        break;

    case InternalState::kWaitExit:
        if (ctx_->latest_boundary && !ctx_->latest_boundary->dashed_line_detected)
            return true;
        cmd_velocity(0.2f, 0.0f, 0.0f, 0.0f, 0.0f, cmd_seq_++);
        break;
    }
    return false;
}

void Stage2BallHunt::infer_orange_ball_layout() {}

bool Stage2BallHunt::select_next_target()
{
    if (!ctx_->latest_objects) return false;
    for (const auto& obj : ctx_->latest_objects->objects) {
        if (obj.object_type == DO::TYPE_ORANGE_BALL &&
            obj.is_target &&
            completed_balls_.find(obj.object_id) == completed_balls_.end())
        {
            target_ball_id_ = obj.object_id;
            return true;
        }
    }
    return false;
}

bool Stage2BallHunt::detect_ball_oscillation(uint32_t /*ball_id*/) const
{
    // 简化判断：假设下一帧目标状态已更新为 INTERACTED
    if (!ctx_->latest_objects) return false;
    for (const auto& obj : ctx_->latest_objects->objects) {
        if (obj.object_id == target_ball_id_ &&
            obj.object_status == DO::STATUS_INTERACTED)
            return true;
    }
    return false;
}

}  // namespace competition_manager
