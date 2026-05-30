/// stage5_balance_beam.cpp — 赛段五：孤梁稳渡（编译存根）

#include "competition_manager/stages/stage5_balance_beam.hpp"
#include <cmath>
#include <algorithm>

namespace competition_manager {

void Stage5BalanceBeam::on_enter()
{
    RCLCPP_INFO(node_->get_logger(), "[Stage5] Enter — 孤梁稳渡");
    internal_state_    = InternalState::kMountBeam;
    all_feet_past_line_ = false;
    cmd_seq_           = 5000;
}

void Stage5BalanceBeam::on_exit()
{
    RCLCPP_INFO(node_->get_logger(), "[Stage5] Exit");
    cmd_stop(cmd_seq_++);
}

bool Stage5BalanceBeam::tick()
{
    if (is_robot_fallen()) { cmd_stop(cmd_seq_++); return false; }

    switch (internal_state_) {
    case InternalState::kMountBeam:
        // 慢速上桥
        cmd_velocity(kBeamWalkVelX, 0.0f, 0.0f, kBeamBodyHeight, 0.0f, cmd_seq_++);
        if (ctx_->latest_robot_state) {
            // 四足全部接触视为上桥完成
            const auto& c = ctx_->latest_robot_state->contact_estimate;
            if (c[0] > 0.5f && c[1] > 0.5f && c[2] > 0.5f && c[3] > 0.5f)
                internal_state_ = InternalState::kWalkOnBeam;
        }
        break;

    case InternalState::kWalkOnBeam: {
        float vyaw = 0.0f;
        compute_beam_balance_cmd(vyaw);
        cmd_velocity(kBeamWalkVelX, 0.0f, vyaw, kBeamBodyHeight, 0.0f, cmd_seq_++);
        if (ctx_->latest_boundary && ctx_->latest_boundary->dashed_line_detected)
            internal_state_ = InternalState::kDetectDashedLine;
        break;
    }

    case InternalState::kDetectDashedLine:
        internal_state_ = InternalState::kWaitAllFeetPass;
        break;

    case InternalState::kWaitAllFeetPass:
        if (check_all_feet_past_line()) {
            internal_state_ = InternalState::kExecuteJump;
        } else {
            float vyaw = 0.0f;
            compute_beam_balance_cmd(vyaw);
            cmd_velocity(kBeamWalkVelX, 0.0f, vyaw, kBeamBodyHeight, 0.0f, cmd_seq_++);
        }
        break;

    case InternalState::kExecuteJump:
        cmd_jump_down(cmd_seq_++);
        internal_state_ = InternalState::kConfirmLanded;
        break;

    case InternalState::kConfirmLanded:
        if (ctx_->latest_robot_state &&
            !ctx_->latest_robot_state->is_fallen)
        {
            RCLCPP_INFO(node_->get_logger(), "[Stage5] 跳下成功，赛段五完成");
            return true;
        }
        break;
    }
    return false;
}

void Stage5BalanceBeam::compute_beam_balance_cmd(float& vyaw)
{
    if (!ctx_->latest_robot_state) { vyaw = 0.0f; return; }
    float roll = ctx_->latest_robot_state->rpy[0];
    vyaw = -kRollGain * roll;
    vyaw = std::clamp(vyaw, -0.5f, 0.5f);
}

bool Stage5BalanceBeam::check_all_feet_past_line() const
{
    // 简化：虚线消失即认为全部通过
    if (!ctx_->latest_boundary) return false;
    return !ctx_->latest_boundary->dashed_line_detected;
}

}  // namespace competition_manager
