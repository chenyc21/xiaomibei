/// stage6_final_goal.cpp — 赛段六：撷金建功（编译存根）

#include "competition_manager/stages/stage6_final_goal.hpp"
#include <cmath>
#include <algorithm>

namespace competition_manager {

using DO = competition_msgs::msg::DetectedObject;

void Stage6FinalGoal::on_enter()
{
    RCLCPP_INFO(node_->get_logger(), "[Stage6] Enter — 撷金建功");
    internal_state_ = InternalState::kLocateSoccer;
    cmd_seq_        = 6000;
    cmd_stand_up(cmd_seq_++);
}

void Stage6FinalGoal::on_exit()
{
    RCLCPP_INFO(node_->get_logger(), "[Stage6] Exit — 比赛结束");
    // 已趴下，不再发送指令
}

bool Stage6FinalGoal::tick()
{
    if (is_robot_fallen()) { cmd_stop(cmd_seq_++); return false; }

    switch (internal_state_) {
    case InternalState::kLocateSoccer: {
        if (!ctx_->latest_objects) break;
        for (const auto& obj : ctx_->latest_objects->objects) {
            if (obj.object_type == DO::TYPE_SOCCER) {
                internal_state_ = InternalState::kAlignForKick;
                break;
            }
        }
        // 未找到，原地慢速扫描
        cmd_velocity(0.0f, 0.0f, 0.3f, 0.0f, 0.0f, cmd_seq_++);
        break;
    }

    case InternalState::kAlignForKick: {
        if (!ctx_->latest_objects) break;
        for (const auto& obj : ctx_->latest_objects->objects) {
            if (obj.object_type == DO::TYPE_SOCCER) {
                float vy = -obj.rel_y * 0.6f;
                vy = std::clamp(vy, -0.3f, 0.3f);
                if (std::abs(obj.rel_y) < 0.05f && obj.distance < 0.7f) {
                    internal_state_ = InternalState::kKickOut;
                } else {
                    cmd_velocity(0.2f, vy, 0.0f, 0.0f, 0.0f, cmd_seq_++);
                }
                break;
            }
        }
        break;
    }

    case InternalState::kKickOut:
        cmd_kick_forward(cmd_seq_++);
        internal_state_ = InternalState::kNavigateToFinish;
        break;

    case InternalState::kNavigateToFinish: {
        float off_x = 0.0f, off_y = 0.0f;
        if (detect_finish_circle(off_x, off_y)) {
            if (std::abs(off_x) < 0.1f && std::abs(off_y) < 0.1f)
                internal_state_ = InternalState::kAlignInCircle;
            else {
                float vy = -off_y * 0.5f;
                vy = std::clamp(vy, -0.3f, 0.3f);
                cmd_velocity(0.3f, vy, 0.0f, 0.0f, 0.0f, cmd_seq_++);
            }
        } else {
            cmd_velocity(0.3f, 0.0f, 0.0f, 0.0f, 0.0f, cmd_seq_++);
        }
        break;
    }

    case InternalState::kAlignInCircle:
        internal_state_ = InternalState::kLieDown;
        break;

    case InternalState::kLieDown:
        cmd_lie_down(cmd_seq_++);
        internal_state_ = InternalState::kConfirmFinish;
        break;

    case InternalState::kConfirmFinish:
        if (confirm_feet_in_circle()) {
            RCLCPP_INFO(node_->get_logger(), "[Stage6] 趴下成功，比赛完成！");
            return true;
        }
        break;
    }
    return false;
}

bool Stage6FinalGoal::detect_finish_circle(float& offset_x, float& offset_y) const
{
    if (!ctx_->latest_objects) return false;
    for (const auto& obj : ctx_->latest_objects->objects) {
        if (obj.object_type == DO::TYPE_FINISH_CIRCLE) {
            offset_x = obj.rel_x;
            offset_y = obj.rel_y;
            return true;
        }
    }
    return false;
}

bool Stage6FinalGoal::confirm_feet_in_circle() const
{
    // 简化：机器人已趴下且检测到终点圆圈在正前方极近处
    float ox = 0.0f, oy = 0.0f;
    if (!detect_finish_circle(ox, oy)) return false;
    return std::abs(oy) < 0.15f && ox < 0.3f;
}

}  // namespace competition_manager
