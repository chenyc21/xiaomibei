/// stage_handler_base.cpp — 赛段处理器基类辅助函数实现

#include "competition_manager/stage_handler_base.hpp"
#include "competition_msgs/msg/motion_command.hpp"

namespace competition_manager {

using MC = competition_msgs::msg::MotionCommand;
using MF = competition_msgs::msg::MotionFeedback;

// ── 停止 ─────────────────────────────────────────────────────────────────
void StageHandlerBase::cmd_stop(uint32_t seq_id)
{
    MC cmd;
    cmd.header.stamp = node_->now();
    cmd.mode   = MC::MODE_STOP;
    cmd.seq_id = seq_id;
    publish_motion(cmd);
}

// ── 起身站立 ──────────────────────────────────────────────────────────────
void StageHandlerBase::cmd_stand_up(uint32_t seq_id)
{
    MC cmd;
    cmd.header.stamp = node_->now();
    cmd.mode   = MC::MODE_STAND_UP;
    cmd.seq_id = seq_id;
    publish_motion(cmd);
}

// ── 趴下 ──────────────────────────────────────────────────────────────────
void StageHandlerBase::cmd_lie_down(uint32_t seq_id)
{
    MC cmd;
    cmd.header.stamp = node_->now();
    cmd.mode   = MC::MODE_LIE_DOWN;
    cmd.seq_id = seq_id;
    publish_motion(cmd);
}

// ── 速度闭环控制 ──────────────────────────────────────────────────────────
void StageHandlerBase::cmd_velocity(
    float vx, float vy, float vyaw,
    float body_h, float duration_s,
    uint32_t seq_id, float step_h)
{
    MC cmd;
    cmd.header.stamp = node_->now();
    cmd.mode        = MC::MODE_VELOCITY_CTRL;
    cmd.vel_x       = vx;
    cmd.vel_y       = vy;
    cmd.vel_yaw     = vyaw;
    cmd.body_height = body_h;
    cmd.step_height = step_h;
    cmd.duration_s  = duration_s;
    cmd.seq_id      = seq_id;
    publish_motion(cmd);
}

// ── 低姿走 ───────────────────────────────────────────────────────────────
void StageHandlerBase::cmd_low_walk(
    float vx, float vy, float body_h,
    float vyaw, float duration_s, uint32_t seq_id)
{
    MC cmd;
    cmd.header.stamp = node_->now();
    cmd.mode        = MC::MODE_LOW_WALK;
    cmd.vel_x       = vx;
    cmd.vel_y       = vy;
    cmd.vel_yaw     = vyaw;
    cmd.body_height = body_h;
    cmd.duration_s  = duration_s;
    cmd.seq_id      = seq_id;
    publish_motion(cmd);
}

// ── 跳下 ─────────────────────────────────────────────────────────────────
void StageHandlerBase::cmd_jump_down(uint32_t seq_id)
{
    MC cmd;
    cmd.header.stamp = node_->now();
    cmd.mode   = MC::MODE_JUMP_DOWN;
    cmd.seq_id = seq_id;
    publish_motion(cmd);
}

// ── 向前撞击 ─────────────────────────────────────────────────────────────
void StageHandlerBase::cmd_hit_forward(uint32_t seq_id)
{
    MC cmd;
    cmd.header.stamp = node_->now();
    cmd.mode   = MC::MODE_HIT_FORWARD;
    cmd.seq_id = seq_id;
    publish_motion(cmd);
}

// ── 向前踢球 ─────────────────────────────────────────────────────────────
void StageHandlerBase::cmd_kick_forward(uint32_t seq_id)
{
    MC cmd;
    cmd.header.stamp = node_->now();
    cmd.mode   = MC::MODE_KICK_FORWARD;
    cmd.seq_id = seq_id;
    publish_motion(cmd);
}

// ── 查询：机器人是否跌倒 ──────────────────────────────────────────────────
bool StageHandlerBase::is_robot_fallen() const
{
    const auto& rs = ctx_->latest_robot_state;
    return rs && rs->is_fallen;
}

// ── 查询：指定 seq_id 的运动是否已完成 ───────────────────────────────────
bool StageHandlerBase::motion_completed(uint32_t seq_id) const
{
    const auto& fb = ctx_->latest_motion_feedback;
    if (!fb) return false;
    return fb->seq_id == seq_id &&
           fb->status == MF::EXEC_COMPLETED;
}

}  // namespace competition_manager
