/// stage3_curved_track.cpp — 赛段三：曲道冲锋
/// 策略：基于 RobotState 的【绝对位姿差】判断每个 phase 完成
///   - 弯道：phase 起点记录 yaw0，运行时 |wrapToPi(yaw - yaw0)| ≥ π/2 即完成
///   - 直道：phase 起点记录 (x0,y0)，运行时 hypot(x-x0,y-y0) ≥ 1.4m 即完成
/// 数据来源：ctx_->latest_robot_state（由 competition_manager_node 从
/// /gazebo/model_states 真值或 LCM 桥的 /robot/state 注入，谁先到用谁）
/// fallback：若 RobotState 始终不可用，按 kFallback*Sec 时序兜底（保留旧行为）
/// 赛道形状：起身 → 右弯 → 中间直道 → 左弯（S形）

#include "competition_manager/stages/stage3_curved_track.hpp"
#include <cmath>
#include <algorithm>

namespace competition_manager {

static constexpr float kStageTimeoutSec    = 80.0f;  // 总赛段超时(s)
static constexpr uint32_t kCmdSeqBase      = 3000;

// ── on_enter ──────────────────────────────────────────────────────────────
void Stage3CurvedTrack::on_enter()
{
    RCLCPP_INFO(node_->get_logger(), "[Stage3] Enter — 曲道冲锋（绝对位姿闭环）");
    RCLCPP_INFO(node_->get_logger(),
        "[Stage3] 目标：弯道半径=%.1fm 圆心角=90° 直道=%.1fm",
        kCurveRadius, kStraightDist);
    if (!ctx_->latest_robot_state) {
        RCLCPP_WARN(node_->get_logger(),
            "[Stage3] ⚠ RobotState 尚未就绪，将按时序兜底 "
            "(curve=%.2fs straight=%.2fs)。检查 /gazebo/model_states 或 LCM 桥！",
            kFallbackCurveSec, kFallbackStraightSec);
    }
    internal_state_   = InternalState::kStandUp;
    cmd_seq_          = kCmdSeqBase;
    phase_origin_set_ = false;
    reset_phase();
    cmd_stand_up(cmd_seq_++);
}

// ── on_exit ───────────────────────────────────────────────────────────────
void Stage3CurvedTrack::on_exit()
{
    RCLCPP_INFO(node_->get_logger(), "[Stage3] Exit — 进入赛段四");
    cmd_stop(cmd_seq_++);
}

// ── tick ──────────────────────────────────────────────────────────────────
bool Stage3CurvedTrack::tick()
{
    // 总超时保护
    auto total_elapsed = std::chrono::duration<float>(
        std::chrono::steady_clock::now() - ctx_->stage_start_time).count();
    if (total_elapsed > kStageTimeoutSec) {
        RCLCPP_WARN(node_->get_logger(),
            "[Stage3] TIMEOUT %.1fs — 强制结束", total_elapsed);
        return true;
    }

    // 跌倒保护
    if (is_robot_fallen()) {
        cmd_stop(cmd_seq_++);
        return false;
    }

    const float t = phase_elapsed();  // 当前 phase 已用时
    const auto& rs = ctx_->latest_robot_state;
    const bool has_state = (rs != nullptr);

    // 进入运动 phase 后，第一帧（且 has_state）记录起点位姿
    auto ensure_origin = [&]() {
        if (!phase_origin_set_ && has_state) {
            phase_start_yaw_ = rs->rpy[2];
            phase_start_x_   = rs->position[0];
            phase_start_y_   = rs->position[1];
            phase_origin_set_ = true;
            RCLCPP_INFO(node_->get_logger(),
                "[Stage3] 记录 phase 起点 yaw0=%.3frad pos0=(%.3f,%.3f)",
                phase_start_yaw_, phase_start_x_, phase_start_y_);
        }
    };

    auto curve_done = [&]() -> bool {
        // 必须经过最少时长，避免 |Δyaw|≈0 的开局误判
        if (t < kPhaseMinSec) return false;
        if (has_state && phase_origin_set_) {
            float dyaw = std::fabs(wrap_to_pi(rs->rpy[2] - phase_start_yaw_));
            return dyaw >= kCurveAngle;
        }
        // fallback：按理论时序
        return t >= kFallbackCurveSec;
    };

    auto straight_done = [&]() -> bool {
        if (t < kPhaseMinSec) return false;
        if (has_state && phase_origin_set_) {
            float dx = rs->position[0] - phase_start_x_;
            float dy = rs->position[1] - phase_start_y_;
            return std::hypot(dx, dy) >= kStraightDist;
        }
        return t >= kFallbackStraightSec;
    };

    switch (internal_state_) {

    // ── 起身等待 ───────────────────────────────────────────────────────
    case InternalState::kStandUp:
        if (t >= kStandUpWaitSec) {
            RCLCPP_INFO(node_->get_logger(),
                "[Stage3] 起身完成 → 右弯（目标 Δyaw=%.2frad）", kCurveAngle);
            internal_state_ = InternalState::kCurveRight;
            reset_phase();
        }
        break;

    // ── 右弯：vyaw < 0 ──────────────────────────────────────────────────
    case InternalState::kCurveRight:
        cmd_velocity(kVxCurve, 0.0f, -kVyawCurveRight, 0.0f, 0.0f, cmd_seq_++);
        ensure_origin();
        if (cmd_seq_ % 50 == 0) {
            float dyaw = (has_state && phase_origin_set_)
                       ? std::fabs(wrap_to_pi(rs->rpy[2] - phase_start_yaw_)) : -1.0f;
            RCLCPP_INFO(node_->get_logger(),
                "[Stage3][右弯] %s yaw=%.3f Δyaw=%.3f/%.3frad t=%.2fs",
                has_state ? "GT" : "FALLBACK",
                has_state ? rs->rpy[2] : 0.0f, dyaw, kCurveAngle, t);
        }
        if (curve_done()) {
            RCLCPP_INFO(node_->get_logger(), "[Stage3] 右弯完成 → 中间直道");
            internal_state_ = InternalState::kMidStraight;
            reset_phase();
        }
        break;

    // ── 中间直道（闭环锁航向，防偏航漂移）─────────────────────────────
    case InternalState::kMidStraight: {
        ensure_origin();
        float vyaw_corr = 0.0f;
        if (has_state && phase_origin_set_) {
            // 直道应正东行进（yaw=0），以 0 为目标而非 phase_start_yaw_
            // 避免把右弯出口残余偏南航向锁住造成持续南漂
            float yaw_err = wrap_to_pi(rs->rpy[2] - 0.0f);
            vyaw_corr = -kStraightYawKp * yaw_err;
            vyaw_corr = std::clamp(vyaw_corr, -kStraightYawClamp, kStraightYawClamp);
        }
        cmd_velocity(kVxFast, 0.0f, vyaw_corr, 0.0f, 0.0f, cmd_seq_++);
        if (cmd_seq_ % 50 == 0) {
            float dist = (has_state && phase_origin_set_)
                       ? std::hypot(rs->position[0] - phase_start_x_,
                                    rs->position[1] - phase_start_y_) : -1.0f;
            RCLCPP_INFO(node_->get_logger(),
                "[Stage3][直道] %s dist=%.3f/%.3fm t=%.2fs",
                has_state ? "GT" : "FALLBACK", dist, kStraightDist, t);
        }
        if (straight_done()) {
            RCLCPP_INFO(node_->get_logger(), "[Stage3] 直道完成 → 左弯");
            internal_state_ = InternalState::kCurveLeft;
            reset_phase();
        }
        break;
    }

    // ── 左弯：vyaw > 0 ──────────────────────────────────────────────────
    case InternalState::kCurveLeft:
        cmd_velocity(kVxCurve, 0.0f, kVyawCurveLeft, 0.0f, 0.0f, cmd_seq_++);
        ensure_origin();
        if (cmd_seq_ % 50 == 0) {
            float dyaw = (has_state && phase_origin_set_)
                       ? std::fabs(wrap_to_pi(rs->rpy[2] - phase_start_yaw_)) : -1.0f;
            RCLCPP_INFO(node_->get_logger(),
                "[Stage3][左弯] %s yaw=%.3f Δyaw=%.3f/%.3frad t=%.2fs",
                has_state ? "GT" : "FALLBACK",
                has_state ? rs->rpy[2] : 0.0f, dyaw, kCurveAngle, t);
        }
        if (curve_done()) {
            if (has_state) {
                RCLCPP_INFO(node_->get_logger(),
                    "[Stage3] 左弯完成 → 二次校准 pos=(%.3f,%.3f) yaw=%.3f"
                    "  目标=(3.10,6.60) yaw≈1.571",
                    rs->position[0], rs->position[1], rs->rpy[2]);
            } else {
                RCLCPP_INFO(node_->get_logger(),
                    "[Stage3] 左弯完成 → 二次校准 (无 GT，按超时兜底)");
            }
            internal_state_ = InternalState::kCalibrate;
            reset_phase();
        }
        break;

    // ── 二次校准：闭环把位姿收敛到 (3.10, 6.60) yaw=+π/2 ───────────────
    case InternalState::kCalibrate: {
        if (!has_state) {
            // 无真值则兜底：直接结束
            RCLCPP_WARN(node_->get_logger(),
                "[Stage3] kCalibrate 无 RobotState，跳过校准直接结束");
            cmd_velocity(0, 0, 0, 0, 0, cmd_seq_++);
            return true;
        }
        float ex_w = kCalibTargetX - rs->position[0];   // world Δx
        float ey_w = kCalibTargetY - rs->position[1];   // world Δy
        float yaw  = rs->rpy[2];
        float yaw_err = wrap_to_pi(yaw - kCalibTargetYaw);
        // 世界→机体： body_vx =  ex_w*cos(yaw) + ey_w*sin(yaw)
        //             body_vy = -ex_w*sin(yaw) + ey_w*cos(yaw)
        float cs = std::cos(yaw), sn = std::sin(yaw);
        float bvx =  ex_w * cs + ey_w * sn;
        float bvy = -ex_w * sn + ey_w * cs;
        float vx   = std::clamp(kCalibKpPos * bvx, -kCalibVxMax, kCalibVxMax);
        float vy   = std::clamp(kCalibKpPos * bvy, -kCalibVyMax, kCalibVyMax);
        float vyaw = std::clamp(-kCalibKpYaw * yaw_err, -kCalibVyawMax, kCalibVyawMax);
        cmd_velocity(vx, vy, vyaw, 0.0f, 0.0f, cmd_seq_++);

        float dist = std::hypot(ex_w, ey_w);
        if (cmd_seq_ % 20 == 0) {
            RCLCPP_INFO(node_->get_logger(),
                "[Stage3][校准] pos=(%.3f,%.3f) dist=%.3f yaw_err=%.3f "
                "→ body_v=(%.2f,%.2f) vyaw=%.2f t=%.2fs",
                rs->position[0], rs->position[1], dist, yaw_err, vx, vy, vyaw, t);
        }
        if ((dist <= kCalibPosTol && std::fabs(yaw_err) <= kCalibYawTol)
            || t >= kCalibMaxSec) {
            RCLCPP_INFO(node_->get_logger(),
                "[Stage3] 校准完成 终点=(%.3f,%.3f) yaw=%.3f dist=%.3f "
                "→ 赛段三结束（目标=(3.10,6.60) yaw≈1.571）",
                rs->position[0], rs->position[1], yaw, dist);
            cmd_velocity(0, 0, 0, 0, 0, cmd_seq_++);
            return true;
        }
        break;
    }

    } // switch

    return false;
}

}  // namespace competition_manager
