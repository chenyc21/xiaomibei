/// stage4_deep_tunnel.cpp — 赛段四：深隧寻珍（仿真简化版，沿走廊巡线）
#include "competition_manager/stages/stage4_deep_tunnel.hpp"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <string>

namespace competition_manager {

using DO       = competition_msgs::msg::DetectedObject;
using VoiceCmd = competition_msgs::msg::VoiceCommand;

// ── 时长 ───────────────────────────────────────────────────────
static constexpr float kStandUpDur     = 3.0f;
static constexpr float kStrafeMaxDur   = 60.0f;  // 横移很慢，给足时间
static constexpr float kWalkMaxDur     = 50.0f;
static constexpr float kLowWalkDur     = 20.0f;  // 上限保护：低姿下 MPC 慢，预留充足时间
static constexpr float kBarPassY       = 0.35f;  // 在杆 y 后方 0.35m 认为已穿越
static constexpr float kBlockBypassDur = 4.5f;
static constexpr float kHitDur         = 2.5f;
static constexpr float kStageTimeoutSec = 600.0f;

// ── 速度 ───────────────────────────────────────────────────────
static constexpr float kForwardSpeed = 0.60f;   // 直行速度（翻倍）
static constexpr float kBackSpeed    = 0.20f;   // 后退速度
static constexpr float kSlowSpeed    = 0.18f;
static constexpr float kStrafeSpeed  = 0.90f;   // 横移速度（×3，实际输出仍受控制器限幅）
static constexpr float kLowBodyH     = 0.18f;  // 折中：身体顶高≈0.26m 安全穿 0.40m 横梁，腿仍能行走
static constexpr float kBarCenterX     = -0.13f; // 门洞中心 (柱 x 为 -0.63 / +0.37 的中点)
static constexpr float kBarAlignTolX   = 0.05f;  // 站立时对中的容差
static constexpr float kBarAlignSpeed  = 0.30f;  // 站立横移速度
static constexpr float kBarAlignMaxDur = 20.0f;  // 对中保护上限
static constexpr float kBarLateralKp   = 0.0f;   // 低姿下不再做横向修正（无效且耗时）
static constexpr float kBarLateralMax  = 0.0f;

// === 新流程参数 ===
static constexpr float kCorridor1X      = -0.10f;  // 可乐的 x
static constexpr float kCorridor2X      =  1.00f;  // 悬球所在第二走廊中轴线
static constexpr float kColaHitY        = 11.40f;  // 推过可乐 0.30m
static constexpr float kBackOffY        = 10.30f;  // 调头前退到的 y
static constexpr float kExitSouthY      =  7.30f;  // 完全出走廊后的 y (6.6<y<7.6 中段)
static constexpr float kExitSouthXTol   =  0.10f;  // 出走廊时要求 |x - kCorridor1X| < 此值
static constexpr float kBallHitY        = 11.40f;  // 推过悬球
static constexpr float kAlignTolXNorm   = 0.06f;
static constexpr float kAlignSpeedNorm  = 0.30f;
static constexpr float kTurnSpeed       = 0.6f;    // rad/s
static constexpr float kYawTol          = 0.10f;   // rad
static constexpr float kCubeDetourX     = 1.45f;   // 方块东侧绕行 x (方块 0.78~1.28)
static constexpr float kCubeClearY      =  9.30f;  // 绕过后达到的 y
static constexpr float kPhaseTimeoutLong = 30.0f;

// ── 导航闭环阈值（世界坐标） ───────────────────────────────────
// 出生 (3.1, 6.6) yaw=+π/2 朝 +Y。全程保持朝 +Y，不做 90° 转向。
// 机体 +X(前) = 世界 +Y;  机体 +Y(左) = 世界 −X
//   故：横移左 (vy>0) 让 world_x 变小，前进 (vx>0) 让 world_y 变大。
// 路径：(3.1,6.6) → 左横移到 x≈−0.3 → 前进到 y≈11.0 →
//       右横移到 x≈2.7 (巡 cola/ball/football) → 后退到 y≈7.6 (回桥)
static constexpr float kBypassXTarget    = -0.30f;  // 桥西侧
static constexpr float kBypassEntryY     =  7.10f;  // 起身后先前推到此 y，再开始横移（避免踩黄线）
static constexpr float kCorridorYTarget  = 11.00f;  // 走廊中线
static constexpr float kPatrolXEnd       =  2.70f;  // 巡线终点
static constexpr float kReturnYTarget    =  7.80f;  // 桥北口
static constexpr float kBridgePos[2]     = {3.13f, 7.60f};

// ── 触发距离 / 检出距离 ────────────────────────────────────────
static constexpr float kTrigObstacle = 1.0f;
static constexpr float kTrigTarget   = 0.55f;
static constexpr float kBeamReach    = 0.55f;
static constexpr uint32_t kCmdSeqBase = 4000;

// ────────────────────────────────────────────────────────────────
void Stage4DeepTunnel::on_enter()
{
    const char* c3_env = std::getenv("STAGE4_C3_ONLY");
    c3_only_mode_ = (c3_env && std::string(c3_env) != "0" && std::string(c3_env) != "");
    const char* bridge_env = std::getenv("STAGE4_BRIDGE_ONLY");
    bridge_only_mode_ = (bridge_env && std::string(bridge_env) != "0" && std::string(bridge_env) != "");
    RCLCPP_INFO(node_->get_logger(),
        "[Stage4] Enter \u2014 \u6df1\u96a7\u5bfb\u73cd (corridor mode)%s",
        c3_only_mode_ ? " [\u8d70\u5ec33 \u5355\u72ec\u6d4b\u8bd5]" : "");
    phase_     = Phase::kStandUp;
    pending_type_ = 0;
    announced_ = false;
    done_      = {};
    cmd_seq_   = kCmdSeqBase;
    bar_align_x_           = kBarCenterX;
    phase_after_bar_valid_ = false;
    phase_enter_time_ = std::chrono::steady_clock::now();
    cmd_stand_up(cmd_seq_++);
}

void Stage4DeepTunnel::on_exit()
{
    RCLCPP_INFO(node_->get_logger(), "[Stage4] Exit — 进入赛段五");
    cmd_stop(cmd_seq_++);
}

void Stage4DeepTunnel::enter_phase(Phase p, uint8_t pending)
{
    phase_ = p;
    pending_type_ = pending;
    phase_enter_time_ = std::chrono::steady_clock::now();
    announced_ = false;
    yaw_aligned_since_ = -1.0f;
}

float Stage4DeepTunnel::phase_elapsed() const
{
    return std::chrono::duration<float>(
        std::chrono::steady_clock::now() - phase_enter_time_).count();
}

bool Stage4DeepTunnel::already_done(uint8_t type) const
{
    switch (type) {
    case DO::TYPE_COLA_BOTTLE:    return done_.cola;
    case DO::TYPE_ORANGE_BALL:    return done_.orange;
    case DO::TYPE_SOCCER:         return done_.soccer;
    case DO::TYPE_HEIGHT_BAR:
        return done_.bar1 && done_.bar2;  // 两根都过完才算完
    case DO::TYPE_BLOCK_OBSTACLE: return done_.block;
    default: return false;
    }
}

void Stage4DeepTunnel::mark_done(uint8_t type)
{
    switch (type) {
    case DO::TYPE_COLA_BOTTLE:    done_.cola = true; break;
    case DO::TYPE_ORANGE_BALL:    done_.orange = true; break;
    case DO::TYPE_SOCCER:         done_.soccer = true; break;
    case DO::TYPE_HEIGHT_BAR:
        if (!done_.bar1) done_.bar1 = true;
        else             done_.bar2 = true;
        break;
    case DO::TYPE_BLOCK_OBSTACLE: done_.block = true; break;
    }
}

const DO* Stage4DeepTunnel::find_object(uint8_t type) const
{
    if (!ctx_->latest_objects) return nullptr;
    const DO* best = nullptr;
    for (const auto& obj : ctx_->latest_objects->objects) {
        if (obj.object_type != type) continue;
        if (obj.object_status == DO::STATUS_INTERACTED) continue;
        if (!best || obj.distance < best->distance) best = &obj;
    }
    return best;
}

uint8_t Stage4DeepTunnel::scan_next_action(float trig_dist) const
{
    if (!ctx_->latest_objects) return 0;
    // 优先顺序：障碍 > 目标（避免撞上障碍）
    const uint8_t order[] = {
        DO::TYPE_HEIGHT_BAR, DO::TYPE_BLOCK_OBSTACLE,
        DO::TYPE_COLA_BOTTLE, DO::TYPE_ORANGE_BALL, DO::TYPE_SOCCER,
    };
    for (uint8_t t : order) {
        const DO* o = find_object(t);
        if (!o) continue;
        // 限高杆/障碍：在 trig_dist 内即响应
        // 目标：在 kTrigTarget 内才撞击（更近）
        bool is_target = (t == DO::TYPE_COLA_BOTTLE ||
                          t == DO::TYPE_ORANGE_BALL ||
                          t == DO::TYPE_SOCCER);
        float th = is_target ? kTrigTarget : trig_dist;
        if (o->distance > th) continue;
        if (already_done(t)) continue;
        return t;
    }
    return 0;
}

// ────────────────────────────────────────────────────────────────
// 取世界位姿（来自 GT 注入的 RobotState）
struct WorldPose { float x, y, z, yaw; bool ok; };
static WorldPose get_world_pose(const FSMContext* ctx) {
    WorldPose p{0, 0, 0, 0, false};
    const auto& rs = ctx->latest_robot_state;
    if (!rs) return p;
    p.x = rs->position[0]; p.y = rs->position[1]; p.z = rs->position[2]; p.yaw = rs->rpy[2];
    p.ok = true;
    return p;
}
static float wrap_pi(float a) {
    while (a >  M_PI) a -= 2 * M_PI;
    while (a < -M_PI) a += 2 * M_PI;
    return a;
}

// ────────────────────────────────────────────────────────────────
bool Stage4DeepTunnel::tick()
{
    auto stage_elapsed = std::chrono::duration<float>(
        std::chrono::steady_clock::now() - ctx_->stage_start_time).count();
    if (stage_elapsed > kStageTimeoutSec) {
        RCLCPP_WARN(node_->get_logger(),
            "[Stage4] TIMEOUT %.1fs — 强制结束", stage_elapsed);
        return true;
    }
    if (is_robot_fallen()) {
        cmd_stand_up(cmd_seq_++);
        return false;
    }

    const float t = phase_elapsed();
    const auto wp = get_world_pose(ctx_);

    switch (phase_) {

    case Phase::kStandUp:
        if (t >= kStandUpDur) {
            if (bridge_only_mode_) {
                RCLCPP_INFO(node_->get_logger(),
                    "[Stage4][BRIDGE-ONLY] 起身完成 → 直接跳 kAdvanceToBridgeTop (仅测试上桥)");
                enter_phase(Phase::kAdvanceToBridgeTop);
                cmd_velocity(0, 0, 0, 0, 0, cmd_seq_++);
            } else if (c3_only_mode_) {
                RCLCPP_INFO(node_->get_logger(),
                    "[Stage4][C3-ONLY] 起身完成 → 直接跳 kAdvanceC3 (跳过走廊1/2)");
                enter_phase(Phase::kAdvanceC3);
                cmd_velocity(kForwardSpeed, 0, 0, 0, 0, cmd_seq_++);
            } else {
                RCLCPP_INFO(node_->get_logger(),
                    "[Stage4] 起身完成 → 先直行到 y≥%.2f 避免踩黄线",
                    kBypassEntryY);
                enter_phase(Phase::kAdvanceToBypassY);
                cmd_velocity(kForwardSpeed, 0, 0, 0, 0, cmd_seq_++);
            }
        }
        break;

    // 起身后先前进一小段到 (3.10, 7.10) 一带，再开始向西横移。
    // 默认出生点 (3.10, 6.60)，起步即开始横移会挪到桥西侧的黄线。
    case Phase::kAdvanceToBypassY: {
        cmd_velocity(kForwardSpeed, 0, 0, 0, 0, cmd_seq_++);
        if ((wp.ok && wp.y >= kBypassEntryY) || t >= 10.0f) {
            RCLCPP_INFO(node_->get_logger(),
                "[Stage4][WP0] 到达横移起点 @ world=(%.2f,%.2f) → 左横移绕开独木桥 (目标 x≤%.2f)",
                wp.x, wp.y, kBypassXTarget);
            enter_phase(Phase::kStrafeWest);
            cmd_velocity(0, kStrafeSpeed, 0, 0, 0, cmd_seq_++);
        }
        break;
    }

    // 狗朝 +Y; 机体 +Y(左) = 世界 −X，故 vy>0 → world_x 减小
    case Phase::kStrafeWest: {
        // 接近 waypoint 减速（"减速转向"思想：剩余 < 0.4m 时按比例放缓）
        float remain = wp.ok ? std::max(0.0f, wp.x - kBypassXTarget) : 1.0f;
        float k = std::clamp(remain / 0.4f, 0.4f, 1.0f);
        cmd_velocity(0, kStrafeSpeed * k, 0, 0, 0, cmd_seq_++);
        if ((wp.ok && wp.x <= kBypassXTarget) || t >= kStrafeMaxDur) {
            RCLCPP_INFO(node_->get_logger(),
                "[Stage4][WP1] 已绕开桥 @ world=(%.2f,%.2f) → 前进进入走廊",
                wp.x, wp.y);
            enter_phase(Phase::kAdvanceNorth);
            cmd_velocity(kForwardSpeed, 0, 0, 0, 0, cmd_seq_++);
        }
        break;
    }

    // 机体 +X(前) = 世界 +Y, 前进让 world_y 增大
    case Phase::kAdvanceNorth: {
        // 沿途如遇限高杆触发低姿
        uint8_t next = scan_next_action(kTrigObstacle);
        if (next == DO::TYPE_HEIGHT_BAR && !already_done(next)) {
            const DO* bar = find_object(next);
            announce(VoiceCmd::ANNOUNCE_HEIGHT_BAR);
            bar_align_x_           = kBarCenterX;
            phase_after_bar_       = Phase::kAlignCola;
            phase_after_bar_valid_ = true;
            RCLCPP_INFO(node_->get_logger(),
                "[Stage4] 前进途中识别限高杆 (d=%.2f)，先站立对中 (x→%.2f)", bar->distance, bar_align_x_);
            enter_phase(Phase::kAlignBar, next);
            cmd_velocity(0, 0, 0, 0, 0, cmd_seq_++);
            break;
        }
        float remain = wp.ok ? std::max(0.0f, kCorridorYTarget - wp.y) : 1.0f;
        float k = std::clamp(remain / 0.5f, 0.4f, 1.0f);
        cmd_velocity(kForwardSpeed * k, 0, 0, 0, 0, cmd_seq_++);
        if ((wp.ok && wp.y >= kCorridorYTarget) || t >= kWalkMaxDur) {
            RCLCPP_INFO(node_->get_logger(),
                "[Stage4][WP2] 已进入走廊 @ world=(%.2f,%.2f) → 右横移巡线寻珍",
                wp.x, wp.y);
            enter_phase(Phase::kStrafeEast);
            cmd_velocity(0, -kStrafeSpeed, 0, 0, 0, cmd_seq_++);
        }
        break;
    }

    // vy<0 = 右移; 朝北时机体右 = 世界 +X
    case Phase::kStrafeEast: {
        // 走完巡线终点 → 准备返回桥
        if ((wp.ok && wp.x >= kPatrolXEnd) || t >= kStrafeMaxDur) {
            RCLCPP_INFO(node_->get_logger(),
                "[Stage4][WP3] 巡线完毕 (%d/3 目标 @ x=%.2f) → 后退回独木桥",
                done_.target_count(), wp.x);
            enter_phase(Phase::kRetreatSouth);
            cmd_velocity(-kBackSpeed, 0, 0, 0, 0, cmd_seq_++);
            break;
        }
        // 沿途处理限高杆/挡块/目标
        uint8_t next = scan_next_action(kTrigObstacle);
        if (next == DO::TYPE_HEIGHT_BAR) {
            const DO* bar = find_object(next);
            announce(VoiceCmd::ANNOUNCE_HEIGHT_BAR);
            bar_align_x_           = kBarCenterX;
            phase_after_bar_       = Phase::kStrafeEast;
            phase_after_bar_valid_ = true;
            RCLCPP_INFO(node_->get_logger(),
                "[Stage4] 巡线识别限高杆 (d=%.2f)，先站立对中", bar->distance);
            enter_phase(Phase::kAlignBar, next);
            cmd_velocity(0, 0, 0, 0, 0, cmd_seq_++);
            break;
        }
        if (next == DO::TYPE_BLOCK_OBSTACLE) {
            const DO* blk = find_object(next);
            announce(VoiceCmd::ANNOUNCE_IMPASSABLE);
            RCLCPP_INFO(node_->get_logger(),
                "[Stage4] 巡线识别挡块 (d=%.2f)，绕行", blk->distance);
            enter_phase(Phase::kHandleBlock, next);
            break;
        }
        if (next == DO::TYPE_COLA_BOTTLE || next == DO::TYPE_ORANGE_BALL ||
            next == DO::TYPE_SOCCER)
        {
            switch (next) {
            case DO::TYPE_COLA_BOTTLE:
                announce(VoiceCmd::ANNOUNCE_COLA);    break;
            case DO::TYPE_ORANGE_BALL:
                announce(VoiceCmd::ANNOUNCE_ORANGE_BALL); break;
            case DO::TYPE_SOCCER:
                announce(VoiceCmd::ANNOUNCE_SOCCER);  break;
            }
            mark_done(next);
            RCLCPP_INFO(node_->get_logger(),
                "[Stage4] 识别目标 type=%u (累计 %d/3)",
                next, done_.target_count());
            // 不触发跳/踢动作：横移姿态下撞击方向不对，仅播报识别
            // 继续右横移（接近巡线终点减速）
            float remain = wp.ok ? std::max(0.0f, kPatrolXEnd - wp.x) : 1.0f;
            float k = std::clamp(remain / 0.4f, 0.4f, 1.0f);
            cmd_velocity(0, -kStrafeSpeed * k, 0, 0, 0, cmd_seq_++);
            break;
        }
        // 默认右横移（接近 waypoint 减速）
        {
            float remain = wp.ok ? std::max(0.0f, kPatrolXEnd - wp.x) : 1.0f;
            float k = std::clamp(remain / 0.4f, 0.4f, 1.0f);
            cmd_velocity(0, -kStrafeSpeed * k, 0, 0, 0, cmd_seq_++);
        }
        break;
    }

    case Phase::kAlignBar: {
        // 站立姿态：先对正航向到 ±π/2（与走廊轴线平行），再做 x 对中；完全对齐后才蹲下直走。
        // 用比例控制 + 连续达标 0.6s 才放行，避免航向状态由上个阶段惯性带偏。
        float yaw_target = (wp.ok && wp.yaw < 0.0f)
                           ? -static_cast<float>(M_PI) / 2.0f
                           :  static_cast<float>(M_PI) / 2.0f;
        float yaw_err = wp.ok ? wrap_pi(yaw_target - wp.yaw) : 0.0f;
        const float kYawAlignTol    = 0.04f;   // ~2.3°
        const float kYawSettleHold  = 0.6f;    // 必须连续达标这么久
        const float kYawAlignMaxRate = 0.30f;  // 对中阶段用较低的转速
        const float kYawAlignKp      = 1.2f;

        // 1) 航向调整 + 需要连续达标
        if (wp.ok && std::fabs(yaw_err) > kYawAlignTol) {
            yaw_aligned_since_ = -1.0f;
            float vyaw = std::clamp(yaw_err * kYawAlignKp, -kYawAlignMaxRate, kYawAlignMaxRate);
            cmd_velocity(0, 0, vyaw, 0, 0, cmd_seq_++);
            if ((cmd_seq_ % 25) == 0) {
                RCLCPP_INFO(node_->get_logger(),
                    "[Stage4][ALIGN] t=%.1fs yaw=%.2f→%.2f err=%+.3f vyaw=%+.2f",
                    t, wp.yaw, yaw_target, yaw_err, vyaw);
            }
            if (t >= kBarAlignMaxDur) {
                RCLCPP_WARN(node_->get_logger(),
                    "[Stage4][ALIGN] 航向对正超时 err=%.3f，强制继续", yaw_err);
                yaw_lock_target_ = yaw_target;
                yaw_lock_valid_  = true;
                enter_phase(Phase::kHandleHeightBar, pending_type_);
                cmd_low_walk(kSlowSpeed, 0.0f, kLowBodyH, 0.0f, 0.0f, cmd_seq_++);
            }
            break;
        }
        // 达标中：计时 + 发送开套制动，促进稳定
        if (yaw_aligned_since_ < 0.0f) yaw_aligned_since_ = t;
        if (t - yaw_aligned_since_ < kYawSettleHold) {
            cmd_velocity(0, 0, 0, 0, 0, cmd_seq_++);
            if ((cmd_seq_ % 25) == 0) {
                RCLCPP_INFO(node_->get_logger(),
                    "[Stage4][ALIGN] yaw 达标稳定中 %.2fs/%.2fs (yaw=%.2f err=%+.3f)",
                    t - yaw_aligned_since_, kYawSettleHold, wp.yaw, yaw_err);
            }
            break;
        }

        // 2) 航向已稳定对正，做横移对中
        float x_err = wp.ok ? (wp.x - bar_align_x_) : 0.0f;
        bool aligned = wp.ok && std::fabs(x_err) < kBarAlignTolX;
        if (aligned || t >= kBarAlignMaxDur) {
            RCLCPP_INFO(node_->get_logger(),
                "[Stage4][ALIGN] 对中完成 yaw=%.2f x_err=%+.3f t=%.1f → 蹲下直走",
                wp.yaw, x_err, t);
            yaw_lock_target_ = yaw_target;
            yaw_lock_valid_  = true;
            enter_phase(Phase::kHandleHeightBar, pending_type_);
            cmd_low_walk(kSlowSpeed, 0.0f, kLowBodyH, 0.0f, 0.0f, cmd_seq_++);
            break;
        }
        // 机体 +Y(左) 方向：北向(yaw=+π/2)时左=世界-X；南向(yaw=-π/2)时左=世界+X
        // 故 vy 符号取决于朝向：err>0 (狗偏世界+X) 需向 -X → 北向 vy>0 / 南向 vy<0
        bool facing_north = wp.ok && wp.yaw > 0.0f;
        float dir = facing_north ? +1.0f : -1.0f;
        float vy = (x_err > 0 ? +1.0f : -1.0f) * dir * kBarAlignSpeed;
        cmd_velocity(0, vy, 0, 0, 0, cmd_seq_++);
        if ((cmd_seq_ % 50) == 0) {
            RCLCPP_INFO(node_->get_logger(),
                "[Stage4][ALIGN] t=%.1fs yaw=%.2f world_x=%.2f x_err=%+.3f vy=%+.2f",
                t, wp.yaw, wp.x, x_err, vy);
        }
        break;
    }

    case Phase::kHandleHeightBar: {
        // 已对中：低姿下只直走；yaw 锁定到对中阶段记录的目标航向，防止漂航带偏。
        float vyaw_corr = 0.0f;
        if (yaw_lock_valid_ && wp.ok) {
            float yerr = wrap_pi(yaw_lock_target_ - wp.yaw);
            vyaw_corr  = std::clamp(yerr * 1.0f, -0.20f, 0.20f);
        }
        cmd_low_walk(kSlowSpeed, 0.0f, kLowBodyH, vyaw_corr, 0.0f, cmd_seq_++);
        if ((cmd_seq_ % 50) == 0) {
            RCLCPP_INFO(node_->get_logger(),
                "[Stage4][LOW_WALK] t=%.1fs body_h=%.2f vx=%.2f vyaw=%+.2f @ world=(%.2f,%.2f) yaw=%.2f err_x=%+.2f",
                t, kLowBodyH, kSlowSpeed, vyaw_corr, wp.x, wp.y, wp.yaw, wp.ok ? (wp.x - bar_align_x_) : 0.0f);
        }
        // 出口条件：感知里所有 HEIGHT_BAR 都已在身后 (rel_x < -0.20m) 或超时。
        // 关键：在杆身下方时绝不能放手——否则 MPC 会把身体抬回 0.32m 顶到横梁。
        {
            bool any_ahead = false;
            const DO* nearest = nullptr;
            if (ctx_->latest_objects) {
                for (const auto& obj : ctx_->latest_objects->objects) {
                    if (obj.object_type != DO::TYPE_HEIGHT_BAR) continue;
                    if (obj.rel_x > -0.20f && obj.rel_x < 2.5f) {  // 还没完全穿过
                        any_ahead = true;
                        if (!nearest || obj.distance < nearest->distance) nearest = &obj;
                    }
                }
            }
            bool passed = !any_ahead && t >= 2.0f;
            if (passed || t >= kLowWalkDur) {
                mark_done(DO::TYPE_HEIGHT_BAR);
                bool facing_south = wp.ok && wp.yaw < 0.0f;
                Phase nxt;
                if (phase_after_bar_valid_) {
                    nxt = phase_after_bar_;
                    phase_after_bar_valid_ = false;
                } else {
                    nxt = facing_south ? Phase::kExitCorridor1 : Phase::kAlignCola;
                }
                RCLCPP_INFO(node_->get_logger(),
                    "[Stage4] 限高杆已穿越 (passed=%d, t=%.1f, dir=%s) @ world=(%.2f,%.2f)",
                    (int)passed, t, facing_south ? "南" : "北", wp.x, wp.y);
                enter_phase(nxt);
                if (facing_south)
                    cmd_velocity(kForwardSpeed, 0, 0, 0, 0, cmd_seq_++);
                else
                    cmd_velocity(0, 0, 0, 0, 0, cmd_seq_++);
            }
        }
        break;
    }

    // ============== 新流程 ==============
    case Phase::kAlignCola: {
        float err = wp.ok ? (wp.x - kCorridor1X) : 0.0f;
        if (wp.ok && std::fabs(err) < kAlignTolXNorm) {
            RCLCPP_INFO(node_->get_logger(),
                "[Stage4][C1] 对准可乐完成 err=%.3f → 撞瓶", err);
            enter_phase(Phase::kHitCola);
            cmd_velocity(kSlowSpeed, 0, 0, 0, 0, cmd_seq_++);
            break;
        }
        float vy = (err > 0 ? +1.0f : -1.0f) * kAlignSpeedNorm;
        cmd_velocity(0, vy, 0, 0, 0, cmd_seq_++);
        if (t > kPhaseTimeoutLong) { enter_phase(Phase::kHitCola); cmd_velocity(kSlowSpeed, 0, 0, 0, 0, cmd_seq_++); }
        break;
    }

    case Phase::kHitCola: {
        cmd_velocity(kSlowSpeed, 0, 0, 0, 0, cmd_seq_++);
        if ((wp.ok && wp.y >= kColaHitY) || t >= kWalkMaxDur) {
            mark_done(DO::TYPE_COLA_BOTTLE);
            announce(VoiceCmd::ANNOUNCE_COLA);
            RCLCPP_INFO(node_->get_logger(),
                "[Stage4][C1] 可乐撞击完成 @ y=%.2f → 后退调头", wp.y);
            enter_phase(Phase::kBackOffCola);
            cmd_velocity(-kBackSpeed, 0, 0, 0, 0, cmd_seq_++);
        }
        break;
    }

    case Phase::kBackOffCola: {
        cmd_velocity(-kBackSpeed, 0, 0, 0, 0, cmd_seq_++);
        if ((wp.ok && wp.y <= kBackOffY) || t >= 8.0f) {
            RCLCPP_INFO(node_->get_logger(),
                "[Stage4][C1] 后退完成 @ y=%.2f → 180°调头 (target yaw=-π/2, 带二次校准)", wp.y);
            turn_target_yaw_  = -static_cast<float>(M_PI) / 2.0f;
            phase_after_turn_ = Phase::kExitCorridor1;
            enter_phase(Phase::kTurnTo);
            cmd_velocity(0, 0, 0, 0, 0, cmd_seq_++);
        }
        break;
    }

    case Phase::kTurnAround1: {
        // 兼容保留：实际不再使用，以防意外调用 → 依然走 kTurnTo。
        turn_target_yaw_  = -static_cast<float>(M_PI) / 2.0f;
        phase_after_turn_ = Phase::kExitCorridor1;
        enter_phase(Phase::kTurnTo);
        break;
    }

    case Phase::kExitCorridor1: {
        // 南向走出走廊；途中会再次遇到限高杆 (y≈9.6)。
        // 撞可乐后位姿可能漂移，必须先做一次站立对中(位置+航向)再低姿穿过。
        const DO* bar_ahead = nullptr;
        if (ctx_->latest_objects) {
            for (const auto& obj : ctx_->latest_objects->objects) {
                if (obj.object_type != DO::TYPE_HEIGHT_BAR) continue;
                // rel_x > 0 表示在身前；distance 用作触发阈值
                if (obj.rel_x > 0.10f && obj.distance < kTrigObstacle) {
                    if (!bar_ahead || obj.distance < bar_ahead->distance)
                        bar_ahead = &obj;
                }
            }
        }
        if (bar_ahead) {
            announce(VoiceCmd::ANNOUNCE_HEIGHT_BAR);
            bar_align_x_           = kBarCenterX;
            phase_after_bar_       = Phase::kExitCorridor1;
            phase_after_bar_valid_ = true;
            RCLCPP_INFO(node_->get_logger(),
                "[Stage4][EXIT] 南向识别限高杆 (d=%.2f, rel_x=%.2f)，站立对中(位置+航向)再低姿穿过",
                bar_ahead->distance, bar_ahead->rel_x);
            enter_phase(Phase::kAlignBar, DO::TYPE_HEIGHT_BAR);
            cmd_velocity(0, 0, 0, 0, 0, cmd_seq_++);
            break;
        }
        // 面向 -Y，前进使 y 减小
        cmd_velocity(kForwardSpeed, 0, 0, 0, 0, cmd_seq_++);
        bool y_in_band = wp.ok && wp.y <= kExitSouthY && wp.y >= 6.6f;
        bool x_on_axis = wp.ok && std::fabs(wp.x - kCorridor1X) < kExitSouthXTol;
        if ((y_in_band && x_on_axis) || t >= kWalkMaxDur) {
            RCLCPP_INFO(node_->get_logger(),
                "[Stage4][EXIT] 完全走出走廊 @ world=(%.2f,%.2f) → 横移到走廊2 (target x=%.2f)", wp.x, wp.y, kCorridor2X);
            enter_phase(Phase::kStrafeToC2);
            cmd_velocity(0, kStrafeSpeed, 0, 0, 0, cmd_seq_++);
        }
        break;
    }

    case Phase::kStrafeToC2: {
        // 面向 -Y 时：body +Y(左) = 世界 +X，所以 vy>0 让 world_x 增大
        float err = wp.ok ? (kCorridor2X - wp.x) : 0.0f;
        if (wp.ok && std::fabs(err) < kAlignTolXNorm) {
            RCLCPP_INFO(node_->get_logger(),
                "[Stage4][C2] 到达走廊2中线 @ x=%.2f → 180°调头回 +π/2 (带二次校准)", wp.x);
            turn_target_yaw_  = static_cast<float>(M_PI) / 2.0f;
            phase_after_turn_ = Phase::kAdvanceC2;
            enter_phase(Phase::kTurnTo);
            cmd_velocity(0, 0, 0, 0, 0, cmd_seq_++);
            break;
        }
        float vy = (err > 0 ? +1.0f : -1.0f) * kStrafeSpeed;
        cmd_velocity(0, vy, 0, 0, 0, cmd_seq_++);
        if (t > kStrafeMaxDur) {
            turn_target_yaw_  = static_cast<float>(M_PI) / 2.0f;
            phase_after_turn_ = Phase::kAdvanceC2;
            enter_phase(Phase::kTurnTo);
        }
        break;
    }

    case Phase::kTurnAround2: {
        // 兼容保留：走 kTurnTo
        turn_target_yaw_  = static_cast<float>(M_PI) / 2.0f;
        phase_after_turn_ = Phase::kAdvanceC2;
        enter_phase(Phase::kTurnTo);
        break;
    }

    // 通用转到指定 yaw（二次校准）——完后跳 phase_after_turn_
    case Phase::kTurnTo: {
        float err = wp.ok ? wrap_pi(turn_target_yaw_ - wp.yaw) : 0.0f;
        const float kTurnToTol    = 0.04f;
        const float kTurnSettle   = 0.3f;   // 偏航达标后静止稳定时间（0.6→0.3s，上桥前减半停顿）
        const float kTurnToMaxRate = 0.40f;
        const float kTurnToKp      = 1.2f;
        const float kTurnToCoarse  = 0.50f;  // |err| > 此值时用恒速 kTurnSpeed
        if (wp.ok && std::fabs(err) > kTurnToTol) {
            yaw_aligned_since_ = -1.0f;
            float vyaw;
            if (std::fabs(err) > kTurnToCoarse)
                vyaw = (err > 0 ? +1.0f : -1.0f) * kTurnSpeed;
            else
                vyaw = std::clamp(err * kTurnToKp, -kTurnToMaxRate, kTurnToMaxRate);
            cmd_velocity(0, 0, vyaw, 0, 0, cmd_seq_++);
            if ((cmd_seq_ % 25) == 0) {
                RCLCPP_INFO(node_->get_logger(),
                    "[Stage4][TURN] t=%.1fs yaw=%.2f→%.2f err=%+.3f vyaw=%+.2f",
                    t, wp.yaw, turn_target_yaw_, err, vyaw);
            }
            if (t > 15.0f) {
                RCLCPP_WARN(node_->get_logger(),
                    "[Stage4][TURN] 超时 err=%.3f，强制跳转", err);
                enter_phase(phase_after_turn_);
            }
            break;
        }
        if (yaw_aligned_since_ < 0.0f) yaw_aligned_since_ = t;
        if (t - yaw_aligned_since_ < kTurnSettle) {
            cmd_velocity(0, 0, 0, 0, 0, cmd_seq_++);
            if ((cmd_seq_ % 25) == 0) {
                RCLCPP_INFO(node_->get_logger(),
                    "[Stage4][TURN] yaw 达标稳定中 %.2fs/%.2fs (yaw=%.2f err=%+.3f)",
                    t - yaw_aligned_since_, kTurnSettle, wp.yaw, err);
            }
            break;
        }
        RCLCPP_INFO(node_->get_logger(),
            "[Stage4][TURN] 完成二次校准 yaw=%.2f target=%.2f → next phase", wp.yaw, turn_target_yaw_);
        enter_phase(phase_after_turn_);
        break;
    }

    case Phase::kAdvanceC2: {
        // 检测前方方块
        uint8_t next = scan_next_action(kTrigObstacle);
        if (next == DO::TYPE_BLOCK_OBSTACLE && !already_done(next)) {
            const DO* blk = find_object(next);
            announce(VoiceCmd::ANNOUNCE_IMPASSABLE);
            RCLCPP_INFO(node_->get_logger(),
                "[Stage4][C2] 识别不可跨越方块 (d=%.2f) → 启动绕行序列", blk ? blk->distance : 0.0f);
            detour_substep_ = 0;
            enter_phase(Phase::kDetourCube);
            cmd_velocity(0, -kStrafeSpeed, 0, 0, 0, cmd_seq_++);
            break;
        }
        cmd_velocity(kForwardSpeed, 0, 0, 0, 0, cmd_seq_++);
        if ((wp.ok && wp.y >= 11.00f) || t >= kWalkMaxDur) {
            RCLCPP_INFO(node_->get_logger(),
                "[Stage4][C2] 进入走廊完成 @ world=(%.2f,%.2f) → 撞悬球", wp.x, wp.y);
            enter_phase(Phase::kHitBall);
            cmd_velocity(kSlowSpeed, 0, 0, 0, 0, cmd_seq_++);
        }
        break;
    }

    // 绕方块 + 撞球 + 进入走廊3 的完整序列。
    // 按用户路径：(1.0,?)→横移到 x=1.6 →前进到 y=9.0 →转 -X →走到 (1.0,9.0)
        // →转 +Y →撞球 →转 -Y →回 (1.0,9.0) →转 +X →走到 (2.1,9.0) →转 +Y →进走廊3
    case Phase::kDetourCube: {
        // 点序列参数
        constexpr float kDetourEastX   = 1.60f;
        constexpr float kCubeRowY      = 9.00f;
        constexpr float kCenterAfterDetourX = 1.00f;
        constexpr float kCorridor3X    = 2.10f;
        constexpr float kBallY         = 11.40f;
        constexpr float kPosTol        = 0.06f;
        // 绕行专用速度：全局 kForwardSpeed/kStrafeSpeed 提速后，
        // 这里仍按提速前数值 (0.30) 走，避免每一步因开环冲过 waypoint。
        constexpr float kDetourFwd     = 0.30f;
        constexpr float kDetourStrafe  = 0.30f;

        auto launch_turn = [this](float target) {
            turn_target_yaw_  = target;
            phase_after_turn_ = Phase::kDetourCube;
            enter_phase(Phase::kTurnTo);
        };

        switch (detour_substep_) {
        case 0: {
            // 面向 +Y: vy<0 = 世界 +X（东移）
            cmd_velocity(0, -kDetourStrafe, 0, 0, 0, cmd_seq_++);
            if (wp.ok && wp.x >= kDetourEastX - kPosTol) {
                RCLCPP_INFO(node_->get_logger(), "[Stage4][DETOUR] sub0 完成 → 东侧 x=%.2f", wp.x);
                detour_substep_ = 1;
            }
            if (t > 25.0f) { detour_substep_ = 1; }
            break;
        }
        case 1: {
            cmd_velocity(kDetourFwd, 0, 0, 0, 0, cmd_seq_++);
            if (wp.ok && wp.y >= kCubeRowY - kPosTol) {
                RCLCPP_INFO(node_->get_logger(), "[Stage4][DETOUR] sub1 完成 → 到达 y=%.2f", wp.y);
                detour_substep_ = 2;
            }
            if (t > 40.0f) { detour_substep_ = 2; }
            break;
        }
        case 2: {
            RCLCPP_INFO(node_->get_logger(), "[Stage4][DETOUR] sub2 转向 -X (yaw→π)");
            detour_substep_ = 3;
            launch_turn(static_cast<float>(M_PI));
            break;
        }
        case 3: {
            // 面向 -X (yaw=±π): 机体 +X(前进) = 世界 -X，所以 vx>0 让 world_x 减小
            cmd_velocity(kDetourFwd, 0, 0, 0, 0, cmd_seq_++);
            if (wp.ok && wp.x <= kCenterAfterDetourX + kPosTol) {
                RCLCPP_INFO(node_->get_logger(), "[Stage4][DETOUR] sub3 完成 → 到达 (%.2f,%.2f)", wp.x, wp.y);
                detour_substep_ = 4;
            }
            if (t > 30.0f) { detour_substep_ = 4; }
            break;
        }
        case 4: {
            RCLCPP_INFO(node_->get_logger(), "[Stage4][DETOUR] sub4 转向 +Y (yaw→+π/2)");
            detour_substep_ = 5;
            launch_turn(static_cast<float>(M_PI) / 2.0f);
            break;
        }
        case 5: {
            // 走向橙色小球的速度：原 kSlowSpeed=0.18 → 0.27 (+50%)
            constexpr float kBallApproachVx = 0.27f;
            cmd_velocity(kBallApproachVx, 0, 0, 0, 0, cmd_seq_++);
            if ((wp.ok && wp.y >= kBallY) || t > 30.0f) {
                mark_done(DO::TYPE_ORANGE_BALL);
                announce(VoiceCmd::ANNOUNCE_ORANGE_BALL);
                RCLCPP_INFO(node_->get_logger(),
                    "[Stage4][DETOUR] sub5 悬球撞击完成 @ y=%.2f", wp.y);
                detour_substep_ = 6;
            }
            break;
        }
        case 6: {
            RCLCPP_INFO(node_->get_logger(), "[Stage4][DETOUR] sub6 转向 -Y (yaw→-π/2)");
            detour_substep_ = 7;
            launch_turn(-static_cast<float>(M_PI) / 2.0f);
            break;
        }
        case 7: {
            // 面向 -Y: vx>0 = 世界 -Y，让 y 减小
            cmd_velocity(kDetourFwd, 0, 0, 0, 0, cmd_seq_++);
            if (wp.ok && wp.y <= kCubeRowY + kPosTol) {
                RCLCPP_INFO(node_->get_logger(), "[Stage4][DETOUR] sub7 完成 → 回到 (%.2f,%.2f)", wp.x, wp.y);
                detour_substep_ = 8;
            }
            if (t > 30.0f) { detour_substep_ = 8; }
            break;
        }
        case 8: {
            RCLCPP_INFO(node_->get_logger(), "[Stage4][DETOUR] sub8 转向 +X (yaw→0)");
            detour_substep_ = 9;
            launch_turn(0.0f);
            break;
        }
        case 9: {
            // 面向 +X: vx>0 = 世界 +X
            cmd_velocity(kDetourFwd, 0, 0, 0, 0, cmd_seq_++);
            if (wp.ok && wp.x >= kCorridor3X - kPosTol) {
                RCLCPP_INFO(node_->get_logger(), "[Stage4][DETOUR] sub9 完成 → 到达 (%.2f,%.2f)", wp.x, wp.y);
                detour_substep_ = 10;
            }
            if (t > 30.0f) { detour_substep_ = 10; }
            break;
        }
        case 10: {
            RCLCPP_INFO(node_->get_logger(), "[Stage4][DETOUR] sub10 转向 +Y (yaw→+π/2) → 进走廊3");
            detour_substep_ = 11;
            turn_target_yaw_  = static_cast<float>(M_PI) / 2.0f;
            phase_after_turn_ = Phase::kAdvanceC3;
            enter_phase(Phase::kTurnTo);
            break;
        }
        default: {
            mark_done(DO::TYPE_BLOCK_OBSTACLE);
            RCLCPP_INFO(node_->get_logger(), "[Stage4][DETOUR] 序列完成 → kAdvanceC3");
            enter_phase(Phase::kAdvanceC3);
            break;
        }
        }
        break;
    }

    case Phase::kHitBall: {
        cmd_velocity(kSlowSpeed, 0, 0, 0, 0, cmd_seq_++);
        if ((wp.ok && wp.y >= kBallHitY) || t >= kWalkMaxDur) {
            mark_done(DO::TYPE_ORANGE_BALL);
            announce(VoiceCmd::ANNOUNCE_ORANGE_BALL);
            RCLCPP_INFO(node_->get_logger(),
                "[Stage4][C2] 悬球撞击完成 @ y=%.2f → 赛段四完成", wp.y);
            enter_phase(Phase::kReachedBeam);
            cmd_stop(cmd_seq_++);
        }
        break;
    }

    // ============== 走廊3 ==============
    // 走廊3 中轴 x=2.10；限高杆位置 ≈ (2.08, 10.58)；足球目标 y=11.20（不能撞球门）
    case Phase::kAdvanceC3: {
        // 面向 +Y, 前进；遇限高杆 → 站立对中(x=2.10) → 低姿穿过 → kHitFootball
        // 注意：不要走 scan_next_action(), 因为 already_done(HEIGHT_BAR) 在第一走廊
        // 已两次 mark_done 后会永远返回 true。直接扫 latest_objects。
        const DO* bar_ahead = nullptr;
        if (ctx_->latest_objects) {
            for (const auto& obj : ctx_->latest_objects->objects) {
                if (obj.object_type != DO::TYPE_HEIGHT_BAR) continue;
                if (obj.rel_x > 0.10f && obj.distance < kTrigObstacle) {
                    if (!bar_ahead || obj.distance < bar_ahead->distance)
                        bar_ahead = &obj;
                }
            }
        }
        if (bar_ahead) {
            announce(VoiceCmd::ANNOUNCE_HEIGHT_BAR);
            bar_align_x_           = 2.10f;
            phase_after_bar_       = Phase::kHitFootball;
            phase_after_bar_valid_ = true;
            RCLCPP_INFO(node_->get_logger(),
                "[Stage4][C3] 北向识别限高杆 (d=%.2f, rel_x=%.2f)，站立对中 (x→%.2f)",
                bar_ahead->distance, bar_ahead->rel_x, bar_align_x_);
            enter_phase(Phase::kAlignBar, DO::TYPE_HEIGHT_BAR);
            cmd_velocity(0, 0, 0, 0, 0, cmd_seq_++);
            break;
        }
        cmd_velocity(kForwardSpeed, 0, 0, 0, 0, cmd_seq_++);
        if ((wp.ok && wp.y >= 11.20f) || t >= kWalkMaxDur) {
            // 未识别到杆也兜底，进入足球处理
            RCLCPP_INFO(node_->get_logger(),
                "[Stage4][C3] 未触发杆，直接进入撞球 @ world=(%.2f,%.2f)", wp.x, wp.y);
            enter_phase(Phase::kHitFootball);
        }
        break;
    }

    case Phase::kHitFootball: {
        // 关键：低姿穿杆刚结束，dog 正在从 body_h=0.18 恢复站立 (0.32m)，腿在伸展，
        // 直接发 0.9 m/s 脉冲会被吃掉。先用前 0.8s 站立恢复 + 慢速逼近，再切冲撞。
        constexpr float kStandRestoreDur = 0.8f;
        if (t < kStandRestoreDur) {
            // 站立恢复期：维持 kSlowSpeed 让身体抬起来同时缓慢逼近
            cmd_velocity(kSlowSpeed, 0, 0, 0, 0, cmd_seq_++);
            break;
        }
        // 恢复完毕：继续慢速逼近到 y≈10.55
        cmd_velocity(kSlowSpeed, 0, 0, 0, 0, cmd_seq_++);
        if ((wp.ok && wp.y >= 10.55f) || t >= kWalkMaxDur) {
            RCLCPP_INFO(node_->get_logger(),
                "[Stage4][C3] 已逼近足球 @ y=%.2f (t=%.2fs) → 顶撞脉冲 (vx=1.2, ≤0.55s, 安全门 y≤10.90)", wp.y, t);
            announce(VoiceCmd::ANNOUNCE_SOCCER);
            enter_phase(Phase::kKickBall);
            cmd_velocity(1.2f, 0, 0, 0, 0, cmd_seq_++);
        }
        break;
    }

    case Phase::kKickBall: {
        // 高速短脉冲：把动量传给球，自己几何中心绝不越 y=10.90 (球门 11.0 留 0.1m)
        cmd_velocity(1.2f, 0, 0, 0, 0, cmd_seq_++);
        if (t >= 0.55f || (wp.ok && wp.y >= 10.90f)) {
            RCLCPP_INFO(node_->get_logger(),
                "[Stage4][C3] 顶撞完成 @ y=%.2f (t=%.2fs) → 立即后撤", wp.y, t);
            enter_phase(Phase::kBackOffBall);
            cmd_velocity(-0.5f, 0, 0, 0, 0, cmd_seq_++);
        }
        break;
    }

    case Phase::kBackOffBall: {
        // 倒退脱离球门口，给后续 180°调头留空间
        cmd_velocity(-0.5f, 0, 0, 0, 0, cmd_seq_++);
        if (t >= 1.0f || (wp.ok && wp.y <= 10.20f)) {
            mark_done(DO::TYPE_SOCCER);
            RCLCPP_INFO(node_->get_logger(),
                "[Stage4][C3] 后撤完成 @ y=%.2f → 180°调头 (target yaw=-π/2, 带二次校准)", wp.y);
            turn_target_yaw_  = -static_cast<float>(M_PI) / 2.0f;
            phase_after_turn_ = Phase::kAlignC3Center;
            enter_phase(Phase::kTurnTo);
            cmd_velocity(0, 0, 0, 0, 0, cmd_seq_++);
        }
        break;
    }

    case Phase::kAlignC3Center: {
        // 面向 -Y, body+Y(左) = 世界 +X, 故 vy>0 让 world_x 增大；err = target - x
        constexpr float kC3CenterX = 2.10f;
        float err = wp.ok ? (kC3CenterX - wp.x) : 0.0f;
        if (wp.ok && std::fabs(err) < kAlignTolXNorm) {
            RCLCPP_INFO(node_->get_logger(),
                "[Stage4][C3] 中线对齐完成 @ x=%.2f → 南向出走廊", wp.x);
            // 启动南向走廊3 — 直接走 kExitC3South，途中若遇杆会处理
            enter_phase(Phase::kExitC3South);
            cmd_velocity(kForwardSpeed, 0, 0, 0, 0, cmd_seq_++);
            break;
        }
        float vy = (err > 0 ? +1.0f : -1.0f) * kAlignSpeedNorm;
        cmd_velocity(0, vy, 0, 0, 0, cmd_seq_++);
        if (t > kStrafeMaxDur) {
            enter_phase(Phase::kExitC3South);
            cmd_velocity(kForwardSpeed, 0, 0, 0, 0, cmd_seq_++);
        }
        break;
    }

    case Phase::kExitC3South: {
        // 面向 -Y, 前进让 y 减小；遇杆 → 站立对中 → 低姿穿 → 回到本阶段继续
        const DO* bar_ahead = nullptr;
        if (ctx_->latest_objects) {
            for (const auto& obj : ctx_->latest_objects->objects) {
                if (obj.object_type != DO::TYPE_HEIGHT_BAR) continue;
                if (obj.rel_x > 0.10f && obj.distance < kTrigObstacle) {
                    if (!bar_ahead || obj.distance < bar_ahead->distance)
                        bar_ahead = &obj;
                }
            }
        }
        if (bar_ahead) {
            announce(VoiceCmd::ANNOUNCE_HEIGHT_BAR);
            bar_align_x_           = 2.10f;
            phase_after_bar_       = Phase::kExitC3South;
            phase_after_bar_valid_ = true;
            RCLCPP_INFO(node_->get_logger(),
                "[Stage4][C3] 南向识别限高杆 (d=%.2f, rel_x=%.2f)，站立对中(x=2.10)再低姿穿过",
                bar_ahead->distance, bar_ahead->rel_x);
            enter_phase(Phase::kAlignBar, DO::TYPE_HEIGHT_BAR);
            cmd_velocity(0, 0, 0, 0, 0, cmd_seq_++);
            break;
        }
        cmd_velocity(kForwardSpeed, 0, 0, 0, 0, cmd_seq_++);
        if ((wp.ok && wp.y <= 7.10f) || t >= kWalkMaxDur) {
            RCLCPP_INFO(node_->get_logger(),
                "[Stage4][C3] 南出完成 @ world=(%.2f,%.2f) → 横移到独木桥 (target x=3.15)", wp.x, wp.y);
            enter_phase(Phase::kStrafeToBridge);
            cmd_velocity(0, kStrafeSpeed, 0, 0, 0, cmd_seq_++);
        }
        break;
    }

    case Phase::kStrafeToBridge: {
        // 面向 -Y: vy>0 = 世界 +X, 故 err = 3.15 - x；err>0 用 vy>0
        // 独木桥窄，对中容差收紧到 0.025m，并按 err 动态降速防超调
        constexpr float kBridgeAxisX = 3.15f;
        constexpr float kBridgeXTol  = 0.025f;
        float err = wp.ok ? (kBridgeAxisX - wp.x) : 0.0f;
        if (wp.ok && std::fabs(err) < kBridgeXTol) {
            RCLCPP_INFO(node_->get_logger(),
                "[Stage4][C3] 到达独木桥中轴 @ x=%.3f (err=%+.3f) → 180°调头回 +π/2 (带二次校准)", wp.x, err);
            turn_target_yaw_  = static_cast<float>(M_PI) / 2.0f;
            phase_after_turn_ = Phase::kAdvanceToBridgeTop;
            enter_phase(Phase::kTurnTo);
            cmd_velocity(0, 0, 0, 0, 0, cmd_seq_++);
            break;
        }
        // 进入减速：|err|<0.2m 时按比例放缓到 0.08 m/s 下限
        float speed_mag = std::clamp(std::fabs(err) * 1.5f, 0.08f, kStrafeSpeed);
        float vy = (err > 0 ? +1.0f : -1.0f) * speed_mag;
        cmd_velocity(0, vy, 0, 0, 0, cmd_seq_++);
        if (t > kStrafeMaxDur) {
            RCLCPP_WARN(node_->get_logger(),
                "[Stage4][C3] 横移超时 @ x=%.3f err=%+.3f → 强制调头", wp.x, err);
            turn_target_yaw_  = static_cast<float>(M_PI) / 2.0f;
            phase_after_turn_ = Phase::kAdvanceToBridgeTop;
            enter_phase(Phase::kTurnTo);
        }
        break;
    }

    case Phase::kAdvanceToBridgeTop: {
        // 面向 +Y，上独木桥。桥窄 + 入口 5cm 阶差，必须**同时持续闭环**：
        //   · yaw = +π/2  (防偏航)
        //   · x   = 3.15   (防脱轨)
        // 双 P 控制：vyaw 修航向，vy 修侧偏，vx 用更慢速度面对阶差。
        // 关键：step_height 抬到 0.10m（默认 0.06m 跨不上 5cm 台阶）。
        // 注意：body_height **保持默认 0.32m** — 实测覆盖到 0.34 会让髋膝
        //       接近伸直极限，控制器进入站立保护，四腿僵直无法迈步。
        constexpr float kBridgeAxisX  = 3.15f;
        constexpr float kBridgeTopY   = 7.60f;
        constexpr float kBridgeYawTgt = static_cast<float>(M_PI) / 2.0f;
        // 注意：太慢时 MPC 会退化到近站立、腾空相较弱；
        //       若上桥后手跟不上可适当上调。
        constexpr float kBridgeVx     = 0.15f;
        constexpr float kBridgeStepH  = 0.15f;          // 抬脚 10cm（默认 6cm），跨 5cm 阶差有余量
        constexpr float kKpYaw        = 1.2f;
        constexpr float kKpY          = 1.5f;
        constexpr float kVyawCap      = 0.20f;
        constexpr float kVyCap        = 0.10f;
        constexpr float kBridgeZTop   = 0.33f;          // 上桥后机身 z（地面≈0.27，桥+5cm≈0.32+）
        constexpr float kWarmupSec    = 0.3f;           // 暖机：让 trot 切到高抬脚后再前进

        // 暖机相：转向刚结束就给前进指令，trot 第一拍就要踩台阶；
        // 先原地踏步 0.3s，让 step_height 在控制器内生效再迈步。
        if (t < kWarmupSec) {
            cmd_velocity(0, 0, 0, 0.0f, 0, cmd_seq_++, kBridgeStepH);
            break;
        }
        if (!wp.ok) {
            cmd_velocity(kBridgeVx, 0, 0, 0.0f, 0, cmd_seq_++, kBridgeStepH);
            break;
        }
        float err_yaw = wrap_pi(kBridgeYawTgt - wp.yaw);
        float err_x   = kBridgeAxisX - wp.x;
        // 面向 +Y 时，vy>0 = 世界 -X，所以要让 x 变大需要 vy<0。
        float vy_cmd   = std::clamp(-kKpY * err_x, -kVyCap, kVyCap);
        float vyaw_cmd = std::clamp( kKpYaw * err_yaw, -kVyawCap, kVyawCap);
        cmd_velocity(kBridgeVx, vy_cmd, vyaw_cmd,
                     0.0f, 0, cmd_seq_++, kBridgeStepH);
        if (((cmd_seq_ - kCmdSeqBase) % 25) == 0) {
            RCLCPP_INFO(node_->get_logger(),
                "[Stage4][BRIDGE] t=%.1fs world=(%.3f,%.3f,z=%.3f) yaw=%.2f err_x=%+.3f err_yaw=%+.3f vy=%+.2f vyaw=%+.2f step_h=%.2f",
                t, wp.x, wp.y, wp.z, wp.yaw, err_x, err_yaw, vy_cmd, vyaw_cmd, kBridgeStepH);
        }
        // 完成判据：
        //   · 主路：检测到「前腿上桥」（身体 z 持续 ≥ kFrontOnZ 超过 kFrontOnHold 秒）→ 停止
        //   · 备份：y 越过桥南端 AND z 完全上桥也直接停止
        //   · 超时则强制停止（避免永久卡在本相位）
        constexpr float kFrontOnZ    = 0.271f;          // 站立基线 ≈0.27，trot 抖动半幅 ≈0.01，>0.276 即视为前腿落桥
        constexpr float kFrontOnHold = 0.5f;            // 必须连续保持 ≥ 0.5s 才触发，过滤 trot 抖动
        bool z_hit       = wp.ok && wp.z >= kFrontOnZ;
        // 复用 yaw_aligned_since_ 作为「首次达到高度的时刻 (s within phase)」
        // enter_phase 已将其重置为 -1.0f；z 跌回阈值以下也重置以要求连续保持。
        if (z_hit) {
            if (yaw_aligned_since_ < 0.0f) yaw_aligned_since_ = t;
        } else {
            yaw_aligned_since_ = -1.0f;
        }
        bool front_on    = z_hit && (t - yaw_aligned_since_ >= kFrontOnHold);
        bool y_reached   = wp.ok && wp.y >= kBridgeTopY;
        bool z_reached   = wp.ok && wp.z >= kBridgeZTop;
        if (front_on || (y_reached && z_reached) || t >= kWalkMaxDur) {
            if (front_on || (y_reached && z_reached)) {
                RCLCPP_INFO(node_->get_logger(),
                    "[Stage4][C3] 前腿上桥(持续 %.2fs) @ world=(%.3f,%.3f,z=%.3f) yaw=%.2f → 切零速 trot 保持平衡",
                    t - yaw_aligned_since_, wp.x, wp.y, wp.z, wp.yaw);
            } else {
                RCLCPP_WARN(node_->get_logger(),
                    "[Stage4][C3] 上独木桥超时 @ world=(%.3f,%.3f,z=%.3f) yaw=%.2f → 切零速 trot 保持平衡",
                    wp.x, wp.y, wp.z, wp.yaw);
            }
            enter_phase(Phase::kReachedBeam);
            // 不发 MODE_STOP：bridge 会翻译成 RecoveryStand (按 b 键 3 秒)，
            // 在独木桥上触发会让前腿伸直回名义站立姿态而失衡。
            // 改为零速 trot：MPC trot 步态原地踏步保持平衡，桥接 50Hz 持续重发。
            cmd_velocity(0.0f, 0.0f, 0.0f, 0.0f, 0, cmd_seq_++, kBridgeStepH);
        }
        break;
    }

    case Phase::kBridgeStabilize: {
        // 已弃用：用户要求前腿上桥即停。保留 case 防止 enum 未处理告警。
        enter_phase(Phase::kReachedBeam);
        cmd_velocity(0.0f, 0.0f, 0.0f, 0.0f, 0, cmd_seq_++, 0.10f);
        break;
    }

    case Phase::kHandleBlock:
        // 朝北时绕挡块用前进 + 后退（避免横移与主巡线冲突）
        if      (t < 1.5f) cmd_velocity(kSlowSpeed, kStrafeSpeed, 0, 0, 0, cmd_seq_++);
        else if (t < 3.0f) cmd_velocity(kSlowSpeed, 0, 0, 0, 0, cmd_seq_++);
        else if (t < 4.5f) cmd_velocity(kSlowSpeed, -kStrafeSpeed, 0, 0, 0, cmd_seq_++);
        else {
            mark_done(DO::TYPE_BLOCK_OBSTACLE);
            enter_phase(Phase::kStrafeEast);
            cmd_velocity(0, -kStrafeSpeed, 0, 0, 0, cmd_seq_++);
        }
        break;

    case Phase::kHitTarget:
        if (t >= kHitDur) {
            enter_phase(Phase::kStrafeEast);
            cmd_velocity(0, -kStrafeSpeed, 0, 0, 0, cmd_seq_++);
        }
        break;

    // 后退回桥：机体 −X = 世界 −Y → wp.y 减小
    case Phase::kRetreatSouth: {
        float remain = wp.ok ? std::max(0.0f, wp.y - kReturnYTarget) : 1.0f;
        float k = std::clamp(remain / 0.5f, 0.4f, 1.0f);
        cmd_velocity(-kBackSpeed * k, 0, 0, 0, 0, cmd_seq_++);
        float dist_world = wp.ok ? std::hypot(wp.x - kBridgePos[0],
                                              wp.y - kBridgePos[1])
                                 : 999.0f;
        const DO* beam = find_object(DO::TYPE_BALANCE_BEAM);
        bool reached = (wp.ok && wp.y <= kReturnYTarget) ||
                       dist_world <= kBeamReach ||
                       (beam && beam->distance <= kBeamReach) ||
                       t >= kWalkMaxDur;
        if (reached) {
            RCLCPP_INFO(node_->get_logger(),
                "[Stage4] 抵达独木桥前 world=(%.2f,%.2f) d=%.2f → 赛段四完成",
                wp.x, wp.y, dist_world);
            enter_phase(Phase::kReachedBeam);
            cmd_stop(cmd_seq_++);
        }
        break;
    }

    case Phase::kApproachBridge:
        // 兼容字段，未使用
        enter_phase(Phase::kRetreatSouth);
        break;

    case Phase::kReachedBeam:
        return true;
    }
    return false;
}

}  // namespace competition_manager
