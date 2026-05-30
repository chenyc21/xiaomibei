#pragma once
#include "competition_manager/stage_handler_base.hpp"
#include <chrono>

namespace competition_manager {

/// 赛段四：深隧寻珍（仿真简化版）
///
/// 出生点 (3.1, 6.6, 0.3) 朝 +Y。前方约 1m 处即独木桥 (3.15, 7.60)，
/// 必须先**绕开独木桥**到走廊西侧，再沿走廊巡视目标。
///
/// 走廊目标 (y≈11)：
///   可乐  (-0.10, 11.10, 0.17)
///   橙球  ( 0.95, 11.10, 0.30)
///   足球  ( 2.10, 10.80, 0.10)
/// 独木桥起点 (3.15, 7.60, 0.05)
///
/// 路径（机体系，初始 yaw=+90° 即朝 +Y，左 = −X，右 = +X）：
///   1) 起身
///   2) **左移 ~3.0m** (vy = +0.3) 绕到走廊西侧 ⇒ 世界坐标 (~0.1, 6.6)
///   3) **前进 ~4.4m** (vx = +0.3) 进入走廊 ⇒ 世界坐标 (~0.1, 11.0)
///   4) **右转 90°**，机体 +X 现在指向世界 +X，沿走廊巡线
///   5) 沿 +X 推进，对扫到的物体即时反应：
///      · 限高杆      → 播报 + 低姿 4s 通过
///      · 无法跨越障碍 → 播报 + 左移→前进→右移 绕过
///      · 可乐 / 橙球 → 播报 + cmd_hit_forward
///      · 足球        → 播报 + cmd_kick_forward
///   6) 看到独木桥且 3 个目标完成 → 右转 90° 朝 −Y
///   7) 沿 −Y 走到独木桥前 (BALANCE_BEAM 距离 < 0.45m)
class Stage4DeepTunnel : public StageHandlerBase {
public:
    using StageHandlerBase::StageHandlerBase;

    void on_enter() override;
    bool tick() override;
    void on_exit() override;

private:
    enum class Phase {
        kStandUp,
        kAdvanceToBypassY,    // 起身后先直行到 y≈7.10，避免踩到黄线
        kStrafeWest,          // 横移 −X 绕开独木桥（保持 +Y 朝向，不转向）
        kAdvanceNorth,        // 前进 +Y 进入走廊（y → 11.10）
        kStrafeEast,          // 横移 +X 巡线寻珍（仍朝 +Y）
        kAlignBar,            // 站立姿态下先把身位横移对准杆中线
        kHandleHeightBar,     // 低姿通过限高杆
        // === 新流程：逐走廊处理 ===
        kAlignCola,           // 对准可乐东西向
        kHitCola,             // 前进撞可乐
        kBackOffCola,         // 后退腾出调头空间
        kTurnAround1,         // 180°调头 (yaw→-π/2 面向 -Y)
        kExitCorridor1,       // 前进（世界-Y）走出走廊
        kStrafeToC2,          // 横移到走廊2中线 x≈0.95
        kTurnAround2,         // 180°调头回 +π/2
        kAdvanceC2,           // 进入走廊2（遇方块转 kDetourCube）
        kDetourCube,          // 绕过不可跨越方块
        kHitBall,             // 前进撞悬球
        kTurnTo,              // 通用转到指定 yaw（带二次校准），完后跳 phase_after_turn_
        kAdvanceC3,           // 走廊3：面 +Y 前进，遇限高杆转 kAlignBar
        kHitFootball,         // 走廊3：慢速逼近足球到 y≈10.55
        kKickBall,            // 走廊3：高速短脉冲 → 顶球进门 (球门 y≈11.0)
        kBackOffBall,         // 走廊3：立即后撤脱离球门口
        kAlignC3Center,       // 走廊3：调头后横移回 x=2.10 中线
        kExitC3South,         // 走廊3：南出走廊到 y≤7.10
        kStrafeToBridge,      // 走廊3：横移到 x=3.15 独木桥中轴
        kAdvanceToBridgeTop,  // 走廊3：调头后前进到 (3.15, 7.60)
        kBridgeStabilize,     // 走廊3：检测到前腿上桥后再走几步稳定然后停止
        kHandleBlock,         // 绕过无法跨越障碍
        kHitTarget,           // 撞击/踢击 目标
        kRetreatSouth,        // 倒退 −Y 回到独木桥（仍朝 +Y，故 vx<0）
        kApproachBridge,      // 接近独木桥（与 RetreatSouth 合并的备用）
        kReachedBeam,
    };

    Phase    phase_       = Phase::kStandUp;
    uint8_t  pending_type_ = 0;          // 当前正在处理的物体类型
    uint32_t cmd_seq_     = 0;
    bool     announced_   = false;

    struct DoneFlags {
        bool cola = false, orange = false, soccer = false;
        bool bar1 = false, bar2 = false, block = false;
        int  target_count() const {
            return (cola ? 1 : 0) + (orange ? 1 : 0) + (soccer ? 1 : 0);
        }
    } done_;

    std::chrono::steady_clock::time_point phase_enter_time_;
    float    yaw_aligned_since_ = -1.0f;  // kAlignBar 航向达标起始时间(s)在本阶段内
    float    yaw_lock_target_   = 0.0f;   // kHandleHeightBar 低姿穿杆时锁定的 yaw
    bool     yaw_lock_valid_    = false;
    float    turn_target_yaw_   = 0.0f;   // kTurnTo 目标航向 (rad, wrap_pi)
    Phase    phase_after_turn_  = Phase::kReachedBeam;
    int      detour_substep_    = 0;      // kDetourCube 的子步骤
    float    bar_align_x_       = 0.0f;   // kAlignBar/kHandleHeightBar 中线 x (调用前赋值)
    Phase    phase_after_bar_       = Phase::kReachedBeam;
    bool     phase_after_bar_valid_ = false;
    bool     c3_only_mode_          = false;  // STAGE4_C3_ONLY=1 时，起身后直接跳到 kAdvanceC3
    bool     bridge_only_mode_      = false;  // STAGE4_BRIDGE_ONLY=1 时，起身后直接跳到 kAdvanceToBridgeTop

    void   enter_phase(Phase p, uint8_t pending = 0);
    float  phase_elapsed() const;

    /// 标记并返回此类目标/障碍是否已经处理过
    bool   already_done(uint8_t type) const;
    void   mark_done(uint8_t type);

    /// 在当前感知里查找指定类型且未交互的最近目标
    const competition_msgs::msg::DetectedObject*
    find_object(uint8_t type) const;

    /// 找最近的尚需处理的对象（任意类型），返回类型，0 表示无
    uint8_t scan_next_action(float trig_dist) const;
};

}  // namespace competition_manager
