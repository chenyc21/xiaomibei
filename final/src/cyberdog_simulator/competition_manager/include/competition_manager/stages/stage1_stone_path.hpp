#pragma once
#include "competition_manager/stage_handler_base.hpp"

namespace competition_manager {

/// 赛段一：石径探路
/// 任务：沿石板路前进 → 检测弯道 → 完成转弯
/// 结束条件：后腿离开弯道虚线
class Stage1StonePath : public StageHandlerBase {
public:
    using StageHandlerBase::StageHandlerBase;

    void on_enter() override;
    bool tick() override;
    void on_exit() override;

private:
    enum class InternalState {
        kStandUp,       // 起身
        kFollowPath,    // 沿石板路前进（视觉引导）
        kDetectTurn,    // 检测弯道
        kExecuteTurn,   // 执行转弯
        kWaitExit,      // 等待后腿离开虚线
    };

    InternalState internal_state_ = InternalState::kStandUp;
    uint32_t      cmd_seq_        = 0;

    // ── 辅助：边界跟踪控制律（PD控制）
    void boundary_follow_control();

    // ── 辅助：弯道角度估计
    float estimate_turn_angle() const;
};

}  // namespace competition_manager
