#pragma once
#include "competition_manager/stage_handler_base.hpp"

namespace competition_manager {

/// 赛段六：撷金建功
/// 任务：将足球从出口位置踢出 → 前往终点 → 四足在圈内趴下
/// 结束条件：四足在终点圆圈内成功趴下
class Stage6FinalGoal : public StageHandlerBase {
public:
    using StageHandlerBase::StageHandlerBase;

    void on_enter() override;
    bool tick() override;
    void on_exit() override;

private:
    enum class InternalState {
        kLocateSoccer,       // 寻找足球位置
        kAlignForKick,       // 对准踢球角度（朝向出口方向）
        kKickOut,            // 执行踢球动作（将球踢出出口）
        kNavigateToFinish,   // 前进至终点圆圈
        kAlignInCircle,      // 精确对准终点圆圈中心
        kLieDown,            // 执行趴下动作
        kConfirmFinish,      // 确认四足在圈内且趴下
    };

    InternalState internal_state_ = InternalState::kLocateSoccer;
    uint32_t      cmd_seq_        = 0;

    // ── 辅助：检测终点圆圈，计算到圆心的偏差
    bool detect_finish_circle(float& offset_x, float& offset_y) const;

    // ── 辅助：确认四足均在终点圆圈内
    bool confirm_feet_in_circle() const;
};

}  // namespace competition_manager
