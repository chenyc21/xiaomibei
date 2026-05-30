#pragma once
#include "competition_manager/stage_handler_base.hpp"

namespace competition_manager {

/// 赛段五：孤梁稳渡
/// 任务：四足全程在独木桥上行走，四足越过虚线后跳下
/// 结束条件：成功跳下独木桥
class Stage5BalanceBeam : public StageHandlerBase {
public:
    using StageHandlerBase::StageHandlerBase;

    void on_enter() override;
    bool tick() override;
    void on_exit() override;

private:
    enum class InternalState {
        kMountBeam,         // 上桥：前腿上桥后调整姿态
        kWalkOnBeam,        // 独木桥上行走（低速+平衡控制）
        kDetectDashedLine,  // 检测桥上虚线
        kWaitAllFeetPass,   // 等待四足均越过虚线
        kExecuteJump,       // 执行跳下动作
        kConfirmLanded,     // 确认落地成功
    };

    InternalState internal_state_ = InternalState::kMountBeam;
    uint32_t      cmd_seq_        = 0;

    bool all_feet_past_line_ = false;  // 四足是否均越过虚线

    // 独木桥行走参数
    static constexpr float kBeamWalkVelX    = 0.15f;  // 极慢前进速度 m/s
    static constexpr float kBeamBodyHeight  = 0.25f;  // 较低机身高度 m（增加稳定性）
    static constexpr float kRollGain        = 1.5f;   // 横滚补偿增益

    // ── 辅助：独木桥横向平衡控制（根据roll角补偿yaw速度）
    void compute_beam_balance_cmd(float& vyaw);

    // ── 辅助：检测四足是否均越过桥上虚线
    bool check_all_feet_past_line() const;
};

}  // namespace competition_manager
