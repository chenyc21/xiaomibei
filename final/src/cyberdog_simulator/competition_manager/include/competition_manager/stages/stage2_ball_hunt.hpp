#pragma once
#include "competition_manager/stage_handler_base.hpp"
#include <set>

namespace competition_manager {

/// 赛段二：荒野寻珠
/// 任务：在4×4球阵中找出橙色球（每行每列各1个），逐一撞击使其晃动
/// 结束条件：所有橙色球完成交互，后腿离开出口虚线
class Stage2BallHunt : public StageHandlerBase {
public:
    using StageHandlerBase::StageHandlerBase;

    void on_enter() override;
    bool tick() override;
    void on_exit() override;

private:
    enum class InternalState {
        kScanArea,       // 扫描区域，建立球阵位置图
        kSelectTarget,   // 选择下一个目标橙色球
        kApproach,       // 接近目标球
        kHit,            // 执行撞击
        kConfirmHit,     // 等待球晃动确认（视觉验证）
        kNavigateExit,   // 前往出口
        kWaitExit,       // 等待后腿离开出口虚线
    };

    InternalState internal_state_ = InternalState::kScanArea;
    uint32_t      cmd_seq_        = 0;

    std::set<uint32_t> completed_balls_;  // 已完成交互的球ID集合
    uint32_t target_ball_id_ = 0;         // 当前目标球ID

    // ── 辅助：基于每行每列规则推理橙色球位置
    void infer_orange_ball_layout();

    // ── 辅助：从感知结果中选择最优下一目标
    bool select_next_target();

    // ── 辅助：检测目标球是否已晃动（比较连续帧bbox变化）
    bool detect_ball_oscillation(uint32_t ball_id) const;
};

}  // namespace competition_manager
