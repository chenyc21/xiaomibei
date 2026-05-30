#pragma once
#include "competition_manager/stage_handler_base.hpp"

namespace competition_manager {

/// 赛段三：曲道冲锋
/// 任务：在弯道赛道内前进，全程不踩线、不出线
/// 结束条件：后腿离开弯道出口虚线
class Stage3CurvedTrack : public StageHandlerBase {
public:
    using StageHandlerBase::StageHandlerBase;

    void on_enter() override;
    bool tick() override;
    void on_exit() override;

private:
    // 时序控制（开环）：每段时长和速度对应 S 弯赛道几何
    enum class InternalState {
        kStandUp,       // 起身等待（3s）
        kCurveLeft,     // 左弯
        kMidStraight,   // 中间直道
        kCurveRight,    // 右弯
        kCalibrate,     // 二次校准位置到 (3.1, 6.6) yaw=+π/2
    };

    InternalState internal_state_ = InternalState::kStandUp;
    uint32_t      cmd_seq_        = 0;

    // 各阶段起始时间戳
    std::chrono::steady_clock::time_point phase_start_;

    // ── 几何目标（不依赖匀速假设）───────────────────────────────────────
    // 弯道：半径 r=1m，圆心角 π/2；须满足 kVyawCurve = kVxCurve / r
    static constexpr float kCurveRadius  = 1.0f;              // 弯道半径 (m)
    static constexpr float kCurveAngle   = 3.14159265f / 2.0f; // π/2 rad = 90°
    static constexpr float kStraightDist = 1.4f;              // 中间直道 (m)

    // ── 速度参数（已根据真值标定）────────────────────────────────────────
    // 实测发现：cmd vyaw=0.30 时实际转得快约 27%，cmd vx=0.30 时实际走得慢约 12%
    // 导致命令半径 1.0m → 实际半径仅 0.69m。把命令比 vx/vyaw 调到 1.45 补偿。
    static constexpr float kStandUpWaitSec  = 3.0f;   
    // 加速阶段最少持续时间：避免起步瞬间 |Δyaw| 还未脱离 0 时被误判完成
    static constexpr float kPhaseMinSec     = 0.3f;
    static constexpr float kVxFast    = 0.35f;         // 直道前进速度 m/s
    static constexpr float kVxCurve   = 0.35f;         // 弯道前进速度 m/s
    // 实测发现左右不对称：同样 cmd vyaw 下左转实际转得慢约 5-6%。
    // 右弯 r=0.99m 调好后，左弯 r=1.054m 仍偏大，需单独提高 vyaw。
    static constexpr float kVyawCurveRight = 0.241f;   // rad/s
    static constexpr float kVyawCurveLeft  = 0.261f;   // rad/s (+8.3% vs right, 补偿左右不对称+y偏高)
    // 直道航向锁定 P 增益：vyaw_cmd = -kStraightYawKp * (yaw - yaw0)
    static constexpr float kStraightYawKp = 1.5f;
    static constexpr float kStraightYawClamp = 0.30f;  // 限幅 rad/s

    // ── 二次校准参数 ──────────────────────────────────────────
    // 赛段3 结束位姿可能偏离理论点 (3.1, 6.6) yaw=+π/2，
    // 进入 kCalibrate 后闭环收敛到该点，为赛段4 的横移提供准确起点。
    static constexpr float kCalibTargetX   = 3.10f;
    static constexpr float kCalibTargetY   = 6.60f;
    static constexpr float kCalibTargetYaw = 3.14159265f / 2.0f;
    static constexpr float kCalibPosTol    = 0.08f;   // m
    static constexpr float kCalibYawTol    = 0.06f;   // rad (~3.4°)
    static constexpr float kCalibKpPos     = 0.6f;    // 1/s
    static constexpr float kCalibKpYaw     = 1.5f;    // 1/s
    static constexpr float kCalibVxMax     = 0.25f;   // m/s
    static constexpr float kCalibVyMax     = 0.25f;   // m/s
    static constexpr float kCalibVyawMax   = 0.30f;   // rad/s
    static constexpr float kCalibMaxSec    = 8.0f;    // 超时兑底

    // ── 位姿基准（每次进入新 phase 时重置）────────────────────────────────
    // 用 RobotState 的绝对 yaw / position 与 phase 起始值作差判断完成度，
    // 不再积分速度，彻底避免加速段 / 噪声 / 漂移误差。
    bool  phase_origin_set_ = false;
    float phase_start_yaw_  = 0.0f;
    float phase_start_x_    = 0.0f;
    float phase_start_y_    = 0.0f;
    // fallback 路径用：若 RobotState 始终不可用，仍按时序兜底
    static constexpr float kFallbackCurveSec    = 5.24f;  // π/2 / 0.30 ≈ 5.24s
    static constexpr float kFallbackStraightSec = 4.00f;  // 1.4 / 0.35 = 4.0s

    // 工具函数
    float phase_elapsed() const {
        return std::chrono::duration<float>(
            std::chrono::steady_clock::now() - phase_start_).count();
    }
    void reset_phase() {
        phase_start_      = std::chrono::steady_clock::now();
        phase_origin_set_ = false;
    }
    static float wrap_to_pi(float a) {
        while (a >  3.14159265f) a -= 6.2831853f;
        while (a < -3.14159265f) a += 6.2831853f;
        return a;
    }
};

}  // namespace competition_manager
