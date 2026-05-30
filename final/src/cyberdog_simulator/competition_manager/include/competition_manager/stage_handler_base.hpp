#pragma once

#include "competition_manager/global_state_machine.hpp"

namespace competition_manager {

// ─────────────────────────────────────────────────────────────
// 赛段处理器基类
// 所有具体赛段（Stage1~Stage6）均继承此类
// ─────────────────────────────────────────────────────────────
class StageHandlerBase {
public:
    explicit StageHandlerBase(rclcpp::Node* node, FSMContext* ctx)
        : node_(node), ctx_(ctx) {}

    virtual ~StageHandlerBase() = default;

    // ── 生命周期接口 ──────────────────────────────────────────
    /// 进入赛段时调用（初始化内部状态）
    virtual void on_enter() = 0;

    /// 每个 tick 调用（50Hz）：返回 true 表示赛段已完成，应转入下一赛段
    virtual bool tick() = 0;

    /// 离开赛段时调用（清理资源）
    virtual void on_exit() = 0;

    // ── 辅助发布接口 ─────────────────────────────────────────
    void publish_motion(competition_msgs::msg::MotionCommand cmd) {
        if (ctx_->publish_motion) ctx_->publish_motion(std::move(cmd));
    }

    void announce(uint8_t announce_type, const std::string& custom_text = "") {
        competition_msgs::msg::VoiceCommand voice_cmd;
        voice_cmd.announce_type = announce_type;
        voice_cmd.text          = custom_text;
        voice_cmd.volume        = 1.0f;
        if (ctx_->publish_voice) ctx_->publish_voice(std::move(voice_cmd));
    }

    // ── 速捷运动帮助函数 ─────────────────────────────────────
    void cmd_stop(uint32_t seq_id = 0);
    void cmd_stand_up(uint32_t seq_id = 0);
    void cmd_lie_down(uint32_t seq_id = 0);
    void cmd_velocity(float vx, float vy, float vyaw,
                      float body_h = 0.0f, float duration_s = 0.0f,
                      uint32_t seq_id = 0, float step_h = 0.0f);
    void cmd_low_walk(float vx, float vy, float body_h = 0.15f,
                      float vyaw = 0.0f,
                      float duration_s = 0.0f, uint32_t seq_id = 0);
    void cmd_jump_down(uint32_t seq_id = 0);
    void cmd_hit_forward(uint32_t seq_id = 0);
    void cmd_kick_forward(uint32_t seq_id = 0);

    // ── 查询辅助函数 ────────────────────────────────────────
    bool is_robot_fallen() const;
    bool motion_completed(uint32_t seq_id) const;

protected:
    rclcpp::Node* node_;
    FSMContext*   ctx_;
};

}  // namespace competition_manager
