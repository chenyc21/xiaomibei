/// voice_node.cpp — TTS语音播报节点骨架
///
/// 功能：
///   1. 订阅 VoiceCommand
///   2. 将预定义播报类型或自定义文本转为音频输出
///   3. 支持两种后端：
///      a) espeak-ng（离线，轻量）
///      b) 系统TTS（如 text_to_speech ROS2包）

#include <rclcpp/rclcpp.hpp>
#include <cstdlib>
#include <string>
#include "competition_msgs/msg/voice_command.hpp"

using VoiceCmd = competition_msgs::msg::VoiceCommand;

namespace competition_voice {

// 预定义播报文本（对应赛规要求的固定语句）
static const std::string kAnnounceTexts[] = {
    "",                         // 0: NONE
    "识别到可乐瓶",              // 1: COLA
    "识别到橙色小球",            // 2: ORANGE_BALL
    "识别到足球",                // 3: SOCCER
    "识别到限高杆",              // 4: HEIGHT_BAR
    "识别到无法跨越障碍",        // 5: IMPASSABLE
};

class VoiceNode : public rclcpp::Node {
public:
    explicit VoiceNode(const rclcpp::NodeOptions& options)
        : Node("competition_voice", options)
    {
        declare_parameter("tts_backend", "espeak");  // "espeak" 或 "system"
        declare_parameter("language", "zh");

        tts_backend_ = get_parameter("tts_backend").as_string();

        voice_sub_ = create_subscription<VoiceCmd>(
            "/competition/voice_cmd", 10,
            [this](VoiceCmd::SharedPtr msg) { on_voice_command(msg); });

        RCLCPP_INFO(get_logger(), "VoiceNode started (backend: %s).", tts_backend_.c_str());
    }

private:
    void on_voice_command(VoiceCmd::SharedPtr msg) {
        std::string text;

        if (msg->announce_type == VoiceCmd::ANNOUNCE_CUSTOM) {
            text = msg->text;
        } else if (msg->announce_type < sizeof(kAnnounceTexts) / sizeof(kAnnounceTexts[0])) {
            text = kAnnounceTexts[msg->announce_type];
        } else {
            RCLCPP_WARN(get_logger(), "Unknown announce_type: %d", msg->announce_type);
            return;
        }

        if (text.empty()) return;

        RCLCPP_INFO(get_logger(), "[TTS] %s", text.c_str());
        speak(text);
    }

    void speak(const std::string& text) {
        if (tts_backend_ == "espeak") {
            // espeak-ng 支持中文（需安装 espeak-ng-data 中文语音包）
            std::string cmd = "espeak-ng -v zh -s 150 \"" + text + "\" &";
            std::system(cmd.c_str());  // NOLINT: 竞赛环境可控，此处可接受
        } else {
            // 备用：使用 aplay 播放预录制的音频文件
            // 音频文件路径约定：/opt/competition/audio/<announce_type>.wav
            RCLCPP_WARN(get_logger(), "System TTS backend not implemented, falling back to log.");
        }
    }

    std::string tts_backend_;
    rclcpp::Subscription<VoiceCmd>::SharedPtr voice_sub_;
};

}  // namespace competition_voice

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto node = std::make_shared<competition_voice::VoiceNode>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
