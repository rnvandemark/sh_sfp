#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sh_common/heartbeat_node.hpp>
#include <sh_sfp_interfaces/msg/labeled_audio_visualization.hpp>
#include <sh_sfp_interfaces/msg/playback_update.hpp>

namespace sh {

class AudioVisualizerI : public HeartbeatNode
{
protected:
    rclcpp::Subscription<sh_sfp_interfaces::msg::LabeledAudioVisualization>::SharedPtr labeled_audio_visualization_sub;
    rclcpp::Subscription<sh_sfp_interfaces::msg::PlaybackUpdate>::SharedPtr playback_update_sub;

    sh_sfp_interfaces::msg::LabeledAudioVisualization curr_visualization;

    virtual void clear_visuals() = 0;
    virtual void update_visuals(const double curr_time) = 0;

public:
    AudioVisualizerI(const std::string& name);
    virtual ~AudioVisualizerI();

    virtual void handle_labeled_audio_visualization(
        const sh_sfp_interfaces::msg::LabeledAudioVisualization::SharedPtr msg);

    virtual void handle_playback_update(
        const sh_sfp_interfaces::msg::PlaybackUpdate::SharedPtr msg);
};

}
