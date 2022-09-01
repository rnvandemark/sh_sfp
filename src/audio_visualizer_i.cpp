#include "sh_sfp/audio_visualizer_i.hpp"

#include <sh_common/ros_names.hpp>

namespace sh {

AudioVisualizerI::AudioVisualizerI(const std::string& name) :
    HeartbeatNode(name)
{
    labeled_audio_visualization_sub = create_subscription<sh_sfp_interfaces::msg::LabeledAudioVisualization>(
        sh::names::topics::DEMO_ASCII_AUDIO_VISUALIZER,
        1,
        std::bind(
            &sh::AudioVisualizerI::handle_labeled_audio_visualization,
            this,
            std::placeholders::_1
        )
    );
    playback_update_sub = create_subscription<sh_sfp_interfaces::msg::PlaybackUpdate>(
        sh::names::topics::PLAYBACK_UPDATES_VERBOSE,
        1,
        std::bind(
            &sh::AudioVisualizerI::handle_playback_update,
            this,
            std::placeholders::_1
        )
    );

    RCLCPP_INFO(get_logger(), "Initialized audio visualizer.");
}
AudioVisualizerI::~AudioVisualizerI()
{
}

void AudioVisualizerI::handle_labeled_audio_visualization(
    const sh_sfp_interfaces::msg::LabeledAudioVisualization::SharedPtr msg)
{
    curr_visualization = *msg;
}

void AudioVisualizerI::handle_playback_update(
    const sh_sfp_interfaces::msg::PlaybackUpdate::SharedPtr msg)
{
    if (0 != msg->video_id.compare(curr_visualization.video_id))
    {
        clear_visuals();
    }

    if (!msg->is_paused)
    {
        update_visuals(rclcpp::Duration(msg->duration_current).seconds());
    }
}
}
