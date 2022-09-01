#pragma once

#include "sh_sfp/audio_visualizer_bar_graph_i.hpp"

namespace sh {

// Forward declare
class AsciiVizImpl;
using AsciiVizImplSharedPtr = std::shared_ptr<AsciiVizImpl>;

class AudioVisualizerAscii : public AudioVisualizerBarGraphI
{
protected:
    AsciiVizImplSharedPtr impl;

    rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr heartbeat_sub;

    void update_heartbeat(const std_msgs::msg::Header& beat);

    virtual void clear_visuals();
    virtual bool update_actors(const std::vector<float>& sample_data);

public:
    AudioVisualizerAscii(AsciiVizImplSharedPtr& impl);

    void handle_heartbeat(
        const std_msgs::msg::Header::SharedPtr msg);
};

}
