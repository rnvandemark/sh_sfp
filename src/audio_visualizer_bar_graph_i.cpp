#include "sh_sfp/audio_visualizer_ascii.hpp"

#include <sh_common/ros_names.hpp>

#include <cmath>

namespace sh {

AudioVisualizerBarGraphI::AudioVisualizerBarGraphI(const std::string& name) :
    AudioVisualizerI(name)
{
}

void AudioVisualizerBarGraphI::update_visuals(const double curr_time)
{
    const auto timestamps = curr_visualization.visualization.timestamps.data;
    const auto translation = curr_visualization.visualization.translation;
    if (timestamps.empty())
    {
        RCLCPP_ERROR(get_logger(), "No timestamps in current visualization!");
        return;
    }

    const auto iter_stamp_rhs = std::lower_bound(timestamps.cbegin(),
                                                 timestamps.cend(),
                                                 curr_time);
    if (timestamps.cend() == iter_stamp_rhs)
    {
        // Nothing left to do
        return;
    }
    else if (timestamps.cbegin() == iter_stamp_rhs)
    {
        // The playback timestamp is before the first sample, don't bother
        // interpolating, just send that
        if (!translation.empty())
        {
            update_actors(translation.front().data);
        }
        return;
    }

    const auto iter_stamp_lhs = std::prev(iter_stamp_rhs);
    const float stamp_lhs = ((timestamps.cend() == iter_stamp_lhs) ? 0.0 : *iter_stamp_lhs);
    const float stamp_rhs = *iter_stamp_rhs;

    const auto dist_lhs = ((timestamps.cend() == iter_stamp_lhs) ? 0 : std::distance(timestamps.cbegin(), iter_stamp_lhs));
    const auto dist_rhs = std::distance(timestamps.cbegin(), iter_stamp_rhs);

    const auto iter_trans_lhs = translation.cbegin() + dist_lhs;
    const auto iter_trans_rhs = translation.cbegin() + dist_rhs;
    if ((translation.cend() == iter_trans_lhs) || (translation.cend() == iter_trans_rhs))
    {
        RCLCPP_ERROR(get_logger(), "Failed to get translation corresponding to timestamp!");
        return;
    }

    const std::vector<float> trans_lhs = iter_trans_lhs->data;
    const std::vector<float> trans_rhs = iter_trans_rhs->data;
    std::vector<float> data(trans_lhs.size());
    for (int i = 0; i < trans_lhs.size(); i++)
    {
        const double exp_a = trans_lhs[i];
        const double exp_b = std::pow(trans_rhs[i]/exp_a, 1/(stamp_rhs-stamp_lhs));
        data[i] = exp_a * std::pow(exp_b, (curr_time-stamp_lhs));
    }

    update_actors(data);
}
}
