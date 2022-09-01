#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sh_common/heartbeat_node.hpp>
#include <sh_sfp_interfaces/msg/labeled_audio_characteristics.hpp>
#include <sh_sfp_interfaces/msg/labeled_audio_visualization.hpp>

namespace sh {

class AudioTranslatorI : public HeartbeatNode
{
protected:
    class StampedSamples
    {
    public:
        const std::vector<float> timestamps;
        const std::vector<float> samples;

        StampedSamples(const std::vector<float>& timestamps,
                       const std::vector<float>& samples);

        bool valid() const;
        size_t size() const;
        bool empty() const;
        bool get(const int index,
                 float& timestamp,
                 float& sample) const;

        StampedSamples filled() const;
    };

    rclcpp::Subscription<sh_sfp_interfaces::msg::LabeledAudioCharacteristics>::SharedPtr labeled_audio_characteristics_sub;
    rclcpp::Publisher<sh_sfp_interfaces::msg::LabeledAudioVisualization>::SharedPtr labeled_audio_visualization_pub;

    size_t num_actors;

    void set_num_actors(const size_t new_num_actors);

    virtual bool translate(const StampedSamples& stamped_onsets,
                           const StampedSamples& stamped_beats,
                           std::vector<float>& timestamps,
                           std::vector<std::vector<float>>& translation) const = 0;

    static void parallel_insert(const float timestamp,
                                const std::vector<float>& sample,
                                std::vector<float>& timestamps,
                                std::vector<std::vector<float>>& translation);

public:
    AudioTranslatorI(const std::string& name);
    ~AudioTranslatorI();

    void handle_labeled_audio_characteristics(
        const sh_sfp_interfaces::msg::LabeledAudioCharacteristics::SharedPtr msg);
};

}
