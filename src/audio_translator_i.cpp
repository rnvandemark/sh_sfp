#include "sh_sfp/audio_translator_i.hpp"

#include <sh_common/ros_names.hpp>

#include <algorithm>
#include <iterator>
#include <utility>

#define TRANSLATOR_ACTOR_COUNT_NAME sh::names::params::TRANSLATOR_ACTOR_COUNT

namespace {
    bool is_0 (const float s)
    {
        return s < 0.001;
    }
    bool is_not_0 (const float s)
    {
        return !is_0(s);
    }
}

namespace sh {

AudioTranslatorI::StampedSamples::StampedSamples(const std::vector<float>& timestamps,
                                                 const std::vector<float>& samples) :
    timestamps(timestamps),
    samples(samples)
{
}

bool AudioTranslatorI::StampedSamples::valid() const
{
    return timestamps.size() == samples.size();
}

size_t AudioTranslatorI::StampedSamples::size() const
{
    return timestamps.size();
}
bool AudioTranslatorI::StampedSamples::empty() const
{
    return 0 != size();
}

bool AudioTranslatorI::StampedSamples::get(const int index,
                                           float& timestamp,
                                           float& sample) const
{
    if (valid() && index < size())
    {
        timestamp = timestamps[index];
        sample = samples[index];
        return true;
    }
    else
    {
        return false;
    }
}

AudioTranslatorI::StampedSamples AudioTranslatorI::StampedSamples::filled() const
{
    const std::vector<float> other_timestamps = timestamps;
    std::vector<float> other_samples = samples;

    if (valid() && !empty())
    {
        const size_t ss = size();
        for (int i = 1; i < ss;)
        {
            if (is_0(other_samples[i]))
            {
                int j = ss-1;
                for (int k = i; k < ss; k++)
                {
                    if (is_not_0(other_samples[k]))
                    {
                        j = k;
                        break;
                    }
                }
                const float ds = (other_samples[j] - other_samples[i-1]) / (j-(i-1));
                for (int k = i; k < j; k++)
                {
                    other_samples[k] += (ds*(k-i+1));
                }
                i = j+1;
            }
            else
            {
                i++;
            }
        }
    }

    return AudioTranslatorI::StampedSamples(other_timestamps, other_samples);
}

AudioTranslatorI::AudioTranslatorI(const std::string& name) :
    HeartbeatNode(name)
{
    declare_parameter<int>(TRANSLATOR_ACTOR_COUNT_NAME);
    set_num_actors(get_parameter(TRANSLATOR_ACTOR_COUNT_NAME).as_int());

    labeled_audio_characteristics_sub = create_subscription<sh_sfp_interfaces::msg::LabeledAudioCharacteristics>(
        sh::names::topics::PLAYBACK_AUDIO_CHARACTERISTICS,
        1,
        std::bind(
            &sh::AudioTranslatorI::handle_labeled_audio_characteristics,
            this,
            std::placeholders::_1
        )
    );
    labeled_audio_visualization_pub = create_publisher<sh_sfp_interfaces::msg::LabeledAudioVisualization>(
        sh::names::topics::DEMO_ASCII_AUDIO_VISUALIZER,
        1
    );

    RCLCPP_INFO(get_logger(), "Initialized audio translator.");
}
AudioTranslatorI::~AudioTranslatorI()
{
}

void AudioTranslatorI::set_num_actors(const size_t new_num_actors)
{
    num_actors = new_num_actors;
}

void AudioTranslatorI::handle_labeled_audio_characteristics(
    const sh_sfp_interfaces::msg::LabeledAudioCharacteristics::SharedPtr msg)
{
    // Start building the audio translated to some visualization's data
    sh_sfp_interfaces::msg::LabeledAudioVisualization labeled_viz;
    labeled_viz.video_id = msg->video_id;
    labeled_viz.visualization.num_actors = num_actors;

    const StampedSamples stamped_onsets(
        msg->characteristics.onsets.data,
        msg->characteristics.onset_pitches.data
    );
    if (!stamped_onsets.valid())
    {
        RCLCPP_ERROR(
            get_logger(),
            "A 1-to-1 correspondence for onsets' timestamps and samples is required, got '%ld' and '%ld'",
            stamped_onsets.timestamps.size(),
            stamped_onsets.samples.size()
        );
        return;
    }

    const StampedSamples stamped_beats(
        msg->characteristics.beats.data,
        msg->characteristics.beat_pitches.data
    );
    if (!stamped_beats.valid())
    {
        RCLCPP_ERROR(
            get_logger(),
            "A 1-to-1 correspondence for beats' timestamps and samples is required, got '%ld' and '%ld'",
            stamped_beats.timestamps.size(),
            stamped_beats.samples.size()
        );
        return;
    }

    std::vector<std::vector<float>> translation;
    if (translate(stamped_onsets,
                  stamped_beats,
                  labeled_viz.visualization.timestamps.data,
                  translation))
    {
        labeled_viz.visualization.translation.reserve(translation.size());
        for (auto iter = translation.cbegin(); iter != translation.cend(); ++iter)
        {
            sh_common_interfaces::msg::Float32Arr arr;
            arr.data = *iter;
            labeled_viz.visualization.translation.push_back(arr);
        }
        labeled_audio_visualization_pub->publish(labeled_viz);
    }
    // else implementation logged an error
}

void AudioTranslatorI::parallel_insert(const float timestamp,
                                       const std::vector<float>& sample,
                                       std::vector<float>& timestamps,
                                       std::vector<std::vector<float>>& translation)
{
    const auto iter_timestamp = std::upper_bound(
        timestamps.cbegin(),
        timestamps.cend(),
        timestamp
    );
    const auto dist_sample = std::distance(
        timestamps.cbegin(),
        iter_timestamp
    );
    timestamps.insert(iter_timestamp, timestamp);
    translation.insert(
        translation.cbegin() + dist_sample,
        sample
    );
}
}
