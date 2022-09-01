#include "sh_sfp/audio_translator_bar_graph.hpp"

#include <random>

using RNG = std::mt19937;
using UID = std::uniform_int_distribution<std::mt19937::result_type>;

namespace sh {

AudioTranslatorBarGraph::AudioTranslatorBarGraph() :
    AudioTranslatorI("audio_translator_bar_graph")
{
}

bool AudioTranslatorBarGraph::translate(const StampedSamples& stamped_onsets,
                                        const StampedSamples& stamped_beats,
                                        std::vector<float>& timestamps,
                                        std::vector<std::vector<float>>& translation) const
{
//    WindowedAverage<float> wavg(20, 1);
//    wavg.reset(0.0f);
//
//    std::vector<float> DDD;
//    for (int i = 0; i < stamped_beats.timestamps.size(); i++)
//    {
//        const float beat_stamp = stamped_beats.timestamps[i];
//        const float beat_pitch = stamped_beats.samples[i];
//
//        if (!wavg.push_and_eval({beat_stamp}, DDD))
//        {
//            RCLCPP_ERROR(get_logger(), "Failed to add windowed average sample.");
//            return;
//        }
//
//        sh_common_interfaces::msg::Float32Arr translation;
//
//        
//    }

    // Fill the empty portions of the audio analysis results
    const StampedSamples fs_onsets = stamped_onsets.filled();
    const StampedSamples fs_beats = stamped_beats.filled();
    if (!(fs_onsets.valid() && fs_beats.valid()))
    {
        RCLCPP_ERROR(get_logger(), "Invalid audio characteristics.");
        return false;
    }

    std::random_device device;
    RNG rng(device());
    UID dist_bool(0, 2);
    UID dist_num_actors(0, num_actors);

    // Constants which can be tuned
    const double beat_range = 0.25;
    const double actor_index_period = 2.0;

    const int rand_range = static_cast<int>(beat_range * static_cast<int>(num_actors));
    if (0 >= rand_range)
    {
        RCLCPP_ERROR(
            get_logger(),
            "Invalid range=%d due to beat_range=%lf and num_actors=%ld",
            rand_range,
            beat_range,
            num_actors
        );
        return false;
    }

    // Other constants
    const float max_beat_pitch = *(std::max_element(fs_beats.samples.cbegin(),
                                                    fs_beats.samples.cend()));

    int curr_actor_index = -1;
    float last_timestamp = -100.0;  // 'negative infinity'
    for (size_t i = 0; i < fs_beats.size(); i++)
    {
        float timestamp = 0.0f, sample = 0.0f;
        if (!fs_beats.get(i, timestamp, sample))
        {
            continue;
        }

        const float adjusted_sample = std::max(sample / max_beat_pitch,
                                               0.05f);

        if (timestamp - last_timestamp > actor_index_period)
        {
            curr_actor_index = dist_num_actors(rng);
            if (1 == dist_bool(rng))
            {
                curr_actor_index = static_cast<int>(num_actors) - curr_actor_index;
            }
            last_timestamp = timestamp;
        }

        std::vector<float> translation_sample(num_actors);
        translation_sample[curr_actor_index] = adjusted_sample;

        parallel_insert(
            timestamp,
            translation_sample,
            timestamps,
            translation
        );
    }

    return true;
}
}

int main(int argc, char** argv) 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<sh::AudioTranslatorBarGraph>());
    return 0;
}
