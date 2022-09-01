#pragma once

#include "sh_sfp/audio_visualizer_i.hpp"

#include <mutex>

namespace sh {

class AudioVisualizerBarGraphI : public AudioVisualizerI
{
protected:
    virtual void update_visuals(const double curr_time);

    virtual bool update_actors(const std::vector<float>& sample_data) = 0;

public:
    AudioVisualizerBarGraphI(const std::string& name);

};

}
