#pragma once

#include "sh_sfp/audio_translator_i.hpp"

namespace sh {

class AudioTranslatorBarGraph : public AudioTranslatorI
{
protected:
    virtual bool translate(const StampedSamples& stamped_onsets,
                           const StampedSamples& stamped_beats,
                           std::vector<float>& timestamps,
                           std::vector<std::vector<float>>& translation) const;

public:
    AudioTranslatorBarGraph();
};

}
