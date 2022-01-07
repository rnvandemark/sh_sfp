#ifndef __SMART_HOME_SFP_SOUNDHELPERS_HPP
#define __SMART_HOME_SFP_SOUNDHELPERS_HPP

#include <utility>
#include <vector>

namespace smart_home {

using FreqIdxPair = std::pair<int, double>;
using FreqIdxPairList = std::vector<FreqIdxPair>;

FreqIdxPairList do_1d_dft(std::vector<double> item_buffer);

std::vector<float> get_max_fft_freqs(
    const FreqIdxPairList& mags,
    const float sample_rate,
    const int num_freqs);

}

#endif // __SMART_HOME_SFP_SOUNDHELPERS_HPP
