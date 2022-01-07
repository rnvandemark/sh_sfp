#include "smart_home_sfp/SoundHelpers.hpp"

#include <cstdint>
#include <math.h>
#include <algorithm>
#include <fftw3.h>

namespace smart_home {

FreqIdxPairList do_1d_dft(std::vector<double> item_buffer)
{
    // Constants
    const uint64_t items = item_buffer.size();
    const uint64_t half_items = items/2;

    // Do FFT on input data
    fftw_complex* in  = static_cast<fftw_complex*>(fftw_malloc(sizeof(fftw_complex) * items));
    fftw_complex* out = static_cast<fftw_complex*>(fftw_malloc(sizeof(fftw_complex) * items));
    for (int i = 0; i < items; i++)
    {
        in[i][0] = item_buffer[i];
        in[i][1] = 0;
    }

    // 1D Complex DFT
    fftw_plan plan = fftw_plan_dft_1d(items, in, out, FFTW_FORWARD, FFTW_ESTIMATE);
    fftw_execute(plan);

    // Magnitude: sqrt(abs(complex_num))
    FreqIdxPairList fft_mags(half_items);
    for (int i = 0; i < half_items; i++)
    {
        fft_mags[i].first = i;
        fft_mags[i].second = std::pow(std::pow(out[i][0],2) + std::pow(out[i][1],2), 0.25);
    }

    // Release memory
    fftw_destroy_plan(plan);
    fftw_free(in);
    fftw_free(out);

    return fft_mags;
}

std::vector<float> get_max_fft_freqs(const FreqIdxPairList& mags, const float sample_rate, const int num_freqs)
{
    // Sort according to magnitudes, using indices to calculate frequency
    const uint64_t half_items = mags.size() / 2;
    FreqIdxPairList idxs(mags);
    std::sort(idxs.begin(), idxs.begin() + half_items, [](FreqIdxPair a, FreqIdxPair b) { return a.second > b.second; });

    std::vector<float> max_freqs(num_freqs);
    const float s = sample_rate / static_cast<float>(mags.size());
    for (int i = 0; i < num_freqs; i++)
    {
        max_freqs[i] = s * idxs[i].first;
    }
    return max_freqs;
}

}
