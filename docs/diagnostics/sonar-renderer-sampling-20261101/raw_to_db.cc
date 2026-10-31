// SPDX-License-Identifier: Apache-2.0
// Same expression as POSIM MultibeamSonarSensor.cc's raw-message publication.
#include <cmath>
#include <complex>
#include <cstddef>
extern "C" void posim_trace_raw_db(const float *input, float *output,
                                    std::size_t beams, std::size_t bins) {
  for (std::size_t r = 0; r < bins; ++r)
    for (std::size_t b = 0; b < beams; ++b) {
      const auto i = 2 * (b * bins + r);
      float power_dB = 20.0f * log10(std::abs(std::complex<float>(input[i], input[i + 1])));
      output[r * beams + b] = power_dB;
    }
}
