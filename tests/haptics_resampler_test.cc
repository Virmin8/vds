// SPDX-License-Identifier: MIT
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <vector>

#include "../src/vds_protocol.hh"

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr std::size_t kWindow = vds::kSpeakerInputFrames; // 512 -> 32
constexpr std::size_t kBlocks = 100;                      // ~1 s
constexpr std::size_t kSkipBlocks = 2;                    // filter warm-up

// Feeds a tone (both channels) through the resampler and returns the 3 kHz
// output in 16-bit units (8-bit value * 256), left channel only.
std::vector<double> run(double hz, double amplitude,
                        std::uint16_t gain_percent = 100) {
  vds::HapticsResampler resampler(kWindow);
  resampler.set_gain_percent(gain_percent);
  std::vector<std::int16_t> window(kWindow * 2);
  std::vector<double> out;
  for (std::size_t block = 0; block < kBlocks; ++block) {
    for (std::size_t frame = 0; frame < kWindow; ++frame) {
      const double t = static_cast<double>(block * kWindow + frame);
      const auto sample = static_cast<std::int16_t>(
          std::lrint(amplitude * std::sin(2.0 * kPi * hz * t / 48000.0)));
      window[frame * 2] = sample;
      window[frame * 2 + 1] = sample;
    }
    const auto chunk = resampler.process(window);
    if (block < kSkipBlocks) {
      continue;
    }
    for (std::size_t frame = 0; frame < vds::kHapticsFrames; ++frame) {
      out.push_back(static_cast<double>(chunk[frame * 2]) * 256.0);
    }
  }
  return out;
}

// Amplitude of the hz component (least-squares fit of sin + cos).
double component(const std::vector<double> &out, double hz) {
  double s = 0.0;
  double c = 0.0;
  for (std::size_t i = 0; i < out.size(); ++i) {
    const double phase = 2.0 * kPi * hz * static_cast<double>(i) / 3000.0;
    s += out[i] * std::sin(phase);
    c += out[i] * std::cos(phase);
  }
  return 2.0 * std::hypot(s, c) / static_cast<double>(out.size());
}

double rms(const std::vector<double> &out) {
  double sum = 0.0;
  for (const double value : out) {
    sum += value * value;
  }
  return std::sqrt(sum / static_cast<double>(out.size()));
}

double db(double ratio) { return 20.0 * std::log10(ratio); }

} // namespace

int main() {
  // The actuator band comes through at the right level.
  for (const double hz : {60.0, 160.0, 320.0, 640.0, 800.0}) {
    const double level = db(component(run(hz, 16000.0), hz) / 16000.0);
    std::printf("%5.0f Hz: %+.2f dB\n", hz, level);
    assert(std::abs(level) < 0.5);
  }

  // Content above the 1.5 kHz output Nyquist is removed instead of folding
  // back into the actuator band (2.5 kHz would alias to 500 Hz).
  for (const double hz : {1800.0, 2500.0, 6000.0}) {
    const double level = db(rms(run(hz, 16000.0)) / (16000.0 / std::sqrt(2.0)));
    std::printf("%5.0f Hz: %+.1f dB\n", hz, level);
    assert(level < -40.0);
  }

  // Detail below one 8-bit step (256 in 16-bit units) survives; plain
  // truncation turned it into silence.
  {
    const double level = component(run(160.0, 120.0), 160.0);
    std::printf("quiet 160 Hz (amp 120): %.0f\n", level);
    assert(level > 100.0 && level < 140.0);
  }

  // Silence stays exactly silent.
  {
    const auto out = run(160.0, 0.0);
    assert(std::all_of(out.begin(), out.end(),
                       [](double value) { return value == 0.0; }));
  }

  // Gain is applied before quantization.
  {
    const double level = db(component(run(160.0, 4000.0, 200), 160.0) / 4000.0);
    std::printf("gain 200%%: %+.2f dB\n", level);
    assert(std::abs(level - 6.02) < 0.5);
  }
  return 0;
}
