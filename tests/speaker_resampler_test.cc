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
constexpr double kRate = 48000.0;
constexpr double kAmplitude = 16000.0;
constexpr std::size_t kBlocks = 12;

// Resamples kBlocks windows of a stereo tone (left hz_left, right hz_right)
// and returns the worst per-channel error against the ideal band-limited tone
// and the output RMS, skipping the first window (filter warm-up).
struct ToneResult {
  double max_error;
  double rms;
};

ToneResult run_tone(std::size_t input_frames, double hz) {
  vds::SpeakerResampler resampler(input_frames);
  const double step = static_cast<double>(input_frames) /
                      static_cast<double>(vds::kSpeakerOpusFrames);
  const double delay =
      input_frames == vds::kSpeakerOpusFrames
          ? 0.0
          : static_cast<double>(vds::kSpeakerResamplerHalfTaps);
  std::vector<std::int16_t> window(input_frames * vds::kSpeakerChannels);
  double max_error = 0.0;
  double square_sum = 0.0;
  std::size_t counted = 0;
  for (std::size_t block = 0; block < kBlocks; ++block) {
    for (std::size_t frame = 0; frame < input_frames; ++frame) {
      const double t = static_cast<double>(block * input_frames + frame);
      const auto sample = static_cast<std::int16_t>(
          std::lrint(kAmplitude * std::sin(2.0 * kPi * hz * t / kRate)));
      window[frame * 2] = sample;
      window[frame * 2 + 1] = static_cast<std::int16_t>(-sample);
    }
    const auto out = resampler.process(window);
    if (block == 0) {
      continue;
    }
    for (std::size_t frame = 0; frame < vds::kSpeakerOpusFrames; ++frame) {
      const double k =
          static_cast<double>(block * vds::kSpeakerOpusFrames + frame);
      const double ideal =
          kAmplitude * std::sin(2.0 * kPi * hz * (k * step - delay) / kRate);
      max_error = std::max(max_error, std::abs(out[frame * 2] - ideal));
      max_error = std::max(max_error, std::abs(out[frame * 2 + 1] + ideal));
      square_sum += static_cast<double>(out[frame * 2]) * out[frame * 2];
      ++counted;
    }
  }
  return {max_error, std::sqrt(square_sum / static_cast<double>(counted))};
}

} // namespace

int main() {
  // 1:1 windows are passed through untouched.
  {
    vds::SpeakerResampler resampler(vds::kSpeakerOpusFrames);
    std::vector<std::int16_t> window(vds::kSpeakerOpusFrames * 2);
    for (std::size_t i = 0; i < window.size(); ++i) {
      window[i] = static_cast<std::int16_t>(i * 37 - 9000);
    }
    const auto out = resampler.process(window);
    assert(std::equal(out.begin(), out.end(), window.begin()));
  }

  // 512 -> 480: audible tones come through accurately, including across the
  // joins between windows (any discontinuity would show up as error).
  for (const double hz : {100.0, 1000.0, 5000.0, 12000.0, 17000.0}) {
    const auto result = run_tone(vds::kSpeakerInputFrames, hz);
    std::printf("%6.0f Hz: max error %.2f%%\n", hz,
                100.0 * result.max_error / kAmplitude);
    assert(result.max_error < 0.01 * kAmplitude);
  }

  // Content above the output Nyquist (22.5 kHz relative to the input clock)
  // is removed instead of folding back as audible aliasing.
  for (const double hz : {22800.0, 23500.0}) {
    const auto result = run_tone(vds::kSpeakerInputFrames, hz);
    const double db = 20.0 * std::log10(result.rms / (kAmplitude / std::sqrt(2.0)));
    std::printf("%6.0f Hz: %.1f dB\n", hz, db);
    assert(db < -40.0);
  }

  // Silence stays silent.
  {
    vds::SpeakerResampler resampler(vds::kSpeakerInputFrames);
    std::vector<std::int16_t> window(vds::kSpeakerInputFrames * 2, 0);
    const auto out = resampler.process(window);
    assert(std::all_of(out.begin(), out.end(),
                       [](std::int16_t s) { return s == 0; }));
  }
  return 0;
}
