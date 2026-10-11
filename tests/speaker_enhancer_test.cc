// SPDX-License-Identifier: MIT
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <utility>
#include <vector>

#include "../src/vds_protocol.hh"

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr std::size_t kFrames = 48000; // one second
constexpr std::size_t kSettle = 9600;  // skip filter/limiter settling

std::vector<std::int16_t> tone(double hz, double amplitude) {
  std::vector<std::int16_t> samples(kFrames * 2);
  for (std::size_t frame = 0; frame < kFrames; ++frame) {
    const auto value = static_cast<std::int16_t>(std::lrint(
        amplitude * std::sin(2.0 * kPi * hz * static_cast<double>(frame) /
                             48000.0)));
    samples[frame * 2] = value;
    samples[frame * 2 + 1] = value;
  }
  return samples;
}

// Processes in 512-frame windows, like vdsd.
void run(vds::SpeakerEnhancer &enhancer, std::vector<std::int16_t> &samples) {
  for (std::size_t frame = 0; frame < kFrames; frame += 512) {
    const std::size_t count = std::min<std::size_t>(512, kFrames - frame);
    enhancer.process(std::span<std::int16_t>(samples.data() + frame * 2,
                                             count * 2),
                     count);
  }
}

double peak(const std::vector<std::int16_t> &samples) {
  int value = 0;
  for (std::size_t i = kSettle * 2; i < samples.size(); ++i) {
    value = std::max(value, std::abs(static_cast<int>(samples[i])));
  }
  return value;
}

double db(double ratio) { return 20.0 * std::log10(ratio); }

} // namespace

int main() {
  const double ceiling = 0.891 * 32767.0;

  // Bypassed: untouched.
  {
    vds::SpeakerEnhancer enhancer;
    auto samples = tone(1000.0, 20000.0);
    const auto original = samples;
    run(enhancer, samples);
    assert(samples == original);
  }

  // A quiet 1 kHz tone gets the full boost.
  {
    vds::SpeakerEnhancer enhancer;
    enhancer.set_boost_db(8.0f);
    auto samples = tone(1000.0, 1000.0);
    run(enhancer, samples);
    const double gain = db(peak(samples) / 1000.0);
    std::printf("quiet 1 kHz: %+.2f dB\n", gain);
    assert(gain > 7.5 && gain < 8.5);
  }

  // A full-scale tone is boosted up to the ceiling and never clips.
  for (const double hz : {300.0, 1000.0, 6000.0}) {
    vds::SpeakerEnhancer enhancer;
    enhancer.set_boost_db(18.0f);
    auto samples = tone(hz, 32767.0);
    run(enhancer, samples);
    const double level = peak(samples);
    std::printf("full-scale %5.0f Hz +18 dB: peak %.1f dBFS\n", hz,
                db(level / 32767.0));
    assert(level <= ceiling + 1.0);
    assert(level > ceiling * 0.9);
  }

  // The volume slider follows a cubic curve, with and without the boost, and
  // is applied after the limiter (no clipping, no limiter interaction).
  for (const float boost : {0.0f, 8.0f}) {
    for (const auto [percent, expected_db] :
         {std::pair{90, -2.75}, std::pair{50, -18.06}}) {
      vds::SpeakerEnhancer enhancer;
      enhancer.set_boost_db(boost);
      enhancer.set_volume_percent(static_cast<std::uint16_t>(percent));
      auto samples = tone(1000.0, 1000.0);
      run(enhancer, samples);
      const double level = db(peak(samples) / 1000.0) - boost;
      std::printf("volume %d%% boost %.0f: %+.2f dB\n", percent, boost, level);
      assert(std::abs(level - expected_db) < 0.3);
    }
  }

  // Bass the speaker cannot reproduce is removed, not boosted.
  {
    vds::SpeakerEnhancer enhancer;
    enhancer.set_boost_db(8.0f);
    auto samples = tone(50.0, 10000.0);
    run(enhancer, samples);
    const double level = db(peak(samples) / 10000.0);
    std::printf("50 Hz: %+.1f dB\n", level);
    assert(level < -25.0);
  }

  // Limiter stats: a full-scale tone at +18 dB is cut ~19 dB (to -1 dBFS),
  // a quiet one not at all, and silence is not counted.
  {
    vds::SpeakerEnhancer enhancer;
    enhancer.set_boost_db(18.0f);
    auto loud = tone(1000.0, 32767.0);
    run(enhancer, loud);
    const vds::SpeakerLimiterStats stats = enhancer.take_limiter_stats();
    std::printf("full-scale +18 dB limiting: avg %.1f dB, max %.1f dB\n",
                stats.average_db(), stats.max_reduction_db);
    assert(stats.frames > kFrames / 2);
    assert(stats.average_db() > 17.0f && stats.max_reduction_db < 20.0f);
    assert(enhancer.take_limiter_stats().frames == 0);

    auto quiet = tone(1000.0, 1000.0);
    run(enhancer, quiet); // let the limiter release first
    enhancer.take_limiter_stats();
    quiet = tone(1000.0, 1000.0);
    run(enhancer, quiet);
    const vds::SpeakerLimiterStats quiet_stats = enhancer.take_limiter_stats();
    std::printf("quiet +18 dB limiting: avg %.2f dB\n", quiet_stats.average_db());
    assert(quiet_stats.frames > 0 && quiet_stats.average_db() < 0.1f);

    // Silence: only the high-pass filter's short tail after the tone counts.
    std::vector<std::int16_t> silence(kFrames * 2, 0);
    run(enhancer, silence);
    assert(enhancer.take_limiter_stats().frames < kFrames / 100);
    silence.assign(kFrames * 2, 0); // process() wrote the tail into it
    run(enhancer, silence);
    assert(enhancer.take_limiter_stats().frames == 0);
  }
  return 0;
}
