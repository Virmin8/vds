// SPDX-License-Identifier: MIT
// Copyright (C) 2026 Jihong Min <hurryman2212@gmail.com>
//
// Partly influenced by DS5Dongle source:
// Copyright (c) 2026 awalol, released under the MIT license.
//
// DualSense USB/Bluetooth protocol helpers: CRC, output state merging, HID
// report framing, and USB audio to Bluetooth haptics/speaker packet conversion.

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cmath>
#include <cstring>
#include <memory>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <string>

#include <opus/opus.h>

#include "uapi/vds.h"
#include "vds_companion.hh"
#include "vds_protocol.hh"

namespace vds {

namespace {

constexpr std::uint32_t kOutputCrcSeed = 0xeada2d49;
constexpr std::uint32_t kFeatureCrcSeed = 0x2060efc3;
constexpr std::size_t kPcmChannels = VDS_AUDIO_CHANNELS;
constexpr std::size_t kPcmSampleSize = sizeof(std::int16_t);
constexpr std::size_t kPcmFrameSize = kPcmChannels * kPcmSampleSize;
constexpr std::uint32_t kHapticsDecimation =
    VDS_AUDIO_SAMPLE_RATE / VDS_HAPTICS_SAMPLE_RATE;
constexpr std::size_t kSetStateSize = VDS_SET_STATE_SIZE;
constexpr std::size_t kBtStateOffset = 3;
constexpr std::size_t kSpeakerBlockOffset = 142;
constexpr std::size_t kSpeakerDataOffset = 144;
constexpr int kSpeakerOpusBitrate = kSpeakerOpusSize * 8 * 100;
constexpr std::size_t kOutputFlag0Offset = 0;
constexpr std::size_t kOutputFlag1Offset = 1;
constexpr std::size_t kOutputHeadphoneVolumeOffset = 4;
constexpr std::size_t kOutputSpeakerVolumeOffset = 5;
constexpr std::size_t kOutputMicVolumeOffset = 6;
constexpr std::size_t kOutputAudioControlOffset = 7;
constexpr std::size_t kOutputMuteLightModeOffset = 8;
constexpr std::size_t kOutputPowerSaveControlOffset = 9;
constexpr std::size_t kOutputAudioControl2Offset = 37;
constexpr std::size_t kOutputLightBrightnessOffset =
    offsetof(vds_set_state_data, light_brightness);
constexpr std::size_t kOutputLedColorOffset =
    offsetof(vds_set_state_data, led_red);
constexpr std::uint8_t kBtHidpInputPrefix = 0xa1;
constexpr std::uint8_t kBtHidpOutputPrefix = VDS_BT_OUTPUT_PREFIX;
constexpr std::uint8_t kBtHidpFeaturePrefix = 0xa3;
constexpr std::uint8_t kBtHidpGetFeaturePrefix = 0x43;
constexpr std::uint8_t kBtHidpSetFeaturePrefix = 0x53;
constexpr std::size_t kBtInputReportIdOffset = 1;
constexpr std::size_t kBtInputFlagsOffset = 2;
constexpr std::size_t kBtInputUsbPayloadOffset = 3;
constexpr std::size_t kBtInputMinimumSize =
    kBtInputUsbPayloadOffset + VDS_USB_INPUT_PAYLOAD_SIZE;
constexpr std::uint8_t kBtInputFlagMicrophoneAudio = 0x02;
constexpr std::uint8_t kOutputFlag0HeadphoneVolumeEnable = 0x10;
constexpr std::uint8_t kOutputFlag0SpeakerVolumeEnable = 0x20;
constexpr std::uint8_t kOutputFlag0MicVolumeEnable = 0x40;
constexpr std::uint8_t kOutputFlag0AudioControlEnable = 0x80;
constexpr std::uint8_t kOutputFlag1MicMuteLedControlEnable = 0x01;
constexpr std::uint8_t kOutputFlag1PowerSaveControlEnable = 0x02;
constexpr std::uint8_t kOutputFlag1AudioControl2Enable = 0x80;
constexpr std::uint8_t kOutputHeadphoneVolumeMax = 0x7f;
constexpr std::uint8_t kOutputSpeakerVolumeMax = 0x64;
constexpr std::uint8_t kOutputPathHeadphones = 0x00;
constexpr std::uint8_t kOutputPathSpeaker = 0x30;
constexpr std::uint8_t kOutputSpeakerPreampGain = 0x02;
constexpr DsState kInitialDsState{
    0xfd, 0xf7, 0x00, 0x00, 0x7f, 0x64, 0xff, 0x09, 0x00, 0x0f, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x0a, 0x07, 0x00, 0x00, 0x02, 0x01, 0x00, 0xff, 0xd7, 0x00};
static_assert(kHapticsSampleSize % kSpeakerChannels == 0);

constexpr std::uint32_t crc32_table_entry(std::uint32_t index) {
  for (unsigned bit = 0; bit < 8; ++bit) {
    index = (index >> 1) ^ (0xedb88320u & (0u - (index & 1u)));
  }
  return index;
}

constexpr auto make_crc32_table() {
  std::array<std::uint32_t, 256> table{};
  for (std::uint32_t i = 0; i < table.size(); ++i) {
    table[i] = crc32_table_entry(i);
  }
  return table;
}

constexpr auto kCrc32Table = make_crc32_table();

std::int16_t read_le_s16(const std::uint8_t *data) {
  const auto lo = static_cast<std::uint16_t>(data[0]);
  const auto hi = static_cast<std::uint16_t>(data[1]) << 8;
  return static_cast<std::int16_t>(lo | hi);
}

// Modified Bessel function of the first kind, order 0 (Kaiser window).
double bessel_i0(double x) {
  double sum = 1.0;
  double term = 1.0;
  for (int k = 1; k < 32; ++k) {
    const double half = x / (2.0 * k);
    term *= half * half;
    sum += term;
  }
  return sum;
}

void opus_ctl_checked(int result, const char *name) {
  if (result != OPUS_OK) {
    throw std::runtime_error(std::string(name) +
                             " failed: " + opus_strerror(result));
  }
}

std::uint32_t crc32_seeded(std::span<const std::uint8_t> data,
                           std::uint32_t seed) {
  std::uint32_t crc = ~seed;

  for (const std::uint8_t byte : data) {
    crc = (crc >> 8) ^ kCrc32Table[(crc ^ byte) & 0xffu];
  }

  return ~crc;
}

std::uint32_t output_crc32(std::span<const std::uint8_t> data) {
  return crc32_seeded(data, kOutputCrcSeed);
}

std::uint32_t feature_crc32(std::span<const std::uint8_t> data) {
  return crc32_seeded(data, kFeatureCrcSeed);
}

void copy_state_bytes(DsState &state, std::span<const std::uint8_t> update,
                      std::size_t offset, std::size_t length) {
  std::copy(update.begin() + static_cast<std::ptrdiff_t>(offset),
            update.begin() + static_cast<std::ptrdiff_t>(offset + length),
            state.begin() + static_cast<std::ptrdiff_t>(offset));
}

void set_state_bit(std::uint8_t &byte, int bit, bool value) {
  const std::uint8_t mask = static_cast<std::uint8_t>(1u << bit);
  byte = static_cast<std::uint8_t>((byte & ~mask) |
                                   (value ? mask : std::uint8_t{0}));
}

std::uint8_t scale_light_component(std::uint8_t component,
                                   std::uint8_t brightness) {
  std::uint16_t scale = 255;
  switch (brightness) {
  case 1:
    scale = 128;
    break;
  case 2:
    scale = 64;
    break;
  default:
    break;
  }

  return static_cast<std::uint8_t>(
      (static_cast<std::uint16_t>(component) * scale + 127) / 255);
}

} // namespace

BandlimitedResampler::BandlimitedResampler(std::size_t input_frames,
                                           std::size_t output_frames,
                                           std::size_t half_taps,
                                           double cutoff, double kaiser_beta)
    : input_frames_(input_frames), output_frames_(output_frames),
      half_taps_(half_taps),
      history_(2 * half_taps * kSpeakerChannels, 0.0f),
      frames_(history_.size() + input_frames * kSpeakerChannels, 0.0f) {
  if (input_frames_ == output_frames_) {
    return; // 1:1, passed through untouched
  }
  const std::size_t taps = 2 * half_taps_;
  const double half = static_cast<double>(half_taps_);
  constexpr double kPi = 3.14159265358979323846;
  // Cutoff in units of the input Nyquist.
  const double cutoff_in =
      cutoff * std::min(1.0, static_cast<double>(output_frames_) /
                                 static_cast<double>(input_frames_));

  // Output k sits at input position k * input_frames / output_frames; its
  // fractional part is a multiple of gcd / output_frames, so gcd-spaced phases
  // cover every output.
  phase_divisor_ = std::gcd(input_frames_, output_frames_);
  const std::size_t phases = output_frames_ / phase_divisor_;
  taps_.resize(phases * taps);
  for (std::size_t phase = 0; phase < phases; ++phase) {
    const double fraction = static_cast<double>(phase * phase_divisor_) /
                            static_cast<double>(output_frames_);
    float *row = taps_.data() + phase * taps;
    double sum = 0.0;
    for (std::size_t tap = 0; tap < taps; ++tap) {
      const double x = static_cast<double>(tap) - (half - 1.0) - fraction;
      const double arg = cutoff_in * x;
      const double sinc =
          arg == 0.0 ? 1.0 : std::sin(kPi * arg) / (kPi * arg);
      const double r = x / half;
      const double window =
          std::abs(r) < 1.0
              ? bessel_i0(kaiser_beta * std::sqrt(1.0 - r * r)) /
                    bessel_i0(kaiser_beta)
              : 0.0;
      const double weight = cutoff_in * sinc * window;
      row[tap] = static_cast<float>(weight);
      sum += weight;
    }
    for (std::size_t tap = 0; tap < taps; ++tap) {
      row[tap] = static_cast<float>(row[tap] / sum); // unity DC gain
    }
  }
}

void BandlimitedResampler::process(std::span<const std::int16_t> input,
                                   std::span<float> output) {
  const std::size_t samples = input_frames_ * kSpeakerChannels;
  if (input_frames_ == output_frames_) {
    std::transform(input.begin(),
                   input.begin() + static_cast<std::ptrdiff_t>(samples),
                   output.begin(), [](std::int16_t sample) {
                     return static_cast<float>(sample);
                   });
    return;
  }

  const std::size_t taps = 2 * half_taps_;
  const std::size_t history_frames = history_.size() / kSpeakerChannels;
  // [history | this window], interleaved stereo.
  std::copy(history_.begin(), history_.end(), frames_.begin());
  std::transform(input.begin(),
                 input.begin() + static_cast<std::ptrdiff_t>(samples),
                 frames_.begin() + static_cast<std::ptrdiff_t>(history_.size()),
                 [](std::int16_t sample) { return static_cast<float>(sample); });

  for (std::size_t frame = 0; frame < output_frames_; ++frame) {
    const std::size_t position = frame * input_frames_;
    const std::size_t whole = position / output_frames_;
    const std::size_t phase = (position % output_frames_) / phase_divisor_;
    const float *row = taps_.data() + phase * taps;
    // First tap, delayed by half_taps so every tap is already buffered:
    // window index whole - 2 * half + 1, plus history.
    const std::size_t first = history_frames + whole + 1 - taps;
    for (std::size_t channel = 0; channel < kSpeakerChannels; ++channel) {
      float acc = 0.0f;
      for (std::size_t tap = 0; tap < taps; ++tap) {
        acc += row[tap] * frames_[(first + tap) * kSpeakerChannels + channel];
      }
      output[frame * kSpeakerChannels + channel] = acc;
    }
  }

  std::copy(frames_.end() - static_cast<std::ptrdiff_t>(history_.size()),
            frames_.end(), history_.begin());
}

// Cutoff ~20.2 kHz for 512 -> 480: flat to ~18 kHz, ~80 dB down by 22.5 kHz.
SpeakerResampler::SpeakerResampler(std::size_t input_frames)
    : resampler_(input_frames, kSpeakerOpusFrames, kSpeakerResamplerHalfTaps,
                 0.9, 8.0) {}

SpeakerOpusInput SpeakerResampler::process(std::span<const std::int16_t> input) {
  std::array<float, kSpeakerOpusFrames * kSpeakerChannels> resampled{};
  resampler_.process(input, resampled);
  SpeakerOpusInput output{};
  std::transform(resampled.begin(), resampled.end(), output.begin(),
                 [](float sample) {
                   return static_cast<std::int16_t>(
                       std::clamp(std::lrint(sample), -32768L, 32767L));
                 });
  return output;
}

// Cutoff ~1.27 kHz (0.85 of the 1.5 kHz output Nyquist).
HapticsResampler::HapticsResampler(std::size_t input_frames)
    : resampler_(input_frames, kHapticsFrames, kHapticsResamplerHalfTaps, 0.85,
                 7.0) {}

void HapticsResampler::set_gain_percent(std::uint16_t percent) {
  gain_ = static_cast<float>(std::min<std::uint16_t>(percent, 1000)) / 100.0f;
}

HapticsChunk HapticsResampler::process(std::span<const std::int16_t> input) {
  std::array<float, kHapticsFrames * kSpeakerChannels> resampled{};
  resampler_.process(input, resampled);
  HapticsChunk output{};
  for (std::size_t i = 0; i < resampled.size(); ++i) {
    const std::size_t channel = i % kSpeakerChannels;
    // 8-bit units; first-order error feedback moves the rounding error up
    // towards 1.5 kHz, where the actuator responds least.
    const float wanted = resampled[i] * gain_ / 256.0f - error_[channel];
    const float quantized = std::clamp(std::nearbyint(wanted), -128.0f, 127.0f);
    error_[channel] = std::clamp(quantized - wanted, -1.0f, 1.0f);
    output[i] = static_cast<std::int8_t>(quantized);
  }
  return output;
}

void SpeakerEnhancer::set_boost_db(float boost_db) {
  boost_db = std::clamp(boost_db, 0.0f, kMaxSpeakerBoostDb);
  if (boost_db == boost_db_) {
    return;
  }
  const bool was_active = boost_db_ > 0.0f;
  boost_db_ = boost_db;
  gain_ = std::pow(10.0f, boost_db_ / 20.0f);
  if (!was_active && boost_db_ > 0.0f) {
    reset();
  }
}

void SpeakerEnhancer::set_volume_percent(std::uint16_t percent) {
  const float level = static_cast<float>(std::min<std::uint16_t>(percent, 100)) /
                      100.0f;
  volume_target_ = level * level * level;
}

void SpeakerEnhancer::reset() {
  // 4th-order Butterworth high-pass at 200 Hz as two biquads (RBJ cookbook).
  constexpr double kPi = 3.14159265358979323846;
  constexpr double kCutoffHz = 200.0;
  constexpr std::array<double, 2> kQ = {0.54119610, 1.30656296};
  const double w0 = 2.0 * kPi * kCutoffHz / VDS_AUDIO_SAMPLE_RATE;
  for (std::size_t stage = 0; stage < high_pass_.size(); ++stage) {
    const double alpha = std::sin(w0) / (2.0 * kQ[stage]);
    const double cosw = std::cos(w0);
    const double a0 = 1.0 + alpha;
    Biquad &biquad = high_pass_[stage];
    biquad.b0 = static_cast<float>((1.0 + cosw) / 2.0 / a0);
    biquad.b1 = static_cast<float>(-(1.0 + cosw) / a0);
    biquad.b2 = biquad.b0;
    biquad.a1 = static_cast<float>(-2.0 * cosw / a0);
    biquad.a2 = static_cast<float>((1.0 - alpha) / a0);
    biquad.z1 = {};
    biquad.z2 = {};
  }
  limiter_gain_ = 1.0f;
  required_gain_.fill(1.0f);
  min_gain_.fill(1.0f);
  delay_.fill(0.0f);
  ring_pos_ = 0;
  delay_pos_ = 0;
}

float SpeakerEnhancer::Biquad::run(float x, std::size_t channel) {
  // Transposed direct form II.
  const float y = b0 * x + z1[channel];
  z1[channel] = b1 * x - a1 * y + z2[channel];
  z2[channel] = b2 * x - a2 * y;
  return y;
}

void SpeakerEnhancer::process(std::span<std::int16_t> samples,
                              std::size_t frames) {
  // ~10 ms ramp so slider moves don't click.
  const float ramp = 1.0f - std::exp(-1.0f / (0.010f * VDS_AUDIO_SAMPLE_RATE));
  if (boost_db_ <= 0.0f) {
    if (volume_ == 1.0f && volume_target_ == 1.0f) {
      return;
    }
    for (std::size_t frame = 0; frame < frames; ++frame) {
      volume_ += (volume_target_ - volume_) * ramp;
      for (std::size_t channel = 0; channel < kSpeakerChannels; ++channel) {
        auto &sample = samples[frame * kSpeakerChannels + channel];
        sample = static_cast<std::int16_t>(std::clamp(
            std::lrint(static_cast<float>(sample) * volume_), -32768L,
            32767L));
      }
    }
    if (std::abs(volume_ - volume_target_) < 1e-4f) {
      volume_ = volume_target_;
    }
    return;
  }
  // -1 dBFS leaves margin for the Opus encode.
  constexpr float kCeiling = 0.891f * 32767.0f;
  // 80 ms release back towards unity gain.
  const float release =
      1.0f - std::exp(-1.0f / (0.080f * VDS_AUDIO_SAMPLE_RATE));
  constexpr std::size_t kWindow = kSpeakerLimiterLookahead;

  for (std::size_t frame = 0; frame < frames; ++frame) {
    std::array<float, kSpeakerChannels> boosted{};
    float peak = 0.0f;
    for (std::size_t channel = 0; channel < kSpeakerChannels; ++channel) {
      float x = samples[frame * kSpeakerChannels + channel];
      for (auto &stage : high_pass_) {
        x = stage.run(x, channel);
      }
      boosted[channel] = x * gain_;
      peak = std::max(peak, std::abs(boosted[channel]));
    }

    // Look-ahead limiter: the minimum required gain over the window, then a
    // box average of that minimum, reaches each peak's required gain exactly
    // when the peak leaves the (window - 1)-frame delay line, so the output
    // never exceeds the ceiling and gain changes stay smooth.
    required_gain_[ring_pos_] = peak > kCeiling ? kCeiling / peak : 1.0f;
    min_gain_[ring_pos_] =
        *std::min_element(required_gain_.begin(), required_gain_.end());
    float average = 0.0f;
    for (const float value : min_gain_) {
      average += value;
    }
    average /= static_cast<float>(kWindow);
    ring_pos_ = (ring_pos_ + 1) % kWindow;
    limiter_gain_ = average < limiter_gain_
                        ? average
                        : limiter_gain_ + (average - limiter_gain_) * release;
    volume_ += (volume_target_ - volume_) * ramp;
    const float output_gain = limiter_gain_ * volume_;

    for (std::size_t channel = 0; channel < kSpeakerChannels; ++channel) {
      float &slot = delay_[delay_pos_ * kSpeakerChannels + channel];
      const float delayed = slot;
      slot = boosted[channel];
      samples[frame * kSpeakerChannels + channel] =
          static_cast<std::int16_t>(std::clamp(
              std::lrint(delayed * output_gain), -32768L, 32767L));
    }
    delay_pos_ = (delay_pos_ + 1) % (kWindow - 1);
  }
}

struct PcmAudioExtractor::SpeakerEncoder {
  explicit SpeakerEncoder(std::size_t input_frames) : resampler(input_frames) {
    int error = OPUS_OK;
    encoder = opus_encoder_create(VDS_AUDIO_SAMPLE_RATE, kSpeakerChannels,
                                  OPUS_APPLICATION_AUDIO, &error);
    if (!encoder || error != OPUS_OK) {
      throw std::runtime_error("opus_encoder_create failed: " +
                               std::string(opus_strerror(error)));
    }

    opus_ctl_checked(opus_encoder_ctl(encoder, OPUS_SET_VBR(0)),
                     "OPUS_SET_VBR");
    // The bitrate is fixed by the 0x36 speaker slot, so encoder effort is the
    // quality lever; on a PC complexity 10 costs well under 1 ms per 10 ms frame.
    opus_ctl_checked(opus_encoder_ctl(encoder, OPUS_SET_COMPLEXITY(10)),
                     "OPUS_SET_COMPLEXITY");
    opus_ctl_checked(opus_encoder_ctl(encoder, OPUS_SET_EXPERT_FRAME_DURATION(
                                                   OPUS_FRAMESIZE_10_MS)),
                     "OPUS_SET_EXPERT_FRAME_DURATION");
    opus_ctl_checked(
        opus_encoder_ctl(encoder, OPUS_SET_BITRATE(kSpeakerOpusBitrate)),
        "OPUS_SET_BITRATE");
  }

  ~SpeakerEncoder() {
    if (encoder) {
      opus_encoder_destroy(encoder);
    }
  }

  SpeakerEncoder(const SpeakerEncoder &) = delete;
  SpeakerEncoder &operator=(const SpeakerEncoder &) = delete;

  SpeakerChunk encode(std::span<std::int16_t> input) {
    SpeakerChunk output{};
    enhancer.process(input, input.size() / kSpeakerChannels);
    const auto opus_input = resampler.process(input);
    const int bytes =
        opus_encode(encoder, opus_input.data(), kSpeakerOpusFrames,
                    output.data(), static_cast<opus_int32>(output.size()));
    if (bytes < 0) {
      throw std::runtime_error("opus_encode failed: " +
                               std::string(opus_strerror(bytes)));
    }
    if (static_cast<std::size_t>(bytes) < output.size()) {
      std::fill(output.begin() + static_cast<std::ptrdiff_t>(bytes),
                output.end(), 0);
    }
    return output;
  }

  void set_mono(bool value) {
    if (value == mono) {
      return;
    }
    mono = value;
    opus_ctl_checked(opus_encoder_ctl(encoder, OPUS_SET_FORCE_CHANNELS(
                                                   mono ? 1 : OPUS_AUTO)),
                     "OPUS_SET_FORCE_CHANNELS");
  }

  SpeakerEnhancer enhancer;
  SpeakerResampler resampler;
  OpusEncoder *encoder = nullptr;
  bool mono = false;
};

PcmAudioExtractor::PcmAudioExtractor(std::size_t speaker_input_frames)
    : speaker_encoder_(std::make_unique<SpeakerEncoder>(speaker_input_frames)),
      haptics_resampler_(speaker_input_frames),
      speaker_input_frames_(speaker_input_frames) {
  if (speaker_input_frames_ == 0 ||
      speaker_input_frames_ > kSpeakerInputFrames ||
      speaker_input_frames_ % kHapticsDecimation != 0) {
    throw std::invalid_argument("unsupported speaker input frame window");
  }
}

PcmAudioExtractor::~PcmAudioExtractor() = default;

void PcmAudioExtractor::set_speaker_boost_db(float boost_db) {
  speaker_encoder_->enhancer.set_boost_db(boost_db);
}

void PcmAudioExtractor::set_speaker_volume_percent(std::uint16_t percent) {
  speaker_encoder_->enhancer.set_volume_percent(percent);
}

void PcmAudioExtractor::set_speaker_mono(bool mono) {
  speaker_encoder_->set_mono(mono);
}

void PcmAudioExtractor::set_haptics_gain_percent(std::uint16_t percent) {
  haptics_resampler_.set_gain_percent(percent);
}

PcmAudioExtractor::PcmAudioExtractor(PcmAudioExtractor &&) noexcept = default;

PcmAudioExtractor &
PcmAudioExtractor::operator=(PcmAudioExtractor &&) noexcept = default;

void fill_output_report_checksum(std::span<std::uint8_t> report) {
  if (report.size() < sizeof(std::uint32_t)) {
    throw std::invalid_argument("output report is too small for CRC");
  }
  const std::uint32_t crc =
      output_crc32(report.first(report.size() - sizeof(std::uint32_t)));
  const std::size_t offset = report.size() - sizeof(std::uint32_t);
  report[offset + 0] = static_cast<std::uint8_t>((crc >> 0) & 0xff);
  report[offset + 1] = static_cast<std::uint8_t>((crc >> 8) & 0xff);
  report[offset + 2] = static_cast<std::uint8_t>((crc >> 16) & 0xff);
  report[offset + 3] = static_cast<std::uint8_t>((crc >> 24) & 0xff);
}

void fill_feature_report_checksum(std::span<std::uint8_t> report) {
  if (report.size() < sizeof(std::uint32_t)) {
    throw std::invalid_argument("feature report is too small for CRC");
  }
  const std::uint32_t crc =
      feature_crc32(report.first(report.size() - sizeof(std::uint32_t)));
  const std::size_t offset = report.size() - sizeof(std::uint32_t);
  report[offset + 0] = static_cast<std::uint8_t>((crc >> 0) & 0xff);
  report[offset + 1] = static_cast<std::uint8_t>((crc >> 8) & 0xff);
  report[offset + 2] = static_cast<std::uint8_t>((crc >> 16) & 0xff);
  report[offset + 3] = static_cast<std::uint8_t>((crc >> 24) & 0xff);
}

std::vector<std::uint8_t>
hidp_output_packet(std::span<const std::uint8_t> report) {
  std::vector<std::uint8_t> packet;
  packet.reserve(report.size() + 1);
  packet.push_back(kBtHidpOutputPrefix);
  packet.insert(packet.end(), report.begin(), report.end());
  return packet;
}

std::vector<std::uint8_t> feature_get_packet(std::uint8_t report_id) {
  return {kBtHidpGetFeaturePrefix, report_id};
}

std::vector<std::uint8_t>
feature_set_packet(std::span<const std::uint8_t> report) {
  if (report.empty()) {
    return {};
  }

  std::vector<std::uint8_t> checked_report(report.begin(), report.end());
  if (checked_report.size() >= sizeof(std::uint32_t)) {
    fill_feature_report_checksum(checked_report);
  }

  std::vector<std::uint8_t> packet;
  packet.reserve(report.size() + 1);
  packet.push_back(kBtHidpSetFeaturePrefix);
  packet.insert(packet.end(), checked_report.begin(), checked_report.end());
  return packet;
}

std::optional<UsbInputReport>
bt_input_to_usb_input(std::span<const std::uint8_t> packet) {
  if (packet.size() < kBtInputMinimumSize || packet[0] != kBtHidpInputPrefix ||
      packet[kBtInputReportIdOffset] != VDS_BT_STATE_REPORT_ID) {
    return std::nullopt;
  }
  if ((packet[kBtInputFlagsOffset] & kBtInputFlagMicrophoneAudio) != 0) {
    return std::nullopt;
  }

  UsbInputReport report{};
  report[0] = VDS_USB_INPUT_REPORT_ID;
  std::copy(packet.begin() +
                static_cast<std::ptrdiff_t>(kBtInputUsbPayloadOffset),
            packet.begin() + static_cast<std::ptrdiff_t>(kBtInputMinimumSize),
            report.begin() + 1);
  return report;
}

std::optional<std::vector<std::uint8_t>>
bt_feature_to_usb_feature_reply(std::span<const std::uint8_t> packet) {
  if (packet.size() < 2 || packet[0] != kBtHidpFeaturePrefix) {
    return std::nullopt;
  }
  return std::vector<std::uint8_t>(packet.begin() + 1, packet.end());
}

namespace {

constexpr std::uint8_t kTriggerEffectOff = 0x05;
constexpr std::uint8_t kTriggerEffectFeedback = 0x21;
constexpr std::uint8_t kTriggerEffectSlope = 0x22;
constexpr std::uint8_t kTriggerEffectWeapon = 0x25;
constexpr std::uint8_t kTriggerEffectVibration = 0x26;

std::uint8_t trigger_strength_from_percent(std::uint8_t percent) {
  if (percent == 0) {
    return 0;
  }
  const std::uint8_t clamped = percent > 100 ? 100 : percent;
  const std::uint8_t strength = static_cast<std::uint8_t>((clamped * 8 + 99) / 100);
  return strength == 0 ? 1 : strength;
}

std::uint8_t trigger_position_from_percent(std::uint8_t percent) {
  const std::uint8_t clamped = percent > 100 ? 100 : percent;
  return static_cast<std::uint8_t>(
      std::min<std::uint16_t>(9, (static_cast<std::uint16_t>(clamped) + 5) / 10));
}

std::uint8_t trigger_frequency_from_percent(std::uint8_t percent) {
  const std::uint8_t clamped = percent > 100 ? 100 : percent;
  const std::uint8_t frequency = static_cast<std::uint8_t>(
      (static_cast<std::uint16_t>(clamped) * 28 + 50) / 100);
  return frequency == 0 ? 1 : frequency;
}

void set_trigger_off(std::span<std::uint8_t, kTriggerEffectSize> trigger) {
  std::fill(trigger.begin(), trigger.end(), 0);
  trigger[0] = kTriggerEffectOff;
}

void set_trigger_zones(std::span<std::uint8_t, kTriggerEffectSize> trigger,
                       std::uint8_t effect, std::uint8_t position,
                       std::uint8_t strength) {
  std::fill(trigger.begin(), trigger.end(), 0);
  position = position > 9 ? 9 : position;
  strength = strength > 8 ? 8 : strength;
  const std::uint8_t code = static_cast<std::uint8_t>((strength - 1) & 0x07);
  std::uint16_t active_zones = 0;
  std::uint32_t code_zones = 0;
  for (std::uint8_t zone = position; zone < 10; ++zone) {
    active_zones |= static_cast<std::uint16_t>(1u << zone);
    code_zones |= static_cast<std::uint32_t>(code) << (3 * zone);
  }
  trigger[0] = effect;
  trigger[1] = static_cast<std::uint8_t>(active_zones & 0xff);
  trigger[2] = static_cast<std::uint8_t>((active_zones >> 8) & 0xff);
  trigger[3] = static_cast<std::uint8_t>(code_zones & 0xff);
  trigger[4] = static_cast<std::uint8_t>((code_zones >> 8) & 0xff);
  trigger[5] = static_cast<std::uint8_t>((code_zones >> 16) & 0xff);
  trigger[6] = static_cast<std::uint8_t>((code_zones >> 24) & 0xff);
}

std::uint8_t scale_strength_code(std::uint8_t code, std::uint16_t percent) {
  // Codes store strength-1 (0..7 for strength 1..8).
  const unsigned strength = code + 1u;
  unsigned scaled = (strength * percent + 50) / 100;
  if (percent > 0 && scaled == 0) {
    scaled = 1;
  }
  if (scaled > 8) {
    scaled = 8;
  }
  return scaled == 0 ? 0 : static_cast<std::uint8_t>(scaled - 1);
}

} // namespace

void scale_trigger_effect(std::span<std::uint8_t, kTriggerEffectSize> trigger,
                          std::uint16_t percent) {
  if (percent == 100) {
    return;
  }
  switch (trigger[0]) {
  case kTriggerEffectFeedback:
  case kTriggerEffectVibration: {
    if (percent == 0) {
      set_trigger_off(trigger);
      return;
    }
    const std::uint16_t active_zones =
        static_cast<std::uint16_t>(trigger[1] | (trigger[2] << 8));
    std::uint32_t code_zones = static_cast<std::uint32_t>(trigger[3]) |
                               (static_cast<std::uint32_t>(trigger[4]) << 8) |
                               (static_cast<std::uint32_t>(trigger[5]) << 16) |
                               (static_cast<std::uint32_t>(trigger[6]) << 24);
    for (std::uint8_t zone = 0; zone < 10; ++zone) {
      if ((active_zones & (1u << zone)) == 0) {
        continue;
      }
      const std::uint8_t code = (code_zones >> (3 * zone)) & 0x07;
      code_zones &= ~(0x07u << (3 * zone));
      code_zones |= static_cast<std::uint32_t>(scale_strength_code(code, percent))
                    << (3 * zone);
    }
    trigger[3] = static_cast<std::uint8_t>(code_zones & 0xff);
    trigger[4] = static_cast<std::uint8_t>((code_zones >> 8) & 0xff);
    trigger[5] = static_cast<std::uint8_t>((code_zones >> 16) & 0xff);
    trigger[6] = static_cast<std::uint8_t>((code_zones >> 24) & 0xff);
    return;
  }
  case kTriggerEffectWeapon:
    if (percent == 0) {
      set_trigger_off(trigger);
      return;
    }
    trigger[3] = scale_strength_code(trigger[3] & 0x07, percent);
    return;
  case kTriggerEffectSlope: {
    if (percent == 0) {
      set_trigger_off(trigger);
      return;
    }
    // trigger[3] packs two 3-bit strength codes (stored as strength-1):
    // bits 0-2 start strength, bits 3-5 end strength.
    const std::uint8_t start_code =
        scale_strength_code(trigger[3] & 0x07, percent);
    const std::uint8_t end_code =
        scale_strength_code((trigger[3] >> 3) & 0x07, percent);
    trigger[3] = static_cast<std::uint8_t>((start_code & 0x07) |
                                           ((end_code & 0x07) << 3));
    return;
  }
  default:
    return;
  }
}

namespace {

std::uint8_t scale_byte(std::uint8_t value, std::uint16_t percent) {
  const unsigned scaled = (static_cast<unsigned>(value) * percent) / 100;
  return static_cast<std::uint8_t>(scaled > 255 ? 255 : scaled);
}

} // namespace

void encode_companion_trigger_effect(
    std::span<std::uint8_t, kTriggerEffectSize> trigger, std::uint8_t mode,
    std::uint8_t start_percent, std::uint8_t wall_percent,
    std::uint8_t force_percent) {
  const std::uint8_t strength = trigger_strength_from_percent(force_percent);
  std::uint8_t start_position = trigger_position_from_percent(start_percent);
  std::uint8_t wall_position = trigger_position_from_percent(wall_percent);
  const std::uint8_t frequency = trigger_frequency_from_percent(wall_percent);

  if (strength == 0) {
    set_trigger_off(trigger);
  } else if (mode == 1) {
    start_position = std::clamp<std::uint8_t>(start_position, 2, 7);
    wall_position = std::clamp<std::uint8_t>(
        wall_position, static_cast<std::uint8_t>(start_position + 1), 8);
    const std::uint16_t zones = static_cast<std::uint16_t>(
        (1u << start_position) | (1u << wall_position));
    std::fill(trigger.begin(), trigger.end(), 0);
    trigger[0] = kTriggerEffectWeapon;
    trigger[1] = static_cast<std::uint8_t>(zones & 0xff);
    trigger[2] = static_cast<std::uint8_t>((zones >> 8) & 0xff);
    trigger[3] = static_cast<std::uint8_t>((strength - 1) & 0x07);
  } else if (mode == 2) {
    set_trigger_zones(trigger, kTriggerEffectVibration, start_position,
                      strength);
    trigger[9] = frequency;
  } else {
    set_trigger_zones(trigger, kTriggerEffectFeedback, start_position,
                      strength);
  }
}

namespace {

// Packs per-zone strengths (percent 0-100 each) into the DualSense
// multi-position zone layout shared by feedback (0x21) and vibration (0x26).
// Returns false when every zone is inactive.
bool set_trigger_multi_zones(std::span<std::uint8_t, kTriggerEffectSize> trigger,
                             std::uint8_t effect,
                             const std::array<std::uint8_t, 10> &zone_percents) {
  std::uint16_t active_zones = 0;
  std::uint32_t code_zones = 0;
  for (std::uint8_t zone = 0; zone < 10; ++zone) {
    const std::uint8_t strength =
        trigger_strength_from_percent(zone_percents[zone]);
    if (strength == 0) {
      continue;
    }
    active_zones |= static_cast<std::uint16_t>(1u << zone);
    code_zones |= static_cast<std::uint32_t>((strength - 1) & 0x07)
                  << (3 * zone);
  }
  if (active_zones == 0) {
    return false;
  }
  std::fill(trigger.begin(), trigger.end(), 0);
  trigger[0] = effect;
  trigger[1] = static_cast<std::uint8_t>(active_zones & 0xff);
  trigger[2] = static_cast<std::uint8_t>((active_zones >> 8) & 0xff);
  trigger[3] = static_cast<std::uint8_t>(code_zones & 0xff);
  trigger[4] = static_cast<std::uint8_t>((code_zones >> 8) & 0xff);
  trigger[5] = static_cast<std::uint8_t>((code_zones >> 16) & 0xff);
  trigger[6] = static_cast<std::uint8_t>((code_zones >> 24) & 0xff);
  return true;
}

} // namespace

void encode_companion_trigger_effect_v2(
    std::span<std::uint8_t, kTriggerEffectSize> trigger,
    const CompanionTriggerEffect &effect) {
  switch (effect.mode) {
  case 4: // multi-feedback
    if (!set_trigger_multi_zones(trigger, kTriggerEffectFeedback,
                                 effect.zone_percents)) {
      set_trigger_off(trigger);
    }
    return;
  case 5: { // slope
    const std::uint8_t start_strength =
        trigger_strength_from_percent(effect.force_percent);
    const std::uint8_t end_strength =
        trigger_strength_from_percent(effect.end_force_percent);
    if (start_strength == 0 && end_strength == 0) {
      set_trigger_off(trigger);
      return;
    }
    std::uint8_t start_position =
        trigger_position_from_percent(effect.start_percent);
    std::uint8_t end_position =
        trigger_position_from_percent(effect.end_percent);
    start_position = std::min<std::uint8_t>(start_position, 8);
    end_position = std::clamp<std::uint8_t>(
        end_position, static_cast<std::uint8_t>(start_position + 1), 9);
    const std::uint16_t zones = static_cast<std::uint16_t>(
        (1u << start_position) | (1u << end_position));
    std::fill(trigger.begin(), trigger.end(), 0);
    trigger[0] = kTriggerEffectSlope;
    trigger[1] = static_cast<std::uint8_t>(zones & 0xff);
    trigger[2] = static_cast<std::uint8_t>((zones >> 8) & 0xff);
    // Strength code pair: bits 0-2 start strength, bits 3-5 end strength
    // (codes store strength-1, matching the zone packing convention).
    const std::uint8_t start_code =
        start_strength == 0 ? 0 : static_cast<std::uint8_t>(start_strength - 1);
    const std::uint8_t end_code =
        end_strength == 0 ? 0 : static_cast<std::uint8_t>(end_strength - 1);
    trigger[3] =
        static_cast<std::uint8_t>((start_code & 0x07) | ((end_code & 0x07) << 3));
    return;
  }
  case 6: // multi-vibration
    if (!set_trigger_multi_zones(trigger, kTriggerEffectVibration,
                                 effect.zone_percents)) {
      set_trigger_off(trigger);
      return;
    }
    trigger[9] = effect.frequency_hz == 0 ? 1 : effect.frequency_hz;
    return;
  case 3: // off
    set_trigger_off(trigger);
    return;
  default: // V1 modes: 0 feedback, 1 weapon, 2 vibration.
    encode_companion_trigger_effect(trigger, effect.mode, effect.start_percent,
                                    effect.wall_percent, effect.force_percent);
    if (effect.mode == 2 && effect.frequency_hz != 0 &&
        trigger[0] == kTriggerEffectVibration) {
      trigger[9] = effect.frequency_hz;
    }
    return;
  }
}

DsOutputState::DsOutputState()
    : state_(kInitialDsState), light_color_{state_[kOutputLedColorOffset + 0],
                                            state_[kOutputLedColorOffset + 1],
                                            state_[kOutputLedColorOffset + 2]},
      light_brightness_(state_[kOutputLightBrightnessOffset]) {
  recompute_effective_state();
}

void DsOutputState::set_companion_overrides(
    const DsCompanionOverrides &overrides) {
  companion_ = overrides;
  recompute_effective_state();
}

void DsOutputState::recompute_effective_state() {
  effective_state_ = state_;

  if (companion_.lightbar_override) {
    set_state_bit(effective_state_[1], 2, true); // allow_led_color
    std::array<std::uint8_t, 3> color{};
    if (companion_.lightbar_enabled) {
      for (std::size_t i = 0; i < 3; ++i) {
        color[i] = scale_byte(companion_.lightbar_color[i],
                              companion_.lightbar_brightness_percent);
      }
    }
    effective_state_[kOutputLedColorOffset + 0] = color[0];
    effective_state_[kOutputLedColorOffset + 1] = color[1];
    effective_state_[kOutputLedColorOffset + 2] = color[2];
  } else if (companion_.lightbar_brightness_percent != 100) {
    // Without an override, brightness still dims whatever color is active
    // (game-driven or the controller default).
    set_state_bit(effective_state_[1], 2, true); // allow_led_color
    for (std::size_t i = 0; i < 3; ++i) {
      effective_state_[kOutputLedColorOffset + i] =
          scale_byte(effective_state_[kOutputLedColorOffset + i],
                     companion_.lightbar_brightness_percent);
    }
  }

  // There is no microphone over vds, so the mute light would only mislead:
  // Linux hid-playstation toggles it on every Mute press. Keep it off.
  effective_state_[kOutputFlag1Offset] |= kOutputFlag1MicMuteLedControlEnable;
  effective_state_[kOutputMuteLightModeOffset] = 0;

  if (!companion_.player_led_enabled) {
    set_state_bit(effective_state_[1], 4, true); // allow_player_indicators
    effective_state_[kOutputLedColorOffset - 1] = 0;
  }

  const std::size_t rumble_right =
      offsetof(vds_set_state_data, rumble_emulation_right);
  if (companion_.test_rumble_active) {
    set_state_bit(effective_state_[0], 1, true); // use_rumble_not_haptics
    effective_state_[rumble_right] = companion_.test_rumble_power;
    effective_state_[rumble_right + 1] = companion_.test_rumble_power;
  } else if (companion_.classic_rumble_gain_percent != 100) {
    effective_state_[rumble_right] = scale_byte(
        effective_state_[rumble_right], companion_.classic_rumble_gain_percent);
    effective_state_[rumble_right + 1] =
        scale_byte(effective_state_[rumble_right + 1],
                   companion_.classic_rumble_gain_percent);
  }
  // Like the DS5 Bridge firmware: while the motors are driven (haptics select
  // + non-zero motors, from a game or the app's rumble test), pick the rumble
  // mode from the companion setting. A zero-motor stop is passed through
  // untouched so the game's handback to the actuator audio path still works.
  const bool haptics_select = (effective_state_[0] & 0x02) != 0;
  const bool motors_on = (effective_state_[rumble_right] |
                          effective_state_[rumble_right + 1]) != 0;
  if (haptics_select && motors_on) {
    constexpr std::uint8_t kFlag2ImprovedRumble = 0x04;
    constexpr std::uint8_t kFlag2RumbleNotHaptics2 = 0x08;
    if (companion_.classic_rumble_v1) {
      effective_state_[0] |= 0x01; // compatible vibration
      effective_state_[38] &= static_cast<std::uint8_t>(
          ~(kFlag2ImprovedRumble | kFlag2RumbleNotHaptics2));
    } else {
      effective_state_[0] &= static_cast<std::uint8_t>(~0x01);
      effective_state_[38] |= kFlag2ImprovedRumble | kFlag2RumbleNotHaptics2;
    }
  }

  const std::size_t right_ffb = offsetof(vds_set_state_data, right_trigger_ffb);
  const std::size_t left_ffb = offsetof(vds_set_state_data, left_trigger_ffb);
  if (companion_.right_trigger_active) {
    set_state_bit(effective_state_[0], 2, true); // allow_right_trigger_ffb
    std::copy(companion_.right_trigger.begin(), companion_.right_trigger.end(),
              effective_state_.begin() + right_ffb);
  } else {
    scale_trigger_effect(std::span<std::uint8_t, kTriggerEffectSize>(
                             effective_state_.data() + right_ffb,
                             kTriggerEffectSize),
                         companion_.trigger_intensity_percent);
  }
  if (companion_.left_trigger_active) {
    set_state_bit(effective_state_[0], 3, true); // allow_left_trigger_ffb
    std::copy(companion_.left_trigger.begin(), companion_.left_trigger.end(),
              effective_state_.begin() + left_ffb);
  } else {
    scale_trigger_effect(std::span<std::uint8_t, kTriggerEffectSize>(
                             effective_state_.data() + left_ffb,
                             kTriggerEffectSize),
                         companion_.trigger_intensity_percent);
  }

  // companion_.speaker_volume_percent is applied to the audio itself
  // (SpeakerEnhancer): the hardware volume register is far from linear, so
  // scaling it made small slider moves very quiet.
  // Like the DS5 Bridge firmware, only own the speaker path and preamp gain
  // while the speaker stream is active; otherwise leave the game's values.
  if (audio_out_stream_active_) {
    effective_state_[kOutputFlag0Offset] |=
        kOutputFlag0AudioControlEnable | kOutputFlag0SpeakerVolumeEnable;
    effective_state_[kOutputAudioControlOffset] = kOutputPathSpeaker;
    effective_state_[kOutputFlag1Offset] |= kOutputFlag1AudioControl2Enable;
    effective_state_[kOutputAudioControl2Offset] =
        static_cast<std::uint8_t>(
            (effective_state_[kOutputAudioControl2Offset] & 0xf8) |
            std::clamp<std::uint8_t>(companion_.speaker_gain_level, 1, 7));
  }
}

DsOutputState::HostState DsOutputState::host_state() const {
  return {state_, light_color_, light_brightness_, emulate_light_brightness_};
}

void DsOutputState::restore_host_state(const HostState &snapshot) {
  state_ = snapshot.state;
  light_color_ = snapshot.light_color;
  light_brightness_ = snapshot.light_brightness;
  emulate_light_brightness_ = snapshot.emulate_light_brightness;
  // Re-applies the speaker routing bits for the stream as it is now, and
  // recomputes the effective state.
  set_audio_out_stream_active(audio_out_stream_active_);
}

BtInitReport DsOutputState::build_bt_init_report() {
  BtInitReport report{};
  report[0] = 0x32;
  report[1] = 0x10;
  report[2] = 0x10 | (1 << 7);
  report[3] = kDsStateSize;
  std::copy(effective_state_.begin(), effective_state_.end(),
            report.begin() + 4);
  fill_output_report_checksum(report);
  return report;
}

bool DsOutputState::apply_usb_output_report(
    std::span<const std::uint8_t> report) {
  std::span<const std::uint8_t> update;

  if (report.size() >= 1 + kSetStateSize &&
      report[0] == VDS_USB_OUTPUT_REPORT_ID) {
    update = report.subspan(1, kSetStateSize);
  } else if (report.size() == kSetStateSize) {
    update = report.first(kSetStateSize);
  } else {
    return false;
  }

  vds_set_state_data decoded{};
  std::memcpy(&decoded, update.data(), sizeof(decoded));

  set_state_bit(state_[0], 0, decoded.enable_rumble_emulation);
  set_state_bit(state_[0], 1, decoded.use_rumble_not_haptics);
  set_state_bit(state_[38], 2, decoded.enable_improved_rumble_emulation);

  if (decoded.use_rumble_not_haptics || decoded.enable_rumble_emulation) {
    copy_state_bytes(state_, update,
                     offsetof(vds_set_state_data, rumble_emulation_right), 2);
  }
  if (decoded.allow_headphone_volume) {
    state_[kOutputFlag0Offset] |= kOutputFlag0HeadphoneVolumeEnable;
    copy_state_bytes(state_, update,
                     offsetof(vds_set_state_data, volume_headphones),
                     sizeof(decoded.volume_headphones));
  }
  if (decoded.allow_speaker_volume) {
    state_[kOutputFlag0Offset] |= kOutputFlag0SpeakerVolumeEnable;
    copy_state_bytes(state_, update,
                     offsetof(vds_set_state_data, volume_speaker),
                     sizeof(decoded.volume_speaker));
  }
  if (decoded.allow_audio_control) {
    state_[kOutputFlag0Offset] |= kOutputFlag0AudioControlEnable;
    copy_state_bytes(state_, update, kOutputAudioControlOffset, 1);
  }
  if (decoded.allow_audio_control2) {
    state_[kOutputFlag1Offset] |= kOutputFlag1AudioControl2Enable;
    copy_state_bytes(state_, update, kOutputAudioControl2Offset, 1);
  }
  if (decoded.allow_speaker_volume && decoded.volume_speaker != 0) {
    state_[kOutputFlag0Offset] |=
        kOutputFlag0AudioControlEnable | kOutputFlag0SpeakerVolumeEnable;
    state_[kOutputFlag1Offset] |= kOutputFlag1AudioControl2Enable;
    state_[kOutputAudioControlOffset] = kOutputPathSpeaker;
    state_[kOutputAudioControl2Offset] = kOutputSpeakerPreampGain;
  }
  if (decoded.allow_mute_light) {
    copy_state_bytes(state_, update,
                     offsetof(vds_set_state_data, mute_light_mode),
                     sizeof(decoded.mute_light_mode));
  }
  if (decoded.allow_right_trigger_ffb) {
    copy_state_bytes(state_, update,
                     offsetof(vds_set_state_data, right_trigger_ffb),
                     sizeof(decoded.right_trigger_ffb));
  }
  if (decoded.allow_left_trigger_ffb) {
    copy_state_bytes(state_, update,
                     offsetof(vds_set_state_data, left_trigger_ffb),
                     sizeof(decoded.left_trigger_ffb));
  }
  if (decoded.allow_color_light_fade_animation) {
    copy_state_bytes(state_, update,
                     offsetof(vds_set_state_data, light_fade_animation),
                     sizeof(decoded.light_fade_animation));
  }
  if (decoded.allow_light_brightness_change) {
    copy_state_bytes(state_, update, kOutputLightBrightnessOffset,
                     sizeof(decoded.light_brightness));
    light_brightness_ = decoded.light_brightness;
    emulate_light_brightness_ = true;
  }
  if (decoded.allow_player_indicators) {
    // player_light* is a bitfield byte immediately before led_red.
    copy_state_bytes(state_, update, kOutputLedColorOffset - 1, 1);
  }
  if (decoded.allow_led_color) {
    light_color_ = {update[kOutputLedColorOffset + 0],
                    update[kOutputLedColorOffset + 1],
                    update[kOutputLedColorOffset + 2]};
    copy_state_bytes(state_, update, kOutputLedColorOffset, 3);
  }
  if (emulate_light_brightness_ &&
      (decoded.allow_light_brightness_change || decoded.allow_led_color)) {
    state_[kOutputLedColorOffset + 0] =
        scale_light_component(light_color_[0], light_brightness_);
    state_[kOutputLedColorOffset + 1] =
        scale_light_component(light_color_[1], light_brightness_);
    state_[kOutputLedColorOffset + 2] =
        scale_light_component(light_color_[2], light_brightness_);
  }
  recompute_effective_state();
  return true;
}

void DsOutputState::set_audio_out_stream_active(bool active) {
  audio_out_stream_active_ = active;
  state_[kOutputFlag0Offset] |= kOutputFlag0AudioControlEnable;
  state_[kOutputHeadphoneVolumeOffset] = kOutputHeadphoneVolumeMax;

  if (active) {
    state_[kOutputFlag0Offset] |= kOutputFlag0SpeakerVolumeEnable;
    state_[kOutputFlag1Offset] |= kOutputFlag1AudioControl2Enable;
    state_[kOutputSpeakerVolumeOffset] = kOutputSpeakerVolumeMax;
    state_[kOutputAudioControlOffset] = kOutputPathSpeaker;
    state_[kOutputAudioControl2Offset] = kOutputSpeakerPreampGain;
    recompute_effective_state();
    return;
  }

  state_[kOutputFlag0Offset] &= ~kOutputFlag0SpeakerVolumeEnable;
  state_[kOutputFlag1Offset] &= ~kOutputFlag1AudioControl2Enable;
  state_[kOutputSpeakerVolumeOffset] = 0;
  state_[kOutputAudioControlOffset] = kOutputPathHeadphones;
  state_[kOutputAudioControl2Offset] = 0;
  recompute_effective_state();
}

BtStateReport DsOutputState::build_bt_state_report() {
  BtStateReport report{};
  report[0] = VDS_BT_STATE_REPORT_ID;
  report[1] = report_sequence_ << 4;
  report_sequence_ = (report_sequence_ + 1) & 0x0f;
  report[2] = 0x10;
  std::copy(effective_state_.begin(),
            effective_state_.begin() + kSetStateSize,
            report.begin() + kBtStateOffset);
  fill_output_report_checksum(report);
  return report;
}

DsState state_after_audio_packet(const std::optional<DsState> &previous,
                                 const DsState &state) {
  constexpr std::uint8_t kFlag0MicMask = kOutputFlag0MicVolumeEnable;
  constexpr std::uint8_t kFlag1MicMask = kOutputFlag1MicMuteLedControlEnable |
                                         kOutputFlag1PowerSaveControlEnable;
  const DsState prior = previous.value_or(DsState{});
  DsState result = state;
  result[kOutputFlag0Offset] = static_cast<std::uint8_t>(
      (state[kOutputFlag0Offset] & ~kFlag0MicMask) |
      (prior[kOutputFlag0Offset] & kFlag0MicMask));
  result[kOutputFlag1Offset] = static_cast<std::uint8_t>(
      (state[kOutputFlag1Offset] & ~kFlag1MicMask) |
      (prior[kOutputFlag1Offset] & kFlag1MicMask));
  result[kOutputMicVolumeOffset] = prior[kOutputMicVolumeOffset];
  result[kOutputMuteLightModeOffset] = prior[kOutputMuteLightModeOffset];
  result[kOutputPowerSaveControlOffset] = prior[kOutputPowerSaveControlOffset];
  return result;
}

bool AudioQueueTrimmer::should_trim(std::size_t depth) {
  window_min_depth_ = std::min(window_min_depth_, depth);
  if (++window_packets_ < kWindowPackets) {
    return false;
  }
  const bool trim = window_min_depth_ > kTargetMinDepth;
  reset();
  return trim;
}

void AudioQueueTrimmer::reset() {
  window_packets_ = 0;
  window_min_depth_ = SIZE_MAX;
}

namespace {

// Buffer length (3 kHz samples) that covers `us`, rounded up.
std::int64_t haptics_samples_for_us(std::int64_t us) {
  return (us * VDS_HAPTICS_SAMPLE_RATE + 999999) / 1000000;
}

} // namespace

HapticsBufferTuner::HapticsBufferTuner(std::uint8_t initial)
    : length_(std::clamp(initial, kMinLength, kMaxLength)) {}

std::uint8_t HapticsBufferTuner::on_send(std::int64_t gap_us) {
  const auto needed = static_cast<std::uint8_t>(std::clamp<std::int64_t>(
      haptics_samples_for_us(gap_us + kHeadroomUs), 0, 255));
  if (filled_ == kWindowPackets) {
    --counts_[needed_[next_]];
  } else {
    ++filled_;
  }
  needed_[next_] = needed;
  ++counts_[needed];
  next_ = (next_ + 1) % kWindowPackets;

  if (filled_ < kCoveredRank) {
    return length_;
  }
  // The Nth-largest need in the window.
  std::size_t seen = 0;
  for (std::size_t length = counts_.size(); length-- > 0;) {
    seen += counts_[length];
    if (seen >= kCoveredRank) {
      length_ = static_cast<std::uint8_t>(std::clamp<std::size_t>(
          length, kMinLength, kMaxLength));
      break;
    }
  }
  return length_;
}

void HapticsPacketBuilder::write_audio_blocks(
    std::span<std::uint8_t> packet, std::uint8_t report_id,
    std::span<const std::int8_t, kHapticsSampleSize> haptics,
    std::span<const std::uint8_t, kDsStateSize> state) {
  packet[0] = report_id;
  packet[1] = report_sequence_ << 4;
  report_sequence_ = (report_sequence_ + 1) & 0x0f;
  packet[2] = 0x11 | (1 << 7);
  packet[3] = 7;
  packet[4] = 0xfe;
  packet[5] = buffer_length_;
  packet[6] = buffer_length_;
  packet[7] = buffer_length_;
  packet[8] = buffer_length_;
  packet[9] = buffer_length_;
  packet[10] = packet_sequence_++;
  packet[11] = 0x10 | (1 << 7);
  packet[12] = kDsStateSize;
  DsState audio_state;
  std::copy(state.begin(), state.end(), audio_state.begin());
  audio_state[kOutputFlag0Offset] &= ~kOutputFlag0MicVolumeEnable;
  // Leave power-save (including mic mute) untouched: writing it here with the
  // mic-mute bit cleared would unmute the mic on every audio packet.
  audio_state[kOutputFlag1Offset] &=
      ~(kOutputFlag1MicMuteLedControlEnable |
        kOutputFlag1PowerSaveControlEnable);
  audio_state[kOutputPowerSaveControlOffset] = 0;
  audio_state[kOutputMicVolumeOffset] = 0;
  audio_state[kOutputMuteLightModeOffset] = 0;
  // Keep the companion/game speaker volume; only default to max when nothing
  // has set it.
  if ((audio_state[kOutputFlag0Offset] & kOutputFlag0SpeakerVolumeEnable) ==
      0) {
    audio_state[kOutputSpeakerVolumeOffset] = kOutputSpeakerVolumeMax;
  }
  audio_state[kOutputFlag0Offset] |=
      kOutputFlag0AudioControlEnable | kOutputFlag0SpeakerVolumeEnable;
  audio_state[kOutputFlag1Offset] |= kOutputFlag1AudioControl2Enable;
  audio_state[kOutputHeadphoneVolumeOffset] = kOutputHeadphoneVolumeMax;
  audio_state[kOutputAudioControlOffset] = kOutputPathSpeaker;
  std::copy(audio_state.begin(), audio_state.end(), packet.begin() + 13);
  packet[76] = 0x12 | (1 << 7);
  packet[77] = kHapticsSampleSize;
  std::memcpy(packet.data() + 78, haptics.data(), haptics.size());
}

BtReport HapticsPacketBuilder::build_packet(
    std::span<const std::int8_t, kHapticsSampleSize> haptics,
    std::span<const std::uint8_t, kSpeakerOpusSize> speaker,
    std::span<const std::uint8_t, kDsStateSize> state) {
  BtReport packet{};
  write_audio_blocks(packet, VDS_BT_HAPTICS_REPORT_ID, haptics, state);
  packet[kSpeakerBlockOffset] = 0x13 | (1 << 7);
  packet[kSpeakerBlockOffset + 1] = static_cast<std::uint8_t>(kSpeakerOpusSize);
  std::memcpy(packet.data() + kSpeakerDataOffset, speaker.data(),
              speaker.size());
  fill_output_report_checksum(packet);
  return packet;
}

BtHapticsOnlyReport HapticsPacketBuilder::build_haptics_only_packet(
    std::span<const std::int8_t, kHapticsSampleSize> haptics,
    std::span<const std::uint8_t, kDsStateSize> state) {
  static_assert(kSpeakerBlockOffset + sizeof(std::uint32_t) <=
                kBtHapticsOnlyReportSize);
  BtHapticsOnlyReport packet{};
  write_audio_blocks(packet, kBtHapticsOnlyReportId, haptics, state);
  fill_output_report_checksum(packet);
  return packet;
}

std::vector<AudioChunk>
PcmAudioExtractor::push_usb_audio(std::span<const std::uint8_t> pcm_bytes) {
  std::vector<AudioChunk> chunks;
  chunks.reserve(pcm_bytes.size() / (kPcmFrameSize * speaker_input_frames_) +
                 1);

  const auto consume_frame = [this, &chunks](const std::uint8_t *base) {
    std::array<std::int16_t, kPcmChannels> samples{};
    for (std::size_t channel = 0; channel < kPcmChannels; ++channel) {
      samples[channel] = read_le_s16(base + channel * kPcmSampleSize);
      chunk_has_signal_ |= samples[channel] != 0;
    }

    speaker_input_[speaker_frame_pos_ * kSpeakerChannels + 0] = samples[0];
    speaker_input_[speaker_frame_pos_ * kSpeakerChannels + 1] = samples[1];
    haptics_input_[speaker_frame_pos_ * kSpeakerChannels + 0] = samples[2];
    haptics_input_[speaker_frame_pos_ * kSpeakerChannels + 1] = samples[3];
    chunk_has_haptics_signal_ |= samples[2] != 0 || samples[3] != 0;
    chunk_has_speaker_signal_ |= samples[0] != 0 || samples[1] != 0;
    ++speaker_frame_pos_;

    if (speaker_frame_pos_ != speaker_input_frames_) {
      return;
    }

    speaker_frame_pos_ = 0;
    AudioChunk chunk{};
    chunk.haptics = haptics_resampler_.process(std::span<const std::int16_t>(
        haptics_input_.data(), speaker_input_frames_ * kSpeakerChannels));
    chunk.speaker = speaker_encoder_->encode(std::span<std::int16_t>(
        speaker_input_.data(), speaker_input_frames_ * kSpeakerChannels));
    chunk.has_signal = chunk_has_signal_;
    chunk.has_haptics_signal = chunk_has_haptics_signal_;
    chunk.has_speaker_signal = chunk_has_speaker_signal_;
    chunks.push_back(chunk);
    chunk_has_signal_ = false;
    chunk_has_haptics_signal_ = false;
    chunk_has_speaker_signal_ = false;
  };

  while (!pcm_bytes.empty()) {
    if (pending_frame_pos_ == 0 && pcm_bytes.size() >= kPcmFrameSize) {
      const std::size_t frames = pcm_bytes.size() / kPcmFrameSize;
      for (std::size_t frame = 0; frame < frames; ++frame) {
        consume_frame(pcm_bytes.data() + frame * kPcmFrameSize);
      }
      pcm_bytes = pcm_bytes.subspan(frames * kPcmFrameSize);
      continue;
    }

    const std::size_t copy_size =
        std::min(pcm_bytes.size(), kPcmFrameSize - pending_frame_pos_);
    std::copy_n(pcm_bytes.begin(), copy_size,
                pending_frame_.begin() +
                    static_cast<std::ptrdiff_t>(pending_frame_pos_));
    pending_frame_pos_ += copy_size;
    pcm_bytes = pcm_bytes.subspan(copy_size);
    if (pending_frame_pos_ == kPcmFrameSize) {
      consume_frame(pending_frame_.data());
      pending_frame_pos_ = 0;
    }
  }

  return chunks;
}

std::vector<std::uint8_t> frame_bytes(std::uint16_t type,
                                      std::span<const std::uint8_t> payload) {
  if (payload.size() > VDS_FRAME_MAX_PAYLOAD) {
    throw std::invalid_argument("frame payload exceeds VDS_FRAME_MAX_PAYLOAD");
  }

  vds_frame_header header{};
  header.type = type;
  header.length = static_cast<std::uint32_t>(payload.size());

  std::vector<std::uint8_t> bytes(sizeof(header) + payload.size());
  std::memcpy(bytes.data(), &header, sizeof(header));
  std::memcpy(bytes.data() + sizeof(header), payload.data(), payload.size());
  return bytes;
}

std::string frame_type_name(std::uint16_t type) {
  switch (type) {
  case VDS_FRAME_STATUS:
    return "STATUS";
  case VDS_FRAME_USB_HID_OUT:
    return "USB_HID_OUT";
  case VDS_FRAME_USB_FEATURE_GET:
    return "USB_FEATURE_GET";
  case VDS_FRAME_USB_FEATURE_SET:
    return "USB_FEATURE_SET";
  case VDS_FRAME_USB_AUDIO_OUT:
    return "USB_AUDIO_OUT";
  case VDS_FRAME_USB_HID_IN:
    return "USB_HID_IN";
  case VDS_FRAME_USB_FEATURE_REPLY:
    return "USB_FEATURE_REPLY";
  case VDS_FRAME_USB_INTERFACE:
    return "USB_INTERFACE";
  case VDS_FRAME_BT_CONTROL_PACKET:
    return "BT_CONTROL_PACKET";
  case VDS_FRAME_BT_INTERRUPT_PACKET:
    return "BT_INTERRUPT_PACKET";
  default:
    return "UNKNOWN";
  }
}

} // namespace vds
