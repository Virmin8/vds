// SPDX-License-Identifier: MIT
// Copyright (C) 2026 Jihong Min <hurryman2212@gmail.com>
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include "vds/ds5_protocol.h"

namespace vds {

constexpr std::size_t kBtHapticsReportSize = VDS_BT_HAPTICS_REPORT_SIZE;
constexpr std::size_t kBtInitReportSize = 142;
constexpr std::size_t kBtStateReportSize = VDS_BT_STATE_REPORT_SIZE;
constexpr std::size_t kHapticsSampleSize = VDS_HAPTICS_SAMPLE_SIZE;
constexpr std::size_t kUsbInputReportSize = VDS_USB_INPUT_REPORT_SIZE;
constexpr std::size_t kDsStateSize = 63;
constexpr std::size_t kSpeakerChannels = 2;
constexpr std::size_t kSpeakerInputFrames = 512;
constexpr std::size_t kSpeakerOpusFrames = 480;
constexpr std::size_t kSpeakerOpusSize = 200;

using BtReport = std::array<std::uint8_t, kBtHapticsReportSize>;
// 0x33: the 0x36 layout without the speaker block, for when the speaker is
// silent; about half the Bluetooth airtime per packet.
constexpr std::uint8_t kBtHapticsOnlyReportId = 0x33;
constexpr std::size_t kBtHapticsOnlyReportSize = 206;
using BtHapticsOnlyReport = std::array<std::uint8_t, kBtHapticsOnlyReportSize>;
using BtInitReport = std::array<std::uint8_t, kBtInitReportSize>;
using BtStateReport = std::array<std::uint8_t, kBtStateReportSize>;
using HapticsChunk = std::array<std::int8_t, kHapticsSampleSize>;
using SpeakerInput =
    std::array<std::int16_t, kSpeakerInputFrames * kSpeakerChannels>;
using SpeakerChunk = std::array<std::uint8_t, kSpeakerOpusSize>;
using SpeakerOpusInput =
    std::array<std::int16_t, kSpeakerOpusFrames * kSpeakerChannels>;
using DsState = std::array<std::uint8_t, kDsStateSize>;

/*
 * Streaming band-limited stereo resampler (windowed-sinc polyphase FIR) from
 * input_frames to output_frames per window. History carries across windows so
 * window joins stay continuous; the output lags the input by half_taps input
 * frames. Equal frame counts pass through untouched.
 */
class BandlimitedResampler {
public:
  // cutoff: fraction of the lower of the two Nyquist rates.
  BandlimitedResampler(std::size_t input_frames, std::size_t output_frames,
                       std::size_t half_taps, double cutoff,
                       double kaiser_beta);

  // `input`: input_frames interleaved stereo frames; `output`: output_frames.
  void process(std::span<const std::int16_t> input, std::span<float> output);

private:
  std::size_t input_frames_;
  std::size_t output_frames_;
  std::size_t half_taps_;
  std::size_t phase_divisor_ = 1;
  std::vector<float> taps_;    // [phase][tap]
  std::vector<float> history_; // interleaved stereo, 2 * half_taps frames
  std::vector<float> frames_;  // scratch: history + window
};

// Speaker: input window -> one 480-frame Opus block, flat to ~18 kHz.
constexpr std::size_t kSpeakerResamplerHalfTaps = 32; // ~0.67 ms latency

class SpeakerResampler {
public:
  explicit SpeakerResampler(std::size_t input_frames);

  // `input` holds input_frames interleaved stereo frames.
  SpeakerOpusInput process(std::span<const std::int16_t> input);

private:
  BandlimitedResampler resampler_;
};

/*
 * HD haptics: input window -> 32 frames at 3 kHz, 8-bit. Flat to ~850 Hz with
 * content above 1.5 kHz filtered out rather than aliased into the actuator
 * band; gain is applied before quantization, and first-order noise shaping
 * keeps detail below one 8-bit step (silence stays exactly zero).
 */
constexpr std::size_t kHapticsFrames = kHapticsSampleSize / kSpeakerChannels;
constexpr std::size_t kHapticsResamplerHalfTaps = 128; // ~2.7 ms latency

class HapticsResampler {
public:
  explicit HapticsResampler(std::size_t input_frames);

  void set_gain_percent(std::uint16_t percent);
  HapticsChunk process(std::span<const std::int16_t> input);

private:
  BandlimitedResampler resampler_;
  float gain_ = 1.0f;
  std::array<float, kSpeakerChannels> error_{};
};

/*
 * Loudness processing for the controller's built-in speaker (not headphones):
 * a 4th-order high-pass drops bass the tiny driver cannot reproduce, then a
 * boost and a look-ahead peak limiter raise the level without clipping.
 * Adds kSpeakerLimiterLookahead - 1 frames of latency (~1 ms) while active.
 */
// Default to the full boost: the limiter keeps it clip-free, the preamp gain
// can stay at its lowest level, and volume is turned down on the host.
constexpr float kDefaultSpeakerBoostDb = 18.0f;
constexpr float kMaxSpeakerBoostDb = 18.0f;
constexpr std::size_t kSpeakerLimiterLookahead = 48;

class SpeakerEnhancer {
public:
  // 0 (or less) bypasses all processing.
  void set_boost_db(float boost_db);
  float boost_db() const { return boost_db_; }

  // Companion speaker volume, applied digitally after the limiter on a cubic
  // (perceptual) curve. The controller's hardware volume register responds
  // far from linearly, so it is left alone.
  void set_volume_percent(std::uint16_t percent);

  // In place, `frames` interleaved stereo frames.
  void process(std::span<std::int16_t> samples, std::size_t frames);

private:
  struct Biquad {
    float b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0;
    std::array<float, 2> z1{}, z2{}; // per channel
    float run(float x, std::size_t channel);
  };
  void reset();

  float boost_db_ = 0.0f;
  float gain_ = 1.0f;
  std::array<Biquad, 2> high_pass_{};
  float limiter_gain_ = 1.0f;
  std::size_t ring_pos_ = 0;
  std::array<float, kSpeakerLimiterLookahead> required_gain_{}; // min window
  std::array<float, kSpeakerLimiterLookahead> min_gain_{};      // box window
  std::array<float, (kSpeakerLimiterLookahead - 1) * 2> delay_{};
  std::size_t delay_pos_ = 0;
  float volume_target_ = 1.0f;
  float volume_ = 1.0f; // ramps towards volume_target_
};
using UsbInputReport = std::array<std::uint8_t, kUsbInputReportSize>;

struct AudioChunk {
  HapticsChunk haptics;
  SpeakerChunk speaker;
  bool has_signal = false;
  bool has_haptics_signal = false;
  bool has_speaker_signal = false;
};

void fill_output_report_checksum(std::span<std::uint8_t> report);
void fill_feature_report_checksum(std::span<std::uint8_t> report);
std::vector<std::uint8_t>
hidp_output_packet(std::span<const std::uint8_t> report);
std::vector<std::uint8_t> feature_get_packet(std::uint8_t report_id);
std::vector<std::uint8_t>
feature_set_packet(std::span<const std::uint8_t> report);
std::optional<UsbInputReport>
bt_input_to_usb_input(std::span<const std::uint8_t> packet);
std::optional<std::vector<std::uint8_t>>
bt_feature_to_usb_feature_reply(std::span<const std::uint8_t> packet);

class HapticsPacketBuilder {
public:
  // Controller audio buffer length (packet header bytes 5-9); lower values
  // cut speaker/haptics latency, higher values ride out Bluetooth jitter.
  void set_buffer_length(std::uint8_t length) { buffer_length_ = length; }

  BtReport
  build_packet(std::span<const std::int8_t, kHapticsSampleSize> haptics,
               std::span<const std::uint8_t, kSpeakerOpusSize> speaker,
               std::span<const std::uint8_t, kDsStateSize> state);
  // Same haptics and state, no speaker audio (report 0x33).
  BtHapticsOnlyReport build_haptics_only_packet(
      std::span<const std::int8_t, kHapticsSampleSize> haptics,
      std::span<const std::uint8_t, kDsStateSize> state);

private:
  // Report ID, sequence, the 0x11 control, 0x10 state and 0x12 haptics blocks
  // shared by both report sizes.
  void write_audio_blocks(std::span<std::uint8_t> packet, std::uint8_t report_id,
                          std::span<const std::int8_t, kHapticsSampleSize> haptics,
                          std::span<const std::uint8_t, kDsStateSize> state);

  std::uint8_t report_sequence_ = 0;
  std::uint8_t packet_sequence_ = 0;
  std::uint8_t buffer_length_ = 64;
};

// Keeps the queue between USB audio and Bluetooth sends at one spare chunk.
// Input and output run at the same rate, so a backlog left by a burst would
// otherwise add its latency for the rest of the stream.
class AudioQueueTrimmer {
public:
  // Packets per window (~2 s); trims at most once per window.
  static constexpr std::size_t kWindowPackets = 188;
  // The chunk being sent plus one spare: with none, USB bursts that do not
  // line up with the send times keep running the queue dry.
  static constexpr std::size_t kTargetMinDepth = 2;

  // Call before each send with the queue depth, including the chunk about to
  // be sent. True when the queue never fell to the target over the window, so
  // one chunk should be dropped.
  bool should_trim(std::size_t depth);
  // The queue ran dry and will re-prime.
  void reset();

private:
  std::size_t window_packets_ = 0;
  std::size_t window_min_depth_ = SIZE_MAX;
};

// Picks the controller's audio buffer (packet buffer_length, 3 kHz samples)
// from the gaps between Bluetooth sends, re-checked after every packet: the
// buffer covers the 3rd-longest gap of the last ~3 s plus headroom. It rises on
// the packet that brings a third spike into the window and falls as soon as
// spikes age out, allowing at most ~2 late packets per window.
class HapticsBufferTuner {
public:
  // 13.3 ms: one packet interval (10.7 ms) plus a little jitter.
  static constexpr std::uint8_t kMinLength = 40;
  static constexpr std::uint8_t kMaxLength = 128;
  static constexpr std::int64_t kHeadroomUs = 2000;
  // ~3 s of packets.
  static constexpr std::size_t kWindowPackets = 282;
  // The buffer covers the window's Nth-longest gap, so N - 1 may be late.
  static constexpr std::size_t kCoveredRank = 3;

  HapticsBufferTuner() = default;
  explicit HapticsBufferTuner(std::uint8_t initial);

  // Call after each send with the gap since the previous one. Returns the
  // buffer length to use from now on.
  std::uint8_t on_send(std::int64_t gap_us);
  std::uint8_t buffer_length() const { return length_; }

private:
  std::uint8_t length_ = 64;
  // Buffer length each packet in the window needed, oldest at next_.
  std::array<std::uint8_t, kWindowPackets> needed_{};
  std::size_t next_ = 0;
  std::size_t filled_ = 0;
  // How many packets in the window needed each length.
  std::array<std::uint16_t, 256> counts_{};
};

// The state the controller holds after a 0x36 audio packet built from `state`:
// 0x36 packets omit the mic/power-save fields, so those stay as they were in
// `previous` (the last state actually sent). Use this when tracking what has
// been delivered so a pending mic mute/unmute is not treated as already sent.
DsState state_after_audio_packet(const std::optional<DsState> &previous,
                                 const DsState &state);

constexpr std::size_t kTriggerEffectSize = 11;

// DS5 Bridge companion adjustments layered on top of the game-driven output
// state before each BT send. Percent fields at 100 mean pass-through.
struct DsCompanionOverrides {
  bool lightbar_override = false;
  bool lightbar_enabled = true;
  std::array<std::uint8_t, 3> lightbar_color{0, 0, 255};
  std::uint8_t lightbar_brightness_percent = 100;
  bool player_led_enabled = true;
  std::uint16_t classic_rumble_gain_percent = 100;
  // Classic (two-motor) rumble mode while a game drives the motors: v1 is
  // the controller's compatible-vibration mode, v2 its improved emulation.
  bool classic_rumble_v1 = false;
  std::uint16_t trigger_intensity_percent = 100;
  bool right_trigger_active = false;
  std::array<std::uint8_t, kTriggerEffectSize> right_trigger{};
  bool left_trigger_active = false;
  std::array<std::uint8_t, kTriggerEffectSize> left_trigger{};
  bool test_rumble_active = false;
  std::uint8_t test_rumble_power = 0;
  std::uint16_t speaker_volume_percent = 100;
  std::uint8_t speaker_gain_level = 1;
};

// Encodes a DS5 Bridge companion trigger effect (mode 0=feedback, 1=weapon,
// 2=vibration; percents 0-100) using the same zone packing as the DS5 Bridge
// Pico firmware.
void encode_companion_trigger_effect(
    std::span<std::uint8_t, kTriggerEffectSize> trigger, std::uint8_t mode,
    std::uint8_t start_percent, std::uint8_t wall_percent,
    std::uint8_t force_percent);

struct CompanionTriggerEffect;

// Encodes the full V2 companion trigger effect surface. V1 modes (0 feedback,
// 1 weapon, 2 vibration) delegate to encode_companion_trigger_effect;
// V2-only modes are 3 off, 4 multi-feedback, 5 slope, 6 multi-vibration.
void encode_companion_trigger_effect_v2(
    std::span<std::uint8_t, kTriggerEffectSize> trigger,
    const CompanionTriggerEffect &effect);

// Scales a game-supplied trigger FFB buffer by the global trigger-effect
// intensity percent (SET_TRIGGER_EFFECT_INTENSITY). percent 100 is a no-op;
// percent 0 turns the effect off. Covers the feedback (0x21), weapon (0x25),
// vibration (0x26), and slope (0x22) opcodes.
void scale_trigger_effect(std::span<std::uint8_t, kTriggerEffectSize> trigger,
                          std::uint16_t percent);

class DsOutputState {
public:
  DsOutputState();

  bool apply_usb_output_report(std::span<const std::uint8_t> report);
  void set_audio_out_stream_active(bool active);
  void set_companion_overrides(const DsCompanionOverrides &overrides);
  BtInitReport build_bt_init_report();
  BtStateReport build_bt_state_report();
  const DsState &state() const { return effective_state_; }
  const DsCompanionOverrides &companion_overrides() const {
    return companion_;
  }

  // Output state written by the host (kernel driver, Steam, games). A native
  // DualSense game changes it for its session; vdsd snapshots it when the
  // game opens the controller and restores it when the game closes it.
  struct HostState {
    DsState state{};
    std::array<std::uint8_t, 3> light_color{};
    std::uint8_t light_brightness = 0;
    bool emulate_light_brightness = false;
  };
  HostState host_state() const;
  // Restores a snapshot; the current speaker-stream routing is kept.
  void restore_host_state(const HostState &snapshot);

private:
  void recompute_effective_state();

  DsState state_{};
  DsState effective_state_{};
  DsCompanionOverrides companion_;
  std::array<std::uint8_t, 3> light_color_{};
  std::uint8_t light_brightness_ = 0;
  bool emulate_light_brightness_ = false;
  bool audio_out_stream_active_ = false;
  std::uint8_t report_sequence_ = 0;
};

class PcmAudioExtractor {
public:
  explicit PcmAudioExtractor(
      std::size_t speaker_input_frames = kSpeakerInputFrames);
  ~PcmAudioExtractor();

  PcmAudioExtractor(PcmAudioExtractor &&) noexcept;
  PcmAudioExtractor &operator=(PcmAudioExtractor &&) noexcept;

  PcmAudioExtractor(const PcmAudioExtractor &) = delete;
  PcmAudioExtractor &operator=(const PcmAudioExtractor &) = delete;

  std::vector<AudioChunk>
  push_usb_audio(std::span<const std::uint8_t> pcm_bytes);

  // Speaker loudness boost in dB; 0 bypasses (e.g. while headphones are in).
  void set_speaker_boost_db(float boost_db);
  // Companion speaker volume 0-100 (100 while headphones are in).
  void set_speaker_volume_percent(std::uint16_t percent);
  // Haptics gain in percent (companion setting, up to 1000).
  void set_haptics_gain_percent(std::uint16_t percent);
  // Encode the speaker block as mono. The built-in speaker is a single
  // driver, so all 160 kbps go to one channel; headphones need stereo.
  void set_speaker_mono(bool mono);

private:
  struct SpeakerEncoder;

  std::unique_ptr<SpeakerEncoder> speaker_encoder_;
  std::array<std::uint8_t, VDS_AUDIO_CHANNELS * sizeof(std::int16_t)>
      pending_frame_{};
  SpeakerInput speaker_input_{};
  SpeakerInput haptics_input_{};
  HapticsResampler haptics_resampler_;
  std::size_t speaker_input_frames_ = kSpeakerInputFrames;
  std::size_t pending_frame_pos_ = 0;
  std::size_t speaker_frame_pos_ = 0;
  bool chunk_has_signal_ = false;
  bool chunk_has_haptics_signal_ = false;
  bool chunk_has_speaker_signal_ = false;
};

std::vector<std::uint8_t> frame_bytes(std::uint16_t type,
                                      std::span<const std::uint8_t> payload);
std::string frame_type_name(std::uint16_t type);

} // namespace vds
