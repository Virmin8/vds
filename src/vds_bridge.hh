// SPDX-License-Identifier: MIT
#pragma once

/*
 * Platform-neutral half of one bridged controller: the virtual USB side's
 * frames, the controller's Bluetooth input, the paced audio/haptics packets,
 * the controller mic and the companion app's settings. vdsd on Linux (BlueZ
 * L2CAP + vds_hcd) and on Windows (HID + usbip) keep only their own I/O and
 * reach the controller and the virtual device through BridgeBluetooth and
 * BridgeUsb.
 *
 * Nothing here locks: Linux calls it from its single event loop, Windows
 * holds its bridge mutex around every call.
 */

#include <array>
#include <chrono>
#include <cstdint>
#include <deque>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "uapi/vds.h"
#include "vds_companion.hh"
#include "vds_log.hh"
#include "vds_persona.hh"
#include "vds_protocol.hh"

namespace vds {

using BridgeClock = std::chrono::steady_clock;

// Speaker frames per 0x36 packet (~10.67 ms); the extractor resamples each
// window to one Opus frame.
constexpr std::size_t kBridgeSpeakerInputFrames = kPcmWindowFrames;
constexpr auto kBridgeAudioOutputInterval = std::chrono::microseconds(
    (static_cast<std::int64_t>(kBridgeSpeakerInputFrames) * 1000000 +
     VDS_AUDIO_SAMPLE_RATE / 2) /
    VDS_AUDIO_SAMPLE_RATE);
// Poll interval while a deferred 0x31 state report waits to be sent.
constexpr int kBridgePendingOutputPollMs = 2;
// Controller mic open re-sends while waiting for its first packet.
constexpr std::chrono::milliseconds kBridgeMicOpenRetry{250};
constexpr std::uint32_t kBridgeMicOpenMaxResends = 20;

// Daemon-wide settings from the companion app and the command line.
struct BridgeSettings {
  HostPersona host_persona = HostPersona::DualSense;
  // Built-in speaker loudness boost; bypassed while headphones are plugged in.
  float speaker_boost_db = kDefaultSpeakerBoostDb;
  // Controller mic: off unless the companion app turns it on (or vdsd runs
  // with --mic). While the controller streams its mic, speaker/haptics audio
  // degrades, so it stays closed by default.
  bool mic_forced = false;
  bool mic_allowed = false;
  bool mic_muted = false;
  bool mute_led_enabled = true;
  std::uint16_t mic_volume_percent = 100;

  bool mic_live() const { return mic_allowed && !mic_muted; }
};

BridgeSettings &bridge_settings();

// The controller's Bluetooth link.
class BridgeBluetooth {
public:
  virtual ~BridgeBluetooth() = default;
  // Sends an output report without blocking; false when the link is busy.
  virtual bool try_send_output_report(std::span<const std::uint8_t> report) = 0;
  // Fetches a feature report. Linux answers asynchronously (nullopt; the
  // reply arrives through bridge_handle_feature_reply), Windows directly.
  virtual std::optional<std::vector<std::uint8_t>>
  get_feature_report(std::uint8_t report_id) = 0;
  virtual void set_feature_report(std::span<const std::uint8_t> report) = 0;
};

// The virtual USB controller.
class BridgeUsb {
public:
  virtual ~BridgeUsb() = default;
  // Writes one vds frame; false when the device's queue is full (dropped).
  virtual bool write_frame(std::uint32_t type,
                           std::span<const std::uint8_t> payload) = 0;
};

struct BridgeLatencyStats {
  std::uint64_t count = 0;
  std::uint64_t total_us = 0;
  std::uint64_t max_us = 0;
};

/*
 * Always-on audio delivery stats for the companion Diagnostics view. The
 * controller does not report buffer underruns, so a send gap longer than the
 * controller's audio buffer is counted as a probable dropout ("late").
 */
struct BridgeAudioStats {
  std::uint8_t buffer_length = 64; // 3 kHz samples
  bool buffer_auto = false;
  std::uint64_t trims_total = 0;
  bool haptics_only = false;
  std::uint64_t haptics_only_total = 0;
  BridgeClock::time_point last_send{};
  std::uint64_t packets_total = 0;
  std::uint64_t late_total = 0;
  std::uint64_t refills_total = 0;
  // Since the previous `audio` control request.
  BridgeClock::time_point window_start{};
  std::uint64_t window_packets = 0;
  std::uint64_t window_mic_packets = 0;
  // Controller mic packets: forwarded, and arrived while the mic was closed
  // (each burst of those re-sends the close report).
  std::uint64_t mic_packets_total = 0;
  std::uint64_t mic_while_closed_total = 0;
  BridgeClock::time_point last_mic_close_resend{};
  // Time from opening the controller mic to its first packet (last start).
  std::int64_t mic_startup_ms = -1;
  // Time from opening the mic to the first packet above near-silence (last
  // start); shows a mic that warms up after it is switched on.
  std::int64_t mic_first_sound_ms = -1;
  std::uint64_t window_late = 0;
  std::uint64_t window_refills = 0;
  std::int64_t window_max_gap_us = 0;
};

struct BridgeTraceState {
  std::vector<std::uint8_t> last_hid_out;
  std::uint64_t dropped_usb_frame_count = 0;
  std::uint64_t dropped_audio_haptics_count = 0;
  std::uint64_t queue_dropped_audio_haptics_count = 0;
  std::uint64_t blocked_audio_haptics_count = 0;
  std::uint64_t deferred_bt_state_count = 0;
  std::uint64_t coalesced_bt_state_count = 0;
  std::uint64_t blocked_bt_state_count = 0;
  std::uint64_t audio_usb_frame_count = 0;
  std::uint64_t bt_input_count = 0;
  std::uint64_t bt_input_unknown_count = 0;
  std::uint64_t bt_mic_packet_count = 0;
  std::uint64_t bt_mic_drop_count = 0;
  std::uint64_t bt_mic_decode_fail_count = 0;
  std::uint64_t audio_in_forward_count = 0;
  std::uint64_t audio_in_write_short_count = 0;
  std::uint16_t mic_last_decoded_bytes = 0;
  std::uint16_t mic_peak_permille = 0;
  BridgeLatencyStats output_hid_latency;
  BridgeLatencyStats output_feature_latency;
  BridgeLatencyStats output_audio_latency;
  BridgeLatencyStats output_audio_extract_latency;
  BridgeLatencyStats output_audio_send_latency;
  std::uint64_t input_write_count = 0;
  std::uint64_t input_write_total_us = 0;
  std::uint64_t input_write_max_us = 0;
  std::uint64_t input_gap_max_us = 0;
  BridgeClock::time_point last_input_time{};
  bool have_last_input_time = false;
  std::array<std::uint8_t, 4> last_input_buttons{};
  bool have_last_input_buttons = false;
  bool audio_haptics_nonzero_seen = false;
  bool haptics_burst_active = false;
  std::uint64_t haptics_burst_chunks = 0;
};

// One bridged controller's state; `path` names it in logs.
struct BridgePort {
  std::string path;
  PcmAudioExtractor extractor{kBridgeSpeakerInputFrames};
  PcmAudioExtractor waveout_extractor{kBridgeSpeakerInputFrames};
  MicAudioDecoder mic_decoder;
  HapticsPacketBuilder haptics_builder;
  DsOutputState output_state;
  std::deque<AudioChunk> pending_audio_chunks;
  std::optional<DsState> last_sent_bt_state;
  std::optional<DsState> pending_bt_state;
  std::optional<BtStateReport> pending_bt_state_report;
  BridgeClock::time_point next_haptics_send_time{};
  bool audio_primed = false;
  // Chunks buffered before sending after the queue runs dry; the trimmer
  // keeps the standing queue at this depth. Each chunk is one Bluetooth
  // packet interval of latency. Windows raises it: USB/IP over loopback TCP
  // delivers audio in larger bursts than the Linux module.
  std::size_t audio_prime_chunks = AudioQueueTrimmer::kTargetMinDepth;
  AudioQueueTrimmer queue_trimmer;
  HapticsBufferTuner buffer_tuner;
  std::size_t silent_speaker_packets = 0;
  HostPersona persona = HostPersona::DualSense;
  PersonaState persona_state;
  bool audio_out_stream_active = false;
  // Host capture stream open (audio-in altsetting) vs. controller mic open.
  bool audio_in_requested = false;
  bool audio_in_stream_active = false;
  // Controller mic start: when it was opened and whether its first packet has
  // arrived; until then the open report is re-sent (a lost one would leave
  // the start of a recording silent).
  BridgeClock::time_point mic_opened_at{};
  BridgeClock::time_point mic_open_resent_at{};
  std::uint32_t mic_open_resends = 0;
  bool mic_first_packet = false;
  bool mic_first_sound = false;
  bool mic_muted = false;
  bool mute_button_down = false;
  bool headset_plugged = false;
  bool headset_mic_plugged = false;
  bool speaker_waveout_selected = true;
  bool speaker_waveout_active = false;
  std::uint32_t speaker_waveout_phase = 0;
  std::uint16_t haptics_gain_percent = 100;
  std::uint8_t battery_status = 0xff;
  // Idle Disconnect: last input that counts as activity.
  BridgeClock::time_point last_input_activity{};
  // Diagnostic counters: the `audio` control request resets the window part.
  BridgeAudioStats audio_stats;
  CompanionInputState companion_input;
  std::array<std::vector<std::uint8_t>, 256> feature_cache;
  std::array<bool, 256> feature_cached{};
  std::vector<std::uint8_t> pending_feature_reports;
  BridgeTraceState trace_state;
};

float bridge_speaker_boost_db(const BridgePort &port);
// The companion speaker volume slider only applies to the built-in speaker.
std::uint16_t bridge_speaker_volume_percent(const BridgePort &port);

// Back to a freshly connected controller's state (keeps `path`).
void bridge_reset(BridgePort &port);
// Stops audio and the mic after the controller or host went away.
void bridge_on_disconnect(BridgePort &port);

// Sends a deferred 0x31 state report; false while it is still blocked.
bool bridge_flush_pending_state(BridgePort &port, BridgeBluetooth &bt,
                                std::uint32_t trace_flags, Logger &logger);
// Sends the output state as a 0x31 report if it changed (deferred when the
// link is busy).
void bridge_forward_state_if_changed(BridgePort &port, BridgeBluetooth &bt,
                                     std::uint32_t trace_flags, Logger &logger,
                                     std::string_view reason);

// Opens/closes the controller mic to match bridge_settings() and sets the
// mute light. Callers forward the 0x31 state afterwards for the light.
void bridge_sync_mic(BridgePort &port, BridgeBluetooth *bt, Logger &logger);

// Caches a feature report from the controller and answers the host if it
// asked for it.
void bridge_handle_feature_reply(BridgePort &port,
                                 std::span<const std::uint8_t> report,
                                 BridgeUsb &usb, std::uint32_t trace_flags,
                                 Logger &logger);
// Caches a feature report without sending it to the host.
void bridge_store_feature_report(BridgePort &port,
                                 std::span<const std::uint8_t> report);
// Caches a feature report and hands it to the host (initial priming).
void bridge_cache_feature_report(BridgePort &port,
                                 std::span<const std::uint8_t> report,
                                 BridgeUsb &usb, bool trace, Logger &logger);

// One frame from the virtual USB device (vds_frame_header type + payload).
void bridge_handle_usb_frame(BridgePort &port, std::uint32_t type,
                             std::span<const std::uint8_t> payload,
                             BridgeBluetooth *bt, BridgeUsb &usb,
                             std::uint32_t trace_flags, Logger &logger);

struct BridgeInputResult {
  // Idle Disconnect timed out: the platform should turn the controller off.
  bool idle_timeout = false;
};
// One Bluetooth interrupt packet from the controller (input or mic audio).
BridgeInputResult bridge_handle_bt_input(BridgePort &port,
                                         std::span<const std::uint8_t> packet,
                                         BridgeClock::time_point read_time,
                                         BridgeBluetooth &bt, BridgeUsb &usb,
                                         CompanionRuntime &companion,
                                         std::uint32_t trace_flags,
                                         Logger &logger);

// Audio/haptics pacing: whether a packet is due to be considered, and the
// next send. bridge_flush_audio sends at most one packet; false when the link
// was busy (retry soon).
bool bridge_audio_chunk_ready(const BridgePort &port);
bool bridge_flush_audio(BridgePort &port, BridgeBluetooth &bt,
                        std::uint32_t trace_flags, Logger &logger);
// Queues the WebHID speaker test tone while it is on.
void bridge_enqueue_waveout(BridgePort &port, std::uint32_t trace_flags,
                            Logger &logger);
// When this port next needs service (at or before `now`: immediately), or
// nullopt when it has nothing scheduled; bridge_wakeup_ms in milliseconds.
std::optional<BridgeClock::time_point>
bridge_next_wakeup(const BridgePort &port, BridgeClock::time_point now);
std::optional<int> bridge_wakeup_ms(const BridgePort &port,
                                    BridgeClock::time_point now);

// Re-sends the controller mic open until its first packet arrives.
void bridge_retry_mic_open(BridgePort &port, BridgeBluetooth &bt,
                           Logger &logger);
bool bridge_mic_retry_pending(const BridgePort &port);

// Copies the companion settings into bridge_settings() and returns the
// output overrides for every port.
DsCompanionOverrides bridge_companion_overrides(CompanionRuntime &companion);
// Applies companion settings to one connected port and forwards its state.
void bridge_apply_companion(BridgePort &port, const CompanionSettings &settings,
                            const DsCompanionOverrides &overrides,
                            BridgeBluetooth &bt, std::uint32_t trace_flags,
                            Logger &logger);

// The `audio` control reply's entry for one port (a JSON object); resets the
// window counters.
std::string bridge_audio_stats_json(BridgePort &port,
                                    BridgeClock::time_point now);
// Mic and headset state for the companion AUDIO_STATUS report.
CompanionAudioStatus bridge_audio_status(const BridgePort &port);

} // namespace vds
