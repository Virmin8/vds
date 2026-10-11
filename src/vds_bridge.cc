// SPDX-License-Identifier: MIT
#include "vds_bridge.hh"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <sstream>

#include "jsonl.hh"
#include "vds_common.hh"
#include "vdsd_common.hh"

namespace vds {
namespace {

using Clock = BridgeClock;

constexpr std::size_t kTraceDumpMaxBytes = 96;
constexpr std::size_t kUsbInputButtonsOffset = 8;
constexpr std::size_t kUsbInputButtonsSize = 4;
constexpr std::size_t kUsbInputMuteButtonOffset = 10;
constexpr std::size_t kUsbInputHeadsetOffset = 54;
constexpr std::uint8_t kUsbInputHeadphonesPluggedMask = 0x01;
constexpr std::uint8_t kUsbInputMicPluggedMask = 0x02;
constexpr std::uint8_t kUsbInputMuteButtonMask = 0x04;
constexpr std::uint64_t kInputTraceSummaryInterval = 1000;
constexpr std::uint64_t kOutputTraceSummaryInterval = 250;
constexpr auto kInputTraceGapWarn = std::chrono::milliseconds(20);
constexpr auto kInputTraceSlowWriteWarn = std::chrono::milliseconds(5);
constexpr auto kOutputTraceSlowWarn = std::chrono::milliseconds(5);
constexpr auto kOutputTraceFeatureSlowWarn = std::chrono::milliseconds(20);
/*
 * A send that is at most this late keeps the fixed cadence (the next packet
 * goes out immediately to catch up); anything later resyncs to the wall clock
 * so a stalled stream does not replay a backlog.
 */
constexpr auto kMaxAudioCatchup = 2 * kBridgeAudioOutputInterval;
constexpr auto kHapticsOutputBlockedRetry = std::chrono::milliseconds(2);
constexpr std::size_t kMaxPendingAudioChunks = 8;
/*
 * Speaker-silent packets (~1 s) before switching to haptics-only 0x33 reports:
 * the same haptics and state in about half the airtime of 0x36. Speaker audio
 * switches back to 0x36 on its first packet; the Opus encoder keeps running on
 * the silence, so it resumes cleanly.
 */
constexpr std::size_t kHapticsOnlyAfterSilentPackets = 94;
// Mic peak (permille of full scale) above which a packet counts as sound.
constexpr std::uint16_t kMicSoundPermille = 10;
// Controller audio buffer (3 kHz samples) used while the mic records; the
// value upstream sends with the mic.
constexpr std::uint8_t kMicBufferLength = 64;
constexpr std::uint8_t kTestCommandReportId = 0x80;
constexpr std::uint8_t kTestCommandResultReportId = 0x81;
constexpr std::uint8_t kTestCommandCompleteStatus = 0x02;
constexpr std::uint8_t kTestCommandAudioDevice = 0x06;
constexpr std::uint8_t kTestCommandWaveoutPrepare = 0x04;
constexpr std::uint8_t kTestCommandWaveoutControl = 0x02;
constexpr std::uint8_t kTestCommandSpeakerParam = 0x08;
constexpr std::uint32_t kSpeakerWaveoutFrequencyHz = 1000;
constexpr std::uint32_t kSpeakerWaveoutPeriodFrames =
    VDS_AUDIO_SAMPLE_RATE / kSpeakerWaveoutFrequencyHz;
constexpr double kSpeakerWaveoutTwoPi = 6.28318530717958647692;
/* Arbitrary fixed amplitude used only for synthetic WebHID speaker waveout. */
constexpr std::int16_t kSpeakerWaveoutAmplitude = 12000;

const char *usb_interface_type_name(std::uint8_t type) {
  switch (type) {
  case VDS_USB_INTERFACE_HID:
    return "hid";
  case VDS_USB_INTERFACE_AUDIO_OUT:
    return "audio_out";
  case VDS_USB_INTERFACE_AUDIO_IN:
    return "audio_in";
  default:
    return "unknown";
  }
}

void trace_output_latency(const std::string &device, std::string_view label,
                          BridgeLatencyStats &stats, Clock::duration duration,
                          Clock::duration slow_threshold, Logger &logger) {
  const std::uint64_t elapsed_us = duration_us(duration);
  ++stats.count;
  stats.total_us += elapsed_us;
  stats.max_us = std::max(stats.max_us, elapsed_us);

  if (duration >= slow_threshold) {
    logger.log(LogScope::Output, LogLevel::Warn,
               device + " slow " + std::string(label) +
                   " latency count=" + std::to_string(stats.count) +
                   " latency_us=" + std::to_string(elapsed_us));
  }
  if (stats.count == 1 || stats.count % kOutputTraceSummaryInterval == 0) {
    logger.log(LogScope::Output, LogLevel::Debug,
               device + " " + std::string(label) +
                   " latency summary count=" + std::to_string(stats.count) +
                   " avg_us=" + std::to_string(stats.total_us / stats.count) +
                   " max_us=" + std::to_string(stats.max_us));
  }
}

void trace_hid_out_report(const std::string &device,
                          std::span<const std::uint8_t> payload,
                          BridgeTraceState &trace_state, Logger &logger) {
  std::ostringstream line;
  line << device << " hid out";
  if (!payload.empty()) {
    line << " report=" << hex_u8(payload[0]);
  }
  if (payload.size() >= 5) {
    /*
     * USB report 0x02 carries SetStateData after the report ID. These fields
     * show whether a plain rumble request exists before the HD haptics audio
     * path is involved.
     */
    line << " flags0=" << hex_u8(payload[1]) << " flags1=" << hex_u8(payload[2])
         << " rumble_r=" << static_cast<unsigned>(payload[3])
         << " rumble_l=" << static_cast<unsigned>(payload[4]);
  }
  if (payload.size() >= 40) {
    line << " light_flags=" << hex_u8(payload[39]);
  }
  logger.log(LogScope::Hid, LogLevel::Debug, line.str());

  const bool hid_out_changed =
      trace_state.last_hid_out.size() != payload.size() ||
      !std::equal(trace_state.last_hid_out.begin(),
                  trace_state.last_hid_out.end(), payload.begin());
  if (!hid_out_changed) {
    return;
  }

  trace_state.last_hid_out.assign(payload.begin(), payload.end());
  std::ostringstream changed;
  changed << device
          << " hid out changed raw=" << hex_bytes(payload, kTraceDumpMaxBytes);
  if (payload.size() >= 33 && payload[0] == VDS_USB_OUTPUT_REPORT_ID) {
    /*
     * SetStateData begins after the report ID. Offsets 10..20 and 21..31 are
     * the right/left adaptive trigger blocks from the DualSense USB report.
     */
    changed << " right_trigger=" << hex_bytes(payload.subspan(11, 11), 11)
            << " left_trigger=" << hex_bytes(payload.subspan(22, 11), 11);
  }
  logger.log(LogScope::Hid, LogLevel::Debug, changed.str());
}

// Writes a frame to the virtual device, counting (and occasionally logging)
// frames dropped because its queue was full.
bool write_usb_frame(BridgePort &port, BridgeUsb &usb, std::uint32_t type,
                     std::span<const std::uint8_t> payload, bool trace,
                     Logger &logger) {
  if (usb.write_frame(type, payload)) {
    return true;
  }
  ++port.trace_state.dropped_usb_frame_count;
  if (trace && (port.trace_state.dropped_usb_frame_count == 1 ||
                port.trace_state.dropped_usb_frame_count % 1000 == 0)) {
    logger.log(LogScope::Port, LogLevel::Warn,
               port.path + " vDS write queue full count=" +
                   std::to_string(port.trace_state.dropped_usb_frame_count));
  }
  return false;
}

std::uint8_t mic_buffer_length(const BridgePort &port) {
  return std::max(port.audio_stats.buffer_length, kMicBufferLength);
}

} // namespace

BridgeSettings &bridge_settings() {
  static BridgeSettings settings;
  return settings;
}

float bridge_speaker_boost_db(const BridgePort &port) {
  return port.headset_plugged ? 0.0f : bridge_settings().speaker_boost_db;
}

std::uint16_t bridge_speaker_volume_percent(const BridgePort &port) {
  return port.headset_plugged
             ? 100
             : port.output_state.companion_overrides().speaker_volume_percent;
}

void bridge_reset(BridgePort &port) {
  port.extractor = PcmAudioExtractor{kBridgeSpeakerInputFrames};
  port.waveout_extractor = PcmAudioExtractor{kBridgeSpeakerInputFrames};
  port.mic_decoder = MicAudioDecoder{};
  port.haptics_builder = HapticsPacketBuilder{};
  port.haptics_builder.set_buffer_length(port.audio_stats.buffer_length);
  port.output_state = DsOutputState{};
  port.pending_audio_chunks.clear();
  port.audio_primed = false;
  port.last_sent_bt_state.reset();
  port.pending_bt_state.reset();
  port.pending_bt_state_report.reset();
  port.next_haptics_send_time = {};
  port.audio_out_stream_active = false;
  port.audio_in_requested = false;
  port.audio_in_stream_active = false;
  port.mic_muted = false;
  port.mute_button_down = false;
  port.headset_plugged = false;
  port.headset_mic_plugged = false;
  port.speaker_waveout_selected = true;
  port.speaker_waveout_active = false;
  port.speaker_waveout_phase = 0;
  port.haptics_gain_percent = 100;
  port.battery_status = 0xff;
  port.last_input_activity = {};
  port.audio_stats = {};
  port.companion_input = {};
  port.feature_cache = {};
  port.feature_cached = {};
  port.pending_feature_reports.clear();
  port.trace_state = BridgeTraceState{};
}

void bridge_on_disconnect(BridgePort &port) {
  port.pending_audio_chunks.clear();
  port.audio_primed = false;
  port.next_haptics_send_time = {};
  port.audio_out_stream_active = false;
  port.audio_in_requested = false;
  port.audio_in_stream_active = false;
  port.mic_muted = false;
  port.mute_button_down = false;
  port.speaker_waveout_active = false;
  port.speaker_waveout_phase = 0;
  port.pending_bt_state.reset();
  port.pending_bt_state_report.reset();
}

bool bridge_flush_pending_state(BridgePort &port, BridgeBluetooth &bt,
                                std::uint32_t trace_flags, Logger &logger) {
  if (!port.pending_bt_state_report || !port.pending_bt_state) {
    return true;
  }

  const bool output_trace = trace_enabled(trace_flags, kTraceOutput);
  if (bt.try_send_output_report(*port.pending_bt_state_report)) {
    port.last_sent_bt_state = *port.pending_bt_state;
    port.pending_bt_state.reset();
    port.pending_bt_state_report.reset();
    if (output_trace) {
      logger.log(LogScope::Hid, LogLevel::Debug,
                 port.path + " flushed pending BT 0x31 state report");
    }
    return true;
  }

  ++port.trace_state.blocked_bt_state_count;
  if (output_trace && (port.trace_state.blocked_bt_state_count == 1 ||
                       port.trace_state.blocked_bt_state_count % 1000 == 0)) {
    logger.log(LogScope::Hid, LogLevel::Warn,
               port.path +
                   " pending BT 0x31 state report still blocked "
                   "count=" +
                   std::to_string(port.trace_state.blocked_bt_state_count));
  }
  return false;
}

void bridge_forward_state_if_changed(BridgePort &port, BridgeBluetooth &bt,
                                     std::uint32_t trace_flags, Logger &logger,
                                     std::string_view reason) {
  const bool output_trace = trace_enabled(trace_flags, kTraceOutput);

  (void)bridge_flush_pending_state(port, bt, trace_flags, logger);

  const DsState state = port.output_state.state();
  if (port.last_sent_bt_state && *port.last_sent_bt_state == state) {
    if (port.pending_bt_state) {
      port.pending_bt_state.reset();
      port.pending_bt_state_report.reset();
      ++port.trace_state.coalesced_bt_state_count;
    }
    if (output_trace) {
      logger.log(LogScope::Hid, LogLevel::Debug,
                 port.path + " " + std::string(reason) +
                     " skipped: BT state unchanged");
    }
    return;
  }
  if (port.pending_bt_state && *port.pending_bt_state == state) {
    if (output_trace) {
      logger.log(LogScope::Hid, LogLevel::Debug,
                 port.path + " " + std::string(reason) +
                     " skipped: BT state already pending");
    }
    return;
  }

  const auto packet = port.output_state.build_bt_state_report();
  if (bt.try_send_output_report(packet)) {
    port.last_sent_bt_state = state;
    port.pending_bt_state.reset();
    port.pending_bt_state_report.reset();
    if (output_trace) {
      logger.log(LogScope::Hid, LogLevel::Debug,
                 port.path + " " + std::string(reason) +
                     " forwarded as BT 0x31 state report");
    }
    return;
  }

  if (port.pending_bt_state) {
    ++port.trace_state.coalesced_bt_state_count;
  }
  ++port.trace_state.deferred_bt_state_count;
  port.pending_bt_state = state;
  port.pending_bt_state_report = packet;
  if (output_trace && (port.trace_state.deferred_bt_state_count == 1 ||
                       port.trace_state.deferred_bt_state_count % 1000 == 0)) {
    logger.log(LogScope::Hid, LogLevel::Warn,
               port.path + " deferred BT 0x31 state report count=" +
                   std::to_string(port.trace_state.deferred_bt_state_count) +
                   " coalesced=" +
                   std::to_string(port.trace_state.coalesced_bt_state_count));
  }
}

// Keeps the controller mic open for as long as the mic is on (switch on and
// not muted), so a recording starts without the controller's warm-up; its
// audio is forwarded only while the host records. Also pushes mic mute (and
// the mute light).
void bridge_sync_mic(BridgePort &port, BridgeBluetooth *bt, Logger &logger) {
  const BridgeSettings &settings = bridge_settings();
  const bool open = settings.mic_live();
  const bool open_changed = open != port.audio_in_stream_active;
  const bool mute_changed = settings.mic_muted != port.mic_muted;
  port.mic_muted = settings.mic_muted;
  port.audio_in_stream_active = open;
  if (open_changed) {
    port.mic_decoder = MicAudioDecoder{};
    port.mic_opened_at = Clock::now();
    port.mic_open_resent_at = port.mic_opened_at;
    port.mic_open_resends = 0;
    port.mic_first_packet = false;
    port.mic_first_sound = false;
  }
  if (bt != nullptr && (open_changed || (open && mute_changed))) {
    bt->try_send_output_report(
        port.output_state.build_bt_mic_state_report(open, port.mic_muted));
    if (open_changed) {
      const bool sent = bt->try_send_output_report(
          port.output_state.build_bt_mic_report(open, mic_buffer_length(port)));
      logger.log(LogScope::InputAudio, LogLevel::Info,
                 port.path + " controller mic " +
                     std::string(open ? "opened" : "closed") +
                     " sent=" + (sent ? "yes" : "no"));
    }
  }
  // Mute light on whenever the mic is not live (off or muted), like a PS5,
  // unless the companion turned the light off.
  port.output_state.set_mic_mute_led(settings.mute_led_enabled &&
                                     !settings.mic_live());
}

void bridge_handle_feature_reply(BridgePort &port,
                                 std::span<const std::uint8_t> report,
                                 BridgeUsb &usb, std::uint32_t trace_flags,
                                 Logger &logger) {
  if (report.empty()) {
    return;
  }

  const std::uint8_t report_id = report[0];
  port.feature_cache[report_id].assign(report.begin(), report.end());
  port.feature_cached[report_id] = true;

  const auto pending = std::find(port.pending_feature_reports.begin(),
                                 port.pending_feature_reports.end(), report_id);
  const bool usb_waiting = pending != port.pending_feature_reports.end();
  const bool output_trace = trace_enabled(trace_flags, kTraceOutput);
  if (output_trace) {
    logger.log(LogScope::Hid, LogLevel::Debug,
               port.path + " bt feature cached report=" + hex_u8(report_id) +
                   " len=" + std::to_string(report.size()) +
                   " usb_waiting=" + (usb_waiting ? "yes" : "no"));
  }
  if (!usb_waiting) {
    return;
  }

  port.pending_feature_reports.erase(pending);
  (void)write_usb_frame(port, usb, VDS_FRAME_USB_FEATURE_REPLY, report,
                        output_trace, logger);
}

void bridge_store_feature_report(BridgePort &port,
                                 std::span<const std::uint8_t> report) {
  if (!report.empty()) {
    port.feature_cache[report[0]].assign(report.begin(), report.end());
    port.feature_cached[report[0]] = true;
  }
}

void bridge_cache_feature_report(BridgePort &port,
                                 std::span<const std::uint8_t> report,
                                 BridgeUsb &usb, bool trace, Logger &logger) {
  if (report.empty()) {
    return;
  }

  const std::uint8_t report_id = report[0];
  port.feature_cache[report_id].assign(report.begin(), report.end());
  port.feature_cached[report_id] = true;
  (void)write_usb_frame(port, usb, VDS_FRAME_USB_FEATURE_REPLY,
                        port.feature_cache[report_id], trace, logger);
}

void bridge_handle_usb_frame(BridgePort &port, std::uint32_t type,
                             std::span<const std::uint8_t> payload,
                             BridgeBluetooth *bt, BridgeUsb &usb,
                             std::uint32_t trace_flags, Logger &logger) {
  if (type == VDS_FRAME_USB_INTERFACE) {
    if (payload.size() != sizeof(vds_usb_interface_event)) {
      logger.log(LogScope::Usb, LogLevel::Warn,
                 port.path + " malformed USB interface event len=" +
                     std::to_string(payload.size()));
      return;
    }

    vds_usb_interface_event event{};
    std::memcpy(&event, payload.data(), sizeof(event));

    std::ostringstream line;
    line << port.path << " USB set_interface type="
         << usb_interface_type_name(event.interface_type)
         << " number=" << static_cast<unsigned>(event.interface_number)
         << " alt=" << static_cast<unsigned>(event.altsetting);
    if (event.interface_type == VDS_USB_INTERFACE_AUDIO_OUT ||
        event.interface_type == VDS_USB_INTERFACE_AUDIO_IN) {
      line << " stream=" << (event.altsetting != 0 ? "on" : "off");
    }
    logger.log(LogScope::Usb, LogLevel::Info, line.str());
    if (event.interface_type == VDS_USB_INTERFACE_AUDIO_OUT) {
      const bool active = event.altsetting != 0;
      port.audio_out_stream_active = active;
      port.output_state.set_audio_out_stream_active(active,
                                                    port.headset_plugged);
      port.pending_audio_chunks.clear();
      port.audio_primed = false;
      port.next_haptics_send_time = {};
      if (active) {
        port.extractor = PcmAudioExtractor{kBridgeSpeakerInputFrames};
      }
      if (bt != nullptr) {
        bridge_forward_state_if_changed(port, *bt, trace_flags, logger,
                                        active
                                            ? "audio out interface route on"
                                            : "audio out interface route off");
      }
    } else if (event.interface_type == VDS_USB_INTERFACE_AUDIO_IN) {
      port.audio_in_requested = event.altsetting != 0;
      bridge_sync_mic(port, bt, logger);
    }
    return;
  }

  const bool output_trace = trace_enabled(trace_flags, kTraceOutput);
  if (output_trace) {
    std::ostringstream line;
    line << port.path << " frame "
         << frame_type_name(static_cast<std::uint16_t>(type))
         << " len=" << payload.size();

    bool emit_trace = true;
    if (type == VDS_FRAME_USB_AUDIO_OUT &&
        payload.size() >= VDS_AUDIO_CHANNELS * sizeof(std::int16_t)) {
      std::array<int, VDS_AUDIO_CHANNELS> peaks{};
      const std::size_t frame_size = VDS_AUDIO_CHANNELS * sizeof(std::int16_t);
      const std::size_t frames = payload.size() / frame_size;
      for (std::size_t frame = 0; frame < frames; ++frame) {
        const std::uint8_t *base = payload.data() + frame * frame_size;
        for (std::size_t channel = 0; channel < VDS_AUDIO_CHANNELS; ++channel) {
          const auto low =
              static_cast<std::uint16_t>(base[channel * sizeof(std::int16_t)]);
          const auto high = static_cast<std::uint16_t>(
                                base[channel * sizeof(std::int16_t) + 1])
                            << 8;
          const auto sample = static_cast<std::int16_t>(low | high);
          peaks[channel] =
              std::max(peaks[channel], std::abs(static_cast<int>(sample)));
        }
      }

      ++port.trace_state.audio_usb_frame_count;
      const bool haptics_nonzero = peaks[2] != 0 || peaks[3] != 0;
      emit_trace =
          port.trace_state.audio_usb_frame_count == 1 ||
          port.trace_state.audio_usb_frame_count % 250 == 0 ||
          (haptics_nonzero && !port.trace_state.audio_haptics_nonzero_seen);
      port.trace_state.audio_haptics_nonzero_seen |= haptics_nonzero;
      line << " pcm_peak_ch0=" << peaks[0] << " pcm_peak_ch1=" << peaks[1]
           << " pcm_peak_ch2=" << peaks[2] << " pcm_peak_ch3=" << peaks[3];
    }

    if (emit_trace) {
      logger.log(LogScope::Usb, LogLevel::Debug, line.str());
    }
  }

  if (type == VDS_FRAME_USB_HID_OUT) {
    const auto start = Clock::now();
    if (output_trace) {
      trace_hid_out_report(port.path, payload, port.trace_state, logger);
    }
    std::optional<std::array<std::uint8_t, 1 + VDS_SET_STATE_SIZE>>
        persona_output;
    std::span<const std::uint8_t> ds5_output = payload;
    if (port.persona != HostPersona::DualSense) {
      persona_output =
          persona_output_to_ds5(port.persona, payload, port.persona_state);
      if (!persona_output) {
        return;
      }
      ds5_output = *persona_output;
    }
    if (!port.output_state.apply_usb_output_report(ds5_output)) {
      if (output_trace) {
        logger.log(LogScope::Hid, LogLevel::Debug,
                   port.path + " hid out ignored: unsupported output report");
      }
      return;
    }
    if (port.audio_out_stream_active || port.speaker_waveout_active) {
      port.output_state.set_audio_out_stream_active(true, port.headset_plugged);
    }
    if (output_trace) {
      logger.log(LogScope::Hid, LogLevel::Debug,
                 port.path + " hid out accepted for BT state update");
    }

    if (bt != nullptr) {
      bridge_forward_state_if_changed(port, *bt, trace_flags, logger,
                                      "hid out");
    }

    if (output_trace) {
      trace_output_latency(port.path, "hid_out",
                           port.trace_state.output_hid_latency,
                           Clock::now() - start, kOutputTraceSlowWarn, logger);
    }
    return;
  }

  if (type == VDS_FRAME_USB_FEATURE_GET) {
    const auto start = Clock::now();
    if (payload.empty()) {
      return;
    }
    const std::uint8_t report_id = payload[0];
    const unsigned requested_length =
        payload.size() >= 3
            ? static_cast<unsigned>(payload[1] |
                                    (static_cast<unsigned>(payload[2]) << 8))
            : (payload.size() >= 2 ? static_cast<unsigned>(payload[1]) : 0);
    if (port.persona == HostPersona::Ds4) {
      // DS4 feature reports are answered locally; calibration and MAC come
      // from the DualSense's cached reports 0x05 and 0x09.
      const auto cached = [&](std::uint8_t id) {
        return port.feature_cached[id]
                   ? std::span<const std::uint8_t>(port.feature_cache[id])
                   : std::span<const std::uint8_t>();
      };
      const auto reply = ds4_feature_report(report_id, requested_length,
                                            cached(0x05), cached(0x09));
      (void)write_usb_frame(port, usb, VDS_FRAME_USB_FEATURE_REPLY, reply,
                            output_trace, logger);
      return;
    }
    if (port.persona != HostPersona::DualSense) {
      return;
    }
    const bool cache_hit = port.feature_cached[report_id];
    if (output_trace) {
      logger.log(LogScope::Hid, LogLevel::Debug,
                 port.path + " feature get report=" + hex_u8(report_id) +
                     " request_len=" + std::to_string(requested_length) +
                     " cache=" + (cache_hit ? "hit" : "miss") + " forwarded=" +
                     ((bt != nullptr && !cache_hit) ? "yes" : "no"));
    }
    if (cache_hit) {
      (void)write_usb_frame(port, usb, VDS_FRAME_USB_FEATURE_REPLY,
                            port.feature_cache[report_id], output_trace,
                            logger);
    } else if (bt != nullptr) {
      port.pending_feature_reports.push_back(report_id);
      if (auto reply = bt->get_feature_report(report_id)) {
        bridge_handle_feature_reply(port, *reply, usb, trace_flags, logger);
      }
    }
    if (output_trace) {
      trace_output_latency(port.path, "feature_get",
                           port.trace_state.output_feature_latency,
                           Clock::now() - start, kOutputTraceFeatureSlowWarn,
                           logger);
    }
    return;
  }

  if (type == VDS_FRAME_USB_FEATURE_SET) {
    const auto start = Clock::now();
    if (payload.empty()) {
      return;
    }
    const std::uint8_t report_id = payload[0];
    if (port.persona != HostPersona::DualSense) {
      // Persona feature writes (DS4 probes) must not reach the DualSense.
      return;
    }
    if (output_trace) {
      logger.log(LogScope::Hid, LogLevel::Debug,
                 port.path + " feature set report=" + hex_u8(report_id) +
                     " len=" + std::to_string(payload.size()) +
                     " forwarded=" + (bt != nullptr ? "yes" : "no"));
      logger.log(LogScope::Hid, LogLevel::Debug,
                 port.path + " feature set payload=" +
                     hex_bytes(payload, kTraceDumpMaxBytes));
    }
    if (payload.size() >= 3 && payload[0] == kTestCommandReportId) {
      std::size_t command_offset = 1;
      if (payload.size() > command_offset &&
          payload[command_offset] == kTestCommandReportId) {
        ++command_offset;
      }
      if (payload.size() > command_offset + 1) {
        const std::uint8_t command_device = payload[command_offset];
        const std::uint8_t command_action = payload[command_offset + 1];
        const std::size_t command_data_offset = command_offset + 2;

        std::array<std::uint8_t, VDS_USB_INPUT_REPORT_SIZE> test_result{};
        test_result[0] = kTestCommandResultReportId;
        test_result[1] = command_device;
        test_result[2] = command_action;
        test_result[3] = kTestCommandCompleteStatus;
        bridge_cache_feature_report(port, test_result, usb, output_trace,
                                    logger);
        if (output_trace) {
          logger.log(LogScope::Hid, LogLevel::Debug,
                     port.path + " cached WebHID test result device=" +
                         hex_u8(command_device) + " action=" +
                         hex_u8(command_action) + " status=complete");
        }

        if (command_device == kTestCommandAudioDevice &&
            command_action == kTestCommandWaveoutPrepare) {
          port.speaker_waveout_selected =
              payload.size() > command_data_offset + 2 &&
              payload[command_data_offset + 2] == kTestCommandSpeakerParam;
          if (output_trace) {
            logger.log(LogScope::Output, LogLevel::Debug,
                       port.path + " WebHID waveout target=" +
                           std::string(port.speaker_waveout_selected
                                           ? "speaker"
                                           : "headphone"));
          }
        } else if (command_device == kTestCommandAudioDevice &&
                   command_action == kTestCommandWaveoutControl &&
                   payload.size() > command_data_offset) {
          const bool enable = payload[command_data_offset] != 0;
          const bool speaker_waveout = enable && port.speaker_waveout_selected;
          port.speaker_waveout_active = speaker_waveout;
          port.speaker_waveout_phase = 0;
          port.waveout_extractor =
              PcmAudioExtractor{kBridgeSpeakerInputFrames};
          port.output_state.set_audio_out_stream_active(speaker_waveout,
                                                        port.headset_plugged);
          port.audio_out_stream_active = speaker_waveout;
          if (!speaker_waveout) {
            port.pending_audio_chunks.clear();
            port.audio_primed = false;
          }
          if (bt != nullptr) {
            bridge_forward_state_if_changed(
                port, *bt, trace_flags, logger,
                speaker_waveout ? "WebHID speaker waveout route on"
                                : "WebHID waveout route off");
          }
          if (output_trace) {
            logger.log(LogScope::Output, LogLevel::Info,
                       port.path + " WebHID waveout " +
                           std::string(enable ? "on" : "off") + " target=" +
                           std::string(port.speaker_waveout_selected
                                           ? "speaker"
                                           : "headphone") +
                           " synthesized=" + (speaker_waveout ? "yes" : "no"));
          }
        }
      }
    }
    if (bt != nullptr) {
      bt->set_feature_report(payload);
    }
    if (output_trace) {
      trace_output_latency(port.path, "feature_set",
                           port.trace_state.output_feature_latency,
                           Clock::now() - start, kOutputTraceFeatureSlowWarn,
                           logger);
    }
    return;
  }

  if (type != VDS_FRAME_USB_AUDIO_OUT || bt == nullptr) {
    return;
  }

  std::size_t queued_chunks = 0;
  std::size_t dropped_chunks = 0;
  const auto audio_start = Clock::now();
  const auto extract_start = Clock::now();
  port.extractor.set_speaker_boost_db(bridge_speaker_boost_db(port));
  port.extractor.set_speaker_volume_percent(
      bridge_speaker_volume_percent(port));
  port.extractor.set_haptics_gain_percent(port.haptics_gain_percent);
  port.extractor.set_speaker_mono(!port.headset_plugged);
  const auto chunks = port.extractor.push_usb_audio(payload);
  const auto extract_duration = Clock::now() - extract_start;
  for (const auto &chunk : chunks) {
    if (output_trace) {
      if (chunk.has_haptics_signal) {
        if (!port.trace_state.haptics_burst_active) {
          logger.log(LogScope::Output, LogLevel::Debug,
                     port.path + " haptics burst start");
          port.trace_state.haptics_burst_active = true;
          port.trace_state.haptics_burst_chunks = 0;
        }
        ++port.trace_state.haptics_burst_chunks;
      } else if (port.trace_state.haptics_burst_active) {
        logger.log(LogScope::Output, LogLevel::Debug,
                   port.path + " haptics burst end chunks=" +
                       std::to_string(port.trace_state.haptics_burst_chunks));
        port.trace_state.haptics_burst_active = false;
        port.trace_state.haptics_burst_chunks = 0;
      }
    }

    /*
     * USB isochronous URBs can arrive in bursts. Queue completed 0x36 audio
     * chunks here and let bridge_flush_audio() send them at the
     * kBridgeAudioOutputInterval speaker frame interval, otherwise
     * speaker/haptics audio turns into audible bursts.
     */
    if (port.pending_audio_chunks.size() >= kMaxPendingAudioChunks) {
      port.pending_audio_chunks.pop_front();
      ++dropped_chunks;
      ++port.trace_state.dropped_audio_haptics_count;
      ++port.trace_state.queue_dropped_audio_haptics_count;
    }

    port.pending_audio_chunks.push_back(chunk);
    ++queued_chunks;
  }
  if (dropped_chunks > 0 &&
      (port.trace_state.dropped_audio_haptics_count == dropped_chunks ||
       port.trace_state.dropped_audio_haptics_count % 1000 == 0)) {
    logger.log(
        LogScope::Output, LogLevel::Warn,
        port.path + " dropped BT 0x36 haptics packets count=" +
            std::to_string(port.trace_state.dropped_audio_haptics_count) +
            " queue=" +
            std::to_string(port.trace_state.queue_dropped_audio_haptics_count) +
            " blocked=" +
            std::to_string(port.trace_state.blocked_audio_haptics_count));
  }
  if (output_trace && queued_chunks > 0) {
    logger.log(
        LogScope::Output, LogLevel::Debug,
        port.path + " audio out queued " + std::to_string(queued_chunks) +
            " BT 0x36 haptics packets pending=" +
            std::to_string(port.pending_audio_chunks.size()) +
            " extract_encode_us=" +
            std::to_string(duration_us(extract_duration)) + " total_us=" +
            std::to_string(duration_us(Clock::now() - audio_start)));
  }
  if (output_trace) {
    trace_output_latency(port.path, "audio_out",
                         port.trace_state.output_audio_latency,
                         Clock::now() - audio_start, kOutputTraceSlowWarn,
                         logger);
    trace_output_latency(port.path, "audio_extract_encode",
                         port.trace_state.output_audio_extract_latency,
                         extract_duration, kOutputTraceSlowWarn, logger);
  }
}

BridgeInputResult bridge_handle_bt_input(BridgePort &port,
                                         std::span<const std::uint8_t> packet,
                                         Clock::time_point read_time,
                                         BridgeBluetooth &bt, BridgeUsb &usb,
                                         CompanionRuntime &companion,
                                         std::uint32_t trace_flags,
                                         Logger &logger) {
  BridgeInputResult result;
  BridgeSettings &settings = bridge_settings();

  if (bt_input_payload_type(packet) == BtInputPayloadType::Audio) {
    const bool input_audio_trace = trace_enabled(trace_flags, kTraceInputAudio);
    ++port.trace_state.bt_mic_packet_count;
    if (!port.audio_in_stream_active) {
      ++port.trace_state.bt_mic_drop_count;
      // The mic should be closed: tell the controller again so it stops
      // using airtime on mic packets nobody reads.
      BridgeAudioStats &stats = port.audio_stats;
      ++stats.mic_while_closed_total;
      if (read_time - stats.last_mic_close_resend >= std::chrono::seconds(1)) {
        stats.last_mic_close_resend = read_time;
        const bool sent = bt.try_send_output_report(
            port.output_state.build_bt_mic_report(false,
                                                   mic_buffer_length(port)));
        logger.log(LogScope::InputAudio, LogLevel::Info,
                   port.path + " mic packets while closed, re-sent close sent=" +
                       (sent ? "yes" : "no"));
      }
      if (input_audio_trace &&
          (port.trace_state.bt_mic_drop_count == 1 ||
           port.trace_state.bt_mic_drop_count % 1000 == 0)) {
        logger.log(LogScope::InputAudio, LogLevel::Debug,
                   port.path + " dropped mic packet count=" +
                       std::to_string(port.trace_state.bt_mic_drop_count) +
                       " reason=audio_in_stream_off");
      }
      return result;
    }

    const auto opus_payload = bt_mic_opus_payload(packet);
    if (!opus_payload) {
      ++port.trace_state.bt_mic_decode_fail_count;
      if (input_audio_trace) {
        logger.log(LogScope::InputAudio, LogLevel::Warn,
                   port.path + " malformed mic packet len=" +
                       std::to_string(packet.size()));
      }
      return result;
    }

    ++port.audio_stats.mic_packets_total;
    ++port.audio_stats.window_mic_packets;
    if (!port.mic_first_packet) {
      port.mic_first_packet = true;
      port.audio_stats.mic_startup_ms =
          std::chrono::duration_cast<std::chrono::milliseconds>(
              read_time - port.mic_opened_at)
              .count();
      logger.log(LogScope::InputAudio, LogLevel::Info,
                 port.path + " controller mic first packet after " +
                     std::to_string(port.audio_stats.mic_startup_ms) +
                     " ms, open re-sent " +
                     std::to_string(port.mic_open_resends) + " times");
    }
    try {
      auto pcm = port.mic_decoder.decode(*opus_payload);
      port.trace_state.mic_peak_permille =
          scale_mic_pcm(pcm, settings.mic_volume_percent, port.mic_muted);
      if (!port.mic_first_sound &&
          port.trace_state.mic_peak_permille >= kMicSoundPermille) {
        port.mic_first_sound = true;
        port.audio_stats.mic_first_sound_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                read_time - port.mic_opened_at)
                .count();
        logger.log(LogScope::InputAudio, LogLevel::Info,
                   port.path + " controller mic first sound after " +
                       std::to_string(port.audio_stats.mic_first_sound_ms) +
                       " ms");
      }
      port.trace_state.mic_last_decoded_bytes =
          static_cast<std::uint16_t>(pcm.size());
      if (!port.audio_in_requested) {
        return result; // mic kept ready; nothing is recording
      }
      const bool wrote = write_usb_frame(port, usb, VDS_FRAME_USB_AUDIO_IN, pcm,
                                         input_audio_trace, logger);
      ++port.trace_state.audio_in_forward_count;
      if (!wrote) {
        ++port.trace_state.audio_in_write_short_count;
      }
      if (input_audio_trace && (port.trace_state.audio_in_forward_count == 1 ||
                                port.trace_state.audio_in_forward_count %
                                        kInputTraceSummaryInterval ==
                                    0)) {
        logger.log(LogScope::InputAudio, LogLevel::Debug,
                   port.path + " mic forwarded count=" +
                       std::to_string(port.trace_state.audio_in_forward_count) +
                       " len=" + std::to_string(pcm.size()) +
                       " written=" + (wrote ? "yes" : "no"));
      }
    } catch (const std::exception &error) {
      ++port.trace_state.bt_mic_decode_fail_count;
      if (input_audio_trace) {
        logger.log(
            LogScope::InputAudio, LogLevel::Warn,
            port.path + " mic decode failed count=" +
                std::to_string(port.trace_state.bt_mic_decode_fail_count) +
                ": " + error.what());
      }
    }
    return result;
  }

  auto report = bt_input_to_usb_input(packet);
  if (!report) {
    // Not controller input or mic audio. Logged without tracing so an
    // unexpected header shows up if input ever stops.
    if (packet.size() < 2 || packet[1] != VDS_BT_STATE_REPORT_ID) {
      return result; // e.g. the basic 0x01 report sent once at connect
    }
    ++port.trace_state.bt_input_unknown_count;
    if (port.trace_state.bt_input_unknown_count == 1 ||
        port.trace_state.bt_input_unknown_count % 1000 == 0) {
      logger.log(LogScope::InputControl, LogLevel::Warn,
                 port.path + " ignored BT input packet count=" +
                     std::to_string(port.trace_state.bt_input_unknown_count) +
                     " len=" + std::to_string(packet.size()) +
                     " head=" + hex_bytes(packet, 4));
    }
    return result;
  }
  // Idle Disconnect, checked on the raw report (before shortcuts or remapping
  // hide buttons). The platform turns the controller off, which drops the
  // link like the companion's Turn Off; the normal hang-up path cleans up.
  if (port.last_input_activity == Clock::time_point{} ||
      controller_input_active(*report)) {
    port.last_input_activity = read_time;
  } else if (companion.settings.idle_disconnect_enabled &&
             read_time - port.last_input_activity >
                 std::chrono::minutes(
                     companion.settings.idle_disconnect_timeout_minutes)) {
    port.last_input_activity = read_time;
    logger.log(LogScope::Bluetooth, LogLevel::Info,
               port.path + " idle for " +
                   std::to_string(
                       companion.settings.idle_disconnect_timeout_minutes) +
                   " min; turning controller off");
    result.idle_timeout = true;
  }
  companion_translate_input(companion, port.companion_input,
                            std::span<std::uint8_t>(*report));
  // USB input payload offset 52 (report byte 53) carries the DualSense
  // power status: low nibble battery capacity 0-10, high nibble state.
  port.battery_status = (*report)[53];

  const bool input_trace = trace_enabled(trace_flags, kTraceInputControl);
  if (input_trace) {
    ++port.trace_state.bt_input_count;
    if (port.trace_state.have_last_input_time) {
      const auto gap = read_time - port.trace_state.last_input_time;
      const auto gap_us = static_cast<std::uint64_t>(
          std::chrono::duration_cast<std::chrono::microseconds>(gap).count());
      port.trace_state.input_gap_max_us =
          std::max(port.trace_state.input_gap_max_us, gap_us);
      if (gap >= kInputTraceGapWarn) {
        logger.log(LogScope::InputControl, LogLevel::Warn,
                   port.path + " input gap count=" +
                       std::to_string(port.trace_state.bt_input_count) +
                       " gap_us=" + std::to_string(gap_us));
      }
    }
    port.trace_state.last_input_time = read_time;
    port.trace_state.have_last_input_time = true;

    const std::span buttons(report->data() + kUsbInputButtonsOffset,
                            kUsbInputButtonsSize);
    const bool buttons_changed =
        !port.trace_state.have_last_input_buttons ||
        !std::equal(buttons.begin(), buttons.end(),
                    port.trace_state.last_input_buttons.begin());
    if (buttons_changed) {
      std::copy(buttons.begin(), buttons.end(),
                port.trace_state.last_input_buttons.begin());
      port.trace_state.have_last_input_buttons = true;
      logger.log(LogScope::InputControl, LogLevel::Debug,
                 port.path + " buttons changed raw=" +
                     hex_bytes(buttons, kUsbInputButtonsSize));
    }
  }

  const std::uint8_t headset_status = report->at(kUsbInputHeadsetOffset);
  const bool headset_plugged =
      (headset_status & kUsbInputHeadphonesPluggedMask) != 0;
  const bool headset_mic_plugged =
      (headset_status & kUsbInputMicPluggedMask) != 0;
  const bool mute_button_down =
      (report->at(kUsbInputMuteButtonOffset) & kUsbInputMuteButtonMask) != 0;
  bool headset_mic_changed = false;
  bool mic_mute_changed = false;
  bool output_state_changed = false;
  // Mute switches the mic on/off (pass-through + mute together, like the
  // app's Microphone switch) unless the Mute button mode is Chord (Mute starts
  // chords); vdsd has no keyboard/quiet modes, so those act as Normal. The app
  // follows through the status report (mic muted = mic off).
  constexpr std::uint8_t kMuteButtonModeChord = 3;
  if (mute_button_down && !port.mute_button_down &&
      companion.settings.mute_button_mode != kMuteButtonModeChord) {
    const bool mic_on = !settings.mic_live();
    companion.settings.duplex_enabled = mic_on;
    companion.settings.mic_muted = !mic_on;
    ++companion.settings_revision;
    settings.mic_allowed = settings.mic_forced || mic_on;
    settings.mic_muted = !mic_on;
    mic_mute_changed = true;
    // DS5 Bridge MIC_MUTE_ON (0x04) / MIC_MUTE_OFF (0x05) input events.
    if (companion.pending_input_events.size() <
        kCompanionMaxPendingInputEvents) {
      companion.pending_input_events.push_back(mic_on ? 0x05 : 0x04);
    }
  }
  port.mute_button_down = mute_button_down;
  if (headset_mic_plugged != port.headset_mic_plugged) {
    port.headset_mic_plugged = headset_mic_plugged;
    headset_mic_changed = true;
  }
  if (mic_mute_changed) {
    bridge_sync_mic(port, &bt, logger);
    bridge_forward_state_if_changed(port, bt, trace_flags, logger, "mic mute");
    logger.log(LogScope::InputAudio, LogLevel::Info,
               port.path + " mic " +
                   std::string(settings.mic_live() ? "on" : "off") +
                   " (Mute button)");
  } else if (headset_mic_changed && port.audio_in_stream_active) {
    bt.try_send_output_report(
        port.output_state.build_bt_mic_state_report(true, port.mic_muted));
  }
  if (headset_plugged != port.headset_plugged) {
    port.headset_plugged = headset_plugged;
    if (port.audio_out_stream_active || port.speaker_waveout_active) {
      port.output_state.set_audio_out_stream_active(true, port.headset_plugged);
      output_state_changed = true;
    }
    logger.log(LogScope::Output, LogLevel::Info,
               port.path + (port.headset_plugged
                                ? " headphones plugged in; stereo, "
                                  "speaker boost off"
                                : " headphones unplugged; mono speaker, "
                                  "boost on"));
  }
  if (output_state_changed) {
    bridge_forward_state_if_changed(port, bt, trace_flags, logger,
                                    "headset route change");
  }
  if (headset_mic_changed && input_trace) {
    logger.log(LogScope::InputControl, LogLevel::Info,
               port.path + " headset mic " +
                   std::string(port.headset_mic_plugged ? "plugged"
                                                        : "unplugged"));
  }

  // DualShock 4 / Xbox 360 personas get the DualSense report translated.
  std::span<const std::uint8_t> host_report(*report);
  const std::vector<std::uint8_t> persona_report =
      persona_input_report(port.persona, host_report, port.persona_state);
  if (!persona_report.empty()) {
    host_report = persona_report;
  }

  const auto write_start = Clock::now();
  const bool wrote = write_usb_frame(port, usb, VDS_FRAME_USB_HID_IN,
                                     host_report, input_trace, logger);
  const auto write_duration = Clock::now() - write_start;
  if (input_trace) {
    const auto write_us = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(write_duration)
            .count());
    ++port.trace_state.input_write_count;
    port.trace_state.input_write_total_us += write_us;
    port.trace_state.input_write_max_us =
        std::max(port.trace_state.input_write_max_us, write_us);

    if (write_duration >= kInputTraceSlowWriteWarn) {
      logger.log(LogScope::InputControl, LogLevel::Warn,
                 port.path + " slow input write count=" +
                     std::to_string(port.trace_state.bt_input_count) +
                     " write_us=" + std::to_string(write_us) +
                     " written=" + (wrote ? "yes" : "no"));
    }
    if (port.trace_state.bt_input_count == 1 ||
        port.trace_state.bt_input_count % kInputTraceSummaryInterval == 0) {
      const std::uint64_t avg_write_us =
          port.trace_state.input_write_count == 0
              ? 0
              : port.trace_state.input_write_total_us /
                    port.trace_state.input_write_count;
      logger.log(LogScope::InputControl, LogLevel::Debug,
                 port.path + " input summary count=" +
                     std::to_string(port.trace_state.bt_input_count) +
                     " len=" + std::to_string(report->size()) +
                     " avg_write_us=" + std::to_string(avg_write_us) +
                     " max_write_us=" +
                     std::to_string(port.trace_state.input_write_max_us) +
                     " max_gap_us=" +
                     std::to_string(port.trace_state.input_gap_max_us));
    }
  }
  return result;
}

/*
 * After the queue runs dry, hold sending until port.audio_prime_chunks are
 * buffered (2 on Linux, ~21 ms). Input and output run at the same rate, so
 * this stays as latency on every speaker sample and haptic for the rest of the
 * stream; AudioQueueTrimmer keeps it from growing past this. One chunk is not
 * enough: USB delivers audio in bursts that do not line up with the send
 * times, and the queue keeps running dry (tested: refills climbed
 * continuously). The WebHID waveout generator only queues one chunk at a
 * time, so it is exempt.
 */
bool bridge_audio_chunk_ready(const BridgePort &port) {
  const std::size_t needed = port.audio_primed || port.speaker_waveout_active
                                 ? 1
                                 : port.audio_prime_chunks;
  return port.pending_audio_chunks.size() >= needed;
}

std::optional<Clock::time_point>
bridge_next_wakeup(const BridgePort &port, Clock::time_point now) {
  std::optional<Clock::time_point> wake;
  const auto at_most = [&](Clock::time_point when) {
    wake = wake ? std::min(*wake, when) : when;
  };
  if (bridge_mic_retry_pending(port)) {
    at_most(port.mic_open_resent_at + kBridgeMicOpenRetry);
  }
  if (port.pending_bt_state_report) {
    at_most(now + std::chrono::milliseconds(kBridgePendingOutputPollMs));
  }
  const bool audio_due =
      bridge_audio_chunk_ready(port) || port.speaker_waveout_active;
  if (audio_due) {
    const auto next_time = port.next_haptics_send_time;
    at_most(next_time == Clock::time_point{} ? now : next_time);
  }
  return wake;
}

std::optional<int> bridge_wakeup_ms(const BridgePort &port,
                                    Clock::time_point now) {
  const auto wake = bridge_next_wakeup(port, now);
  if (!wake) {
    return std::nullopt;
  }
  if (*wake <= now) {
    return 0;
  }
  return static_cast<int>(
      std::chrono::ceil<std::chrono::milliseconds>(*wake - now).count());
}

bool bridge_flush_audio(BridgePort &port, BridgeBluetooth &bt,
                        std::uint32_t trace_flags, Logger &logger) {
  if (port.pending_audio_chunks.empty()) {
    if (port.audio_primed) {
      // Ran dry mid-stream: the next packets wait for a refill.
      ++port.audio_stats.refills_total;
      ++port.audio_stats.window_refills;
    }
    port.audio_primed = false;
    return true;
  }
  if (!bridge_audio_chunk_ready(port)) {
    return true;
  }

  const auto now = Clock::now();
  if (port.next_haptics_send_time != Clock::time_point{} &&
      now < port.next_haptics_send_time) {
    return true;
  }

  if (!port.audio_primed) {
    port.queue_trimmer.reset();
  }
  if (port.queue_trimmer.should_trim(port.pending_audio_chunks.size(),
                                     port.audio_prime_chunks)) {
    // A standing backlog is pure haptics latency: skip one chunk to drain it.
    port.pending_audio_chunks.pop_front();
    ++port.audio_stats.trims_total;
  }

  const bool output_trace = trace_enabled(trace_flags, kTraceOutput);
  const auto &chunk = port.pending_audio_chunks.front();
  // Haptics gain is applied by the extractor before 8-bit quantization.
  // Four packet kinds, the smallest that carries what is live: 0x33 when the
  // speaker is silent, 0x36 with speaker audio, each with the mic section
  // enabled only while the mic records (buffer at least the mic's length).
  const bool mic_open = port.audio_in_stream_active;
  port.haptics_builder.set_buffer_length(
      mic_open ? mic_buffer_length(port) : port.audio_stats.buffer_length);
  const bool haptics_only =
      !port.speaker_waveout_active &&
      port.silent_speaker_packets >= kHapticsOnlyAfterSilentPackets &&
      !chunk.has_speaker_signal;
  const auto send_start = Clock::now();
  const bool sent =
      haptics_only
          ? bt.try_send_output_report(
                port.haptics_builder.build_haptics_only_packet(
                    chunk.haptics, port.output_state.state(), mic_open,
                    port.headset_plugged))
          : bt.try_send_output_report(port.haptics_builder.build_packet(
                chunk.haptics, chunk.speaker, port.output_state.state(),
                mic_open, port.headset_plugged));
  if (!sent) {
    ++port.trace_state.dropped_audio_haptics_count;
    ++port.trace_state.blocked_audio_haptics_count;
    port.next_haptics_send_time = Clock::now() + kHapticsOutputBlockedRetry;
    if (output_trace &&
        (port.trace_state.blocked_audio_haptics_count == 1 ||
         port.trace_state.blocked_audio_haptics_count % 1000 == 0)) {
      logger.log(
          LogScope::Output, LogLevel::Warn,
          port.path + " pending BT 0x36 haptics packet still blocked count=" +
              std::to_string(port.trace_state.blocked_audio_haptics_count));
    }
    return false;
  }

  const auto send_duration = Clock::now() - send_start;
  port.silent_speaker_packets =
      chunk.has_speaker_signal
          ? 0
          : std::min(port.silent_speaker_packets + 1,
                     kHapticsOnlyAfterSilentPackets);
  port.audio_stats.haptics_only = haptics_only;
  if (haptics_only) {
    ++port.audio_stats.haptics_only_total;
  }
  port.pending_audio_chunks.pop_front();
  port.audio_primed = true;
  // 0x36 packets omit mic fields, so a pending mic mute/unmute still needs
  // its own 0x31 state report.
  port.last_sent_bt_state = state_after_audio_packet(
      port.last_sent_bt_state, port.output_state.state());
  if (port.pending_bt_state &&
      *port.pending_bt_state == *port.last_sent_bt_state) {
    port.pending_bt_state.reset();
    port.pending_bt_state_report.reset();
  }
  // Step from the previous deadline to keep the packet cadence without
  // accumulating wakeup rounding. A slightly late send catches up on the next
  // wakeup; one more than kMaxAudioCatchup late resyncs to now instead of
  // replaying a backlog.
  const auto sent_at = Clock::now();
  BridgeAudioStats &stats = port.audio_stats;
  if (stats.last_send != Clock::time_point{}) {
    const auto gap = sent_at - stats.last_send;
    // Longer pauses are the stream stopping, not a late packet.
    if (gap < std::chrono::seconds(1)) {
      const auto gap_us =
          std::chrono::duration_cast<std::chrono::microseconds>(gap).count();
      stats.window_max_gap_us =
          std::max<std::int64_t>(stats.window_max_gap_us, gap_us);
      const auto buffer_us = static_cast<std::int64_t>(stats.buffer_length) *
                             1000000 / VDS_HAPTICS_SAMPLE_RATE;
      if (gap_us > buffer_us) {
        ++stats.late_total;
        ++stats.window_late;
      }
      if (stats.buffer_auto) {
        stats.buffer_length = port.buffer_tuner.on_send(gap_us);
        port.haptics_builder.set_buffer_length(stats.buffer_length);
      }
    }
  }
  stats.last_send = sent_at;
  ++stats.packets_total;
  ++stats.window_packets;
  auto next_deadline = port.next_haptics_send_time;
  if (next_deadline == Clock::time_point{}) {
    next_deadline = send_start;
  }
  next_deadline += kBridgeAudioOutputInterval;
  if (sent_at - next_deadline > kMaxAudioCatchup) {
    next_deadline = sent_at + kBridgeAudioOutputInterval;
  }
  port.next_haptics_send_time = next_deadline;

  if (output_trace) {
    trace_output_latency(port.path, "audio_bt_send",
                         port.trace_state.output_audio_send_latency,
                         send_duration, kOutputTraceSlowWarn, logger);
  }
  return true;
}

void bridge_enqueue_waveout(BridgePort &port, std::uint32_t trace_flags,
                            Logger &logger) {
  if (!port.speaker_waveout_active || !port.pending_audio_chunks.empty()) {
    return;
  }

  std::array<std::uint8_t,
             kPcmWindowFrames * VDS_AUDIO_CHANNELS * sizeof(std::int16_t)>
      pcm{};
  for (std::size_t frame = 0; frame < kPcmWindowFrames; ++frame) {
    const double angle = kSpeakerWaveoutTwoPi *
                         static_cast<double>(port.speaker_waveout_phase) /
                         static_cast<double>(kSpeakerWaveoutPeriodFrames);
    const auto sample = static_cast<std::int16_t>(
        std::sin(angle) * static_cast<double>(kSpeakerWaveoutAmplitude));
    port.speaker_waveout_phase =
        (port.speaker_waveout_phase + 1) % kSpeakerWaveoutPeriodFrames;

    for (std::size_t channel = 0; channel < kSpeakerChannels; ++channel) {
      const std::size_t offset =
          (frame * VDS_AUDIO_CHANNELS + channel) * sizeof(std::int16_t);
      const auto value = static_cast<std::uint16_t>(sample);
      pcm[offset + 0] = static_cast<std::uint8_t>(value & 0xff);
      pcm[offset + 1] = static_cast<std::uint8_t>((value >> 8) & 0xff);
    }
  }

  port.waveout_extractor.set_speaker_boost_db(bridge_speaker_boost_db(port));
  port.waveout_extractor.set_speaker_volume_percent(
      bridge_speaker_volume_percent(port));
  port.waveout_extractor.set_haptics_gain_percent(port.haptics_gain_percent);
  port.waveout_extractor.set_speaker_mono(!port.headset_plugged);
  const auto chunks = port.waveout_extractor.push_usb_audio(pcm);
  for (const auto &chunk : chunks) {
    if (port.pending_audio_chunks.size() >= kMaxPendingAudioChunks) {
      break;
    }
    port.pending_audio_chunks.push_back(chunk);
  }
  if (trace_enabled(trace_flags, kTraceOutput) && !chunks.empty()) {
    logger.log(LogScope::Output, LogLevel::Debug,
               port.path + " queued WebHID speaker waveout chunk pending=" +
                   std::to_string(port.pending_audio_chunks.size()));
  }
}

void bridge_retry_mic_open(BridgePort &port, BridgeBluetooth &bt,
                           Logger &logger) {
  if (!bridge_mic_retry_pending(port)) {
    return;
  }
  const auto now = Clock::now();
  if (now - port.mic_open_resent_at < kBridgeMicOpenRetry) {
    return;
  }
  port.mic_open_resent_at = now;
  ++port.mic_open_resends;
  bt.try_send_output_report(
      port.output_state.build_bt_mic_state_report(true, port.mic_muted));
  const bool sent = bt.try_send_output_report(
      port.output_state.build_bt_mic_report(true, mic_buffer_length(port)));
  logger.log(LogScope::InputAudio, LogLevel::Debug,
             port.path + " controller mic open re-sent count=" +
                 std::to_string(port.mic_open_resends) +
                 " sent=" + (sent ? "yes" : "no"));
}

bool bridge_mic_retry_pending(const BridgePort &port) {
  return port.audio_in_stream_active && !port.mic_first_packet &&
         port.mic_open_resends < kBridgeMicOpenMaxResends;
}

DsCompanionOverrides bridge_companion_overrides(CompanionRuntime &companion) {
  const CompanionSettings &settings = companion.settings;
  const CompanionActuation &actuation = companion.actuation;

  BridgeSettings &bridge = bridge_settings();
  bridge.host_persona = host_persona_from_value(settings.host_persona_mode)
                            .value_or(HostPersona::DualSense);
  bridge.mic_allowed = bridge.mic_forced || settings.duplex_enabled;
  bridge.mic_muted = settings.mic_muted;
  bridge.mic_volume_percent = settings.mic_volume_percent;
  bridge.mute_led_enabled = settings.mute_led_enabled;
  bridge.speaker_boost_db = settings.speaker_boost_db;

  DsCompanionOverrides overrides;
  overrides.lightbar_override = settings.lightbar_override_enabled;
  overrides.lightbar_color = {settings.lightbar_red, settings.lightbar_green,
                              settings.lightbar_blue};
  overrides.lightbar_brightness_percent = settings.lightbar_brightness_percent;
  overrides.player_led_enabled = settings.player_led_enabled;
  overrides.classic_rumble_gain_percent = settings.classic_rumble_gain_percent;
  overrides.classic_rumble_v1 = settings.classic_rumble_v1;
  overrides.trigger_intensity_percent =
      settings.trigger_effect_intensity_percent;
  overrides.speaker_volume_percent = settings.speaker_volume_percent;
  overrides.speaker_gain_level = settings.speaker_gain_level;
  overrides.mic_processing = settings.mic_processing;
  overrides.mic_speaker_gain_level = settings.mic_speaker_gain_level;
  overrides.test_rumble_active = actuation.test_rumble_active;
  overrides.test_rumble_power = actuation.test_rumble_power;

  if (actuation.test_trigger.active) {
    const CompanionTriggerEffect &effect = actuation.test_trigger;
    // Target: 0 both, 1 left, 2 right (DS5 Bridge firmware convention).
    if (effect.target != 2) {
      overrides.left_trigger_active = true;
      encode_companion_trigger_effect_v2(
          std::span<std::uint8_t, kTriggerEffectSize>(overrides.left_trigger),
          effect);
    }
    if (effect.target != 1) {
      overrides.right_trigger_active = true;
      encode_companion_trigger_effect_v2(
          std::span<std::uint8_t, kTriggerEffectSize>(overrides.right_trigger),
          effect);
    }
  } else {
    // Persistent effects are stored per trigger; render each independently.
    const CompanionTriggerEffect &left = actuation.persistent_trigger_left;
    if (left.active) {
      overrides.left_trigger_active = true;
      encode_companion_trigger_effect_v2(
          std::span<std::uint8_t, kTriggerEffectSize>(overrides.left_trigger),
          left);
    }
    const CompanionTriggerEffect &right = actuation.persistent_trigger_right;
    if (right.active) {
      overrides.right_trigger_active = true;
      encode_companion_trigger_effect_v2(
          std::span<std::uint8_t, kTriggerEffectSize>(overrides.right_trigger),
          right);
    }
  }
  return overrides;
}

void bridge_apply_companion(BridgePort &port, const CompanionSettings &settings,
                            const DsCompanionOverrides &overrides,
                            BridgeBluetooth &bt, std::uint32_t trace_flags,
                            Logger &logger) {
  port.haptics_gain_percent = settings.haptics_gain_percent;
  port.audio_stats.buffer_auto = settings.haptics_buffer_auto;
  port.audio_stats.buffer_length = settings.haptics_buffer_auto
                                       ? port.buffer_tuner.buffer_length()
                                       : settings.haptics_buffer_length;
  port.haptics_builder.set_buffer_length(port.audio_stats.buffer_length);
  port.output_state.set_companion_overrides(overrides);
  bridge_sync_mic(port, &bt, logger);
  try {
    bridge_forward_state_if_changed(port, bt, trace_flags, logger,
                                    "companion update");
  } catch (const std::exception &error) {
    logger.log(LogScope::Companion, LogLevel::Warn,
               port.path + " companion state send failed: " + error.what());
  }
}

std::string bridge_audio_stats_json(BridgePort &port, Clock::time_point now) {
  const auto fixed = [](double value, int decimals) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(decimals) << value;
    return out.str();
  };
  BridgeAudioStats &stats = port.audio_stats;
  const double window_s =
      stats.window_start == Clock::time_point{}
          ? 0.0
          : std::chrono::duration<double>(now - stats.window_start).count();
  const bool streaming = stats.last_send != Clock::time_point{} &&
                         now - stats.last_send < std::chrono::milliseconds(250);
  std::string reply = "{";
  reply += jsonl_string_field("path", port.path);
  reply += ',';
  reply += jsonl_bool_field("streaming", streaming);
  reply += ",\"buffer_length\":" + std::to_string(stats.buffer_length);
  reply += ",\"buffer_ms\":" +
           fixed(1000.0 * stats.buffer_length / VDS_HAPTICS_SAMPLE_RATE, 1);
  reply += std::string(",\"buffer_auto\":") +
           (stats.buffer_auto ? "true" : "false");
  reply += ",\"trims_total\":" + std::to_string(stats.trims_total);
  reply += std::string(",\"haptics_only\":") +
           (stats.haptics_only ? "true" : "false");
  // haptics | haptics+speaker | haptics+mic | haptics+speaker+mic, or mic
  // (capture with no output stream) / idle.
  const bool mic_open = port.audio_in_stream_active;
  std::string packet_mode =
      streaming ? (stats.haptics_only ? "haptics" : "haptics+speaker")
                : (mic_open ? "mic" : "idle");
  if (streaming && mic_open) {
    packet_mode += "+mic";
  }
  reply += ',';
  reply += jsonl_string_field("packet_mode", packet_mode);
  reply += ',';
  reply += jsonl_bool_field("mic_open", mic_open);
  reply += ",\"mic_packets_per_second\":" +
           fixed(window_s > 0.0 ? stats.window_mic_packets / window_s : 0.0, 1);
  reply += ",\"mic_packets_total\":" + std::to_string(stats.mic_packets_total);
  reply += ",\"mic_startup_ms\":" + std::to_string(stats.mic_startup_ms);
  reply += ",\"mic_first_sound_ms\":" + std::to_string(stats.mic_first_sound_ms);
  reply += ",\"mic_while_closed_total\":" +
           std::to_string(stats.mic_while_closed_total);
  reply += ",\"haptics_only_total\":" + std::to_string(stats.haptics_only_total);
  reply += ",\"window_seconds\":" + fixed(window_s, 2);
  reply += ",\"packets_per_second\":" +
           fixed(window_s > 0.0 ? stats.window_packets / window_s : 0.0, 1);
  reply += ",\"max_gap_ms\":" + fixed(stats.window_max_gap_us / 1000.0, 1);
  reply += ",\"late\":" + std::to_string(stats.window_late);
  reply += ",\"refills\":" + std::to_string(stats.window_refills);
  reply += ",\"packets_total\":" + std::to_string(stats.packets_total);
  reply += ",\"late_total\":" + std::to_string(stats.late_total);
  reply += ",\"refills_total\":" + std::to_string(stats.refills_total);
  reply += ",\"queue_depth\":" + std::to_string(port.pending_audio_chunks.size());
  reply += ",\"queue_dropped_total\":" +
           std::to_string(port.trace_state.queue_dropped_audio_haptics_count);
  reply += ",\"blocked_total\":" +
           std::to_string(port.trace_state.blocked_audio_haptics_count);
  reply += ',';
  reply += jsonl_bool_field("headphones", port.headset_plugged);
  reply += ',';
  reply += jsonl_bool_field("speaker_mono", !port.headset_plugged);
  reply += ",\"speaker_boost_db\":" + fixed(bridge_speaker_boost_db(port), 1);
  SpeakerLimiterStats limiter = port.extractor.take_speaker_limiter_stats();
  limiter.merge(port.waveout_extractor.take_speaker_limiter_stats());
  reply += ",\"speaker_limiting_db\":" + fixed(limiter.average_db(), 1);
  reply += ",\"speaker_limiting_max_db\":" + fixed(limiter.max_reduction_db, 1);
  reply += ",\"speaker_volume_percent\":" +
           std::to_string(bridge_speaker_volume_percent(port));
  reply += ",\"haptics_gain_percent\":" +
           std::to_string(port.haptics_gain_percent);
  reply += '}';
  stats.window_start = now;
  stats.window_packets = 0;
  stats.window_mic_packets = 0;
  stats.window_late = 0;
  stats.window_refills = 0;
  stats.window_max_gap_us = 0;
  return reply;
}

CompanionAudioStatus bridge_audio_status(const BridgePort &port) {
  const BridgeTraceState &trace = port.trace_state;
  const auto u32 = [](std::uint64_t value) {
    return static_cast<std::uint32_t>(value);
  };
  CompanionAudioStatus status;
  status.controller_ready = true;
  status.duplex_active = port.audio_in_stream_active;
  status.mic_streaming = port.audio_in_requested;
  status.headset_plugged = port.headset_plugged;
  status.headset_route =
      port.headset_plugged &&
      (port.audio_out_stream_active || port.speaker_waveout_active);
  status.mic_packets_received = u32(trace.bt_mic_packet_count);
  status.mic_packets_dropped = u32(trace.bt_mic_drop_count);
  status.mic_decode_fail = u32(trace.bt_mic_decode_fail_count);
  status.mic_decode_success = u32(trace.audio_in_forward_count);
  status.mic_write_short = u32(trace.audio_in_write_short_count);
  status.mic_write_success =
      u32(trace.audio_in_forward_count - trace.audio_in_write_short_count);
  status.mic_last_decoded_samples = static_cast<std::uint16_t>(
      trace.mic_last_decoded_bytes / (2 * sizeof(std::int16_t)));
  status.mic_last_written_bytes = trace.mic_last_decoded_bytes;
  status.mic_peak_permille = trace.mic_peak_permille;
  return status;
}

} // namespace vds
