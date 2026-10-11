// SPDX-License-Identifier: MIT
// vds_bridge with fake Bluetooth and USB links: the logic both vdsd
// platforms share.
#include <cassert>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <optional>
#include <span>
#include <vector>

#include "uapi/vds.h"
#include "../src/vds_bridge.hh"

namespace {

struct FakeBluetooth final : vds::BridgeBluetooth {
  std::vector<std::vector<std::uint8_t>> sent;
  std::vector<std::uint8_t> feature_gets;
  std::vector<std::vector<std::uint8_t>> feature_sets;
  bool busy = false;
  // Windows-style: answer feature gets directly.
  bool sync_features = false;

  bool try_send_output_report(std::span<const std::uint8_t> report) override {
    if (busy) {
      return false;
    }
    sent.emplace_back(report.begin(), report.end());
    return true;
  }
  std::optional<std::vector<std::uint8_t>>
  get_feature_report(std::uint8_t report_id) override {
    feature_gets.push_back(report_id);
    if (!sync_features) {
      return std::nullopt;
    }
    return std::vector<std::uint8_t>{report_id, 0xaa, 0xbb};
  }
  void set_feature_report(std::span<const std::uint8_t> report) override {
    feature_sets.emplace_back(report.begin(), report.end());
  }

  std::size_t count(std::uint8_t report_id) const {
    std::size_t n = 0;
    for (const auto &report : sent) {
      n += !report.empty() && report[0] == report_id;
    }
    return n;
  }
};

struct FakeUsb final : vds::BridgeUsb {
  struct Frame {
    std::uint32_t type;
    std::vector<std::uint8_t> payload;
  };
  std::vector<Frame> frames;

  bool write_frame(std::uint32_t type,
                   std::span<const std::uint8_t> payload) override {
    frames.push_back({type, {payload.begin(), payload.end()}});
    return true;
  }

  std::size_t count(std::uint32_t type) const {
    std::size_t n = 0;
    for (const auto &frame : frames) {
      n += frame.type == type;
    }
    return n;
  }
};

// A controller input packet as it arrives over Bluetooth (0xa1 0x31 ...);
// USB report byte r is packet byte r + 2.
std::vector<std::uint8_t> input_packet(bool mute_down) {
  std::vector<std::uint8_t> packet(3 + VDS_USB_INPUT_PAYLOAD_SIZE, 0);
  packet[0] = 0xa1;
  packet[1] = VDS_BT_STATE_REPORT_ID;
  packet[2] = 0x01;
  if (mute_down) {
    packet[10 + 2] = 0x04;
  }
  return packet;
}

std::vector<std::uint8_t> mic_packet() {
  std::vector<std::uint8_t> packet(4 + vds::kMicOpusSize, 0);
  packet[0] = 0xa1;
  packet[1] = VDS_BT_STATE_REPORT_ID;
  packet[2] = 0x02;
  return packet;
}

} // namespace

int main() {
  vds::Logger logger(
      (std::filesystem::temp_directory_path() / "vds_bridge_test.log")
          .string());
  vds::BridgeSettings &settings = vds::bridge_settings();

  // Mic off: closed, mute light on. Switching it on opens the controller mic
  // (state + open reports) and turns the light off.
  {
    vds::BridgePort port;
    port.path = "test";
    FakeBluetooth bt;
    settings.mic_allowed = false;
    settings.mic_muted = false;
    vds::bridge_sync_mic(port, &bt, logger);
    assert(!port.audio_in_stream_active);
    assert(port.output_state.state()[8] == 1);

    settings.mic_allowed = true;
    vds::bridge_sync_mic(port, &bt, logger);
    assert(port.audio_in_stream_active);
    assert(bt.count(VDS_BT_STATE_REPORT_ID) == 1); // mic state 0x31
    assert(bt.count(0x32) == 1);                   // mic open
    assert(port.output_state.state()[8] == 0);
    assert(vds::bridge_mic_retry_pending(port));
  }

  // The Mute button flips the mic and queues the companion event; the host
  // still gets the input report.
  {
    vds::BridgePort port;
    port.path = "test";
    FakeBluetooth bt;
    FakeUsb usb;
    vds::CompanionRuntime companion;
    settings.mic_allowed = false;
    settings.mic_muted = true;
    vds::bridge_sync_mic(port, &bt, logger);

    const auto now = vds::BridgeClock::now();
    vds::bridge_handle_bt_input(port, input_packet(true), now, bt, usb,
                                companion, 0, logger);
    assert(settings.mic_live());
    assert(companion.settings.duplex_enabled);
    assert(!companion.settings.mic_muted);
    assert(!companion.pending_input_events.empty() &&
           companion.pending_input_events.back() == 0x05);
    assert(port.audio_in_stream_active);
    assert(usb.count(VDS_FRAME_USB_HID_IN) == 1);

    // Held: no second toggle. Released then pressed: off again.
    vds::bridge_handle_bt_input(port, input_packet(true), now, bt, usb,
                                companion, 0, logger);
    assert(settings.mic_live());
    vds::bridge_handle_bt_input(port, input_packet(false), now, bt, usb,
                                companion, 0, logger);
    vds::bridge_handle_bt_input(port, input_packet(true), now, bt, usb,
                                companion, 0, logger);
    assert(!settings.mic_live());
    assert(companion.pending_input_events.back() == 0x04);
    assert(!port.audio_in_stream_active);
  }

  // Mic packets: dropped while closed (close re-sent), counted while open,
  // forwarded to USB only while the host records.
  {
    vds::BridgePort port;
    port.path = "test";
    FakeBluetooth bt;
    FakeUsb usb;
    vds::CompanionRuntime companion;
    settings.mic_allowed = false;
    settings.mic_muted = false;
    vds::bridge_sync_mic(port, &bt, logger);
    const auto now = vds::BridgeClock::now();
    vds::bridge_handle_bt_input(port, mic_packet(), now, bt, usb, companion, 0,
                                logger);
    assert(port.audio_stats.mic_while_closed_total == 1);
    assert(bt.count(0x32) == 1);

    settings.mic_allowed = true;
    vds::bridge_sync_mic(port, &bt, logger);
    vds::bridge_handle_bt_input(port, mic_packet(), now, bt, usb, companion, 0,
                                logger);
    assert(port.audio_stats.mic_packets_total == 1);
    assert(port.mic_first_packet);
    assert(!vds::bridge_mic_retry_pending(port));
    assert(usb.count(VDS_FRAME_USB_AUDIO_IN) == 0);

    port.audio_in_requested = true;
    vds::bridge_handle_bt_input(port, mic_packet(), now, bt, usb, companion, 0,
                                logger);
    assert(usb.count(VDS_FRAME_USB_AUDIO_IN) == 1);
  }

  // Audio out: two chunks prime the queue, then one paced packet per flush;
  // a busy link keeps the chunk for a retry.
  {
    vds::BridgePort port;
    port.path = "test";
    FakeBluetooth bt;
    FakeUsb usb;
    settings.mic_allowed = false;
    vds::bridge_sync_mic(port, &bt, logger);
    bt.sent.clear();

    vds_usb_interface_event event{};
    event.interface_type = VDS_USB_INTERFACE_AUDIO_OUT;
    event.altsetting = 1;
    vds::bridge_handle_usb_frame(
        port, VDS_FRAME_USB_INTERFACE,
        std::span(reinterpret_cast<const std::uint8_t *>(&event),
                  sizeof(event)),
        &bt, usb, 0, logger);
    assert(port.audio_out_stream_active);

    const std::vector<std::uint8_t> pcm(
        2 * vds::kPcmWindowFrames * VDS_AUDIO_CHANNELS * sizeof(std::int16_t),
        0);
    vds::bridge_handle_usb_frame(port, VDS_FRAME_USB_AUDIO_OUT, pcm, &bt, usb,
                                 0, logger);
    assert(port.pending_audio_chunks.size() == 2);
    assert(vds::bridge_audio_chunk_ready(port));
    assert(vds::bridge_wakeup_ms(port, vds::BridgeClock::now()) == 0);

    bt.busy = true;
    assert(!vds::bridge_flush_audio(port, bt, 0, logger));
    assert(port.pending_audio_chunks.size() == 2);
    bt.busy = false;
    port.next_haptics_send_time = {};
    assert(vds::bridge_flush_audio(port, bt, 0, logger));
    assert(port.pending_audio_chunks.size() == 1);
    assert(bt.count(VDS_BT_HAPTICS_REPORT_ID) == 1);
    assert(port.audio_stats.packets_total == 1);

    const std::string stats =
        vds::bridge_audio_stats_json(port, vds::BridgeClock::now());
    assert(stats.find("\"packet_mode\":\"haptics+speaker\"") !=
           std::string::npos);
  }

  // Feature GET: answered from cache, or fetched (async on Linux, direct on
  // Windows) and handed to the host.
  {
    vds::BridgePort port;
    port.path = "test";
    FakeBluetooth bt;
    FakeUsb usb;
    const std::uint8_t get[] = {0x20};
    vds::bridge_handle_usb_frame(port, VDS_FRAME_USB_FEATURE_GET, get, &bt,
                                 usb, 0, logger);
    assert(bt.feature_gets.size() == 1);
    assert(usb.count(VDS_FRAME_USB_FEATURE_REPLY) == 0);
    const std::uint8_t reply[] = {0x20, 0x01};
    vds::bridge_handle_feature_reply(port, reply, usb, 0, logger);
    assert(usb.count(VDS_FRAME_USB_FEATURE_REPLY) == 1);

    vds::bridge_handle_usb_frame(port, VDS_FRAME_USB_FEATURE_GET, get, &bt,
                                 usb, 0, logger);
    assert(bt.feature_gets.size() == 1); // cache hit
    assert(usb.count(VDS_FRAME_USB_FEATURE_REPLY) == 2);

    FakeBluetooth sync_bt;
    sync_bt.sync_features = true;
    FakeUsb sync_usb;
    vds::BridgePort sync_port;
    const std::uint8_t get09[] = {0x09};
    vds::bridge_handle_usb_frame(sync_port, VDS_FRAME_USB_FEATURE_GET, get09,
                                 &sync_bt, sync_usb, 0, logger);
    assert(sync_usb.count(VDS_FRAME_USB_FEATURE_REPLY) == 1);
    assert(sync_port.pending_feature_reports.empty());
  }

  // Companion settings reach bridge_settings() and the port.
  {
    vds::BridgePort port;
    port.path = "test";
    FakeBluetooth bt;
    vds::CompanionRuntime companion;
    companion.settings.duplex_enabled = true;
    companion.settings.mic_muted = false;
    companion.settings.mute_led_enabled = false;
    companion.settings.haptics_gain_percent = 150;
    const auto overrides = vds::bridge_companion_overrides(companion);
    vds::bridge_apply_companion(port, companion.settings, overrides, bt, 0,
                                logger);
    assert(settings.mic_live());
    assert(port.audio_in_stream_active);
    assert(port.haptics_gain_percent == 150);
    companion.settings.mic_muted = true;
    vds::bridge_apply_companion(port, companion.settings,
                                vds::bridge_companion_overrides(companion), bt,
                                0, logger);
    assert(!port.audio_in_stream_active);
    assert(port.output_state.state()[8] == 0); // light turned off
  }
}
