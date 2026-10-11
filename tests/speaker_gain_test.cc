// SPDX-License-Identifier: MIT
#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "../src/vds_protocol.hh"

int main() {
  vds::DsOutputState state;
  vds::DsCompanionOverrides overrides;
  assert(overrides.speaker_gain_level == 1);
  overrides.speaker_gain_level = 6;
  const auto idle_byte37 = state.state()[37];
  state.set_companion_overrides(overrides);
  // Gain is only applied while the speaker stream is active.
  assert(state.state()[37] == idle_byte37);
  state.set_audio_out_stream_active(true);

  assert(state.state()[37] == 6);
  assert((state.state()[7] & 0x30) == 0x30); // speaker path; low bits: mic select

  std::array<std::int8_t, vds::kHapticsSampleSize> haptics{};
  std::array<std::uint8_t, vds::kSpeakerOpusSize> speaker{};
  vds::HapticsPacketBuilder builder;
  const auto packet = builder.build_packet(haptics, speaker, state.state(), false, false);

  assert(packet[4] == 0xfe);
  // Mic sections on and a headset plugged: headphone speaker block and path.
  vds::HapticsPacketBuilder headset_builder;
  const auto headset_packet =
      headset_builder.build_packet(haptics, speaker, state.state(), true, true);
  assert(headset_packet[4] == 0xff);
  assert(headset_packet[142] == (0x16 | 0x80));
  assert((headset_packet[13 + 7] & 0x30) == 0x00);
  // Audio buffer length: default 64, then whatever the companion app sets.
  for (std::size_t i = 5; i <= 9; ++i) {
    assert(packet[i] == 64);
  }
  vds::HapticsPacketBuilder short_buffer;
  short_buffer.set_buffer_length(24);
  const auto short_packet =
      short_buffer.build_packet(haptics, speaker, state.state(), false, false);
  for (std::size_t i = 5; i <= 9; ++i) {
    assert(short_packet[i] == 24);
  }
  assert((packet[13 + 7] & 0x30) == 0x30);
  assert((packet[13 + 37] & 0x07) == 6);
  assert((packet[13] & 0x40) == 0);
  assert((packet[13 + 1] & 0x01) == 0);
  assert(packet[13 + 6] == 0);
  assert(packet[13 + 8] == 0);
  assert(packet[13 + 5] == 0x64);

  // Companion speaker volume is applied to the audio (SpeakerEnhancer), so the
  // hardware volume register stays at full scale.
  overrides.speaker_volume_percent = 50;
  state.set_companion_overrides(overrides);
  const auto scaled = builder.build_packet(haptics, speaker, state.state(), false, false);
  assert(scaled[13 + 5] == 0x64);

  // vdsd owns the mute light: the host (hid-playstation toggles it on every
  // Mute press) cannot turn it on; it shows mic mute while the mic is on.
  std::array<std::uint8_t, 1 + VDS_SET_STATE_SIZE> mute_light{};
  mute_light[0] = VDS_USB_OUTPUT_REPORT_ID;
  mute_light[1 + 1] = 0x01; // allow_mute_light
  mute_light[1 + 8] = 0x01; // mute light on
  assert(state.apply_usb_output_report(mute_light));
  assert((state.state()[1] & 0x01) != 0);
  assert(state.state()[8] == 0);
  state.set_mic_mute_led(true);
  assert(state.state()[8] == 1);
  state.set_mic_mute_led(false);
  assert(state.state()[8] == 0);

  // While the mic is open: companion mic processing bits and the separate
  // speaker gain; closing restores both.
  {
    vds::DsOutputState mic_state;
    vds::DsCompanionOverrides mic_overrides;
    assert(mic_overrides.mic_processing == vds::kMicProcessingEchoCancel);
    mic_overrides.mic_processing = vds::kMicProcessingNoiseCancel;
    mic_overrides.speaker_gain_level = 3;
    mic_state.set_companion_overrides(mic_overrides);
    mic_state.set_audio_out_stream_active(true);
    const auto opened = mic_state.build_bt_mic_state_report(true, false);
    assert((mic_state.state()[7] & 0x0c) == 0x08);
    assert((mic_state.state()[37] & 0x07) == 3);
    assert((opened[3 + 7] & 0x0c) == 0x08); // 0x31: state at byte 3
    mic_overrides.mic_processing = vds::kMicProcessingEchoCancel;
    mic_overrides.mic_speaker_gain_level = 5;
    mic_state.set_companion_overrides(mic_overrides);
    assert((mic_state.state()[7] & 0x0c) == 0x04);
    assert((mic_state.state()[37] & 0x07) == 5);
    mic_overrides.mic_processing = 0;
    mic_state.set_companion_overrides(mic_overrides);
    assert((mic_state.state()[7] & 0x0c) == 0);
    // hid-playstation's mic mute (power save bit 0x10) and mic volume 0 do
    // not reach the controller while the mic is open; closed, it is muted.
    std::array<std::uint8_t, 1 + VDS_SET_STATE_SIZE> host_mute{};
    host_mute[0] = VDS_USB_OUTPUT_REPORT_ID;
    host_mute[1 + 0] = 0x40;       // allow_mic_volume
    host_mute[1 + 1] = 0x02;       // allow_audio_mute
    host_mute[1 + 6] = 0x00;       // mic volume 0
    host_mute[1 + 9] = 0x10;       // mic mute
    assert(mic_state.apply_usb_output_report(host_mute));
    assert((mic_state.state()[9] & 0x10) == 0);
    mic_state.set_audio_out_stream_active(false);
    mic_overrides.mic_processing = vds::kMicProcessingNoiseCancel;
    mic_state.set_companion_overrides(mic_overrides);
    assert((mic_state.state()[7] & 0x0c) == 0x08);
    mic_overrides.mic_processing = 0;
    mic_state.set_companion_overrides(mic_overrides);
    mic_state.set_audio_out_stream_active(true);
    assert((mic_state.state()[1] & 0x02) != 0);
    assert(mic_state.state()[6] != 0);
    (void)mic_state.build_bt_mic_state_report(false, true);
    assert((mic_state.state()[9] & 0x10) != 0);
    assert((mic_state.state()[7] & 0x3f) == 0x30);
    assert((mic_state.state()[37] & 0x07) == 3);
  }

  // Mic level: 100% passes, lower levels scale on a cubic curve, mute is
  // silence; the peak is reported in permille.
  {
    std::vector<std::uint8_t> pcm = {0x00, 0x40, 0x00, 0xc0}; // 16384, -16384
    assert(vds::scale_mic_pcm(pcm, 100, false) == 500);
    assert(pcm[1] == 0x40 && pcm[3] == 0xc0);
    vds::scale_mic_pcm(pcm, 50, false); // x0.125 -> 2048, -2048
    assert(pcm[0] == 0x00 && pcm[1] == 0x08 && pcm[2] == 0x00 && pcm[3] == 0xf8);
    assert(vds::scale_mic_pcm(pcm, 100, true) == 0);
    assert(pcm[0] == 0 && pcm[1] == 0 && pcm[2] == 0 && pcm[3] == 0);
  }

  // Audio packets must never write power-save/mic-mute state.
  vds::DsState muted = state.state();
  muted[1] |= 0x02;
  muted[9] = 0x10 | 0x01;
  const auto mic = builder.build_packet(haptics, speaker, muted, false, false);
  assert((mic[13 + 1] & 0x02) == 0);
  assert(mic[13 + 9] == 0);

  // An audio packet must not count as delivering a mic mute/unmute change.
  vds::DsState unmuted = muted;
  unmuted[9] = 0x01;
  const auto delivered = vds::state_after_audio_packet(muted, unmuted);
  assert(delivered != unmuted);
  assert(delivered[9] == 0x11);
  assert(vds::state_after_audio_packet(unmuted, unmuted) == unmuted);

  vds::PcmAudioExtractor extractor{vds::kPcmWindowFrames};
  const std::vector<std::uint8_t> pcm(
      vds::kPcmWindowFrames * VDS_AUDIO_CHANNELS * sizeof(std::int16_t),
      0);
  assert(extractor.push_usb_audio(pcm).size() == 1);
}
