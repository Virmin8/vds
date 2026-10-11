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
  assert(state.state()[7] == 0x30);

  std::array<std::int8_t, vds::kHapticsSampleSize> haptics{};
  std::array<std::uint8_t, vds::kSpeakerOpusSize> speaker{};
  vds::HapticsPacketBuilder builder;
  const auto packet = builder.build_packet(haptics, speaker, state.state());

  assert(packet[4] == 0xfe);
  // Audio buffer length: default 64, then whatever the companion app sets.
  for (std::size_t i = 5; i <= 9; ++i) {
    assert(packet[i] == 64);
  }
  vds::HapticsPacketBuilder short_buffer;
  short_buffer.set_buffer_length(24);
  const auto short_packet =
      short_buffer.build_packet(haptics, speaker, state.state());
  for (std::size_t i = 5; i <= 9; ++i) {
    assert(short_packet[i] == 24);
  }
  assert(packet[13 + 7] == 0x30);
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
  const auto scaled = builder.build_packet(haptics, speaker, state.state());
  assert(scaled[13 + 5] == 0x64);

  // The mute light is always forced off (no microphone over vds), even when
  // the host (hid-playstation) asks for it.
  std::array<std::uint8_t, 1 + VDS_SET_STATE_SIZE> mute_light{};
  mute_light[0] = VDS_USB_OUTPUT_REPORT_ID;
  mute_light[1 + 1] = 0x01; // allow_mute_light
  mute_light[1 + 8] = 0x01; // mute light on
  assert(state.apply_usb_output_report(mute_light));
  assert((state.state()[1] & 0x01) != 0);
  assert(state.state()[8] == 0);

  // Audio packets must never write power-save/mic-mute state.
  vds::DsState muted = state.state();
  muted[1] |= 0x02;
  muted[9] = 0x10 | 0x01;
  const auto mic = builder.build_packet(haptics, speaker, muted);
  assert((mic[13 + 1] & 0x02) == 0);
  assert(mic[13 + 9] == 0);

  // An audio packet must not count as delivering a mic mute/unmute change.
  vds::DsState unmuted = muted;
  unmuted[9] = 0x01;
  const auto delivered = vds::state_after_audio_packet(muted, unmuted);
  assert(delivered != unmuted);
  assert(delivered[9] == 0x11);
  assert(vds::state_after_audio_packet(unmuted, unmuted) == unmuted);

  vds::PcmAudioExtractor extractor{vds::kSpeakerInputFrames};
  const std::vector<std::uint8_t> pcm(
      vds::kSpeakerInputFrames * VDS_AUDIO_CHANNELS * sizeof(std::int16_t),
      0);
  assert(extractor.push_usb_audio(pcm).size() == 1);
}
