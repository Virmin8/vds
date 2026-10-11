// SPDX-License-Identifier: MIT
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

#include "../src/vds_protocol.hh"

namespace {

// One 512-frame window of 4-channel PCM: a different tone on each speaker
// channel (so the encoder would choose stereo if allowed), silent haptics.
std::vector<std::uint8_t> window(std::size_t index) {
  std::vector<std::uint8_t> pcm(vds::kSpeakerInputFrames * VDS_AUDIO_CHANNELS * 2);
  for (std::size_t f = 0; f < vds::kSpeakerInputFrames; ++f) {
    const double t = static_cast<double>(index * vds::kSpeakerInputFrames + f) / 48000.0;
    const std::int16_t left = static_cast<std::int16_t>(8000 * std::sin(2 * 3.14159265 * 440 * t));
    const std::int16_t right = static_cast<std::int16_t>(8000 * std::sin(2 * 3.14159265 * 660 * t));
    for (const auto [channel, value] : {std::pair{0, left}, std::pair{1, right}}) {
      const auto bits = static_cast<std::uint16_t>(value);
      pcm[(f * VDS_AUDIO_CHANNELS + channel) * 2] = static_cast<std::uint8_t>(bits & 0xff);
      pcm[(f * VDS_AUDIO_CHANNELS + channel) * 2 + 1] = static_cast<std::uint8_t>(bits >> 8);
    }
  }
  return pcm;
}

// Opus TOC byte bit 2: stereo packet.
bool stereo_packet(const vds::AudioChunk &chunk) { return (chunk.speaker[0] & 0x04) != 0; }

} // namespace

int main() {
  vds::PcmAudioExtractor extractor;
  std::size_t index = 0;

  // Default: stereo (headphones).
  for (int i = 0; i < 5; ++i) {
    const auto chunks = extractor.push_usb_audio(window(index++));
    assert(chunks.size() == 1);
    assert(stereo_packet(chunks[0]));
  }

  // Built-in speaker: mono packets, switched without restarting the stream.
  extractor.set_speaker_mono(true);
  for (int i = 0; i < 5; ++i) {
    const auto chunks = extractor.push_usb_audio(window(index++));
    assert(!stereo_packet(chunks[0]));
    assert(chunks[0].speaker.size() == vds::kSpeakerOpusSize);
  }

  // Headphones plugged back in: stereo again.
  extractor.set_speaker_mono(false);
  for (int i = 0; i < 5; ++i) {
    const auto chunks = extractor.push_usb_audio(window(index++));
    assert(stereo_packet(chunks[0]));
  }
  return 0;
}
