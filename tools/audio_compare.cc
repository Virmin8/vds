// SPDX-License-Identifier: MIT
/*
 * vds_audio_compare: measures how faithfully the Bluetooth audio path carries
 * speaker and HD-haptics audio compared with a USB connection.
 *
 * Over USB the DualSense receives the 4-channel 48 kHz PCM untouched, so the
 * test signal itself is the reference. Over Bluetooth vdsd must convert it:
 * speaker -> 160 kbps Opus, haptics -> 3 kHz 8-bit. This tool runs a test
 * signal through the current conversion (PcmAudioExtractor) and through the
 * 1.9.0 conversion, decodes both the way the controller does, brings them back
 * to the 48 kHz timeline, aligns them and reports the accuracy per section.
 *
 *   vds_audio_compare [output-dir]   (writes WAVs when a directory is given)
 */
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <functional>
#include <random>
#include <string>
#include <vector>

#include <opus/opus.h>

#include "vds_protocol.hh"

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr int kRate = 48000;
constexpr std::size_t kWindow = vds::kSpeakerInputFrames;     // 512
constexpr std::size_t kOpusFrames = vds::kSpeakerOpusFrames;  // 480
constexpr std::size_t kHapticFrames = vds::kHapticsFrames;    // 32
constexpr std::size_t kSectionFrames = 3 * kRate;
constexpr std::size_t kSections = 4;
constexpr std::size_t kTotalFrames =
    (kSections * kSectionFrames + kWindow - 1) / kWindow * kWindow;

using Stereo = std::vector<float>; // interleaved

// ---------------------------------------------------------------- signals --

struct TestSignal {
  Stereo speaker;
  Stereo haptics;
};

float dbfs(double db) { return static_cast<float>(32767.0 * std::pow(10.0, db / 20.0)); }

TestSignal make_test_signal() {
  TestSignal signal{Stereo(kTotalFrames * 2, 0.0f), Stereo(kTotalFrames * 2, 0.0f)};
  std::mt19937 rng(1234);
  std::normal_distribution<float> noise(0.0f, 1.0f);
  float hp1 = 0, hp2 = 0, hp3 = 0, prev = 0;
  const auto bright_noise = [&]() {
    // White noise differentiated three times: energy rises 18 dB/oct, so it
    // is concentrated well above 2 kHz (cymbal-like).
    const float x = noise(rng);
    const float d1 = x - prev;
    prev = x;
    const float d2 = d1 - hp1;
    hp1 = d1;
    const float d3 = d2 - hp2;
    hp2 = d2;
    (void)hp3;
    return d3 * 0.12f;
  };

  for (std::size_t i = 0; i < kSections * kSectionFrames; ++i) {
    const std::size_t section = i / kSectionFrames;
    const double t = static_cast<double>(i % kSectionFrames) / kRate;
    const double length = static_cast<double>(kSectionFrames) / kRate;
    double speaker = 0.0;
    double haptic = 0.0;
    switch (section) {
    case 0: { // speaker: harmonic chord; haptics: 30 Hz - 1 kHz log sweep
      for (const double root : {220.0, 277.18, 329.63}) {
        for (int n = 1; n <= 10; ++n) {
          speaker += std::sin(2 * kPi * root * n * t) / n;
        }
      }
      speaker *= dbfs(-12) / 5.0;
      const double k = std::log(1000.0 / 30.0) / length;
      haptic = dbfs(-6) * std::sin(2 * kPi * 30.0 * (std::exp(k * t) - 1) / k);
      break;
    }
    case 1: { // speaker: cymbal-like bursts; haptics: 150 Hz clicks
      const double in_burst = std::fmod(t, 0.25);
      speaker = dbfs(-12) * bright_noise() * std::exp(-in_burst / 0.06);
      const double in_click = std::fmod(t, 0.2);
      haptic = in_click < 2.0 / 150.0
                   ? dbfs(-6) * std::sin(2 * kPi * 150.0 * in_click)
                   : 0.0;
      break;
    }
    case 2: { // speaker: 50 Hz - 16 kHz log sweep; haptics: 200 Hz fade-out
      const double k = std::log(16000.0 / 50.0) / length;
      speaker = dbfs(-12) * std::sin(2 * kPi * 50.0 * (std::exp(k * t) - 1) / k);
      const double level_db = -6.0 - 60.0 * t / length;
      haptic = dbfs(level_db) * std::sin(2 * kPi * 200.0 * t);
      break;
    }
    case 3: { // speaker: quiet chord (-45 dBFS); haptics: bright audio content
      for (const double root : {220.0, 277.18, 329.63}) {
        for (int n = 1; n <= 10; ++n) {
          speaker += std::sin(2 * kPi * root * n * t) / n;
        }
      }
      speaker *= dbfs(-45) / 5.0;
      haptic = dbfs(-6) * bright_noise();
      break;
    }
    }
    signal.speaker[i * 2] = signal.speaker[i * 2 + 1] = static_cast<float>(speaker);
    signal.haptics[i * 2] = signal.haptics[i * 2 + 1] = static_cast<float>(haptic);
  }
  return signal;
}

std::int16_t to_s16(float value) {
  return static_cast<std::int16_t>(std::clamp(std::lrint(value), -32768L, 32767L));
}

// ------------------------------------------------- 1.9.0 conversion (old) --

std::int16_t old_lerp(std::int16_t left, std::int16_t right, std::uint32_t num,
                      std::uint32_t den) {
  const auto mixed = (static_cast<std::int32_t>(left) * static_cast<std::int32_t>(den - num) +
                      static_cast<std::int32_t>(right) * static_cast<std::int32_t>(num)) /
                     static_cast<std::int32_t>(den);
  return static_cast<std::int16_t>(std::clamp(mixed, -32768, 32767));
}

struct OldPipeline {
  OpusEncoder *encoder = nullptr;
  OldPipeline() {
    int error = OPUS_OK;
    encoder = opus_encoder_create(kRate, 2, OPUS_APPLICATION_AUDIO, &error);
    opus_encoder_ctl(encoder, OPUS_SET_VBR(0));
    opus_encoder_ctl(encoder, OPUS_SET_COMPLEXITY(0));
    opus_encoder_ctl(encoder, OPUS_SET_EXPERT_FRAME_DURATION(OPUS_FRAMESIZE_10_MS));
    opus_encoder_ctl(encoder, OPUS_SET_BITRATE(vds::kSpeakerOpusSize * 8 * 100));
  }
  ~OldPipeline() { opus_encoder_destroy(encoder); }

  vds::AudioChunk process(const std::int16_t *speaker, const std::int16_t *haptics) {
    vds::AudioChunk chunk{};
    std::array<opus_int16, kOpusFrames * 2> resampled{};
    for (std::size_t frame = 0; frame < kOpusFrames; ++frame) {
      const std::size_t num = frame * kWindow;
      const std::size_t source = num / kOpusFrames;
      const auto fraction = static_cast<std::uint32_t>(num % kOpusFrames);
      const std::size_t next = std::min(source + 1, kWindow - 1);
      for (std::size_t ch = 0; ch < 2; ++ch) {
        resampled[frame * 2 + ch] = old_lerp(speaker[source * 2 + ch],
                                             speaker[next * 2 + ch], fraction,
                                             kOpusFrames);
      }
    }
    if (opus_encode(encoder, resampled.data(), kOpusFrames, chunk.speaker.data(),
                    static_cast<opus_int32>(chunk.speaker.size())) < 0) {
      std::fprintf(stderr, "opus_encode failed\n");
    }
    for (std::size_t frame = 0; frame < kHapticFrames; ++frame) {
      const std::size_t begin = frame * kWindow / kHapticFrames;
      const std::size_t end = (frame + 1) * kWindow / kHapticFrames;
      std::array<std::int32_t, 2> sum{};
      for (std::size_t s = begin; s < end; ++s) {
        sum[0] += haptics[s * 2];
        sum[1] += haptics[s * 2 + 1];
      }
      for (std::size_t ch = 0; ch < 2; ++ch) {
        const auto average = static_cast<std::int16_t>(
            sum[ch] / static_cast<std::int32_t>(end - begin));
        chunk.haptics[frame * 2 + ch] =
            static_cast<std::int8_t>(std::clamp(average / 256, -128, 127));
      }
    }
    return chunk;
  }
};

// ------------------------------------------------------ controller side --

struct Received {
  Stereo speaker; // back on the 48 kHz input timeline
  Stereo haptics;
};

// Decodes the chunks as the controller would (Opus at 48 kHz; haptics as
// 3 kHz 8-bit) and resamples both back to the input timeline with the same
// high-quality filter, so only the Bluetooth conversion differs.
struct Chunk : vds::AudioChunk {
  vds::SpeakerOpusInput raw_speaker{}; // pre-codec block (floor measurement)
};

Received receive(const std::vector<Chunk> &chunks, bool decode = true) {
  Received out{Stereo(), Stereo()};
  int error = OPUS_OK;
  OpusDecoder *decoder = opus_decoder_create(kRate, 2, &error);
  vds::BandlimitedResampler speaker_up(kOpusFrames, kWindow, 32, 0.9, 8.0);
  vds::BandlimitedResampler haptics_up(kHapticFrames, kWindow, 16, 0.9, 8.0);
  std::vector<opus_int16> pcm(kOpusFrames * 2);
  std::vector<float> up(kWindow * 2);
  std::vector<std::int16_t> haptic16(kHapticFrames * 2);
  for (const auto &chunk : chunks) {
    if (decode) {
      if (opus_decode(decoder, chunk.speaker.data(),
                      static_cast<opus_int32>(chunk.speaker.size()), pcm.data(),
                      kOpusFrames, 0) < 0) {
        std::fprintf(stderr, "opus_decode failed\n");
      }
    } else {
      std::copy(chunk.raw_speaker.begin(), chunk.raw_speaker.end(), pcm.begin());
    }
    speaker_up.process(pcm, up);
    out.speaker.insert(out.speaker.end(), up.begin(), up.end());
    for (std::size_t i = 0; i < haptic16.size(); ++i) {
      haptic16[i] = static_cast<std::int16_t>(chunk.haptics[i] * 256);
    }
    haptics_up.process(haptic16, up);
    out.haptics.insert(out.haptics.end(), up.begin(), up.end());
  }
  opus_decoder_destroy(decoder);
  return out;
}

// ------------------------------------------------------------ analysis --

// Linear-phase band-pass (windowed-sinc low-pass minus a 20 Hz high-pass by
// DC-blocking), applied to the left channel.
std::vector<double> band(const Stereo &stereo, double high_hz) {
  constexpr int kHalf = 256;
  std::vector<double> taps(2 * kHalf + 1);
  const double fc = high_hz / kRate;
  double sum = 0;
  for (int i = -kHalf; i <= kHalf; ++i) {
    const double x = 2 * fc * i;
    const double sinc = i == 0 ? 1.0 : std::sin(kPi * x) / (kPi * x);
    const double w = 0.42 + 0.5 * std::cos(kPi * i / kHalf) + 0.08 * std::cos(2 * kPi * i / kHalf);
    taps[i + kHalf] = 2 * fc * sinc * w;
    sum += taps[i + kHalf];
  }
  for (auto &t : taps) t /= sum;
  const std::size_t n = stereo.size() / 2;
  std::vector<double> out(n, 0.0);
  for (std::size_t i = 0; i < n; ++i) {
    double acc = 0;
    for (int k = -kHalf; k <= kHalf; ++k) {
      const auto j = static_cast<std::ptrdiff_t>(i) + k;
      if (j >= 0 && j < static_cast<std::ptrdiff_t>(n)) acc += taps[k + kHalf] * stereo[j * 2];
    }
    out[i] = acc;
  }
  // 20 Hz DC blocker (one-pole high-pass), run forwards and backwards.
  const double a = std::exp(-2 * kPi * 20.0 / kRate);
  for (int pass = 0; pass < 2; ++pass) {
    double x1 = 0, y1 = 0;
    for (std::size_t k = 0; k < n; ++k) {
      const std::size_t i = pass == 0 ? k : n - 1 - k;
      const double y = out[i] - x1 + a * y1;
      x1 = out[i];
      y1 = y;
      out[i] = y;
    }
  }
  return out;
}

// Shifts `x` by a fractional number of frames (windowed-sinc interpolation):
// out[i] = x(i + delay).
std::vector<double> shift(const std::vector<double> &x, double delay) {
  constexpr int kHalf = 24;
  const auto whole = static_cast<std::ptrdiff_t>(std::floor(delay));
  const double frac = delay - static_cast<double>(whole);
  std::array<double, 2 * kHalf> taps{};
  double sum = 0;
  for (int k = -kHalf + 1; k <= kHalf; ++k) {
    const double t = k - frac;
    const double sinc = t == 0 ? 1.0 : std::sin(kPi * t) / (kPi * t);
    const double w = 0.42 + 0.5 * std::cos(kPi * t / kHalf) + 0.08 * std::cos(2 * kPi * t / kHalf);
    taps[k + kHalf - 1] = sinc * w;
    sum += sinc * w;
  }
  std::vector<double> out(x.size(), 0.0);
  const auto n = static_cast<std::ptrdiff_t>(x.size());
  for (std::ptrdiff_t i = 0; i < n; ++i) {
    double acc = 0;
    for (int k = -kHalf + 1; k <= kHalf; ++k) {
      const std::ptrdiff_t j = i + whole + k;
      if (j >= 0 && j < n) acc += taps[k + kHalf - 1] * x[j];
    }
    out[i] = acc / sum;
  }
  return out;
}

// Signal-to-error ratio per section, in dB (`test` already aligned).
double ser_db(const std::vector<double> &ref, const std::vector<double> &test,
              std::size_t section, std::size_t stride = 1) {
  const std::size_t begin = section * kSectionFrames + 4096;
  const std::size_t end = (section + 1) * kSectionFrames - 4096;
  double signal = 0, error = 0;
  for (std::size_t i = begin; i < end; i += stride) {
    signal += ref[i] * ref[i];
    const double e = test[i] - ref[i];
    error += e * e;
  }
  return 10 * std::log10(signal / std::max(error, 1e-12));
}

// Finds the conversion delay to a fraction of a frame (best SER over
// `section`), returns the aligned signal and stores the delay in ms.
std::vector<double> align(const std::vector<double> &ref, const std::vector<double> &test,
                          std::size_t section, double &delay_ms) {
  const std::size_t begin = section * kSectionFrames + 4096;
  const std::size_t span = kSectionFrames - 8192;
  std::size_t coarse = 0;
  double best_score = -1e300;
  for (std::size_t d = 0; d < 2048; ++d) {
    double score = 0;
    for (std::size_t i = begin; i < begin + span; i += 4) score += ref[i] * test[i + d];
    if (score > best_score) {
      best_score = score;
      coarse = d;
    }
  }
  double best_delay = static_cast<double>(coarse);
  double best_ser = -1e300;
  for (double d = static_cast<double>(coarse) - 1.0; d <= static_cast<double>(coarse) + 1.0; d += 0.05) {
    const double ser = ser_db(ref, shift(test, d), section, 7);
    if (ser > best_ser) {
      best_ser = ser;
      best_delay = d;
    }
  }
  delay_ms = 1000.0 * best_delay / kRate;
  return shift(test, best_delay);
}

// Perceptual tonal-balance error: mean |dB difference| of 1/3-octave band
// energies (50 Hz - 16 kHz) over 1024-frame windows, ignoring bands more than
// 60 dB below the window's loudest band. Unlike SER it does not penalise
// Opus for rebuilding noise-like sound with the right spectrum.
double band_error_db(const std::vector<double> &ref, const std::vector<double> &test,
                     std::size_t section) {
  constexpr std::size_t kN = 1024;
  const auto spectrum = [](const double *x) {
    // Hann-windowed DFT power, computed by a radix-2 FFT.
    std::vector<double> re(kN), im(kN, 0.0);
    for (std::size_t i = 0; i < kN; ++i) {
      re[i] = x[i] * (0.5 - 0.5 * std::cos(2 * kPi * i / kN));
    }
    for (std::size_t i = 1, j = 0; i < kN; ++i) {
      std::size_t bit = kN >> 1;
      for (; j & bit; bit >>= 1) j ^= bit;
      j ^= bit;
      if (i < j) {
        std::swap(re[i], re[j]);
        std::swap(im[i], im[j]);
      }
    }
    for (std::size_t len = 2; len <= kN; len <<= 1) {
      const double ang = -2 * kPi / static_cast<double>(len);
      for (std::size_t i = 0; i < kN; i += len) {
        for (std::size_t k = 0; k < len / 2; ++k) {
          const double wr = std::cos(ang * k), wi = std::sin(ang * k);
          const double ur = re[i + k], ui = im[i + k];
          const double vr = re[i + k + len / 2] * wr - im[i + k + len / 2] * wi;
          const double vi = re[i + k + len / 2] * wi + im[i + k + len / 2] * wr;
          re[i + k] = ur + vr;
          im[i + k] = ui + vi;
          re[i + k + len / 2] = ur - vr;
          im[i + k + len / 2] = ui - vi;
        }
      }
    }
    std::vector<double> power(kN / 2);
    for (std::size_t i = 0; i < kN / 2; ++i) power[i] = re[i] * re[i] + im[i] * im[i];
    return power;
  };
  const auto bands = [&](const std::vector<double> &power) {
    std::vector<double> out;
    for (double center = 50.0; center <= 16000.0; center *= std::pow(2.0, 1.0 / 3.0)) {
      const double lo = center / std::pow(2.0, 1.0 / 6.0), hi = center * std::pow(2.0, 1.0 / 6.0);
      double sum = 1e-9;
      for (std::size_t i = 0; i < power.size(); ++i) {
        const double hz = static_cast<double>(i) * kRate / kN;
        if (hz >= lo && hz < hi) sum += power[i];
      }
      out.push_back(10 * std::log10(sum));
    }
    return out;
  };
  const std::size_t begin = section * kSectionFrames + 4096;
  const std::size_t end = (section + 1) * kSectionFrames - 4096;
  double total = 0;
  std::size_t count = 0;
  for (std::size_t i = begin; i + kN < end; i += kN / 2) {
    const auto r = bands(spectrum(ref.data() + i));
    const auto t = bands(spectrum(test.data() + i));
    const double loudest = *std::max_element(r.begin(), r.end());
    for (std::size_t b = 0; b < r.size(); ++b) {
      if (r[b] < loudest - 60.0) continue;
      total += std::abs(t[b] - r[b]);
      ++count;
    }
  }
  return count ? total / static_cast<double>(count) : 0.0;
}

// RMS level of a section in dBFS.
double level_db(const std::vector<double> &x, std::size_t section) {
  const std::size_t begin = section * kSectionFrames + 4096;
  const std::size_t end = (section + 1) * kSectionFrames - 4096;
  double sum = 0;
  for (std::size_t i = begin; i < end; ++i) sum += x[i] * x[i];
  return 20 * std::log10(std::max(std::sqrt(sum / static_cast<double>(end - begin)), 1e-9) / 32767.0);
}

// ---------------------------------------------------------------- output --

void write_wav(const std::string &path, const std::vector<double> &mono) {
  std::ofstream file(path, std::ios::binary);
  const std::size_t frames = mono.size();
  const auto u32 = [&](std::uint32_t v) { file.write(reinterpret_cast<const char *>(&v), 4); };
  const auto u16 = [&](std::uint16_t v) { file.write(reinterpret_cast<const char *>(&v), 2); };
  file.write("RIFF", 4);
  u32(static_cast<std::uint32_t>(36 + frames * 2));
  file.write("WAVEfmt ", 8);
  u32(16); u16(1); u16(1); u32(kRate); u32(kRate * 2); u16(2); u16(16);
  file.write("data", 4);
  u32(static_cast<std::uint32_t>(frames * 2));
  for (const double v : mono) u16(static_cast<std::uint16_t>(to_s16(static_cast<float>(v))));
}

} // namespace

int main(int argc, char **argv) {
  const TestSignal signal = make_test_signal();

  // Run both Bluetooth conversions window by window, as vdsd does.
  vds::PcmAudioExtractor extractor(kWindow); // boost off: fidelity only
  vds::PcmAudioExtractor mono_extractor(kWindow);
  mono_extractor.set_speaker_mono(true); // built-in speaker
  vds::SpeakerResampler floor_resampler(kWindow);
  OldPipeline old_pipeline;
  std::vector<Chunk> new_chunks, old_chunks, floor_chunks, mono_chunks;
  std::vector<std::uint8_t> pcm(kWindow * VDS_AUDIO_CHANNELS * 2);
  std::vector<std::int16_t> speaker16(kWindow * 2), haptics16(kWindow * 2);
  for (std::size_t start = 0; start < kTotalFrames; start += kWindow) {
    for (std::size_t f = 0; f < kWindow; ++f) {
      const std::size_t i = start + f;
      const std::array<std::int16_t, 4> frame = {
          to_s16(signal.speaker[i * 2]), to_s16(signal.speaker[i * 2 + 1]),
          to_s16(signal.haptics[i * 2]), to_s16(signal.haptics[i * 2 + 1])};
      for (std::size_t ch = 0; ch < 4; ++ch) {
        const auto v = static_cast<std::uint16_t>(frame[ch]);
        pcm[(f * 4 + ch) * 2] = static_cast<std::uint8_t>(v & 0xff);
        pcm[(f * 4 + ch) * 2 + 1] = static_cast<std::uint8_t>(v >> 8);
      }
      speaker16[f * 2] = frame[0];
      speaker16[f * 2 + 1] = frame[1];
      haptics16[f * 2] = frame[2];
      haptics16[f * 2 + 1] = frame[3];
    }
    for (const auto &chunk : extractor.push_usb_audio(pcm)) {
      Chunk c;
      static_cast<vds::AudioChunk &>(c) = chunk;
      new_chunks.push_back(c);
    }
    for (const auto &chunk : mono_extractor.push_usb_audio(pcm)) {
      Chunk c;
      static_cast<vds::AudioChunk &>(c) = chunk;
      mono_chunks.push_back(c);
    }
    Chunk old_chunk;
    static_cast<vds::AudioChunk &>(old_chunk) =
        old_pipeline.process(speaker16.data(), haptics16.data());
    old_chunks.push_back(old_chunk);
    Chunk floor_chunk;
    floor_chunk.raw_speaker = floor_resampler.process(speaker16);
    floor_chunks.push_back(floor_chunk);
  }
  const Received bt_new = receive(new_chunks);
  const Received bt_old = receive(old_chunks);
  const Received bt_floor = receive(floor_chunks, false);
  const Received bt_mono = receive(mono_chunks);

  const auto ref_speaker = band(signal.speaker, 18000.0);
  const auto ref_haptics = band(signal.haptics, 1000.0);
  double ms_new_s = 0, ms_old_s = 0, ms_floor_s = 0, ms_mono_s = 0, ms_new_h = 0, ms_old_h = 0;
  // Align on the broadband sweeps.
  const auto new_speaker = align(ref_speaker, band(bt_new.speaker, 18000.0), 2, ms_new_s);
  const auto old_speaker = align(ref_speaker, band(bt_old.speaker, 18000.0), 2, ms_old_s);
  const auto floor_speaker = align(ref_speaker, band(bt_floor.speaker, 18000.0), 2, ms_floor_s);
  const auto mono_speaker = align(ref_speaker, band(bt_mono.speaker, 18000.0), 2, ms_mono_s);
  const auto new_haptics = align(ref_haptics, band(bt_new.haptics, 1000.0), 0, ms_new_h);
  const auto old_haptics = align(ref_haptics, band(bt_old.haptics, 1000.0), 0, ms_old_h);

  std::printf("Bluetooth vs USB (USB delivers the test signal exactly)\n\n");
  std::printf("SPEAKER, 20 Hz - 18 kHz\n");
  std::printf("  waveform accuracy (SER, higher is better)   floor*   BT 1.9.0   BT now  BT mono\n");
  const char *speaker_names[] = {"music-like chord (-12 dBFS)", "cymbal bursts", "sweep 50 Hz - 16 kHz", "quiet chord (-45 dBFS)"};
  for (std::size_t s = 0; s < kSections; ++s) {
    std::printf("    %-40s %5.1f dB  %6.1f dB  %6.1f dB  %5.1f dB\n", speaker_names[s],
                ser_db(ref_speaker, floor_speaker, s), ser_db(ref_speaker, old_speaker, s),
                ser_db(ref_speaker, new_speaker, s), ser_db(ref_speaker, mono_speaker, s));
  }
  std::printf("  tonal balance error (1/3-octave, lower is better)  BT 1.9.0   BT now  BT mono\n");
  for (std::size_t s = 0; s < kSections; ++s) {
    std::printf("    %-40s          %5.2f dB  %6.2f dB  %5.2f dB\n", speaker_names[s],
                band_error_db(ref_speaker, old_speaker, s), band_error_db(ref_speaker, new_speaker, s),
                band_error_db(ref_speaker, mono_speaker, s));
  }
  std::printf("  * floor: resampling only, no Opus (the limit of this measurement)\n");

  std::printf("\nHD HAPTICS, 20 Hz - 1 kHz (actuator band)\n");
  std::printf("  waveform accuracy (SER, higher is better)             BT 1.9.0   BT now\n");
  const char *haptic_names[] = {"sweep 30 Hz - 1 kHz", "150 Hz clicks", "200 Hz fade -6 to -66 dB"};
  for (std::size_t s = 0; s < 3; ++s) {
    std::printf("    %-40s          %6.1f dB  %6.1f dB\n", haptic_names[s],
                ser_db(ref_haptics, old_haptics, s), ser_db(ref_haptics, new_haptics, s));
  }
  std::printf("  spurious vibration from bright audio (lower is better)\n");
  std::printf("    USB %.1f dBFS   BT 1.9.0 %.1f dBFS   BT now %.1f dBFS\n",
              level_db(ref_haptics, 3), level_db(old_haptics, 3), level_db(new_haptics, 3));
  std::printf("\nExtra latency vs 1.9.0: speaker %+.2f ms, haptics %+.2f ms\n",
              ms_new_s - ms_old_s, ms_new_h - ms_old_h);

  if (argc > 1) {
    const std::string dir = argv[1];
    write_wav(dir + "/speaker-usb.wav", ref_speaker);
    write_wav(dir + "/speaker-bt-1.9.0.wav", old_speaker);
    write_wav(dir + "/speaker-bt-now.wav", new_speaker);
    write_wav(dir + "/haptics-usb.wav", ref_haptics);
    write_wav(dir + "/haptics-bt-1.9.0.wav", old_haptics);
    write_wav(dir + "/haptics-bt-now.wav", new_haptics);
    std::printf("\nWAVs (aligned, band-limited) written to %s\n", dir.c_str());
  }
  return 0;
}
