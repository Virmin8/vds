// SPDX-License-Identifier: MIT
#include <array>
#include <cassert>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <thread>

#include "../src/vds_companion.hh"

namespace {

using namespace std::chrono_literals;

// USB input report: byte 8 low nibble is the d-pad hat (8 = neutral),
// byte 10 bit 0 is PS, bit 1 the touchpad click (shortcut modifier) and
// bit 2 Mute.
constexpr std::uint8_t kHatUp = 0;
constexpr std::uint8_t kHatRight = 2;
constexpr std::uint8_t kHatDown = 4;
constexpr std::uint8_t kHatLeft = 6;
constexpr std::uint8_t kHatNeutral = 8;
constexpr std::uint8_t kPs = 0x01;
constexpr std::uint8_t kTouchpad = 0x02;
constexpr std::uint8_t kMute = 0x04;

struct Result {
  std::uint8_t buttons; // byte 10
  std::uint8_t hat;
};

Result feed(vds::CompanionRuntime &runtime, vds::CompanionInputState &state,
            std::uint8_t buttons, std::uint8_t hat) {
  std::array<std::uint8_t, 64> r{};
  r[0] = 0x01;
  r[8] = hat;
  r[10] = buttons;
  vds::companion_translate_input(runtime, state, r);
  return {r[10], static_cast<std::uint8_t>(r[8] & 0x0f)};
}

} // namespace

int main() {
  // Disabled: Touchpad + d-pad is passed through untouched.
  {
    vds::CompanionRuntime runtime;
    vds::CompanionInputState state;
    const auto out = feed(runtime, state, kTouchpad, kHatUp);
    assert(out.buttons == kTouchpad && out.hat == kHatUp);
    assert(runtime.pending_input_events.empty());
  }

  // Touchpad + D-pad up: volume up immediately, repeating every 180 ms while
  // held; the host never sees the d-pad or the touchpad click.
  {
    vds::CompanionRuntime runtime;
    runtime.settings.speaker_volume_shortcut_enabled = true;
    vds::CompanionInputState state;
    auto out = feed(runtime, state, kTouchpad, kHatNeutral); // touchpad alone: passes
    assert(out.buttons == kTouchpad);
    const auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < 400ms) {
      out = feed(runtime, state, kTouchpad, kHatUp);
      assert(out.buttons == 0 && out.hat == kHatNeutral);
      std::this_thread::sleep_for(4ms); // ~250 Hz input
    }
    assert(runtime.pending_input_events.size() == 3); // 0, 180, 360 ms
    for (const auto event : runtime.pending_input_events) {
      assert(event == 0x02);
    }
    // Releasing the d-pad first keeps the touchpad hidden until it is released too.
    out = feed(runtime, state, kTouchpad, kHatNeutral);
    assert(out.buttons == 0);
    out = feed(runtime, state, 0, kHatNeutral);
    assert(out.buttons == 0);
    out = feed(runtime, state, kTouchpad, kHatNeutral); // fresh touchpad press passes
    assert(out.buttons == kTouchpad);
    runtime.pending_input_events.clear();
    out = feed(runtime, state, kTouchpad, kHatDown);
    assert(out.buttons == 0 && runtime.pending_input_events.size() == 1 &&
           runtime.pending_input_events.front() == 0x01);
  }

  // Profile shortcut: Touchpad + left/right sends previous/next once per press.
  {
    vds::CompanionRuntime runtime;
    runtime.settings.profile_shortcut_enabled = true;
    vds::CompanionInputState state;
    const auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < 400ms) {
      const auto out = feed(runtime, state, kTouchpad, kHatRight);
      assert(out.buttons == 0 && out.hat == kHatNeutral);
      std::this_thread::sleep_for(4ms);
    }
    assert(runtime.pending_input_events.size() == 1 &&
           runtime.pending_input_events.front() == 0x07);
    feed(runtime, state, kTouchpad, kHatNeutral);
    feed(runtime, state, kTouchpad, kHatLeft);
    assert(runtime.pending_input_events.size() == 2 &&
           runtime.pending_input_events.back() == 0x06);
  }

  // Turn off: Touchpad + Triangle sends the SLEEP_CONTROLLER event once.
  {
    vds::CompanionRuntime runtime;
    runtime.settings.sleep_keybind_enabled = true;
    vds::CompanionInputState state;
    std::array<std::uint8_t, 64> r{};
    r[0] = 0x01;
    r[8] = kHatNeutral | 0x80; // triangle
    r[10] = kTouchpad;
    for (int i = 0; i < 20; ++i) {
      auto copy = r;
      vds::companion_translate_input(runtime, state, copy);
      assert((copy[8] & 0x80) == 0 && (copy[10] & kTouchpad) == 0);
    }
    assert(runtime.pending_input_events.size() == 1 &&
           runtime.pending_input_events.front() == 0x03);
  }

  // PS is a normal button with the shortcuts on: no hold-back, no events.
  {
    vds::CompanionRuntime runtime;
    runtime.settings.speaker_volume_shortcut_enabled = true;
    runtime.settings.profile_shortcut_enabled = true;
    vds::CompanionInputState state;
    auto out = feed(runtime, state, kPs, kHatUp);
    assert(out.buttons == kPs && out.hat == kHatUp);
    out = feed(runtime, state, kPs, kHatRight);
    assert(out.buttons == kPs && out.hat == kHatRight);
    assert(runtime.pending_input_events.empty());
  }

  // A PS-starter chord still holds a lone PS tap back and replays it.
  {
    vds::CompanionRuntime runtime;
    runtime.chord_bindings.push_back({.event = 0x20, .starter = 1, .button = 13});
    vds::CompanionInputState state;
    assert((feed(runtime, state, kPs, kHatNeutral).buttons & kPs) == 0);
    std::this_thread::sleep_for(50ms);
    assert((feed(runtime, state, 0, kHatNeutral).buttons & kPs) != 0);
    std::this_thread::sleep_for(100ms);
    assert((feed(runtime, state, 0, kHatNeutral).buttons & kPs) == 0);
  }

  // Idle Disconnect activity rules.
  {
    std::array<std::uint8_t, 64> idle{};
    idle[0] = 0x01;
    idle[1] = idle[2] = idle[3] = idle[4] = 128; // sticks centred
    idle[8] = kHatNeutral;
    assert(!vds::controller_input_active(idle));
    auto drift = idle;
    drift[1] = 136; // stick noise inside the centre band
    assert(!vds::controller_input_active(drift));
    auto stick = idle;
    stick[3] = 200;
    assert(vds::controller_input_active(stick));
    auto trigger = idle;
    trigger[6] = 1;
    assert(vds::controller_input_active(trigger));
    auto dpad = idle;
    dpad[8] = kHatUp;
    assert(vds::controller_input_active(dpad));
    auto face = idle;
    face[8] = kHatNeutral | 0x20; // cross
    assert(vds::controller_input_active(face));
    auto ps = idle;
    ps[10] = kPs;
    assert(vds::controller_input_active(ps));
    auto motion = idle;
    for (std::size_t i = 16; i < 28; ++i) motion[i] = 0x55; // gyro/accel
    assert(!vds::controller_input_active(motion));
  }

  // D-pad without the touchpad click is untouched.
  {
    vds::CompanionRuntime runtime;
    runtime.settings.speaker_volume_shortcut_enabled = true;
    vds::CompanionInputState state;
    const auto out = feed(runtime, state, 0, kHatUp);
    assert(out.buttons == 0 && out.hat == kHatUp);
    assert(runtime.pending_input_events.empty());
  }

  // Mute is no longer the modifier: Mute + d-pad reaches the host untouched
  // (vdsd uses Mute for mic mute).
  {
    vds::CompanionRuntime runtime;
    runtime.settings.speaker_volume_shortcut_enabled = true;
    runtime.settings.profile_shortcut_enabled = true;
    vds::CompanionInputState state;
    const auto out = feed(runtime, state, kMute, kHatUp);
    assert(out.buttons == kMute && out.hat == kHatUp);
    assert(runtime.pending_input_events.empty());
  }
  return 0;
}
