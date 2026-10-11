// SPDX-License-Identifier: MIT
#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>

#include "../src/vds_protocol.hh"

namespace {

constexpr std::size_t kRight = offsetof(vds_set_state_data, rumble_emulation_right);

// A game rumble output report as Linux hid-playstation sends it on current
// firmware: haptics select + vibration v2 (flag2 bit 2), both motors set.
std::array<std::uint8_t, 1 + VDS_SET_STATE_SIZE> rumble(std::uint8_t right,
                                                        std::uint8_t left,
                                                        bool v1_flags) {
  std::array<std::uint8_t, 1 + VDS_SET_STATE_SIZE> report{};
  report[0] = VDS_USB_OUTPUT_REPORT_ID;
  report[1 + 0] = 0x02 | (v1_flags ? 0x01 : 0x00);
  report[1 + 38] = v1_flags ? 0x00 : 0x04;
  report[1 + kRight] = right;
  report[1 + kRight + 1] = left;
  return report;
}

bool v1_mode(const vds::DsState &state) {
  return (state[0] & 0x01) != 0 && (state[38] & 0x0c) == 0;
}

bool v2_mode(const vds::DsState &state) {
  return (state[0] & 0x01) == 0 && (state[38] & 0x0c) == 0x0c;
}

} // namespace

int main() {
  vds::DsOutputState state;
  vds::DsCompanionOverrides overrides;

  // Default (v1 off): v2 emulation, whichever mode the game asked for.
  state.set_companion_overrides(overrides);
  assert(state.apply_usb_output_report(rumble(200, 100, false)));
  assert(v2_mode(state.state()));
  assert(state.state()[kRight] == 200 && state.state()[kRight + 1] == 100);
  assert(state.apply_usb_output_report(rumble(200, 100, true)));
  assert(v2_mode(state.state()));

  // v1 on: compatible vibration.
  overrides.classic_rumble_v1 = true;
  state.set_companion_overrides(overrides);
  assert(v1_mode(state.state()));
  assert(state.apply_usb_output_report(rumble(50, 255, false)));
  assert(v1_mode(state.state()));
  assert(state.state()[kRight] == 50 && state.state()[kRight + 1] == 255);

  // Stopping rumble (zero motors) is passed through as the game sent it, so
  // its handback to the actuator audio path is not rewritten.
  assert(state.apply_usb_output_report(rumble(0, 0, false)));
  assert((state.state()[0] & 0x01) == 0);
  assert((state.state()[38] & 0x0c) == 0x04);
  // The app's rumble test follows the same setting.
  vds::DsOutputState tester;
  vds::DsCompanionOverrides test_overrides;
  test_overrides.test_rumble_active = true;
  test_overrides.test_rumble_power = 180;
  tester.set_companion_overrides(test_overrides);
  assert(v2_mode(tester.state()));
  assert(tester.state()[kRight] == 180);
  test_overrides.classic_rumble_v1 = true;
  tester.set_companion_overrides(test_overrides);
  assert(v1_mode(tester.state()));
  return 0;
}
