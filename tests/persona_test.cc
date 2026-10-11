// SPDX-License-Identifier: MIT
#include <cassert>
#include <cstdint>
#include <vector>

#include "../src/vds_persona.hh"

int main() {
  using vds::HostPersona;

  // Neutral DualSense USB report with cross + L1 + PS held.
  std::vector<std::uint8_t> ds5(64, 0);
  ds5[0] = 0x01;
  ds5[1] = 0x80; // left X centre
  ds5[2] = 0x00; // left Y fully up
  ds5[3] = 0x80;
  ds5[4] = 0xff; // right Y fully down
  ds5[5] = 0x10; // L2
  ds5[6] = 0x20; // R2
  ds5[8] = 0x20 | 0x00; // cross, hat up
  ds5[9] = 0x01;        // L1
  ds5[10] = 0x01;       // PS
  ds5[16] = 0x34;       // gyro low byte
  ds5[33] = 0x80;       // touch point 0 inactive
  ds5[53] = 0x15;       // charging, 50%

  vds::PersonaState state;

  const auto ds4 = vds::persona_input_report(HostPersona::Ds4, ds5, state);
  assert(ds4.size() == 64);
  assert(ds4[0] == 0x01);
  assert(ds4[2] == 0x00);
  assert(ds4[5] == 0x20); // cross + hat up
  assert(ds4[6] == 0x01); // L1
  assert((ds4[7] & 0x01) == 0x01); // PS
  assert(ds4[8] == 0x10 && ds4[9] == 0x20);
  assert(ds4[13] == 0x34);
  assert(ds4[30] == (0x10 | 5)); // cable + capacity
  assert(ds4[35] == 0x80);

  const auto x360 = vds::persona_input_report(HostPersona::Xbox360, ds5, state);
  assert(x360.size() == 20);
  assert(x360[0] == 0x00 && x360[1] == 0x14);
  const std::uint16_t buttons = x360[2] | (x360[3] << 8);
  assert(buttons & 0x0001); // dpad up
  assert(buttons & 0x1000); // A
  assert(buttons & 0x0100); // LB
  assert(buttons & 0x0400); // guide
  const auto left_y = static_cast<std::int16_t>(x360[8] | (x360[9] << 8));
  assert(left_y == 32767); // up is positive
  const auto right_y = static_cast<std::int16_t>(x360[12] | (x360[13] << 8));
  assert(right_y == -32768);

  assert(vds::persona_input_report(HostPersona::DualSense, ds5, state).empty());

  // DS4 output: rumble + lightbar.
  std::vector<std::uint8_t> ds4_out(32, 0);
  ds4_out[0] = 0x05;
  ds4_out[1] = 0x03;
  ds4_out[4] = 0x40; // weak / right
  ds4_out[5] = 0x80; // strong / left
  ds4_out[6] = 0x11;
  ds4_out[7] = 0x22;
  ds4_out[8] = 0x33;
  const auto ds5_out = vds::persona_output_to_ds5(HostPersona::Ds4, ds4_out, state);
  assert(ds5_out);
  assert((*ds5_out)[0] == 0x02);
  assert(((*ds5_out)[1] & 0x03) == 0x03);
  assert((*ds5_out)[3] == 0x40 && (*ds5_out)[4] == 0x80);
  assert(((*ds5_out)[2] & 0x04) == 0x04);
  assert((*ds5_out)[45] == 0x11 && (*ds5_out)[47] == 0x33);

  // Lightbar-only update keeps the motors running.
  ds4_out[1] = 0x02;
  const auto keep = vds::persona_output_to_ds5(HostPersona::Ds4, ds4_out, state);
  assert(keep && (*keep)[3] == 0x40 && (*keep)[4] == 0x80);

  // Xbox 360 rumble and LED.
  const std::vector<std::uint8_t> rumble = {0x00, 0x08, 0x00, 0xff, 0x10,
                                            0x00, 0x00, 0x00};
  const auto x_out = vds::persona_output_to_ds5(HostPersona::Xbox360, rumble, state);
  assert(x_out && (*x_out)[4] == 0xff && (*x_out)[3] == 0x10);
  const std::vector<std::uint8_t> led = {0x01, 0x03, 0x06};
  const auto led_out = vds::persona_output_to_ds5(HostPersona::Xbox360, led, state);
  assert(led_out && (*led_out)[44] == 0x04);

  // Calibration reorder from DualSense 0x05.
  std::vector<std::uint8_t> ds5_cal(41, 0);
  ds5_cal[0] = 0x05;
  const std::int16_t values[17] = {1, 2, 3, 10, -10, 20, -20, 30, -30,
                                   540, 540, 8000, -8000, 8100, -8100, 8200, -8200};
  for (int i = 0; i < 17; ++i) {
    ds5_cal[1 + i * 2] = static_cast<std::uint8_t>(values[i] & 0xff);
    ds5_cal[2 + i * 2] = static_cast<std::uint8_t>((values[i] >> 8) & 0xff);
  }
  const auto cal = vds::ds4_feature_report(0x02, 37, ds5_cal, {});
  assert(cal.size() == 37 && cal[0] == 0x02);
  auto field = [&](int i) {
    return static_cast<std::int16_t>(cal[1 + i * 2] | (cal[2 + i * 2] << 8));
  };
  assert(field(3) == 10 && field(4) == 20 && field(5) == 30);
  assert(field(6) == -10 && field(7) == -20 && field(8) == -30);
  assert(field(9) == 540 && field(16) == -8200);

  std::vector<std::uint8_t> pairing(20, 0);
  pairing[0] = 0x09;
  for (int i = 0; i < 6; ++i) pairing[1 + i] = static_cast<std::uint8_t>(0xa0 + i);
  const auto mac = vds::ds4_feature_report(0x12, 16, {}, pairing);
  assert(mac.size() == 16 && mac[1] == 0xa0 && mac[6] == 0xa5);

  assert(vds::ds4_feature_report(0xa3, 49, {}, {}).size() == 49);
  assert(vds::usb_profile_for_persona(HostPersona::Ds4, VDS_PROFILE_DS5) == VDS_PROFILE_DS4);
  assert(vds::usb_profile_for_persona(HostPersona::DualSense, VDS_PROFILE_DSE) == VDS_PROFILE_DSE);
}
