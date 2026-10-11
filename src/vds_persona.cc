// SPDX-License-Identifier: MIT
#include "vds_persona.hh"

#include <algorithm>
#include <cstddef>
#include <cstring>

namespace vds {
namespace {

// DualSense USB input report offsets (report ID at 0).
constexpr std::size_t kDs5LeftX = 1;
constexpr std::size_t kDs5LeftY = 2;
constexpr std::size_t kDs5RightX = 3;
constexpr std::size_t kDs5RightY = 4;
constexpr std::size_t kDs5L2 = 5;
constexpr std::size_t kDs5R2 = 6;
constexpr std::size_t kDs5Buttons0 = 8; // hat | square/cross/circle/triangle
constexpr std::size_t kDs5Buttons1 = 9; // L1 R1 L2 R2 create options L3 R3
constexpr std::size_t kDs5Buttons2 = 10; // PS touchpad mute
constexpr std::size_t kDs5Gyro = 16;
constexpr std::size_t kDs5Timestamp = 28;
constexpr std::size_t kDs5Touch = 33;
constexpr std::size_t kDs5Status = 53;
constexpr std::size_t kDs5MinimumReport = 54;

constexpr std::uint8_t kDs5ButtonPs = 0x01;
constexpr std::uint8_t kDs5ButtonTouchpad = 0x02;

// DualShock 4 USB input report offsets (report ID at 0).
constexpr std::size_t kDs4Buttons0 = 5;
constexpr std::size_t kDs4Buttons1 = 6;
constexpr std::size_t kDs4Buttons2 = 7;
constexpr std::size_t kDs4L2 = 8;
constexpr std::size_t kDs4R2 = 9;
constexpr std::size_t kDs4Timestamp = 10;
constexpr std::size_t kDs4Gyro = 13;
constexpr std::size_t kDs4Status = 30;
constexpr std::size_t kDs4TouchReports = 33;
constexpr std::size_t kDs4TouchTimestamp = 34;
constexpr std::size_t kDs4Touch = 35;
constexpr std::uint8_t kDs4StatusCable = 0x10;
constexpr unsigned kDs5TouchHeight = 1080;
constexpr unsigned kDs4TouchHeight = 942;

// DualSense SetStateData offsets.
constexpr std::size_t kSetFlag0 = 0;
constexpr std::size_t kSetFlag1 = 1;
constexpr std::size_t kSetRumbleRight = 2;
constexpr std::size_t kSetRumbleLeft = 3;
constexpr std::size_t kSetFlag2 = 38;
constexpr std::size_t kSetPlayerLights = 43;
constexpr std::size_t kSetLedRed = 44;
constexpr std::uint8_t kSetFlag0Rumble = 0x01 | 0x02;
constexpr std::uint8_t kSetFlag1LedColor = 0x04;
constexpr std::uint8_t kSetFlag1PlayerLights = 0x10;
constexpr std::uint8_t kSetFlag2ImprovedRumble = 0x04;

// Xbox 360 button bits (little-endian 16-bit field at offset 2).
constexpr std::uint16_t kX360DpadUp = 0x0001;
constexpr std::uint16_t kX360DpadDown = 0x0002;
constexpr std::uint16_t kX360DpadLeft = 0x0004;
constexpr std::uint16_t kX360DpadRight = 0x0008;
constexpr std::uint16_t kX360Start = 0x0010;
constexpr std::uint16_t kX360Back = 0x0020;
constexpr std::uint16_t kX360LeftStick = 0x0040;
constexpr std::uint16_t kX360RightStick = 0x0080;
constexpr std::uint16_t kX360LeftShoulder = 0x0100;
constexpr std::uint16_t kX360RightShoulder = 0x0200;
constexpr std::uint16_t kX360Guide = 0x0400;
constexpr std::uint16_t kX360A = 0x1000;
constexpr std::uint16_t kX360B = 0x2000;
constexpr std::uint16_t kX360X = 0x4000;
constexpr std::uint16_t kX360Y = 0x8000;

// Hat value -> up/right/down/left bits, 8 = neutral.
constexpr std::uint8_t kHatDirections[9] = {0x1, 0x3, 0x2, 0x6, 0x4,
                                            0xc, 0x8, 0x9, 0x0};

void write_le16(std::uint8_t *data, std::uint16_t value) {
  data[0] = static_cast<std::uint8_t>(value & 0xff);
  data[1] = static_cast<std::uint8_t>(value >> 8);
}

std::int16_t read_le16s(const std::uint8_t *data) {
  return static_cast<std::int16_t>(data[0] | (data[1] << 8));
}

std::uint32_t read_le32(const std::uint8_t *data) {
  return static_cast<std::uint32_t>(data[0]) |
         (static_cast<std::uint32_t>(data[1]) << 8) |
         (static_cast<std::uint32_t>(data[2]) << 16) |
         (static_cast<std::uint32_t>(data[3]) << 24);
}

void write_ds4_touch_point(std::uint8_t *out, const std::uint8_t *in) {
  // Same packing as the DualSense (bit 7 of byte 0 = not touching, 12-bit X
  // and Y); only the touchpad height differs.
  const unsigned x = in[1] | ((in[2] & 0x0f) << 8);
  const unsigned y = (in[2] >> 4) | (in[3] << 4);
  const unsigned scaled_y =
      std::min(kDs4TouchHeight - 1,
               (y * kDs4TouchHeight + kDs5TouchHeight / 2) / kDs5TouchHeight);
  out[0] = in[0];
  out[1] = static_cast<std::uint8_t>(x & 0xff);
  out[2] = static_cast<std::uint8_t>(((x >> 8) & 0x0f) | ((scaled_y & 0x0f) << 4));
  out[3] = static_cast<std::uint8_t>(scaled_y >> 4);
}

std::uint8_t ds4_status_from_ds5(std::uint8_t ds5_status) {
  // DualSense: low nibble capacity 0-10, high nibble 0 discharging,
  // 1 charging, 2 full. DualShock 4: 0x10 = cable, low nibble 0-10 (11 = full
  // on cable).
  const std::uint8_t capacity =
      std::min<std::uint8_t>(ds5_status & 0x0f, 10);
  const std::uint8_t state = ds5_status >> 4;
  if (state == 2) {
    return kDs4StatusCable | 11;
  }
  if (state == 1) {
    return kDs4StatusCable | capacity;
  }
  return capacity;
}

std::vector<std::uint8_t> ds4_input_report(std::span<const std::uint8_t> in,
                                           PersonaState &state) {
  std::vector<std::uint8_t> out(kDs4UsbInputReportSize, 0);
  out[0] = 0x01;
  out[1] = in[kDs5LeftX];
  out[2] = in[kDs5LeftY];
  out[3] = in[kDs5RightX];
  out[4] = in[kDs5RightY];
  // Hat/face and shoulder/stick button bytes share the DualShock 4 layout.
  out[kDs4Buttons0] = in[kDs5Buttons0];
  out[kDs4Buttons1] = in[kDs5Buttons1];
  std::uint8_t buttons2 =
      static_cast<std::uint8_t>((state.ds4_report_counter++ & 0x3f) << 2);
  buttons2 |= in[kDs5Buttons2] & (kDs5ButtonPs | kDs5ButtonTouchpad);
  out[kDs4Buttons2] = buttons2;
  out[kDs4L2] = in[kDs5L2];
  out[kDs4R2] = in[kDs5R2];
  // DualSense timestamps tick every 0.33 us, DualShock 4 every 5.33 us.
  write_le16(out.data() + kDs4Timestamp,
             static_cast<std::uint16_t>(read_le32(in.data() + kDs5Timestamp) /
                                        16));
  // Gyro and accelerometer: same raw units; the calibration reply maps the
  // DualSense calibration across.
  std::memcpy(out.data() + kDs4Gyro, in.data() + kDs5Gyro, 12);
  out[kDs4Status] = ds4_status_from_ds5(in[kDs5Status]);
  out[kDs4TouchReports] = 1;
  out[kDs4TouchTimestamp] = out[kDs4Timestamp];
  write_ds4_touch_point(out.data() + kDs4Touch, in.data() + kDs5Touch);
  write_ds4_touch_point(out.data() + kDs4Touch + 4, in.data() + kDs5Touch + 4);
  return out;
}

std::int16_t x360_axis(std::uint8_t value, bool invert) {
  int scaled = static_cast<int>(value) * 257 - 32768;
  if (invert) {
    scaled = -scaled - 1;
  }
  return static_cast<std::int16_t>(std::clamp(scaled, -32768, 32767));
}

std::vector<std::uint8_t> x360_input_report(std::span<const std::uint8_t> in) {
  std::vector<std::uint8_t> out(kX360InputReportSize, 0);
  const std::uint8_t face = in[kDs5Buttons0];
  const std::uint8_t shoulder = in[kDs5Buttons1];
  const std::uint8_t system = in[kDs5Buttons2];
  const std::uint8_t directions =
      kHatDirections[std::min<std::uint8_t>(face & 0x0f, 8)];

  std::uint16_t buttons = 0;
  if (directions & 0x1) buttons |= kX360DpadUp;
  if (directions & 0x2) buttons |= kX360DpadRight;
  if (directions & 0x4) buttons |= kX360DpadDown;
  if (directions & 0x8) buttons |= kX360DpadLeft;
  if (face & 0x10) buttons |= kX360X;      // square
  if (face & 0x20) buttons |= kX360A;      // cross
  if (face & 0x40) buttons |= kX360B;      // circle
  if (face & 0x80) buttons |= kX360Y;      // triangle
  if (shoulder & 0x01) buttons |= kX360LeftShoulder;
  if (shoulder & 0x02) buttons |= kX360RightShoulder;
  if (shoulder & 0x10) buttons |= kX360Back;  // create
  if (shoulder & 0x20) buttons |= kX360Start; // options
  if (shoulder & 0x40) buttons |= kX360LeftStick;
  if (shoulder & 0x80) buttons |= kX360RightStick;
  if (system & kDs5ButtonPs) buttons |= kX360Guide;

  out[0] = 0x00;
  out[1] = static_cast<std::uint8_t>(kX360InputReportSize);
  write_le16(out.data() + 2, buttons);
  out[4] = in[kDs5L2];
  out[5] = in[kDs5R2];
  // XInput Y axes point up; the DualSense's point down.
  write_le16(out.data() + 6,
             static_cast<std::uint16_t>(x360_axis(in[kDs5LeftX], false)));
  write_le16(out.data() + 8,
             static_cast<std::uint16_t>(x360_axis(in[kDs5LeftY], true)));
  write_le16(out.data() + 10,
             static_cast<std::uint16_t>(x360_axis(in[kDs5RightX], false)));
  write_le16(out.data() + 12,
             static_cast<std::uint16_t>(x360_axis(in[kDs5RightY], true)));
  return out;
}

void write_rumble(std::array<std::uint8_t, 1 + VDS_SET_STATE_SIZE> &report,
                  const PersonaState &state) {
  std::uint8_t *set = report.data() + 1;
  set[kSetFlag0] |= kSetFlag0Rumble;
  set[kSetRumbleRight] = state.rumble_right;
  set[kSetRumbleLeft] = state.rumble_left;
  set[kSetFlag2] |= kSetFlag2ImprovedRumble;
}

std::uint8_t ds5_player_lights_for_x360_led(std::uint8_t pattern) {
  // xpad LED patterns: 2-5 flash then 6-9 steady for players 1-4.
  static constexpr std::uint8_t kPlayerLights[4] = {0x04, 0x0a, 0x15, 0x1b};
  if (pattern >= 2 && pattern <= 5) {
    return kPlayerLights[pattern - 2];
  }
  if (pattern >= 6 && pattern <= 9) {
    return kPlayerLights[pattern - 6];
  }
  return 0;
}

} // namespace

std::optional<HostPersona> host_persona_from_value(std::uint8_t value) {
  switch (value) {
  case 0:
    return HostPersona::DualSense;
  case 1:
    return HostPersona::Xbox360;
  case 2:
    return HostPersona::Ds4;
  default:
    return std::nullopt;
  }
}

std::uint32_t usb_profile_for_persona(HostPersona persona,
                                      std::uint32_t dualsense_profile) {
  switch (persona) {
  case HostPersona::Ds4:
    return VDS_PROFILE_DS4;
  case HostPersona::Xbox360:
    return VDS_PROFILE_X360;
  case HostPersona::DualSense:
    break;
  }
  return dualsense_profile;
}

std::vector<std::uint8_t>
persona_input_report(HostPersona persona,
                     std::span<const std::uint8_t> ds5_usb_report,
                     PersonaState &state) {
  if (persona == HostPersona::DualSense ||
      ds5_usb_report.size() < kDs5MinimumReport) {
    return {};
  }
  if (persona == HostPersona::Ds4) {
    return ds4_input_report(ds5_usb_report, state);
  }
  return x360_input_report(ds5_usb_report);
}

std::optional<std::array<std::uint8_t, 1 + VDS_SET_STATE_SIZE>>
persona_output_to_ds5(HostPersona persona,
                      std::span<const std::uint8_t> report,
                      PersonaState &state) {
  std::array<std::uint8_t, 1 + VDS_SET_STATE_SIZE> out{};
  out[0] = VDS_USB_OUTPUT_REPORT_ID;
  std::uint8_t *set = out.data() + 1;

  if (persona == HostPersona::Ds4) {
    // DS4 USB output 0x05: flags0 (0x01 rumble, 0x02 lightbar), flags1,
    // reserved, weak (right) motor, strong (left) motor, R, G, B, ...
    if (report.size() < 9 || report[0] != kDs4UsbOutputReportId) {
      return std::nullopt;
    }
    const std::uint8_t flags = report[1];
    if (flags & 0x01) {
      state.rumble_right = report[4];
      state.rumble_left = report[5];
    }
    write_rumble(out, state);
    if (flags & 0x02) {
      set[kSetFlag1] |= kSetFlag1LedColor;
      set[kSetLedRed + 0] = report[6];
      set[kSetLedRed + 1] = report[7];
      set[kSetLedRed + 2] = report[8];
    }
    return out;
  }

  if (persona == HostPersona::Xbox360) {
    if (report.size() >= 5 && report[0] == 0x00 && report[1] == 0x08) {
      // Rumble: 00 08 00 <strong/left> <weak/right> 00 00 00.
      state.rumble_left = report[3];
      state.rumble_right = report[4];
      write_rumble(out, state);
      return out;
    }
    if (report.size() >= 3 && report[0] == 0x01 && report[1] == 0x03) {
      const std::uint8_t lights = ds5_player_lights_for_x360_led(report[2]);
      if (lights == 0) {
        return std::nullopt;
      }
      write_rumble(out, state);
      set[kSetFlag1] |= kSetFlag1PlayerLights;
      set[kSetPlayerLights] = lights;
      return out;
    }
  }
  return std::nullopt;
}

std::vector<std::uint8_t>
ds4_feature_report(std::uint8_t report_id, std::size_t requested_length,
                   std::span<const std::uint8_t> ds5_calibration,
                   std::span<const std::uint8_t> ds5_pairing) {
  std::vector<std::uint8_t> out;
  switch (report_id) {
  case 0x02: {
    // Gyro/accel calibration. The DualSense's report 0x05 holds the same 17
    // fields with the gyro plus/minus values interleaved; the DS4 USB layout
    // lists all three plus values before the minus values.
    out.assign(37, 0);
    out[0] = report_id;
    std::int16_t fields[17] = {0,    0,     0,    1024,  -1024, 1024,
                               -1024, 1024, -1024, 540,   540,   8192,
                               -8192, 8192, -8192, 8192,  -8192};
    if (ds5_calibration.size() >= 35 && ds5_calibration[0] == 0x05) {
      for (std::size_t i = 0; i < 17; ++i) {
        fields[i] = read_le16s(ds5_calibration.data() + 1 + i * 2);
      }
      const std::int16_t pitch_minus = fields[4];
      const std::int16_t yaw_plus = fields[5];
      const std::int16_t yaw_minus = fields[6];
      const std::int16_t roll_plus = fields[7];
      // DualSense order: pitch+, pitch-, yaw+, yaw-, roll+, roll-.
      // DS4 USB order:   pitch+, yaw+, roll+, pitch-, yaw-, roll-.
      fields[4] = yaw_plus;
      fields[5] = roll_plus;
      fields[6] = pitch_minus;
      fields[7] = yaw_minus;
    }
    for (std::size_t i = 0; i < 17; ++i) {
      write_le16(out.data() + 1 + i * 2, static_cast<std::uint16_t>(fields[i]));
    }
    break;
  }
  case 0x12:
    // Pairing info: controller MAC (little-endian) then host MAC. The
    // DualSense's report 0x09 carries its MAC little-endian at offset 1.
    out.assign(16, 0);
    out[0] = report_id;
    if (ds5_pairing.size() >= 7 && ds5_pairing[0] == 0x09) {
      std::copy_n(ds5_pairing.begin() + 1, 6, out.begin() + 1);
    }
    out[7] = 0x08;
    out[8] = 0x25;
    break;
  case 0xa3:
    // Firmware info: build date/time strings, hardware and firmware
    // versions (hid-playstation reads le16 at 35 and 41).
    out.assign(49, 0);
    out[0] = report_id;
    std::memcpy(out.data() + 1, "Sep 21 2018", 11);
    std::memcpy(out.data() + 17, "04:50:51", 8);
    write_le16(out.data() + 33, 0x0001);
    write_le16(out.data() + 35, 0xb400);
    write_le16(out.data() + 37, 0x0001);
    write_le16(out.data() + 41, 0xa00b);
    out[47] = 0x01;
    break;
  default:
    out.assign(std::clamp<std::size_t>(requested_length, 1, 64), 0);
    out[0] = report_id;
    break;
  }
  return out;
}

} // namespace vds
