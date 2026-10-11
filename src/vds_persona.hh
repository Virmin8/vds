// SPDX-License-Identifier: MIT
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

#include "uapi/vds.h"
#include "vds/ds5_protocol.h"

namespace vds {

/*
 * Host personas: the bridged DualSense can be presented to the PC as a
 * DualShock 4 or an Xbox 360 controller for games without DualSense support.
 * Values match the DS5 Bridge companion SET_HOST_PERSONA command.
 */
enum class HostPersona : std::uint8_t {
  DualSense = 0,
  Xbox360 = 1,
  Ds4 = 2,
};

inline constexpr std::uint8_t kSupportedHostPersonaMask = 0x07;

std::optional<HostPersona> host_persona_from_value(std::uint8_t value);

// The virtual USB profile for a persona; DualSense keeps the configured
// DualSense / DualSense Edge profile.
std::uint32_t usb_profile_for_persona(HostPersona persona,
                                      std::uint32_t dualsense_profile);

inline constexpr std::size_t kDs4UsbInputReportSize = 64;
inline constexpr std::size_t kX360InputReportSize = 20;
inline constexpr std::uint8_t kDs4UsbOutputReportId = 0x05;

// Persona-side state carried between reports.
struct PersonaState {
  std::uint8_t ds4_report_counter = 0;
  std::uint8_t rumble_right = 0;
  std::uint8_t rumble_left = 0;
};

// Converts a DualSense USB input report (report ID 0x01, 64 bytes) into the
// persona's input report. Returns an empty vector for the DualSense persona.
std::vector<std::uint8_t>
persona_input_report(HostPersona persona,
                     std::span<const std::uint8_t> ds5_usb_report,
                     PersonaState &state);

// Converts a persona output report (DS4 report 0x05, Xbox 360 rumble/LED
// packets) into a DualSense USB output report (0x02 + SetStateData).
std::optional<std::array<std::uint8_t, 1 + VDS_SET_STATE_SIZE>>
persona_output_to_ds5(HostPersona persona,
                      std::span<const std::uint8_t> report,
                      PersonaState &state);

// Builds a DS4 USB feature report reply (report ID first). Calibration (0x02)
// and the pairing MAC (0x12) are derived from the DualSense's own feature
// reports 0x05 and 0x09 when they are available.
std::vector<std::uint8_t>
ds4_feature_report(std::uint8_t report_id, std::size_t requested_length,
                   std::span<const std::uint8_t> ds5_calibration,
                   std::span<const std::uint8_t> ds5_pairing);

} // namespace vds
