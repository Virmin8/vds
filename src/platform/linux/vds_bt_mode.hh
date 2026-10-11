// SPDX-License-Identifier: MIT
#pragma once

#include <optional>
#include <string>

namespace vds {

inline constexpr const char *kDefaultBluetoothOverrideScript =
    "/usr/share/opends5/override-bluetoothd.sh";

// Whether the running bluetoothd has its input plugin enabled, i.e. it was not
// started with --noplugin=input / -P input. nullopt when bluetoothd is not
// running.
std::optional<bool> bluez_input_plugin_enabled();

// Runs `script enable-input|disable-input --restart` and waits for it.
// Returns an empty string on success, otherwise a description of the failure.
std::string run_bluetooth_override_script(const std::string &script,
                                          bool enable_input);

} // namespace vds
