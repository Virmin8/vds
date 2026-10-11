// SPDX-License-Identifier: MIT
#pragma once

#include <cstddef>
#include <optional>
#include <set>
#include <string>

#include <sys/types.h>

namespace vds {

/*
 * Host session tracking for "restore after native game": a game with native
 * DualSense support talks to the virtual controller through its hidraw node
 * and changes lights, triggers and so on for its session. vdsd watches who
 * has that node open so it can restore the previous output state once the
 * game closes it.
 */

// /dev/hidrawN of the virtual DualSense on port `port_index` (/dev/vdsN ->
// platform device vds_hcd.N), if the kernel has bound one.
std::optional<std::string> find_port_hidraw(std::size_t port_index);

// Processes (other than `exclude`) holding `node` open, from /proc/*/fd.
std::set<pid_t> node_openers(const std::string &node, pid_t exclude);

} // namespace vds
