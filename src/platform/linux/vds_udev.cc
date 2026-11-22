// SPDX-License-Identifier: MIT
// Copyright (C) 2026 Jihong Min <hurryman2212@gmail.com>

#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <libudev.h>

#include "vds_common.hh"
#include "vds_config.hh"
#include "vds_io.hh"
#include "vds_udev.hh"

namespace vds {

bool bluetooth_hid_device_present(const std::string &address) {
  const std::string normalized_address = normalize_bluetooth_address(address);
  udev *context = ::udev_new();
  if (context == nullptr) {
    return false;
  }

  udev_enumerate *enumerate = ::udev_enumerate_new(context);
  if (enumerate == nullptr) {
    ::udev_unref(context);
    return false;
  }

  if (::udev_enumerate_add_match_subsystem(enumerate, "hid") < 0 ||
      ::udev_enumerate_scan_devices(enumerate) < 0) {
    ::udev_enumerate_unref(enumerate);
    ::udev_unref(context);
    return false;
  }

  bool present = false;
  udev_list_entry *entries = ::udev_enumerate_get_list_entry(enumerate);
  udev_list_entry *entry = nullptr;
  udev_list_entry_foreach(entry, entries) {
    const char *syspath = ::udev_list_entry_get_name(entry);
    if (syspath == nullptr) {
      continue;
    }

    udev_device *device = ::udev_device_new_from_syspath(context, syspath);
    if (device == nullptr) {
      continue;
    }

    const char *name = ::udev_device_get_sysname(device);
    const char *uniq = ::udev_device_get_sysattr_value(device, "uniq");
    if (name != nullptr && std::string_view(name).rfind("0005:", 0) == 0 &&
        uniq != nullptr) {
      try {
        present = normalize_bluetooth_address(uniq) == normalized_address;
      } catch (const std::exception &) {
        present = false;
      }
    }

    ::udev_device_unref(device);
    if (present) {
      break;
    }
  }

  ::udev_enumerate_unref(enumerate);
  ::udev_unref(context);
  return present;
}

} // namespace vds
