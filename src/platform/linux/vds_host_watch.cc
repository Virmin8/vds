// SPDX-License-Identifier: MIT
#include "vds_host_watch.hh"

#include <cstdlib>
#include <filesystem>
#include <system_error>

namespace vds {

namespace fs = std::filesystem;

std::optional<std::string> find_port_hidraw(std::size_t port_index) {
  const std::string marker = "/vds_hcd." + std::to_string(port_index) + "/";
  std::error_code error;
  for (const auto &entry : fs::directory_iterator("/sys/class/hidraw", error)) {
    const fs::path device = fs::canonical(entry.path() / "device", error);
    if (error) {
      error.clear();
      continue;
    }
    if (device.string().find(marker) != std::string::npos) {
      return "/dev/" + entry.path().filename().string();
    }
  }
  return std::nullopt;
}

std::set<pid_t> node_openers(const std::string &node, pid_t exclude) {
  std::set<pid_t> openers;
  std::error_code error;
  for (const auto &process : fs::directory_iterator("/proc", error)) {
    const std::string name = process.path().filename().string();
    char *end = nullptr;
    const long pid = std::strtol(name.c_str(), &end, 10);
    if (end == name.c_str() || *end != '\0' || pid == exclude) {
      continue;
    }
    std::error_code fd_error;
    for (const auto &fd : fs::directory_iterator(process.path() / "fd", fd_error)) {
      std::error_code link_error;
      if (fs::read_symlink(fd.path(), link_error) == node) {
        openers.insert(static_cast<pid_t>(pid));
        break;
      }
    }
  }
  return openers;
}

} // namespace vds
