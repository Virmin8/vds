// SPDX-License-Identifier: MIT
#include "vds_bt_mode.hh"

#include <cerrno>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string_view>
#include <vector>

#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

extern char **environ;

namespace vds {
namespace {

bool plugin_list_contains_input(std::string_view list) {
  while (!list.empty()) {
    const std::size_t separator = list.find_first_of(", ");
    if (list.substr(0, separator) == "input") {
      return true;
    }
    if (separator == std::string_view::npos) {
      break;
    }
    list.remove_prefix(separator + 1);
  }
  return false;
}

bool cmdline_disables_input(const std::vector<std::string> &args) {
  constexpr std::string_view kLongOption = "--noplugin=";
  for (std::size_t i = 0; i < args.size(); ++i) {
    const std::string_view arg = args[i];
    if (arg.starts_with(kLongOption) &&
        plugin_list_contains_input(arg.substr(kLongOption.size()))) {
      return true;
    }
    if ((arg == "--noplugin" || arg == "-P") && i + 1 < args.size() &&
        plugin_list_contains_input(args[i + 1])) {
      return true;
    }
    if (arg.starts_with("-P") && arg.size() > 2 &&
        plugin_list_contains_input(arg.substr(2))) {
      return true;
    }
  }
  return false;
}

std::string read_file(const std::filesystem::path &path) {
  std::ifstream file(path, std::ios::binary);
  return std::string(std::istreambuf_iterator<char>(file), {});
}

} // namespace

std::optional<bool> bluez_input_plugin_enabled() {
  std::error_code error;
  for (const auto &entry :
       std::filesystem::directory_iterator("/proc", error)) {
    const std::string name = entry.path().filename().string();
    if (name.empty() || name.find_first_not_of("0123456789") !=
                            std::string::npos) {
      continue;
    }
    std::string comm = read_file(entry.path() / "comm");
    while (!comm.empty() && (comm.back() == '\n' || comm.back() == '\0')) {
      comm.pop_back();
    }
    if (comm != "bluetoothd") {
      continue;
    }

    const std::string cmdline = read_file(entry.path() / "cmdline");
    std::vector<std::string> args;
    std::size_t start = 0;
    while (start < cmdline.size()) {
      const std::size_t end = cmdline.find('\0', start);
      args.push_back(cmdline.substr(start, end - start));
      if (end == std::string::npos) {
        break;
      }
      start = end + 1;
    }
    return !cmdline_disables_input(args);
  }
  return std::nullopt;
}

std::string run_bluetooth_override_script(const std::string &script,
                                          bool enable_input) {
  if (::access(script.c_str(), X_OK) != 0) {
    return "override script not executable: " + script + ": " +
           std::strerror(errno);
  }

  std::string action = enable_input ? "enable-input" : "disable-input";
  std::string restart = "--restart";
  std::string program = script;
  char *argv[] = {program.data(), restart.data(), action.data(), nullptr};

  pid_t pid = 0;
  const int spawn_error =
      ::posix_spawn(&pid, script.c_str(), nullptr, nullptr, argv, environ);
  if (spawn_error != 0) {
    return "failed to run " + script + ": " + std::strerror(spawn_error);
  }

  int status = 0;
  while (::waitpid(pid, &status, 0) < 0) {
    if (errno != EINTR) {
      return "failed to wait for " + script + ": " + std::strerror(errno);
    }
  }
  if (WIFEXITED(status) && WEXITSTATUS(status) == 0) {
    return {};
  }
  if (WIFEXITED(status)) {
    return script + " " + action + " exited with status " +
           std::to_string(WEXITSTATUS(status));
  }
  return script + " " + action + " terminated abnormally";
}

} // namespace vds
