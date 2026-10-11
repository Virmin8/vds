// SPDX-License-Identifier: MIT
// Copyright (C) 2026 Jihong Min <hurryman2212@gmail.com>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <filesystem>
#include <fstream>
#include <set>
#include <iostream>
#include <map>
#include <mutex>
#include <optional>
#include <span>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>
#include <dirent.h>
#include <grp.h>
#include <linux/input.h>
#include <sys/epoll.h>
#include <sys/inotify.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

#include "uapi/vds.h"
#include "unique_fd.hh"
#include "vds/ds5_protocol.h"
#include "vds_bluez.hh"
#include "vds_sleep_watch.hh"
#include "vds_host_watch.hh"
#include "vds_bt.hh"
#include "vds_bridge.hh"
#include "vds_bt_mode.hh"
#include "vds_build_info.hh"
#include "vds_common.hh"
#include "vds_config.hh"
#include "vds_io.hh"
#include "vds_log.hh"
#include "vds_persona.hh"
#include "vds_profile.hh"
#include "vds_protocol.hh"
#include "vds_udev.hh"
#include "vds_companion.hh"
#include "vdsd_common.hh"

namespace {

using Clock = std::chrono::steady_clock;
using vds::duration_us;
using vds::hex_bytes;
using vds::hex_u8;

constexpr const char *kDefaultControlSocket = "/run/vdsd.sock";
constexpr const char *kVirtualPortProviderUnavailableReason =
    "virtual port provider unavailable";
constexpr const char *kLinuxVirtualPortProviderUnavailable =
    "vds_hcd kernel module is not loaded or no /dev/vds# ports are available";
constexpr int kMaxPortFramesPerWake = 64;
constexpr int kMaxBtPacketsPerWake = 64;
constexpr auto kBluetoothPreemptWait = std::chrono::milliseconds(2000);
constexpr auto kBluetoothPreemptPoll = std::chrono::milliseconds(100);
constexpr int kInitialFeatureReportPollMs = 5000;
constexpr std::array<std::uint8_t, 3> kInitialFeatureReportIds = {0x09, 0x20,
                                                                  0x05};

volatile sig_atomic_t g_stop_requested = 0;
volatile sig_atomic_t g_log_reopen_requested = 0;
// True while BlueZ's input plugin owns the HID PSMs ("normal" Bluetooth mode);
// vdsd must then leave BlueZ-connected controllers alone.
bool g_bluetooth_normal_mode = false;

struct Options {
  std::string socket = kDefaultControlSocket;
  std::string log_path;
  std::string db_path;
  std::string bluetooth_override_script = vds::kDefaultBluetoothOverrideScript;
  float speaker_boost_db = vds::kDefaultSpeakerBoostDb;
  bool mic_enabled = false;
};

using vds::kTraceOutput;
using vds::trace_enabled;
using vds::trim_command;

// A vds_hcd port: the shared bridge state plus its device node and the
// native-game watch.
struct VirtualPort : vds::BridgePort {
  vds::UniqueFd fd;
  // Restore after a native DualSense game: watch the virtual controller's
  // hidraw node. Each program that opens it (in order) keeps a snapshot of
  // the output state from just before it did; when the latest one closes it,
  // that snapshot is restored (a Steam started later stays underneath).
  struct HostSession {
    pid_t pid;
    vds::DsOutputState::HostState before;
  };
  std::string hidraw_node;
  int hidraw_watch = -1;
  std::set<pid_t> host_baseline;
  std::vector<HostSession> host_sessions;
};

struct ControllerRuntime {
  vds::ControllerConfig config;
  std::string device;
  std::optional<std::uint32_t> detected_profile;
  std::optional<vds::BtL2capBackend> backend;
  vds::UniqueFd pending_control_fd;
  vds::UniqueFd pending_interrupt_fd;
  bool virtual_connected = false;
  std::string last_error;
};

bool write_vds_frame(VirtualPort &port, std::span<const std::uint8_t> bytes);

// vds_bridge's view of a controller's L2CAP link: feature reports are fetched
// asynchronously and answered from handle_bt_control.
class LinuxBridgeBluetooth final : public vds::BridgeBluetooth {
public:
  explicit LinuxBridgeBluetooth(vds::BtL2capBackend &backend)
      : backend_(backend) {}

  bool try_send_output_report(std::span<const std::uint8_t> report) override {
    return backend_.try_send_output_report(report);
  }
  std::optional<std::vector<std::uint8_t>>
  get_feature_report(std::uint8_t report_id) override {
    backend_.send_feature_get(report_id);
    return std::nullopt;
  }
  void set_feature_report(std::span<const std::uint8_t> report) override {
    backend_.send_feature_set(report);
  }

private:
  vds::BtL2capBackend &backend_;
};

// vds_bridge's view of a /dev/vds# port.
class LinuxBridgeUsb final : public vds::BridgeUsb {
public:
  explicit LinuxBridgeUsb(VirtualPort &port) : port_(port) {}

  bool write_frame(std::uint32_t type,
                   std::span<const std::uint8_t> payload) override {
    return write_vds_frame(
        port_, vds::frame_bytes(static_cast<std::uint16_t>(type), payload));
  }

private:
  VirtualPort &port_;
};

enum class EventType : std::uint32_t {
  Control = 1,
  Port = 2,
  BtControl = 3,
  BtInterrupt = 4,
  BtAcceptControl = 5,
  BtAcceptInterrupt = 6,
  Udev = 7,
  Sleep = 8,
  HostWatch = 9,
  ControlClient = 10,
};

struct EventSource {
  EventType type;
  std::size_t index;
};

// A companion/CLI connection on the control socket. Clients are non-blocking
// and served from the main epoll loop: a client that connects and writes
// later must not stall audio, haptics or controller input while it does.
struct ControlClient {
  std::uint32_t id = 0;
  vds::UniqueFd fd;
  std::string request;
  std::string reply;
  std::size_t reply_sent = 0;
  bool replying = false;
  Clock::time_point deadline;
};

constexpr auto kControlClientTimeout = std::chrono::seconds(1);
constexpr std::size_t kMaxControlClients = 16;
constexpr std::size_t kMaxControlRequest = 8192;

class SocketPathGuard {
public:
  explicit SocketPathGuard(std::string path) : path_(std::move(path)) {}
  ~SocketPathGuard() { ::unlink(path_.c_str()); }

  SocketPathGuard(const SocketPathGuard &) = delete;
  SocketPathGuard &operator=(const SocketPathGuard &) = delete;

private:
  std::string path_;
};

void signal_handler(int signal) {
  if (signal == SIGHUP) {
    g_log_reopen_requested = 1;
    return;
  }
  g_stop_requested = 1;
}

void install_signal_handler(int signal) {
  struct sigaction action{};
  action.sa_handler = signal_handler;
  sigemptyset(&action.sa_mask);
  if (::sigaction(signal, &action, nullptr) < 0) {
    throw std::runtime_error("sigaction failed: " +
                             std::string(std::strerror(errno)));
  }
}

Options parse_platform_args(int argc, char **argv) {
  Options options;
  vds::VdsdCommonOptions common = vds::default_vdsd_common_options();
  const std::string platform_options =
      std::string("[--socket ") + kDefaultControlSocket +
      "] [--bluetooth-override-script " +
      vds::kDefaultBluetoothOverrideScript + "] [--speaker-boost-db 0-18] [--mic|--no-mic]";
  const auto next_value = [&](int &index, std::string_view option) {
    if (index + 1 >= argc) {
      throw std::runtime_error(std::string(option) + " requires a value");
    }
    return std::string(argv[++index]);
  };

  for (int i = 1; i < argc; ++i) {
    const std::string_view arg = argv[i];
    if (vds::parse_vdsd_common_option(argc, argv, i, common)) {
      if (common.help_requested) {
        vds::print_vdsd_usage(std::cerr, vds::kVersion, vds::kBuildYear,
                              platform_options);
        std::exit(0);
      }
    } else if (arg == "--socket") {
      options.socket = next_value(i, arg);
    } else if (arg == "--bluetooth-override-script") {
      options.bluetooth_override_script = next_value(i, arg);
    } else if (arg == "--speaker-boost-db") {
      const std::string value = next_value(i, arg);
      try {
        options.speaker_boost_db = std::stof(value);
      } catch (const std::exception &) {
        throw std::runtime_error("--speaker-boost-db needs a number");
      }
      if (options.speaker_boost_db < 0.0f ||
          options.speaker_boost_db > vds::kMaxSpeakerBoostDb) {
        throw std::runtime_error("--speaker-boost-db must be 0-18");
      }
    } else if (arg == "--mic") {
      options.mic_enabled = true;
    } else if (arg == "--no-mic") {
      options.mic_enabled = false;
    } else {
      throw std::runtime_error("unknown argument: " + std::string(arg));
    }
  }
  options.db_path = common.db_path;
  options.log_path = common.log_path;
  return options;
}

int open_required(const std::string &path, int flags) {
  const int fd = ::open(path.c_str(), flags);
  if (fd < 0) {
    throw std::runtime_error("failed to open " + path + ": " +
                             std::strerror(errno));
  }
  return fd;
}

const char *profile_name(std::uint32_t profile) {
  if (profile == VDS_PROFILE_DSE) {
    return "dualsense-edge";
  }
  return "dualsense";
}

std::uint32_t detect_bluetooth_profile(const std::string &address) {
  const auto modalias = vds::bluez_device_modalias(address);
  if (!modalias) {
    throw std::runtime_error("failed to query BlueZ modalias for " + address);
  }
  const auto profile = vds::controller_profile_from_modalias(*modalias);
  if (!profile) {
    throw std::runtime_error("failed to detect Bluetooth device ID for " +
                             address);
  }
  return vds::usb_profile_from_controller_profile(*profile);
}

std::uint32_t resolve_config_profile(const vds::ControllerConfig &controller) {
  switch (controller.profile) {
  case vds::ControllerProfile::Unspecified:
    return detect_bluetooth_profile(controller.address);
  case vds::ControllerProfile::Ds5:
  case vds::ControllerProfile::Dse:
    return vds::usb_profile_from_controller_profile(controller.profile);
  }
  throw std::runtime_error("unknown controller profile");
}

int open_control_socket(const std::string &path) {
  vds::UniqueFd fd(
      ::socket(AF_UNIX, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0));
  if (!fd) {
    throw std::runtime_error("failed to open control socket: " +
                             std::string(std::strerror(errno)));
  }

  ::unlink(path.c_str());
  sockaddr_un address{};
  address.sun_family = AF_UNIX;
  if (path.size() >= sizeof(address.sun_path)) {
    throw std::runtime_error("control socket path is too long: " + path);
  }
  std::strncpy(address.sun_path, path.c_str(), sizeof(address.sun_path) - 1);

  if (::bind(fd.get(), reinterpret_cast<const sockaddr *>(&address),
             sizeof(address)) < 0) {
    const int error = errno;
    throw std::runtime_error("failed to bind control socket " + path + ": " +
                             std::strerror(error));
  }
  // When a "vds" group exists, grant it socket access so unprivileged
  // clients (the companion app, vdsctl) can talk to the daemon.
  if (const struct group *vds_group = ::getgrnam("vds")) {
    (void)::chown(path.c_str(), 0, vds_group->gr_gid);
  }
  (void)::chmod(path.c_str(), 0660);

  if (::listen(fd.get(), 4) < 0) {
    const int error = errno;
    throw std::runtime_error("failed to listen on control socket " + path +
                             ": " + std::strerror(error));
  }
  return fd.release();
}

// Writes one frame to the port; false when its queue is full (dropped).
bool write_vds_frame(VirtualPort &port, std::span<const std::uint8_t> bytes) {
  while (true) {
    const ssize_t written = ::write(port.fd.get(), bytes.data(), bytes.size());
    if (written == static_cast<ssize_t>(bytes.size())) {
      return true;
    }
    if (written >= 0) {
      throw std::runtime_error("short " + port.path + " frame write");
    }
    if (errno == EINTR) {
      continue;
    }
    if (errno == EAGAIN || errno == EWOULDBLOCK || errno == ENOSPC) {
      return false;
    }
    throw std::runtime_error("write failed: " +
                             std::string(std::strerror(errno)));
  }
}

void ioctl_noarg(int fd, unsigned long request, const char *name) {
  if (::ioctl(fd, request) < 0) {
    throw std::runtime_error(std::string(name) +
                             " failed: " + std::strerror(errno));
  }
}

void ioctl_set_profile(int fd, std::uint32_t profile) {
  vds_profile_config config{
      .profile = profile,
      .polling_rate_mode = 0,
  };
  if (::ioctl(fd, VDS_IOC_SET_PROFILE, &config) < 0) {
    throw std::runtime_error("VDS_IOC_SET_PROFILE failed: " +
                             std::string(std::strerror(errno)));
  }
}

void reset_virtual_port(VirtualPort &port) {
  vds::bridge_reset(port);
  port.host_sessions.clear();
}

void disconnect_virtual_port(VirtualPort &port, vds::Logger &logger) {
  vds::bridge_on_disconnect(port);
  try {
    ioctl_noarg(port.fd.get(), VDS_IOC_DISCONNECT, "VDS_IOC_DISCONNECT");
    logger.log(vds::LogScope::Usb, vds::LogLevel::Info,
               port.path + " virtual USB disconnected");
  } catch (const std::exception &error) {
    logger.log(vds::LogScope::Usb, vds::LogLevel::Warn,
               port.path + " disconnect failed: " + error.what());
  }
}

void sync_port_mic(VirtualPort &port, vds::BtL2capBackend *bt_backend,
                   vds::Logger &logger) {
  if (bt_backend == nullptr) {
    vds::bridge_sync_mic(port, nullptr, logger);
    return;
  }
  LinuxBridgeBluetooth bt(*bt_backend);
  vds::bridge_sync_mic(port, &bt, logger);
}

void forward_bt_state_if_changed(VirtualPort &port,
                                 vds::BtL2capBackend &bt_backend,
                                 std::uint32_t trace_flags, vds::Logger &logger,
                                 std::string_view reason) {
  LinuxBridgeBluetooth bt(bt_backend);
  vds::bridge_forward_state_if_changed(port, bt, trace_flags, logger, reason);
}

void handle_frame(const vds_frame_header &header,
                  std::span<const std::uint8_t> payload,
                  vds::BtL2capBackend *bt_backend, std::uint32_t trace_flags,
                  VirtualPort &port, vds::Logger &logger) {
  LinuxBridgeUsb usb(port);
  if (bt_backend == nullptr) {
    vds::bridge_handle_usb_frame(port, header.type, payload, nullptr, usb,
                                 trace_flags, logger);
    return;
  }
  LinuxBridgeBluetooth bt(*bt_backend);
  vds::bridge_handle_usb_frame(port, header.type, payload, &bt, usb,
                               trace_flags, logger);
}

bool handle_vds_frame(VirtualPort &port, vds::BtL2capBackend *bt_backend,
                      std::uint32_t trace_flags, vds::Logger &logger) {
  std::array<std::uint8_t, sizeof(vds_frame_header) + VDS_FRAME_MAX_PAYLOAD>
      frame;
  const ssize_t got = ::read(port.fd.get(), frame.data(), frame.size());
  if (got == 0) {
    throw std::runtime_error(port.path + " closed");
  }
  if (got < 0) {
    if (errno == EINTR || errno == EAGAIN) {
      return false;
    }
    throw std::runtime_error(
        port.path + " read failed: " + std::string(std::strerror(errno)));
  }
  if (static_cast<std::size_t>(got) < sizeof(vds_frame_header)) {
    throw std::runtime_error(port.path + " short VDS frame header");
  }
  vds_frame_header header{};
  std::memcpy(&header, frame.data(), sizeof(header));
  if (header.length > VDS_FRAME_MAX_PAYLOAD) {
    throw std::runtime_error(port.path + " oversized VDS frame");
  }
  if (static_cast<std::size_t>(got) != sizeof(header) + header.length) {
    throw std::runtime_error(port.path + " short VDS frame payload");
  }
  const std::span payload(frame.data() + sizeof(header), header.length);
  handle_frame(header, payload, bt_backend, trace_flags, port, logger);
  return true;
}

bool handle_bt_input(VirtualPort &port, vds::BtL2capBackend &bt_backend,
                     vds::CompanionRuntime &companion,
                     std::uint32_t trace_flags, vds::Logger &logger) {
  const auto read_time = Clock::now();
  const auto packet = bt_backend.read_interrupt_packet();
  if (!packet) {
    return false;
  }

  LinuxBridgeBluetooth bt(bt_backend);
  LinuxBridgeUsb usb(port);
  const vds::BridgeInputResult result = vds::bridge_handle_bt_input(
      port, *packet, read_time, bt, usb, companion, trace_flags, logger);
  if (result.idle_timeout) {
    // Turning the controller off drops the link like the companion's Turn
    // Off; the normal hang-up path then cleans up.
    try {
      (void)vds::disconnect_bluez_device(bt_backend.address());
    } catch (const std::exception &error) {
      logger.log(vds::LogScope::Bluetooth, vds::LogLevel::Warn,
                 "idle turn off failed address=" + bt_backend.address() +
                     " error=" + error.what());
    }
  }
  return true;
}

bool handle_bt_control(VirtualPort &port, vds::BtL2capBackend &bt_backend,
                       std::uint32_t trace_flags, vds::Logger &logger) {
  const auto report = bt_backend.read_feature_report();
  if (!report) {
    return false;
  }
  LinuxBridgeUsb usb(port);
  vds::bridge_handle_feature_reply(port, *report, usb, trace_flags, logger);
  return true;
}

bool is_bluetooth_error(const std::exception &error) {
  return std::string_view(error.what()).rfind("Bluetooth L2CAP", 0) == 0 ||
         std::string_view(error.what())
                 .rfind("failed to connect Bluetooth", 0) == 0;
}

void initialize_bt_controller(vds::BtL2capBackend &bt_backend,
                              VirtualPort &port, vds::Logger &logger) {
  for (const std::uint8_t report_id : kInitialFeatureReportIds) {
    bt_backend.send_feature_get(report_id);

    while (true) {
      pollfd pfd{.fd = bt_backend.control_fd(), .events = POLLIN, .revents = 0};
      const int ready = ::poll(&pfd, 1, kInitialFeatureReportPollMs);
      if (ready < 0 && errno == EINTR) {
        continue;
      }
      if (ready <= 0) {
        throw std::runtime_error("timed out waiting for initial Bluetooth "
                                 "feature report " +
                                 hex_u8(report_id));
      }
      if ((pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
        throw std::runtime_error("Bluetooth control channel closed during "
                                 "initial feature report fetch");
      }
      if ((pfd.revents & POLLIN) == 0) {
        continue;
      }

      const auto report = bt_backend.read_feature_report();
      if (!report || report->empty()) {
        continue;
      }

      LinuxBridgeUsb usb(port);
      vds::bridge_cache_feature_report(port, *report, usb, false, logger);
      if ((*report)[0] == report_id) {
        break;
      }
    }
  }

  logger.log(vds::LogScope::Hid, vds::LogLevel::Info,
             port.path + " primed initial Bluetooth feature cache");
}

std::optional<std::size_t> find_port_index(std::span<const VirtualPort> ports,
                                           const std::string &path) {
  for (std::size_t i = 0; i < ports.size(); ++i) {
    if (ports[i].path == path) {
      return i;
    }
  }
  return std::nullopt;
}

std::vector<unsigned> present_port_indices(std::span<const VirtualPort> ports) {
  std::vector<unsigned> indices;
  indices.reserve(ports.size());
  for (const auto &port : ports) {
    if (const auto index = vds::port_index_from_path(port.path)) {
      indices.push_back(*index);
    }
  }
  return indices;
}

bool controller_uses_port(const ControllerRuntime &controller) {
  return controller.backend || controller.pending_control_fd ||
         controller.pending_interrupt_fd || controller.virtual_connected;
}

bool port_used_by_other_controller(
    std::span<const ControllerRuntime> controllers,
    const ControllerRuntime &self, const std::string &device) {
  for (const auto &controller : controllers) {
    if (&controller == &self) {
      continue;
    }
    if (controller.device == device && controller_uses_port(controller)) {
      return true;
    }
  }
  return false;
}

std::optional<std::size_t>
available_port_index(std::span<const VirtualPort> ports,
                     std::span<const ControllerRuntime> controllers,
                     const ControllerRuntime &controller) {
  const std::vector<unsigned> candidate_ports = present_port_indices(ports);
  std::vector<unsigned> occupied_ports;
  occupied_ports.reserve(controllers.size());
  for (const auto &other : controllers) {
    if (&other == &controller || !controller_uses_port(other)) {
      continue;
    }
    if (const auto port = vds::port_index_from_path(other.device)) {
      occupied_ports.push_back(*port);
    }
  }

  const auto selected_port = vds::select_controller_config_port(
      controller.config, candidate_ports, occupied_ports);
  if (!selected_port) {
    return std::nullopt;
  }

  for (std::size_t i = 0; i < ports.size(); ++i) {
    if (vds::port_index_from_path(ports[i].path) == selected_port) {
      return i;
    }
  }
  return std::nullopt;
}

bool controller_has_present_allowed_port(std::span<const VirtualPort> ports,
                                         const vds::ControllerConfig &config) {
  return vds::controller_config_has_candidate_port(config,
                                                   present_port_indices(ports));
}

ControllerRuntime *
controller_for_port(std::vector<ControllerRuntime> &controllers,
                    const std::string &device) {
  for (auto &controller : controllers) {
    if (controller.device == device && controller_uses_port(controller)) {
      return &controller;
    }
  }
  return nullptr;
}

ControllerRuntime *
controller_for_address(std::vector<ControllerRuntime> &controllers,
                       const std::string &address) {
  for (auto &controller : controllers) {
    if (controller.config.address == address) {
      return &controller;
    }
  }
  return nullptr;
}

void drop_bt_backend(ControllerRuntime &controller, VirtualPort &port,
                     const std::string &reason, vds::Logger &logger) {
  if (!controller.backend && !controller.virtual_connected) {
    return;
  }

  port.pending_bt_state.reset();
  port.pending_bt_state_report.reset();
  logger.log(vds::LogScope::Bluetooth, vds::LogLevel::Warn,
             "backend disconnected address=" + controller.config.address +
                 " device=" + controller.device + " reason=" + reason);
  controller.backend.reset();
  controller.pending_control_fd.reset();
  controller.pending_interrupt_fd.reset();
  controller.virtual_connected = false;
  controller.device.clear();
  disconnect_virtual_port(port, logger);
}

void sync_virtual_ports(std::vector<VirtualPort> &ports,
                        std::span<const std::string> devices,
                        vds::Logger &logger) {
  std::vector<bool> moved(ports.size(), false);
  std::vector<VirtualPort> updated;
  updated.reserve(devices.size());

  for (const auto &device : devices) {
    auto it =
        std::find_if(ports.begin(), ports.end(), [&](const VirtualPort &port) {
          return port.path == device;
        });
    if (it != ports.end()) {
      const std::size_t old_index =
          static_cast<std::size_t>(std::distance(ports.begin(), it));
      moved[old_index] = true;
      updated.push_back(std::move(*it));
      continue;
    }

    vds::UniqueFd fd(open_required(device, O_RDWR | O_NONBLOCK | O_CLOEXEC));
    vds_driver_info driver_info{};
    std::string driver_message = "driver connected name=vds_hcd path=" + device;
    vds::LogLevel driver_log_level = vds::LogLevel::Info;
    if (::ioctl(fd.get(), VDS_IOC_GET_DRIVER_INFO, &driver_info) < 0) {
      driver_log_level = vds::LogLevel::Warn;
      driver_message =
          "driver version unavailable name=vds_hcd path=" + device +
          " detail=" + std::strerror(errno);
    } else if (driver_info.version != VDS_DRIVER_INFO_VERSION ||
               driver_info.size != sizeof(driver_info) ||
               driver_info.driver_version[0] == '\0' ||
               driver_info.driver_version[VDS_DRIVER_VERSION_MAX - 1] != '\0') {
      driver_log_level = vds::LogLevel::Warn;
      driver_message =
          "driver version unavailable name=vds_hcd path=" + device +
          " detail=invalid driver version reply";
    } else {
      driver_message = "driver connected name=vds_hcd version=" +
                       std::string(driver_info.driver_version) +
                       " path=" + device;
    }
    logger.log(vds::LogScope::Port, driver_log_level, driver_message);
    VirtualPort port;
    port.path = device;
    port.fd = std::move(fd);
    updated.push_back(std::move(port));
  }

  for (std::size_t i = 0; i < ports.size(); ++i) {
    if (!moved[i]) {
      logger.log(vds::LogScope::Port, vds::LogLevel::Info,
                 "closed " + ports[i].path);
    }
  }
  if (updated.empty()) {
    logger.log(vds::LogScope::Port, vds::LogLevel::Warn,
               "no /dev/vds* endpoints found");
  }
  ports = std::move(updated);
}

void preempt_default_bluetooth_owner(const std::string &address,
                                     vds::Logger &logger) {
  if (g_bluetooth_normal_mode ||
      !vds::bluetooth_hid_device_present(address)) {
    return;
  }

  logger.log(vds::LogScope::Bluetooth, vds::LogLevel::Warn,
             "preempting default Bluetooth HID owner address=" + address);
  try {
    if (!vds::disconnect_bluez_device(address)) {
      logger.log(vds::LogScope::Bluetooth, vds::LogLevel::Warn,
                 "BlueZ device not found for disconnect address=" + address);
    }
  } catch (const std::exception &error) {
    logger.log(vds::LogScope::Bluetooth, vds::LogLevel::Warn,
               "BlueZ disconnect failed address=" + address +
                   " error=" + error.what());
  }

  const auto deadline = Clock::now() + kBluetoothPreemptWait;
  while (Clock::now() < deadline) {
    if (!vds::bluetooth_hid_device_present(address)) {
      logger.log(vds::LogScope::Bluetooth, vds::LogLevel::Info,
                 "default stack released address=" + address);
      return;
    }
    std::this_thread::sleep_for(kBluetoothPreemptPoll);
  }

  logger.log(vds::LogScope::Bluetooth, vds::LogLevel::Warn,
             "default Bluetooth HID owner still present address=" + address);
}

void complete_pending_controller(ControllerRuntime &controller,
                                 VirtualPort &port, vds::Logger &logger) {
  const std::uint32_t profile = resolve_config_profile(controller.config);
  preempt_default_bluetooth_owner(controller.config.address, logger);

  vds::BtL2capBackend candidate(controller.config.address,
                                std::move(controller.pending_control_fd),
                                std::move(controller.pending_interrupt_fd));

  reset_virtual_port(port);
  port.persona = vds::bridge_settings().host_persona;
  port.persona_state = {};
  ioctl_noarg(port.fd.get(), VDS_IOC_DISCONNECT, "VDS_IOC_DISCONNECT");
  ioctl_set_profile(port.fd.get(),
                    vds::usb_profile_for_persona(port.persona, profile));
  initialize_bt_controller(candidate, port, logger);
  candidate.send_output_report(port.output_state.build_bt_init_report());
  port.last_sent_bt_state = port.output_state.state();
  logger.log(vds::LogScope::Hid, vds::LogLevel::Info,
             port.path + " sent initial Bluetooth state report");

  controller.backend = std::move(candidate);
  controller.detected_profile = profile;
  ioctl_noarg(port.fd.get(), VDS_IOC_CONNECT, "VDS_IOC_CONNECT");
  controller.virtual_connected = true;
  controller.last_error.clear();
  // Mic on: open the controller mic now so it is ready before any recording.
  sync_port_mic(port, &*controller.backend, logger);

  logger.log(
      vds::LogScope::Bluetooth, vds::LogLevel::Info,
      "raw L2CAP backend connected address=" + controller.config.address +
          " device=" + port.path + " profile=" + profile_name(profile));
}

void handle_bt_accept(std::vector<VirtualPort> &ports,
                      std::vector<ControllerRuntime> &controllers,
                      vds::BtL2capAcceptor &acceptor, bool control_channel,
                      vds::Logger &logger, bool &epoll_dirty) {
  while (true) {
    auto accepted = control_channel ? acceptor.accept_control()
                                    : acceptor.accept_interrupt();
    if (!accepted) {
      return;
    }

    ControllerRuntime *controller =
        controller_for_address(controllers, accepted->address);
    if (!controller) {
      logger.log(vds::LogScope::Bluetooth, vds::LogLevel::Warn,
                 "rejected raw HID channel from unregistered address=" +
                     accepted->address);
      continue;
    }
    if (controller->backend) {
      logger.log(vds::LogScope::Bluetooth, vds::LogLevel::Warn,
                 "rejected duplicate raw HID channel address=" +
                     accepted->address);
      continue;
    }

    std::optional<std::size_t> port_index;
    if (!controller->device.empty()) {
      port_index = find_port_index(ports, controller->device);
      if (port_index && port_used_by_other_controller(controllers, *controller,
                                                      controller->device)) {
        port_index.reset();
      }
    }
    if (!port_index) {
      if (ports.empty()) {
        controller->last_error = kLinuxVirtualPortProviderUnavailable;
        logger.log(vds::LogScope::Port, vds::LogLevel::Error,
                   "rejected raw HID channel address=" + accepted->address +
                       " reason=" + kVirtualPortProviderUnavailableReason +
                       " detail=" + kLinuxVirtualPortProviderUnavailable);
        continue;
      }
      port_index = available_port_index(ports, controllers, *controller);
      if (!port_index) {
        controller->last_error = "no available virtual port";
        logger.log(vds::LogScope::Port, vds::LogLevel::Warn,
                   "rejected raw HID channel address=" + accepted->address +
                       " reason=no available virtual port");
        continue;
      }
      controller->device = ports[*port_index].path;
      logger.log(vds::LogScope::Config, vds::LogLevel::Info,
                 "controller assigned address=" + controller->config.address +
                     " device=" + controller->device + " profile=\"" +
                     vds::controller_profile_name(controller->config.profile) +
                     "\"");
    }

    if (control_channel) {
      controller->pending_control_fd = std::move(accepted->fd);
      logger.log(vds::LogScope::Bluetooth, vds::LogLevel::Info,
                 "accepted raw HID control channel address=" +
                     accepted->address);
    } else {
      controller->pending_interrupt_fd = std::move(accepted->fd);
      logger.log(vds::LogScope::Bluetooth, vds::LogLevel::Info,
                 "accepted raw HID interrupt channel address=" +
                     accepted->address);
    }

    if (!controller->pending_control_fd || !controller->pending_interrupt_fd) {
      continue;
    }

    port_index = find_port_index(ports, controller->device);
    if (!port_index) {
      controller->pending_control_fd.reset();
      controller->pending_interrupt_fd.reset();
      controller->device.clear();
      controller->last_error = "assigned virtual port disappeared";
      logger.log(vds::LogScope::Port, vds::LogLevel::Error,
                 "controller inactive address=" + controller->config.address +
                     " reason=assigned port disappeared");
      continue;
    }

    try {
      complete_pending_controller(*controller, ports[*port_index], logger);
      epoll_dirty = true;
    } catch (const std::exception &error) {
      controller->backend.reset();
      controller->pending_control_fd.reset();
      controller->pending_interrupt_fd.reset();
      controller->virtual_connected = false;
      controller->device.clear();
      controller->last_error = error.what();
      logger.log(vds::LogScope::Bluetooth, vds::LogLevel::Error,
                 "connect failed address=" + controller->config.address +
                     " device=" + ports[*port_index].path +
                     " error=" + error.what());
    }
  }
}

int next_wakeup_timeout_ms(std::span<const VirtualPort> ports,
                           std::span<const ControllerRuntime> controllers) {
  const auto now = Clock::now();
  int timeout_ms = 60000;
  for (const auto &controller : controllers) {
    if (!controller.backend) {
      continue;
    }
    const auto port_index = find_port_index(ports, controller.device);
    if (!port_index) {
      continue;
    }
    if (const auto wait = vds::bridge_wakeup_ms(ports[*port_index], now)) {
      timeout_ms = std::min(timeout_ms, *wait);
    }
  }
  return timeout_ms;
}

void flush_pending_outputs(std::vector<VirtualPort> &ports,
                           std::vector<ControllerRuntime> &controllers,
                           std::uint32_t trace_flags, vds::Logger &logger,
                           bool &epoll_dirty) {
  for (auto &controller : controllers) {
    if (!controller.backend) {
      continue;
    }
    const auto port_index = find_port_index(ports, controller.device);
    if (!port_index) {
      controller.backend.reset();
      controller.pending_control_fd.reset();
      controller.pending_interrupt_fd.reset();
      controller.virtual_connected = false;
      controller.device.clear();
      epoll_dirty = true;
      continue;
    }

    try {
      auto &port = ports[*port_index];
      LinuxBridgeBluetooth state_bt(*controller.backend);
      if (!vds::bridge_flush_pending_state(port, state_bt, trace_flags,
                                           logger)) {
        continue;
      }
      LinuxBridgeBluetooth bt(*controller.backend);
      vds::bridge_retry_mic_open(port, bt, logger);
      vds::bridge_enqueue_waveout(port, trace_flags, logger);
      (void)vds::bridge_flush_audio(port, bt, trace_flags, logger);
    } catch (const std::exception &error) {
      if (!is_bluetooth_error(error)) {
        throw;
      }
      drop_bt_backend(controller, ports[*port_index], error.what(), logger);
      epoll_dirty = true;
    }
  }
}

void reconcile_controller_configs(std::vector<VirtualPort> &ports,
                                  std::vector<ControllerRuntime> &controllers,
                                  const std::string &db_path,
                                  vds::Logger &logger) {
  const std::vector<std::string> devices = vds::discover_vds_devices();
  const bool virtual_port_provider_available = !devices.empty();
  for (auto &controller : controllers) {
    if (!controller_uses_port(controller) ||
        std::binary_search(devices.begin(), devices.end(), controller.device)) {
      continue;
    }
    if (const auto port_index = find_port_index(ports, controller.device)) {
      disconnect_virtual_port(ports[*port_index], logger);
    }
    controller.backend.reset();
    controller.pending_control_fd.reset();
    controller.pending_interrupt_fd.reset();
    controller.virtual_connected = false;
    controller.device.clear();
  }

  sync_virtual_ports(ports, devices, logger);
  if (!virtual_port_provider_available) {
    logger.log(vds::LogScope::Port, vds::LogLevel::Error,
               std::string(kVirtualPortProviderUnavailableReason) +
                   " detail=" + kLinuxVirtualPortProviderUnavailable);
  }
  const vds::ConfigDb db = vds::load_config_db(db_path);
  logger.log(vds::LogScope::Config, vds::LogLevel::Info,
             "loaded controller config count=" +
                 std::to_string(db.controllers.size()));

  std::vector<bool> preserved(controllers.size(), false);
  std::vector<std::string> active_devices;
  std::vector<ControllerRuntime> next_controllers;
  next_controllers.reserve(db.controllers.size());

  for (const auto &config : db.controllers) {
    if (!virtual_port_provider_available) {
      logger.log(vds::LogScope::Config, vds::LogLevel::Warn,
                 "controller binding unavailable address=" + config.address +
                     " reason=" + kVirtualPortProviderUnavailableReason);
    } else if (!controller_has_present_allowed_port(ports, config)) {
      logger.log(vds::LogScope::Config, vds::LogLevel::Warn,
                 "controller binding unavailable address=" + config.address +
                     " reason=no allowed virtual port present ports=[" +
                     vds::format_ports(config.ports) + "]");
    }
    if (virtual_port_provider_available) {
      preempt_default_bluetooth_owner(config.address, logger);
    }
    ControllerRuntime next{
        .config = config,
        .device = {},
        .detected_profile = std::nullopt,
        .backend = std::nullopt,
        .pending_control_fd = {},
        .pending_interrupt_fd = {},
        .virtual_connected = false,
        .last_error = {},
    };

    for (std::size_t old_index = 0; old_index < controllers.size();
         ++old_index) {
      auto &old = controllers[old_index];
      if (preserved[old_index] || old.config.address != config.address ||
          old.config.profile != config.profile) {
        continue;
      }
      const bool old_uses_port = controller_uses_port(old);
      const bool old_device_present =
          !old.device.empty() && find_port_index(ports, old.device);
      const bool old_device_allowed =
          old_device_present &&
          vds::controller_config_allows_path(config, old.device);
      const bool old_device_available =
          old_device_allowed &&
          std::find(active_devices.begin(), active_devices.end(), old.device) ==
              active_devices.end();

      next.detected_profile = old.detected_profile;
      next.last_error = std::move(old.last_error);
      if (old_uses_port && old_device_available) {
        next.device = std::move(old.device);
        next.backend = std::move(old.backend);
        next.pending_control_fd = std::move(old.pending_control_fd);
        next.pending_interrupt_fd = std::move(old.pending_interrupt_fd);
        next.virtual_connected = old.virtual_connected;
        active_devices.push_back(next.device);
      } else if (old_uses_port) {
        if (old.virtual_connected && old_device_present) {
          const auto old_port_index = find_port_index(ports, old.device);
          disconnect_virtual_port(ports[*old_port_index], logger);
        }
        if (old.backend) {
          logger.log(vds::LogScope::Bluetooth, vds::LogLevel::Info,
                     "raw backend removed address=" + old.config.address +
                         " device=" + old.device);
        }
        old.backend.reset();
        old.pending_control_fd.reset();
        old.pending_interrupt_fd.reset();
        old.virtual_connected = false;
        old.device.clear();
      }
      preserved[old_index] = true;
      break;
    }

    logger.log(vds::LogScope::Config, vds::LogLevel::Info,
               "controller registered address=" + config.address +
                   " profile=\"" +
                   vds::controller_profile_name(config.profile) + "\"");
    next_controllers.push_back(std::move(next));
  }

  for (std::size_t i = 0; i < controllers.size(); ++i) {
    if (preserved[i]) {
      continue;
    }
    if (controllers[i].virtual_connected) {
      if (const auto port_index =
              find_port_index(ports, controllers[i].device)) {
        disconnect_virtual_port(ports[*port_index], logger);
      }
    }
    if (controllers[i].backend) {
      logger.log(
          vds::LogScope::Bluetooth, vds::LogLevel::Info,
          "raw backend removed address=" + controllers[i].config.address +
              " device=" + controllers[i].device);
    }
  }

  controllers = std::move(next_controllers);
}

// Reads the ACL link RSSI for a connected controller through the HCI
// device. Returns nothing when the adapter or connection handle is
// unavailable.
std::optional<std::int8_t> read_controller_rssi(const std::string &address) {
  bdaddr_t bdaddr{};
  if (str2ba(address.c_str(), &bdaddr) < 0) {
    return std::nullopt;
  }
  const int dev_id = hci_get_route(&bdaddr);
  if (dev_id < 0) {
    return std::nullopt;
  }
  const int dd = hci_open_dev(dev_id);
  if (dd < 0) {
    return std::nullopt;
  }
  std::optional<std::int8_t> result;
  std::vector<std::uint8_t> buffer(sizeof(hci_conn_info_req) +
                                   sizeof(hci_conn_info));
  auto *request = reinterpret_cast<hci_conn_info_req *>(buffer.data());
  bacpy(&request->bdaddr, &bdaddr);
  request->type = ACL_LINK;
  if (::ioctl(dd, HCIGETCONNINFO, buffer.data()) == 0) {
    std::int8_t rssi = 0;
    if (hci_read_rssi(dd, htobs(request->conn_info->handle), &rssi, 200) ==
        0) {
      result = rssi;
    }
  }
  hci_close_dev(dd);
  return result;
}

// Reads controller RSSI on its own thread. hci_read_rssi waits for the
// adapter's reply (~5 ms), and doing that per control request on the main
// loop delayed audio chunks every time the companion polled.
class RssiPoller {
public:
  RssiPoller() : thread_([this] { run(); }) {}
  ~RssiPoller() {
    {
      std::lock_guard lock(mutex_);
      stop_ = true;
    }
    wake_.notify_all();
    thread_.join();
  }
  RssiPoller(const RssiPoller &) = delete;
  RssiPoller &operator=(const RssiPoller &) = delete;

  void set_addresses(std::vector<std::string> addresses) {
    std::lock_guard lock(mutex_);
    if (addresses != addresses_) {
      addresses_ = std::move(addresses);
      wake_.notify_all();
    }
  }

  std::optional<std::int8_t> rssi(const std::string &address) const {
    std::lock_guard lock(mutex_);
    const auto found = values_.find(address);
    if (found == values_.end()) {
      return std::nullopt;
    }
    return found->second;
  }

private:
  void run() {
    std::unique_lock lock(mutex_);
    while (!stop_) {
      const std::vector<std::string> addresses = addresses_;
      lock.unlock();
      std::map<std::string, std::int8_t> values;
      for (const auto &address : addresses) {
        if (const auto value = read_controller_rssi(address)) {
          values.emplace(address, *value);
        }
      }
      lock.lock();
      values_ = std::move(values);
      wake_.wait_for(lock, std::chrono::seconds(1),
                     [&] { return stop_ || addresses_ != addresses; });
    }
  }

  mutable std::mutex mutex_;
  std::condition_variable wake_;
  std::vector<std::string> addresses_;
  std::map<std::string, std::int8_t> values_;
  bool stop_ = false;
  std::thread thread_;
};

/*
 * Bluetooth mode. In "vds" mode bluetoothd runs with --noplugin=input and vdsd
 * listens on the HID L2CAP PSMs. In "normal" mode vdsd closes its listeners
 * and BlueZ's input plugin is enabled, so other Bluetooth keyboards, mice and
 * controllers work. Switching restarts bluetoothd through the override script.
 *
 * With auto switching, vdsd enters normal mode once no registered controller
 * has been connected for idle_timeout, and returns to vds mode when BlueZ
 * reports a registered controller connected (it then has to reconnect to
 * vdsd after the bluetoothd restart).
 */
constexpr auto kDefaultBluetoothIdleTimeout = std::chrono::seconds(120);
constexpr auto kMinBluetoothIdleTimeout = std::chrono::seconds(10);
constexpr auto kMaxBluetoothIdleTimeout = std::chrono::seconds(3600);
constexpr auto kBluetoothModePoll = std::chrono::seconds(2);
constexpr int kBluetoothModePollMs = 2000;

struct BluetoothModeState {
  std::string script;
  std::string settings_path;
  bool normal_mode = false;
  std::optional<bool> requested_normal_mode;
  bool auto_switch = false;
  std::chrono::seconds idle_timeout = kDefaultBluetoothIdleTimeout;
  Clock::time_point idle_since{};
  Clock::time_point next_bluez_poll{};
  Clock::time_point next_listen_retry{};
  bool listen_error_logged = false;
  std::string last_error;
};

void set_bluetooth_normal_mode(BluetoothModeState &state, bool normal) {
  state.normal_mode = normal;
  g_bluetooth_normal_mode = normal;
}

std::chrono::seconds clamp_bluetooth_idle_timeout(unsigned long seconds) {
  return std::clamp(std::chrono::seconds(seconds), kMinBluetoothIdleTimeout,
                    kMaxBluetoothIdleTimeout);
}

void load_bluetooth_mode_settings(BluetoothModeState &state) {
  std::ifstream file(state.settings_path);
  std::string line;
  while (std::getline(file, line)) {
    const std::size_t separator = line.find('=');
    if (separator == std::string::npos) {
      continue;
    }
    const std::string key = line.substr(0, separator);
    const std::string value = line.substr(separator + 1);
    if (key == "auto") {
      state.auto_switch = value == "1";
    } else if (key == "idle_seconds") {
      try {
        state.idle_timeout = clamp_bluetooth_idle_timeout(std::stoul(value));
      } catch (const std::exception &) {
      }
    }
  }
}

void save_bluetooth_mode_settings(const BluetoothModeState &state,
                                  vds::Logger &logger) {
  const std::string tmp_path = state.settings_path + ".tmp";
  {
    std::ofstream file(tmp_path, std::ios::trunc);
    file << "auto=" << (state.auto_switch ? 1 : 0) << '\n'
         << "idle_seconds=" << state.idle_timeout.count() << '\n';
    if (!file) {
      logger.log(vds::LogScope::Bluetooth, vds::LogLevel::Error,
                 "failed to write " + tmp_path);
      return;
    }
  }
  std::error_code error;
  std::filesystem::rename(tmp_path, state.settings_path, error);
  if (error) {
    logger.log(vds::LogScope::Bluetooth, vds::LogLevel::Error,
               "failed to save " + state.settings_path + ": " +
                   error.message());
  }
}

std::string format_bluetooth_mode_reply(const BluetoothModeState &state,
                                        bool listening, bool ok,
                                        std::string_view error) {
  const bool normal =
      state.requested_normal_mode.value_or(state.normal_mode);
  std::string reply = "{";
  reply += vds::jsonl_bool_field("OK", ok);
  reply += ',';
  reply += vds::jsonl_string_field("error", error);
  reply += ',';
  reply += vds::jsonl_string_field("mode", normal ? "normal" : "vds");
  reply += ',';
  reply += vds::jsonl_bool_field(
      "switching", state.requested_normal_mode.has_value() &&
                       *state.requested_normal_mode != state.normal_mode);
  reply += ',';
  reply += vds::jsonl_bool_field("listening", listening);
  reply += ',';
  reply += vds::jsonl_bool_field("auto", state.auto_switch);
  reply += ",\"idle_seconds\":" + std::to_string(state.idle_timeout.count());
  reply += ',';
  reply += vds::jsonl_string_field("last_error", state.last_error);
  reply += "}\n";
  return reply;
}

std::string handle_bluetooth_mode_request(
    std::span<const vds::JsonlField> fields, BluetoothModeState &state,
    bool listening, vds::Logger &logger) {
  constexpr std::string_view context = "bluetooth request";
  try {
    static constexpr std::string_view expected[] = {
        "command", "op", "mode", "auto", "idle_seconds",
    };
    vds::reject_unknown_jsonl_fields(fields, expected, context);
    const std::string op = vds::require_jsonl_string(fields, "op", context);
    if (op == "get") {
      return format_bluetooth_mode_reply(state, listening, true, "");
    }
    if (op != "set") {
      throw std::runtime_error("bluetooth op must be get or set");
    }

    bool settings_changed = false;
    if (vds::find_jsonl_field(fields, "auto") != nullptr) {
      const std::string value =
          vds::require_jsonl_string(fields, "auto", context);
      if (value != "on" && value != "off") {
        throw std::runtime_error("bluetooth auto must be on or off");
      }
      state.auto_switch = value == "on";
      state.idle_since = {};
      settings_changed = true;
    }
    if (vds::find_jsonl_field(fields, "idle_seconds") != nullptr) {
      const std::string value =
          vds::require_jsonl_string(fields, "idle_seconds", context);
      if (value.empty() ||
          value.find_first_not_of("0123456789") != std::string::npos ||
          value.size() > 6) {
        throw std::runtime_error("bluetooth idle_seconds must be a number");
      }
      state.idle_timeout = clamp_bluetooth_idle_timeout(std::stoul(value));
      settings_changed = true;
    }
    if (vds::find_jsonl_field(fields, "mode") != nullptr) {
      const std::string mode =
          vds::require_jsonl_string(fields, "mode", context);
      if (mode != "normal" && mode != "vds") {
        throw std::runtime_error("bluetooth mode must be normal or vds");
      }
      state.requested_normal_mode = mode == "normal";
    }
    if (settings_changed) {
      save_bluetooth_mode_settings(state, logger);
      logger.log(vds::LogScope::Bluetooth, vds::LogLevel::Info,
                 std::string("auto switching ") +
                     (state.auto_switch ? "on" : "off") + " idle_seconds=" +
                     std::to_string(state.idle_timeout.count()));
    }
    return format_bluetooth_mode_reply(state, listening, true, "");
  } catch (const std::exception &error) {
    return format_bluetooth_mode_reply(state, listening, false, error.what());
  }
}

void enter_normal_bluetooth_mode(BluetoothModeState &state,
                                 std::optional<vds::BtL2capAcceptor> &acceptor,
                                 std::vector<VirtualPort> &ports,
                                 std::vector<ControllerRuntime> &controllers,
                                 vds::Logger &logger) {
  logger.log(vds::LogScope::Bluetooth, vds::LogLevel::Info,
             "switching to normal Bluetooth mode (BlueZ input plugin on)");
  for (auto &controller : controllers) {
    if (const auto port_index = find_port_index(ports, controller.device)) {
      drop_bt_backend(controller, ports[*port_index],
                      "Bluetooth switched to normal mode", logger);
    }
    controller.backend.reset();
    controller.pending_control_fd.reset();
    controller.pending_interrupt_fd.reset();
  }
  acceptor.reset();

  const std::string error =
      vds::run_bluetooth_override_script(state.script, true);
  state.last_error = error;
  if (!error.empty()) {
    logger.log(vds::LogScope::Bluetooth, vds::LogLevel::Error,
               "switch to normal mode failed: " + error);
  }
  set_bluetooth_normal_mode(state,
                            vds::bluez_input_plugin_enabled().value_or(false));
  state.next_bluez_poll = Clock::now() + kBluetoothModePoll;
  state.next_listen_retry = {};
}

void enter_vds_bluetooth_mode(BluetoothModeState &state, vds::Logger &logger) {
  logger.log(vds::LogScope::Bluetooth, vds::LogLevel::Info,
             "switching to vds Bluetooth mode (BlueZ input plugin off)");
  const std::string error =
      vds::run_bluetooth_override_script(state.script, false);
  state.last_error = error;
  if (!error.empty()) {
    logger.log(vds::LogScope::Bluetooth, vds::LogLevel::Error,
               "switch to vds mode failed: " + error);
  }
  set_bluetooth_normal_mode(state,
                            vds::bluez_input_plugin_enabled().value_or(false));
  state.idle_since = Clock::now();
  state.next_listen_retry = {};
  state.listen_error_logged = false;
}

bool bluez_registered_controller_online(const std::string &db_path) {
  const vds::ConfigDb db = vds::load_config_db(db_path);
  for (const auto &target : vds::list_bluez_controller_targets()) {
    if (!target.online) {
      continue;
    }
    for (const auto &config : db.controllers) {
      if (config.address == target.address) {
        return true;
      }
    }
  }
  return false;
}

void tick_bluetooth_mode(BluetoothModeState &state,
                         std::optional<vds::BtL2capAcceptor> &acceptor,
                         std::vector<VirtualPort> &ports,
                         std::vector<ControllerRuntime> &controllers,
                         const std::string &db_path, vds::Logger &logger,
                         bool &epoll_dirty) {
  if (state.requested_normal_mode) {
    const bool normal = *state.requested_normal_mode;
    state.requested_normal_mode.reset();
    if (normal && !state.normal_mode) {
      enter_normal_bluetooth_mode(state, acceptor, ports, controllers, logger);
      epoll_dirty = true;
    } else if (!normal && state.normal_mode) {
      enter_vds_bluetooth_mode(state, logger);
    }
  }

  const auto now = Clock::now();
  if (!state.normal_mode && !acceptor && now >= state.next_listen_retry) {
    try {
      acceptor.emplace();
      state.listen_error_logged = false;
      epoll_dirty = true;
      logger.log(vds::LogScope::Bluetooth, vds::LogLevel::Info,
                 "listening for controller-initiated raw HID channels");
    } catch (const std::exception &error) {
      state.next_listen_retry = now + kBluetoothModePoll;
      if (!state.listen_error_logged) {
        state.listen_error_logged = true;
        logger.log(vds::LogScope::Bluetooth, vds::LogLevel::Warn,
                   std::string("cannot listen for controllers yet: ") +
                       error.what());
      }
    }
  }

  if (!state.auto_switch) {
    state.idle_since = {};
    return;
  }

  if (!state.normal_mode) {
    const bool active =
        std::any_of(controllers.begin(), controllers.end(),
                    [](const ControllerRuntime &controller) {
                      return controller.backend || controller.virtual_connected ||
                             controller.pending_control_fd ||
                             controller.pending_interrupt_fd;
                    });
    if (active) {
      state.idle_since = {};
    } else if (state.idle_since == Clock::time_point{}) {
      state.idle_since = now;
    } else if (now - state.idle_since >= state.idle_timeout) {
      logger.log(vds::LogScope::Bluetooth, vds::LogLevel::Info,
                 "no controller connected for " +
                     std::to_string(state.idle_timeout.count()) +
                     "s; auto switching to normal mode");
      enter_normal_bluetooth_mode(state, acceptor, ports, controllers, logger);
      epoll_dirty = true;
    }
    return;
  }

  if (now < state.next_bluez_poll) {
    return;
  }
  state.next_bluez_poll = now + kBluetoothModePoll;
  try {
    if (bluez_registered_controller_online(db_path)) {
      logger.log(vds::LogScope::Bluetooth, vds::LogLevel::Info,
                 "registered controller connected to BlueZ; auto switching "
                 "to vds mode");
      enter_vds_bluetooth_mode(state, logger);
    }
  } catch (const std::exception &error) {
    logger.log(vds::LogScope::Bluetooth, vds::LogLevel::Warn,
               std::string("BlueZ controller poll failed: ") + error.what());
  }
}

bool bluetooth_mode_needs_poll(const BluetoothModeState &state,
                               const std::optional<vds::BtL2capAcceptor> &acceptor) {
  return state.auto_switch || state.requested_normal_mode.has_value() ||
         (!state.normal_mode && !acceptor);
}

/*
 * `audio` control command: per-port audio delivery stats for the companion
 * Diagnostics view. Window counters cover the time since the previous
 * request and are reset by it.
 */
std::string handle_audio_stats_request(std::span<VirtualPort> ports,
                                       std::span<const ControllerRuntime> controllers) {
  const auto now = Clock::now();
  std::string reply = "{";
  reply += vds::jsonl_bool_field("OK", true);
  reply += ',';
  reply += vds::jsonl_string_field("error", "");
  reply += ",\"ports\":[";
  bool first = true;
  for (auto &port : ports) {
    const bool connected = std::any_of(
        controllers.begin(), controllers.end(), [&](const ControllerRuntime &c) {
          return c.virtual_connected && c.device == port.path;
        });
    if (!connected) {
      continue;
    }
    if (!first) {
      reply += ',';
    }
    first = false;
    reply += vds::bridge_audio_stats_json(port, now);
  }
  reply += "]}\n";
  return reply;
}

// Mic and headset state of the first connected controller, for the companion
// AUDIO_STATUS report.
void publish_companion_audio_status(std::span<const VirtualPort> ports,
                                    std::span<const ControllerRuntime> controllers,
                                    vds::CompanionRuntime &companion) {
  vds::CompanionAudioStatus status;
  for (const auto &controller : controllers) {
    if (!controller.backend || !controller.virtual_connected) {
      continue;
    }
    const auto port = std::find_if(ports.begin(), ports.end(),
                                   [&](const VirtualPort &candidate) {
                                     return candidate.path == controller.device;
                                   });
    if (port == ports.end()) {
      continue;
    }
    status = vds::bridge_audio_status(*port);
    break;
  }
  companion.audio_status = status;
}

std::string build_control_reply(std::string request,
                                std::span<VirtualPort> ports,
                                std::span<ControllerRuntime> controllers,
                                const std::string &db_path,
                                std::uint32_t &trace_flags,
                                bool &reload_requested,
                                vds::CompanionRuntime &companion,
                                BluetoothModeState &bt_mode, bool bt_listening,
                                const std::string &log_path,
                                RssiPoller &rssi_poller,
                                vds::Logger &logger) {
  publish_companion_audio_status(ports, controllers, companion);
  std::vector<std::string> connected;
  for (const auto &controller : controllers) {
    if (controller.virtual_connected) {
      connected.push_back(controller.config.address);
    }
  }
  rssi_poller.set_addresses(std::move(connected));

  const std::string command = trim_command(std::move(request));
  std::vector<vds::JsonlField> fields;
  const std::optional<std::string> local_command =
      vds::parse_control_command_name(command, fields);
  if (local_command == "audio") {
    return handle_audio_stats_request(ports, controllers);
  }
  if (local_command == "bluetooth") {
    return handle_bluetooth_mode_request(fields, bt_mode, bt_listening, logger);
  }
  if (local_command == "log") {
    return vds::handle_daemon_log_request(fields, log_path, trace_flags, logger);
  }
  if (local_command == "version") {
    return vds::format_control_version_reply(vds::kVersion);
  }
  std::vector<vds::VdsdControlControllerStatus> controller_statuses;
  controller_statuses.reserve(controllers.size());
  for (const auto &controller : controllers) {
    std::uint8_t battery_status = 0xff;
    if (controller.virtual_connected) {
      for (const auto &port : ports) {
        if (port.path == controller.device) {
          battery_status = port.battery_status;
          break;
        }
      }
    }
    std::optional<std::int8_t> rssi;
    if (controller.virtual_connected) {
      rssi = rssi_poller.rssi(controller.config.address);
    }
    controller_statuses.push_back(vds::VdsdControlControllerStatus{
        .address = controller.config.address,
        .connected = controller.virtual_connected,
        .path = controller.virtual_connected ? controller.device : "",
        .battery_status = battery_status,
        .rssi_valid = rssi.has_value(),
        .rssi = rssi.value_or(0),
    });
  }

  std::vector<vds::VdsdControlPortCandidate> port_candidates;
  port_candidates.reserve(ports.size());
  for (const auto &port : ports) {
    const auto port_index = vds::port_index_from_path(port.path);
    if (!port_index) {
      continue;
    }

    port_candidates.push_back(vds::VdsdControlPortCandidate{
        .port = *port_index,
        .path = port.path,
    });
  }

  std::vector<vds::VdsdControlPortBinding> port_bindings;
  port_bindings.reserve(controllers.size());
  for (const auto &controller : controllers) {
    if (!controller_uses_port(controller)) {
      continue;
    }
    const auto port = vds::port_index_from_path(controller.device);
    if (!port) {
      continue;
    }
    port_bindings.push_back(vds::VdsdControlPortBinding{
        .port = *port,
        .address = controller.config.address,
        .device_address = {},
    });
  }

  const std::vector<vds::VdsdControlPortStatus> port_statuses =
      vds::build_vdsd_control_port_statuses(port_candidates, port_bindings);

  return vds::handle_vdsd_control_command(
      command, db_path, controller_statuses, port_statuses,
      [] { return vds::list_bluez_controller_targets(); }, trace_flags,
      reload_requested, companion, logger);
}

// Grabs every DualSense touchpad evdev node (EVIOCGRAB) so the compositor
// stops receiving pointer events from it; raw HID touch used by games is
// unaffected. Returns the grabbed fds.
std::vector<vds::UniqueFd> grab_dualsense_touchpads(vds::Logger &logger) {
  std::vector<vds::UniqueFd> grabs;
  DIR *dir = ::opendir("/dev/input");
  if (dir == nullptr) {
    return grabs;
  }
  while (const dirent *entry = ::readdir(dir)) {
    if (std::strncmp(entry->d_name, "event", 5) != 0) {
      continue;
    }
    const std::string node = std::string("/dev/input/") + entry->d_name;
    vds::UniqueFd fd(::open(node.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC));
    if (!fd) {
      continue;
    }
    char name[256] = {};
    if (::ioctl(fd.get(), EVIOCGNAME(sizeof(name) - 1), name) < 0) {
      continue;
    }
    if (std::strstr(name, "DualSense") == nullptr ||
        std::strstr(name, "Touchpad") == nullptr) {
      continue;
    }
    if (::ioctl(fd.get(), EVIOCGRAB, 1) < 0) {
      logger.log(vds::LogScope::Companion, vds::LogLevel::Warn,
                 node + " touchpad grab failed: " +
                     std::string(std::strerror(errno)));
      continue;
    }
    logger.log(vds::LogScope::Companion, vds::LogLevel::Info,
               node + " touchpad pointer grabbed (" + name + ")");
    grabs.push_back(std::move(fd));
  }
  ::closedir(dir);
  return grabs;
}

// Translates companion settings and momentary effects into per-port output
// overrides, then forwards the resulting BT state to connected controllers.
const char *host_persona_name(vds::HostPersona persona) {
  switch (persona) {
  case vds::HostPersona::Ds4:
    return "ds4";
  case vds::HostPersona::Xbox360:
    return "xbox360";
  case vds::HostPersona::DualSense:
    break;
  }
  return "dualsense";
}

// Re-enumerates the virtual USB controller with the persona's profile while
// keeping the Bluetooth link to the DualSense up.
void switch_port_persona(VirtualPort &port, ControllerRuntime &controller,
                         vds::HostPersona persona, vds::Logger &logger) {
  const std::uint32_t dualsense_profile =
      controller.detected_profile.value_or(VDS_PROFILE_DS5);
  try {
    ioctl_noarg(port.fd.get(), VDS_IOC_DISCONNECT, "VDS_IOC_DISCONNECT");
    ioctl_set_profile(port.fd.get(),
                      vds::usb_profile_for_persona(persona, dualsense_profile));
    port.persona = persona;
    port.persona_state = {};
    port.pending_audio_chunks.clear();
    port.audio_primed = false;
    port.extractor = vds::PcmAudioExtractor{vds::kBridgeSpeakerInputFrames};
    ioctl_noarg(port.fd.get(), VDS_IOC_CONNECT, "VDS_IOC_CONNECT");
    logger.log(vds::LogScope::Companion, vds::LogLevel::Info,
               port.path + " host persona " + host_persona_name(persona));
  } catch (const std::exception &error) {
    logger.log(vds::LogScope::Companion, vds::LogLevel::Error,
               port.path + " host persona switch failed: " + error.what());
  }
}

std::string process_name(pid_t pid) {
  std::ifstream comm("/proc/" + std::to_string(pid) + "/comm");
  std::string name;
  std::getline(comm, name);
  return name.empty() ? "?" : name;
}

std::optional<std::size_t> port_index_from_path(const std::string &path) {
  constexpr std::string_view prefix = "/dev/vds";
  if (path.rfind(prefix, 0) != 0 || path.size() == prefix.size()) {
    return std::nullopt;
  }
  try {
    return static_cast<std::size_t>(std::stoul(path.substr(prefix.size())));
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

void clear_host_watch(VirtualPort &port, int inotify_fd) {
  if (port.hidraw_watch >= 0 && inotify_fd >= 0) {
    ::inotify_rm_watch(inotify_fd, port.hidraw_watch);
  }
  port.hidraw_watch = -1;
  port.hidraw_node.clear();
  port.host_baseline.clear();
  port.host_sessions.clear();
}

// Attaches the hidraw watch once a bridged controller's node exists; drops
// it when the controller goes away.
void refresh_host_watches(std::vector<VirtualPort> &ports,
                          std::vector<ControllerRuntime> &controllers,
                          int inotify_fd, vds::Logger &logger) {
  for (auto &port : ports) {
    const ControllerRuntime *controller = controller_for_port(controllers, port.path);
    const bool connected = controller != nullptr && controller->virtual_connected;
    if (!connected) {
      if (port.hidraw_watch >= 0) {
        clear_host_watch(port, inotify_fd);
      }
      continue;
    }
    if (port.hidraw_watch >= 0) {
      continue;
    }
    const auto index = port_index_from_path(port.path);
    const auto node = index ? vds::find_port_hidraw(*index) : std::nullopt;
    if (!node) {
      continue;
    }
    const int watch = ::inotify_add_watch(inotify_fd, node->c_str(),
                                          IN_OPEN | IN_CLOSE);
    if (watch < 0) {
      continue;
    }
    port.hidraw_node = *node;
    port.hidraw_watch = watch;
    port.host_baseline = vds::node_openers(*node, ::getpid());
    port.host_sessions.clear();
    logger.log(vds::LogScope::Host, vds::LogLevel::Info,
               port.path + " watching " + *node + " for native game sessions");
  }
}

// Re-checks who has the port's hidraw node open after an open/close.
void update_host_session(VirtualPort &port, ControllerRuntime *controller,
                         std::uint32_t trace_flags, vds::Logger &logger) {
  const std::set<pid_t> openers = vds::node_openers(port.hidraw_node, ::getpid());
  std::erase_if(port.host_baseline,
                [&](pid_t pid) { return !openers.contains(pid); });

  // Programs that closed it: the latest one restores the state from before it
  // opened; an earlier one hands its (older) snapshot to the next in line.
  bool restored = false;
  for (std::size_t i = port.host_sessions.size(); i-- > 0;) {
    if (openers.contains(port.host_sessions[i].pid)) {
      continue;
    }
    const auto session = port.host_sessions[i];
    port.host_sessions.erase(port.host_sessions.begin() +
                             static_cast<std::ptrdiff_t>(i));
    logger.log(vds::LogScope::Host, vds::LogLevel::Info,
               port.path + " pid " + std::to_string(session.pid) +
                   " closed the controller");
    if (i == port.host_sessions.size()) {
      port.output_state.restore_host_state(session.before);
      restored = true;
    } else {
      port.host_sessions[i].before = session.before;
    }
  }

  for (const pid_t pid : openers) {
    const bool known =
        port.host_baseline.contains(pid) ||
        std::any_of(port.host_sessions.begin(), port.host_sessions.end(),
                    [&](const VirtualPort::HostSession &session) {
                      return session.pid == pid;
                    });
    if (known) {
      continue;
    }
    port.host_sessions.push_back({pid, port.output_state.host_state()});
    logger.log(vds::LogScope::Host, vds::LogLevel::Info,
               port.path + " " + process_name(pid) + " (pid " +
                   std::to_string(pid) + ") opened the controller");
  }

  if (restored) {
    logger.log(vds::LogScope::Host, vds::LogLevel::Info,
               port.path + " restored lights and output state after a native "
                           "game closed the controller");
    if (controller != nullptr && controller->backend) {
      try {
        forward_bt_state_if_changed(port, *controller->backend, trace_flags,
                                    logger, "native game closed");
      } catch (const std::exception &error) {
        logger.log(vds::LogScope::Host, vds::LogLevel::Warn,
                   port.path + " restore send failed: " + error.what());
      }
    }
  }
}

/*
 * "Turn Off Controller": close vdsd's L2CAP channels, then drop the Bluetooth
 * link through BlueZ. A DualSense powers itself off when the host
 * disconnects it; pressing PS turns it back on and it reconnects.
 */
void power_off_controllers(std::vector<VirtualPort> &ports,
                           std::vector<ControllerRuntime> &controllers,
                           vds::Logger &logger) {
  for (auto &controller : controllers) {
    if (!controller.backend && !controller.virtual_connected) {
      continue;
    }
    const std::string address = controller.config.address;
    auto port = std::find_if(ports.begin(), ports.end(), [&](const VirtualPort &p) {
      return p.path == controller.device;
    });
    if (port != ports.end()) {
      drop_bt_backend(controller, *port, "turned off from the companion app",
                      logger);
    }
    try {
      if (!vds::disconnect_bluez_device(address)) {
        logger.log(vds::LogScope::Bluetooth, vds::LogLevel::Warn,
                   "turn off: BlueZ device not found address=" + address);
      }
    } catch (const std::exception &error) {
      logger.log(vds::LogScope::Bluetooth, vds::LogLevel::Warn,
                 "turn off: BlueZ disconnect failed address=" + address +
                     " error=" + error.what());
    }
    logger.log(vds::LogScope::Bluetooth, vds::LogLevel::Info,
               "controller turned off address=" + address);
  }
}

void apply_companion_state(std::vector<VirtualPort> &ports,
                           std::vector<ControllerRuntime> &controllers,
                           vds::CompanionRuntime &companion,
                           std::uint32_t trace_flags, vds::Logger &logger) {
  (void)vds::expire_companion_actuation(companion,
                                        std::chrono::steady_clock::now());

  if (companion.actuation.power_off_pending) {
    companion.actuation.power_off_pending = false;
    power_off_controllers(ports, controllers, logger);
  }

  const vds::CompanionSettings &settings = companion.settings;
  const vds::DsCompanionOverrides overrides =
      vds::bridge_companion_overrides(companion);
  const vds::HostPersona host_persona = vds::bridge_settings().host_persona;
  for (auto &port : ports) {
    ControllerRuntime *controller =
        controller_for_port(controllers, port.path);
    if (controller != nullptr && controller->virtual_connected &&
        port.persona != host_persona) {
      switch_port_persona(port, *controller, host_persona, logger);
    }
  }

  // Touchpad pointer suppression lives for the daemon lifetime alongside
  // the companion runtime state.
  static std::vector<vds::UniqueFd> touchpad_grabs;
  bool any_connected = false;
  for (const auto &controller : controllers) {
    any_connected = any_connected || controller.virtual_connected;
  }
  if (!settings.touchpad_pointer_enabled && any_connected) {
    if (touchpad_grabs.empty()) {
      touchpad_grabs = grab_dualsense_touchpads(logger);
    }
  } else if (!touchpad_grabs.empty()) {
    touchpad_grabs.clear();
    logger.log(vds::LogScope::Companion, vds::LogLevel::Info,
               "touchpad pointer released");
  }

  for (auto &port : ports) {
    ControllerRuntime *controller =
        controller_for_port(controllers, port.path);
    if (controller == nullptr || !controller->backend ||
        !controller->virtual_connected) {
      continue;
    }
    LinuxBridgeBluetooth bt(*controller->backend);
    vds::bridge_apply_companion(port, settings, overrides, bt, trace_flags,
                                logger);
  }
}

std::uint64_t encode_event(EventType type, std::size_t index) {
  return (static_cast<std::uint64_t>(type) << 32) |
         static_cast<std::uint32_t>(index);
}

EventSource decode_event(std::uint64_t data) {
  return EventSource{
      .type = static_cast<EventType>(data >> 32),
      .index = static_cast<std::uint32_t>(data),
  };
}

void add_epoll_fd(int epoll_fd, int fd, EventType type, std::size_t index,
                  std::uint32_t events = EPOLLIN) {
  epoll_event event{};
  event.events = events | EPOLLERR | EPOLLHUP;
  event.data.u64 = encode_event(type, index);
  if (::epoll_ctl(epoll_fd, EPOLL_CTL_ADD, fd, &event) < 0) {
    throw std::runtime_error("epoll add failed: " +
                             std::string(std::strerror(errno)));
  }
}

std::uint32_t control_client_events(const ControlClient &client) {
  return client.replying ? EPOLLOUT : EPOLLIN;
}

void accept_control_client(int control_fd, int epoll_fd,
                           std::vector<ControlClient> &clients,
                           std::uint32_t &next_client_id, vds::Logger &logger) {
  vds::UniqueFd client_fd(
      ::accept4(control_fd, nullptr, nullptr, SOCK_NONBLOCK | SOCK_CLOEXEC));
  if (!client_fd) {
    if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR ||
        errno == ECONNABORTED) {
      return;
    }
    throw std::runtime_error("accept control client failed: " +
                             std::string(std::strerror(errno)));
  }
  if (clients.size() >= kMaxControlClients) {
    logger.log(vds::LogScope::Control, vds::LogLevel::Warn,
               "too many pending control clients; dropping new connection");
    return;
  }
  ControlClient client;
  client.id = next_client_id++;
  client.fd = std::move(client_fd);
  client.deadline = Clock::now() + kControlClientTimeout;
  add_epoll_fd(epoll_fd, client.fd.get(), EventType::ControlClient, client.id);
  clients.push_back(std::move(client));
}

// Reads what is available. Returns true once the request is complete (a
// newline, EOF or the size cap); a client with nothing to say is dropped by
// clearing its fd.
bool read_control_client(ControlClient &client) {
  std::array<char, 512> buffer{};
  while (client.request.find('\n') == std::string::npos &&
         client.request.size() < kMaxControlRequest) {
    const ssize_t got =
        ::read(client.fd.get(), buffer.data(), buffer.size());
    if (got < 0) {
      if (errno == EINTR) {
        continue;
      }
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        return false;
      }
      client.fd.reset();
      return false;
    }
    if (got == 0) {
      if (client.request.empty()) {
        client.fd.reset();
        return false;
      }
      return true;
    }
    client.request.append(buffer.data(), static_cast<std::size_t>(got));
  }
  return true;
}

// Writes as much of the reply as the socket takes. Returns true when the
// client is finished with (reply sent or the socket failed).
bool write_control_client(ControlClient &client, std::uint32_t trace_flags,
                          vds::Logger &logger) {
  while (client.reply_sent < client.reply.size()) {
    const ssize_t written =
        ::write(client.fd.get(), client.reply.data() + client.reply_sent,
                client.reply.size() - client.reply_sent);
    if (written < 0) {
      if (errno == EINTR) {
        continue;
      }
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        return false;
      }
      if (trace_flags != 0) {
        logger.log(vds::LogScope::Control, vds::LogLevel::Error,
                   std::string("reply failed: ") + std::strerror(errno));
      }
      return true;
    }
    client.reply_sent += static_cast<std::size_t>(written);
  }
  return true;
}

void expire_control_clients(std::vector<ControlClient> &clients,
                            vds::Logger &logger) {
  const auto now = Clock::now();
  std::erase_if(clients, [&](const ControlClient &client) {
    if (client.fd && now < client.deadline) {
      return false;
    }
    if (client.fd) {
      logger.log(vds::LogScope::Control, vds::LogLevel::Warn,
                 client.replying ? "control client stopped reading the reply"
                                 : "control client sent no request in time");
    }
    return true;
  });
}

int control_client_timeout_ms(std::span<const ControlClient> clients) {
  int timeout_ms = -1;
  const auto now = Clock::now();
  for (const auto &client : clients) {
    const auto until = std::chrono::duration_cast<std::chrono::milliseconds>(
                           client.deadline - now)
                           .count();
    // Round up so the wake-up lands after the deadline, not just before it.
    const int ms = static_cast<int>(std::max<long long>(until + 1, 0));
    timeout_ms = timeout_ms < 0 ? ms : std::min(timeout_ms, ms);
  }
  return timeout_ms;
}

vds::UniqueFd rebuild_epoll(int control_fd,
                            const std::optional<vds::BtL2capAcceptor> &bt_acceptor,
                            const vds::VdsDeviceMonitor &vds_monitor,
                            std::span<VirtualPort> ports,
                            std::span<ControllerRuntime> controllers,
                            std::span<const ControlClient> control_clients,
                            int sleep_fd, int host_watch_fd) {
  vds::UniqueFd epoll_fd(::epoll_create1(EPOLL_CLOEXEC));
  if (!epoll_fd) {
    throw std::runtime_error("epoll_create1 failed: " +
                             std::string(std::strerror(errno)));
  }

  add_epoll_fd(epoll_fd.get(), control_fd, EventType::Control, 0);
  for (const auto &client : control_clients) {
    add_epoll_fd(epoll_fd.get(), client.fd.get(), EventType::ControlClient,
                 client.id, control_client_events(client));
  }
  add_epoll_fd(epoll_fd.get(), vds_monitor.fd(), EventType::Udev, 0);
  if (sleep_fd >= 0) {
    add_epoll_fd(epoll_fd.get(), sleep_fd, EventType::Sleep, 0);
  }
  if (host_watch_fd >= 0) {
    add_epoll_fd(epoll_fd.get(), host_watch_fd, EventType::HostWatch, 0);
  }
  if (bt_acceptor) {
    add_epoll_fd(epoll_fd.get(), bt_acceptor->control_listener_fd(),
                 EventType::BtAcceptControl, 0);
    add_epoll_fd(epoll_fd.get(), bt_acceptor->interrupt_listener_fd(),
                 EventType::BtAcceptInterrupt, 0);
  }
  for (std::size_t i = 0; i < ports.size(); ++i) {
    add_epoll_fd(epoll_fd.get(), ports[i].fd.get(), EventType::Port, i);
  }
  for (std::size_t i = 0; i < controllers.size(); ++i) {
    if (!controllers[i].backend) {
      continue;
    }
    add_epoll_fd(epoll_fd.get(), controllers[i].backend->control_fd(),
                 EventType::BtControl, i);
    add_epoll_fd(epoll_fd.get(), controllers[i].backend->interrupt_fd(),
                 EventType::BtInterrupt, i);
  }
  return epoll_fd;
}

void disconnect_all(std::vector<VirtualPort> &ports,
                    std::vector<ControllerRuntime> &controllers,
                    vds::Logger &logger) {
  for (auto &controller : controllers) {
    if (!controller.virtual_connected) {
      continue;
    }
    if (const auto port_index = find_port_index(ports, controller.device)) {
      disconnect_virtual_port(ports[*port_index], logger);
    }
    controller.virtual_connected = false;
  }
  controllers.clear();
}

} // namespace

int main(int argc, char **argv) {
  try {
    ::signal(SIGPIPE, SIG_IGN);
    install_signal_handler(SIGINT);
    install_signal_handler(SIGTERM);
    install_signal_handler(SIGHUP);

    const Options options = parse_platform_args(argc, argv);
    vds::Logger logger(options.log_path);
    logger.log(vds::LogScope::Daemon, vds::LogLevel::Info,
               "started socket=" + options.socket + " log=" + options.log_path +
                   " db=" + options.db_path);
    vds::BridgeSettings &bridge = vds::bridge_settings();
    bridge.speaker_boost_db = options.speaker_boost_db;
    bridge.mic_forced = options.mic_enabled;
    bridge.mic_allowed = bridge.mic_forced;
    if (bridge.mic_forced) {
      logger.log(vds::LogScope::InputAudio, vds::LogLevel::Info,
                 "controller microphone enabled (--mic)");
    }
    logger.log(vds::LogScope::Output, vds::LogLevel::Info,
               "speaker boost " + std::to_string(bridge.speaker_boost_db) +
                   " dB");
    if (vds::discover_vds_devices().empty()) {
      logger.log(vds::LogScope::Port, vds::LogLevel::Error,
                 std::string(kVirtualPortProviderUnavailableReason) +
                     " detail=" + kLinuxVirtualPortProviderUnavailable);
      throw std::runtime_error(kLinuxVirtualPortProviderUnavailable);
    }

    vds::UniqueFd control_fd(open_control_socket(options.socket));
    std::optional<vds::BtL2capAcceptor> bt_acceptor;
    vds::VdsDeviceMonitor vds_monitor;
    BluetoothModeState bt_mode;
    // PC Sleep Disconnect: logind PrepareForSleep + a delay inhibitor held
    // only while the companion setting is on.
    std::optional<vds::SleepWatcher> sleep_watcher;
    bool system_sleeping = false;
    vds::UniqueFd host_watch_fd(::inotify_init1(IN_NONBLOCK | IN_CLOEXEC));
    auto next_host_watch_refresh = Clock::now();
    try {
      sleep_watcher.emplace();
    } catch (const std::exception &error) {
      logger.log(vds::LogScope::Power, vds::LogLevel::Warn,
                 std::string("PC sleep disconnect unavailable: ") + error.what());
    }
    bt_mode.script = options.bluetooth_override_script;
    bt_mode.settings_path =
        (std::filesystem::path(options.db_path).parent_path() /
         "bluetooth-mode")
            .string();
    load_bluetooth_mode_settings(bt_mode);
    set_bluetooth_normal_mode(bt_mode,
                              vds::bluez_input_plugin_enabled().value_or(false));
    logger.log(vds::LogScope::Bluetooth, vds::LogLevel::Info,
               std::string("Bluetooth mode=") +
                   (bt_mode.normal_mode ? "normal" : "vds") +
                   " auto=" + (bt_mode.auto_switch ? "on" : "off") +
                   " idle_seconds=" +
                   std::to_string(bt_mode.idle_timeout.count()));
    SocketPathGuard control_socket_path(options.socket);
    std::vector<VirtualPort> ports;
    std::vector<ControllerRuntime> controllers;
    vds::UniqueFd epoll_fd;
    std::uint32_t trace_flags = 0;
    bool reload_requested = true;
    bool epoll_dirty = true;
    vds::CompanionRuntime companion;
    companion.settings.speaker_boost_db =
        static_cast<std::uint8_t>(std::lround(options.speaker_boost_db));
    std::uint64_t applied_companion_version = 0;
    std::vector<ControlClient> control_clients;
    RssiPoller rssi_poller;
    std::uint32_t next_control_client_id = 0;

    while (g_stop_requested == 0) {
      if (g_log_reopen_requested != 0) {
        g_log_reopen_requested = 0;
        try {
          logger.reopen();
          logger.log(vds::LogScope::Daemon, vds::LogLevel::Info,
                     "reopened log file");
        } catch (const std::exception &error) {
          logger.log(vds::LogScope::Daemon, vds::LogLevel::Error,
                     std::string("log reopen failed: ") + error.what());
        }
      }

      if (reload_requested) {
        try {
          reconcile_controller_configs(ports, controllers, options.db_path,
                                       logger);
          epoll_dirty = true;
        } catch (const std::exception &error) {
          logger.log(vds::LogScope::Config, vds::LogLevel::Error,
                     std::string("reload failed: ") + error.what());
        }
        reload_requested = false;
      }

      tick_bluetooth_mode(bt_mode, bt_acceptor, ports, controllers,
                          options.db_path, logger, epoll_dirty);

      if (epoll_dirty || !epoll_fd) {
        epoll_fd = rebuild_epoll(control_fd.get(), bt_acceptor, vds_monitor,
                                 ports, controllers, control_clients,
                                 sleep_watcher ? sleep_watcher->fd() : -1,
                                 host_watch_fd.get());
        epoll_dirty = false;
      }

      if (host_watch_fd && Clock::now() >= next_host_watch_refresh) {
        refresh_host_watches(ports, controllers, host_watch_fd.get(), logger);
        next_host_watch_refresh = Clock::now() + std::chrono::seconds(2);
      }
      expire_control_clients(control_clients, logger);
      std::array<epoll_event, 64> events{};
      int timeout_ms = next_wakeup_timeout_ms(ports, controllers);
      if (const int client_ms = control_client_timeout_ms(control_clients);
          client_ms >= 0) {
        timeout_ms = timeout_ms < 0 ? client_ms : std::min(timeout_ms, client_ms);
      }
      if (host_watch_fd) {
        timeout_ms = timeout_ms < 0 ? 2000 : std::min(timeout_ms, 2000);
      }
      if (bluetooth_mode_needs_poll(bt_mode, bt_acceptor)) {
        timeout_ms = timeout_ms < 0
                         ? kBluetoothModePollMs
                         : std::min(timeout_ms, kBluetoothModePollMs);
      }
      if (const auto deadline =
              vds::next_companion_actuation_deadline(companion)) {
        const auto until = std::chrono::duration_cast<std::chrono::milliseconds>(
                               *deadline - std::chrono::steady_clock::now())
                               .count();
        const int deadline_ms = static_cast<int>(std::max<long long>(until, 0));
        if (timeout_ms < 0 || deadline_ms < timeout_ms) {
          timeout_ms = deadline_ms;
        }
      }
      // Hold logind's sleep delay lock only while PC Sleep Disconnect is on
      // (and not mid-suspend), so sleep is never delayed otherwise.
      if (sleep_watcher) {
        const bool want = companion.settings.usb_suspend_disconnect_enabled &&
                          !system_sleeping;
        if (want && !sleep_watcher->inhibitor_held()) {
          try {
            sleep_watcher->acquire_inhibitor();
          } catch (const std::exception &error) {
            logger.log(vds::LogScope::Power, vds::LogLevel::Warn,
                       std::string("sleep inhibitor unavailable: ") +
                           error.what());
            sleep_watcher.reset();
            epoll_dirty = true;
          }
        } else if (!want && sleep_watcher->inhibitor_held()) {
          sleep_watcher->release_inhibitor();
        }
      }
      const int ready =
          ::epoll_wait(epoll_fd.get(), events.data(),
                       static_cast<int>(events.size()), timeout_ms);
      if (ready < 0) {
        if (errno == EINTR) {
          continue;
        }
        throw std::runtime_error("epoll_wait failed: " +
                                 std::string(std::strerror(errno)));
      }
      const auto companion_deadline =
          vds::next_companion_actuation_deadline(companion);
      const bool companion_due =
          companion.actuation.version != applied_companion_version ||
          (companion_deadline &&
           std::chrono::steady_clock::now() >= *companion_deadline);
      if (companion_due) {
        apply_companion_state(ports, controllers, companion, trace_flags,
                              logger);
        applied_companion_version = companion.actuation.version;
      }

      if (ready == 0) {
        flush_pending_outputs(ports, controllers, trace_flags, logger,
                              epoll_dirty);
        continue;
      }

      for (int i = 0; i < ready; ++i) {
        const EventSource source = decode_event(events[i].data.u64);
        const std::uint32_t revents = events[i].events;

        if (source.type == EventType::Control) {
          if ((revents & EPOLLIN) != 0) {
            accept_control_client(control_fd.get(), epoll_fd.get(),
                                  control_clients, next_control_client_id,
                                  logger);
          }
          if ((revents & (EPOLLERR | EPOLLHUP)) != 0) {
            throw std::runtime_error("control socket epoll error");
          }
          continue;
        }

        if (source.type == EventType::ControlClient) {
          const auto client = std::find_if(
              control_clients.begin(), control_clients.end(),
              [&](const ControlClient &candidate) {
                return candidate.id == source.index && candidate.fd;
              });
          if (client == control_clients.end()) {
            continue;
          }
          bool done = false;
          if (!client->replying) {
            if (read_control_client(*client)) {
              client->reply = build_control_reply(
                  std::move(client->request), ports, controllers,
                  options.db_path, trace_flags, reload_requested, companion,
                  bt_mode, bt_acceptor.has_value(), options.log_path,
                  rssi_poller, logger);
              client->replying = true;
              client->deadline = Clock::now() + kControlClientTimeout;
              done = write_control_client(*client, trace_flags, logger);
              if (!done) {
                epoll_event event{};
                event.events = EPOLLOUT | EPOLLERR | EPOLLHUP;
                event.data.u64 =
                    encode_event(EventType::ControlClient, client->id);
                ::epoll_ctl(epoll_fd.get(), EPOLL_CTL_MOD, client->fd.get(),
                            &event);
              }
            } else {
              done = !client->fd;
            }
          } else {
            done = write_control_client(*client, trace_flags, logger);
          }
          if (done) {
            control_clients.erase(client);
          }
          continue;
        }

        if (source.type == EventType::BtAcceptControl ||
            source.type == EventType::BtAcceptInterrupt) {
          if ((revents & EPOLLIN) != 0 && bt_acceptor) {
            handle_bt_accept(ports, controllers, *bt_acceptor,
                             source.type == EventType::BtAcceptControl, logger,
                             epoll_dirty);
          }
          if ((revents & (EPOLLERR | EPOLLHUP)) != 0) {
            throw std::runtime_error("Bluetooth listener epoll error");
          }
          continue;
        }

        if (source.type == EventType::HostWatch) {
          std::array<char, 4096> buffer{};
          std::set<int> touched;
          for (;;) {
            const ssize_t length =
                ::read(host_watch_fd.get(), buffer.data(), buffer.size());
            if (length <= 0) {
              break;
            }
            for (ssize_t offset = 0; offset < length;) {
              const auto *event =
                  reinterpret_cast<const inotify_event *>(buffer.data() + offset);
              touched.insert(event->wd);
              if ((event->mask & IN_IGNORED) != 0) {
                for (auto &port : ports) {
                  if (port.hidraw_watch == event->wd) {
                    port.hidraw_watch = -1; // node removed; refresh re-attaches
                    port.hidraw_node.clear();
                  }
                }
              }
              offset += static_cast<ssize_t>(sizeof(inotify_event) + event->len);
            }
          }
          for (auto &port : ports) {
            if (port.hidraw_watch >= 0 && touched.contains(port.hidraw_watch)) {
              update_host_session(port, controller_for_port(controllers, port.path),
                                  trace_flags, logger);
            }
          }
          continue;
        }

        if (source.type == EventType::Sleep) {
          if (sleep_watcher) {
            for (const bool going_to_sleep : sleep_watcher->drain()) {
              system_sleeping = going_to_sleep;
              if (!going_to_sleep) {
                logger.log(vds::LogScope::Power, vds::LogLevel::Info, "system resumed");
                continue;
              }
              if (companion.settings.usb_suspend_disconnect_enabled) {
                logger.log(vds::LogScope::Power, vds::LogLevel::Info,
                           "system going to sleep; turning controller off");
                power_off_controllers(ports, controllers, logger);
                epoll_dirty = true;
              }
              sleep_watcher->release_inhibitor();
            }
          }
          continue;
        }

        if (source.type == EventType::Udev) {
          if ((revents & EPOLLIN) != 0 && vds_monitor.drain()) {
            logger.log(vds::LogScope::Port, vds::LogLevel::Info,
                       "virtual endpoint udev event; reloading");
            reload_requested = true;
            epoll_dirty = true;
          }
          if ((revents & (EPOLLERR | EPOLLHUP)) != 0) {
            throw std::runtime_error("udev monitor epoll error");
          }
          continue;
        }

        if (source.type == EventType::Port) {
          if (source.index >= ports.size()) {
            continue;
          }
          auto &port = ports[source.index];
          ControllerRuntime *controller =
              controller_for_port(controllers, port.path);
          vds::BtL2capBackend *backend = controller && controller->backend
                                             ? &*controller->backend
                                             : nullptr;
          if ((revents & EPOLLIN) != 0) {
            for (int frame = 0; frame < kMaxPortFramesPerWake; ++frame) {
              try {
                if (!handle_vds_frame(port, backend, trace_flags, logger)) {
                  break;
                }
              } catch (const std::exception &error) {
                if (!is_bluetooth_error(error) || !controller) {
                  throw;
                }
                drop_bt_backend(*controller, port, error.what(), logger);
                epoll_dirty = true;
                break;
              }
            }
          }
          if ((revents & (EPOLLERR | EPOLLHUP)) != 0) {
            logger.log(vds::LogScope::Port, vds::LogLevel::Error,
                       port.path + " epoll error; reloading ports");
            reload_requested = true;
            epoll_dirty = true;
          }
          continue;
        }

        if (source.index >= controllers.size() ||
            !controllers[source.index].backend) {
          continue;
        }
        auto &controller = controllers[source.index];
        const auto port_index = find_port_index(ports, controller.device);
        if (!port_index) {
          controller.backend.reset();
          controller.virtual_connected = false;
          epoll_dirty = true;
          continue;
        }
        auto &port = ports[*port_index];

        if (source.type == EventType::BtControl) {
          if ((revents & EPOLLIN) != 0) {
            for (int packet = 0; packet < kMaxBtPacketsPerWake; ++packet) {
              try {
                if (!handle_bt_control(port, *controller.backend, trace_flags,
                                       logger)) {
                  break;
                }
              } catch (const std::exception &error) {
                if (!is_bluetooth_error(error)) {
                  throw;
                }
                drop_bt_backend(controller, port, error.what(), logger);
                epoll_dirty = true;
                break;
              }
            }
          }
          if ((revents & (EPOLLERR | EPOLLHUP)) != 0) {
            drop_bt_backend(controller, port,
                            "Bluetooth L2CAP control epoll error", logger);
            epoll_dirty = true;
          }
          continue;
        }

        if (source.type == EventType::BtInterrupt) {
          if ((revents & EPOLLIN) != 0) {
            for (int packet = 0; packet < kMaxBtPacketsPerWake; ++packet) {
              try {
                if (!handle_bt_input(port, *controller.backend, companion,
                                     trace_flags, logger)) {
                  break;
                }
              } catch (const std::exception &error) {
                if (!is_bluetooth_error(error)) {
                  throw;
                }
                drop_bt_backend(controller, port, error.what(), logger);
                epoll_dirty = true;
                break;
              }
            }
          }
          if ((revents & (EPOLLERR | EPOLLHUP)) != 0) {
            drop_bt_backend(controller, port,
                            "Bluetooth L2CAP interrupt epoll error", logger);
            epoll_dirty = true;
          }
        }
      }

      // epoll_dirty doubles as a connection-change signal so fresh
      // controllers receive the current companion state.
      if (companion.actuation.version != applied_companion_version ||
          epoll_dirty) {
        apply_companion_state(ports, controllers, companion, trace_flags,
                              logger);
        applied_companion_version = companion.actuation.version;
      }

      flush_pending_outputs(ports, controllers, trace_flags, logger,
                            epoll_dirty);
    }

    logger.log(vds::LogScope::Daemon, vds::LogLevel::Info, "stopping");
    disconnect_all(ports, controllers, logger);
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "vdsd: " << error.what() << "\n";
    return 1;
  }
}
