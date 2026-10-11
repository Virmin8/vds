// SPDX-License-Identifier: MIT
// Copyright (C) 2026 Jihong Min <hurryman2212@gmail.com>

#ifndef NOMINMAX
#define NOMINMAX
#endif

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <ratio>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

#include <windows.h>

#include <mmsystem.h>
#include <sddl.h>

#include "jsonl.hh"
#include "uapi/vds.h"
#include "unique_handle.hh"
#include "vds/ds5_protocol.h"
#include "vds_bt.hh"
#include "vds_build_info.hh"
#include "vds_bridge.hh"
#include "vds_common.hh"
#include "vds_config.hh"
#include "vds_hidhide.hh"
#include "vds_io.hh"
#include "vds_log.hh"
#include "vds_profile.hh"
#include "vds_protocol.hh"
#include "vds_usbip.hh"
#include "vds_win32.hh"
#include "vds_companion.hh"
#include "vdsd_common.hh"

namespace {

using Clock = std::chrono::steady_clock;

constexpr const char *kDefaultControlPipe = R"(\\.\pipe\vdsd)";
constexpr const char *kVirtualPortProviderUnavailableReason =
    "virtual port provider unavailable";
constexpr const char *kWindowsVirtualPortProviderUnavailable =
    "usbip-win2 is not installed or usbip.exe is unavailable";
constexpr const char *kWindowsHidHideProviderUnavailable =
    "HidHide is not installed or HidHideCLI.exe is unavailable";
constexpr DWORD kDeviceShareMode =
    FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE;
constexpr auto kBridgeOpenRetryDelay = std::chrono::milliseconds(1000);
constexpr auto kBridgeWorkerFailureRetryDelay = std::chrono::milliseconds(1000);
constexpr UINT kWindowsTimerResolutionMs = 1;
constexpr const char *kServiceName = "vdsd";
constexpr std::array<std::uint8_t, 3> kInitialFeatureReportIds = {0x09, 0x20,
                                                                  0x05};
// Control requests carry a 64-byte companion report as a JSON array.
constexpr std::size_t kMaxControlRequest = 8192;
std::atomic_bool g_stop_requested = false;
HANDLE g_stop_event = nullptr;
SERVICE_STATUS_HANDLE g_service_status_handle = nullptr;
SERVICE_STATUS g_service_status{};
DWORD g_service_exit_code = ERROR_SUCCESS;

using vds::trim_command;

using vds::bt_feature_to_usb_feature_reply;
using vds::hidp_output_packet;
using vds::win::BluetoothTransport;
using vds::win::describe_bluetooth_lookup;
using vds::win::Frame;
using vds::win::HidBluetoothDevice;
using vds::win::list_bluetooth_controller_devices;
using vds::win::list_hid_bluetooth_devices;
using vds::win::make_hid_bluetooth_transport;
using vds::win::UniqueHandle;
using vds::win::win32_error_message;

struct Options {
  std::string db_path;
  std::string log_path;
  std::string pipe = kDefaultControlPipe;
  float speaker_boost_db = vds::kDefaultSpeakerBoostDb;
  bool mic_enabled = false;
};

class TimerResolutionGuard {
public:
  explicit TimerResolutionGuard(UINT period_ms) : period_ms_(period_ms) {
    active_ = timeBeginPeriod(period_ms_) == TIMERR_NOERROR;
  }

  ~TimerResolutionGuard() {
    if (active_) {
      timeEndPeriod(period_ms_);
    }
  }

  TimerResolutionGuard(const TimerResolutionGuard &) = delete;
  TimerResolutionGuard &operator=(const TimerResolutionGuard &) = delete;

  bool active() const { return active_; }

private:
  UINT period_ms_ = 0;
  bool active_ = false;
};

class AudioThreadPriorityGuard {
public:
  AudioThreadPriorityGuard() {
    previous_priority_ = GetThreadPriority(GetCurrentThread());
    priority_set_ =
        SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL);

    avrt_ = LoadLibraryA("avrt.dll");
    if (!avrt_) {
      return;
    }

    av_revert_ = reinterpret_cast<AvRevertMmThreadCharacteristicsProc>(
        GetProcAddress(avrt_, "AvRevertMmThreadCharacteristics"));
    const auto av_set = reinterpret_cast<AvSetMmThreadCharacteristicsAProc>(
        GetProcAddress(avrt_, "AvSetMmThreadCharacteristicsA"));
    if (av_set == nullptr || av_revert_ == nullptr) {
      return;
    }

    DWORD task_index = 0;
    mmcss_handle_ = av_set("Pro Audio", &task_index);
    if (mmcss_handle_ != nullptr) {
      mmcss_active_ = true;
    }
  }

  ~AudioThreadPriorityGuard() {
    if (mmcss_active_ && av_revert_ != nullptr) {
      av_revert_(mmcss_handle_);
    }
    if (priority_set_ && previous_priority_ != THREAD_PRIORITY_ERROR_RETURN) {
      SetThreadPriority(GetCurrentThread(), previous_priority_);
    }
    if (avrt_ != nullptr) {
      FreeLibrary(avrt_);
    }
  }

  AudioThreadPriorityGuard(const AudioThreadPriorityGuard &) = delete;
  AudioThreadPriorityGuard &
  operator=(const AudioThreadPriorityGuard &) = delete;

  bool boosted() const { return priority_set_ || mmcss_active_; }

  std::string summary() const {
    return "audio flush thread priority priority_set=" +
           std::string(priority_set_ ? "yes" : "no") +
           " mmcss=" + (mmcss_active_ ? "yes" : "no");
  }

private:
  using AvSetMmThreadCharacteristicsAProc = HANDLE(WINAPI *)(LPCSTR, LPDWORD);
  using AvRevertMmThreadCharacteristicsProc = BOOL(WINAPI *)(HANDLE);

  HMODULE avrt_ = nullptr;
  HANDLE mmcss_handle_ = nullptr;
  AvRevertMmThreadCharacteristicsProc av_revert_ = nullptr;
  int previous_priority_ = THREAD_PRIORITY_ERROR_RETURN;
  bool priority_set_ = false;
  bool mmcss_active_ = false;
};

Options parse_platform_args(int argc, char **argv) {
  Options options;
  vds::VdsdCommonOptions common = vds::default_vdsd_common_options();
  const std::string platform_options =
      std::string("[--pipe ") + kDefaultControlPipe +
      "] [--speaker-boost-db 0-18] [--mic|--no-mic]";
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
    } else if (arg == "--pipe") {
      options.pipe = next_value(i, arg);
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
      throw std::runtime_error("unknown option: " + std::string(arg));
    }
  }
  options.db_path = common.db_path;
  options.log_path = common.log_path;
  return options;
}

vds::ConfigDb load_config_db_for_attempt(const Options &options,
                                         vds::Logger &logger) {
  (void)logger;
  return vds::load_config_db(options.db_path);
}

std::optional<HidBluetoothDevice>
find_connected_bluetooth_device(const std::string &address,
                                std::span<const HidBluetoothDevice> devices,
                                std::string *failure_reason = nullptr) {
  for (const auto &device : devices) {
    if (vds::normalize_bluetooth_address(device.address) != address) {
      continue;
    }
    if (device.bluetooth_connected) {
      return device;
    }
    if (failure_reason != nullptr) {
      *failure_reason =
          "Windows Bluetooth reports address=" + address + " connected=no";
    }
    return std::nullopt;
  }

  if (failure_reason != nullptr) {
    *failure_reason = "Windows HID did not report a connected Bluetooth "
                      "device for address=" +
                      address + "; " + describe_bluetooth_lookup(address);
  }
  return std::nullopt;
}

std::optional<HidBluetoothDevice>
find_connected_bluetooth_device(const std::string &address,
                                std::string *failure_reason = nullptr) {
  try {
    return find_connected_bluetooth_device(
        address, list_hid_bluetooth_devices(), failure_reason);
  } catch (const std::exception &error) {
    if (failure_reason != nullptr) {
      *failure_reason =
          std::string("Windows HID query failed: ") + error.what();
    }
    return std::nullopt;
  }
}

std::vector<vds::ControllerTarget> list_windows_controller_targets() {
  std::vector<vds::ControllerTarget> targets;
  for (const auto &device : list_bluetooth_controller_devices()) {
    if (!device.profile_valid || device.address.empty()) {
      continue;
    }
    targets.push_back(vds::ControllerTarget{
        .address = vds::normalize_bluetooth_address(device.address),
        .name = device.name,
        .profile = device.profile == VDS_PROFILE_DSE
                       ? vds::ControllerProfile::Dse
                       : vds::ControllerProfile::Ds5,
        .online = device.bluetooth_connected,
    });
  }
  return targets;
}

bool virtual_port_is_enabled(unsigned port) {
  return port < vds::kMaxPortCount;
}

std::vector<unsigned> enabled_virtual_ports() {
  std::vector<unsigned> ports;
  ports.reserve(vds::kMaxPortCount);
  for (unsigned port = 0; port < vds::kMaxPortCount; ++port) {
    if (virtual_port_is_enabled(port)) {
      ports.push_back(port);
    }
  }
  return ports;
}

bool virtual_port_provider_available() {
  return vds::win::usbip::client_available();
}

bool virtual_port_is_available(unsigned port) {
  return virtual_port_is_enabled(port);
}

std::vector<unsigned> available_virtual_ports() {
  std::vector<unsigned> ports;
  ports.reserve(vds::kMaxPortCount);
  for (unsigned port = 0; port < vds::kMaxPortCount; ++port) {
    if (virtual_port_is_available(port)) {
      ports.push_back(port);
    }
  }
  return ports;
}

bool controller_has_enabled_virtual_port(
    const vds::ControllerConfig &config,
    std::span<const unsigned> enabled_ports) {
  return vds::controller_config_has_candidate_port(config, enabled_ports);
}

BOOL WINAPI console_handler(DWORD type) {
  if (type == CTRL_C_EVENT || type == CTRL_BREAK_EVENT ||
      type == CTRL_CLOSE_EVENT || type == CTRL_SHUTDOWN_EVENT) {
    g_stop_requested = true;
    if (g_stop_event != nullptr) {
      SetEvent(g_stop_event);
    }
    return TRUE;
  }
  return FALSE;
}

void update_service_status(DWORD state, DWORD win32_exit_code = ERROR_SUCCESS,
                           DWORD wait_hint = 0) {
  if (g_service_status_handle == nullptr) {
    return;
  }

  g_service_status.dwServiceType = SERVICE_WIN32_OWN_PROCESS;
  g_service_status.dwCurrentState = state;
  g_service_status.dwControlsAccepted =
      state == SERVICE_RUNNING ? SERVICE_ACCEPT_STOP | SERVICE_ACCEPT_SHUTDOWN
                               : 0;
  g_service_status.dwWin32ExitCode = win32_exit_code;
  g_service_status.dwServiceSpecificExitCode =
      win32_exit_code == ERROR_SERVICE_SPECIFIC_ERROR ? g_service_exit_code : 0;
  g_service_status.dwCheckPoint =
      state == SERVICE_START_PENDING || state == SERVICE_STOP_PENDING ? 1 : 0;
  g_service_status.dwWaitHint = wait_hint;
  SetServiceStatus(g_service_status_handle, &g_service_status);
}

DWORD WINAPI service_control_handler(DWORD control, DWORD, LPVOID, LPVOID) {
  if (control == SERVICE_CONTROL_STOP || control == SERVICE_CONTROL_SHUTDOWN) {
    g_stop_requested = true;
    if (g_stop_event != nullptr) {
      SetEvent(g_stop_event);
    }
    update_service_status(SERVICE_STOP_PENDING, ERROR_SUCCESS, 5000);
    return ERROR_SUCCESS;
  }
  if (control == SERVICE_CONTROL_INTERROGATE) {
    SetServiceStatus(g_service_status_handle, &g_service_status);
    return ERROR_SUCCESS;
  }
  return ERROR_CALL_NOT_IMPLEMENTED;
}

UniqueHandle open_device(const std::string &path) {
  UniqueHandle handle(CreateFileA(
      path.c_str(), GENERIC_READ | GENERIC_WRITE, kDeviceShareMode, nullptr,
      OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED, nullptr));
  if (!handle) {
    throw std::runtime_error("failed to open " + path + ": " +
                             win32_error_message(GetLastError()));
  }
  return handle;
}

vds::win::HandleIoCancellation handle_io_cancellation() {
  return {
      .stop_event = g_stop_event,
      .stop_requested = &g_stop_requested,
      .operation_aborted_is_stop = false,
  };
}

std::uint32_t resolve_virtual_port_profile(const vds::ControllerConfig &config,
                                           vds::Logger &logger) {
  if (config.profile != vds::ControllerProfile::Unspecified) {
    return vds::usb_profile_from_controller_profile(config.profile);
  }

  std::string discovery_error;
  const auto device =
      find_connected_bluetooth_device(config.address, &discovery_error);
  if (!device) {
    if (discovery_error.empty()) {
      discovery_error = "no matching HID device";
    }
    throw std::runtime_error("failed to detect Bluetooth controller profile: " +
                             discovery_error);
  }
  if (!device->profile_valid) {
    throw std::runtime_error("failed to detect Bluetooth controller profile: "
                             "Bluetooth transport did not report DS5/DSE "
                             "profile");
  }
  logger.log(vds::LogScope::Daemon, vds::LogLevel::Info,
             "detected Bluetooth profile=" +
                 vds::usb_profile_name(device->profile));
  return device->profile;
}

/*
 * Bridge sessions. Each controller runs three threads around vds_bridge:
 * virtual USB frames in, Bluetooth packets in, and the paced audio/haptics
 * sender. g_bridge_mutex guards bridge_settings(), the companion runtime and
 * every session's port, so those threads and the control pipe all go through
 * vds_bridge under it (as the Linux event loop does on one thread).
 */
std::mutex g_bridge_mutex;
vds::CompanionRuntime g_companion;

// vds_bridge's view of the HID Bluetooth transport. Writes go through the
// transport's output queue; feature reports are read directly.
class WinBridgeBluetooth final : public vds::BridgeBluetooth {
public:
  WinBridgeBluetooth(BluetoothTransport &bluetooth, std::mutex &mutex,
                     vds::Logger &logger)
      : bluetooth_(bluetooth), mutex_(mutex), logger_(logger) {}

  bool try_send_output_report(std::span<const std::uint8_t> report) override {
    try {
      std::lock_guard guard(mutex_);
      return bluetooth_.try_write_interrupt_packet(hidp_output_packet(report));
    } catch (const std::exception &error) {
      logger_.log(vds::LogScope::Output, vds::LogLevel::Warn,
                  std::string("output report ignored: ") + error.what());
      return false;
    }
  }

  std::optional<std::vector<std::uint8_t>>
  get_feature_report(std::uint8_t report_id) override {
    try {
      std::lock_guard guard(mutex_);
      return bluetooth_.read_feature_report(report_id);
    } catch (const std::exception &error) {
      logger_.log(vds::LogScope::Usb, vds::LogLevel::Warn,
                  "feature get report_id=" + std::to_string(report_id) +
                      " failed: " + error.what());
      return std::nullopt;
    }
  }

  void set_feature_report(std::span<const std::uint8_t> report) override {
    try {
      std::lock_guard guard(mutex_);
      bluetooth_.write_feature_report(report);
    } catch (const std::exception &error) {
      logger_.log(vds::LogScope::Usb, vds::LogLevel::Warn,
                  std::string("feature set failed: ") + error.what());
    }
  }

private:
  BluetoothTransport &bluetooth_;
  std::mutex &mutex_;
  vds::Logger &logger_;
};

// vds_bridge's view of the usbip virtual controller.
class WinBridgeUsb final : public vds::BridgeUsb {
public:
  explicit WinBridgeUsb(HANDLE device) : device_(device) {}

  bool write_frame(std::uint32_t type,
                   std::span<const std::uint8_t> payload) override {
    vds::win::write_handle_frame(device_, static_cast<std::uint16_t>(type),
                                 payload, "virtual", handle_io_cancellation());
    return true;
  }

private:
  HANDLE device_;
};

struct BridgeSession {
  vds::BridgePort port;
  std::string address;
  WinBridgeBluetooth *bt = nullptr;
  // Wakes the audio thread to re-check its schedule (new audio, settings).
  HANDLE audio_wake = nullptr;
};

// Sessions with a connected controller; guarded by g_bridge_mutex.
std::vector<BridgeSession *> g_sessions;

std::uint32_t load_trace_flags(const std::atomic_uint32_t *trace_flags) {
  return trace_flags == nullptr ? 0 : trace_flags->load();
}

void request_bridge_restart(HANDLE virtual_device,
                            BluetoothTransport *bluetooth, HANDLE audio_wake,
                            std::atomic_bool &bridge_restart_requested) {
  if (bridge_restart_requested.exchange(true)) {
    return;
  }
  if (virtual_device != INVALID_HANDLE_VALUE && virtual_device != nullptr) {
    CancelIoEx(virtual_device, nullptr);
  }
  if (bluetooth != nullptr) {
    bluetooth->cancel();
  }
  if (audio_wake != nullptr) {
    SetEvent(audio_wake);
  }
}

// Turns a controller off (Turn Off Controller, Idle Disconnect).
void power_off_controller(const std::string &address, std::string_view reason,
                          vds::Logger &logger) {
  try {
    if (vds::win::disconnect_bluetooth_device(address)) {
      logger.log(vds::LogScope::Bluetooth, vds::LogLevel::Info,
                 "controller turned off address=" + address + " (" +
                     std::string(reason) + ")");
    } else {
      logger.log(vds::LogScope::Bluetooth, vds::LogLevel::Warn,
                 "turn off: no Bluetooth radio knew address=" + address);
    }
  } catch (const std::exception &error) {
    logger.log(vds::LogScope::Bluetooth, vds::LogLevel::Warn,
               "turn off failed address=" + address + " error=" +
                   error.what());
  }
}

void virtual_to_bluetooth_loop(HANDLE virtual_device, BridgeSession &session,
                               WinBridgeUsb &usb, vds::Logger &logger,
                               const std::atomic_uint32_t *trace_flags,
                               std::atomic_bool &bridge_restart_requested) {
  while (!g_stop_requested.load() && !bridge_restart_requested.load()) {
    const Frame frame = vds::win::read_handle_frame(
        virtual_device, "virtual", handle_io_cancellation());
    {
      std::lock_guard lock(g_bridge_mutex);
      vds::bridge_handle_usb_frame(session.port, frame.header.type,
                                   frame.payload, session.bt, usb,
                                   load_trace_flags(trace_flags), logger);
    }
    SetEvent(session.audio_wake);
  }
}

void bluetooth_to_virtual_loop(BluetoothTransport &bluetooth,
                               BridgeSession &session, WinBridgeUsb &usb,
                               vds::Logger &logger,
                               const std::atomic_uint32_t *trace_flags,
                               std::atomic_bool &bridge_restart_requested) {
  while (!g_stop_requested.load() && !bridge_restart_requested.load()) {
    const Frame frame = bluetooth.read_frame();
    const auto read_time = Clock::now();
    if (frame.header.type == VDS_FRAME_BT_INTERRUPT_PACKET) {
      vds::BridgeInputResult result;
      {
        std::lock_guard lock(g_bridge_mutex);
        result = vds::bridge_handle_bt_input(
            session.port, frame.payload, read_time, *session.bt, usb,
            g_companion, load_trace_flags(trace_flags), logger);
      }
      SetEvent(session.audio_wake);
      if (result.idle_timeout) {
        power_off_controller(session.address, "idle", logger);
      }
    } else if (frame.header.type == VDS_FRAME_BT_CONTROL_PACKET) {
      if (const auto report = bt_feature_to_usb_feature_reply(frame.payload)) {
        std::lock_guard lock(g_bridge_mutex);
        vds::bridge_handle_feature_reply(session.port, *report, usb,
                                         load_trace_flags(trace_flags), logger);
      }
    }
  }
}

// Sends the paced audio/haptics packets, deferred state reports and mic open
// retries, sleeping on a high-resolution timer until the next one is due or
// the other threads queue something.
void audio_loop(BridgeSession &session, vds::Logger &logger,
                const std::atomic_uint32_t *trace_flags,
                std::atomic_bool &bridge_restart_requested) {
  AudioThreadPriorityGuard priority_guard;
  logger.log(vds::LogScope::Output,
             priority_guard.boosted() ? vds::LogLevel::Info
                                      : vds::LogLevel::Warn,
             priority_guard.summary());
  UniqueHandle timer(CreateWaitableTimerExA(
      nullptr, nullptr, CREATE_WAITABLE_TIMER_HIGH_RESOLUTION,
      TIMER_MODIFY_STATE | SYNCHRONIZE));
  if (!timer) {
    timer.reset(CreateWaitableTimerExA(nullptr, nullptr, 0,
                                       TIMER_MODIFY_STATE | SYNCHRONIZE));
  }
  if (!timer) {
    throw std::runtime_error("failed to create audio timer: " +
                             win32_error_message(GetLastError()));
  }

  while (!g_stop_requested.load() && !bridge_restart_requested.load()) {
    std::optional<Clock::time_point> wake;
    {
      std::lock_guard lock(g_bridge_mutex);
      const std::uint32_t flags = load_trace_flags(trace_flags);
      vds::BridgePort &port = session.port;
      vds::bridge_retry_mic_open(port, *session.bt, logger);
      vds::bridge_enqueue_waveout(port, flags, logger);
      if (vds::bridge_flush_pending_state(port, *session.bt, flags, logger)) {
        (void)vds::bridge_flush_audio(port, *session.bt, flags, logger);
      }
      wake = vds::bridge_next_wakeup(port, Clock::now());
    }

    std::vector<HANDLE> handles{session.audio_wake};
    if (g_stop_event != nullptr) {
      handles.push_back(g_stop_event);
    }
    if (wake) {
      const auto delay = *wake - Clock::now();
      if (delay <= Clock::duration::zero()) {
        continue;
      }
      using HundredNanoseconds =
          std::chrono::duration<long long, std::ratio<1, 10000000>>;
      LARGE_INTEGER due_time{};
      due_time.QuadPart = -std::max<long long>(
          1, std::chrono::duration_cast<HundredNanoseconds>(delay).count());
      if (SetWaitableTimerEx(timer.get(), &due_time, 0, nullptr, nullptr,
                             nullptr, 0)) {
        handles.push_back(timer.get());
      } else {
        std::this_thread::sleep_for(delay);
        continue;
      }
    }
    WaitForMultipleObjects(static_cast<DWORD>(handles.size()), handles.data(),
                           FALSE, INFINITE);
  }
}

void run_bridge_session(const std::string &device_address,
                        const std::string &virtual_device,
                        const vds::ControllerConfig *selected_config,
                        vds::Logger &logger,
                        std::atomic_bool *session_stop_requested = nullptr,
                        const std::atomic_uint32_t *trace_flags = nullptr) {
  if (selected_config == nullptr) {
    throw std::runtime_error("Windows bridge session requires config");
  }
  const std::uint32_t profile =
      resolve_virtual_port_profile(*selected_config, logger);

  logger.log(vds::LogScope::Daemon, vds::LogLevel::Info,
             "Windows bridge opening virtual=" + virtual_device +
                 " device_address=" + device_address);
  std::string discovery_error;
  const auto device =
      find_connected_bluetooth_device(device_address, &discovery_error);
  if (!device) {
    throw std::runtime_error("failed to open Bluetooth controller: " +
                             discovery_error);
  }
  {
    std::lock_guard lock(g_bridge_mutex);
    if (vds::bridge_settings().host_persona != vds::HostPersona::DualSense) {
      logger.log(vds::LogScope::Companion, vds::LogLevel::Warn,
                 "DualShock 4 and Xbox 360 personas are not available on "
                 "Windows yet; bridging as a DualSense");
    }
  }
  std::unique_ptr<vds::win::hidhide::DeviceGuard> hidden_device =
      vds::win::hidhide::hide_device(device->instance_path, logger);
  std::unique_ptr<BluetoothTransport> bluetooth =
      make_hid_bluetooth_transport(device_address);
  std::unique_ptr<vds::win::usbip::VirtualPort> virtual_port =
      vds::win::usbip::open_virtual_port(profile, selected_config->ports[0],
                                         logger);
  std::mutex bluetooth_mutex;
  UniqueHandle virtual_device_handle = open_device(virtual_port->pipe_path());
  UniqueHandle audio_wake(CreateEventA(nullptr, FALSE, FALSE, nullptr));
  if (!audio_wake) {
    throw std::runtime_error("failed to create audio wake event: " +
                             win32_error_message(GetLastError()));
  }
  std::atomic_bool bridge_restart_requested = false;
  std::thread stop_monitor;

  WinBridgeBluetooth bt(*bluetooth, bluetooth_mutex, logger);
  WinBridgeUsb usb(virtual_device_handle.get());
  BridgeSession session;
  session.port.path = virtual_device;
  session.address = vds::normalize_bluetooth_address(selected_config->address);
  session.bt = &bt;
  session.audio_wake = audio_wake.get();

  if (session_stop_requested != nullptr) {
    stop_monitor = std::thread([&] {
      while (!g_stop_requested.load() && !bridge_restart_requested.load() &&
             !session_stop_requested->load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
      if (g_stop_requested.load() || session_stop_requested->load()) {
        request_bridge_restart(virtual_device_handle.get(), bluetooth.get(),
                               audio_wake.get(), bridge_restart_requested);
      }
    });
  }
  auto stop_startup_monitors = [&] {
    bridge_restart_requested = true;
    if (stop_monitor.joinable()) {
      stop_monitor.join();
    }
  };

  try {
    // Prime the feature cache (calibration, firmware, MAC) for the host.
    for (const std::uint8_t report_id : kInitialFeatureReportIds) {
      if (const auto report = bt.get_feature_report(report_id)) {
        vds::bridge_store_feature_report(session.port, *report);
      }
    }
    logger.log(vds::LogScope::Bluetooth, vds::LogLevel::Info,
               "sending Windows startup Bluetooth output init report");
    bluetooth->write_interrupt_packet(
        hidp_output_packet(session.port.output_state.build_bt_init_report()));
    session.port.last_sent_bt_state = session.port.output_state.state();
  } catch (...) {
    stop_startup_monitors();
    throw;
  }
  if (bridge_restart_requested.load()) {
    stop_startup_monitors();
    throw std::runtime_error("Bluetooth transport disconnected during startup");
  }

  // Companion settings (and the mic, when it is on) apply right away.
  {
    std::lock_guard lock(g_bridge_mutex);
    g_sessions.push_back(&session);
    const vds::DsCompanionOverrides overrides =
        vds::bridge_companion_overrides(g_companion);
    vds::bridge_apply_companion(session.port, g_companion.settings, overrides,
                                bt, load_trace_flags(trace_flags), logger);
  }

  logger.log(vds::LogScope::Daemon, vds::LogLevel::Info,
             "Windows bridge started virtual=" + virtual_device +
                 " bluetooth=" + bluetooth->description());

  const auto run_thread = [&](const char *name, auto &&body) {
    return std::thread([&, name, body] {
      try {
        body();
      } catch (const std::exception &error) {
        if (!g_stop_requested) {
          logger.log(vds::LogScope::Daemon, vds::LogLevel::Error,
                     std::string(name) + " stopped: " + error.what());
        }
      }
      request_bridge_restart(virtual_device_handle.get(), bluetooth.get(),
                             audio_wake.get(), bridge_restart_requested);
    });
  };
  std::thread to_bluetooth = run_thread("virtual-to-bluetooth", [&] {
    virtual_to_bluetooth_loop(virtual_device_handle.get(), session, usb,
                              logger, trace_flags, bridge_restart_requested);
  });
  std::thread to_virtual = run_thread("bluetooth-to-virtual", [&] {
    bluetooth_to_virtual_loop(*bluetooth, session, usb, logger, trace_flags,
                              bridge_restart_requested);
  });
  std::thread audio = run_thread("audio", [&] {
    audio_loop(session, logger, trace_flags, bridge_restart_requested);
  });

  to_bluetooth.join();
  to_virtual.join();
  audio.join();
  bridge_restart_requested = true;
  if (stop_monitor.joinable()) {
    stop_monitor.join();
  }
  {
    std::lock_guard lock(g_bridge_mutex);
    std::erase(g_sessions, &session);
    vds::bridge_on_disconnect(session.port);
  }
}

// Applies the companion runtime (settings, momentary effects, Turn Off) to
// every connected controller.
void apply_companion_state(std::uint32_t trace_flags, vds::Logger &logger) {
  std::vector<std::string> power_off;
  {
    std::lock_guard lock(g_bridge_mutex);
    (void)vds::expire_companion_actuation(g_companion, Clock::now());
    if (g_companion.actuation.power_off_pending) {
      g_companion.actuation.power_off_pending = false;
      for (const BridgeSession *session : g_sessions) {
        power_off.push_back(session->address);
      }
    }
    const vds::DsCompanionOverrides overrides =
        vds::bridge_companion_overrides(g_companion);
    for (BridgeSession *session : g_sessions) {
      vds::bridge_apply_companion(session->port, g_companion.settings,
                                  overrides, *session->bt, trace_flags, logger);
      SetEvent(session->audio_wake);
    }
  }
  for (const std::string &address : power_off) {
    power_off_controller(address, "turned off from the companion app", logger);
  }
}

// Mic and headset state of the first connected controller, for the companion
// AUDIO_STATUS report. Caller holds g_bridge_mutex.
void publish_companion_audio_status() {
  g_companion.audio_status = g_sessions.empty()
                                 ? vds::CompanionAudioStatus{}
                                 : vds::bridge_audio_status(g_sessions.front()->port);
}

std::string handle_audio_stats_request() {
  const auto now = Clock::now();
  std::string reply = "{";
  reply += vds::jsonl_bool_field("OK", true);
  reply += ',';
  reply += vds::jsonl_string_field("error", "");
  reply += ",\"ports\":[";
  std::lock_guard lock(g_bridge_mutex);
  bool first = true;
  for (BridgeSession *session : g_sessions) {
    if (!first) {
      reply += ',';
    }
    first = false;
    reply += vds::bridge_audio_stats_json(session->port, now);
  }
  reply += "]}\n";
  return reply;
}

std::string control_error_reply(std::string_view error) {
  std::string reply = "{";
  reply += vds::jsonl_bool_field("OK", false);
  reply += ',';
  reply += vds::jsonl_string_field("error", std::string(error));
  reply += "}\n";
  return reply;
}


struct BridgeWorker {
  std::string address;
  std::string device_address;
  unsigned port = 0;
  vds::ControllerProfile profile = vds::ControllerProfile::Unspecified;
  std::string last_error;
  UniqueHandle done_event;
  std::thread thread;
  std::atomic_bool stop_requested = false;
  std::atomic_bool done = false;
};

// The control pipe's security: vdsd runs as a service, and the companion
// app (a normal user) must be able to talk to it, like the vds group's
// access to /run/vdsd.sock on Linux. SYSTEM and Administrators get full
// control, signed-in users read/write.
class PipeSecurity {
public:
  PipeSecurity() {
    if (ConvertStringSecurityDescriptorToSecurityDescriptorA(
            "D:(A;;GA;;;SY)(A;;GA;;;BA)(A;;GRGW;;;AU)", SDDL_REVISION_1,
            &descriptor_, nullptr)) {
      attributes_.nLength = sizeof(attributes_);
      attributes_.lpSecurityDescriptor = descriptor_;
      attributes_.bInheritHandle = FALSE;
    }
  }
  ~PipeSecurity() {
    if (descriptor_ != nullptr) {
      LocalFree(descriptor_);
    }
  }
  PipeSecurity(const PipeSecurity &) = delete;
  PipeSecurity &operator=(const PipeSecurity &) = delete;

  SECURITY_ATTRIBUTES *attributes() {
    return descriptor_ != nullptr ? &attributes_ : nullptr;
  }

private:
  PSECURITY_DESCRIPTOR descriptor_ = nullptr;
  SECURITY_ATTRIBUTES attributes_{};
};

class ControlPipeServer {
public:
  ControlPipeServer(std::string pipe_name, vds::Logger &logger)
      : logger_(logger), pipe_name_(std::move(pipe_name)),
        stop_event_(CreateEventA(nullptr, TRUE, FALSE, nullptr)),
        command_event_(CreateEventA(nullptr, TRUE, FALSE, nullptr)) {
    if (!stop_event_ || !command_event_) {
      throw std::runtime_error("failed to create control pipe event: " +
                               win32_error_message(GetLastError()));
    }
    thread_ = std::thread([this] { thread_main(); });
  }

  ~ControlPipeServer() { stop(); }

  ControlPipeServer(const ControlPipeServer &) = delete;
  ControlPipeServer &operator=(const ControlPipeServer &) = delete;

  HANDLE command_event() const { return command_event_.get(); }

  template <typename Handler> void service_ready(Handler &&handler) {
    while (true) {
      std::shared_ptr<ControlRequest> request;
      {
        std::lock_guard lock(mutex_);
        if (requests_.empty()) {
          ResetEvent(command_event_.get());
          return;
        }
        request = requests_.front();
        requests_.pop_front();
      }

      std::string reply;
      if (request->command.empty()) {
        reply = "{\"OK\":false,\"error\":\"empty command\"}\n";
      } else {
        reply = handler(trim_command(request->command));
      }

      {
        std::lock_guard lock(request->mutex);
        request->reply = std::move(reply);
      }
      SetEvent(request->done_event.get());
    }
  }

  void stop() {
    if (stop_requested_.exchange(true)) {
      return;
    }
    SetEvent(stop_event_.get());
    complete_queued_requests("{\"OK\":false,\"error\":\"daemon stopping\"}\n");

    if (thread_.joinable()) {
      thread_.join();
    }
  }

private:
  struct ControlRequest {
    std::string command;
    std::string reply;
    UniqueHandle done_event;
    std::mutex mutex;
  };

  void complete_queued_requests(std::string reply) {
    std::deque<std::shared_ptr<ControlRequest>> requests;
    {
      std::lock_guard lock(mutex_);
      requests.swap(requests_);
      ResetEvent(command_event_.get());
    }
    for (const auto &request : requests) {
      {
        std::lock_guard lock(request->mutex);
        request->reply = reply;
      }
      SetEvent(request->done_event.get());
    }
  }

  std::string queue_command(std::string command) {
    auto request = std::make_shared<ControlRequest>();
    request->command = std::move(command);
    request->done_event.reset(CreateEventA(nullptr, TRUE, FALSE, nullptr));
    if (!request->done_event) {
      return "{\"OK\":false,\"error\":\"failed to queue command\"}\n";
    }

    {
      std::lock_guard lock(mutex_);
      if (stop_requested_) {
        return "{\"OK\":false,\"error\":\"daemon stopping\"}\n";
      }
      requests_.push_back(request);
      SetEvent(command_event_.get());
    }

    HANDLE wait_handles[2] = {request->done_event.get(), stop_event_.get()};
    const DWORD wait_result =
        WaitForMultipleObjects(2, wait_handles, FALSE, INFINITE);
    if (wait_result != WAIT_OBJECT_0) {
      return "{\"OK\":false,\"error\":\"daemon stopping\"}\n";
    }

    std::lock_guard lock(request->mutex);
    return request->reply;
  }

  void thread_main() {
    while (!stop_requested_) {
      UniqueHandle pipe(CreateNamedPipeA(
          pipe_name_.c_str(), PIPE_ACCESS_DUPLEX | FILE_FLAG_OVERLAPPED,
          PIPE_TYPE_MESSAGE | PIPE_READMODE_MESSAGE | PIPE_WAIT, 1, 65536,
          65536, 0, security_.attributes()));
      if (!pipe) {
        logger_.log(vds::LogScope::Control, vds::LogLevel::Error,
                    "failed to create control pipe " + pipe_name_ + ": " +
                        win32_error_message(GetLastError()));
        WaitForSingleObject(stop_event_.get(), 1000);
        continue;
      }

      if (!connect_client(pipe.get()) || stop_requested_) {
        DisconnectNamedPipe(pipe.get());
        continue;
      }

      handle_client(pipe.get());
      DisconnectNamedPipe(pipe.get());
    }
  }

  bool wait_for_pipe_io(HANDLE pipe, OVERLAPPED &overlapped,
                        DWORD &bytes_transferred, std::string_view label) {
    HANDLE wait_handles[2] = {overlapped.hEvent, stop_event_.get()};
    const DWORD wait_result =
        WaitForMultipleObjects(2, wait_handles, FALSE, INFINITE);
    if (wait_result == WAIT_OBJECT_0 + 1) {
      CancelIoEx(pipe, &overlapped);
      (void)GetOverlappedResult(pipe, &overlapped, &bytes_transferred, TRUE);
      return false;
    }
    if (wait_result != WAIT_OBJECT_0) {
      CancelIoEx(pipe, &overlapped);
      (void)GetOverlappedResult(pipe, &overlapped, &bytes_transferred, TRUE);
      if (!stop_requested_) {
        logger_.log(vds::LogScope::Control, vds::LogLevel::Warn,
                    std::string(label) +
                        " wait failed: " + win32_error_message(GetLastError()));
      }
      return false;
    }
    if (!GetOverlappedResult(pipe, &overlapped, &bytes_transferred, FALSE)) {
      const DWORD error = GetLastError();
      if (error == ERROR_MORE_DATA) {
        SetLastError(ERROR_MORE_DATA);
        return false;
      }
      if (!stop_requested_ && error != ERROR_NO_DATA &&
          error != ERROR_BROKEN_PIPE) {
        logger_.log(vds::LogScope::Control, vds::LogLevel::Warn,
                    std::string(label) +
                        " completion failed: " + win32_error_message(error));
      }
      return false;
    }
    return true;
  }

  bool connect_client(HANDLE pipe) {
    UniqueHandle event(CreateEventA(nullptr, TRUE, FALSE, nullptr));
    if (!event) {
      logger_.log(vds::LogScope::Control, vds::LogLevel::Error,
                  "failed to create control pipe connect event: " +
                      win32_error_message(GetLastError()));
      return false;
    }

    OVERLAPPED overlapped{};
    overlapped.hEvent = event.get();
    if (ConnectNamedPipe(pipe, &overlapped)) {
      return true;
    }

    const DWORD error = GetLastError();
    if (error == ERROR_PIPE_CONNECTED) {
      return true;
    }
    if (error != ERROR_IO_PENDING) {
      if (!stop_requested_) {
        logger_.log(vds::LogScope::Control, vds::LogLevel::Warn,
                    "control pipe connect failed: " +
                        win32_error_message(error));
      }
      return false;
    }

    DWORD ignored = 0;
    return wait_for_pipe_io(pipe, overlapped, ignored, "control pipe connect");
  }

  // Reads one request; message-mode pipes hand long ones over in pieces
  // (ERROR_MORE_DATA).
  std::optional<std::string> read_command(HANDLE pipe) {
    UniqueHandle event(CreateEventA(nullptr, TRUE, FALSE, nullptr));
    if (!event) {
      logger_.log(vds::LogScope::Control, vds::LogLevel::Error,
                  "failed to create control pipe read event: " +
                      win32_error_message(GetLastError()));
      return std::nullopt;
    }

    std::string command;
    std::array<char, 4096> buffer{};
    while (command.size() < kMaxControlRequest) {
      DWORD got = 0;
      OVERLAPPED overlapped{};
      overlapped.hEvent = event.get();
      ResetEvent(event.get());
      bool more = false;
      if (!ReadFile(pipe, buffer.data(), static_cast<DWORD>(buffer.size()),
                    &got, &overlapped)) {
        DWORD error = GetLastError();
        if (error == ERROR_IO_PENDING) {
          if (!wait_for_pipe_io(pipe, overlapped, got, "control pipe read")) {
            if (GetLastError() != ERROR_MORE_DATA) {
              return std::nullopt;
            }
            error = ERROR_MORE_DATA;
          } else {
            error = ERROR_SUCCESS;
          }
        }
        if (error == ERROR_MORE_DATA) {
          more = true;
        } else if (error != ERROR_SUCCESS) {
          if (!stop_requested_ && error != ERROR_NO_DATA &&
              error != ERROR_BROKEN_PIPE) {
            logger_.log(vds::LogScope::Control, vds::LogLevel::Warn,
                        "control pipe read failed: " +
                            win32_error_message(error));
          }
          return std::nullopt;
        }
      }
      command.append(buffer.data(), static_cast<std::size_t>(got));
      if (!more) {
        break;
      }
    }
    if (command.empty()) {
      return std::nullopt;
    }
    return command;
  }

  bool write_reply(HANDLE pipe, const std::string &reply) {
    UniqueHandle event(CreateEventA(nullptr, TRUE, FALSE, nullptr));
    if (!event) {
      logger_.log(vds::LogScope::Control, vds::LogLevel::Error,
                  "failed to create control pipe write event: " +
                      win32_error_message(GetLastError()));
      return false;
    }

    DWORD written = 0;
    OVERLAPPED overlapped{};
    overlapped.hEvent = event.get();
    const BOOL started =
        WriteFile(pipe, reply.data(), static_cast<DWORD>(reply.size()),
                  &written, &overlapped);
    if (!started) {
      const DWORD error = GetLastError();
      if (error != ERROR_IO_PENDING) {
        if (!stop_requested_) {
          logger_.log(vds::LogScope::Control, vds::LogLevel::Warn,
                      "control pipe reply failed: " +
                          win32_error_message(error));
        }
        return false;
      }
      if (!wait_for_pipe_io(pipe, overlapped, written, "control pipe reply")) {
        return false;
      }
    }
    return true;
  }

  void handle_client(HANDLE pipe) {
    const auto command = read_command(pipe);
    if (!command) {
      return;
    }

    const std::string reply = queue_command(*command);
    (void)write_reply(pipe, reply);
  }

  vds::Logger &logger_;
  PipeSecurity security_;
  std::string pipe_name_;
  UniqueHandle stop_event_;
  UniqueHandle command_event_;
  std::thread thread_;
  std::atomic_bool stop_requested_ = false;
  std::mutex mutex_;
  std::deque<std::shared_ptr<ControlRequest>> requests_;
};

bool worker_is_connected(const BridgeWorker &worker) {
  return !worker.done.load() && !worker.stop_requested.load();
}

std::string handle_supervisor_control_command(
    const std::string &command,
    const std::vector<std::unique_ptr<BridgeWorker>> &workers,
    const std::string &db_path, const std::string &log_path,
    std::uint32_t &trace_flags, bool &reload_requested, vds::Logger &logger) {
  std::vector<vds::JsonlField> fields;
  const std::optional<std::string> local_command =
      vds::parse_control_command_name(command, fields);
  if (local_command == "audio") {
    return handle_audio_stats_request();
  }
  if (local_command == "log") {
    return vds::handle_daemon_log_request(fields, log_path, trace_flags,
                                          logger);
  }
  if (local_command == "bluetooth") {
    return control_error_reply(
        "Bluetooth modes are not available on Windows");
  }

  std::vector<vds::VdsdControlControllerStatus> controller_statuses;
  controller_statuses.reserve(workers.size());
  for (const auto &worker : workers) {
    controller_statuses.push_back(vds::VdsdControlControllerStatus{
        .address = worker->address,
        .connected = worker_is_connected(*worker),
        .path = worker_is_connected(*worker)
                    ? vds::win::usbip::endpoint_for_port(worker->port)
                    : "",
    });
  }

  std::vector<vds::VdsdControlPortCandidate> port_candidates;
  port_candidates.reserve(vds::kMaxPortCount);
  for (unsigned port = 0; port < vds::kMaxPortCount; ++port) {
    port_candidates.push_back(vds::VdsdControlPortCandidate{
        .port = port,
        .path = vds::win::usbip::endpoint_for_port(port),
    });
  }

  std::vector<vds::VdsdControlPortBinding> port_bindings;
  port_bindings.reserve(workers.size());
  for (const auto &worker : workers) {
    if (!worker_is_connected(*worker)) {
      continue;
    }
    port_bindings.push_back(vds::VdsdControlPortBinding{
        .port = worker->port,
        .address = worker->address,
        .device_address = worker->device_address,
    });
  }

  const std::vector<vds::VdsdControlPortStatus> port_statuses =
      vds::build_vdsd_control_port_statuses(port_candidates, port_bindings);

  // Companion requests touch the runtime the sessions read; the others may
  // be slow (target discovery) and must not stall the audio threads.
  std::unique_lock<std::mutex> bridge_lock;
  if (local_command == "companion") {
    bridge_lock = std::unique_lock<std::mutex>(g_bridge_mutex);
    publish_companion_audio_status();
  }
  return vds::handle_vdsd_control_command(
      command, db_path, controller_statuses, port_statuses,
      [] { return list_windows_controller_targets(); }, trace_flags,
      reload_requested, g_companion, logger);
}

void run_configured_bridge_worker(vds::ControllerConfig config,
                                  std::string device_address,
                                  vds::Logger &logger,
                                  std::atomic_bool &stop_requested,
                                  std::atomic_bool &done,
                                  std::string &last_error,
                                  const std::atomic_uint32_t &trace_flags) {
  try {
    if (config.ports.size() != 1) {
      throw std::runtime_error("bridge worker requires one reserved port");
    }
    const std::string virtual_device =
        vds::win::usbip::endpoint_for_port(config.ports[0]);
    run_bridge_session(device_address, virtual_device, &config, logger,
                       &stop_requested, &trace_flags);
    if (!g_stop_requested && !stop_requested.load()) {
      last_error = "bridge session ended";
      logger.log(vds::LogScope::Daemon, vds::LogLevel::Warn,
                 "Windows bridge worker ended address=" + config.address +
                     ": " + last_error);
    }
  } catch (const std::exception &error) {
    if (!g_stop_requested && !stop_requested.load()) {
      last_error = error.what();
      logger.log(vds::LogScope::Daemon, vds::LogLevel::Error,
                 "Windows bridge worker failed address=" + config.address +
                     ": " + error.what());
    }
  }
  done = true;
}

void run_bridge_supervisor(const Options &options, vds::Logger &logger) {
  std::uint64_t applied_companion_version = 0;
  std::vector<std::unique_ptr<BridgeWorker>> workers;
  std::vector<vds::VdsdWorkerFailureBackoff> worker_failure_backoffs;
  std::vector<std::string> discovery_warned_states;
  std::vector<std::string> no_port_warned_addresses;
  std::vector<std::string> port_binding_warned_states;
  ControlPipeServer control_pipe(options.pipe, logger);
  UniqueHandle retry_timer(CreateWaitableTimerExA(
      nullptr, nullptr, CREATE_WAITABLE_TIMER_HIGH_RESOLUTION,
      TIMER_MODIFY_STATE | SYNCHRONIZE));
  if (!retry_timer) {
    retry_timer.reset(CreateWaitableTimerExA(nullptr, nullptr, 0,
                                             TIMER_MODIFY_STATE | SYNCHRONIZE));
  }
  if (!retry_timer) {
    throw std::runtime_error("failed to create supervisor retry timer: " +
                             win32_error_message(GetLastError()));
  }
  std::atomic_uint32_t trace_flags = 0;
  bool reload_requested = false;
  bool scan_requested = true;
  bool config_load_logged = false;
  bool virtual_port_provider_warned = false;
  logger.log(vds::LogScope::Daemon, vds::LogLevel::Info,
             "Windows bridge supervisor started pipe=" + options.pipe);

  enum class WaitKind {
    Stop,
    Control,
    Retry,
    WorkerDone,
  };
  struct WaitTarget {
    HANDLE handle = nullptr;
    WaitKind kind = WaitKind::Stop;
    std::size_t index = 0;
  };
  auto arm_retry_timer = [&](Clock::duration duration) {
    using HundredNanoseconds =
        std::chrono::duration<long long, std::ratio<1, 10000000>>;
    const auto ticks = std::max<long long>(
        1, std::chrono::duration_cast<HundredNanoseconds>(duration).count());
    LARGE_INTEGER due_time{};
    due_time.QuadPart = -ticks;
    if (!SetWaitableTimerEx(retry_timer.get(), &due_time, 0, nullptr, nullptr,
                            nullptr, 0)) {
      throw std::runtime_error("failed to arm supervisor retry timer: " +
                               win32_error_message(GetLastError()));
    }
  };

  while (!g_stop_requested) {
    bool worker_removed = false;
    for (auto worker = workers.begin(); worker != workers.end();) {
      if ((*worker)->done.load()) {
        if ((*worker)->thread.joinable()) {
          (*worker)->thread.join();
        }
        if (!(*worker)->last_error.empty()) {
          vds::record_vdsd_worker_failure(
              worker_failure_backoffs, (*worker)->address,
              Clock::now() + kBridgeWorkerFailureRetryDelay);
        }
        worker = workers.erase(worker);
        worker_removed = true;
      } else {
        ++worker;
      }
    }
    if (worker_removed) {
      scan_requested = true;
    }

    if (reload_requested) {
      scan_requested = true;
    }

    if (std::any_of(worker_failure_backoffs.begin(),
                    worker_failure_backoffs.end(), [](const auto &backoff) {
                      return Clock::now() >= backoff.retry_after;
                    })) {
      scan_requested = true;
    }

    if (scan_requested) {
      scan_requested = false;
      try {
        const vds::ConfigDb db = load_config_db_for_attempt(options, logger);
        if (!config_load_logged || reload_requested) {
          logger.log(vds::LogScope::Daemon, vds::LogLevel::Info,
                     "loaded controller config count=" +
                         std::to_string(db.controllers.size()));
          config_load_logged = true;
        }
        const std::vector<unsigned> enabled_ports = enabled_virtual_ports();
        const bool virtual_port_provider_is_available =
            !enabled_ports.empty() && vds::win::usbip::client_available() &&
            vds::win::hidhide::provider_available();
        if (!virtual_port_provider_is_available) {
          if (!virtual_port_provider_warned) {
            std::string detail;
            if (!vds::win::usbip::client_available()) {
              detail = kWindowsVirtualPortProviderUnavailable;
            } else {
              detail = kWindowsHidHideProviderUnavailable;
            }
            logger.log(vds::LogScope::Port, vds::LogLevel::Error,
                       std::string(kVirtualPortProviderUnavailableReason) +
                           " detail=" + detail);
            virtual_port_provider_warned = true;
          }
        } else {
          if (virtual_port_provider_warned) {
            logger.log(vds::LogScope::Port, vds::LogLevel::Info,
                       "virtual port provider recovered");
            virtual_port_provider_warned = false;
          }
        }

        const std::vector<HidBluetoothDevice> hid_devices =
            list_hid_bluetooth_devices();

        for (const auto &worker : workers) {
          const bool stale = vds::vdsd_worker_config_is_stale(
              db, worker->address, worker->profile, worker->port,
              virtual_port_is_enabled(worker->port));
          if (stale && !worker->stop_requested.exchange(true)) {
            logger.log(
                vds::LogScope::Daemon, vds::LogLevel::Info,
                "Windows bridge worker retiring address=" + worker->address +
                    " port=" + std::to_string(worker->port));
          }
        }

        std::vector<unsigned> reserved_ports;
        reserved_ports.reserve(workers.size());
        std::vector<std::string> reserved_device_addresses;
        reserved_device_addresses.reserve(workers.size());
        for (const auto &worker : workers) {
          reserved_ports.push_back(worker->port);
          reserved_device_addresses.push_back(worker->device_address);
        }

        for (const auto &config : db.controllers) {
          if (!vds::consume_vdsd_worker_retry(worker_failure_backoffs,
                                              config.address, Clock::now())) {
            continue;
          }

          const auto existing = std::find_if(
              workers.begin(), workers.end(), [&](const auto &worker) {
                return worker->address == config.address;
              });
          if (existing != workers.end()) {
            continue;
          }
          if (!virtual_port_provider_is_available) {
            if (vds::remember_vdsd_warning_state(
                    port_binding_warned_states, config.address,
                    kVirtualPortProviderUnavailableReason)) {
              logger.log(
                  vds::LogScope::Daemon, vds::LogLevel::Warn,
                  "controller binding unavailable address=" + config.address +
                      " reason=" + kVirtualPortProviderUnavailableReason);
            }
            continue;
          }
          const std::string port_binding_error =
              "no allowed virtual port enabled ports=[" +
              vds::format_ports(config.ports) + "]";
          if (!controller_has_enabled_virtual_port(config, enabled_ports)) {
            if (vds::remember_vdsd_warning_state(port_binding_warned_states,
                                                 config.address,
                                                 port_binding_error)) {
              logger.log(vds::LogScope::Daemon, vds::LogLevel::Warn,
                         "controller binding unavailable address=" +
                             config.address + " reason=" + port_binding_error);
            }
            continue;
          }
          vds::forget_vdsd_warning_state(port_binding_warned_states,
                                         config.address);
          std::string discovery_error;
          const auto connected_device = find_connected_bluetooth_device(
              config.address, hid_devices, &discovery_error);
          if (!connected_device) {
            if (vds::remember_vdsd_warning_state(
                    discovery_warned_states, config.address, discovery_error)) {
              logger.log(vds::LogScope::Daemon, vds::LogLevel::Warn,
                         "controller HID unavailable address=" +
                             config.address + " reason=" + discovery_error);
            }
            continue;
          }
          vds::forget_vdsd_warning_state(discovery_warned_states,
                                         config.address);
          const vds::VdsdWorkerLaunchDecision launch =
              vds::select_vdsd_worker_launch_decision(
                  config, available_virtual_ports(), reserved_ports,
                  config.address, virtual_port_provider_is_available,
                  reserved_device_addresses);
          if (launch.status ==
              vds::VdsdWorkerLaunchStatus::VirtualPortProviderUnavailable) {
            if (vds::remember_vdsd_warning_state(
                    port_binding_warned_states, config.address,
                    kVirtualPortProviderUnavailableReason)) {
              logger.log(
                  vds::LogScope::Daemon, vds::LogLevel::Warn,
                  "rejected HID device address=" + config.address +
                      " reason=" + kVirtualPortProviderUnavailableReason);
            }
            continue;
          }
          if (launch.status ==
              vds::VdsdWorkerLaunchStatus::DeviceAddressReserved) {
            continue;
          }
          if (launch.status == vds::VdsdWorkerLaunchStatus::NoAvailablePort) {
            if (vds::remember_vdsd_warning(no_port_warned_addresses,
                                           config.address)) {
              logger.log(vds::LogScope::Daemon, vds::LogLevel::Warn,
                         "rejected HID device address=" + config.address +
                             " reason=no available virtual port");
            }
            continue;
          }
          vds::forget_vdsd_warning(no_port_warned_addresses, config.address);

          auto worker = std::make_unique<BridgeWorker>();
          worker->address = config.address;
          worker->device_address = launch.device_address;
          worker->port = launch.port;
          worker->profile = config.profile;
          worker->done_event.reset(CreateEventA(nullptr, TRUE, FALSE, nullptr));
          if (!worker->done_event) {
            throw std::runtime_error("failed to create worker event: " +
                                     win32_error_message(GetLastError()));
          }
          BridgeWorker *worker_ptr = worker.get();
          worker->thread = std::thread([worker_config = launch.config,
                                        device_address = launch.device_address,
                                        &logger, worker_ptr, &trace_flags] {
            run_configured_bridge_worker(worker_config, device_address, logger,
                                         worker_ptr->stop_requested,
                                         worker_ptr->done,
                                         worker_ptr->last_error, trace_flags);
            SetEvent(worker_ptr->done_event.get());
          });
          logger.log(
              vds::LogScope::Daemon, vds::LogLevel::Info,
              "Windows bridge worker launched address=" + config.address +
                  " port=" + std::to_string(launch.port) +
                  " device_address=" + launch.device_address);
          workers.push_back(std::move(worker));
          reserved_ports.push_back(launch.port);
          reserved_device_addresses.push_back(launch.device_address);
        }
      } catch (const std::exception &error) {
        logger.log(vds::LogScope::Daemon, vds::LogLevel::Error,
                   std::string("Windows bridge supervisor scan failed: ") +
                       error.what());
        scan_requested = true;
      }
    }

    if (reload_requested) {
      reload_requested = false;
    }

    const auto now = Clock::now();
    std::optional<Clock::time_point> retry_after = now + kBridgeOpenRetryDelay;
    if (!worker_failure_backoffs.empty()) {
      const auto next_retry = std::min_element(
          worker_failure_backoffs.begin(), worker_failure_backoffs.end(),
          [](const auto &left, const auto &right) {
            return left.retry_after < right.retry_after;
          });
      if (next_retry != worker_failure_backoffs.end()) {
        if (next_retry->retry_after <= now) {
          scan_requested = true;
          continue;
        }
        retry_after =
            retry_after
                ? std::min(*retry_after, next_retry->retry_after)
                : std::optional<Clock::time_point>(next_retry->retry_after);
      }
    }

    // Momentary companion effects (rumble/trigger tests) end on a deadline.
    std::optional<Clock::time_point> companion_deadline;
    {
      std::lock_guard lock(g_bridge_mutex);
      companion_deadline = vds::next_companion_actuation_deadline(g_companion);
    }
    if (companion_deadline) {
      if (*companion_deadline <= now) {
        apply_companion_state(trace_flags.load(), logger);
        continue;
      }
      retry_after = retry_after ? std::min(*retry_after, *companion_deadline)
                                : companion_deadline;
    }

    bool retry_timer_armed = false;
    if (retry_after) {
      arm_retry_timer(*retry_after - now);
      retry_timer_armed = true;
    } else {
      CancelWaitableTimer(retry_timer.get());
    }

    std::vector<WaitTarget> wait_targets;
    wait_targets.reserve(workers.size() + 3);
    if (g_stop_event != nullptr) {
      wait_targets.push_back(WaitTarget{
          .handle = g_stop_event,
          .kind = WaitKind::Stop,
          .index = 0,
      });
    }
    wait_targets.push_back(WaitTarget{
        .handle = control_pipe.command_event(),
        .kind = WaitKind::Control,
        .index = 0,
    });
    if (retry_timer_armed) {
      wait_targets.push_back(WaitTarget{
          .handle = retry_timer.get(),
          .kind = WaitKind::Retry,
          .index = 0,
      });
    }
    for (std::size_t index = 0; index < workers.size(); ++index) {
      if (!workers[index]->done_event) {
        continue;
      }
      wait_targets.push_back(WaitTarget{
          .handle = workers[index]->done_event.get(),
          .kind = WaitKind::WorkerDone,
          .index = index,
      });
    }

    std::vector<HANDLE> wait_handles;
    wait_handles.reserve(wait_targets.size());
    for (const auto &target : wait_targets) {
      wait_handles.push_back(target.handle);
    }

    const DWORD wait_result =
        WaitForMultipleObjects(static_cast<DWORD>(wait_handles.size()),
                               wait_handles.data(), FALSE, INFINITE);
    if (wait_result == WAIT_FAILED) {
      throw std::runtime_error("Windows bridge supervisor wait failed: " +
                               win32_error_message(GetLastError()));
    }

    const DWORD wait_index = wait_result - WAIT_OBJECT_0;
    if (wait_index >= wait_targets.size()) {
      throw std::runtime_error("Windows bridge supervisor wait returned an "
                               "unexpected object index");
    }

    switch (wait_targets[wait_index].kind) {
    case WaitKind::Stop:
      g_stop_requested = true;
      break;
    case WaitKind::Control:
      control_pipe.service_ready([&](const std::string &command) {
        std::uint32_t current_trace_flags = trace_flags.load();
        const std::string reply = handle_supervisor_control_command(
            command, workers, options.db_path, options.log_path,
            current_trace_flags, reload_requested, logger);
        trace_flags.store(current_trace_flags);
        return reply;
      });
      {
        std::uint64_t version = 0;
        {
          std::lock_guard lock(g_bridge_mutex);
          version = g_companion.actuation.version;
        }
        if (version != applied_companion_version) {
          applied_companion_version = version;
          apply_companion_state(trace_flags.load(), logger);
        }
      }
      if (reload_requested) {
        scan_requested = true;
      }
      break;
    case WaitKind::Retry:
      CancelWaitableTimer(retry_timer.get());
      scan_requested = true;
      break;
    case WaitKind::WorkerDone:
      scan_requested = true;
      break;
    }
  }

  for (auto &worker : workers) {
    if (worker->thread.joinable()) {
      worker->thread.join();
    }
  }
  logger.log(vds::LogScope::Daemon, vds::LogLevel::Info,
             "Windows bridge supervisor stopped");
}

} // namespace

int run_windows_daemon(int argc, char **argv, bool service_mode) {
  try {
    g_stop_requested = false;
    const Options options = parse_platform_args(argc, argv);
    UniqueHandle stop_event(CreateEventA(nullptr, TRUE, FALSE, nullptr));
    if (!stop_event) {
      throw std::runtime_error("failed to create stop event: " +
                               win32_error_message(GetLastError()));
    }
    g_stop_event = stop_event.get();
    if (!service_mode) {
      SetConsoleCtrlHandler(console_handler, TRUE);
    }

    vds::Logger logger(options.log_path);
    {
      vds::BridgeSettings &bridge = vds::bridge_settings();
      bridge.speaker_boost_db = options.speaker_boost_db;
      bridge.mic_forced = options.mic_enabled;
      bridge.mic_allowed = bridge.mic_forced;
    }
    try {
      TimerResolutionGuard timer_resolution(kWindowsTimerResolutionMs);
      logger.log(
          vds::LogScope::Output,
          timer_resolution.active() ? vds::LogLevel::Info : vds::LogLevel::Warn,
          std::string("Windows timer resolution 1 ms ") +
              (timer_resolution.active() ? "enabled" : "request failed"));
      if (!virtual_port_provider_available()) {
        logger.log(vds::LogScope::Port, vds::LogLevel::Error,
                   std::string(kVirtualPortProviderUnavailableReason) +
                       " detail=" + kWindowsVirtualPortProviderUnavailable);
        throw std::runtime_error(kWindowsVirtualPortProviderUnavailable);
      }
      if (!vds::win::hidhide::provider_available()) {
        logger.log(vds::LogScope::Bluetooth, vds::LogLevel::Error,
                   std::string(kVirtualPortProviderUnavailableReason) +
                       " detail=" + kWindowsHidHideProviderUnavailable);
        throw std::runtime_error(kWindowsHidHideProviderUnavailable);
      }
      vds::win::hidhide::register_daemon();
      if (service_mode) {
        update_service_status(SERVICE_RUNNING);
        logger.log(vds::LogScope::Daemon, vds::LogLevel::Info,
                   "Windows service started");
      }

      run_bridge_supervisor(options, logger);
      g_stop_event = nullptr;
      logger.log(vds::LogScope::Daemon, vds::LogLevel::Info,
                 "Windows bridge stopped");
      return 0;
    } catch (const std::exception &error) {
      try {
        logger.log(vds::LogScope::Daemon, vds::LogLevel::Error,
                   std::string("Windows daemon failed: ") + error.what());
      } catch (...) {
      }
      throw;
    } catch (...) {
      try {
        logger.log(vds::LogScope::Daemon, vds::LogLevel::Error,
                   "Windows daemon failed with an unknown error");
      } catch (...) {
      }
      throw;
    }
  } catch (const std::exception &error) {
    std::cerr << "error: " << error.what() << "\n";
    return 1;
  } catch (...) {
    std::cerr << "error: unknown Windows daemon failure\n";
    return 1;
  }
}

void WINAPI service_main(DWORD argc, LPSTR *argv) {
  g_service_status_handle = RegisterServiceCtrlHandlerExA(
      kServiceName, service_control_handler, nullptr);
  if (g_service_status_handle == nullptr) {
    g_service_exit_code = GetLastError();
    return;
  }

  update_service_status(SERVICE_START_PENDING, ERROR_SUCCESS, 5000);
  g_service_exit_code = static_cast<DWORD>(
      run_windows_daemon(static_cast<int>(argc), argv, true));
  update_service_status(SERVICE_STOPPED, g_service_exit_code == 0
                                             ? ERROR_SUCCESS
                                             : ERROR_SERVICE_SPECIFIC_ERROR);
}

bool dispatch_service_if_needed() {
  SERVICE_TABLE_ENTRYA service_table[] = {
      {const_cast<LPSTR>(kServiceName), service_main},
      {nullptr, nullptr},
  };
  if (StartServiceCtrlDispatcherA(service_table)) {
    return true;
  }

  const DWORD error = GetLastError();
  if (error == ERROR_FAILED_SERVICE_CONTROLLER_CONNECT) {
    return false;
  }
  throw std::runtime_error("failed to connect service dispatcher: " +
                           win32_error_message(error));
}

int main(int argc, char **argv) {
  try {
    if (dispatch_service_if_needed()) {
      return static_cast<int>(g_service_exit_code);
    }
  } catch (const std::exception &error) {
    std::cerr << "error: " << error.what() << "\n";
    return 1;
  }
  return run_windows_daemon(argc, argv, false);
}
