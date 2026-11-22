// SPDX-License-Identifier: MIT
#pragma once

/*
 * Platform-neutral USB/IP device model for a vDS virtual controller.
 *
 * Ported from the former vds_hcd kernel module (vds_usb_core.c +
 * vds_hcd_core.c): the same descriptors, control-request behaviour,
 * HID IN queueing, feature-report cache and isochronous pacing, but driven by
 * USB/IP CMD_SUBMIT / CMD_UNLINK PDUs instead of kernel URBs. A platform
 * backend supplies the USB/IP transport (on Linux, a socket handed to the
 * in-tree vhci_hcd driver) and the frame channel to vdsd.
 *
 * Threading: run_session() blocks on the transport's receive path; a
 * completion thread owned by the session sends RET_SUBMIT / RET_UNLINK in
 * completion order (immediate before timed, timed by due time), so replies
 * for one seqnum can never be reordered against its unlink. Frames from vdsd
 * are fed in from any thread via handle_daemon_frame().
 */

#include <array>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <list>
#include <mutex>
#include <optional>
#include <span>
#include <string>
#include <thread>
#include <vector>

namespace vds {
class Logger;
}

namespace vds::usbip {

// ---- USB/IP wire format (big-endian on the wire) --------------------------

constexpr std::uint32_t kCmdSubmit = 0x00000001;
constexpr std::uint32_t kCmdUnlink = 0x00000002;
constexpr std::uint32_t kRetSubmit = 0x00000003;
constexpr std::uint32_t kRetUnlink = 0x00000004;

constexpr std::uint32_t kDirOut = 0;
constexpr std::uint32_t kDirIn = 1;

// Linux errno values as carried in USB/IP status fields.
constexpr std::int32_t kStatusEpipe = -32;
constexpr std::int32_t kStatusEoverflow = -75;
constexpr std::int32_t kStatusEconnreset = -104;

#pragma pack(push, 1)
struct HeaderBasic {
  std::uint32_t command;
  std::uint32_t seqnum;
  std::uint32_t devid;
  std::uint32_t direction;
  std::uint32_t ep;
};

struct CmdSubmit {
  HeaderBasic base;
  std::uint32_t transfer_flags;
  std::int32_t transfer_buffer_length;
  std::int32_t start_frame;
  std::int32_t number_of_packets;
  std::int32_t interval;
  std::uint8_t setup[8];
};

struct RetSubmit {
  HeaderBasic base;
  std::int32_t status;
  std::int32_t actual_length;
  std::int32_t start_frame;
  std::int32_t number_of_packets;
  std::int32_t error_count;
  std::uint8_t padding[8];
};

struct CmdUnlink {
  HeaderBasic base;
  std::uint32_t unlink_seqnum;
  std::uint8_t padding[24];
};

struct RetUnlink {
  HeaderBasic base;
  std::int32_t status;
  std::uint8_t padding[24];
};

struct IsoPacketDescriptor {
  std::uint32_t offset;
  std::uint32_t length;
  std::int32_t actual_length;
  std::int32_t status;
};
#pragma pack(pop)

static_assert(sizeof(CmdSubmit) == 48);
static_assert(sizeof(RetSubmit) == 48);
static_assert(sizeof(CmdUnlink) == 48);
static_assert(sizeof(RetUnlink) == 48);
static_assert(sizeof(IsoPacketDescriptor) == 16);

std::uint32_t to_be32(std::uint32_t value);
std::uint32_t from_be32(std::uint32_t value);

// ---- Platform hooks --------------------------------------------------------

// The USB/IP byte stream (the host controller side is the peer).
class Transport {
public:
  virtual ~Transport() = default;
  virtual bool recv_exact(void *buffer, std::size_t size) = 0;
  virtual bool send_exact(const void *buffer, std::size_t size) = 0;
};

// Frames from the device model to vdsd (the read side of /dev/vdsN).
class DaemonChannel {
public:
  virtual ~DaemonChannel() = default;
  // One vds frame; returns false when the frame was dropped (backpressure).
  virtual bool write_frame(std::uint16_t type,
                           std::span<const std::uint8_t> payload) = 0;
};

// ---- Controller profiles ---------------------------------------------------

constexpr std::uint8_t kNoInterface = 0xff;
constexpr std::uint8_t kNoEndpoint = 0xff;
constexpr std::uint8_t kNoFeatureUnit = 0xff;
constexpr std::size_t kHidPacketSize = 64;

struct ControllerProfile {
  std::uint32_t profile;
  std::uint16_t vendor_id;
  std::uint16_t product_id;
  std::uint16_t device_version;
  std::uint8_t device_class;
  std::uint8_t device_subclass;
  std::uint8_t device_protocol;
  std::uint8_t max_packet_size0;
  std::uint8_t num_configurations;
  std::uint8_t configuration_value;
  const char *manufacturer;
  const char *product;
  std::span<const std::uint8_t> configuration_descriptor;
  std::span<const std::uint8_t> hid_descriptor;
  std::span<const std::uint8_t> hid_report_descriptor;
  std::uint8_t audio_control_interface;
  std::uint8_t audio_out_interface;
  std::uint8_t audio_in_interface;
  std::uint8_t hid_interface;
  std::uint8_t audio_out_endpoint;
  std::uint8_t audio_in_endpoint;
  std::uint8_t hid_in_endpoint;
  std::uint8_t hid_out_endpoint;
  std::uint32_t hid_in_interval_us;
  std::uint8_t speaker_feature_unit;
  std::uint8_t mic_feature_unit;
};

bool is_valid_profile(std::uint32_t profile);
const ControllerProfile &controller_profile(std::uint32_t profile);

// ---- Device ----------------------------------------------------------------

class Device {
public:
  using Clock = std::chrono::steady_clock;

  struct Hooks {
    // Runs at the start of each session's completion (pacing) thread, e.g. to
    // raise its scheduling priority.
    std::function<void()> on_completion_thread_start;
  };

  Device(DaemonChannel &channel, Logger *logger, std::string label);
  ~Device();

  Device(const Device &) = delete;
  Device &operator=(const Device &) = delete;

  void set_hooks(Hooks hooks);

  // Like VDS_IOC_SET_PROFILE: fails (false) while a session is active.
  bool set_profile(std::uint32_t profile);
  std::uint32_t profile() const;

  // Like VDS_IOC_DISCONNECT's device-side effects: forgets USB state, queued
  // input/mic audio, the feature cache and pacing anchors.
  void reset_state();

  // Serves one USB/IP session until the transport fails or stop_session()
  // is called (the caller must also unblock the transport's recv, e.g. by
  // shutting the socket down). Blocks.
  void run_session(Transport &transport);
  void stop_session();
  bool session_active() const;

  // A frame written by vdsd (VDS_FRAME_USB_HID_IN / FEATURE_REPLY /
  // AUDIO_IN); other types are ignored. Thread-safe.
  void handle_daemon_frame(std::uint16_t type,
                           std::span<const std::uint8_t> payload);

private:
  enum class Deliver { None, IsoOut, IsoIn };
  enum class Kind { Submit, UnlinkReply };

  struct Urb {
    Kind kind = Kind::Submit;
    std::uint32_t seqnum = 0;
    bool dir_in = false;
    std::uint32_t transfer_buffer_length = 0;
    std::int32_t number_of_packets = 0; // > 0 only for isochronous
    // number_of_packets exactly as the host sent it (0, or -1 from some
    // Windows clients, for non-isochronous transfers); echoed back.
    std::int32_t wire_number_of_packets = 0;
    std::int32_t status = 0;
    std::uint32_t out_actual = 0; // bytes consumed by an OUT transfer
    std::vector<std::uint8_t> data; // OUT payload (iso OUT) or IN reply
    std::vector<IsoPacketDescriptor> iso; // host byte order
    std::int32_t error_count = 0;
    std::int32_t start_frame = 0;
    Deliver deliver = Deliver::None;
    bool timed = false;
    Clock::time_point ready{};
    std::uint8_t feature_report_id = 0;
    std::uint16_t reply_limit = 0; // control IN: min(wLength, buffer length)
  };

  enum class ControlResult { Complete, Pending };

  // All below: called with mutex_ held.
  void reset_state_locked();
  void enqueue_completion_locked(Urb urb);
  void schedule_iso_locked(Urb &urb);
  void schedule_hid_in_locked(Urb &urb);
  ControlResult handle_control_locked(Urb &urb, const std::uint8_t *setup,
                                      std::span<const std::uint8_t> out_data);
  int standard_control_locked(Urb &urb, std::uint8_t request_type,
                              std::uint8_t request, std::uint16_t value,
                              std::uint16_t index);
  ControlResult hid_control_locked(Urb &urb, std::uint8_t request,
                                   std::uint16_t value, std::uint16_t index,
                                   std::uint16_t length,
                                   std::span<const std::uint8_t> out_data);
  int audio_control_locked(Urb &urb, bool device_to_host,
                           std::uint8_t request, std::uint16_t value,
                           std::uint16_t index,
                           std::span<const std::uint8_t> out_data);
  void fill_iso_in_locked(Urb &urb);
  bool remove_pending_locked(std::uint32_t seqnum);

  void handle_submit(const CmdSubmit &cmd, std::vector<std::uint8_t> out_data,
                     std::vector<IsoPacketDescriptor> iso);
  void handle_unlink(const CmdUnlink &cmd);
  void deliver_iso_out(Urb &urb);
  void completion_loop(Transport &transport);
  static std::vector<std::uint8_t> serialize(const Urb &urb);

  void write_frame(std::uint16_t type, std::span<const std::uint8_t> payload);
  void log_info(const std::string &message);
  void log_warn(const std::string &message);

  DaemonChannel &channel_;
  Logger *logger_;
  std::string label_;
  Hooks hooks_;

  mutable std::mutex mutex_;
  std::condition_variable completion_cv_;
  std::uint32_t profile_;
  bool session_active_ = false;
  bool session_stop_ = false;

  // USB device state (vds_usb_device).
  std::uint8_t configuration_ = 0;
  std::uint8_t audio_out_altsetting_ = 0;
  std::uint8_t audio_in_altsetting_ = 0;
  std::uint8_t hid_idle_ = 0;
  std::uint8_t hid_protocol_ = 0;
  std::array<std::uint8_t, 2> audio_mute_{};
  std::array<std::int16_t, 2> audio_volume_{};
  std::array<std::vector<std::uint8_t>, 256> feature_cache_{};

  // HCD queues (vds_hcd_dev).
  std::list<Urb> completions_;
  std::deque<Urb> pending_hid_in_;
  std::list<Urb> pending_feature_get_;
  std::optional<std::vector<std::uint8_t>> input_packet_;
  std::deque<std::uint8_t> audio_in_;
  Clock::time_point next_iso_out_ready_{};
  Clock::time_point next_iso_in_ready_{};
  Clock::time_point next_hid_in_ready_{};
};

} // namespace vds::usbip
