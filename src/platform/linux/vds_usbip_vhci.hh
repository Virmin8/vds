// SPDX-License-Identifier: MIT
#pragma once

/*
 * A vDS virtual port backed by the in-tree vhci_hcd driver (USB/IP host
 * side). This replaced the former out-of-tree vds_hcd kernel module.
 *
 * vdsd sees the same thing it gets from /dev/vdsN: a file descriptor that
 * yields one vds frame per read() and accepts one frame per write()
 * (a SOCK_SEQPACKET socketpair), plus connect / disconnect / set_profile in
 * place of the VDS_IOC_* ioctls. Connecting hands one end of a stream socket
 * to /sys/devices/platform/vhci_hcd.0/attach; the USB/IP device model in
 * src/usbip/ serves the other end. Nothing listens on the network.
 */

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "unique_fd.hh"

namespace vds {
class Logger;
}

namespace vds::usbip {

class Device;

// True when vhci_hcd is loaded (or loadable on demand); `reason` explains a
// false result.
bool vhci_available(std::string *reason = nullptr);

class VhciPort {
public:
  VhciPort(unsigned port_index, Logger &logger);
  ~VhciPort();

  VhciPort(const VhciPort &) = delete;
  VhciPort &operator=(const VhciPort &) = delete;

  // vdsd's end of the frame channel (non-blocking, close-on-exec). Ownership
  // moves to the caller; valid once.
  UniqueFd take_daemon_fd();

  // VDS_IOC_SET_PROFILE. Throws while connected.
  void set_profile(std::uint32_t profile);
  // VDS_IOC_CONNECT: plug the device into vhci_hcd. Throws on failure.
  void connect();
  // VDS_IOC_DISCONNECT: unplug. Idempotent, never throws.
  void disconnect() noexcept;
  bool connected() const;

  // Fragment of the attached device's sysfs path ("/vhci_hcd.0/usbB/B-P/"),
  // for matching its hidraw node; empty when not attached.
  std::string sysfs_marker() const;

private:
  struct Channel;
  struct Attachment;

  void reader_loop();
  void session_loop(std::shared_ptr<Attachment> attachment);
  std::shared_ptr<Attachment> attach_once();

  unsigned port_index_;
  Logger &logger_;
  std::string label_;
  UniqueFd daemon_fd_;
  UniqueFd core_fd_;
  std::unique_ptr<Channel> channel_;
  std::unique_ptr<Device> device_;
  std::thread reader_thread_;

  mutable std::mutex mutex_;
  std::atomic_bool want_connected_{false};
  std::shared_ptr<Attachment> attachment_;
  std::thread session_thread_;
};

} // namespace vds::usbip
