// SPDX-License-Identifier: MIT
#pragma once

#include <vector>

struct DBusConnection;

namespace vds {

/*
 * Watches systemd-logind's PrepareForSleep signal on the system bus so vdsd
 * can act before the PC suspends ("PC Sleep Disconnect"). While a "delay"
 * sleep inhibitor is held, logind waits (up to InhibitDelayMaxSec) until it
 * is released, so the controller is turned off before the system sleeps.
 */
class SleepWatcher {
public:
  SleepWatcher(); // throws if the system bus or logind is unavailable
  ~SleepWatcher();

  SleepWatcher(const SleepWatcher &) = delete;
  SleepWatcher &operator=(const SleepWatcher &) = delete;

  // Readable when D-Bus traffic is pending; add it to epoll.
  int fd() const { return fd_; }

  // Reads pending messages; returns each PrepareForSleep value in order
  // (true: about to sleep, false: resumed).
  std::vector<bool> drain();

  void acquire_inhibitor(); // no-op if already held
  void release_inhibitor(); // no-op if not held
  bool inhibitor_held() const { return inhibit_fd_ >= 0; }

private:
  DBusConnection *connection_ = nullptr;
  int fd_ = -1;
  int inhibit_fd_ = -1;
};

} // namespace vds
