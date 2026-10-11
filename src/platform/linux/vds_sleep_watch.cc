// SPDX-License-Identifier: MIT
#include "vds_sleep_watch.hh"

#include <dbus/dbus.h>
#include <unistd.h>

#include <stdexcept>
#include <string>

namespace vds {

namespace {

constexpr const char *kLogindService = "org.freedesktop.login1";
constexpr const char *kLogindPath = "/org/freedesktop/login1";
constexpr const char *kLogindManager = "org.freedesktop.login1.Manager";
constexpr int kInhibitTimeoutMs = 2000;

std::string take_error(DBusError &error) {
  std::string message = error.message != nullptr ? error.message : "unknown";
  ::dbus_error_free(&error);
  return message;
}

} // namespace

SleepWatcher::SleepWatcher() {
  DBusError error;
  ::dbus_error_init(&error);
  connection_ = ::dbus_bus_get_private(DBUS_BUS_SYSTEM, &error);
  if (connection_ == nullptr) {
    throw std::runtime_error("system bus: " + take_error(error));
  }
  ::dbus_connection_set_exit_on_disconnect(connection_, false);

  ::dbus_bus_add_match(connection_,
                       "type='signal',sender='org.freedesktop.login1',"
                       "path='/org/freedesktop/login1',"
                       "interface='org.freedesktop.login1.Manager',"
                       "member='PrepareForSleep'",
                       &error);
  if (::dbus_error_is_set(&error)) {
    const std::string message = take_error(error);
    ::dbus_connection_close(connection_);
    ::dbus_connection_unref(connection_);
    throw std::runtime_error("PrepareForSleep match: " + message);
  }
  ::dbus_connection_flush(connection_);
  if (!::dbus_connection_get_unix_fd(connection_, &fd_)) {
    ::dbus_connection_close(connection_);
    ::dbus_connection_unref(connection_);
    throw std::runtime_error("system bus has no unix fd");
  }
}

SleepWatcher::~SleepWatcher() {
  release_inhibitor();
  if (connection_ != nullptr) {
    ::dbus_connection_close(connection_);
    ::dbus_connection_unref(connection_);
  }
}

std::vector<bool> SleepWatcher::drain() {
  std::vector<bool> events;
  ::dbus_connection_read_write(connection_, 0);
  while (DBusMessage *message = ::dbus_connection_pop_message(connection_)) {
    if (::dbus_message_is_signal(message, kLogindManager, "PrepareForSleep")) {
      dbus_bool_t start = FALSE;
      DBusError error;
      ::dbus_error_init(&error);
      if (::dbus_message_get_args(message, &error, DBUS_TYPE_BOOLEAN, &start,
                                  DBUS_TYPE_INVALID)) {
        events.push_back(start != FALSE);
      } else {
        ::dbus_error_free(&error);
      }
    }
    ::dbus_message_unref(message);
  }
  return events;
}

void SleepWatcher::acquire_inhibitor() {
  if (inhibit_fd_ >= 0) {
    return;
  }
  DBusMessage *call = ::dbus_message_new_method_call(
      kLogindService, kLogindPath, kLogindManager, "Inhibit");
  if (call == nullptr) {
    throw std::runtime_error("failed to allocate logind Inhibit call");
  }
  const char *what = "sleep";
  const char *who = "vdsd";
  const char *why = "Turn the DualSense off before the PC sleeps";
  const char *mode = "delay";
  ::dbus_message_append_args(call, DBUS_TYPE_STRING, &what, DBUS_TYPE_STRING,
                             &who, DBUS_TYPE_STRING, &why, DBUS_TYPE_STRING,
                             &mode, DBUS_TYPE_INVALID);
  DBusError error;
  ::dbus_error_init(&error);
  DBusMessage *reply = ::dbus_connection_send_with_reply_and_block(
      connection_, call, kInhibitTimeoutMs, &error);
  ::dbus_message_unref(call);
  if (reply == nullptr) {
    throw std::runtime_error("logind Inhibit: " + take_error(error));
  }
  int fd = -1;
  const bool ok = ::dbus_message_get_args(reply, &error, DBUS_TYPE_UNIX_FD,
                                          &fd, DBUS_TYPE_INVALID);
  ::dbus_message_unref(reply);
  if (!ok) {
    throw std::runtime_error("logind Inhibit reply: " + take_error(error));
  }
  inhibit_fd_ = fd;
}

void SleepWatcher::release_inhibitor() {
  if (inhibit_fd_ >= 0) {
    ::close(inhibit_fd_);
    inhibit_fd_ = -1;
  }
}

} // namespace vds
