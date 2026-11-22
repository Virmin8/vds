// SPDX-License-Identifier: MIT

#include "vds_usbip_vhci.hh"

#include <array>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <spawn.h>
#include <sys/prctl.h>
#include <sys/resource.h>
#include <sys/socket.h>
#include <sys/syscall.h>
#include <sys/wait.h>
#include <unistd.h>

#include "uapi/vds.h"
#include "usbip/vds_usbip_device.hh"
#include "vds_log.hh"

extern char **environ;

namespace vds::usbip {

namespace {

namespace fs = std::filesystem;

const fs::path kVhciPath = "/sys/devices/platform/vhci_hcd.0";
constexpr unsigned kVdevStatusNull = 4; // VDEV_ST_NULL: port free
constexpr unsigned kUsbSpeedHigh = 3;   // enum usb_device_speed
constexpr auto kReattachDelay = std::chrono::seconds(1);

std::string errno_text(int error) { return std::strerror(error); }

std::string read_trimmed(const fs::path &path) {
  std::ifstream in(path);
  std::string text;
  std::getline(in, text);
  while (!text.empty() && (text.back() == '\n' || text.back() == ' ')) {
    text.pop_back();
  }
  return text;
}

// Writes `text` to a sysfs attribute in one write(); returns 0 or errno.
int write_sysfs(const fs::path &path, const std::string &text) {
  UniqueFd fd(::open(path.c_str(), O_WRONLY | O_CLOEXEC));
  if (!fd) {
    return errno;
  }
  const ssize_t written = ::write(fd.get(), text.data(), text.size());
  if (written < 0) {
    return errno;
  }
  return written == static_cast<ssize_t>(text.size()) ? 0 : EIO;
}

void try_load_vhci_module() {
  if (fs::exists(kVhciPath)) {
    return;
  }
  char arg0[] = "modprobe";
  char arg1[] = "-q";
  char arg2[] = "vhci-hcd";
  char *argv[] = {arg0, arg1, arg2, nullptr};
  pid_t pid = -1;
  if (::posix_spawnp(&pid, "modprobe", nullptr, nullptr, argv, environ) != 0) {
    return;
  }
  int status = 0;
  while (::waitpid(pid, &status, 0) < 0 && errno == EINTR) {
  }
}

// Free high-speed vhci port numbers, as listed in vhci_hcd.0/status:
//   hub port sta spd dev      sockfd local_busid
//   hs  0000 004 000 00000000 000000 0-0
std::vector<unsigned> free_hs_ports() {
  std::ifstream in(kVhciPath / "status");
  if (!in) {
    throw std::runtime_error("cannot read " + (kVhciPath / "status").string());
  }
  std::vector<unsigned> ports;
  std::string line;
  while (std::getline(in, line)) {
    std::istringstream fields(line);
    std::string hub, port, status;
    if (!(fields >> hub >> port >> status) || hub != "hs") {
      continue;
    }
    try {
      if (std::stoul(status) == kVdevStatusNull) {
        ports.push_back(static_cast<unsigned>(std::stoul(port)));
      }
    } catch (const std::exception &) {
    }
  }
  return ports;
}

bool hs_port_free(unsigned port) {
  for (const unsigned candidate : free_hs_ports()) {
    if (candidate == port) {
      return true;
    }
  }
  return false;
}

// True while vhci port `port` is occupied by the device we attached with
// `devid` -- so a detach never unplugs whatever reused a freed port.
bool hs_port_owned(unsigned port, unsigned devid) {
  std::ifstream in(kVhciPath / "status");
  std::string line;
  while (std::getline(in, line)) {
    std::istringstream fields(line);
    std::string hub, port_text, status, speed, dev;
    if (!(fields >> hub >> port_text >> status >> speed >> dev) ||
        hub != "hs") {
      continue;
    }
    try {
      if (std::stoul(port_text) != port) {
        continue;
      }
      return std::stoul(status) != kVdevStatusNull &&
             std::stoul(dev, nullptr, 16) == devid;
    } catch (const std::exception &) {
      return false;
    }
  }
  return false;
}

// Bus number of vhci_hcd.0's USB 2.0 root hub.
std::optional<unsigned> hs_bus_number() {
  std::error_code error;
  for (const auto &entry : fs::directory_iterator(kVhciPath, error)) {
    const std::string name = entry.path().filename().string();
    if (name.rfind("usb", 0) != 0 || name.size() == 3) {
      continue;
    }
    if (read_trimmed(entry.path() / "speed") != "480") {
      continue;
    }
    try {
      return static_cast<unsigned>(std::stoul(name.substr(3)));
    } catch (const std::exception &) {
    }
  }
  return std::nullopt;
}

// A connected loopback TCP pair; fallback for kernels that refuse a UNIX
// stream socket in vhci attach.
std::pair<UniqueFd, UniqueFd> tcp_loopback_pair() {
  UniqueFd listener(::socket(AF_INET, SOCK_STREAM | SOCK_CLOEXEC, 0));
  if (!listener) {
    throw std::runtime_error("socket: " + errno_text(errno));
  }
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  address.sin_port = 0;
  socklen_t length = sizeof(address);
  if (::bind(listener.get(), reinterpret_cast<sockaddr *>(&address),
             sizeof(address)) < 0 ||
      ::listen(listener.get(), 1) < 0 ||
      ::getsockname(listener.get(), reinterpret_cast<sockaddr *>(&address),
                    &length) < 0) {
    throw std::runtime_error("loopback listener: " + errno_text(errno));
  }
  UniqueFd client(::socket(AF_INET, SOCK_STREAM | SOCK_CLOEXEC, 0));
  if (!client || ::connect(client.get(), reinterpret_cast<sockaddr *>(&address),
                           sizeof(address)) < 0) {
    throw std::runtime_error("loopback connect: " + errno_text(errno));
  }
  UniqueFd server(::accept4(listener.get(), nullptr, nullptr, SOCK_CLOEXEC));
  if (!server) {
    throw std::runtime_error("loopback accept: " + errno_text(errno));
  }
  const int one = 1;
  (void)::setsockopt(client.get(), IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
  (void)::setsockopt(server.get(), IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
  return {std::move(client), std::move(server)};
}

std::pair<UniqueFd, UniqueFd> unix_stream_pair() {
  int fds[2];
  if (::socketpair(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0, fds) < 0) {
    throw std::runtime_error("socketpair: " + errno_text(errno));
  }
  return {UniqueFd(fds[0]), UniqueFd(fds[1])};
}

class FdTransport final : public Transport {
public:
  explicit FdTransport(int fd) : fd_(fd) {}

  bool recv_exact(void *buffer, std::size_t size) override {
    auto *cursor = static_cast<std::uint8_t *>(buffer);
    while (size > 0) {
      const ssize_t got = ::recv(fd_, cursor, size, 0);
      if (got < 0 && errno == EINTR) {
        continue;
      }
      if (got <= 0) {
        return false;
      }
      cursor += got;
      size -= static_cast<std::size_t>(got);
    }
    return true;
  }

  bool send_exact(const void *buffer, std::size_t size) override {
    const auto *cursor = static_cast<const std::uint8_t *>(buffer);
    while (size > 0) {
      const ssize_t sent = ::send(fd_, cursor, size, MSG_NOSIGNAL);
      if (sent < 0 && errno == EINTR) {
        continue;
      }
      if (sent <= 0) {
        return false;
      }
      cursor += sent;
      size -= static_cast<std::size_t>(sent);
    }
    return true;
  }

private:
  int fd_;
};

// Pacing thread: the vds_hcd giveback kthread used 50us hrtimer slack; get
// close to that and ahead of ordinary desktop load without going realtime.
void tune_completion_thread() {
  (void)::prctl(PR_SET_TIMERSLACK, 50'000UL, 0, 0, 0);
  (void)::setpriority(PRIO_PROCESS, static_cast<id_t>(::syscall(SYS_gettid)),
                      -10);
}

} // namespace

bool vhci_available(std::string *reason) {
  try_load_vhci_module();
  if (!fs::exists(kVhciPath / "attach")) {
    if (reason != nullptr) {
      *reason = "vhci_hcd is not loaded (modprobe vhci-hcd)";
    }
    return false;
  }
  return true;
}

// ---- Frame channel to vdsd --------------------------------------------------

struct VhciPort::Channel final : DaemonChannel {
  explicit Channel(int fd_in) : fd(fd_in) {}

  bool write_frame(std::uint16_t type,
                   std::span<const std::uint8_t> payload) override {
    if (payload.size() > VDS_FRAME_MAX_PAYLOAD) {
      return false;
    }
    std::array<std::uint8_t, sizeof(vds_frame_header) + VDS_FRAME_MAX_PAYLOAD>
        buffer;
    vds_frame_header header{};
    header.type = type;
    header.flags = 0;
    header.length = static_cast<std::uint32_t>(payload.size());
    header.sequence = sequence.fetch_add(1);
    std::memcpy(buffer.data(), &header, sizeof(header));
    if (!payload.empty()) {
      std::memcpy(buffer.data() + sizeof(header), payload.data(),
                  payload.size());
    }
    const std::size_t size = sizeof(header) + payload.size();
    for (;;) {
      const ssize_t sent =
          ::send(fd, buffer.data(), size, MSG_DONTWAIT | MSG_NOSIGNAL);
      if (sent < 0 && errno == EINTR) {
        continue;
      }
      // Like the kernel's bounded event queue: drop when vdsd is behind.
      return sent == static_cast<ssize_t>(size);
    }
  }

  int fd;
  std::atomic<std::uint64_t> sequence{0};
};

struct VhciPort::Attachment {
  unsigned vhci_port = 0;
  unsigned devid = 0;
  std::string marker;
  UniqueFd fd; // our end of the USB/IP stream

  // Unplug (if vhci still has our device on that port) and end the stream.
  void detach() noexcept {
    try {
      if (hs_port_owned(vhci_port, devid)) {
        (void)write_sysfs(kVhciPath / "detach", std::to_string(vhci_port));
      }
    } catch (...) {
    }
    if (fd) {
      (void)::shutdown(fd.get(), SHUT_RDWR);
    }
  }
};

// ---- VhciPort ---------------------------------------------------------------

VhciPort::VhciPort(unsigned port_index, Logger &logger)
    : port_index_(port_index), logger_(logger),
      label_("usbip port " + std::to_string(port_index)) {
  int fds[2];
  if (::socketpair(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC, 0, fds) < 0) {
    throw std::runtime_error("socketpair for " + label_ + ": " +
                             errno_text(errno));
  }
  daemon_fd_.reset(fds[0]);
  core_fd_.reset(fds[1]);
  const int flags = ::fcntl(daemon_fd_.get(), F_GETFL, 0);
  if (flags < 0 ||
      ::fcntl(daemon_fd_.get(), F_SETFL, flags | O_NONBLOCK) < 0) {
    throw std::runtime_error("fcntl for " + label_ + ": " + errno_text(errno));
  }
  const int buffer = 1 << 20;
  (void)::setsockopt(daemon_fd_.get(), SOL_SOCKET, SO_SNDBUF, &buffer,
                     sizeof(buffer));
  (void)::setsockopt(core_fd_.get(), SOL_SOCKET, SO_SNDBUF, &buffer,
                     sizeof(buffer));

  channel_ = std::make_unique<Channel>(core_fd_.get());
  device_ = std::make_unique<Device>(*channel_, &logger_, label_);
  device_->set_hooks({.on_completion_thread_start = tune_completion_thread});
  reader_thread_ = std::thread([this] { reader_loop(); });
}

VhciPort::~VhciPort() {
  disconnect();
  if (core_fd_) {
    (void)::shutdown(core_fd_.get(), SHUT_RDWR);
  }
  if (reader_thread_.joinable()) {
    reader_thread_.join();
  }
}

UniqueFd VhciPort::take_daemon_fd() { return std::move(daemon_fd_); }

void VhciPort::set_profile(std::uint32_t profile) {
  if (connected()) {
    throw std::runtime_error(label_ + " set profile while connected");
  }
  if (!device_->set_profile(profile)) {
    throw std::runtime_error(label_ + " rejected profile " +
                             std::to_string(profile));
  }
}

bool VhciPort::connected() const {
  std::lock_guard lock(mutex_);
  return want_connected_.load() && session_thread_.joinable();
}

std::string VhciPort::sysfs_marker() const {
  std::lock_guard lock(mutex_);
  return attachment_ ? attachment_->marker : std::string{};
}

void VhciPort::reader_loop() {
  std::array<std::uint8_t, sizeof(vds_frame_header) + VDS_FRAME_MAX_PAYLOAD>
      buffer;
  for (;;) {
    const ssize_t got = ::recv(core_fd_.get(), buffer.data(), buffer.size(), 0);
    if (got < 0 && errno == EINTR) {
      continue;
    }
    if (got <= 0) {
      return; // shut down
    }
    if (static_cast<std::size_t>(got) < sizeof(vds_frame_header)) {
      continue;
    }
    vds_frame_header header{};
    std::memcpy(&header, buffer.data(), sizeof(header));
    if (header.flags != 0 || header.length > VDS_FRAME_MAX_PAYLOAD ||
        static_cast<std::size_t>(got) != sizeof(header) + header.length) {
      continue;
    }
    device_->handle_daemon_frame(
        header.type, std::span(buffer.data() + sizeof(header), header.length));
  }
}

std::shared_ptr<VhciPort::Attachment> VhciPort::attach_once() {
  std::string reason;
  if (!vhci_available(&reason)) {
    throw std::runtime_error(reason);
  }
  const auto bus = hs_bus_number();
  if (!bus) {
    throw std::runtime_error("vhci_hcd.0 has no USB 2.0 root hub");
  }

  // UNIX stream socket first (nothing reachable from the network; vhci only
  // requires SOCK_STREAM), loopback TCP as a fallback. -EBUSY means another
  // USB/IP user took the port first: retry on the next free one.
  bool use_tcp = false;
  for (int attempt = 0; attempt < 8; ++attempt) {
    const std::vector<unsigned> free_ports = free_hs_ports();
    if (free_ports.empty()) {
      throw std::runtime_error("no free vhci_hcd high-speed port");
    }
    const unsigned vhci_port = free_ports.front();
    auto [kernel_end, our_end] =
        use_tcp ? tcp_loopback_pair() : unix_stream_pair();
    const unsigned devid = (1u << 16) | (port_index_ + 1);
    const std::string request = std::to_string(vhci_port) + " " +
                                std::to_string(kernel_end.get()) + " " +
                                std::to_string(devid) + " " +
                                std::to_string(kUsbSpeedHigh);
    const int error = write_sysfs(kVhciPath / "attach", request);
    if (error == 0) {
      // vhci_hcd holds its own reference to the socket now.
      kernel_end.reset();
      auto attachment = std::make_shared<Attachment>();
      attachment->vhci_port = vhci_port;
      attachment->devid = devid;
      attachment->marker = "/vhci_hcd.0/usb" + std::to_string(*bus) + "/" +
                           std::to_string(*bus) + "-" +
                           std::to_string(vhci_port + 1) + "/";
      attachment->fd = std::move(our_end);
      logger_.log(LogScope::Usb, LogLevel::Info,
                  label_ + " attached to vhci_hcd port=" +
                      std::to_string(vhci_port) +
                      " transport=" + (use_tcp ? "tcp-loopback" : "unix"));
      return attachment;
    }
    if (error == EBUSY) {
      continue;
    }
    if (!use_tcp && error == EINVAL && hs_port_free(vhci_port)) {
      logger_.log(LogScope::Usb, LogLevel::Info,
                  label_ + " vhci_hcd refused a UNIX socket; using loopback "
                           "TCP");
      use_tcp = true;
      continue;
    }
    throw std::runtime_error("vhci_hcd attach failed port=" +
                             std::to_string(vhci_port) + ": " +
                             errno_text(error));
  }
  throw std::runtime_error("vhci_hcd attach failed: ports kept getting taken");
}

void VhciPort::connect() {
  std::lock_guard lock(mutex_);
  if (want_connected_.load() && session_thread_.joinable()) {
    return;
  }
  if (session_thread_.joinable()) {
    session_thread_.join();
  }
  auto attachment = attach_once();
  attachment_ = attachment;
  want_connected_ = true;
  session_thread_ =
      std::thread([this, attachment] { session_loop(attachment); });
}

void VhciPort::session_loop(std::shared_ptr<Attachment> attachment) {
  for (;;) {
    FdTransport transport(attachment->fd.get());
    device_->run_session(transport);
    if (!want_connected_.load()) {
      return;
    }

    // The host side dropped the device (vhci error, or someone ran
    // `usbip detach`). vdsd still wants it plugged in: re-plug it, as the
    // kernel HCD would never spontaneously unplug.
    logger_.log(LogScope::Usb, LogLevel::Warn,
                label_ + " USB/IP connection lost; re-attaching");
    {
      std::lock_guard lock(mutex_);
      if (attachment_ == attachment) {
        attachment_.reset();
      }
    }
    attachment->detach();
    device_->reset_state();
    for (;;) {
      const auto resume_at = std::chrono::steady_clock::now() + kReattachDelay;
      while (std::chrono::steady_clock::now() < resume_at) {
        if (!want_connected_.load()) {
          return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
      try {
        auto next = attach_once();
        std::lock_guard lock(mutex_);
        if (!want_connected_.load()) {
          next->detach();
          return;
        }
        attachment_ = next;
        attachment = std::move(next);
        break;
      } catch (const std::exception &error) {
        logger_.log(LogScope::Usb, LogLevel::Error,
                    label_ + " re-attach failed: " + error.what());
      }
    }
  }
}

void VhciPort::disconnect() noexcept {
  std::shared_ptr<Attachment> attachment;
  std::thread session;
  {
    std::lock_guard lock(mutex_);
    want_connected_ = false;
    attachment = std::move(attachment_);
    session = std::move(session_thread_);
  }
  if (attachment) {
    attachment->detach();
  }
  device_->stop_session();
  if (session.joinable()) {
    session.join();
  }
  device_->reset_state();
  if (attachment) {
    try {
      logger_.log(LogScope::Usb, LogLevel::Info,
                  label_ + " detached from vhci_hcd port=" +
                      std::to_string(attachment->vhci_port));
    } catch (...) {
    }
  }
}

} // namespace vds::usbip
