// SPDX-License-Identifier: MIT
//
// Drives vds::usbip::Device the way the Linux vhci_hcd driver does: USB/IP
// CMD_SUBMIT / CMD_UNLINK PDUs over a stream socket, checking replies, the
// frames sent to vdsd, and completion timing against the vds_hcd behaviour.

#include <array>
#include <atomic>
#include <cassert>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <map>
#include <mutex>
#include <optional>
#include <span>
#include <thread>
#include <utility>
#include <vector>

#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include "uapi/vds.h"
#include "usbip/vds_usbip_device.hh"

using namespace vds::usbip;
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;

namespace {

struct CapturedFrame {
  std::uint16_t type;
  std::vector<std::uint8_t> payload;
};

class FakeChannel final : public DaemonChannel {
public:
  bool write_frame(std::uint16_t type,
                   std::span<const std::uint8_t> payload) override {
    std::lock_guard lock(mutex);
    frames.push_back({type, {payload.begin(), payload.end()}});
    return true;
  }
  std::vector<CapturedFrame> take() {
    std::lock_guard lock(mutex);
    return std::exchange(frames, {});
  }
  std::mutex mutex;
  std::vector<CapturedFrame> frames;
};

class FdTransport final : public Transport {
public:
  explicit FdTransport(int fd) : fd_(fd) {}
  bool recv_exact(void *buffer, std::size_t size) override {
    auto *p = static_cast<std::uint8_t *>(buffer);
    while (size) {
      const ssize_t n = ::recv(fd_, p, size, 0);
      if (n <= 0) {
        return false;
      }
      p += n;
      size -= static_cast<std::size_t>(n);
    }
    return true;
  }
  bool send_exact(const void *buffer, std::size_t size) override {
    const auto *p = static_cast<const std::uint8_t *>(buffer);
    while (size) {
      const ssize_t n = ::send(fd_, p, size, MSG_NOSIGNAL);
      if (n <= 0) {
        return false;
      }
      p += n;
      size -= static_cast<std::size_t>(n);
    }
    return true;
  }

private:
  int fd_;
};

std::int32_t be(std::int32_t v) {
  return static_cast<std::int32_t>(to_be32(static_cast<std::uint32_t>(v)));
}
std::int32_t unbe(std::int32_t v) {
  return static_cast<std::int32_t>(from_be32(static_cast<std::uint32_t>(v)));
}

struct Reply {
  bool unlink = false;
  std::uint32_t seqnum = 0;
  std::int32_t status = 0;
  std::int32_t actual_length = 0;
  std::int32_t number_of_packets = 0;
  std::vector<std::uint8_t> data;
  std::vector<IsoPacketDescriptor> iso;
  Clock::time_point at;
};

// The vhci side of the connection.
class Host {
public:
  explicit Host(int fd) : fd_(fd) {}

  std::uint32_t submit(std::uint32_t ep, bool in, std::int32_t length,
                       const std::uint8_t *setup = nullptr,
                       std::vector<std::uint8_t> out = {},
                       std::vector<std::uint32_t> iso_lengths = {}) {
    const std::uint32_t seq = next_seq_++;
    CmdSubmit cmd{};
    cmd.base.command = to_be32(kCmdSubmit);
    cmd.base.seqnum = to_be32(seq);
    cmd.base.devid = to_be32(0x10001);
    cmd.base.direction = to_be32(in ? kDirIn : kDirOut);
    cmd.base.ep = to_be32(ep);
    cmd.transfer_buffer_length = be(length);
    cmd.number_of_packets = be(static_cast<std::int32_t>(iso_lengths.size()));
    if (setup != nullptr) {
      std::memcpy(cmd.setup, setup, 8);
    }
    send(&cmd, sizeof(cmd));
    if (!in && !out.empty()) {
      send(out.data(), out.size());
    }
    std::uint32_t offset = 0;
    for (const std::uint32_t len : iso_lengths) {
      IsoPacketDescriptor d{};
      d.offset = to_be32(offset);
      d.length = to_be32(len);
      send(&d, sizeof(d));
      offset += len;
    }
    in_flight_[seq] = {in, static_cast<std::int32_t>(iso_lengths.size())};
    return seq;
  }

  std::uint32_t control(const std::array<std::uint8_t, 8> &setup,
                        std::vector<std::uint8_t> out = {}) {
    const bool in = (setup[0] & 0x80) != 0;
    const std::int32_t length =
        in ? (setup[6] | (setup[7] << 8)) : static_cast<std::int32_t>(out.size());
    return submit(0, in, length, setup.data(), std::move(out));
  }

  std::uint32_t unlink(std::uint32_t target) {
    const std::uint32_t seq = next_seq_++;
    CmdUnlink cmd{};
    cmd.base.command = to_be32(kCmdUnlink);
    cmd.base.seqnum = to_be32(seq);
    cmd.unlink_seqnum = to_be32(target);
    send(&cmd, sizeof(cmd));
    return seq;
  }

  std::optional<Reply> recv(std::chrono::milliseconds timeout) {
    pollfd pfd{fd_, POLLIN, 0};
    if (::poll(&pfd, 1, static_cast<int>(timeout.count())) <= 0) {
      return std::nullopt;
    }
    RetSubmit ret{};
    recv_exact(&ret, sizeof(ret));
    Reply reply;
    reply.at = Clock::now();
    reply.seqnum = from_be32(ret.base.seqnum);
    const std::uint32_t command = from_be32(ret.base.command);
    if (command == kRetUnlink) {
      RetUnlink unlink{};
      std::memcpy(&unlink, &ret, sizeof(unlink));
      reply.unlink = true;
      reply.status = unbe(unlink.status);
      return reply;
    }
    assert(command == kRetSubmit);
    reply.status = unbe(ret.status);
    reply.actual_length = unbe(ret.actual_length);
    reply.number_of_packets = unbe(ret.number_of_packets);
    const auto flight = in_flight_.at(reply.seqnum);
    in_flight_.erase(reply.seqnum);
    if (flight.in && reply.actual_length > 0) {
      reply.data.resize(static_cast<std::size_t>(reply.actual_length));
      recv_exact(reply.data.data(), reply.data.size());
    }
    if (flight.packets > 0) {
      assert(reply.number_of_packets == flight.packets);
      reply.iso.resize(static_cast<std::size_t>(flight.packets));
      recv_exact(reply.iso.data(), reply.iso.size() * sizeof(IsoPacketDescriptor));
      for (auto &d : reply.iso) {
        d.offset = from_be32(d.offset);
        d.length = from_be32(d.length);
        d.actual_length = unbe(d.actual_length);
        d.status = unbe(d.status);
      }
    }
    return reply;
  }

  Reply expect(std::uint32_t seq, std::chrono::milliseconds timeout = 500ms) {
    auto reply = recv(timeout);
    assert(reply && "expected a reply");
    if (reply->seqnum != seq) {
      std::fprintf(stderr, "expected seq %u got %u\n", seq, reply->seqnum);
    }
    assert(reply->seqnum == seq);
    return *reply;
  }

  void expect_silence(std::chrono::milliseconds timeout) {
    assert(!recv(timeout) && "unexpected reply");
  }

private:
  void send(const void *p, std::size_t n) {
    const ssize_t sent = ::send(fd_, p, n, MSG_NOSIGNAL);
    assert(sent == static_cast<ssize_t>(n));
  }
  void recv_exact(void *p, std::size_t n) {
    auto *c = static_cast<std::uint8_t *>(p);
    while (n) {
      const ssize_t got = ::recv(fd_, c, n, 0);
      assert(got > 0);
      c += got;
      n -= static_cast<std::size_t>(got);
    }
  }

  struct Flight {
    bool in;
    std::int32_t packets;
  };
  int fd_;
  std::uint32_t next_seq_ = 1;
  std::map<std::uint32_t, Flight> in_flight_;
};

std::array<std::uint8_t, 8> setup(std::uint8_t type, std::uint8_t request,
                                  std::uint16_t value, std::uint16_t index,
                                  std::uint16_t length) {
  return {type,
          request,
          static_cast<std::uint8_t>(value & 0xff),
          static_cast<std::uint8_t>(value >> 8),
          static_cast<std::uint8_t>(index & 0xff),
          static_cast<std::uint8_t>(index >> 8),
          static_cast<std::uint8_t>(length & 0xff),
          static_cast<std::uint8_t>(length >> 8)};
}

double ms_between(Clock::time_point a, Clock::time_point b) {
  return std::chrono::duration<double, std::milli>(b - a).count();
}

struct Fixture {
  Fixture() {
    int fds[2];
    const int rc = ::socketpair(AF_UNIX, SOCK_STREAM, 0, fds);
    assert(rc == 0);
    host_fd = fds[0];
    device_fd = fds[1];
    transport.emplace(device_fd);
    session = std::thread([this] { device.run_session(*transport); });
  }
  ~Fixture() {
    ::shutdown(host_fd, SHUT_RDWR);
    ::shutdown(device_fd, SHUT_RDWR);
    device.stop_session();
    session.join();
    ::close(host_fd);
    ::close(device_fd);
  }
  FakeChannel channel;
  Device device{channel, nullptr, "test"};
  int host_fd = -1;
  int device_fd = -1;
  std::optional<FdTransport> transport;
  std::thread session;
};

void test_enumeration() {
  Fixture f;
  Host host(f.host_fd);

  // Device descriptor, short read first (like usbcore).
  auto r = host.expect(host.control(setup(0x80, 0x06, 0x0100, 0, 8)));
  assert(r.status == 0 && r.data.size() == 8 && r.data[1] == 0x01);
  r = host.expect(host.control(setup(0x80, 0x06, 0x0100, 0, 18)));
  assert(r.status == 0 && r.data.size() == 18);
  assert(r.data[8] == 0x4c && r.data[9] == 0x05);  // 054c
  assert(r.data[10] == 0xe6 && r.data[11] == 0x0c); // 0ce6

  r = host.expect(host.control(setup(0x80, 0x06, 0x0200, 0, 9)));
  assert(r.status == 0 && r.data.size() == 9 && r.data[1] == 0x02);
  const std::uint16_t total = r.data[2] | (r.data[3] << 8);
  r = host.expect(host.control(setup(0x80, 0x06, 0x0200, 0, 0xffff)));
  assert(r.status == 0 && r.data.size() == total);

  r = host.expect(host.control(setup(0x80, 0x06, 0x0302, 0x0409, 255)));
  assert(r.status == 0 && r.data[1] == 0x03 && r.data[2] == 'D');
  r = host.expect(host.control(setup(0x80, 0x06, 0x0303, 0x0409, 255)));
  assert(r.status == kStatusEpipe && r.data.empty());

  r = host.expect(host.control(setup(0x81, 0x06, 0x2200, 3, 0xffff)));
  assert(r.status == 0 && r.data.size() > 100 && r.data[0] == 0x05);

  r = host.expect(host.control(setup(0x00, 0x09, 1, 0, 0)));
  assert(r.status == 0);
  r = host.expect(host.control(setup(0x80, 0x08, 0, 0, 1)));
  assert(r.status == 0 && r.data == std::vector<std::uint8_t>{1});

  // HID SET_IDLE is answered (vds_hcd parity), not stalled.
  r = host.expect(host.control(setup(0x21, 0x0a, 0, 3, 0)));
  assert(r.status == 0);

  (void)f.channel.take();
  r = host.expect(host.control(setup(0x01, 0x0b, 1, 1, 0))); // audio OUT alt 1
  assert(r.status == 0);
  auto frames = f.channel.take();
  assert(frames.size() == 1 && frames[0].type == VDS_FRAME_USB_INTERFACE);
  assert(frames[0].payload[0] == 1 && frames[0].payload[1] == 1 &&
         frames[0].payload[2] == VDS_USB_INTERFACE_AUDIO_OUT);

  r = host.expect(host.control(setup(0xc0, 0x01, 0, 0, 4))); // vendor
  assert(r.status == kStatusEpipe);

  // UAC volume: GET_MIN on the speaker feature unit (entity 2, intf 0).
  r = host.expect(host.control(setup(0xa1, 0x82, 0x0200, 0x0200, 2)));
  assert(r.status == 0 && r.data.size() == 2 && r.data[1] == 0x9c);
  std::puts("enumeration ok");
}

void test_hid_in() {
  Fixture f;
  Host host(f.host_fd);
  // No input yet: the URB stays pending.
  const auto seq = host.submit(4, true, 64);
  host.expect_silence(30ms);
  const std::vector<std::uint8_t> report(64, 0x01);
  const auto sent_at = Clock::now();
  f.device.handle_daemon_frame(VDS_FRAME_USB_HID_IN, report);
  auto r = host.expect(seq);
  assert(r.status == 0 && r.data == report);
  // Completion is paced by bInterval (4 ms for the DualSense).
  assert(ms_between(sent_at, r.at) >= 3.5);

  // A buffered report keeps only the newest snapshot.
  std::vector<std::uint8_t> older(64, 0x02), newer(64, 0x03);
  f.device.handle_daemon_frame(VDS_FRAME_USB_HID_IN, older);
  f.device.handle_daemon_frame(VDS_FRAME_USB_HID_IN, newer);
  const auto seq2 = host.submit(4, true, 64);
  auto r2 = host.expect(seq2);
  assert(r2.data == newer);
  assert(ms_between(r.at, r2.at) >= 3.5);
  std::puts("hid in ok");
}

void test_feature_reports() {
  Fixture f;
  Host host(f.host_fd);
  const auto seq = host.control(setup(0xa1, 0x01, 0x0305, 3, 41));
  host.expect_silence(20ms);
  auto frames = f.channel.take();
  assert(frames.size() == 1 && frames[0].type == VDS_FRAME_USB_FEATURE_GET);
  assert((frames[0].payload == std::vector<std::uint8_t>{0x05, 41, 0}));

  std::vector<std::uint8_t> reply(64, 0xaa);
  reply[0] = 0x05;
  f.device.handle_daemon_frame(VDS_FRAME_USB_FEATURE_REPLY, reply);
  auto r = host.expect(seq);
  assert(r.status == 0 && r.data.size() == 41 && r.data[0] == 0x05);

  // Cache hit: answered without asking vdsd again.
  r = host.expect(host.control(setup(0xa1, 0x01, 0x0305, 3, 64)));
  assert(r.status == 0 && r.data.size() == 64);
  assert(f.channel.take().empty());

  // SET_REPORT (feature) is forwarded with the report id prefixed, like the
  // kernel module.
  r = host.expect(host.control(setup(0x21, 0x09, 0x0380, 3, 3), {0x80, 1, 2}));
  assert(r.status == 0 && r.actual_length == 3);
  frames = f.channel.take();
  assert(frames.size() == 1 && frames[0].type == VDS_FRAME_USB_FEATURE_SET);
  assert((frames[0].payload == std::vector<std::uint8_t>{0x80, 0x80, 1, 2}));
  std::puts("feature reports ok");
}

void test_iso_out_pacing_and_ordering() {
  Fixture f;
  Host host(f.host_fd);
  const std::vector<std::uint32_t> packets(10, 384); // 10 ms of 4ch PCM
  std::vector<std::uint8_t> pcm(3840);
  for (std::size_t i = 0; i < pcm.size(); ++i) {
    pcm[i] = static_cast<std::uint8_t>(i);
  }
  const auto start = Clock::now();
  const auto a = host.submit(1, false, 3840, nullptr, pcm, packets);
  const auto b = host.submit(1, false, 3840, nullptr, pcm, packets);
  const auto c = host.submit(1, false, 3840, nullptr, pcm, packets);
  // A control request submitted behind the audio completes first.
  const auto ctrl = host.control(setup(0x80, 0x00, 0, 0, 2));
  auto r = host.expect(ctrl);
  assert(r.status == 0);
  assert(ms_between(start, r.at) < 5.0);

  auto ra = host.expect(a, 100ms);
  auto rb = host.expect(b, 100ms);
  auto rc = host.expect(c, 100ms);
  for (const Reply *x : {&ra, &rb, &rc}) {
    assert(x->status == 0 && x->actual_length == 3840);
    assert(x->iso.size() == 10);
    for (const auto &d : x->iso) {
      assert(d.status == 0 && d.actual_length == 384);
    }
  }
  const double ta = ms_between(start, ra.at), tb = ms_between(start, rb.at),
               tc = ms_between(start, rc.at);
  std::printf("iso out completions at %.2f / %.2f / %.2f ms\n", ta, tb, tc);
  assert(ta >= 9.5 && ta < 15.0);
  assert(tb >= 19.5 && tb < 25.0);
  assert(tc >= 29.5 && tc < 35.0);

  // PCM reaches vdsd at completion time, one frame per contiguous URB.
  std::size_t audio_frames = 0;
  for (const auto &frame : f.channel.take()) {
    if (frame.type == VDS_FRAME_USB_AUDIO_OUT) {
      assert(frame.payload == pcm);
      ++audio_frames;
    }
  }
  assert(audio_frames == 3);
  std::puts("iso out ok");
}

void test_iso_in_packing() {
  Fixture f;
  Host host(f.host_fd);
  std::vector<std::uint8_t> mic(192 * 5, 0x5a);
  f.device.handle_daemon_frame(VDS_FRAME_USB_AUDIO_IN, mic);
  const std::vector<std::uint32_t> packets(10, 200); // larger than 192
  const auto seq = host.submit(2, true, 2000, nullptr, {}, packets);
  auto r = host.expect(seq, 100ms);
  assert(r.status == 0 && r.actual_length == 1920);
  assert(r.data.size() == 1920);
  assert(r.data[0] == 0x5a && r.data[959] == 0x5a && r.data[960] == 0);
  for (const auto &d : r.iso) {
    assert(d.actual_length == 192 && d.length == 200);
  }
  std::puts("iso in ok");
}

void test_unlink() {
  Fixture f;
  Host host(f.host_fd);
  // Pending interrupt IN: unlinked, no RET_SUBMIT may follow.
  const auto pending = host.submit(4, true, 64);
  const auto u1 = host.unlink(pending);
  auto r = host.expect(u1);
  assert(r.unlink && r.status == kStatusEconnreset);
  f.device.handle_daemon_frame(VDS_FRAME_USB_HID_IN,
                               std::vector<std::uint8_t>(64, 1));
  host.expect_silence(30ms);

  // Timed iso OUT still queued: unlinked before completion.
  const std::vector<std::uint32_t> packets(10, 384);
  const auto iso = host.submit(1, false, 3840, nullptr,
                               std::vector<std::uint8_t>(3840), packets);
  const auto u2 = host.unlink(iso);
  r = host.expect(u2);
  assert(r.unlink && r.status == kStatusEconnreset);
  host.expect_silence(30ms);

  // Already completed: status 0, and the RET_SUBMIT came first.
  const auto done = host.control(setup(0x80, 0x00, 0, 0, 2));
  r = host.expect(done);
  const auto u3 = host.unlink(done);
  r = host.expect(u3);
  assert(r.unlink && r.status == 0);
  std::puts("unlink ok");
}

void test_profiles() {
  FakeChannel channel;
  Device device(channel, nullptr, "test");
  assert(device.set_profile(VDS_PROFILE_X360));
  assert(!device.set_profile(99));
  int fds[2];
  const int rc = ::socketpair(AF_UNIX, SOCK_STREAM, 0, fds);
  assert(rc == 0);
  FdTransport transport(fds[1]);
  std::thread session([&] { device.run_session(transport); });
  Host host(fds[0]);
  auto r = host.expect(host.control(setup(0x80, 0x06, 0x0100, 0, 18)));
  assert(r.data[8] == 0x5e && r.data[9] == 0x04 && r.data[7] == 0x08);
  // No profile change while a session is up.
  assert(!device.set_profile(VDS_PROFILE_DS5));
  // Xbox 360 report pipe: 1 IN.
  const auto seq = host.submit(1, true, 32);
  device.handle_daemon_frame(VDS_FRAME_USB_HID_IN,
                             std::vector<std::uint8_t>(20, 7));
  r = host.expect(seq);
  assert(r.data.size() == 20);
  ::shutdown(fds[0], SHUT_RDWR);
  session.join();
  ::close(fds[0]);
  ::close(fds[1]);
  assert(device.set_profile(VDS_PROFILE_DS5));
  std::puts("profiles ok");
}

} // namespace

int main() {
  test_enumeration();
  test_hid_in();
  test_feature_reports();
  test_iso_out_pacing_and_ordering();
  test_iso_in_packing();
  test_unlink();
  test_profiles();
  std::puts("usbip_device_test: all passed");
  return 0;
}
