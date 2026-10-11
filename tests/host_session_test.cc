// SPDX-License-Identifier: MIT
#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>

#include <fcntl.h>
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

#include "../src/platform/linux/vds_host_watch.hh"
#include "../src/vds_protocol.hh"

namespace {

// Output report from the host: lightbar colour (flag1 bit 2) and a right
// trigger effect (flag0 bit 2).
std::array<std::uint8_t, 1 + VDS_SET_STATE_SIZE>
host_report(std::uint8_t red, std::uint8_t green, std::uint8_t blue,
            std::uint8_t trigger_mode) {
  std::array<std::uint8_t, 1 + VDS_SET_STATE_SIZE> report{};
  report[0] = VDS_USB_OUTPUT_REPORT_ID;
  report[1 + 0] = 0x04;
  report[1 + 1] = 0x04;
  report[1 + offsetof(vds_set_state_data, right_trigger_ffb)] = trigger_mode;
  report[1 + offsetof(vds_set_state_data, led_red)] = red;
  report[1 + offsetof(vds_set_state_data, led_red) + 1] = green;
  report[1 + offsetof(vds_set_state_data, led_red) + 2] = blue;
  return report;
}

} // namespace

int main() {
  // Snapshot before a "game", restore after: lights and triggers return,
  // speaker routing follows the stream as it is at restore time.
  {
    vds::DsOutputState state;
    assert(state.apply_usb_output_report(host_report(0, 0, 255, 0)));
    const auto before = state.state();
    const auto snapshot = state.host_state();

    state.set_audio_out_stream_active(true);
    assert(state.apply_usb_output_report(host_report(255, 0, 0, 0x21)));
    assert(state.state() != before);

    state.restore_host_state(snapshot);
    const std::size_t led = offsetof(vds_set_state_data, led_red);
    const std::size_t trigger = offsetof(vds_set_state_data, right_trigger_ffb);
    assert(state.state()[led] == before[led]);
    assert(state.state()[led + 2] == before[led + 2]);
    assert(state.state()[trigger] == before[trigger]);
    assert(state.state()[7] == 0x30); // speaker path still owned while streaming
  }

  // node_openers finds another process holding a file open, not ourselves.
  {
    char path[] = "/tmp/vds_host_session_testXXXXXX";
    const int fd = ::mkstemp(path);
    assert(fd >= 0);
    int ready[2];
    assert(::pipe(ready) == 0);
    const pid_t child = ::fork();
    if (child == 0) {
      const int held = ::open(path, O_RDONLY);
      (void)held;
      char byte = 1;
      (void)::write(ready[1], &byte, 1);
      ::pause();
      ::_exit(0);
    }
    char byte = 0;
    assert(::read(ready[0], &byte, 1) == 1);
    const auto openers = vds::node_openers(path, ::getpid());
    assert(openers.size() == 1 && openers.contains(child));
    ::kill(child, SIGTERM);
    ::waitpid(child, nullptr, 0);
    assert(vds::node_openers(path, ::getpid()).empty());
    ::close(fd);
    ::unlink(path);
  }
  return 0;
}
