// SPDX-License-Identifier: MIT
#include <cassert>
#include <cstdint>
#include <vector>

#include "vds/ds5_protocol.h"
#include "../src/vds_protocol.hh"

int main() {
  std::vector<std::uint8_t> packet(
      3 + VDS_USB_INPUT_PAYLOAD_SIZE, 0);
  packet[0] = 0xa1;
  packet[1] = VDS_BT_STATE_REPORT_ID;

  assert(vds::bt_input_to_usb_input(packet).has_value());

  packet[2] = 0x02;
  assert(!vds::bt_input_to_usb_input(packet).has_value());
}
