// SPDX-License-Identifier: MIT
#include <array>
#include <cassert>
#include <cstdint>
#include <filesystem>
#include <string>

#include "../src/jsonl.hh"
#include "../src/vds_companion.hh"
#include "../src/vds_log.hh"

namespace {

// Sends SET_HOST_PERSONA (0x21) and returns the ack result code.
std::uint8_t set_persona(vds::CompanionRuntime &runtime, std::uint8_t value,
                         vds::Logger &logger) {
  std::array<std::uint8_t, vds::kCompanionReportLength> report{};
  report[0] = 0x02;
  report[1] = 'D';
  report[2] = 'S';
  report[3] = '5';
  report[4] = 'B';
  report[5] = vds::kCompanionProtocolMajor;
  report[7] = 0x21;
  report[9] = value;
  std::string request = R"({"command":"companion","op":"set","report":[)";
  for (std::size_t index = 0; index < report.size(); ++index) {
    request += (index ? "," : "") + std::to_string(report[index]);
  }
  request += "]}";
  const auto fields = vds::parse_jsonl_object(request, "test");
  (void)vds::handle_companion_control_request(fields, runtime, "", {}, logger);
  return runtime.last_result_code;
}

// Byte 49 of the STATUS report: the supported persona mask.
std::uint8_t reported_mask(vds::CompanionRuntime &runtime, vds::Logger &logger) {
  const auto fields = vds::parse_jsonl_object(
      R"({"command":"companion","op":"get","report_id":[1]})", "test");
  const std::string reply =
      vds::handle_companion_control_request(fields, runtime, "", {}, logger);
  // {"OK":true,"report":[a,b,...]}: walk to the 50th number.
  std::size_t pos = reply.find('[') + 1;
  for (int index = 0; index < 49; ++index) {
    pos = reply.find(',', pos) + 1;
  }
  return static_cast<std::uint8_t>(std::stoi(reply.substr(pos)));
}

} // namespace

int main() {
  vds::Logger logger(
      (std::filesystem::temp_directory_path() / "vds_companion_persona_test.log")
          .string());

  // Linux: every persona is offered and accepted.
  {
    vds::CompanionRuntime runtime;
    assert(reported_mask(runtime, logger) == 0x07);
    assert(set_persona(runtime, 2, logger) == 0);
    assert(runtime.settings.host_persona_mode == 2);
  }

  // Windows: DualSense only; DualShock 4 and Xbox 360 are rejected.
  {
    vds::CompanionRuntime runtime;
    runtime.supported_host_persona_mask = 0x01;
    assert(reported_mask(runtime, logger) == 0x01);
    assert(set_persona(runtime, 2, logger) != 0);
    assert(set_persona(runtime, 1, logger) != 0);
    assert(runtime.settings.host_persona_mode == 0);
    assert(set_persona(runtime, 0, logger) == 0);
  }

  // Out-of-range values never index past the mask.
  {
    vds::CompanionRuntime runtime;
    assert(set_persona(runtime, 200, logger) != 0);
  }
  return 0;
}
