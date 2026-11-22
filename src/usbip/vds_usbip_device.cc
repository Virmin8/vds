// SPDX-License-Identifier: MIT
//
// Userspace port of the former vds_hcd kernel module's device model
// (vds_usb_core.c) and URB scheduling (vds_hcd_core.c), speaking USB/IP. Behaviour is
// kept identical to the kernel module unless a comment says otherwise, so a
// Linux host sees the same device whichever backend vdsd uses.

#include "vds_usbip_device.hh"

#include <algorithm>
#include <cstring>
#include <exception>
#include <stdexcept>
#include <utility>

#include "uapi/vds.h"
#include "vds/ds5_protocol.h"
#include "vds/ds5_usb.h"
#include "vds/persona_usb.h"
#include "vds_log.hh"

namespace vds::usbip {

namespace {

constexpr std::size_t kMaxTransferBufferBytes = 1024 * 1024;
constexpr std::int32_t kMaxIsoPackets = 1024;

constexpr std::uint32_t kAudioInBytesPerMs = 192;  // 48 kHz S16_LE stereo
constexpr std::uint32_t kAudioOutBytesPerMs = 384; // 48 kHz S16_LE 4ch
constexpr std::size_t kAudioInQueueMaxBytes = kAudioInBytesPerMs * 100;

// HID class requests.
constexpr std::uint8_t kHidGetReport = 0x01;
constexpr std::uint8_t kHidGetIdle = 0x02;
constexpr std::uint8_t kHidGetProtocol = 0x03;
constexpr std::uint8_t kHidSetReport = 0x09;
constexpr std::uint8_t kHidSetIdle = 0x0a;
constexpr std::uint8_t kHidSetProtocol = 0x0b;
constexpr std::uint8_t kHidReportTypeFeature = 0x03;

// UAC1 class requests / Feature Unit selectors.
constexpr std::uint8_t kUacSetCur = 0x01;
constexpr std::uint8_t kUacGetCur = 0x81;
constexpr std::uint8_t kUacGetMin = 0x82;
constexpr std::uint8_t kUacGetMax = 0x83;
constexpr std::uint8_t kUacGetRes = 0x84;
constexpr std::uint8_t kUacFuMute = 0x01;
constexpr std::uint8_t kUacFuVolume = 0x02;
constexpr std::int16_t kSpeakerVolumeMin = -100 * 256;
constexpr std::int16_t kSpeakerVolumeMax = 0;
constexpr std::int16_t kSpeakerVolumeRes = 0x0100;

// Standard requests.
constexpr std::uint8_t kReqGetStatus = 0x00;
constexpr std::uint8_t kReqClearFeature = 0x01;
constexpr std::uint8_t kReqSetFeature = 0x03;
constexpr std::uint8_t kReqSetAddress = 0x05;
constexpr std::uint8_t kReqGetDescriptor = 0x06;
constexpr std::uint8_t kReqGetConfiguration = 0x08;
constexpr std::uint8_t kReqSetConfiguration = 0x09;
constexpr std::uint8_t kReqGetInterface = 0x0a;
constexpr std::uint8_t kReqSetInterface = 0x0b;

constexpr std::uint8_t kDtDevice = 0x01;
constexpr std::uint8_t kDtConfig = 0x02;
constexpr std::uint8_t kDtString = 0x03;
constexpr std::uint8_t kDtDeviceQualifier = 0x06;
constexpr std::uint8_t kDtHid = 0x21;
constexpr std::uint8_t kDtReport = 0x22;

constexpr std::uint8_t kQualifierDescriptor[] = {
    10, kDtDeviceQualifier, 0x00, 0x02, 0, 0, 0, 64, 1, 0,
};

const ControllerProfile kDualSenseProfile{
    VDS_PROFILE_DS5,
    VDS_SONY_VENDOR_ID,
    VDS_DS5_PRODUCT_ID,
    VDS_USB_DEVICE_BCD,
    VDS_USB_DEVICE_CLASS,
    VDS_USB_DEVICE_SUBCLASS,
    VDS_USB_DEVICE_PROTOCOL,
    VDS_USB_MAX_PACKET_SIZE0,
    VDS_USB_NUM_CONFIGURATIONS,
    VDS_USB_CONFIGURATION_VALUE,
    VDS_USB_MANUFACTURER_STRING,
    VDS_DS5_USB_PRODUCT_STRING,
    vds_ds5_usb_configuration_descriptor,
    vds_ds5_usb_hid_descriptor,
    vds_ds5_usb_hid_report_descriptor,
    VDS_USB_AUDIO_CONTROL_INTERFACE,
    VDS_USB_AUDIO_OUT_INTERFACE,
    VDS_USB_AUDIO_IN_INTERFACE,
    VDS_USB_HID_INTERFACE,
    VDS_USB_AUDIO_OUT_ENDPOINT,
    VDS_USB_AUDIO_IN_ENDPOINT,
    VDS_USB_HID_IN_ENDPOINT,
    VDS_USB_HID_OUT_ENDPOINT,
    VDS_DS5_USB_HID_IN_INTERVAL_US,
    VDS_USB_SPEAKER_FEATURE_UNIT,
    VDS_USB_MIC_FEATURE_UNIT,
};

const ControllerProfile kDualSenseEdgeProfile{
    VDS_PROFILE_DSE,
    VDS_SONY_VENDOR_ID,
    VDS_DSE_PRODUCT_ID,
    VDS_USB_DEVICE_BCD,
    VDS_USB_DEVICE_CLASS,
    VDS_USB_DEVICE_SUBCLASS,
    VDS_USB_DEVICE_PROTOCOL,
    VDS_USB_MAX_PACKET_SIZE0,
    VDS_USB_NUM_CONFIGURATIONS,
    VDS_USB_CONFIGURATION_VALUE,
    VDS_USB_MANUFACTURER_STRING,
    VDS_DSE_USB_PRODUCT_STRING,
    vds_dse_usb_configuration_descriptor,
    vds_dse_usb_hid_descriptor,
    vds_dse_usb_hid_report_descriptor,
    VDS_USB_AUDIO_CONTROL_INTERFACE,
    VDS_USB_AUDIO_OUT_INTERFACE,
    VDS_USB_AUDIO_IN_INTERFACE,
    VDS_USB_HID_INTERFACE,
    VDS_USB_AUDIO_OUT_ENDPOINT,
    VDS_USB_AUDIO_IN_ENDPOINT,
    VDS_USB_HID_IN_ENDPOINT,
    VDS_USB_HID_OUT_ENDPOINT,
    VDS_DSE_USB_HID_IN_INTERVAL_US,
    VDS_USB_SPEAKER_FEATURE_UNIT,
    VDS_USB_MIC_FEATURE_UNIT,
};

const ControllerProfile kDs4Profile{
    VDS_PROFILE_DS4,
    VDS_DS4_USB_VENDOR_ID,
    VDS_DS4_USB_PRODUCT_ID,
    VDS_DS4_USB_DEVICE_BCD,
    0x00,
    0x00,
    0x00,
    0x40,
    1,
    1,
    VDS_DS4_USB_MANUFACTURER_STRING,
    VDS_DS4_USB_PRODUCT_STRING,
    vds_ds4_usb_configuration_descriptor,
    vds_ds4_usb_hid_descriptor,
    vds_ds4_usb_hid_report_descriptor,
    kNoInterface,
    kNoInterface,
    kNoInterface,
    VDS_DS4_USB_HID_INTERFACE,
    kNoEndpoint,
    kNoEndpoint,
    VDS_DS4_USB_HID_IN_ENDPOINT,
    VDS_DS4_USB_HID_OUT_ENDPOINT,
    4000,
    kNoFeatureUnit,
    kNoFeatureUnit,
};

const ControllerProfile kXbox360Profile{
    VDS_PROFILE_X360,
    VDS_X360_USB_VENDOR_ID,
    VDS_X360_USB_PRODUCT_ID,
    VDS_X360_USB_DEVICE_BCD,
    0xff,
    0xff,
    0xff,
    0x08,
    1,
    1,
    VDS_X360_USB_MANUFACTURER_STRING,
    VDS_X360_USB_PRODUCT_STRING,
    vds_x360_usb_configuration_descriptor,
    {},
    {},
    kNoInterface,
    kNoInterface,
    kNoInterface,
    // The gamepad interface carries the controller reports.
    VDS_X360_USB_GAMEPAD_INTERFACE,
    kNoEndpoint,
    kNoEndpoint,
    VDS_X360_USB_IN_ENDPOINT,
    VDS_X360_USB_OUT_ENDPOINT,
    4000,
    kNoFeatureUnit,
    kNoFeatureUnit,
};

bool endpoint_matches(std::uint8_t endpoint_address, std::uint32_t ep,
                      bool dir_in) {
  if (endpoint_address == kNoEndpoint) {
    return false;
  }
  return (endpoint_address & 0x0f) == ep &&
         ((endpoint_address & 0x80) != 0) == dir_in;
}

void put_le16(std::vector<std::uint8_t> &out, std::uint16_t value) {
  out.push_back(static_cast<std::uint8_t>(value & 0xff));
  out.push_back(static_cast<std::uint8_t>(value >> 8));
}

std::vector<std::uint8_t> string_descriptor(const char *text) {
  std::vector<std::uint8_t> out;
  if (text == nullptr) {
    return {4, kDtString, 0x09, 0x04}; // LANGID list: en-US
  }
  out.push_back(0);
  out.push_back(kDtString);
  for (const char *c = text; *c != '\0' && out.size() + 2 <= 255; ++c) {
    out.push_back(static_cast<std::uint8_t>(*c));
    out.push_back(0);
  }
  out[0] = static_cast<std::uint8_t>(out.size());
  return out;
}

std::int32_t to_wire(std::int32_t value) {
  return static_cast<std::int32_t>(to_be32(static_cast<std::uint32_t>(value)));
}

std::int32_t from_wire(std::int32_t value) {
  return static_cast<std::int32_t>(
      from_be32(static_cast<std::uint32_t>(value)));
}

} // namespace

std::uint32_t to_be32(std::uint32_t value) {
  const std::uint8_t bytes[4] = {
      static_cast<std::uint8_t>(value >> 24),
      static_cast<std::uint8_t>(value >> 16),
      static_cast<std::uint8_t>(value >> 8),
      static_cast<std::uint8_t>(value),
  };
  std::uint32_t out;
  std::memcpy(&out, bytes, sizeof(out));
  return out;
}

std::uint32_t from_be32(std::uint32_t value) {
  std::uint8_t bytes[4];
  std::memcpy(bytes, &value, sizeof(bytes));
  return (static_cast<std::uint32_t>(bytes[0]) << 24) |
         (static_cast<std::uint32_t>(bytes[1]) << 16) |
         (static_cast<std::uint32_t>(bytes[2]) << 8) |
         static_cast<std::uint32_t>(bytes[3]);
}

bool is_valid_profile(std::uint32_t profile) {
  return profile == VDS_PROFILE_DS5 || profile == VDS_PROFILE_DSE ||
         profile == VDS_PROFILE_DS4 || profile == VDS_PROFILE_X360;
}

const ControllerProfile &controller_profile(std::uint32_t profile) {
  switch (profile) {
  case VDS_PROFILE_DSE:
    return kDualSenseEdgeProfile;
  case VDS_PROFILE_DS4:
    return kDs4Profile;
  case VDS_PROFILE_X360:
    return kXbox360Profile;
  default:
    return kDualSenseProfile;
  }
}

Device::Device(DaemonChannel &channel, Logger *logger, std::string label)
    : channel_(channel), logger_(logger), label_(std::move(label)),
      profile_(VDS_PROFILE_DS5) {
  std::lock_guard lock(mutex_);
  reset_state_locked();
}

Device::~Device() { stop_session(); }

void Device::set_hooks(Hooks hooks) {
  std::lock_guard lock(mutex_);
  hooks_ = std::move(hooks);
}

bool Device::set_profile(std::uint32_t profile) {
  if (!is_valid_profile(profile)) {
    return false;
  }
  std::lock_guard lock(mutex_);
  if (session_active_) {
    return false;
  }
  profile_ = profile;
  reset_state_locked();
  return true;
}

std::uint32_t Device::profile() const {
  std::lock_guard lock(mutex_);
  return profile_;
}

void Device::reset_state() {
  std::lock_guard lock(mutex_);
  reset_state_locked();
}

bool Device::session_active() const {
  std::lock_guard lock(mutex_);
  return session_active_;
}

void Device::reset_state_locked() {
  configuration_ = 0;
  audio_out_altsetting_ = 0;
  audio_in_altsetting_ = 0;
  hid_idle_ = 0;
  hid_protocol_ = 0;
  audio_mute_ = {};
  audio_volume_ = {kSpeakerVolumeMin, kSpeakerVolumeMin};
  for (auto &report : feature_cache_) {
    report.clear();
  }
  input_packet_.reset();
  audio_in_.clear();
  next_iso_out_ready_ = {};
  next_iso_in_ready_ = {};
  next_hid_in_ready_ = {};
}

// ---- Completion ordering (vds_queue_urb_giveback_locked) ------------------

void Device::enqueue_completion_locked(Urb urb) {
  // Control/interrupt/unlink completions must not sit behind future iso
  // completions (SET_INTERFACE would otherwise time out while an audio
  // stream keeps the timed queue full); timed entries stay sorted by due
  // time, FIFO among equals.
  auto position = completions_.end();
  for (auto it = completions_.begin(); it != completions_.end(); ++it) {
    if (!it->timed) {
      continue;
    }
    if (!urb.timed || it->ready > urb.ready) {
      position = it;
      break;
    }
  }
  completions_.insert(position, std::move(urb));
  completion_cv_.notify_all();
}

void Device::schedule_iso_locked(Urb &urb) {
  const bool input = urb.dir_in;
  const std::uint32_t packets =
      static_cast<std::uint32_t>(std::max(urb.number_of_packets, 1));
  std::uint64_t duration_us = static_cast<std::uint64_t>(packets) * 1000;
  std::uint64_t total = 0;
  for (const auto &packet : urb.iso) {
    total += input ? std::min<std::uint32_t>(packet.length, kAudioInBytesPerMs)
                   : packet.length;
  }
  if (total != 0) {
    // ALSA may put several milliseconds of PCM in one descriptor: pace by
    // audio duration, not just descriptor count.
    const std::uint64_t bytes_per_ms =
        input ? kAudioInBytesPerMs : kAudioOutBytesPerMs;
    const std::uint64_t by_bytes =
        (total * 1000 + bytes_per_ms - 1) / bytes_per_ms;
    duration_us = std::max<std::uint64_t>(by_bytes, 1000);
  }

  const auto now = Clock::now();
  auto &next_ready = input ? next_iso_in_ready_ : next_iso_out_ready_;
  const auto base = next_ready > now ? next_ready : now;
  urb.start_frame = static_cast<std::int32_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(
          base.time_since_epoch())
          .count());
  urb.ready = base + std::chrono::microseconds(duration_us);
  urb.timed = true;
  next_ready = urb.ready;
}

void Device::schedule_hid_in_locked(Urb &urb) {
  const auto now = Clock::now();
  const auto base = next_hid_in_ready_ > now ? next_hid_in_ready_ : now;
  urb.ready = base + std::chrono::microseconds(
                         controller_profile(profile_).hid_in_interval_us);
  urb.timed = true;
  next_hid_in_ready_ = urb.ready;
}

bool Device::remove_pending_locked(std::uint32_t seqnum) {
  const auto matches = [seqnum](const Urb &urb) {
    return urb.kind == Kind::Submit && urb.seqnum == seqnum;
  };
  if (auto it = std::find_if(completions_.begin(), completions_.end(), matches);
      it != completions_.end()) {
    completions_.erase(it);
    return true;
  }
  if (auto it = std::find_if(pending_hid_in_.begin(), pending_hid_in_.end(),
                             matches);
      it != pending_hid_in_.end()) {
    pending_hid_in_.erase(it);
    return true;
  }
  if (auto it = std::find_if(pending_feature_get_.begin(),
                             pending_feature_get_.end(), matches);
      it != pending_feature_get_.end()) {
    pending_feature_get_.erase(it);
    return true;
  }
  return false;
}

// ---- Control requests (vds_usb_device_control_urb) ------------------------

Device::ControlResult
Device::handle_control_locked(Urb &urb, const std::uint8_t *setup,
                              std::span<const std::uint8_t> out_data) {
  const std::uint8_t request_type = setup[0];
  const std::uint8_t request = setup[1];
  const auto value = static_cast<std::uint16_t>(setup[2] | (setup[3] << 8));
  const auto index = static_cast<std::uint16_t>(setup[4] | (setup[5] << 8));
  const auto length = static_cast<std::uint16_t>(setup[6] | (setup[7] << 8));
  const bool device_to_host = (request_type & 0x80) != 0;
  urb.reply_limit = static_cast<std::uint16_t>(
      std::min<std::uint32_t>(length, urb.transfer_buffer_length));

  const ControllerProfile &profile = controller_profile(profile_);
  const std::uint8_t type = request_type & 0x60;
  const std::uint8_t recipient = request_type & 0x1f;

  int status = kStatusEpipe;
  if (type == 0x00) {
    status = standard_control_locked(urb, request_type, request, value, index);
  } else if (type == 0x20 && recipient == 0x01) {
    if ((index & 0xff) == profile.hid_interface) {
      if (hid_control_locked(urb, request, value, index, length, out_data) ==
          ControlResult::Pending) {
        return ControlResult::Pending;
      }
      status = urb.status;
    } else {
      status = audio_control_locked(urb, device_to_host, request, value, index,
                                    out_data);
    }
  }

  urb.status = status;
  if (status != 0) {
    urb.data.clear();
  } else if (urb.dir_in) {
    if (urb.data.size() > urb.reply_limit) {
      urb.data.resize(urb.reply_limit);
    }
  } else {
    urb.out_actual = static_cast<std::uint32_t>(out_data.size());
  }
  return ControlResult::Complete;
}

int Device::standard_control_locked(Urb &urb, std::uint8_t request_type,
                                    std::uint8_t request, std::uint16_t value,
                                    std::uint16_t index) {
  (void)request_type;
  const ControllerProfile &profile = controller_profile(profile_);
  const std::uint8_t interface = index & 0xff;

  switch (request) {
  case kReqGetDescriptor: {
    const std::uint8_t desc_type = value >> 8;
    const std::uint8_t desc_index = value & 0xff;
    switch (desc_type) {
    case kDtDevice: {
      auto &d = urb.data;
      d = {18, kDtDevice};
      put_le16(d, 0x0200);
      d.push_back(profile.device_class);
      d.push_back(profile.device_subclass);
      d.push_back(profile.device_protocol);
      d.push_back(profile.max_packet_size0);
      put_le16(d, profile.vendor_id);
      put_le16(d, profile.product_id);
      put_le16(d, profile.device_version);
      d.push_back(1); // iManufacturer
      d.push_back(2); // iProduct
      d.push_back(0); // iSerialNumber
      d.push_back(profile.num_configurations);
      return 0;
    }
    case kDtConfig:
      urb.data.assign(profile.configuration_descriptor.begin(),
                      profile.configuration_descriptor.end());
      return 0;
    case kDtDeviceQualifier:
      urb.data.assign(std::begin(kQualifierDescriptor),
                      std::end(kQualifierDescriptor));
      return 0;
    case kDtString:
      if (desc_index == 0) {
        urb.data = string_descriptor(nullptr);
      } else if (desc_index == 1) {
        urb.data = string_descriptor(profile.manufacturer);
      } else if (desc_index == 2) {
        urb.data = string_descriptor(profile.product);
      } else {
        return kStatusEpipe;
      }
      return 0;
    case kDtReport:
      if (interface != profile.hid_interface ||
          profile.hid_report_descriptor.empty()) {
        return kStatusEpipe;
      }
      urb.data.assign(profile.hid_report_descriptor.begin(),
                      profile.hid_report_descriptor.end());
      return 0;
    case kDtHid:
      if (interface != profile.hid_interface ||
          profile.hid_descriptor.empty()) {
        return kStatusEpipe;
      }
      urb.data.assign(profile.hid_descriptor.begin(),
                      profile.hid_descriptor.end());
      return 0;
    default:
      return kStatusEpipe;
    }
  }
  case kReqSetAddress:
    return 0;
  case kReqSetConfiguration:
    if (value != 0 && value != profile.configuration_value) {
      return kStatusEpipe;
    }
    configuration_ = value & 0xff;
    audio_out_altsetting_ = 0;
    audio_in_altsetting_ = 0;
    return 0;
  case kReqGetConfiguration:
    urb.data = {configuration_};
    return 0;
  case kReqSetInterface: {
    const auto send_event = [&](std::uint8_t number, std::uint8_t alt,
                                std::uint8_t kind) {
      vds_usb_interface_event event{};
      event.interface_number = number;
      event.altsetting = alt;
      event.interface_type = kind;
      std::uint8_t bytes[sizeof(event)];
      std::memcpy(bytes, &event, sizeof(event));
      write_frame(VDS_FRAME_USB_INTERFACE, bytes);
    };
    if (interface == profile.hid_interface) {
      if (value != 0) {
        return kStatusEpipe;
      }
      send_event(profile.hid_interface, 0, VDS_USB_INTERFACE_HID);
      return 0;
    }
    if (value > 1) {
      return kStatusEpipe;
    }
    if (profile.audio_out_interface != kNoInterface &&
        interface == profile.audio_out_interface) {
      audio_out_altsetting_ = value & 0xff;
      send_event(profile.audio_out_interface, value & 0xff,
                 VDS_USB_INTERFACE_AUDIO_OUT);
      return 0;
    }
    if (profile.audio_in_interface != kNoInterface &&
        interface == profile.audio_in_interface) {
      audio_in_altsetting_ = value & 0xff;
      send_event(profile.audio_in_interface, value & 0xff,
                 VDS_USB_INTERFACE_AUDIO_IN);
      return 0;
    }
    return kStatusEpipe;
  }
  case kReqGetInterface:
    if (profile.audio_out_interface != kNoInterface &&
        interface == profile.audio_out_interface) {
      urb.data = {audio_out_altsetting_};
    } else if (profile.audio_in_interface != kNoInterface &&
               interface == profile.audio_in_interface) {
      urb.data = {audio_in_altsetting_};
    } else if (interface == profile.hid_interface) {
      urb.data = {0};
    } else {
      return kStatusEpipe;
    }
    return 0;
  case kReqGetStatus:
    urb.data = {0, 0};
    return 0;
  case kReqClearFeature:
  case kReqSetFeature:
    return 0;
  default:
    return kStatusEpipe;
  }
}

Device::ControlResult
Device::hid_control_locked(Urb &urb, std::uint8_t request, std::uint16_t value,
                           std::uint16_t index, std::uint16_t length,
                           std::span<const std::uint8_t> out_data) {
  (void)index;
  const std::uint8_t report_type = value >> 8;
  const std::uint8_t report_id = value & 0xff;
  urb.status = 0;

  switch (request) {
  case kHidGetReport:
    if (report_type == kHidReportTypeFeature) {
      if (!feature_cache_[report_id].empty()) {
        urb.data = feature_cache_[report_id];
        return ControlResult::Complete;
      }
      // Cache miss: the physical controller answers. Park the request first
      // so a fast FEATURE_REPLY cannot race it.
      urb.feature_report_id = report_id;
      pending_feature_get_.push_back(std::move(urb));
      const std::uint8_t request_frame[3] = {
          report_id,
          static_cast<std::uint8_t>(length & 0xff),
          static_cast<std::uint8_t>(length >> 8),
      };
      write_frame(VDS_FRAME_USB_FEATURE_GET, request_frame);
      return ControlResult::Pending;
    }
    urb.data.assign(kHidPacketSize, 0);
    urb.data[0] = report_id;
    return ControlResult::Complete;
  case kHidSetReport:
    if (report_type != kHidReportTypeFeature ||
        out_data.size() > kHidPacketSize) {
      urb.status = kStatusEpipe;
      return ControlResult::Complete;
    }
    if (!out_data.empty()) {
      std::vector<std::uint8_t> frame;
      frame.reserve(out_data.size() + 1);
      frame.push_back(report_id);
      frame.insert(frame.end(), out_data.begin(), out_data.end());
      write_frame(VDS_FRAME_USB_FEATURE_SET, frame);
    }
    return ControlResult::Complete;
  case kHidGetIdle:
    urb.data = {hid_idle_};
    return ControlResult::Complete;
  case kHidGetProtocol:
    urb.data = {hid_protocol_};
    return ControlResult::Complete;
  case kHidSetIdle:
    hid_idle_ = value >> 8;
    return ControlResult::Complete;
  case kHidSetProtocol:
    hid_protocol_ = value & 0xff;
    return ControlResult::Complete;
  default:
    urb.status = kStatusEpipe;
    return ControlResult::Complete;
  }
}

int Device::audio_control_locked(Urb &urb, bool device_to_host,
                                 std::uint8_t request, std::uint16_t value,
                                 std::uint16_t index,
                                 std::span<const std::uint8_t> out_data) {
  const ControllerProfile &profile = controller_profile(profile_);
  const std::uint8_t entity_id = index >> 8;
  const std::uint8_t interface = index & 0xff;
  const std::uint8_t selector = value >> 8;

  if (profile.audio_control_interface == kNoInterface ||
      interface != profile.audio_control_interface) {
    return kStatusEpipe;
  }
  std::size_t feature = 0;
  if (entity_id == profile.speaker_feature_unit) {
    feature = 0;
  } else if (profile.mic_feature_unit != kNoFeatureUnit &&
             entity_id == profile.mic_feature_unit) {
    feature = 1;
  } else {
    return kStatusEpipe;
  }

  if (device_to_host) {
    if (selector == kUacFuMute) {
      if (request != kUacGetCur) {
        return kStatusEpipe;
      }
      urb.data = {audio_mute_[feature]};
      return 0;
    }
    if (selector == kUacFuVolume) {
      std::int16_t volume = 0;
      switch (request) {
      case kUacGetCur:
        volume = audio_volume_[feature];
        break;
      case kUacGetMin:
        volume = kSpeakerVolumeMin;
        break;
      case kUacGetMax:
        volume = kSpeakerVolumeMax;
        break;
      case kUacGetRes:
        volume = kSpeakerVolumeRes;
        break;
      default:
        return kStatusEpipe;
      }
      urb.data.clear();
      put_le16(urb.data, static_cast<std::uint16_t>(volume));
      return 0;
    }
    return kStatusEpipe;
  }

  if (request != kUacSetCur) {
    return kStatusEpipe;
  }
  if (selector == kUacFuMute && !out_data.empty()) {
    audio_mute_[feature] = out_data[0];
    return 0;
  }
  if (selector == kUacFuVolume && out_data.size() >= 2) {
    audio_volume_[feature] = static_cast<std::int16_t>(
        static_cast<std::uint16_t>(out_data[0]) |
        (static_cast<std::uint16_t>(out_data[1]) << 8));
    return 0;
  }
  return kStatusEpipe;
}

// ---- Isochronous data (vds_iso_out_urb / vds_iso_in_urb) ------------------

void Device::deliver_iso_out(Urb &urb) {
  urb.error_count = 0;
  std::uint32_t total = 0;
  std::size_t chunk_offset = 0;
  std::size_t chunk_len = 0;
  std::size_t next_offset = 0;
  bool have_chunk = false;

  const auto flush = [&] {
    if (have_chunk && chunk_len != 0) {
      // Realtime audio: if vdsd cannot keep up the chunk is dropped rather
      // than stalling the isochronous pipe.
      write_frame(VDS_FRAME_USB_AUDIO_OUT,
                  std::span(urb.data.data() + chunk_offset, chunk_len));
    }
  };

  for (auto &packet : urb.iso) {
    if (packet.offset > urb.data.size() ||
        packet.length > urb.data.size() - packet.offset) {
      packet.status = kStatusEoverflow;
      packet.actual_length = 0;
      ++urb.error_count;
      continue;
    }
    packet.status = 0;
    packet.actual_length = static_cast<std::int32_t>(packet.length);
    total += packet.length;
    if (packet.length == 0) {
      continue;
    }
    if (!have_chunk) {
      have_chunk = true;
      chunk_offset = packet.offset;
      chunk_len = packet.length;
      next_offset = packet.offset + packet.length;
      continue;
    }
    if (packet.offset == next_offset &&
        chunk_len + packet.length <= VDS_FRAME_MAX_PAYLOAD) {
      chunk_len += packet.length;
      next_offset += packet.length;
      continue;
    }
    flush();
    chunk_offset = packet.offset;
    chunk_len = packet.length;
    next_offset = packet.offset + packet.length;
  }
  flush();
  urb.out_actual = total;
}

void Device::fill_iso_in_locked(Urb &urb) {
  urb.error_count = 0;
  urb.data.clear();
  for (auto &packet : urb.iso) {
    if (packet.offset > urb.transfer_buffer_length ||
        packet.length > urb.transfer_buffer_length - packet.offset) {
      packet.status = kStatusEoverflow;
      packet.actual_length = 0;
      ++urb.error_count;
      continue;
    }
    const std::uint32_t actual =
        std::min<std::uint32_t>(packet.length, kAudioInBytesPerMs);
    const std::size_t available = std::min<std::size_t>(actual, audio_in_.size());
    // USB/IP carries iso IN data packed (only actual_length bytes per
    // packet, in packet order); vhci re-spreads it to the packet offsets.
    urb.data.insert(urb.data.end(), audio_in_.begin(),
                    audio_in_.begin() + static_cast<std::ptrdiff_t>(available));
    audio_in_.erase(audio_in_.begin(),
                    audio_in_.begin() + static_cast<std::ptrdiff_t>(available));
    urb.data.resize(urb.data.size() + (actual - available), 0);
    packet.status = 0;
    packet.actual_length = static_cast<std::int32_t>(actual);
  }
}

// ---- Daemon frames (vds_write) ---------------------------------------------

void Device::handle_daemon_frame(std::uint16_t type,
                                 std::span<const std::uint8_t> payload) {
  std::lock_guard lock(mutex_);
  switch (type) {
  case VDS_FRAME_USB_HID_IN: {
    if (payload.empty() || payload.size() > kHidPacketSize) {
      return;
    }
    if (!pending_hid_in_.empty()) {
      Urb urb = std::move(pending_hid_in_.front());
      pending_hid_in_.pop_front();
      urb.data.assign(payload.begin(),
                      payload.begin() + std::min<std::size_t>(
                                            payload.size(),
                                            urb.transfer_buffer_length));
      urb.status = 0;
      schedule_hid_in_locked(urb);
      enqueue_completion_locked(std::move(urb));
      return;
    }
    // Controller state snapshots: if the host is not polling right now keep
    // only the newest one instead of replaying stale input later.
    input_packet_.emplace(payload.begin(), payload.end());
    return;
  }
  case VDS_FRAME_USB_FEATURE_REPLY: {
    if (payload.empty() || payload.size() > kHidPacketSize) {
      return;
    }
    const std::uint8_t report_id = payload[0];
    feature_cache_[report_id].assign(payload.begin(), payload.end());
    for (auto it = pending_feature_get_.begin();
         it != pending_feature_get_.end();) {
      if (it->feature_report_id != report_id) {
        ++it;
        continue;
      }
      Urb urb = std::move(*it);
      it = pending_feature_get_.erase(it);
      urb.data.assign(payload.begin(),
                      payload.begin() +
                          std::min<std::size_t>(payload.size(),
                                                urb.reply_limit));
      urb.status = 0;
      enqueue_completion_locked(std::move(urb));
    }
    return;
  }
  case VDS_FRAME_USB_AUDIO_IN:
    if (payload.empty()) {
      return;
    }
    audio_in_.insert(audio_in_.end(), payload.begin(), payload.end());
    if (audio_in_.size() > kAudioInQueueMaxBytes) {
      audio_in_.erase(audio_in_.begin(),
                      audio_in_.begin() +
                          static_cast<std::ptrdiff_t>(audio_in_.size() -
                                                      kAudioInQueueMaxBytes));
    }
    return;
  default:
    return;
  }
}

// ---- USB/IP session ---------------------------------------------------------

void Device::handle_submit(const CmdSubmit &cmd,
                           std::vector<std::uint8_t> out_data,
                           std::vector<IsoPacketDescriptor> iso) {
  Urb urb;
  urb.seqnum = cmd.base.seqnum;
  urb.dir_in = cmd.base.direction == kDirIn;
  urb.transfer_buffer_length =
      static_cast<std::uint32_t>(cmd.transfer_buffer_length);
  urb.wire_number_of_packets = cmd.number_of_packets;
  urb.number_of_packets = std::max(cmd.number_of_packets, 0);
  urb.iso = std::move(iso);
  const std::uint32_t ep = cmd.base.ep;

  std::lock_guard lock(mutex_);
  const ControllerProfile &profile = controller_profile(profile_);

  if (ep == 0) {
    if (handle_control_locked(urb, cmd.setup, out_data) ==
        ControlResult::Complete) {
      enqueue_completion_locked(std::move(urb));
    }
    return;
  }

  if (urb.number_of_packets == 0 &&
      endpoint_matches(profile.hid_in_endpoint, ep, urb.dir_in)) {
    if (input_packet_) {
      urb.data.assign(input_packet_->begin(),
                      input_packet_->begin() +
                          std::min<std::size_t>(input_packet_->size(),
                                                urb.transfer_buffer_length));
      input_packet_.reset();
      schedule_hid_in_locked(urb);
      enqueue_completion_locked(std::move(urb));
    } else {
      pending_hid_in_.push_back(std::move(urb));
    }
    return;
  }

  if (urb.number_of_packets == 0 &&
      endpoint_matches(profile.hid_out_endpoint, ep, urb.dir_in)) {
    if (out_data.size() > kHidPacketSize) {
      urb.status = kStatusEpipe;
    } else {
      // Best-effort: vdsd keeps the newest output state, so backpressure
      // must not stall the interrupt pipe.
      if (!out_data.empty()) {
        write_frame(VDS_FRAME_USB_HID_OUT, out_data);
      }
      urb.out_actual = static_cast<std::uint32_t>(out_data.size());
    }
    enqueue_completion_locked(std::move(urb));
    return;
  }

  if (urb.number_of_packets > 0 &&
      endpoint_matches(profile.audio_out_endpoint, ep, urb.dir_in)) {
    urb.data = std::move(out_data);
    urb.deliver = Deliver::IsoOut;
    schedule_iso_locked(urb);
    enqueue_completion_locked(std::move(urb));
    return;
  }

  if (urb.number_of_packets > 0 &&
      endpoint_matches(profile.audio_in_endpoint, ep, urb.dir_in)) {
    urb.deliver = Deliver::IsoIn;
    schedule_iso_locked(urb);
    enqueue_completion_locked(std::move(urb));
    return;
  }

  urb.status = kStatusEpipe;
  enqueue_completion_locked(std::move(urb));
}

void Device::handle_unlink(const CmdUnlink &cmd) {
  std::lock_guard lock(mutex_);
  Urb reply;
  reply.kind = Kind::UnlinkReply;
  reply.seqnum = cmd.base.seqnum;
  // -ECONNRESET: unlinked before completion, no RET_SUBMIT will follow.
  // 0: already completed (its RET_SUBMIT was or is being sent; the single
  // completion thread sends it before this reply).
  reply.status =
      remove_pending_locked(cmd.unlink_seqnum) ? kStatusEconnreset : 0;
  enqueue_completion_locked(std::move(reply));
}

std::vector<std::uint8_t> Device::serialize(const Urb &urb) {
  std::vector<std::uint8_t> bytes;
  if (urb.kind == Kind::UnlinkReply) {
    RetUnlink ret{};
    ret.base.command = to_be32(kRetUnlink);
    ret.base.seqnum = to_be32(urb.seqnum);
    ret.status = to_wire(urb.status);
    bytes.resize(sizeof(ret));
    std::memcpy(bytes.data(), &ret, sizeof(ret));
    return bytes;
  }

  const bool iso = urb.number_of_packets > 0;
  std::uint32_t actual = 0;
  if (urb.status == 0) {
    if (iso) {
      for (const auto &packet : urb.iso) {
        actual += static_cast<std::uint32_t>(packet.actual_length);
      }
    } else {
      actual = urb.dir_in ? static_cast<std::uint32_t>(urb.data.size())
                          : urb.out_actual;
    }
  }
  const bool send_data = urb.dir_in && actual != 0;

  RetSubmit ret{};
  ret.base.command = to_be32(kRetSubmit);
  ret.base.seqnum = to_be32(urb.seqnum);
  ret.status = to_wire(urb.status);
  ret.actual_length = to_wire(static_cast<std::int32_t>(actual));
  ret.start_frame = to_wire(urb.start_frame);
  ret.number_of_packets =
      to_wire(iso ? urb.number_of_packets : urb.wire_number_of_packets);
  ret.error_count = to_wire(urb.error_count);

  bytes.resize(sizeof(ret) + (send_data ? actual : 0) +
               (iso ? urb.iso.size() * sizeof(IsoPacketDescriptor) : 0));
  std::uint8_t *cursor = bytes.data();
  std::memcpy(cursor, &ret, sizeof(ret));
  cursor += sizeof(ret);
  if (send_data) {
    std::memcpy(cursor, urb.data.data(), actual);
    cursor += actual;
  }
  if (iso) {
    for (const auto &packet : urb.iso) {
      IsoPacketDescriptor wire{};
      wire.offset = to_be32(packet.offset);
      wire.length = to_be32(packet.length);
      wire.actual_length = to_wire(urb.status == 0 ? packet.actual_length : 0);
      wire.status = to_wire(urb.status == 0 ? packet.status : urb.status);
      std::memcpy(cursor, &wire, sizeof(wire));
      cursor += sizeof(wire);
    }
  }
  return bytes;
}

void Device::completion_loop(Transport &transport) {
  std::function<void()> on_start;
  {
    std::lock_guard lock(mutex_);
    on_start = hooks_.on_completion_thread_start;
  }
  if (on_start) {
    try {
      on_start();
    } catch (...) {
    }
  }

  std::unique_lock lock(mutex_);
  while (!session_stop_) {
    if (completions_.empty()) {
      completion_cv_.wait(lock);
      continue;
    }
    if (completions_.front().timed) {
      const auto due = completions_.front().ready;
      if (Clock::now() < due) {
        // Woken early when something sooner is queued (or on stop).
        completion_cv_.wait_until(lock, due);
        continue;
      }
    }
    Urb urb = std::move(completions_.front());
    completions_.pop_front();
    const bool ok = urb.kind == Kind::Submit && urb.status == 0;
    if (ok && urb.deliver == Deliver::IsoIn) {
      fill_iso_in_locked(urb);
    }
    lock.unlock();
    if (ok && urb.deliver == Deliver::IsoOut) {
      deliver_iso_out(urb);
    }
    const std::vector<std::uint8_t> bytes = serialize(urb);
    const bool sent = transport.send_exact(bytes.data(), bytes.size());
    lock.lock();
    if (!sent) {
      session_stop_ = true;
    }
  }
}

void Device::run_session(Transport &transport) {
  {
    std::lock_guard lock(mutex_);
    if (session_active_) {
      throw std::runtime_error("USB/IP session already active");
    }
    session_active_ = true;
    session_stop_ = false;
    completions_.clear();
    pending_hid_in_.clear();
    pending_feature_get_.clear();
  }
  log_info("USB/IP session started");

  std::thread completion([this, &transport] { completion_loop(transport); });

  std::string end_reason = "host closed the connection";
  try {
    for (;;) {
      {
        std::lock_guard lock(mutex_);
        if (session_stop_) {
          end_reason = "stopped";
          break;
        }
      }
      CmdSubmit cmd{};
      if (!transport.recv_exact(&cmd, sizeof(cmd))) {
        break;
      }
      cmd.base.command = from_be32(cmd.base.command);
      cmd.base.seqnum = from_be32(cmd.base.seqnum);
      cmd.base.devid = from_be32(cmd.base.devid);
      cmd.base.direction = from_be32(cmd.base.direction);
      cmd.base.ep = from_be32(cmd.base.ep);

      if (cmd.base.command == kCmdUnlink) {
        CmdUnlink unlink{};
        std::memcpy(&unlink, &cmd, sizeof(unlink));
        unlink.unlink_seqnum = from_be32(unlink.unlink_seqnum);
        handle_unlink(unlink);
        continue;
      }
      if (cmd.base.command != kCmdSubmit) {
        end_reason = "unexpected USB/IP command " +
                     std::to_string(cmd.base.command);
        break;
      }

      cmd.transfer_flags = from_be32(cmd.transfer_flags);
      cmd.transfer_buffer_length = from_wire(cmd.transfer_buffer_length);
      cmd.start_frame = from_wire(cmd.start_frame);
      cmd.number_of_packets = from_wire(cmd.number_of_packets);
      cmd.interval = from_wire(cmd.interval);
      if ((cmd.base.direction != kDirOut && cmd.base.direction != kDirIn) ||
          cmd.base.ep > 15 || cmd.transfer_buffer_length < 0 ||
          static_cast<std::size_t>(cmd.transfer_buffer_length) >
              kMaxTransferBufferBytes ||
          cmd.number_of_packets < -1 ||
          cmd.number_of_packets > kMaxIsoPackets) {
        end_reason = "malformed CMD_SUBMIT seqnum=" +
                     std::to_string(cmd.base.seqnum);
        break;
      }

      // Wire order: header, OUT data, then iso packet descriptors.
      std::vector<std::uint8_t> out_data;
      if (cmd.base.direction == kDirOut && cmd.transfer_buffer_length > 0) {
        out_data.resize(static_cast<std::size_t>(cmd.transfer_buffer_length));
        if (!transport.recv_exact(out_data.data(), out_data.size())) {
          break;
        }
      }
      std::vector<IsoPacketDescriptor> iso;
      if (cmd.number_of_packets > 0) {
        iso.resize(static_cast<std::size_t>(cmd.number_of_packets));
        if (!transport.recv_exact(iso.data(),
                                  iso.size() * sizeof(IsoPacketDescriptor))) {
          break;
        }
        for (auto &packet : iso) {
          packet.offset = from_be32(packet.offset);
          packet.length = from_be32(packet.length);
          packet.actual_length = 0;
          packet.status = 0;
        }
      }
      handle_submit(cmd, std::move(out_data), std::move(iso));
    }
  } catch (const std::exception &error) {
    end_reason = std::string("error: ") + error.what();
  }

  {
    std::lock_guard lock(mutex_);
    session_stop_ = true;
  }
  completion_cv_.notify_all();
  completion.join();
  {
    std::lock_guard lock(mutex_);
    completions_.clear();
    pending_hid_in_.clear();
    pending_feature_get_.clear();
    next_iso_out_ready_ = {};
    next_iso_in_ready_ = {};
    next_hid_in_ready_ = {};
    session_active_ = false;
  }
  log_info("USB/IP session ended: " + end_reason);
}

void Device::stop_session() {
  {
    std::lock_guard lock(mutex_);
    session_stop_ = true;
  }
  completion_cv_.notify_all();
}

void Device::write_frame(std::uint16_t type,
                         std::span<const std::uint8_t> payload) {
  (void)channel_.write_frame(type, payload);
}

void Device::log_info(const std::string &message) {
  if (logger_ != nullptr) {
    try {
      logger_->log(LogScope::Usb, LogLevel::Info, label_ + " " + message);
    } catch (...) {
    }
  }
}

void Device::log_warn(const std::string &message) {
  if (logger_ != nullptr) {
    try {
      logger_->log(LogScope::Usb, LogLevel::Warn, label_ + " " + message);
    } catch (...) {
    }
  }
}

} // namespace vds::usbip
