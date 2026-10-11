// SPDX-License-Identifier: MIT
/*
 * Xbox 360 wired controller (045e:028e) persona for games without
 * PlayStation controller support. Only the gamepad interface (vendor class
 * 0xff/0x5d/0x01, bound by the Linux xpad driver) is exposed; there is no HID
 * descriptor. vdsd translates DualSense input into the 20-byte XInput report
 * and rumble output (00 08 00 big small ...) back to the DualSense.
 */

#include "uapi/vds.h"
#include "vds_controller.h"

#define VDS_X360_VENDOR_ID 0x045e
#define VDS_X360_PRODUCT_ID 0x028e
#define VDS_X360_DEVICE_BCD 0x0114
#define VDS_X360_GAMEPAD_INTERFACE 0
#define VDS_X360_IN_ENDPOINT 0x81
#define VDS_X360_OUT_ENDPOINT 0x01
#define VDS_X360_IN_INTERVAL_US 4000

#define VDS_X360_CONFIG_TOTAL_LENGTH (9 + 9 + 17 + 7 + 7)

static const u8 vds_x360_configuration_descriptor[] = {
	/* Configuration */
	0x09, 0x02, VDS_X360_CONFIG_TOTAL_LENGTH, 0x00, 0x01, 0x01, 0x00, 0xa0,
	0xfa,
	/* Interface 0: vendor specific XUSB gamepad */
	0x09, 0x04, VDS_X360_GAMEPAD_INTERFACE, 0x00, 0x02, 0xff, 0x5d, 0x01,
	0x00,
	/* XUSB class-specific descriptor */
	0x11, 0x21, 0x00, 0x01, 0x01, 0x25, VDS_X360_IN_ENDPOINT, 0x14, 0x00,
	0x00, 0x00, 0x00, 0x13, VDS_X360_OUT_ENDPOINT, 0x08, 0x00, 0x00,
	/* Interrupt IN */
	0x07, 0x05, VDS_X360_IN_ENDPOINT, 0x03, 0x20, 0x00, 0x04,
	/* Interrupt OUT */
	0x07, 0x05, VDS_X360_OUT_ENDPOINT, 0x03, 0x20, 0x00, 0x08,
};

_Static_assert(sizeof(vds_x360_configuration_descriptor) ==
		       VDS_X360_CONFIG_TOTAL_LENGTH,
	       "unexpected Xbox 360 configuration descriptor size");

const struct vds_controller_profile vds_xbox360_profile = {
	.profile = VDS_PROFILE_X360,
	.vendor_id = VDS_X360_VENDOR_ID,
	.product_id = VDS_X360_PRODUCT_ID,
	.device_version = VDS_X360_DEVICE_BCD,
	.device_class = 0xff,
	.device_subclass = 0xff,
	.device_protocol = 0xff,
	.max_packet_size0 = 0x08,
	.num_configurations = 1,
	.configuration_value = 1,
	.manufacturer = "Microsoft Corporation",
	.product = "Controller",
	.configuration_descriptor = vds_x360_configuration_descriptor,
	.configuration_descriptor_size =
		sizeof(vds_x360_configuration_descriptor),
	.hid_descriptor = NULL,
	.hid_descriptor_size = 0,
	.hid_report_descriptor = NULL,
	.hid_report_descriptor_size = 0,
	.audio_control_interface = VDS_CONTROLLER_NO_INTERFACE,
	.audio_out_interface = VDS_CONTROLLER_NO_INTERFACE,
	.audio_in_interface = VDS_CONTROLLER_NO_INTERFACE,
	/* The gamepad interface carries the controller reports. */
	.hid_interface = VDS_X360_GAMEPAD_INTERFACE,
	.audio_out_endpoint = VDS_CONTROLLER_NO_ENDPOINT,
	.audio_in_endpoint = VDS_CONTROLLER_NO_ENDPOINT,
	.hid_in_endpoint = VDS_X360_IN_ENDPOINT,
	.hid_out_endpoint = VDS_X360_OUT_ENDPOINT,
	.hid_in_interval_us = VDS_X360_IN_INTERVAL_US,
	.speaker_feature_unit = VDS_CONTROLLER_NO_AUDIO_FEATURE,
	.mic_feature_unit = VDS_CONTROLLER_NO_AUDIO_FEATURE,
};
