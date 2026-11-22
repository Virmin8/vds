/* SPDX-License-Identifier: MIT */
/* Copyright (C) 2026 Jihong Min <hurryman2212@gmail.com> */
#ifndef _UAPI_VDS_H
#define _UAPI_VDS_H

#ifdef _WIN32
#include <stdint.h>
typedef uint8_t __u8;
typedef uint16_t __u16;
typedef uint32_t __u32;
typedef uint64_t __u64;
#else
#include <linux/types.h>
#endif

#define VDS_FRAME_MAX_PAYLOAD 4096

enum {
	VDS_MIN_PORT_COUNT = 1,
	VDS_MAX_PORT_COUNT = 4,
};

enum vds_profile {
	VDS_PROFILE_DS5 = 0,
	VDS_PROFILE_DSE = 1,
	/* Persona profiles: a DualSense presented as another controller. */
	VDS_PROFILE_DS4 = 2,
	VDS_PROFILE_X360 = 3,
};

enum vds_frame_type {
	VDS_FRAME_STATUS = 0,
	VDS_FRAME_USB_HID_OUT = 1,
	VDS_FRAME_USB_FEATURE_GET = 2,
	VDS_FRAME_USB_FEATURE_SET = 3,
	VDS_FRAME_USB_AUDIO_OUT = 4,
	VDS_FRAME_USB_HID_IN = 5,
	VDS_FRAME_USB_FEATURE_REPLY = 6,
	VDS_FRAME_USB_INTERFACE = 7,
	VDS_FRAME_BT_CONTROL_PACKET = 8,
	VDS_FRAME_BT_INTERRUPT_PACKET = 9,
	VDS_FRAME_USB_AUDIO_IN = 10,
};

enum vds_usb_interface_type {
	VDS_USB_INTERFACE_HID = 0,
	VDS_USB_INTERFACE_AUDIO_OUT = 1,
	VDS_USB_INTERFACE_AUDIO_IN = 2,
};

struct vds_frame_header {
	__u16 type;
	__u16 flags;
	__u32 length;
	__u64 sequence;
};

struct vds_usb_interface_event {
	__u8 interface_number;
	__u8 altsetting;
	__u8 interface_type;
	__u8 reserved;
};

#endif /* _UAPI_VDS_H */
