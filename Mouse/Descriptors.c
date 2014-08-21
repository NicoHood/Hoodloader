/*
             LUFA Library
     Copyright (C) Dean Camera, 2014.

  dean [at] fourwalledcubicle [dot] com
           www.lufa-lib.org
*/

/*
  Copyright 2014  Dean Camera (dean [at] fourwalledcubicle [dot] com)

  Permission to use, copy, modify, distribute, and sell this
  software and its documentation for any purpose is hereby granted
  without fee, provided that the above copyright notice appear in
  all copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaims all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/

/** \file
 *
 *  USB Device Descriptors, for library use when in USB device mode. Descriptors are special
 *  computer-readable structures which the host requests upon device enumeration, to determine
 *  the device's capabilities and functions.
 */

#include "Descriptors.h"

/** HID class report descriptor. This is a special descriptor constructed with values from the
 *  USBIF HID class specification to describe the reports and capabilities of the HID device. This
 *  descriptor is parsed by the host and its contents used to determine what data (and in what encoding)
 *  the device will send, and what it may be sent back from the host. Refer to the HID specification for
 *  more details on HID report descriptors.
 */	


#define LSB(_x) ((_x) & 0xFF)
#define MSB(_x) ((_x) >> 8)

#define RAWHID_USAGE_PAGE	0xFFC0 // recommended: 0xFF00 to 0xFFFF
#define RAWHID_USAGE		0x0C00 // recommended: 0x0100 to 0xFFFF
#define RAWHID_TX_SIZE 63 // 1 byte for report ID
#define RAWHID_RX_SIZE 63 // 1 byte for report ID

// activate all by default
#define HID_MOUSE_ENABLE 54
#define HID_KEYBOARD_ENABLE 65
#define HID_RAWKEYBOARD_ENABLE 30
#define HID_MEDIA_ENABLE 25
#define HID_SYSTEM_ENABLE 24
#define HID_GAMEPAD1_ENABLE 71
#define HID_GAMEPAD2_ENABLE 71
#define HID_JOYSTICK1_ENABLE 51
#define HID_JOYSTICK2_ENABLE 51
const USB_Descriptor_HIDReport_Datatype_t PROGMEM MouseReport[] =
{
	/* Use the HID class driver's standard Mouse report.
	 *   Min X/Y Axis values: -1
	 *   Max X/Y Axis values:  1
	 *   Min physical X/Y Axis values (used to determine resolution): -1
	 *   Max physical X/Y Axis values (used to determine resolution):  1
	 *   Buttons: 3
	 *   Absolute screen coordinates: false
	 */
	/* Mouse Report */
	HID_RI_USAGE_PAGE(8, 0x01), /* Generic Desktop */
	HID_RI_USAGE(8, 0x02), /* Mouse */
	HID_RI_COLLECTION(8, 0x01), /* Application */
	HID_RI_REPORT_ID(8, HID_REPORTID_MouseReport),
	HID_RI_USAGE(8, 0x01), /* Pointer */
	HID_RI_COLLECTION(8, 0x00), /* Physical */
	HID_RI_USAGE_PAGE(8, 0x09), /* Button */
	HID_RI_USAGE_MINIMUM(8, 0x01),
	HID_RI_USAGE_MAXIMUM(8, 0x03),
	HID_RI_LOGICAL_MINIMUM(8, 0x00),
	HID_RI_LOGICAL_MAXIMUM(8, 0x01),
	HID_RI_REPORT_COUNT(8, 0x03),
	HID_RI_REPORT_SIZE(8, 0x01),
	HID_RI_INPUT(8, HID_IOF_DATA | HID_IOF_VARIABLE | HID_IOF_ABSOLUTE),
	HID_RI_REPORT_COUNT(8, 0x01),
	HID_RI_REPORT_SIZE(8, 0x05),
	HID_RI_INPUT(8, HID_IOF_CONSTANT),
	HID_RI_USAGE_PAGE(8, 0x01), /* Generic Desktop */
	HID_RI_USAGE(8, 0x30), /* Usage X */
	HID_RI_USAGE(8, 0x31), /* Usage Y */
	HID_RI_LOGICAL_MINIMUM(8, -1),
	HID_RI_LOGICAL_MAXIMUM(8, 1),
	HID_RI_PHYSICAL_MINIMUM(8, -1),
	HID_RI_PHYSICAL_MAXIMUM(8, 1),
	HID_RI_REPORT_COUNT(8, 0x02),
	HID_RI_REPORT_SIZE(8, 0x08),
	HID_RI_INPUT(8, HID_IOF_DATA | HID_IOF_VARIABLE | HID_IOF_RELATIVE),
	HID_RI_END_COLLECTION(0),
	HID_RI_END_COLLECTION(0),


#ifdef HID_KEYBOARD_ENABLE
	// Keyboard
	0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
	0x09, 0x06,                    // USAGE (Keyboard)
	0xa1, 0x01,                    // COLLECTION (Application)
	0x85, HID_REPORTID_KeyboardReport, //   REPORT_ID
	0x05, 0x07,                    //   USAGE_PAGE (Keyboard)
	// modifiers
	0x19, 0xe0,                    //   USAGE_MINIMUM (Keyboard LeftControl)
	0x29, 0xe7,                    //   USAGE_MAXIMUM (Keyboard Right GUI)
	0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
	0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
	0x75, 0x01,                    //   REPORT_SIZE (1)
	0x95, 0x08,                    //   REPORT_COUNT (8)
	0x81, 0x02,                    //   INPUT (Data,Var,Abs)
	// reserved byte
	0x95, 0x01,                    //   REPORT_COUNT (1)
	0x75, 0x08,                    //   REPORT_SIZE (8)
	0x81, 0x03,                    //   INPUT (Cnst,Var,Abs)
	// Key[6] Array
	0x95, 0x06,                    //   REPORT_COUNT (6)
	0x75, 0x08,                    //   REPORT_SIZE (8)
	0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
	0x25, 0x65,                    //   LOGICAL_MAXIMUM (101)
	0x05, 0x07,                    //   USAGE_PAGE (Keyboard)
	0x19, 0x00,                    //   USAGE_MINIMUM (Reserved (no event indicated))
	0x29, 0x65,                    //   USAGE_MAXIMUM (Keyboard Application)
	0x81, 0x00,                    //   INPUT (Data,Ary,Abs)
	// LEDs for num lock etc
	0x05, 0x08,   /*   USAGE_PAGE (LEDs) */
	0x19, 0x01,   /*   USAGE_MINIMUM (Num Lock) */
	0x29, 0x05,   /*   USAGE_MAXIMUM (Kana) */
	0x95, 0x05,   /*   REPORT_COUNT (5) */
	0x75, 0x01,   /*   REPORT_SIZE (1) */
	0x91, 0x02,   /*   OUTPUT (Data,Var,Abs) */
	// Reserved 3 bits
	0x95, 0x01,   /*   REPORT_COUNT (1) */
	0x75, 0x03,   /*   REPORT_SIZE (3) */
	0x91, 0x03,   /*   OUTPUT (Cnst,Var,Abs) */
	// end
	0xc0,                          // END_COLLECTION
#endif

#ifdef HID_RAWKEYBOARD_ENABLE
	// RAW HID
	0x06, LSB(RAWHID_USAGE_PAGE), MSB(RAWHID_USAGE_PAGE),	// 30
	0x0A, LSB(RAWHID_USAGE), MSB(RAWHID_USAGE),

	0xA1, 0x01,								// Collection 0x01
	0x85, HID_REPORTID_RawKeyboardReport,   // REPORT_ID
	0x75, 0x08,								// report size = 8 bits
	0x15, 0x00,								// logical minimum = 0
	0x26, 0xFF, 0x00,						// logical maximum = 255

	0x95, RAWHID_TX_SIZE,					// report count TX
	0x09, 0x01,								// usage
	0x81, 0x02,								// Input (array)

	0x95, RAWHID_RX_SIZE,					// report count RX
	0x09, 0x02,								// usage
	0x91, 0x02,								// Output (array)
	0xC0,									// end collection
#endif

#ifdef HID_MEDIA_ENABLE
	// Media
	0x05, 0x0C,						// usage page (consumer device)
	0x09, 0x01,						// usage -- consumer control
	0xA1, 0x01,						// collection (application)
	0x85, HID_REPORTID_MediaReport, // report id
	// 4 media Keys
	0x15, 0x00,						//logical minimum
	0x26, 0xFF, 0xFF,				//logical maximum (3ff)
	0x19, 0x00,						// usage minimum (0)
	0x2A, 0xFF, 0xFF,				//usage maximum (3ff)
	0x95, 0x04,						//report count (4)
	0x75, 0x10,						//report size (16)
	0x81, 0x00,						//input
	0xC0,							//end collection
#endif

#ifdef HID_SYSTEM_ENABLE
	// System
	0x05, 0x01,							// USAGE_PAGE (Generic Desktop)
	0x09, 0x80,							// USAGE (System Control)
	0xa1, 0x01,							// COLLECTION (Application)
	0x85, HID_REPORTID_SystemReport,	//   REPORT_ID
	// 1 system key
	0x15, 0x00,							//   LOGICAL_MINIMUM (0)
	0x26, 0xff, 0x00,					//   LOGICAL_MAXIMUM (255)
	0x19, 0x00,							//   USAGE_MINIMUM (Undefined)
	0x29, 0xff,							//   USAGE_MAXIMUM (System Menu Down)
	0x95, 0x01,							//   REPORT_COUNT (1)
	0x75, 0x08,							//   REPORT_SIZE (8)
	0x81, 0x00,							//   INPUT (Data,Ary,Abs)
	0xc0,								// END_COLLECTION
#endif

#ifdef HID_GAMEPAD1_ENABLE
	// Gamepad1
	0x05, 0x01,							// USAGE_PAGE (Generic Desktop)
	0x09, 0x04,							// USAGE (Joystick)
	0xa1, 0x01,							// COLLECTION (Application)
	0x85, HID_REPORTID_Gamepad1Report,	//   REPORT_ID
	// 32 Buttons
	0x05, 0x09,							//   USAGE_PAGE (Button)
	0x19, 0x01,							//   USAGE_MINIMUM (Button 1)
	0x29, 0x20,							//   USAGE_MAXIMUM (Button 32)
	0x15, 0x00,							//   LOGICAL_MINIMUM (0)
	0x25, 0x01,							//   LOGICAL_MAXIMUM (1)
	0x75, 0x01,							//   REPORT_SIZE (1)
	0x95, 0x20,							//   REPORT_COUNT (32)
	0x81, 0x02,							//   INPUT (Data,Var,Abs)
	// 6 16bit Axis
	0x05, 0x01,							//   USAGE_PAGE (Generic Desktop)
	0xa1, 0x00,							//   COLLECTION (Physical)
	0x09, 0x30,							//     USAGE (X)
	0x09, 0x31,							//     USAGE (Y)
	0x09, 0x32,							//     USAGE (Z)
	0x09, 0x33,							//     USAGE (Rx)
	0x09, 0x34,							//     USAGE (Ry)
	0x09, 0x35,							//     USAGE (Rz)
	0x15, 0x00,							//     LOGICAL_MINIMUM (0)
	0x27, 0xff, 0xff, 0x00, 0x00,		//     LOGICAL_MAXIMUM (65535)
	0x75, 0x10,							//     REPORT_SIZE (16)
	0x95, 0x06,							//     REPORT_COUNT (6)
	0x81, 0x02,							//     INPUT (Data,Var,Abs)
	0xc0,								//   END_COLLECTION
	// 2 Hat Switches
	0x05, 0x01,							//   USAGE_PAGE (Generic Desktop)
	0x09, 0x39,							//   USAGE (Hat switch)
	0x09, 0x39,							//   USAGE (Hat switch)
	0x15, 0x01,							//   LOGICAL_MINIMUM (1)
	0x25, 0x08,							//   LOGICAL_MAXIMUM (8)
	0x95, 0x02,							//   REPORT_COUNT (2)
	0x75, 0x04,							//   REPORT_SIZE (4)
	0x81, 0x02,							//   INPUT (Data,Var,Abs)
	// 8bit Throttle + Rudder
	//0x05, 0x02,							//   USAGE_PAGE (Simulation Controls)
	//0xa1, 0x00,							//   COLLECTION (Physical)
	//0x15, 0x00,							//     LOGICAL_MINIMUM (0)
	//0x26, 0xff, 0x00,					//     LOGICAL_MAXIMUM (255)
	//0x09, 0xbb,							//     USAGE (Throttle)
	//0x09, 0xba,							//     USAGE (Rudder)
	//0x75, 0x08,							//     REPORT_SIZE (8)
	//0x95, 0x02,							//     REPORT_COUNT (2)
	//0x81, 0x02,							//     INPUT (Data,Var,Abs)
	//0xc0,								//   END_COLLECTION
	0xc0,								// END_COLLECTION
#endif

#ifdef HID_GAMEPAD2_ENABLE
	// Gamepad2
	0x05, 0x01,							// USAGE_PAGE (Generic Desktop)
	0x09, 0x04,							// USAGE (Joystick)
	0xa1, 0x01,							// COLLECTION (Application)
	0x85, HID_REPORTID_Gamepad2Report,	//   REPORT_ID
	// 32 Buttons
	0x05, 0x09,							//   USAGE_PAGE (Button)
	0x19, 0x01,							//   USAGE_MINIMUM (Button 1)
	0x29, 0x20,							//   USAGE_MAXIMUM (Button 32)
	0x15, 0x00,							//   LOGICAL_MINIMUM (0)
	0x25, 0x01,							//   LOGICAL_MAXIMUM (1)
	0x75, 0x01,							//   REPORT_SIZE (1)
	0x95, 0x20,							//   REPORT_COUNT (32)
	0x81, 0x02,							//   INPUT (Data,Var,Abs)
	// 6 16bit Axis
	0x05, 0x01,							//   USAGE_PAGE (Generic Desktop)
	0xa1, 0x00,							//   COLLECTION (Physical)
	0x09, 0x30,							//     USAGE (X)
	0x09, 0x31,							//     USAGE (Y)
	0x09, 0x32,							//     USAGE (Z)
	0x09, 0x33,							//     USAGE (Rx)
	0x09, 0x34,							//     USAGE (Ry)
	0x09, 0x35,							//     USAGE (Rz)
	0x15, 0x00,							//     LOGICAL_MINIMUM (0)
	0x27, 0xff, 0xff, 0x00, 0x00,		//     LOGICAL_MAXIMUM (65535)
	0x75, 0x10,							//     REPORT_SIZE (16)
	0x95, 0x06,							//     REPORT_COUNT (6)
	0x81, 0x02,							//     INPUT (Data,Var,Abs)
	0xc0,								//   END_COLLECTION
	// 2 Hat Switches
	0x09, 0x39,							//   USAGE (Hat switch)
	0x09, 0x39,							//   USAGE (Hat switch)
	0x15, 0x01,							//   LOGICAL_MINIMUM (1)
	0x25, 0x08,							//   LOGICAL_MAXIMUM (8)
	0x95, 0x02,							//   REPORT_COUNT (2)
	0x75, 0x04,							//   REPORT_SIZE (4)
	0x81, 0x02,							//   INPUT (Data,Var,Abs)
	// 8bit Throttle + Rudder
	//0x05, 0x02,							//   USAGE_PAGE (Simulation Controls)
	//0xa1, 0x00,							//   COLLECTION (Physical)
	//0x15, 0x00,							//     LOGICAL_MINIMUM (0)
	//0x26, 0xff, 0x00,					//     LOGICAL_MAXIMUM (255)
	//0x09, 0xbb,							//     USAGE (Throttle)
	//0x09, 0xba,							//     USAGE (Rudder)
	//0x75, 0x08,							//     REPORT_SIZE (8)
	//0x95, 0x02,							//     REPORT_COUNT (2)
	//0x81, 0x02,							//     INPUT (Data,Var,Abs)
	//0xc0,								//   END_COLLECTION
	0xc0,								// END_COLLECTION
#endif

#ifdef HID_JOYSTICK1_ENABLE
	// Joystick1
	0x05, 0x01,							// USAGE_PAGE (Generic Desktop)
	0x09, 0x04,							// USAGE (Joystick)
	0xa1, 0x01,							// COLLECTION (Application)
	0x85, HID_REPORTID_Joystick1Report,	//   REPORT_ID
	0xa1, 0x00,							//   COLLECTION (Physical)
	0x05, 0x09,							//     USAGE_PAGE (Button)
	0x19, 0x01,							//     USAGE_MINIMUM (Button 1)
	0x29, 0x02,							//     USAGE_MAXIMUM (Button 2)
	0x15, 0x00,							//     LOGICAL_MINIMUM (0)
	0x25, 0x01,							//     LOGICAL_MAXIMUM (1)
	0x75, 0x01,							//     REPORT_SIZE (1)
	0x95, 0x02,							//     REPORT_COUNT (2)
	0x81, 0x02,							//     INPUT (Data,Var,Abs)
	0x05, 0x01,							//     USAGE_PAGE (Generic Desktop)
	0x09, 0x30,							//     USAGE (X)
	0x09, 0x31,							//     USAGE (Y)
	0x15, 0x00,							//     LOGICAL_MINIMUM (0)
	0x26, 0xff, 0x03,					//     LOGICAL_MAXIMUM (1023)
	0x75, 0x0a,							//     REPORT_SIZE (10)
	0x95, 0x02,							//     REPORT_COUNT (2)
	0x81, 0x02,							//     INPUT (Data,Var,Abs)
	0x75, 0x02,							//     REPORT_SIZE (2)
	0x95, 0x01,							//     REPORT_COUNT (1)
	0x81, 0x03,							//     INPUT (Cnst,Var,Abs)
	0xc0,								//   END_COLLECTION
	0xc0,								// END_COLLECTION
#endif

#ifdef HID_JOYSTICK2_ENABLE
	// Joystick2
	0x05, 0x01,							// USAGE_PAGE (Generic Desktop)
	0x09, 0x04,							// USAGE (Joystick)
	0xa1, 0x01,							// COLLECTION (Application)
	0x85, HID_REPORTID_Joystick2Report,	//   REPORT_ID
	0xa1, 0x00,							//   COLLECTION (Physical)
	0x05, 0x09,							//     USAGE_PAGE (Button)
	0x19, 0x01,							//     USAGE_MINIMUM (Button 1)
	0x29, 0x02,							//     USAGE_MAXIMUM (Button 2)
	0x15, 0x00,							//     LOGICAL_MINIMUM (0)
	0x25, 0x01,							//     LOGICAL_MAXIMUM (1)
	0x75, 0x01,							//     REPORT_SIZE (1)
	0x95, 0x02,							//     REPORT_COUNT (2)
	0x81, 0x02,							//     INPUT (Data,Var,Abs)
	0x05, 0x01,							//     USAGE_PAGE (Generic Desktop)
	0x09, 0x30,							//     USAGE (X)
	0x09, 0x31,							//     USAGE (Y)
	0x15, 0x00,							//     LOGICAL_MINIMUM (0)
	0x26, 0xff, 0x03,					//     LOGICAL_MAXIMUM (1023)
	0x75, 0x0a,							//     REPORT_SIZE (10)
	0x95, 0x02,							//     REPORT_COUNT (2)
	0x81, 0x02,							//     INPUT (Data,Var,Abs)
	0x75, 0x02,							//     REPORT_SIZE (2)
	0x95, 0x01,							//     REPORT_COUNT (1)
	0x81, 0x03,							//     INPUT (Cnst,Var,Abs)
	0xc0,								//   END_COLLECTION
	0xc0								// END_COLLECTION
#endif

};

/** Device descriptor structure. This descriptor, located in FLASH memory, describes the overall
 *  device characteristics, including the supported USB version, control endpoint size and the
 *  number of device configurations. The descriptor is read out by the USB host when the enumeration
 *  process begins.
 */
const USB_Descriptor_Device_t PROGMEM DeviceDescriptor =
{
	.Header                 = {.Size = sizeof(USB_Descriptor_Device_t), .Type = DTYPE_Device},

	.USBSpecification       = VERSION_BCD(1,1,0),
	.Class                  = USB_CSCP_NoDeviceClass,
	.SubClass               = USB_CSCP_NoDeviceSubclass,
	.Protocol               = USB_CSCP_NoDeviceProtocol,

	.Endpoint0Size          = FIXED_CONTROL_ENDPOINT_SIZE,

	.VendorID               = 0x03EB,
	.ProductID              = 0x2041,
	.ReleaseNumber          = VERSION_BCD(0,0,1),

	.ManufacturerStrIndex   = STRING_ID_Manufacturer,
	.ProductStrIndex        = STRING_ID_Product,
	.SerialNumStrIndex      = NO_DESCRIPTOR,

	.NumberOfConfigurations = FIXED_NUM_CONFIGURATIONS
};

/** Configuration descriptor structure. This descriptor, located in FLASH memory, describes the usage
 *  of the device in one of its supported configurations, including information about any device interfaces
 *  and endpoints. The descriptor is read out by the USB host during the enumeration process when selecting
 *  a configuration so that the host may correctly communicate with the USB device.
 */
const USB_Descriptor_Configuration_t PROGMEM ConfigurationDescriptor =
{
	.Config =
		{
			.Header                 = {.Size = sizeof(USB_Descriptor_Configuration_Header_t), .Type = DTYPE_Configuration},

			.TotalConfigurationSize = sizeof(USB_Descriptor_Configuration_t),
			.TotalInterfaces        = 1,

			.ConfigurationNumber    = 1,
			.ConfigurationStrIndex  = NO_DESCRIPTOR,

			.ConfigAttributes       = (USB_CONFIG_ATTR_RESERVED | USB_CONFIG_ATTR_SELFPOWERED),

			.MaxPowerConsumption    = USB_CONFIG_POWER_MA(100)
		},

	.HID_Interface =
		{
			.Header                 = {.Size = sizeof(USB_Descriptor_Interface_t), .Type = DTYPE_Interface},

			.InterfaceNumber        = INTERFACE_ID_Mouse,
			.AlternateSetting       = 0x00,

			.TotalEndpoints         = 1,

			.Class                  = HID_CSCP_HIDClass,
			.SubClass               = HID_CSCP_BootSubclass,
			.Protocol               = HID_CSCP_MouseBootProtocol,

			.InterfaceStrIndex      = NO_DESCRIPTOR
		},

	.HID_MouseHID =
		{
			.Header                 = {.Size = sizeof(USB_HID_Descriptor_HID_t), .Type = HID_DTYPE_HID},

			.HIDSpec                = VERSION_BCD(1,1,1),
			.CountryCode            = 0x00,
			.TotalReportDescriptors = 1,
			.HIDReportType          = HID_DTYPE_Report,
			.HIDReportLength        = sizeof(MouseReport)
		},

	.HID_ReportINEndpoint =
		{
			.Header                 = {.Size = sizeof(USB_Descriptor_Endpoint_t), .Type = DTYPE_Endpoint},

			.EndpointAddress        = MOUSE_EPADDR,
			.Attributes             = (EP_TYPE_INTERRUPT | ENDPOINT_ATTR_NO_SYNC | ENDPOINT_USAGE_DATA),
			.EndpointSize           = MOUSE_EPSIZE,
			.PollingIntervalMS      = 0x05
		}
};

/** Language descriptor structure. This descriptor, located in FLASH memory, is returned when the host requests
 *  the string descriptor with index 0 (the first index). It is actually an array of 16-bit integers, which indicate
 *  via the language ID table available at USB.org what languages the device supports for its string descriptors.
 */
const USB_Descriptor_String_t PROGMEM LanguageString = USB_STRING_DESCRIPTOR_ARRAY(LANGUAGE_ID_ENG);

/** Manufacturer descriptor string. This is a Unicode string containing the manufacturer's details in human readable
 *  form, and is read out upon request by the host when the appropriate string ID is requested, listed in the Device
 *  Descriptor.
 */
const USB_Descriptor_String_t PROGMEM ManufacturerString = USB_STRING_DESCRIPTOR(L"Dean Camera");

/** Product descriptor string. This is a Unicode string containing the product's details in human readable form,
 *  and is read out upon request by the host when the appropriate string ID is requested, listed in the Device
 *  Descriptor.
 */
const USB_Descriptor_String_t PROGMEM ProductString = USB_STRING_DESCRIPTOR(L"LUFA Mouse Demo");

/** This function is called by the library when in device mode, and must be overridden (see library "USB Descriptors"
 *  documentation) by the application code so that the address and size of a requested descriptor can be given
 *  to the USB library. When the device receives a Get Descriptor request on the control endpoint, this function
 *  is called so that the descriptor details can be passed back and the appropriate descriptor sent back to the
 *  USB host.
 */
uint16_t CALLBACK_USB_GetDescriptor(const uint16_t wValue,
                                    const uint8_t wIndex,
                                    const void** const DescriptorAddress)
{
	const uint8_t  DescriptorType   = (wValue >> 8);
	const uint8_t  DescriptorNumber = (wValue & 0xFF);

	const void* Address = NULL;
	uint16_t    Size    = NO_DESCRIPTOR;

	switch (DescriptorType)
	{
		case DTYPE_Device:
			Address = &DeviceDescriptor;
			Size    = sizeof(USB_Descriptor_Device_t);
			break;
		case DTYPE_Configuration:
			Address = &ConfigurationDescriptor;
			Size    = sizeof(USB_Descriptor_Configuration_t);
			break;
		case DTYPE_String:
			switch (DescriptorNumber)
			{
				case STRING_ID_Language:
					Address = &LanguageString;
					Size    = pgm_read_byte(&LanguageString.Header.Size);
					break;
				case STRING_ID_Manufacturer:
					Address = &ManufacturerString;
					Size    = pgm_read_byte(&ManufacturerString.Header.Size);
					break;
				case STRING_ID_Product:
					Address = &ProductString;
					Size    = pgm_read_byte(&ProductString.Header.Size);
					break;
			}

			break;
		case HID_DTYPE_HID:
			Address = &ConfigurationDescriptor.HID_MouseHID;
			Size    = sizeof(USB_HID_Descriptor_HID_t);
			break;
		case HID_DTYPE_Report:
			Address = &MouseReport;
			Size    = sizeof(MouseReport);
			break;
	}

	*DescriptorAddress = Address;
	return Size;
}

