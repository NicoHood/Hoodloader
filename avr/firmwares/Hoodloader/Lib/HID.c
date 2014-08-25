/*
Copyright (c) 2014 NicoHood
See the readme for credit to other people.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

#include "HID.h"

/** LUFA HID Class driver interface configuration and state information. This structure is
*  passed to all HID Class driver functions, so that multiple instances of the same class
*  within a device can be differentiated from one another.
*/
USB_ClassInfo_HID_Device_t Device_HID_Interface =
{
	.Config =
	{
		.InterfaceNumber = INTERFACE_ID_HID,
		.ReportINEndpoint =
		{
			.Address = HID_IN_EPADDR,
			.Size = HID_EPSIZE,
			.Banks = 1,
		},
		.PrevReportINBuffer = NULL, // we dont cache anything
		.PrevReportINBufferSize = sizeof(HID_HIDReport_Data_t),
	},
};


/** HID class driver callback function for the creation of HID reports to the host.
*
*  \param[in]     HIDInterfaceInfo  Pointer to the HID class interface configuration structure being referenced
*  \param[in,out] ReportID    Report ID requested by the host if non-zero, otherwise callback should set to the generated report ID
*  \param[in]     ReportType  Type of the report to create, either HID_REPORT_ITEM_In or HID_REPORT_ITEM_Feature
*  \param[out]    ReportData  Pointer to a buffer where the created report should be stored
*  \param[out]    ReportSize  Number of bytes written in the report (or zero if no report is to be sent)
*
*  \return Boolean \c true to force the sending of the report, \c false to let the library determine if it needs to be sent
*/
bool CALLBACK_HID_Device_CreateHIDReport(USB_ClassInfo_HID_Device_t* const HIDInterfaceInfo,
	uint8_t* const ReportID,
	const uint8_t ReportType,
	void* ReportData,
	uint16_t* const ReportSize)
{
	// only send report if there is actually a new report
	if (ram.HID.writeHID){
		// set a specific flag that a report was written, ignore rawHID
		if (ram.HID.ID != HID_REPORTID_RawKeyboardReport)
			ram.HID.writtenReport |= (1 << (ram.HID.ID - 1));

		//write report
		uint8_t length = getHIDReportLength(ram.HID.ID);
		memcpy(ReportData, ram.HID.buffer, length);
		*ReportID = ram.HID.ID;
		*ReportSize = length;

		// reset the flush flag and pending ID
		ram.HID.writeHID = false;
		ram.HID.ID = 0;

		// always return true, because we cannot compare with >1 report due to ram limit
		// this will forcewrite the report every time
		return true;
	}
	else{
		//TODO improve ? (!(AVR_NO_HID_PIN & AVR_NO_HID_MASK))
		*ReportID = 0;
		*ReportSize = 0;
		return false;
	}
}

/** HID class driver callback function for the processing of HID reports from the host.
*
*  \param[in] HIDInterfaceInfo  Pointer to the HID class interface configuration structure being referenced
*  \param[in] ReportID    Report ID of the received report from the host
*  \param[in] ReportType  The type of report that the host has sent, either HID_REPORT_ITEM_Out or HID_REPORT_ITEM_Feature
*  \param[in] ReportData  Pointer to a buffer where the received report has been stored
*  \param[in] ReportSize  Size in bytes of the received HID report
*/
void CALLBACK_HID_Device_ProcessHIDReport(USB_ClassInfo_HID_Device_t* const HIDInterfaceInfo,
	const uint8_t ReportID,
	const uint8_t ReportType,
	const void* ReportData,
	const uint16_t ReportSize)
{
	return; //TODO

	// Unused in this demo, since there are no Host->Device reports
	//	uint8_t* LEDReport = (uint8_t*)ReportData;

	if (ReportID == HID_REPORTID_RawKeyboardReport){
		//LEDs_SetAllLEDs(LEDS_ALL_LEDS);
		//while (1); //TODO remove <--
		//TODO get this working
		// Turn on RX LED
		LEDs_TurnOnLEDs(LEDMASK_RX);
		ram.PulseMSRemaining.RxLEDPulse = TX_RX_LED_PULSE_MS;

		// Send bytes
		Serial_SendData(ReportData, ReportSize);
	}
}

void flushHID(void){
	// try to send until its done
	while (ram.HID.writeHID){
		// TODO timeout? <--
		HID_Device_USBTask(&Device_HID_Interface);
	}
}

void clearHIDReports(void){
	// dont do anything if no report is written
	if (!ram.HID.writtenReport) return;

	// check if every report is empty or not
	for (int i = 1; i < HID_REPORTID_LastNotAReport; i++)
		clearHIDReport(i);
}

void clearHIDReport(uint8_t ID){
	// return if already cleared
	if (!(ram.HID.writtenReport & (1 << (ID - 1)))) return;

	// get length and check if report ID is valid
	uint8_t length = getHIDReportLength(ID);
	if (!length) return;

	// flush any pending report first
	flushHID();

	// save new values and prepare for sending
	ram.HID.ID = ID;
	memset(&ram.HID.buffer, 0x00, length);
	ram.HID.writeHID = true;

	// flush HID, needed to clear flag below
	flushHID();

	// save new empty flag
	ram.HID.writtenReport &= ~(1 << (ID - 1));
}

uint8_t getHIDReportLength(uint8_t ID){
	// Get the length of the report
	switch (ID){
	case HID_REPORTID_MouseReport:
		return sizeof(HID_MouseReport_Data_t);
		break;

	case HID_REPORTID_KeyboardReport:
		return sizeof(HID_KeyboardReport_Data_t);
		break;

	case HID_REPORTID_RawKeyboardReport:
		return sizeof(HID_RawKeyboardReport_Data_t);
		break;

	case HID_REPORTID_MediaReport:
		return sizeof(HID_MediaReport_Data_t);
		break;

	case HID_REPORTID_SystemReport:
		return sizeof(HID_SystemReport_Data_t);
		break;

	case HID_REPORTID_Gamepad1Report:
	case HID_REPORTID_Gamepad2Report:
		return sizeof(HID_GamepadReport_Data_t);
		break;

	case HID_REPORTID_Joystick1Report:
	case HID_REPORTID_Joystick2Report:
		return sizeof(HID_JoystickReport_Data_t);
		break;
	}
	// error, ID not presented
	return 0;
}

// Checks for a valid protocol input and writes HID report
void checkNHPProtocol(uint8_t input){
	NHP_Enum_t address = NHPreadChecksum(input, &ram.NHP);

	if (address == 0)
		// reading in progress, dont disturb
		return;
	else if (address < 0){
		// error while reading, write down current buffer (except possible new leads)
		LRingBuffer_Append_Buffer(&ram.RingBuffer, ram.NHP.readbuffer, ram.NHP.readlength);
		ram.skipNHP += ram.NHP.readlength;

		// check if previous reading was a valid Control Address and write it down
		// this needs to be appended after the normal protocol!
		checkNHPControlAddressError();
		return;
	}

	// nearly the same priciple like the Protocol itself: check for control address
	if ((address == NHP_ADDRESS_CONTROL) && (((ram.NHP.mWorkData >> 8) & 0xFF) == NHP_USAGE_ARDUINOHID)){
		// make sure there is no pending report, because we overwrite the buffer now
		flushHID();

		// get the new report ID and reset the received length
		uint8_t ID = ram.NHP.mWorkData & 0xFF;
		ram.HID.recvlength = 0;

		// check if previous reading was a valid Control Address and write it down
		checkNHPControlAddressError();

		// save new ID
		ram.HID.ID = ID;
		if (!ram.HID.ID || ram.HID.ID >= HID_REPORTID_LastNotAReport)
			// error, write down this wrong ID report
			// this needs to be appended after the normal protocol!
			checkNHPControlAddressError();
	}

	// we already got a pending report
	else if (ram.HID.ID && (address == (((ram.HID.recvlength + 2) / 2) + 1))){
		// make sure there is no pending report, because we overwrite the buffer now
		flushHID();

		// check if the new Address is in correct order of HID reports.
		// the first 2 bytes are sent with Address 2 and so on.
		uint8_t length = getHIDReportLength(ram.HID.ID);

		// save the first byte
		ram.HID.buffer[ram.HID.recvlength++] = (ram.NHP.mWorkData & 0xFF);

		// if there is another byte we need (for odd max HID reports important
		// to not write over the buff array)
		if (length != ram.HID.recvlength)
			ram.HID.buffer[ram.HID.recvlength++] = (ram.NHP.mWorkData >> 8);

		// we are ready try to submit the new report to the usb host
		// dont block here, we flush the report on the next reading if needed
		if (ram.HID.recvlength == length){
			ram.HID.writeHID = true;
			HID_Device_USBTask(&Device_HID_Interface);
		}
	}

	// we received a corrupt data packet
	else{
		// just a normal Protocol outside our control address (or corrupted packet), write it down
		LRingBuffer_Append_Buffer(&ram.RingBuffer, ram.NHP.readbuffer, ram.NHP.readlength);
		ram.skipNHP += ram.NHP.readlength;

		// check if previous reading was a valid Control Address and write it down
		// if not discard the bytes because we assume it is corrupted data
		// this needs to be appended after the normal protocol!
		checkNHPControlAddressError();
	}
}

void checkNHPControlAddressError(void){
	// make sure there is no pending report, because we overwrite the buffer now
	flushHID();

	// only write if a control address was just before, maybe it was a random valid address
	// but if we already received some data we handle this as corrupted data and discard all bytes
	if (ram.HID.ID && !ram.HID.recvlength){
		// write the cached buffer (recreate protocol)
		uint8_t buff[6];
		uint8_t length = NHPwriteChecksum(NHP_ADDRESS_CONTROL, (NHP_USAGE_ARDUINOHID << 8) | ram.HID.ID, buff);

		// Writes the NHP read buffer with the given length
		LRingBuffer_Append_Buffer(&ram.RingBuffer, buff, length);
		ram.skipNHP += length;
	}

	// reset any pending HID reports
	ram.HID.ID = 0;
}

//void HIDreset(void){
//	// reset any pending HID reports
//	ram.HID.writeHID = false;
//	ram.HID.ID = 0;
//	ram.HID.writtenReport = false;
//}
