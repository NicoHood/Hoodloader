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
	if (ram.HID.ID && ram.HID.length == ram.HID.recvlength){
		// set a general and specific flag that a report was made, ignore rawHID
		if (ram.HID.ID != HID_REPORTID_RawKeyboardReport){
			ram.HID.isEmpty[HID_REPORTID_NotAReport] = true;
			ram.HID.isEmpty[ram.HID.ID] = true;
		}

		//write report and reset ID
		memcpy(ReportData, ram.HID.buffer, ram.HID.length);
		*ReportID = ram.HID.ID;
		*ReportSize = ram.HID.length;
		ram.HID.ID = 0;
		ram.HID.recvlength = 0; //just to be sure if you call HID_Task by accident again
		ram.HID.length = 0; //just to be sure if you call HID_Task by accident again

		// always return true, because we cannot compare with >1 report due to ram limit
		// this will forcewrite the report every time
		return true;
	}
	else return false;
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
	// Unused in this demo, since there are no Host->Device reports
	//	uint8_t* LEDReport = (uint8_t*)ReportData;

	if (ReportID == HID_REPORTID_RawKeyboardReport){
		//LEDs_SetAllLEDs(LEDS_ALL_LEDS);
		//while (1); //TODO remove <--

		// Turn on RX LED
		LEDs_TurnOnLEDs(LEDMASK_RX);
		ram.PulseMSRemaining.RxLEDPulse = TX_RX_LED_PULSE_MS;

		// Send bytes
		Serial_SendData(ReportData, ReportSize);
	}
}


void clearHIDReports(void){
	// dont do anything if the main flag is empty
	if (ram.HID.isEmpty[HID_REPORTID_NotAReport]) return;

	// check if every report is empty or not
	for (int i = 1; i < HID_REPORTID_LastNotAReport; i++){

		if (!ram.HID.isEmpty[i])
			clearHIDReport(i);
	}
	// clear the flag that >0 reports were set
	ram.HID.isEmpty[HID_REPORTID_NotAReport] = true;
}

void flushHID(void){
	// TODO timeout? <--
	// try to send until its done
	while (ram.HID.ID && ram.HID.length == ram.HID.recvlength)
		HID_Device_USBTask(&Device_HID_Interface);
}

void clearHIDReport(uint8_t ID){
	// RAW HID cannot be cleared
	if (ID == HID_REPORTID_RawKeyboardReport) return;

	// we have a pending HID report, flush it first
	flushHID();

	// get length of the report if its a valid report
	uint8_t length = getHIDReportLength(ID);
	if (!length) return;

	// save new values and prepare for sending
	ram.HID.length = ram.HID.recvlength = length;
	ram.HID.ID = ID;
	memset(&ram.HID.buffer, 0x00, length);

	// flush HID
	flushHID();

	// save new empty state
	ram.HID.isEmpty[ID] = true;
}

uint8_t getHIDReportLength(uint8_t ID){
	// Get the length of the report
	switch (ram.HID.ID){
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

	default:
		// error, write down this wrong ID report
		return 0;
		break;
	} //end switch
	return 0;
}


void writeToCDC(uint8_t buffer[], uint8_t length){
	// TODO CDC task causes bug on slower pcs 1.6 - 1.7 <--
	// refresh DTR state
	//CDC_Device_USBTask(&VirtualSerial_CDC_Interface);

	// Check if a packet is already enqueued to the host - if so, we shouldn't try to send more data
	// until it completes as there is a chance nothing is listening and a lengthy timeout could occur
	Endpoint_SelectEndpoint(VirtualSerial_CDC_Interface.Config.DataINEndpoint.Address);
	while (!Endpoint_IsINReady());

	// Try to send the next bytes to the host, abort if DTR isnt set to not block serial reading
	bool CurrentDTRState = (VirtualSerial_CDC_Interface.State.ControlLineStates.HostToDevice & CDC_CONTROL_LINE_OUT_DTR);
	if (CurrentDTRState)
		CDC_Device_SendData(&VirtualSerial_CDC_Interface, buffer, length);
}

// Checks for a valid protocol input and writes HID report
void checkNHPProtocol(uint8_t input){
	// set new timeout mark
	ram.PulseMSRemaining.NHPTimeout = NHP_TIMEOUT_MS;

	NHP_Enum_t address = NHPreadChecksum(input, &ram.NHP);

	if (address == 0)
		// reading in progress, dont disturb
		return;
	else if (address < 0){
		// check if previous reading was a valid Control Address and write it down
		checkNHPControlAddressError();

		// ignore command
		uint8_t length = ram.NHP.readlength;
		if (ram.NHP.leadError && address == NHP_COMMAND){
			length ++;
			ram.NHP.leadError = false;
		}

		// error while reading, write down current buffer
		writeToCDC(ram.NHP.readbuffer, length);
		//for (int i = 0; i < length; i++)
		//	RingBuffer_Insert(&ram.USARTtoUSB_Buffer, ram.NHP.readbuffer[i]);

		// reset buffer. Tell the timeout function that this part is written down
		ram.NHP.reset = true;
		return;
	}

	// we have a pending HID report, flush it first
	flushHID();

	// nearly the same priciple like the Protocol itself: check for control address
	if ((address == NHP_ADDRESS_CONTROL) && (((ram.NHP.mWorkData >> 8) & 0xFF) == NHP_USAGE_ARDUINOHID)){
		// check if previous reading was a valid Control Address and write it down
		checkNHPControlAddressError();

		// get the new report ID and reset the buffer
		ram.HID.ID = ram.NHP.mWorkData & 0xFF;
		ram.HID.recvlength = 0;
		memset(ram.HID.buffer, 0, sizeof(ram.HID.buffer));

		// Determine which interface must have its report generated
		ram.HID.length = getHIDReportLength(ram.HID.ID);

		// error, write down this wrong ID report
		if (!ram.HID.length)
			checkNHPControlAddressError();

		// The Protocol received a valid signal with inverse checksum
		// Do not write the buff in the loop above or below, filter it out at the end
	}

	// we already got a pending report
	else if (ram.HID.ID && (address == (((ram.HID.recvlength + 2) / 2) + 1))){
		// check if the new Address is in correct order of HID reports.
		// the first 2 bytes are sent with Address 2 and so on.

		// save the first byte
		ram.HID.buffer[ram.HID.recvlength++] = (ram.NHP.mWorkData & 0xFF);

		// if there is another byte we need (for odd max HID reports important
		// to not write over the buff array)
		if (ram.HID.length != ram.HID.recvlength)
			ram.HID.buffer[ram.HID.recvlength++] = (ram.NHP.mWorkData >> 8);

		// we are ready try to submit the new report to the usb host
		// dont block here, we flush the report on the next reading if needed
		if (ram.HID.length == ram.HID.recvlength)
			flushHID();

		// The Protocol received a valid signal with inverse checksum
		// Do not write the buff in the loop above or below, filter it out
	}

	// we received a corrupt data packet
	else{
		// check if previous reading was a valid Control Address and write it down
		// if not discard the bytes because we assume it is corrupted data
		checkNHPControlAddressError();

		// just a normal Protocol outside our control address (or corrupted packet), write it down
		writeToCDC(ram.NHP.readbuffer, ram.NHP.readlength);
	}
}

void checkNHPControlAddressError(void){
	// only write if a control address was just before, maybe it was a random valid address
	// but if we already received some data we handle this as corrupted data and just
	// discard all the bytes
	if (ram.HID.ID && !ram.HID.recvlength){
		// write the cached buffer (recreate protocol)
		uint8_t buff[6];
		uint8_t length = NHPwriteChecksum(NHP_ADDRESS_CONTROL, (NHP_USAGE_ARDUINOHID << 8) | ram.HID.ID, buff);

		// Writes the NHP read buffer with the given length
		// If host is not listening it will discard all bytes to not block any HID reading
		writeToCDC(buff, length);
	}
	// bug in 1.6 - 1.7.2 found
	flushHID();

	// reset any pending HID reports
	ram.HID.ID = 0;
	ram.HID.recvlength = 0; // just to be sure
	ram.HID.length = 0; // just to be sure
}
