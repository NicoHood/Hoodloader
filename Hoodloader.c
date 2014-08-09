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

/** \file
*
*  Main source file for the VirtualSerialMouse demo. This file contains the main tasks of
*  the demo and is responsible for the initial application hardware configuration.
*/

#include "Hoodloader.h"

//================================================================================
// RAM
//================================================================================

// global variable to hold specific ram data
// because we only have 500 bytes we have to free some memory for different modes
static struct{
	// indicates what mode we are in
	uint8_t mode;

	union{
		// normal mode if HID is on
		struct{
			// Circular buffer to hold data from the serial port before it is sent to the host.
			RingBuffer_t USARTtoUSB_Buffer;

			// Underlying data buffer for \ref USARTtoUSB_Buffer, where the stored bytes are located.
			uint8_t      USARTtoUSB_Buffer_Data[128];

			// Pulse generation counters to keep track of the number of milliseconds remaining for each pulse type
			struct{
				uint8_t TxLEDPulse; // Milliseconds remaining for data Tx LED pulse
				uint8_t RxLEDPulse; // Milliseconds remaining for data Rx LED pulse
			}PulseMSRemaining;

			// variables to save hid states
			struct{
				// variable to perform a "HID flush" and to indicate what report should be written down
				uint8_t ID;
				// length of the report
				uint8_t length;
				// number of bytes received
				uint8_t recvlength;
				// Buffer for the incoming HID report
				uint8_t buffer[sizeof(HID_HIDReport_Data_t)];
			}HID;

			struct{
				// in progress reading data
				uint8_t mBlocks;
				uint32_t mWorkData;
				uint8_t mErrorLevel;

				// buffer for read operations
				uint8_t readbuffer[6];
				uint8_t readlength;
			}NHP;

		};

		// if baud == 19200 AVRISP mode
		struct{
			int error; //TODO improve types
			int pmode;
			int _addr; // here
			struct{
				int pagesize;
				int eepromsize;
			} param;

			uint8_t buff[256];
		} isp;

	};
}ram;

//================================================================================
// LUFA CDC + HID
//================================================================================

/** LUFA CDC Class driver interface configuration and state information. This structure is
*  passed to all CDC Class driver functions, so that multiple instances of the same class
*  within a device can be differentiated from one another.
*/
USB_ClassInfo_CDC_Device_t VirtualSerial_CDC_Interface =
{
	.Config =
	{
		.ControlInterfaceNumber = INTERFACE_ID_CDC_CCI,
		.DataINEndpoint =
		{
			.Address = CDC_TX_EPADDR,
			.Size = CDC_TXRX_EPSIZE,
			.Banks = 1,
		},
		.DataOUTEndpoint =
			{
				.Address = CDC_RX_EPADDR,
				.Size = CDC_TXRX_EPSIZE,
				.Banks = 1,
			},
			.NotificationEndpoint =
				{
					.Address = CDC_NOTIFICATION_EPADDR,
					.Size = CDC_NOTIFICATION_EPSIZE,
					.Banks = 1,
				},
	},
};

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

//================================================================================
// Main
//================================================================================

/** Main program entry point. This routine contains the overall program flow, including initial
*  setup of all components and the main program loop.
*/
int main(void)
{
	ram.mode = MODE_NONE;

	SetupHardware();

	GlobalInterruptEnable();

	for (;;)
	{
		// change ram.mode depending on the selected CDC baud rate
		selectMode();

		// run main code, depending on the selected mode
		if (ram.mode == MODE_AVRISP)
			avrisp();
		else{

			// read in bytes from the CDC interface
			int16_t ReceivedByte = CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);
			if (!(ReceivedByte < 0)){
				// Turn on RX LED
				LEDs_TurnOnLEDs(LEDMASK_RX);
				ram.PulseMSRemaining.RxLEDPulse = TX_RX_LED_PULSE_MS;

				// Send byte directly
				Serial_SendByte(ReceivedByte);
			}

			// read in bytes from the Serial buffer
			uint16_t BufferCount = RingBuffer_GetCount(&ram.USARTtoUSB_Buffer);
			if (BufferCount)
			{
				// Turn on TX LED
				LEDs_TurnOnLEDs(LEDMASK_TX);
				ram.PulseMSRemaining.TxLEDPulse = TX_RX_LED_PULSE_MS;



				Endpoint_SelectEndpoint(VirtualSerial_CDC_Interface.Config.DataINEndpoint.Address);

				// Check if a packet is already enqueued to the host - if so, we shouldn't try to send more data
				// until it completes as there is a chance nothing is listening and a lengthy timeout could occur
				if (Endpoint_IsINReady())
				{
					// Never send more than one bank size less one byte to the host at a time, so that we don't block
					// while a Zero Length Packet (ZLP) to terminate the transfer is sent if the host isn't listening
					uint8_t BytesToSend = MIN(BufferCount, (CDC_TXRX_EPSIZE - 1));

					/// Read bytes from the USART receive buffer into the USB IN endpoint */
					while (BytesToSend--)
					{
						if (ram.mode == MODE_DEFAULT){
							// Try to send the next byte of data to the host, abort if there is an error without dequeuing
							if (CDC_Device_SendByte(&VirtualSerial_CDC_Interface,
								RingBuffer_Peek(&ram.USARTtoUSB_Buffer)) != ENDPOINT_READYWAIT_NoError)
							{
								break;
							}
						}

						// Dequeue the already sent byte from the buffer now we have confirmed that no transmission error occurred
						uint8_t b = RingBuffer_Remove(&ram.USARTtoUSB_Buffer);

						// main function to proceed HID input checks
						if (ram.mode == MODE_HID)
							checkNHPProtocol(b);
					}
				}


			}

			// Check if the led flush timer has expired
			if (TIFR0 & (1 << TOV0)){
				// reset the timer
				TIFR0 |= (1 << TOV0);

				// Turn off TX LED(s) once the TX pulse period has elapsed
				if (ram.PulseMSRemaining.TxLEDPulse && !(--ram.PulseMSRemaining.TxLEDPulse)){

					// if reading has timed out write the buffers down the serial
					if (ram.mode == MODE_HID){
						// check if previous reading was a valid Control Address and write it down
						checkNHPControlAddressError();

						// write the rest of the cached NHP buffer down
						writeToCDC(ram.NHP.readbuffer, ram.NHP.readlength);
						resetNHPbuffer();
					}

					LEDs_TurnOffLEDs(LEDMASK_TX);
				}

				// Turn off RX LED(s) once the RX pulse period has elapsed
				if (ram.PulseMSRemaining.RxLEDPulse && !(--ram.PulseMSRemaining.RxLEDPulse))
					LEDs_TurnOffLEDs(LEDMASK_RX);
			}

			// get new reports from the PC side and try to send pending reports
			if (ram.mode == MODE_HID)
				HID_Device_USBTask(&Device_HID_Interface);
		}

		CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
		USB_USBTask();
	}
}

//================================================================================
// Hardwaresetup
//================================================================================

/** Configures the board hardware and chip peripherals for the demo's functionality. */
void SetupHardware(void)
{
	/* Disable watchdog if enabled by bootloader/fuses */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();

	/* Hardware Initialization */
	Serial_Init(115200, true);

	/* Must turn off USART before reconfiguring it, otherwise incorrect operation may occur */
	// Added for correct Serial connection at baud 115200 <--
	PORTD |= (1 << PD3); // Turn ON Tx while USART is being reconfigured
	UCSR1B = 0;
	UCSR1A = 0;
	UCSR1C = 0;

	// these are values for baud 115200. i just read them manual from change
	// its needed to start with baud 115200 on powerup
	UCSR1C = ((1 << UCSZ11) | (1 << UCSZ10)); //C: 0x06
	UCSR1A = (1 << U2X1); //A: 0x02
	UCSR1B = ((1 << RXCIE1) | (1 << TXEN1) | (1 << RXEN1)); //B: 0x98
	PORTD &= ~(1 << PD3); // And turn OFF Tx once USART has been reconfigured (this is overridden by TXEN)

	LEDs_Init();
	USB_Init();

	/* Start the flush timer so that overflows occur rapidly to push received bytes to the USB interface */
	TCCR0B = (1 << CS02);

	/* Pull target /RESET line high */
	AVR_RESET_LINE_PORT |= AVR_RESET_LINE_MASK;
	AVR_RESET_LINE_DDR |= AVR_RESET_LINE_MASK;

	// set hardware SS to output so we can use SPI master mode
	AVR_SPI_DDR |= (1 << AVR_HARDWARE_SS);

	// Hardwaresetup to turn off the HID function with shorting the MOSI pin with GND next to it
	// do not short this pin in AVRISP mode!!!
	AVR_NO_HID_DDR &= ~AVR_NO_HID_MASK; // Input
	AVR_NO_HID_PORT |= AVR_NO_HID_MASK; // Pullup
}


//================================================================================
// Mode selection
//================================================================================

void selectMode(void){
	// check what mode should run
	uint8_t oldMode = ram.mode;
	uint32_t currentBaud = VirtualSerial_CDC_Interface.State.LineEncoding.BaudRateBPS;

	// HID only works for baud 115200 or not configured (startup, no host connection) to work at maximum speed and after reprogramming
	if (currentBaud == 0 || currentBaud == 115200)
		ram.mode = MODE_HID;
	else if (currentBaud == AVRISP_BAUD)
		ram.mode = MODE_AVRISP;
	else ram.mode = MODE_DEFAULT;

	// check if mode has changed
	if (oldMode != ram.mode){
		// coming from AVR ISP, setup ram properly
		if (oldMode == MODE_AVRISP || oldMode == MODE_NONE){
			// Serial tx buffers Setup
			RingBuffer_InitBuffer(&ram.USARTtoUSB_Buffer, ram.USARTtoUSB_Buffer_Data, sizeof(ram.USARTtoUSB_Buffer_Data));
			// reset LEDs
			ram.PulseMSRemaining.TxLEDPulse = 0;
			ram.PulseMSRemaining.RxLEDPulse = 0;
			LEDs_TurnOffLEDs(LEDS_ALL_LEDS);
		}
		if (ram.mode == MODE_HID){
			// HID Setup
			resetNHPbuffer();
			ram.HID.ID = 0;
		}
		else if (ram.mode == MODE_AVRISP){
			// AVR ISP Setup
			ram.isp.error = 0;
			ram.isp.pmode = 0;
			ram.isp._addr = 0; // just to be sure
			LEDs_TurnOffLEDs(LEDS_ALL_LEDS);

			// Hardwaresetup to turn off the HID function with shorting the MOSI pin with GND next to it
			// do not short this pin in AVRISP mode!!!
			// moved here so the pin is INPUT to not damage anything
			AVR_NO_HID_DDR &= ~AVR_NO_HID_MASK; // Input
			AVR_NO_HID_PORT |= AVR_NO_HID_MASK; // Pullup
		}
	}
}

//================================================================================
// Lufa USB functions
//================================================================================

/** Event handler for the library USB Connection event. */
void EVENT_USB_Device_Connect(void)
{
	//LEDs_SetAllLEDs(LEDMASK_USB_ENUMERATING);
}

/** Event handler for the library USB Disconnection event. */
void EVENT_USB_Device_Disconnect(void)
{
	//LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);
}

/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void) //<--new
{
	bool ConfigSuccess = true;

	ConfigSuccess &= HID_Device_ConfigureEndpoints(&Device_HID_Interface);
	ConfigSuccess &= CDC_Device_ConfigureEndpoints(&VirtualSerial_CDC_Interface);

	// This seems to be for the HID to get accurate reports
	USB_Device_EnableSOFEvents(); //<--new

	// Turn off Leds if everything is okay
	//LEDs_SetAllLEDs(ConfigSuccess ? LEDS_NO_LEDS : LEDS_ALL_LEDS);
}

/** Event handler for the library USB Unhandled Control Request event. */
void EVENT_USB_Device_UnhandledControlRequest(void)
{
	CDC_Device_ProcessControlRequest(&VirtualSerial_CDC_Interface);
}

/** Event handler for the CDC Class driver Line Encoding Changed event.
*
*  \param[in] CDCInterfaceInfo  Pointer to the CDC class interface configuration structure being referenced
*/
void EVENT_CDC_Device_LineEncodingChanged(USB_ClassInfo_CDC_Device_t* const CDCInterfaceInfo)
{
	uint8_t ConfigMask = 0;

	switch (CDCInterfaceInfo->State.LineEncoding.ParityType)
	{
	case CDC_PARITY_Odd:
		ConfigMask = ((1 << UPM11) | (1 << UPM10));
		break;
	case CDC_PARITY_Even:
		ConfigMask = (1 << UPM11);
		break;
	}

	if (CDCInterfaceInfo->State.LineEncoding.CharFormat == CDC_LINEENCODING_TwoStopBits)
		ConfigMask |= (1 << USBS1);

	switch (CDCInterfaceInfo->State.LineEncoding.DataBits)
	{
	case 6:
		ConfigMask |= (1 << UCSZ10);
		break;
	case 7:
		ConfigMask |= (1 << UCSZ11);
		break;
	case 8:
		ConfigMask |= ((1 << UCSZ11) | (1 << UCSZ10));
		break;
	}

	/* Must turn off USART before reconfiguring it, otherwise incorrect operation may occur */
	PORTD |= (1 << PD3); // Turn ON Tx while USART is being reconfigured
	UCSR1B = 0;
	UCSR1A = 0;
	UCSR1C = 0;

	/* Special case 57600 baud for compatibility with the ATmega328 bootloader. */
	UBRR1 = (CDCInterfaceInfo->State.LineEncoding.BaudRateBPS == 57600)
		? SERIAL_UBBRVAL(CDCInterfaceInfo->State.LineEncoding.BaudRateBPS)
		: SERIAL_2X_UBBRVAL(CDCInterfaceInfo->State.LineEncoding.BaudRateBPS);

	UCSR1C = ConfigMask;
	UCSR1A = (CDCInterfaceInfo->State.LineEncoding.BaudRateBPS == 57600) ? 0 : (1 << U2X1);
	UCSR1B = ((1 << RXCIE1) | (1 << TXEN1) | (1 << RXEN1));
	PORTD &= ~(1 << PD3); // And turn OFF Tx once USART has been reconfigured (this is overridden by TXEN)
}

/** ISR to manage the reception of data from the serial port, placing received bytes into a circular buffer
*  for later transmission to the host.
*/
ISR(USART1_RX_vect, ISR_BLOCK)
{
	uint8_t ReceivedByte = UDR1;

	// only save the byte if AVRISP is off and if usb device is ready
	if (ram.mode != MODE_AVRISP && USB_DeviceState == DEVICE_STATE_Configured)
		RingBuffer_Insert(&ram.USARTtoUSB_Buffer, ReceivedByte);
}

/** Event handler for the CDC Class driver Host-to-Device Line Encoding Changed event.
*
*  \param[in] CDCInterfaceInfo  Pointer to the CDC class interface configuration structure being referenced
*/
void EVENT_CDC_Device_ControLineStateChanged(USB_ClassInfo_CDC_Device_t* const CDCInterfaceInfo)
{
	bool CurrentDTRState = (CDCInterfaceInfo->State.ControlLineStates.HostToDevice & CDC_CONTROL_LINE_OUT_DTR);

	if (CurrentDTRState){
		AVR_RESET_LINE_PORT &= ~AVR_RESET_LINE_MASK;
	}
	else{
		AVR_RESET_LINE_PORT |= AVR_RESET_LINE_MASK;
		LEDs_SetAllLEDs(LEDS_NO_LEDS); //<--new
	}
}

/** Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest(void)
{
	CDC_Device_ProcessControlRequest(&VirtualSerial_CDC_Interface);
	HID_Device_ProcessControlRequest(&Device_HID_Interface); //<--new
}

/** Event handler for the USB device Start Of Frame event. */
void EVENT_USB_Device_StartOfFrame(void)
{
	HID_Device_MillisecondElapsed(&Device_HID_Interface);
}

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

// Checks for a valid protocol input and writes HID report
void checkNHPProtocol(uint8_t input){
	uint8_t address = NHPreadChecksum(input);

	// if we have got an address this also means the checksum is correct!
	// if not the buff is automatically written down
	if (!address)
		return;

	// we have a pending HID report, flush it first
	if (ram.HID.ID && ram.HID.length == ram.HID.recvlength){
		// TODO timeout? <--
		while (ram.HID.ID)
			HID_Device_USBTask(&Device_HID_Interface);
	}

	// nearly the same priciple like the Protocol itself: check for control address
	if ((address == NHP_ADDRESS_CONTROL) && (((ram.NHP.mWorkData >> 8) & 0xFF) == NHP_USAGE_ARDUINOHID)){
		// check if previous reading was a valid Control Address and write it down
		checkNHPControlAddressError();

		// get the new report ID and reset the buffer
		ram.HID.ID = ram.NHP.mWorkData & 0xFF;
		ram.HID.recvlength = 0;
		memset(ram.HID.buffer, 0, sizeof(ram.HID.buffer));

		// Determine which interface must have its report generated
		switch (ram.HID.ID){
		case HID_REPORTID_MouseReport:
			ram.HID.length = sizeof(HID_MouseReport_Data_t);
			break;

		case HID_REPORTID_KeyboardReport:
			ram.HID.length = sizeof(HID_KeyboardReport_Data_t);
			break;

		case HID_REPORTID_RawKeyboardReport:
			ram.HID.length = sizeof(HID_RawKeyboardReport_Data_t);
			break;

		case HID_REPORTID_MediaReport:
			ram.HID.length = sizeof(HID_MediaReport_Data_t);
			break;

		case HID_REPORTID_SystemReport:
			ram.HID.length = sizeof(HID_SystemReport_Data_t);
			break;

		case HID_REPORTID_Gamepad1Report:
		case HID_REPORTID_Gamepad2Report:
			ram.HID.length = sizeof(HID_GamepadReport_Data_t);
			break;

		case HID_REPORTID_Joystick1Report:
		case HID_REPORTID_Joystick2Report:
			ram.HID.length = sizeof(HID_JoystickReport_Data_t);
			break;

		default:
			// error, write down this wrong ID report
			checkNHPControlAddressError();
			break;
		} //end switch

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
			HID_Device_USBTask(&Device_HID_Interface);

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

	// in any case: clear NHP buffer now and start a new reading
	resetNHPbuffer();
}

void resetNHPbuffer(void){
	ram.NHP.readlength = 0;
	ram.NHP.mBlocks = 0;
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

	// reset any pending HID reports
	ram.HID.ID = 0;
}

void writeToCDC(uint8_t buffer[], uint8_t length){
	// Check if a packet is already enqueued to the host - if so, we shouldn't try to send more data
	// until it completes as there is a chance nothing is listening and a lengthy timeout could occur
	Endpoint_SelectEndpoint(VirtualSerial_CDC_Interface.Config.DataINEndpoint.Address);
	while (!Endpoint_IsINReady());

	// Try to send the next bytes to the host, abort if DTR isnt set to not block serial reading
	bool CurrentDTRState = (VirtualSerial_CDC_Interface.State.ControlLineStates.HostToDevice & CDC_CONTROL_LINE_OUT_DTR);
	if (CurrentDTRState)
		CDC_Device_SendData(&VirtualSerial_CDC_Interface, buffer, length);
}

//================================================================================
// Read NHP
//================================================================================

// reads two bytes and check its inverse
uint8_t NHPreadChecksum(uint8_t input){
	//write input to the buffer
	ram.NHP.readbuffer[ram.NHP.readlength] = input;
	ram.NHP.readlength++;

	// check the lead/end/data indicator
	switch (input & NHP_MASK_START){

	case(NHP_MASK_LEAD) :
	{
		// read command indicator or block length
		uint8_t blocks = (input & NHP_MASK_LENGTH) >> 3;

		// ignore command, return 0 write buff down completely
		if (blocks == 0 || blocks == 1)
			break;

		else if (blocks == 7){
			// save block length + first 4 data bits (special case)
			ram.NHP.mWorkData = input & NHP_MASK_DATA_4BIT;
			blocks -= 2;
		}
		else{
			// save block length + first 3 data bits
			ram.NHP.mWorkData = input & NHP_MASK_DATA_3BIT;
			blocks--;
		}

		// we were still reading!  Log an error
		if (ram.NHP.mBlocks){
			// check if previous reading was a valid Control Address and write it down
			checkNHPControlAddressError();
			// write down the last signal but keep lead
			// substract 1 more because we already added the count
			writeToCDC(ram.NHP.readbuffer, ram.NHP.readlength - 1);
			ram.NHP.readbuffer[0] = ram.NHP.readbuffer[ram.NHP.readlength - 1];
			ram.NHP.readlength = 1;
		}
		// save new block length
		ram.NHP.mBlocks = blocks;
		return 0; // everything is okay
	}
						break;

	case(NHP_MASK_END) :
	{
		if (ram.NHP.mBlocks == 1){
			// save data + address
			// we know its a valid input, left some things out here
			if (((ram.NHP.mWorkData & 0xFFFF) ^ (ram.NHP.mWorkData >> 16)) == 0xFFFF){
				uint8_t address = (input & 0x3F) + 1;
				// do NOT reset for new reading, cause the values might be wrong and need to be written down again.
				return address;
			}
		}
		// wrong checksum or wrong end, write down buffer
	}
					   break;

	default:
	{
		if (ram.NHP.mBlocks >= 2){
			ram.NHP.mBlocks--;
			// get next 7 bits of data
			ram.NHP.mWorkData <<= 7;
			// dont need &NHP_MASK_DATA_7BIT because first bit is zero!
			ram.NHP.mWorkData |= input;
			return 0; // everything is okay
		}
		// log an error, expecting an address or header byte
	}
		break;
	} // end switch

	// check if previous reading was a valid Control Address and write it down
	checkNHPControlAddressError();

	// invalid input, write down buffer
	writeToCDC(ram.NHP.readbuffer, ram.NHP.readlength);
	resetNHPbuffer();
	return 0;
}

//================================================================================
// Write NHP
//================================================================================

// writes two bytes with its inverse
uint8_t NHPwriteChecksum(uint8_t address, uint16_t indata, uint8_t* buff){
	// create checksum data
	uint32_t temp = ~indata;
	uint32_t data = (temp << 16) | indata;

	// start with the maximum size of blocks
	uint8_t blocks = 7;

	// check for the first 7 bit block that doesnt fit into the first 3 bits
	while (blocks > 2){
		uint8_t nextvalue = (data >> (7 * (blocks - 3)));
		if (nextvalue > NHP_MASK_DATA_3BIT){
			// special case for the MSB
			if (blocks == 7) {
				buff[0] = nextvalue;
				blocks--;
			}
			break;
		}
		else{
			// write the possible first 3 bits and check again after if zero
			buff[0] = nextvalue;
			blocks--;
			// we have our first bits, stop (nonzero)
			if (nextvalue)
				break;
		}
	}

	// write the rest of the data bits
	uint8_t datablocks = blocks - 2;
	while (datablocks > 0){
		buff[datablocks] = data & NHP_MASK_DATA_7BIT;
		data >>= 7;
		datablocks--;
	}

	// write lead + length mask
	buff[0] |= NHP_MASK_LEAD | (blocks << 3);

	// write end mask
	buff[blocks - 1] = NHP_MASK_END | ((address - 1) & NHP_MASK_ADDRESS);

	// return the length
	return blocks;
}

//================================================================================
// AVRISP
//================================================================================

void avrisp(void){
	// is pmode active?
	if (ram.isp.pmode) LEDs_TurnOnLEDs(LEDS_PMODE);
	else LEDs_TurnOffLEDs(LEDS_PMODE);

	// is there an error?
	if (ram.isp.error) LEDs_TurnOnLEDs(LEDS_ERR);
	else LEDs_TurnOffLEDs(LEDS_ERR);

	// read in bytes from the CDC interface
	int16_t ReceivedByte = CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);
	if (!(ReceivedByte < 0)){
		switch (ReceivedByte) {
		case STK_GET_SYNC:
			ram.isp.error = 0;
			replyOK();
			break;
		case STK_GET_SIGNON:
			if (getch() == CRC_EOP) {
				sendCDCbyte(STK_INSYNC);
				sendCDCbyte('A');
				sendCDCbyte('V');
				sendCDCbyte('R');
				sendCDCbyte(' ');
				sendCDCbyte('I');
				sendCDCbyte('S');
				sendCDCbyte('P');
				sendCDCbyte(STK_OK);
			}
			break;
		case STK_GET_PARM:
			get_parameters(getch());
			break;
		case STK_SET_PARM:
			fill(20);
			set_parameters();
			replyOK();
			break;
		case STK_SET_PARM_EXT: // extended parameters - ignore for now
			fill(5);
			replyOK();
			break;

		case STK_PMODE_START:
			start_pmode();
			replyOK();
			break;
		case STK_SET_ADDR:
			ram.isp._addr = getch();
			ram.isp._addr += 256 * getch();
			replyOK();
			break;

		case STK_PROG_FLASH:
			//uint8_t low = getch();
			getch();
			//uint8_t high = getch();
			getch();
			replyOK();
			break;
		case STK_PROG_DATA:
			//uint8_t data = getch();
			getch();
			replyOK();
			break;

		case STK_PROG_PAGE:
			program_page();
			break;

		case STK_READ_PAGE:
			read_page();
			break;

		case STK_UNIVERSAL:
			universal();
			break;
		case STK_PMODE_END:
			ram.isp.error = 0;
			end_pmode();
			replyOK();
			break;

		case STK_READ_SIGN:
			read_signature();
			break;

			// expecting a command, not CRC_EOP
			// this is how we can get back in sync
		case CRC_EOP:
			ram.isp.error++;
			sendCDCbyte(STK_NOSYNC);
			break;

			// anything else we will return STK_UNKNOWN
		default:
			ram.isp.error++;
			if (CRC_EOP == getch())
				sendCDCbyte(STK_UNKNOWN);
			else
				sendCDCbyte(STK_NOSYNC);
		}
	}

}

void sendCDCbyte(uint8_t b){
	// try to send until sucess
	while (CDC_Device_SendByte(&VirtualSerial_CDC_Interface, b) != ENDPOINT_READYWAIT_NoError){
		CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
		USB_USBTask();
		while (1){
			// TODO remove this freezing loop!
			LEDs_TurnOnLEDs(LEDS_ERR);
			_delay_ms(100);
			LEDs_TurnOnLEDs(LEDS_ERR);
			_delay_ms(100);
		}
	}
}

uint8_t getch() {
	int16_t ReceivedByte = -1;
	// wait until CDC sends a byte
	while (ReceivedByte < 0)
		ReceivedByte = CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);
	return ReceivedByte;
}

void fill(int n) {
	// fill the buffer with the number of bytes passed in from CDC Serial 
	for (int x = 0; x < n; x++)
		ram.isp.buff[x] = getch();
}

void get_parameters(uint8_t c) {
	switch (c) {
	case 0x80:
		breply(HWVER);
		break;
	case 0x81:
		breply(SWMAJ);
		break;
	case 0x82:
		breply(SWMIN);
		break;
	case 0x93:
		breply('S'); // serial programmer
		break;
	default:
		breply(0);
	}
}

void set_parameters(void) {
	// parameters not used yet <--

	// call this after reading paramter packet into buff[]
	//param.devicecode = buff[0];
	//param.revision = buff[1];
	//param.progtype = buff[2];
	//param.parmode = buff[3];
	//param.polling = buff[4];
	//param.selftimed = buff[5];
	//param.lockbytes = buff[6];
	//param.fusebytes = buff[7];
	//param.flashpoll = buff[8];
	// ignore buff[9] (= buff[8])
	// following are 16 bits (big endian)
#define beget16(addr) (*addr * 256 + *(addr+1) )
	//param.eeprompoll = beget16(&buff[10]);
	ram.isp.param.pagesize = beget16(&ram.isp.buff[12]);
	ram.isp.param.eepromsize = beget16(&ram.isp.buff[14]);

	// 32 bits flashsize (big endian)
	//param.flashsize = buff[16] * 0x01000000
	//	+ buff[17] * 0x00010000
	//	+ buff[18] * 0x00000100
	//	+ buff[19];

}

void breply(uint8_t b) {
	if (CRC_EOP == getch()) {  // EOP should be next char
		sendCDCbyte(STK_INSYNC);
		sendCDCbyte(b);
		sendCDCbyte(STK_OK);
	}
	else {
		sendCDCbyte(STK_NOSYNC);
		ram.isp.error++;
	}
}

void replyOK(void) {
	//  if (EOP_SEEN == true) {
	if (CRC_EOP == getch()) {  // EOP should be next char
		sendCDCbyte(STK_INSYNC);
		sendCDCbyte(STK_OK);
	}
	else {
		// signalize Error
		LEDs_TurnOnLEDs(LEDS_PMODE);
		_delay_ms(50);
		LEDs_TurnOffLEDs(LEDS_PMODE);
		_delay_ms(50);
		LEDs_TurnOnLEDs(LEDS_PMODE);
		_delay_ms(50);
		LEDs_TurnOffLEDs(LEDS_PMODE);

		sendCDCbyte(STK_NOSYNC);
		ram.isp.error++;
	}
}

void start_pmode(void) {
	spi_init();
	// following delays may not work on all targets...
	DDRB |= (1 << AVR_SS); // OUTPUT
	PORTB |= (1 << AVR_SS); // HIGH
	DDRB |= (1 << AVR_SCK); // OUTPUT
	PORTB &= ~(1 << AVR_SCK); // LOW
	_delay_ms(50 + EXTRA_SPI_DELAY);
	PORTB &= ~(1 << AVR_SS); // LOW
	_delay_ms(50 + EXTRA_SPI_DELAY); // extra delay added from adafruit <--
	DDRB &= ~(1 << AVR_MISO); // INPUT
	DDRB |= (1 << AVR_MOSI); // OUTPUT
	spi_transaction(0xAC, 0x53, 0x00, 0x00);
	ram.isp.pmode = 1;
}

void spi_init(void) {
	SPCR = 0x53;
#ifdef ISP_LOW_SPEED
	SPCR = SPCR | B00000011;
#endif
	SPSR;
	SPDR;
}

uint8_t spi_transaction(uint8_t a, uint8_t b, uint8_t c, uint8_t d) {
	//uint8_t n;
	spi_send(a);
	//n = spi_send(b);
	spi_send(b);
	//if (n != a) error = -1;
	//n = spi_send(c);
	spi_send(c);
	return spi_send(d);
}

uint8_t spi_send(uint8_t b) {
	uint8_t reply;
#ifdef ISP_LOW_SPEED
	cli();
	CLKPR = B10000000;
	CLKPR = B00000011;
	sei();
#endif
	SPDR = b;
	spi_wait();
	reply = SPDR;
#ifdef ISP_LOW_SPEED
	cli();
	CLKPR = B10000000;
	CLKPR = B00000000;
	sei();
#endif
	return reply;
}

void spi_wait(void) {
	do {
	} while (!(SPSR & (1 << SPIF)));
}

void program_page(void) {
	char result = (char)STK_FAILED;
	int length = 256 * getch();
	length += getch();

	// added from ada <--
	if (length > 256) {
		sendCDCbyte(STK_FAILED);
		ram.isp.error++;
		return;
	}

	// todo compare with ada <--

	char memtype = getch();
	// flash memory @here, (length) bytes
	if (memtype == 'F') {
		write_flash(length);
		return;
	}
	if (memtype == 'E') {
		result = (char)write_eeprom(length);
		if (CRC_EOP == getch()) {
			sendCDCbyte(STK_INSYNC);
			sendCDCbyte(result);
		}
		else {
			ram.isp.error++;
			sendCDCbyte(STK_NOSYNC);
		}
		return;
	}
	sendCDCbyte(STK_FAILED);
	return;
}

uint8_t flash_read(uint8_t hilo, int addr) {
	return spi_transaction(0x20 + hilo * 8,
		(addr >> 8) & 0xFF,
		addr & 0xFF,
		0);
}

char flash_read_page(int length) {
	for (int x = 0; x < length; x += 2) {
		uint8_t low = flash_read(LOW, ram.isp._addr);
		sendCDCbyte(low);
		uint8_t high = flash_read(HIGH, ram.isp._addr);
		sendCDCbyte(high);
		ram.isp._addr++;
	}
	return STK_OK;
}

void universal(void) {
	uint8_t ch;

	fill(4);
	ch = spi_transaction(ram.isp.buff[0], ram.isp.buff[1], ram.isp.buff[2], ram.isp.buff[3]);
	breply(ch);
}

void read_signature(void) {
	if (CRC_EOP != getch()) {
		ram.isp.error++;
		sendCDCbyte(STK_NOSYNC);
		return;
	}
	sendCDCbyte(STK_INSYNC);
	uint8_t high = spi_transaction(0x30, 0x00, 0x00, 0x00);
	sendCDCbyte(high);
	uint8_t middle = spi_transaction(0x30, 0x00, 0x01, 0x00);
	sendCDCbyte(middle);
	uint8_t low = spi_transaction(0x30, 0x00, 0x02, 0x00);
	sendCDCbyte(low);
	sendCDCbyte(STK_OK);
}

void end_pmode(void) {
	AVR_SPI_DDR &= ~(1 << AVR_MISO); // INPUT
	AVR_SPI_DDR &= ~(1 << AVR_MOSI); // INPUT

	// Hardwaresetup to turn off the HID function with shorting the MOSI pin with GND next to it
	// do not short this pin in AVRISP mode!!!
	AVR_SPI_DDR |= (1 << AVR_MOSI); // PULLUP

	AVR_SPI_DDR &= ~(1 << AVR_SCK); // INPUT
	AVR_SPI_DDR &= ~(1 << AVR_SS); // INPUT
	ram.isp.pmode = 0;

	//LEDs_TurnOnLEDs(LEDS_ALL_LEDS);
	//_delay_ms(200);
	//LEDs_TurnOffLEDs(LEDS_ALL_LEDS);
}

void read_page(void) {
	char result = (char)STK_FAILED;
	int length = 256 * getch();
	length += getch();
	char memtype = getch();
	if (CRC_EOP != getch()) {
		ram.isp.error++;
		sendCDCbyte(STK_NOSYNC);
		return;
	}
	sendCDCbyte(STK_INSYNC);
	if (memtype == 'F') result = flash_read_page(length);
	if (memtype == 'E') result = eeprom_read_page(length);
	sendCDCbyte(result);
	return;
}

void write_flash(int length) {
	// TODO compare with ada <--
	fill(length);
	if (CRC_EOP == getch()) {
		sendCDCbyte(STK_INSYNC);
		sendCDCbyte(write_flash_pages(length));
	}
	else {
		ram.isp.error++;
		sendCDCbyte(STK_NOSYNC);
	}
}


#define EECHUNK (32)
uint8_t write_eeprom(int length) {
	// here is a word address, get the byte address
	int start = ram.isp._addr * 2;
	int remaining = length;
	if (length > ram.isp.param.eepromsize) {
		ram.isp.error++;
		return STK_FAILED;
	}
	while (remaining > EECHUNK) {
		write_eeprom_chunk(start, EECHUNK);
		start += EECHUNK;
		remaining -= EECHUNK;
	}
	write_eeprom_chunk(start, remaining);
	return STK_OK;
}
// write (length) bytes, (start) is a byte address
uint8_t write_eeprom_chunk(int start, int length) {
	// this writes byte-by-byte,
	// page writing may be faster (4 bytes at a time)
	fill(length);
	LEDs_TurnOffLEDs(LEDS_PMODE);
	for (int x = 0; x < length; x++) {
		int addr = start + x;
		spi_transaction(0xC0, (addr >> 8) & 0xFF, addr & 0xFF, ram.isp.buff[x]);
		_delay_ms(45);
	}
	LEDs_TurnOnLEDs(LEDS_PMODE);
	return STK_OK;
}

char eeprom_read_page(int length) {
	// TODO comapre with ada
	// here again we have a word address
	int start = ram.isp._addr * 2;
	for (int x = 0; x < length; x++) {
		int addr = start + x;
		uint8_t ee = spi_transaction(0xA0, (addr >> 8) & 0xFF, addr & 0xFF, 0xFF);
		sendCDCbyte(ee);
	}
	return STK_OK;
}

uint8_t write_flash_pages(int length) {
	int x = 0;
	int page = current_page();
	while (x < length) {
		if (page != current_page()) {
			commit(page);
			page = current_page();
		}
		flash(LOW, ram.isp._addr, ram.isp.buff[x++]);
		flash(HIGH, ram.isp._addr, ram.isp.buff[x++]);
		ram.isp._addr++;
	}
	commit(page);
	return STK_OK;
}

void commit(int addr) {
	LEDs_TurnOffLEDs(LEDS_PMODE);
	spi_transaction(0x4C, (addr >> 8) & 0xFF, addr & 0xFF, 0);
	_delay_ms(30);
	LEDs_TurnOnLEDs(LEDS_PMODE);
}

int current_page(void) {
	// TODO input useless??
	if (ram.isp.param.pagesize == 32)  return ram.isp._addr & 0xFFFFFFF0;
	if (ram.isp.param.pagesize == 64)  return ram.isp._addr & 0xFFFFFFE0;
	if (ram.isp.param.pagesize == 128) return ram.isp._addr & 0xFFFFFFC0;
	if (ram.isp.param.pagesize == 256) return ram.isp._addr & 0xFFFFFF80;
	return ram.isp._addr;
}

void flash(uint8_t hilo, int addr, uint8_t data) {
	spi_transaction(0x40 + 8 * hilo,
		addr >> 8 & 0xFF,
		addr & 0xFF,
		data);
}

void delay(unsigned long ms){
	// workaround to avoid micros() implemenation
	for (unsigned long i = 0; i < ms; i++)
		delayMicroseconds(1000);
}

// Delay for the given number of microseconds.  Assumes a 8 or 16 MHz clock.
void delayMicroseconds(unsigned int us){
	// calling avrlib's delay_us() function with low values (e.g. 1 or
	// 2 microseconds) gives delays longer than desired.
	//delay_us(us);
#if F_CPU >= 20000000L
	// for the 20 MHz clock on rare Arduino boards

	// for a one-microsecond delay, simply wait 2 cycle and return. The overhead
	// of the function call yields a delay of exactly a one microsecond.
	__asm__ __volatile__(
		"nop" "\n\t"
		"nop"); //just waiting 2 cycle
	if (--us == 0)
		return;

	// the following loop takes a 1/5 of a microsecond (4 cycles)
	// per iteration, so execute it five times for each microsecond of
	// delay requested.
	us = (us << 2) + us; // x5 us

	// account for the time taken in the preceeding commands.
	us -= 2;

#elif F_CPU >= 16000000L
	// for the 16 MHz clock on most Arduino boards

	// for a one-microsecond delay, simply return.  the overhead
	// of the function call yields a delay of approximately 1 1/8 us.
	if (--us == 0)
		return;

	// the following loop takes a quarter of a microsecond (4 cycles)
	// per iteration, so execute it four times for each microsecond of
	// delay requested.
	us <<= 2;

	// account for the time taken in the preceeding commands.
	us -= 2;
#else
	// for the 8 MHz internal clock on the ATmega168

	// for a one- or two-microsecond delay, simply return.  the overhead of
	// the function calls takes more than two microseconds.  can't just
	// subtract two, since us is unsigned; we'd overflow.
	if (--us == 0)
		return;
	if (--us == 0)
		return;

	// the following loop takes half of a microsecond (4 cycles)
	// per iteration, so execute it twice for each microsecond of
	// delay requested.
	us <<= 1;

	// partially compensate for the time taken by the preceeding commands.
	// we can't subtract any more than this or we'd overflow w/ small delays.
	us--;
#endif

	// busy wait
	__asm__ __volatile__(
		"1: sbiw %0,1" "\n\t" // 2 cycles
		"brne 1b" : "=w" (us) : "0" (us) // 2 cycles
		);
}

