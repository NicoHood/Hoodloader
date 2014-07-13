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

	// Circular buffer to hold data from the host before it is sent to the device via the serial port.
	RingBuffer_t USBtoUSART_Buffer;

	// Underlying data buffer for \ref USBtoUSART_Buffer, where the stored bytes are located. 
	uint8_t      USBtoUSART_Buffer_Data[128];

	union{
		// normal mode if HID is on
		struct{
			// Circular buffer to hold data from the serial port before it is sent to the host.
			RingBuffer_t USARTtoUSB_Buffer;

			// Underlying data buffer for \ref USARTtoUSB_Buffer, where the stored bytes are located.
			uint8_t      USARTtoUSB_Buffer_Data[128];

			// Pulse generation counters to keep track of the number of milliseconds remaining for each pulse type
			volatile struct{
				uint8_t TxLEDPulse; // Milliseconds remaining for data Tx LED pulse
				uint8_t RxLEDPulse; // Milliseconds remaining for data Rx LED pulse
			}PulseMSRemaining;

			// variables to save hid states
			struct{
				// variable to perform a "HID flush" and to indicate what report should be written down
				uint8_t ID;
				// length of the report
				uint8_t Length;
				// number of bytes received
				uint8_t recvlength;
				// Buffer for the incoming HID report
				uint8_t reportBuffer[sizeof(HID_HIDReport_Data_t)];
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
			uint8_t error;
			uint8_t pmode;
			int here;
			int pagesize;
			int eepromsize;
			uint8_t buffer[128];
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
	SetupHardware();

	// Serial rx/tx buffers Setup
	RingBuffer_InitBuffer(&ram.USBtoUSART_Buffer, ram.USBtoUSART_Buffer_Data, sizeof(ram.USBtoUSART_Buffer_Data));

	// normal Setup
	RingBuffer_InitBuffer(&ram.USARTtoUSB_Buffer, ram.USARTtoUSB_Buffer_Data, sizeof(ram.USARTtoUSB_Buffer_Data));

	/*
	// HID Setup
	RingBuffer_InitBuffer(&ram.USARTtoUSB_Buffer, ram.USARTtoUSB_Buffer_Data, sizeof(ram.USARTtoUSB_Buffer_Data));
	NHPbuff.mErrorLevel = NHP_INPUT_RESET; // enough to initialize all variables
	HIDReportState.recvlength = 0;
	HIDReportState.ID = 0;

	// AVR ISP Setup
	isp.error = 0;
	isp.pmode = 0;
	*/

	ram.mode = MODE_DEFAULT;

	GlobalInterruptEnable();

	for (;;)
	{
		if (ram.mode == MODE_DEFAULT)
			mode_default();

		//new Send reports <--
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

	// set hardware SS to output so we can SPI use master mode
	AVR_SPI_DDR |= AVR_HARDWARE_SS;

	// Hardwaresetup to turn off the HID function with shorting the MOSI pin with GND next to it
	AVR_NO_HID_DDR &= ~AVR_NO_HID_MASK; // Input
	AVR_NO_HID_PORT |= AVR_NO_HID_MASK; // Pullup
}

//================================================================================
// Default Mode
//================================================================================

void mode_default(void){
	// Only try to read in bytes from the CDC interface if the transmit buffer is not full
	if (!(RingBuffer_IsFull(&ram.USBtoUSART_Buffer)))
	{
		int16_t ReceivedByte = CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);

		//( Store received byte into the USART transmit buffer */
		if (!(ReceivedByte < 0))
			RingBuffer_Insert(&ram.USBtoUSART_Buffer, ReceivedByte);
	}

	// Check if the UART receive buffer flush timer has expired or the buffer is nearly full
	uint16_t BufferCount = RingBuffer_GetCount(&ram.USARTtoUSB_Buffer);
	if ((TIFR0 & (1 << TOV0)) || (BufferCount > (sizeof(ram.USARTtoUSB_Buffer_Data) - 32)))
	{
		TIFR0 |= (1 << TOV0);

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
					// Try to send the next byte of data to the host, abort if there is an error without dequeuing
					if (CDC_Device_SendByte(&VirtualSerial_CDC_Interface,
						RingBuffer_Peek(&ram.USARTtoUSB_Buffer)) != ENDPOINT_READYWAIT_NoError)
					{
						break;
					}

					// Dequeue the already sent byte from the buffer now we have confirmed that no transmission error occurred
					RingBuffer_Remove(&ram.USARTtoUSB_Buffer);
				}
			}
		}

		// Turn off TX LED(s) once the TX pulse period has elapsed
		if (ram.PulseMSRemaining.TxLEDPulse && !(--ram.PulseMSRemaining.TxLEDPulse))
			LEDs_TurnOffLEDs(LEDMASK_TX);

		// Turn off RX LED(s) once the RX pulse period has elapsed
		if (ram.PulseMSRemaining.RxLEDPulse && !(--ram.PulseMSRemaining.RxLEDPulse))
			LEDs_TurnOffLEDs(LEDMASK_RX);
	}

	// Load the next byte from the USART transmit buffer into the USART
	if (!(RingBuffer_IsEmpty(&ram.USBtoUSART_Buffer))){
		Serial_SendByte(RingBuffer_Remove(&ram.USBtoUSART_Buffer));

		// Turn on RX LED
		LEDs_TurnOnLEDs(LEDMASK_RX);
		ram.PulseMSRemaining.RxLEDPulse = TX_RX_LED_PULSE_MS;
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

	if (USB_DeviceState == DEVICE_STATE_Configured)
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
	/*
	//write report and reset ID
	memcpy(ReportData, HIDReportBuffer, HIDReportState.length);
	*ReportID = HIDReportState.ID;
	*ReportSize = HIDReportState.length;
	HIDReportState.ID = 0;
	HIDReportState.recvlength = 0; //just to be sure if you call HID_Task by accident
	// always return true, because we cannot compare with >1 report due to ram limit
	// this will forcewrite the report everytime
	*/
	return true;
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
}
/*

//================================================================================
// HID
//================================================================================

void mode_hid(void){
// Only try to read in bytes from the CDC interface if the transmit buffer is not full
if (!(RingBuffer_IsFull(&ram.USBtoUSART_Buffer)))
{
int16_t ReceivedByte = CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);

// Read bytes from the USB OUT endpoint into the USART transmit buffer
if (!(ReceivedByte < 0))
RingBuffer_Insert(&ram.USBtoUSART_Buffer, ReceivedByte);
}

// discard Serial bytes if baud is ISP baud to not interrupt programming
if (VirtualSerial_CDC_Interface.State.LineEncoding.BaudRateBPS != 19200){
uint16_t BufferCount = RingBuffer_GetCount(&ram.USARTtoUSB_Buffer);
// Check if the UART receive buffer flush timer has expired or the buffer is nearly full
if ((TIFR0 & (1 << TOV0)) || BufferCount >(sizeof(ram.USARTtoUSB_Buffer_Data) - 32)){
TIFR0 |= (1 << TOV0);

if (ram.USARTtoUSB_Buffer.Count) {
LEDs_TurnOnLEDs(LEDMASK_TX);
PulseMSRemaining.TxLEDPulse = TX_RX_LED_PULSE_MS;
}

// Read bytes from the USART receive buffer
while (BufferCount--){

//read newest byte and check for Protocol if activated
uint8_t  b = RingBuffer_Remove(&ram.USARTtoUSB_Buffer);

// only process signal if HID is turned on
// make sure that no report is still active (like holding down a keyboard key)!
// Or dont recognize if baud is not 115200 to ensure that there is no conflict with other bauds
// Secound != 0 is for starting from powerup when no lineEncoding is set
if (!(AVR_NO_HID_PIN & AVR_NO_HID_MASK) ||
(VirtualSerial_CDC_Interface.State.LineEncoding.BaudRateBPS != 115200 && VirtualSerial_CDC_Interface.State.LineEncoding.BaudRateBPS != 0)){
// HID is off

// clear NHP buffer
NHPreset();
// reset any pending HID reports
HIDReportState.ID = 0;
//just to be sure if you call HID_Task by accident
HIDReportState.recvlength = 0;
// send byte directly with no DTR check!
CDC_Device_SendByte(&VirtualSerial_CDC_Interface, b);
}
else{
// HID is on
// new Protocol check<--
// if a reading finished succesfull without valid checksum or an error occured (ignore a reset)
if (NHPgetErrorLevel()&(~NHP_INPUT_RESET)){
// check if previous reading was a valid Control Address and write it down
if (HIDReportState.ID)
checkNHPControlAddressError();

// Write the last invalid signals. This will not write a possible new lead to keep
// it for the next reading. This is implemented in the Protocol itself.
writeNHPreadBuffer(NHPreadlength);
}
// main function to proceed HID input checks
checkNHPProtocol(b);
}
}


// if reading has timed out write the buffers down the serial and turn off the led
if (PulseMSRemaining.TxLEDPulse && !(--PulseMSRemaining.TxLEDPulse)){

// check if previous reading was a valid Control Address and write it down
if (HIDReportState.ID)
checkNHPControlAddressError();

// only write if there is input (ignore a reset)
if (!(NHPgetErrorLevel() & NHP_INPUT_RESET)){
// Lead errors are not in the buff length to keep them for next reading.
// But we want to write it down now after timeout.
uint8_t len = NHPreadlength;
if (NHPgetErrorLevel()& NHP_ERR_LEAD) len++;
writeNHPreadBuffer(len);
}
// do not write again in the while loop above anyways
NHPreset();

// Turn off TX LED(s) once the TX pulse period has elapsed
LEDs_TurnOffLEDs(LEDMASK_TX);
}

// Turn off RX LED(s) once the RX pulse period has elapsed
if (PulseMSRemaining.RxLEDPulse && !(--PulseMSRemaining.RxLEDPulse))
LEDs_TurnOffLEDs(LEDMASK_RX);
}
}
if (VirtualSerial_CDC_Interface.State.LineEncoding.BaudRateBPS == 19200){
// run ISP function for this baud
// is pmode active?
if (isp.pmode) LEDs_TurnOnLEDs(LEDMASK_TX);
else LEDs_TurnOffLEDs(LEDMASK_TX);
// is there an error?
if (isp.error) LEDs_TurnOnLEDs(LEDMASK_RX);
else LEDs_TurnOffLEDs(LEDMASK_RX);

if (!(RingBuffer_IsEmpty(&ram.USBtoUSART_Buffer)))
avrisp();
}
else{
// Load the next byte from the USART transmit buffer into the USART
if (!(RingBuffer_IsEmpty(&ram.USBtoUSART_Buffer))) {

Serial_SendByte(RingBuffer_Remove(&ram.USBtoUSART_Buffer)); //<--new syntax

LEDs_TurnOnLEDs(LEDMASK_RX);
PulseMSRemaining.RxLEDPulse = TX_RX_LED_PULSE_MS;
}
}
}


// Writes the NHP read buffer with the given length
void writeNHPreadBuffer(uint8_t length){
for (int i = 0; i < length; i++){
bool CurrentDTRState = (VirtualSerial_CDC_Interface.State.ControlLineStates.HostToDevice & CDC_CONTROL_LINE_OUT_DTR); //new <--
if (CurrentDTRState)
CDC_Device_SendByte(&VirtualSerial_CDC_Interface, NHPreadbuffer[i]);
}
}

// Checks for a valid protocol input and writes HID report
void checkNHPProtocol(uint8_t input){
uint8_t address = NHPreadChecksum(input);
if (address){
// nearly the same priciple like the Protocol itself: check for control address
if (address == NHP_ADDRESS_CONTROL && NHPgetChecksumData1() & NHP_USAGE_ARDUINOHID){

// check if previous reading was a valid Control Address and write it down
if (HIDReportState.ID)
checkNHPControlAddressError();

// get the new report ID and reset the buffer
HIDReportState.ID = NHPgetChecksumData0();
HIDReportState.recvlength = 0;
memset(HIDReportBuffer, 0, sizeof(HIDReportBuffer));

// Determine which interface must have its report generated
switch (HIDReportState.ID){
case HID_REPORTID_MouseReport:
HIDReportState.length = sizeof(HID_MouseReport_Data_t);
//HIDReportState.forcewrite = true;
break;

case HID_REPORTID_KeyboardReport:
HIDReportState.length = sizeof(HID_KeyboardReport_Data_t);
//HIDReportState.forcewrite = true; // set to true because of some bugs with joystick 1+2
break;

case HID_REPORTID_MediaReport:
HIDReportState.length = sizeof(HID_MediaReport_Data_t);
//HIDReportState.forcewrite = true;
break;

case HID_REPORTID_SystemReport:
HIDReportState.length = sizeof(HID_SystemReport_Data_t);
//HIDReportState.forcewrite = true;
break;

case HID_REPORTID_Gamepad1Report:
case HID_REPORTID_Gamepad2Report:
HIDReportState.length = sizeof(HID_GamepadReport_Data_t);
//HIDReportState.forcewrite = true;
break;

case HID_REPORTID_Joystick1Report:
case HID_REPORTID_Joystick2Report:
HIDReportState.length = sizeof(HID_JoystickReport_Data_t);
//HIDReportState.forcewrite = true;
break;

// not supported yet <--
//case HID_REPORTID_MidiReport:
//HID_Report_Size=sizeof(HID_MidiReport_Data_t);
//HID_Report_ForceWrite=false;
//break;

default:
// error
checkNHPControlAddressError();
break;
} //end switch

// The Protocol received a valid signal with inverse checksum
// Do not write the buff in the loop above or below, filter it out
NHPreset();
} // end if control address byte

else if (HIDReportState.ID){
// check if the new Address is in correct order of HID reports.
// the first 2 bytes are sent with Address 2 and so on.
if (address == (((HIDReportState.recvlength + 2) / 2) + 1)){
// save the first byte
HIDReportBuffer[HIDReportState.recvlength++] = NHPgetChecksumData0();

// if there is another byte we need (for odd max HID reports important
// to not write over the buff array)
if (HIDReportState.length != HIDReportState.recvlength)
HIDReportBuffer[HIDReportState.recvlength++] = NHPgetChecksumData1();

// we are ready to submit the new report to the usb host
if (HIDReportState.length == HIDReportState.recvlength){
// TODO timeout? <--
while (HIDReportState.ID)
HID_Device_USBTask(&Device_HID_Interface);
}
// The Protocol received a valid signal with inverse checksum
// Do not write the buff in the loop above or below, filter it out
NHPreset();
}

// we received a corrupt data packet
else
// check if previous reading was a valid Control Address and write it down
checkNHPControlAddressError();

} // end if HIDReportState.ID

else {
// just a normal Protocol outside our control address
}
} // end if readChecksum
}

void checkNHPControlAddressError(void){
// only write if it was just before, maybe it was a random valid address
// but if we already received some data we handle this as corrupted data and just
// discard all the bytes
if (HIDReportState.recvlength == 0){
// write the cached buffer (recreate protocol)
uint8_t buff[6];
uint8_t length = NHPwriteChecksum(NHP_ADDRESS_CONTROL, (NHP_USAGE_ARDUINOHID << 8) | HIDReportState.ID, buff);
for (int i = 0; i < length; i++){
bool CurrentDTRState = (VirtualSerial_CDC_Interface.State.ControlLineStates.HostToDevice & CDC_CONTROL_LINE_OUT_DTR); //new <--
if (CurrentDTRState)
CDC_Device_SendByte(&VirtualSerial_CDC_Interface, buff[i]);
}
}
// reset any pending HID reports
HIDReportState.ID = 0;
HIDReportState.recvlength = 0; //just to be sure if you call HID_Task by accident
}


//================================================================================
// Read NHP
//================================================================================

// reads two bytes and check its inverse
bool NHPreadChecksum(uint8_t input, NHPbuffer buff){
//reset if previous read was with an input/error
if (buff.mErrorLevel){
// cancel any pending data reads if a reset was triggered
if (buff.mErrorLevel & NHP_INPUT_RESET){
buff.mBlocks = 0;
buff.mWorkData = 0;
}
// if previous read was a lead error keep this byte
if (buff.mErrorLevel & NHP_ERR_LEAD){
buff.readbuffer[0] = buff.readbuffer[buff.readlength];
buff.readlength = 1;
}
else buff.readlength = 0;
}

// reset fully read data
buff.mErrorLevel = 0;

//write input to the buffer
buff.readbuffer[buff.readlength] = input;
buff.readlength++;

// check the lead/end/data indicator
switch (input & NHP_MASK_START){

case(NHP_MASK_LEAD) :
{
// we were still reading!  Log an error
if (buff.mBlocks){
buff.mErrorLevel |= NHP_ERR_LEAD | NHP_ERR_READ;
buff.readlength--;
}

// read command indicator or block length
buff.mBlocks = (input & NHP_MASK_LENGTH) >> 3;
switch (buff.mBlocks){
case 0:
case 1:
// ignore command, return false
buff.mBlocks = 0;
buff.mErrorLevel |= NHP_INPUT_COMMAND | NHP_INPUT_NEW;
return false;
break;
case 7:
// save block length + first 4 data bits (special case)
buff.mWorkData = input & NHP_MASK_DATA_4BIT;
buff.mBlocks -= 2;
break;
default:
// save block length + first 3 data bits
buff.mWorkData = input & NHP_MASK_DATA_3BIT;
buff.mBlocks--;
break;
}
}
break;

case(NHP_MASK_END) :
{
if (buff.mBlocks-- == 1){
// save data + address
buff.mErrorLevel |= NHP_INPUT_ADDRESS | NHP_INPUT_NEW;

// we know its a valid input, left some things out here
if (((buff.mWorkData & 0xFFFF) ^ (buff.mWorkData >> 16)) == 0xFFFF){
uint8_t address = (input & 0x3F) + 1;
return address;
}
else
return false;
}
else{
// log an error, not ready for an address byte, and reset data counters
buff.mErrorLevel |= NHP_ERR_DATA | NHP_ERR_READ;
buff.mBlocks = 0;
}
}
break;

default:
{
if (buff.mBlocks-- < 2){
// log an error, expecting an address or header byte
buff.mErrorLevel |= NHP_ERR_END | NHP_ERR_READ;
buff.mBlocks = 0;
}
else{
// get next 7 bits of data
buff.mWorkData <<= 7;
// dont need &NHP_MASK_DATA_7BIT because first bit is zero!
buff.mWorkData |= input;
}
}
break;
} // end switch

// no new input
return false;
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
uint8_t ch = getch();
switch (ch) {
case '0': // signon
isp.error = 0;
empty_reply();
break;
case '1':
if (getch() == CRC_EOP) {
CDC_Device_SendByte(&VirtualSerial_CDC_Interface, (char)STK_INSYNC);
CDC_Device_SendByte(&VirtualSerial_CDC_Interface, 'A');
CDC_Device_SendByte(&VirtualSerial_CDC_Interface, 'V');
CDC_Device_SendByte(&VirtualSerial_CDC_Interface, 'R');
CDC_Device_SendByte(&VirtualSerial_CDC_Interface, ' ');
CDC_Device_SendByte(&VirtualSerial_CDC_Interface, 'I');
CDC_Device_SendByte(&VirtualSerial_CDC_Interface, 'S');
CDC_Device_SendByte(&VirtualSerial_CDC_Interface, 'P');
CDC_Device_SendByte(&VirtualSerial_CDC_Interface, (char)STK_OK);
}
break;
case 'A':
//get_version
switch (getch()) {
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
break;
}
break;
case 'B':{
//set_parameters
// discard this information to save ram. its not needed
for (int i = 0; i < 12; i++)
getch();

uint8_t temp[2];
temp[0] = getch();
temp[1] = getch();
isp.pagesize = beget16(&temp[0]);
temp[0] = getch();
temp[1] = getch();
isp.eepromsize = beget16(&temp[0]);

// discard this information to save ram. its not needed
for (int i = 0; i < 4; i++)
getch();
empty_reply();
}
break;
case 'E': // extended parameters - ignore for now
//fill(5);
for (int i = 0; i < 5; i++)
getch();
empty_reply();
break;

case 'P':
{
// spi_init
SPCR = 0x53;
SPSR;
SPDR;
// following delays may not work on all targets...
DDRB |= (1 << AVR_SS); // OUTPUT
PORTB |= (1 << AVR_SS); // HIGH
DDRB |= (1 << AVR_SCK); // OUTPUT
PORTB &= ~(1 << AVR_SCK); // LOW
delayMicroseconds(50000);
PORTB &= ~(1 << AVR_SS); // LOW
delayMicroseconds(50000);
DDRB &= ~(1 << AVR_MISO); // INPUT
DDRB |= (1 << AVR_MOSI); // OUTPUT
spi_transaction(0xAC, 0x53, 0x00, 0x00);
isp.pmode = 1;
empty_reply();
}
break;
case 'U': // set address (word)
isp.here = getch();
isp.here += 256 * getch();
empty_reply();
break;

case 0x60: //STK_PROG_FLASH
//low = getch();
//high = getch();
getch();
getch();
empty_reply();
break;
case 0x61: //STK_PROG_DATA
//data = getch();
getch();
empty_reply();
break;

case 0x64: //STK_PROG_PAGE
{
// program_page
char result = (char)STK_FAILED;
int length = 256 * getch();
//Serial1.println(length);
length += getch();
//Serial1.println(length);
char memtype = getch();
// flash memory @here, (length) bytes
if (memtype == 'F') {
write_flash(length);
return;
}
if (memtype == 'E') {
result = (char)write_eeprom(length);
if (CRC_EOP == getch()) {
CDC_Device_SendByte(&VirtualSerial_CDC_Interface, (char)STK_INSYNC);
CDC_Device_SendByte(&VirtualSerial_CDC_Interface, result);
}
else {
isp.error++;
CDC_Device_SendByte(&VirtualSerial_CDC_Interface, (char)STK_NOSYNC);
}
break;
}
CDC_Device_SendByte(&VirtualSerial_CDC_Interface, (char)STK_FAILED);
}
break;

case 0x74: //STK_READ_PAGE 't'
{
// read_page
char result = (char)STK_FAILED;
int length = 256 * getch();
length += getch();
char memtype = getch();
if (CRC_EOP != getch()) {
isp.error++;
CDC_Device_SendByte(&VirtualSerial_CDC_Interface, (char)STK_NOSYNC);
return;
}
CDC_Device_SendByte(&VirtualSerial_CDC_Interface, (char)STK_INSYNC);
if (memtype == 'F') {
// flash_read_page
for (int x = 0; x < length; x += 2) {
uint8_t low = flash_read(0, isp.here);
CDC_Device_SendByte(&VirtualSerial_CDC_Interface, (char)low);
uint8_t high = flash_read(1, isp.here);
CDC_Device_SendByte(&VirtualSerial_CDC_Interface, (char)high);
isp.here++;
}
result = STK_OK;
}
if (memtype == 'E') {
// eeprom_read_page
// here again we have a word address
int start = isp.here * 2;
for (int x = 0; x < length; x++) {
int addr = start + x;
uint8_t ee = spi_transaction(0xA0, (addr >> 8) & 0xFF, addr & 0xFF, 0xFF);
CDC_Device_SendByte(&VirtualSerial_CDC_Interface, (char)ee);
}
result = STK_OK;
}
CDC_Device_SendByte(&VirtualSerial_CDC_Interface, result);
}
break;

case 'V': //0x56
{
// universal
uint8_t ch;
uint8_t bytebuff[4];
fillbuffer(bytebuff, sizeof(bytebuff));
ch = spi_transaction(bytebuff[0], bytebuff[1], bytebuff[2], bytebuff[3]);
breply(ch);
}
break;
case 'Q': //0x51
isp.error = 0;
// end_pmode
DDRB &= ~(1 << AVR_MISO); // INPUT
DDRB &= ~(1 << AVR_MOSI); // INPUT
DDRB &= ~(1 << AVR_SCK); // INPUT
DDRB &= ~(1 << AVR_SS); // INPUT
isp.pmode = 0;
empty_reply();
break;

case 0x75: //STK_READ_SIGN 'u'
{
// read_signature
if (CRC_EOP != getch()) {
isp.error++;
CDC_Device_SendByte(&VirtualSerial_CDC_Interface, (char)STK_NOSYNC);
break;
}
CDC_Device_SendByte(&VirtualSerial_CDC_Interface, (char)STK_INSYNC);
uint8_t high = spi_transaction(0x30, 0x00, 0x00, 0x00);
CDC_Device_SendByte(&VirtualSerial_CDC_Interface, (char)high);
uint8_t middle = spi_transaction(0x30, 0x00, 0x01, 0x00);
CDC_Device_SendByte(&VirtualSerial_CDC_Interface, (char)middle);
uint8_t low = spi_transaction(0x30, 0x00, 0x02, 0x00);
CDC_Device_SendByte(&VirtualSerial_CDC_Interface, (char)low);
CDC_Device_SendByte(&VirtualSerial_CDC_Interface, (char)STK_OK);
}
break;

// expecting a command, not CRC_EOP
// this is how we can get back in sync
case CRC_EOP:
isp.error++;
CDC_Device_SendByte(&VirtualSerial_CDC_Interface, (char)STK_NOSYNC);
break;

// anything else we will return STK_UNKNOWN
default:
isp.error++;
if (CRC_EOP == getch())
CDC_Device_SendByte(&VirtualSerial_CDC_Interface, (char)STK_UNKNOWN);
else
CDC_Device_SendByte(&VirtualSerial_CDC_Interface, (char)STK_NOSYNC);
}
}

uint8_t getch(void) {
// wait until there is an usb input
while ((RingBuffer_IsEmpty(&ram.USBtoUSART_Buffer)));
return RingBuffer_Remove(&ram.USBtoUSART_Buffer);
}

void empty_reply(void) {
if (CRC_EOP == getch()) {
CDC_Device_SendByte(&VirtualSerial_CDC_Interface, (char)STK_INSYNC);
CDC_Device_SendByte(&VirtualSerial_CDC_Interface, (char)STK_OK);
}
else {
isp.error++;
CDC_Device_SendByte(&VirtualSerial_CDC_Interface, (char)STK_NOSYNC);
}
}

void breply(uint8_t b) {
if (CRC_EOP == getch()) {
CDC_Device_SendByte(&VirtualSerial_CDC_Interface, (char)STK_INSYNC);
CDC_Device_SendByte(&VirtualSerial_CDC_Interface, (char)b);
CDC_Device_SendByte(&VirtualSerial_CDC_Interface, (char)STK_OK);
}
else {
isp.error++;
CDC_Device_SendByte(&VirtualSerial_CDC_Interface, (char)STK_NOSYNC);
}
}

void spi_init(void) {
SPCR = 0x53;
SPSR;
SPDR;
}

uint8_t spi_transaction(uint8_t a, uint8_t b, uint8_t c, uint8_t d) {
//uint8_t n;
spi_send(a);
spi_send(b);
//if (n != a) error = -1;
spi_send(c);
return spi_send(d);
}

uint8_t spi_send(uint8_t b) {
uint8_t reply;
SPDR = b;
do {
} while (!(SPSR & (1 << SPIF)));
reply = SPDR;
return reply;
}

void program_page(void) {
char result = (char)STK_FAILED;
int length = 256 * getch();
length += getch();
char memtype = getch();
// flash memory @here, (length) bytes
if (memtype == 'F') {
write_flash(length);
return;
}
if (memtype == 'E') {
result = (char)write_eeprom(length);
if (CRC_EOP == getch()) {
CDC_Device_SendByte(&VirtualSerial_CDC_Interface, (char)STK_INSYNC);
CDC_Device_SendByte(&VirtualSerial_CDC_Interface, result);
}
else {
isp.error++;
CDC_Device_SendByte(&VirtualSerial_CDC_Interface, (char)STK_NOSYNC);
}
return;
}
CDC_Device_SendByte(&VirtualSerial_CDC_Interface, (char)STK_FAILED);
return;
}

uint8_t write_flash_pages(uint8_t buffer[], int length) {
int x = 0;
int page = current_page();
while (x < length) {
if (page != current_page()) {
commit(page);
page = current_page();
}
flash(0, isp.here, buffer[x++]);
flash(1, isp.here, buffer[x++]);
isp.here++;
}

commit(page);

return STK_OK;
}

void commit(int addr) {
LEDs_TurnOffLEDs(LEDMASK_TX);
spi_transaction(0x4C, (addr >> 8) & 0xFF, addr & 0xFF, 0);
delayMicroseconds(PTIME * 1000);
LEDs_TurnOnLEDs(LEDMASK_TX);
}

int current_page(void) {
if (isp.pagesize == 32)  return isp.here & 0xFFFFFFF0;
if (isp.pagesize == 64)  return isp.here & 0xFFFFFFE0;
if (isp.pagesize == 128) return isp.here & 0xFFFFFFC0;
if (isp.pagesize == 256) return isp.here & 0xFFFFFF80;
return isp.here;
}

void flash(uint8_t hilo, int addr, uint8_t data) {
spi_transaction(0x40 + 8 * hilo,
addr >> 8 & 0xFF,
addr & 0xFF,
data);
}

uint8_t flash_read(uint8_t hilo, int addr) {
return spi_transaction(0x20 + hilo * 8,
(addr >> 8) & 0xFF,
addr & 0xFF,
0);
}

#define EECHUNK (32)
uint8_t write_eeprom(int length) {
// isp.here is a word address, get the byte address
int start = isp.here * 2;
int remaining = length;
if (length > isp.eepromsize) {
isp.error++;
return STK_FAILED;
}
// create new buffer for eeprom
uint8_t eechunkbuffer[EECHUNK];
while (remaining > EECHUNK) {
// fill the buffer and pass it to the write function
fillbuffer(eechunkbuffer, EECHUNK);
write_eeprom_chunkbuffer(start, eechunkbuffer, EECHUNK);
start += EECHUNK;
remaining -= EECHUNK;
}
fillbuffer(eechunkbuffer, remaining);
write_eeprom_chunkbuffer(start, eechunkbuffer, remaining);
return STK_OK;
}

// write (length) bytes, (start) is a byte address
uint8_t write_eeprom_chunkbuffer(int start, uint8_t buffer[], int length) {
// this writes byte-by-byte,
// page writing may be faster (4 bytes at a time)
//fill(length);
LEDs_TurnOffLEDs(LEDMASK_TX);
for (int x = 0; x < length; x++) {
int addr = start + x;
spi_transaction(0xC0, (addr >> 8) & 0xFF, addr & 0xFF, buffer[x]);
delayMicroseconds(45000);
}
LEDs_TurnOnLEDs(LEDMASK_TX);
return STK_OK;
}

void read_signature(void) {
if (CRC_EOP != getch()) {
isp.error++;
CDC_Device_SendByte(&VirtualSerial_CDC_Interface, (char)STK_NOSYNC);
return;
}
CDC_Device_SendByte(&VirtualSerial_CDC_Interface, (char)STK_INSYNC);
uint8_t high = spi_transaction(0x30, 0x00, 0x00, 0x00);
CDC_Device_SendByte(&VirtualSerial_CDC_Interface, (char)high);
uint8_t middle = spi_transaction(0x30, 0x00, 0x01, 0x00);
CDC_Device_SendByte(&VirtualSerial_CDC_Interface, (char)middle);
uint8_t low = spi_transaction(0x30, 0x00, 0x02, 0x00);
CDC_Device_SendByte(&VirtualSerial_CDC_Interface, (char)low);
CDC_Device_SendByte(&VirtualSerial_CDC_Interface, (char)STK_OK);
}

void read_page(void) {
char result = (char)STK_FAILED;
int length = 256 * getch();
length += getch();
char memtype = getch();
if (CRC_EOP != getch()) {
isp.error++;
CDC_Device_SendByte(&VirtualSerial_CDC_Interface, (char)STK_NOSYNC);
return;
}
CDC_Device_SendByte(&VirtualSerial_CDC_Interface, (char)STK_INSYNC);
if (memtype == 'F') result = flash_read_page(length);
if (memtype == 'E') result = eeprom_read_page(length);
CDC_Device_SendByte(&VirtualSerial_CDC_Interface, result);
return;
}

char flash_read_page(int length) {
for (int x = 0; x < length; x += 2) {
uint8_t low = flash_read(0, isp.here);
CDC_Device_SendByte(&VirtualSerial_CDC_Interface, (char)low);
uint8_t high = flash_read(1, isp.here);
CDC_Device_SendByte(&VirtualSerial_CDC_Interface, (char)high);
isp.here++;
}
return STK_OK;
}

char eeprom_read_page(int length) {
// isp.here again we have a word address
int start = isp.here * 2;
for (int x = 0; x < length; x++) {
int addr = start + x;
uint8_t ee = spi_transaction(0xA0, (addr >> 8) & 0xFF, addr & 0xFF, 0xFF);
CDC_Device_SendByte(&VirtualSerial_CDC_Interface, (char)ee);
}
return STK_OK;
}

void universal(void) {
uint8_t ch;

uint8_t bytebuff[4];
fillbuffer(bytebuff, sizeof(bytebuff));
ch = spi_transaction(bytebuff[0], bytebuff[1], bytebuff[2], bytebuff[3]);
breply(ch);
}

void write_flash(int length) {
// create an array of size length
uint8_t* tempbuff;
tempbuff = (uint8_t*)malloc(length * sizeof(*tempbuff));
if (tempbuff) {
// allocation succeeded
fillbuffer(tempbuff, length);
}
else {
// ram is full!
isp.error++;
CDC_Device_SendByte(&VirtualSerial_CDC_Interface, (char)STK_NOSYNC);
return;
}

if (CRC_EOP == getch()) {
CDC_Device_SendByte(&VirtualSerial_CDC_Interface, (char)STK_INSYNC);
CDC_Device_SendByte(&VirtualSerial_CDC_Interface, (char)write_flash_pages(tempbuff, length));
}
else {
isp.error++;
CDC_Device_SendByte(&VirtualSerial_CDC_Interface, (char)STK_NOSYNC);
}
free(tempbuff); // release the memory
}

void fillbuffer(uint8_t buffer[], int n) {
for (int x = 0; x < n; x++) {
buffer[x] = getch();
}
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

*/