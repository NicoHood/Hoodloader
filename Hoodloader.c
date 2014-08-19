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

#include "Hoodloader.h"

//================================================================================
// Main
//================================================================================

/** Main program entry point. This routine contains the overall program flow, including initial
*  setup of all components and the main program loop.
*/
int main(void)
{
	ram.mode = MODE_NONE;

	// all reports are empty by default
	memset(&ram.HID.isEmpty, true, sizeof(ram.HID.isEmpty));

	SetupHardware();

	GlobalInterruptEnable();

	for (;;)
	{
		// change ram.mode depending on the selected CDC baud rate
		selectMode();

		// clear HID reports if chip gets restarted
		//if (!(AVR_RESET_LINE_PIN & AVR_RESET_LINE_MASK))
		//	clearHIDReports();

		// run main code, depending on the selected mode
		if (ram.mode == MODE_AVRISP){
#if (PRODUCTID != HOODLOADER_LITE_PID)
			// only run this on a 16u2 with enough flash
			avrisp();
#endif
		}
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
			uint16_t BufferCount = LRingBuffer_GetCount(&ram.USARTtoUSB_Buffer);
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
								LRingBuffer_Peek(&ram.USARTtoUSB_Buffer, ram.USARTtoUSB_Buffer_Data)) != ENDPOINT_READYWAIT_NoError)
							{
								break;
							}
						}

						// Dequeue the already sent byte from the buffer now we have confirmed that no transmission error occurred
						uint8_t b = LRingBuffer_Remove(&ram.USARTtoUSB_Buffer, ram.USARTtoUSB_Buffer_Data);

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
						uint8_t start = 0;
						uint8_t length = 0;
						// only write buffer if its new
						if (ram.NHP.reset) start += ram.NHP.readlength;
						else length += ram.NHP.readlength;
						// also write started leads
						if (ram.NHP.leadError) length++;
						if (length) // just to be sure <--
							writeToCDC(&ram.NHP.readbuffer[start], length);
						// reset variables
						ram.NHP.reset = true;
						ram.NHP.leadError = false;
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
	AVR_NO_HID_DDR &= ~(1 << AVR_NO_HID); // INPUT
	AVR_NO_HID_PORT |= (1 << AVR_NO_HID); // PULLUP
}


//================================================================================
// Mode selection
//================================================================================

void selectMode(void){
	// check what mode should run
	uint8_t oldMode = ram.mode;
	uint32_t currentBaud = VirtualSerial_CDC_Interface.State.LineEncoding.BaudRateBPS;

	// HID only works for baud 115200 or not configured (startup, no host connection) to work at maximum speed and after reprogramming
	if ((currentBaud == 0 || currentBaud == 115200) && (AVR_NO_HID_PIN & (1 << AVR_NO_HID)))
		ram.mode = MODE_HID;
	else if (currentBaud == AVRISP_BAUD)
		ram.mode = MODE_AVRISP;
	else ram.mode = MODE_DEFAULT;

	// check if mode has changed
	if (oldMode != ram.mode){
		// coming from AVR ISP, setup ram properly
		if (oldMode == MODE_AVRISP || oldMode == MODE_NONE){
			// Serial tx buffers Setup
			LRingBuffer_InitBuffer(&ram.USARTtoUSB_Buffer);
			// reset LEDs
			ram.PulseMSRemaining.TxLEDPulse = 0;
			ram.PulseMSRemaining.RxLEDPulse = 0;
			LEDs_TurnOffLEDs(LEDS_ALL_LEDS);
		}
		// setup for HID and AVRISP mode
		if (ram.mode == MODE_HID){
			// HID Setup
			ram.NHP.reset = true;
			ram.NHP.leadError = false;
			ram.HID.ID = 0;
		}
		else{
			// we have a pending HID report, flush it first
			flushHID();
			// clear all reports
			//clearHIDReports();

			if (ram.mode == MODE_AVRISP){
				// AVR ISP Setup
				ram.isp.error = 0;
				ram.isp.pmode = 0;
				ram.isp._addr = 0; // just to be sure
				LEDs_TurnOffLEDs(LEDS_ALL_LEDS);

				// Hardwaresetup to turn off the HID function with shorting the MOSI pin with GND next to it
				// do not short this pin in AVRISP mode!!!
				// moved here so the pin is INPUT to not damage anything
				AVR_NO_HID_DDR &= ~(1 << AVR_NO_HID); // INPUT
				AVR_NO_HID_PORT |= (1 << AVR_NO_HID); // PULLUP
			}
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

/** ISR to manage the reception of data from the serial port, placing received bytes into a circular buffer
*  for later transmission to the host.
*/
ISR(USART1_RX_vect, ISR_BLOCK)
{
	uint8_t ReceivedByte = UDR1;

	// TODO move to NHP buffer direct

	// only save the byte if AVRISP is off and if usb device is ready
	if (ram.mode != MODE_AVRISP && USB_DeviceState == DEVICE_STATE_Configured)
		//checkNHPProtocol(ReceivedByte);
		LRingBuffer_Insert(&ram.USARTtoUSB_Buffer, ram.USARTtoUSB_Buffer_Data, ReceivedByte);
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

