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
	// Serial tx buffers Setup
	LRingBuffer_InitBuffer(&ram.RingBuffer, ram.RingBuffer_Data, sizeof(ram.RingBuffer_Data));

	//NHP setup
	ram.skipNHP = 0;
	NHPreset(&ram.NHP);

	// HID Setup
	ram.HID.writeHID = false;
	ram.HID.ID = 0;
	ram.HID.writtenReport = false;

	// AVR ISP Setup, includes SPI Hardware Setup Slave
	end_pmode();

	// Hardware Setup needs to be called at the very end!
	// includes WDT, SerialHID, SPI, LED, USB, Reset, HID deactivation setup
	SetupHardware();

	GlobalInterruptEnable();

	for (;;)
	{
		// try to clear HID reports if HID is disabled by hardware
		if (!(AVR_NO_HID_PIN &= AVR_NO_HID_MASK))
			clearHIDReports();

		//================================================================================
		// CDC: read in bytes from the CDC interface
		//================================================================================

		int16_t ReceivedByte = CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);
		if (!(ReceivedByte < 0)){
			if (VirtualSerial_CDC_Interface.State.LineEncoding.BaudRateBPS == AVRISP_BAUD){
#if (PRODUCTID != HOODLOADER_LITE_PID)
				// only run this on a 16u2 with enough flash
				// avrisp can disable the serial buffer to free programming byte information
				avrisp(ReceivedByte);
#endif
			}
			else{
				// Turn on RX LED
				LEDs_TurnOnLEDs(LEDMASK_RX);
				ram.PulseMSRemaining.RxLEDPulse = TX_RX_LED_PULSE_MS;

				// Send byte directly
				Serial_SendByte(ReceivedByte);
			}
		}

		//================================================================================
		// Serial: read in bytes from the Serial buffer
		//================================================================================

		// only access Buffer & Leds if pmode is deactivated
		if (!ram.isp.pmode){
			uint16_t BufferCount = LRingBuffer_GetCount(&ram.RingBuffer);
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

					// Read bytes from the USART receive buffer into the USB IN endpoint */
					while (BytesToSend--)
					{
						// ignoe HID check if: we need to write a pending NHP buff, its deactivated or not the right baud
						uint32_t baud = VirtualSerial_CDC_Interface.State.LineEncoding.BaudRateBPS;
						if (ram.skipNHP || (baud != AVRISP_BAUD && baud != 0 && baud != 115200) || (!(AVR_NO_HID_PIN &= AVR_NO_HID_MASK))){
							// Try to send the next bytes to the host, if DTR is set to not block serial reading in HID mode
							// outside HID mode always write the byte (!ram.skipNHP) is only null outside hid mode
							// discard the byte if host is not connected (needed to get new HID bytes and empty buffer)
							bool CurrentDTRState = (VirtualSerial_CDC_Interface.State.ControlLineStates.HostToDevice & CDC_CONTROL_LINE_OUT_DTR);
							if (CurrentDTRState || (!ram.skipNHP)){

								// Try to send the next byte of data to the host, abort if there is an error without dequeuing
								if (CDC_Device_SendByte(&VirtualSerial_CDC_Interface,
									LRingBuffer_Peek(&ram.RingBuffer)) != ENDPOINT_READYWAIT_NoError)
								{
									break;
								}
							}
							// Dequeue the already sent byte from the buffer now we have confirmed that no transmission error occurred
							LRingBuffer_Remove(&ram.RingBuffer);

							// Dequeue the NHP buffer byte
							if (ram.skipNHP){
								ram.skipNHP--;

								// set new timeout mark
								ram.PulseMSRemaining.NHPTimeout = NHP_TIMEOUT_MS;
							}
						}
						else{
							// set new timeout mark
							ram.PulseMSRemaining.NHPTimeout = NHP_TIMEOUT_MS;

							// main function to proceed HID input checks
							checkNHPProtocol(LRingBuffer_Remove(&ram.RingBuffer));
						}
					}
				}

			}

			//================================================================================
			// Timer: check if the led/timeout flush timer has expired
			//================================================================================

			if (TIFR0 & (1 << TOV0)){
				// reset the timer
				TIFR0 |= (1 << TOV0);

				// if reading has timed out write the cached NHP buffer down the serial
				if (ram.PulseMSRemaining.NHPTimeout && !(--ram.PulseMSRemaining.NHPTimeout)){
					// write started lead
					if (ram.NHP.leadError){
						LRingBuffer_Append(&ram.RingBuffer, ram.NHP.readbuffer[ram.NHP.readlength]);
						ram.skipNHP += ram.NHP.leadError;
					}

					// write buffer if it contains in progress reading data
					if (!ram.NHP.reset){
						LRingBuffer_Append_Buffer(&ram.RingBuffer, ram.NHP.readbuffer, ram.NHP.readlength);
						ram.skipNHP += ram.NHP.readlength;
					}

					// reset variables
					NHPreset(&ram.NHP);

					// check if previous reading was a valid Control Address and write it down
					// this needs to be appended after the normal protocol!
					checkNHPControlAddressError();

				}

				// Turn off TX LED(s) once the TX pulse period has elapsed
				if (ram.PulseMSRemaining.TxLEDPulse && !(--ram.PulseMSRemaining.TxLEDPulse))
					LEDs_TurnOffLEDs(LEDMASK_TX);

				// Turn off RX LED(s) once the RX pulse period has elapsed
				if (ram.PulseMSRemaining.RxLEDPulse && !(--ram.PulseMSRemaining.RxLEDPulse))
					LEDs_TurnOffLEDs(LEDMASK_RX);
			}
		}

		//================================================================================
		// CDC + USB Task
		//================================================================================

		HID_Device_USBTask(&Device_HID_Interface);
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

	// Setup the TX Pin to OUTPUT and RX with PULLUP
	DDRD |= (1 << 3);
	PORTD |= (1 << 2);

	// start Serial at HID baud to recognize new keypresses
	EVENT_CDC_Device_LineEncodingChanged(&VirtualSerial_CDC_Interface);

	// reset LEDs
	ram.PulseMSRemaining.whole = 0;
	LEDs_Init();

	// initialize USB
	USB_Init();

	/* Start the flush timer so that overflows occur rapidly to push received bytes to the USB interface */
	TCCR0B = (1 << CS02);

	/* Pull target /RESET line high */
	AVR_RESET_LINE_PORT |= AVR_RESET_LINE_MASK;
	AVR_RESET_LINE_DDR |= AVR_RESET_LINE_MASK;

	// Hardwaresetup to turn off the HID function with shorting the pin to GND
	AVR_NO_HID_DDR &= ~AVR_NO_HID_MASK; // INPUT
	AVR_NO_HID_PORT |= AVR_NO_HID_MASK; // PULLUP

	// Hardwaresetup to turn off the auto reset function with shorting the pin to GND
	AVR_NO_AUTORESET_LINE_DDR &= ~AVR_NO_AUTORESET_LINE_MASK; // INPUT
	AVR_NO_AUTORESET_LINE_PORT |= AVR_NO_AUTORESET_LINE_MASK; // PULLUP
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

	// save new byte to the buffer (automatically discards if its disabled or full)
	if ((!ram.isp.pmode) && (USB_DeviceState == DEVICE_STATE_Configured))
		LRingBuffer_Insert(&ram.RingBuffer, ReceivedByte);
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

