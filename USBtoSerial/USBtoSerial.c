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
 *  Main source file for the USBtoSerial project. This file contains the main tasks of
 *  the project and is responsible for the initial application hardware configuration.
 */

#include "USBtoSerial.h"

/** LUFA CDC Class driver interface configuration and state information. This structure is
 *  passed to all CDC Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
//USB_ClassInfo_CDC_Device_t VirtualSerial_CDC_Interface =
//	{
//		.Config =
//			{
//				.ControlInterfaceNumber         = INTERFACE_ID_CDC_CCI,
//				.DataINEndpoint                 =
//					{
//						.Address                = CDC_TX_EPADDR,
//						.Size                   = CDC_TXRX_EPSIZE,
//						.Banks                  = 1,
//					},
//				.DataOUTEndpoint                =
//					{
//						.Address                = CDC_RX_EPADDR,
//						.Size                   = CDC_TXRX_EPSIZE,
//						.Banks                  = 1,
//					},
//				.NotificationEndpoint           =
//					{
//						.Address                = CDC_NOTIFICATION_EPADDR,
//						.Size                   = CDC_NOTIFICATION_EPSIZE,
//						.Banks                  = 1,
//					},
//			},
//	};


/** Main program entry point. This routine contains the overall program flow, including initial
 *  setup of all components and the main program loop.
 */
int main(void)
{
	// Serial tx buffers Setup
	LRingBuffer_InitBuffer(&ram.USARTtoUSB_Buffer, ram.USARTtoUSB_Buffer_Data, sizeof(ram.USARTtoUSB_Buffer_Data));

	//NHP setup
	ram.skipNHP = 0;
	NHPreset(&ram.NHP);

	// reset LEDs
	ram.PulseMSRemaining.whole = 0;

	// HID Setup
	HIDreset();

	// AVR ISP Setup
	avrispReset();

	SetupHardware();

	GlobalInterruptEnable();

	for (;;)
	{
		//================================================================================
		// CDC: read in bytes from the CDC interface
		//================================================================================

		int16_t ReceivedByte = CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);
		if (!(ReceivedByte < 0)){
			// Turn on RX LED
			LEDs_TurnOnLEDs(LEDMASK_RX);
			ram.PulseMSRemaining.RxLEDPulse = TX_RX_LED_PULSE_MS;

			// Send byte directly
			Serial_SendByte(ReceivedByte);
		}



		uint16_t BufferCount = LRingBuffer_GetCount(&ram.USARTtoUSB_Buffer);
		if (BufferCount)
		{
			// Turn on TX LED
			LEDs_TurnOnLEDs(LEDMASK_TX);
			ram.PulseMSRemaining.TxLEDPulse = TX_RX_LED_PULSE_MS;

			Endpoint_SelectEndpoint(VirtualSerial_CDC_Interface.Config.DataINEndpoint.Address);

			/* Check if a packet is already enqueued to the host - if so, we shouldn't try to send more data
			* until it completes as there is a chance nothing is listening and a lengthy timeout could occur */
			if (Endpoint_IsINReady())
			{
				/* Never send more than one bank size less one byte to the host at a time, so that we don't block
				* while a Zero Length Packet (ZLP) to terminate the transfer is sent if the host isn't listening */
				uint8_t BytesToSend = MIN(BufferCount, (CDC_TXRX_EPSIZE - 1));

				/* Read bytes from the USART receive buffer into the USB IN endpoint */
				while (BytesToSend--)
				{


					// Try to send the next byte of data to the host, abort if there is an error without dequeuing
					if (CDC_Device_SendByte(&VirtualSerial_CDC_Interface,
						LRingBuffer_Peek(&ram.USARTtoUSB_Buffer)) != ENDPOINT_READYWAIT_NoError)
					{
						break;
					}
			
				// Dequeue the already sent byte from the buffer now we have confirmed that no transmission error occurred
				LRingBuffer_Remove(&ram.USARTtoUSB_Buffer);
				}
			}
		}


		if (TIFR0 & (1 << TOV0)){
			// reset the timer
			TIFR0 |= (1 << TOV0);

			// Turn off TX LED(s) once the TX pulse period has elapsed
			if (ram.PulseMSRemaining.TxLEDPulse && !(--ram.PulseMSRemaining.TxLEDPulse))
				LEDs_TurnOffLEDs(LEDMASK_TX);

			// Turn off RX LED(s) once the RX pulse period has elapsed
			if (ram.PulseMSRemaining.RxLEDPulse && !(--ram.PulseMSRemaining.RxLEDPulse))
				LEDs_TurnOffLEDs(LEDMASK_RX);
		}

		CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
		USB_USBTask();
	}
	for (;;){
		// TODO
		// try to clear HID reports if HID is disabled by hardware
		if (!ram.isp.pmode && (!(AVR_NO_HID_PIN &= AVR_NO_HID_MASK)))
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
					// ignoe HID check if: we need to write a pending NHP buff, its deactivated or not the right baud
					uint32_t baud = VirtualSerial_CDC_Interface.State.LineEncoding.BaudRateBPS;
					if (ram.skipNHP || (baud != AVRISP_BAUD && baud != 0 && baud != 115200) || (!(AVR_NO_HID_PIN &= AVR_NO_HID_MASK))){
						// set new timeout mark <-- needed? TODO
						if (ram.skipNHP)
							ram.PulseMSRemaining.NHPTimeout = NHP_TIMEOUT_MS;
						// if HID disabled try to clean reports if there are any
						else
							clearHIDReports();

						// Try to send the next bytes to the host, if DTR is set to not block serial reading in HID mode
						// outside HID mode always write the byte (!ram.skipNHP) is only null outside hid mode
						// discard the byte if host is not connected (needed to get new HID bytes and empty buffer)
						bool CurrentDTRState = (VirtualSerial_CDC_Interface.State.ControlLineStates.HostToDevice & CDC_CONTROL_LINE_OUT_DTR);
						if (CurrentDTRState || (!ram.skipNHP)){

							// Try to send the next byte of data to the host, abort if there is an error without dequeuing
							if (CDC_Device_SendByte(&VirtualSerial_CDC_Interface,
								LRingBuffer_Peek(&ram.USARTtoUSB_Buffer)) != ENDPOINT_READYWAIT_NoError)
							{
								break;
							}
						}
						// Dequeue the already sent byte from the buffer now we have confirmed that no transmission error occurred
						LRingBuffer_Remove(&ram.USARTtoUSB_Buffer);

						// Dequeue the NHP buffer byte
						if (ram.skipNHP)
							ram.skipNHP--;
					}
					else
						// main function to proceed HID input checks
						checkNHPProtocol(LRingBuffer_Remove(&ram.USARTtoUSB_Buffer));

				}
			}
		}

		//================================================================================
		// Timer: check if the led/timeout flush timer has expired
		//================================================================================

		if (TIFR0 & (1 << TOV0)){
			// reset the timer
			TIFR0 |= (1 << TOV0);

			// if reading has timed out write the buffers down the serial
			if (ram.PulseMSRemaining.NHPTimeout && !(--ram.PulseMSRemaining.NHPTimeout)){
				// write the rest of the cached NHP buffer down
				if (!ram.isp.pmode){

					// write started lead
					if (ram.NHP.leadError){
						LRingBuffer_Append(&ram.USARTtoUSB_Buffer, ram.NHP.readbuffer[ram.NHP.readlength]);
						ram.skipNHP += ram.NHP.leadError;
					}

					// write buffer if it contains in progress reading data
					if (!ram.NHP.reset){
						LRingBuffer_Append_Buffer(&ram.USARTtoUSB_Buffer, ram.NHP.readbuffer, ram.NHP.readlength);
						ram.skipNHP += ram.NHP.readlength;
					}

					// reset variables
					NHPreset(&ram.NHP);

					// check if previous reading was a valid Control Address and write it down
					// this needs to be appended after the normal protocol!
					checkNHPControlAddressError();
				}
			}

			// Turn off TX LED(s) once the TX pulse period has elapsed
			if (ram.PulseMSRemaining.TxLEDPulse && !(--ram.PulseMSRemaining.TxLEDPulse))
				LEDs_TurnOffLEDs(LEDMASK_TX);

			// Turn off RX LED(s) once the RX pulse period has elapsed
			if (ram.PulseMSRemaining.RxLEDPulse && !(--ram.PulseMSRemaining.RxLEDPulse))
				LEDs_TurnOffLEDs(LEDMASK_RX);
		}

		//================================================================================
		// CDC + USB Task
		//================================================================================

		CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
		USB_USBTask();
	}
}

/** Configures the board hardware and chip peripherals for the demo's functionality. */
void SetupHardware(void)
{
#if (ARCH == ARCH_AVR8)
	/* Disable watchdog if enabled by bootloader/fuses */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();

	/* Disable clock division */
	//clock_prescale_set(clock_div_1);
#endif
	/* Start the flush timer so that overflows occur rapidly to push received bytes to the USB interface */
	TCCR0B = (1 << CS02);

	Serial_Init(115200, true);

	/* Must turn off USART before reconfiguring it, otherwise incorrect operation may occur */
	// Added for correct Serial connection at baud 115200 <--
	// TODO PD3 ??
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

	/* Hardware Initialization */
	LEDs_Init();
	USB_Init();

	/* Pull target /RESET line high */
	AVR_RESET_LINE_PORT |= AVR_RESET_LINE_MASK;
	AVR_RESET_LINE_DDR |= AVR_RESET_LINE_MASK;
}

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
void EVENT_USB_Device_ConfigurationChanged(void)
{
	bool ConfigSuccess = true;

	ConfigSuccess &= CDC_Device_ConfigureEndpoints(&VirtualSerial_CDC_Interface);

	//LEDs_SetAllLEDs(ConfigSuccess ? LEDMASK_USB_READY : LEDMASK_USB_ERROR);
}

/** Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest(void)
{
	CDC_Device_ProcessControlRequest(&VirtualSerial_CDC_Interface);
}

/** ISR to manage the reception of data from the serial port, placing received bytes into a circular buffer
 *  for later transmission to the host.
 */
ISR(USART1_RX_vect, ISR_BLOCK)
{
	uint8_t ReceivedByte = UDR1;

	// save new byte to the buffer (automatically discards if its disabled or full)
	if (USB_DeviceState == DEVICE_STATE_Configured)
		LRingBuffer_Insert(&ram.USARTtoUSB_Buffer, ReceivedByte);
}


///** Event handler for the CDC Class driver Line Encoding Changed event.
// *
// *  \param[in] CDCInterfaceInfo  Pointer to the CDC class interface configuration structure being referenced
// */
//void EVENT_CDC_Device_LineEncodingChanged(USB_ClassInfo_CDC_Device_t* const CDCInterfaceInfo)
//{
//	uint8_t ConfigMask = 0;
//
//	switch (CDCInterfaceInfo->State.LineEncoding.ParityType)
//	{
//	case CDC_PARITY_Odd:
//		ConfigMask = ((1 << UPM11) | (1 << UPM10));
//		break;
//	case CDC_PARITY_Even:
//		ConfigMask = (1 << UPM11);
//		break;
//	}
//
//	if (CDCInterfaceInfo->State.LineEncoding.CharFormat == CDC_LINEENCODING_TwoStopBits)
//		ConfigMask |= (1 << USBS1);
//
//	switch (CDCInterfaceInfo->State.LineEncoding.DataBits)
//	{
//	case 6:
//		ConfigMask |= (1 << UCSZ10);
//		break;
//	case 7:
//		ConfigMask |= (1 << UCSZ11);
//		break;
//	case 8:
//		ConfigMask |= ((1 << UCSZ11) | (1 << UCSZ10));
//		break;
//	}
//
//	/* Must turn off USART before reconfiguring it, otherwise incorrect operation may occur */
//	PORTD |= (1 << PD3); // Turn ON Tx while USART is being reconfigured
//	UCSR1B = 0;
//	UCSR1A = 0;
//	UCSR1C = 0;
//
//	/* Special case 57600 baud for compatibility with the ATmega328 bootloader. */
//	UBRR1 = (CDCInterfaceInfo->State.LineEncoding.BaudRateBPS == 57600)
//		? SERIAL_UBBRVAL(CDCInterfaceInfo->State.LineEncoding.BaudRateBPS)
//		: SERIAL_2X_UBBRVAL(CDCInterfaceInfo->State.LineEncoding.BaudRateBPS);
//
//	UCSR1C = ConfigMask;
//	UCSR1A = (CDCInterfaceInfo->State.LineEncoding.BaudRateBPS == 57600) ? 0 : (1 << U2X1);
//	UCSR1B = ((1 << RXCIE1) | (1 << TXEN1) | (1 << RXEN1));
//	PORTD &= ~(1 << PD3); // And turn OFF Tx once USART has been reconfigured (this is overridden by TXEN)
//
//}
//
//void EVENT_CDC_Device_ControLineStateChanged(USB_ClassInfo_CDC_Device_t* const CDCInterfaceInfo)
//{
//	bool CurrentDTRState = (CDCInterfaceInfo->State.ControlLineStates.HostToDevice & CDC_CONTROL_LINE_OUT_DTR);
//
//	if (CurrentDTRState){
//		AVR_RESET_LINE_PORT &= ~AVR_RESET_LINE_MASK;
//		
//	}
//	else{
//		AVR_RESET_LINE_PORT |= AVR_RESET_LINE_MASK;
//		
//	}
//}