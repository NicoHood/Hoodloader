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

#include "CDC.h"

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



/** Event handler for the CDC Class driver Line Encoding Changed event.
*
*  \param[in] CDCInterfaceInfo  Pointer to the CDC class interface configuration structure being referenced
*/
void EVENT_CDC_Device_LineEncodingChanged(USB_ClassInfo_CDC_Device_t* const CDCInterfaceInfo)
{
	// enable Serial buffer again
	if (!LRingBuffer_IsEnabled(&ram.USARTtoUSB_Buffer))
		LRingBuffer_InitBuffer(&ram.USARTtoUSB_Buffer);

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

	// configure Serial with HID baud to work after reprogramming
	if (CDCInterfaceInfo->State.LineEncoding.BaudRateBPS == AVRISP_BAUD){
		Serial_Init(115200, true);
		SerialInitHID();
	}

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

// change Serial baud to 115200 for HID
void SerialInitHID(void){
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
}


