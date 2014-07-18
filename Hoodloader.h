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
*  Header file for Hoodloader.c.
*/

#ifndef HOODLOADER_H
#define HOODLOADER_H

/* Includes: */
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include <string.h>

#include "Descriptors.h"

#include <LUFA/Drivers/Board/LEDs.h>
#include <LUFA/Drivers/USB/USB.h>
#include <LUFA/Platform/Platform.h>

#include <LUFA/Version.h>
#include <LUFA/Drivers/Peripheral/Serial.h>
#include <LUFA/Drivers/Misc/RingBuffer.h>

//================================================================================
// Modes
//================================================================================

#define MODE_DEFAULT 0
#define MODE_TRIGGER_DEFAULT 1
#define MODE_HID 2
#define MODE_TRIGGER_HID 3
#define MODE_AVRISP 4
#define MODE_TRIGGER_AVRISP 5

void mode_default(void);
void mode_hid(void);
void mode_avrisp(void);

//================================================================================
// Hardware
//================================================================================

#define TX_RX_LED_PULSE_MS 3
#define PING_PONG_LED_PULSE_MS 100

#define AVR_RESET_LINE_PORT PORTD
#define AVR_RESET_LINE_DDR DDRD
#define AVR_RESET_LINE_MASK (1 << 7)

#define AVR_NO_HID_PORT PORTB
#define AVR_NO_HID_DDR DDRB
#define AVR_NO_HID_PIN PINB
#define AVR_NO_HID_MASK (1 << 2)

// LED mask for the library LED driver, to indicate TX activity.
#define LEDMASK_TX               LEDS_LED1

// LED mask for the library LED driver, to indicate RX activity.
#define LEDMASK_RX               LEDS_LED2

// LED mask for the library LED driver, to indicate that an error has occurred in the USB interface.
#define LEDMASK_ERROR            (LEDS_LED1 | LEDS_LED2)

// LED mask for the library LED driver, to indicate that the USB interface is busy.
#define LEDMASK_BUSY             (LEDS_LED1 | LEDS_LED2)	

#define LEDMASK_AVRISP_ERR		LEDS_LED2
#define LEDMASL_AVRISP_PMODE	LEDS_LED1

void SetupHardware(void);

//================================================================================
// AVRISP
//================================================================================

// AVRISP
#include <LUFA/Drivers/Peripheral/SPI.h>
#include <stdlib.h> //for malloc
#define HWVER 2
#define SWMAJ 1
#define SWMIN 18

// STK Definitions
#define STK_OK      0x10
#define STK_FAILED  0x11
#define STK_UNKNOWN 0x12
#define STK_INSYNC  0x14
#define STK_NOSYNC  0x15
#define CRC_EOP     0x20 //ok it is a space...

#define PTIME 30

#define AVR_SPI_PORT PORTB
#define AVR_SPI_DDR DDRB
#define AVR_MOSI 2
#define AVR_MISO 3
#define AVR_SCK 1
// the "real" SS pin is not connected on a normal Arduino so we use another pin as SS.
// the hardware SS pin still needs to be in output mode to enable SPI master mode.
#define AVR_SS 4
#define AVR_HARDWARE_SS 0

#define beget16(addr) (*addr * 256 + *(addr+1) )

void avrisp(void);
uint8_t getch(void);
void empty_reply(void);
void breply(uint8_t b);
void start_pmode(void);
void spi_init(void);
void delayMicroseconds(unsigned int us);
uint8_t spi_transaction(uint8_t a, uint8_t b, uint8_t c, uint8_t d);
uint8_t spi_send(uint8_t b);
void program_page(void);
uint8_t write_flash_pages(uint8_t buffer[], int length);
void commit(int addr);
int current_page(void);
void flash(uint8_t hilo, int addr, uint8_t data);
uint8_t flash_read(uint8_t hilo, int addr);
uint8_t write_eeprom(int length);
uint8_t write_eeprom_chunkbuffer(int start, uint8_t buffer[], int length);
void end_pmode(void);
void read_signature(void);
void read_page(void);
char flash_read_page(int length);
char eeprom_read_page(int length);
void universal(void);
void write_flash(int length);
void fillbuffer(uint8_t buffer[], int n);

//================================================================================
// HID
//================================================================================

void checkNHPProtocol(uint8_t input);
void checkNHPControlAddressError(void);
uint8_t writeToCDC(uint8_t buffer[], uint8_t length);

//================================================================================
// NHP Definitions/Prototypes
//================================================================================

// ErrorLevel
#define NHP_MASK_INPUT		0x0F
#define NHP_INPUT_NO		0x00
#define NHP_INPUT_NEW		0x01
#define NHP_INPUT_ADDRESS	0x02
#define NHP_INPUT_COMMAND	0x04
#define NHP_INPUT_RESET		0x08
#define NHP_MASK_ERR		0xF0
#define NHP_ERR_NO			0x00
#define NHP_ERR_READ		0x10
#define NHP_ERR_END			0x20
#define NHP_ERR_DATA		0x40
#define NHP_ERR_LEAD		0x80
#define NHP_ERR_LIMIT		20	 //0-255, only for the user function

// Start Mask
#define NHP_MASK_START		0xC0 //B11|000000 the two MSB bits
#define NHP_MASK_LEAD		0xC0 //B11|000000
#define NHP_MASK_DATA		0x00 //B0|0000000 only the first MSB is important
#define NHP_MASK_END		0x80 //B10|000000

// Content Mask
#define NHP_MASK_LENGTH		0x38 //B00|111|000
#define NHP_MASK_COMMAND	0x0F //B0000|1111
#define NHP_MASK_DATA_7BIT	0x7F //B0|1111111
#define NHP_MASK_DATA_4BIT	0x0F //B0000|1111
#define NHP_MASK_DATA_3BIT	0x07 //B00000|111
#define NHP_MASK_ADDRESS	0x3F //B00|111111

// Reserved Addresses
#define NHP_ADDRESS_CONTROL 0x01

// Reserved Usages
#define NHP_USAGE_ARDUINOHID 0x01

// general multifunctional read/write functions for NHP
uint8_t NHPreadChecksum(uint8_t input);
uint8_t NHPwriteChecksum(uint8_t address, uint16_t indata, uint8_t* buff);

//================================================================================
// Lufa USB functions
//================================================================================

// general USB
void EVENT_USB_Device_Connect(void);
void EVENT_USB_Device_Disconnect(void);
void EVENT_USB_Device_ConfigurationChanged(void);
void EVENT_USB_Device_UnhandledControlRequest(void);

// CDC Serial
void EVENT_CDC_Device_LineEncodingChanged(USB_ClassInfo_CDC_Device_t* const CDCInterfaceInfo);
void EVENT_CDC_Device_ControLineStateChanged(USB_ClassInfo_CDC_Device_t* const CDCInterfaceInfo);	

// HID
void EVENT_USB_Device_ControlRequest(void);
void EVENT_USB_Device_StartOfFrame(void);
bool CALLBACK_HID_Device_CreateHIDReport(USB_ClassInfo_HID_Device_t* const HIDInterfaceInfo,
										 uint8_t* const ReportID,
										 const uint8_t ReportType,
										 void* ReportData,
										 uint16_t* const ReportSize);
void CALLBACK_HID_Device_ProcessHIDReport(USB_ClassInfo_HID_Device_t* const HIDInterfaceInfo,
										  const uint8_t ReportID,
										  const uint8_t ReportType,
										  const void* ReportData,
										  const uint16_t ReportSize);
#endif

