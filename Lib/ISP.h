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


#ifndef ISP_H
#define ISP_H

#include "Metainclude.h"


//================================================================================
// AVRISP
//================================================================================

// extra delay added from adafruit <--
//#define LOW_SPEED
#ifdef ISP_LOW_SPEED
#define EXTRA_SPI_DELAY 125
#else
#define EXTRA_SPI_DELAY 0
#endif

// STK Definitions
#define HWVER 2
#define SWMAJ 1
#define SWMIN 18

#define STK_OK      0x10
#define STK_FAILED  0x11
#define STK_UNKNOWN 0x12
#define STK_INSYNC  0x14
#define STK_NOSYNC  0x15
#define CRC_EOP     0x20 //ok it is a space...

#define STK_GET_SYNC 0x30
#define STK_GET_SIGNON 0x31
#define STK_GET_PARM 0x41
#define STK_SET_PARM 0x42
#define STK_SET_PARM_EXT 0x45
#define STK_PMODE_START 0x50
#define STK_PMODE_END 0x51
#define STK_SET_ADDR 0x55
#define STK_UNIVERSAL 0x56
#define STK_PROG_FLASH 0x60
#define STK_PROG_DATA 0x61
#define STK_PROG_PAGE 0x64
#define STK_READ_PAGE 0x74
#define STK_READ_SIGN 0x75

// hardware configuration
#define AVR_SPI_PIN PINB
#define AVR_SPI_PORT PORTB
#define AVR_SPI_DDR DDRB
#define AVR_MOSI 2
#define AVR_MISO 3
#define AVR_SCK 1
// the "real" SS pin is not connected on a normal Arduino so we use another pin as SS.
// the hardware SS pin still needs to be in output mode to enable SPI master mode.
// SS is the pin thats close to the TX Led (bottom right)
#define AVR_SS 4
#define AVR_HARDWARE_SS 0

#define AVRISP_BAUD 1

#define LEDS_PMODE LEDS_LED1
#define LEDS_ERR LEDS_LED2

#define LOW 0
#define HIGH 1

void avrisp(void);

void sendCDCbyte(uint8_t b);
uint8_t getch(void);
void fill(int n);
void get_parameters(uint8_t c); // get_version
void set_parameters(void);
void breply(uint8_t b);
void replyOK(void); // empty_reply
void start_pmode(void);
void spi_init(void);
uint8_t spi_transaction(uint8_t a, uint8_t b, uint8_t c, uint8_t d);
uint8_t spi_send(uint8_t b);
void spi_wait(void);
void program_page(void);
uint8_t flash_read(uint8_t hilo, int addr);
char flash_read_page(int length);
void universal(void);
void read_signature(void);
void end_pmode(void);
void read_page(void);
void write_flash(int length);
uint8_t write_eeprom(int length);
uint8_t write_eeprom_chunk(int start, int length);
char eeprom_read_page(int length);
uint8_t write_flash_pages(int length);
void commit(int addr);
int current_page(void);
void flash(uint8_t hilo, int addr, uint8_t data);

// not used, replaved with intern delay TODO
void delay(unsigned long ms);
void delayMicroseconds(unsigned int us);

#endif

