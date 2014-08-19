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

#include "ISP.h"


//================================================================================
// AVRISP
//================================================================================

void avrisp(int ReceivedByte){
	// is pmode active?
	if (ram.isp.pmode) LEDs_TurnOnLEDs(LEDS_PMODE);
	else LEDs_TurnOffLEDs(LEDS_PMODE);

	// is there an error?
	if (ram.isp.error) LEDs_TurnOnLEDs(LEDS_ERR);
	else LEDs_TurnOffLEDs(LEDS_ERR);

	// read in bytes from the CDC interface
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
		ram.ispBuffer[x] = getch();
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
	ram.isp.param.pagesize = beget16(&ram.ispBuffer[12]);
	ram.isp.param.eepromsize = beget16(&ram.ispBuffer[14]);

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
	// do not write into Serial buffer, we need this ram now
	LRingBuffer_DisableBuffer(&ram.USARTtoUSB_Buffer);

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
	ch = spi_transaction(ram.ispBuffer[0], ram.ispBuffer[1], ram.ispBuffer[2], ram.ispBuffer[3]);
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

	// enable Serial buffer again
	if (!LRingBuffer_IsEnabled(&ram.USARTtoUSB_Buffer))
		LRingBuffer_InitBuffer(&ram.USARTtoUSB_Buffer);

	// HID Setup
	ram.NHP.reset = true;
	ram.NHP.leadError = false;
	ram.HID.ID = 0;

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
		spi_transaction(0xC0, (addr >> 8) & 0xFF, addr & 0xFF, ram.ispBuffer[x]);
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
		flash(LOW, ram.isp._addr, ram.ispBuffer[x++]);
		flash(HIGH, ram.isp._addr, ram.ispBuffer[x++]);
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

