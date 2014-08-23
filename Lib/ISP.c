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

void avrispReset(void){
	ram.isp.error = 0;
	ram.isp.pmode = false;
	ram.isp._addr = 0; // just to be sure
}

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

//================================================================================
// Serial send/receive
//================================================================================

void sendCDCbyte(uint8_t b){
	//TODO improve this
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
	int ReceivedByte = -1;
	// wait until CDC sends a byte
	while (ReceivedByte < 0)
		ReceivedByte = CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);
	return ReceivedByte;
}

void fill(int n) {
	// fill the buffer with the number of bytes passed in from CDC Serial 
	for (int x = 0; x < n; x++)
		ram.RingBuffer_Data[x] = getch();
}

//================================================================================
// Start/End Pmode
//================================================================================

void start_pmode(void) {
	// clear all pending HID reports
	clearHIDReports();

	// set hardware SS to output so we can use SPI master mode
	AVR_SPI_DDR |= (1 << AVR_HARDWARE_SS);
	AVR_SPI_PORT |= (1 << AVR_HARDWARE_SS);

	spi_init();

	// following delays may not work on all targets...
	AVR_SPI_DDR |= (1 << AVR_SS); // OUTPUT
	AVR_SPI_PORT |= (1 << AVR_SS); // HIGH

	AVR_SPI_DDR |= (1 << AVR_SCK); // OUTPUT
	AVR_SPI_PORT &= ~(1 << AVR_SCK); // LOW
	_delay_ms(50 + EXTRA_SPI_DELAY);

	AVR_SPI_PORT &= ~(1 << AVR_SS); // LOW

	_delay_ms(50 + EXTRA_SPI_DELAY); // extra delay added from adafruit <--
	AVR_SPI_DDR &= ~(1 << AVR_MISO); // INPUT
	AVR_SPI_DDR |= (1 << AVR_MOSI); // OUTPUT

	spi_transaction(0xAC, 0x53, 0x00, 0x00);
	ram.isp.pmode = true;

	//TODO need a reset here?
	// do not write Serial stuff into buffer, we need this ram now
	LRingBuffer_ResetBuffer(&ram.RingBuffer);
	ram.skipNHP = 0;
	NHPreset(&ram.NHP);
	return;
}

void end_pmode(void) {
	AVR_SPI_DDR &= ~(1 << AVR_MISO); // INPUT
	AVR_SPI_DDR &= ~(1 << AVR_MOSI); // INPUT
	AVR_SPI_DDR &= ~(1 << AVR_SCK);  // INPUT
	AVR_SPI_DDR &= ~(1 << AVR_SS);   // INPUT

	// set hardware SS to input so we can use SPI slave mode
	AVR_SPI_DDR &= ~(1 << AVR_HARDWARE_SS); // INPUT

	// release this buffer for Serial again
	LRingBuffer_ResetBuffer(&ram.RingBuffer);

	ram.isp.pmode = false;

	ram.skipNHP = 0;
	NHPreset(&ram.NHP);
}

//================================================================================
// General Programmer functions
//================================================================================

void breply(uint8_t b) {
	if (CRC_EOP == getch()) {
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
	if (CRC_EOP == getch()) {
		sendCDCbyte(STK_INSYNC);
		sendCDCbyte(STK_OK);
	}
	else {
		sendCDCbyte(STK_NOSYNC);
		ram.isp.error++;
	}
}

void universal(void) {
	uint8_t ch;

	fill(4);
	ch = spi_transaction(ram.RingBuffer_Data[0], ram.RingBuffer_Data[1], ram.RingBuffer_Data[2], ram.RingBuffer_Data[3]);
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
	fill(20);

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
	ram.isp.param.pagesize = beget16(&ram.RingBuffer_Data[12]);
	ram.isp.param.eepromsize = beget16(&ram.RingBuffer_Data[14]);

	// 32 bits flashsize (big endian)
	//param.flashsize = buff[16] * 0x01000000
	//	+ buff[17] * 0x00010000
	//	+ buff[18] * 0x00000100
	//	+ buff[19];
}

//================================================================================
// SPI
//================================================================================

void spi_init(void) {
	SPCR = 0x53;
#ifdef ISP_LOW_SPEED
	SPCR = SPCR | B00000011;
#endif
	SPSR;
	SPDR;
}

void spi_wait(void) {
	do {
	} while (!(SPSR & (1 << SPIF)));
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

uint8_t spi_transaction(uint8_t a, uint8_t b, uint8_t c, uint8_t d) {
	// here was an unused error correction? <--
	spi_send(a);
	spi_send(b);
	spi_send(c);
	return spi_send(d);
}

uint8_t accessData(uint8_t type, int addr, uint8_t data){
	// spi: type, address MSB, address LSB, data
	return spi_transaction(type, (addr >> 8) & 0xFF, addr & 0xFF, data);
}

//================================================================================
// Read page
//================================================================================

void read_page(void) {
	// get length, memtype and check if signal is still okay
	int length = 256 * getch();
	length += getch();
	char memtype = getch();

	if (CRC_EOP != getch()) {
		ram.isp.error++;
		sendCDCbyte(STK_NOSYNC);
		return;
	}

	sendCDCbyte(STK_INSYNC);
	char result = (char)STK_FAILED;

	// determine what memtype is requested: flash or eeprom
	if (memtype == 'F'){
		// flash_read_page
		for (int x = 0; x < length; x += 2) {
			// flash_read
			uint8_t low = accessData(0x20 + LOW * 8, ram.isp._addr, 0x00);
			sendCDCbyte(low);

			// flash_read
			uint8_t high = accessData(0x20 + HIGH * 8, ram.isp._addr, 0x00);
			sendCDCbyte(high);

			ram.isp._addr++;
		}
		result = STK_OK;
	}

	if (memtype == 'E'){
		// eeprom_read_page
		// here again we have a word address
		int start = ram.isp._addr * 2;
		for (int x = 0; x < length; x++) {
			int addr = start + x;
			// TODO itegrate intern address ? <--
			uint8_t ee = accessData(0xA0, addr, 0xFF);
			sendCDCbyte(ee);
		}
		result = STK_OK;
	}

	sendCDCbyte(result);
	return;
}

//================================================================================
// Programm page
//================================================================================

void program_page(void) {
	char result = (char)STK_FAILED;
	int length = 256 * getch();
	length += getch();

	char memtype = getch();
	// flash memory @here, (length) bytes
	if (memtype == 'F') {
		uint8_t result = write_flash(length);
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

uint8_t write_flash(int length) {
	// here is a word address, get the byte address
	int start = ram.isp._addr * 2;
	int remaining = length;

	while (remaining > sizeof(ram.RingBuffer_Data)) {
		write_flash_chunk(start, sizeof(ram.RingBuffer_Data));
		start += sizeof(ram.RingBuffer_Data);
		remaining -= sizeof(ram.RingBuffer_Data);
	}
	write_flash_chunk(start, remaining);
	return STK_OK;

}

int current_page(void) {
	if (ram.isp.param.pagesize == 32)  return ram.isp._addr & 0xFFFFFFF0;
	if (ram.isp.param.pagesize == 64)  return ram.isp._addr & 0xFFFFFFE0;
	if (ram.isp.param.pagesize == 128) return ram.isp._addr & 0xFFFFFFC0;
	if (ram.isp.param.pagesize == 256) return ram.isp._addr & 0xFFFFFF80;
	return ram.isp._addr;
}

void commit(int addr) {
	LEDs_TurnOffLEDs(LEDS_PMODE);
	accessData(0x4C, addr, 0x00);
	_delay_ms(30);
	LEDs_TurnOnLEDs(LEDS_PMODE);
}

uint8_t write_flash_chunk(int start, int length) {
	// this writes byte-by-byte,
	// page writing may be faster (4 bytes at a time)
	fill(length);
	int x = 0;
	int page = current_page();
	while (x < length) {
		// flash
		accessData(0x40 + 8 * LOW, ram.isp._addr, ram.RingBuffer_Data[x++]);

		// flash
		accessData(0x40 + 8 * HIGH, ram.isp._addr, ram.RingBuffer_Data[x++]);

		ram.isp._addr++;

		// TODO check if its okay with this commit + chunk comibation <--
		// could be a possible good fix if i understood this right
		// check if current page finished, commit and start a new page
		if (page != current_page()) {
			// TODO merge commit here
			commit(page);
			page = current_page();
		}
	}
	//commit(page);

	return STK_OK;
}

uint8_t write_eeprom(int length) {
	// here is a word address, get the byte address
	int start = ram.isp._addr * 2;
	int remaining = length;
	if (length > ram.isp.param.eepromsize) {
		ram.isp.error++;
		return STK_FAILED;
	}
	while (remaining > sizeof(ram.RingBuffer_Data)) {
		write_eeprom_chunk(start, sizeof(ram.RingBuffer_Data));
		start += sizeof(ram.RingBuffer_Data);
		remaining -= sizeof(ram.RingBuffer_Data);
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
		//TODO check here if we can use intern address
		int addr = start + x;
		accessData(0xC0, addr, ram.RingBuffer_Data[x]);
		_delay_ms(45);
	}
	LEDs_TurnOnLEDs(LEDS_PMODE);
	return STK_OK;
}

