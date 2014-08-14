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


#ifndef NHP_H
#define NHP_H

#include "Metainclude.h"


//================================================================================
// NHP Definitions/Prototypes
//================================================================================

typedef struct{
	// in progress reading data
	uint8_t mBlocks;
	uint32_t mWorkData;

	// buffer for read operations
	uint8_t readbuffer[6];
	uint8_t readlength;
}NHP_Data_t;


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
void resetNHPbuffer(void);

#endif

