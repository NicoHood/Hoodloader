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

#include <stdint.h> //uint_t definitions
#include <stdbool.h> //bool type

//================================================================================
// NHP Definitions/Prototypes
//================================================================================

// Reserved Addresses
#define NHP_ADDRESS_CONTROL 0x01

// Reserved Usages for control address
#define NHP_USAGE_ARDUINOHID 0x01

// Header Masks
#define NHP_MASK_HEADER		0xC0 // B11|000000 the two MSB bits determine the block type
#define NHP_HEADER_LEAD		0xC0 // B11|000000 11 MSB
#define NHP_HEADER_DATA_A	0x00 // B00|000000 0X MSB only the first MSB is important
#define NHP_HEADER_DATA_B	0x40 // B01|000000 0X MSB only the first MSB is important
#define NHP_HEADER_END		0x80 // B10|000000 01 MSB

// Lead
#define NHP_MASK_LENGTH		0x38 // B00|111|000
#define NHP_MASK_COMMAND	0x0F // B0000|1111

// Data
#define NHP_MASK_DATA_7BIT	0x7F // B0|1111111 // data in data block
#define NHP_MASK_DATA_4BIT	0x0F // B0000|1111 // data in lead (32 bit)
#define NHP_MASK_DATA_3BIT	0x07 // B00000|111 // data in lead

// End
#define NHP_MASK_ADDRESS	0x3F // B00|111111 // 64 bit address in end block

// enum that contains address or errorLevel
typedef enum{
	// 1-64 valid addresses
	NHP_NO_ERR		=  0,
	NHP_COMMAND		= -1, // special case if you want to use the command
	NHP_ERR_LEAD	= -2,
	NHP_ERR_DATA	= -3,
	NHP_ERR_END		= -4,
	NHP_ERR_CHECKSUM= -5,
} NHP_Enum_t;

// protocol data for temporary variables
typedef struct{
	// in progress reading data variables
	uint8_t mBlocks : 3; // 0-7
	uint8_t readlength : 3; //0-6
	uint8_t leadError : 1; // true/false
	uint8_t reset : 1; // true/false
	uint32_t mWorkData;

	// buffer for read operations
	uint8_t readbuffer[6];
}NHP_Data_t;

// general multifunctional read/write functions for NHP
int8_t NHPreadChecksum(uint8_t input, NHP_Data_t* protocol);
int8_t NHPread(uint8_t input, NHP_Data_t* protocol);
//uint8_t NHPreadChecksum(uint8_t input);
uint8_t NHPwriteChecksum(uint8_t address, uint16_t indata, uint8_t* buff);

#endif

