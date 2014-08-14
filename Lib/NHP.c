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

#include "NHP.h"


//================================================================================
// Read NHP
//================================================================================

// reads two bytes and check its inverse
uint8_t NHPreadChecksum(uint8_t input){
	//write input to the buffer
	ram.NHP.readbuffer[ram.NHP.readlength] = input;
	ram.NHP.readlength++;

	// check the lead/end/data indicator
	switch (input & NHP_MASK_START){

	case(NHP_MASK_LEAD) :
	{
		// read command indicator or block length
		uint8_t blocks = (input & NHP_MASK_LENGTH) >> 3;

		// ignore command, return 0 write buff down completely
		if (blocks == 0 || blocks == 1)
			break;

		else if (blocks == 7){
			// save block length + first 4 data bits (special case)
			ram.NHP.mWorkData = input & NHP_MASK_DATA_4BIT;
			blocks -= 2;
		}
		else{
			// save block length + first 3 data bits
			ram.NHP.mWorkData = input & NHP_MASK_DATA_3BIT;
			blocks--;
		}

		// we were still reading!  Log an error
		if (ram.NHP.mBlocks){
			// check if previous reading was a valid Control Address and write it down
			checkNHPControlAddressError();
			// write down the last signal but keep lead
			// substract 1 more because we already added the count
			writeToCDC(ram.NHP.readbuffer, ram.NHP.readlength - 1);
			ram.NHP.readbuffer[0] = ram.NHP.readbuffer[ram.NHP.readlength - 1];
			ram.NHP.readlength = 1;
		}
		// save new block length
		ram.NHP.mBlocks = blocks;
		return 0; // everything is okay
	}
						break;

	case(NHP_MASK_END) :
	{
		if (ram.NHP.mBlocks == 1){
			// save data + address
			// we know its a valid input, left some things out here
			if (((ram.NHP.mWorkData & 0xFFFF) ^ (ram.NHP.mWorkData >> 16)) == 0xFFFF){
				uint8_t address = (input & 0x3F) + 1;
				// do NOT reset for new reading, cause the values might be wrong and need to be written down again.
				return address;
			}
		}
		// wrong checksum or wrong end, write down buffer
	}
					   break;

	default:
	{
		if (ram.NHP.mBlocks >= 2){
			ram.NHP.mBlocks--;
			// get next 7 bits of data
			ram.NHP.mWorkData <<= 7;
			// dont need &NHP_MASK_DATA_7BIT because first bit is zero!
			ram.NHP.mWorkData |= input;
			return 0; // everything is okay
		}
		// log an error, expecting an address or header byte
	}
		break;
	} // end switch

	// check if previous reading was a valid Control Address and write it down
	checkNHPControlAddressError();

	// invalid input, write down buffer
	writeToCDC(ram.NHP.readbuffer, ram.NHP.readlength);
	resetNHPbuffer();
	return 0;
}

//================================================================================
// Write NHP
//================================================================================

// writes two bytes with its inverse
uint8_t NHPwriteChecksum(uint8_t address, uint16_t indata, uint8_t* buff){
	// create checksum data
	uint32_t temp = ~indata;
	uint32_t data = (temp << 16) | indata;

	// start with the maximum size of blocks
	uint8_t blocks = 7;

	// check for the first 7 bit block that doesnt fit into the first 3 bits
	while (blocks > 2){
		uint8_t nextvalue = (data >> (7 * (blocks - 3)));
		if (nextvalue > NHP_MASK_DATA_3BIT){
			// special case for the MSB
			if (blocks == 7) {
				buff[0] = nextvalue;
				blocks--;
			}
			break;
		}
		else{
			// write the possible first 3 bits and check again after if zero
			buff[0] = nextvalue;
			blocks--;
			// we have our first bits, stop (nonzero)
			if (nextvalue)
				break;
		}
	}

	// write the rest of the data bits
	uint8_t datablocks = blocks - 2;
	while (datablocks > 0){
		buff[datablocks] = data & NHP_MASK_DATA_7BIT;
		data >>= 7;
		datablocks--;
	}

	// write lead + length mask
	buff[0] |= NHP_MASK_LEAD | (blocks << 3);

	// write end mask
	buff[blocks - 1] = NHP_MASK_END | ((address - 1) & NHP_MASK_ADDRESS);

	// return the length
	return blocks;
}
