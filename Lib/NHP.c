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

NHP_Enum_t NHPreadChecksum(uint8_t input, NHP_Data_t* protocol){
	NHP_Enum_t errorLevel = NHPread(input, protocol);
	// if its an valid address, check the data inverse too
	if (errorLevel > 0 && (((protocol->mWorkData & 0xFFFF) ^ (protocol->mWorkData >> 16)) != 0xFFFF)){
		errorLevel = NHP_ERR_CHECKSUM;
		protocol->reset = true;
	}
	return errorLevel;
}

NHP_Enum_t NHPread(uint8_t input, NHP_Data_t* protocol){
	// check if previous reading had a lead error, copy that lead byte to the beginning
	if (protocol->leadError){
		protocol->readbuffer[0] = protocol->readbuffer[protocol->readlength];
		protocol->readlength = 1;

		// reset leadError indicator
		protocol->leadError = false;
	}
	// completely reset the protocol after sucessfull reading/error last time
	else if (protocol->reset){
		protocol->mBlocks = 0;
		protocol->readlength = 0;
	}

	//write input to the buffer
	protocol->readbuffer[protocol->readlength++] = input;

	// create errorLevel that will be returned (contains errors or address)
	NHP_Enum_t errorLevel;
	
	// check the header(lead/data/end) indicator
	switch (input & NHP_MASK_HEADER){

	case(NHP_HEADER_LEAD) :
	{
		// read command indicator or block length
		uint8_t blocks = (input & NHP_MASK_LENGTH) >> 3;

		if (protocol->mBlocks){
			// we were still reading! Log an error but continue reading with this new lead
			errorLevel = NHP_ERR_LEAD;
			// write the buffer without the new lead, move it next reading
			protocol->readlength--;
			// set indicator to move this lead byte to the beginning next reading
			protocol->leadError = true;
		}
		else
			errorLevel = NHP_NO_ERR;

		if (blocks == 0 || blocks == 1){
			// save command in data variable
			//protocol->mWorkData = (input & NHP_MASK_COMMAND) + 1;

			// ignore command
			protocol->readlength += protocol->leadError;
			protocol->leadError = false;

			// return command indicator
			errorLevel = NHP_COMMAND;
			break;
		}

		else if (blocks == 7){
			// save block length + first 4 data bits (special 32 bit case)
			protocol->mWorkData = input & NHP_MASK_DATA_4BIT;
			blocks -= 2;
		}
		else{
			// save block length + first 3 data bits
			protocol->mWorkData = input & NHP_MASK_DATA_3BIT;
			blocks--;
		}

		// save new block length to the protocol data
		protocol->mBlocks = blocks;
	}
						  break;

	case NHP_HEADER_DATA_A:
	case NHP_HEADER_DATA_B:
	default:
	{
		if (protocol->mBlocks >= 2){
			// get next 7 bits of data
			protocol->mBlocks--;
			protocol->mWorkData <<= 7;
			// dont need &NHP_MASK_DATA_7BIT because first MSB bit is zero!
			protocol->mWorkData |= input;
			errorLevel = NHP_NO_ERR;
		}
		else
			// log an error, expecting a lead or end byte
			errorLevel = NHP_ERR_DATA;
	}
		break;

	case(NHP_HEADER_END) :
	{
		if (protocol->mBlocks == 1){
			// return the address
			uint8_t address = (input & 0x3F) + 1;
			errorLevel = address;
		}
		else
			errorLevel = NHP_ERR_END;
		// wrong checksum or wrong end, write down buffer
	}
						 break;
	} // end switch

	// reset next reading on valid input/error/command
	if (errorLevel != NHP_NO_ERR)
		protocol->reset = true;
	// ignore in progress reading
	else
		protocol->reset = false;

	// return the errors
	return errorLevel;
}

//================================================================================
// Write NHP
//================================================================================

uint8_t NHPwrite(uint8_t address, uint16_t data, uint8_t* buff){
	// writes two bytes with its inverse
	uint32_t temp = ~data;
	uint32_t checksum = (temp << 16) | data;
	return NHPwriteChecksum(address, checksum, buff);
}

uint8_t NHPwriteChecksum(uint8_t address, uint16_t data, uint8_t* buff){
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
	buff[0] |= NHP_HEADER_LEAD | (blocks << 3);

	// write end mask
	buff[blocks - 1] = NHP_HEADER_END | ((address - 1) & NHP_MASK_ADDRESS);

	// return the length
	return blocks;
}

//
//// reads two bytes and check its inverse
//uint8_t NHPreadChecksum(uint8_t input){
//	//write input to the buffer
//	ram.NHP.readbuffer[ram.NHP.readlength] = input;
//	ram.NHP.readlength++;
//
//	// check the lead/end/data indicator
//	switch (input & NHP_MASK_START){
//
//	case(NHP_MASK_LEAD) :
//	{
//		// read command indicator or block length
//		uint8_t blocks = (input & NHP_MASK_LENGTH) >> 3;
//
//		// ignore command, return 0 write buff down completely
//		if (blocks == 0 || blocks == 1)
//			break;
//
//		else if (blocks == 7){
//			// save block length + first 4 data bits (special case)
//			ram.NHP.mWorkData = input & NHP_MASK_DATA_4BIT;
//			blocks -= 2;
//		}
//		else{
//			// save block length + first 3 data bits
//			ram.NHP.mWorkData = input & NHP_MASK_DATA_3BIT;
//			blocks--;
//		}
//
//		// we were still reading!  Log an error
//		if (ram.NHP.mBlocks){
//			// check if previous reading was a valid Control Address and write it down
//			checkNHPControlAddressError();
//			// write down the last signal but keep lead
//			// substract 1 more because we already added the count
//			writeToCDC(ram.NHP.readbuffer, ram.NHP.readlength - 1);
//			ram.NHP.readbuffer[0] = ram.NHP.readbuffer[ram.NHP.readlength - 1];
//			ram.NHP.readlength = 1;
//		}
//		// save new block length
//		ram.NHP.mBlocks = blocks;
//		return 0; // everything is okay
//	}
//						break;
//
//	case(NHP_MASK_END) :
//	{
//		if (ram.NHP.mBlocks == 1){
//			// save data + address
//			// we know its a valid input, left some things out here
//			if (((ram.NHP.mWorkData & 0xFFFF) ^ (ram.NHP.mWorkData >> 16)) == 0xFFFF){
//				uint8_t address = (input & 0x3F) + 1;
//				// do NOT reset for new reading, cause the values might be wrong and need to be written down again.
//				return address;
//			}
//		}
//		// wrong checksum or wrong end, write down buffer
//	}
//					   break;
//
//	default:
//	{
//		if (ram.NHP.mBlocks >= 2){
//			ram.NHP.mBlocks--;
//			// get next 7 bits of data
//			ram.NHP.mWorkData <<= 7;
//			// dont need &NHP_MASK_DATA_7BIT because first bit is zero!
//			ram.NHP.mWorkData |= input;
//			return 0; // everything is okay
//		}
//		// log an error, expecting an address or header byte
//	}
//		break;
//	} // end switch
//
//	// check if previous reading was a valid Control Address and write it down
//	checkNHPControlAddressError();
//
//	// invalid input, write down buffer
//	writeToCDC(ram.NHP.readbuffer, ram.NHP.readlength);
//	resetNHPbuffer();
//	return 0;
//}