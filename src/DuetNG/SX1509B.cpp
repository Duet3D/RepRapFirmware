/*
 * SX1509B.cpp
 *
 *  Created on: 19 Oct 2016
 *      Author: David
 */

#include "SX1509B.h"

const uint8_t DueXnAddress = 0x3E;				// address of the SX1509B on the DueX0/DueX2/DueX5

// Initialise the device and identify which expansion board (if any) is attached
ExpansionBoardType SX1509B::Init()
{
	Wire.begin();								// initialise TWI as master

	Wire.beginTransmission(DueXnAddress);		// start a block

	// Return codes from Wire.endTransmission are:
	// 0: success
	// 1: data too long to fit in transmit buffer
	// 2: received NACK on transmit of address
	// 3: received NACK on transmit of data
	// 4: other error
	// We assume that any return code other than 0 means the device was not found.
	if (Wire.endTransmission() != 0)
	{
		// No device found at that address, or a serious error
		return ExpansionBoardType::none;
	}

	return ExpansionBoardType::none;	//TODO
}


// End
