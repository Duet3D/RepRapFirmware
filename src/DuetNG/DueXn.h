/*
 * DueXn.h
 *
 *  Created on: 19 Oct 2016
 *      Author: David
 */

#ifndef SRC_DUETNG_DUEXN_H_
#define SRC_DUETNG_DUEXN_H_

#include "ecv.h"
#include "Core.h"
#include "MessageType.h"

enum class ExpansionBoardType : uint8_t
{
	none,
	DueX0,
	DueX2,
	DueX5
};

namespace DuetExpansion
{
	ExpansionBoardType DueXnInit();								// Look for a DueXn, initialise it and return which expansion board is attached
	void AdditionalOutputInit();								// Look for an additional output pin expander
	void DueXnTaskInit();										// Create the DueXn task and enable the associated interrupt from the DueXn
	const char* array null GetExpansionBoardName();				// Return the name of the expansion board, or nullptr if no expansion board
	const char* array null GetAdditionalExpansionBoardName();	// Return the name of the additional expansion board, or nullptr if no expansion board
	void SetPinMode(Pin pin, PinMode mode);						// Set the I/O mode of a pin
	bool DigitalRead(Pin pin);									// Read a pin
	void DigitalWrite(Pin pin, bool high);						// Write a pin
	void AnalogOut(Pin pin, float pwm);							// Set the PWM value on this pin
	uint16_t DiagnosticRead();									// Diagnose the SX1509 by setting all pins as inputs and reading them
	void Diagnostics(MessageType mtype);						// Print diagnostic data
};

#endif /* SRC_DUETNG_DUEXN_H_ */
