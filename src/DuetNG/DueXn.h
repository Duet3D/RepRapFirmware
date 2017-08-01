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

enum class ExpansionBoardType : uint8_t
{
	none,
	DueX0,
	DueX2,
	DueX5
};

namespace DuetExpansion
{
	ExpansionBoardType Init();						// Initialise the device and identify which expansion board is attached
	const char* array null GetExpansionBoardName();	// Return the name of the expansion board, or nullptr if no expansion board
	void SetPinMode(Pin pin, PinMode mode);			// Set the I/O mode of a pin
	bool DigitalRead(Pin pin);						// Read a pin
	void DigitalWrite(Pin pin, bool high);			// Write a pin
	void AnalogOut(Pin pin, float pwm);				// Set the PWM value on this pin
	uint16_t DiagnosticRead();						// Diagnose the SX1509 by setting all pins as inputs and reading them
	void Spin(bool full);							// Task to keep the endstop inputs up to date
};

#endif /* SRC_DUETNG_DUEXN_H_ */
