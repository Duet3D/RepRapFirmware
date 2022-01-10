/*
 * DueXn.h
 *
 *  Created on: 19 Oct 2016
 *      Author: David
 */

#ifndef SRC_DUETNG_DUEXN_H_
#define SRC_DUETNG_DUEXN_H_

#include <RepRapFirmware.h>

enum class ExpansionBoardType : uint8_t
{
	none,
	DueX0,
	DueX2,
	DueX2_v0_11 = (uint8_t)DueX2 + 1,
	DueX5,
	DueX5_v0_11 = (uint8_t)DueX5 + 1
};

namespace DuetExpansion
{
	ExpansionBoardType DueXnInit() noexcept;								// Look for a DueXn, initialise it and return which expansion board is attached
	void Exit();															// Stop the expander polling task
	void AdditionalOutputInit() noexcept;									// Look for an additional output pin expander
	void DueXnTaskInit() noexcept;											// Create the DueXn task and enable the associated interrupt from the DueXn
	const char* _ecv_array null GetExpansionBoardName() noexcept;			// Return the name of the expansion board, or nullptr if no expansion board
	const char* _ecv_array null GetAdditionalExpansionBoardName() noexcept;	// Return the name of the additional expansion board, or nullptr if no expansion board
	void SetPinMode(Pin pin, PinMode mode) noexcept;						// Set the I/O mode of a pin
	bool DigitalRead(Pin pin) noexcept;										// Read a pin
	void DigitalWrite(Pin pin, bool high) noexcept;							// Write a pin
	void AnalogOut(Pin pin, float pwm) noexcept;							// Set the PWM value on this pin
	uint16_t DiagnosticRead() noexcept;										// Diagnose the SX1509 by setting all pins as inputs and reading them
	void Diagnostics(MessageType mtype) noexcept;							// Print diagnostic data
}

#endif /* SRC_DUETNG_DUEXN_H_ */
