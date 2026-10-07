/*
 * DriverModes.h
 *
 *  Created on: 27 Apr 2018
 *      Author: David
 */

#ifndef SRC_MOVEMENT_STEPPERDRIVERS_DRIVERMODE_H_
#define SRC_MOVEMENT_STEPPERDRIVERS_DRIVERMODE_H_

enum class DriverMode : unsigned int
{
	// The values are reported in the object model, so they must not depend on build options
	constantOffTime = 0,
	randomOffTime = 1,
	spreadCycle = 2,
	stealthChop = 3,		// includes stealthChop2
	direct = 4,				// field-oriented control
	assistedOpen = 5,		// direct mode with assisted open loop control, closed loop expansion boards only
	unknown = 6				// must be last!
};

const char *_ecv_array TranslateDriverMode(unsigned int mode) noexcept;

inline const char *_ecv_array TranslateDriverMode(DriverMode mode) noexcept
{
	return TranslateDriverMode((unsigned int)mode);
}

// Register codes used to implement M569 command parameters and closed-loop control.
// This common set is used for all smart drivers. Not all are complete registers, some are just parts of registers.

enum class SmartDriverRegister : unsigned int
{
	toff,
	tblank,
	hstart,
	hend,
	hdec,
	chopperControl,
	coolStep,
	tpwmthrs,
	tcoolthrs,
	thigh,
	mstepPos,
	pwmScale,
	pwmAuto,
};

#endif /* SRC_MOVEMENT_STEPPERDRIVERS_DRIVERMODE_H_ */
