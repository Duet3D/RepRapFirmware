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
	constantOffTime = 0,
	randomOffTime,
	spreadCycle,
	stealthChop,			// includes stealthChop2
	unknown					// must be last!
};

const char* TranslateDriverMode(unsigned int mode);

inline const char* TranslateDriverMode(DriverMode mode)
{
	return TranslateDriverMode((unsigned int)mode);
}

// Register codes used to implement M569 command parameters.
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
	thigh,
	mstepPos,
	pwmScale
};

#endif /* SRC_MOVEMENT_STEPPERDRIVERS_DRIVERMODE_H_ */
