/*
 * DriverModes.h
 *
 *  Created on: 27 Apr 2018
 *      Author: David
 */

#ifndef SRC_GCODES_DRIVERMODE_H_
#define SRC_GCODES_DRIVERMODE_H_

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

#endif /* SRC_GCODES_DRIVERMODE_H_ */
