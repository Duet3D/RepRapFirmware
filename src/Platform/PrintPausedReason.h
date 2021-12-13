/*
 * PrintPausedReason.h
 *
 *  Created on: 12 Dec 2021
 *      Author: David
 */

#ifndef SRC_PLATFORM_PRINTPAUSEDREASON_H_
#define SRC_PLATFORM_PRINTPAUSEDREASON_H_

#include <cstdint>

// The following values must be kept in sync with DSF! So don't change them unless making major changes to the SBC interface.
enum class PrintPausedReason : uint8_t
{
	dontPause = 0,						// used by RRF but not by DSF
	user = 1,
	gcode = 2,
	filamentChange = 3,
	trigger = 4,
	heaterFault = 5,
	filamentError = 6,
	stall = 7,
	lowVoltage = 8,
	driverError = 9
};

#endif /* SRC_PLATFORM_PRINTPAUSEDREASON_H_ */
