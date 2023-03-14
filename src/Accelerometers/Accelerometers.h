/*
 * Accelerometers.h
 *
 *  Created on: 19 Mar 2021
 *      Author: David
 */

#ifndef SRC_ACCELEROMETERS_ACCELEROMETERS_H_
#define SRC_ACCELEROMETERS_ACCELEROMETERS_H_

#include <RepRapFirmware.h>

#if SUPPORT_ACCELEROMETERS

#include <GCodes/GCodeException.h>

#if SUPPORT_CAN_EXPANSION
# include <CanId.h>
#endif

class CanMessageAccelerometerData;

namespace Accelerometers
{
	bool HasLocalAccelerometer() noexcept;
	float GetLocalAccelerometerLastRunAverage(const int& axis) noexcept;
	unsigned int GetLocalAccelerometerRuns() noexcept;
	unsigned int GetLocalAccelerometerDataPoints() noexcept;
	GCodeResult ConfigureAccelerometer(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);
	GCodeResult StartAccelerometer(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);
	void Exit() noexcept;
#if SUPPORT_CAN_EXPANSION
	void ProcessReceivedData(CanAddress src, const CanMessageAccelerometerData& msg, size_t msgLen) noexcept;
#endif
}

#endif

#endif /* SRC_ACCELEROMETERS_ACCELEROMETERS_H_ */
