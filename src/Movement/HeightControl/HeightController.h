/*
 * HeightController.h
 *
 *  Created on: 18 Apr 2019
 *      Author: David
 */

#ifndef SRC_MOVEMENT_HEIGHTCONTROL_HEIGHTCONTROLLER_H_
#define SRC_MOVEMENT_HEIGHTCONTROL_HEIGHTCONTROLLER_H_

#include "RepRapFirmware.h"
#undef array

#if SUPPORT_ASYNC_MOVES

#include "GCodes/GCodeResult.h"
#include <RTOSIface/RTOSIface.h>

class HeightController
{
public:
	HeightController() noexcept;

	GCodeResult Configure(GCodeBuffer& gb, const StringRef& reply) THROWS_GCODE_EXCEPTION;
	GCodeResult StartHeightFollowing(GCodeBuffer& gb, const StringRef& reply) noexcept;		// Start/stop height following
	void Stop() noexcept;							// stop height following mode

	[[noreturn]] void RunTask() noexcept;

private:
	void CalcDerivedValues() noexcept;

	static constexpr unsigned int HeightControllerTaskStackWords = 100;
	static constexpr uint32_t DefaultSampleInterval = 200;

	Task<HeightControllerTaskStackWords> *heightControllerTask;
	int sensorNumber;								// which sensor, normally a virtual heater, or -1 if not configured
	uint32_t sampleInterval;						// in milliseconds
	uint32_t lastWakeTime;
	float setPoint;									// the sensor output we are aiming for
	float lastReading;								// the last reading we took from the sensor
	float pidP;
	float configuredPidI, configuredPidD;			// the PID parameters
	float actualPidI, actualPidD;
	float iAccumulator;								// the integral PID component
	float zMin, zMax;								// the control limits
	float currentZ;
	float startSpeed;
	float maxSpeed;
	float acceleration;
	float maxZAdjustmentPerSample;					// how much Z adjustment is possible in one sample period

	enum class PidState : uint8_t
	{
		stopped,
		starting,
		running
	};

	volatile PidState state;						// volatile because it is accessed by more than one task
	bool lastReadingOk;
};

#endif

#endif /* SRC_MOVEMENT_HEIGHTCONTROL_HEIGHTCONTROLLER_H_ */
