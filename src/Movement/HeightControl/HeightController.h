/*
 * HeightController.h
 *
 *  Created on: 18 Apr 2019
 *      Author: David
 */

#ifndef SRC_MOVEMENT_HEIGHTCONTROL_HEIGHTCONTROLLER_H_
#define SRC_MOVEMENT_HEIGHTCONTROL_HEIGHTCONTROLLER_H_

#include "RepRapFirmware.h"
#include "GCodes/GCodeResult.h"
#include <RTOSIface/RTOSIface.h>

class HeightController
{
public:
	HeightController(size_t p_sensorNumber);
	~HeightController();

	GCodeResult Configure(GCodeBuffer& gb, const StringRef& reply);

	void Start();
	void Stop();
	[[noreturn]] void RunTask();

private:
	void CalcDerivedValues();

	static constexpr unsigned int HeightControllerTaskStackWords = 100;
	static constexpr uint32_t DefaultSampleInterval = 200;

	Task<HeightControllerTaskStackWords> *heightControllerTask;
	size_t sensorNumber;							// which sensor, normally a virtual heater
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

#endif /* SRC_MOVEMENT_HEIGHTCONTROL_HEIGHTCONTROLLER_H_ */
