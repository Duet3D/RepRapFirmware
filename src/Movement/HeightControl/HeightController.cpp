/*
 * HeightController.cpp
 *
 *  Created on: 18 Apr 2019
 *      Author: David
 */

#include "HeightController.h"

#if SUPPORT_ASYNC_MOVES

#include "RepRap.h"
#include "Platform.h"
#include "GCodes/GCodeBuffer/GCodeBuffer.h"
#include "Heating/Heat.h"
#include "Heating/Sensors/TemperatureSensor.h"
#include "Movement/Move.h"
#include <TaskPriorities.h>

HeightController::HeightController() noexcept
	: heightControllerTask(nullptr), sensorNumber(-1),
		sampleInterval(DefaultSampleInterval), setPoint(1.0), pidP(1.0), configuredPidI(0.0), configuredPidD(0.0), iAccumulator(0.0),
		zMin(5.0), zMax(10.0), state(PidState::stopped)
{
	CalcDerivedValues();
}

extern "C" [[noreturn]] void HeightControllerTaskStart(void *p) noexcept
{
	static_cast<HeightController*>(p)->RunTask();
}

GCodeResult HeightController::Configure(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	bool seen = false;
	uint32_t sn;
	gb.TryGetUIValue('H', sn, seen);
	if (seen)
	{
		sensorNumber = (int)sn;
	}
	gb.TryGetFValue('P', pidP, seen);
	gb.TryGetFValue('I', configuredPidI, seen);
	gb.TryGetFValue('D', configuredPidD, seen);
	if (gb.Seen('F'))
	{
		const float freq = gb.GetFValue();
		if (freq >= 0.1 && freq <= 200.0)
		{
			sampleInterval = lrintf(1000/freq);
		}
	}

	float zLimits[2];
	bool seenZ = false;
	if (gb.TryGetFloatArray('Z', 2, zLimits, reply, seenZ, false))
	{
		return GCodeResult::error;
	}
	if (seenZ && zLimits[0] < zLimits[1])
	{
		zMin = zLimits[0];
		zMax = zLimits[1];
	}

	if (seen || seenZ)
	{
		CalcDerivedValues();

		TaskCriticalSectionLocker lock;			// make sure we don't create the task more than once

		if (heightControllerTask == nullptr && sensorNumber >= 0)
		{
			state = PidState::stopped;
			heightControllerTask = new Task<HeightControllerTaskStackWords>;
			heightControllerTask->Create(HeightControllerTaskStart, "HEIGHT", (void*)this, TaskPriority::HeightFollowingPriority);
		}
	}
	else if (sensorNumber < 0)
	{
		reply.copy("Height controller is not configured");
	}
	else
	{
		reply.printf("Height controller uses sensor %u, frequency %.1f, P%.1f I%.1f D%.1f, Z%.1f to %.1f",
						sensorNumber, (double)(1000.0/(float)sampleInterval), (double)pidP, (double)configuredPidI, (double)configuredPidD, (double)zMin, (double)zMax);
	}
	return GCodeResult::ok;
}

// Start/stop height following
GCodeResult HeightController::StartHeightFollowing(GCodeBuffer& gb, const StringRef& reply) noexcept
{
	if (gb.Seen('P'))
	{
		if (gb.GetIValue() == 1)
		{
			// Start height following
			if (sensorNumber < 0 || heightControllerTask == nullptr)
			{
				reply.copy("Height controller is not configured");
				return GCodeResult::error;
			}

			bool dummy;
			gb.TryGetFValue('S', setPoint, dummy);

			float zLimits[2];
			bool seenZ = false;
			if (gb.TryGetFloatArray('Z', 2, zLimits, reply, seenZ, false))
			{
				return GCodeResult::error;
			}
			if (seenZ && zLimits[0] < zLimits[1])
			{
				zMin = zLimits[0];
				zMax = zLimits[1];
			}

			if (state == PidState::stopped)
			{
				state = PidState::starting;
				heightControllerTask->Give();
			}
		}
		else
		{
			// Stop height following
			Stop();
		}
	}
	else
	{
		reply.printf("Height following mode is %sactive", (state == PidState::stopped) ? "in" : "");
	}
	return GCodeResult::ok;
}

// Stop height following mode
void HeightController::Stop() noexcept
{
	state = PidState::stopped;
}

[[noreturn]] void HeightController::RunTask() noexcept
{
	lastWakeTime = xTaskGetTickCount();
	for (;;)
	{
		if (state == PidState::stopped)
		{
			(void)TaskBase::Take();

			// Here when we get woken up again, normally because the state has been changed to 'starting'. So get ready to start.
			lastWakeTime = xTaskGetTickCount();
			lastReadingOk = false;
			float machinePos[MaxAxes];
			reprap.GetMove().GetCurrentMachinePosition(machinePos, false);
			currentZ = machinePos[Z_AXIS];
			iAccumulator = constrain<float>(currentZ, zMin, zMax);
		}
		else if (sensorNumber < 0)
		{
			state = PidState::stopped;
		}
		else
		{
			TemperatureError err;
			const float sensorVal = reprap.GetHeat().GetSensorTemperature(sensorNumber, err);
			if (err == TemperatureError::success)
			{
				AsyncMove * const move = reprap.GetMove().LockAuxMove();
				if (move != nullptr)
				{
					// Calculate the new target Z height using the PID algorithm
					const float difference = setPoint - sensorVal;
					iAccumulator = constrain<float>(iAccumulator + difference * actualPidI, zMin, zMax);
					float newZ = currentZ + pidP * difference + iAccumulator;
					if (lastReadingOk)
					{
						newZ += actualPidD * (lastReading - sensorVal);
					}


					// Constrain the target Z height to be within the limits
					newZ = constrain<float>(newZ, zMin, zMax);

					// Calculate how far we need to move Z and constrain it to the maximum permissible Z movement per sample interval.
					// During the startup phase, Z may be initial outside the limits, so the resulting Z may be outside the limits too.
					// Note, if the initial Z is well outside the limits, moving it to be within the limits in several small steps like this is non-optimal.
					// It is faster to move Z to be within the limits before engaging height following mode.
					const float adjustment = constrain<float>(newZ - currentZ, -maxZAdjustmentPerSample, maxZAdjustmentPerSample);
					currentZ += adjustment;

					// Schedule an async move to adjust Z
					move->SetDefaults();
					move->movements[Z_AXIS] = adjustment;
					move->startSpeed = move->endSpeed = startSpeed;
					move->requestedSpeed = reprap.GetPlatform().MaxFeedrate(Z_AXIS);
					move->acceleration = move->deceleration = acceleration;
					reprap.GetMove().ReleaseAuxMove(true);
				}

				lastReading = sensorVal;
				lastReadingOk = true;
			}
			else
			{
				lastReadingOk = false;
			}
			vTaskDelayUntil(&lastWakeTime, sampleInterval);
		}
	}
}

void HeightController::CalcDerivedValues() noexcept
{
	actualPidI = configuredPidI * ((float)sampleInterval * MillisToSeconds);
	actualPidD = configuredPidD * (SecondsToMillis/(float)sampleInterval);

	// Calculate the maximum Z adjustment per sample interval.
	// We always start and end at half the Z jerk speed so that back-to-back Z movements are always possible.
	startSpeed = reprap.GetPlatform().GetInstantDv(Z_AXIS) * 0.5;
	maxSpeed = reprap.GetPlatform().MaxFeedrate(Z_AXIS);
	acceleration = reprap.GetPlatform().Acceleration(Z_AXIS);
	const float interval = sampleInterval * MillisToSeconds;
	if (startSpeed + acceleration * interval * 0.5 < maxSpeed)
	{
		// Acceleration limited case
		// We have s = 2 * ((u * (t/2)) + (0.5 * a *(t/2)^2)) = (u * t) + (0.25 * a * t^2)
		maxZAdjustmentPerSample = startSpeed * interval + acceleration * fsquare(interval) * 0.25;
	}
	else
	{
		// Top-speed limited
		const float accelDecelTime = (maxSpeed - startSpeed)/acceleration;
		maxZAdjustmentPerSample = interval * maxSpeed - accelDecelTime * (maxSpeed - startSpeed);
	}
}

#endif

// End
