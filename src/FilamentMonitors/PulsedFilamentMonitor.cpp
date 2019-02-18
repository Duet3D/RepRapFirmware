/*
 * PulsedFilamentSensor.cpp
 *
 *  Created on: 9 Jan 2018
 *      Author: David
 */

#include "PulsedFilamentMonitor.h"
#include "GCodes/GCodeBuffer.h"
#include "Platform.h"
#include "RepRap.h"
#include "Movement/Move.h"

// Unless we set the option to compare filament on all type of move, we reject readings if the last retract or reprime move wasn't completed
// well before the start bit was received. This is because those moves have high accelerations and decelerations, so the measurement delay
// is more likely to cause errors. This constant sets the delay required after a retract or reprime move before we accept the measurement.
const int32_t SyncDelayMillis = 10;

PulsedFilamentMonitor::PulsedFilamentMonitor(unsigned int extruder, int type)
	: FilamentMonitor(extruder, type),
	  mmPerPulse(DefaultMmPerPulse),
	  minMovementAllowed(DefaultMinMovementAllowed), maxMovementAllowed(DefaultMaxMovementAllowed),
	  minimumExtrusionCheckLength(DefaultMinimumExtrusionCheckLength), comparisonEnabled(false)
{
	Init();
}

void PulsedFilamentMonitor::Init()
{
	sensorValue = 0;
	calibrationStarted = false;
	samplesReceived = 0;
	lastMeasurementTime = 0;
	Reset();
}

void PulsedFilamentMonitor::Reset()
{
	extrusionCommandedThisSegment = extrusionCommandedSinceLastSync = movementMeasuredThisSegment = movementMeasuredSinceLastSync = 0.0;
	comparisonStarted = false;
	haveInterruptData = false;
	wasPrintingAtInterrupt = false;			// force a resync
}

// Configure this sensor, returning true if error and setting 'seen' if we processed any configuration parameters
bool PulsedFilamentMonitor::Configure(GCodeBuffer& gb, const StringRef& reply, bool& seen)
{
	if (ConfigurePin(gb, reply, INTERRUPT_MODE_RISING, seen))
	{
		return true;
	}

	gb.TryGetFValue('L', mmPerPulse, seen);
	gb.TryGetFValue('E', minimumExtrusionCheckLength, seen);

	if (gb.Seen('R'))
	{
		seen = true;
		size_t numValues = 2;
		uint32_t minMax[2];
		gb.GetUnsignedArray(minMax, numValues, false);
		if (numValues > 0)
		{
			minMovementAllowed = (float)minMax[0] * 0.01;
		}
		if (numValues > 1)
		{
			maxMovementAllowed = (float)minMax[1] * 0.01;
		}
	}

	if (gb.Seen('S'))
	{
		seen = true;
		comparisonEnabled = (gb.GetIValue() > 0);
	}

	if (seen)
	{
		Init();
	}
	else
	{
		reply.printf("Pulse-type filament monitor on endstop input %u, %s, sensitivity %.2fmm/pulse, allowed movement %ld%% to %ld%%, check every %.1fmm, ",
						GetEndstopNumber(),
						(comparisonEnabled) ? "enabled" : "disabled",
						(double)mmPerPulse,
						lrintf(minMovementAllowed * 100.0),
						lrintf(maxMovementAllowed * 100.0),
						(double)minimumExtrusionCheckLength);

		if (samplesReceived < 2)
		{
			reply.cat("no data received");
		}
		else
		{
			reply.catf("current position %.1f, ", (double)GetCurrentPosition());
			if (calibrationStarted && fabsf(totalMovementMeasured) > 1.0 && totalExtrusionCommanded > 20.0)
			{
				const float measuredMmPerPulse = totalExtrusionCommanded/totalMovementMeasured;
				reply.catf("measured sensitivity %.3fmm/pulse, measured minimum %ld%%, maximum %ld%% over %.1fmm\n",
					(double)measuredMmPerPulse,
					lrintf(100 * minMovementRatio),
					lrintf(100 * maxMovementRatio),
					(double)totalExtrusionCommanded);
			}
			else
			{
				reply.cat("no calibration data");
			}
		}
	}

	return false;
}

// ISR for when the pin state changes. It should return true if the ISR wants the commanded extrusion to be fetched.
bool PulsedFilamentMonitor::Interrupt()
{
	++sensorValue;
	if (samplesReceived < 100)
	{
		++samplesReceived;
	}
	lastMeasurementTime = millis();
	return true;
}

// Call the following regularly to keep the status up to date
void PulsedFilamentMonitor::Poll()
{
	cpu_irq_disable();
	const uint32_t locSensorVal = sensorValue;
	sensorValue = 0;
	cpu_irq_enable();
	movementMeasuredSinceLastSync += (float)locSensorVal;

	if (haveInterruptData)					// if we have a synchronised value for the amount of extrusion commanded
	{
		if (wasPrintingAtInterrupt && (int32_t)(lastSyncTime - reprap.GetMove().ExtruderPrintingSince()) > SyncDelayMillis)
		{
			// We can use this measurement
			extrusionCommandedThisSegment += extrusionCommandedAtInterrupt;
			movementMeasuredThisSegment += movementMeasuredSinceLastSync;
		}
		lastSyncTime = lastIsrTime;
		extrusionCommandedSinceLastSync -= extrusionCommandedAtInterrupt;
		movementMeasuredSinceLastSync = 0.0;

		haveInterruptData = false;
	}
}

// Return the current wheel angle
float PulsedFilamentMonitor::GetCurrentPosition() const
{
	return (float)sensorValue;
}

// Call the following at intervals to check the status. This is only called when extrusion is in progress or imminent.
// 'filamentConsumed' is the net amount of extrusion since the last call to this function.
// 'isPrinting' is true unless a non-printing extruder move was in progress
// 'fromIsr' is true if this measurement was taken at the end of the ISR because the ISR returned true
FilamentSensorStatus PulsedFilamentMonitor::Check(bool isPrinting, bool fromIsr, uint32_t isrMillis, float filamentConsumed)
{
	// 1. Update the extrusion commanded
	extrusionCommandedSinceLastSync += filamentConsumed;

	// 2. If this call passes values synced to the interrupt, save the data
	if (fromIsr)
	{
		extrusionCommandedAtInterrupt = extrusionCommandedSinceLastSync;
		wasPrintingAtInterrupt = isPrinting;
		lastIsrTime = isrMillis;
		haveInterruptData = true;
	}

	// 3. Process the received data and update if we have received anything
	Poll();														// this may update movementMeasured

	// 4. Decide whether it is time to do a comparison, and return the status
	FilamentSensorStatus ret = FilamentSensorStatus::ok;
	if (extrusionCommandedThisSegment >= minimumExtrusionCheckLength)
	{
		ret = CheckFilament(extrusionCommandedThisSegment, movementMeasuredThisSegment, false);
		extrusionCommandedThisSegment = movementMeasuredThisSegment = 0.0;
	}
	else if (extrusionCommandedThisSegment + extrusionCommandedSinceLastSync >= minimumExtrusionCheckLength * 2 && millis() - lastMeasurementTime > 220)
	{
		// A sync is overdue
		ret = CheckFilament(extrusionCommandedThisSegment + extrusionCommandedSinceLastSync, movementMeasuredThisSegment + movementMeasuredSinceLastSync, true);
		extrusionCommandedThisSegment = extrusionCommandedSinceLastSync = movementMeasuredThisSegment = movementMeasuredSinceLastSync = 0.0;
	}

	return ret;
}

// Compare the amount commanded with the amount of extrusion measured, and set up for the next comparison
FilamentSensorStatus PulsedFilamentMonitor::CheckFilament(float amountCommanded, float amountMeasured, bool overdue)
{
	if (reprap.Debug(moduleFilamentSensors))
	{
		debugPrintf("Extr req %.3f meas %.3f%s\n", (double)amountCommanded, (double)amountMeasured, (overdue) ? " overdue" : "");
	}

	FilamentSensorStatus ret = FilamentSensorStatus::ok;
	const float extrusionMeasured = amountMeasured * mmPerPulse;

	if (!comparisonStarted)
	{
		// The first measurement after we start extruding is often a long way out, so discard it
		comparisonStarted = true;
		calibrationStarted = false;
	}
	else if (comparisonEnabled)
	{
		const float minExtrusionExpected = (amountCommanded >= 0.0)
											 ? amountCommanded * minMovementAllowed
												: amountCommanded * maxMovementAllowed;
		if (extrusionMeasured < minExtrusionExpected)
		{
			ret = FilamentSensorStatus::tooLittleMovement;
		}
		else
		{
			const float maxExtrusionExpected = (amountCommanded >= 0.0)
												 ? amountCommanded * maxMovementAllowed
													: amountCommanded * minMovementAllowed;
			if (extrusionMeasured > maxExtrusionExpected)
			{
				ret = FilamentSensorStatus::tooMuchMovement;
			}
		}
	}

	// Update the calibration accumulators, even if the user hasn't asked to do calibration
	const float ratio = extrusionMeasured/amountCommanded;
	if (calibrationStarted)
	{
		if (ratio < minMovementRatio)
		{
			minMovementRatio = ratio;
		}
		if (ratio > maxMovementRatio)
		{
			maxMovementRatio = ratio;
		}
		totalExtrusionCommanded += amountCommanded;
		totalMovementMeasured += amountMeasured;
	}
	else
	{
		minMovementRatio = maxMovementRatio = ratio;
		totalExtrusionCommanded = amountCommanded;
		totalMovementMeasured = amountMeasured;
		calibrationStarted = true;
	}

	return ret;
}

// Clear the measurement state - called when we are not printing a file. Return the present/not present status if available.
FilamentSensorStatus PulsedFilamentMonitor::Clear()
{
	Poll();								// to keep the diagnostics up to date
	Reset();
	return FilamentSensorStatus::ok;
}

// Print diagnostic info for this sensor
void PulsedFilamentMonitor::Diagnostics(MessageType mtype, unsigned int extruder)
{
	Poll();
	const char* const statusText = (samplesReceived < 2) ? "no data received" : "ok";
	reprap.GetPlatform().MessageF(mtype, "Extruder %u sensor: position %.2f, %s, ", extruder, (double)GetCurrentPosition(), statusText);
	if (calibrationStarted && fabsf(totalMovementMeasured) > 1.0 && totalExtrusionCommanded > 20.0)
	{
		const float measuredMmPerRev = totalExtrusionCommanded/totalMovementMeasured;
		const float normalRatio = 1.0/measuredMmPerRev;
		const int measuredPosTolerance = lrintf(100.0 * (((normalRatio > 0.0) ? maxMovementRatio : minMovementRatio) - normalRatio)/normalRatio);
		const int measuredNegTolerance = lrintf(100.0 * (normalRatio - ((normalRatio > 0.0) ? minMovementRatio : maxMovementRatio))/normalRatio);
		reprap.GetPlatform().MessageF(mtype,"measured sensitivity %.3fmm/pulse +%d%% -%d%%\n",
										(double)measuredMmPerRev, measuredPosTolerance, measuredNegTolerance);
	}
	else
	{
		reprap.GetPlatform().Message(mtype, "no calibration data\n");
	}
}

// End
