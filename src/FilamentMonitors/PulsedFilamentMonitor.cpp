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

PulsedFilamentMonitor::PulsedFilamentMonitor(unsigned int extruder, int type)
	: FilamentMonitor(extruder, type),
	  mmPerPulse(DefaultMmPerPulse), tolerance(DefaultTolerance), minimumExtrusionCheckLength(DefaultMinimumExtrusionCheckLength)
{
	Init();
}

void PulsedFilamentMonitor::Init()
{
	sensorValue = 0;
	calibrationStarted = dataReceived = false;
	Reset();
}

void PulsedFilamentMonitor::Reset()
{
	extrusionCommanded = movementMeasured = extrusionCommandedAtLastMeasurement = movementMeasuredAtLastCheck = 0.0;
	samplesReceived = 0;
	comparisonStarted = false;
}

// Configure this sensor, returning true if error and setting 'seen' if we processed any configuration parameters
bool PulsedFilamentMonitor::Configure(GCodeBuffer& gb, StringRef& reply, bool& seen)
{
	if (ConfigurePin(gb, reply, RISING, seen))
	{
		return true;
	}

	gb.TryGetFValue('S', mmPerPulse, seen);								// this is mm per rev for Duet3D sensors, mm per pulse for pulsed sensors
	gb.TryGetFValue('E', minimumExtrusionCheckLength, seen);

	if (gb.Seen('R'))
	{
		seen = true;
		tolerance = gb.GetFValue() * 0.01;
	}

	if (seen)
	{
		Init();
	}
	else
	{
		reply.printf("Pulse-type filament monitor on endstop input %u, sensitivity %.2fmm/pulse, check every %.1fmm, tolerance %.1f%%, ",
						GetEndstopNumber(),
						(double)mmPerPulse,
						(double)minimumExtrusionCheckLength,
						(double)(tolerance * 100.0));

		if (!dataReceived)
		{
			reply.cat("no data received");
		}
		else
		{
			reply.catf("current position %.1f, ", (double)GetCurrentPosition());
			if (calibrationStarted && fabsf(totalMovementMeasured) > 1.0 && totalExtrusionCommanded > 20.0)
			{
				const float measuredCalibration = totalExtrusionCommanded/totalMovementMeasured;
				const float normalRatio = 1.0/measuredCalibration;
				const int measuredPosTolerance = lrintf(100.0 * (((normalRatio > 0.0) ? maxMovementRatio : minMovementRatio) - normalRatio)/normalRatio);
				const int measuredNegTolerance = lrintf(100.0 * (normalRatio - ((normalRatio > 0.0) ? minMovementRatio : maxMovementRatio))/normalRatio);
				reply.catf("measured sensitivity %.2fmm/pulse over %.1fmm, tolerance +%d%% -%d%%",
					(double)measuredCalibration,
					(double)totalExtrusionCommanded,
					measuredPosTolerance,
					measuredNegTolerance);
			}
			else
			{
				reply.cat("no calibration data");
			}
		}
	}

	return false;
}

// ISR for when the pin state changes
bool PulsedFilamentMonitor::Interrupt()
{
	++sensorValue;
	extrusionCommandedAtLastMeasurement = extrusionCommanded;
	if (samplesReceived < 100)
	{
		++samplesReceived;
	}
	return true;
}

// Call the following regularly to keep the status up to date
void PulsedFilamentMonitor::Poll()
{
	cpu_irq_disable();
	const uint16_t locSensorVal = sensorValue;
	sensorValue = 0;
	cpu_irq_enable();
	movementMeasured += (float)locSensorVal;
	extrusionCommandedAtLastMeasurement = extrusionCommanded;
}

// Return the current wheel angle
float PulsedFilamentMonitor::GetCurrentPosition() const
{
	return (float)sensorValue;
}

// Call the following at intervals to check the status. This is only called when extrusion is in progress or imminent.
// 'filamentConsumed' is the net amount of extrusion since the last call to this function.
// 'hadNonPrintingMove' is called if filamentConsumed includes extruder movement form non-printing moves.
FilamentSensorStatus PulsedFilamentMonitor::Check(bool full, bool hadNonPrintingMove, bool fromIsr, float filamentConsumed)
{
	Poll();														// this may update movementMeasured

	FilamentSensorStatus ret = FilamentSensorStatus::ok;
	if (!hadNonPrintingMove)
	{
		// We have had a non-printing move recently and we are configured to not check non-printing moves. Reset the counters.
		movementMeasured = movementMeasuredAtLastCheck;			// ignore measured extrusion since last check
	}
	else
	{
		extrusionCommanded += filamentConsumed;					// include the extrusion we have just been told about
		movementMeasuredAtLastCheck = movementMeasured;			// save for next time

		if (full)
		{
			if (extrusionCommandedAtLastMeasurement >= minimumExtrusionCheckLength)
			{
				ret = CheckFilament(extrusionCommandedAtLastMeasurement, false);
			}
			else if (extrusionCommanded >= minimumExtrusionCheckLength * 1.1 && millis() - lastMeasurementTime > 110)
			{
				ret = CheckFilament(extrusionCommanded, true);
			}
		}
	}

	return ret;
}

// Compare the amount commanded with the amount of extrusion measured, and set up for the next comparison
FilamentSensorStatus PulsedFilamentMonitor::CheckFilament(float amountCommanded, bool overdue)
{
	const float extrusionMeasured = movementMeasured * mmPerPulse;
	if (reprap.Debug(moduleFilamentSensors))
	{
		debugPrintf("Extr req %.3f meas %.3f rem %.3f %s\n", (double)amountCommanded, (double)extrusionMeasured, (double)(extrusionCommanded - amountCommanded),
			(overdue) ? " overdue" : "");
	}

	FilamentSensorStatus ret = FilamentSensorStatus::ok;
	if (!comparisonStarted)
	{
		// The first measurement after we start extruding is often a long way out, so discard it
		comparisonStarted = true;
		calibrationStarted = false;
	}
	else if (tolerance >= 0.0)
	{
		const float minExtrusionExpected = (amountCommanded >= 0.0)
											 ? amountCommanded * (1.0 - tolerance)
												: amountCommanded * (1.0 + tolerance);
		if (extrusionMeasured < minExtrusionExpected)
		{
			ret = FilamentSensorStatus::tooLittleMovement;
		}
		else
		{
			const float maxExtrusionExpected = (amountCommanded >= 0.0)
												 ? amountCommanded * (1.0 + tolerance)
													: amountCommanded * (1.0 - tolerance);
			if (extrusionMeasured > maxExtrusionExpected)
			{
				ret = FilamentSensorStatus::tooMuchMovement;
			}
		}
	}

	// Update the calibration accumulators, even if the user hasn't asked to do calibration
	const float ratio = movementMeasured/amountCommanded;
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
		totalMovementMeasured += movementMeasured;
	}
	else
	{
		minMovementRatio = maxMovementRatio = ratio;
		totalExtrusionCommanded = amountCommanded;
		totalMovementMeasured = movementMeasured;
		calibrationStarted = true;
	}

	extrusionCommanded -= amountCommanded;
	extrusionCommandedAtLastMeasurement = movementMeasured = 0.0;

	return ret;
}

// Clear the measurement state - called when we are not printing a file. Return the present/not present status if available.
FilamentSensorStatus PulsedFilamentMonitor::Clear(bool full)
{
	Poll();								// to keep the diagnostics up to date
	Reset();

	return FilamentSensorStatus::ok;
}

// Print diagnostic info for this sensor
void PulsedFilamentMonitor::Diagnostics(MessageType mtype, unsigned int extruder)
{
	Poll();
	const char* const statusText = (!dataReceived) ? "no data received" : "ok";
	reprap.GetPlatform().MessageF(mtype, "Extruder %u sensor: position %.2f, %s, ", extruder, (double)GetCurrentPosition(), statusText);
	if (calibrationStarted && fabsf(totalMovementMeasured) > 1.0 && totalExtrusionCommanded > 20.0)
	{
		const float measuredMmPerRev = totalExtrusionCommanded/totalMovementMeasured;
		const float normalRatio = 1.0/measuredMmPerRev;
		const int measuredPosTolerance = lrintf(100.0 * (((normalRatio > 0.0) ? maxMovementRatio : minMovementRatio) - normalRatio)/normalRatio);
		const int measuredNegTolerance = lrintf(100.0 * (normalRatio - ((normalRatio > 0.0) ? minMovementRatio : maxMovementRatio))/normalRatio);
		reprap.GetPlatform().MessageF(mtype,"measured sensitivity %.2fmm/pulse +%d%% -%d%%\n",
										(double)measuredMmPerRev, measuredPosTolerance, measuredNegTolerance);
	}
	else
	{
		reprap.GetPlatform().Message(mtype, "no calibration data\n");
	}
}

// End
