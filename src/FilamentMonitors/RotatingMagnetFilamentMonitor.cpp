/*
 * RotatingMagnetFilamentMonitor.cpp
 *
 *  Created on: 9 Jan 2018
 *      Author: David
 */

#include "RotatingMagnetFilamentMonitor.h"
#include "GCodes/GCodeBuffer.h"
#include "Platform.h"
#include "RepRap.h"

RotatingMagnetFilamentMonitor::RotatingMagnetFilamentMonitor(unsigned int extruder, int type)
	: Duet3DFilamentMonitor(extruder, type),
	  mmPerRev(DefaultMmPerRev),
	  minMovementAllowed(DefaultMinMovementAllowed), maxMovementAllowed(DefaultMaxMovementAllowed),
	  minimumExtrusionCheckLength(DefaultMinimumExtrusionCheckLength), comparisonEnabled(false), checkNonPrintingMoves(true)
{
	switchOpenMask = (type == 4) ? TypeMagnetSwitchOpenMask : 0;
	Init();
}

void RotatingMagnetFilamentMonitor::Init()
{
	sensorValue = 0;
	framingErrorCount = 0;
	calibrationStarted = dataReceived = false;
	lastMeasurementTime = 0;
	InitReceiveBuffer();
	Reset();
}

void RotatingMagnetFilamentMonitor::Reset()
{
	extrusionCommandedThisSegment = extrusionCommandedSinceLastSync = movementMeasuredThisSegment = movementMeasuredSinceLastSync = 0.0;
	comparisonStarted = false;
	haveStartBitData = false;
	hadNonPrintingMoveSinceLastSync = true;			// force a resync
}

// Configure this sensor, returning true if error and setting 'seen' if we processed any configuration parameters
bool RotatingMagnetFilamentMonitor::Configure(GCodeBuffer& gb, const StringRef& reply, bool& seen)
{
	if (ConfigurePin(gb, reply, INTERRUPT_MODE_CHANGE, seen))
	{
		return true;
	}

	gb.TryGetFValue('L', mmPerRev, seen);
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
		reply.printf("Duet3D rotating magnet filament monitor%s on endstop input %u, %s, sensitivity %.2fmm/rev, allowed movement %ld%% to %ld%%, check every %.1fmm, ",
						(switchOpenMask != 0) ? " with microswitch" : "",
						GetEndstopNumber(),
						(comparisonEnabled) ? "enabled" : "disabled",
						(double)mmPerRev,
						lrintf(minMovementAllowed * 100.0),
						lrintf(maxMovementAllowed * 100.0),
						(double)minimumExtrusionCheckLength);
		if (!dataReceived)
		{
			reply.cat("no data received");
		}
		else if ((sensorValue & TypeMagnetErrorMask) != 0)
		{
			reply.cat("error");
		}
		else
		{
			reply.catf("current position %.1f, ", (double)GetCurrentPosition());
			if (calibrationStarted && fabsf(totalMovementMeasured) > 1.0 && totalExtrusionCommanded > 20.0)
			{
				const float measuredMmPerRev = totalExtrusionCommanded/totalMovementMeasured;
				reply.catf("measured sensitivity %.2fmm/rev, measured minimum %ld%%, maximum %ld%% over %.1fmm\n",
					(double)measuredMmPerRev,
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

// Deal with any received data
void RotatingMagnetFilamentMonitor::HandleIncomingData()
{
	uint16_t val;
	PollResult res;
	while ((res = PollReceiveBuffer(val)) != PollResult::incomplete)
	{
		// We have either received a report or there has been a receive error
		if (res == PollResult::complete)
		{
			// We have a completed a position report
			dataReceived = true;
			lastMeasurementTime = millis();
			const uint16_t angleChange = (val - sensorValue) & TypeMagnetAngleMask;			// angle change in range 0..1023
			const int32_t movement = (angleChange <= 512) ? (int32_t)angleChange : (int32_t)angleChange - 1024;
			movementMeasuredSinceLastSync += (float)movement/1024;
			sensorValue = val;

			if (haveStartBitData)					// if we have a synchronised value for the amount of extrusion commanded
			{
				if (!hadNonPrintingMoveAtStartBit)
				{
					extrusionCommandedThisSegment += extrusionCommandedAtStartBit;
					movementMeasuredThisSegment += movementMeasuredSinceLastSync;
				}
				movementMeasuredSinceLastSync = 0.0;
			}
		}
		else
		{
			// A receive error occurred. Any start bit data we stored is wrong.
			++framingErrorCount;
			if (haveStartBitData)
			{
				extrusionCommandedSinceLastSync += extrusionCommandedAtStartBit;
				if (hadNonPrintingMoveAtStartBit)
				{
					hadNonPrintingMoveSinceLastSync = true;
				}
			}
		}
		haveStartBitData = false;
	}
}

// Return the current wheel angle
float RotatingMagnetFilamentMonitor::GetCurrentPosition() const
{
	return (sensorValue & TypeMagnetAngleMask) * (360.0/1024.0);
}

// Call the following at intervals to check the status. This is only called when extrusion is in progress or imminent.
// 'filamentConsumed' is the net amount of extrusion commanded since the last call to this function.
// 'hadNonPrintingMove' is true if filamentConsumed includes extruder movement from non-printing moves.
// 'fromIsr' is true if this measurement was taken dat the end of the ISR because a potential start bit was seen
FilamentSensorStatus RotatingMagnetFilamentMonitor::Check(bool full, bool hadNonPrintingMove, bool fromIsr, float filamentConsumed)
{
	// 1. Update the extrusion commanded and whether we have had an extruding but non-printing move
	if (hadNonPrintingMove && !checkNonPrintingMoves)
	{
		hadNonPrintingMoveSinceLastSync = true;
	}
	else
	{
		extrusionCommandedSinceLastSync += filamentConsumed;
	}

	// 2. If this call passes values synced to the start bit, save the data for the next completed measurement.
	if (fromIsr && IsWaitingForStartBit())
	{
		extrusionCommandedAtStartBit = extrusionCommandedSinceLastSync;
		hadNonPrintingMoveAtStartBit = hadNonPrintingMoveSinceLastSync;
		haveStartBitData = true;

		extrusionCommandedSinceLastSync = 0.0;
		hadNonPrintingMoveSinceLastSync = false;
	}

	// 3. Process the receive buffer and update everything if we have received anything or had a receive error
	HandleIncomingData();

	// 4. Decide whether it is time to do a comparison, and return the status
	FilamentSensorStatus ret = FilamentSensorStatus::ok;
	if (full)
	{
		if ((sensorValue & TypeMagnetErrorMask) != 0)
		{
			ret = FilamentSensorStatus::sensorError;
		}
		else if ((sensorValue & switchOpenMask) != 0)
		{
			ret = FilamentSensorStatus::noFilament;
		}
		else if (extrusionCommandedThisSegment >= minimumExtrusionCheckLength)
		{
			ret = CheckFilament(extrusionCommandedThisSegment, movementMeasuredThisSegment, false);
			extrusionCommandedThisSegment = movementMeasuredThisSegment = 0.0;
		}
		else if (extrusionCommandedThisSegment + extrusionCommandedSinceLastSync >= minimumExtrusionCheckLength * 2 && millis() - lastMeasurementTime > 220 && !IsReceiving())
		{
			// A sync is overdue
			ret = CheckFilament(extrusionCommandedThisSegment + extrusionCommandedSinceLastSync, movementMeasuredThisSegment + movementMeasuredSinceLastSync, true);
			extrusionCommandedThisSegment = extrusionCommandedSinceLastSync = movementMeasuredThisSegment = movementMeasuredSinceLastSync = 0.0;
		}
	}

	return ret;
}

// Compare the amount commanded with the amount of extrusion measured, and set up for the next comparison
FilamentSensorStatus RotatingMagnetFilamentMonitor::CheckFilament(float amountCommanded, float amountMeasured, bool overdue)
{
	if (reprap.Debug(moduleFilamentSensors))
	{
		debugPrintf("Extr req %.3f meas %.3f%s\n", (double)amountCommanded, (double)amountMeasured, (overdue) ? " overdue" : "");
	}

	FilamentSensorStatus ret = FilamentSensorStatus::ok;
	const float extrusionMeasured = amountMeasured * mmPerRev;

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
FilamentSensorStatus RotatingMagnetFilamentMonitor::Clear(bool full)
{
	HandleIncomingData();								// to keep the diagnostics up to date
	Reset();

	FilamentSensorStatus ret = FilamentSensorStatus::ok;
	if (full)
	{
		if ((sensorValue & TypeMagnetErrorMask) != 0)
		{
			ret = FilamentSensorStatus::sensorError;
		}
		else if ((sensorValue & switchOpenMask) != 0)
		{
			ret = FilamentSensorStatus::noFilament;
		}
	}
	return ret;
}

// Print diagnostic info for this sensor
void RotatingMagnetFilamentMonitor::Diagnostics(MessageType mtype, unsigned int extruder)
{
	const char* const statusText = (!dataReceived) ? "no data received"
									: ((sensorValue & TypeMagnetErrorMask) != 0) ? "error"
										: ((sensorValue & switchOpenMask) != 0) ? "no filament"
											: "ok";
	reprap.GetPlatform().MessageF(mtype, "Extruder %u sensor: position %.2f, %s, ", extruder, (double)GetCurrentPosition(), statusText);
	if (dataReceived)
	{
		reprap.GetPlatform().MessageF(mtype, "%" PRIu32 " framing errors, ", framingErrorCount);
	}
	if (calibrationStarted && fabsf(totalMovementMeasured) > 1.0 && totalExtrusionCommanded > 20.0)
	{
		const float measuredMmPerRev = totalExtrusionCommanded/totalMovementMeasured;
		reprap.GetPlatform().MessageF(mtype, "measured sensitivity %.2fmm/rev, measured minimum %ld%%, maximum %ld%% over %.1fmm\n",
			(double)measuredMmPerRev,
			lrintf(100 * minMovementRatio),
			lrintf(100 * maxMovementRatio),
			(double)totalExtrusionCommanded);
	}
	else
	{
		reprap.GetPlatform().Message(mtype, "no calibration data\n");
	}
}

// End
