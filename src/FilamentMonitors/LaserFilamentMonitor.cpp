/*
 * LaserFilamentMonitor.cpp
 *
 *  Created on: 9 Jan 2018
 *      Author: David
 */

#include "LaserFilamentMonitor.h"
#include "GCodes/GCodeBuffer.h"
#include "Platform.h"
#include "RepRap.h"

LaserFilamentMonitor::LaserFilamentMonitor(unsigned int extruder, int type)
	: Duet3DFilamentMonitor(extruder, type),
	  minMovementAllowed(DefaultMinMovementAllowed), maxMovementAllowed(DefaultMaxMovementAllowed),
	  minimumExtrusionCheckLength(DefaultMinimumExtrusionCheckLength), comparisonEnabled(false)
{
	switchOpenMask = (type == 6) ? TypeLaserSwitchOpenMask : 0;
	Init();
}

void LaserFilamentMonitor::Init()
{
	sensorValue = 0;
	parityErrorCount = framingErrorCount = 0;
	lastMeasurementTime = 0;
	backwards = false;
	InitReceiveBuffer();
	Reset();
}

void LaserFilamentMonitor::Reset()
{
	extrusionCommandedThisSegment = extrusionCommandedSinceLastSync = movementMeasuredThisSegment = movementMeasuredSinceLastSync = 0.0;
	laserMonitorState = LaserMonitorState::idle;
	haveStartBitData = false;
	hadNonPrintingMoveSinceLastSync = true;			// force a resync
}

// Configure this sensor, returning true if error and setting 'seen' if we processed any configuration parameters
bool LaserFilamentMonitor::Configure(GCodeBuffer& gb, const StringRef& reply, bool& seen)
{
	if (ConfigurePin(gb, reply, INTERRUPT_MODE_CHANGE, seen))
	{
		return true;
	}

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
		reply.printf("Duet3D laser filament monitor%s on endstop input %u, %s, allowed movement %ld%% to %ld%%, check every %.1fmm, ",
						(switchOpenMask != 0) ? " with microswitch" : "",
						GetEndstopNumber(),
						(comparisonEnabled) ? "enabled" : "disabled",
						lrintf(minMovementAllowed * 100.0),
						lrintf(maxMovementAllowed * 100.0),
						(double)minimumExtrusionCheckLength);

		if (!dataReceived)
		{
			reply.cat("no data received");
		}
		else if ((sensorValue & TypeLaserErrorMask) != 0)
		{
			reply.cat("error");
		}
		else
		{
			reply.catf("current position %.1f, brightness %u, shutter %u, ", (double)GetCurrentPosition(), qualityWord & 0x00FF, (qualityWord >> 8) & 0x3F);
			if (laserMonitorState != LaserMonitorState::calibrating && totalExtrusionCommanded > 10.0)
			{
				reply.catf("measured minimum %ld%%, average %ld%%, maximum %ld%% over %.1fmm",
					lrintf(100 * minMovementRatio),
					lrintf((100 * totalMovementMeasured)/totalExtrusionCommanded),
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

// Return the current position
float LaserFilamentMonitor::GetCurrentPosition() const
{
	int32_t pos = (int32_t)(sensorValue & TypeLaserPositionMask);
	if (pos > 512)
	{
		pos -= 1024;
	}
	return (float)pos * 0.02;									// each count is nominally 0.02mm of filament motion @ 5 * 254 cpi
}

// Deal with any received data
void LaserFilamentMonitor::HandleIncomingData()
{
	uint16_t val;
	PollResult res;
	while ((res = PollReceiveBuffer(val)) != PollResult::incomplete)
	{
		// We have either received a report or there has been a framing error
		bool receivedPositionReport = false;
		if (res == PollResult::complete)
		{
			// Check the parity
			uint8_t data8 = (uint8_t)((val >> 8) ^ val);
			data8 ^= (data8 >> 4);
			data8 ^= (data8 >> 2);
			data8 ^= (data8 >> 1);
			if ((data8 & 1) != 0)
			{
				++parityErrorCount;
			}
			else if ((val & TypeLaserQualityMask) != 0)
			{
				qualityWord = val & ~(TypeLaserQualityMask | TypeLaserParityMask);		// record the quality report
			}
			else
			{
				receivedPositionReport = true;
				dataReceived = true;
			}
		}
		else
		{
			++framingErrorCount;
		}

		if (receivedPositionReport)
		{
			// We have a completed a position report
			lastMeasurementTime = millis();
			const uint16_t positionChange = (val - sensorValue) & TypeLaserPositionMask;	// position change in range 0..1023
			const int32_t movement = (positionChange <= 512) ? (int32_t)positionChange : (int32_t)positionChange - 1024;
			movementMeasuredSinceLastSync += (float)movement * 0.02;
			sensorValue = val;

			if (haveStartBitData)					// if we have a synchronised  value for the amount of extrusion commanded
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
			// A receive error occurred, or we received a quality report not a position report. Any start bit data we stored is wrong.
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

// Call the following at intervals to check the status. This is only called when extrusion is in progress or imminent.
// 'filamentConsumed' is the net amount of extrusion since the last call to this function.
// 'hadNonPrintingMove' is true if filamentConsumed includes extruder movement from non-printing moves.
// 'fromIsr' is true if this measurement was taken at the end of the ISR because a potential start bit was seen
FilamentSensorStatus LaserFilamentMonitor::Check(bool full, bool hadNonPrintingMove, bool fromIsr, float filamentConsumed)
{
	// 1. Update the extrusion commanded and whether we have had an extruding but non-printing move
	if (hadNonPrintingMove)
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
		if ((sensorValue & TypeLaserErrorMask) != 0)
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
FilamentSensorStatus LaserFilamentMonitor::CheckFilament(float amountCommanded, float amountMeasured, bool overdue)
{
	if (reprap.Debug(moduleFilamentSensors))
	{
		debugPrintf("Extr req %.3f meas %.3f%s\n", (double)amountCommanded, (double)amountMeasured, (overdue) ? " overdue" : "");
	}

	FilamentSensorStatus ret = FilamentSensorStatus::ok;

	switch (laserMonitorState)
	{
	case LaserMonitorState::idle:
		laserMonitorState = LaserMonitorState::calibrating;
		totalExtrusionCommanded = amountCommanded;
		totalMovementMeasured = amountMeasured;
		break;

	case LaserMonitorState::calibrating:
		totalExtrusionCommanded += amountCommanded;
		totalMovementMeasured += amountMeasured;
		if (totalExtrusionCommanded >= 10.0)
		{
			backwards = (totalMovementMeasured < 0.0);
			if (backwards)
			{
				totalMovementMeasured = -totalMovementMeasured;
			}
			minMovementRatio = maxMovementRatio = totalMovementMeasured/totalExtrusionCommanded;
			if (comparisonEnabled)
			{
				if (minMovementRatio < minMovementAllowed)
				{
					ret = FilamentSensorStatus::tooLittleMovement;
				}
				else if (maxMovementRatio > maxMovementAllowed)
				{
					ret = FilamentSensorStatus::tooMuchMovement;
				}
			}
			laserMonitorState = LaserMonitorState::comparing;
		}
		break;

	case LaserMonitorState::comparing:
		{
			totalExtrusionCommanded += amountCommanded;
			if (backwards)
			{
				amountMeasured = -amountMeasured;
			}
			totalMovementMeasured += amountMeasured;
			const float ratio = amountMeasured/amountCommanded;
			if (ratio > maxMovementRatio)
			{
				maxMovementRatio = ratio;
			}
			else if (ratio < minMovementRatio)
			{
				minMovementRatio = ratio;
			}
			if (comparisonEnabled)
			{
				if (ratio < minMovementAllowed)
				{
					ret = FilamentSensorStatus::tooLittleMovement;
				}
				else if (ratio > maxMovementAllowed)
				{
					ret = FilamentSensorStatus::tooMuchMovement;
				}
			}
		}
		break;
	}

	return ret;
}

// Clear the measurement state - called when we are not printing a file. Return the present/not present status if available.
FilamentSensorStatus LaserFilamentMonitor::Clear(bool full)
{
	HandleIncomingData();
	Reset();

	FilamentSensorStatus ret = FilamentSensorStatus::ok;
	if (full)
	{
		if ((sensorValue & TypeLaserErrorMask) != 0)
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
void LaserFilamentMonitor::Diagnostics(MessageType mtype, unsigned int extruder)
{
	const char* const statusText = (!dataReceived) ? "no data received"
									: ((sensorValue & TypeLaserErrorMask) != 0) ? "error"
										: ((sensorValue & switchOpenMask) != 0) ? "no filament"
											: "ok";
	reprap.GetPlatform().MessageF(mtype, "Extruder %u sensor: position %.2f, %s, ", extruder, (double)GetCurrentPosition(), statusText);
	if (dataReceived)
	{
		reprap.GetPlatform().MessageF(mtype, "framing errors %" PRIu32 ", parity errors %" PRIu32 ", ", framingErrorCount, parityErrorCount);
	}
	if (laserMonitorState != LaserMonitorState::calibrating && totalExtrusionCommanded > 10.0)
	{
		reprap.GetPlatform().MessageF(mtype, "measured minimum %ld%%, average %ld%%, maximum %ld%% over %.1fmm\n",
			lrintf(100 * minMovementRatio),
			lrintf((100 * totalMovementMeasured)/totalExtrusionCommanded),
			lrintf(100 * maxMovementRatio),
			(double)totalExtrusionCommanded);
	}
	else
	{
		reprap.GetPlatform().Message(mtype, "no calibration data\n");
	}
}

// End
