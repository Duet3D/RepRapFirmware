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

LaserFilamentMonitor::LaserFilamentMonitor(int type)
	: Duet3DFilamentMonitor(type),
	  minMovementAllowed(DefaultMinMovementAllowed), maxMovementAllowed(DefaultMaxMovementAllowed),
	  minimumExtrusionCheckLength(DefaultMinimumExtrusionCheckLength), comparisonEnabled(false)
{
	switchOpenMask = (type == 6) ? TypeLaserSwitchOpenMask : 0;
	Init();
}

void LaserFilamentMonitor::Init()
{
	sensorValue = 0;
	parityErrorCount = 0;
	lastMeasurementTime = 0;
	backwards = false;
	InitReceiveBuffer();
	Reset();
}

void LaserFilamentMonitor::Reset()
{
	extrusionCommanded = movementMeasured = extrusionCommandedAtLastMeasurement = extrusionCommandedAtStartBit = movementMeasuredAtLastCheck = 0.0;
	samplesReceived = 0;
	laserMonitorState = LaserMonitorState::idle;
}

// Configure this sensor, returning true if error and setting 'seen' if we processed any configuration parameters
bool LaserFilamentMonitor::Configure(GCodeBuffer& gb, StringRef& reply, bool& seen)
{
	if (ConfigurePin(gb, reply, CHANGE, seen))
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

// This is called from the poll function when we receive what could be a start bit
void LaserFilamentMonitor::OnStartBitReceived() /*override*/
{
	extrusionCommandedAtStartBit = extrusionCommanded;		// record the extrusion
}

// This is called from the poll function when we have received a complete word of data
void LaserFilamentMonitor::ProcessReceivedWord(uint16_t val) /*override*/
{
	// Check the parity
	uint8_t data8 = (uint8_t)((val >> 8) ^ val);
	data8 ^= (data8 >> 4);
	data8 ^= (data8 >> 2);
	data8 ^= (data8 >> 1);
	if ((data8 & 1) != 0)
	{
		++parityErrorCount;
		return;					// parity error, so ignore the data
	}

	// Check whether this is a quality report
	if ((val & TypeLaserQualityMask) != 0)
	{
		qualityWord = val & ~(TypeLaserQualityMask | TypeLaserParityMask);		// record the quality report
		return;
	}

	// We received a position report
	if (samplesReceived == 0)
	{
		dataReceived = true;
		extrusionCommanded -= extrusionCommandedAtStartBit;
		extrusionCommandedAtStartBit = 0;
		movementMeasured = 0.0;		// use the first measurement sample as a baseline
	}
	else
	{
		const uint16_t positionChange = (val - sensorValue) & TypeLaserPositionMask;	// position change in range 0..1023
		const int32_t movement = (positionChange <= 512) ? (int32_t)positionChange : (int32_t)positionChange - 1024;
		movementMeasured += (float)movement * 0.02;
	}

	lastMeasurementTime = millis();
	extrusionCommandedAtLastMeasurement = extrusionCommandedAtStartBit;
	sensorValue = val;
	if (samplesReceived < 100)
	{
		++samplesReceived;
	}
}

// Return the current wheel angle
float LaserFilamentMonitor::GetCurrentPosition() const
{
	int32_t pos = (int32_t)(sensorValue & TypeLaserPositionMask);
	if (pos > 512)
	{
		pos -= 1024;
	}
	return (float)pos * 0.02;											// each count is nominally 0.02mm of filament motion @ 5 * 254 cpi
}

// Call the following at intervals to check the status. This is only called when extrusion is in progress or imminent.
// 'filamentConsumed' is the net amount of extrusion since the last call to this function.
// 'hadNonPrintingMove' is called if filamentConsumed includes extruder movement form non-printing moves.
FilamentSensorStatus LaserFilamentMonitor::Check(bool full, bool hadNonPrintingMove, float filamentConsumed)
{
	PollReceiveBuffer();										// this may update movementMeasured

	FilamentSensorStatus ret = FilamentSensorStatus::ok;
	if (hadNonPrintingMove)
	{
		// We have had a non-printing move recently and we are configured to not check non-printing moves. Reset the counters.
		movementMeasured = movementMeasuredAtLastCheck;			// ignore measured extrusion since last check
	}
	else
	{
		extrusionCommanded += filamentConsumed;					// include the extrusion we have just been told about

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
			else if (samplesReceived >= 10 && !IsReceiving())
			{
				if (extrusionCommandedAtLastMeasurement >= minimumExtrusionCheckLength)
				{
					ret = CheckFilament(extrusionCommandedAtLastMeasurement, movementMeasured, false);
					extrusionCommanded -= extrusionCommandedAtLastMeasurement;
					extrusionCommandedAtLastMeasurement = 0.0;
					movementMeasured = 0.0;
				}
				else if (extrusionCommanded >= minimumExtrusionCheckLength * 1.1 && millis() - lastMeasurementTime > 110)
				{
					ret = CheckFilament(extrusionCommanded, movementMeasured, true);
					extrusionCommanded = 0.0;
					extrusionCommandedAtLastMeasurement = 0.0;
					movementMeasured = 0.0;
				}
			}
		}
		movementMeasuredAtLastCheck = movementMeasured;			// save for next time
	}

	return ret;
}

// Compare the amount commanded with the amount of extrusion measured, and set up for the next comparison
FilamentSensorStatus LaserFilamentMonitor::CheckFilament(float amountCommanded, float amountMeasured, bool overdue)
{
	if (reprap.Debug(moduleFilamentSensors))
	{
		debugPrintf("Extr req %.3f meas %.3f rem %.3f %s\n", (double)amountCommanded, (double)amountMeasured, (double)(extrusionCommanded - amountCommanded),
			(overdue) ? " overdue" : "");
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
				amountMeasured = -amountMeasured;
			}
			minMovementRatio = maxMovementRatio = amountMeasured/amountCommanded;
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
			if (ratio < minMovementRatio)
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
	PollReceiveBuffer();								// to keep the diagnostics up to date
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
	PollReceiveBuffer();
	const char* const statusText = (!dataReceived) ? "no data received"
									: ((sensorValue & TypeLaserErrorMask) != 0) ? "error"
										: ((sensorValue & switchOpenMask) != 0) ? "no filament"
											: "ok";
	reprap.GetPlatform().MessageF(mtype, "Extruder %u sensor: position %.2f, %s, ", extruder, (double)GetCurrentPosition(), statusText);
	if (dataReceived)
	{
		reprap.GetPlatform().MessageF(mtype, "%" PRIu32 " parity errors, ", parityErrorCount);
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
