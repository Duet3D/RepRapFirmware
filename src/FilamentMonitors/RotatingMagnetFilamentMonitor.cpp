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
#include "Movement/Move.h"

// Unless we set the option to compare filament on all type of move, we reject readings if the last retract or reprime move wasn't completed
// well before the start bit was received. This is because those moves have high accelerations and decelerations, so the measurement delay
// is more likely to cause errors. This constant sets the delay required after a retract or reprime move before we accept the measurement.
const int32_t SyncDelayMillis = 10;

RotatingMagnetFilamentMonitor::RotatingMagnetFilamentMonitor(unsigned int extruder, int type)
	: Duet3DFilamentMonitor(extruder, type),
	  mmPerRev(DefaultMmPerRev),
	  minMovementAllowed(DefaultMinMovementAllowed), maxMovementAllowed(DefaultMaxMovementAllowed),
	  minimumExtrusionCheckLength(DefaultMinimumExtrusionCheckLength), comparisonEnabled(false), checkNonPrintingMoves(false)
{
	switchOpenMask = (type == 4) ? TypeMagnetV1SwitchOpenMask : 0;
	Init();
}

void RotatingMagnetFilamentMonitor::Init()
{
	dataReceived = false;
	sensorValue = 0;
	parityErrorCount = framingErrorCount = overrunErrorCount = polarityErrorCount = overdueCount = 0;
	lastMeasurementTime = 0;
	lastErrorCode = 0;
	version = 1;
	backwards = false;
	sensorError = false;
	InitReceiveBuffer();
	Reset();
}

void RotatingMagnetFilamentMonitor::Reset()
{
	extrusionCommandedThisSegment = extrusionCommandedSinceLastSync = movementMeasuredThisSegment = movementMeasuredSinceLastSync = 0.0;
	magneticMonitorState = MagneticMonitorState::idle;
	haveStartBitData = false;
	synced = false;							// force a resync
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

	if (gb.Seen('A'))
	{
		seen = true;
		checkNonPrintingMoves = (gb.GetIValue() > 0);
	}

	if (seen)
	{
		Init();
	}
	else
	{
		reply.printf("Duet3D rotating magnet filament monitor v%u%s on input %u, %s, sensitivity %.2fmm/rev, allow %ld%% to %ld%%, check every %.1fmm, ",
						version,
						(switchOpenMask != 0) ? " with switch" : "",
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
		else if (sensorError)
		{
			reply.cat("error");
			if (lastErrorCode != 0)
			{
				reply.catf(" %u", lastErrorCode);
			}
		}
		else
		{
			reply.catf("current pos %.1f, ", (double)GetCurrentPosition());
			if (magneticMonitorState != MagneticMonitorState::calibrating && totalExtrusionCommanded > 10.0)
			{
				const float measuredMmPerRev = totalExtrusionCommanded/totalMovementMeasured;
				reply.catf("measured sensitivity %.2fmm/rev, min %ld%% max %ld%% over %.1fmm\n",
					(double)measuredMmPerRev,
					lrintf(100 * minMovementRatio * measuredMmPerRev),
					lrintf(100 * maxMovementRatio * measuredMmPerRev),
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

// Return the current wheel angle
float RotatingMagnetFilamentMonitor::GetCurrentPosition() const
{
	return (sensorValue & TypeMagnetAngleMask) * (360.0/1024.0);
}

// Deal with any received data
void RotatingMagnetFilamentMonitor::HandleIncomingData()
{
	uint16_t val;
	PollResult res;
	while ((res = PollReceiveBuffer(val)) != PollResult::incomplete)
	{
		// We have either received a report or there has been a framing error
		bool receivedPositionReport = false;
		if (res == PollResult::complete)
		{
			// We have a completed a position report. Check the parity.
			uint8_t data8 = (uint8_t)((val >> 8) ^ val);
			data8 ^= (data8 >> 4);
			data8 ^= (data8 >> 2);
			data8 ^= (data8 >> 1);

			// Version 1 sensor:
			//  Data word:			0S00 00pp pppppppp		S = switch open, pppppppppp = 10-bit filament position
			//  Error word:			1000 0000 00000000
			//
			// Version 2 sensor (this firmware):
			//  Data word:			P00S 10pp pppppppp		S = switch open, ppppppppppp = 10-bit filament position
			//  Error word:			P010 0000 0000eeee		eeee = error code
			//	Version word:		P110 0000 vvvvvvvv		vvvvvvvv = sensor/firmware version, at least 2
			if (version == 1)
			{
				if ((data8 & 1) == 0 && (val & 0x7F00) == 0x6000 && (val & 0x00FF) >= 2)
				{
					// Received a version word with the correct parity, so must be version 2 or later
					version = val & 0x00FF;
					if (switchOpenMask != 0)
					{
						switchOpenMask = TypeMagnetV2SwitchOpenMask;
					}
				}
				else if (val == TypeMagnetV1ErrorMask)
				{
					sensorError = true;
					lastErrorCode = 0;
				}
				else if ((val & 0xBC00) == 0)
				{
					receivedPositionReport = true;
					dataReceived = true;
					sensorError = false;
				}
			}
			else if ((data8 & 1) != 0)
			{
				++parityErrorCount;
			}
			else
			{
				switch (val & TypeMagnetV2MessageTypeMask)
				{
				case TypeMagnetV2MessageTypePosition:
					receivedPositionReport = true;
					dataReceived = true;
					sensorError = false;
					break;

				case TypeMagnetV2MessageTypeError:
					lastErrorCode = val & 0x00FF;
					sensorError = true;
					break;

				case TypeMagnetV2MessageTypeInfo:
					if ((val & TypeMagnetV2InfoTypeMask) == TypeMagnetV2InfoTypeVersion)
					{
						version = val & 0x00FF;
					}
					break;

				default:
					break;
				}
			}
		}
		else
		{
			// A receive error occurred. Any start bit data we stored is wrong.
			++framingErrorCount;
		}

		if (receivedPositionReport)
		{
			// We have a completed a position report
			const uint16_t angleChange = (val - sensorValue) & TypeMagnetAngleMask;			// angle change in range 0..1023
			const int32_t movement = (angleChange <= 512) ? (int32_t)angleChange : (int32_t)angleChange - 1024;
			movementMeasuredSinceLastSync += (float)movement/1024;
			sensorValue = val;
			lastMeasurementTime = millis();

			if (haveStartBitData)					// if we have a synchronised value for the amount of extrusion commanded
			{
				if (synced)
				{
					if (   checkNonPrintingMoves
						|| (wasPrintingAtStartBit && (int32_t)(lastSyncTime - reprap.GetMove().ExtruderPrintingSince()) >= SyncDelayMillis)
					   )
					{
						// We can use this measurement
						extrusionCommandedThisSegment += extrusionCommandedAtCandidateStartBit;
						movementMeasuredThisSegment += movementMeasuredSinceLastSync;
					}
				}
				lastSyncTime = candidateStartBitTime;
				extrusionCommandedSinceLastSync -= extrusionCommandedAtCandidateStartBit;
				movementMeasuredSinceLastSync = 0.0;
				synced = checkNonPrintingMoves || wasPrintingAtStartBit;
			}
		}
		haveStartBitData = false;
	}
}

// Call the following at intervals to check the status. This is only called when printing is in progress.
// 'filamentConsumed' is the net amount of extrusion commanded since the last call to this function.
// 'hadNonPrintingMove' is true if filamentConsumed includes extruder movement from non-printing moves.
// 'fromIsr' is true if this measurement was taken at the end of the ISR because a potential start bit was seen
FilamentSensorStatus RotatingMagnetFilamentMonitor::Check(bool isPrinting, bool fromIsr, uint32_t isrMillis, float filamentConsumed)
{
	// 1. Update the extrusion commanded and whether we have had an extruding but non-printing move
	extrusionCommandedSinceLastSync += filamentConsumed;

	// 2. If this call passes values synced to the start bit, save the data for the next completed measurement.
	if (fromIsr && IsWaitingForStartBit())
	{
		extrusionCommandedAtCandidateStartBit = extrusionCommandedSinceLastSync;
		wasPrintingAtStartBit = isPrinting;
		candidateStartBitTime = isrMillis;
		haveStartBitData = true;
	}

	// 3. Process the receive buffer and update everything if we have received anything or had a receive error
	HandleIncomingData();

	// 4. Decide whether it is time to do a comparison, and return the status
	FilamentSensorStatus ret = FilamentSensorStatus::ok;
	if (sensorError)
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
	else if (   extrusionCommandedThisSegment + extrusionCommandedSinceLastSync >= minimumExtrusionCheckLength * 3
			 && millis() - lastMeasurementTime > 500
			 && !IsReceiving()
			)
	{
		// A sync is overdue
		ret = CheckFilament(extrusionCommandedThisSegment + extrusionCommandedSinceLastSync, movementMeasuredThisSegment + movementMeasuredSinceLastSync, true);
		extrusionCommandedThisSegment = extrusionCommandedSinceLastSync = movementMeasuredThisSegment = movementMeasuredSinceLastSync = 0.0;
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

	switch (magneticMonitorState)
	{
	case MagneticMonitorState::idle:
		magneticMonitorState = MagneticMonitorState::calibrating;
		totalExtrusionCommanded = amountCommanded;
		totalMovementMeasured = amountMeasured;
		break;

	case MagneticMonitorState::calibrating:
		totalExtrusionCommanded += amountCommanded;
		totalMovementMeasured += amountMeasured;
		if (totalExtrusionCommanded >= 10.0)
		{
			backwards = (totalMovementMeasured < 0.0);
			if (backwards)
			{
				totalMovementMeasured = -totalMovementMeasured;
			}
			float ratio = totalMovementMeasured/totalExtrusionCommanded;
			minMovementRatio = maxMovementRatio = ratio;

			if (comparisonEnabled)
			{
				ratio *= mmPerRev;
				if (ratio < minMovementAllowed)
				{
					ret = FilamentSensorStatus::tooLittleMovement;
				}
				else if (ratio > maxMovementAllowed)
				{
					ret = FilamentSensorStatus::tooMuchMovement;
				}
			}
			magneticMonitorState = MagneticMonitorState::comparing;
		}
		break;

	case MagneticMonitorState::comparing:
		{
			totalExtrusionCommanded += amountCommanded;
			if (backwards)
			{
				amountMeasured = -amountMeasured;
			}
			totalMovementMeasured += amountMeasured;
			float ratio = amountMeasured/amountCommanded;
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
				ratio *= mmPerRev;
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

// Clear the measurement state. Called when we are not printing a file. Return the present/not present status if available.
FilamentSensorStatus RotatingMagnetFilamentMonitor::Clear()
{
	Reset();											// call this first so that haveStartBitData and synced are false when we call HandleIncomingData
	HandleIncomingData();								// to keep the diagnostics up to date

	return (sensorError) ? FilamentSensorStatus::sensorError
			: ((sensorValue & switchOpenMask) != 0) ? FilamentSensorStatus::noFilament
				: FilamentSensorStatus::ok;
}

// Print diagnostic info for this sensor
void RotatingMagnetFilamentMonitor::Diagnostics(MessageType mtype, unsigned int extruder)
{
	const char* const statusText = (!dataReceived) ? "no data received"
									: (sensorError) ? "error"
										: ((sensorValue & switchOpenMask) != 0) ? "no filament"
											: "ok";
	reprap.GetPlatform().MessageF(mtype, "Extruder %u: pos %.2f, %s, ", extruder, (double)GetCurrentPosition(), statusText);
	if (magneticMonitorState != MagneticMonitorState::calibrating && totalExtrusionCommanded > 10.0)
	{
		const float measuredMmPerRev = totalExtrusionCommanded/totalMovementMeasured;
		reprap.GetPlatform().MessageF(mtype, "measured sens %.2fmm/rev min %ld%% max %ld%% over %.1fmm",
			(double)measuredMmPerRev,
			lrintf(100 * minMovementRatio * measuredMmPerRev),
			lrintf(100 * maxMovementRatio * measuredMmPerRev),
			(double)totalExtrusionCommanded);
	}
	else
	{
		reprap.GetPlatform().Message(mtype, "no calibration data");
	}
	if (dataReceived)
	{
		reprap.GetPlatform().MessageF(mtype, ", errs: frame %" PRIu32 " parity %" PRIu32 " ovrun %" PRIu32 " pol %" PRIu32  " ovdue %" PRIu32 "\n",
			framingErrorCount, parityErrorCount, overrunErrorCount, polarityErrorCount, overdueCount);
	}
}

// End
