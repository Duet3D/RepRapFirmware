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

RotatingMagnetFilamentMonitor::RotatingMagnetFilamentMonitor(int type)
	: Duet3DFilamentMonitor(type),
	  mmPerRev(DefaultMmPerRev), tolerance(DefaultTolerance), minimumExtrusionCheckLength(DefaultMinimumExtrusionCheckLength), checkNonPrintingMoves(true)
{
	switchOpenMask = (type == 4) ? TypeMagnetSwitchOpenMask : 0;
	Init();
}

void RotatingMagnetFilamentMonitor::Init()
{
	sensorValue = 0;
	calibrationStarted = dataReceived = false;
	lastMeasurementTime = 0;
	InitReceiveBuffer();
	Reset();
}

void RotatingMagnetFilamentMonitor::Reset()
{
	extrusionCommanded = movementMeasured = extrusionCommandedAtLastMeasurement = extrusionCommandedAtStartBit = movementMeasuredAtLastCheck = 0.0;
	samplesReceived = 0;
	comparisonStarted = false;
}

// Configure this sensor, returning true if error and setting 'seen' if we processed any configuration parameters
bool RotatingMagnetFilamentMonitor::Configure(GCodeBuffer& gb, StringRef& reply, bool& seen)
{
	if (ConfigurePin(gb, reply, CHANGE, seen))
	{
		return true;
	}

	gb.TryGetFValue('S', mmPerRev, seen);								// this is mm per rev for Duet3D sensors, mm per pulse for pulsed sensors
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
		reply.printf("Duet3D rotating magnet filament monitor%s on endstop input %u, sensitivity %.2fmm/rev, check every %.1fmm, tolerance %.1f%%, ",
						(switchOpenMask != 0) ? " with microswitch" : "",
						GetEndstopNumber(),
						(double)mmPerRev,
						(double)minimumExtrusionCheckLength,
						(double)(tolerance * 100.0));
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
				const float measuredCalibration = totalExtrusionCommanded/totalMovementMeasured;
				const float normalRatio = 1.0/measuredCalibration;
				const int measuredPosTolerance = lrintf(100.0 * (((normalRatio > 0.0) ? maxMovementRatio : minMovementRatio) - normalRatio)/normalRatio);
				const int measuredNegTolerance = lrintf(100.0 * (normalRatio - ((normalRatio > 0.0) ? minMovementRatio : maxMovementRatio))/normalRatio);
				reply.catf("measured sensitivity %.2fmm/rev over %.1fmm, tolerance +%d%% -%d%%",
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

// This is called from the poll function when we receive what could be a start bit
void RotatingMagnetFilamentMonitor::OnStartBitReceived() /*override*/
{
	extrusionCommandedAtStartBit = extrusionCommanded;		// record the extrusion
}

// This is called from the poll function when we have received a complete word of data
void RotatingMagnetFilamentMonitor::ProcessReceivedWord(uint16_t val)
{
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
		const uint16_t angleChange = (val - sensorValue) & TypeMagnetAngleMask;			// angle change in range 0..1023
		const int32_t movement = (angleChange <= 512) ? (int32_t)angleChange : (int32_t)angleChange - 1024;
		movementMeasured += (float)movement/1024;
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
float RotatingMagnetFilamentMonitor::GetCurrentPosition() const
{
	return (sensorValue & TypeMagnetAngleMask) * (360.0/1024.0);
}

// Call the following at intervals to check the status. This is only called when extrusion is in progress or imminent.
// 'filamentConsumed' is the net amount of extrusion since the last call to this function.
// 'hadNonPrintingMove' is called if filamentConsumed includes extruder movement form non-printing moves.
FilamentSensorStatus RotatingMagnetFilamentMonitor::Check(bool full, bool hadNonPrintingMove, float filamentConsumed)
{
	const bool useThisMeasurement = checkNonPrintingMoves || !hadNonPrintingMove;

	PollReceiveBuffer();										// this may update movementMeasured

	FilamentSensorStatus ret = FilamentSensorStatus::ok;
	if (!useThisMeasurement)
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
			if ((sensorValue & TypeMagnetErrorMask) != 0)
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
	}

	return ret;
}

// Compare the amount commanded with the amount of extrusion measured, and set up for the next comparison
FilamentSensorStatus RotatingMagnetFilamentMonitor::CheckFilament(float amountCommanded, float amountMeasured, bool overdue)
{
	if (reprap.Debug(moduleFilamentSensors))
	{
		debugPrintf("Extr req %.3f meas %.3f rem %.3f %s\n", (double)amountCommanded, (double)amountMeasured, (double)(extrusionCommanded - amountCommanded),
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
		const float extrusionMeasured = amountMeasured * mmPerRev;
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
	const float ratio = amountMeasured/amountCommanded;
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
	PollReceiveBuffer();								// to keep the diagnostics up to date
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
	PollReceiveBuffer();
	const char* const statusText = (!dataReceived) ? "no data received"
									: ((sensorValue & TypeMagnetErrorMask) != 0) ? "error"
										: ((sensorValue & switchOpenMask) != 0) ? "no filament"
											: "ok";
	reprap.GetPlatform().MessageF(mtype, "Extruder %u sensor: position %.2f, %s, ", extruder, (double)GetCurrentPosition(), statusText);
	if (calibrationStarted && fabsf(totalMovementMeasured) > 1.0 && totalExtrusionCommanded > 20.0)
	{
		const float measuredMmPerRev = totalExtrusionCommanded/totalMovementMeasured;
		const float normalRatio = 1.0/measuredMmPerRev;
		const int measuredPosTolerance = lrintf(100.0 * (((normalRatio > 0.0) ? maxMovementRatio : minMovementRatio) - normalRatio)/normalRatio);
		const int measuredNegTolerance = lrintf(100.0 * (normalRatio - ((normalRatio > 0.0) ? minMovementRatio : maxMovementRatio))/normalRatio);
		reprap.GetPlatform().MessageF(mtype,"measured sensitivity %.2fmm/rev +%d%% -%d%%\n",
										(double)measuredMmPerRev, measuredPosTolerance, measuredNegTolerance);
	}
	else
	{
		reprap.GetPlatform().Message(mtype, "no calibration data\n");
	}
}

// End
