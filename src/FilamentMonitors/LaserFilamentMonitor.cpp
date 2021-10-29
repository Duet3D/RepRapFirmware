/*
 * LaserFilamentMonitor.cpp
 *
 *  Created on: 9 Jan 2018
 *      Author: David
 */

#include "LaserFilamentMonitor.h"
#include <GCodes/GCodeBuffer/GCodeBuffer.h>
#include <Platform/Platform.h>
#include <Platform/RepRap.h>
#include <Movement/Move.h>

#if SUPPORT_REMOTE_COMMANDS
# include <CanMessageGenericParser.h>
#endif

// Unless we set the option to compare filament on all type of move, we reject readings if the last retract or reprime move wasn't completed
// well before the start bit was received. This is because those moves have high accelerations and decelerations, so the measurement delay
// is more likely to cause errors. This constant sets the delay required after a retract or reprime move before we accept the measurement.
const int32_t SyncDelayMillis = 10;

#if SUPPORT_OBJECT_MODEL

// Object model table and functions
// Note: if using GCC version 7.3.1 20180622 and lambda functions are used in this table, you must compile this file with option -std=gnu++17.
// Otherwise the table will be allocated in RAM instead of flash, which wastes too much RAM.

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(...) OBJECT_MODEL_FUNC_BODY(LaserFilamentMonitor, __VA_ARGS__)
#define OBJECT_MODEL_FUNC_IF(...) OBJECT_MODEL_FUNC_IF_BODY(LaserFilamentMonitor, __VA_ARGS__)

constexpr ObjectModelTableEntry LaserFilamentMonitor::objectModelTable[] =
{
	// Within each group, these entries must be in alphabetical order
	// 0. LaserFilamentMonitor members
#ifdef DUET3_ATE
	{ "brightness",			OBJECT_MODEL_FUNC((int32_t)self->brightness),															ObjectModelEntryFlags::live },
#endif
	{ "calibrated", 		OBJECT_MODEL_FUNC_IF(self->IsLocal() && self->dataReceived && self->HaveCalibrationData(), self, 1), 	ObjectModelEntryFlags::live },
	{ "configured", 		OBJECT_MODEL_FUNC(self, 2), 																			ObjectModelEntryFlags::none },
	{ "enabled",			OBJECT_MODEL_FUNC(self->comparisonEnabled),		 														ObjectModelEntryFlags::none },
#ifdef DUET3_ATE
	{ "position",			OBJECT_MODEL_FUNC((int32_t)self->sensorValue),															ObjectModelEntryFlags::live },
	{ "shutter",			OBJECT_MODEL_FUNC((int32_t)self->shutter),																ObjectModelEntryFlags::live },
#endif
	{ "status",				OBJECT_MODEL_FUNC(self->GetStatusText()),																ObjectModelEntryFlags::live },
	{ "type",				OBJECT_MODEL_FUNC_NOSELF("laser"), 																		ObjectModelEntryFlags::none },

	// 1. LaserFilamentMonitor.calibrated members
	{ "percentMax",			OBJECT_MODEL_FUNC(ConvertToPercent(self->maxMovementRatio)), 											ObjectModelEntryFlags::live },
	{ "percentMin",			OBJECT_MODEL_FUNC(ConvertToPercent(self->minMovementRatio)), 											ObjectModelEntryFlags::live },
	{ "sensitivity",		OBJECT_MODEL_FUNC(ConvertToPercent(self->MeasuredSensitivity())), 										ObjectModelEntryFlags::live },
	{ "totalDistance",		OBJECT_MODEL_FUNC(self->totalExtrusionCommanded, 1), 													ObjectModelEntryFlags::live },

	// 2. LaserFilamentMonitor.configured members
	{ "allMoves",			OBJECT_MODEL_FUNC(self->checkNonPrintingMoves), 														ObjectModelEntryFlags::none },
	{ "calibrationFactor",	OBJECT_MODEL_FUNC(self->calibrationFactor, 3), 															ObjectModelEntryFlags::none },
	{ "percentMax",			OBJECT_MODEL_FUNC(ConvertToPercent(self->maxMovementAllowed)), 											ObjectModelEntryFlags::none },
	{ "percentMin",			OBJECT_MODEL_FUNC(ConvertToPercent(self->minMovementAllowed)), 											ObjectModelEntryFlags::none },
	{ "sampleDistance",	 	OBJECT_MODEL_FUNC(self->minimumExtrusionCheckLength, 1), 												ObjectModelEntryFlags::none },
};

constexpr uint8_t LaserFilamentMonitor::objectModelTableDescriptor[] =
{
	3,
#ifdef DUET3_ATE
	8,
#else
	5,
#endif
	4,
	5
};

DEFINE_GET_OBJECT_MODEL_TABLE(LaserFilamentMonitor)

#endif

LaserFilamentMonitor::LaserFilamentMonitor(unsigned int drv, unsigned int monitorType, DriverId did) noexcept
	: Duet3DFilamentMonitor(drv, monitorType, did),
	  calibrationFactor(1.0),
	  minMovementAllowed(DefaultMinMovementAllowed), maxMovementAllowed(DefaultMaxMovementAllowed),
	  minimumExtrusionCheckLength(DefaultMinimumExtrusionCheckLength), comparisonEnabled(false), checkNonPrintingMoves(false)
{
	switchOpenMask = (monitorType == 6) ? TypeLaserSwitchOpenBitMask : 0;
	Init();
}

void LaserFilamentMonitor::Init() noexcept
{
	dataReceived = false;
	sensorValue = 0;
	parityErrorCount = framingErrorCount = overrunErrorCount = polarityErrorCount = overdueCount = 0;
	lastMeasurementTime = 0;
	imageQuality = shutter = brightness = lastErrorCode = 0;
	version = 1;
	backwards = false;
	sensorError = false;
	InitReceiveBuffer();
	Reset();
}

void LaserFilamentMonitor::Reset() noexcept
{
	extrusionCommandedThisSegment = extrusionCommandedSinceLastSync = movementMeasuredThisSegment = movementMeasuredSinceLastSync = 0.0;
	laserMonitorState = LaserMonitorState::idle;
	haveStartBitData = false;
	synced = false;							// force a resync
}

bool LaserFilamentMonitor::HaveCalibrationData() const noexcept
{
	return laserMonitorState != LaserMonitorState::calibrating && totalExtrusionCommanded > 10.0;
}

float LaserFilamentMonitor::MeasuredSensitivity() const noexcept
{
	return totalMovementMeasured/totalExtrusionCommanded;
}

// Configure this sensor, returning true if error and setting 'seen' if we processed any configuration parameters
GCodeResult LaserFilamentMonitor::Configure(GCodeBuffer& gb, const StringRef& reply, bool& seen) THROWS(GCodeException)
{
	const GCodeResult rslt = CommonConfigure(gb, reply, InterruptMode::change, seen);
	if (rslt <= GCodeResult::warning)
	{
		gb.TryGetFValue('L', calibrationFactor, seen);
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
			reprap.SensorsUpdated();
		}
		else
		{
			reply.printf("Duet3D laser filament monitor v%u%s on pin ", version, (switchOpenMask != 0) ? " with switch" : "");
			GetPort().AppendPinName(reply);
			reply.catf(", %s, allow %ld%% to %ld%%, check %s moves every %.1fmm, calibration factor %.3f, ",
						(comparisonEnabled) ? "enabled" : "disabled",
						ConvertToPercent(minMovementAllowed),
						ConvertToPercent(maxMovementAllowed),
						(checkNonPrintingMoves) ? "all" : "printing",
						(double)minimumExtrusionCheckLength,
						(double)calibrationFactor);

			if (!dataReceived)
			{
				reply.cat("no data received");
			}
			else
			{
				reply.catf("version %u, ", version);
				if (switchOpenMask != 0)
				{
					reply.cat(((sensorValue & switchOpenMask) != 0) ? "no filament, " : "filament present, ");
				}
				if (imageQuality != 0)
				{
					reply.catf("quality %u, ", imageQuality);
				}
				if (version >= 2)
				{
					reply.catf("brightness %u, shutter %u, ", brightness, shutter);
				}
				if (sensorError)
				{
					reply.cat("error");
					if (lastErrorCode != 0)
					{
						reply.catf(" %u", lastErrorCode);
					}
				}
				else if (HaveCalibrationData())
				{
					reply.catf("measured min %ld%% avg %ld%% max %ld%% over %.1fmm",
						ConvertToPercent(minMovementRatio),
						ConvertToPercent(MeasuredSensitivity()),
						ConvertToPercent(maxMovementRatio),
						(double)totalExtrusionCommanded);
				}
				else
				{
					reply.cat("no calibration data");
				}
			}
		}
	}
	return rslt;
}

// Return the current position
float LaserFilamentMonitor::GetCurrentPosition() const noexcept
{
	const uint16_t positionRange = (sensorValue & TypeLaserLargeDataRangeBitMask) ? TypeLaserLargeRange : TypeLaserDefaultRange;
	int32_t pos = (int32_t)(sensorValue & (positionRange - 1));
	if (pos > positionRange/2)
	{
		pos -= positionRange;
	}
	return (float)pos * ((sensorValue & TypeLaserLargeDataRangeBitMask) ? 0.01 : 0.02);		// each count is nominally 0.01 or 0.02mm of filament motion
}

// Deal with any received data
void LaserFilamentMonitor::HandleIncomingData() noexcept
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
			if ((data8 & 1) != 0)
			{
				++parityErrorCount;
			}
			else
			{
				switch (val & TypeLaserMessageTypeMask)
				{
				case TypeLaserMessageTypePosition:
					receivedPositionReport = true;
					dataReceived = true;
					sensorError = false;
					break;

				case TypeLaserMessageTypeError:
					lastErrorCode = val & 0x00FF;
					sensorError = true;
					break;

				case TypeLaserMessageTypeQuality:
					brightness = val & 0x00FF;
					shutter = (val >> 8) & 0x1F;
					break;

				case TypeLaserMessageTypeInfo:
					switch (val & TypeLaserInfoTypeMask)
					{
					case TypeLaserInfoTypeVersion:
						version = val & 0x00FF;
						break;

					case TypeLaserInfoTypeImageQuality:
						imageQuality = val & 0x00FF;
						break;

					case TypeLaserInfoTypeBrightness:
						brightness = val & 0x00FF;
						break;

					case TypeLaserInfoTypeShutter:
						shutter = val & 0x00FF;
						break;
					}
					break;
				}
			}
		}
		else
		{
			++framingErrorCount;
		}

		if (receivedPositionReport)
		{
			// We have a completed a position report
			const uint16_t positionRange = (val & TypeLaserLargeDataRangeBitMask) ? TypeLaserLargeRange : TypeLaserDefaultRange;
			const uint16_t positionChange = (val - sensorValue) & (positionRange - 1);			// 10- or 11-bit position change
			const int32_t movement = (positionChange <= positionRange/2) ? (int32_t)positionChange : (int32_t)positionChange - positionRange;
			movementMeasuredSinceLastSync += (float)movement * ((val & TypeLaserLargeDataRangeBitMask) ? 0.01 : 0.02);
			sensorValue = val;
			lastMeasurementTime = millis();

			if (haveStartBitData)	// if we have a synchronised  value for the amount of extrusion commanded
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
FilamentSensorStatus LaserFilamentMonitor::Check(bool isPrinting, bool fromIsr, uint32_t isrMillis, float filamentConsumed) noexcept
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

	return (comparisonEnabled) ? ret : FilamentSensorStatus::ok;
}

// Compare the amount commanded with the amount of extrusion measured, and set up for the next comparison
FilamentSensorStatus LaserFilamentMonitor::CheckFilament(float amountCommanded, float amountMeasured, bool overdue) noexcept
{
	if (!dataReceived)
	{
		return FilamentSensorStatus::noDataReceived;
	}

	if (reprap.Debug(moduleFilamentSensors))
	{
		debugPrintf("Extr req %.3f meas %.3f%s\n", (double)amountCommanded, (double)amountMeasured, (overdue) ? " overdue" : "");
	}

	FilamentSensorStatus ret = FilamentSensorStatus::ok;
	float extrusionMeasured = amountMeasured * calibrationFactor;

	switch (laserMonitorState)
	{
	case LaserMonitorState::idle:
		laserMonitorState = LaserMonitorState::calibrating;
		totalExtrusionCommanded = amountCommanded;
		totalMovementMeasured = extrusionMeasured;
		break;

	case LaserMonitorState::calibrating:
		totalExtrusionCommanded += amountCommanded;
		totalMovementMeasured += extrusionMeasured;
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
				extrusionMeasured = -extrusionMeasured;
			}
			totalMovementMeasured += extrusionMeasured;
			const float ratio = extrusionMeasured/amountCommanded;
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

// Clear the measurement state. Called when we are not printing a file. Return the present/not present status if available.
FilamentSensorStatus LaserFilamentMonitor::Clear() noexcept
{
	Reset();											// call this first so that haveStartBitData and synced are false when we call HandleIncomingData
	HandleIncomingData();								// to keep the diagnostics up to date

	return (!comparisonEnabled) ? FilamentSensorStatus::ok
			: (!dataReceived) ? FilamentSensorStatus::noDataReceived
				: (sensorError) ? FilamentSensorStatus::sensorError
					: ((sensorValue & switchOpenMask) != 0) ? FilamentSensorStatus::noFilament
						: FilamentSensorStatus::ok;
}

// Print diagnostic info for this sensor
void LaserFilamentMonitor::Diagnostics(MessageType mtype, unsigned int extruder) noexcept
{
	String<FormatStringLength> buf;
	buf.printf("Extruder %u: ", extruder);
	if (dataReceived)
	{
		buf.catf("pos %.2f, errs: frame %" PRIu32 " parity %" PRIu32 " ovrun %" PRIu32 " pol %" PRIu32 " ovdue %" PRIu32 "\n",
					(double)GetCurrentPosition(), framingErrorCount, parityErrorCount, overrunErrorCount, polarityErrorCount, overdueCount);
	}
	else
	{
		buf.cat("no data received\n");
	}
	reprap.GetPlatform().Message(mtype, buf.c_str());
}

#if SUPPORT_REMOTE_COMMANDS

// Configure this sensor, returning true if error and setting 'seen' if we processed any configuration parameters
GCodeResult LaserFilamentMonitor::Configure(const CanMessageGenericParser& parser, const StringRef& reply) noexcept
{
	bool seen = false;
	const GCodeResult rslt = CommonConfigure(parser, reply, InterruptMode::change, seen);
	if (rslt <= GCodeResult::warning)
	{
		if (parser.GetFloatParam('L', calibrationFactor))
		{
			seen = true;
		}
		if (parser.GetFloatParam('E', minimumExtrusionCheckLength))
		{
			seen = true;
		}

		uint16_t minMax[2];
		size_t numValues = 2;
		if (parser.GetUint16ArrayParam('R', numValues, minMax))
		{
			if (numValues > 0)
			{
				seen = true;
				minMovementAllowed = (float)minMax[0] * 0.01;
			}
			if (numValues > 1)
			{
				maxMovementAllowed = (float)minMax[1] * 0.01;
			}
		}

		uint16_t temp;
		if (parser.GetUintParam('S', temp))
		{
			seen = true;
			comparisonEnabled = (temp > 0);
		}

		if (parser.GetUintParam('A', temp))
		{
			seen = true;
			checkNonPrintingMoves = (temp > 0);
		}

		if (seen)
		{
			Init();
		}
		else
		{
			reply.printf("Duet3D laser filament monitor v%u%s on pin ", version, (switchOpenMask != 0) ? " with switch" : "");
			GetPort().AppendPinName(reply);
			reply.catf(", %s, allow %ld%% to %ld%%, check every %.1fmm, calibration factor %.3f, ",
						(comparisonEnabled) ? "enabled" : "disabled",
						ConvertToPercent(minMovementAllowed),
						ConvertToPercent(maxMovementAllowed),
						(double)minimumExtrusionCheckLength,
						(double)calibrationFactor);

			if (!dataReceived)
			{
				reply.cat("no data received");
			}
			else
			{
				reply.catf("version %u, ", version);
				if (switchOpenMask != 0)
				{
					reply.cat(((sensorValue & switchOpenMask) != 0) ? "no filament, " : "filament present, ");
				}
				if (imageQuality != 0)
				{
					reply.catf("quality %u, ", imageQuality);
				}
				if (version >= 2)
				{
					reply.catf("brightness %u, shutter %u, ", brightness, shutter);
				}
				if (sensorError)
				{
					reply.cat("error");
					if (lastErrorCode != 0)
					{
						reply.catf(" %u", lastErrorCode);
					}
				}
				else if (HaveCalibrationData())
				{
					reply.catf("measured min %ld%% avg %ld%% max %ld%% over %.1fmm",
						ConvertToPercent(minMovementRatio),
						ConvertToPercent(MeasuredSensitivity()),
						ConvertToPercent(maxMovementRatio),
						(double)totalExtrusionCommanded);
				}
				else
				{
					reply.cat("no calibration data");
				}
			}
		}
	}
	return rslt;
}

#endif

// End
