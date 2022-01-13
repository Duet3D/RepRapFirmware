/*
 * PulsedFilamentSensor.cpp
 *
 *  Created on: 9 Jan 2018
 *      Author: David
 */

#include "PulsedFilamentMonitor.h"
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
#define OBJECT_MODEL_FUNC(...) OBJECT_MODEL_FUNC_BODY(PulsedFilamentMonitor, __VA_ARGS__)
#define OBJECT_MODEL_FUNC_IF(...) OBJECT_MODEL_FUNC_IF_BODY(PulsedFilamentMonitor, __VA_ARGS__)

constexpr ObjectModelTableEntry PulsedFilamentMonitor::objectModelTable[] =
{
	// Within each group, these entries must be in alphabetical order
	// 0. PulsedFilamentMonitor members
	{ "calibrated", 	OBJECT_MODEL_FUNC_IF(self->IsLocal() && self->DataReceived() && self->HaveCalibrationData(), self, 1), 	ObjectModelEntryFlags::live },
	{ "configured", 	OBJECT_MODEL_FUNC(self, 2), 																			ObjectModelEntryFlags::none },
	{ "enabled",		OBJECT_MODEL_FUNC(self->comparisonEnabled),		 														ObjectModelEntryFlags::none },
	{ "status",			OBJECT_MODEL_FUNC(self->GetStatusText()),																ObjectModelEntryFlags::live },
	{ "type",			OBJECT_MODEL_FUNC_NOSELF("pulsed"), 																	ObjectModelEntryFlags::none },

	// 1. PulsedFilamentMonitor.calibrated members
	{ "mmPerPulse",		OBJECT_MODEL_FUNC(self->MeasuredSensitivity(), 3), 														ObjectModelEntryFlags::live },
	{ "percentMax",		OBJECT_MODEL_FUNC(ConvertToPercent(self->maxMovementRatio)), 											ObjectModelEntryFlags::live },
	{ "percentMin",		OBJECT_MODEL_FUNC(ConvertToPercent(self->minMovementRatio)), 											ObjectModelEntryFlags::live },
	{ "totalDistance",	OBJECT_MODEL_FUNC(self->totalExtrusionCommanded, 1), 													ObjectModelEntryFlags::live },

	// 2. PulsedFilamentMonitor.configured members
	{ "mmPerPulse",		OBJECT_MODEL_FUNC(self->mmPerPulse, 3), 																ObjectModelEntryFlags::none },
	{ "percentMax",		OBJECT_MODEL_FUNC(ConvertToPercent(self->maxMovementAllowed)), 											ObjectModelEntryFlags::none },
	{ "percentMin",		OBJECT_MODEL_FUNC(ConvertToPercent(self->minMovementAllowed)), 											ObjectModelEntryFlags::none },
	{ "sampleDistance", OBJECT_MODEL_FUNC(self->minimumExtrusionCheckLength, 1), 												ObjectModelEntryFlags::none },
};

constexpr uint8_t PulsedFilamentMonitor::objectModelTableDescriptor[] = { 3, 5, 4, 4 };

DEFINE_GET_OBJECT_MODEL_TABLE(PulsedFilamentMonitor)

#endif

PulsedFilamentMonitor::PulsedFilamentMonitor(unsigned int drv, unsigned int monitorType, DriverId did) noexcept
	: FilamentMonitor(drv, monitorType, did),
	  mmPerPulse(DefaultMmPerPulse),
	  minMovementAllowed(DefaultMinMovementAllowed), maxMovementAllowed(DefaultMaxMovementAllowed),
	  minimumExtrusionCheckLength(DefaultMinimumExtrusionCheckLength), comparisonEnabled(false)
{
	Init();
}

void PulsedFilamentMonitor::Init() noexcept
{
	sensorValue = 0;
	calibrationStarted = false;
	samplesReceived = 0;
	lastMeasurementTime = 0;
	Reset();
}

void PulsedFilamentMonitor::Reset() noexcept
{
	extrusionCommandedThisSegment = extrusionCommandedSinceLastSync = movementMeasuredThisSegment = movementMeasuredSinceLastSync = 0.0;
	comparisonStarted = false;
	haveInterruptData = false;
	wasPrintingAtInterrupt = false;			// force a resync
}

bool PulsedFilamentMonitor::DataReceived() const noexcept
{
	return samplesReceived >= 2;
}

bool PulsedFilamentMonitor::HaveCalibrationData() const noexcept
{
	return calibrationStarted && fabsf(totalMovementMeasured) > 1.0 && totalExtrusionCommanded > 20.0;
}

float PulsedFilamentMonitor::MeasuredSensitivity() const noexcept
{
	return totalExtrusionCommanded/totalMovementMeasured;
}

// Configure this sensor, returning true if error and setting 'seen' if we processed any configuration parameters
GCodeResult PulsedFilamentMonitor::Configure(GCodeBuffer& gb, const StringRef& reply, bool& seen) THROWS(GCodeException)
{
	const GCodeResult rslt = CommonConfigure(gb, reply, InterruptMode::rising, seen);
	if (Succeeded(rslt))
	{
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
			reprap.SensorsUpdated();
		}
		else
		{
			reply.copy("Pulse-type filament monitor on pin ");
			GetPort().AppendPinName(reply);
			reply.catf(", %s, sensitivity %.3fmm/pulse, allowed movement %ld%% to %ld%%, check every %.1fmm, ",
						(comparisonEnabled) ? "enabled" : "disabled",
						(double)mmPerPulse,
						ConvertToPercent(minMovementAllowed),
						ConvertToPercent(maxMovementAllowed),
						(double)minimumExtrusionCheckLength);

			if (!DataReceived())
			{
				reply.cat("no data received");
			}
			else if (HaveCalibrationData())
			{
				reply.catf("measured sensitivity %.3fmm/pulse, measured minimum %ld%%, maximum %ld%% over %.1fmm\n",
					(double)MeasuredSensitivity(),
					ConvertToPercent(minMovementRatio),
					ConvertToPercent(maxMovementRatio),
					(double)totalExtrusionCommanded);
			}
			else
			{
				reply.cat("no calibration data");
			}
		}
	}
	return rslt;
}

// ISR for when the pin state changes. It should return true if the ISR wants the commanded extrusion to be fetched.
bool PulsedFilamentMonitor::Interrupt() noexcept
{
	++sensorValue;
	if (samplesReceived < 100)
	{
		++samplesReceived;
	}

	// Most pulsed filament monitors have low resolution, but at least one user has a high-resolution one.
	// So don't automatically try to sync on every interrupt.
	const uint32_t now = millis();
	if (now - lastMeasurementTime >= 50)
	{
		lastMeasurementTime = millis();
		return true;
	}
	return false;
}

// Call the following regularly to keep the status up to date
void PulsedFilamentMonitor::Poll() noexcept
{
	IrqDisable();
	const uint32_t locSensorVal = sensorValue;
	sensorValue = 0;
	IrqEnable();
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

// Call the following at intervals to check the status. This is only called when extrusion is in progress or imminent.
// 'filamentConsumed' is the net amount of extrusion since the last call to this function.
// 'isPrinting' is true unless a non-printing extruder move was in progress
// 'fromIsr' is true if this measurement was taken at the end of the ISR because the ISR returned true
FilamentSensorStatus PulsedFilamentMonitor::Check(bool isPrinting, bool fromIsr, uint32_t isrMillis, float filamentConsumed) noexcept
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

	return (comparisonEnabled) ? ret : FilamentSensorStatus::ok;
}

// Compare the amount commanded with the amount of extrusion measured, and set up for the next comparison
FilamentSensorStatus PulsedFilamentMonitor::CheckFilament(float amountCommanded, float amountMeasured, bool overdue) noexcept
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
FilamentSensorStatus PulsedFilamentMonitor::Clear() noexcept
{
	Poll();								// to keep the diagnostics up to date
	Reset();
	return FilamentSensorStatus::ok;
}

// Print diagnostic info for this sensor
void PulsedFilamentMonitor::Diagnostics(MessageType mtype, unsigned int extruder) noexcept
{
	Poll();
	const char* const statusText = (samplesReceived < 2) ? "no data received" : "ok";
	reprap.GetPlatform().MessageF(mtype, "Extruder %u sensor: %s\n", extruder, statusText);
}

#if SUPPORT_REMOTE_COMMANDS

// Configure this sensor, returning true if error and setting 'seen' if we processed any configuration parameters
GCodeResult PulsedFilamentMonitor::Configure(const CanMessageGenericParser& parser, const StringRef& reply) noexcept
{
	bool seen = false;
	const GCodeResult rslt = CommonConfigure(parser, reply, InterruptMode::rising, seen);
	if (rslt <= GCodeResult::warning)
	{
		if (parser.GetFloatParam('L', mmPerPulse))
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

		if (seen)
		{
			Init();
		}
		else
		{
			reply.copy("Pulse-type filament monitor on pin ");
			GetPort().AppendPinName(reply);
			reply.catf(", %s, sensitivity %.3fmm/pulse, allowed movement %ld%% to %ld%%, check every %.1fmm, ",
						(comparisonEnabled) ? "enabled" : "disabled",
						(double)mmPerPulse,
						ConvertToPercent(minMovementAllowed),
						ConvertToPercent(maxMovementAllowed),
						(double)minimumExtrusionCheckLength);

			if (!DataReceived())
			{
				reply.cat("no data received");
			}
			else if (HaveCalibrationData())
			{
				reply.catf("measured sensitivity %.3fmm/pulse, measured minimum %ld%%, maximum %ld%% over %.1fmm\n",
					(double)MeasuredSensitivity(),
					ConvertToPercent(minMovementRatio),
					ConvertToPercent(maxMovementRatio),
					(double)totalExtrusionCommanded);
			}
			else
			{
				reply.cat("no calibration data");
			}
		}
	}
	return rslt;
}

#endif

// End
