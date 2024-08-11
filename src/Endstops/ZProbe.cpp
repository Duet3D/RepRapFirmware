/*
 * ZProbe.cpp
 *
 *  Created on: 13 Feb 2018
 *      Author: David
 */

#include "ZProbe.h"
#include <Platform/RepRap.h>
#include <Platform/Platform.h>
#include <GCodes/GCodes.h>
#include <GCodes/GCodeBuffer/GCodeBuffer.h>
#include <Storage/FileStore.h>
#include <Heating/Heat.h>
#include <Math/Matrix.h>
#include <Movement/MoveDebugFlags.h>

#if SUPPORT_OBJECT_MODEL

// Object model table and functions
// Note: if using GCC version 7.3.1 20180622 and lambda functions are used in this table, you must compile this file with option -std=gnu++17.
// Otherwise the table will be allocated in RAM instead of flash, which wastes too much RAM.

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(...)							OBJECT_MODEL_FUNC_BODY(ZProbe, __VA_ARGS__)
#define OBJECT_MODEL_FUNC_IF(_condition, ...)			OBJECT_MODEL_FUNC_IF_BODY(ZProbe, _condition, __VA_ARGS__)
#define OBJECT_MODEL_FUNC_ARRAY_IF(_condition, ...)		OBJECT_MODEL_FUNC_ARRAY_IF_BODY(ZProbe, _condition, __VA_ARGS__)

constexpr ObjectModelArrayTableEntry ZProbe::objectModelArrayTable[] =
{
	// 0. Offsets
	{
		nullptr,					// no lock needed
		[] (const ObjectModel *self, const ObjectExplorationContext&) noexcept -> size_t { return reprap.GetGCodes().GetVisibleAxes(); },
		[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue { return ExpressionValue(((const ZProbe*)self)->offsets[context.GetLastIndex()], 2); }
	},
	// 1. Speeds
	{
		nullptr,
		[] (const ObjectModel *self, const ObjectExplorationContext&) noexcept -> size_t { return (((const ZProbe*)self)->type == ZProbeType::scanningAnalog) ? 3 : 2; },
		[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue
					{ return ExpressionValue(InverseConvertSpeedToMmPerMin(((const ZProbe*)self)->probeSpeeds[context.GetLastIndex()]), 1); }
	},
	// 2. Temperature coefficients
	{
		nullptr,
		[] (const ObjectModel *self, const ObjectExplorationContext&) noexcept -> size_t { return ARRAY_SIZE(ZProbe::temperatureCoefficients); },
		[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue
					{ return ExpressionValue(((const ZProbe*)self)->temperatureCoefficients[context.GetLastIndex()], 5); }
	},
	// 3. Values
	{
		nullptr,
		[] (const ObjectModel *self, const ObjectExplorationContext&) noexcept -> size_t { return (((const ZProbe*)self)->type == ZProbeType::dumbModulated) ? 2 : 1; },
		[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue
					{	int32_t v1 = 0;
						return ExpressionValue
								(	(context.GetLastIndex() == 0)
									? (int32_t)((const ZProbe*)self)->GetReading()
									: (((const ZProbe*)self)->GetSecondaryValues(v1), v1)
								);
					}
	},
	// 4. Dive heights
	{
		nullptr,
		[] (const ObjectModel *self, const ObjectExplorationContext&) noexcept -> size_t { return ARRAY_SIZE(ZProbe::diveHeights); },
		[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue
				{ return ExpressionValue(((const ZProbe*)self)->diveHeights[context.GetLastIndex()], 1); }
	},
#if SUPPORT_SCANNING_PROBES
	// 5. Scanning probe coefficients
	{
		nullptr,
		[] (const ObjectModel *self, const ObjectExplorationContext&) noexcept -> size_t { return ARRAY_SIZE(ZProbe::scanCoefficients); },
		[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue
				{ return ExpressionValue(((const ZProbe*)self)->scanCoefficients[context.GetLastIndex()], 7); }
	},
#endif
};

DEFINE_GET_OBJECT_MODEL_ARRAY_TABLE(ZProbe)

constexpr ObjectModelTableEntry ZProbe::objectModelTable[] =
{
	// Within each group, these entries must be in alphabetical order
	// 0. Probe members
	{ "calibrationTemperature",		OBJECT_MODEL_FUNC(self->calibTemperature, 1), 												ObjectModelEntryFlags::none },
	{ "deployedByUser",				OBJECT_MODEL_FUNC(self->isDeployedByUser), 													ObjectModelEntryFlags::none },
	{ "disablesHeaters",			OBJECT_MODEL_FUNC((bool)self->misc.parts.turnHeatersOff), 									ObjectModelEntryFlags::none },
	{ "diveHeight",					OBJECT_MODEL_FUNC(self->diveHeights[0], 1), 												ObjectModelEntryFlags::obsolete },
	{ "diveHeights",				OBJECT_MODEL_FUNC_ARRAY(4), 																ObjectModelEntryFlags::none },
#if SUPPORT_SCANNING_PROBES
	{ "isCalibrated",				OBJECT_MODEL_FUNC_IF(self->IsScanning(), self->isCalibrated), 								ObjectModelEntryFlags::none },
#endif
	{ "lastStopHeight",				OBJECT_MODEL_FUNC(self->lastStopHeight, 3), 												ObjectModelEntryFlags::none },
	{ "maxProbeCount",				OBJECT_MODEL_FUNC((int32_t)self->misc.parts.maxTaps), 										ObjectModelEntryFlags::none },
#if SUPPORT_SCANNING_PROBES
	{ "measuredHeight",				OBJECT_MODEL_FUNC_IF(self->IsScanning() && self->isCalibrated, self->GetLatestHeight()),	ObjectModelEntryFlags::live },
#endif
	{ "offsets",					OBJECT_MODEL_FUNC_ARRAY(0), 																ObjectModelEntryFlags::none },
	{ "recoveryTime",				OBJECT_MODEL_FUNC(self->recoveryTime, 1), 													ObjectModelEntryFlags::none },
#if SUPPORT_SCANNING_PROBES
	{ "scanCoefficients",			OBJECT_MODEL_FUNC_ARRAY_IF(self->IsScanning(), 5), 											ObjectModelEntryFlags::none },
#endif
	{ "speeds",						OBJECT_MODEL_FUNC_ARRAY(1), 																ObjectModelEntryFlags::none },
	{ "temperatureCoefficients",	OBJECT_MODEL_FUNC_ARRAY(2), 																ObjectModelEntryFlags::none },
	{ "threshold",					OBJECT_MODEL_FUNC((int32_t)self->targetAdcValue), 											ObjectModelEntryFlags::none },
	{ "tolerance",					OBJECT_MODEL_FUNC(self->tolerance, 3), 														ObjectModelEntryFlags::none },
	{ "travelSpeed",				OBJECT_MODEL_FUNC(InverseConvertSpeedToMmPerMin(self->travelSpeed), 1), 					ObjectModelEntryFlags::none },
	{ "triggerHeight",				OBJECT_MODEL_FUNC(-self->offsets[Z_AXIS], 3), 												ObjectModelEntryFlags::none },
	{ "type",						OBJECT_MODEL_FUNC((int32_t)self->type), 													ObjectModelEntryFlags::none },
	{ "value",						OBJECT_MODEL_FUNC_ARRAY(3), 																ObjectModelEntryFlags::live },
};

constexpr uint8_t ZProbe::objectModelTableDescriptor[] = { 1, 17 + 3 * SUPPORT_SCANNING_PROBES };

DEFINE_GET_OBJECT_MODEL_TABLE(ZProbe)

#endif

ZProbe::ZProbe(unsigned int num, ZProbeType p_type) noexcept : EndstopOrZProbe(), lastStopHeight(0.0), number(num), isDeployedByUser(false)
{
	SetDefaults();
	type = p_type;
}

void ZProbe::SetDefaults() noexcept
{
	targetAdcValue = DefaultZProbeADValue;
	for (float& offset : offsets)
	{
		offset = 0.0;
	}
	offsets[Z_AXIS] = -DefaultZProbeTriggerHeight;
	calibTemperature = DefaultZProbeTemperature;
	for (float& tc : temperatureCoefficients)
	{
		tc = 0.0;
	}
	diveHeights[0] = diveHeights[1] = DefaultZDive;
	probeSpeeds[0] = probeSpeeds[1] = probeSpeeds[2] = ConvertSpeedFromMmPerSec(DefaultProbingSpeed);
	travelSpeed = ConvertSpeedFromMmPerSec(DefaultZProbeTravelSpeed);
	recoveryTime = 0.0;
	tolerance = DefaultZProbeTolerance;
	misc.parts.maxTaps = DefaultZProbeTaps;
	misc.parts.turnHeatersOff = misc.parts.saveToConfigOverride = misc.parts.probingAway = false;
	type = ZProbeType::none;
	sensor = -1;
}

// Prepare to use this Z probe
void ZProbe::PrepareForUse(const bool probingAway) noexcept
{
	misc.parts.probingAway = probingAway;

	// We can't read temperature sensors from within the step ISR so calculate the actual trigger height now
	actualTriggerHeight = -offsets[Z_AXIS] + GetTriggerHeightCompensation();
}

// Return the amount by which the trigger height is increased by temperature compensation
float ZProbe::GetTriggerHeightCompensation() const noexcept
{
	if (sensor >= 0)
	{
		TemperatureError err(TemperatureError::unknownError);
		const float temperature = reprap.GetHeat().GetSensorTemperature(sensor, err);
		if (err == TemperatureError::ok)
		{
			const float dt = temperature - calibTemperature;
			return (dt * temperatureCoefficients[0]) + (fsquare(dt) * temperatureCoefficients[1]);
		}
	}
	return 0.0;
}

// Get the dive height that is in effect for the next tap
float ZProbe::GetDiveHeight(int tapsDone) const noexcept
{
	if (FastThenSlowProbing())
	{
		++tapsDone;
	}
	return diveHeights[(tapsDone < 1) ? 0 : 1];
}

// Get the height we should move to when starting this probe.
// If firstTap is true, always move to the primary dive height.
// Otherwise if the secondary dive height is smaller than the primary, move to the secondary dive height plus the error.
float ZProbe::GetStartingHeight(bool firstTap, float previousHeightError) const noexcept
{
	return ((!firstTap && diveHeights[1] < diveHeights[0]) ? diveHeights[1] + previousHeightError : diveHeights[0])
			+ GetActualTriggerHeight();
}

#if SUPPORT_SCANNING_PROBES

float ZProbe::GetScanningHeight() const noexcept
{
	return GetActualTriggerHeight();
}

#endif

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE

bool ZProbe::WriteParameters(FileStore *f, unsigned int probeNumber) const noexcept
{
	const char* axisLetters = reprap.GetGCodes().GetAxisLetters();
	const size_t numTotalAxes = reprap.GetGCodes().GetTotalAxes();
	String<StringLength256> scratchString;
	scratchString.printf("G31 K%u P%" PRIi32, probeNumber, targetAdcValue);
	for (size_t i = 0; i < numTotalAxes; ++i)
	{
		if (axisLetters[i] != 'Z')
		{
			scratchString.catf(" %c%.1f", axisLetters[i], (double)offsets[i]);
		}
	}
	scratchString.catf(" Z%.2f\n", (double)-offsets[Z_AXIS]);
	return f->Write(scratchString.c_str());
}

#endif

int32_t ZProbe::GetReading() const noexcept
{
	int32_t zProbeVal = 0;
	const Platform& p = reprap.GetPlatform();
	if (type == ZProbeType::unfilteredDigital || type == ZProbeType::blTouch || (p.GetZProbeOnFilter().IsValid() && p.GetZProbeOffFilter().IsValid()))
	{
		switch (type)
		{
		case ZProbeType::analog:				// Simple or intelligent IR sensor
		case ZProbeType::alternateAnalog:		// Alternate sensor
		case ZProbeType::digital:				// Switch connected to Z probe input
			zProbeVal = (int32_t)((p.GetZProbeOnFilter().GetSum() + p.GetZProbeOffFilter().GetSum()) / (int32_t)(2 * ZProbeAverageReadings));
			break;

		case ZProbeType::dumbModulated:		// Dumb modulated IR sensor.
			// We assume that zProbeOnFilter and zProbeOffFilter average the same number of readings.
			// Because of noise, it is possible to get a negative reading, so allow for this.
			zProbeVal = ((int32_t)p.GetZProbeOnFilter().GetSum() - (int32_t)p.GetZProbeOffFilter().GetSum())/(int32_t)ZProbeAverageReadings;
			break;

		case ZProbeType::unfilteredDigital:		// Switch connected to Z probe input, no filtering
		case ZProbeType::blTouch:				// blTouch is now unfiltered too
		case ZProbeType::scanningAnalog:		// scanning analog probes are unfiltered for speed
			zProbeVal = GetRawReading();
			break;

		case ZProbeType::zMotorStall:
#if HAS_STALL_DETECT
			{
				const DriversBitmap zDrivers = reprap.GetPlatform().GetAxisDriversConfig(Z_AXIS).GetDriversBitmap();
				zProbeVal = (GetStalledDrivers(zDrivers).IsNonEmpty()) ? 1000 : 0;
			}
#else
			return 1000;
#endif
			break;

		default:
			return 1000;
		}
	}

	return zProbeVal;
}

int32_t ZProbe::GetSecondaryValues(int32_t& v1) const noexcept
{
	const Platform& p = reprap.GetPlatform();
	if (p.GetZProbeOnFilter().IsValid() && p.GetZProbeOffFilter().IsValid())
	{
		switch (type)
		{
		case ZProbeType::dumbModulated:		// modulated IR sensor
			v1 = (int)(p.GetZProbeOnFilter().GetSum() / ZProbeAverageReadings);	// pass back the reading with IR turned on
			return 1;
		default:
			break;
		}
	}
	return 0;
}

// Test whether we are at or near the stop. May be called from an ISR as well and from a normal task context.
bool ZProbe::Stopped() const noexcept
{
	const int32_t zProbeVal = GetReading();
	return zProbeVal >= targetAdcValue;
}

// Check whether the probe is triggered and return the action that should be performed. Called from the step ISR.
EndstopHitDetails ZProbe::CheckTriggered() noexcept
{
	bool b = Stopped();
	if (misc.parts.probingAway)
	{
		b = !b;
	}

	EndstopHitDetails rslt;			// initialised by default constructor
	if (b)
	{
		rslt.SetAction(EndstopHitAction::stopAll);
		rslt.isZProbe = true;
	}
	return rslt;
}

// This is called by the ISR to acknowledge that it is acting on the return from calling CheckTriggered. Called from the step ISR.
// Return true if we have finished with this endstop or probe in this move.
bool ZProbe::Acknowledge(EndstopHitDetails what) noexcept
{
	return what.GetAction() == EndstopHitAction::stopAll;
}

GCodeResult ZProbe::HandleG31(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	GCodeResult err = GCodeResult::ok;
	bool seen = false;

	// Warning - don't change any values until we know that we are not going to return notFinished!
	// This because users may use the existing values when calculating the new ones (e.g. "make babystepping permanent" macro)

	// Do the temperature coefficient first because it may return notFinished
	int8_t newSensor;
	if (gb.Seen('H'))
	{
		seen = true;
		newSensor = gb.GetIValue();
	}
	else
	{
		newSensor = sensor;
	}

	if (gb.Seen('T'))
	{
		seen = true;
		for (float& tc : temperatureCoefficients)
		{
			tc = 0.0;
		}

		TemperatureError terr(TemperatureError::unknownError);
		const float currentTemperature = reprap.GetHeat().GetSensorTemperature(newSensor, terr);
		if (terr == TemperatureError::unknownSensor)
		{
			reply.printf("Temperature coefficients ignored due to invalid sensor number %d", newSensor);
			err = GCodeResult::warning;
		}
		else
		{
			size_t numValues = ARRAY_SIZE(temperatureCoefficients);
			gb.GetFloatArray(temperatureCoefficients, numValues, false);
			float newCalibTemperature = DefaultZProbeTemperature;

			if (gb.Seen('S'))
			{
				newCalibTemperature = gb.GetFValue();
			}
			else if (terr == TemperatureError::ok)
			{
				newCalibTemperature = currentTemperature;
			}
			else if (!gb.IsTimerRunning())				// the sensor may have only just been configured, so give it 500ms to produce a reading
			{
				gb.StartTimer();
				return GCodeResult::notFinished;
			}
			else if (millis() - gb.WhenTimerStarted() < 500)
			{
				return GCodeResult::notFinished;
			}
			else
			{
				gb.StopTimer();
				reply.printf("Sensor %d did not provide a valid temperature reading, using default", newSensor);
				err = GCodeResult::warning;
			}
			gb.StopTimer();
			calibTemperature = newCalibTemperature;
		}
	}

	// After this we don't return notFinished, so it is safe to amend values directly
	const char* axisLetters = reprap.GetGCodes().GetAxisLetters();
	const size_t numTotalAxes = reprap.GetGCodes().GetTotalAxes();
	for (size_t i = 0; i < numTotalAxes; ++i)
	{
		if (axisLetters[i] != 'Z')
		{
			gb.TryGetFValue(axisLetters[i], offsets[i], seen);
		}
	}

	{
		float triggerHeight;
		if (gb.TryGetFValue(axisLetters[Z_AXIS], triggerHeight, seen))
		{
			offsets[Z_AXIS] = -triggerHeight;				// logically, the Z offset of the Z probe is the negative of the trigger height
		}
	}

	if (gb.Seen('P'))
	{
		seen = true;
		targetAdcValue = gb.GetIValue();
	}

	if (seen)
	{
		sensor = newSensor;
		reprap.SensorsUpdated();
		if (gb.LatestMachineState().runningM501)
		{
			misc.parts.saveToConfigOverride = true;			// we are loading these parameters from config-override.g, so a subsequent M500 should save them to config-override.g
		}
	}
	else
	{
		const int v0 = GetReading();
		reply.printf("Z probe %u: current reading %d", number, v0);
		int32_t v1;
		if (GetSecondaryValues(v1) == 1)
		{
			reply.catf(" (%" PRIi32 ")", v1);
		}
		reply.catf(", threshold %" PRIi32 ", trigger height %.3f", targetAdcValue, (double)-offsets[Z_AXIS]);
		if (temperatureCoefficients[0] != 0.0 || temperatureCoefficients[1] != 0.0)
		{
			reply.catf(" at %.1f" DEGREE_SYMBOL "C, temperature coefficients [%.1f/" DEGREE_SYMBOL "C, %.1f/" DEGREE_SYMBOL "C^2]",
						(double)calibTemperature, (double)temperatureCoefficients[0], (double)temperatureCoefficients[1]);
		}
		reply.cat(", offsets");
		for (size_t i = 0; i < numTotalAxes; ++i)
		{
			if (axisLetters[i] != 'Z')
			{
				reply.catf(" %c%.1f", axisLetters[i], (double)offsets[i]);
			}
		}
	}
	return err;
}

GCodeResult ZProbe::Configure(GCodeBuffer& gb, const StringRef &reply, bool& seen) THROWS(GCodeException)
{
	if (gb.Seen('H'))										// dive heights
	{
		size_t numHeights = 2;
		gb.GetFloatArray(diveHeights, numHeights, true);
		seen = true;
	}

	if (gb.Seen('F'))										// feed rate i.e. probing speed
	{
		float userProbeSpeeds[3];
		size_t numSpeeds = 3;
		gb.GetFloatArray(userProbeSpeeds, numSpeeds, true);
		probeSpeeds[0] = ConvertSpeedFromMmPerMin(userProbeSpeeds[0]);
		probeSpeeds[1] = ConvertSpeedFromMmPerMin(userProbeSpeeds[1]);
		probeSpeeds[2] = (numSpeeds == 3) ? ConvertSpeedFromMmPerMin(userProbeSpeeds[2]) : probeSpeeds[0];
		seen = true;
	}

	if (gb.Seen('T'))		// travel speed to probe point
	{
		travelSpeed = gb.GetSpeedFromMm(false);
		seen = true;
	}

	if (gb.Seen('B'))
	{
		misc.parts.turnHeatersOff = (gb.GetIValue() == 1);
		seen = true;
	}

	gb.TryGetFValue('R', recoveryTime, seen);				// Z probe recovery time
	gb.TryGetFValue('S', tolerance, seen);					// tolerance when multi-tapping

	if (gb.Seen('A'))
	{
		misc.parts.maxTaps = min<uint32_t>(gb.GetUIValue(), ZProbe::MaxTapsLimit);
		seen = true;
	}

	if (seen)
	{
		reprap.SensorsUpdated();
		return GCodeResult::ok;
	}

	reply.printf("Z Probe %u: type %u", number, (unsigned int)type);
	const GCodeResult rslt = AppendPinNames(reply);
	reply.catf(", dive heights %.1f,%.1fmm, probe speeds %d,%d",
					(double)diveHeights[0], (double)diveHeights[1],
					(int)InverseConvertSpeedToMmPerMin(probeSpeeds[0]),
					(int)InverseConvertSpeedToMmPerMin(probeSpeeds[1]));
	if (type == ZProbeType::scanningAnalog)
	{
		reply.catf(",%d", (int)InverseConvertSpeedToMmPerMin(probeSpeeds[2]));
	}
	reply.catf("mm/min, travel speed %dmm/min, recovery time %.2f sec, heaters %s, max taps %u, max diff %.2f",
					(int)InverseConvertSpeedToMmPerMin(travelSpeed),
					(double)recoveryTime,
					(misc.parts.turnHeatersOff) ? "suspended" : "normal",
						misc.parts.maxTaps, (double)tolerance);
	return rslt;
}

// Default implementation of SendProgram, overridden in some classes
GCodeResult ZProbe::SendProgram(const uint32_t zProbeProgram[], size_t len, const StringRef& reply) noexcept
{
	reply.copy("This configuration of Z probe does not support programming");
	return GCodeResult::error;
}

void ZProbe::SetLastStoppedHeight(float h) noexcept
{
	lastStopHeight = h;
	reprap.SensorsUpdated();
}

#if SUPPORT_SCANNING_PROBES

// Scanning support
GCodeResult ZProbe::SetScanningCoefficients(float aParam, float bParam, float cParam) noexcept
{
	scanCoefficients[0] = 0.0;
	scanCoefficients[1] = aParam;
	scanCoefficients[2] = bParam;
	scanCoefficients[3] =  cParam;
	isCalibrated = true;
	reprap.SensorsUpdated();
	return GCodeResult::ok;
}

float ZProbe::ConvertReadingToHeightDifference(int32_t reading) const noexcept
{
	const float diff = (float)(reading - targetAdcValue);
	return scanCoefficients[0] + diff * (scanCoefficients[1] + diff * (scanCoefficients[2] + diff * scanCoefficients[3]));
}

GCodeResult ZProbe::ReportScanningCoefficients(const StringRef& reply) noexcept
{
	if (isCalibrated)
	{
		reply.printf("Scanning probe offset: %.3fmm, A: %.3e, B: %.3e, C: %.3e", (double)scanCoefficients[0], (double)scanCoefficients[1], (double)scanCoefficients[2], (double)scanCoefficients[3]);
		return GCodeResult::ok;
	}

	reply.copy("Probe has not been calibrated");
	return GCodeResult::error;
}

void ZProbe::CalibrateScanningProbe(const int32_t calibrationReadings[], size_t numCalibrationReadingsTaken, float heightChangePerPoint, const StringRef& reply) noexcept
{
	const int32_t referenceReading = calibrationReadings[numCalibrationReadingsTaken/2];
	FixedMatrix<float, 4, 5> matrix;
	matrix.Fill(0.0);

	// Do a least squares fit of a cubic polynomial to the data
	// Store { N, sum(X), sum(X^2), sum(X^3) sum(Y) } in row 0
	// Store { sum(X), sum(X^2), sum(X^3), sum(X^4), sum(XY) } in row 1
	// Store { sum(X^2), sum(X^3), sum(X^4), sum(X^5), sum(X^2Y) } in row 2
	// Store { sum(X^3), sum(X^4), sum(X^5), sum(X^6), sum(X^3Y) } in row 3
	matrix(0, 0) = (float)numCalibrationReadingsTaken;
	for (size_t i = 0; i < numCalibrationReadingsTaken; ++i)
	{
		const float y = ((int)(numCalibrationReadingsTaken/2) - (int)i) * heightChangePerPoint;		// the height different from the trigger height
		const float x = (float)(calibrationReadings[i] - referenceReading);							// the difference in reading from the target reading at the trigger height
		const float x2 = fsquare(x);
		const float x3 = x * x2;
		const float x4 = fsquare(x2);
		const float x5 = x * x4;
		const float x6 = x3 * x3;
		const float xy = x * y;
		const float x2y = x2 * y;
		const float x3y = x3 * y;
		matrix(0, 1) += x;
		matrix(0, 2) += x2;
		matrix(0, 3) += x3;
		matrix(0, 4) += y;
		matrix(1, 0) += x;
		matrix(1, 1) += x2;
		matrix(1, 2) += x3;
		matrix(1, 3) += x4;
		matrix(1, 4) += xy;
		matrix(2, 0) += x2;
		matrix(2, 1) += x3;
		matrix(2, 2) += x4;
		matrix(2, 3) += x5;
		matrix(2, 4) += x2y;
		matrix(3, 0) += x3;
		matrix(3, 1) += x4;
		matrix(3, 2) += x5;
		matrix(3, 3) += x6;
		matrix(3, 4) += x3y;
	}
	matrix.GaussJordan(4, 5);

	scanCoefficients[0] = matrix(0, 4);
	scanCoefficients[1] = matrix(1, 4);
	scanCoefficients[2] = matrix(2, 4);
	scanCoefficients[3] = matrix(3, 4);
	targetAdcValue = referenceReading;
	isCalibrated = true;
	reprap.SensorsUpdated();
	ReportScanningCoefficients(reply);

	// Calculate the RMS error after subtracting the mean error
	float sumOfErrorSquares = 0.0;
	for (size_t i = 0; i < numCalibrationReadingsTaken; ++i)
	{
		const float readingDiff = (float)(calibrationReadings[i] - referenceReading);
		const float actualHeightDiff = ((int)(numCalibrationReadingsTaken/2) - (int)i) * heightChangePerPoint;	// the height different from the trigger height
		const float predictedHeightDiff = scanCoefficients[0] + readingDiff * (scanCoefficients[1] + readingDiff * (scanCoefficients[2] + readingDiff * scanCoefficients[3]));	// the predicted value from the fitted curve
		sumOfErrorSquares += fsquare(predictedHeightDiff - actualHeightDiff);
	}

	if (reprap.GetDebugFlags(Module::Move).IsBitSet(MoveDebugFlags::ZProbing))
	{
		debugPrintf("NumReadings: %u\nHeight interval: %.5f\nScanned data:", numCalibrationReadingsTaken, (double)heightChangePerPoint);
		for (size_t i = 0; i < numCalibrationReadingsTaken; ++i)
		{
			debugPrintf("%c%" PRIi32, (i == 0) ? ' ' : ',', calibrationReadings[i] - referenceReading);
		}
		debugPrintf("\nFit: %.5f,%.5g,%.5g,%.5g\n", (double)scanCoefficients[0], (double)scanCoefficients[1], (double)scanCoefficients[2], (double)scanCoefficients[3]);
	}

	reply.catf(", reading at trigger height %" PRIi32 ", rms error %.3fmm",
					referenceReading, (double)sqrtf(sumOfErrorSquares/(float)numCalibrationReadingsTaken));
}

#endif

// End
