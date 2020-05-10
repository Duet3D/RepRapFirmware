/*
 * ZProbe.cpp
 *
 *  Created on: 13 Feb 2018
 *      Author: David
 */

#include "ZProbe.h"
#include "RepRap.h"
#include "Platform.h"
#include "GCodes/GCodes.h"
#include "GCodes/GCodeBuffer/GCodeBuffer.h"
#include "Storage/FileStore.h"
#include "Heating/Heat.h"

#if SUPPORT_OBJECT_MODEL

// Object model table and functions
// Note: if using GCC version 7.3.1 20180622 and lambda functions are used in this table, you must compile this file with option -std=gnu++17.
// Otherwise the table will be allocated in RAM instead of flash, which wastes too much RAM.

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(...) OBJECT_MODEL_FUNC_BODY(ZProbe, __VA_ARGS__)

constexpr ObjectModelArrayDescriptor ZProbe::offsetsArrayDescriptor =
{
	nullptr,
	[] (const ObjectModel *self, const ObjectExplorationContext&) noexcept -> size_t { return 2; },
	[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue
				{ return ExpressionValue((context.GetLastIndex() == 0) ? ((const ZProbe*)self)->xOffset : ((const ZProbe*)self)->yOffset, 1); }
};

constexpr ObjectModelArrayDescriptor ZProbe::valueArrayDescriptor =
{
	nullptr,
	[] (const ObjectModel *self, const ObjectExplorationContext&) noexcept -> size_t { return (((const ZProbe*)self)->type == ZProbeType::dumbModulated) ? 2 : 1; },
	[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue
				{	int v1 = 0;
					return ExpressionValue
							(	(context.GetLastIndex() == 0)
								? (int32_t)((const ZProbe*)self)->GetReading()
								: (((const ZProbe*)self)->GetSecondaryValues(v1), v1)
							);
				}
};

constexpr ObjectModelTableEntry ZProbe::objectModelTable[] =
{
	// Within each group, these entries must be in alphabetical order
	// 0. Probe members
	{ "calibrationTemperature",	OBJECT_MODEL_FUNC(self->calibTemperature, 1), 				ObjectModelEntryFlags::none },
	{ "deployedByUser",			OBJECT_MODEL_FUNC(self->isDeployedByUser), 					ObjectModelEntryFlags::none },
	{ "disablesHeaters",		OBJECT_MODEL_FUNC((bool)self->misc.parts.turnHeatersOff), 	ObjectModelEntryFlags::none },
	{ "diveHeight",				OBJECT_MODEL_FUNC(self->diveHeight, 1), 					ObjectModelEntryFlags::none },
	{ "maxProbeCount",			OBJECT_MODEL_FUNC((int32_t)self->misc.parts.maxTaps), 		ObjectModelEntryFlags::none },
	{ "offsets",				OBJECT_MODEL_FUNC_NOSELF(&offsetsArrayDescriptor), 			ObjectModelEntryFlags::none },
	{ "recoveryTime",			OBJECT_MODEL_FUNC(self->recoveryTime, 1), 					ObjectModelEntryFlags::none },
	{ "speed",					OBJECT_MODEL_FUNC(self->probeSpeed, 1), 					ObjectModelEntryFlags::none },
	{ "temperatureCoefficient",	OBJECT_MODEL_FUNC(self->temperatureCoefficient, 3), 		ObjectModelEntryFlags::none },
	{ "threshold",				OBJECT_MODEL_FUNC((int32_t)self->adcValue), 				ObjectModelEntryFlags::none },
	{ "tolerance",				OBJECT_MODEL_FUNC(self->tolerance, 3), 						ObjectModelEntryFlags::none },
	{ "travelSpeed",			OBJECT_MODEL_FUNC(self->travelSpeed, 1), 					ObjectModelEntryFlags::none },
	{ "triggerHeight",			OBJECT_MODEL_FUNC(self->triggerHeight, 3), 					ObjectModelEntryFlags::none },
	{ "type",					OBJECT_MODEL_FUNC((int32_t)self->type), 					ObjectModelEntryFlags::none },
	{ "value",					OBJECT_MODEL_FUNC_NOSELF(&valueArrayDescriptor), 			ObjectModelEntryFlags::live },
};

constexpr uint8_t ZProbe::objectModelTableDescriptor[] = { 1, 15 };

DEFINE_GET_OBJECT_MODEL_TABLE(ZProbe)

#endif

ZProbe::ZProbe(unsigned int num, ZProbeType p_type) noexcept : EndstopOrZProbe(), number(num), isDeployedByUser(false)
{
	SetDefaults();
	type = p_type;
}

void ZProbe::SetDefaults() noexcept
{
	adcValue = DefaultZProbeADValue;
	xOffset = yOffset = 0.0;
	triggerHeight = DefaultZProbeTriggerHeight;
	calibTemperature = DefaultZProbeTemperature;
	temperatureCoefficient = 0.0;
	diveHeight = DefaultZDive;
	probeSpeed = DefaultProbingSpeed;
	travelSpeed = DefaultZProbeTravelSpeed;
	recoveryTime = 0.0;
	tolerance = DefaultZProbeTolerance;
	misc.parts.maxTaps = DefaultZProbeTaps;
	misc.parts.turnHeatersOff = misc.parts.saveToConfigOverride = misc.parts.probingAway = false;
	type = ZProbeType::none;
	sensor = -1;
}

float ZProbe::GetActualTriggerHeight() const noexcept
{
	if (sensor >= 0)
	{
		TemperatureError err;
		const float temperature = reprap.GetHeat().GetSensorTemperature(sensor, err);
		if (err == TemperatureError::success)
		{
			return ((temperature - calibTemperature) * temperatureCoefficient) + triggerHeight;
		}
	}
	return triggerHeight;
}

#if HAS_MASS_STORAGE

bool ZProbe::WriteParameters(FileStore *f, unsigned int probeNumber) const noexcept
{
	String<StringLength100> scratchString;
	scratchString.printf("G31 K%u P%d X%.1f Y%.1f Z%.2f\n", probeNumber, adcValue, (double)xOffset, (double)yOffset, (double)triggerHeight);
	return f->Write(scratchString.c_str());
}

#endif

int ZProbe::GetReading() const noexcept
{
	int zProbeVal = 0;
	const Platform& p = reprap.GetPlatform();
	if (type == ZProbeType::unfilteredDigital || type == ZProbeType::blTouch || (p.GetZProbeOnFilter().IsValid() && p.GetZProbeOffFilter().IsValid()))
	{
		switch (type)
		{
		case ZProbeType::analog:				// Simple or intelligent IR sensor
		case ZProbeType::alternateAnalog:		// Alternate sensor
		case ZProbeType::digital:				// Switch connected to Z probe input
			zProbeVal = (int) ((p.GetZProbeOnFilter().GetSum() + p.GetZProbeOffFilter().GetSum()) / (2 * ZProbeAverageReadings));
			break;

		case ZProbeType::dumbModulated:		// Dumb modulated IR sensor.
			// We assume that zProbeOnFilter and zProbeOffFilter average the same number of readings.
			// Because of noise, it is possible to get a negative reading, so allow for this.
			zProbeVal = (int) (((int32_t)p.GetZProbeOnFilter().GetSum() - (int32_t)p.GetZProbeOffFilter().GetSum())/(int)ZProbeAverageReadings);
			break;

		case ZProbeType::unfilteredDigital:		// Switch connected to Z probe input, no filtering
		case ZProbeType::blTouch:				// blTouch is now unfiltered too
			zProbeVal = GetRawReading();
			break;

		case ZProbeType::zMotorStall:
#if HAS_STALL_DETECT
			{
				const DriversBitmap zDrivers = reprap.GetPlatform().GetAxisDriversConfig(Z_AXIS).GetDriversBitmap();
				zProbeVal = (zDrivers.Intersects(GetStalledDrivers())) ? 1000 : 0;
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

int ZProbe::GetSecondaryValues(int& v1) const noexcept
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

// Test whether we are at or near the stop
EndStopHit ZProbe::Stopped() const noexcept
{
	const int zProbeVal = GetReading();
	return (zProbeVal >= adcValue) ? EndStopHit::atStop
			: (zProbeVal * 10 >= adcValue * 9) ? EndStopHit::nearStop	// if we are at/above 90% of the target value
				: EndStopHit::noStop;
}

// Check whether the probe is triggered and return the action that should be performed. Called from the step ISR.
EndstopHitDetails ZProbe::CheckTriggered(bool goingSlow) noexcept
{
	EndStopHit e = Stopped();
	if (misc.parts.probingAway)
	{
		e = (e == EndStopHit::atStop) ? EndStopHit::noStop : EndStopHit::atStop;
	}

	EndstopHitDetails rslt;			// initialised by default constructor
	switch (e)
	{
	case EndStopHit::atStop:
		rslt.SetAction(EndstopHitAction::stopAll);
		rslt.isZProbe = true;
		break;

	case EndStopHit::nearStop:
		if (!goingSlow)
		{
			rslt.SetAction(EndstopHitAction::reduceSpeed);
		}
		break;

	default:
		break;
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

	// Do the temperature coefficient first because it may return notFinished
	if (gb.Seen('H'))
	{
		seen = true;
		sensor = gb.GetIValue();
	}

	if (gb.Seen('C'))
	{
		seen = true;
		temperatureCoefficient = gb.GetFValue();
		calibTemperature = DefaultZProbeTemperature;

		TemperatureError terr;
		const float currentTemperature = reprap.GetHeat().GetSensorTemperature(sensor, terr);
		if (terr == TemperatureError::unknownSensor)
		{
			reply.copy("Cannot set a temperature coefficient without a valid sensor number");
			temperatureCoefficient = 0.0;
			err = GCodeResult::error;
		}
		else if (gb.Seen('S'))
		{
			calibTemperature = gb.GetFValue();
		}
		else if (terr == TemperatureError::success)
		{
			calibTemperature = currentTemperature;
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
			reply.printf("Sensor %d did not provide a valid temperature reading", sensor);
			err = GCodeResult::warning;
		}
	}

	const char* axisLetters = reprap.GetGCodes().GetAxisLetters();

	gb.TryGetFValue(axisLetters[X_AXIS], xOffset, seen);
	gb.TryGetFValue(axisLetters[Y_AXIS], yOffset, seen);
	gb.TryGetFValue(axisLetters[Z_AXIS], triggerHeight, seen);
	if (gb.Seen('P'))
	{
		seen = true;
		adcValue = gb.GetIValue();
	}

	if (seen)
	{
		if (!reprap.GetGCodes().LockMovementAndWaitForStandstill(gb))
		{
			return GCodeResult::notFinished;
		}
		if (gb.MachineState().runningM501)
		{
			misc.parts.saveToConfigOverride = true;		// we are loading these parameters from config-override.g, so a subsequent M500 should save them to config-override.g
		}
	}
	else
	{
		const int v0 = GetReading();
		reply.printf("Z probe %u: current reading %d", number, v0);
		int v1;
		if (GetSecondaryValues(v1) == 1)
		{
			reply.catf(" (%d)", v1);
		}
		reply.catf(", threshold %d, trigger height %.3f", adcValue, (double)triggerHeight);
		if (temperatureCoefficient != 0.0)
		{
			reply.catf(" at %.1f" DEGREE_SYMBOL "C, temperature coefficient %.1f/" DEGREE_SYMBOL "C", (double)calibTemperature, (double)temperatureCoefficient);
		}
		reply.catf(", offsets X%.1f Y%.1f", (double)xOffset, (double)yOffset);
	}
	return err;
}

GCodeResult ZProbe::Configure(GCodeBuffer& gb, const StringRef &reply, bool& seen) THROWS(GCodeException)
{
	gb.TryGetFValue('H', diveHeight, seen);					// dive height
	if (gb.Seen('F'))										// feed rate i.e. probing speed
	{
		probeSpeed = gb.GetFValue() * SecondsToMinutes;
		seen = true;
	}

	if (gb.Seen('T'))		// travel speed to probe point
	{
		travelSpeed = gb.GetFValue() * SecondsToMinutes;
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
		return GCodeResult::ok;
	}

	reply.printf("Z Probe %u: type %u", number, (unsigned int)type);
	const GCodeResult rslt = AppendPinNames(reply);
	reply.catf(", dive height %.1fmm, probe speed %dmm/min, travel speed %dmm/min, recovery time %.2f sec, heaters %s, max taps %u, max diff %.2f",
					(double)diveHeight,
					(int)(probeSpeed * MinutesToSeconds), (int)(travelSpeed * MinutesToSeconds),
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

// End
