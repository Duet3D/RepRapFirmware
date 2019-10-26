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

ZProbe::ZProbe(unsigned int num, ZProbeType p_type) : EndstopOrZProbe(), number(num)
{
	SetDefaults();
	type = p_type;
}

void ZProbe::SetDefaults()
{
	adcValue = DefaultZProbeADValue;
	xOffset= yOffset = 0.0;
	triggerHeight = DefaultZProbeTriggerHeight;
	calibTemperature = DefaultZProbeTemperature;
	temperatureCoefficient = 0.0;
	diveHeight = DefaultZDive;
	probeSpeed = DefaultProbingSpeed;
	travelSpeed = DefaultZProbeTravelSpeed;
	recoveryTime = 0.0;
	tolerance = DefaultZProbeTolerance;
	misc.parts.maxTaps = DefaultZProbeTaps;
	misc.parts.invertReading = misc.parts.turnHeatersOff = misc.parts.saveToConfigOverride = misc.parts.probingAway = false;
	type = ZProbeType::none;
	sensor = -1;
}

float ZProbe::GetActualTriggerHeight() const
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

bool ZProbe::WriteParameters(FileStore *f, unsigned int probeNumber) const
{
	String<ScratchStringLength> scratchString;
	scratchString.printf("G31 K%u P%d X%.1f Y%.1f Z%.2f\n", probeNumber, adcValue, (double)xOffset, (double)yOffset, (double)triggerHeight);
	return f->Write(scratchString.c_str());
}

#endif

int ZProbe::GetReading() const
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
			zProbeVal = (int) (((int32_t)p.GetZProbeOnFilter().GetSum() - (int32_t)p.GetZProbeOffFilter().GetSum())/ZProbeAverageReadings);
			break;

		case ZProbeType::unfilteredDigital:		// Switch connected to Z probe input, no filtering
		case ZProbeType::blTouch:				// blTouch is now unfiltered too
			zProbeVal = GetRawReading();
			break;

		case ZProbeType::zMotorStall:
#if HAS_STALL_DETECT
			{
				const DriversBitmap zDrivers = reprap.GetPlatform().GetAxisDriversConfig(Z_AXIS).GetDriversBitmap();
				zProbeVal = ((zDrivers & GetStalledDrivers()) != 0) ? 1000 : 0;
			}
#else
			return 1000;
#endif
			break;

		default:
			return 1000;
		}
	}

	return (misc.parts.invertReading) ? 1000 - zProbeVal : zProbeVal;
}

int ZProbe::GetSecondaryValues(int& v1, int& v2)
{
	const Platform& p = reprap.GetPlatform();
	if (p.GetZProbeOnFilter().IsValid() && p.GetZProbeOffFilter().IsValid())
	{
		switch (type)
		{
		case ZProbeType::dumbModulated:		// modulated IR sensor
			v1 = (int) (p.GetZProbeOnFilter().GetSum() / ZProbeAverageReadings);	// pass back the reading with IR turned on
			return 1;
		default:
			break;
		}
	}
	return 0;
}

// Test whether we are at or near the stop
EndStopHit ZProbe::Stopped() const
{
	const int zProbeVal = GetReading();
	return (zProbeVal >= adcValue) ? EndStopHit::atStop
			: (zProbeVal * 10 >= adcValue * 9) ? EndStopHit::nearStop	// if we are at/above 90% of the target value
				: EndStopHit::noStop;
}

// Check whether the probe is triggered and return the action that should be performed. Called from the step ISR.
EndstopHitDetails ZProbe::CheckTriggered(bool goingSlow)
{
	EndstopHitDetails rslt;
	EndStopHit e = Stopped();

	// Note: This might need to be moved into Stopped() to not having to duplicate it to ZProbeEndstop
	if (misc.parts.probingAway) {
		switch (e) {
		case EndStopHit::atStop:
			e = EndStopHit::noStop;
			break;
		default:
			e = EndStopHit::atStop;
			break;
		}
	}

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
bool ZProbe::Acknowledge(EndstopHitDetails what)
{
	return what.GetAction() == EndstopHitAction::stopAll;
}

GCodeResult ZProbe::HandleG31(GCodeBuffer& gb, const StringRef& reply)
{
	GCodeResult err = GCodeResult::ok;
	bool seen = false;
	const char* axisLetters = reprap.GetGCodes().GetAxisLetters();

	gb.TryGetFValue(axisLetters[X_AXIS], xOffset, seen);
	gb.TryGetFValue(axisLetters[Y_AXIS], yOffset, seen);
	gb.TryGetFValue(axisLetters[Z_AXIS], triggerHeight, seen);
	if (gb.Seen('P'))
	{
		seen = true;
		adcValue = gb.GetIValue();
	}

	if (gb.Seen('H'))
	{
		seen = true;
		sensor = gb.GetIValue();
	}

	if (gb.Seen('C'))
	{
		seen = true;
		TemperatureError terr;
		float currentTemperature = reprap.GetHeat().GetSensorTemperature(sensor, terr);
		if (terr != TemperatureError::success)
		{
			reply.copy("Cannot set a temperature coefficient without a valid heater number");
			err = GCodeResult::error;
			currentTemperature = DefaultZProbeTemperature;
		}
		temperatureCoefficient = gb.GetFValue();
		calibTemperature = (gb.Seen('S')) ? gb.GetFValue() : currentTemperature;
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
		int v1, v2;
		switch (GetSecondaryValues(v1, v2))
		{
		case 1:
			reply.printf("Current reading %d (%d)", v0, v1);
			break;
		case 2:
			reply.printf("Current reading %d (%d, %d)", v0, v1, v2);
			break;
		default:
			reply.printf("Current reading %d", v0);
			break;
		}
		reply.catf(", threshold %d, trigger height %.2f", adcValue, (double)triggerHeight);
		if (temperatureCoefficient != 0.0)
		{
			reply.catf(" at %.1f" DEGREE_SYMBOL "C, temperature coefficient %.1f/" DEGREE_SYMBOL "C", (double)calibTemperature, (double)temperatureCoefficient);
		}
		reply.catf(", offsets X%.1f Y%.1f", (double)xOffset, (double)yOffset);
	}
	return err;
}

GCodeResult ZProbe::Configure(GCodeBuffer& gb, const StringRef &reply, bool& seen)
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

	if (gb.Seen('I'))
	{
		misc.parts.invertReading = (gb.GetIValue() != 0);
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
	reply.catf(", invert %s, dive height %.1fmm, probe speed %dmm/min, travel speed %dmm/min, recovery time %.2f sec, heaters %s, max taps %u, max diff %.2f",
					(misc.parts.invertReading) ? "yes" : "no", (double)diveHeight,
					(int)(probeSpeed * MinutesToSeconds), (int)(travelSpeed * MinutesToSeconds),
					(double)recoveryTime,
					(misc.parts.turnHeatersOff) ? "suspended" : "normal",
						misc.parts.maxTaps, (double)tolerance);
	return rslt;
}

// Default implementation of SendProgram, overridden in some classes
GCodeResult ZProbe::SendProgram(const uint32_t zProbeProgram[], size_t len, const StringRef& reply)
{
	reply.copy("This configuration of Z probe does not support programming");
	return GCodeResult::error;
}

// End
