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

ZProbe::ZProbe() : EndstopOrZProbe()
{
	SetDefaults();
}

ZProbe::~ZProbe()
{
	inputPort.Release();
	modulationPort.Release();
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
	maxTaps = DefaultZProbeTaps;
	invertReading = turnHeatersOff = saveToConfigOverride = false;
	type = ZProbeType::none;
	heater = -1;
	inputPort.Release();
	modulationPort.Release();
}

bool ZProbe::AssignPorts(GCodeBuffer& gb, const StringRef& reply)
{
	IoPort* const ports[] = { &inputPort, &modulationPort };
	const PinAccess access[] = { PinAccess::read, PinAccess::write0 };
	return IoPort::AssignPorts(gb, reply, PinUsedBy::zprobe, 2, ports, access);
}

bool ZProbe::AssignPorts(const char* pinNames, const StringRef& reply)
{
	IoPort* const ports[] = { &inputPort, &modulationPort };
	const PinAccess access[] = { PinAccess::read, PinAccess::write0 };
	return IoPort::AssignPorts(pinNames, reply, PinUsedBy::zprobe, 2, ports, access);
}

float ZProbe::GetActualTriggerHeight() const
{
	if (heater >= 0)
	{
		const float temperature = reprap.GetHeat().GetTemperature(heater);
		if (temperature >= -100.0)
		{
			return ((temperature - calibTemperature) * temperatureCoefficient) + triggerHeight;
		}
	}
	return triggerHeight;
}

bool ZProbe::WriteParameters(FileStore *f, unsigned int probeNumber) const
{
	String<ScratchStringLength> scratchString;
	scratchString.printf("G31 K%u P%d X%.1f Y%.1f Z%.2f\n", probeNumber, adcValue, (double)xOffset, (double)yOffset, (double)triggerHeight);
	return f->Write(scratchString.c_str());
}

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
			zProbeVal = (int) ((p.GetZProbeOnFilter().GetSum() + p.GetZProbeOffFilter().GetSum()) / (8 * Z_PROBE_AVERAGE_READINGS));
			break;

		case ZProbeType::dumbModulated:		// Dumb modulated IR sensor.
			// We assume that zProbeOnFilter and zProbeOffFilter average the same number of readings.
			// Because of noise, it is possible to get a negative reading, so allow for this.
			zProbeVal = (int) (((int32_t)p.GetZProbeOnFilter().GetSum() - (int32_t)p.GetZProbeOffFilter().GetSum()) / (int)(4 * Z_PROBE_AVERAGE_READINGS));
			break;

		case ZProbeType::unfilteredDigital:		// Switch connected to Z probe input, no filtering
		case ZProbeType::blTouch:				// blTouch is now unfiltered too
			zProbeVal = GetRawReading()/4;
			break;

		case ZProbeType::zMotorStall:
#if HAS_STALL_DETECT
			{
				const DriversBitmap zDrivers = reprap.GetPlatform().GetAxisDriversConfig(Z_AXIS).GetDriversBitmap();
				zProbeVal = ((zDrivers & GetStalledDrivers()) != 0) ? 1000 : 0;
			}
#else
			zProbeVal = 1000;
#endif
			break;

		default:
			zProbeVal = 1000;
		}
	}

	return (invertReading) ? 1000 - zProbeVal : zProbeVal;
}

int ZProbe::GetSecondaryValues(int& v1, int& v2)
{
	const Platform& p = reprap.GetPlatform();
	if (p.GetZProbeOnFilter().IsValid() && p.GetZProbeOffFilter().IsValid())
	{
		switch (type)
		{
		case ZProbeType::dumbModulated:		// modulated IR sensor
			v1 = (int) (p.GetZProbeOnFilter().GetSum() / (4 * Z_PROBE_AVERAGE_READINGS));	// pass back the reading with IR turned on
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
	switch (Stopped())
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

// This is called by the tick ISR to get the raw Z probe reading to feed to the filter
uint16_t ZProbe::GetRawReading() const
{
	switch (type)
	{
	case ZProbeType::analog:
	case ZProbeType::dumbModulated:
	case ZProbeType::alternateAnalog:
		return min<uint16_t>(inputPort.ReadAnalog(), 4000);

	case ZProbeType::digital:
	case ZProbeType::unfilteredDigital:
	case ZProbeType::blTouch:
		return (inputPort.Read()) ? 4000 : 0;

	case ZProbeType::zMotorStall:
	default:
		return 4000;
	}
}

void ZProbe::SetProbing(bool isProbing) const
{
	// For Z probe types other than 1/2/3 and bltouch we set the modulation pin high at the start of a probing move and low at the end
	// Don't do this for bltouch because on the Maestro, the MOD pin is normally used as the servo control output
	if (type > ZProbeType::alternateAnalog && type != ZProbeType::blTouch)
	{
		modulationPort.WriteDigital(isProbing);
	}
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
		heater = gb.GetIValue();
	}

	if (gb.Seen('C'))
	{
		seen = true;
		float currentTemperature = reprap.GetHeat().GetTemperature(heater);
		if (currentTemperature < -100.0)
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
			saveToConfigOverride = true;			// we are loading these parameters from config-override.g, so a subsequent M500 should save them to config-override.g
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

GCodeResult ZProbe::HandleM558(GCodeBuffer& gb, const StringRef &reply, unsigned int probeNumber)
{
	bool seen = false;

	// We must get and set the Z probe type first before setting the dive height etc. because different probe types need different port settings
	if (gb.Seen('P'))
	{
		seen = true;
		const uint32_t requestedType = gb.GetUIValue();
		if (   requestedType >= (uint32_t)ZProbeType::numTypes
			|| requestedType == (uint32_t)ZProbeType::endstopSwitch_obsolete
			|| requestedType == (uint32_t)ZProbeType::e1Switch_obsolete
			|| requestedType == (uint32_t)ZProbeType::zSwitch_obsolete
		   )
		{
			reply.copy("Invalid Z probe type");
			type = ZProbeType::none;
			return GCodeResult::error;
		}

		type = (ZProbeType)requestedType;
		if (!gb.DoDwellTime(100))							// delay a little to allow the averaging filters to accumulate data from the new source
		{
			return GCodeResult::notFinished;
		}
	}

	// Determine the required pin access
	PinAccess access[2];
	switch (type)
	{
	case ZProbeType::analog:
	case ZProbeType::dumbModulated:
		access[0] = PinAccess::readAnalog;
		access[1] = PinAccess::write1;
		break;

	case ZProbeType::alternateAnalog:
		access[0] = PinAccess::readAnalog;
		access[1] = PinAccess::write0;
		break;

	default:
		access[0] = PinAccess::readWithPullup;
		access[1] = PinAccess::write0;
		break;
	}

	// Do the input channel next so that 'seen' will be true only if the type and/or the channel has been specified
	if (gb.Seen('C'))										// input channel
	{
		seen = true;
		IoPort* const ports[] = { &inputPort, &modulationPort };

		if (!IoPort::AssignPorts(gb, reply, PinUsedBy::zprobe, 2, ports, access))
		{
			return GCodeResult::error;
		}
	}
	else if (seen)											// if we had a P parameter then the type may have changed
	{
		(void)inputPort.SetMode(access[0]);
		(void)modulationPort.SetMode(access[1]);
	}

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
		invertReading = (gb.GetIValue() != 0);
		seen = true;
	}

	if (gb.Seen('B'))
	{
		turnHeatersOff = (gb.GetIValue() == 1);
		seen = true;
	}

	gb.TryGetFValue('R', recoveryTime, seen);				// Z probe recovery time
	gb.TryGetFValue('S', tolerance, seen);					// tolerance when multi-tapping

	if (gb.Seen('A'))
	{
		maxTaps = min<uint32_t>(gb.GetUIValue(), ZProbe::MaxTapsLimit);
		seen = true;
	}

	if (!seen)
	{
		reply.printf("Z Probe %u: type %u, input pin ", probeNumber, (unsigned int)type);
		inputPort.AppendPinName(reply);
		reply.cat(", output pin ");
		modulationPort.AppendPinName(reply);
		reply.catf(", invert %s, dive height %.1fmm, probe speed %dmm/min, travel speed %dmm/min, recovery time %.2f sec, heaters %s, max taps %u, max diff %.2f",
						(invertReading) ? "yes" : "no", (double)diveHeight,
						(int)(probeSpeed * MinutesToSeconds), (int)(travelSpeed * MinutesToSeconds),
						(double)recoveryTime,
						(turnHeatersOff) ? "suspended" : "normal",
						maxTaps, (double)tolerance);
	}

	return GCodeResult::ok;
}

// End
