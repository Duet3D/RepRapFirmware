/*
 * ZProbe.cpp
 *
 *  Created on: 13 Feb 2018
 *      Author: David
 */

#include "ZProbe.h"

#include "Storage/FileStore.h"

// ZProbeParameters class
void ZProbe::Init(float h)
{
	adcValue = DefaultZProbeADValue;
	xOffset = yOffset = 0.0;
	triggerHeight = h;
	calibTemperature = 20.0;
	temperatureCoefficient = 0.0;	// no default temperature correction
	diveHeight = DefaultZDive;
	probeSpeed = DefaultProbingSpeed;
	travelSpeed = DefaultZProbeTravelSpeed;
	recoveryTime = 0.0;
	tolerance = DefaultZProbeTolerance;
	maxTaps = DefaultZProbeTaps;
	inputChannel = 0;
	invertReading = turnHeatersOff = saveToConfigOverride = false;
}

float ZProbe::GetStopHeight(float temperature) const
{
	return ((temperature - calibTemperature) * temperatureCoefficient) + triggerHeight;
}

bool ZProbe::WriteParameters(FileStore *f, unsigned int probeType) const
{
	String<ScratchStringLength> scratchString;
	scratchString.printf("G31 T%u P%d X%.1f Y%.1f Z%.2f\n", probeType, adcValue, (double)xOffset, (double)yOffset, (double)triggerHeight);
	return f->Write(scratchString.c_str());
}

// End
