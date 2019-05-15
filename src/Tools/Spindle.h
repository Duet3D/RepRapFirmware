/*
 * Spindle.h
 *
 *  Created on: Mar 21, 2018
 *      Author: Christian
 */

#ifndef SPINDLE_H
#define SPINDLE_H

#include "RepRapFirmware.h"
#include "Hardware/IoPorts.h"

class Spindle
{
private:
	PwmPort spindleForwardPort, spindleReversePort;
	float currentRpm, configuredRpm, maxRpm;
	int toolNumber;

public:
	Spindle() : currentRpm(0.0), configuredRpm(0.0), maxRpm(DefaultMaxSpindleRpm), toolNumber(-1) { }

	bool AllocatePins(GCodeBuffer& gb, const StringRef& reply);			// Allocate the pins returning true if successful

	void SetFrequency(PwmFrequency freq);

	int GetToolNumber() const { return toolNumber; }
	void SetToolNumber(int tool) { toolNumber = tool; }

	void SetMaxRpm(float max) { maxRpm = max; }

	float GetCurrentRpm() const { return currentRpm; }
	float GetRpm() const { return configuredRpm; }
	void SetRpm(float rpm);

	void TurnOff();
};

#endif
