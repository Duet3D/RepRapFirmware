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
	bool inverted;
	float currentRpm, configuredRpm, maxRpm;
	int toolNumber;

public:
	Spindle() : inverted(false), currentRpm(0.0), configuredRpm(0.0), maxRpm(DefaultMaxSpindleRpm), toolNumber(-1) { }

	bool SetPins(LogicalPin lpr, LogicalPin lpf, bool invert);
	void GetPins(LogicalPin& lpf, LogicalPin& lpr, bool& invert) const;

	int GetToolNumber() const { return toolNumber; }
	void SetToolNumber(int tool) { toolNumber = tool; }

	void SetPwmFrequency(float freq);
	void SetMaxRpm(float max) { maxRpm = max; }

	float GetCurrentRpm() const { return currentRpm; }
	float GetRpm() const { return configuredRpm; }
	void SetRpm(float rpm);

	void TurnOff();
};

#endif
