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
#include "ObjectModel/ObjectModel.h"

class Spindle INHERIT_OBJECT_MODEL
{
private:
	PwmPort spindleForwardPort, spindleReversePort;
	float currentRpm, configuredRpm, maxRpm;
	PwmFrequency frequency;
	int toolNumber;

protected:
	DECLARE_OBJECT_MODEL

public:
	Spindle() noexcept : currentRpm(0.0), configuredRpm(0.0), maxRpm(DefaultMaxSpindleRpm), frequency(0), toolNumber(-1) { }

	bool AllocatePins(GCodeBuffer& gb, const StringRef& reply) noexcept;			// Allocate the pins returning true if successful

	void SetFrequency(PwmFrequency freq) noexcept;

	int GetToolNumber() const noexcept { return toolNumber; }
	void SetToolNumber(int tool) noexcept { toolNumber = tool; }

	void SetMaxRpm(float max) noexcept { maxRpm = max; }

	float GetCurrentRpm() const noexcept { return currentRpm; }
	float GetRpm() const noexcept { return configuredRpm; }
	void SetRpm(float rpm) noexcept;

	void TurnOff() noexcept;
};

#endif
