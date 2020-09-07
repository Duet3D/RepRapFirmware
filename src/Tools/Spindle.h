/*
 * Spindle.h
 *
 *  Created on: Mar 21, 2018
 *      Author: Christian
 */

#ifndef SPINDLE_H
#define SPINDLE_H

#include <RepRapFirmware.h>
#include <Hardware/IoPorts.h>
#include <ObjectModel/ObjectModel.h>
#include <GCodes/GCodeResult.h>

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
	Spindle() noexcept;

	GCodeResult Configure(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);

	int GetToolNumber() const noexcept { return toolNumber; }
	float GetCurrentRpm() const noexcept { return currentRpm; }
	float GetRpm() const noexcept { return configuredRpm; }
	void SetRpm(float rpm) noexcept;
	void TurnOff() noexcept;
};

#endif
