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
	PwmPort pwmPort, onOffPort, reverseNotForwardPort;
	int32_t currentRpm, configuredRpm, minRpm, maxRpm;
	PwmFrequency frequency;
	int toolNumber;

protected:
	DECLARE_OBJECT_MODEL

public:
	Spindle() noexcept;

	GCodeResult Configure(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);

	int GetToolNumber() const noexcept { return toolNumber; }
	int32_t GetCurrentRpm() const noexcept { return currentRpm; }
	int32_t GetRpm() const noexcept { return configuredRpm; }
	void SetRpm(int32_t rpm) noexcept;
	void TurnOff() noexcept;
};

#endif
