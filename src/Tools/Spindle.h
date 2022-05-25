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
#include <General/NamedEnum.h>

NamedEnum(SpindleState, uint8_t, unconfigured, stopped, forward, reverse);

class Spindle INHERIT_OBJECT_MODEL
{
private:
	void SetRpm(const uint32_t rpm) noexcept;

	PwmPort pwmPort, onOffPort, reverseNotForwardPort;
	float minPwm, maxPwm, idlePwm;
	uint32_t currentRpm, configuredRpm, minRpm, maxRpm;
	PwmFrequency frequency;
	SpindleState state;

protected:
	DECLARE_OBJECT_MODEL

public:
	Spindle() noexcept;

	GCodeResult Configure(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);

	uint32_t GetCurrentRpm() const noexcept { return currentRpm; }
	uint32_t GetMinRpm() const noexcept { return minRpm; }
	uint32_t GetMaxRpm() const noexcept { return maxRpm; }
	uint32_t GetRpm() const noexcept { return configuredRpm; }
	bool IsValidRpm(const uint32_t rpm) const noexcept { return rpm >= minRpm && rpm <= maxRpm; }
	void SetConfiguredRpm(const uint32_t rpm, const bool updateCurrentRpm) noexcept;
	SpindleState GetState() const noexcept { return state; }
	void SetState(const SpindleState newState) noexcept;
};

#endif
