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
NamedEnum(SpindleType, uint8_t, enaDir, fwdRev);

const SpindleType DefaultSpindleType(SpindleType::enaDir);

class Spindle INHERIT_OBJECT_MODEL
{
private:
	void SetRpm(uint32_t rpm) noexcept;
	PwmPort pwmPort, onOffPort, reverseNotForwardPort;
	float minPwm, maxPwm, idlePwm;
	uint32_t currentRpm, configuredRpm, minRpm, maxRpm;
	PwmFrequency frequency;
	SpindleType type;
	SpindleState state;

protected:
	DECLARE_OBJECT_MODEL

public:
	Spindle() noexcept;

	GCodeResult Configure(uint32_t spindleNumber, GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);

	uint32_t GetCurrentRpm() const noexcept { return currentRpm; }
	uint32_t GetMinRpm() const noexcept { return minRpm; }
	uint32_t GetMaxRpm() const noexcept { return maxRpm; }
	uint32_t GetRpm() const noexcept { return configuredRpm; }
	bool IsValidRpm(uint32_t rpm) const noexcept { return rpm >= minRpm && rpm <= maxRpm; }
	void SetConfiguredRpm(uint32_t rpm, bool updateCurrentRpm) noexcept;
	SpindleType GetType() const noexcept { return type; }
	SpindleState GetState() const noexcept { return state; }
	void SetState(SpindleState newState) noexcept;
	bool IsConfigured() const noexcept { return state != SpindleState::unconfigured; }
};

#endif
