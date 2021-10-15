/*
 * RemoteFan.h
 *
 *  Created on: 3 Sep 2019
 *      Author: David
 */

#ifndef SRC_FANS_REMOTEFAN_H_
#define SRC_FANS_REMOTEFAN_H_

#include "Fan.h"

#if SUPPORT_CAN_EXPANSION

class RemoteFan : public Fan
{
public:
	RemoteFan(unsigned int fanNum, CanAddress boardNum) noexcept;
	~RemoteFan() noexcept;

	bool Check(bool checkSensors) noexcept override;						// update the fan PWM returning true if it is a thermostatic fan that is on
	bool IsEnabled() const noexcept override;
	int32_t GetRPM() const noexcept override { return lastRpm; }
	float GetPwm() const noexcept override { return lastPwm; }
	PwmFrequency GetPwmFrequency() const noexcept override { return frequency; }
	GCodeResult SetPwmFrequency(PwmFrequency freq, const StringRef& reply) noexcept override;
	GCodeResult ReportPortDetails(const StringRef& str) const noexcept override;
	void UpdateFromRemote(CanAddress src, const FanReport& report) noexcept override;
#if SUPPORT_REMOTE_COMMANDS
	bool IsLocal() const noexcept override { return false; }
#endif

	GCodeResult ConfigurePort(const char *pinNames, PwmFrequency freq, const StringRef& reply) noexcept;

protected:
	bool UpdateFanConfiguration(const StringRef& reply) noexcept override;
	GCodeResult Refresh(const StringRef& reply) noexcept override;

private:
	int32_t lastRpm;
	float lastPwm;
	uint32_t whenLastReportReceived;
	PwmFrequency frequency;					// saved copy of the PWM frequency so that we can report it in the OM
	CanAddress boardNumber;
};

#endif

#endif /* SRC_FANS_REMOTEFAN_H_ */
