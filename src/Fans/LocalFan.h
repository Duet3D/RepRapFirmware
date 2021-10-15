/*
 * LocalFan.h
 *
 *  Created on: 3 Sep 2019
 *      Author: David
 */

#ifndef SRC_FANS_LOCALFAN_H_
#define SRC_FANS_LOCALFAN_H_

#include "Fan.h"

class LocalFan : public Fan
{
public:
	LocalFan(unsigned int fanNum) noexcept;
	~LocalFan() noexcept;

	bool Check(bool checkSensors) noexcept override;						// update the fan PWM returning true if it is a thermostatic fan that is on
	bool IsEnabled() const noexcept override { return port.IsValid(); }
	int32_t GetRPM() const noexcept override;
	float GetPwm() const noexcept override { return lastVal; }
	PwmFrequency GetPwmFrequency() const noexcept override { return port.GetFrequency(); }
	GCodeResult SetPwmFrequency(PwmFrequency freq, const StringRef& reply) noexcept override;
	GCodeResult ReportPortDetails(const StringRef& str) const noexcept override;

#if SUPPORT_CAN_EXPANSION
	void UpdateFromRemote(CanAddress src, const FanReport& report) noexcept override { }
#endif
#if SUPPORT_REMOTE_COMMANDS
	bool IsLocal() const noexcept override { return true; }
#endif

	bool AssignPorts(const char *pinNames, const StringRef& reply) noexcept;

	void Interrupt() noexcept;

protected:
	GCodeResult Refresh(const StringRef& reply) noexcept override;
	bool UpdateFanConfiguration(const StringRef& reply) noexcept override;

private:
	void SetHardwarePwm(float pwmVal) noexcept;
	void InternalRefresh(bool checkSensors) noexcept;

	PwmPort port;											// port used to control the fan
	IoPort tachoPort;										// port used to read the tacho

	float lastPwm;											// the last PWM value we wrote to the hardware
	float lastVal;											// the last PWM value we sent to the fan, not allowing for blipping, or -1 if we don't know it

	// Variables used to read the tacho
	static constexpr uint32_t fanMaxInterruptCount = 16;	// number of fan interrupts that we average over. We time out after 3 seconds, so a count of 16 allows us to read rpm down to (16/3) * (60/2) = 160.
	uint32_t fanInterruptCount;								// accessed only in ISR, so no need to declare it volatile
	volatile uint32_t fanLastResetTime;						// time (in step clocks) at which we last reset the interrupt count, accessed inside and outside ISR
	volatile uint32_t fanInterval;							// written by ISR, read outside the ISR

	uint32_t blipStartTime;
	bool blipping;
};

#endif /* SRC_FANS_LOCALFAN_H_ */
