/*
 * RemoteHeater.h
 *
 *  Created on: 24 Jul 2019
 *      Author: David
 */

#ifndef SRC_HEATING_REMOTEHEATER_H_
#define SRC_HEATING_REMOTEHEATER_H_

#include "Heater.h"

#if SUPPORT_CAN_EXPANSION

class RemoteHeater : public Heater
{
public:
	RemoteHeater(unsigned int num, CanAddress board);
	~RemoteHeater();

	GCodeResult ConfigurePortAndSensor(const char *portName, PwmFrequency freq, unsigned int sensorNumber, const StringRef& reply) override;
	GCodeResult SetPwmFrequency(PwmFrequency freq, const StringRef& reply) override;
	GCodeResult ReportDetails(const StringRef& reply) const override;

	void Spin() override;							// Called in a tight loop to keep things running
	void SwitchOff() override;						// Not even standby - all heater power off
	GCodeResult ResetFault(const StringRef& reply) override;	// Reset a fault condition - only call this if you know what you are doing
	float GetTemperature() const override;			// Get the current temperature
	float GetAveragePWM() const override;			// Return the running average PWM to the heater. Answer is a fraction in [0, 1].
	float GetAccumulator() const override;			// Return the integral accumulator
	void StartAutoTune(float targetTemp, float maxPwm, const StringRef& reply) override;	// Start an auto tune cycle for this PID
	void GetAutoTuneStatus(const StringRef& reply) const override;	// Get the auto tune status or last result
	void Suspend(bool sus) override;				// Suspend the heater to conserve power or while doing Z probing
	HeaterStatus GetStatus() const override;
	void UpdateRemoteStatus(CanAddress src, const CanHeaterReport& report) override;

protected:
	void ResetHeater() override;
	HeaterMode GetMode() const override;
	GCodeResult SwitchOn(const StringRef& reply) override;		// Turn the heater on and set the mode
	GCodeResult UpdateModel(const StringRef& reply) override;	// Called when the heater model has been changed

private:
	static constexpr uint32_t RemoteStatusTimeout = 2000;

	CanAddress boardAddress;
	enum HeaterStatus lastStatus;
	uint8_t averagePwm;
	float lastTemperature;
	uint32_t whenLastStatusReceived;
};

#endif

#endif /* SRC_HEATING_REMOTEHEATER_H_ */
