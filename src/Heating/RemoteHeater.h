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
	RemoteHeater(unsigned int num, CanAddress board) : Heater(num), boardAddress(board) { }
	RemoteHeater(const Heater& h, CanAddress board) : Heater(h), boardAddress(board) { }

	void Spin() override;							// Called in a tight loop to keep things running
	GCodeResult ConfigurePortAndSensor(GCodeBuffer& gb, const StringRef& reply) override;
	void SwitchOff() override;						// Not even standby - all heater power off
	void ResetFault() override;						// Reset a fault condition - only call this if you know what you are doing
	float GetTemperature() const override;			// Get the current temperature
	float GetAveragePWM() const override;			// Return the running average PWM to the heater. Answer is a fraction in [0, 1].
	float GetAccumulator() const override;			// Return the integral accumulator
	void StartAutoTune(float targetTemp, float maxPwm, const StringRef& reply) override;	// Start an auto tune cycle for this PID
	void GetAutoTuneStatus(const StringRef& reply) const override;	// Get the auto tune status or last result
	void Suspend(bool sus) override;				// Suspend the heater to conserve power or while doing Z probing

protected:
	void ResetHeater() override;
	HeaterMode GetMode() const override;
	void SwitchOn() override;						// Turn the heater on and set the mode
	GCodeResult UpdateModel(const StringRef& reply) override;	// Called when the heater model has been changed

private:
	CanAddress boardAddress;
};

#endif

#endif /* SRC_HEATING_REMOTEHEATER_H_ */
