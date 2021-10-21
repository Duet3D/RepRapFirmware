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
	RemoteHeater(unsigned int num, CanAddress board) noexcept;
	~RemoteHeater() noexcept;

	GCodeResult ConfigurePortAndSensor(const char *portName, PwmFrequency freq, unsigned int sn, const StringRef& reply) override;
	GCodeResult SetPwmFrequency(PwmFrequency freq, const StringRef& reply) override;
	GCodeResult ReportDetails(const StringRef& reply) const noexcept override;

	void Spin() noexcept override;											// Called in a tight loop to keep things running
	void SwitchOff() noexcept override;										// Not even standby - all heater power off
	GCodeResult ResetFault(const StringRef& reply) noexcept override;		// Reset a fault condition - only call this if you know what you are doing
	float GetTemperature() const noexcept override;							// Get the latest temperature
	float GetAveragePWM() const noexcept override;							// Return the running average PWM to the heater. Answer is a fraction in [0, 1].
	float GetAccumulator() const noexcept override;							// Return the integral accumulator
	void Suspend(bool sus) noexcept override;								// Suspend the heater to conserve power or while doing Z probing
	void FeedForwardAdjustment(float fanPwmChange, float extrusionChange) noexcept override;
	void SetExtrusionFeedForward(float pwm) noexcept override { }			// We can't yet set feedforward on remote heaters because this is called from an ISR
	bool IsLocal() const noexcept override { return false; }
	void UpdateRemoteStatus(CanAddress src, const CanHeaterReport& report) noexcept override;
	void UpdateHeaterTuning(CanAddress src, const CanMessageHeaterTuningReport& msg) noexcept override;

#if SUPPORT_REMOTE_COMMANDS
	GCodeResult TuningCommand(const CanMessageHeaterTuningCommand& msg, const StringRef& reply) noexcept override;
#endif

protected:
	void ResetHeater() noexcept override;
	HeaterMode GetMode() const noexcept override;
	GCodeResult SwitchOn(const StringRef& reply) noexcept override;			// Turn the heater on and set the mode
	GCodeResult UpdateModel(const StringRef& reply) noexcept override;		// Called when the heater model has been changed
	GCodeResult UpdateFaultDetectionParameters(const StringRef& reply) noexcept override;
	GCodeResult UpdateHeaterMonitors(const StringRef& reply) noexcept override;
	GCodeResult StartAutoTune(const StringRef& reply, bool seenA, float ambientTemp) noexcept override;
																			// Start an auto tune cycle for this heater
private:
	enum class TuningState : uint8_t
	{
		notTuning = 0,
		stabilising,
		heatingUp,
		idleCycles,
		cycling
	};

	GCodeResult SendTuningCommand(const StringRef& reply, bool on) noexcept;
	void StopTuning() noexcept;

	static constexpr uint32_t RemoteStatusTimeout = 2000;

	CanAddress boardAddress;
	HeaterMode lastMode;
	uint8_t averagePwm;
	TuningState tuningState;
	float lastTemperature;
	uint32_t whenLastStatusReceived;

	// Variables used only during tuning
	static uint32_t timeSetHeating;											// When we turned on the heater at the start of auto tuning
	static float currentCoolingRate;
	static unsigned int tuningCyclesDone;
	static bool newTuningResult;
};

#endif

#endif /* SRC_HEATING_REMOTEHEATER_H_ */
