/*
 * Pid.h
 *
 *  Created on: 21 Jul 2016
 *      Author: David
 */

#ifndef SRC_LOCALHEATER_H_
#define SRC_LOCALHEATER_H_

/**
 * This class implements a PID controller for the heaters
 */

#include "Heater.h"
#include "FOPDT.h"
#include "TemperatureError.h"
#include <Hardware/IoPorts.h>
#include <GCodes/GCodeResult.h>
#include <Math/DeviationAccumulator.h>

class HeaterMonitor;

class LocalHeater : public Heater
{
	static const size_t NumPreviousTemperatures = 4;		// How many samples we average the temperature derivative over

public:
	struct HeaterParameters
	{
		float heatingRate;
		float coolingRate;
		float deadTime;
		unsigned int numCycles;
	};

	LocalHeater(unsigned int heaterNum) noexcept;
	~LocalHeater() noexcept;

	GCodeResult ConfigurePortAndSensor(const char *portName, PwmFrequency freq, unsigned int sn, const StringRef& reply) override;
	GCodeResult SetPwmFrequency(PwmFrequency freq, const StringRef& reply) noexcept override;
	GCodeResult ReportDetails(const StringRef& reply) const noexcept override;

	void Spin() noexcept override;							// Called in a tight loop to keep things running
	void SwitchOff() noexcept override;						// Not even standby - all heater power off
	GCodeResult ResetFault(const StringRef& reply) noexcept override;	// Reset a fault condition - only call this if you know what you are doing
	float GetTemperature() const noexcept override;			// Get the current temperature
	float GetAveragePWM() const noexcept override;			// Return the running average PWM to the heater. Answer is a fraction in [0, 1].
	float GetAccumulator() const noexcept override;			// Return the integral accumulator
	GCodeResult StartAutoTune(GCodeBuffer& gb, const StringRef& reply, FansBitmap fans) THROWS(GCodeException) override;	// Start an auto tune cycle for this heater
	void GetAutoTuneStatus(const StringRef& reply) const noexcept override;	// Get the auto tune status or last result
	void Suspend(bool sus) noexcept override;				// Suspend the heater to conserve power or while doing Z probing
	void PrintCoolingFanPwmChanged(float pwmChange) noexcept override;

#if SUPPORT_CAN_EXPANSION
	void UpdateRemoteStatus(CanAddress src, const CanHeaterReport& report) noexcept override { }
#endif

protected:
	void ResetHeater() noexcept override;
	HeaterMode GetMode() const noexcept override { return mode; }
	GCodeResult SwitchOn(const StringRef& reply) noexcept override;	// Turn the heater on and set the mode
	GCodeResult UpdateModel(const StringRef& reply) noexcept override;	// Called when the heater model has been changed
	GCodeResult UpdateFaultDetectionParameters(const StringRef& reply) noexcept override { return GCodeResult::ok; }
	GCodeResult UpdateHeaterMonitors(const StringRef& reply) noexcept override { return GCodeResult::ok; }

private:
	void SetHeater(float power) const noexcept;				// Power is a fraction in [0,1]
	TemperatureError ReadTemperature() noexcept;			// Read and store the temperature of this heater
	void DoTuningStep() noexcept;							// Called on each temperature sample when auto tuning
	void CalculateModel(HeaterParameters& params) noexcept;	// Calculate G, td and tc from the accumulated readings
	void SetAndReportModel(bool usingFans) noexcept;
	float GetExpectedHeatingRate() const noexcept;			// Get the minimum heating rate we expect
	void RaiseHeaterFault(const char *format, ...) noexcept;

	PwmPort port;											// The port that drives the heater
	float temperature;										// The current temperature
	float previousTemperatures[NumPreviousTemperatures]; 	// The temperatures of the previous NumDerivativeSamples measurements, used for calculating the derivative
	size_t previousTemperatureIndex;						// Which slot in previousTemperature we fill in next
	float iAccumulator;										// The integral LocalHeater component
	float lastPwm;											// The last PWM value we output, before scaling by kS
	float averagePWM;										// The running average of the PWM, after scaling.
	uint32_t timeSetHeating;								// When we turned on the heater
	uint32_t lastSampleTime;								// Time when the temperature was last sampled by Spin()

	uint16_t heatingFaultCount;								// Count of questionable heating behaviours

	uint8_t previousTemperaturesGood;						// Bitmap indicating which previous temperature were good readings
	HeaterMode mode;										// Current state of the heater
	bool tuned;												// True if tuning was successful
	uint8_t badTemperatureCount;							// Count of sequential dud readings

	static_assert(sizeof(previousTemperaturesGood) * 8 >= NumPreviousTemperatures, "too few bits in previousTemperaturesGood");

	static constexpr unsigned int TuningHeaterMinIdleCycles = 3;	// minimum number of idle cycles after heating up, including the initial overshoot and cool down
	static constexpr unsigned int TuningHeaterMaxIdleCycles = 10;
	static constexpr unsigned int MinTuningHeaterCycles = 5;
	static constexpr unsigned int MaxTuningHeaterCycles = 25;
	static constexpr float TuningHysteresis = 5.0;
	static constexpr float TuningPeakTempDrop = 2.0;		// must be well below TuningHysteresis
	static constexpr float FeedForwardMultiplier = 1.3;		// how much we over-compensate feedforward to allow for heat reservoirs during tuning
	static constexpr float HeaterSettledCoolingTimeRatio = 0.93;
};

#endif /* SRC_LOCALHEATER_H_ */
