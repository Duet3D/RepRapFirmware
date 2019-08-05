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
#include "Hardware/IoPorts.h"
#include "GCodes/GCodeResult.h"

class HeaterProtection;

class LocalHeater : public Heater
{
	static const size_t NumPreviousTemperatures = 4; // How many samples we average the temperature derivative over

public:
	LocalHeater(unsigned int heaterNum);
	LocalHeater(const Heater& h);

	void Spin() override;							// Called in a tight loop to keep things running
	GCodeResult ConfigurePortAndSensor(GCodeBuffer& gb, const StringRef& reply) override;
	void ReleasePort() override;					// If it's a local heater, turn it off and release its port. If it is remote, delete the remote heater.
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
	HeaterMode GetMode() const override { return mode; }
	void SwitchOn() override;						// Turn the heater on and set the mode
	GCodeResult UpdateModel(const StringRef& reply) override;	// Called when the heater model has been changed

private:
	void SetHeater(float power) const;				// Power is a fraction in [0,1]
	TemperatureError ReadTemperature();				// Read and store the temperature of this heater
	void DoTuningStep();							// Called on each temperature sample when auto tuning
	static bool ReadingsStable(size_t numReadings, float maxDiff)
		pre(numReadings >= 2; numReadings <= MaxTuningTempReadings);
	static int GetPeakTempIndex();					// Auto tune helper function
	static int IdentifyPeak(size_t numToAverage);	// Auto tune helper function
	void CalculateModel();							// Calculate G, td and tc from the accumulated readings
	void DisplayBuffer(const char *intro);			// Debug helper
	float GetExpectedHeatingRate() const;			// Get the minimum heating rate we expect

	PwmPort port;									// The port that drives the heater
	float temperature;								// The current temperature
	float previousTemperatures[NumPreviousTemperatures]; // The temperatures of the previous NumDerivativeSamples measurements, used for calculating the derivative
	size_t previousTemperatureIndex;				// Which slot in previousTemperature we fill in next
	float iAccumulator;								// The integral LocalHeater component
	float lastPwm;									// The last PWM value we output, before scaling by kS
	float averagePWM;								// The running average of the PWM, after scaling.
	uint32_t timeSetHeating;						// When we turned on the heater
	uint32_t lastSampleTime;						// Time when the temperature was last sampled by Spin()

	uint16_t heatingFaultCount;						// Count of questionable heating behaviours

	uint8_t previousTemperaturesGood;				// Bitmap indicating which previous temperature were good readings
	HeaterMode mode;								// Current state of the heater
	bool tuned;										// True if tuning was successful
	uint8_t badTemperatureCount;					// Count of sequential dud readings

	static_assert(sizeof(previousTemperaturesGood) * 8 >= NumPreviousTemperatures, "too few bits in previousTemperaturesGood");

	// Variables used during heater tuning
	static const size_t MaxTuningTempReadings = 128; // The maximum number of readings we keep. Must be an even number.

	static float *tuningTempReadings;				// the readings from the heater being tuned
	static float tuningStartTemp;					// the temperature when we turned on the heater
	static float tuningPwm;							// the PWM to use, 0..1
	static float tuningTargetTemp;						// the maximum temperature we are allowed to reach
	static uint32_t tuningBeginTime;				// when we started the tuning process
	static uint32_t tuningPhaseStartTime;			// when we started the current tuning phase
	static uint32_t tuningReadingInterval;			// how often we are sampling, in milliseconds
	static size_t tuningReadingsTaken;				// how many temperature samples we have taken
	static float tuningHeaterOffTemp;				// the temperature when we turned the heater off
	static float tuningPeakTemperature;				// the peak temperature reached, averaged over 3 readings (so slightly less than the true peak)
	static uint32_t tuningHeatingTime;				// how long we had the heating on for
	static uint32_t tuningPeakDelay;				// how many milliseconds the temperature continues to rise after turning the heater off
};

#endif /* SRC_LOCALHEATER_H_ */
