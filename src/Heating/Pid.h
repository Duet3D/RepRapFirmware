/*
 * Pid.h
 *
 *  Created on: 21 Jul 2016
 *      Author: David
 */

#ifndef SRC_PID_H_
#define SRC_PID_H_

/**
 * This class implements a PID controller for the heaters
 */

#include "RepRapFirmware.h"
#include "FOPDT.h"
#include "TemperatureError.h"

class HeaterProtection;

class PID
{
	enum class HeaterMode : uint8_t
	{
		// The order of these is important because we test "mode > HeatingMode::suspended" to determine whether the heater is active
		// and "mode >= HeatingMode::off" to determine whether the heater is eitehr active or suspended
		fault,
		off,
		suspended,
		heating,
		cooling,
		stable,
		// All states from here onwards must be PID tuning states because function IsTuning assumes that
		tuning0,
		tuning1,
		tuning2,
		tuning3,
		lastTuningMode = tuning3
	};

	static const size_t NumPreviousTemperatures = 4; // How many samples we average the temperature derivative over

public:

	PID(Platform& p, int8_t h);
	void Init(float pGain, float pTc, float pTd, bool usePid, bool inverted);	// (Re)Set everything to start
	void Reset();
	void Spin();									// Called in a tight loop to keep things running
	void SetActiveTemperature(float t);
	float GetActiveTemperature() const;
	void SetStandbyTemperature(float t);
	float GetStandbyTemperature() const;
	void SetHeaterProtection(HeaterProtection *h);
	float GetHighestTemperatureLimit() const;		// Get the highest temperature limit
	float GetLowestTemperatureLimit() const;		// Get the lowest temperature limit
	void Activate();								// Switch from idle to active
	void Standby();									// Switch from active to idle
	bool Active() const;							// Are we active?
	void SwitchOff();								// Not even standby - all heater power off
	bool SwitchedOff() const;						// Are we switched off?
	bool CheckProtection() const;					// Check heater protection elements and return true if everything is good
	bool FaultOccurred() const;						// Has a heater fault occurred?
	void ResetFault();								// Reset a fault condition - only call this if you know what you are doing
	float GetTemperature() const;					// Get the current temperature
	float GetAveragePWM() const;					// Return the running average PWM to the heater. Answer is a fraction in [0, 1].
	uint32_t GetLastSampleTime() const;				// Return when the temp sensor was last sampled
	float GetAccumulator() const;					// Return the integral accumulator
	void StartAutoTune(float targetTemp, float maxPwm, const StringRef& reply);	// Start an auto tune cycle for this PID
	bool IsTuning() const;
	void GetAutoTuneStatus(const StringRef& reply);	// Get the auto tune status or last result

	const FopDt& GetModel() const					// Get the process model
		{ return model; }

	bool SetModel(float gain, float tc, float td, float maxPwm, float voltage, bool usePid, bool inverted, PwmFrequency pwmFreq);	// Set the process model

	bool IsHeaterSignalInverted() const				// Is the PWM output signal inverted?
		{ return invertPwmSignal; }
	void SetHeaterSignalInverted(bool inverted)		// Set PWM output signal inversion
		{ invertPwmSignal = inverted; }

	bool IsHeaterEnabled() const					// Is this heater enabled?
		{ return model.IsEnabled(); }

	void GetFaultDetectionParameters(float& pMaxTempExcursion, float& pMaxFaultTime) const
		{ pMaxTempExcursion = maxTempExcursion; pMaxFaultTime = maxHeatingFaultTime; }

	void SetFaultDetectionParameters(float pMaxTempExcursion, float pMaxFaultTime)
		{ maxTempExcursion = pMaxTempExcursion; maxHeatingFaultTime = pMaxFaultTime; }

	void SetM301PidParameters(const M301PidParameters& params)
		{ model.SetM301PidParameters(params); }

	void Suspend(bool sus);							// Suspend the heater to conserve power or while doing Z probing

private:

	void SwitchOn();								// Turn the heater on and set the mode
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

	Platform& platform;								// The instance of the class that is the RepRap hardware
	HeaterProtection *heaterProtection;				// The first element of assigned heater protection items
	float activeTemperature;						// The required active temperature
	float standbyTemperature;						// The required standby temperature
	float maxTempExcursion;							// The maximum temperature excursion permitted while maintaining the setpoint
	float maxHeatingFaultTime;						// How long a heater fault is permitted to persist before a heater fault is raised
	float temperature;								// The current temperature
	float previousTemperatures[NumPreviousTemperatures]; // The temperatures of the previous NumDerivativeSamples measurements, used for calculating the derivative
	size_t previousTemperatureIndex;				// Which slot in previousTemperature we fill in next
	FopDt model;									// The process model and PID parameters
	float iAccumulator;								// The integral PID component
	float lastPwm;									// The last PWM value we output, before scaling by kS
	float averagePWM;								// The running average of the PWM, after scaling.
	uint32_t timeSetHeating;						// When we turned on the heater
	uint32_t lastSampleTime;						// Time when the temperature was last sampled by Spin()

	uint16_t heatingFaultCount;						// Count of questionable heating behaviours

	int8_t heater;									// The index of our heater
	uint8_t previousTemperaturesGood;				// Bitmap indicating which previous temperature were good readings
	HeaterMode mode;								// Current state of the heater
	bool invertPwmSignal;							// Invert the final PWM output signal (same behaviour as with HEAT_ON in earlier firmware versions)
	bool active;									// Are we active or standby?
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


inline bool PID::Active() const
{
	return active;
}

inline float PID::GetActiveTemperature() const
{
	return activeTemperature;
}

inline float PID::GetStandbyTemperature() const
{
	return standbyTemperature;
}

inline float PID::GetTemperature() const
{
	return temperature;
}

inline bool PID::FaultOccurred() const
{
	return mode == HeaterMode::fault;
}

inline bool PID::SwitchedOff() const
{
	return mode == HeaterMode::off;
}

inline uint32_t PID::GetLastSampleTime() const
{
	return lastSampleTime;
}

inline float PID::GetAccumulator() const
{
	return iAccumulator;
}

inline bool PID::IsTuning() const
{
	return mode >= HeaterMode::tuning0;
}

#endif /* SRC_PID_H_ */
