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

class PID
{
	enum class HeaterMode : uint8_t
	{
		// The order of these is important because we test "mode > HeatingMode::off" to determine whether the heater is active
		fault,
		off,
		heating,
		cooling,
		stable
	};

	static const size_t NumPreviousTemperatures = 4; // How many samples we average the temperature derivative over

public:

    PID(Platform* p, int8_t h);
    void Init();									// (Re)Set everything to start
    void Spin();									// Called in a tight loop to keep things running
    void SetActiveTemperature(float t);
    float GetActiveTemperature() const;
    void SetStandbyTemperature(float t);
    float GetStandbyTemperature() const;
    void Activate();								// Switch from idle to active
    void Standby();									// Switch from active to idle
    bool Active() const;							// Are we active?
    void SwitchOff();								// Not even standby - all heater power off
    bool SwitchedOff() const;						// Are we switched off?
	bool FaultOccurred() const;						// Has a heater fault occurred?
    void ResetFault();								// Reset a fault condition - only call this if you know what you are doing
    float GetTemperature() const;					// Get the current temperature
    float GetAveragePWM() const;					// Return the running average PWM to the heater. Answer is a fraction in [0, 1].
    uint32_t GetLastSampleTime() const;				// Return when the temp sensor was last sampled
    float GetAccumulator() const;					// Return the integral accumulator

private:

    void SwitchOn();								// Turn the heater on and set the mode
    void SetHeater(float power) const;				// Power is a fraction in [0,1]
    TemperatureError ReadTemperature();				// Read and store the temperature of this heater
    float GetExpectedHeatingRate(float temp, float pwm, float& startupTime) const;	// Get the minimum heating rate we expect and the heater startup time

    Platform* platform;								// The instance of the class that is the RepRap hardware
    float activeTemperature;						// The required active temperature
    float standbyTemperature;						// The required standby temperature
    float temperature;								// The current temperature
    float previousTemperatures[NumPreviousTemperatures]; // The temperatures of the previous NumDerivativeSamples measurements, used for calculating the derivative
    size_t previousTemperatureIndex;				// Which slot in previousTemperature we fill in next
    float temp_iState;								// The integral PID component
    float lastPWM;									// The last PWM value we output, before scaling by kS
    float averagePWM;								// The running average of the PWM, after scaling.
    float timeSetHeating;							// When we turned on the heater
	uint32_t lastSampleTime;						// Time when the temperature was last sampled by Spin()

    uint16_t heatingFaultCount;						// Count of questionable heating behaviours

    int8_t heater;									// The index of our heater
	uint8_t previousTemperaturesGood;				// Bitmap indicating which previous temperature were good readings
    HeaterMode mode;								// Current state of the heater
    bool active;									// Are we active or standby?
    uint8_t badTemperatureCount;					// Count of sequential dud readings

    static_assert(sizeof(previousTemperaturesGood) * 8 >= NumPreviousTemperatures, "too few bits in previousTemperaturesGood");
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
	return temp_iState;
}

inline void PID::SetHeater(float power) const
{
	platform->SetHeater(heater, power);
}

#endif /* SRC_PID_H_ */
