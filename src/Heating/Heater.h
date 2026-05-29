/*
 * Heater.h
 *
 *  Created on: 24 Jul 2019
 *      Author: David
 */

#ifndef SRC_HEATING_HEATER_H_
#define SRC_HEATING_HEATER_H_

#include <RepRapFirmware.h>
#include <General/NamedEnum.h>
#include "FOPDT.h"
#include "HeaterMonitor.h"
#include <ObjectModel/ObjectModel.h>
#include <Math/DeviationAccumulator.h>
#include <RRF3Common.h>

#if SUPPORT_CAN_EXPANSION
# include "CanId.h"
#endif

#define TUNE_WITH_HALF_FAN	0

class HeaterMonitor;
struct CanMessageHeaterTuningReport;
struct CanHeaterReport;

#if SUPPORT_REMOTE_COMMANDS
struct CanMessageHeaterModelV3;
struct CanMessageSetHeaterTemperatureV1;
struct CanMessageSetHeaterMonitors;
struct CanMessageHeaterTuningCommand;
struct CanMessageSetHeaterFaultDetectionParameters;
struct CanMessageHeaterFeedForwardV1;
class CanMessageBuffer;
#endif

// Enumeration to describe the status of a heater. Note that the web interface returns the numerical values, so don't change them.
NamedEnum(HeaterStatus, uint8_t, off, standby, active, fault, tuning, offline);

class Heater INHERIT_OBJECT_MODEL
{
public:
	explicit Heater(unsigned int num) noexcept;
	virtual ~Heater() noexcept override;
	Heater(const Heater &_ecv_from) = delete;

	// Configuration methods
	virtual GCodeResult ConfigurePortAndSensor(const char *_ecv_array portName, PwmFrequency freq, unsigned int sn, const StringRef& reply) = 0;
	virtual GCodeResult SetPwmFrequency(PwmFrequency freq, const StringRef& reply) = 0;
	virtual GCodeResult ReportDetails(const StringRef& reply) const noexcept = 0;

	virtual float GetTemperature() const noexcept = 0;						// Get the current temperature and error status
	virtual float GetAveragePWM() const noexcept = 0;						// Return the running average PWM to the heater. Answer is a fraction in [0, 1].
	virtual GCodeResult ResetFault(const StringRef& reply) noexcept = 0;	// Reset a fault condition - only call this if you know what you are doing
	virtual void SwitchOff() noexcept;
	virtual void Spin() noexcept = 0;
	virtual void Suspend(bool sus) noexcept = 0;							// Suspend the heater to conserve power or while doing Z probing
	virtual float GetAccumulator() const noexcept = 0;						// Get the inertial term accumulator
	virtual void SetFanFeedForwardPwm(float fanPwm) noexcept = 0;
	virtual GCodeResult SetDefaultModel(HeaterFunction func) noexcept = 0;	// set a default model depending on the heater type

#if SUPPORT_CAN_EXPANSION
	virtual bool IsLocal() const noexcept = 0;
	virtual void UpdateRemoteStatus(CanAddress src, const CanHeaterReport& report) noexcept = 0;
	virtual void UpdateHeaterTuning(CanAddress src, const CanMessageHeaterTuningReport& msg) noexcept = 0;
#endif

	HeaterStatus GetStatus() const noexcept;								// Get the status of the heater
	unsigned int GetHeaterNumber() const noexcept { return heaterNumber; }
	void SetTemperature(float t, bool activeNotStandby) THROWS(GCodeException);
	float GetActiveTemperature() const noexcept { return activeTemperature; }
	float GetStandbyTemperature() const noexcept { return standbyTemperature; }
	GCodeResult SetActiveOrStandby(bool setActive, const StringRef& reply) noexcept;	// Switch from idle to active or standby
	GCodeResult StartAutoTune(GCodeBuffer& gb, const StringRef& reply, FansBitmap fans) THROWS(GCodeException);
																			// Start an auto tune cycle for this heater
	void GetAutoTuneStatus(const StringRef& reply) const noexcept;			// Get the auto tune status or last result
	GCodeResult ConfigureFaultDetectionParameters(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);
	GCodeResult ConfigureMonitor(GCodeBuffer &gb, const StringRef &reply) THROWS(GCodeException);

	float GetHighestTemperatureLimit() const noexcept;
	float GetLowestTemperatureLimit() const noexcept;						// Get the lowest temperature limit

	const FopDt& GetModel() const noexcept { return model; }				// Get the process model
	GCodeResult SetOrReportModel(unsigned int heater, GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);
	void SetExtrusionFeedForward(float pwmBoost, float tempBoost) noexcept;

#if SUPPORT_REMOTE_COMMANDS
	virtual GCodeResult TuningCommand(const CanMessageHeaterTuningCommand& msg, const StringRef& reply) noexcept = 0;
	GCodeResult SetModel(unsigned int heater, const CanMessageHeaterModelV3& msg, const StringRef& reply) noexcept;
	GCodeResult SetTemperature(const CanMessageSetHeaterTemperatureV1& msg, const StringRef& reply) noexcept;
	GCodeResult SetFaultDetectionParameters(const CanMessageSetHeaterFaultDetectionParameters& msg, const StringRef& reply) noexcept;
	GCodeResult SetHeaterMonitors(const CanMessageSetHeaterMonitors& msg, const StringRef& reply) noexcept;
	virtual GCodeResult ApplyFeedForward(const CanMessageHeaterFeedForwardV1& msg, const StringRef& reply) noexcept = 0;
	uint8_t GetModeByte() const { return (uint8_t)GetMode(); }
	virtual void SetDefaultHeaterModel(CanMessageBuffer& buf) noexcept = 0;	// set and return the default heater model
#endif

	bool IsHeaterEnabled() const noexcept									// Is this heater enabled?
		{ return model.IsEnabled(); }

	void ClearModelAndMonitors() noexcept;

	HeaterFunction GetFunction() const noexcept { return function; }
	void SetFunction(HeaterFunction func) noexcept;
	void SetAsToolHeater() noexcept { SetFunction(HeaterFunction::tool); }
	void SetAsBedHeater() noexcept { SetFunction(HeaterFunction::bed); }
	void SetAsChamberHeater() noexcept { SetFunction(HeaterFunction::chamber); }

	bool IsCoolingDevice() const noexcept { return model.IsInverted(); }

protected:
	DECLARE_OBJECT_MODEL_WITH_ARRAYS

	struct HeaterParameters
	{
		float heatingRate;
		float coolingRate;
		float deadTime;
		float gain;
		unsigned int numCycles;

		float GetNormalGain() const noexcept { return heatingRate/coolingRate; }
	};

	virtual void ResetHeater() noexcept;
	virtual HeaterMode GetMode() const noexcept = 0;
	virtual GCodeResult SwitchOn(const StringRef& reply) noexcept = 0;
	virtual GCodeResult UpdateRemoteModel(const StringRef& reply) noexcept = 0;
	virtual GCodeResult UpdateFaultDetectionParameters(const StringRef& reply) noexcept = 0;
	virtual GCodeResult UpdateHeaterMonitors(const StringRef& reply) noexcept = 0;
	virtual GCodeResult StartAutoTune(const StringRef& reply, bool seenA, float ambientTemp) noexcept = 0;
	virtual void ApplyExtrusionFeedForward() noexcept = 0;

	int GetSensorNumber() const noexcept { return sensorNumber; }
	void SetSensorNumber(int sn) noexcept;
	float GetMaxTemperatureExcursion() const noexcept { return maxTempExcursion; }
	float GetMaxHeatingFaultTime() const noexcept { return maxHeatingFaultTime; }
	float GetMaxPwmFaultTime() const noexcept { return maxPwmFaultTime; }
	uint32_t GetMaxBadTemperatureCount() const noexcept { return maxBadTemperatureCount; }
	float GetTargetTemperature() const noexcept { return (active) ? activeTemperature : standbyTemperature; }

	GCodeResult SetModel(float hr, float bcr, float fcr, float coolingRateExponent, float td, float maxPwm, float voltage, bool usePid, bool inverted, const StringRef& reply) noexcept;
																	// set the process model
	void ReportTuningUpdate() noexcept;								// tell the user what's happening
	void CalculateModel(HeaterParameters& params) noexcept;			// calculate G, td and tc from the accumulated readings
	void SetAndReportModelAfterTuning(bool usingFans) noexcept;

	HeaterMonitor monitors[MaxMonitorsPerHeater];					// embedding them in the Heater uses less memory than dynamic allocation
	volatile float lastFanPwm;										// The fan PWM when we last calculated heater feedforward for the fan
	volatile float extrusionPwmBoost;								// The value of extrusion PWM boost to apply
	volatile float extrusionTemperatureBoost;						// the amount of extrusion temperature boost we are currently applying
	float previousExtrusionPwmBoost;								// The previoius value of extrusion boost we applied

	bool tuned = false;												// true if tuning was successful

	// Constants used during heater tuning
	static constexpr uint32_t TempSettleTimeout = 20000;			// how long we allow the initial temperature to settle
	static constexpr unsigned int TuningHeaterMinIdleCycles = 3;	// minimum number of idle cycles after heating up, including the initial overshoot and cool down
	static constexpr unsigned int TuningHeaterMaxIdleCycles = 10;
	static constexpr unsigned int MinTuningHeaterCycles = 5;
	static constexpr unsigned int MaxTuningHeaterCycles = 25;

	static constexpr float MinTuningHeaterPwm = 0.1;
	static constexpr float MinTuningHysteresis = 1.0;
	static constexpr float DefaultTuningHysteresis = 5.0;
	static constexpr float MaxTuningHysteresis = 20.0;
	static constexpr float MinTuningFanPwm = 0.1;
	static constexpr float DefaultTuningFanPwm = 0.7;
	static constexpr float TuningPeakTempDrop = 2.0;				// must be well below TuningHysteresis
	static constexpr float HeaterSettledCoolingTimeRatio = 0.93;

	// Variables used during heater tuning
	static float tuningPwm;											// the PWM to use, 0..1
	static float tuningTargetTemp;									// the target temperature
	static float tuningHysteresis;
	static float tuningFanPwm;

	static DeviationAccumulator tuningStartTemp;					// the temperature when we turned on the heater
	static uint32_t tuningBeginTime;								// when we started the tuning process
	static DeviationAccumulator dHigh;
	static DeviationAccumulator dLow;
	static DeviationAccumulator tOn;
	static DeviationAccumulator tOff;
	static DeviationAccumulator heatingRateAcc;
	static DeviationAccumulator coolingRateAcc;
	static DeviationAccumulator tuningVoltage;						// sum of the voltage readings we take during the heating phase

	static uint32_t lastOffTime;
	static uint32_t lastOnTime;
	static float peakTemp;											// max or min temperature
	static uint32_t peakTime;										// the time at which we recorded peakTemp
	static float afterPeakTemp;										// temperature after max from which we start timing the cooling rate
	static uint32_t afterPeakTime;									// the time at which we recorded afterPeakTemp
	static float lastCoolingRate;
	static FansBitmap tuningFans;
	static unsigned int tuningPhase;
	static uint8_t idleCyclesDone;
	static bool tuningQuietMode;

	static HeaterParameters fanOffParams, fanOnParams;

	static void ClearCounters() noexcept;

	FopDt model;
	float maxHeatingFaultTime = DefaultMaxHeatingFaultTime;				// how long a heater fault is permitted to persist before a heater fault is raised
	float maxPwmFaultTime = DefaultMaxPwmFaultTime;						// how long a heater pwm fault is permitted to persist before a heater fault is raised

private:
	static const char *_ecv_array const TuningPhaseText[];

	unsigned int heaterNumber;
	int sensorNumber = -1;												// the sensor number used by this heater
	float activeTemperature = 0.0;										// the required active temperature
	float standbyTemperature = 0.0;										// the required standby temperature
	float maxTempExcursion = DefaultMaxTempExcursion;					// the maximum temperature excursion permitted while maintaining the setpoint
	uint32_t maxBadTemperatureCount = DefaultMaxBadTemperatureCount;	// the number of consecutive bad sensor readings we allow before raising a fault

	HeaterFunction function = HeaterFunction::tool;						// what type of heater this is
	bool active = false;												// are we active or standby?
	bool usingFeedForward = false;										// true if we have ever applied feedforward even if the amounts are zero
	bool modelSetByUser = false;
	bool monitorsSetByUser = false;
};

#endif /* SRC_HEATING_HEATER_H_ */
