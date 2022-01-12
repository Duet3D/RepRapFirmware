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
struct CanMessageHeaterModelNewNew;
struct CanMessageSetHeaterTemperature;
struct CanMessageSetHeaterMonitors;
struct CanMessageHeaterTuningCommand;
#endif

// Enumeration to describe the status of a heater. Note that the web interface returns the numerical values, so don't change them.
NamedEnum(HeaterStatus, uint8_t, off, standby, active, fault, tuning, offline);

class Heater INHERIT_OBJECT_MODEL
{
public:
	Heater(unsigned int num) noexcept;
	virtual ~Heater() noexcept;
	Heater(const Heater&) = delete;

	// Configuration methods
	virtual GCodeResult ConfigurePortAndSensor(const char *portName, PwmFrequency freq, unsigned int sn, const StringRef& reply) = 0;
	virtual GCodeResult SetPwmFrequency(PwmFrequency freq, const StringRef& reply) = 0;
	virtual GCodeResult ReportDetails(const StringRef& reply) const noexcept = 0;

	virtual float GetTemperature() const noexcept = 0;					// Get the current temperature and error status
	virtual float GetAveragePWM() const noexcept = 0;					// Return the running average PWM to the heater. Answer is a fraction in [0, 1].
	virtual GCodeResult ResetFault(const StringRef& reply) noexcept = 0;	// Reset a fault condition - only call this if you know what you are doing
	virtual void SwitchOff() noexcept = 0;
	virtual void Spin() noexcept = 0;
	virtual void Suspend(bool sus) noexcept = 0;						// Suspend the heater to conserve power or while doing Z probing
	virtual float GetAccumulator() const noexcept = 0;					// Get the inertial term accumulator
	virtual void FeedForwardAdjustment(float fanPwmChange, float extrusionChange) noexcept = 0;
	virtual void SetExtrusionFeedForward(float pwm) noexcept = 0;

#if SUPPORT_CAN_EXPANSION
	virtual bool IsLocal() const noexcept = 0;
	virtual void UpdateRemoteStatus(CanAddress src, const CanHeaterReport& report) noexcept = 0;
	virtual void UpdateHeaterTuning(CanAddress src, const CanMessageHeaterTuningReport& msg) noexcept = 0;
#endif

	HeaterStatus GetStatus() const noexcept;							// Get the status of the heater
	unsigned int GetHeaterNumber() const noexcept { return heaterNumber; }
	const char *GetSensorName() const noexcept;							// Get the name of the sensor for this heater, or nullptr if it hasn't been named
	void SetTemperature(float t, bool activeNotStandby) THROWS(GCodeException);
	float GetActiveTemperature() const noexcept { return activeTemperature; }
	float GetStandbyTemperature() const noexcept { return standbyTemperature; }
	GCodeResult SetActiveOrStandby(bool setActive, const StringRef& reply) noexcept;	// Switch from idle to active or standby
	GCodeResult StartAutoTune(GCodeBuffer& gb, const StringRef& reply, FansBitmap fans) THROWS(GCodeException);
																		// Start an auto tune cycle for this heater
	void GetAutoTuneStatus(const StringRef& reply) const noexcept;		// Get the auto tune status or last result

	void GetFaultDetectionParameters(float& pMaxTempExcursion, float& pMaxFaultTime) const noexcept
		{ pMaxTempExcursion = maxTempExcursion; pMaxFaultTime = maxHeatingFaultTime; }
	GCodeResult SetFaultDetectionParameters(float pMaxTempExcursion, float pMaxFaultTime, const StringRef& reply) noexcept;

	GCodeResult ConfigureMonitor(GCodeBuffer &gb, const StringRef &reply) THROWS(GCodeException);

	float GetHighestTemperatureLimit() const noexcept;
	float GetLowestTemperatureLimit() const noexcept;					// Get the lowest temperature limit

	const FopDt& GetModel() const noexcept { return model; }			// Get the process model
	GCodeResult SetOrReportModel(unsigned int heater, GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);

#if SUPPORT_REMOTE_COMMANDS
	virtual GCodeResult TuningCommand(const CanMessageHeaterTuningCommand& msg, const StringRef& reply) noexcept = 0;
	GCodeResult SetModel(unsigned int heater, const CanMessageHeaterModelNewNew& msg, const StringRef& reply) noexcept;
	GCodeResult SetTemperature(const CanMessageSetHeaterTemperature& msg, const StringRef& reply) noexcept;
	GCodeResult SetHeaterMonitors(const CanMessageSetHeaterMonitors& msg, const StringRef& reply) noexcept;
#endif

	bool IsHeaterEnabled() const noexcept								// Is this heater enabled?
		{ return model.IsEnabled(); }

	void SetM301PidParameters(const M301PidParameters& params) noexcept
		{ model.SetM301PidParameters(params); }

	void ClearModelAndMonitors() noexcept;
	void SetAsToolHeater() noexcept;
	void SetAsBedOrChamberHeater() noexcept;

#if SUPPORT_REMOTE_COMMANDS
	uint8_t GetModeByte() const { return (uint8_t)GetMode(); }
#endif

protected:
	DECLARE_OBJECT_MODEL
	OBJECT_MODEL_ARRAY(monitors)

	struct HeaterParameters
	{
		float heatingRate;
		float coolingRate;
		float deadTime;
		float gain;
		unsigned int numCycles;

		float GetNormalGain() const noexcept { return heatingRate/coolingRate; }
	};

	virtual void ResetHeater() noexcept = 0;
	virtual HeaterMode GetMode() const noexcept = 0;
	virtual GCodeResult SwitchOn(const StringRef& reply) noexcept = 0;
	virtual GCodeResult UpdateModel(const StringRef& reply) noexcept = 0;
	virtual GCodeResult UpdateFaultDetectionParameters(const StringRef& reply) noexcept = 0;
	virtual GCodeResult UpdateHeaterMonitors(const StringRef& reply) noexcept = 0;
	virtual GCodeResult StartAutoTune(const StringRef& reply, bool seenA, float ambientTemp) noexcept = 0;

	int GetSensorNumber() const noexcept { return sensorNumber; }
	void SetSensorNumber(int sn) noexcept;
	float GetMaxTemperatureExcursion() const noexcept { return maxTempExcursion; }
	float GetMaxHeatingFaultTime() const noexcept { return maxHeatingFaultTime; }
	float GetTargetTemperature() const noexcept { return (active) ? activeTemperature : standbyTemperature; }
	bool IsBedOrChamber() const noexcept { return isBedOrChamber; }

	GCodeResult SetModel(float hr, float bcr, float fcr, float coolingRateExponent, float td, float maxPwm, float voltage, bool usePid, bool inverted, const StringRef& reply) noexcept;
															// set the process model
	void ReportTuningUpdate() noexcept;						// tell the user what's happening
	void CalculateModel(HeaterParameters& params) noexcept;	// calculate G, td and tc from the accumulated readings
	void SetAndReportModelAfterTuning(bool usingFans) noexcept;

	HeaterMonitor monitors[MaxMonitorsPerHeater];			// embedding them in the Heater uses less memory than dynamic allocation
	bool tuned;												// true if tuning was successful

	// Constants used during heater tuning
	static constexpr uint32_t TempSettleTimeout = 20000;			// how long we allow the initial temperature to settle
	static constexpr unsigned int TuningHeaterMinIdleCycles = 3;	// minimum number of idle cycles after heating up, including the initial overshoot and cool down
	static constexpr unsigned int TuningHeaterMaxIdleCycles = 10;
	static constexpr unsigned int MinTuningHeaterCycles = 5;
	static constexpr unsigned int MaxTuningHeaterCycles = 25;
	static constexpr float DefaultTuningHysteresis = 5.0;
	static constexpr float TuningPeakTempDrop = 2.0;		// must be well below TuningHysteresis
	static constexpr float FeedForwardMultiplier = 1.3;		// how much we over-compensate feedforward to allow for heat reservoirs during tuning
	static constexpr float HeaterSettledCoolingTimeRatio = 0.93;

	// Variables used during heater tuning
	static float tuningPwm;									// the PWM to use, 0..1
	static float tuningTargetTemp;							// the target temperature
	static float tuningHysteresis;
	static float tuningFanPwm;

	static DeviationAccumulator tuningStartTemp;			// the temperature when we turned on the heater
	static uint32_t tuningBeginTime;						// when we started the tuning process
	static DeviationAccumulator dHigh;
	static DeviationAccumulator dLow;
	static DeviationAccumulator tOn;
	static DeviationAccumulator tOff;
	static DeviationAccumulator heatingRate;
	static DeviationAccumulator coolingRate;
	static DeviationAccumulator tuningVoltage;				// sum of the voltage readings we take during the heating phase

	static uint32_t lastOffTime;
	static uint32_t lastOnTime;
	static float peakTemp;									// max or min temperature
	static uint32_t peakTime;								// the time at which we recorded peakTemp
	static float afterPeakTemp;								// temperature after max from which we start timing the cooling rate
	static uint32_t afterPeakTime;							// the time at which we recorded afterPeakTemp
	static float lastCoolingRate;
	static FansBitmap tuningFans;
	static unsigned int tuningPhase;
	static uint8_t idleCyclesDone;

	static HeaterParameters fanOffParams, fanOnParams;

	static void ClearCounters() noexcept;

private:
	static const char* const TuningPhaseText[];

	FopDt model;
	unsigned int heaterNumber;
	int sensorNumber;								// the sensor number used by this heater
	float activeTemperature;						// the required active temperature
	float standbyTemperature;						// the required standby temperature
	float maxTempExcursion;							// the maximum temperature excursion permitted while maintaining the setpoint
	float maxHeatingFaultTime;						// how long a heater fault is permitted to persist before a heater fault is raised

	bool isBedOrChamber;							// true if this was a bed or chamber heater when we were switched on
	bool active;									// are we active or standby?
	bool modelSetByUser;
	bool monitorsSetByUser;
};

#endif /* SRC_HEATING_HEATER_H_ */
