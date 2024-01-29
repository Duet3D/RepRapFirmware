/*
 * ZProbe.h
 *
 *  Created on: 13 Feb 2018
 *      Author: David
 */

#ifndef SRC_ZPROBE_H_
#define SRC_ZPROBE_H_

#include "Endstop.h"

class ZProbe : public EndstopOrZProbe
{
public:
	ZProbe(unsigned int num, ZProbeType p_type) noexcept;

	virtual void SetIREmitter(bool on) const noexcept = 0;			// caution, this is called from within the tick ISR
	virtual uint32_t GetRawReading() const noexcept = 0;
	virtual bool SetProbing(bool isProbing) noexcept = 0;			// put the probe in the probing state, returning true if successful
	virtual GCodeResult AppendPinNames(const StringRef& str) noexcept = 0;		// not const because it may update the state too
	virtual GCodeResult Configure(GCodeBuffer& gb, const StringRef& reply, bool& seen) THROWS(GCodeException);		// 'seen' is an in-out parameter
	virtual GCodeResult SendProgram(const uint32_t zProbeProgram[], size_t len, const StringRef& reply) noexcept;
	virtual GCodeResult HandleG31(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);		// this is overridden in class RemoteZProbe

#if SUPPORT_SCANNING_PROBES
	// The following should only be called for scanning Z probes, so for other types they return these default values
	virtual GCodeResult GetCalibratedReading(float& val) const noexcept { return GCodeResult::error; }
	virtual float GetLatestHeight() const noexcept { return 0.0; }

	// The following should never be called for a non-scanning probe, so by default we just return error with no message
	virtual GCodeResult CalibrateDriveLevel(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException) { return GCodeResult::error; }
#endif

#if SUPPORT_CAN_EXPANSION
	// Process a remote input change that relates to this Z probe
	virtual void HandleRemoteInputChange(CanAddress src, uint8_t handleMinor, bool newState, uint32_t reading) noexcept { }

	// Process a remote reading that relates to this Z probe
	virtual void UpdateRemoteReading(CanAddress src, uint8_t handleMinor, uint32_t reading) noexcept { }
#endif

	bool Stopped() const noexcept override;
	EndstopHitDetails CheckTriggered() noexcept override;
	bool Acknowledge(EndstopHitDetails what) noexcept override;

	void SetDefaults() noexcept;

	ZProbeType GetProbeType() const noexcept { return type; }
	float GetOffset(size_t axisNumber) const noexcept { return offsets[axisNumber]; }
	float GetConfiguredTriggerHeight() const noexcept { return -offsets[Z_AXIS]; }
	float GetActualTriggerHeight() const noexcept { return actualTriggerHeight; }
	float GetDiveHeight(int tapsDone) const noexcept;
	float GetStartingHeight(bool firstTap, float previousHeightError = 0.0) const noexcept;
	float GetProbingSpeed(int tapsDone) const noexcept { return probeSpeeds[(tapsDone < 0) ? 0 : 1]; }
	float GetScanningSpeed() const noexcept { return probeSpeeds[2]; }
	float FastThenSlowProbing() const noexcept { return probeSpeeds[1] < probeSpeeds[0]; }
	float GetTravelSpeed() const noexcept { return travelSpeed; }
	float GetRecoveryTime() const noexcept { return recoveryTime; }
	float GetTolerance() const noexcept { return tolerance; }
	float GetLastStoppedHeight() const noexcept { return lastStopHeight; }
	bool GetTurnHeatersOff() const noexcept { return misc.parts.turnHeatersOff; }
	bool GetSaveToConfigOverride() const noexcept { return misc.parts.saveToConfigOverride; }
	int32_t GetTargetAdcValue() const noexcept { return targetAdcValue; }
	unsigned int GetMaxTaps() const { return misc.parts.maxTaps; }
	int32_t GetReading() const noexcept;
	int32_t GetSecondaryValues(int32_t& v1) const noexcept;
	bool IsDeployedByUser() const noexcept { return isDeployedByUser; }

	void PrepareForUse(const bool probingAway) noexcept;
	void SetTriggerHeight(float height) noexcept { offsets[Z_AXIS] = -height; }
	void SetSaveToConfigOverride() noexcept { misc.parts.saveToConfigOverride = true; }
	void SetDeployedByUser(bool b) noexcept { isDeployedByUser = b; }
	void SetLastStoppedHeight(float h) noexcept;

#if SUPPORT_SCANNING_PROBES
	// Scanning Z probe support
	bool IsScanning() const noexcept { return type == ZProbeType::scanningAnalog; }			// this is currently the only type of scanning probe we support
	float GetScanningHeight() const noexcept;
	GCodeResult SetScanningCoefficients(float aParam, float bParam, float cParam) noexcept;
	GCodeResult ReportScanningCoefficients(const StringRef& reply) noexcept;
	void CalibrateScanningProbe(const int32_t calibrationReadings[], size_t numCalibrationReadingsTaken, float heightChangePerPoint, const StringRef& reply) noexcept;
	float ConvertReadingToHeightDifference(int32_t reading) const noexcept;
#endif

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
	bool WriteParameters(FileStore *f, unsigned int probeNumber) const noexcept;
#endif

	static constexpr unsigned int MaxTapsLimit = 31;			// must be low enough to fit in the maxTaps field

protected:
	DECLARE_OBJECT_MODEL_WITH_ARRAYS

	float GetTriggerHeightCompensation() const noexcept;		// return the amount by which the trigger height is increased by temperature compensation

	int32_t targetAdcValue;					// the target ADC value, after inversion if enabled
	union
	{
		struct
		{
			uint16_t maxTaps : 5,			// maximum probes at each point
				probingAway : 1,			// true if we are probing away, i.e. until contact lost
				turnHeatersOff : 1,			// true to turn heaters off while probing
				saveToConfigOverride : 1;	// true if the trigger height should be saved to config-override.g
		} parts;
		uint16_t all;
	} misc;
	float offsets[MaxAxes];				// the offset of the probe relative to the print head. The Z offset is the negation of the trigger height.
	float calibTemperature;				// the temperature at which we did the calibration
	float temperatureCoefficients[2];	// the variation of height with bed temperature and with the square of temperature
	float diveHeights[2];				// the dive heights we use when probing in mm
	float probeSpeeds[3];				// the speeds of probing in mm per step clock. First is probing speed, second is travel speed, third is scanning speed.
	float travelSpeed;					// the speed at which we travel to the probe point in mm per step clock
	float recoveryTime;					// Z probe recovery time
	float tolerance;					// maximum difference between probe heights when doing >1 taps
	float actualTriggerHeight;			// the actual trigger height of the probe, taking account of the temperature coefficient
	float lastStopHeight;				// the height at which the last G30 probe move stopped

#if SUPPORT_SCANNING_PROBES
	// Scanning support
	float scanCoefficients[4];
	bool isCalibrated = false;
#endif

	uint8_t number;
	ZProbeType type;
	int8_t sensor;						// the sensor number used for temperature calibration
	bool isDeployedByUser;				// true if the user has used the M401 command to deploy this probe and not sent M402 to retract it
};

// MotorStall Z probes have no port, also in a CAN environment the local and remote proxy versions are the same
class MotorStallZProbe final : public ZProbe
{
public:
	DECLARE_FREELIST_NEW_DELETE(MotorStallZProbe)

	MotorStallZProbe(unsigned int num) noexcept : ZProbe(num, ZProbeType::zMotorStall) { }
	~MotorStallZProbe() override { }

	void SetIREmitter(bool on) const noexcept override { }
	uint32_t GetRawReading() const noexcept override { return 4000; }
	bool SetProbing(bool isProbing) noexcept override { return true; }
	GCodeResult AppendPinNames(const StringRef& str) noexcept override { return GCodeResult::ok; }

private:
};

// Dummy Z probes only exist locally
class DummyZProbe final : public ZProbe
{
public:
	DECLARE_FREELIST_NEW_DELETE(DummyZProbe)

	DummyZProbe(unsigned int num) noexcept : ZProbe(num, ZProbeType::none) { }
	~DummyZProbe() noexcept override { }

	void SetIREmitter(bool on) const noexcept override { }
	uint32_t GetRawReading() const noexcept override { return 4000; }
	bool SetProbing(bool isProbing) noexcept override { return true; }
	GCodeResult AppendPinNames(const StringRef& str) noexcept override { return GCodeResult::ok; }

private:
};

#endif /* SRC_ZPROBE_H_ */
