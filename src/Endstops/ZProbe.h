/*
 * ZProbe.h
 *
 *  Created on: 13 Feb 2018
 *      Author: David
 */

#ifndef SRC_ZPROBE_H_
#define SRC_ZPROBE_H_

#include "Endstop.h"
#include "GCodes/GCodeResult.h"

class ZProbe : public EndstopOrZProbe
{
public:
	ZProbe(unsigned int num, ZProbeType p_type) noexcept;

	virtual void SetIREmitter(bool on) const noexcept = 0;			// caution, this is called from within the tick ISR
	virtual uint16_t GetRawReading() const noexcept = 0;
	virtual bool SetProbing(bool isProbing) noexcept = 0;			// put the probe in the probing state, returning true if successful
	virtual GCodeResult AppendPinNames(const StringRef& str) noexcept = 0;		// not const because it may update the state too
	virtual GCodeResult Configure(GCodeBuffer& gb, const StringRef& reply, bool& seen) THROWS(GCodeException);		// 'seen' is an in-out parameter
	virtual GCodeResult SendProgram(const uint32_t zProbeProgram[], size_t len, const StringRef& reply) noexcept;

#if SUPPORT_CAN_EXPANSION
	// Process a remote input change that relates to this Z probe
	virtual void HandleRemoteInputChange(CanAddress src, uint8_t handleMinor, bool newState) noexcept { }
#endif

	EndStopHit Stopped() const noexcept override;
	EndstopHitDetails CheckTriggered(bool goingSlow) noexcept override;
	bool Acknowledge(EndstopHitDetails what) noexcept override;

	void SetDefaults() noexcept;

	ZProbeType GetProbeType() const noexcept { return type; }
	float GetXOffset() const noexcept { return xOffset; }
	float GetYOffset() const noexcept { return yOffset; }
	float GetConfiguredTriggerHeight() const noexcept { return triggerHeight; }
	float GetActualTriggerHeight() const noexcept;
	float GetDiveHeight() const noexcept { return diveHeight; }
	float GetStartingHeight() const noexcept { return diveHeight + GetActualTriggerHeight(); }
	float GetProbingSpeed() const noexcept { return probeSpeed; }
	float GetTravelSpeed() const noexcept { return travelSpeed; }
	float GetRecoveryTime() const noexcept { return recoveryTime; }
	float GetTolerance() const noexcept { return tolerance; }
	bool GetTurnHeatersOff() const noexcept { return misc.parts.turnHeatersOff; }
	bool GetSaveToConfigOverride() const noexcept { return misc.parts.saveToConfigOverride; }
	int GetAdcValue() const noexcept { return adcValue; }
	unsigned int GetMaxTaps() const { return misc.parts.maxTaps; }
	int GetReading() const noexcept;
	int GetSecondaryValues(int& v1) const noexcept;
	bool IsDeployedByUser() const noexcept { return isDeployedByUser; }

	void SetProbingAway(const bool probingAway) noexcept { misc.parts.probingAway = probingAway; }
	GCodeResult HandleG31(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);
	void SetTriggerHeight(float height) noexcept { triggerHeight = height; }
	void SetSaveToConfigOverride() noexcept { misc.parts.saveToConfigOverride = true; }
	void SetDeployedByUser(bool b) noexcept { isDeployedByUser = b; }

#if HAS_MASS_STORAGE
	bool WriteParameters(FileStore *f, unsigned int probeNumber) const noexcept;
#endif

	static constexpr unsigned int MaxTapsLimit = 31;	// must be low enough to fit in the maxTaps field

protected:
	DECLARE_OBJECT_MODEL
	OBJECT_MODEL_ARRAY(offsets)
	OBJECT_MODEL_ARRAY(value)

	uint8_t number;
	ZProbeType type;
	int8_t sensor;					// the sensor number used for temperature calibration
	int16_t adcValue;				// the target ADC value, after inversion if enabled
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
	float xOffset, yOffset;			// the offset of the probe relative to the print head
	float triggerHeight;			// the nozzle height at which the target ADC value is returned
	float calibTemperature;			// the temperature at which we did the calibration
	float temperatureCoefficient;	// the variation of height with bed temperature
	float diveHeight;				// the dive height we use when probing
	float probeSpeed;				// the initial speed of probing
	float travelSpeed;				// the speed at which we travel to the probe point
	float recoveryTime;				// Z probe recovery time
	float tolerance;				// maximum difference between probe heights when doing >1 taps

	bool isDeployedByUser;			// true if the user has used the M401 command to deploy this probe and not sent M402 to retract it
};

// MotorStall Z probes have no port, also in a CAN environment the local and remote proxy versions are the same
class MotorStallZProbe final : public ZProbe
{
public:
	void* operator new(size_t sz) noexcept { return FreelistManager::Allocate<MotorStallZProbe>(); }
	void operator delete(void* p) noexcept { FreelistManager::Release<MotorStallZProbe>(p); }

	MotorStallZProbe(unsigned int num) noexcept : ZProbe(num, ZProbeType::zMotorStall) { }
	~MotorStallZProbe() noexcept override { }
	void SetIREmitter(bool on) const noexcept override { }
	uint16_t GetRawReading() const noexcept override { return 4000; }
	bool SetProbing(bool isProbing) noexcept override { return true; }
	GCodeResult AppendPinNames(const StringRef& str) noexcept override { return GCodeResult::ok; }

private:
};

// Dummy Z probes only exist locally
class DummyZProbe final : public ZProbe
{
public:
	void* operator new(size_t sz) noexcept { return FreelistManager::Allocate<DummyZProbe>(); }
	void operator delete(void* p) noexcept { FreelistManager::Release<DummyZProbe>(p); }

	DummyZProbe(unsigned int num) noexcept : ZProbe(num, ZProbeType::none) { }
	~DummyZProbe() noexcept override { }
	void SetIREmitter(bool on) const noexcept override { }
	uint16_t GetRawReading() const noexcept override { return 4000; }
	bool SetProbing(bool isProbing) noexcept override { return true; }
	GCodeResult AppendPinNames(const StringRef& str) noexcept override { return GCodeResult::ok; }

private:
};

#endif /* SRC_ZPROBE_H_ */
