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

	virtual void SetIREmitter(bool on) const noexcept = 0;
	virtual uint16_t GetRawReading() const noexcept = 0;
	virtual void SetProbing(bool isProbing) const noexcept = 0;
	virtual GCodeResult AppendPinNames(const StringRef& str) const noexcept = 0;
	virtual GCodeResult Configure(GCodeBuffer& gb, const StringRef& reply, bool& seen) THROWS_GCODE_EXCEPTION;		// 'seen' is an in-out parameter
	virtual GCodeResult SendProgram(const uint32_t zProbeProgram[], size_t len, const StringRef& reply) noexcept;

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
	void SetProbingAway(const bool probingAway) noexcept { misc.parts.probingAway = probingAway; }

	int GetReading() const noexcept;
	int GetSecondaryValues(int& v1) const noexcept;

	GCodeResult HandleG31(GCodeBuffer& gb, const StringRef& reply) THROWS_GCODE_EXCEPTION;
	void SetTriggerHeight(float height) noexcept { triggerHeight = height; }
	void SetSaveToConfigOverride() noexcept { misc.parts.saveToConfigOverride = true; }

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
				invertReading : 1,			// true if we need to invert the reading
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
	void SetProbing(bool isProbing) const noexcept override { }
	GCodeResult AppendPinNames(const StringRef& str) const noexcept override { return GCodeResult::ok; }

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
	void SetProbing(bool isProbing) const noexcept override { }
	GCodeResult AppendPinNames(const StringRef& str) const noexcept override { return GCodeResult::ok; }

private:
};

#endif /* SRC_ZPROBE_H_ */
