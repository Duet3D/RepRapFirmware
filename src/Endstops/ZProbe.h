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
	ZProbe(unsigned int num, ZProbeType p_type);

	virtual void SetIREmitter(bool on) const = 0;
	virtual uint16_t GetRawReading() const = 0;
	virtual void SetProbing(bool isProbing) const = 0;
	virtual GCodeResult AppendPinNames(const StringRef& str) const = 0;
	virtual GCodeResult Configure(GCodeBuffer& gb, const StringRef& reply, bool& seen);		// 'seen' is an in-out parameter
	virtual GCodeResult SendProgram(const uint32_t zProbeProgram[], size_t len, const StringRef& reply);

	EndStopHit Stopped() const override;
	EndstopHitDetails CheckTriggered(bool goingSlow) override;
	bool Acknowledge(EndstopHitDetails what) override;

	void SetDefaults();

	ZProbeType GetProbeType() const { return type; }
	float GetXOffset() const { return xOffset; }
	float GetYOffset() const { return yOffset; }
	float GetConfiguredTriggerHeight() const { return triggerHeight; }
	float GetActualTriggerHeight() const;
	float GetDiveHeight() const { return diveHeight; }
	float GetStartingHeight() const { return diveHeight + GetActualTriggerHeight(); }
	float GetProbingSpeed() const { return probeSpeed; }
	float GetTravelSpeed() const { return travelSpeed; }
	float GetRecoveryTime() const { return recoveryTime; }
	float GetTolerance() const { return tolerance; }
	bool GetTurnHeatersOff() const { return misc.parts.turnHeatersOff; }
	bool GetSaveToConfigOverride() const { return misc.parts.saveToConfigOverride; }
	int GetAdcValue() const { return adcValue; }
	unsigned int GetMaxTaps() const { return misc.parts.maxTaps; }

	int GetReading() const;
	int GetSecondaryValues(int& v1, int& v2);

	GCodeResult HandleG31(GCodeBuffer& gb, const StringRef& reply);
	void SetTriggerHeight(float height) { triggerHeight = height; }
	void SetSaveToConfigOverride() { misc.parts.saveToConfigOverride = true; }

#if HAS_MASS_STORAGE
	bool WriteParameters(FileStore *f, unsigned int probeNumber) const;
#endif

	static constexpr unsigned int MaxTapsLimit = 31;	// must be low enough to fit in the maxTaps field

protected:
	uint8_t number;
	ZProbeType type;
	int8_t sensor;					// the sensor number used for temperature calibration
	int16_t adcValue;				// the target ADC value, after inversion if enabled
	union
	{
		struct
		{
			uint16_t maxTaps : 5,		// maximum probes at each point
				invertReading : 1,		// true if we need to invert the reading
				turnHeatersOff : 1,		// true to turn heaters off while probing
				saveToConfigOverride : 1; // true if the trigger height should be saved to config-override.g
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
	void* operator new(size_t sz) { return Allocate<MotorStallZProbe>(); }
	void operator delete(void* p) { Release<MotorStallZProbe>(p); }

	MotorStallZProbe(unsigned int num) : ZProbe(num, ZProbeType::zMotorStall) { }
	~MotorStallZProbe() override { }
	void SetIREmitter(bool on) const override { }
	uint16_t GetRawReading() const override { return 4000; }
	void SetProbing(bool isProbing) const override { }
	GCodeResult AppendPinNames(const StringRef& str) const override { return GCodeResult::ok; }

private:
};

// Dummy Z probes only exist locally
class DummyZProbe final : public ZProbe
{
public:
	void* operator new(size_t sz) { return Allocate<DummyZProbe>(); }
	void operator delete(void* p) { Release<DummyZProbe>(p); }

	DummyZProbe(unsigned int num) : ZProbe(num, ZProbeType::none) { }
	~DummyZProbe() override { }
	void SetIREmitter(bool on) const override { }
	uint16_t GetRawReading() const override { return 4000; }
	void SetProbing(bool isProbing) const override { }
	GCodeResult AppendPinNames(const StringRef& str) const override { return GCodeResult::ok; }

private:
};

#endif /* SRC_ZPROBE_H_ */
