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

class ZProbe final : public EndstopOrZProbe
{
public:
	void* operator new(size_t sz) { return Allocate<ZProbe>(); }
	void operator delete(void* p) { Release<ZProbe>(p); }

	ZProbe();
	~ZProbe() override;

	EndStopHit Stopped() const override;
	EndstopHitDetails CheckTriggered(bool goingSlow) override;
	bool Acknowledge(EndstopHitDetails what) override;

	void SetDefaults();
	bool AssignPorts(GCodeBuffer& gb, const StringRef& reply);
	bool AssignPorts(const char *pinNames, const StringRef& reply);

	ZProbeType GetProbeType() const { return type; }
	float GetXOffset() const { return xOffset; }
	float GetYOffset() const { return yOffset; }
	float GetConfiguredTriggerHeight() const { return triggerHeight; }
	float GetActualTriggerHeight() const;
	float GetDiveHeight() const { return diveHeight; }
	float GetProbingSpeed() const { return probeSpeed; }
	float GetTravelSpeed() const { return travelSpeed; }
	float GetRecoveryTime() const { return recoveryTime; }
	float GetTolerance() const { return tolerance; }
	bool GetTurnHeatersOff() const { return turnHeatersOff; }
	bool GetSaveToConfigOverride() const { return saveToConfigOverride; }
	int GetAdcValue() const { return adcValue; }
	unsigned int GetMaxTaps() const { return maxTaps; }

	int GetReading() const;
	int GetSecondaryValues(int& v1, int& v2);
	uint16_t GetRawReading() const;
	void SetProbing(bool isProbing) const;
	void SetProgramOutput(bool b) const;
	void SetIREmitter(bool on) const;

	GCodeResult HandleG31(GCodeBuffer& gb, const StringRef& reply, bool printDetails);
	GCodeResult HandleM558(GCodeBuffer& gb, const StringRef& reply, unsigned int probeNumber);
	void SetTriggerHeight(float height) { triggerHeight = height; }
	void SetSaveToConfigOverride() { saveToConfigOverride = true; }
	bool WriteParameters(FileStore *f, unsigned int probeType) const;

	static constexpr unsigned int MaxTapsLimit = 31;	// must be low enough to fit in the maxTaps field

private:
	float xOffset, yOffset;			// the offset of the probe relative to the print head
	float triggerHeight;			// the nozzle height at which the target ADC value is returned
	float calibTemperature;			// the temperature at which we did the calibration
	float temperatureCoefficient;	// the variation of height with bed temperature
	float diveHeight;				// the dive height we use when probing
	float probeSpeed;				// the initial speed of probing
	float travelSpeed;				// the speed at which we travel to the probe point
	float recoveryTime;				// Z probe recovery time
	float tolerance;				// maximum difference between probe heights when doing >1 taps
	IoPort inputPort;
	IoPort modulationPort;			// the modulation port we are using
	int16_t adcValue;				// the target ADC value, after inversion if enabled
	uint16_t maxTaps : 5,			// maximum probes at each point
			invertReading : 1,		// true if we need to invert the reading
			turnHeatersOff : 1,		// true to turn heaters off while probing
			saveToConfigOverride : 1; // true if the trigger height should be saved to config-override.g
	ZProbeType type;
	int8_t heater;					// the heater number used for temperature calibration
};

// If this is a dumb modulated IR probe, set the IR LED on or off. Called from the tick ISR, so inlined for speed.
inline void ZProbe::SetIREmitter(bool on) const
{
	if (type == ZProbeType::dumbModulated)
	{
		modulationPort.WriteDigital(on);
	}
}

inline void ZProbe::SetProgramOutput(bool b) const
{
	modulationPort.WriteDigital(b);
}

#endif /* SRC_ZPROBE_H_ */
