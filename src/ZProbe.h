/*
 * ZProbe.h
 *
 *  Created on: 13 Feb 2018
 *      Author: David
 */

#ifndef SRC_ZPROBE_H_
#define SRC_ZPROBE_H_

#include "RepRapFirmware.h"

enum class ZProbeType : uint8_t
{
	none = 0,
	analog = 1,
	dumbModulated = 2,
	alternateAnalog = 3,
	endstopSwitch = 4,
	digital = 5,
	e1Switch_obsolete = 6,
	zSwitch_obsolete = 7,
	unfilteredDigital = 8,
	blTouch = 9,
	zMotorStall = 10,
	numTypes = 11					// must be 1 higher than the last type
};

class ZProbe
{
public:
	float xOffset, yOffset;			// the offset of the probe relative to the print head
	float triggerHeight;			// the nozzle height at which the target ADC value is returned
	float calibTemperature;			// the temperature at which we did the calibration
	float temperatureCoefficient;	// the variation of height with bed temperature
	float diveHeight;				// the dive height we use when probing
	float probeSpeed;				// the initial speed of probing
	float travelSpeed;				// the speed at which we travel to the probe point
	float recoveryTime;				// Z probe recovery time
	float tolerance;				// maximum difference between probe heights when doing >1 taps
	int16_t adcValue;				// the target ADC value, after inversion if enabled
	uint16_t maxTaps : 5,			// maximum probes at each point
			invertReading : 1,		// true if we need to invert the reading
			turnHeatersOff : 1,		// true to turn heaters off while probing
			saveToConfigOverride : 1, // true if the trigger height should be saved to config-override.g
			inputChannel : 4;		// input channel, use when the selected Z probe type is a switch

	static constexpr unsigned int MaxTapsLimit = 31;	// must be low enough to fit in the maxTaps field

	void Init(float h);
	float GetStopHeight(float temperature) const;
	bool WriteParameters(FileStore *f, unsigned int probeType) const;
};

#endif /* SRC_ZPROBE_H_ */
