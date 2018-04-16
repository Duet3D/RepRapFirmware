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
	e0Switch = 4,
	digital = 5,
	e1Switch = 6,
	zSwitch = 7,
	unfilteredDigital = 8,
	blTouch = 9,
	zMotorStall = 10,
	numTypes = 11					// must be 1 higher than the last type
};

class ZProbe
{
public:
	int32_t adcValue;				// the target ADC value, after inversion if enabled
	float xOffset, yOffset;			// the offset of the probe relative to the print head
	float triggerHeight;			// the nozzle height at which the target ADC value is returned
	float calibTemperature;			// the temperature at which we did the calibration
	float temperatureCoefficient;	// the variation of height with bed temperature
	float diveHeight;				// the dive height we use when probing
	float probeSpeed;				// the initial speed of probing
	float travelSpeed;				// the speed at which we travel to the probe point
	float recoveryTime;				// Z probe recovery time
	float tolerance;				// maximum difference between probe heights when doing >1 taps
	uint8_t maxTaps;				// maximum probes at each point
	bool invertReading;				// true if we need to invert the reading
	bool turnHeatersOff;			// true to turn heaters off while probing

	void Init(float h);
	float GetStopHeight(float temperature) const;
	bool WriteParameters(FileStore *f, unsigned int probeType) const;
};

#endif /* SRC_ZPROBE_H_ */
