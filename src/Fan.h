/*
 * Fan.h
 *
 *  Created on: 29 Jun 2016
 *      Author: David
 */

#ifndef SRC_FAN_H_
#define SRC_FAN_H_

#include "RepRapFirmware.h"

class GCodeBuffer;

class Fan
{
public:
	typedef uint32_t HeatersMonitoredBitmap;				// needs to be wide enough for 8 real heaters + 10 virtual heaters

	// Set or report the parameters for this fan
	// If 'mCode' is an M-code used to set parameters for the current kinematics (which should only ever be 106 or 107)
	// then search for parameters used to configure the fan. If any are found, perform appropriate actions and return true.
	// If errors were discovered while processing parameters, put an appropriate error message in 'reply' and set 'error' to true.
	// If no relevant parameters are found, print the existing ones to 'reply' and return false.
	bool Configure(unsigned int mcode, int fanNum, GCodeBuffer& gb, const StringRef& reply, bool& error);
	bool IsConfigured() const { return isConfigured && IsEnabled(); }

	bool IsEnabled() const { return pin != NoPin; }
	float GetConfiguredPwm() const { return val; }			// returns the configured PWM. Actual PWM may be different, e.g. due to blipping or for thermostatic fans.

	void Init(Pin p_pin, bool hwInverted);
	void SetPwm(float speed);
	bool HasMonitoredHeaters() const { return heatersMonitored != 0; }
	void SetHeatersMonitored(HeatersMonitoredBitmap h);
	bool Check();											// update the fan PWM returning true if it is a thermostatic fan that is on
	void Disable();
	bool WriteSettings(FileStore *f, size_t fanNum) const;	// save the settings of this fan if it isn't thermostatic

private:

	float val;
	float lastVal;
	float minVal;
	float triggerTemperatures[2];
	float lastPwm;
	uint32_t blipTime;						// in milliseconds
	uint32_t blipStartTime;
	HeatersMonitoredBitmap heatersMonitored;
	PwmFrequency freq;
	Pin pin;
	bool isConfigured;
	bool inverted;
	bool hardwareInverted;
	bool blipping;

	void Refresh();
	void SetHardwarePwm(float pwmVal);
};

#endif /* SRC_FAN_H_ */
