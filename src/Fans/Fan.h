/*
 * Fan.h
 *
 *  Created on: 29 Jun 2016
 *      Author: David
 */

#ifndef SRC_FAN_H_
#define SRC_FAN_H_

#include "RepRapFirmware.h"
#include "Hardware/IoPorts.h"

class GCodeBuffer;

class Fan
{
public:
	typedef uint32_t HeatersMonitoredBitmap;				// needs to be wide enough for 8 real heaters + 10 virtual heaters

	Fan();

	// Set or report the parameters for this fan
	// If 'mCode' is an M-code used to set parameters for the current kinematics (which should only ever be 106 or 107)
	// then search for parameters used to configure the fan. If any are found, perform appropriate actions and return true.
	// If errors were discovered while processing parameters, put an appropriate error message in 'reply' and set 'error' to true.
	// If no relevant parameters are found, print the existing ones to 'reply' and return false.
	bool Configure(unsigned int mcode, size_t fanNum, GCodeBuffer& gb, const StringRef& reply, bool& error);
	bool IsConfigured() const { return isConfigured && IsEnabled(); }

	bool IsEnabled() const { return port.IsValid(); }
	float GetConfiguredPwm() const { return val; }			// returns the configured PWM. Actual PWM may be different, e.g. due to blipping or for thermostatic fans.

	bool AssignPorts(GCodeBuffer& gb, const StringRef& reply);
	bool AssignPorts(const char *pinNames, const StringRef& reply);
	void SetPwm(float speed);
	void SetPwmFrequency(PwmFrequency freq) { port.SetFrequency(freq); }
	bool HasMonitoredHeaters() const { return heatersMonitored != 0; }
	void SetHeatersMonitored(HeatersMonitoredBitmap h);
	const char *GetName() const { return name.c_str(); }
	void AppendPortDetails(const StringRef& str) { port.AppendDetails(str); }

	bool Check();											// update the fan PWM returning true if it is a thermostatic fan that is on
	void Disable();
	bool WriteSettings(FileStore *f, size_t fanNum) const;	// save the settings of this fan if it isn't thermostatic

	// Tacho interface
	int32_t GetRPM() const;
	void Interrupt();

private:
	void Refresh();
	void SetHardwarePwm(float pwmVal);

	PwmPort port;											// port used to control the fan
	IoPort tachoPort;										// port used to read the tacho

	// Variables that control the fan
	float val;
	float lastVal;
	float minVal;
	float maxVal;
	float triggerTemperatures[2];
	float lastPwm;
	uint32_t blipTime;										// in milliseconds
	uint32_t blipStartTime;

	// Variables used to read the tacho
	static constexpr uint32_t fanMaxInterruptCount = 32;	// number of fan interrupts that we average over
	uint32_t fanInterruptCount;								// accessed only in ISR, so no need to declare it volatile
	volatile uint32_t fanLastResetTime;						// time (in step clocks) at which we last reset the interrupt count, accessed inside and outside ISR
	volatile uint32_t fanInterval;							// written by ISR, read outside the ISR

	// More fan control variables
	HeatersMonitoredBitmap heatersMonitored;
	String<MaxFanNameLength> name;
	bool isConfigured;
	bool blipping;
};

#endif /* SRC_FAN_H_ */
