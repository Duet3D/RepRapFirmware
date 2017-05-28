/*
 * Fan.h
 *
 *  Created on: 29 Jun 2016
 *      Author: David
 */

#ifndef SRC_FAN_H_
#define SRC_FAN_H_

#include "RepRapFirmware.h"

class Fan
{
private:
	float val;
	float minVal;
	float triggerTemperature;
	float lastPwm;
	uint32_t blipTime;						// in milliseconds
	uint32_t blipStartTime;
	uint16_t freq;
	uint16_t heatersMonitored;
	Pin pin;
	bool inverted;
	bool hardwareInverted;
	bool blipping;
	bool thermostatIsOn;					// if it is a thermostatic fan, true if it is turned on

	void Refresh();
	void SetHardwarePwm(float pwmVal);

public:
	bool IsEnabled() const { return pin != NoPin; }
	float GetValue() const { return val; }
	float GetMinValue() const { return minVal; }
	float GetBlipTime() const { return (float)blipTime * MillisToSeconds; }
	float GetPwmFrequency() const { return freq; }
	bool GetInverted() const { return inverted; }
	uint16_t GetHeatersMonitored() const { return heatersMonitored; }
	float GetTriggerTemperature() const { return triggerTemperature; }

	void Init(Pin p_pin, bool hwInverted);
	void SetValue(float speed);
	void SetMinValue(float speed);
	void SetBlipTime(float t);
	void SetInverted(bool inv);
	void SetPwmFrequency(float p_freq);
	void SetTriggerTemperature(float t) { triggerTemperature = t; }
	void SetHeatersMonitored(uint16_t h);
	void Check();
	void Disable();
};

#endif /* SRC_FAN_H_ */
