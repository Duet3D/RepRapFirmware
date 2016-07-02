/*
 * Fan.h
 *
 *  Created on: 29 Jun 2016
 *      Author: David
 */

#ifndef SRC_FAN_H_
#define SRC_FAN_H_

#include "Types.h"

class Fan
{
private:
	float val;
	float triggerTemperature;
	uint16_t freq;
	uint16_t heatersMonitored;
	Pin pin;
	bool inverted;
	bool hardwareInverted;

	void Refresh();
	void SetHardwarePwm(float pwmVal);

public:
	float GetValue() const { return val; }
	float GetPwmFrequency() const { return freq; }
	bool GetInverted() const { return inverted; }
	uint16_t GetHeatersMonitored() const { return heatersMonitored; }
	float GetTriggerTemperature() const { return triggerTemperature; }

	void Init(Pin p_pin, bool hwInverted);
	void SetValue(float speed);
	void SetInverted(bool inv);
	void SetPwmFrequency(float p_freq);
	void SetTriggerTemperature(float t) { triggerTemperature = t; }
	void SetHeatersMonitored(uint16_t h);
	void Check();
};

#endif /* SRC_FAN_H_ */
