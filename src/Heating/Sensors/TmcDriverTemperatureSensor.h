/*
 * TmcDriverTemperatureSensor.h
 *
 *  Created on: 8 Jun 2017
 *      Author: David
 */

#ifndef SRC_HEATING_SENSORS_TMCDRIVERTEMPERATURESENSOR_H_
#define SRC_HEATING_SENSORS_TMCDRIVERTEMPERATURESENSOR_H_

#include "TemperatureSensor.h"

#if HAS_SMART_DRIVERS

class TmcDriverTemperatureSensor : public TemperatureSensor
{
public:
	TmcDriverTemperatureSensor(unsigned int sensorNum, unsigned int chan);
	void Init() override;

	static constexpr const char *PrimaryTypeName = "drivers";
#ifdef DUET_NG
	static constexpr const char *DuexTypeName = "drivers-duex";
#endif

	// Get the smart drivers channel that this sensor monitors, or -1 if it doesn't
	int GetSmartDriversChannel() const override { return (int) channel; }

protected:
	TemperatureError TryGetTemperature(float& t) override;

private:
	unsigned int channel;
};

#endif

#endif /* SRC_HEATING_SENSORS_TMCDRIVERTEMPERATURESENSOR_H_ */
