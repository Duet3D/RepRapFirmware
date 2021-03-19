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
	TmcDriverTemperatureSensor(unsigned int sensorNum, unsigned int chan) noexcept;

	int GetSmartDriversChannel() const noexcept override { return (int) channel; }		// Get the smart drivers channel that this sensor monitors, or -1 if it doesn't
	void Poll() noexcept override;
	const char *GetShortSensorType() const noexcept override;

	static constexpr const char *PrimaryTypeName = "drivers";
#if defined(DUET_NG) || defined(PCCB_10)
	static constexpr const char *DuexTypeName = "drivers-duex";
	static constexpr const char *DuexTypeShortName = "driversduex";
#endif

private:
	unsigned int channel;
};

#endif

#endif /* SRC_HEATING_SENSORS_TMCDRIVERTEMPERATURESENSOR_H_ */
