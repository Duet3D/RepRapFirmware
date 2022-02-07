/*
 * RtdSensor31865.h
 *
 *  Created on: 8 Jun 2017
 *      Author: David
 */

#ifndef SRC_HEATING_RTDSENSOR31865_H_
#define SRC_HEATING_RTDSENSOR31865_H_

#include "SpiTemperatureSensor.h"

#if SUPPORT_SPI_SENSORS

class RtdSensor31865 : public SpiTemperatureSensor
{
public:
	RtdSensor31865(unsigned int sensorNum) noexcept;

	GCodeResult Configure(GCodeBuffer& gb, const StringRef& reply, bool& changed) override THROWS(GCodeException);

#if SUPPORT_REMOTE_COMMANDS
	GCodeResult Configure(const CanMessageGenericParser& parser, const StringRef& reply) noexcept override; // configure the sensor from M308 parameters
#endif

	void Poll() noexcept override;
	const char *GetShortSensorType() const noexcept override { return TypeName; }

	static constexpr const char *TypeName = "rtdmax31865";

private:
	TemperatureError TryInitRtd() const noexcept;
	GCodeResult FinishConfiguring(bool changed, const StringRef& reply) noexcept;

	uint16_t rref;				// reference resistor in ohms
	uint8_t cr0;
};

#endif //SUPPORT_SPI_SENSORS

#endif /* SRC_HEATING_RTDSENSOR31865_H_ */
