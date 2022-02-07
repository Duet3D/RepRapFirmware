/*
 * ThermocoupleSensor31855.h
 *
 *  Created on: 8 Jun 2017
 *      Author: David
 */

#ifndef SRC_HEATING_THERMOCOUPLESENSOR31855_H_
#define SRC_HEATING_THERMOCOUPLESENSOR31855_H_

#include "SpiTemperatureSensor.h"

#if SUPPORT_SPI_SENSORS

class ThermocoupleSensor31855 : public SpiTemperatureSensor
{
public:
	ThermocoupleSensor31855(unsigned int sensorNum) noexcept;
	GCodeResult Configure(GCodeBuffer& gb, const StringRef& reply, bool& changed) override THROWS(GCodeException);

#if SUPPORT_REMOTE_COMMANDS
	GCodeResult Configure(const CanMessageGenericParser& parser, const StringRef& reply) noexcept override; // configure the sensor from M308 parameters
#endif

	void Poll() noexcept override;
	const char *GetShortSensorType() const noexcept override { return TypeName; }

	static constexpr const char *TypeName = "thermocouplemax31855";

private:
	GCodeResult FinishConfiguring(bool changed, const StringRef& reply) noexcept;
};

#endif // SUPPORT_SPI_SENSORS

#endif /* SRC_HEATING_THERMOCOUPLESENSOR31855_H_ */
