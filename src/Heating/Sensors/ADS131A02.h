/*
 * ADS131A02.h
 *
 *  Created on: 7 Oct 2024
 *      Author: David
 */

#ifndef SRC_HEATING_SENSORS_ADS131A02_H_
#define SRC_HEATING_SENSORS_ADS131A02_H_

#include "SpiTemperatureSensor.h"

#if SUPPORT_SPI_SENSORS && SUPPORT_ADS131A02

class ADS131A02 : public SpiTemperatureSensor
{
public:
	explicit ADS131A02(unsigned int sensorNum) noexcept;
	GCodeResult Configure(GCodeBuffer& gb, const StringRef& reply, bool& changed) THROWS(GCodeException) override;

#if SUPPORT_REMOTE_COMMANDS
	GCodeResult Configure(const CanMessageGenericParser& parser, const StringRef& reply) noexcept override; // configure the sensor from M308 parameters
#endif

	void Poll() noexcept override;
	const char *_ecv_array GetShortSensorType() const noexcept override { return TypeName; }

	static constexpr const char *_ecv_array TypeName = "ads131";

private:
	static SensorTypeDescriptor typeDescriptor;

	TemperatureError TryGetLinearAdcTemperature(float& t) noexcept;
	GCodeResult FinishConfiguring(bool changed, const StringRef& reply) noexcept;
	void CalcDerivedParameters() noexcept;

	// Configurable parameters
	float readingAtMin = DefaultReadingAtMin;
	float readingAtMax = DefaultReadingAtmax;

	static constexpr float DefaultReadingAtMin = 0.0;
	static constexpr float DefaultReadingAtmax = 100.0;

};

#endif

#endif /* SRC_HEATING_SENSORS_ADS131A02_H_ */
