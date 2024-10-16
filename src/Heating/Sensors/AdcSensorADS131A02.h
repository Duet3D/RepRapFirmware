/*
 * ADS131A02.h
 *
 *  Created on: 7 Oct 2024
 *      Author: David
 */

#ifndef SRC_HEATING_SENSORS_ADCSENSORADS131A02_H_
#define SRC_HEATING_SENSORS_ADCSENSORADS131A02_H_

#include "SpiTemperatureSensor.h"

#if SUPPORT_SPI_SENSORS && SUPPORT_ADS131A02

class AdcSensorADS131A02 : public SpiTemperatureSensor
{
public:
	explicit AdcSensorADS131A02(unsigned int sensorNum, bool p_24bit) noexcept;
	GCodeResult Configure(GCodeBuffer& gb, const StringRef& reply, bool& changed) THROWS(GCodeException) override;

#if SUPPORT_REMOTE_COMMANDS
	GCodeResult Configure(const CanMessageGenericParser& parser, const StringRef& reply) noexcept override;		// configure the sensor from M308 parameters
#endif

	void Poll() noexcept override;
	const char *_ecv_array GetShortSensorType() const noexcept override { return (use24bitFrames) ? TypeName_24bit : TypeName_16bit; }

	static constexpr const char *_ecv_array TypeName_16bit = "ads131.16b";
	static constexpr const char *_ecv_array TypeName_24bit = "ads131.24b";

private:
	static SensorTypeDescriptor typeDescriptor_16bit;
	static SensorTypeDescriptor typeDescriptor_24bit;

	TemperatureError TryGetLinearAdcTemperature(float& t) noexcept;
	GCodeResult FinishConfiguring(bool changed, const StringRef& reply) noexcept;
	void CalcDerivedParameters() noexcept;

	// Configurable parameters
	float readingAtMin = DefaultReadingAtMin;
	float readingAtMax = DefaultReadingAtmax;

	bool use24bitFrames;

	static constexpr float DefaultReadingAtMin = 0.0;
	static constexpr float DefaultReadingAtmax = 100.0;

};

#endif

#endif /* SRC_HEATING_SENSORS_ADCSENSORADS131A02_H_ */
