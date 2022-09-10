/*
 * BME280.h
 *
 *  Created on: 10 Sept 2022
 *      Author: David
 */

#ifndef SRC_HEATING_SENSORS_BME280_H_
#define SRC_HEATING_SENSORS_BME280_H_

#include "SpiTemperatureSensor.h"

#if SUPPORT_BME280

#include "AdditionalOutputSensor.h"
#include "bme280_defs.h"

class BME280TemperatureSensor : public SpiTemperatureSensor
{
public:
	BME280TemperatureSensor(unsigned int sensorNum) noexcept;

	GCodeResult Configure(GCodeBuffer& gb, const StringRef& reply, bool& changed) override THROWS(GCodeException);

#if SUPPORT_REMOTE_COMMANDS
	GCodeResult Configure(const CanMessageGenericParser& parser, const StringRef& reply) noexcept override; // configure the sensor from M308 parameters
#endif

	void Poll() noexcept override;
	const char *GetShortSensorType() const noexcept override { return TypeName; }

	static constexpr const char *TypeName = "bme280temperature";

private:
	int8_t bme280_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len) noexcept;
	int8_t bme280_set_regs(uint8_t *reg_addr, const uint8_t *reg_data, uint8_t len) noexcept;
	int8_t get_calib_data() noexcept;
	void bme280_compensate_data(uint8_t sensor_comp, const bme280_uncomp_data *uncomp_data) noexcept;
	float compensate_temperature(const struct bme280_uncomp_data *uncomp_data) noexcept;
	float compensate_pressure(const struct bme280_uncomp_data *uncomp_data) const noexcept;
	float compensate_humidity(const struct bme280_uncomp_data *uncomp_data) const noexcept;
	void parse_temp_press_calib_data(const uint8_t *reg_data) noexcept;
	void parse_humidity_calib_data(const uint8_t *reg_data) noexcept;
	int8_t bme280_soft_reset() noexcept;
	int8_t bme280_get_sensor_data(uint8_t sensor_comp, struct bme280_data *comp_data) noexcept;
	void bme280_parse_sensor_data(const uint8_t *reg_data, struct bme280_uncomp_data *uncomp_data) noexcept;

	bme280_dev dev;
    float compPressure;			    /*< Compensated pressure */
    float compTemperature;    		/*< Compensated temperature */
    float compHumidity;    			/*< Compensated humidity */
};

// This class represents a DHT humidity sensor
class BME280PressureSensor : public AdditionalOutputSensor
{
public:
	BME280PressureSensor(unsigned int sensorNum) noexcept;
	~BME280PressureSensor() noexcept;

	const char *GetShortSensorType() const noexcept override { return TypeName; }

	static constexpr const char *TypeName = "bme280pressure";
};

// This class represents a DHT humidity sensor
class BME280HumiditySensor : public AdditionalOutputSensor
{
public:
	BME280HumiditySensor(unsigned int sensorNum) noexcept;
	~BME280HumiditySensor() noexcept;

	const char *GetShortSensorType() const noexcept override { return TypeName; }

	static constexpr const char *TypeName = "bme280humidity";
};

#endif	// SUPPORT_BME280

#endif /* SRC_HEATING_SENSORS_BME280_H_ */
