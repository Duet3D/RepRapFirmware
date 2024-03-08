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
	GCodeResult Configure(const CanMessageGenericParser& parser, const StringRef& reply) noexcept override;
#endif

	const uint8_t GetNumAdditionalOutputs() const noexcept override { return 2; }
	TemperatureError GetAdditionalOutput(float& t, uint8_t outputNumber) noexcept override;
	void Poll() noexcept override;
	const char *GetShortSensorType() const noexcept override { return TypeName; }

	static constexpr const char *TypeName = "bme280";

private:
	TemperatureError bme280_init() noexcept;
	TemperatureError bme280_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len) const noexcept;
	TemperatureError bme280_set_reg(uint8_t reg_addr, uint8_t reg_data) const noexcept;
	TemperatureError bme280_set_sensor_settings(uint8_t desired_settings) const noexcept;
	TemperatureError bme280_set_sensor_mode(uint8_t sensor_mode) const noexcept;
	TemperatureError bme280_get_sensor_mode(uint8_t *sensor_mode) const noexcept;
	TemperatureError get_calib_data() noexcept;
	TemperatureError bme280_soft_reset() const noexcept;
	TemperatureError set_filter_standby_settings(uint8_t desired_settings, const struct bme280_settings *settings) const noexcept;
	TemperatureError write_power_mode(uint8_t sensor_mode) const noexcept;
	TemperatureError put_device_to_sleep() const noexcept;
	TemperatureError reload_device_settings(const struct bme280_settings *settings) const noexcept;
	TemperatureError bme280_get_sensor_data() noexcept;
	TemperatureError set_osr_settings(uint8_t desired_settings, const struct bme280_settings *settings) const noexcept;
	TemperatureError set_osr_humidity_settings(const struct bme280_settings *settings) const noexcept;
	TemperatureError set_osr_press_temp_settings(uint8_t desired_settings, const struct bme280_settings *settings) const noexcept;
	float compensate_temperature(const struct bme280_uncomp_data *uncomp_data) noexcept;
	float compensate_pressure(const struct bme280_uncomp_data *uncomp_data) const noexcept;
	float compensate_humidity(const struct bme280_uncomp_data *uncomp_data) const noexcept;
	void bme280_compensate_data(const bme280_uncomp_data *uncomp_data) noexcept;
	void parse_temp_press_calib_data(const uint8_t *reg_data) noexcept;
	void parse_humidity_calib_data(const uint8_t *reg_data) noexcept;
	void bme280_parse_sensor_data(const uint8_t *reg_data, struct bme280_uncomp_data *uncomp_data) noexcept;
	static bool are_settings_changed(uint8_t sub_settings, uint8_t desired_settings) noexcept;
	GCodeResult FinishConfiguring(bool changed, const StringRef& reply) noexcept;

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

	static constexpr const char *TypeName = "bmepressure";
};

// This class represents a DHT humidity sensor
class BME280HumiditySensor : public AdditionalOutputSensor
{
public:
	BME280HumiditySensor(unsigned int sensorNum) noexcept;
	~BME280HumiditySensor() noexcept;

	const char *GetShortSensorType() const noexcept override { return TypeName; }

	static constexpr const char *TypeName = "bmehumidity";
};

#endif	// SUPPORT_BME280

#endif /* SRC_HEATING_SENSORS_BME280_H_ */
