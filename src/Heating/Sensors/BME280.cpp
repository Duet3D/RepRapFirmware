/*
 * BME280.cpp
 *
 *  Created on: 10 Sept 2022
 *      Author: David
 *
 * Portions of this driver are derived from code at https://github.com/BoschSensortec/BME280_driver. The following applies to those portions:
 *
 * Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
 *
 * BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "BME280.h"

#if SUPPORT_BME280

constexpr uint16_t MinimumReadInterval = 1000;			// ms
constexpr uint32_t BME280_Frequency = 4000000;			// maximum for BME280 is 10MHz
constexpr SpiMode BME280_SpiMode = SPI_MODE_0;			// BME280 does mode 0 or mode 3 depending on value of CLK at falling edge of CS
constexpr size_t MaxRegistersToRead = 26;

// BME280 support functions, derived from code at https://github.com/BoschSensortec/BME280_driver

/**\name Internal macros */
/* To identify osr settings selected by user */
constexpr uint8_t OVERSAMPLING_SETTINGS = 0x07;

/* To identify filter and standby settings selected by user */
constexpr uint8_t FILTER_STANDBY_SETTINGS = 0x18;

/*!
 *  @brief This API is the entry point. It reads the chip-id and calibration data from the sensor.
 */
TemperatureError BME280TemperatureSensor::bme280_init() noexcept
{
    /* chip id read try count */
    uint8_t try_count = 5;

	while (try_count)
	{
		/* Read the chip-id of bme280 sensor */
		static_assert(1 <= MaxRegistersToRead);
	    uint8_t chip_id = 0;
		TemperatureError rslt = bme280_get_regs(BME280_CHIP_ID_ADDR, &chip_id, 1);

		/* Check for chip id validity */
		if ((rslt == TemperatureError::ok) && (chip_id == BME280_CHIP_ID))
		{
			dev.chip_id = chip_id;

			/* Reset the sensor */
			rslt = bme280_soft_reset();

			if (rslt == TemperatureError::ok)
			{
				/* Read the calibration data */
				rslt = get_calib_data();
				return rslt;
			}

			break;
		}

		/* Wait for at least 1 ms */
		delay(2);
		--try_count;
	}

    return TemperatureError::hardwareError;
}

/*!
 * @brief This API reads the data from the given register address of the sensor.
 */
TemperatureError BME280TemperatureSensor::bme280_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len) const noexcept
{
	uint8_t addrBuff[MaxRegistersToRead + 1];				// only the first byte is used but the remainder need to be value addresses
	uint8_t dataBuff[MaxRegistersToRead + 1];

	/* Read the data  */
	addrBuff[0] = reg_addr | 0x80;							// for SPI bit 7 is set to read, clear to write
	TemperatureError err = DoSpiTransaction(addrBuff, dataBuff, len + 1);

	/* Check for communication error */
	if (err == TemperatureError::ok)
	{
		memcpy(reg_data, &dataBuff[1], len);
	}
	return err;
}

/*!
 * @brief This API writes the given data to the register address of the sensor.
 */
TemperatureError BME280TemperatureSensor::bme280_set_reg(uint8_t reg_addr, uint8_t reg_data) const noexcept
{
    uint8_t temp_buff[2] = { (uint8_t)(reg_addr & 0x7F), reg_data };
	return DoSpiTransaction(temp_buff, nullptr, 2);
}

/*!
 * @brief This API sets the oversampling, filter and standby duration (normal mode) settings in the sensor.
 */
TemperatureError BME280TemperatureSensor::bme280_set_sensor_settings(uint8_t desired_settings) const noexcept
{
	uint8_t sensor_mode;
	TemperatureError rslt = bme280_get_sensor_mode(&sensor_mode);

	if ((rslt == TemperatureError::ok) && (sensor_mode != BME280_SLEEP_MODE))
	{
		rslt = put_device_to_sleep();
	}

	if (rslt == TemperatureError::ok)
	{
		/* Check if user wants to change oversampling settings */
		if (are_settings_changed(OVERSAMPLING_SETTINGS, desired_settings))
		{
			rslt = set_osr_settings(desired_settings, &dev.settings);
		}

		/* Check if user wants to change filter and/or standby settings */
		if ((rslt == TemperatureError::ok) && are_settings_changed(FILTER_STANDBY_SETTINGS, desired_settings))
		{
			rslt = set_filter_standby_settings(desired_settings, &dev.settings);
		}
	}

    return rslt;
}

/*!
 * @brief This API sets the power mode of the sensor.
 */
TemperatureError BME280TemperatureSensor::bme280_set_sensor_mode(uint8_t sensor_mode) const noexcept
{
	uint8_t last_set_mode;
	TemperatureError rslt = bme280_get_sensor_mode(&last_set_mode);

	/* If the sensor is not in sleep mode put the device to sleep
	 * mode
	 */
	if ((rslt == TemperatureError::ok) && (last_set_mode != BME280_SLEEP_MODE))
	{
		rslt = put_device_to_sleep();
	}

	/* Set the power mode */
	if (rslt == TemperatureError::ok)
	{
		rslt = write_power_mode(sensor_mode);
	}

	return rslt;
}

/*!
 * @brief This API gets the power mode of the sensor.
 */
TemperatureError BME280TemperatureSensor::bme280_get_sensor_mode(uint8_t *sensor_mode) const noexcept
{
	/* Read the power mode register */
	TemperatureError rslt = bme280_get_regs(BME280_PWR_CTRL_ADDR, sensor_mode, 1);

	/* Assign the power mode in the device structure */
	*sensor_mode = BME280_GET_BITS_POS_0(*sensor_mode, BME280_SENSOR_MODE);

	return rslt;
}

/*!
 * @brief This API performs the soft reset of the sensor.
 */
TemperatureError BME280TemperatureSensor::bme280_soft_reset() const noexcept
{
	/* Write the soft reset command to the sensor */
	TemperatureError rslt = bme280_set_reg(BME280_RESET_ADDR, BME280_SOFT_RESET_COMMAND);
	if (rslt == TemperatureError::ok)
	{
		uint8_t status_reg = 0;
		uint8_t try_run = 5;

		/* If NVM not copied yet, Wait for NVM to copy */
		do
		{
			/* As per data sheet - Table 1, startup time is 2 ms. */
			delay(3);
			static_assert(1 <= MaxRegistersToRead);
			rslt = bme280_get_regs(BME280_STATUS_REG_ADDR, &status_reg, 1);
		} while ((rslt == TemperatureError::ok) && (try_run--) && (status_reg & BME280_STATUS_IM_UPDATE));

		if (status_reg & BME280_STATUS_IM_UPDATE)
		{
			rslt = TemperatureError::hardwareError;
		}
	}

	return rslt;
}

/*!
 * @brief This API reads the pressure, temperature and humidity data from the sensor,
 * compensates the data and store it in the bme280_data structure instance passed by the user.
 */
TemperatureError BME280TemperatureSensor::bme280_get_sensor_data() noexcept
{
	/* Array to store the pressure, temperature and humidity data read from the sensor */
	uint8_t reg_data[BME280_P_T_H_DATA_LEN] = { 0 };
	struct bme280_uncomp_data uncomp_data = { 0 };

	/* Read the pressure and temperature data from the sensor */
	static_assert(BME280_P_T_H_DATA_LEN <= MaxRegistersToRead);
	const TemperatureError rslt = bme280_get_regs(BME280_DATA_ADDR, reg_data, BME280_P_T_H_DATA_LEN);

	if (rslt == TemperatureError::ok)
	{
		/* Parse the read data from the sensor */
		bme280_parse_sensor_data(reg_data, &uncomp_data);

		/* Compensate the pressure and/or temperature and/or humidity data from the sensor */
		bme280_compensate_data(&uncomp_data);
	}

	return rslt;
}

/*!
 * @brief This internal API fills the filter settings provided by the user
 * in the data buffer so as to write in the sensor.
 */
static void fill_filter_settings(uint8_t *reg_data, const struct bme280_settings *settings)
{
	*reg_data = BME280_SET_BITS(*reg_data, BME280_FILTER, settings->filter);
}

/*!
 * @brief This internal API fills the standby duration settings provided by
 * the user in the data buffer so as to write in the sensor.
 */
static void fill_standby_settings(uint8_t *reg_data, const struct bme280_settings *settings)
{
	*reg_data = BME280_SET_BITS(*reg_data, BME280_STANDBY, settings->standby_time);
}

/*!
 * @brief This internal API fills the pressure oversampling settings provided by
 * the user in the data buffer so as to write in the sensor.
 */
static void fill_osr_press_settings(uint8_t *reg_data, const struct bme280_settings *settings)
{
	*reg_data = BME280_SET_BITS(*reg_data, BME280_CTRL_PRESS, settings->osr_p);
}

/*!
 * @brief This internal API fills the temperature oversampling settings
 * provided by the user in the data buffer so as to write in the sensor.
 */
static void fill_osr_temp_settings(uint8_t *reg_data, const struct bme280_settings *settings)
{
	*reg_data = BME280_SET_BITS(*reg_data, BME280_CTRL_TEMP, settings->osr_t);
}

/*!
 * @brief This internal API sets the filter and/or standby duration settings
 * in the sensor according to the settings selected by the user.
 */
TemperatureError BME280TemperatureSensor::set_filter_standby_settings(uint8_t desired_settings, const struct bme280_settings *settings) const noexcept
{
	uint8_t reg_data;
	TemperatureError rslt = bme280_get_regs(BME280_CONFIG_ADDR, &reg_data, 1);

	if (rslt == TemperatureError::ok)
	{
		if (desired_settings & BME280_FILTER_SEL)
		{
			fill_filter_settings(&reg_data, settings);
		}

		if (desired_settings & BME280_STANDBY_SEL)
		{
			fill_standby_settings(&reg_data, settings);
		}

		/* Write the oversampling settings in the register */
		rslt = bme280_set_reg(BME280_CONFIG_ADDR, reg_data);
	}

	return rslt;
}

/*!
 * @brief This internal API parse the oversampling(pressure, temperature and humidity),
 * filter and standby duration settings and store in the device structure.
 */
static void parse_device_settings(const uint8_t *reg_data, struct bme280_settings *settings)
{
	settings->osr_h = BME280_GET_BITS_POS_0(reg_data[0], BME280_CTRL_HUM);
	settings->osr_p = BME280_GET_BITS(reg_data[2], BME280_CTRL_PRESS);
	settings->osr_t = BME280_GET_BITS(reg_data[2], BME280_CTRL_TEMP);
	settings->filter = BME280_GET_BITS(reg_data[3], BME280_FILTER);
	settings->standby_time = BME280_GET_BITS(reg_data[3], BME280_STANDBY);
}

/*!
 * @brief This internal API writes the power mode in the sensor.
 */
TemperatureError BME280TemperatureSensor::write_power_mode(uint8_t sensor_mode) const noexcept
{
	/* Variable to store the value read from power mode register */
	uint8_t sensor_mode_reg_val;

	/* Read the power mode register */
	TemperatureError rslt = bme280_get_regs(BME280_PWR_CTRL_ADDR, &sensor_mode_reg_val, 1);

	/* Set the power mode */
	if (rslt == TemperatureError::ok)
	{
		sensor_mode_reg_val = BME280_SET_BITS_POS_0(sensor_mode_reg_val, BME280_SENSOR_MODE, sensor_mode);

		/* Write the power mode in the register */
		rslt = bme280_set_reg(BME280_PWR_CTRL_ADDR, sensor_mode_reg_val);
	}

	return rslt;
}

/*!
 * @brief This internal API puts the device to sleep mode.
 */
TemperatureError BME280TemperatureSensor::put_device_to_sleep() const noexcept
{
	uint8_t reg_data[4];
	TemperatureError rslt = bme280_get_regs(BME280_CTRL_HUM_ADDR, reg_data, 4);

	if (rslt == TemperatureError::ok)
	{
		struct bme280_settings settings;
		parse_device_settings(reg_data, &settings);
		rslt = bme280_soft_reset();

		if (rslt == TemperatureError::ok)
		{
			rslt = reload_device_settings(&settings);
		}
	}

	return rslt;
}

/*!
 * @brief This internal API reloads the already existing device settings in the sensor after soft reset.
 */
TemperatureError BME280TemperatureSensor::reload_device_settings(const struct bme280_settings *settings) const noexcept
{
	TemperatureError rslt = set_osr_settings(BME280_ALL_SETTINGS_SEL, settings);
	if (rslt == TemperatureError::ok)
	{
		rslt = set_filter_standby_settings(BME280_ALL_SETTINGS_SEL, settings);
	}

	return rslt;
}

/*!
 * @brief This internal API is used to compensate the raw temperature data and return the compensated temperature data in float data type.
 */
float BME280TemperatureSensor::compensate_temperature(const bme280_uncomp_data *uncomp_data) noexcept
{
	constexpr float temperature_min = -40;
	constexpr float temperature_max = 85;

	float var1 = ((float)uncomp_data->temperature) / 16384.0 - ((float)dev.calib_data.dig_t1) / 1024.0;
	var1 = var1 * ((float)dev.calib_data.dig_t2);
	float var2 = (((float)uncomp_data->temperature) / 131072.0 - ((float)dev.calib_data.dig_t1) / 8192.0);
	var2 = (var2 * var2) * ((float)dev.calib_data.dig_t3);
	dev.calib_data.t_fine = (int32_t)(var1 + var2);
	float temperature = (var1 + var2) / 5120.0;
	temperature = constrain<float>(temperature, temperature_min, temperature_max);

	return temperature;
}

/*!
 * @brief This internal API is used to compensate the raw pressure data and return the compensated pressure data in float data type.
 */
float BME280TemperatureSensor::compensate_pressure(const bme280_uncomp_data *uncomp_data) const noexcept
{
	constexpr float pressure_min = 30000.0;
	constexpr float pressure_max = 110000.0;

	float var1 = ((float)dev.calib_data.t_fine / 2.0) - 64000.0;
	float var2 = var1 * var1 * ((float)dev.calib_data.dig_p6) / 32768.0;
	var2 = var2 + var1 * ((float)dev.calib_data.dig_p5) * 2.0;
	var2 = (var2 / 4.0) + (((float)dev.calib_data.dig_p4) * 65536.0);
	const float var3 = ((float)dev.calib_data.dig_p3) * var1 * var1 / 524288.0;
	var1 = (var3 + ((float)dev.calib_data.dig_p2) * var1) / 524288.0;
	var1 = (1.0 + var1 / 32768.0) * ((float)dev.calib_data.dig_p1);
	float pressure;

	/* avoid exception caused by division by zero */
	if (var1 > (0.0))
	{
		pressure = 1048576.0 - (float) uncomp_data->pressure;
		pressure = (pressure - (var2 / 4096.0)) * 6250.0 / var1;
		var1 = ((float)dev.calib_data.dig_p9) * pressure * pressure / 2147483648.0;
		var2 = pressure * ((float)dev.calib_data.dig_p8) / 32768.0;
		pressure = pressure + (var1 + var2 + ((float)dev.calib_data.dig_p7)) / 16.0;
		pressure = constrain<float>(pressure, pressure_min, pressure_max);
	}
	else /* Invalid case */
	{
		pressure = pressure_min;
	}

	return pressure * 0.01;				// we want pressure in hectopascals, not pascals
}

/*!
 * @brief This internal API is used to compensate the raw humidity data and return the compensated humidity data in float data type.
 */
float BME280TemperatureSensor::compensate_humidity(const bme280_uncomp_data *uncomp_data) const noexcept
{
	constexpr float humidity_min = 0.0;
	constexpr float humidity_max = 100.0;

	const float var1 = ((float)dev.calib_data.t_fine) - 76800.0;
	const float var2 = (((float)dev.calib_data.dig_h4) * 64.0 + (((float)dev.calib_data.dig_h5) / 16384.0) * var1);
	const float var3 = uncomp_data->humidity - var2;
	const float var4 = ((float)dev.calib_data.dig_h2) / 65536.0;
	const float var5 = (1.0 + (((float)dev.calib_data.dig_h3) / 67108864.0) * var1);
	float var6 = 1.0 + (((float)dev.calib_data.dig_h6) / 67108864.0) * var1 * var5;
	var6 = var3 * var4 * (var5 * var6);
	float humidity = var6 * (1.0 - ((float)dev.calib_data.dig_h1) * var6 / 524288.0);
	humidity = constrain<float>(humidity, humidity_min, humidity_max);
	return humidity;
}

/*!
 *  @brief This API is used to parse the pressure, temperature and humidity data and store it in the bme280_uncomp_data structure instance.
 */
void BME280TemperatureSensor::bme280_parse_sensor_data(const uint8_t *reg_data, bme280_uncomp_data *uncomp_data) noexcept
{
	/* Variables to store the sensor data */
	uint32_t data_xlsb;
	uint32_t data_lsb;
	uint32_t data_msb;

	/* Store the parsed register values for pressure data */
	data_msb = (uint32_t)reg_data[0] << 12;
	data_lsb = (uint32_t)reg_data[1] << 4;
	data_xlsb = (uint32_t)reg_data[2] >> 4;
	uncomp_data->pressure = data_msb | data_lsb | data_xlsb;

	/* Store the parsed register values for temperature data */
	data_msb = (uint32_t)reg_data[3] << 12;
	data_lsb = (uint32_t)reg_data[4] << 4;
	data_xlsb = (uint32_t)reg_data[5] >> 4;
	uncomp_data->temperature = data_msb | data_lsb | data_xlsb;

	/* Store the parsed register values for humidity data */
	data_msb = (uint32_t)reg_data[6] << 8;
	data_lsb = (uint32_t)reg_data[7];
	uncomp_data->humidity = data_msb | data_lsb;
}

/*!
 * @brief This API is used to compensate the pressure and/or temperature and/or humidity data according to the component selected by the user.
 */
void BME280TemperatureSensor::bme280_compensate_data(const bme280_uncomp_data *uncomp_data) noexcept
{
	/* Compensate the temperature data */
	compTemperature = compensate_temperature(uncomp_data);
	/* Compensate the pressure data */
	compPressure = compensate_pressure(uncomp_data);
	/* Compensate the humidity data */
	compHumidity = compensate_humidity(uncomp_data);
}

/*!
 * @brief This internal API sets the oversampling settings for pressure,
 * temperature and humidity in the sensor.
 */
TemperatureError BME280TemperatureSensor::set_osr_settings(uint8_t desired_settings, const bme280_settings *settings) const noexcept
{
	TemperatureError rslt = TemperatureError::ok;

    if (desired_settings & BME280_OSR_HUM_SEL)
    {
        rslt = set_osr_humidity_settings(settings);
    }

    if (desired_settings & (BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL))
    {
        rslt = set_osr_press_temp_settings(desired_settings, settings);
    }

    return rslt;
}

/*!
 * @brief This API sets the humidity oversampling settings of the sensor.
 */
TemperatureError BME280TemperatureSensor::set_osr_humidity_settings(const struct bme280_settings *settings) const noexcept
{
    uint8_t ctrl_hum = settings->osr_h & BME280_CTRL_HUM_MSK;

    /* Write the humidity control value in the register */
    TemperatureError rslt = bme280_set_reg(BME280_CTRL_HUM_ADDR, ctrl_hum);

    /* Humidity related changes will be only effective after a write operation to ctrl_meas register */
    if (rslt == TemperatureError::ok)
    {
        uint8_t ctrl_meas;
        rslt = bme280_get_regs(BME280_CTRL_MEAS_ADDR, &ctrl_meas, 1);
        if (rslt == TemperatureError::ok)
        {
            rslt = bme280_set_reg(BME280_CTRL_MEAS_ADDR, ctrl_meas);
        }
    }

    return rslt;
}

/*!
 * @brief This API sets the pressure and/or temperature oversampling settings in the sensor according to the settings selected by the user. */
TemperatureError BME280TemperatureSensor::set_osr_press_temp_settings(uint8_t desired_settings, const struct bme280_settings *settings) const noexcept
{
    uint8_t reg_data;
    TemperatureError rslt = bme280_get_regs(BME280_CTRL_MEAS_ADDR, &reg_data, 1);

    if (rslt == TemperatureError::ok)
    {
        if (desired_settings & BME280_OSR_PRESS_SEL)
        {
            fill_osr_press_settings(&reg_data, settings);
        }

        if (desired_settings & BME280_OSR_TEMP_SEL)
        {
            fill_osr_temp_settings(&reg_data, settings);
        }

        /* Write the oversampling settings in the register */
        rslt = bme280_set_reg(BME280_CTRL_MEAS_ADDR, reg_data);
    }

    return rslt;
}

/*!
 *  @brief This internal API is used to parse the temperature and pressure calibration data and store it in device structure.
 */
void BME280TemperatureSensor::parse_temp_press_calib_data(const uint8_t *reg_data) noexcept
{
    dev.calib_data.dig_t1 = BME280_CONCAT_BYTES(reg_data[1], reg_data[0]);
    dev.calib_data.dig_t2 = (int16_t)BME280_CONCAT_BYTES(reg_data[3], reg_data[2]);
    dev.calib_data.dig_t3 = (int16_t)BME280_CONCAT_BYTES(reg_data[5], reg_data[4]);
    dev.calib_data.dig_p1 = BME280_CONCAT_BYTES(reg_data[7], reg_data[6]);
    dev.calib_data.dig_p2 = (int16_t)BME280_CONCAT_BYTES(reg_data[9], reg_data[8]);
    dev.calib_data.dig_p3 = (int16_t)BME280_CONCAT_BYTES(reg_data[11], reg_data[10]);
    dev.calib_data.dig_p4 = (int16_t)BME280_CONCAT_BYTES(reg_data[13], reg_data[12]);
    dev.calib_data.dig_p5 = (int16_t)BME280_CONCAT_BYTES(reg_data[15], reg_data[14]);
    dev.calib_data.dig_p6 = (int16_t)BME280_CONCAT_BYTES(reg_data[17], reg_data[16]);
    dev.calib_data.dig_p7 = (int16_t)BME280_CONCAT_BYTES(reg_data[19], reg_data[18]);
    dev.calib_data.dig_p8 = (int16_t)BME280_CONCAT_BYTES(reg_data[21], reg_data[20]);
    dev.calib_data.dig_p9 = (int16_t)BME280_CONCAT_BYTES(reg_data[23], reg_data[22]);
    dev.calib_data.dig_h1 = reg_data[25];
}

/*!
 *  @brief This internal API is used to parse the humidity calibration data and store it in device structure.
 */
void BME280TemperatureSensor::parse_humidity_calib_data(const uint8_t *reg_data) noexcept
{
    dev.calib_data.dig_h2 = (int16_t)BME280_CONCAT_BYTES(reg_data[1], reg_data[0]);
    dev.calib_data.dig_h3 = reg_data[2];
    const int16_t dig_h4_msb = (int16_t)(int8_t)reg_data[3] * 16;
    const int16_t dig_h4_lsb = (int16_t)(reg_data[4] & 0x0F);
    dev.calib_data.dig_h4 = dig_h4_msb | dig_h4_lsb;
    const int16_t dig_h5_msb = (int16_t)(int8_t)reg_data[5] * 16;
    const int16_t dig_h5_lsb = (int16_t)(reg_data[4] >> 4);
    dev.calib_data.dig_h5 = dig_h5_msb | dig_h5_lsb;
    dev.calib_data.dig_h6 = (int8_t)reg_data[6];
}

/*!
 * @brief This internal API is used to identify the settings which the user
 * wants to modify in the sensor.
 */
/*static*/ bool BME280TemperatureSensor::are_settings_changed(uint8_t sub_settings, uint8_t desired_settings) noexcept
{
	return (sub_settings & desired_settings);
}

/*!
 * @brief This internal API reads the calibration data from the sensor, parse it and store in the device structure.
 */
TemperatureError BME280TemperatureSensor::get_calib_data() noexcept
{
    /* Array to store calibration data */
    uint8_t temp_calib_data[BME280_TEMP_PRESS_CALIB_DATA_LEN] = { 0 };

    /* Read the calibration data from the sensor */
	static_assert(BME280_TEMP_PRESS_CALIB_DATA_LEN <= MaxRegistersToRead);
    TemperatureError rslt = bme280_get_regs(BME280_TEMP_PRESS_CALIB_DATA_ADDR, temp_calib_data, BME280_TEMP_PRESS_CALIB_DATA_LEN);

    if (rslt == TemperatureError::ok)
    {
        /* Parse temperature and pressure calibration data and store it in device structure */
        parse_temp_press_calib_data(temp_calib_data);

        /* Read the humidity calibration data from the sensor */
    	static_assert(BME280_HUMIDITY_CALIB_DATA_LEN <= MaxRegistersToRead);
        rslt = bme280_get_regs(BME280_HUMIDITY_CALIB_DATA_ADDR, temp_calib_data, BME280_HUMIDITY_CALIB_DATA_LEN);

        if (rslt == TemperatureError::ok)
        {
            /* Parse humidity calibration data and store it in device structure */
            parse_humidity_calib_data(temp_calib_data);
        }
    }

    return rslt;
}

// BME280TemperatureSensor members

BME280TemperatureSensor::BME280TemperatureSensor(unsigned int sensorNum) noexcept
	: SpiTemperatureSensor(sensorNum, "BME280 temperature", BME280_SpiMode, BME280_Frequency)
{
}

GCodeResult BME280TemperatureSensor::Configure(GCodeBuffer &gb, const StringRef &reply, bool &changed)
{
	if (!ConfigurePort(gb, reply, changed))
	{
		return GCodeResult::error;
	}
	ConfigureCommonParameters(gb, changed);
	return FinishConfiguring(changed, reply);
}

#if SUPPORT_REMOTE_COMMANDS

GCodeResult BME280TemperatureSensor::Configure(const CanMessageGenericParser& parser, const StringRef& reply) noexcept
{
	bool seen = false;
	if (!ConfigurePort(parser, reply, seen))
	{
		return GCodeResult::error;
	}
	ConfigureCommonParameters(parser, seen);
	return FinishConfiguring(seen, reply);
}

#endif

GCodeResult BME280TemperatureSensor::FinishConfiguring(bool changed, const StringRef& reply) noexcept
{
	if (changed)
	{
		// Initialise the sensor
		InitSpi();
		TemperatureError rslt = bme280_init();
		SetResult(0.0, rslt);

		if (rslt == TemperatureError::ok)
		{
			/* Recommended mode of operation: Indoor navigation */
			dev.settings.osr_h = BME280_OVERSAMPLING_1X;
			dev.settings.osr_p = BME280_OVERSAMPLING_16X;
			dev.settings.osr_t = BME280_OVERSAMPLING_2X;
			dev.settings.filter = BME280_FILTER_COEFF_16;
			dev.settings.standby_time = BME280_STANDBY_TIME_62_5_MS;

			uint8_t settings_sel = BME280_OSR_PRESS_SEL;
			settings_sel |= BME280_OSR_TEMP_SEL;
			settings_sel |= BME280_OSR_HUM_SEL;
			settings_sel |= BME280_STANDBY_SEL;
			settings_sel |= BME280_FILTER_SEL;
			rslt = bme280_set_sensor_settings(settings_sel);
			if (rslt == TemperatureError::ok)
			{
				rslt = bme280_set_sensor_mode(BME280_NORMAL_MODE);
			}
		}

		if (rslt != TemperatureError::ok)
		{
			reply.printf("Failed to initialise BME280 sensor: %s", rslt.ToString());
			return GCodeResult::error;
		}
	}
	else
	{
		CopyBasicDetails(reply);
	}
	return GCodeResult::ok;
}

TemperatureError BME280TemperatureSensor::GetAdditionalOutput(float &t, uint8_t outputNumber) noexcept
{
	const auto result = TemperatureSensor::GetLatestTemperature(t);
	switch (outputNumber)
	{
	case 1:
		t = compPressure;
		break;

	case 2:
		t = compHumidity;
		break;

	default:
		t = BadErrorTemperature;
		return TemperatureError::invalidOutputNumber;
	}
	return result;
}

void BME280TemperatureSensor::Poll() noexcept
{
	const auto now = millis();
	if ((now - GetLastReadingTime()) >= MinimumReadInterval)
	{
		if (bme280_get_sensor_data() == TemperatureError::ok)
		{
			SetResult(compTemperature, TemperatureError::ok);
		}
		else
		{
			SetResult(TemperatureError::hardwareError);
		}
	}
}

// Bme280PressureSensor members

BME280PressureSensor::BME280PressureSensor(unsigned int sensorNum) noexcept
	: AdditionalOutputSensor(sensorNum, "BME280-pressure", false)
{
}

BME280PressureSensor::~BME280PressureSensor() noexcept
{
}

// Bme280HumiditySensor members

BME280HumiditySensor::BME280HumiditySensor(unsigned int sensorNum) noexcept
	: AdditionalOutputSensor(sensorNum, "BME280-humidity", false)
{
}

BME280HumiditySensor::~BME280HumiditySensor() noexcept
{
}

#endif

// End
