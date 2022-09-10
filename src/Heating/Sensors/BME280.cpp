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

const uint32_t BME280_Frequency = 4000000;			// maximum for BME280 is 10MHz
const SpiMode BME280_SpiMode = SPI_MODE_2;			// clock is high when idle, data sampled on rising edge

// BME280 support functions, derived from code at https://github.com/BoschSensortec/BME280_driver

/*!
 * @brief This API reads the data from the given register address of the sensor.
 */
int8_t BME280TemperatureSensor::bme280_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len) noexcept
{
	/* If interface selected is SPI */
	if (true)
	{
		reg_addr = reg_addr | 0x80;
	}

	constexpr size_t MaxRegistersToRead = 26;
	static_assert(MaxRegistersToRead >= BME280_P_T_H_DATA_LEN);
	static_assert(MaxRegistersToRead >= BME280_TEMP_PRESS_CALIB_DATA_LEN);
	static_assert(MaxRegistersToRead >= BME280_HUMIDITY_CALIB_DATA_LEN);
	uint8_t addrBuff[MaxRegistersToRead + 1];				// only the first bytes is used but the remainder need to be value addresses
	uint8_t dataBuff[MaxRegistersToRead + 1];

	/* Read the data  */
	addrBuff[0] = reg_addr;
	TemperatureError err = DoSpiTransaction(addrBuff, dataBuff, len + 1);

	/* Check for communication error */
	if (err == TemperatureError::success)
	{
		memcpy(reg_data, &dataBuff[1], len);
		return BME280_OK;
	}
	else
	{
		return BME280_E_COMM_FAIL;
	}
}

/*!
 * @brief This API writes the given data to the register address
 * of the sensor.
 */
int8_t BME280TemperatureSensor::bme280_set_regs(uint8_t *reg_addr, const uint8_t *reg_data, uint8_t len) noexcept
{
    uint8_t temp_buff[21]; /* Typically not to write more than 10 registers */

    if (len > 10)
    {
        len = 10;
    }

    int8_t rslt = BME280_OK;

    /* Check for arguments validity */
	if (len != 0)
	{
		/* Interleave register address w.r.t data for burst write */
	    for (uint8_t index = 0; index < len; index++)
	    {
	        temp_buff[index * 2] = reg_addr[index] & 0x7F;
	        temp_buff[(index * 2) + 1] = reg_data[index];
	    }

		TemperatureError err = DoSpiTransaction(temp_buff, nullptr, len * 2);

		/* Check for communication error */
		if (err != TemperatureError::success)
		{
			rslt = BME280_E_COMM_FAIL;
		}
	}
	else
	{
		rslt = BME280_E_INVALID_LEN;
	}

    return rslt;
}

/*!
 * @brief This internal API is used to compensate the raw temperature data and
 * return the compensated temperature data in double data type.
 */
float BME280TemperatureSensor::compensate_temperature(const struct bme280_uncomp_data *uncomp_data) noexcept
{
    constexpr float temperature_min = -40;
    constexpr float temperature_max = 85;

    float var1 = ((float)uncomp_data->temperature) / 16384.0 - ((float)dev.calib_data.dig_t1) / 1024.0;
    var1 = var1 * ((float)dev.calib_data.dig_t2);
    float var2 = (((float)uncomp_data->temperature) / 131072.0 - ((float)dev.calib_data.dig_t1) / 8192.0);
    var2 = (var2 * var2) * ((float)dev.calib_data.dig_t3);
    dev.calib_data.t_fine = (int32_t)(var1 + var2);
    float temperature = (var1 + var2) / 5120.0;

    if (temperature < temperature_min)
    {
        temperature = temperature_min;
    }
    else if (temperature > temperature_max)
    {
        temperature = temperature_max;
    }

    return temperature;
}

/*!
 * @brief This internal API is used to compensate the raw pressure data and
 * return the compensated pressure data in double data type.
 */
float BME280TemperatureSensor::compensate_pressure(const struct bme280_uncomp_data *uncomp_data) const noexcept
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

        if (pressure < pressure_min)
        {
            pressure = pressure_min;
        }
        else if (pressure > pressure_max)
        {
            pressure = pressure_max;
        }
    }
    else /* Invalid case */
    {
        pressure = pressure_min;
    }

    return pressure;
}

/*!
 * @brief This internal API is used to compensate the raw humidity data and
 * return the compensated humidity data in double data type.
 */
float BME280TemperatureSensor::compensate_humidity(const struct bme280_uncomp_data *uncomp_data) const noexcept
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

    if (humidity > humidity_max)
    {
        humidity = humidity_max;
    }
    else if (humidity < humidity_min)
    {
        humidity = humidity_min;
    }

    return humidity;
}

/*!
 * @brief This API performs the soft reset of the sensor.
 */
int8_t BME280TemperatureSensor::bme280_soft_reset() noexcept
{
    uint8_t reg_addr = BME280_RESET_ADDR;
    uint8_t status_reg = 0;
    uint8_t try_run = 5;

    /* 0xB6 is the soft reset command */
    uint8_t soft_rst_cmd = BME280_SOFT_RESET_COMMAND;

	/* Write the soft reset command in the sensor */
	int8_t rslt = bme280_set_regs(&reg_addr, &soft_rst_cmd, 1);

	if (rslt == BME280_OK)
	{
		/* If NVM not copied yet, Wait for NVM to copy */
		do
		{
			/* As per data sheet - Table 1, startup time is 2 ms. */
			delay(3);
			rslt = bme280_get_regs(BME280_STATUS_REG_ADDR, &status_reg, 1);
		} while ((rslt == BME280_OK) && (try_run--) && (status_reg & BME280_STATUS_IM_UPDATE));

		if (status_reg & BME280_STATUS_IM_UPDATE)
		{
			rslt = BME280_E_NVM_COPY_FAILED;
		}
	}

    return rslt;
}

/*!
 * @brief This API reads the pressure, temperature and humidity data from the
 * sensor, compensates the data and store it in the bme280_data structure
 * instance passed by the user.
 */
int8_t BME280TemperatureSensor::bme280_get_sensor_data(uint8_t sensor_comp, struct bme280_data *comp_data) noexcept
{
    /* Array to store the pressure, temperature and humidity data read from
     * the sensor
     */
    uint8_t reg_data[BME280_P_T_H_DATA_LEN] = { 0 };
    struct bme280_uncomp_data uncomp_data = { 0 };

	/* Read the pressure and temperature data from the sensor */
	int8_t rslt = bme280_get_regs(BME280_DATA_ADDR, reg_data, BME280_P_T_H_DATA_LEN);

	if (rslt == BME280_OK)
	{
		/* Parse the read data from the sensor */
		bme280_parse_sensor_data(reg_data, &uncomp_data);

		/* Compensate the pressure and/or temperature and/or
		 * humidity data from the sensor
		 */
		bme280_compensate_data(sensor_comp, &uncomp_data);
	}

    return rslt;
}

/*!
 *  @brief This API is used to parse the pressure, temperature and
 *  humidity data and store it in the bme280_uncomp_data structure instance.
 */
void BME280TemperatureSensor::bme280_parse_sensor_data(const uint8_t *reg_data, struct bme280_uncomp_data *uncomp_data) noexcept
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
 * @brief This API is used to compensate the pressure and/or
 * temperature and/or humidity data according to the component selected
 * by the user.
 */
void BME280TemperatureSensor::bme280_compensate_data(uint8_t sensor_comp, const struct bme280_uncomp_data *uncomp_data) noexcept
{
	/* Compensate the temperature data */
	compTemperature = compensate_temperature(uncomp_data);
	/* Compensate the pressure data */
	compPressure = compensate_pressure(uncomp_data);
	/* Compensate the humidity data */
	compHumidity = compensate_humidity(uncomp_data);
}

/*!
 *  @brief This internal API is used to parse the temperature and
 *  pressure calibration data and store it in device structure.
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
 *  @brief This internal API is used to parse the humidity calibration data
 *  and store it in device structure.
 */
void BME280TemperatureSensor::parse_humidity_calib_data(const uint8_t *reg_data) noexcept
{
    int16_t dig_h4_lsb;
    int16_t dig_h4_msb;
    int16_t dig_h5_lsb;
    int16_t dig_h5_msb;

    dev.calib_data.dig_h2 = (int16_t)BME280_CONCAT_BYTES(reg_data[1], reg_data[0]);
    dev.calib_data.dig_h3 = reg_data[2];
    dig_h4_msb = (int16_t)(int8_t)reg_data[3] * 16;
    dig_h4_lsb = (int16_t)(reg_data[4] & 0x0F);
    dev.calib_data.dig_h4 = dig_h4_msb | dig_h4_lsb;
    dig_h5_msb = (int16_t)(int8_t)reg_data[5] * 16;
    dig_h5_lsb = (int16_t)(reg_data[4] >> 4);
    dev.calib_data.dig_h5 = dig_h5_msb | dig_h5_lsb;
    dev.calib_data.dig_h6 = (int8_t)reg_data[6];
}

/*!
 * @brief This internal API reads the calibration data from the sensor, parse
 * it and store in the device structure.
 */
int8_t BME280TemperatureSensor::get_calib_data() noexcept
{
    uint8_t reg_addr = BME280_TEMP_PRESS_CALIB_DATA_ADDR;

    /* Array to store calibration data */
    uint8_t temp_calib_data[BME280_TEMP_PRESS_CALIB_DATA_LEN] = { 0 };

    /* Read the calibration data from the sensor */
    int8_t rslt = bme280_get_regs(reg_addr, temp_calib_data, BME280_TEMP_PRESS_CALIB_DATA_LEN);

    if (rslt == BME280_OK)
    {
        /* Parse temperature and pressure calibration data and store
         * it in device structure
         */
        parse_temp_press_calib_data(temp_calib_data);
        reg_addr = BME280_HUMIDITY_CALIB_DATA_ADDR;

        /* Read the humidity calibration data from the sensor */
        rslt = bme280_get_regs(reg_addr, temp_calib_data, BME280_HUMIDITY_CALIB_DATA_LEN);

        if (rslt == BME280_OK)
        {
            /* Parse humidity calibration data and store it in
             * device structure
             */
            parse_humidity_calib_data(temp_calib_data);
        }
    }

    return rslt;
}

// BME280TemperatureSensor members

BME280TemperatureSensor::BME280TemperatureSensor(unsigned int sensorNum) noexcept
	: SpiTemperatureSensor(sensorNum, "BME280 temperature", BME280_SpiMode, BME280_Frequency)
{
	//TODO
}

GCodeResult BME280TemperatureSensor::Configure(GCodeBuffer &gb, const StringRef &reply, bool &changed)
{
	//TODO
	reply.copy("not implemented");
	return GCodeResult::error;
}

#if SUPPORT_REMOTE_COMMANDS

GCodeResult BME280TemperatureSensor::Configure(const CanMessageGenericParser& parser, const StringRef& reply) noexcept
{
	//TODO
	reply.copy("not implemented");
	return GCodeResult::error;
}

#endif

void BME280TemperatureSensor::Poll() noexcept
{
	// TODO
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
