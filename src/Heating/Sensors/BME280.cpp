/*
 * BME280.cpp
 *
 *  Created on: 10 Sept 2022
 *      Author: David
 */

#include "BME280.h"

#if SUPPORT_BME280

// BME280 support functions

const uint32_t BME280_Frequency = 4000000;			// maximum for BME280 is 10MHz
const SpiMode BME280_SpiMode = SPI_MODE_2;			// clock is high when idle, data sampled on rising edge

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
 * @brief This API is used to compensate the pressure and/or
 * temperature and/or humidity data according to the component selected
 * by the user.
 */
int8_t BME280TemperatureSensor::bme280_compensate_data(uint8_t sensor_comp, const struct bme280_uncomp_data *uncomp_data, struct bme280_data *comp_data) noexcept
{
    int8_t rslt = BME280_OK;

    if ((uncomp_data != NULL) && (comp_data != NULL))
    {
        /* Initialize to zero */
        comp_data->temperature = 0;
        comp_data->pressure = 0;
        comp_data->humidity = 0;

        /* If pressure or temperature component is selected */
        if (sensor_comp & (BME280_PRESS | BME280_TEMP | BME280_HUM))
        {
            /* Compensate the temperature data */
            comp_data->temperature = compensate_temperature(uncomp_data);
        }

        if (sensor_comp & BME280_PRESS)
        {
            /* Compensate the pressure data */
            comp_data->pressure = compensate_pressure(uncomp_data);
        }

        if (sensor_comp & BME280_HUM)
        {
            /* Compensate the humidity data */
            comp_data->humidity = compensate_humidity(uncomp_data);
        }
    }
    else
    {
        rslt = BME280_E_NULL_PTR;
    }

    return rslt;
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
