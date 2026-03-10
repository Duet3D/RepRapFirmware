/*
 * BME68x.cpp
 *
 *  Created on: 9 Mar 2026
 *      Author: Christian
 *
 * Portions of this driver are derived from code at https://github.com/boschsensortec/BME68x_SensorAPI. The following applies to those portions:
 *
 * Copyright (c) 2023 Bosch Sensortec GmbH. All rights reserved.
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

#include "BME68x.h"

#if SUPPORT_BME68X


constexpr uint16_t MinimumReadInterval = 1000;			// ms
constexpr uint32_t BME68x_Frequency = 4000000;			// maximum for BME68x is 10MHz
constexpr SpiMode BME68x_SpiMode = SpiMode::mode0;		// BME68x supports SPI mode 0

// Gas range lookup table for resistance calculation
static const float gas_range_lookup1[16] = {
	1.0, 1.0, 1.0, 1.0, 1.0, 0.99, 1.0, 0.992,
	1.0, 1.0, 0.998, 0.995, 1.0, 0.99, 1.0, 1.0
};

static const float gas_range_lookup2[16] = {
	8000000.0, 4000000.0, 2000000.0, 1000000.0,
	499500.4995, 248262.1648, 125000.0, 63004.03226,
	31281.28128, 15625.0, 7812.5, 3906.25,
	1953.125, 976.5625, 488.28125, 244.140625
};

// Sensor type descriptors
TemperatureSensor::SensorTypeDescriptor BME68xTemperatureSensor::typeDescriptor(TypeName, [](unsigned int sensorNum) noexcept -> TemperatureSensor *_ecv_from { return new BME68xTemperatureSensor(sensorNum); } );
TemperatureSensor::SensorTypeDescriptor BME68xPressureSensor::typeDescriptor(TypeName, [](unsigned int sensorNum) noexcept -> TemperatureSensor *_ecv_from { return new BME68xPressureSensor(sensorNum); } );
TemperatureSensor::SensorTypeDescriptor BME68xHumiditySensor::typeDescriptor(TypeName, [](unsigned int sensorNum) noexcept -> TemperatureSensor *_ecv_from { return new BME68xHumiditySensor(sensorNum); } );
TemperatureSensor::SensorTypeDescriptor BME68xGasResistanceSensor::typeDescriptor(TypeName, [](unsigned int sensorNum) noexcept -> TemperatureSensor *_ecv_from { return new BME68xGasResistanceSensor(sensorNum); } );

/*!
 * @brief This API reads the data from the given register address of the sensor.
 *        Handles SPI memory page switching as needed.
 */
TemperatureError BME68xTemperatureSensor::bme68x_get_regs(uint8_t reg_addr, uint8_t *_ecv_array reg_data, uint16_t len) noexcept
{
	TemperatureError rslt = set_mem_page(reg_addr);
	if (rslt != TemperatureError::ok)
	{
		return rslt;
	}

	uint8_t addrBuff[MaxRegistersToRead + 1];
	uint8_t dataBuff[MaxRegistersToRead + 1];

	addrBuff[0] = reg_addr | BME68X_SPI_RD_MSK;
	rslt = DoSpiTransaction(addrBuff, dataBuff, len + 1);

	if (rslt == TemperatureError::ok)
	{
		memcpy(reg_data, &dataBuff[1], len);
	}
	return rslt;
}

/*!
 * @brief This API writes the given data to the register address of the sensor.
 *        Handles SPI memory page switching as needed.
 */
TemperatureError BME68xTemperatureSensor::bme68x_set_reg(uint8_t reg_addr, uint8_t reg_data) noexcept
{
	TemperatureError rslt = set_mem_page(reg_addr);
	if (rslt != TemperatureError::ok)
	{
		return rslt;
	}

	uint8_t temp_buff[2] = { (uint8_t)(reg_addr & BME68X_SPI_WR_MSK), reg_data };
	return DoSpiTransaction(temp_buff, nullptr, 2);
}

/*!
 * @brief This API sets the correct SPI memory page for the given register address.
 *        BME68x has two SPI memory pages: page 0 (registers 0x00-0x7F) and page 1 (registers 0x80-0xFF).
 */
TemperatureError BME68xTemperatureSensor::set_mem_page(uint8_t reg_addr) noexcept
{
	const uint8_t desired_page = (reg_addr > 0x7F) ? BME68X_MEM_PAGE1 : BME68X_MEM_PAGE0;

	if (desired_page != mem_page)
	{
		// Read current mem_page register, modify, and write back
		uint8_t addrBuff[2];
		uint8_t dataBuff[2];

		addrBuff[0] = BME68X_REG_MEM_PAGE | BME68X_SPI_RD_MSK;
		TemperatureError rslt = DoSpiTransaction(addrBuff, dataBuff, 2);
		if (rslt != TemperatureError::ok)
		{
			return rslt;
		}

		uint8_t reg = (dataBuff[1] & ~BME68X_MEM_PAGE_MSK) | (desired_page & BME68X_MEM_PAGE_MSK);
		uint8_t writeBuff[2] = { (uint8_t)(BME68X_REG_MEM_PAGE & BME68X_SPI_WR_MSK), reg };
		rslt = DoSpiTransaction(writeBuff, nullptr, 2);
		if (rslt == TemperatureError::ok)
		{
			mem_page = desired_page;
		}
		return rslt;
	}
	return TemperatureError::ok;
}

/*!
 * @brief This API performs the soft reset of the sensor.
 */
TemperatureError BME68xTemperatureSensor::bme68x_soft_reset() noexcept
{
	TemperatureError rslt = bme68x_set_reg(BME68X_REG_SOFT_RESET, BME68X_SOFT_RESET_CMD);
	if (rslt == TemperatureError::ok)
	{
		delay(5);
	}
	return rslt;
}

/*!
 * @brief This API is the entry point. It reads the chip-id and calibration data from the sensor.
 */
TemperatureError BME68xTemperatureSensor::bme68x_init() noexcept
{
	// Force hardware mem_page to the power-on default (page 1 = registers 0x80-0xFF) so that
	// the firmware state matches even if the chip was left on page 0 by a previous run.
	{
		uint8_t writeBuff[2] = { (uint8_t)(BME68X_REG_MEM_PAGE & BME68X_SPI_WR_MSK), BME68X_MEM_PAGE1 };
		DoSpiTransaction(writeBuff, nullptr, 2);
		mem_page = BME68X_MEM_PAGE1;
	}

	uint8_t try_count = 5;
	while (try_count)
	{
		/* Read the chip-id */
		uint8_t chip_id = 0;
		TemperatureError rslt = bme68x_get_regs(BME68X_REG_CHIP_ID, &chip_id, 1);

		if ((rslt == TemperatureError::ok) && (chip_id == BME68X_CHIP_ID))
		{
			rslt = bme68x_soft_reset();
			if (rslt == TemperatureError::ok)
			{
				/* Read the variant id */
				uint8_t var_id = 0;
				rslt = bme68x_get_regs(BME68X_REG_VARIANT_ID, &var_id, 1);
				if (rslt == TemperatureError::ok)
				{
					variant_id = var_id;
					return get_calib_data();
				}
			}
			break;
		}

		delay(2);
		--try_count;
	}

	return TemperatureError::hardwareError;
}

/*!
 * @brief This API reads the calibration data from the sensor.
 */
TemperatureError BME68xTemperatureSensor::get_calib_data() noexcept
{
	uint8_t coeff_array[BME68X_LEN_COEFF_ALL] = { 0 };

	/* Read first group of coefficients (23 bytes at 0x8A) */
	static_assert(BME68X_LEN_COEFF1 <= MaxRegistersToRead);
	TemperatureError rslt = bme68x_get_regs(BME68X_REG_COEFF1, coeff_array, BME68X_LEN_COEFF1);
	if (rslt != TemperatureError::ok)
	{
		return rslt;
	}

	/* Read second group of coefficients (14 bytes at 0xE1) */
	static_assert(BME68X_LEN_COEFF2 <= MaxRegistersToRead);
	rslt = bme68x_get_regs(BME68X_REG_COEFF2, &coeff_array[BME68X_LEN_COEFF1], BME68X_LEN_COEFF2);
	if (rslt != TemperatureError::ok)
	{
		return rslt;
	}

	/* Read third group of coefficients (5 bytes at 0x00) */
	static_assert(BME68X_LEN_COEFF3 <= MaxRegistersToRead);
	rslt = bme68x_get_regs(BME68X_REG_COEFF3, &coeff_array[BME68X_LEN_COEFF1 + BME68X_LEN_COEFF2], BME68X_LEN_COEFF3);
	if (rslt == TemperatureError::ok)
	{
		parse_calib_data(coeff_array);
	}

	return rslt;
}

/*!
 * @brief This API parses the calibration data from the coefficient array and stores it in the device structure.
 */
void BME68xTemperatureSensor::parse_calib_data(const uint8_t *_ecv_array coeff) noexcept
{
	/* Temperature calibration */
	calib.par_t1 = BME68X_CONCAT_BYTES(coeff[BME68X_IDX_T1_MSB], coeff[BME68X_IDX_T1_LSB]);
	calib.par_t2 = (int16_t)BME68X_CONCAT_BYTES(coeff[BME68X_IDX_T2_MSB], coeff[BME68X_IDX_T2_LSB]);
	calib.par_t3 = (int8_t)coeff[BME68X_IDX_T3];

	/* Pressure calibration */
	calib.par_p1 = BME68X_CONCAT_BYTES(coeff[BME68X_IDX_P1_MSB], coeff[BME68X_IDX_P1_LSB]);
	calib.par_p2 = (int16_t)BME68X_CONCAT_BYTES(coeff[BME68X_IDX_P2_MSB], coeff[BME68X_IDX_P2_LSB]);
	calib.par_p3 = (int8_t)coeff[BME68X_IDX_P3];
	calib.par_p4 = (int16_t)BME68X_CONCAT_BYTES(coeff[BME68X_IDX_P4_MSB], coeff[BME68X_IDX_P4_LSB]);
	calib.par_p5 = (int16_t)BME68X_CONCAT_BYTES(coeff[BME68X_IDX_P5_MSB], coeff[BME68X_IDX_P5_LSB]);
	calib.par_p6 = (int8_t)coeff[BME68X_IDX_P6];
	calib.par_p7 = (int8_t)coeff[BME68X_IDX_P7];
	calib.par_p8 = (int16_t)BME68X_CONCAT_BYTES(coeff[BME68X_IDX_P8_MSB], coeff[BME68X_IDX_P8_LSB]);
	calib.par_p9 = (int16_t)BME68X_CONCAT_BYTES(coeff[BME68X_IDX_P9_MSB], coeff[BME68X_IDX_P9_LSB]);
	calib.par_p10 = coeff[BME68X_IDX_P10];

	/* Humidity calibration */
	calib.par_h1 = (uint16_t)(((uint16_t)coeff[BME68X_IDX_H1_MSB] << 4) | (coeff[BME68X_IDX_H1_LSB] & BME68X_BIT_H1_DATA_MSK));
	calib.par_h2 = (uint16_t)(((uint16_t)coeff[BME68X_IDX_H2_MSB] << 4) | (coeff[BME68X_IDX_H2_LSB] >> 4));
	calib.par_h3 = (int8_t)coeff[BME68X_IDX_H3];
	calib.par_h4 = (int8_t)coeff[BME68X_IDX_H4];
	calib.par_h5 = (int8_t)coeff[BME68X_IDX_H5];
	calib.par_h6 = coeff[BME68X_IDX_H6];
	calib.par_h7 = (int8_t)coeff[BME68X_IDX_H7];

	/* Gas calibration */
	calib.par_gh1 = (int8_t)coeff[BME68X_IDX_GH1];
	calib.par_gh2 = (int16_t)BME68X_CONCAT_BYTES(coeff[BME68X_IDX_GH2_MSB], coeff[BME68X_IDX_GH2_LSB]);
	calib.par_gh3 = (int8_t)coeff[BME68X_IDX_GH3];

	/* Heater calibration */
	calib.res_heat_range = (coeff[BME68X_IDX_RES_HEAT_RANGE] & BME68X_RHRANGE_MSK) >> 4;
	calib.res_heat_val = (int8_t)coeff[BME68X_IDX_RES_HEAT_VAL];
	calib.range_sw_err = (int8_t)(coeff[BME68X_IDX_RANGE_SW_ERR] & BME68X_RSERROR_MSK) >> 4;
}

/*!
 * @brief This API configures the sensor settings: oversampling, filter and gas heater.
 */
TemperatureError BME68xTemperatureSensor::configure_sensor() noexcept
{
	/* Set humidity oversampling */
	uint8_t reg_data;
	TemperatureError rslt = bme68x_get_regs(BME68X_REG_CTRL_HUM, &reg_data, 1);
	if (rslt != TemperatureError::ok)
	{
		return rslt;
	}

	reg_data = BME68X_SET_BITS_POS_0(reg_data, BME68X_OSH, BME68X_OS_1X);
	rslt = bme68x_set_reg(BME68X_REG_CTRL_HUM, reg_data);
	if (rslt != TemperatureError::ok)
	{
		return rslt;
	}

	/* Set temperature and pressure oversampling, mode to sleep */
	rslt = bme68x_get_regs(BME68X_REG_CTRL_MEAS, &reg_data, 1);
	if (rslt != TemperatureError::ok)
	{
		return rslt;
	}

	reg_data = BME68X_SET_BITS(reg_data, BME68X_OST, BME68X_OS_2X);
	reg_data = BME68X_SET_BITS(reg_data, BME68X_OSP, BME68X_OS_16X);
	reg_data = BME68X_SET_BITS_POS_0(reg_data, BME68X_MODE, BME68X_SLEEP_MODE);
	rslt = bme68x_set_reg(BME68X_REG_CTRL_MEAS, reg_data);
	if (rslt != TemperatureError::ok)
	{
		return rslt;
	}

	/* Set IIR filter */
	rslt = bme68x_get_regs(BME68X_REG_CONFIG, &reg_data, 1);
	if (rslt != TemperatureError::ok)
	{
		return rslt;
	}

	reg_data = BME68X_SET_BITS(reg_data, BME68X_FILTER, BME68X_FILTER_SIZE_3);
	rslt = bme68x_set_reg(BME68X_REG_CONFIG, reg_data);
	if (rslt != TemperatureError::ok)
	{
		return rslt;
	}

	/* Configure gas heater: 320°C target, 150ms duration */
	rslt = set_gas_config(320, 150);
	return rslt;
}

/*!
 * @brief This API configures the gas heater temperature and duration for a single forced-mode measurement.
 */
TemperatureError BME68xTemperatureSensor::set_gas_config(uint16_t targetTemp, uint16_t duration) noexcept
{
	/* Calculate heater resistance value for target temperature */
	const uint8_t res_heat = calc_res_heat(targetTemp);
	TemperatureError rslt = bme68x_set_reg(BME68X_REG_RES_HEAT0, res_heat);
	if (rslt != TemperatureError::ok)
	{
		return rslt;
	}

	/* Calculate and set gas wait time */
	const uint8_t gas_wait = calc_gas_wait(duration);
	rslt = bme68x_set_reg(BME68X_REG_GAS_WAIT0, gas_wait);
	if (rslt != TemperatureError::ok)
	{
		return rslt;
	}

	/* Enable gas measurement and select heater profile 0 */
	uint8_t reg_data;
	rslt = bme68x_get_regs(BME68X_REG_CTRL_GAS_1, &reg_data, 1);
	if (rslt != TemperatureError::ok)
	{
		return rslt;
	}

	const uint8_t run_gas = (variant_id == BME68X_VARIANT_GAS_HIGH) ? BME68X_ENABLE_GAS_MEAS_H : BME68X_ENABLE_GAS_MEAS_L;
	reg_data = BME68X_SET_BITS(reg_data, BME68X_RUN_GAS, run_gas);
	reg_data = BME68X_SET_BITS_POS_0(reg_data, BME68X_NBCONV, 0);	// heater profile 0
	rslt = bme68x_set_reg(BME68X_REG_CTRL_GAS_1, reg_data);
	if (rslt != TemperatureError::ok)
	{
		return rslt;
	}

	/* Enable heater */
	rslt = bme68x_get_regs(BME68X_REG_CTRL_GAS_0, &reg_data, 1);
	if (rslt != TemperatureError::ok)
	{
		return rslt;
	}

	reg_data = BME68X_SET_BITS(reg_data, BME68X_HCTRL, BME68X_ENABLE_HEATER);
	rslt = bme68x_set_reg(BME68X_REG_CTRL_GAS_0, reg_data);
	return rslt;
}

/*!
 * @brief This API triggers a forced mode measurement.
 */
TemperatureError BME68xTemperatureSensor::start_forced_measurement() noexcept
{
	uint8_t reg_data;
	TemperatureError rslt = bme68x_get_regs(BME68X_REG_CTRL_MEAS, &reg_data, 1);
	if (rslt == TemperatureError::ok)
	{
		reg_data = BME68X_SET_BITS_POS_0(reg_data, BME68X_MODE, BME68X_FORCED_MODE);
		rslt = bme68x_set_reg(BME68X_REG_CTRL_MEAS, reg_data);
	}
	return rslt;
}

/*!
 * @brief Read and compensate sensor data from field 0.
 */
TemperatureError BME68xTemperatureSensor::bme68x_get_sensor_data() noexcept
{
	uint8_t reg_data[BME68X_LEN_FIELD] = { 0 };

	static_assert(BME68X_LEN_FIELD <= MaxRegistersToRead);
	const TemperatureError rslt = bme68x_get_regs(BME68X_REG_FIELD0, reg_data, BME68X_LEN_FIELD);
	if (rslt == TemperatureError::ok)
	{
		parse_field_data(reg_data);
	}
	return rslt;
}

/*!
 * @brief Parse field data from register buffer and compensate all outputs.
 *
 * Field 0 register layout (17 bytes starting at 0x1D):
 *   [0]    = meas_status_0 (new_data[7], gas_measuring[6], measuring[5], gas_meas_index[3:0])
 *   [1]    = meas_index
 *   [2..4] = pressure ADC (MSB, LSB, XLSB)
 *   [5..7] = temperature ADC (MSB, LSB, XLSB)
 *   [8..9] = humidity ADC (MSB, LSB)
 *   [10]   = sub_meas_index
 *   [11]   = idac_heat
 *   [12]   = res_heat
 *   [13..14] = gas_r low variant (MSB, LSB[7:6] | gas_valid[5] | heat_stab[4] | gas_range[3:0])
 *   [15..16] = gas_r high variant (MSB, LSB[7:6] | gas_valid[5] | heat_stab[4] | gas_range[3:0])
 */
void BME68xTemperatureSensor::parse_field_data(const uint8_t *_ecv_array reg_data) noexcept
{
	/* Check if new data is available */
	if (!(reg_data[0] & BME68X_NEW_DATA_MSK))
	{
		return;								// no new data yet
	}

	/* Extract raw ADC data for pressure */
	const uint32_t pres_adc = ((uint32_t)reg_data[2] << 12) | ((uint32_t)reg_data[3] << 4) | ((uint32_t)reg_data[4] >> 4);

	/* Extract raw ADC data for temperature */
	const uint32_t temp_adc = ((uint32_t)reg_data[5] << 12) | ((uint32_t)reg_data[6] << 4) | ((uint32_t)reg_data[7] >> 4);

	/* Extract raw ADC data for humidity */
	const uint16_t hum_adc = (uint16_t)(((uint32_t)reg_data[8] << 8) | (uint32_t)reg_data[9]);

	/* Extract gas resistance ADC and range: bytes 13-14 for BME680 (low variant), 15-16 for BME688 (high variant) */
	const uint8_t gas_lsb_idx = (variant_id == BME68X_VARIANT_GAS_HIGH) ? 16 : 14;
	const uint16_t gas_res_adc = (uint16_t)(((uint32_t)reg_data[gas_lsb_idx - 1] << 2) | (reg_data[gas_lsb_idx] >> 6));
	const uint8_t gas_range = reg_data[gas_lsb_idx] & BME68X_GAS_RANGE_MSK;
	gasValid = (reg_data[gas_lsb_idx] & BME68X_GASM_VALID_MSK) != 0;

	/* Compensate: temperature must be first (sets t_fine) */
	compTemperature = compensate_temperature(temp_adc);
	compPressure = compensate_pressure(pres_adc);
	compHumidity = compensate_humidity(hum_adc);

	if (gasValid)
	{
		if (variant_id == BME68X_VARIANT_GAS_HIGH)
		{
			compGasResistance = calc_gas_resistance_high(gas_res_adc, gas_range);
		}
		else
		{
			compGasResistance = calc_gas_resistance_low(gas_res_adc, gas_range);
		}
	}
}

/*!
 * @brief Compensate raw temperature data.
 * @note Must be called before pressure and humidity compensation because it sets t_fine.
 */
float BME68xTemperatureSensor::compensate_temperature(uint32_t temp_adc) noexcept
{
	const float var1 = (((float)temp_adc / 16384.0f) - ((float)calib.par_t1 / 1024.0f)) * (float)calib.par_t2;
	const float var2 = ((((float)temp_adc / 131072.0f) - ((float)calib.par_t1 / 8192.0f))
						* (((float)temp_adc / 131072.0f) - ((float)calib.par_t1 / 8192.0f)))
						* ((float)calib.par_t3 * 16.0f);
	calib.t_fine = var1 + var2;
	float temperature = calib.t_fine / 5120.0f;
	temperature = constrain<float>(temperature, -40.0f, 85.0f);
	return temperature;
}

/*!
 * @brief Compensate raw pressure data. Returns pressure in Pascals.
 */
float BME68xTemperatureSensor::compensate_pressure(uint32_t pres_adc) const noexcept
{
	float var1 = (calib.t_fine / 2.0f) - 64000.0f;
	float var2 = var1 * var1 * ((float)calib.par_p6 / 131072.0f);
	var2 = var2 + (var1 * (float)calib.par_p5 * 2.0f);
	var2 = (var2 / 4.0f) + ((float)calib.par_p4 * 65536.0f);
	var1 = ((((float)calib.par_p3 * var1 * var1) / 16384.0f) + ((float)calib.par_p2 * var1)) / 524288.0f;
	var1 = (1.0f + (var1 / 32768.0f)) * (float)calib.par_p1;
	float pressure;

	if (var1 > 0.0f)
	{
		pressure = 1048576.0f - (float)pres_adc;
		pressure = ((pressure - (var2 / 4096.0f)) * 6250.0f) / var1;
		var1 = ((float)calib.par_p9 * pressure * pressure) / 2147483648.0f;
		var2 = pressure * ((float)calib.par_p8 / 32768.0f);
		const float var3 = (pressure / 256.0f) * (pressure / 256.0f) * (pressure / 256.0f) * (calib.par_p10 / 131072.0f);
		pressure = pressure + (var1 + var2 + var3 + ((float)calib.par_p7 * 128.0f)) / 16.0f;
	}
	else
	{
		pressure = 0.0f;
	}

	return pressure * 0.01f;				// convert Pa to hPa
}

/*!
 * @brief Compensate raw humidity data. Returns humidity in %RH.
 */
float BME68xTemperatureSensor::compensate_humidity(uint16_t hum_adc) const noexcept
{
	const float temp_comp = calib.t_fine / 5120.0f;
	const float var1 = (float)hum_adc - (((float)calib.par_h1 * 16.0f) + (((float)calib.par_h3 / 2.0f) * temp_comp));
	const float var2 = var1 * (((float)calib.par_h2 / 262144.0f)
						* (1.0f + (((float)calib.par_h4 / 16384.0f) * temp_comp)
						+ (((float)calib.par_h5 / 1048576.0f) * temp_comp * temp_comp)));
	const float var3 = (float)calib.par_h6 / 16384.0f;
	const float var4 = (float)calib.par_h7 / 2097152.0f;
	float humidity = var2 + ((var3 + (var4 * temp_comp)) * var2 * var2);
	humidity = constrain<float>(humidity, 0.0f, 100.0f);
	return humidity;
}

/*!
 * @brief Calculate gas resistance for BME680 (low gas variant).
 */
float BME68xTemperatureSensor::calc_gas_resistance_low(uint16_t gas_res_adc, uint8_t gas_range) const noexcept
{
	const float var1 = (1340.0f + (5.0f * calib.range_sw_err)) * gas_range_lookup1[gas_range];
	const float gas_res = var1 * gas_range_lookup2[gas_range] / ((float)gas_res_adc - 512.0f + var1);
	return gas_res;
}

/*!
 * @brief Calculate gas resistance for BME688 (high gas variant).
 */
float BME68xTemperatureSensor::calc_gas_resistance_high(uint16_t gas_res_adc, uint8_t gas_range) const noexcept
{
	const float var1 = (float)(UINT32_C(262144) >> gas_range);
	float var2 = (float)gas_res_adc - 512.0f;
	var2 *= 3.0f;
	var2 += 4096.0f;
	const float gas_res = 1000000.0f * var1 / var2;
	return gas_res;
}

/*!
 * @brief Calculate heater resistance register value for a target temperature.
 */
uint8_t BME68xTemperatureSensor::calc_res_heat(uint16_t temp) const noexcept
{
	if (temp > 400)
	{
		temp = 400;
	}

	const float var1 = (((float)calib.par_gh1 / 16.0f) + 49.0f);
	const float var2 = ((((float)calib.par_gh2 / 32768.0f) * 0.0005f) + 0.00235f);
	const float var3 = (float)calib.par_gh3 / 1024.0f;
	const float var4 = var1 * (1.0f + (var2 * (float)temp));
	const float var5 = var4 + (var3 * 25.0f);		// ambient temp assumed ~25°C
	const float res_heat = (uint8_t)(3.4f * ((var5 * (4.0f / (4.0f + (float)calib.res_heat_range))
								* (1.0f / (1.0f + ((float)calib.res_heat_val * 0.002f)))) - 25.0f));
	return (uint8_t)res_heat;
}

/*!
 * @brief Calculate gas wait register value for a given duration in milliseconds.
 * The gas wait register encodes the timing as: duration = multiplication_factor * time_value
 * where multiplication_factor is 1, 4, 16 or 64 depending on bits 7:6.
 */
uint8_t BME68xTemperatureSensor::calc_gas_wait(uint16_t dur) noexcept
{
	uint8_t factor = 0;
	uint8_t durval;

	if (dur >= 0xFC0)
	{
		durval = 0xFF;							// maximum duration
	}
	else
	{
		while (dur > 0x3F)
		{
			dur = dur >> 2;
			factor += 1;
		}
		durval = (uint8_t)(dur + (factor * 64));
	}
	return durval;
}

// BME68xTemperatureSensor members

BME68xTemperatureSensor::BME68xTemperatureSensor(unsigned int sensorNum) noexcept
	: SpiTemperatureSensor(sensorNum, "BME68x temperature", BME68x_SpiMode, BME68x_Frequency),
	  variant_id(0), mem_page(BME68X_MEM_PAGE1),
	  compTemperature(0.0f), compPressure(0.0f), compHumidity(0.0f),
	  compGasResistance(0.0f), gasValid(false)
{
	memset(&calib, 0, sizeof(calib));
}

GCodeResult BME68xTemperatureSensor::Configure(GCodeBuffer &gb, const StringRef &reply, bool &changed)
{
	if (!ConfigurePort(gb, reply, changed))
	{
		return GCodeResult::error;
	}
	ConfigureCommonParameters(gb, changed);
	return FinishConfiguring(changed, reply);
}

#if SUPPORT_REMOTE_COMMANDS

GCodeResult BME68xTemperatureSensor::Configure(const CanMessageGenericParser& parser, const StringRef& reply) noexcept
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

GCodeResult BME68xTemperatureSensor::FinishConfiguring(bool changed, const StringRef& reply) noexcept
{
	if (changed)
	{
		InitSpi();
		TemperatureError rslt = bme68x_init();
		SetResult(0.0, rslt);

		if (rslt == TemperatureError::ok)
		{
			rslt = configure_sensor();
		}

		if (rslt != TemperatureError::ok)
		{
			reply.printf("Failed to initialise BME68x sensor: %s", rslt.ToString());
			return GCodeResult::error;
		}
	}
	else
	{
		CopyBasicDetails(reply);
	}
	return GCodeResult::ok;
}

TemperatureError BME68xTemperatureSensor::GetAdditionalOutput(float &t, uint8_t outputNumber) noexcept
{
	float dummy;
	const auto result = TemperatureSensor::GetLatestTemperature(dummy);
	switch (outputNumber)
	{
	case 1:
		t = compPressure;
		break;

	case 2:
		t = compHumidity;
		break;

	case 3:
		t = gasValid ? compGasResistance : -1.0f;
		break;

	default:
		t = BadErrorTemperature;
		return TemperatureError::invalidOutputNumber;
	}
	return result;
}

void BME68xTemperatureSensor::Poll() noexcept
{
	const auto now = millis();
	if ((now - GetLastReadingTime()) >= MinimumReadInterval)
	{
		if (bme68x_get_sensor_data() == TemperatureError::ok)
		{
			SetResult(compTemperature, TemperatureError::ok);

			/* Trigger next forced mode measurement */
			start_forced_measurement();
		}
		else
		{
			SetResult(TemperatureError::hardwareError);
		}
	}
}

// BME68xPressureSensor members

BME68xPressureSensor::BME68xPressureSensor(unsigned int sensorNum) noexcept
	: AdditionalOutputSensor(sensorNum, "BME68x-pressure", false)
{
}

BME68xPressureSensor::~BME68xPressureSensor() noexcept
{
}

// BME68xHumiditySensor members

BME68xHumiditySensor::BME68xHumiditySensor(unsigned int sensorNum) noexcept
	: AdditionalOutputSensor(sensorNum, "BME68x-humidity", false)
{
}

BME68xHumiditySensor::~BME68xHumiditySensor() noexcept
{
}

// BME68xGasResistanceSensor members

BME68xGasResistanceSensor::BME68xGasResistanceSensor(unsigned int sensorNum) noexcept
	: AdditionalOutputSensor(sensorNum, "BME68x-gas", false)
{
}

BME68xGasResistanceSensor::~BME68xGasResistanceSensor() noexcept
{
}

#endif

// End
