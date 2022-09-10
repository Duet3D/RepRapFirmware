/**
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
*
* @file       bme280_defs.h
* @date       2020-03-28
* @version    v3.5.0
*
*/

#ifndef BME280_DEFS_H_
#define BME280_DEFS_H_

/********************************************************/
/* header includes */
#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/kernel.h>
#else
#include <stdint.h>
#include <stddef.h>
#endif

/********************************************************/
/*! @name       Common macros               */
/********************************************************/

#if !defined(UINT8_C) && !defined(INT8_C)
#define INT8_C(x)    S8_C(x)
#define UINT8_C(x)   U8_C(x)
#endif

#if !defined(UINT16_C) && !defined(INT16_C)
#define INT16_C(x)   S16_C(x)
#define UINT16_C(x)  U16_C(x)
#endif

#if !defined(INT32_C) && !defined(UINT32_C)
#define INT32_C(x)   S32_C(x)
#define UINT32_C(x)  U32_C(x)
#endif

#if !defined(INT64_C) && !defined(UINT64_C)
#define INT64_C(x)   S64_C(x)
#define UINT64_C(x)  U64_C(x)
#endif

/**@}*/
/**\name C standard macros */
#ifndef NULL
#ifdef __cplusplus
#define NULL         0
#else
#define NULL         ((void *) 0)
#endif
#endif

/********************************************************/

#ifndef BME280_64BIT_ENABLE /*< Check if 64-bit integer (using BME280_64BIT_ENABLE) is enabled */
#ifndef BME280_32BIT_ENABLE /*< Check if 32-bit integer (using BME280_32BIT_ENABLE) is enabled */
#ifndef BME280_FLOAT_ENABLE /*< If any of the integer data types not enabled then enable BME280_FLOAT_ENABLE */
#define BME280_FLOAT_ENABLE
#endif
#endif
#endif

#ifndef TRUE
#define TRUE                                      UINT8_C(1)
#endif
#ifndef FALSE
#define FALSE                                     UINT8_C(0)
#endif

/**
 * BME280_INTF_RET_TYPE is the read/write interface return type which can be overwritten by the build system.
 */
#ifndef BME280_INTF_RET_TYPE
#define BME280_INTF_RET_TYPE                      int8_t
#endif

/**
 * The last error code from read/write interface is stored in the device structure as intf_rslt.
 */
#ifndef BME280_INTF_RET_SUCCESS
#define BME280_INTF_RET_SUCCESS                   INT8_C(0)
#endif

/**\name I2C addresses */
#define BME280_I2C_ADDR_PRIM                      UINT8_C(0x76)
#define BME280_I2C_ADDR_SEC                       UINT8_C(0x77)

/**\name BME280 chip identifier */
#define BME280_CHIP_ID                            UINT8_C(0x60)

/**\name Register Address */
#define BME280_CHIP_ID_ADDR                       UINT8_C(0xD0)
#define BME280_RESET_ADDR                         UINT8_C(0xE0)
#define BME280_TEMP_PRESS_CALIB_DATA_ADDR         UINT8_C(0x88)
#define BME280_HUMIDITY_CALIB_DATA_ADDR           UINT8_C(0xE1)
#define BME280_PWR_CTRL_ADDR                      UINT8_C(0xF4)
#define BME280_CTRL_HUM_ADDR                      UINT8_C(0xF2)
#define BME280_CTRL_MEAS_ADDR                     UINT8_C(0xF4)
#define BME280_CONFIG_ADDR                        UINT8_C(0xF5)
#define BME280_DATA_ADDR                          UINT8_C(0xF7)

/**\name API success code */
#define BME280_OK                                 INT8_C(0)

/**\name API error codes */
#define BME280_E_NULL_PTR                         INT8_C(-1)
#define BME280_E_DEV_NOT_FOUND                    INT8_C(-2)
#define BME280_E_INVALID_LEN                      INT8_C(-3)
#define BME280_E_COMM_FAIL                        INT8_C(-4)
#define BME280_E_SLEEP_MODE_FAIL                  INT8_C(-5)
#define BME280_E_NVM_COPY_FAILED                  INT8_C(-6)

/**\name API warning codes */
#define BME280_W_INVALID_OSR_MACRO                INT8_C(1)

/**\name Macros related to size */
#define BME280_TEMP_PRESS_CALIB_DATA_LEN          UINT8_C(26)
#define BME280_HUMIDITY_CALIB_DATA_LEN            UINT8_C(7)
#define BME280_P_T_H_DATA_LEN                     UINT8_C(8)

/**\name Sensor power modes */
#define BME280_SLEEP_MODE                         UINT8_C(0x00)
#define BME280_FORCED_MODE                        UINT8_C(0x01)
#define BME280_NORMAL_MODE                        UINT8_C(0x03)

/**\name Macro to combine two 8 bit data's to form a 16 bit data */
#define BME280_CONCAT_BYTES(msb, lsb)             (((uint16_t)msb << 8) | (uint16_t)lsb)

#define BME280_SET_BITS(reg_data, bitname, data) \
    ((reg_data & ~(bitname##_MSK)) | \
     ((data << bitname##_POS) & bitname##_MSK))
#define BME280_SET_BITS_POS_0(reg_data, bitname, data) \
    ((reg_data & ~(bitname##_MSK)) | \
     (data & bitname##_MSK))

#define BME280_GET_BITS(reg_data, bitname)        ((reg_data & (bitname##_MSK)) >> \
                                                   (bitname##_POS))
#define BME280_GET_BITS_POS_0(reg_data, bitname)  (reg_data & (bitname##_MSK))

/**\name Macros for bit masking */
#define BME280_SENSOR_MODE_MSK                    UINT8_C(0x03)
#define BME280_SENSOR_MODE_POS                    UINT8_C(0x00)

#define BME280_CTRL_HUM_MSK                       UINT8_C(0x07)
#define BME280_CTRL_HUM_POS                       UINT8_C(0x00)

#define BME280_CTRL_PRESS_MSK                     UINT8_C(0x1C)
#define BME280_CTRL_PRESS_POS                     UINT8_C(0x02)

#define BME280_CTRL_TEMP_MSK                      UINT8_C(0xE0)
#define BME280_CTRL_TEMP_POS                      UINT8_C(0x05)

#define BME280_FILTER_MSK                         UINT8_C(0x1C)
#define BME280_FILTER_POS                         UINT8_C(0x02)

#define BME280_STANDBY_MSK                        UINT8_C(0xE0)
#define BME280_STANDBY_POS                        UINT8_C(0x05)

/**\name Sensor component selection macros
 * These values are internal for API implementation. Don't relate this to
 * data sheet.
 */
#define BME280_PRESS                              UINT8_C(1)
#define BME280_TEMP                               UINT8_C(1 << 1)
#define BME280_HUM                                UINT8_C(1 << 2)
#define BME280_ALL                                UINT8_C(0x07)

/**\name Settings selection macros */
#define BME280_OSR_PRESS_SEL                      UINT8_C(1)
#define BME280_OSR_TEMP_SEL                       UINT8_C(1 << 1)
#define BME280_OSR_HUM_SEL                        UINT8_C(1 << 2)
#define BME280_FILTER_SEL                         UINT8_C(1 << 3)
#define BME280_STANDBY_SEL                        UINT8_C(1 << 4)
#define BME280_ALL_SETTINGS_SEL                   UINT8_C(0x1F)

/**\name Oversampling macros */
#define BME280_NO_OVERSAMPLING                    UINT8_C(0x00)
#define BME280_OVERSAMPLING_1X                    UINT8_C(0x01)
#define BME280_OVERSAMPLING_2X                    UINT8_C(0x02)
#define BME280_OVERSAMPLING_4X                    UINT8_C(0x03)
#define BME280_OVERSAMPLING_8X                    UINT8_C(0x04)
#define BME280_OVERSAMPLING_16X                   UINT8_C(0x05)

/**\name Measurement delay calculation macros  */
#define BME280_MEAS_OFFSET                        UINT16_C(1250)
#define BME280_MEAS_DUR                           UINT16_C(2300)
#define BME280_PRES_HUM_MEAS_OFFSET               UINT16_C(575)
#define BME280_MEAS_SCALING_FACTOR                UINT16_C(1000)

/**\name Standby duration selection macros */
#define BME280_STANDBY_TIME_0_5_MS                (0x00)
#define BME280_STANDBY_TIME_62_5_MS               (0x01)
#define BME280_STANDBY_TIME_125_MS                (0x02)
#define BME280_STANDBY_TIME_250_MS                (0x03)
#define BME280_STANDBY_TIME_500_MS                (0x04)
#define BME280_STANDBY_TIME_1000_MS               (0x05)
#define BME280_STANDBY_TIME_10_MS                 (0x06)
#define BME280_STANDBY_TIME_20_MS                 (0x07)

/**\name Filter coefficient selection macros */
#define BME280_FILTER_COEFF_OFF                   (0x00)
#define BME280_FILTER_COEFF_2                     (0x01)
#define BME280_FILTER_COEFF_4                     (0x02)
#define BME280_FILTER_COEFF_8                     (0x03)
#define BME280_FILTER_COEFF_16                    (0x04)

#define BME280_STATUS_REG_ADDR                    (0xF3)
#define BME280_SOFT_RESET_COMMAND                 (0xB6)
#define BME280_STATUS_IM_UPDATE                   (0x01)

/*!
 * @brief Interface selection Enums
 */
enum bme280_intf {
    /*< SPI interface */
    BME280_SPI_INTF,
    /*< I2C interface */
    BME280_I2C_INTF
};

/*!
 * @brief Type definitions
 */

/*!
 * @brief Bus communication function pointer which should be mapped to
 * the platform specific read functions of the user
 *
 * @param[in] reg_addr       : Register address from which data is read.
 * @param[out] reg_data     : Pointer to data buffer where read data is stored.
 * @param[in] len            : Number of bytes of data to be read.
 * @param[in, out] intf_ptr  : Void pointer that can enable the linking of descriptors
 *                                  for interface related call backs.
 *
 * @retval   0 -> Success.
 * @retval Non zero value -> Fail.
 *
 */
typedef BME280_INTF_RET_TYPE (*bme280_read_fptr_t)(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);

/*!
 * @brief Bus communication function pointer which should be mapped to
 * the platform specific write functions of the user
 *
 * @param[in] reg_addr      : Register address to which the data is written.
 * @param[in] reg_data     : Pointer to data buffer in which data to be written
 *                            is stored.
 * @param[in] len           : Number of bytes of data to be written.
 * @param[in, out] intf_ptr : Void pointer that can enable the linking of descriptors
 *                            for interface related call backs
 *
 * @retval   0   -> Success.
 * @retval Non zero value -> Fail.
 *
 */
typedef BME280_INTF_RET_TYPE (*bme280_write_fptr_t)(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len,
                                                    void *intf_ptr);

/*!
 * @brief Delay function pointer which should be mapped to
 * delay function of the user
 *
 * @param[in] period              : Delay in microseconds.
 * @param[in, out] intf_ptr       : Void pointer that can enable the linking of descriptors
 *                                  for interface related call backs
 *
 */
typedef void (*bme280_delay_us_fptr_t)(uint32_t period, void *intf_ptr);

/*!
 * @brief Calibration data
 */
struct bme280_calib_data
{
    /*< Calibration coefficient for the temperature sensor */
    uint16_t dig_t1;

    /*< Calibration coefficient for the temperature sensor */
    int16_t dig_t2;

    /*< Calibration coefficient for the temperature sensor */
    int16_t dig_t3;

    /*< Calibration coefficient for the pressure sensor */
    uint16_t dig_p1;

    /*< Calibration coefficient for the pressure sensor */
    int16_t dig_p2;

    /*< Calibration coefficient for the pressure sensor */
    int16_t dig_p3;

    /*< Calibration coefficient for the pressure sensor */
    int16_t dig_p4;

    /*< Calibration coefficient for the pressure sensor */
    int16_t dig_p5;

    /*< Calibration coefficient for the pressure sensor */
    int16_t dig_p6;

    /*< Calibration coefficient for the pressure sensor */
    int16_t dig_p7;

    /*< Calibration coefficient for the pressure sensor */
    int16_t dig_p8;

    /*< Calibration coefficient for the pressure sensor */
    int16_t dig_p9;

    /*< Calibration coefficient for the humidity sensor */
    uint8_t dig_h1;

    /*< Calibration coefficient for the humidity sensor */
    int16_t dig_h2;

    /*< Calibration coefficient for the humidity sensor */
    uint8_t dig_h3;

    /*< Calibration coefficient for the humidity sensor */
    int16_t dig_h4;

    /*< Calibration coefficient for the humidity sensor */
    int16_t dig_h5;

    /*< Calibration coefficient for the humidity sensor */
    int8_t dig_h6;

    /*< Variable to store the intermediate temperature coefficient */
    int32_t t_fine;
};

/*!
 * @brief bme280 sensor structure which comprises of temperature, pressure and
 * humidity data
 */
#ifdef BME280_FLOAT_ENABLE
struct bme280_data
{
    /*< Compensated pressure */
    double pressure;

    /*< Compensated temperature */
    double temperature;

    /*< Compensated humidity */
    double humidity;
};
#else
struct bme280_data
{
    /*< Compensated pressure */
    uint32_t pressure;

    /*< Compensated temperature */
    int32_t temperature;

    /*< Compensated humidity */
    uint32_t humidity;
};
#endif /*! BME280_USE_FLOATING_POINT */

/*!
 * @brief bme280 sensor structure which comprises of uncompensated temperature,
 * pressure and humidity data
 */
struct bme280_uncomp_data
{
    /*< un-compensated pressure */
    uint32_t pressure;

    /*< un-compensated temperature */
    uint32_t temperature;

    /*< un-compensated humidity */
    uint32_t humidity;
};

/*!
 * @brief bme280 sensor settings structure which comprises of mode,
 * oversampling and filter settings.
 */
struct bme280_settings
{
    /*< pressure oversampling */
    uint8_t osr_p;

    /*< temperature oversampling */
    uint8_t osr_t;

    /*< humidity oversampling */
    uint8_t osr_h;

    /*< filter coefficient */
    uint8_t filter;

    /*< standby time */
    uint8_t standby_time;
};

/*!
 * @brief bme280 device structure
 */
struct bme280_dev
{
    /*< Chip Id */
    uint8_t chip_id;

    /*< Interface function pointer used to enable the device address for I2C and chip selection for SPI */
    void *intf_ptr;

    /*< Interface Selection
     * For SPI, intf = BME280_SPI_INTF
     * For I2C, intf = BME280_I2C_INTF
     * */
    enum bme280_intf intf;

    /*< Read function pointer */
    bme280_read_fptr_t read;

    /*< Write function pointer */
    bme280_write_fptr_t write;

    /*< Delay function pointer */
    bme280_delay_us_fptr_t delay_us;

    /*< Trim data */
    struct bme280_calib_data calib_data;

    /*< Sensor settings */
    struct bme280_settings settings;

    /*< Variable to store result of read/write function */
    BME280_INTF_RET_TYPE intf_rslt;
};

#endif /* BME280_DEFS_H_ */
