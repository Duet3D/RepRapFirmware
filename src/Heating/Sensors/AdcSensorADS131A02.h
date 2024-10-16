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
	TemperatureError TryInitAdc() const noexcept;

	// Commands that can be sent to the ADC
	enum ADS131Command : uint16_t
	{
		nullcmd = 0,
		reset = 0x0011,
		standby = 0x0022,
		wakeup = 0x0033,
		lock = 0x0555,
		unlock = 0x0655,
		rreg = 0x2000,				// put register number in bits 8-12
		rregs = rreg,				// put start register number in bits 8-12 and (numRegisters-1) in bits 0-7
		wreg = 0x4000,				// put register number in bits 8-12 and data in bits 0-7
		//wregs = 0x6000			// put start register number in bits 8-12 and (numRegisters-1) in bits 0-7. Data follows in subsequent bytes, see table 13 in the datasheet. Not used by this driver.
	};

	enum ADS131Register : int8_t
	{
		none =0,
		STAT_1 = 0x02,
		STAT_P = 0x03,
		STAT_N = 0x04,
		STAT_S = 0x05,
		ERROR_CNT = 0x06,
		STAT_M2 = 0x07,

		A_SYS_CFG = 0x0B,
		D_SYS_CFG = 0x0C,
		CLK1 = 0x0D,
		CLK2 = 0x0E,
		ADC_ENA = 0x0F,

		ADC1 = 0x11,
		ADC2 = 0x12
	};

	// Send a command and receive the response
	TemperatureError DoTransaction(ADS131Command command, ADS131Register regNum, uint8_t data, uint16_t &status, uint32_t readings[2]) const noexcept;

	// Wait for the device to become ready after a reset returning TemperatureError::ok if successful
	TemperatureError WaitReady() const noexcept;

	// Configurable parameters
	float readingAtMin = DefaultReadingAtMin;
	float readingAtMax = DefaultReadingAtmax;

	bool use24bitFrames;

	static constexpr float DefaultReadingAtMin = 0.0;
	static constexpr float DefaultReadingAtmax = 100.0;

	struct InitTableEntry
	{
		ADS131Register regNum;
		uint8_t val;
	};

	static const InitTableEntry initTable[];
};

#endif

#endif /* SRC_HEATING_SENSORS_ADCSENSORADS131A02_H_ */
