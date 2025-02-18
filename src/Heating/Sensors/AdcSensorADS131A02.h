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

#include "AdditionalOutputSensor.h"

#define FOUR_CHANNELS	(0)			// set to 1 if we use the ADS131A04, 0 for the ADS131A02

class AdcSensorADS131A02Chan0 : public SpiTemperatureSensor
{
public:
	explicit AdcSensorADS131A02Chan0(unsigned int sensorNum, bool p_bipolar) noexcept;
	GCodeResult Configure(GCodeBuffer& gb, const StringRef& reply, bool& changed) THROWS(GCodeException) override;

#if SUPPORT_REMOTE_COMMANDS
	GCodeResult Configure(const CanMessageGenericParser& parser, const StringRef& reply) noexcept override;		// configure the sensor from M308 parameters
	GCodeResult ConfigureAdditionalOutput(const CanMessageGenericParser& parser, const StringRef& reply, bool& changed, uint8_t outputNumber) noexcept override;
#endif

	const uint8_t GetNumAdditionalOutputs() const noexcept override { return NumChannels - 1; }
	TemperatureError GetAdditionalOutput(float& t, uint8_t outputNumber) noexcept override;
	GCodeResult ConfigureAdditionalOutput(GCodeBuffer& gb, const StringRef& reply, bool& changed, uint8_t outputNumber) THROWS(GCodeException) override;
	void AppendAdditionalOutputParameters(const StringRef& reply, uint8_t outputNumber) noexcept override;

	void Poll() noexcept override;
	const char *_ecv_array GetShortSensorType() const noexcept override { return (bipolar) ? TypeName_chan0_bipolar : TypeName_chan0_unipolar; }

	static constexpr const char *_ecv_array TypeName_chan0_unipolar = "ads131.chan0.u";
	static constexpr const char *_ecv_array TypeName_chan0_bipolar = "ads131.chan0.b";

protected:
	DECLARE_OBJECT_MODEL

private:
	static SensorTypeDescriptor typeDescriptor_chan0_unipolar;
	static SensorTypeDescriptor typeDescriptor_chan0_bipolar;

#if FOUR_CHANNELS
	static constexpr unsigned int NumChannels = 4;
#else
	static constexpr unsigned int NumChannels = 2;
#endif

	// The maximum reading is supposed to be provided for a unipolar input of 10V. The reference voltage is 2.442V. The ratio of these is 4.095:1.
	// We use a voltage divider ratio in the inputs of 13.0:3.0 which is a little higher, so we need to scale the reading up a little.
	static constexpr float ReadingScalingFactor = (13.5 * 2.442)/(3.0 * 10.0);

	TemperatureError TakeReading() noexcept;
	GCodeResult FinishConfiguring(bool changed, const StringRef& reply) noexcept;
	void CalcDerivedParameters() noexcept;
	TemperatureError TryInitAdc() noexcept;

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

		ADC1_GAIN = 0x11,
		ADC2_GAIN = 0x12,
#if FOUR_CHANNELS
		ADC3_GAIN = 0x13,
		ADC4_GAIN = 0x14,
#endif
	};

	// STAT_1 status register bits
	enum ADS131Status : uint8_t
	{
		f_check = 1u << 0,			// uncorrectable Hamming or CRC error on data in
		f_drdy = 1u << 1,			// data ready fault
		f_resync = 1u << 2,			// resync fault, only applies to sync slave mode
		f_wdt = 1u << 3,			// watchdog timed out
		f_adcin = 1u << 4,			// ADC range fault, read STAT_P and STAT_N to identify and clear it
		f_spi = 1u << 5,			// SPI fault, read STAT_S to identify and clear it
		f_opc = 1u << 6,			// invalid command, or command sent before unlocked
	};

	static constexpr float DefaultReadingAtMin = 0.0;
	static constexpr float DefaultReadingAtMax = 100.0;

	// Send a command and receive the response
	TemperatureError DoTransaction(ADS131Command command, ADS131Register regNum, uint8_t data, uint16_t &status, uint32_t readings[NumChannels], bool checkResponse) noexcept;

	// Wait for the device to become ready after a reset returning TemperatureError::ok if successful
	TemperatureError WaitReady() noexcept;

	bool configured;

	// Configurable parameters
	float readingAtMin[NumChannels];
	float readingAtMax [NumChannels];

	// Collected data
	float lastReadings[NumChannels];
	uint16_t lastCommand;
	TemperatureError lastResult = TemperatureError::notInitialised;

	bool bipolar;

	struct InitTableEntry
	{
		ADS131Register regNum;
		uint8_t valUnipolar;
		uint8_t valBipolar;
	};

	static const InitTableEntry initTable[];
};

class AdcSensorADS131A02Chan1 : public AdditionalOutputSensor
{
public:
	explicit AdcSensorADS131A02Chan1(unsigned int sensorNum) noexcept;

	const char *_ecv_array GetShortSensorType() const noexcept override { return TypeName_chan1; }

	static constexpr const char *_ecv_array TypeName_chan1 = "ads131.chan1";

private:
	static SensorTypeDescriptor typeDescriptor_chan1;
};

#endif

#endif /* SRC_HEATING_SENSORS_ADCSENSORADS131A02_H_ */
