/*
 * DhtSensor.h
 *
 *  Created on: 15 Sep 2017
 *      Author: Christian
 */

#ifndef SRC_HEATING_SENSORS_DHTSENSOR_H_
#define SRC_HEATING_SENSORS_DHTSENSOR_H_

#include "RepRapFirmware.h"

#if SUPPORT_DHT_SENSOR

# include "AdditionalOutputSensor.h"
# include "SensorWithPort.h"
# include <RTOSIface/RTOSIface.h>

enum class DhtSensorType : uint8_t
{
	Dht21,
	Dht22
};

// This class represents a DHT temperature sensor
class DhtTemperatureSensor : public SensorWithPort
{
public:
	DhtTemperatureSensor(unsigned int sensorNum, DhtSensorType t) noexcept;
	~DhtTemperatureSensor() noexcept;

	GCodeResult Configure(GCodeBuffer& gb, const StringRef& reply, bool& changed) override THROWS(GCodeException);
#if 0 //SUPPORT_REMOTE_COMMANDS
	GCodeResult Configure(const CanMessageGenericParser& parser, const StringRef& reply) noexcept override;
#endif

	TemperatureError GetAdditionalOutput(float& t, uint8_t outputNumber) noexcept override;
	const uint8_t GetNumAdditionalOutputs() const noexcept override { return 1; }
	void Poll() noexcept override;
	uint32_t GetTemperatureReadingTimeout() const noexcept override { return (MinimumReadInterval * (MaxDhtBadTemperatureCount + 1)) + 500; }
	const char *_ecv_array GetShortSensorType() const noexcept override;

	void Interrupt() noexcept;
	void TakeReading() noexcept;
	TemperatureError ProcessReadings(float& t, float& h) noexcept;

	static constexpr const char *_ecv_array TypeNameDht21 = "dht21";
	static constexpr const char *_ecv_array TypeNameDht22 = "dht22";

protected:
#if SAME5x
	void AppendPinDetails(const StringRef& reply) const noexcept override;
#endif

private:
	static constexpr uint32_t MinimumReadInterval = 2100;			// ms - datasheet https://www.sparkfun.com/datasheets/Sensors/Temperature/DHT22.pdf says "Collecting period should be : >2 second"

#if SAME5x
	// On the SAME5c we need a separate port to get the interrupt, because the output ports don't support interrupts
	IoPort interruptPort;
#endif

	float lastHumidity;
	uint32_t lastReadingAttemptTime;

	DhtSensorType type;
	uint8_t badTemperatureCount;

	volatile uint16_t lastPulseTime;
	volatile uint8_t numPulses;
	uint16_t pulses[41];			// 1 start bit + 40 data bits
};

// This class represents a DHT humidity sensor
class DhtHumiditySensor : public AdditionalOutputSensor
{
public:
	explicit DhtHumiditySensor(unsigned int sensorNum) noexcept;
	~DhtHumiditySensor() noexcept;

	const char *_ecv_array GetShortSensorType() const noexcept override { return TypeName; }

	static constexpr const char *_ecv_array TypeName = "dhthumidity";
};

#endif

#endif /* SRC_HEATING_SENSORS_DHTSENSOR_H_ */
