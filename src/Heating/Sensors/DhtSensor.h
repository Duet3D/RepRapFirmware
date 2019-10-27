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
# include "RTOSIface/RTOSIface.h"

enum class DhtSensorType
{
	Dht11,
	Dht21,
	Dht22
};


// This class represents a DHT temperature sensor
class DhtTemperatureSensor : public SensorWithPort
{
public:
	DhtTemperatureSensor(unsigned int sensorNum, DhtSensorType type);
	~DhtTemperatureSensor();

	GCodeResult Configure(GCodeBuffer& gb, const StringRef& reply) override;
	TemperatureError GetLatestTemperature(float& t, uint8_t outputNumber = 0) override;
	const uint8_t GetNumAdditionalOutputs() const override { return 1; }

	void Poll() override;
	bool PollInTask() override;

	void Interrupt();
	void TakeReading();
	TemperatureError ProcessReadings(float& t, float& h);

	static constexpr const char *TypeNameDht11 = "dht11";
	static constexpr const char *TypeNameDht21 = "dht21";
	static constexpr const char *TypeNameDht22 = "dht22";

private:
	DhtSensorType type;

	float lastHumidity;
	uint8_t badTemperatureCount;

	uint32_t lastReadTime;
	volatile uint16_t lastPulseTime;
	volatile uint8_t numPulses;
	uint16_t pulses[41];			// 1 start bit + 40 data bits
};

// This class represents a DHT humidity sensor
class DhtHumiditySensor : public AdditionalOutputSensor
{
public:
	DhtHumiditySensor(unsigned int sensorNum);
	~DhtHumiditySensor();

	static constexpr const char *TypeName = "dhthumidity";
};

#endif

#endif /* SRC_HEATING_SENSORS_DHTSENSOR_H_ */
