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
# include "TemperatureSensor.h"
# include "RTOSIface/RTOSIface.h"

enum class DhtSensorType
{
	none,
	Dht11,
	Dht21,
	Dht22
};

// This class represents a DHT sensor attached to a particular SPI CS pin
class DhtSensorHardwareInterface
{
public:
	DhtSensorHardwareInterface(IoPort& port, DhtSensorType type);

	void SetType(DhtSensorType t) { type = t; }

	void Interrupt();
	TemperatureError GetTemperatureOrHumidity(float& t, bool wantHumidity) const;

	static DhtSensorHardwareInterface* GetOrCreate(unsigned int relativeChannel, IoPort& port, DhtSensorType type);
	static void Remove(DhtSensorHardwareInterface* dht);
	static void InitStatic();
	static void SensorTask();

private:
	DhtSensorHardwareInterface(unsigned int sensorNum);

	GCodeResult ConfigureType(TemperatureSensor *ts, GCodeBuffer& gb, const StringRef& reply);
	void TakeReading();
	TemperatureError ProcessReadings();

	static constexpr unsigned int DhtTaskStackWords = 100;		// task stack size in dwords. 80 was not enough. Use 300 if debugging is enabled.
	static Mutex dhtMutex;
	static Task<DhtTaskStackWords> *dhtTask;
	static DhtSensorHardwareInterface *activeSensors[MaxSpiTempSensors];

	IoPort& port;
	DhtSensorType type;
	TemperatureError lastResult;
	float lastTemperature, lastHumidity;
	size_t badTemperatureCount;

	volatile uint16_t lastPulseTime;
	volatile size_t numPulses;
	uint16_t pulses[41];			// 1 start bit + 40 data bits
};

// This class represents a DHT temperature sensor
class DhtTemperatureSensor : public SensorWithPort
{
public:
	DhtTemperatureSensor(unsigned int sensorNum);
	~DhtTemperatureSensor();

	GCodeResult Configure(GCodeBuffer& gb, const StringRef& reply) override;
	TemperatureError GetLatestTemperature(float& t, uint8_t outputNumber = 0) override;
	const uint8_t GetNumAdditionalOutputs() const override { return 1; }
	void Poll() override;

	static constexpr const char *TypeName = "dhttemp";

private:
	DhtSensorHardwareInterface* dht;
	DhtSensorType type;
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
