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

#ifndef RTOS
# error DHT sensors are only supported in RTOS builds
#endif

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

	static GCodeResult Configure(TemperatureSensor *ts, unsigned int relativeChannel, unsigned int mCode, unsigned int heater, GCodeBuffer& gb, const StringRef& reply);
	void Interrupt();

	static DhtSensorHardwareInterface *Create(unsigned int relativeChannel);
	static TemperatureError GetTemperatureOrHumidity(unsigned int relativeChannel, float& t, bool wantHumidity);
	static void InitStatic();
	static void SensorTask();

private:
	DhtSensorHardwareInterface(Pin p_pin);

	GCodeResult Configure(TemperatureSensor *ts, unsigned int mCode, unsigned int heater, GCodeBuffer& gb, const StringRef& reply);
	TemperatureError GetTemperatureOrHumidity(float& t, bool wantHumidity) const;
	void TakeReading();
	TemperatureError ProcessReadings();

	static constexpr unsigned int DhtTaskStackWords = 100;		// task stack size in dwords. 80 was not enough. Use 300 if debugging is enabled.
	static Mutex dhtMutex;
	static Task<DhtTaskStackWords> *dhtTask;
	static DhtSensorHardwareInterface *activeSensors[MaxSpiTempSensors];

	Pin sensorPin;
	DhtSensorType type;
	TemperatureError lastResult;
	float lastTemperature, lastHumidity;
	size_t badTemperatureCount;

	volatile uint16_t lastPulseTime;
	volatile size_t numPulses;
	uint16_t pulses[41];			// 1 start bit + 40 data bits
};

// This class represents a DHT temperature sensor
class DhtTemperatureSensor : public TemperatureSensor
{
public:
	DhtTemperatureSensor(unsigned int channel);
	~DhtTemperatureSensor();

	GCodeResult Configure(unsigned int mCode, unsigned int heater, GCodeBuffer& gb, const StringRef& reply) override;
	void Init() override;

protected:
	TemperatureError TryGetTemperature(float& t) override;
};

// This class represents a DHT humidity sensor
class DhtHumiditySensor : public TemperatureSensor
{
public:
	DhtHumiditySensor(unsigned int channel);
	~DhtHumiditySensor();

	GCodeResult Configure(unsigned int mCode, unsigned int heater, GCodeBuffer& gb, const StringRef& reply) override;
	void Init() override;

protected:
	TemperatureError TryGetTemperature(float& t) override;
};

#endif

#endif /* SRC_HEATING_SENSORS_DHTSENSOR_H_ */
