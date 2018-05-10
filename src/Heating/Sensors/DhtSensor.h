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

# include "TemperatureSensor.h"
# include "RTOSIface.h"

enum class DhtSensorType
{
	Dht11,
	Dht21,
	Dht22
};

class DhtSensor : public TemperatureSensor
{
public:
	friend class Heat;

	DhtSensor(unsigned int channel);
	~DhtSensor();

#ifdef RTOS
	static void SensorTask();
#endif

	GCodeResult Configure(unsigned int mCode, unsigned int heater, GCodeBuffer& gb, const StringRef& reply) override;
	void Init() override;
	static void InitStatic();
	TemperatureError GetTemperature(float& t) override;

private:
	static size_t numInstances;
	static DhtSensorType type;
	static TemperatureError lastResult;
	static float lastTemperature, lastHumidity;
	static size_t badTemperatureCount;
	static Mutex dhtMutex;

#ifndef RTOS
	static uint32_t lastReadTime;
	static enum SensorState
	{
		Initialising,
		Starting,
		Starting2,
		Reading
	} state;
	static uint32_t lastOperationTime;

	static void Spin();
#endif

	static void ProcessReadings();
};

#endif

#endif /* SRC_HEATING_SENSORS_DHTSENSOR_H_ */
