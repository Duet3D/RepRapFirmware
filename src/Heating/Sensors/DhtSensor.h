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

#include "TemperatureSensor.h"

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

	bool Configure(unsigned int mCode, unsigned int heater, GCodeBuffer& gb, const StringRef& reply, bool& error) override;
	void Init() override;
	TemperatureError GetTemperature(float& t) override;

private:
	static size_t numInstances;
	static DhtSensorType type;
	static uint32_t lastReadTime;
	static TemperatureError lastResult;
	static float lastTemperature, lastHumidity;

	static enum SensorState
	{
		Initialising,
		Starting,
		Starting2,
		Reading
	} state;
	static uint32_t lastOperationTime;

	static void Spin();
};

#endif

#endif /* SRC_HEATING_SENSORS_DHTSENSOR_H_ */
