#ifndef TEMPERATURESENSOR_H
#define TEMPERATURESENSOR_H

#include "RepRapFirmware.h"
#include "Heating/TemperatureError.h"		// for result codes
#include "Hardware/IoPorts.h"
#include "GCodes/GCodeResult.h"

class GCodeBuffer;

class TemperatureSensor
{
public:
	TemperatureSensor(unsigned int sensorNum, const char *type);

	// Try to get a temperature reading
	TemperatureError GetTemperature(float& t);

	// Configure the sensor from M305 parameters.
	// If we find any parameters, process them and return true. If an error occurs while processing them, set 'error' to true and write an error message to 'reply.
	// if we find no relevant parameters, report the current parameters to 'reply' and return 'false'.
	virtual GCodeResult Configure(GCodeBuffer& gb, const StringRef& reply);

	// Initialise or re-initialise the temperature sensor
	virtual void Init() = 0;

	// Return the sensor type
	const char *GetSensorType() const { return sensorType; }

	// Return the sensor number
	unsigned int GetSensorNumber() const { return sensorNumber; }

	TemperatureError GetLastError() const { return lastError; }

	// Configure then heater name, if it is provided
	void TryConfigureSensorName(GCodeBuffer& gb, bool& seen);

	// Virtual destructor
	virtual ~TemperatureSensor();

	// Set the name - normally called only once
	void SetSensorName(const char *newName);

	// Get the name. Returns nullptr if no name has been assigned.
	const char *GetSensorName() const { return sensorName; }

	// Copy the basic details to the reply buffer
	void CopyBasicDetails(const StringRef& reply) const;

	// Get/set the next sensor in the linked list
	TemperatureSensor *GetNext() const { return next; }
	void SetNext(TemperatureSensor *n) { next = n; }

	// Get the smart drivers channel that this sensor monitors, or -1 if it doesn't
	virtual int GetSmartDriversChannel() const { return -1; }

	// Factory method
	static TemperatureSensor *Create(unsigned int sensorNum, const char *typeName);

protected:
	// Try to get a temperature reading
	virtual TemperatureError TryGetTemperature(float& t) = 0;

	static TemperatureError GetPT100Temperature(float& t, uint16_t ohmsx100);		// shared function used by two derived classes

private:
	TemperatureSensor *next;
	unsigned int sensorNumber;
	const char * const sensorType;
	const char *sensorName;
	TemperatureError lastError;
};

#endif // TEMPERATURESENSOR_H
