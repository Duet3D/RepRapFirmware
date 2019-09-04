#ifndef TEMPERATURESENSOR_H
#define TEMPERATURESENSOR_H

#include "RepRapFirmware.h"
#include "Heating/TemperatureError.h"		// for result codes
#include "Hardware/IoPorts.h"
#include "GCodes/GCodeResult.h"

class GCodeBuffer;
struct CanTemperatureReport;

class TemperatureSensor
{
public:
	TemperatureSensor(unsigned int sensorNum, const char *type);

	// Virtual destructor
	virtual ~TemperatureSensor();

	// Try to get a temperature reading
	TemperatureError GetLatestTemperature(float& t);

	// Get the most recent reading without checking for timeout
	float GetStoredReading() const { return lastTemperature; }

	// Configure the sensor from M308 parameters.
	// If we find any parameters, process them, if successful then initialise the sensor and return GCodeResult::ok.
	// If an error occurs while processing the parameters, return GCodeResult::error and write an error message to 'reply.
	// if we find no relevant parameters, report the current parameters to 'reply' and return 'false'.
	virtual GCodeResult Configure(GCodeBuffer& gb, const StringRef& reply);

	// Return the sensor type
	const char *GetSensorType() const { return sensorType; }

	// Return the sensor number
	unsigned int GetSensorNumber() const { return sensorNumber; }

	// Return the code for the most recent error
	TemperatureError GetLastError() const { return lastRealError; }

	// Configure the sensor name, if it is provided
	void TryConfigureSensorName(GCodeBuffer& gb, bool& seen);

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

#if SUPPORT_CAN_EXPANSION
	// Get the expansion board address. Overridden for remote sensors.
	virtual CanAddress GetBoardAddress() const { return 0; }

	// Update the temperature, if it is a remote sensor. Overridden in class RemoteSensor.
	virtual void UpdateRemoteTemperature(CanAddress src, const CanTemperatureReport& report) { }
#endif

	// Factory method
#if SUPPORT_CAN_EXPANSION
	static TemperatureSensor *Create(unsigned int sensorNum, CanAddress boardAddress, const char *typeName, const StringRef& reply);
#else
	static TemperatureSensor *Create(unsigned int sensorNum, const char *typeName, const StringRef& reply);
#endif

	// Try to get a temperature reading
	virtual void Poll() = 0;

protected:
	void SetResult(float t, TemperatureError rslt);
	void SetResult(TemperatureError rslt);

	static TemperatureError GetPT100Temperature(float& t, uint16_t ohmsx100);		// shared function used by two derived classes

private:
	static constexpr uint32_t TemperatureReadingTimeout = 2000;			// any reading older than this number of milliseconds is considered unreliable

	TemperatureSensor *next;
	unsigned int sensorNumber;											// the number of this sensor
	const char * const sensorType;
	const char *sensorName;
	float lastTemperature;
	uint32_t whenLastRead;
	TemperatureError lastResult, lastRealError;
};

#endif // TEMPERATURESENSOR_H
