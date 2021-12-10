#ifndef TEMPERATURESENSOR_H
#define TEMPERATURESENSOR_H

#include <RepRapFirmware.h>
#include <Heating/TemperatureError.h>		// for result codes
#include <Hardware/IoPorts.h>
#include <ObjectModel/ObjectModel.h>

class GCodeBuffer;
class CanMessageGenericParser;
struct CanSensorReport;

class TemperatureSensor INHERIT_OBJECT_MODEL
{
public:
	TemperatureSensor(unsigned int sensorNum, const char *type) noexcept;
	TemperatureSensor(const TemperatureSensor&) = delete;

	// Virtual destructor
	virtual ~TemperatureSensor() noexcept;

	// Try to get a temperature reading
	virtual TemperatureError GetLatestTemperature(float& t, uint8_t outputNumber = 0) noexcept;

	// How many additional outputs does this sensor have
	virtual const uint8_t GetNumAdditionalOutputs() const noexcept { return 0; }

	// Get the most recent reading without checking for timeout
	float GetStoredReading() const noexcept { return lastTemperature; }

	// Configure the sensor from M308 parameters.
	// If we find any parameters, process them, if successful then initialise the sensor and return GCodeResult::ok.
	// If an error occurs while processing the parameters, return GCodeResult::error and write an error message to 'reply.
	// if we find no relevant parameters, report the current parameters to 'reply' and return 'false'.
	virtual GCodeResult Configure(GCodeBuffer& gb, const StringRef& reply, bool& changed) THROWS(GCodeException);

#if SUPPORT_REMOTE_COMMANDS
	// Configure the sensor from M308 parameters.
	// If we find any parameters, process them and return true. If an error occurs while processing them, return error and write an error message to 'reply.
	// If we find no relevant parameters, report the current parameters to 'reply' and return ok.
	virtual GCodeResult Configure(const CanMessageGenericParser& parser, const StringRef& reply) noexcept;
#endif

#if SUPPORT_OBJECT_MODEL
	// Report the sensor type in the form corresponding to the Y parameter of M308.
	virtual const char *GetShortSensorType() const noexcept = 0;
#endif

	// Return the sensor type
	const char *GetSensorType() const noexcept { return sensorType; }

	// Return the sensor number
	unsigned int GetSensorNumber() const noexcept { return sensorNumber; }

	// Return the code for the most recent error
	TemperatureError GetLastError() const noexcept { return lastRealError; }

	// Configure the sensor name, if it is provided
	void TryConfigureSensorName(GCodeBuffer& gb, bool& seen) THROWS(GCodeException);

	// Set the name - normally called only once
	void SetSensorName(const char *newName) noexcept;

	// Get the name. Returns nullptr if no name has been assigned.
	const char *GetSensorName() const noexcept { return sensorName; }

	// Copy the basic details to the reply buffer
	void CopyBasicDetails(const StringRef& reply) const noexcept;

	// Get/set the next sensor in the linked list
	TemperatureSensor *GetNext() const noexcept { return next; }
	void SetNext(TemperatureSensor *n) noexcept { next = n; }

	// Get the smart drivers channel that this sensor monitors, or -1 if it doesn't
	virtual int GetSmartDriversChannel() const noexcept { return -1; }

#if SUPPORT_CAN_EXPANSION
	// Get the expansion board address. Overridden for remote sensors.
	virtual CanAddress GetBoardAddress() const noexcept;

	// Update the temperature, if it is a remote sensor. Overridden in class RemoteSensor.
	virtual void UpdateRemoteTemperature(CanAddress src, const CanSensorReport& report) noexcept;
#endif

	// Factory method
#if SUPPORT_CAN_EXPANSION
	static TemperatureSensor *Create(unsigned int sensorNum, CanAddress boardAddress, const char *typeName, const StringRef& reply) noexcept;
#else
	static TemperatureSensor *Create(unsigned int sensorNum, const char *typeName, const StringRef& reply) noexcept;
#endif

	// Try to get a temperature reading
	virtual void Poll() noexcept = 0;

	static TemperatureError GetPT100Temperature(float& t, uint16_t ohmsx100) noexcept;		// shared function used by two derived classes and the ATE

protected:
	DECLARE_OBJECT_MODEL

	void SetResult(float t, TemperatureError rslt) noexcept;
	void SetResult(TemperatureError rslt) noexcept;

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
