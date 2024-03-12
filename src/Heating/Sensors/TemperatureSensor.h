#ifndef TEMPERATURESENSOR_H
#define TEMPERATURESENSOR_H

#include <RepRapFirmware.h>
#include <TemperatureError.h>		// for result codes
#include <Hardware/IoPorts.h>
#include <ObjectModel/ObjectModel.h>

class GCodeBuffer;
class CanMessageGenericParser;
struct CanSensorReport;

class TemperatureSensor INHERIT_OBJECT_MODEL
{
public:
	TemperatureSensor(unsigned int sensorNum, const char *_ecv_array type) noexcept;
	TemperatureSensor(const TemperatureSensor &_ecv_from) = delete;

	// Virtual destructor
	virtual ~TemperatureSensor() noexcept;

	// Configure the sensor from M308 parameters.
	// If we find any parameters, process them, if successful then initialise the sensor and return GCodeResult::ok.
	// If an error occurs while processing the parameters, return GCodeResult::error and write an error message to 'reply.
	// if we find no relevant parameters, report the current parameters to 'reply' and return 'false'.
	virtual GCodeResult Configure(GCodeBuffer& gb, const StringRef& reply, bool& changed) THROWS(GCodeException);

	// Try to get a temperature reading
	virtual void Poll() noexcept = 0;

	// Try to get an additional output temperature reading
	virtual TemperatureError GetAdditionalOutput(float& t, uint8_t outputNumber) noexcept;

	// How many additional outputs does this sensor have
	virtual const uint8_t GetNumAdditionalOutputs() const noexcept { return 0; }

	// Get the smart drivers channel that this sensor monitors, or -1 if it doesn't
	virtual int GetSmartDriversChannel() const noexcept { return -1; }

	// How long after a reading before we consider the reading to be unreliable - this has to be increased for DHT sensors
	virtual uint32_t GetTemperatureReadingTimeout() const noexcept { return DefaultTemperatureReadingTimeout; }

	// Report the sensor type in the form corresponding to the Y parameter of M308.
	virtual const char *_ecv_array GetShortSensorType() const noexcept = 0;

#if SUPPORT_REMOTE_COMMANDS
	// Configure the sensor from M308 parameters.
	// If we find any parameters, process them and return true. If an error occurs while processing them, return error and write an error message to 'reply.
	// If we find no relevant parameters, report the current parameters to 'reply' and return ok.
	virtual GCodeResult Configure(const CanMessageGenericParser& parser, const StringRef& reply) noexcept;
#endif

#if SUPPORT_CAN_EXPANSION
	// Get the expansion board address. Overridden for remote sensors.
	virtual CanAddress GetBoardAddress() const noexcept;

	// Update the temperature, if it is a remote sensor. Overridden in class RemoteSensor.
	virtual void UpdateRemoteTemperature(CanAddress src, const CanSensorReport& report) noexcept;
#endif

	// Try to get a temperature reading
	TemperatureError GetLatestTemperature(float& t) noexcept;

	// Return the sensor type
	const char *_ecv_array GetSensorType() const noexcept { return sensorType; }

	// Return the sensor number
	unsigned int GetSensorNumber() const noexcept { return sensorNumber; }

	// Return the code for the most recent error
	TemperatureError GetLastError() const noexcept { return lastRealError; }

	// Set the name - normally called only once
	void SetSensorName(const char *_ecv_array _ecv_null newName) noexcept;

	// Get the name. Returns nullptr if no name has been assigned.
	const char *_ecv_array GetSensorName() const noexcept { return sensorName; }

	// Get/set the next sensor in the linked list
	TemperatureSensor *_ecv_from GetNext() const noexcept { return next; }
	void SetNext(TemperatureSensor *_ecv_from n) noexcept { next = n; }

	// Get the time of the last reading
	uint32_t GetLastReadingTime() const noexcept { return whenLastRead; }

	// Factory method
#if SUPPORT_CAN_EXPANSION
	static TemperatureSensor *_ecv_from Create(unsigned int sensorNum, CanAddress boardAddress, const char *_ecv_array typeName, const StringRef& reply) noexcept;
#else
	static TemperatureSensor *_ecv_from Create(unsigned int sensorNum, const char *_ecv_array typeName, const StringRef& reply) noexcept;
#endif

	static TemperatureError GetPT100Temperature(float& t, uint16_t ohmsx100) noexcept;		// shared function used by two derived classes and the ATE

protected:
	DECLARE_OBJECT_MODEL

	virtual void AppendPinDetails(const StringRef& reply) const noexcept { }				// append the details of the pin(s) used, only done for some sensor types

	void ConfigureCommonParameters(GCodeBuffer& gb, bool& seen) THROWS(GCodeException);		// configure the sensor name and reading adjustment parameters
#if SUPPORT_REMOTE_COMMANDS
	void ConfigureCommonParameters(const CanMessageGenericParser& parser, bool& seen) noexcept;	// configure the reading adjustment parameters
#endif
	void CopyBasicDetails(const StringRef& reply) const noexcept;							// copy the common details to the reply buffer - not called for remote sensors
	void SetResult(float t, TemperatureError rslt) noexcept;
	void SetResult(TemperatureError rslt) noexcept;

#if SUPPORT_CAN_EXPANSION
	void ClearAdjustments() noexcept { offsetAdjustment = slopeAdjustment = 0.0; }			// clear the adjustment parameters
#endif

private:
	static constexpr uint32_t DefaultTemperatureReadingTimeout = 2000;						// any reading older than this number of milliseconds is considered unreliable

	TemperatureSensor *_ecv_from _ecv_null next;
	unsigned int sensorNumber;																// the number of this sensor
	const char *_ecv_array const sensorType;
	const char *_ecv_array _ecv_null sensorName;
	float lastTemperature;
	uint32_t whenLastRead;
	float offsetAdjustment = 0.0;
	float slopeAdjustment = 0.0;
	volatile TemperatureError lastResult, lastRealError;
};

#endif // TEMPERATURESENSOR_H
