/*
 * FilamentSensor.h
 *
 *  Created on: 20 Jul 2017
 *      Author: David
 */

#ifndef SRC_FILAMENTSENSORS_FILAMENTSENSOR_H_
#define SRC_FILAMENTSENSORS_FILAMENTSENSOR_H_

#include "RepRapFirmware.h"
#include "MessageType.h"

enum class FilamentSensorStatus : uint8_t
{
	ok,
	noFilament,
	tooLittleMovement,
	tooMuchMovement,
	sensorError
};

class FilamentSensor
{
public:
	// Configure this sensor, returning true if error and setting 'seen' if we processed any configuration parameters
	virtual bool Configure(GCodeBuffer& gb, StringRef& reply, bool& seen) = 0;

	// Call the following at intervals to check the status. This is only called when extrusion is in progress or imminent.
	// 'filamentConsumed' is the net amount of extrusion since the last call to this function.
	virtual FilamentSensorStatus Check(float filamentConsumed) = 0;

	// Clear the measurement state - called when we are not printing a file. Return the present/not present status if available.
	virtual FilamentSensorStatus Clear() = 0;

	// Print diagnostic info for this sensor
	virtual void Diagnostics(MessageType mtype, unsigned int extruder) = 0;

	// ISR for when the pin state changes
	virtual void Interrupt() = 0;

	// Override the virtual destructor if your derived class allocates any dynamic memory
	virtual ~FilamentSensor();

	// Return the type of this sensor
	int GetType() const { return type; }

	// Create a filament sensor returning null if not a valid sensor type
	static FilamentSensor *Create(int type);

	// Return an error message corresponding to a status code
	static const char *GetErrorMessage(FilamentSensorStatus f);

protected:
	FilamentSensor(int t) : type(t), pin(NoPin) { }

	bool ConfigurePin(GCodeBuffer& gb, StringRef& reply, bool& seen);

	int GetEndstopNumber() const { return endstopNumber; }

	Pin GetPin() const { return pin; }

private:
	static void InterruptEntry(void *param);

	int type;
	int endstopNumber;
	Pin pin;
};

#endif /* SRC_FILAMENTSENSORS_FILAMENTSENSOR_H_ */
