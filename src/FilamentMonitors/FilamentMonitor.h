/*
 * FilamentSensor.h
 *
 *  Created on: 20 Jul 2017
 *      Author: David
 */

#ifndef SRC_FILAMENTSENSORS_FILAMENTMONITOR_H_
#define SRC_FILAMENTSENSORS_FILAMENTMONITOR_H_

#include "RepRapFirmware.h"
#include "MessageType.h"
#include "GCodes/GCodeResult.h"
#include "RTOSIface/RTOSIface.h"

enum class FilamentSensorStatus : uint8_t
{
	ok,
	noFilament,
	tooLittleMovement,
	tooMuchMovement,
	sensorError
};

class FilamentMonitor
{
public:
	// Configure this sensor, returning true if error and setting 'seen' if we processed any configuration parameters
	virtual bool Configure(GCodeBuffer& gb, const StringRef& reply, bool& seen) = 0;

	// Call the following at intervals to check the status. This is only called when extrusion is in progress or imminent.
	// 'filamentConsumed' is the net amount of extrusion since the last call to this function.
	virtual FilamentSensorStatus Check(bool isPrinting, bool fromIsr, uint32_t isrMillis, float filamentConsumed) = 0;

	// Clear the measurement state - called when we are not printing a file. Return the present/not present status if available.
	virtual FilamentSensorStatus Clear() = 0;

	// Print diagnostic info for this sensor
	virtual void Diagnostics(MessageType mtype, unsigned int extruder) = 0;

	// ISR for when the pin state changes. It should return true if the ISR wants the commanded extrusion to be fetched.
	virtual bool Interrupt() = 0;

	// Call this to disable the interrupt before deleting a filament monitor
	virtual void Disable();

	// Override the virtual destructor if your derived class allocates any dynamic memory
	virtual ~FilamentMonitor();

	// Return the type of this sensor
	int GetType() const { return type; }

	// Static initialisation
	static void InitStatic();

	// Return an error message corresponding to a status code
	static const char *GetErrorMessage(FilamentSensorStatus f);

	// Poll the filament sensors
	static void Spin();

	// Handle M591
	static GCodeResult Configure(GCodeBuffer& gb, const StringRef& reply, unsigned int extruder)
	pre(extruder < MaxExtruders);

	// Send diagnostics info
	static void Diagnostics(MessageType mtype);

protected:
	FilamentMonitor(unsigned int extruder, int t) : extruderNumber(extruder), type(t), pin(NoPin) { }

	bool ConfigurePin(GCodeBuffer& gb, const StringRef& reply, InterruptMode interruptMode, bool& seen);

	int GetEndstopNumber() const { return endstopNumber; }

	Pin GetPin() const { return pin; }
	bool HaveIsrStepsCommanded() const { return haveIsrStepsCommanded; }

private:
	// Create a filament sensor returning null if not a valid sensor type
	static FilamentMonitor *Create(unsigned int extruder, int type);

	static void InterruptEntry(CallbackParameter param);

	static Mutex filamentSensorsMutex;
	static FilamentMonitor *filamentSensors[MaxExtruders];

	int32_t isrExtruderStepsCommanded;
	uint32_t isrMillis;
	unsigned int extruderNumber;
	int type;
	int endstopNumber;
	Pin pin;
	bool isrWasPrinting;
	bool haveIsrStepsCommanded;
};

#endif /* SRC_FILAMENTSENSORS_FILAMENTMONITOR_H_ */
