/*
 * FilamentSensor.h
 *
 *  Created on: 20 Jul 2017
 *      Author: David
 */

#ifndef SRC_FILAMENTSENSORS_FILAMENTMONITOR_H_
#define SRC_FILAMENTSENSORS_FILAMENTMONITOR_H_

#include "RepRapFirmware.h"
#include "Hardware/IoPorts.h"
#include "MessageType.h"
#include "GCodes/GCodeResult.h"
#include <ObjectModel/ObjectModel.h>
#include "RTOSIface/RTOSIface.h"

enum class FilamentSensorStatus : uint8_t
{
	ok,
	noFilament,
	tooLittleMovement,
	tooMuchMovement,
	sensorError
};

class FilamentMonitor INHERIT_OBJECT_MODEL
{
public:
	// Configure this sensor, returning true if error and setting 'seen' if we processed any configuration parameters
	virtual bool Configure(GCodeBuffer& gb, const StringRef& reply, bool& seen) = 0;

	// Call the following at intervals to check the status. This is only called when extrusion is in progress or imminent.
	// 'filamentConsumed' is the net amount of extrusion since the last call to this function.
	virtual FilamentSensorStatus Check(bool isPrinting, bool fromIsr, uint32_t isrMillis, float filamentConsumed) noexcept = 0;

	// Clear the measurement state - called when we are not printing a file. Return the present/not present status if available.
	virtual FilamentSensorStatus Clear() noexcept = 0;

	// Print diagnostic info for this sensor
	virtual void Diagnostics(MessageType mtype, unsigned int extruder) noexcept = 0;

	// ISR for when the pin state changes. It should return true if the ISR wants the commanded extrusion to be fetched.
	virtual bool Interrupt() noexcept = 0;

	// Call this to disable the interrupt before deleting a filament monitor
	virtual void Disable() noexcept;

	// Override the virtual destructor if your derived class allocates any dynamic memory
	virtual ~FilamentMonitor() noexcept;

	// Return the type of this sensor
	unsigned int GetType() const noexcept { return type; }

	// Static initialisation
	static void InitStatic() noexcept;

	// Return an error message corresponding to a status code
	static const char *GetErrorMessage(FilamentSensorStatus f) noexcept;

	// Poll the filament sensors
	static void Spin() noexcept;

	// Handle M591
	static GCodeResult Configure(GCodeBuffer& gb, const StringRef& reply, unsigned int extruder)
	pre(extruder < MaxExtruders);

	// Send diagnostics info
	static void Diagnostics(MessageType mtype) noexcept;

#if SUPPORT_OBJECT_MODEL
	// Get the number of monitors to report in the OM
	static size_t GetNumMonitorsToReport();

	// Get access to a filament monitor when we already have a read lock
	static FilamentMonitor *GetMonitorAlreadyLocked(size_t extruder) { return filamentSensors[extruder]; }
#endif

	// This must be public so that the array descriptor in class RepRap can lock it
	static ReadWriteLock filamentMonitorsLock;

protected:
	FilamentMonitor(unsigned int extruder, unsigned int t) noexcept : extruderNumber(extruder), type(t) { }

	bool ConfigurePin(GCodeBuffer& gb, const StringRef& reply, InterruptMode interruptMode, bool& seen);

	const IoPort& GetPort() const noexcept { return port; }
	bool HaveIsrStepsCommanded() const noexcept { return haveIsrStepsCommanded; }

	static int32_t ConvertToPercent(float f)
	{
		return lrintf(100 * f);
	}

private:
	// Create a filament sensor returning null if not a valid sensor type
	static FilamentMonitor *Create(unsigned int extruder, unsigned int type) noexcept;
	static void InterruptEntry(CallbackParameter param) noexcept;

	static FilamentMonitor *filamentSensors[MaxExtruders];

	int32_t isrExtruderStepsCommanded;
	uint32_t isrMillis;
	unsigned int extruderNumber;
	unsigned int type;
	IoPort port;
	bool isrWasPrinting;
	bool haveIsrStepsCommanded;
};

#endif /* SRC_FILAMENTSENSORS_FILAMENTMONITOR_H_ */
