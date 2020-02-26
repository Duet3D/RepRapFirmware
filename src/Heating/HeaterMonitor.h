/*
 * HeaterMonitor.h
 *
 *  Created on: 16 Nov 2017
 *      Author: Christian
 */

#ifndef HEATERMONITOR_H
#define HEATERMONITOR_H

#include <RepRapFirmware.h>
#include <General/FreelistManager.h>

// Condition of a heater monitor event
enum class HeaterMonitorTrigger : int8_t
{
	Disabled = -1,
	TemperatureExceeded = 0,
	TemperatureTooLow
};

const HeaterMonitorTrigger MaxHeaterMonitorTrigger = HeaterMonitorTrigger::TemperatureTooLow;

// The action to trigger when the target condition is met
enum class HeaterMonitorAction : uint8_t
{
	GenerateFault = 0,
	PermanentSwitchOff,
	TemporarySwitchOff,
	ShutDown
};

const HeaterMonitorAction MaxHeaterMonitorAction = HeaterMonitorAction::ShutDown;

// A note about using this class. Its size is currently 8 bytes, and will be 12 bytes of object model support is added.
// - If we allocate them statically within the heater object, then assuming 3 per heater we need 24 bytes, or 36 bytes with OM support.
// - If we allocate them dynamically then there is an overhead of at least 8 bytes per object. All heaters have at least 2.
//   So each heater needs 12 bytes for the pointers plus 32 bytes (without OM support) or 40 bytes (with OM support). Total 44 or 52 bytes.
// For unconfigured heaters, the cost is 24 or 36 bytes each using static allocation, or 4 bytes each using dynamic allocation.
// Summary:
// - Static allocation saves 20 bytes (no OM) or 16 bytes (with OM) per configured heater, if the 3rd heater monitor is not used (and more if it is used)
// - Dynamic allocation saves 20 bytes (no OM) or 32 bytes (with OM) per unconfigured heater
// For now we use static allocation, i.e. we embed the heater monitor in the heater object.
class HeaterMonitor
{
public:
	HeaterMonitor() noexcept;

	void Set(int sn, float lim, HeaterMonitorAction act, HeaterMonitorTrigger trig) noexcept;
	void Disable() noexcept;
	bool Check() noexcept;												// Check if any action needs to be taken

	int GetSensorNumber() const noexcept { return sensorNumber; }		// Get the supervisory sensor number
	float GetTemperatureLimit() const noexcept { return limit; }		// Get the temperature limit
	HeaterMonitorAction GetAction() const noexcept { return action; }	// Get the action to trigger when a temperature event occurs
	HeaterMonitorTrigger GetTrigger() const noexcept { return trigger; }	// Get the condition for a temperature event
	const char *GetTriggerName() const noexcept;						// Get the condition for a temperature event

	void Report(unsigned int heater, unsigned int index, const StringRef& reply) const noexcept;	// Append a report of this monitor to the string

private:
	float limit;														// temperature limit
	int8_t sensorNumber;												// the sensor that we use to monitor the heater
	HeaterMonitorAction action;											// what action we take of we detect a fault
	HeaterMonitorTrigger trigger;										// what is treated a fault
	uint8_t badTemperatureCount;										// how many consecutive sensor reading faults we have had
};

inline void HeaterMonitor::Set(int sn, float lim, HeaterMonitorAction act, HeaterMonitorTrigger trig) noexcept
{
	sensorNumber = sn;
	limit = lim;
	action = act;
	trigger = trig;
	badTemperatureCount = 0;
}

inline void HeaterMonitor::Disable() noexcept
{
	trigger = HeaterMonitorTrigger::Disabled;
}

#endif
