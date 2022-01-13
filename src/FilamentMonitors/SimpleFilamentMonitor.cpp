/*
 * SimpleFilamentSensor.cpp
 *
 *  Created on: 20 Jul 2017
 *      Author: David
 */

#include "SimpleFilamentMonitor.h"
#include <Platform/RepRap.h>
#include <Platform/Platform.h>
#include <GCodes/GCodeBuffer/GCodeBuffer.h>

#if SUPPORT_REMOTE_COMMANDS
# include <CanMessageGenericParser.h>
#endif

#if SUPPORT_OBJECT_MODEL

// Object model table and functions
// Note: if using GCC version 7.3.1 20180622 and lambda functions are used in this table, you must compile this file with option -std=gnu++17.
// Otherwise the table will be allocated in RAM instead of flash, which wastes too much RAM.

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(...) OBJECT_MODEL_FUNC_BODY(SimpleFilamentMonitor, __VA_ARGS__)

constexpr ObjectModelTableEntry SimpleFilamentMonitor::objectModelTable[] =
{
	// Within each group, these entries must be in alphabetical order
	{ "enabled",			OBJECT_MODEL_FUNC(self->enabled),		 			ObjectModelEntryFlags::none },
	{ "status",				OBJECT_MODEL_FUNC(self->GetStatusText()),			ObjectModelEntryFlags::live },
	{ "type",				OBJECT_MODEL_FUNC_NOSELF("simple"), 				ObjectModelEntryFlags::none },
};

constexpr uint8_t SimpleFilamentMonitor::objectModelTableDescriptor[] = { 1, 3 };

DEFINE_GET_OBJECT_MODEL_TABLE(SimpleFilamentMonitor)

#endif

SimpleFilamentMonitor::SimpleFilamentMonitor(unsigned int drv, unsigned int monitorType, DriverId did) noexcept
	: FilamentMonitor(drv, monitorType, did), highWhenNoFilament(monitorType == 2), filamentPresent(false), enabled(false)
{
}

// Configure this sensor, returning true if error and setting 'seen' if we processed any configuration parameters
GCodeResult SimpleFilamentMonitor::Configure(GCodeBuffer& gb, const StringRef& reply, bool& seen) THROWS(GCodeException)
{
	const GCodeResult rslt = CommonConfigure(gb, reply, InterruptMode::none, seen);
	if (Succeeded(rslt))
	{
		if (gb.Seen('S'))
		{
			seen = true;
			enabled = (gb.GetIValue() > 0);
		}

		if (seen)
		{
			Check(false, false, 0, 0.0);
			reprap.SensorsUpdated();
		}
		else
		{
			reply.copy("Simple filament sensor on pin ");
			GetPort().AppendPinName(reply);
			reply.catf(", %s, output %s when no filament, filament present: %s",
						(enabled) ? "enabled" : "disabled",
						(highWhenNoFilament) ? "high" : "low",
						(filamentPresent) ? "yes" : "no");
		}
	}
	return rslt;
}

// ISR for when the pin state changes
bool SimpleFilamentMonitor::Interrupt() noexcept
{
	// Nothing needed here
	GetPort().DetachInterrupt();
	return false;
}

// Call the following regularly to keep the status up to date
void SimpleFilamentMonitor::Poll() noexcept
{
	const bool b = GetPort().ReadDigital();
	filamentPresent = (highWhenNoFilament) ? !b : b;
}

// Call the following at intervals to check the status. This is only called when extrusion is in progress or imminent.
// 'filamentConsumed' is the net amount of extrusion since the last call to this function.
FilamentSensorStatus SimpleFilamentMonitor::Check(bool isPrinting, bool fromIsr, uint32_t isrMillis, float filamentConsumed) noexcept
{
	Poll();
	return (!enabled || filamentPresent) ? FilamentSensorStatus::ok : FilamentSensorStatus::noFilament;
}

// Clear the measurement state - called when we are not printing a file. Return the present/not present status if available.
FilamentSensorStatus SimpleFilamentMonitor::Clear() noexcept
{
	Poll();
	return (!enabled || filamentPresent) ? FilamentSensorStatus::ok : FilamentSensorStatus::noFilament;
}

// Print diagnostic info for this sensor
void SimpleFilamentMonitor::Diagnostics(MessageType mtype, unsigned int extruder) noexcept
{
	Poll();
	reprap.GetPlatform().MessageF(mtype, "Extruder %u sensor: %s\n", extruder, (filamentPresent) ? "ok" : "no filament");
}

#if SUPPORT_REMOTE_COMMANDS

// Configure this sensor, returning true if error and setting 'seen' if we processed any configuration parameters
GCodeResult SimpleFilamentMonitor::Configure(const CanMessageGenericParser& parser, const StringRef& reply) noexcept
{
	bool seen = false;
	const GCodeResult rslt = CommonConfigure(parser, reply, InterruptMode::none, seen);
	if (rslt <= GCodeResult::warning)
	{
		uint16_t temp;
		if (parser.GetUintParam('S', temp))
		{
			seen = true;
			enabled = (temp > 0);
		}


		if (seen)
		{
			Check(false, false, 0, 0.0);
		}
		else
		{
			reply.copy("Simple filament sensor on pin ");
			GetPort().AppendPinName(reply);
			reply.catf(", %s, output %s when no filament, filament present: %s",
						(enabled) ? "enabled" : "disabled",
						(highWhenNoFilament) ? "high" : "low",
						(filamentPresent) ? "yes" : "no");
		}
	}
	return rslt;
}

#endif

// End
