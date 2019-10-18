/*
 * FilamentSensor.cpp
 *
 *  Created on: 20 Jul 2017
 *      Author: David
 */

#include "FilamentMonitor.h"
#include "SimpleFilamentMonitor.h"
#include "RotatingMagnetFilamentMonitor.h"
#include "LaserFilamentMonitor.h"
#include "PulsedFilamentMonitor.h"
#include "RepRap.h"
#include "Platform.h"
#include "GCodes/GCodeBuffer.h"
#include "Movement/Move.h"
#include "PrintMonitor.h"

// Static data
Mutex FilamentMonitor::filamentSensorsMutex;
FilamentMonitor *FilamentMonitor::filamentSensors[MaxExtruders] = { 0 };

// Default destructor
FilamentMonitor::~FilamentMonitor()
{
}

// Call this to disable the interrupt before deleting or re-configuring a filament monitor
void FilamentMonitor::Disable()
{
	if (pin != NoPin)
	{
		detachInterrupt(pin);
		pin = NoPin;
	}
}

// Try to get the pin number from the GCode command in the buffer, setting Seen if a pin number was provided and returning true if error.
// Also attaches the ISR.
bool FilamentMonitor::ConfigurePin(GCodeBuffer& gb, const StringRef& reply, InterruptMode interruptMode, bool& seen)
{
	if (gb.Seen('C'))
	{
		seen = true;
		// The C parameter is an endstop number in RRF
		const int endstop = gb.GetIValue();
		const Pin p = reprap.GetPlatform().GetEndstopPin(endstop);
		if (p == NoPin)
		{
			reply.copy("bad endstop number");
			return true;
		}
		endstopNumber = endstop;
		pin = p;
		haveIsrStepsCommanded = false;
		if (interruptMode != INTERRUPT_MODE_NONE && !attachInterrupt(pin, InterruptEntry, interruptMode, this))
		{
			reply.copy("unsuitable endstop number");
			return true;
		}
		setPullup(pin, false);				// disable the pullup resistor to provide greater noise immunity when using a Duet3D laser of magnetic filament monitor
	}
	else if (seen)
	{
		// We already had a P parameter, therefore it is an error not to have a C parameter too
		reply.copy("no endstop number given");
		return true;
	}
	return false;
}

// Static initialisation
/*static*/ void FilamentMonitor::InitStatic()
{
	filamentSensorsMutex.Create("FilamentSensors");
}

// Handle M591
/*static*/ GCodeResult FilamentMonitor::Configure(GCodeBuffer& gb, const StringRef& reply, unsigned int extruder)
{

	bool seen = false;
	long newSensorType;
	gb.TryGetIValue('P', newSensorType, seen);

	MutexLocker lock(filamentSensorsMutex);
	FilamentMonitor*& sensor = filamentSensors[extruder];

	if (seen)
	{
		// We are setting the filament monitor type, so see if it has changed
		if (sensor != nullptr && newSensorType != sensor->GetType())
		{
			// We already have a sensor of a different type, so delete the old sensor
			sensor->Disable();
			delete sensor;
			sensor = nullptr;
		}

		if (sensor == nullptr)
		{
			sensor = Create(extruder, newSensorType);					// create the new sensor type, if any
		}
	}

	if (sensor != nullptr)
	{
		// Configure the sensor
		const bool error = sensor->Configure(gb, reply, seen);
		if (error)
		{
			sensor->Disable();
			delete sensor;
			sensor = nullptr;
		}
		return GetGCodeResultFromError(error);
	}
	else if (!seen)
	{
		reply.printf("Extruder %u has no filament sensor", extruder);
	}
	return GCodeResult::ok;
}

// Factory function
/*static*/ FilamentMonitor *FilamentMonitor::Create(unsigned int extruder, int type)
{
	switch (type)
	{
	case 1:		// active high switch
	case 2:		// active low switch
		return new SimpleFilamentMonitor(extruder, type);
		break;

	case 3:		// duet3d rotating magnet, no switch
	case 4:		// duet3d rotating magnet + switch
		return new RotatingMagnetFilamentMonitor(extruder, type);

	case 5:		// duet3d laser, no switch
	case 6:		// duet3d laser + switch
		return new LaserFilamentMonitor(extruder, type);

	case 7:		// simple pulse output sensor
		return new PulsedFilamentMonitor(extruder, type);
		break;

	default:	// no sensor, or unknown sensor
		return nullptr;
	}
}

// Return an error message corresponding to a status code
/*static*/ const char *FilamentMonitor::GetErrorMessage(FilamentSensorStatus f)
{
	switch(f)
	{
	case FilamentSensorStatus::ok:					return "no error";
	case FilamentSensorStatus::noFilament:			return "no filament";
	case FilamentSensorStatus::tooLittleMovement:	return "too little movement";
	case FilamentSensorStatus::tooMuchMovement:		return "too much movement";
	case FilamentSensorStatus::sensorError:			return "sensor not working";
	default:										return "unknown error";
	}
}

// ISR
/*static*/ void FilamentMonitor::InterruptEntry(CallbackParameter param)
{
	FilamentMonitor * const fm = static_cast<FilamentMonitor*>(param.vp);
	if (fm->Interrupt())
	{
		fm->isrExtruderStepsCommanded = reprap.GetMove().GetAccumulatedExtrusion(fm->extruderNumber, fm->isrWasPrinting);
		fm->haveIsrStepsCommanded = true;
		fm->isrMillis = millis();
	}
}

/*static*/ void FilamentMonitor::Spin()
{
	MutexLocker lock(filamentSensorsMutex);

	// Filament sensors
	for (size_t extruder = 0; extruder < MaxExtruders; ++extruder)
	{
		if (filamentSensors[extruder] != nullptr)
		{
			FilamentMonitor& fs = *filamentSensors[extruder];
			GCodes& gCodes = reprap.GetGCodes();
			bool isPrinting;
			bool fromIsr;
			int32_t extruderStepsCommanded;
			uint32_t isrMillis;
			cpu_irq_disable();
			if (fs.haveIsrStepsCommanded)
			{
				extruderStepsCommanded = fs.isrExtruderStepsCommanded;
				isPrinting = fs.isrWasPrinting;
				isrMillis = fs.isrMillis;
				fs.haveIsrStepsCommanded = false;
				cpu_irq_enable();
				fromIsr = true;
			}
			else
			{
				cpu_irq_enable();
				extruderStepsCommanded = reprap.GetMove().GetAccumulatedExtrusion(extruder, isPrinting);		// get and clear the net extrusion commanded
				fromIsr = false;
				isrMillis = 0;
			}
			if (gCodes.IsReallyPrinting() && !gCodes.IsSimulating())
			{
				const float extrusionCommanded = (float)extruderStepsCommanded/reprap.GetPlatform().DriveStepsPerUnit(extruder + gCodes.GetTotalAxes());
				const FilamentSensorStatus fstat = fs.Check(isPrinting, fromIsr, isrMillis, extrusionCommanded);
				if (fstat != FilamentSensorStatus::ok)
				{
					if (reprap.Debug(moduleFilamentSensors))
					{
						debugPrintf("Filament error: extruder %u reports %s\n", extruder, FilamentMonitor::GetErrorMessage(fstat));
					}
					else
					{
						gCodes.FilamentError(extruder, fstat);
					}
				}
			}
			else
			{
				fs.Clear();
			}
		}
	}
}

// Send diagnostics info
/*static*/ void FilamentMonitor::Diagnostics(MessageType mtype)
{
	bool first = true;
	for (size_t i = 0; i < MaxExtruders; ++i)
	{
		if (filamentSensors[i] != nullptr)
		{
			if (first)
			{
				reprap.GetPlatform().Message(mtype, "=== Filament sensors ===\n");
				first = false;
			}
			filamentSensors[i]->Diagnostics(mtype, i);
		}
	}
}

// End
