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
#include "GCodes/GCodeBuffer/GCodeBuffer.h"
#include "Movement/Move.h"
#include "PrintMonitor.h"

#if SUPPORT_CAN_EXPANSION
# include <CAN/CanInterface.h>
#endif

// Static data
ReadWriteLock FilamentMonitor::filamentMonitorsLock;
FilamentMonitor *FilamentMonitor::filamentSensors[MaxExtruders] = { 0 };

#if SUPPORT_OBJECT_MODEL

// Get the number of monitors to report in the OM
size_t FilamentMonitor::GetNumMonitorsToReport() noexcept
{
	size_t rslt = ARRAY_SIZE(filamentSensors);
	while (rslt != 0 && filamentSensors[rslt - 1] == nullptr)
	{
		--rslt;
	}
	return rslt;
}

#endif

// Constructor
FilamentMonitor::FilamentMonitor(unsigned int extruder, unsigned int t) noexcept
	: extruderNumber(extruder), type(t), lastStatus(FilamentSensorStatus::noDataReceived)
#if SUPPORT_CAN_EXPANSION
	  , hasRemote(false)
#endif
{
	driver = reprap.GetPlatform().GetExtruderDriver(extruder);
}

// Default destructor
FilamentMonitor::~FilamentMonitor() noexcept
{
#if SUPPORT_CAN_EXPANSION
	if (!IsLocal() && hasRemote)
	{
		String<1> dummy;
		(void)CanInterface::DeleteFilamentMonitor(driver, nullptr, dummy.GetRef());
	}
#endif
}

// Call this to disable the interrupt before deleting or re-configuring a local filament monitor
void FilamentMonitor::Disable() noexcept
{
	port.Release();
}

// Do the configuration that is
// Try to get the pin number from the GCode command in the buffer, setting Seen if a pin number was provided and returning true if error.
// Also attaches the ISR.
// For a remote filament monitor, this does the full configuration or query of the remote object instead, and we always return seen true because we don't need to report local status.
GCodeResult FilamentMonitor::CommonConfigure(GCodeBuffer& gb, const StringRef& reply, InterruptMode interruptMode, bool& seen) noexcept
{
#if SUPPORT_CAN_EXPANSION
	// Check that the port (if given) is on the same board as the extruder
	String<StringLength20> portName;
	if (gb.TryGetQuotedString('C', portName.GetRef(), seen))
	{
		const CanAddress portAddress = IoPort::RemoveBoardAddress(portName.GetRef());
		if (portAddress != driver.boardAddress)
		{
			reply.copy("Filament monitor port must be on same board as extruder driver");
			return GCodeResult::error;
		}
	}

	if (!IsLocal())
	{
		seen = true;				// this tells the local filament monitor not to report anything
		return CanInterface::ConfigureFilamentMonitor(driver, gb, reply);
	}
#endif

	if (gb.Seen('C'))
	{
		seen = true;
		if (!port.AssignPort(gb, reply, PinUsedBy::filamentMonitor, PinAccess::read))
		{
			return GCodeResult::error;
		}

		haveIsrStepsCommanded = false;
		if (interruptMode != INTERRUPT_MODE_NONE && !port.AttachInterrupt(InterruptEntry, interruptMode, this))
		{
			reply.copy("unsuitable pin");
			return GCodeResult::error;
		}
	}
	return GCodeResult::ok;
}

// Check that the extruder referenced by this filament monitor is still valid
bool FilamentMonitor::IsValid() const noexcept
{
	return extruderNumber < reprap.GetGCodes().GetNumExtruders() && reprap.GetPlatform().GetExtruderDriver(extruderNumber) == driver;
}

// Static initialisation
/*static*/ void FilamentMonitor::InitStatic() noexcept
{
	// Nothing needed here yet
}

// Handle M591
/*static*/ GCodeResult FilamentMonitor::Configure(GCodeBuffer& gb, const StringRef& reply, unsigned int extruder) THROWS(GCodeException)
{
	bool seen = false;
	uint32_t newSensorType;
	gb.TryGetUIValue('P', newSensorType, seen);

	WriteLocker lock(filamentMonitorsLock);
	FilamentMonitor* sensor = filamentSensors[extruder];

	if (seen)
	{
		// Creating a filament monitor. First delete the old one for this extruder.
		if (sensor != nullptr)
		{
			sensor->Disable();
			sensor = nullptr;
			std::swap(sensor, filamentSensors[extruder]);
			delete sensor;
			reprap.SensorsUpdated();
		}

		gb.MustSee('C');														// make sure the port name parameter is present
		sensor = Create(extruder, newSensorType, gb, reply);					// create the new sensor
		if (sensor == nullptr)
		{
			return GCodeResult::error;
		}

		try
		{
			const GCodeResult rslt = sensor->Configure(gb, reply, seen);		// configure the sensor (may throw)
			if (rslt <= GCodeResult::warning)
			{
				filamentSensors[extruder] = sensor;
				reprap.SensorsUpdated();
			}
			else
			{
				delete sensor;
			}
			return rslt;
		}
		catch (...)
		{
			delete sensor;
			throw;
		}
	}

	// Here if configuring or reporting on an existing filament monitor
	if (sensor == nullptr)
	{
		reply.printf("Extruder %u has no filament sensor", extruder);
		return GCodeResult::ok;
	}

	return sensor->Configure(gb, reply, seen);									// configure or report on the existing sensor (may throw)
}

// Factory function to create a filament monitor
/*static*/ FilamentMonitor *FilamentMonitor::Create(unsigned int extruder, unsigned int monitorType, GCodeBuffer& gb, const StringRef& reply) noexcept
{
	FilamentMonitor *fm;
	switch (monitorType)
	{
	case 1:		// active high switch
	case 2:		// active low switch
		fm = new SimpleFilamentMonitor(extruder, monitorType);
		break;

	case 3:		// duet3d rotating magnet, no switch
	case 4:		// duet3d rotating magnet + switch
		fm = new RotatingMagnetFilamentMonitor(extruder, monitorType);
		break;

	case 5:		// duet3d laser, no switch
	case 6:		// duet3d laser + switch
		fm = new LaserFilamentMonitor(extruder, monitorType);
		break;

	case 7:		// simple pulse output sensor
		fm = new PulsedFilamentMonitor(extruder, monitorType);
		break;

	default:	// no sensor, or unknown sensor
		reply.printf("Unknown filament monitor type %u", monitorType);
		return nullptr;
	}
#if SUPPORT_CAN_EXPANSION
	if (fm != nullptr && !fm->IsLocal())
	{
		// Create the remote filament monitor on the expansion board
		if (CanInterface::CreateFilamentMonitor(fm->driver, monitorType, gb, reply) != GCodeResult::ok)
		{
			delete fm;
			return nullptr;
		}
		fm->hasRemote = true;
	}
#endif
	return fm;
}

// ISR
/*static*/ void FilamentMonitor::InterruptEntry(CallbackParameter param) noexcept
{
	FilamentMonitor * const fm = static_cast<FilamentMonitor*>(param.vp);
	if (fm->Interrupt())
	{
		fm->isrExtruderStepsCommanded = reprap.GetMove().GetAccumulatedExtrusion(fm->extruderNumber, fm->isrWasPrinting);
		fm->haveIsrStepsCommanded = true;
		fm->lastIsrMillis = millis();
	}
}

/*static*/ void FilamentMonitor::Spin() noexcept
{
	ReadLocker lock(filamentMonitorsLock);

	for (size_t extruder = 0; extruder < MaxExtruders; ++extruder)
	{
		if (filamentSensors[extruder] != nullptr)
		{
			FilamentMonitor& fs = *filamentSensors[extruder];
#if SUPPORT_CAN_EXPANSION
			if (fs.IsLocal())
#endif
			{
				bool isPrinting;
				bool fromIsr;
				int32_t extruderStepsCommanded;
				uint32_t locIsrMillis;
				cpu_irq_disable();
				if (fs.haveIsrStepsCommanded)
				{
					extruderStepsCommanded = fs.isrExtruderStepsCommanded;
					isPrinting = fs.isrWasPrinting;
					locIsrMillis = fs.lastIsrMillis;
					fs.haveIsrStepsCommanded = false;
					cpu_irq_enable();
					fromIsr = true;
				}
				else
				{
					cpu_irq_enable();
					extruderStepsCommanded = reprap.GetMove().GetAccumulatedExtrusion(extruder, isPrinting);		// get and clear the net extrusion commanded
					fromIsr = false;
					locIsrMillis = 0;
				}

				GCodes& gCodes = reprap.GetGCodes();
				if (gCodes.IsReallyPrinting() && !gCodes.IsSimulating())
				{
					const float extrusionCommanded = (float)extruderStepsCommanded/reprap.GetPlatform().DriveStepsPerUnit(ExtruderToLogicalDrive(extruder));
					const FilamentSensorStatus fstat = fs.Check(isPrinting, fromIsr, locIsrMillis, extrusionCommanded);
					fs.lastStatus = fstat;
					if (fstat != FilamentSensorStatus::ok)
					{
						if (reprap.Debug(moduleFilamentSensors))
						{
							debugPrintf("Filament error: extruder %u reports %s\n", extruder, fstat.ToString());
						}
						else
						{
							gCodes.FilamentError(extruder, fstat);
						}
					}
				}
				else
				{
					fs.lastStatus = fs.Clear();
				}
			}
		}
	}
}

#if SUPPORT_CAN_EXPANSION

/*static*/ void FilamentMonitor::UpdateRemoteFilamentStatus(CanAddress src, CanMessageFilamentMonitorsStatus& msg) noexcept
{
	ReadLocker lock(filamentMonitorsLock);

	for (size_t extruder = 0; extruder < MaxExtruders; ++extruder)
	{
		if (filamentSensors[extruder] != nullptr)
		{
			FilamentMonitor& fs = *filamentSensors[extruder];
			if (fs.driver.boardAddress == src && fs.driver.localDriver < msg.numMonitorsReported)
			{
				const FilamentSensorStatus fstat(msg.data[fs.driver.localDriver].status);
				fs.lastStatus = fstat;
				GCodes& gCodes = reprap.GetGCodes();
				if (gCodes.IsReallyPrinting() && !gCodes.IsSimulating())
				{
					if (fstat != FilamentSensorStatus::ok)
					{
						if (reprap.Debug(moduleFilamentSensors))
						{
							debugPrintf("Filament error: extruder %u reports %s\n", extruder, fstat.ToString());
						}
						else
						{
							gCodes.FilamentError(extruder, fstat);
						}
					}
				}
			}
		}
	}
}

#endif

// Close down the filament monitors, in particular stop them generating interrupts. Called when we are about to update firmware.
/*static*/ void FilamentMonitor::Exit() noexcept
{
	WriteLocker lock(filamentMonitorsLock);

	for (FilamentMonitor *&f : filamentSensors)
	{
		FilamentMonitor *temp;
		std::swap(temp, f);
		delete temp;
	}
}

// Send diagnostics info
/*static*/ void FilamentMonitor::Diagnostics(MessageType mtype) noexcept
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

// Check whether the drivers that filament monitor are attached to are still valid. If any are invalid, delete them, append a warning to 'reply', and return true.
// This is needed because when supporting CAN, we don't want to handle extruders that move from one driver to another.
/*static*/ bool FilamentMonitor::CheckDriveAssignments(const StringRef &reply) noexcept
{
	bool warn = false;
	WriteLocker lock(filamentMonitorsLock);

	for (size_t extruder = 0; extruder < MaxExtruders; ++extruder)
	{
		if (filamentSensors[extruder] != nullptr && !filamentSensors[extruder]->IsValid())
		{
			reply.lcatf("Filament monitor for extruder %u has been deleted due to configuration change", extruder);
			warn = true;
			FilamentMonitor *f = nullptr;
			std::swap(f, filamentSensors[extruder]);
			delete f;
		}
	}
	return warn;
}

// End
