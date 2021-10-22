/*
 * FilamentMonitor.cpp
 *
 *  Created on: 20 Jul 2017
 *      Author: David
 */

#include "FilamentMonitor.h"
#include "SimpleFilamentMonitor.h"
#include "RotatingMagnetFilamentMonitor.h"
#include "LaserFilamentMonitor.h"
#include "PulsedFilamentMonitor.h"
#include <Platform/RepRap.h>
#include <Platform/Platform.h>
#include <GCodes/GCodeBuffer/GCodeBuffer.h>
#include <Movement/Move.h>
#include <PrintMonitor/PrintMonitor.h>

#if SUPPORT_CAN_EXPANSION
# include <CAN/CanInterface.h>
#endif

#if SUPPORT_REMOTE_COMMANDS
# include <CanMessageFormats.h>
# include <CanMessageGenericParser.h>
# include <CanMessageGenericTables.h>
#endif

// Static data
ReadWriteLock FilamentMonitor::filamentMonitorsLock;
FilamentMonitor *FilamentMonitor::filamentSensors[NumFilamentMonitors] = { 0 };

#if SUPPORT_REMOTE_COMMANDS
uint32_t FilamentMonitor::whenStatusLastSent = 0;
#endif

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
FilamentMonitor::FilamentMonitor(unsigned int drv, unsigned int monitorType, DriverId did) noexcept
	: driveNumber(drv), type(monitorType), driverId(did), lastStatus(FilamentSensorStatus::noDataReceived)
#if SUPPORT_CAN_EXPANSION
	  , hasRemote(false)
#endif
{
}

// Default destructor
FilamentMonitor::~FilamentMonitor() noexcept
{
#if SUPPORT_CAN_EXPANSION
	if (!IsLocal() && hasRemote)
	{
		String<1> dummy;
		(void)CanInterface::DeleteFilamentMonitor(driverId, nullptr, dummy.GetRef());
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
GCodeResult FilamentMonitor::CommonConfigure(GCodeBuffer& gb, const StringRef& reply, InterruptMode interruptMode, bool& seen) THROWS(GCodeException)
{
#if SUPPORT_CAN_EXPANSION
	// Check that the port (if given) is on the same board as the extruder
	String<StringLength20> portName;
	if (gb.TryGetQuotedString('C', portName.GetRef(), seen))
	{
		const CanAddress portAddress = IoPort::RemoveBoardAddress(portName.GetRef());
		if (portAddress != driverId.boardAddress)
		{
			reply.copy("Filament monitor port must be on same board as extruder driver");
			return GCodeResult::error;
		}
	}

	if (!IsLocal())
	{
		seen = true;				// this tells the local filament monitor not to report anything
		return CanInterface::ConfigureFilamentMonitor(driverId, gb, reply);
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
		if (interruptMode != InterruptMode::none && !port.AttachInterrupt(InterruptEntry, interruptMode, this))
		{
			reply.copy("unsuitable pin");
			return GCodeResult::error;
		}
	}
	return GCodeResult::ok;
}

// Check that the extruder referenced by this filament monitor is still valid
bool FilamentMonitor::IsValid(size_t extruderNumber) const noexcept
{
	return extruderNumber < reprap.GetGCodes().GetNumExtruders()
		&& reprap.GetPlatform().GetExtruderDriver(extruderNumber) == driverId;
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
			DeleteObject(filamentSensors[extruder]);
			reprap.SensorsUpdated();
		}

		if (newSensorType == 0)
		{
			return GCodeResult::ok;												// M591 D# P0 just deletes any existing sensor
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
	const DriverId did = reprap.GetPlatform().GetExtruderDriver(extruder);
	FilamentMonitor *fm;
	switch (monitorType)
	{
	case 1:		// active high switch
	case 2:		// active low switch
		fm = new SimpleFilamentMonitor(extruder, monitorType, did);
		break;

	case 3:		// duet3d rotating magnet, no switch
	case 4:		// duet3d rotating magnet + switch
		fm = new RotatingMagnetFilamentMonitor(extruder, monitorType, did);
		break;

	case 5:		// duet3d laser, no switch
	case 6:		// duet3d laser + switch
		fm = new LaserFilamentMonitor(extruder, monitorType, did);
		break;

	case 7:		// simple pulse output sensor
		fm = new PulsedFilamentMonitor(extruder, monitorType, did);
		break;

	default:	// no sensor, or unknown sensor
		reply.printf("Unknown filament monitor type %u", monitorType);
		return nullptr;
	}
#if SUPPORT_CAN_EXPANSION
	if (fm != nullptr && !fm->IsLocal())
	{
		// Create the remote filament monitor on the expansion board
		if (CanInterface::CreateFilamentMonitor(fm->driverId, monitorType, gb, reply) != GCodeResult::ok)
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
		fm->isrExtruderStepsCommanded = reprap.GetMove().GetAccumulatedExtrusion(fm->driveNumber, fm->isrWasPrinting);
		fm->haveIsrStepsCommanded = true;
		fm->lastIsrMillis = millis();
	}
}

/*static*/ void FilamentMonitor::Spin() noexcept
{
#if SUPPORT_REMOTE_COMMANDS
	CanMessageBuffer buf(nullptr);
	auto msg = buf.SetupStatusMessage<CanMessageFilamentMonitorsStatus>(CanInterface::GetCanAddress(), CanInterface::GetCurrentMasterAddress());
	bool statusChanged = false;
	bool haveMonitor = false;
#endif

	ReadLocker lock(filamentMonitorsLock);

	for (size_t drv = 0; drv < NumFilamentMonitors; ++drv)
	{
		FilamentSensorStatus fst(FilamentSensorStatus::noMonitor);
		if (filamentSensors[drv] != nullptr)
		{
#if SUPPORT_REMOTE_COMMANDS
			haveMonitor = true;
#endif
			FilamentMonitor& fs = *filamentSensors[drv];
#if SUPPORT_CAN_EXPANSION
			if (fs.IsLocal())
#endif
			{
				bool isPrinting;
				bool fromIsr;
				int32_t extruderStepsCommanded;
				uint32_t locIsrMillis;
				IrqDisable();
				if (fs.haveIsrStepsCommanded)
				{
					extruderStepsCommanded = fs.isrExtruderStepsCommanded;
					isPrinting = fs.isrWasPrinting;
					locIsrMillis = fs.lastIsrMillis;
					fs.haveIsrStepsCommanded = false;
					IrqEnable();
					fromIsr = true;
				}
				else
				{
					IrqEnable();
					extruderStepsCommanded = reprap.GetMove().GetAccumulatedExtrusion(fs.driveNumber, isPrinting);		// get and clear the net extrusion commanded
					fromIsr = false;
					locIsrMillis = 0;
				}

				GCodes& gCodes = reprap.GetGCodes();
				if (gCodes.IsReallyPrinting() && !gCodes.IsSimulating())
				{
					const float extrusionCommanded = (float)extruderStepsCommanded/reprap.GetPlatform().DriveStepsPerUnit(fs.driveNumber);
					fst = fs.Check(isPrinting, fromIsr, locIsrMillis, extrusionCommanded);
				}
				else
				{
					fst = fs.Clear();
				}

				if (fst != fs.lastStatus)
				{
#if SUPPORT_REMOTE_COMMANDS
					statusChanged = true;
#endif
					fs.lastStatus = fst;
					if (fst != FilamentSensorStatus::ok
#if SUPPORT_REMOTE_COMMANDS
						&& !CanInterface::InExpansionMode()
#endif
						)
					{
						const size_t extruder = LogicalDriveToExtruder(fs.driveNumber);
						if (reprap.Debug(moduleFilamentSensors))
						{
							debugPrintf("Filament error: extruder %u reports %s\n", extruder, fst.ToString());
						}
						else
						{
							gCodes.FilamentError(extruder, fst);
						}
					}
				}
			}
		}
#if SUPPORT_REMOTE_COMMANDS
		if (drv < NumDirectDrivers)
		{
			msg->data[drv].Set(fst.ToBaseType());
		}
#endif
	}

#if SUPPORT_REMOTE_COMMANDS
	if (CanInterface::InExpansionMode() && (statusChanged || (haveMonitor && millis() - whenStatusLastSent >= StatusUpdateInterval)))
	{
		msg->SetStandardFields(NumDirectDrivers);
		buf.dataLength = msg->GetActualDataLength();
		CanInterface::SendMessageNoReplyNoFree(&buf);
		whenStatusLastSent = millis();
	}
#endif
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
			if (fs.driverId.boardAddress == src && fs.driverId.localDriver < msg.numMonitorsReported)
			{
				const FilamentSensorStatus fstat(msg.data[fs.driverId.localDriver].status);
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
		DeleteObject(f);
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

	for (size_t extruder = 0; extruder < ARRAY_SIZE(filamentSensors); ++extruder)
	{
		if (filamentSensors[extruder] != nullptr && !filamentSensors[extruder]->IsValid(extruder))
		{
			reply.lcatf("Filament monitor for extruder %u has been deleted due to configuration change", extruder);
			warn = true;
			DeleteObject(filamentSensors[extruder]);
		}
	}
	return warn;
}

#if SUPPORT_REMOTE_COMMANDS

// Do the configuration that is common to all filament monitor types
// Try to get the pin number from the GCode command in the buffer, setting Seen if a pin number was provided and returning true if error.
// Also attaches the ISR.
// For a remote filament monitor, this does the full configuration or query of the remote object instead, and we always return seen true because we don't need to report local status.
GCodeResult FilamentMonitor::CommonConfigure(const CanMessageGenericParser& parser, const StringRef& reply, InterruptMode interruptMode, bool& seen) noexcept
{
	String<StringLength20> portName;
	if (parser.GetStringParam('C', portName.GetRef()))
	{
		seen = true;
		if (!port.AssignPort(portName.c_str(), reply, PinUsedBy::filamentMonitor, PinAccess::read))
		{
			return GCodeResult::error;
		}

		haveIsrStepsCommanded = false;
		if (interruptMode != InterruptMode::none && !port.AttachInterrupt(InterruptEntry, interruptMode, this))
		{
			reply.copy("unsuitable pin");
			return GCodeResult::error;
		}
	}
	return GCodeResult::ok;
}

// Create a new filament monitor, or replace an existing one
/*static*/ GCodeResult FilamentMonitor::Create(const CanMessageCreateFilamentMonitor& msg, const StringRef& reply) noexcept
{
	const uint8_t p_driver = msg.driver;
	if (p_driver >= NumDirectDrivers)
	{
		reply.copy("Driver number out of range");
		return GCodeResult::error;
	}

	DriverId did;
	did.SetLocal(p_driver);

	WriteLocker lock(filamentMonitorsLock);

	// Delete any existing filament monitor
	FilamentMonitor *fm = nullptr;
	std::swap(fm, filamentSensors[p_driver]);
	delete fm;

	// Create the new one
	const uint8_t monitorType = msg.type;
	switch (msg.type)
	{
	case 1:		// active high switch
	case 2:		// active low switch
		fm = new SimpleFilamentMonitor(p_driver, monitorType, did);
		break;

	case 3:		// duet3d rotating magnet, no switch
	case 4:		// duet3d rotating magnet + switch
		fm = new RotatingMagnetFilamentMonitor(p_driver, monitorType, did);
		break;

	case 5:		// duet3d laser, no switch
	case 6:		// duet3d laser + switch
		fm = new LaserFilamentMonitor(p_driver, monitorType, did);
		break;

	case 7:		// simple pulse output sensor
		fm = new PulsedFilamentMonitor(p_driver, monitorType, did);
		break;

	default:	// no sensor, or unknown sensor
		reply.printf("Unknown filament monitor type %u", monitorType);
		return GCodeResult::error;
	}

	filamentSensors[p_driver] = fm;
	return GCodeResult::ok;
}

// Delete a filament monitor
/*static*/ GCodeResult FilamentMonitor::Delete(const CanMessageDeleteFilamentMonitor& msg, const StringRef& reply) noexcept
{
	const uint8_t p_driver = msg.driver;
	if (p_driver >= NumDirectDrivers)
	{
		reply.copy("Driver number out of range");
		return GCodeResult::error;
	}

	WriteLocker lock(filamentMonitorsLock);

	FilamentMonitor *fm = nullptr;
	std::swap(fm, filamentSensors[p_driver]);

	if (fm == nullptr)
	{
		reply.printf("Driver %u.%u has no filament monitor", CanInterface::GetCanAddress(), p_driver);
		return GCodeResult::warning;
	}

	delete fm;
	return GCodeResult::ok;
}

// Configure a filament monitor
/*static*/ GCodeResult FilamentMonitor::Configure(const CanMessageGeneric& msg, const StringRef& reply) noexcept
{
	CanMessageGenericParser parser(msg, ConfigureFilamentMonitorParams);
	uint8_t p_driver;
	if (!parser.GetUintParam('d', p_driver) || p_driver >= NumDirectDrivers)
	{
		reply.copy("Bad or missing driver number");
		return GCodeResult::error;
	}

	WriteLocker lock(filamentMonitorsLock);

	FilamentMonitor *fm = filamentSensors[p_driver];
	if (fm == nullptr)
	{
		reply.printf("Driver %u.%u has no filament monitor", CanInterface::GetCanAddress(), p_driver);
		return GCodeResult::error;
	}

	return fm->Configure(parser, reply);
}

// Delete all filament monitors
/*static*/ void FilamentMonitor::DeleteAll() noexcept
{
	WriteLocker lock(filamentMonitorsLock);

	for (FilamentMonitor*& fm : filamentSensors)
	{
		if (fm != nullptr)
		{
			DeleteObject(fm);
		}
	}
}

#endif

// End
