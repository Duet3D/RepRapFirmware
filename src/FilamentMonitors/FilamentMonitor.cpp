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
#include <Platform/Event.h>
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
size_t FilamentMonitor::firstDriveToSend = 0;
#endif

#if SUPPORT_OBJECT_MODEL

// Object model table and functions
// Note: if using GCC version 7.3.1 20180622 and lambda functions are used in this table, you must compile this file with option -std=gnu++17.
// Otherwise the table will be allocated in RAM instead of flash, which wastes too much RAM.

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(...) OBJECT_MODEL_FUNC_BODY(FilamentMonitor, __VA_ARGS__)

constexpr ObjectModelTableEntry FilamentMonitor::objectModelTable[] =
{
	// Within each group, these entries must be in alphabetical order
	{ "enableMode",			OBJECT_MODEL_FUNC((int32_t)self->GetEnableMode()),		ObjectModelEntryFlags::none },
	{ "enabled",			OBJECT_MODEL_FUNC(self->GetEnableMode() != 0),		 	ObjectModelEntryFlags::obsolete },
	{ "status",				OBJECT_MODEL_FUNC(self->GetStatusText()),				ObjectModelEntryFlags::live },
	{ "type",				OBJECT_MODEL_FUNC(self->GetTypeText()), 				ObjectModelEntryFlags::none },
};

constexpr uint8_t FilamentMonitor::objectModelTableDescriptor[] = { 1, 4 };

DEFINE_GET_OBJECT_MODEL_TABLE(FilamentMonitor)

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
	: driveNumber(drv), type(monitorType), driverId(did), enableMode(0), lastStatus(FilamentSensorStatus::noDataReceived)
#if SUPPORT_CAN_EXPANSION
	  , lastRemoteStatus(FilamentSensorStatus::noDataReceived), hasRemote(false)
#endif
{
}

// Default destructor
FilamentMonitor::~FilamentMonitor() noexcept
{
#if SUPPORT_CAN_EXPANSION
	if (hasRemote)
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

// Do the configuration that is common to all types of filament monitor
// Try to get the pin number from the GCode command in the buffer, setting Seen if a pin number was provided. Also attaches the ISR.
// For a remote filament monitor, this does the full configuration or query of the remote object instead, and we always return seen true because we don't need to report local status.
GCodeResult FilamentMonitor::CommonConfigure(GCodeBuffer& gb, const StringRef& reply, InterruptMode interruptMode, bool& seen) THROWS(GCodeException)
{
	if (gb.Seen('S'))
	{
		seen = true;
		enableMode = gb.GetLimitedUIValue('S', 3);
	}

#if SUPPORT_CAN_EXPANSION
	if (hasRemote)
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
		if (interruptMode != InterruptMode::none && !port.AttachInterrupt(InterruptEntry, interruptMode, CallbackParameter(this)))
		{
			reply.copy("no interrupt available on pin ");
			port.AppendPinName(reply);
			port.Release();
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
	// Don't allow C parameter without P parameter
	if (gb.Seen('C'))
	{
		gb.MustSee('P');
	}

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

		sensor = Create(extruder, newSensorType, gb, reply);					// create the new sensor (may throw)

		try
		{
			const GCodeResult rslt = sensor->Configure(gb, reply, seen);		// configure the sensor (may throw)
			if (Succeeded(rslt))
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

	const GCodeResult rslt = sensor->Configure(gb, reply, seen);									// configure or report on the existing sensor (may throw)
	if (seen)
	{
		reprap.SensorsUpdated();
	}
	return rslt;
}

// Factory function to create a filament monitor.
// If successful, return the filament monitor object; else throw a GCodeException.
/*static*/ FilamentMonitor *FilamentMonitor::Create(unsigned int extruder, unsigned int monitorType, GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	const size_t drv = ExtruderToLogicalDrive(extruder);
	const DriverId did = reprap.GetPlatform().GetExtruderDriver(extruder);
	gb.MustSee('C');																// make sure the port name parameter is present

#if SUPPORT_CAN_EXPANSION
	// Find out which board the sensor is connected to
	String<StringLength50> portName;
	gb.GetQuotedString(portName.GetRef());
	const CanAddress locBoardAddress = IoPort::RemoveBoardAddress(portName.GetRef());
	if (locBoardAddress != did.boardAddress	)										// most filament monitor types must be on the same board as the extruder
	{
		if (monitorType > 2)
		{
			gb.ThrowGCodeException("Filament monitor must be connected to same CAN board as extruder driver");
		}
		else if (locBoardAddress != CanInterface::GetCanAddress())					// a simple switch may be on the main board instead
		{
			gb.ThrowGCodeException("Switch-type filament monitor must be connected to same CAN board as extruder driver or to main board");
		}
	}
#endif

	FilamentMonitor *fm;
	switch (monitorType)
	{
	case 1:		// active high switch
	case 2:		// active low switch
		fm = new SimpleFilamentMonitor(drv, monitorType, did);
		break;

	case 3:		// duet3d rotating magnet, no switch
	case 4:		// duet3d rotating magnet + switch
		fm = new RotatingMagnetFilamentMonitor(drv, monitorType, did);
		break;

	case 5:		// duet3d laser, no switch
	case 6:		// duet3d laser + switch
		fm = new LaserFilamentMonitor(drv, monitorType, did);
		break;

	case 7:		// simple pulse output sensor
		fm = new PulsedFilamentMonitor(drv, monitorType, did);
		break;

	default:	// no sensor, or unknown sensor
		gb.ThrowGCodeException("Unknown filament monitor type %u", monitorType);
	}
#if SUPPORT_CAN_EXPANSION
	if (locBoardAddress != CanInterface::GetCanAddress())
	{
		// Create the remote filament monitor on the expansion board
		if (CanInterface::CreateFilamentMonitor(fm->driverId, monitorType, gb, reply) != GCodeResult::ok)
		{
			delete fm;
			gb.ThrowGCodeException("Failed to create filament monitor on CAN-connected expansion board");
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

static uint32_t checkCalls = 0, clearCalls = 0;		//TEMP DEBUG

// Check the status of all the filament monitors.
// Currently, the status for all filament monitors (on expansion boards as well as on the main board) is checked by the main board, which generates any necessary events.
/*static*/ void FilamentMonitor::Spin() noexcept
{
#if SUPPORT_REMOTE_COMMANDS
	CanMessageBuffer buf;
	auto msg = buf.SetupStatusMessage<CanMessageFilamentMonitorsStatusNew>(CanInterface::GetCanAddress(), CanInterface::GetCurrentMasterAddress());
	size_t slotIndex = 0;
	size_t firstDriveNotSent = NumDirectDrivers;
	Bitmap<uint32_t> driversReported;
	bool forceSend = false, haveLiveData = false;
#endif

	{
		ReadLocker lock(filamentMonitorsLock);

		for (size_t drv = 0; drv < NumFilamentMonitors; ++drv)
		{
			FilamentSensorStatus fst(FilamentSensorStatus::noMonitor);
			if (filamentSensors[drv] != nullptr)
			{
				FilamentMonitor& fs = *filamentSensors[drv];
				GCodes& gCodes = reprap.GetGCodes();
#if SUPPORT_CAN_EXPANSION
				if (!fs.hasRemote)
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
						extruderStepsCommanded = reprap.GetMove().GetAccumulatedExtrusion(fs.driveNumber, isPrinting);		// get and clear the net extrusion commanded
						IrqEnable();
						fromIsr = false;
						locIsrMillis = 0;
					}
					if ((fs.enableMode == 2 || gCodes.IsReallyPrinting()) && !gCodes.IsSimulating())
					{
						const float extrusionCommanded = (float)extruderStepsCommanded/reprap.GetPlatform().DriveStepsPerUnit(fs.driveNumber);
						fst = fs.Check(isPrinting, fromIsr, locIsrMillis, extrusionCommanded);
						++checkCalls;
					}
					else
					{
						fst = fs.Clear();
						++clearCalls;
					}

#if SUPPORT_REMOTE_COMMANDS
					if (CanInterface::InExpansionMode())
					{
						if (drv >= firstDriveToSend && drv < NumDirectDrivers)
						{
							if (slotIndex < ARRAY_SIZE(msg->data))
							{
								auto& slot = msg->data[slotIndex];
								slot.status = fst.ToBaseType();
								fs.GetLiveData(slot);
								if (fst != fs.lastStatus)
								{
									forceSend = true;
									fs.lastStatus = fst;
								}
								else if (slot.hasLiveData)
								{
									haveLiveData = true;
								}
								driversReported.SetBit(drv);
								++slotIndex;
							}
							else if (drv < firstDriveNotSent)
							{
								firstDriveNotSent = drv;
							}
						}
						continue;
					}
#endif
				}
#if SUPPORT_CAN_EXPANSION
				else
				{
					fst = fs.lastRemoteStatus;
				}
#endif
				if (   fst != fs.lastStatus
#if SUPPORT_REMOTE_COMMANDS
					&& !CanInterface::InExpansionMode()
#endif
					)
				{
					fs.lastStatus = fst;
					if (   fst != FilamentSensorStatus::ok
						&& !gCodes.IsSimulating()
						&& (fs.GetEnableMode() == 2 || (fs.GetEnableMode() == 1 && gCodes.IsReallyPrinting()))
					   )
					{
						const size_t extruder = LogicalDriveToExtruder(fs.driveNumber);
						Event::AddEvent(EventType::filament_error, (uint16_t)fst.ToBaseType(), fs.driverId.GetBoardAddress(), extruder, "");
					}
				}
			}
		}
	}

#if SUPPORT_REMOTE_COMMANDS
	if (CanInterface::InExpansionMode())
	{
		uint32_t now;
		if (   slotIndex != 0
			&& (   forceSend
				|| (now = millis()) - whenStatusLastSent >= StatusUpdateInterval
				|| (haveLiveData && now - whenStatusLastSent >= LiveStatusUpdateInterval)

			   )
		   )
		{
			msg->SetStandardFields(driversReported);
			buf.dataLength = msg->GetActualDataLength();
			CanInterface::SendMessageNoReplyNoFree(&buf);
			whenStatusLastSent = millis();
		}
		firstDriveToSend = (firstDriveNotSent < NumDirectDrivers) ? firstDriveNotSent : 0;
	}
#endif
}

#if SUPPORT_CAN_EXPANSION

/*static*/ void FilamentMonitor::UpdateRemoteFilamentStatus(CanAddress src, CanMessageFilamentMonitorsStatusNew& msg) noexcept
{
	Bitmap<uint32_t> drivers(msg.driversReported);
	size_t slotIndex = 0;

	ReadLocker lock(filamentMonitorsLock);
	while (!drivers.IsEmpty())
	{
		const unsigned int driverNumber = drivers.LowestSetBit();

		for (size_t extruder = 0; extruder < MaxExtruders; ++extruder)
		{
			if (filamentSensors[extruder] != nullptr)
			{
				FilamentMonitor& fs = *filamentSensors[extruder];
				if (fs.driverId.boardAddress == src && fs.driverId.localDriver == driverNumber)
				{
					const auto& slot = msg.data[slotIndex];
					const FilamentSensorStatus newStatus = FilamentSensorStatus(slot.status);
					if (reprap.Debug(Module::FilamentSensors) && newStatus != fs.lastRemoteStatus)
					{
						debugPrintf("Remote extruder %u status change from %s to %s\n", LogicalDriveToExtruder(fs.driveNumber), fs.lastRemoteStatus.ToString(), newStatus.ToString());
					}
					fs.lastRemoteStatus = newStatus;
					fs.UpdateLiveData(slot);
				}
			}
		}
		drivers.ClearBit(driverNumber);
		++slotIndex;
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
#if 1	//TEMP DEBUG
				reprap.GetPlatform().MessageF(mtype, "=== Filament sensors ===\ncheck %" PRIu32 " clear %" PRIu32 "\n", checkCalls, clearCalls);
#else
				reprap.GetPlatform().Message(mtype, "=== Filament sensors ===\n");
#endif
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
	if (parser.GetUintParam('S', enableMode))
	{
		seen = true;
		if (enableMode > 2)
		{
			enableMode = 2;
		}
	}

	String<StringLength20> portName;
	if (parser.GetStringParam('C', portName.GetRef()))
	{
		seen = true;
		if (!port.AssignPort(portName.c_str(), reply, PinUsedBy::filamentMonitor, PinAccess::read))
		{
			return GCodeResult::error;
		}

		haveIsrStepsCommanded = false;
		if (interruptMode != InterruptMode::none && !port.AttachInterrupt(InterruptEntry, interruptMode, CallbackParameter(this)))
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

	DeleteObject(filamentSensors[p_driver]);					// delete any existing filament monitor
	FilamentMonitor *fm;

	// Create the new one
	const uint8_t monitorType = msg.type;
	switch (monitorType)
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

// Send diagnostics info
/*static*/ void FilamentMonitor::GetDiagnostics(const StringRef& reply) noexcept
{
	bool first = true;
	ReadLocker lock(filamentMonitorsLock);

	for (size_t i = 0; i < NumDirectDrivers; ++i)
	{
		FilamentMonitor * const fs = filamentSensors[i];
		if (fs != nullptr)
		{
			if (first)
			{
#if 1	//TEMP DEBUG
				reply.lcatf("=== Filament sensors ===\ncheck %" PRIu32 " clear %" PRIu32 "\n", checkCalls, clearCalls);
#else
				reply.lcat("=== Filament sensors ===\n");
#endif
				first = false;
			}
			fs->Diagnostics(reply);
		}
	}
}

#endif

// End
