/*
 * StallDetectionEndstop.cpp
 *
 *  Created on: 15 Sep 2019
 *      Author: David
 */

#include "StallDetectionEndstop.h"

#if HAS_STALL_DETECT || SUPPORT_CAN_EXPANSION

#include <Platform/RepRap.h>
#include <Movement/Move.h>
#include <Movement/Kinematics/Kinematics.h>

#if SUPPORT_CAN_EXPANSION
# include <CAN/CanInterface.h>
#endif

// Stall detection endstop constructor, used for axis endstops
StallDetectionEndstop::StallDetectionEndstop(uint8_t p_axis, EndStopPosition pos, bool p_individualMotors) noexcept
	: Endstop(p_axis, pos),
#if SUPPORT_CAN_EXPANSION
	  newStallReported(false),
#endif
	  individualMotors(p_individualMotors)
{
}

// Constructor used for extruder endstops
StallDetectionEndstop::StallDetectionEndstop() noexcept
	: Endstop(NO_AXIS, EndStopPosition::noEndStop),
#if SUPPORT_CAN_EXPANSION
	  newStallReported(false),
#endif
	  individualMotors(false), stopAll(true)
{
}

// Test whether we are at or near the stop
bool StallDetectionEndstop::Stopped() const noexcept
{
#if SUPPORT_CAN_EXPANSION
	return
# if HAS_STALL_DETECT
		GetStalledDrivers(localDriversMonitored).IsNonEmpty() ||
# endif
		newStallReported;
#else		// must have HAS_STALL_DETECT
	return GetStalledDrivers(localDriversMonitored).IsNonEmpty();
#endif
}

// This is called to prime axis endstops
void StallDetectionEndstop::PrimeAxis(const Kinematics &_ecv_from kin, const AxisDriversConfig& axisDrivers, float speed) THROWS(GCodeException)
{
	// Find which drivers are relevant, and decide whether we stop just the driver, just the axis, or everything
	const LogicalDrivesBitmap logicalDrivesToMonitor = kin.GetControllingDrives(GetAxis(), true);
	stopAll = logicalDrivesToMonitor.Intersects(~LogicalDrivesBitmap::MakeFromBits(GetAxis()));

	// Build the lists of local and remote drivers to monitor
	localDriversMonitored.Clear();
#if SUPPORT_CAN_EXPANSION
	remoteDriversMonitored.Clear();
	newStallReported = false;
#endif
	numDriversLeft = 0;
	logicalDrivesToMonitor.IterateWithExceptions([this, speed](unsigned int drive, unsigned int count) THROWS(GCodeException)
													{
														const Move& move = reprap.GetMove();
														const AxisDriversConfig& config = move.GetAxisDriversConfig(drive);
														for (size_t i = 0; i < config.numDrivers; ++i)
														{
															AddDriverToMonitoredList(config.driverNumbers[i], speed * move.DriveStepsPerMm(drive));
														}
													}
												);
}

void StallDetectionEndstop::PrimeExtruders(ExtrudersBitmap extruders, const float speeds[MaxExtruders]) THROWS(GCodeException)
{
	localDriversMonitored.Clear();
#if SUPPORT_CAN_EXPANSION
	remoteDriversMonitored.Clear();
	newStallReported = false;
#endif
	numDriversLeft = 0;
	extruders.IterateWithExceptions([this, speeds](unsigned int extruder, unsigned int count) THROWS(GCodeException)
									{
										const Move& move = reprap.GetMove();
										AddDriverToMonitoredList(move.GetExtruderDriver(extruder), speeds[extruder] * move.DriveStepsPerMm(ExtruderToLogicalDrive(extruder)));
									}
								   );
}

// Add a remote driver to the list of remote drivers monitored. We maintain a bitmap of local drivers monitored on each relevant CAN-connected board.
void StallDetectionEndstop::AddDriverToMonitoredList(DriverId did, float speed) THROWS(GCodeException)
{
#if SUPPORT_CAN_EXPANSION
	if (did.IsRemote())
	{
		try
		{
			CanInterface::EnableRemoteStallEndstop(did, fabsf(speed));						// may throw
		}
		catch (GCodeException&)
		{
			DeleteRemoteStallEndstops();
			throw;
		}

		bool found = false;
		for (size_t i = 0; i < remoteDriversMonitored.Size(); ++i)
		{
			if (remoteDriversMonitored[i].boardId == did.boardAddress)
			{
				remoteDriversMonitored[i].driversMonitored.SetBit(did.localDriver);
				found = true;
				break;
			}
		}

		if (!found)
		{
			(void)remoteDriversMonitored.Add(RemoteDriversMonitored(did.boardAddress, LocalDriversBitmap::MakeFromBits(did.localDriver)));		// we don't expect the vector to overflow
		}
	}
	else
#endif
	{
#if HAS_STALL_DETECT
		reprap.GetMove().CheckStallDetectionViable(did.localDriver, speed);
		localDriversMonitored.SetBit(did.localDriver);
#else
		ThrowGCodeException("drivers on board %u do not support stall detection", CanInterface::GetCanAddress());
#endif
	}
	++numDriversLeft;
}

// Construct and return a result object
EndstopHitDetails StallDetectionEndstop::GetResult(
#if SUPPORT_CAN_EXPANSION
													CanAddress boardAddress,
#endif
													unsigned int driverWithinBoard) noexcept
{
	EndstopHitDetails rslt;
	rslt.axis = GetAxis();
	if (rslt.axis == NO_AXIS)
	{
		rslt.SetAction(EndstopHitAction::stopAll);
	}
	else if (stopAll)
	{
		rslt.SetAction(EndstopHitAction::stopAll);
		if (GetAtHighEnd())
		{
			rslt.setAxisHigh = true;
		}
		else
		{
			rslt.setAxisLow = true;
		}
	}
	else if (individualMotors && numDriversLeft > 1)
	{
		rslt.SetAction(EndstopHitAction::stopDriver);
#if SUPPORT_CAN_EXPANSION
		rslt.driver.boardAddress = boardAddress;
#endif
		rslt.driver.localDriver = driverWithinBoard;
	}
	else
	{
		rslt.SetAction(EndstopHitAction::stopAxis);
		if (GetAtHighEnd())
		{
			rslt.setAxisHigh = true;
		}
		else
		{
			rslt.setAxisLow = true;
		}
	}
	return rslt;
}

// Check whether the endstop is triggered and return the action that should be performed. Called from the step ISR.
// Note, the result will not necessarily be acted on because there may be a higher priority endstop!
EndstopHitDetails StallDetectionEndstop::CheckTriggered() noexcept
{
#if HAS_STALL_DETECT
	// Check for local stalled drivers first
	const LocalDriversBitmap relevantStalledDrivers = GetStalledDrivers(localDriversMonitored);
	if (relevantStalledDrivers.IsNonEmpty())
	{
		return GetResult(
#if SUPPORT_CAN_EXPANSION
							CanInterface::GetCanAddress(),
#endif
							relevantStalledDrivers.LowestSetBit());
	}
#endif

#if SUPPORT_CAN_EXPANSION
	// Account for CAN-connected drivers
	if (newStallReported.exchange(false))
	{
		// Find the board/driver that has stalled
		for (size_t i = 0; i < remoteDriversMonitored.Size(); ++i)
		{
			const RemoteDriversMonitored& elem = remoteDriversMonitored[i];
			const LocalDriversBitmap stalledRemoteDrives = elem.driversMonitored & elem.driversStalled;
			if (stalledRemoteDrives.IsNonEmpty())
			{
				newStallReported = true;					// there may be more than one stalled drive reported, so make sure we check again
				return GetResult(elem.boardId, stalledRemoteDrives.LowestSetBit());
			}
		}
	}
#endif
	return EndstopHitDetails();
}

// This is called by the ISR to acknowledge that it is acting on the return from calling CheckTriggered. Called from the step ISR.
// Return true if we have finished with this endstop or probe in this move.
bool StallDetectionEndstop::Acknowledge(EndstopHitDetails what) noexcept
{
	switch (what.GetAction())
	{
	case EndstopHitAction::stopAll:
	case EndstopHitAction::stopAxis:
		return true;

	case EndstopHitAction::stopDriver:
#if SUPPORT_CAN_EXPANSION
		if (what.driver.boardAddress != CanInterface::GetCanAddress())
		{
			for (size_t i = 0; i < remoteDriversMonitored.Size(); ++i)
			{
				RemoteDriversMonitored& elem = remoteDriversMonitored[i];
				if (elem.boardId == what.driver.boardAddress)
				{
					elem.driversMonitored.ClearBit(what.driver.localDriver);
					break;
				}
			}
		}
		else
#endif
		{
			localDriversMonitored.ClearBit(what.driver.localDriver);
		}
		--numDriversLeft;
		return false;

	default:
		return false;
	}
}

void StallDetectionEndstop::AppendDetails(const StringRef& str) noexcept
{
	str.cat((individualMotors) ? "motor stall (individual motors)" : "motor stall (any motor)");
}

void StallDetectionEndstop::SetDrivers(LocalDriversBitmap extruderDrivers) noexcept
{
	localDriversMonitored = extruderDrivers;
	stopAll = true;
}

#if SUPPORT_CAN_EXPANSION

// Delete all remote endstops that have already been set up
void StallDetectionEndstop::DeleteRemoteStallEndstops() noexcept
{
	remoteDriversMonitored.Iterate([](const RemoteDriversMonitored& entry, size_t count) noexcept
									{
										CanInterface::DisableRemoteStallEndstops(entry.boardId);
									}
								  );
	remoteDriversMonitored.Clear();
}

// Record any notifications of stalled remote drivers that we are interested in
void StallDetectionEndstop::HandleStalledRemoteDrivers(CanAddress boardAddress, LocalDriversBitmap driversReportedStalled) noexcept
{
	remoteDriversMonitored.IterateWhile([this, boardAddress, driversReportedStalled](RemoteDriversMonitored& entry, size_t count) noexcept -> bool
										{
											if (boardAddress == entry.boardId)
											{
												const LocalDriversBitmap pending = entry.driversMonitored & ~entry.driversStalled;
												const LocalDriversBitmap newStalls = pending & driversReportedStalled;
												if (newStalls.IsNonEmpty())
												{
													entry.driversStalled |= newStalls;
													newStallReported = true;
												}
												return false;
											}
											return true;
										}
									   );
}

#endif

#endif

// End
