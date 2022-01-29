/*
 * StallDetectionEndstop.cpp
 *
 *  Created on: 15 Sep 2019
 *      Author: David
 */

#include "StallDetectionEndstop.h"

#if HAS_STALL_DETECT

#include <Platform/Platform.h>
#include <Movement/Kinematics/Kinematics.h>

// Stall detection endstop
StallDetectionEndstop::StallDetectionEndstop(uint8_t p_axis, EndStopPosition pos, bool p_individualMotors) noexcept
	: Endstop(p_axis, pos), individualMotors(p_individualMotors)
{
}

StallDetectionEndstop::StallDetectionEndstop() noexcept
	: Endstop(NO_AXIS, EndStopPosition::noEndStop), individualMotors(false), stopAll(true)
{
}

// Test whether we are at or near the stop
bool StallDetectionEndstop::Stopped() const noexcept
{
	return GetStalledDrivers(driversMonitored).IsNonEmpty();
}

// This is called to prime axis endstops
bool StallDetectionEndstop::Prime(const Kinematics& kin, const AxisDriversConfig& axisDrivers) noexcept
{
	// Find which drivers are relevant, and decide whether we stop just the driver, just the axis, or everything
	stopAll = kin.GetConnectedAxes(GetAxis()).Intersects(~AxesBitmap::MakeFromBits(GetAxis()));
	numDriversLeft = axisDrivers.numDrivers;
	driversMonitored = axisDrivers.GetDriversBitmap();

#if SUPPORT_CAN_EXPANSION
	//TODO if there any remote stall endstops, check they are set up to report
#endif

	return true;
}

// Check whether the endstop is triggered and return the action that should be performed. Called from the step ISR.
// Note, the result will not necessarily be acted on because there may be a higher priority endstop!
EndstopHitDetails StallDetectionEndstop::CheckTriggered() noexcept
{
	EndstopHitDetails rslt;				// initialised by default constructor
	const DriversBitmap relevantStalledDrivers = GetStalledDrivers(driversMonitored);
	if (relevantStalledDrivers.IsNonEmpty())
	{
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
			rslt.driver.boardAddress = 0;
#endif
			rslt.driver.localDriver = relevantStalledDrivers.LowestSetBit();
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
	}

	return rslt;
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
		driversMonitored.ClearBit(what.driver.localDriver);
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

void StallDetectionEndstop::SetDrivers(DriversBitmap extruderDrivers) noexcept
{
	driversMonitored = extruderDrivers;
	stopAll = true;
}

#endif

// End
