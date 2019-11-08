/*
 * StallDetectionEndstop.cpp
 *
 *  Created on: 15 Sep 2019
 *      Author: David
 */

#include "StallDetectionEndstop.h"

#include "Platform.h"
#include "Movement/Kinematics/Kinematics.h"

// Stall detection endstop
StallDetectionEndstop::StallDetectionEndstop(uint8_t axis, EndStopPosition pos, bool p_individualMotors)
	: Endstop(axis, pos), driversMonitored(0), individualMotors(p_individualMotors)
{
}

// Test whether we are at or near the stop
EndStopHit StallDetectionEndstop::Stopped() const
{
	return ((GetStalledDrivers() & driversMonitored) != 0) ? EndStopHit::atStop : EndStopHit::noStop;
}

// This is called to prime axis endstops
bool StallDetectionEndstop::Prime(const Kinematics& kin, const AxisDriversConfig& axisDrivers)
{
	// Find which drivers are relevant, and decide whether we stop just the driver, just the axis, or everything
	stopAll = (kin.GetConnectedAxes(GetAxis()) & ~MakeBitmap<AxesBitmap>(GetAxis())) != 0;
	numDriversLeft = axisDrivers.numDrivers;
	driversMonitored = axisDrivers.GetDriversBitmap();

#if SUPPORT_CAN_EXPANSION
	//TODO if there any remote stall endstops, check they are set up to report
#endif

	return true;
}

// Check whether the endstop is triggered and return the action that should be performed. Called from the step ISR.
// Note, the result will not necessarily be acted on because there may be a higher priority endstop!
EndstopHitDetails StallDetectionEndstop::CheckTriggered(bool goingSlow)
{
	EndstopHitDetails rslt;				// initialised by default constructor
	const DriversBitmap relevantStalledDrivers = driversMonitored & GetStalledDrivers();
	if (relevantStalledDrivers != 0)
	{
		rslt.axis = GetAxis();
		if (stopAll)
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
#else
			rslt.driver.localDriver = LowestSetBitNumber(relevantStalledDrivers);
#endif
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
bool StallDetectionEndstop::Acknowledge(EndstopHitDetails what)
{
	switch (what.GetAction())
	{
	case EndstopHitAction::stopAll:
	case EndstopHitAction::stopAxis:
		return true;

	case EndstopHitAction::stopDriver:
		ClearBit(driversMonitored, what.driver.localDriver);
		--numDriversLeft;
		return false;

	default:
		return false;
	}
}

void StallDetectionEndstop::AppendDetails(const StringRef& str)
{
	str.cat((individualMotors) ? "motor stall (individual motors)" : "motor stall (any motor)");
}

// End
