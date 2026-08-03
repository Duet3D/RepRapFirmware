/*
 * GpInPortEndstop.cpp
 *
 *  Created on: 3 Aug 2026
 *      Author: Christian
 */

#include "GpInPortEndstop.h"
#include <Platform/RepRap.h>
#include <Platform/Platform.h>
#include <GPIO/GpInPort.h>

GpInPortEndstop::GpInPortEndstop() noexcept : Endstop(NO_AXIS, EndStopPosition::noEndStop)
{
}

// This is never called because this endstop is not assigned to an axis
void GpInPortEndstop::PrimeAxis(const Kinematics &_ecv_from kin, const AxisDriversConfig& axisDrivers, float speed) THROWS(GCodeException)
{
}

// Add an input to monitor during the next move
void GpInPortEndstop::AddInput(uint8_t gpinNumber, bool stopWhenActive) noexcept
{
	if (numInputs < ARRAY_SIZE(inputs))
	{
		inputs[numInputs].gpinNumber = gpinNumber;
		inputs[numInputs].stopWhenActive = stopWhenActive;
		numInputs++;
	}
}

// Test whether any of the monitored inputs is at its stop state
bool GpInPortEndstop::Stopped() const noexcept
{
	for (size_t i = 0; i < numInputs; ++i)
	{
		if (reprap.GetPlatform().GetGpInPort(inputs[i].gpinNumber).GetState() == inputs[i].stopWhenActive)
		{
			return true;
		}
	}
	return false;
}

// Check whether the endstop is triggered and return the action that should be performed. Called from the step ISR
EndstopHitDetails GpInPortEndstop::CheckTriggered() noexcept
{
	EndstopHitDetails rslt;
	if (Stopped())
	{
		rslt.SetAction(EndstopHitAction::stopAll, 0, false);
	}
	return rslt;
}

// This is called by the ISR to acknowledge that it is acting on the return from calling CheckTriggered. Return true if we can now be deleted or recycled
bool GpInPortEndstop::Acknowledge(EndstopHitDetails what) noexcept
{
	return what.GetAction() == EndstopHitAction::stopAll;
}

void GpInPortEndstop::AppendDetails(const StringRef& str) noexcept
{
	str.cat("filament input");
}

// End
