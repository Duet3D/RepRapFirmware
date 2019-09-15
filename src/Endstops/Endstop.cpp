/*
 * Endstop.cpp
 *
 *  Created on: 4 Apr 2019
 *      Author: David
 */

#include "Endstop.h"

// Endstop base class
DriversBitmap EndstopOrZProbe::stalledDrivers = 0;			// used to track which drivers are reported as stalled, for stall detect endstops and stall detect Z probes

Endstop::Endstop(uint8_t p_axis, EndStopPosition pos) : axis(p_axis), atHighEnd(pos == EndStopPosition::highEndStop)
{
}

// End
