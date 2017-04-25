/*
 * Kinematics.cpp
 *
 *  Created on: 24 Apr 2017
 *      Author: David
 */

#include "Kinematics.h"

Kinematics::Kinematics(unsigned int nHeadAxes, unsigned int nMachineAxes)
	: numHeadAxes(nHeadAxes), numMachineAxes(nMachineAxes), useSegmentation(false), useRawG0(true)
{
}

Kinematics::Kinematics(unsigned int nHeadAxes, unsigned int nMachineAxes, float segsPerSecond, float minSegLength, bool doUseRawG0)
	: numHeadAxes(nHeadAxes), numMachineAxes(nMachineAxes), useSegmentation(true), useRawG0(doUseRawG0),
	  segmentsPerSecond(segsPerSecond), minSegmentLength(minSegLength)
{
}

// Set or report the parameters from a M665, M666 or M669 command
// This is the fallback function for when the derived class doesn't use the specified M-code
bool Kinematics::SetOrReportParameters(unsigned int mCode, GCodeBuffer& gb, StringRef& reply, bool& error)
{
	reply.printf("M%u parameters do nt apply to %s kinematics", mCode, GetName());
	error = true;
	return false;
}

// End
