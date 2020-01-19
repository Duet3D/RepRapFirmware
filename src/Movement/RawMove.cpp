/*
 * RawMove.cpp
 *
 *  Created on: 18 Apr 2019
 *      Author: David
 */

#include "RawMove.h"

// Set up some default values in the move buffer for special moves, e.g. for Z probing and firmware retraction
void RawMove::SetDefaults(size_t firstDriveToZero) noexcept
{
	moveType = 0;
	isCoordinated = false;
	usingStandardFeedrate = false;
	usePressureAdvance = false;
	checkEndstops = false;
	reduceAcceleration = false;
	hasExtrusion = false;
	filePos = noFilePosition;
	tool = nullptr;
	for (size_t drive = firstDriveToZero; drive < MaxAxesPlusExtruders; ++drive)
	{
		coords[drive] = 0.0;			// clear extrusion
	}
}

#if SUPPORT_ASYNC_MOVES

void AsyncMove::SetDefaults() noexcept
{
	for (float& f : movements)
	{
		f = 0.0;			// clear extrusion
	}
	startSpeed = endSpeed = 0.0;
}

#endif

// End
