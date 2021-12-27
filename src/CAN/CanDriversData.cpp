/*
 * CanDriversData.cpp
 *
 *  Created on: 23 Dec 2021
 *      Author: David
 */

#include "CanDriversData.h"

#if SUPPORT_CAN_EXPANSION

// Insert a new entry, keeping the list ordered
void CanDriversList::AddEntry(DriverId driver) noexcept
{
	if (numEntries < ARRAY_SIZE(drivers))
	{
		// We could do a binary search here but the number of CAN drivers supported isn't huge, so linear search instead
		size_t insertPoint = 0;
		while (insertPoint < numEntries && drivers[insertPoint] < driver)
		{
			++insertPoint;
		}

		if (insertPoint == numEntries)
		{
			drivers[numEntries] = driver;
			++numEntries;
		}
		else if (drivers[insertPoint] != driver)
		{
			memmove(drivers + (insertPoint + 1), drivers + insertPoint, (numEntries - insertPoint) * sizeof(drivers[0]));
			drivers[insertPoint] = driver;
			++numEntries;
		}
	}
}

// Get the details of the drivers on the next board and advance startFrom beyond the entries for this board
CanAddress CanDriversList::GetNextBoardDriverBitmap(size_t& startFrom, CanDriversBitmap& driversBitmap) const noexcept
{
	driversBitmap.Clear();
	if (startFrom >= numEntries)
	{
		return CanId::NoAddress;
	}
	const CanAddress boardAddress = drivers[startFrom].boardAddress;
	do
	{
		driversBitmap.SetBit(drivers[startFrom].localDriver);
		++startFrom;
	} while (startFrom < numEntries && drivers[startFrom].boardAddress == boardAddress);
	return boardAddress;
}

#endif

// End
