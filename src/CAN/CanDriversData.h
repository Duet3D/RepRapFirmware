/*
 * CanDriversData.h
 *
 *  Created on: 23 Dec 2021
 *      Author: David
 */

#ifndef SRC_CAN_CANDRIVERSDATA_H_
#define SRC_CAN_CANDRIVERSDATA_H_

#include "RepRapFirmware.h"

#if SUPPORT_CAN_EXPANSION

typedef Bitmap<uint16_t> CanDriversBitmap;

// Class to accumulate a set of values relating to CAN-connected drivers
template<class T> class CanDriversData
{
public:
	CanDriversData() noexcept;
	void AddEntry(DriverId id, T val) noexcept;
	size_t GetNumEntries() const noexcept { return numEntries; }
	CanAddress GetNextBoardDriverBitmap(size_t& startFrom, CanDriversBitmap& driversBitmap) const noexcept;
	T GetElement(size_t n) const pre(n < GetnumEntries()) noexcept { return data[n].val; }

private:
	struct DriverDescriptor
	{
		DriverId driver;
		T val;
	};

	size_t numEntries;
	DriverDescriptor data[MaxCanDrivers];
};

// Class to represent a set of CAN-connected drivers with no associated data
class CanDriversList
{
public:
	CanDriversList() noexcept : numEntries(0) { }
	void Clear() noexcept { numEntries = 0; }
	void AddEntry(DriverId id) noexcept;
	size_t GetNumEntries() const noexcept { return numEntries; }
	bool IsEmpty() const noexcept { return numEntries == 0; }
	CanAddress GetNextBoardDriverBitmap(size_t& startFrom, CanDriversBitmap& driversBitmap) const noexcept;

private:
	size_t numEntries;
	DriverId drivers[MaxCanDrivers];
};

// Members of template class CanDriversData
template<class T> CanDriversData<T>::CanDriversData() noexcept
{
	numEntries = 0;
}

// Insert a new entry, keeping the list ordered by driver ID
template<class T> void CanDriversData<T>::AddEntry(DriverId driver, T val) noexcept
{
	if (numEntries < ARRAY_SIZE(data))
	{
		// We could do a binary search here but the number of CAN drivers supported isn't huge, so linear search instead
		size_t insertPoint = 0;
		while (insertPoint < numEntries && data[insertPoint].driver < driver)
		{
			++insertPoint;
		}
		memmove(data + (insertPoint + 1), data + insertPoint, (numEntries - insertPoint) * sizeof(data[0]));
		data[insertPoint].driver = driver;
		data[insertPoint].val = val;
		++numEntries;
	}
}

// Get the details of the drivers on the next board and advance startFrom beyond the entries for this board
template<class T> CanAddress CanDriversData<T>::GetNextBoardDriverBitmap(size_t& startFrom, CanDriversBitmap& driversBitmap) const noexcept
{
	driversBitmap.Clear();
	if (startFrom >= numEntries)
	{
		return CanId::NoAddress;
	}
	const CanAddress boardAddress = data[startFrom].driver.boardAddress;
	do
	{
		driversBitmap.SetBit(data[startFrom].driver.localDriver);
		++startFrom;
	} while (startFrom < numEntries && data[startFrom].driver.boardAddress == boardAddress);
	return boardAddress;
}

#endif

#endif /* SRC_CAN_CANDRIVERSDATA_H_ */
