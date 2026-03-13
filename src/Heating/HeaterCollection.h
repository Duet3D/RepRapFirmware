/*
 * HeaterCollection.h
 *
 *  Created on: 13 Mar 2026
 *      Author: Christian
 *
 */

#ifndef SRC_HEATING_HEATERCOLLECTION_H_
#define SRC_HEATING_HEATERCOLLECTION_H_

#include <RepRapFirmware.h>

// A collection of heater slots, each of which can hold multiple heater indices.
// This class is used for bed and chamber heater management
template<size_t NumSlots, size_t MaxPerSlot> class HeaterCollection
{
public:
	HeaterCollection() noexcept;

	// Get the first heater in a slot, or -1 if the slot is empty
	int GetFirstHeater(size_t slot) const noexcept
		pre(slot < NumSlots) { return (count[slot] > 0) ? mapping[slot][0] : -1; }

	// Get the number of heaters assigned to a slot
	size_t GetHeaterCount(size_t slot) const noexcept
		pre(slot < NumSlots) { return count[slot]; }

	// Get a specific heater in a slot, or -1 if the index is out of range
	int GetHeaterAt(size_t slot, size_t index) const noexcept
		pre(slot < NumSlots) { return (index < count[slot]) ? mapping[slot][index] : -1; }

	// Set the heater mapping for a slot (data only, does not interact with heater objects)
	void SetHeaters(size_t slot, const int32_t heaterNumbers[], size_t numHeaters) noexcept
		pre(slot < NumSlots);

	// Clear the mapping for a slot (data only, does not interact with heater objects)
	void ClearHeaters(size_t slot) noexcept
		pre(slot < NumSlots);

	// Check whether a heater number is contained in any slot
	bool ContainsHeater(int heater) const noexcept;

	// Get the number of non-empty slots to report (highest used slot + 1)
	size_t GetNumSlotsToReport() const noexcept;

	// Append diagnostic information to a string
	void AppendDiagnostics(const StringRef& reply) const noexcept;

	static constexpr size_t GetMaxSlots() noexcept { return NumSlots; }
	static constexpr size_t GetMaxPerSlot() noexcept { return MaxPerSlot; }

private:
	int8_t mapping[NumSlots][MaxPerSlot];
	size_t count[NumSlots];
};

template<size_t NumSlots, size_t MaxPerSlot> HeaterCollection<NumSlots, MaxPerSlot>::HeaterCollection() noexcept
{
	for (size_t i = 0; i < NumSlots; i++)
	{
		count[i] = 0;
		for (int8_t& h : mapping[i])
		{
			h = -1;
		}
	}
}

template<size_t NumSlots, size_t MaxPerSlot> void HeaterCollection<NumSlots, MaxPerSlot>::SetHeaters(size_t slot, const int32_t heaterNumbers[], size_t numHeaters) noexcept
{
	const size_t numToSet = min<size_t>(numHeaters, MaxPerSlot);
	for (size_t i = 0; i < numToSet; i++)
	{
		mapping[slot][i] = (int8_t)heaterNumbers[i];
	}
	count[slot] = numToSet;
}

template<size_t NumSlots, size_t MaxPerSlot> void HeaterCollection<NumSlots, MaxPerSlot>::ClearHeaters(size_t slot) noexcept
{
	for (size_t i = 0; i < count[slot]; i++)
	{
		mapping[slot][i] = -1;
	}
	count[slot] = 0;
}

template<size_t NumSlots, size_t MaxPerSlot> bool HeaterCollection<NumSlots, MaxPerSlot>::ContainsHeater(int heater) const noexcept
{
	for (size_t i = 0; i < NumSlots; i++)
	{
		for (size_t k = 0; k < count[i]; k++)
		{
			if (heater == mapping[i][k])
			{
				return true;
			}
		}
	}
	return false;
}

template<size_t NumSlots, size_t MaxPerSlot> size_t HeaterCollection<NumSlots, MaxPerSlot>::GetNumSlotsToReport() const noexcept
{
	size_t ret = NumSlots;
	while (ret != 0 && count[ret - 1] == 0)
	{
		--ret;
	}
	return ret;
}

template<size_t NumSlots, size_t MaxPerSlot> void HeaterCollection<NumSlots, MaxPerSlot>::AppendDiagnostics(const StringRef& reply) const noexcept
{
	for (size_t i = 0; i < NumSlots; i++)
	{
		if (count[i] > 0)
		{
			reply.catf(" [%u:", i);
			for (size_t k = 0; k < count[i]; k++)
			{
				reply.catf("%s%d", (k > 0) ? "," : "", mapping[i][k]);
			}
			reply.cat(']');
		}
	}
}

#endif // SRC_HEATING_HEATERCOLLECTION_H_
