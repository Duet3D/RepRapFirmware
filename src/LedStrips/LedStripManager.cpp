/*
 * LedStripManager.cpp
 *
 *  Created on: 30 Apr 2023
 *      Author: David
 */

#include <LedStrips/LedStripManager.h>

#if SUPPORT_LED_STRIPS

#include "LedStripBase.h"

// Object model table and functions
// Note: if using GCC version 7.3.1 20180622 and lambda functions are used in this table, you must compile this file with option -std=gnu++17.
// Otherwise the table will be allocate in RAM instead of flash, which wastes too much RAM.

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(...) OBJECT_MODEL_FUNC_BODY(LedStripManager, __VA_ARGS__)
#define OBJECT_MODEL_FUNC_IF(...) OBJECT_MODEL_FUNC_IF_BODY(LedStripManager, __VA_ARGS__)

constexpr ObjectModelArrayTableEntry LedStripManager::objectModelArrayTable[] =
{
	// 0. LED strips
	{
		nullptr,					// no lock needed
		[] (const ObjectModel *self, const ObjectExplorationContext& context) noexcept -> size_t { return ((const LedStripManager*)self)->GetNumLedStrips(); },
		[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue
				{ return ExpressionValue(((const LedStripManager*)self)->strips[context.GetLastIndex()]); }
	}
};

DEFINE_GET_OBJECT_MODEL_ARRAY_TABLE(LedStripManager)

constexpr ObjectModelTableEntry LedStripManager::objectModelTable[] =
{
	// 0. ledStrips
	{ "ledStrips",	OBJECT_MODEL_FUNC_ARRAY(0),	ObjectModelEntryFlags::none },
};

constexpr uint8_t LedStripManager::objectModelTableDescriptor[] =
{
	1,							// number of sections
	1							// number in section 0
};

DEFINE_GET_OBJECT_MODEL_TABLE(LedStripManager)

LedStripManager::LedStripManager()
{
	for (LedStripBase*& strip : strips)
	{
		strip = nullptr;
	}
}

// Handle M950 with E parameter
GCodeResult LedStripManager::CreateStrip(GCodeBuffer &gb, const StringRef &rslt)
{
	//TODO
	return GCodeResult::errorNotSupported;
}

// Handle M150
GCodeResult LedStripManager::ExecM150(GCodeBuffer &gb, const StringRef &rslt)
{
	//TODO
	return GCodeResult::errorNotSupported;
}

// Return the number of LED strips, excluding trailing null entries
size_t LedStripManager::GetNumLedStrips() const noexcept
{
	size_t ret = MaxLedStrips;
	while (ret != 0 && strips[ret - 1] == nullptr)
	{
		--ret;
	}
	return ret;
}

#endif

// End
