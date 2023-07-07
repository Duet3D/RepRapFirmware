/*
 * LedStripBase.cpp
 *
 *  Created on: 30 Apr 2023
 *      Author: David
 */

#include "LedStripBase.h"

#if SUPPORT_LED_STRIPS

// Object model table and functions
// Note: if using GCC version 7.3.1 20180622 and lambda functions are used in this table, you must compile this file with option -std=gnu++17.
// Otherwise the table will be allocate in RAM instead of flash, which wastes too much RAM.

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(...)					OBJECT_MODEL_FUNC_BODY(LedStripBase, __VA_ARGS__)
#define OBJECT_MODEL_FUNC_IF(_condition, ...)	OBJECT_MODEL_FUNC_IF_BODY(LedStripBase, _condition, __VA_ARGS__)

constexpr ObjectModelTableEntry LedStripBase::objectModelTable[] =
{
	// 0.
	{ "stopMovement",	OBJECT_MODEL_FUNC(self->MustStopMovement()), 	ObjectModelEntryFlags::none },
	{ "type",			OBJECT_MODEL_FUNC(self->GetTypeText()),			ObjectModelEntryFlags::none },
};

constexpr uint8_t LedStripBase::objectModelTableDescriptor[] =
{
	1,							// number of sections
	2							// number in section 0
};

DEFINE_GET_OBJECT_MODEL_TABLE(LedStripBase)

const char *_ecv_array LedStripBase::GetTypeText() const noexcept
{
	return type.ToString();
}

#endif

// End
