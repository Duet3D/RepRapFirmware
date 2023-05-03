/*
 * LedStripManager.cpp
 *
 *  Created on: 30 Apr 2023
 *      Author: David
 */

#include <LedStrips/LedStripManager.h>

#if SUPPORT_LED_STRIPS

#include "LedStripBase.h"
#include "DotStarLedStrip.h"
#include "NeoPixelLedStrip.h"

#if SUPPORT_CAN_EXPANSION
#include "RemoteLedStrip.h"
#endif

#include <GCodes/GCodeBuffer/GCodeBuffer.h>
#include <Platform/RepRap.h>
#include <GCodes/GCodes.h>

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

LedStripManager::LedStripManager() noexcept
{
	for (LedStripBase*& strip : strips)
	{
		strip = nullptr;
	}
}

// Handle M950 with E parameter
GCodeResult LedStripManager::CreateStrip(GCodeBuffer &gb, const StringRef &reply) THROWS(GCodeException)
{
	const uint32_t stripNumber = gb.GetLimitedUIValue('E', MaxLedStrips);
	LedStripBase*& slot = strips[stripNumber];

	if (!gb.Seen('C') && !gb.Seen('T'))
	{
		// Just reporting on an existing LED strip, or changing its minor parameters (not the pin name)
		ReadLocker lock(ledLock);
		if (slot == nullptr)
		{
			reply.printf("LED strip %u does not exist", (unsigned int)stripNumber);
			return GCodeResult::error;
		}
		return slot->Configure(gb, reply, nullptr);
	}

	const LedStripType ledType = (gb.Seen('T')) ? (LedStripType)gb.GetLimitedUIValue('T', 3) : LedStripType::NeoPixel_RGB;
	gb.MustSee('C');
	String<StringLength50> pinName;
	gb.GetReducedString(pinName.GetRef());

	WriteLocker lock(ledLock);
	DeleteObject(slot);		//TODO handle this properly over CAN

	LedStripBase *newStrip = nullptr;

#if SUPPORT_CAN_EXPANSION
	const CanAddress board = IoPort::RemoveBoardAddress(pinName.GetRef());
	if (board != CanInterface::GetCanAddress())
	{
		newStrip = new RemoteLedStrip((LedStripType)ledType, stripNumber, board);
	}
	else
#endif
	{
		switch (ledType.RawValue())
		{
#if SUPPORT_DMA_DOTSTAR
		case LedStripType::DotStar:
			newStrip = new DotStarLedStrip();
			break;
#endif

		case LedStripType::NeoPixel_RGB:
			newStrip = new NeoPixelLedStrip(false);
			break;

		case LedStripType::NeoPixel_RGBW:
			newStrip = new NeoPixelLedStrip(true);
			break;

		default:
			reply.copy("Unsupported LED strip type");
			return GCodeResult::error;
		}
	}

	GCodeResult rslt;
	try
	{
		rslt = newStrip->Configure(gb, reply, pinName.c_str());
	}
	catch (const GCodeException& ex)
	{
		ex.GetMessage(reply, &gb);
		rslt = GCodeResult::error;
	}

	if (rslt <= GCodeResult::warning)
	{
		slot = newStrip;
	}
	else
	{
		delete newStrip;
	}
	return rslt;
}

// Handle M150
GCodeResult LedStripManager::HandleM150(GCodeBuffer &gb, const StringRef &reply) THROWS(GCodeException)
{
	uint32_t stripNumber = 0;
	bool dummy;
	gb.TryGetLimitedUIValue('E', stripNumber, dummy, MaxLedStrips);

	{
		ReadLocker locker(ledLock);
		LedStripBase *const strip = strips[stripNumber];
		if (strip != nullptr)
		{
			if (strip->MustStopMovement())
			{
				if (!reprap.GetGCodes().LockAllMovementSystemsAndWaitForStandstill(gb))
				{
					return GCodeResult::notFinished;
				}
			}
			return strip->HandleM150(gb, reply);
		}
	}

	reply.printf("LED strip #%u has not been configured", (unsigned int)stripNumber);
	return GCodeResult::error;
}

// Return true if we must stop movement before writing to the strip, e.g. because it will be sent by bit-banging with interrupts disabled
bool LedStripManager::MustStopMovement(GCodeBuffer& gb) noexcept
{
	if (gb.Seen('F') && gb.GetUIValue() != 0)
	{
		return false;
	}

	const uint32_t stripNumber = (gb.Seen('E')) ? gb.GetUIValue() : 0;
	if (stripNumber >= MaxLedStrips)
	{
		return false;
	}

	ReadLocker locker(ledLock);
	return strips[stripNumber] != nullptr && strips[stripNumber]->MustStopMovement();
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
