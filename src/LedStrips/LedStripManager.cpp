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
# include "RemoteLedStrip.h"
#endif

#if SUPPORT_REMOTE_COMMANDS
# include <CanMessageFormats.h>
# include <CanMessageGenericTables.h>
# include <CanMessageGenericParser.h>
#endif

#include <GCodes/GCodeBuffer/GCodeBuffer.h>
#include <Platform/RepRap.h>
#include <GCodes/GCodes.h>

ReadWriteLock LedStripManager::ledLock;

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

	if (!gb.Seen('C') && !gb.Seen('T') && !gb.Seen('U'))
	{
		// Just reporting on an existing LED strip, or changing its minor parameters (not the pin name)
		ReadLocker locker(ledLock);
		if (slot == nullptr)
		{
			reply.printf("LED strip %u does not exist", (unsigned int)stripNumber);
			return GCodeResult::error;
		}
		return slot->Configure(gb, reply, nullptr);
	}

	const LedStripType ledType = (gb.Seen('T')) ? (LedStripType)gb.GetLimitedUIValue('T', 3) : DefaultLedStripType;
	gb.MustSee('C');
	String<StringLength50> pinName;
	gb.GetReducedString(pinName.GetRef());

	WriteLocker lock(ledLock);
	DeleteObject(slot);

#if SUPPORT_CAN_EXPANSION
	const CanAddress board = IoPort::RemoveBoardAddress(pinName.GetRef());
#endif

	if (StringEqualsIgnoreCase(pinName.c_str(), NoPinName))
	{
		return GCodeResult::ok;							// just deleting an existing strip
	}

	LedStripBase *newStrip = nullptr;

#if SUPPORT_CAN_EXPANSION
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
		delete newStrip;
		throw;
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

// Return the number of LED strips, excluding trailing null entries. Called to build the object model.
size_t LedStripManager::GetNumLedStrips() const noexcept
{
	size_t ret = MaxLedStrips;
	while (ret != 0 && strips[ret - 1] == nullptr)
	{
		--ret;
	}
	return ret;
}

// Retrieve an LED strip. Caller must acquire ledLock before calling this. Called to build the object model.
const LedStripBase *LedStripManager::GetLedStrip(size_t index) const noexcept
{
	return (index < MaxLedStrips) ? strips[index] : nullptr;
}

#if SUPPORT_REMOTE_COMMANDS

// Configure an LED strip. If success and the strip does not require motion to be paused when sending data to the strip, set bit 0 of 'extra'.
GCodeResult LedStripManager::HandleM950Led(const CanMessageGeneric &msg, const StringRef& reply, uint8_t& extra) noexcept
{
	// Get and validate the port number
	CanMessageGenericParser parser(msg, M950LedParams);
	uint16_t stripNumber;
	if (!parser.GetUintParam('E', stripNumber))
	{
		reply.copy("Missing strip number parameter in M950Led message");
		return GCodeResult::remoteInternalError;
	}
	if (stripNumber >= MaxLedStrips)
	{
		reply.printf("LED strip number %u is too high for expansion board %u", stripNumber, CanInterface::GetCanAddress());
		return GCodeResult::error;
	}

	uint8_t rawStripType;
	const LedStripType t = (parser.GetUintParam('T', rawStripType)) ? (LedStripType)rawStripType : DefaultLedStripType;

	LedStripBase*& slot = strips[stripNumber];
	GCodeResult rslt;
	WriteLocker lock(ledLock);
	if (parser.HasParameter('C'))
	{
		// Configuring a new strip
		DeleteObject(slot);

		LedStripBase *newStrip = nullptr;
		switch (t.RawValue())
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
			reprap.LedStripsUpdated();
			return GCodeResult::error;
		}

		rslt = newStrip->Configure(parser, reply, extra);
		if (rslt <= GCodeResult::warning)
		{
			slot = newStrip;
		}
		else
		{
			delete newStrip;
		}
	}
	else
	{
		// Reconfiguring or reporting on an existing strip
		rslt = slot->Configure(parser, reply, extra);
	}
	reprap.LedStripsUpdated();
	return rslt;
}

// Set the colours of a configured LED strip
GCodeResult LedStripManager::HandleLedSetColours(const CanMessageGeneric &msg, const StringRef& reply) noexcept
{
	CanMessageGenericParser parser(msg, M150Params);
	uint16_t stripNumber = 0;								// strip number may be omitted, defaults to 0
	parser.GetUintParam('E', stripNumber);
	if (stripNumber >= MaxLedStrips)
	{
		reply.printf("LED strip number %u is too high for expansion board %u", stripNumber, CanInterface::GetCanAddress());
		return GCodeResult::error;
	}

	{
		ReadLocker locker(ledLock);
		LedStripBase *const strip = strips[stripNumber];
		if (strip != nullptr)
		{
			return strip->HandleM150(parser, reply);
		}
	}

	reply.printf("Board %u does not have LED strip #%u", CanInterface::GetCanAddress(), stripNumber);
	return GCodeResult::error;
}

#endif

#endif

// End
