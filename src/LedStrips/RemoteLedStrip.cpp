/*
 * RemoteLedStrip.cpp
 *
 *  Created on: 30 Apr 2023
 *      Author: David
 */

#include "RemoteLedStrip.h"

#if SUPPORT_LED_STRIPS && SUPPORT_CAN_EXPANSION

#include <CAN/CanMessageGenericConstructor.h>
#include <CanMessageGenericTables.h>

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(...) OBJECT_MODEL_FUNC_BODY(RemoteLedStrip, __VA_ARGS__)

constexpr ObjectModelTableEntry RemoteLedStrip::objectModelTable[] =
{
	// Within each group, these entries must be in alphabetical order
	// 0. kinematics members
	{ "board",		OBJECT_MODEL_FUNC((int32_t)self->boardNumber), 		ObjectModelEntryFlags::none },
	{ "pin",		OBJECT_MODEL_FUNC(self->pinNameString), 			ObjectModelEntryFlags::none },
};

constexpr uint8_t RemoteLedStrip::objectModelTableDescriptor[] = { 1, 2 };

DEFINE_GET_OBJECT_MODEL_TABLE_WITH_PARENT(RemoteLedStrip, LedStripBase)

// Constructor
RemoteLedStrip::RemoteLedStrip(LedStripType p_type, size_t p_slotNumber, CanAddress p_boardNumber) noexcept
	: LedStripBase(p_type), slotNumber(p_slotNumber), boardNumber(p_boardNumber), remoteProperties(0)
{
}

// Delete the remote strip
RemoteLedStrip::~RemoteLedStrip() noexcept
{
	CanMessageGenericConstructor cons(M950LedParams);
	try
	{
		cons.AddUParam('E', slotNumber);
		cons.AddStringParam('C', "nil");
		String<1> dummy;
		(void)cons.SendAndGetResponse(CanMessageType::m950Led, boardNumber, dummy.GetRef());
	}
	catch (...)
	{
	}
}

// Configure this strip
GCodeResult RemoteLedStrip::Configure(GCodeBuffer& gb, const StringRef& reply, const char *_ecv_array pinName) THROWS(GCodeException)
{
	CanMessageGenericConstructor cons(M950LedParams);
	cons.PopulateFromCommand(gb);
	const GCodeResult rslt = cons.SendAndGetResponse(CanMessageType::m950Led, boardNumber, reply, &remoteProperties);
	if (rslt <= GCodeResult::warning)
	{
		// Save the pin name for the object model
		pinNameString.Assign(pinName);
	}
	return rslt;
}

// Send a M150 command to this strip
GCodeResult RemoteLedStrip::HandleM150(GCodeBuffer &gb, const StringRef &reply) THROWS(GCodeException)
{
	CanMessageGenericConstructor cons(M150Params);
	cons.PopulateFromCommand(gb);
	return cons.SendAndGetResponse(CanMessageType::writeLedStrip, boardNumber, reply, &remoteProperties);
}

#endif

// End
