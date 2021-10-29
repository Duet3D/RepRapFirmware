/*
 * Event.h
 *
 *  Created on: 18 Oct 2021
 *      Author: David
 */

#ifndef SRC_PLATFORM_EVENT_H_
#define SRC_PLATFORM_EVENT_H_

#include <cstdint>
#include <cstddef>
#include <CoreTypes.h>
#include <RRF3Common.h>
#include <General/FreelistManager.h>
#include <General/StringRef.h>

class Event
{
public:
	void* operator new(size_t sz) noexcept { return FreelistManager::Allocate<Event>(); }
	void operator delete(void* p) noexcept { FreelistManager::Release<Event>(p); }

	Event(Event *p_next, EventType et, EventParameter p_param, CanAddress p_ba, uint8_t devNum) noexcept;

	// Append a description of the event to a string
	void AppendText(const StringRef& str) const noexcept;

	// Get the name of the macro that we run when this event occurs
	void GetMacroFileName(const StringRef& fname) const noexcept;

private:
	Event *next;						// next event in a linked list
	EventParameter param;				// details about the event
	EventType type;						// what type of event it is
	CanAddress boardAddress;			// which board it came from
	uint8_t deviceNumber;				// which device raised it
};

#endif /* SRC_PLATFORM_EVENT_H_ */
