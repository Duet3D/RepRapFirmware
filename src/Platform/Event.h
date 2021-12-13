/*
 * Event.h
 *
 *  Created on: 18 Oct 2021
 *      Author: David
 *
 * This class manages events. An event is an occurrence reported by a machine sensor that may need to be reported or may require action to be taken.
 * The various event types are listed in file CANlib/RRF3Common.h.
 * When an event on a main board occurs, a corresponding Event object is created and added to the event queue, unless there is a similar event already in the queue.
 * When an event on an expansion board occurs, it is transmitted to the main board over CAN and then treated in the same way as a main board event.
 * The event queue is kept in priority order, with the highest priority event at the head of the queue; except that if the event at the head of the queue is
 * being processed, it remains at the head of the queue until processing is complete. Leaving it in the queue while it is being processed allows other similar
 * events to be ignored.
 *
 * The event queue is emptied by the AutoPause GCode channel. It flags the entry at the head of the queue as being processed, takes whatever action is needed,
 * and removes it from the queue.
 *
 * A main board power failure bypasses the event mechanism. Triggers do not use the event mechanism.
 */

#ifndef SRC_PLATFORM_EVENT_H_
#define SRC_PLATFORM_EVENT_H_

#include <cstdint>
#include <cstddef>
#include <CoreTypes.h>
#include <RRF3Common.h>
#include <General/FreelistManager.h>
#include <General/String.h>
#include <General/SafeVsnprintf.h>
#include <Platform/MessageType.h>
#include <Platform/PrintPausedReason.h>

class VariableSet;

class Event
{
public:
	void* operator new(size_t sz) noexcept { return FreelistManager::Allocate<Event>(); }
	void operator delete(void* p) noexcept { FreelistManager::Release<Event>(p); }

	// Get a description of the current event and return the appropriate message type
	static MessageType GetTextDescription(const StringRef& str) noexcept;

	// Queue an event, or release it if we have a similar event pending already. Returns true if the event was added, false if it was released.
	static bool AddEvent(EventType et, uint16_t p_param, CanAddress p_ba, uint8_t devNum, const char *_ecv_array format, ...) noexcept;

	// Queue an event, or release it if we have a similar event pending already. Returns true if the event was added, false if it was released.
	static bool AddEventV(EventType et, uint16_t p_param, CanAddress p_ba, uint8_t devNum, const char *_ecv_array format, va_list vargs) noexcept;

	// Get the highest priority event if there is one start processing it
	static bool StartProcessing() noexcept;

	// Get the name of the macro that we run when this event occurs
	static void GetMacroFileName(const StringRef& fname) noexcept;

	// Get the parameters for invoking the macro file the current event
	static void GetParameters(VariableSet& vars) noexcept;

	// Get the default action for the current event
	static PrintPausedReason GetDefaultPauseReason() noexcept;

	// Mark the highest priority event as completed
	static void FinishedProcessing() noexcept;

	// Generate diagnostic data
	static void Diagnostics(MessageType mt, Platform& p) noexcept;

private:
	Event(Event *_ecv_null pnext, EventType et, uint16_t p_param, uint8_t devNum, CanAddress p_ba, const char *_ecv_array format, va_list vargs) noexcept;

	Event *_ecv_null next;					// next event in a linked list
	uint16_t param;							// details about the event, e.g. for a heater fault it is the type of the fault
	EventType type;							// what type of event it is
	CanAddress boardAddress;				// which board it came from
	uint8_t deviceNumber;					// which device raised it, e.g. heater number, driver number, trigger number
	volatile bool isBeingProcessed;			// true if this event is being processed, so it must remain at the head of the queue
	String<50> text;						// additional info to display to the user

	static Event * _ecv_null eventsPending;	// linked list of events waiting to be processed
	static unsigned int eventsQueued;
	static unsigned int eventsProcessed;
};

#endif /* SRC_PLATFORM_EVENT_H_ */
