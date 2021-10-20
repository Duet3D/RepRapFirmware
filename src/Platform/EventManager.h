/*
 * EventManager.h
 *
 *  Created on: 19 Oct 2021
 *      Author: David
 */

#ifndef SRC_PLATFORM_EVENTMANAGER_H_
#define SRC_PLATFORM_EVENTMANAGER_H_

#include "Event.h"

class EventManager
{
public:
	EventManager();

private:
	Event *eventsPending;								// linked list of pending events
	uint32_t lastWarningMillis;							// when we last sent a warning message
};

#endif /* SRC_PLATFORM_EVENTMANAGER_H_ */
