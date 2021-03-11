/*
 * NetworkInterface.cpp
 *
 *  Created on: 10 Mar 2020
 *      Author: David
 */

#include "NetworkInterface.h"
#include <Platform/RepRap.h>

void NetworkInterface::SetState(NetworkState::RawType newState) noexcept
{
	state = newState;
	reprap.NetworkUpdated();
}

// End
