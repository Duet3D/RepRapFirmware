/*
 * MulricastResponder.cpp
 *
 *  Created on: 12 Jul 2022
 *      Author: David
 */

#include "MulticastResponder.h"

#if SUPPORT_MULTICAST_DISCOVERY

MulticastResponder::MulticastResponder(NetworkResponder *n) noexcept : NetworkResponder(n)
{
	// TODO Auto-generated constructor stub
}

// Do some work, returning true if we did anything significant
bool MulticastResponder::Spin() noexcept
{
	//TODO
	return false;
}

// Terminate the responder if it is serving the specified protocol on the specified interface
void MulticastResponder::Terminate(NetworkProtocol protocol, NetworkInterface *interface) noexcept
{
	//TODO
}

void MulticastResponder::Diagnostics(MessageType mtype) const noexcept
{
	//TODO
}

// Offer the responder a UDP packet, return true if it was accepted
bool MulticastResponder::AcceptUdp( /*TODO parameters*/ ) noexcept
{
	//TODO
	return false;
}

void MulticastResponder::Disable() noexcept
{
	//TODO
}

#endif

// End
