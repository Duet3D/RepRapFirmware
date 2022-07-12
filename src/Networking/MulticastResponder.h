/*
 * MulricastResponder.h
 *
 *  Created on: 12 Jul 2022
 *      Author: David
 */

#ifndef SRC_NETWORKING_MULTICASTRESPONDER_H_
#define SRC_NETWORKING_MULTICASTRESPONDER_H_

#include "NetworkResponder.h"

#if SUPPORT_MULTICAST_DISCOVERY

class MulticastResponder : public NetworkResponder
{
public:
	MulticastResponder(NetworkResponder *n) noexcept;
	bool Spin() noexcept override;																// do some work, returning true if we did anything significant
	bool Accept(Socket *s, NetworkProtocol protocol) noexcept override { return false; }		// ask the responder to accept this connection, returns true if it did
	void Terminate(NetworkProtocol protocol, NetworkInterface *interface) noexcept override;	// terminate the responder if it is serving the specified protocol on the specified interface
	void Diagnostics(MessageType mtype) const noexcept override;
	virtual bool AcceptUdp(/*TODO parameters*/) noexcept override;								// offer the responder a UDP packet, return true if it was accepted

	static void Disable() noexcept;
};

#endif

#endif /* SRC_NETWORKING_MULTICASTRESPONDER_H_ */
