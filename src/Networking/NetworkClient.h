/*
 * NetworkClient.h
 *
 *  Created on: 01 Nov 2022
 *      Author: rechrtb
 */

#ifndef SRC_NETWORKING_NETWORKCLIENT_H_
#define SRC_NETWORKING_NETWORKCLIENT_H_

#include <RepRapFirmware.h>
#include "NetworkResponder.h"

// Forward declarations
class NetworkClient;
class NetworkInterface;
class Socket;

// Base class for implementing a network client. Unlike a plain responder,
// a client requests an outgoing connection instead of just listening for incoming ones.
class NetworkClient : public NetworkResponder
{
public:
	NetworkClient(const NetworkClient&) = delete;
	NetworkClient(NetworkResponder *n, NetworkClient *c) noexcept;

	bool Start(NetworkProtocol protocol, NetworkInterface *iface) noexcept;
	void Stop(NetworkProtocol protocol, NetworkInterface *iface) noexcept;
	bool Accept(Socket *s, NetworkProtocol protocol) noexcept override;
	void Terminate(NetworkProtocol protocol, NetworkInterface *iface) noexcept override;

	NetworkInterface *GetInterface() { return interface; }
	NetworkClient *GetNext() const noexcept { return next; }

	virtual bool HandlesProtocol(NetworkProtocol p) noexcept = 0;

protected:
	NetworkInterface *interface; 	// interface this client is associated with

	virtual bool Start() noexcept;
	virtual void Stop() noexcept = 0;
	virtual bool Accept(Socket *s) noexcept = 0;
	virtual void Terminate() noexcept = 0;

	virtual void ConnectionLost() noexcept override;

private:
	NetworkClient *next;	// Next network client in the list
	uint32_t whenRequest;	// Use to keep track of time spent requesting a connection

	static NetworkClient *clients;	// Head of the list of all network clients
};

#endif /* SRC_NETWORKING_NETWORKCLIENT_H_ */
