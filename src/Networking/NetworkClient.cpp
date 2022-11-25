/*
 * MqttClient.cpp
 *
 *  Created on: 01 Nov 2022
 *      Author: rechrtb
 */

#include "NetworkClient.h"
#include "Socket.h"
#include <Platform/Platform.h>

static const uint32_t connectTimeout = ConnectTimeout + 1000; // Make sure the socket, if any, is closed before attempting another connection.

NetworkClient::NetworkClient(NetworkResponder *n, NetworkClient *c) noexcept : NetworkResponder(n),
																			interface(nullptr),
																			next(c),
																			whenRequest(0)
{
	clients = this;
}

bool NetworkClient::Start(NetworkProtocol protocol, NetworkInterface *interface) noexcept
{
	if (HandlesProtocol(protocol))
	{
		if (!skt)
		{
			if (this->interface)
			{
				if (this->interface == interface)
				{
					if (millis() - whenRequest < connectTimeout)
					{
						// Client is currently waiting for the previous start request to time out,
						// or for a socket to be available; do not start.
						return false;
					}
					// Else the previous start request has timed out before getting a socket;
					// do it again.
				}
				else
				{
					// Already starting on another interface.
					return false;
				}
			}

			// Enforce a single client on each interface.
			for (NetworkClient *c = clients; c != nullptr; c = c->GetNext())
			{
				if (c != this && c->HandlesProtocol(protocol) && c->interface == interface)
				{
					return false;
				}
			}

			// If everything passes, confirm the specific client implementation requests a start.
			if (Start())
			{
				this->interface = interface;
				whenRequest = millis();
				return true;
			}
		}
	}

	// Client does not handle the protocol, or has a socket already; do not start.
	return false;
}

void NetworkClient::Stop(NetworkProtocol protocol, NetworkInterface *interface) noexcept
{
	if (HandlesProtocol(protocol) && this->interface == interface)
	{
		Stop(); // Allow the specific client implementation to disconnect gracefully
	}
}

bool NetworkClient::Accept(Socket *s, NetworkProtocol protocol) noexcept
{
	if (HandlesProtocol(protocol) && s->GetInterface() == interface)
	{
		return Accept(s);
	}

	return false;
}

void NetworkClient::Terminate(NetworkProtocol protocol, NetworkInterface *interface) noexcept
{
	if ((HandlesProtocol(protocol) || protocol == AnyProtocol) && this->interface == interface)
	{
		Terminate();
	}
}

bool NetworkClient::Start() noexcept
{
	// Default behavior is that the client will always request a start when it is not active.
	// A specific client implementation (child class) might override this function with custom logic,
	// such as starting only during certain events, etc.
	return true;
}

void NetworkClient::ConnectionLost() noexcept
{
	NetworkResponder::ConnectionLost();
	interface = nullptr;
}

NetworkClient *NetworkClient::clients = nullptr;
