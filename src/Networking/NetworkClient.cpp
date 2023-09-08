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

NetworkClient::NetworkClient(NetworkResponder *n, NetworkClient *c) noexcept
	: NetworkResponder(n), interface(nullptr), next(c), whenRequest(0)
{
	clients = this;
}

bool NetworkClient::Start(NetworkProtocol protocol, NetworkInterface *iface) noexcept
{
	if (HandlesProtocol(protocol))
	{
		if (skt == nullptr)
		{
			if (interface != nullptr)
			{
				if (interface == iface)
				{
					if (millis() - whenRequest < connectTimeout)
					{
						// Client is currently waiting for the previous start request to time out,
						// or for a socket to be available; do not start.
						return false;
					}
					// Else the previous start request has timed out before getting a socket; do it again.
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
				if (c != this && c->HandlesProtocol(protocol) && c->interface == iface)
				{
					return false;
				}
			}

			// If everything passes, confirm the specific client implementation requests a start.
			if (Start())
			{
				interface = iface;
				whenRequest = millis();
				return true;
			}
		}
	}

	// Client does not handle the protocol, or has a socket already; do not start.
	return false;
}

void NetworkClient::Stop(NetworkProtocol protocol, NetworkInterface *iface) noexcept
{
	if (HandlesProtocol(protocol) && interface == iface)
	{
		Stop();					// allow the specific client implementation to disconnect gracefully
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

void NetworkClient::Terminate(NetworkProtocol protocol, NetworkInterface *iface) noexcept
{
	if ((HandlesProtocol(protocol) || protocol == AnyProtocol) && interface == iface)
	{
		Terminate();
		interface = nullptr;
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
}

NetworkClient *NetworkClient::clients = nullptr;
