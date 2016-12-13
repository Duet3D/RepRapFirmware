/*
 * Network.cpp
 *
 *  Created on: 13 Dec 2016
 *      Author: David
 */

#include "Network.h"

const uint8_t *Network::IPAddress() const
{
	return ipAddress;
}

void Network::SetIPAddress(const uint8_t p_ipAddress[], const uint8_t p_netmask[], const uint8_t p_gateway[])
{
	memcpy(ipAddress, p_ipAddress, sizeof(ipAddress));
	memcpy(netmask, p_netmask, sizeof(netmask));
	memcpy(gateway, p_gateway, sizeof(gateway));
}

Network::Network(Platform* p)
	: httpPort(DEFAULT_HTTP_PORT)
{
	SetIPAddress(IP_ADDRESS, NET_MASK, GATE_WAY);
}

void Network::Init()
{

}

void Network::Activate()
{

}

void Network::Exit()
{

}

void Network::Spin()
{

}

void Network::Diagnostics(MessageType mtype)
{

}

void Network::Start()
{

}

void Network::Stop()
{

}

void Network::Enable()
{

}

void Network::Disable()
{

}

bool Network::IsEnabled() const
{
	return false;
}

void Network::SetHttpPort(uint16_t port)
{
	httpPort = port;
}

uint16_t Network::GetHttpPort() const
{
	return httpPort;
}

void Network::SetHostname(const char *name)
{

}

// End
