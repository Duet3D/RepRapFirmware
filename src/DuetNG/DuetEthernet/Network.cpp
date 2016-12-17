/*
 * Network.cpp
 *
 *  Created on: 13 Dec 2016
 *      Author: David
 */

#include "RepRapFirmware.h"
#include "compiler.h"
#include "Pins.h"
#include "Ethernet3/Ethernet3.h"


void Network::SetIPAddress(const uint8_t p_ipAddress[], const uint8_t p_netmask[], const uint8_t p_gateway[])
{
	memcpy(ipAddress, p_ipAddress, sizeof(ipAddress));
	memcpy(netmask, p_netmask, sizeof(netmask));
	memcpy(gateway, p_gateway, sizeof(gateway));
}

Network::Network(Platform* p) : platform(p), responseCode(0), responseBody(nullptr), responseText(nullptr), responseFile(nullptr),
	httpPort(DEFAULT_HTTP_PORT),
	state(disabled), activated(false)
{
	SetIPAddress(IP_ADDRESS, NET_MASK, GATE_WAY);
	strcpy(hostname, HOSTNAME);
}

void Network::Init()
{
	// Ensure that the chip is in the reset state
	pinMode(EspResetPin, OUTPUT_LOW);
	state = disabled;
}

// This is called at the end of config.g processing.
// Start the network if it was enabled
void Network::Activate()
{
	activated = true;
	if (state == enabled)
	{
		Start();
	}
}

void Network::Exit()
{
	Stop();
}

void Network::Spin()
{
#if 1
#if 0
	if (state == starting)
	{
		const int rc = Ethernet.begin(platform->MACAddress(), 12000, 5000);		// for now we always use DHCP
		if (rc == 1)
		{
			state = running;
		}
	}
	else if (state == running)
#else
	if (state == starting || state == running)
#endif
	{
		// Check DHCP
		const int rc = Ethernet.maintain();
		if (state == starting && (rc == DHCP_CHECK_RENEW_OK || rc == DHCP_CHECK_REBIND_OK))
		{
			state = running;
		}
	}
#endif
	platform->ClassReport(longWait);
}

void Network::Diagnostics(MessageType mtype)
{

}

void Network::Start()
{
	pinMode(EspResetPin, OUTPUT_LOW);
	delayMicroseconds(550);						// W550 reset pulse must be at least 500us long
	Platform::WriteDigital(EspResetPin, HIGH);	// raise /Reset pin
	delay(55);									// W5500 needs 50ms to start up
	state = starting;
#if 0
	w5500.init();
	w5500.setMACAddress(platform->MACAddress());
#endif
	const int rc = Ethernet.begin(platform->MACAddress(), 12000, 5000);		// for now we always use DHCP
	if (rc == 1)
	{
		state = running;
	}
	else
	{
#if 1
		{
			uint8_t tmp = w5500.readPHYCFGR();
			uint8_t version = w5500.readVERSIONR();
			uint8_t macBuf[6];
			w5500.readSHAR(macBuf);
			platform->MessageF(GENERIC_MESSAGE, "Phy %02x ver %02x Mac %02x:%02x:%02x:%02x:%02x:%02x\n",
					tmp, version, macBuf[0], macBuf[1], macBuf[2], macBuf[3], macBuf[4], macBuf[5]);
		}
#endif
		platform->Message(GENERIC_MESSAGE, "Failed to start Ethernet interface\n");
		Stop();
	}
}

void Network::Stop()
{
	if (state != disabled)
	{
		Ethernet.stop();
		digitalWrite(EspResetPin, LOW);	// put the ESP back into reset
		state = disabled;
	}
}

void Network::Enable()
{
	if (state == disabled)
	{
		state = enabled;
		if (activated)
		{
			Start();
		}
	}
#if 1
	else if (state == starting)
	{
		uint8_t tmp = w5500.readPHYCFGR();
		uint8_t version = w5500.readVERSIONR();
		uint8_t macBuf[6];
		w5500.readSHAR(macBuf);
		platform->MessageF(GENERIC_MESSAGE, "Phy %02x ver %02x Mac %02x:%02x:%02x:%02x:%02x:%02x\n",
				tmp, version, macBuf[0], macBuf[1], macBuf[2], macBuf[3], macBuf[4], macBuf[5]);
	}
#endif
}

void Network::Disable()
{
	if (activated && state != disabled)
	{
		Stop();
		platform->Message(GENERIC_MESSAGE, "WiFi server stopped\n");
	}
}

bool Network::IsEnabled() const
{
	return state != disabled;
}

const uint8_t *Network::GetIPAddress() const
{
	if (state == running)
	{
		static IPAddress ip;
		ip = Ethernet.localIP();
		return ip.GetRawAddress();
	}
	else
	{
		return ipAddress;
	}
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
	size_t i = 0;
	while (*name && i < ARRAY_UPB(hostname))
	{
		char c = *name++;
		if (c >= 'A' && c <= 'Z')
		{
			c += 'a' - 'A';
		}

		if ((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || (c == '-') || (c == '_'))
		{
			hostname[i++] = c;
		}
	}

	if (i)
	{
		hostname[i] = 0;
	}
	else
	{
		strcpy(hostname, HOSTNAME);
	}
}

// End
