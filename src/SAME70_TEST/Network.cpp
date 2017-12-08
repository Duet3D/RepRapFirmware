/*
 * Network.cpp
 *
 *  Created on: 20 Nov 2017
 *      Authors: David and Christian
 */

// Define this to keep the ASF status codes from being included. Without it ERR_TIMEOUT is defined twice
#define NO_STATUS_CODES

#include "Network.h"
#include "NetworkBuffer.h"
#include "EthernetInterface.h"
#include "WiFiInterface.h"

#include "Platform.h"
#include "RepRap.h"
#include "HttpResponder.h"
#include "FtpResponder.h"
#include "TelnetResponder.h"
#include "Libraries/General/IP4String.h"
#include "Version.h"


Network::Network(Platform& p) : platform(p), responders(nullptr), nextResponderToPoll(nullptr)
{
	// Create the network modules
	interfaces[EthernetInterfaceIndex] = new EthernetInterface(p);
	interfaces[WiFiInterfaceIndex] = new WiFiInterface(p);

	// Create the responders
	for (size_t i = 0; i < NumTelnetResponders; ++i)
	{
		responders = new TelnetResponder(responders);
	}
	for (size_t i = 0; i < NumFtpResponders; ++i)
	{
		responders = new FtpResponder(responders);
	}
	for (size_t i = 0; i < NumHttpResponders; ++i)
	{
		responders = new HttpResponder(responders);
	}
}

void Network::Init()
{
	longWait = millis();
	strcpy(hostname, HOSTNAME);

	NetworkBuffer::AllocateBuffers(NetworkBufferCount);

	for (NetworkInterface *iface : interfaces)
	{
		iface->Init();
	}
}

void Network::EnableProtocol(int interface, int protocol, int port, int secure, StringRef& reply)
{
	if (interface >= 0 && interface < (int)NumNetworkInterfaces)
	{
		interfaces[interface]->EnableProtocol(protocol, port, secure, reply);
	}
	else
	{
		reply.printf("Error: Invalid network interface '%d'\n", interface);
	}
}

void Network::DisableProtocol(int interface, int protocol, StringRef& reply)
{
	if (interface >= 0 && interface < (int)NumNetworkInterfaces)
	{
		interfaces[interface]->DisableProtocol(protocol, reply);
	}
	else
	{
		reply.printf("Error: Invalid network interface '%d'\n", interface);
	}
}

// Report the protocols and ports in use
void Network::ReportProtocols(int interface, StringRef& reply) const
{
	if (interface >= 0 && interface < (int)NumNetworkInterfaces)
	{
		interfaces[interface]->ReportProtocols(reply);
	}
	else
	{
		reply.printf("Error: Invalid network interface '%d'\n", interface);
	}
}

void Network::EnableWiFi(int mode, const StringRef& ssid, StringRef& reply)
{
	WiFiInterface *wifiInterface = static_cast<WiFiInterface *>(interfaces[WiFiInterfaceIndex]);
	wifiInterface->Enable(mode, ssid, reply);
}

GCodeResult Network::HandleWiFiCode(int mcode, GCodeBuffer &gb, StringRef& reply, OutputBuffer*& longReply)
{
	WiFiInterface *wifiInterface = static_cast<WiFiInterface *>(interfaces[WiFiInterfaceIndex]);
	return wifiInterface->HandleWiFiCode(mcode, gb, reply, longReply);
}

WifiFirmwareUploader& Network::GetWifiUploader()
{
	WiFiInterface *wifiInterface = static_cast<WiFiInterface *>(interfaces[WiFiInterfaceIndex]);
	return wifiInterface->GetWifiUploader();
}

void Network::ResetWiFiForUpload(bool external)
{
	WiFiInterface *wifiInterface = static_cast<WiFiInterface *>(interfaces[WiFiInterfaceIndex]);
	wifiInterface->ResetWiFiForUpload(external);
}

// This is called at the end of config.g processing.
// Start the network if it was enabled
void Network::Activate()
{
	for (NetworkInterface *iface : interfaces)
	{
		iface->Activate();
	}
}

void Network::Exit()
{
	for (NetworkInterface *iface : interfaces)
	{
		iface->Exit();
	}
}

// Get the network state into the reply buffer, returning true if there is some sort of error
bool Network::GetNetworkState(int interface, StringRef& reply)
{
	if (interface >= 0 && interface < (int)NumNetworkInterfaces)
	{
		return interfaces[interface]->GetNetworkState(reply);
	}

	reply.printf("Error: Invalid network interface '%d'\n", interface);
	return true;
}

// Start up the network
void Network::Start(int interface)
{
	if (interface >= 0 && interface < (int)NumNetworkInterfaces)
	{
		interfaces[interface]->Start();
	}
}

// Stop the network
void Network::Stop(int interface)
{
#if 0	// chrishamm: I wonder if this is actually needed - when sockets are disabled their state changes anyway
	for (NetworkResponder *r = responders; r != nullptr; r = r->GetNext())
	{
		r->Terminate(AnyProtocol);
	}
#endif

	if (interface >= 0 && interface < (int)NumNetworkInterfaces)
	{
		interfaces[interface]->Stop();
	}
}

bool Network::IsWiFiInterface(int interface) const
{
	switch (interface)
	{
	case EthernetInterfaceIndex:	return false;
	case WiFiInterfaceIndex: 		return true;
	}
	return false;
}


// Main spin loop. If 'full' is true then we are being called from the main spin loop. If false then we are being called during HSMCI idle time.
void Network::Spin(bool full)
{
	// Keep the network modules running
	for (NetworkInterface *iface : interfaces)
	{
		iface->Spin(full);
	}

	// Poll the responders
	if (full)
	{
		NetworkResponder *nr = nextResponderToPoll;
		bool doneSomething = false;
		do
		{
			if (nr == nullptr)
			{
				nr = responders;		// 'responders' can't be null at this point
			}
			doneSomething = nr->Spin();
			nr = nr->GetNext();
		} while (!doneSomething && nr != nextResponderToPoll);
		nextResponderToPoll = nr;

		platform.ClassReport(longWait);
	}
}

void Network::Interrupt()
{
	for (NetworkInterface *iface : interfaces)
	{
		iface->Interrupt();
	}
}

void Network::Diagnostics(MessageType mtype)
{
	platform.Message(mtype, "=== Network ===\n");
	platform.Message(mtype, "Responder states:");
	for (NetworkResponder *r = responders; r != nullptr; r = r->GetNext())
	{
		r->Diagnostics(mtype);
	}
	platform.Message(mtype, "\n");

	HttpResponder::CommonDiagnostics(mtype);

	for (NetworkInterface *iface : interfaces)
	{
		iface->Diagnostics(mtype);
	}
}

bool Network::InLwip() const
{
	return EthernetInterface::InLwip();
}

void Network::EnableEthernet(int mode, StringRef& reply)
{
	interfaces[EthernetInterfaceIndex]->Enable(mode, reply);
}

int Network::EnableState(int interface) const
{
	if (interface >= 0 && interface < (int)NumNetworkInterfaces)
	{
		return interfaces[interface]->EnableState();
	}
	return -1;
}

const uint8_t *Network::GetIPAddress(int interface) const
{
	if (interface >= 0 && interface < (int)NumNetworkInterfaces)
	{
		return interfaces[interface]->GetIPAddress();
	}

	return 0;
}

void Network::SetIPAddress(int interface, const uint8_t ipAddress[], const uint8_t netmask[], const uint8_t gateway[])
{
	if (interface >= 0 && interface < (int)NumNetworkInterfaces && !IsWiFiInterface(interface))
	{
		interfaces[interface]->SetIPAddress(ipAddress, netmask, gateway);
	}
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

	if (i != 0)
	{
		hostname[i] = 0;
	}
	else
	{
		strcpy(hostname, HOSTNAME);
	}

	for (NetworkInterface *iface : interfaces)
	{
		iface->SetHostname(hostname);
	}
}

// Find a responder to process a new connection
bool Network::FindResponder(Socket *skt, Protocol protocol)
{
	for (NetworkResponder *r = responders; r != nullptr; r = r->GetNext())
	{
		if (r->Accept(skt, protocol))
		{
			return true;
		}
	}
	return false;
}

void Network::HandleHttpGCodeReply(const char *msg)
{
	HttpResponder::HandleGCodeReply(msg);
}

void Network::HandleTelnetGCodeReply(const char *msg)
{
	TelnetResponder::HandleGCodeReply(msg);
}

void Network::HandleHttpGCodeReply(OutputBuffer *buf)
{
	HttpResponder::HandleGCodeReply(buf);
}

void Network::HandleTelnetGCodeReply(OutputBuffer *buf)
{
	TelnetResponder::HandleGCodeReply(buf);
}

uint32_t Network::GetHttpReplySeq()
{
	return HttpResponder::GetReplySeq();
}

// End
