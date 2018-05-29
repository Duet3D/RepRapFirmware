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
#include "NetworkInterface.h"

#if HAS_LWIP_NETWORKING
#include "LwipEthernet/LwipEthernetInterface.h"
#endif

#if HAS_W5500_NETWORKING
#include "W5500Ethernet/W5500Interface.h"
#endif

#if HAS_WIFI_NETWORKING
#include "ESP8266WiFi/WiFiInterface.h"
#endif

#include "Platform.h"
#include "RepRap.h"
#include "HttpResponder.h"
#include "FtpResponder.h"
#include "TelnetResponder.h"
#include "Libraries/General/IP4String.h"
#include "Version.h"

#ifdef RTOS

# include "Tasks.h"
# include "RTOSIface.h"

constexpr size_t NetworkStackWords = 550;
static Task<NetworkStackWords> networkTask;

#endif

Network::Network(Platform& p) : platform(p), responders(nullptr), nextResponderToPoll(nullptr)
{
#if defined(SAME70_TEST_BOARD)
	interfaces[0] = new LwipEthernetInterface(p);
	interfaces[1] = new WiFiInterface(p);
#elif defined(DUET_NG)
	interfaces[0] = nullptr;			// we set this up in Init()
#elif defined(DUET_M)
	interfaces[0] = new W5500Interface(p);
#else
# error Unknown board
#endif
}

// Note that Platform::Init() must be called before this to that Platform::IsDuetWiFi() returns the correct value
void Network::Init()
{
	httpMutex.Create();
	telnetMutex.Create();

#if defined(DUET_NG)
	interfaces[0] = (platform.IsDuetWiFi()) ? static_cast<NetworkInterface*>(new WiFiInterface(platform)) : static_cast<NetworkInterface*>(new W5500Interface(platform));
#endif

	// Create the responders
	HttpResponder::InitStatic();
	TelnetResponder::InitStatic();

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

	strcpy(hostname, DEFAULT_HOSTNAME);

	NetworkBuffer::AllocateBuffers(NetworkBufferCount);

	for (NetworkInterface *iface : interfaces)
	{
		iface->Init();
	}

	fastLoop = UINT32_MAX;
	slowLoop = 0;
}

GCodeResult Network::EnableProtocol(unsigned int interface, NetworkProtocol protocol, int port, int secure, const StringRef& reply)
{
	if (interface < NumNetworkInterfaces)
	{
		return interfaces[interface]->EnableProtocol(protocol, port, secure, reply);
	}

	reply.printf("Invalid network interface '%d'\n", interface);
	return GCodeResult::error;
}

GCodeResult Network::DisableProtocol(unsigned int interface, NetworkProtocol protocol, const StringRef& reply)
{
	if (interface < NumNetworkInterfaces)
	{
		return interfaces[interface]->DisableProtocol(protocol, reply);
	}
	else
	{
		reply.printf("Invalid network interface '%d'\n", interface);
		return GCodeResult::error;
	}
}

// Report the protocols and ports in use
GCodeResult Network::ReportProtocols(unsigned int interface, const StringRef& reply) const
{
	if (interface < NumNetworkInterfaces)
	{
		return interfaces[interface]->ReportProtocols(reply);
	}

	reply.printf("Invalid network interface '%d'\n", interface);
	return GCodeResult::error;
}

GCodeResult Network::EnableInterface(unsigned int interface, int mode, const StringRef& ssid, const StringRef& reply)
{
	if (interface < NumNetworkInterfaces)
	{
		return interfaces[interface]->EnableInterface(mode, ssid, reply);
	}
	reply.printf("Invalid network interface '%d'\n", interface);
	return GCodeResult::error;
}

WiFiInterface *Network::FindWiFiInterface() const
{
#if HAS_WIFI_NETWORKING
	for (NetworkInterface *iface : interfaces)
	{
		if (iface->IsWiFiInterface())
		{
			return static_cast<WiFiInterface *>(iface);
		}
	}
#endif

	return nullptr;
}

GCodeResult Network::HandleWiFiCode(int mcode, GCodeBuffer &gb, const StringRef& reply, OutputBuffer*& longReply)
{
#if HAS_WIFI_NETWORKING
	WiFiInterface * const wifiInterface = FindWiFiInterface();
	if (wifiInterface != nullptr)
	{
		return wifiInterface->HandleWiFiCode(mcode, gb, reply, longReply);
	}
#endif

	reply.copy("No WiFi interface available");
	return GCodeResult::error;
}

const char* Network::GetWiFiServerVersion() const
{
#if HAS_WIFI_NETWORKING
	WiFiInterface * const wifiInterface = FindWiFiInterface();
	if (wifiInterface != nullptr)
	{
		return wifiInterface->GetWiFiServerVersion();
	}
#endif

	return "no WiFi interface";
}

WifiFirmwareUploader *Network::GetWifiUploader() const
{
#if HAS_WIFI_NETWORKING
	WiFiInterface * const wifiInterface = FindWiFiInterface();
	if (wifiInterface != nullptr)
	{
		return wifiInterface->GetWifiUploader();
	}
#endif

	return nullptr;
}

void Network::ResetWiFiForUpload(bool external)
{
#if HAS_WIFI_NETWORKING
	WiFiInterface * const wifiInterface = FindWiFiInterface();
	if (wifiInterface != nullptr)
	{
		wifiInterface->ResetWiFiForUpload(external);
	}
#endif
}

extern "C" void NetworkLoop(void *)
{
	for (;;)
	{
		reprap.GetNetwork().Spin(true);
		RTOSIface::Yield();
	}
}

// This is called at the end of config.g processing.
// Start the network if it was enabled
void Network::Activate()
{
	for (NetworkInterface *iface : interfaces)
	{
		if (iface != nullptr)
		{
			iface->Activate();
		}
	}

#ifdef RTOS
	networkTask.Create(NetworkLoop, "NETWORK", nullptr, TaskBase::SpinPriority);
#endif

}

void Network::Exit()
{
	for (NetworkInterface *iface : interfaces)
	{
		if (iface != nullptr)
		{
			iface->Exit();
		}
	}

	// TODO: close down the network and suspend the network task. Not trivial because currently, the caller may be the network task.
}

// Get the network state into the reply buffer, returning true if there is some sort of error
GCodeResult Network::GetNetworkState(unsigned int interface, const StringRef& reply)
{
	if (interface < NumNetworkInterfaces)
	{
		return interfaces[interface]->GetNetworkState(reply);
	}

	reply.printf("Invalid network interface '%d'\n", interface);
	return GCodeResult::error;
}

bool Network::IsWiFiInterface(unsigned int interface) const
{
	return interface < NumNetworkInterfaces && interfaces[interface]->IsWiFiInterface();
}

// Main spin loop. If 'full' is true then we are being called from the main spin loop. If false then we are being called during HSMCI idle time.
void Network::Spin(bool full)
{
	const uint32_t lastTime = Platform::GetInterruptClocks();

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
	}

	HttpResponder::CheckSessions();		// time out any sessions that have gone away

	// Keep track of the loop time
	const uint32_t dt = Platform::GetInterruptClocks() - lastTime;
	if (dt < fastLoop)
	{
		fastLoop = dt;
	}
	if (dt > slowLoop)
	{
		slowLoop = dt;
	}
}

// Process the network timer interrupt
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

	platform.MessageF(mtype, "Slowest loop: %.2fms; fastest: %.2fms\n", (double)(slowLoop * StepClocksToMillis), (double)(fastLoop * StepClocksToMillis));
	fastLoop = UINT32_MAX;
	slowLoop = 0;

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

bool Network::InNetworkStack() const
{
	for (NetworkInterface *iface : interfaces)
	{
		if (iface->InNetworkStack())
		{
			return true;
		}
	}
	return false;
}

int Network::EnableState(unsigned int interface) const
{
	if (interface < NumNetworkInterfaces)
	{
		return interfaces[interface]->EnableState();
	}
	return -1;
}

void Network::SetEthernetIPAddress(const uint8_t ipAddress[], const uint8_t netmask[], const uint8_t gateway[])
{
	for (NetworkInterface *iface : interfaces)
	{
		if (!iface->IsWiFiInterface())
		{
			iface->SetIPAddress(ipAddress, netmask, gateway);
		}
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
		strcpy(hostname, DEFAULT_HOSTNAME);
	}

	for (NetworkInterface *iface : interfaces)
	{
		iface->UpdateHostname(hostname);
	}
}

// Net the MAC address. Pass -1 as the interface number to set the default MAC address for interfaces that don't have one.
void Network::SetMacAddress(unsigned int interface, const uint8_t mac[])
{
	if (interface < NumNetworkInterfaces)
	{
		interfaces[interface]->SetMacAddress(mac);
	}
}

const uint8_t *Network::GetMacAddress(unsigned int interface) const
{
	if (interface >= NumNetworkInterfaces)
	{
		interface = 0;
	}
	return interfaces[interface]->GetMacAddress();
}

// Find a responder to process a new connection
bool Network::FindResponder(Socket *skt, NetworkProtocol protocol)
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
	MutexLocker lock(httpMutex);
	HttpResponder::HandleGCodeReply(msg);
}

void Network::HandleTelnetGCodeReply(const char *msg)
{
	MutexLocker lock(telnetMutex);
	TelnetResponder::HandleGCodeReply(msg);
}

void Network::HandleHttpGCodeReply(OutputBuffer *buf)
{
	MutexLocker lock(httpMutex);
	HttpResponder::HandleGCodeReply(buf);
}

void Network::HandleTelnetGCodeReply(OutputBuffer *buf)
{
	MutexLocker lock(telnetMutex);
	TelnetResponder::HandleGCodeReply(buf);
}

uint32_t Network::GetHttpReplySeq()
{
	return HttpResponder::GetReplySeq();
}

// End
