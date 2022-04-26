/*
 * Network.cpp
 *
 *  Created on: 20 Nov 2017
 *      Authors: David and Christian
 */

// Define this to keep the ASF status codes from being included. Without it ERR_TIMEOUT is defined twice
#define NO_STATUS_CODES

#include "Network.h"
#include <Platform/Platform.h>
#include <Platform/RepRap.h>
#include <General/IP4String.h>
#include <Version.h>
#include <Movement/StepTimer.h>
#include <Platform/TaskPriorities.h>

#if HAS_NETWORKING
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

#if HAS_RTOSPLUSTCP_NETWORKING
#include "RTOSPlusTCPEthernet/RTOSPlusTCPEthernetInterface.h"
#endif

#if SUPPORT_HTTP
#include "HttpResponder.h"
#endif
#if SUPPORT_FTP
#include "FtpResponder.h"
#endif
#if SUPPORT_TELNET
#include "TelnetResponder.h"
#endif

#ifdef __LPC17xx__
constexpr size_t NetworkStackWords = 375;
#elif defined(DEBUG)
constexpr size_t NetworkStackWords = 1000;				// needs to be enough to support rr_model
#else
constexpr size_t NetworkStackWords = 600;				// needs to be enough to support rr_model
#endif

static Task<NetworkStackWords> networkTask;

#else
const char * const notSupportedText = "Networking is not supported on this hardware";
#endif

// MacAddress members
uint32_t MacAddress::LowWord() const noexcept
{
	return (((((bytes[3] << 8) | bytes[2]) << 8) | bytes[1]) << 8) | bytes[0];
}

uint16_t MacAddress::HighWord() const noexcept
{
	return (bytes[5] << 8) | bytes[4];
}

void MacAddress::SetFromBytes(const uint8_t mb[6]) noexcept
{
	memcpy(bytes, mb, sizeof(bytes));
}

// Network members
Network::Network(Platform& p) noexcept : platform(p)
#if HAS_RESPONDERS
			, responders(nullptr), nextResponderToPoll(nullptr)
#endif
{
#if HAS_NETWORKING
#if defined(DUET3_MB6HC) || defined(DUET3_MB6XD)
	interfaces[0] = new LwipEthernetInterface(p);
#elif defined(DUET_NG) || defined(DUET3MINI_V04)
	interfaces[0] = nullptr;			// we set this up in Init()
#elif defined(FMDC_V02)
	interfaces[0] = new WiFiInterface(p);
#elif defined(DUET_M)
	interfaces[0] = new W5500Interface(p);
#elif defined(__LPC17xx__)
# if HAS_WIFI_NETWORKING
	interfaces[0] = new WiFiInterface(p);
 #else
	interfaces[0] = new RTOSPlusTCPEthernetInterface(p);
 #endif
#else
# error Unknown board
#endif
#endif // HAS_NETWORKING
}

#if SUPPORT_OBJECT_MODEL
// Object model table and functions
// Note: if using GCC version 7.3.1 20180622 and lambda functions are used in this table, you must compile this file with option -std=gnu++17.
// Otherwise the table will be allocated in RAM instead of flash, which wastes too much RAM.

constexpr ObjectModelArrayDescriptor Network::interfacesArrayDescriptor =
{
	nullptr,
	[] (const ObjectModel *self, const ObjectExplorationContext& context) noexcept -> size_t { return NumNetworkInterfaces; },
#if HAS_NETWORKING
	[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue { return ExpressionValue(((Network*)self)->interfaces[context.GetIndex(0)]); }
#endif
};

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(_ret) OBJECT_MODEL_FUNC_BODY(Network, _ret)

constexpr ObjectModelTableEntry Network::objectModelTable[] =
{
	// These entries must be in alphabetical order
#if HAS_NETWORKING
# if SUPPORT_HTTP
	{ "corsSite",	OBJECT_MODEL_FUNC(self->GetCorsSite()),					ObjectModelEntryFlags::none },
# endif
	{ "hostname",	OBJECT_MODEL_FUNC(self->GetHostname()),					ObjectModelEntryFlags::none },
	{ "interfaces", OBJECT_MODEL_FUNC_NOSELF(&interfacesArrayDescriptor),	ObjectModelEntryFlags::none },
#endif
	{ "name",		OBJECT_MODEL_FUNC_NOSELF(reprap.GetName()), 			ObjectModelEntryFlags::none },
};

constexpr uint8_t Network::objectModelTableDescriptor[] = { 1,
#if HAS_NETWORKING
# if SUPPORT_HTTP
		4
# else
		3
# endif
#else
		1
#endif
};

DEFINE_GET_OBJECT_MODEL_TABLE(Network)

#endif

// Note that Platform::Init() must be called before this to that Platform::IsDuetWiFi() returns the correct value
void Network::Init() noexcept
{
#if HAS_NETWORKING
# if SUPPORT_HTTP
	httpMutex.Create("HTTP");
	HttpResponder::InitStatic();
# endif
# if SUPPORT_FTP
	FtpResponder::InitStatic();
# endif
# if SUPPORT_TELNET
	telnetMutex.Create("Telnet");
	TelnetResponder::InitStatic();
# endif

# if defined(DUET_NG)
#  if HAS_WIFI_NETWORKING && HAS_W5500_NETWORKING
	interfaces[0] = (platform.IsDuetWiFi()) ? static_cast<NetworkInterface*>(new WiFiInterface(platform)) : static_cast<NetworkInterface*>(new W5500Interface(platform));
#  elif HAS_WIFI_NETWORKING
	interfaces[0] = static_cast<NetworkInterface*>(new WiFiInterface(platform));
#  elif HAS_W5500_NETWORKING
	interfaces[0] = static_cast<NetworkInterface*>(new W5500Interface(platform));
#  endif
# endif

# if defined(DUET3MINI_V04)
#  if HAS_WIFI_NETWORKING && HAS_LWIP_NETWORKING
	interfaces[0] = (platform.IsDuetWiFi()) ? static_cast<NetworkInterface*>(new WiFiInterface(platform)) : static_cast<NetworkInterface*>(new LwipEthernetInterface(platform));
#  elif HAS_WIFI_NETWORKING
	interfaces[0] = static_cast<NetworkInterface*>(new WiFiInterface(platform));
#  elif HAS_LWIP_NETWORKING
	interfaces[0] = static_cast<NetworkInterface*>(new LwipEthernetInterface(platform));
#  endif
# endif

	SafeStrncpy(hostname, DEFAULT_HOSTNAME, ARRAY_SIZE(hostname));

	for (NetworkInterface *iface : interfaces)
	{
		iface->Init();
	}
#endif

	fastLoop = UINT32_MAX;
	slowLoop = 0;
}

GCodeResult Network::EnableProtocol(unsigned int interface, NetworkProtocol protocol, int port, int secure, const StringRef& reply) noexcept
{
#if HAS_NETWORKING
	if (interface < NumNetworkInterfaces)
	{
		return interfaces[interface]->EnableProtocol(protocol, port, secure, reply);
	}

	reply.printf("Invalid network interface '%d'\n", interface);
	return GCodeResult::error;
#else
	reply.copy(notSupportedText);
	return GCodeResult::error;
#endif
}

GCodeResult Network::DisableProtocol(unsigned int interface, NetworkProtocol protocol, const StringRef& reply) noexcept
{
#if HAS_NETWORKING
	if (interface < NumNetworkInterfaces)
	{
		NetworkInterface * const iface = interfaces[interface];
		const GCodeResult ret = iface->DisableProtocol(protocol, reply);
		if (ret == GCodeResult::ok)
		{
#if HAS_RESPONDERS
			for (NetworkResponder *r = responders; r != nullptr; r = r->GetNext())
			{
				r->Terminate(protocol, iface);
			}

			// The following isn't quite right, because we shouldn't free up output buffers if another network interface is still serving this protocol.
			// However, the only supported hardware with more than one network interface is the early Duet 3 prototype, so we'll leave this be.
			switch (protocol)
			{
#if SUPPORT_HTTP
			case HttpProtocol:
				HttpResponder::Disable();			// free up output buffers etc.
				break;
#endif

#if SUPPORT_FTP
			case FtpProtocol:
				FtpResponder::Disable();
				break;
#endif

#if SUPPORT_TELNET
			case TelnetProtocol:
				TelnetResponder::Disable();
				break;
#endif

			default:
				break;
			}
#endif // HAS_RESPONDERS
		}
		return ret;
	}
	else
	{
		reply.printf("Invalid network interface '%d'\n", interface);
		return GCodeResult::error;
	}
#else
	reply.copy(notSupportedText);
	return GCodeResult::error;
#endif // HAS_NETWORKING
}

// Report the protocols and ports in use
GCodeResult Network::ReportProtocols(unsigned int interface, const StringRef& reply) const noexcept
{
#if HAS_NETWORKING
	if (interface < NumNetworkInterfaces)
	{
		return interfaces[interface]->ReportProtocols(reply);
	}

	reply.printf("Invalid network interface '%d'\n", interface);
	return GCodeResult::error;
#else
	reply.copy(notSupportedText);
	return GCodeResult::error;
#endif
}

GCodeResult Network::EnableInterface(unsigned int interface, int mode, const StringRef& ssid, const StringRef& reply) noexcept
{
#if HAS_NETWORKING
	if (interface < NumNetworkInterfaces)
	{
		NetworkInterface * const iface = interfaces[interface];
		const GCodeResult ret = iface->EnableInterface(mode, ssid, reply);
		if (ret == GCodeResult::ok && mode < 1)			// if disabling the interface
		{
#if HAS_RESPONDERS
			for (NetworkResponder *r = responders; r != nullptr; r = r->GetNext())
			{
				r->Terminate(AnyProtocol, iface);
			}

#if SUPPORT_HTTP
			// The following isn't quite right, because we shouldn't free up output buffers if another network interface is still enabled and serving this protocol.
			// However, the only supported hardware with more than one network interface is the early Duet 3 prototype, so we'll leave this be.
			HttpResponder::Disable();
#endif
#if SUPPORT_FTP
			FtpResponder::Disable();
#endif
#if SUPPORT_TELNET
			TelnetResponder::Disable();
#endif
#endif // HAS_RESPONDERS
		}
		return ret;
	}
	reply.printf("Invalid network interface '%d'\n", interface);
	return GCodeResult::error;
#else
	reply.copy(notSupportedText);
	return GCodeResult::error;
#endif // HAS_NETWORKING
}

WiFiInterface *Network::FindWiFiInterface() const noexcept
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

const char* Network::GetWiFiServerVersion() const noexcept
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

WifiFirmwareUploader *Network::GetWifiUploader() const noexcept
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

void Network::ResetWiFiForUpload(bool external) noexcept
{
#if HAS_WIFI_NETWORKING
	WiFiInterface * const wifiInterface = FindWiFiInterface();
	if (wifiInterface != nullptr)
	{
		wifiInterface->ResetWiFiForUpload(external);
	}
#endif
}

#if HAS_NETWORKING
extern "C" [[noreturn]]void NetworkLoop(void *) noexcept
{
	reprap.GetNetwork().Spin();
}
#endif

// This is called at the end of config.g processing. It must only be called once.
// Start the network if it was enabled
void Network::Activate() noexcept
{
#if HAS_NETWORKING
	// Allocate network buffers
	NetworkBuffer::AllocateBuffers(NetworkBufferCount);

	// Activate the interfaces
	for (NetworkInterface *iface : interfaces)
	{
		if (iface != nullptr)
		{
			iface->Activate();
		}
	}

	// Create the network responders
# if SUPPORT_TELNET
	for (size_t i = 0; i < NumTelnetResponders; ++i)
	{
		responders = new TelnetResponder(responders);
	}
# endif

# if SUPPORT_FTP
	for (size_t i = 0; i < NumFtpResponders; ++i)
	{
		responders = new FtpResponder(responders);
	}
# endif

# if SUPPORT_HTTP
	for (size_t i = 0; i < NumHttpResponders; ++i)
	{
		responders = new HttpResponder(responders);
	}
# endif

	// Finally, create the network task
	networkTask.Create(NetworkLoop, "NETWORK", nullptr, TaskPriority::SpinPriority);
#endif
}

void Network::Exit() noexcept
{
#if HAS_NETWORKING
	for (NetworkInterface *iface : interfaces)
	{
		if (iface != nullptr)
		{
			iface->Exit();
		}
	}

#if SUPPORT_HTTP
	HttpResponder::Disable();
#endif
#if SUPPORT_FTP
	FtpResponder::Disable();
#endif
#if SUPPORT_TELNET
	TelnetResponder::Disable();
#endif

	if (TaskBase::GetCallerTaskHandle() != &networkTask)
	{
		// Terminate the network task. Not trivial because currently, the caller may be the network task.
		networkTask.TerminateAndUnlink();
	}
#endif // HAS_NETWORKING
}

// Get the network state into the reply buffer, returning true if there is some sort of error
GCodeResult Network::GetNetworkState(unsigned int interface, const StringRef& reply) noexcept
{
#if HAS_NETWORKING
	if (interface < NumNetworkInterfaces)
	{
		return interfaces[interface]->GetNetworkState(reply);
	}

	reply.printf("Invalid network interface '%d'\n", interface);
	return GCodeResult::error;
#else
	reply.copy(notSupportedText);
	return GCodeResult::error;
#endif
}

bool Network::IsWiFiInterface(unsigned int interface) const noexcept
{
#if HAS_NETWORKING
	return interface < NumNetworkInterfaces && interfaces[interface]->IsWiFiInterface();
#else
	return false;
#endif
}

#if HAS_NETWORKING

// Main spin loop
void Network::Spin() noexcept
{
	for (;;)
	{
		const uint32_t lastTime = StepTimer::GetTimerTicks();

		// Keep the network modules running
		for (NetworkInterface *iface : interfaces)
		{
			iface->Spin();
		}

#if HAS_RESPONDERS
		// Poll the responders
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
#endif

#if SUPPORT_HTTP
		HttpResponder::CheckSessions();		// time out any sessions that have gone away
#endif

		// Keep track of the loop time
		const uint32_t dt = StepTimer::GetTimerTicks() - lastTime;
		if (dt < fastLoop)
		{
			fastLoop = dt;
		}
		if (dt > slowLoop)
		{
			slowLoop = dt;
		}
		RTOSIface::Yield();
	}
}
#endif

void Network::Diagnostics(MessageType mtype) noexcept
{
#if HAS_NETWORKING
	platform.Message(mtype, "=== Network ===\n");

	platform.MessageF(mtype, "Slowest loop: %.2fms; fastest: %.2fms\n", (double)(slowLoop * StepClocksToMillis), (double)(fastLoop * StepClocksToMillis));
	fastLoop = UINT32_MAX;
	slowLoop = 0;

#if HAS_RESPONDERS
	platform.Message(mtype, "Responder states:");
	for (NetworkResponder *r = responders; r != nullptr; r = r->GetNext())
	{
		r->Diagnostics(mtype);
	}
	platform.Message(mtype, "\n");
#endif

#if SUPPORT_HTTP
	HttpResponder::CommonDiagnostics(mtype);
#endif

	for (NetworkInterface *iface : interfaces)
	{
		iface->Diagnostics(mtype);
	}
#endif
}

int Network::EnableState(unsigned int interface) const noexcept
{
#if HAS_NETWORKING
	if (interface < NumNetworkInterfaces)
	{
		return interfaces[interface]->EnableState();
	}
#endif
	return -1;
}

void Network::SetEthernetIPAddress(IPAddress p_ipAddress, IPAddress p_netmask, IPAddress p_gateway) noexcept
{
#if HAS_NETWORKING
	for (NetworkInterface *iface : interfaces)
	{
		if (!iface->IsWiFiInterface())
		{
			iface->SetIPAddress(p_ipAddress, p_netmask, p_gateway);
		}
	}
#endif
}

IPAddress Network::GetIPAddress(unsigned int interface) const noexcept
{
	return
#if HAS_NETWORKING
			(interface < NumNetworkInterfaces) ? interfaces[interface]->GetIPAddress() :
#endif
					IPAddress();
}

void Network::SetHostname(const char *name) noexcept
{
#if HAS_NETWORKING
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
#endif
}

// Net the MAC address. Pass -1 as the interface number to set the default MAC address for interfaces that don't have one.
GCodeResult Network::SetMacAddress(unsigned int interface, const MacAddress& mac, const StringRef& reply) noexcept
{
#if HAS_NETWORKING
	if (interface < NumNetworkInterfaces)
	{
		return interfaces[interface]->SetMacAddress(mac, reply);
	}
	reply.copy("unknown interface ");
	return GCodeResult::error;
#else
	reply.copy(notSupportedText);
	return GCodeResult::error;
#endif
}

const MacAddress& Network::GetMacAddress(unsigned int interface) const noexcept
{
#if HAS_NETWORKING
	if (interface >= NumNetworkInterfaces)
	{
		interface = 0;
	}
	return interfaces[interface]->GetMacAddress();
#else
	// TODO: Is this initialized?
	return platform.GetDefaultMacAddress();
#endif
}

// Find a responder to process a new connection
bool Network::FindResponder(Socket *skt, NetworkProtocol protocol) noexcept
{
#if HAS_RESPONDERS
	for (NetworkResponder *r = responders; r != nullptr; r = r->GetNext())
	{
		if (r->Accept(skt, protocol))
		{
			return true;
		}
	}
#endif
	return false;
}

void Network::HandleHttpGCodeReply(const char *msg) noexcept
{
#if SUPPORT_HTTP
	MutexLocker lock(httpMutex);
	HttpResponder::HandleGCodeReply(msg);
#endif
}

void Network::HandleTelnetGCodeReply(const char *msg) noexcept
{
#if SUPPORT_TELNET
	MutexLocker lock(telnetMutex);
	TelnetResponder::HandleGCodeReply(msg);
#endif
}

void Network::HandleHttpGCodeReply(OutputBuffer *buf) noexcept
{
#if SUPPORT_HTTP
	MutexLocker lock(httpMutex);
	HttpResponder::HandleGCodeReply(buf);
#else
	OutputBuffer::ReleaseAll(buf);
#endif
}

void Network::HandleTelnetGCodeReply(OutputBuffer *buf) noexcept
{
#if SUPPORT_TELNET
	MutexLocker lock(telnetMutex);
	TelnetResponder::HandleGCodeReply(buf);
#else
	OutputBuffer::ReleaseAll(buf);
#endif
}

uint32_t Network::GetHttpReplySeq() noexcept
{
#if SUPPORT_HTTP
	return HttpResponder::GetReplySeq();
#else
	return -1;
#endif
}

// End
