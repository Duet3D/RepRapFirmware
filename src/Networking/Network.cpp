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
#include "NetworkClient.h"
#include "NetworkBuffer.h"
#include "NetworkInterface.h"
#include "GCodes/GCodeBuffer/GCodeBuffer.h"

#if HAS_LWIP_NETWORKING
# include "LwipEthernet/LwipEthernetInterface.h"
#endif

#if HAS_W5500_NETWORKING
# include "W5500Ethernet/W5500Interface.h"
#endif

#if HAS_WIFI_NETWORKING
# include "ESP8266WiFi/WiFiInterface.h"
#endif

#if HAS_RTOSPLUSTCP_NETWORKING
# include "RTOSPlusTCPEthernet/RTOSPlusTCPEthernetInterface.h"
#endif

#if HAS_WIFI_NETWORKING && HAS_LWIP_NETWORKING && defined(DUET3MINI_V04)
# include "LwipEthernet/AllocateFromPbufPool.h"
#endif
#if SUPPORT_HTTP
# include "HttpResponder.h"
#endif
#if SUPPORT_FTP
# include "FtpResponder.h"
#endif
#if SUPPORT_TELNET
# include "TelnetResponder.h"
#endif
#if SUPPORT_MQTT
#include "MQTT/MqttClient.h"
#endif
#if SUPPORT_MULTICAST_DISCOVERY
# include "MulticastDiscovery/MulticastResponder.h"
#endif

#if defined(DEBUG)
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
#if HAS_CLIENTS
			, clients(nullptr)
#endif
{
#if HAS_NETWORKING
# if defined(DUET3_MB6HC) || defined(DUET3_MB6XD)
	interfaces[0] = new LwipEthernetInterface(p);
# elif defined(DUET_NG) || defined(DUET3MINI_V04)
	interfaces[0] = nullptr;			// we set this up in Init()
# elif defined(FMDC_V02) || defined(FMDC_V03)
	interfaces[0] = new WiFiInterface(p);
# elif defined(DUET_M)
	interfaces[0] = new W5500Interface(p);
# else
#  error Unknown board
# endif
# if defined(DUET3_MB6HC)
	interfaces[1] = nullptr;			// no WiFi interface yet
# endif
#endif // HAS_NETWORKING
}

#if SUPPORT_OBJECT_MODEL

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(_ret) OBJECT_MODEL_FUNC_BODY(Network, _ret)

// Object model table and functions
// Note: if using GCC version 7.3.1 20180622 and lambda functions are used in this table, you must compile this file with option -std=gnu++17.
// Otherwise the table will be allocated in RAM instead of flash, which wastes too much RAM.

constexpr ObjectModelArrayTableEntry Network::objectModelArrayTable[] =
{
	// 0. Interfaces
	{
		nullptr,
		[] (const ObjectModel *self, const ObjectExplorationContext& context) noexcept -> size_t { return ((Network*)self)->GetNumNetworkInterfaces(); },
#if HAS_NETWORKING
		[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue { return ExpressionValue(((Network*)self)->interfaces[context.GetLastIndex()]); }
#endif
	}
};

DEFINE_GET_OBJECT_MODEL_ARRAY_TABLE(Network)

constexpr ObjectModelTableEntry Network::objectModelTable[] =
{
	// These entries must be in alphabetical order
#if HAS_NETWORKING
# if SUPPORT_HTTP
	{ "corsSite",	OBJECT_MODEL_FUNC(self->GetCorsSite()),					ObjectModelEntryFlags::none },
# endif
	{ "hostname",	OBJECT_MODEL_FUNC(self->GetHostname()),					ObjectModelEntryFlags::none },
	{ "interfaces", OBJECT_MODEL_FUNC_ARRAY(0),								ObjectModelEntryFlags::none },
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
	if (platform.IsDuetWiFi())
	{
		InitAllocationFromPbufPool();				// we have no wired Ethernet interface so we can use thr PBUF pool memory
	}
	interfaces[0] = (platform.IsDuetWiFi()) ? static_cast<NetworkInterface*>(new WiFiInterface(platform)) : static_cast<NetworkInterface*>(new LwipEthernetInterface(platform));
#  elif HAS_WIFI_NETWORKING
	interfaces[0] = static_cast<NetworkInterface*>(new WiFiInterface(platform));
#  elif HAS_LWIP_NETWORKING
	interfaces[0] = static_cast<NetworkInterface*>(new LwipEthernetInterface(platform));
#  endif
# endif

	SafeStrncpy(hostname, DEFAULT_HOSTNAME, ARRAY_SIZE(hostname));

	// Only the MB6HC has more than one interface, and at this point we haven't created the second one yet. So initialise just the first.
	interfaces[0]->Init();
#endif

	fastLoop = UINT32_MAX;
	slowLoop = 0;
}

#if defined(DUET3_MB6HC)

// Create the additional interface. Called after we have established that we are not running in SBC mode but before config.g is run.
void Network::CreateAdditionalInterface() noexcept
{
	if (platform.GetBoardType() >= BoardType::Duet3_6HC_v102)
	{
		interfaces[1] = new WiFiInterface(platform);
		numActualNetworkInterfaces = 2;
		interfaces[1]->Init();
		interfaces[1]->UpdateHostname(hostname);
	}
}

#endif

// Terminate all responders that handle a specified protocol (unless AnyProtocol is passed) on a specified interface
void Network::TerminateResponders(const NetworkInterface *iface, NetworkProtocol protocol) noexcept
{
	for (NetworkResponder *r = responders; r != nullptr; r = r->GetNext())
	{
		r->Terminate(protocol, iface);
	}
}

GCodeResult Network::EnableProtocol(unsigned int interface, NetworkProtocol protocol, int port, uint32_t ip, int secure, const StringRef& reply) noexcept
{
#if HAS_NETWORKING

	if (interface < GetNumNetworkInterfaces())
	{
		return interfaces[interface]->EnableProtocol(protocol, port, ip, secure, reply);
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
	if (interface < GetNumNetworkInterfaces())
	{
		bool client = false;

#if HAS_CLIENTS
		// Check if a client handles the protocol. If so, termination is handled
		// by the client itself, after attempting to disconnect gracefully.
		for (NetworkClient *c = clients; c != nullptr; c = c->GetNext())
		{
			if (c->HandlesProtocol(protocol))
			{
				client = true;
				break;
			}
		}
#endif

		NetworkInterface * const iface = interfaces[interface];
		const GCodeResult ret = iface->DisableProtocol(protocol, reply, !client);

		if (ret == GCodeResult::ok)
		{
#if HAS_RESPONDERS
			if (!client)
			{
				TerminateResponders(iface, protocol);
			}

			switch (protocol)
			{
#if SUPPORT_HTTP
			case HttpProtocol:
				HttpResponder::DisableInterface(iface);			// free up output buffers etc.
				break;
#endif

#if SUPPORT_FTP
			case FtpProtocol:
				// TODO the following isn't quite right, because we shouldn't free up output buffers if another network interface is still serving this protocol.
				FtpResponder::Disable();
				break;
#endif

#if SUPPORT_TELNET
			case TelnetProtocol:
				// TODO the following isn't quite right, because we shouldn't free up output buffers if another network interface is still serving this protocol.
				TelnetResponder::Disable();
				break;
#endif

#if SUPPORT_MULTICAST_DISCOVERY
				// TODO the following isn't quite right, because we shouldn't free up output buffers if another network interface is still serving this protocol.
			case MulticastDiscoveryProtocol:
				break;
#endif

#if SUPPORT_MQTT
			case MqttProtocol:
				// TODO the following isn't quite right, because we shouldn't free up output buffers if another network interface is still serving this protocol.
				MqttClient::Disable();
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
	if (interface < GetNumNetworkInterfaces())
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
	if (interface < GetNumNetworkInterfaces())
	{
		NetworkInterface * const iface = interfaces[interface];
		if (mode < 1)			// if disabling the interface
		{
#if HAS_RESPONDERS
			TerminateResponders(iface, AnyProtocol);

# if SUPPORT_HTTP
			HttpResponder::DisableInterface(iface);		// remove sessions that use this interface
# endif
# if SUPPORT_FTP
			FtpResponder::Disable();					// TODO leave any Telnet session using a different interface alone
# endif
# if SUPPORT_TELNET
			TelnetResponder::Disable();					// TODO leave any Telnet session using a different interface alone
# endif
# if SUPPORT_MQTT
			MqttClient::Disable();
# endif
#endif // HAS_RESPONDERS
		}
		return iface->EnableInterface(mode, ssid, reply);
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
		if (iface != nullptr && iface->IsWiFiInterface())
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

#if SUPPORT_MULTICAST_DISCOVERY
	MulticastResponder::Init();
#endif

#if SUPPORT_MQTT
	responders = clients = MqttClient::Init(responders, clients);
#endif

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
#if SUPPORT_MQTT
	MqttClient::Disable();
#endif

	if (TaskBase::GetCallerTaskHandle() != &networkTask)
	{
		// Terminate the network task. Not trivial because currently, the caller may be the network task.
		networkTask.TerminateAndUnlink();
	}
#endif // HAS_NETWORKING
}

#if HAS_NETWORKING
GCodeResult Network::ConfigureNetworkProtocol(GCodeBuffer& gb, const StringRef& reply)
{
	GCodeResult result = GCodeResult::ok;

	switch (gb.GetCommandFraction())
	{
		case -1:
			{
				bool seen = false;
# if SUPPORT_HTTP
				if (gb.Seen('C'))
				{
					String<StringLength20> corsSite;
					gb.GetQuotedString(corsSite.GetRef(), true);
					SetCorsSite(corsSite.c_str());
					seen = true;
				}
# endif
				const unsigned int interface = (gb.Seen('I') ? gb.GetUIValue() : 0);

				if (gb.Seen('P'))
				{
					const unsigned int protocol = gb.GetUIValue();
					if (gb.Seen('S'))
					{
						const bool enable = (gb.GetIValue() == 1);
						if (enable)
						{
							const int port = (gb.Seen('R')) ? gb.GetIValue() : -1;
							const int secure = (gb.Seen('T')) ? gb.GetIValue() : -1;

#if SUPPORT_MQTT
							if (interface < GetNumNetworkInterfaces() && protocol == MqttProtocol)
							{
								IPAddress ip;
								gb.MustSee('H');
								{
									gb.GetIPAddress(ip);
								}

								// Check if same interface - might just be changing broker ip address or remote port;
								// or if not yet associated with an interface.
								int mqttInterface =  MqttClient::GetInterface();
								if (mqttInterface == static_cast<int>(interface) || mqttInterface < 0)
								{
									result = EnableProtocol(interface, protocol, port,
																				ip.GetV4LittleEndian(), secure, reply);

									if (mqttInterface < 0 && result == GCodeResult::ok)
									{
										MqttClient::SetInterface(interface); // associate with interface
									}
								}
								else
								{
									reply.printf("MQTT is already enabled on interface '%d'\n", mqttInterface);
									result = GCodeResult::error;
								}
							}
							else
#endif
							{
								result = EnableProtocol(interface, protocol, port, AcceptAnyIp, secure, reply);
							}
						}
						else
						{
							result = DisableProtocol(interface, protocol, reply);
#if SUPPORT_MQTT
							if (protocol == MqttProtocol && result == GCodeResult::ok)
							{
								if (MqttClient::GetInterface() == static_cast<int>(interface))
								{
									MqttClient::SetInterface(-1); // do not associate with any interface
								}
							}
#endif
						}
						seen = true;
					}
				}

				if (!seen)
				{
# if SUPPORT_HTTP
					if (GetCorsSite() != nullptr)
					{
						reply.printf("CORS enabled for site '%s'", GetCorsSite());
					}
					else
					{
						reply.copy("CORS disabled");
					}
# endif
					// Default to reporting current protocols if P or S parameter missing
					result = ReportProtocols(interface, reply);
				}
			}
			break;
# if SUPPORT_MQTT
		case MqttProtocol:
			result = MqttClient::Configure(gb, reply);
			break;
# endif
		default:
			reply.printf("unsupported subcommand M586.%d", gb.GetCommandFraction());
			result = GCodeResult::error;
			break;
	}

	return result;
}
#endif

// Get the network state into the reply buffer, returning true if there is some sort of error
GCodeResult Network::GetNetworkState(unsigned int interface, const StringRef& reply) noexcept
{
#if HAS_NETWORKING
	if (interface < GetNumNetworkInterfaces())
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
	return interface < GetNumNetworkInterfaces() && interfaces[interface]->IsWiFiInterface();
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
			if (iface != nullptr)
			{
				iface->Spin();
			}
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
#if SUPPORT_MULTICAST_DISCOVERY
				MulticastResponder::Spin();
#endif
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

		if (!doneSomething)
		{
			TaskBase::SetCurrentTaskPriority(TaskPriority::SpinPriority);		// restore normal priority
			RTOSIface::Yield();
		}
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
		if (iface != nullptr)
		{
			iface->Diagnostics(mtype);
		}
	}
#endif

#if SUPPORT_MULTICAST_DISCOVERY
	MulticastResponder::Diagnostics(mtype);
#endif
}

int Network::EnableState(unsigned int interface) const noexcept
{
#if HAS_NETWORKING
	if (interface < GetNumNetworkInterfaces())
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
		if (iface != nullptr && !iface->IsWiFiInterface())
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
			(interface < GetNumNetworkInterfaces()) ? interfaces[interface]->GetIPAddress() :
#endif
					IPAddress();
}

IPAddress Network::GetNetmask(unsigned int interface) const noexcept
{
	return
#if HAS_NETWORKING
			(interface < GetNumNetworkInterfaces()) ? interfaces[interface]->GetNetmask() :
#endif
					IPAddress();
}

IPAddress Network::GetGateway(unsigned int interface) const noexcept
{
	return
#if HAS_NETWORKING
			(interface < GetNumNetworkInterfaces()) ? interfaces[interface]->GetGateway() :
#endif
					IPAddress();
}

bool Network::UsingDhcp(unsigned int interface) const noexcept
{
#if HAS_NETWORKING
	return interface < GetNumNetworkInterfaces() && interfaces[interface]->UsingDhcp();
#else
	return false;
#endif
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
		if (iface != nullptr)
		{
			iface->UpdateHostname(hostname);
		}
	}
#endif
}

// Net the MAC address. Pass -1 as the interface number to set the default MAC address for interfaces that don't have one.
GCodeResult Network::SetMacAddress(unsigned int interface, const MacAddress& mac, const StringRef& reply) noexcept
{
#if HAS_NETWORKING
	if (interface < GetNumNetworkInterfaces())
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
	if (interface >= GetNumNetworkInterfaces())
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

#if HAS_CLIENTS
bool Network::StartClient(NetworkInterface *interface, NetworkProtocol protocol) noexcept
{
	for (NetworkClient *c = clients; c != nullptr; c = c->GetNext())
	{
		if (c->Start(protocol, interface))
		{
			return true;
		}
	}
	return false;
}

void Network::StopClient(NetworkInterface *interface, NetworkProtocol protocol) noexcept
{
	for (NetworkClient *c = clients; c != nullptr; c = c->GetNext())
	{
		c->Stop(protocol, interface);
	}
}
#endif

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

#if SUPPORT_MQTT
void Network::MqttPublish(const char *msg, const char *topic, int qos, bool retain, bool dup) noexcept
{
	MqttClient::Publish(msg, topic, qos, retain, dup);
}
#endif

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
