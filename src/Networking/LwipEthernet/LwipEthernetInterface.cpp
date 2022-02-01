/*
 * EthernetInterface.cpp
 *
 *  Created on: 24 Nov 2017
 *      Authors: David and Christian
 */

// Define this to keep the ASF status codes from being included. Without it ERR_TIMEOUT is defined twice
#define NO_STATUS_CODES

#include "LwipEthernetInterface.h"

#if HAS_LWIP_NETWORKING

#include <Networking/NetworkBuffer.h>
#include "LwipSocket.h"

#include <Platform/Platform.h>
#include <Platform/RepRap.h>
#include <Networking/HttpResponder.h>
#include <Networking/FtpResponder.h>
#include <Networking/TelnetResponder.h>
#include <General/IP4String.h>
#include <Version.h>								// version is reported by MDNS
#include "GMAC/ethernet_sam.h"

#if SAME70
# include <Hardware/SAME70/Ethernet/GmacInterface.h>
#elif SAME5x
# include <Hardware/SAME5x/Ethernet/GmacInterface.h>
#endif

extern "C"
{

#ifdef LWIP_STATS
#include "lwip/stats.h"
#endif

#include "lwip/dhcp.h"
#include "lwip/tcp.h"

#include "lwip/apps/netbiosns.h"
#include "lwip/apps/mdns.h"

extern struct netif gs_net_if;
}

const char * const MdnsServiceStrings[NumProtocols] = { "_http", "_ftp", "_telnet" };
const char * const MdnsTxtRecords[2] = { "product=" FIRMWARE_NAME, "version=" VERSION };
const unsigned int MdnsTtl = 10 * 60;			// same value as on the Duet 0.6/0.8.5

/*-----------------------------------------------------------------------------------*/

static LwipEthernetInterface *ethernetInterface;

# include <RTOSIface/RTOSIface.h>
Mutex lwipMutex;

extern "C"
{
	// Task function to keep the GMAC and LwIP running
	void DoEthernetTask()
	{
		ethernet_task();
	}

	// Callback functions for LWIP (may be called from ISR)
	// This occasionally seems to get called with a null pcb argument, so check for that here
	static err_t conn_accept(void *arg, tcp_pcb *pcb, err_t err)
	{
		LWIP_UNUSED_ARG(arg);
		LWIP_UNUSED_ARG(err);

		if (pcb != nullptr)
		{
			if (ethernetInterface->ConnectionEstablished(pcb))
			{
				// A socket has accepted this connection and will deal with it
				return ERR_OK;
			}

			tcp_abort(pcb);
		}
		return ERR_ABRT;
	}

}	// end extern "C"

/*-----------------------------------------------------------------------------------*/

LwipEthernetInterface::LwipEthernetInterface(Platform& p) noexcept
	: platform(p), closeDataPort(false), activated(false), initialised(false), usingDhcp(false)
{
	ethernetInterface = this;

	// Create the sockets
	for (size_t i : ARRAY_INDICES(sockets))
	{
		sockets[i] = new LwipSocket(this);
	}

	// Initialise default ports
	for (size_t i = 0; i < NumProtocols; ++i)
	{
		portNumbers[i] = DefaultPortNumbers[i];
		protocolEnabled[i] = (i == HttpProtocol);
	}
}

#if SUPPORT_OBJECT_MODEL

// Object model table and functions
// Note: if using GCC version 7.3.1 20180622 and lambda functions are used in this table, you must compile this file with option -std=gnu++17.
// Otherwise the table will be allocated in RAM instead of flash, which wastes too much RAM.

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(...) OBJECT_MODEL_FUNC_BODY(LwipEthernetInterface, __VA_ARGS__)

constexpr ObjectModelTableEntry LwipEthernetInterface::objectModelTable[] =
{
	// These entries must be in alphabetical order
	{ "actualIP",			OBJECT_MODEL_FUNC(self->ipAddress),			ObjectModelEntryFlags::none },
	{ "gateway",			OBJECT_MODEL_FUNC(self->gateway),			ObjectModelEntryFlags::none },
	{ "mac",				OBJECT_MODEL_FUNC(self->macAddress),		ObjectModelEntryFlags::none },
	{ "state",				OBJECT_MODEL_FUNC(self->GetStateName()),	ObjectModelEntryFlags::none },
	{ "subnet",				OBJECT_MODEL_FUNC(self->netmask),			ObjectModelEntryFlags::none },
	{ "type",				OBJECT_MODEL_FUNC_NOSELF("ethernet"),		ObjectModelEntryFlags::verbose },
};

constexpr uint8_t LwipEthernetInterface::objectModelTableDescriptor[] = { 1, 6 };

DEFINE_GET_OBJECT_MODEL_TABLE(LwipEthernetInterface)

#endif

void LwipEthernetInterface::Init() noexcept
{
	interfaceMutex.Create("LwipIface");
	//TODO we don't yet use this mutex anywhere!

	lwipMutex.Create("LwipCore");

	// Clear the PCBs
	for (size_t i = 0; i < NumTcpPorts; ++i)
	{
		listeningPcbs[i] = nullptr;
	}

	macAddress = platform.GetDefaultMacAddress();
}

GCodeResult LwipEthernetInterface::EnableProtocol(NetworkProtocol protocol, int port, int secure, const StringRef& reply) noexcept
{
	if (secure != 0 && secure != -1)
	{
		reply.copy("this firmware does not support TLS");
		return GCodeResult::error;
	}

	if (protocol < NumProtocols)
	{
		const TcpPort portToUse = (port < 0) ? DefaultPortNumbers[protocol] : port;
		if (portToUse != portNumbers[protocol] && GetState() == NetworkState::active)
		{
			// We need to shut down and restart the protocol if it is active because the port number has changed
			ShutdownProtocol(protocol);
			protocolEnabled[protocol] = false;
		}
		portNumbers[protocol] = portToUse;
		if (!protocolEnabled[protocol])
		{
			// Enable the corresponding socket
			protocolEnabled[protocol] = true;
			if (GetState() == NetworkState::active)
			{
				StartProtocol(protocol);
				RebuildMdnsServices();
			}
		}
		ReportOneProtocol(protocol, reply);
		return GCodeResult::ok;
	}

	reply.copy("Invalid protocol parameter");
	return GCodeResult::error;
}

GCodeResult LwipEthernetInterface::DisableProtocol(NetworkProtocol protocol, const StringRef& reply) noexcept
{
	if (protocol < NumProtocols)
	{
		if (GetState() == NetworkState::active)
		{
			ShutdownProtocol(protocol);
		}
		protocolEnabled[protocol] = false;
		ReportOneProtocol(protocol, reply);
		return GCodeResult::ok;
	}

	reply.copy("Invalid protocol parameter");
	return GCodeResult::error;
}

void LwipEthernetInterface::StartProtocol(NetworkProtocol protocol) noexcept
{
	if (listeningPcbs[protocol] == nullptr)
	{
		tcp_pcb *pcb = tcp_new();
		if (pcb == nullptr)
		{
			platform.Message(ErrorMessage, "unable to allocate a pcb\n");
		}
		else
		{
			tcp_bind(pcb, IP_ADDR_ANY, portNumbers[protocol]);
			pcb = tcp_listen(pcb);
			if (pcb == nullptr)
			{
				platform.Message(ErrorMessage, "tcp_listen call failed\n");
			}
			else
			{
				listeningPcbs[protocol] = pcb;
				tcp_accept(listeningPcbs[protocol], conn_accept);
			}
		}
	}

	switch(protocol)
	{
	case HttpProtocol:
		for (SocketNumber skt = 0; skt < NumHttpSockets; ++skt)
		{
			sockets[skt]->Init(skt, portNumbers[protocol], protocol);
		}
		break;

	case FtpProtocol:
		sockets[FtpSocketNumber]->Init(FtpSocketNumber, portNumbers[protocol], protocol);
		break;

	case TelnetProtocol:
		sockets[TelnetSocketNumber]->Init(TelnetSocketNumber, portNumbers[protocol], protocol);
		break;

	default:
		break;
	}
}

void LwipEthernetInterface::ShutdownProtocol(NetworkProtocol protocol) noexcept
{
	switch(protocol)
	{
	case HttpProtocol:
		for (SocketNumber skt = 0; skt < NumHttpSockets; ++skt)
		{
			sockets[skt]->TerminateAndDisable();
		}
		break;

	case FtpProtocol:
		sockets[FtpSocketNumber]->TerminateAndDisable();
		sockets[FtpDataSocketNumber]->TerminateAndDisable();
		break;

	case TelnetProtocol:
		sockets[TelnetSocketNumber]->TerminateAndDisable();
		break;

	default:
		break;
	}

	if (listeningPcbs[protocol] != nullptr)
	{
		tcp_close(listeningPcbs[protocol]);
		listeningPcbs[protocol] = nullptr;
	}
}

// Report the protocols and ports in use
GCodeResult LwipEthernetInterface::ReportProtocols(const StringRef& reply) const noexcept
{
	for (size_t i = 0; i < NumProtocols; ++i)
	{
		ReportOneProtocol(i, reply);
	}
	return GCodeResult::ok;
}

void LwipEthernetInterface::ReportOneProtocol(NetworkProtocol protocol, const StringRef& reply) const noexcept
{
	if (protocolEnabled[protocol])
	{
		reply.lcatf("%s is enabled on port %u", ProtocolNames[protocol], portNumbers[protocol]);
	}
	else
	{
		reply.lcatf("%s is disabled", ProtocolNames[protocol]);
	}
}

// This is called at the end of config.g processing.
// Start the network if it was enabled
void LwipEthernetInterface::Activate() noexcept
{
	if (!activated)
	{
		activated = true;
		if (GetState() == NetworkState::enabled)
		{
			Start();
		}
		else
		{
			platform.Message(NetworkInfoMessage, "Ethernet is disabled.\n");
		}
	}
}

void LwipEthernetInterface::Exit() noexcept
{
	Stop();
	ethernet_terminate();
}

// Get the network state into the reply buffer, returning true if there is some sort of error
GCodeResult LwipEthernetInterface::GetNetworkState(const StringRef& reply) noexcept
{
	ethernet_get_ipaddress(ipAddress, netmask, gateway);
	const int enableState = EnableState();
	reply.printf("Ethernet is %s, configured IP address: %s, actual IP address: %s",
					(enableState == 0) ? "disabled" : "enabled",
					IP4String(platform.GetIPAddress()).c_str(), IP4String(ipAddress).c_str());
	return GCodeResult::ok;
}

// Start up the network
void LwipEthernetInterface::Start() noexcept
{
	digitalWrite(EthernetPhyResetPin, true);			// bring the Ethernet Phy out of reset

	if (initialised)
	{
		// Bring the netif up again
		netif_set_up(&gs_net_if);
	}
	else
	{
		const char *hostname = reprap.GetNetwork().GetHostname();

		// Allow the MAC address to be set only before LwIP is started...
		ethernet_configure_interface(macAddress.bytes, hostname);
		init_ethernet(DefaultIpAddress, DefaultNetMask, DefaultGateway);

		// Initialise mDNS subsystem
		mdns_resp_init();
		mdns_resp_add_netif(&gs_net_if, hostname, MdnsTtl);

		// Initialise NetBIOS responder
		netbiosns_init();
		netbiosns_set_name(hostname);

		initialised = true;
	}

	SetState(NetworkState::establishingLink);
}

// Stop the network
void LwipEthernetInterface::Stop() noexcept
{
	if (GetState() != NetworkState::disabled)
	{
		netif_set_down(&gs_net_if);

		pinMode(EthernetPhyResetPin, OUTPUT_LOW);		// hold the Ethernet Phy chip in reset
		SetState(NetworkState::disabled);
	}
}

// Main spin loop. If 'full' is true then we are being called from the main spin loop. If false then we are being called during HSMCI idle time.
void LwipEthernetInterface::Spin() noexcept
{
	MutexLocker lock(lwipMutex);

	switch(GetState())
	{
	case NetworkState::enabled:
	case NetworkState::disabled:
	default:
		// Nothing to do
		break;

	case NetworkState::establishingLink:
		if (ethernet_establish_link())
		{
			usingDhcp = platform.GetIPAddress().IsNull();
			if (usingDhcp)
			{
				// IP address is all zeros, so use DHCP
				SetState(NetworkState::obtainingIP);
//				debugPrintf("Link established, getting IP address\n");
				IPAddress nullAddress;
				ethernet_set_configuration(nullAddress, nullAddress, nullAddress);
				dhcp_start(&gs_net_if);
			}
			else
			{
				// Using static IP address
				SetState(NetworkState::connected);
//				debugPrintf("Link established, network running\n");
				ethernet_set_configuration(platform.GetIPAddress(), platform.NetMask(), platform.GateWay());
			}
		}
		break;

	case NetworkState::obtainingIP:
		if (ethernet_link_established())
		{
			// Check for incoming packets
			DoEthernetTask();

			// Have we obtained an IP address yet?
			ethernet_get_ipaddress(ipAddress, netmask, gateway);
			if (!ipAddress.IsNull())
			{
				// Notify the mDNS responder about this
				SetState(NetworkState::connected);
//				debugPrintf("IP address obtained, network running\n");
			}
		}
		else
		{
//			debugPrintf("Lost phy link\n");
			TerminateSockets();
			SetState(NetworkState::establishingLink);
		}
		break;

	case NetworkState::connected:
		InitSockets();
		RebuildMdnsServices();
		ethernet_get_ipaddress(ipAddress, netmask, gateway);
		platform.MessageF(NetworkInfoMessage, "Ethernet running, IP address = %s\n", IP4String(ipAddress).c_str());
		SetState(NetworkState::active);
		reprap.NetworkUpdated();
		break;

	case NetworkState::active:
		// Check that the link is still up
		if (ethernet_link_established())
		{
			// Check for incoming packets
			DoEthernetTask();

			// Poll the next TCP socket
			sockets[nextSocketToPoll]->Poll();

			// Move on to the next TCP socket for next time
			++nextSocketToPoll;
			if (nextSocketToPoll == NumEthernetSockets)
			{
				nextSocketToPoll = 0;
			}

			// Check if the data port needs to be closed
			if (closeDataPort && !sockets[FtpDataSocketNumber]->IsClosing())
			{
				TerminateDataPort();
			}
		}
		else
		{
//			debugPrintf("Lost phy link\n");
			TerminateSockets();
			SetState(NetworkState::establishingLink);
		}
		break;
	}
}

void LwipEthernetInterface::Diagnostics(MessageType mtype) noexcept
{
	platform.MessageF(mtype, "- Ethernet -\nState: %s\n", GetStateName());
	platform.MessageF(mtype, "Error counts: %u %u %u %u %u\nSocket states:",
								rxErrorCount, rxBuffersNotFullyPopulatedCount, txErrorCount, txBufferNotFreeCount, txBufferTooShortCount);
	for (LwipSocket *s : sockets)
	{
		platform.MessageF(mtype, " %d", s->GetState());
	}
	platform.Message(mtype, "\n");

#if LWIP_STATS
	if (reprap.Debug(moduleNetwork))
	{
		// This prints LwIP diagnostics data to the USB port - blocking!
		stats_display();
	}
#endif
}

// Enable or disable the network. For Ethernet the ssid parameter is not used.
GCodeResult LwipEthernetInterface::EnableInterface(int mode, const StringRef& ssid, const StringRef& reply) noexcept
{
	if (!activated)
	{
		SetState((mode == 0) ? NetworkState::disabled : NetworkState::enabled);
	}
	else if (mode == 0)
	{
		if (GetState() != NetworkState::disabled)
		{
			Stop();
			platform.Message(NetworkInfoMessage, "Network stopped\n");
		}

	}
	else if (GetState() == NetworkState::disabled)
	{
		SetState(NetworkState::enabled);
		Start();
	}
	return GCodeResult::ok;
}

int LwipEthernetInterface::EnableState() const noexcept
{
	return (GetState() == NetworkState::disabled) ? 0 : 1;
}

bool LwipEthernetInterface::ConnectionEstablished(tcp_pcb *pcb) noexcept
{
	for (LwipSocket *s : sockets)
	{
		if (s->AcceptConnection(pcb))
		{
			// Socket has accepted the incoming connection
			return true;
		}
	}

	// No more free socket for this connection, terminate it
	return false;
}

IPAddress LwipEthernetInterface::GetIPAddress() const noexcept
{
	return ipAddress;
}

void LwipEthernetInterface::SetIPAddress(IPAddress p_ipAddress, IPAddress p_netmask, IPAddress p_gateway) noexcept
{
	if (GetState() == NetworkState::obtainingIP || GetState() == NetworkState::active)
	{
		const bool wantDhcp = p_ipAddress.IsNull();
		if (wantDhcp)
		{
			// Acquire dynamic IP address
			if (!usingDhcp)
			{
				SetState(NetworkState::obtainingIP);
				IPAddress nullAddress;
				ethernet_set_configuration(nullAddress, nullAddress, nullAddress);
				dhcp_start(&gs_net_if);
				usingDhcp = true;
			}
		}
		else
		{
			// Set static IP address
			if (usingDhcp)
			{
				if (GetState() == NetworkState::obtainingIP)
				{
					dhcp_stop(&gs_net_if);
				}
				SetState(NetworkState::active);
				usingDhcp = false;
			}

			ethernet_set_configuration(p_ipAddress, p_netmask, p_gateway);
			mdns_resp_netif_settings_changed(&gs_net_if);
		}
	}
}

void LwipEthernetInterface::UpdateHostname(const char *hostname) noexcept
{
	if (initialised)
	{
		netbiosns_set_name(hostname);
		RebuildMdnsServices();			// This updates the mDNS hostname too
	}
}

GCodeResult LwipEthernetInterface::SetMacAddress(const MacAddress& mac, const StringRef& reply) noexcept
{
	macAddress = mac;
	return GCodeResult::ok;
}

void LwipEthernetInterface::OpenDataPort(TcpPort port) noexcept
{
	if (listeningPcbs[NumProtocols] != nullptr)
	{
		closeDataPort = true;
		TerminateDataPort();
	}

	tcp_pcb *pcb = tcp_new();
	if (pcb == nullptr)
	{
		platform.Message(ErrorMessage, "unable to allocate a pcb\n");
	}
	else
	{
		tcp_bind(pcb, IP_ADDR_ANY, port);
		pcb = tcp_listen(pcb);
		if (pcb == nullptr)
		{
			platform.Message(ErrorMessage, "tcp_listen call failed\n");
		}
		else
		{
			listeningPcbs[NumProtocols] = pcb;
			tcp_accept(listeningPcbs[NumProtocols], conn_accept);
			sockets[FtpDataSocketNumber]->Init(FtpDataSocketNumber, port, FtpDataProtocol);
		}
	}
}

// Close FTP data port and purge associated resources
void LwipEthernetInterface::TerminateDataPort() noexcept
{
	if (closeDataPort || !sockets[FtpDataSocketNumber]->IsClosing())
	{
		closeDataPort = false;
		sockets[FtpDataSocketNumber]->TerminateAndDisable();

		if (listeningPcbs[NumProtocols] != nullptr)
		{
			tcp_close(listeningPcbs[NumProtocols]);
			listeningPcbs[NumProtocols] = nullptr;
		}
	}
	else
	{
		// The socket may be waiting for a ACKs and a graceful disconnect.
		// Give it some more time
		closeDataPort = true;
	}
}

void LwipEthernetInterface::InitSockets() noexcept
{
	for (size_t i = 0; i < NumProtocols; ++i)
	{
		if (protocolEnabled[i])
		{
			StartProtocol(i);
		}
	}
	nextSocketToPoll = 0;
}

void LwipEthernetInterface::TerminateSockets() noexcept
{
	for (LwipSocket *socket : sockets)
	{
		socket->Terminate();
	}
}

void GetServiceTxtEntries(struct mdns_service *service, void *txt_userdata)
{
	for (size_t i = 0; i < ARRAY_SIZE(MdnsTxtRecords); i++)
	{
		mdns_resp_add_service_txtitem(service, MdnsTxtRecords[i], strlen(MdnsTxtRecords[i]));
	}
}

void LwipEthernetInterface::RebuildMdnsServices() noexcept
{
	mdns_resp_remove_netif(&gs_net_if);
	mdns_resp_add_netif(&gs_net_if, reprap.GetNetwork().GetHostname(), MdnsTtl);
	mdns_resp_add_service(&gs_net_if, "echo", "_echo", DNSSD_PROTO_TCP, 0, 0, nullptr, nullptr);

	for (size_t protocol = 0; protocol < NumProtocols; protocol++)
	{
		if (protocolEnabled[protocol])
		{
			service_get_txt_fn_t txtFunc = (protocol == HttpProtocol) ? GetServiceTxtEntries : nullptr;
			mdns_resp_add_service(&gs_net_if, ProtocolNames[protocol], MdnsServiceStrings[protocol], DNSSD_PROTO_TCP, portNumbers[protocol], MdnsTtl, txtFunc, nullptr);
		}
	}

	mdns_resp_netif_settings_changed(&gs_net_if);
}

#endif	// HAS_LWIP_NETWORKING

// End
