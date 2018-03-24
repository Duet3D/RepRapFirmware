/*
 * EthernetInterface.cpp
 *
 *  Created on: 24 Nov 2017
 *      Authors: David and Christian
 */

// Define this to keep the ASF status codes from being included. Without it ERR_TIMEOUT is defined twice
#define NO_STATUS_CODES

#include "LwipEthernetInterface.h"
#include "Networking/NetworkBuffer.h"
#include "LwipSocket.h"

#include "Platform.h"
#include "RepRap.h"
#include "Networking/HttpResponder.h"
#include "Networking/FtpResponder.h"
#include "Networking/TelnetResponder.h"
#include "Libraries/General/IP4String.h"
#include "Version.h"

extern "C"
{
#include "GMAC/ethernet_sam.h"

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

extern "C"
{
static volatile bool lwipLocked = false;
static volatile bool resetCallback = false;

// Lock functions for LwIP (LwIP isn't thread-safe when working with the raw API)
bool LockLWIP()
{
	if (lwipLocked)
	{
		return false;
	}

	lwipLocked = true;
	return true;
}

void UnlockLWIP()
{
	lwipLocked = false;
}

// Callback functions for the GMAC driver and for LwIP

// Called from ISR
static void ethernet_rx_callback(uint32_t ul_status)
{
	// Because the LWIP stack can become corrupted if we work with it in parallel,
	// we may have to wait for the next Spin() call to read the next packet.
	if (LockLWIP())
	{
		ethernet_task();
		UnlockLWIP();
	}
	else
	{
		ethernet_set_rx_callback(nullptr);
		resetCallback = true;
	}
}

// Task function to keep the GMAC and LwIP running
void DoEthernetTask()
{
	ethernet_task();
	if (resetCallback)
	{
		resetCallback = false;
		ethernet_set_rx_callback(&ethernet_rx_callback);
	}
}

// Callback functions for LWIP (may be called from ISR)

static err_t conn_accept(void *arg, tcp_pcb *pcb, err_t err)
{
	LWIP_UNUSED_ARG(arg);
	LWIP_UNUSED_ARG(err);

	if (ethernetInterface->ConnectionEstablished(pcb))
	{
		// A socket has accepted this connection and will deal with it
		return ERR_OK;
	}

	tcp_abort(pcb);
	return ERR_ABRT;
}

}

/*-----------------------------------------------------------------------------------*/

LwipEthernetInterface::LwipEthernetInterface(Platform& p) : platform(p), closeDataPort(false), state(NetworkState::disabled),
		activated(false), initialised(false), usingDhcp(false)
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

void LwipEthernetInterface::Init()
{
	// Clear the PCBs
	for (size_t i = 0; i < NumTcpPorts; ++i)
	{
		listeningPcbs[i] = nullptr;
	}

	memcpy(macAddress, platform.GetDefaultMacAddress(), sizeof(macAddress));
}

GCodeResult LwipEthernetInterface::EnableProtocol(NetworkProtocol protocol, int port, int secure, const StringRef& reply)
{
	if (secure != 0 && secure != -1)
	{
		reply.copy("this firmware does not support TLS");
		return GCodeResult::error;
	}

	if (protocol < NumProtocols)
	{
		const Port portToUse = (port < 0) ? DefaultPortNumbers[protocol] : port;
		if (portToUse != portNumbers[protocol] && state == NetworkState::active)
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
			if (state == NetworkState::active)
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

GCodeResult LwipEthernetInterface::DisableProtocol(NetworkProtocol protocol, const StringRef& reply)
{
	if (protocol < NumProtocols)
	{
		if (state == NetworkState::active)
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

void LwipEthernetInterface::StartProtocol(NetworkProtocol protocol)
{
	if (listeningPcbs[protocol] == nullptr)
	{
		tcp_pcb *pcb = tcp_new();
		tcp_bind(pcb, IP_ADDR_ANY, portNumbers[protocol]);
		listeningPcbs[protocol] = tcp_listen(pcb);
		tcp_accept(listeningPcbs[protocol], conn_accept);
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

void LwipEthernetInterface::ShutdownProtocol(NetworkProtocol protocol)
{
#if 0	// chrishamm: Also see Network::Stop
	for (NetworkResponder* r = responders; r != nullptr; r = r->GetNext())
	{
		r->Terminate(protocol);
	}
#endif

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
GCodeResult LwipEthernetInterface::ReportProtocols(const StringRef& reply) const
{
	reply.Clear();
	for (size_t i = 0; i < NumProtocols; ++i)
	{
		if (i != 0)
		{
			reply.cat('\n');
		}
		ReportOneProtocol(i, reply);
	}
	return GCodeResult::ok;
}

void LwipEthernetInterface::ReportOneProtocol(NetworkProtocol protocol, const StringRef& reply) const
{
	if (protocolEnabled[protocol])
	{
		reply.catf("%s is enabled on port %u", ProtocolNames[protocol], portNumbers[protocol]);
	}
	else
	{
		reply.catf("%s is disabled", ProtocolNames[protocol]);
	}
}

// This is called at the end of config.g processing.
// Start the network if it was enabled
void LwipEthernetInterface::Activate()
{
	if (!activated)
	{
		activated = true;
		if (state == NetworkState::enabled)
		{
			Start();
		}
		else
		{
			platform.Message(NetworkInfoMessage, "Ethernet is disabled.\n");
		}
	}
}

void LwipEthernetInterface::Exit()
{
	Stop();
}

// Get the network state into the reply buffer, returning true if there is some sort of error
GCodeResult LwipEthernetInterface::GetNetworkState(const StringRef& reply)
{
	const uint8_t * const config_ip = platform.GetIPAddress();
	const int enableState = EnableState();
	reply.printf("Ethernet is %s, configured IP address: %s, actual IP address: %s",
			(enableState == 0) ? "disabled" : "enabled",
					IP4String(config_ip).c_str(), IP4String(ethernet_get_ipaddress()).c_str());
	return GCodeResult::ok;
}

// Start up the network
void LwipEthernetInterface::Start()
{
	if (initialised)
	{
		// Bring the netif up again
		netif_set_up(&gs_net_if);
	}
	else
	{
		const char *hostname = reprap.GetNetwork().GetHostname();

		// Allow the MAC address to be set only before LwIP is started...
		ethernet_configure_interface(platform.GetDefaultMacAddress(), hostname);
		init_ethernet(DefaultIpAddress, DefaultNetMask, DefaultGateway);

		// Initialise mDNS subsystem
		mdns_resp_init();
		mdns_resp_add_netif(&gs_net_if, hostname, MdnsTtl);

		// Initialise NetBIOS responder
		netbiosns_init();
		netbiosns_set_name(hostname);

		initialised = true;
	}

	resetCallback = true;			// reset EMAC RX callback on next Spin call
	state = NetworkState::establishingLink;
}

// Stop the network
void LwipEthernetInterface::Stop()
{
	if (state != NetworkState::disabled)
	{
		netif_set_down(&gs_net_if);
		resetCallback = false;
		ethernet_set_rx_callback(nullptr);
		state = NetworkState::disabled;
	}
}

// Main spin loop. If 'full' is true then we are being called from the main spin loop. If false then we are being called during HSMCI idle time.
void LwipEthernetInterface::Spin(bool full)
{
	if (LockLWIP())							// basically we can't do anything if we can't interact with LWIP
	{
		switch(state)
		{
		case NetworkState::enabled:
		case NetworkState::disabled:
		default:
			// Nothing to do
			break;

		case NetworkState::establishingLink:
			if (ethernet_establish_link())
			{
				const uint8_t *ipAddress = platform.GetIPAddress();
				usingDhcp = (ipAddress[0] == 0 && ipAddress[1] == 0 && ipAddress[2] == 0 && ipAddress[3] == 0);
				if (usingDhcp)
				{
					// IP address is all zeros, so use DHCP
					state = NetworkState::obtainingIP;
//					debugPrintf("Link established, getting IP address\n");
					uint8_t nullAddress[4] = { 0, 0, 0, 0 };
					ethernet_set_configuration(nullAddress, nullAddress, nullAddress);
					dhcp_start(&gs_net_if);
				}
				else
				{
					// Using static IP address
					state = NetworkState::connected;
//					debugPrintf("Link established, network running\n");
					ethernet_set_configuration(platform.GetIPAddress(), platform.NetMask(), platform.GateWay());
				}
			}
			break;

		case NetworkState::obtainingIP:
			if (full)
			{
				if (ethernet_link_established())
				{
					// Check for incoming packets
					DoEthernetTask();

					// Have we obtained an IP address yet?
					const uint8_t * const ip = ethernet_get_ipaddress();
					if (ip[0] != 0 || ip[1] != 0 || ip[2] != 0 || ip[3] != 0)
					{
						// Notify the mDNS responder about this
						state = NetworkState::connected;
//						debugPrintf("IP address obtained, network running\n");
					}
				}
				else
				{
//					debugPrintf("Lost phy link\n");
					TerminateSockets();
					state = NetworkState::establishingLink;
				}
			}
			break;

		case NetworkState::connected:
			if (usingDhcp)
			{
				dhcp_stop(&gs_net_if);
			}

			if (full)
			{
				InitSockets();
				RebuildMdnsServices();
				platform.MessageF(NetworkInfoMessage, "Ethernet running, IP address = %s\n", IP4String(ethernet_get_ipaddress()).c_str());
				state = NetworkState::active;
			}
			break;

		case NetworkState::active:
			// Check that the link is still up
			if (ethernet_link_established())
			{
				// Check for incoming packets
				DoEthernetTask();

				// Poll the next TCP socket
				sockets[nextSocketToPoll]->Poll(full);

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
			else if (full)
			{
//				debugPrintf("Lost phy link\n");
				TerminateSockets();
				state = NetworkState::establishingLink;
			}
			break;
		}
		UnlockLWIP();
	}
}

void LwipEthernetInterface::Interrupt()
{
	if (initialised && LockLWIP())
	{
		ethernet_timers_update();
		UnlockLWIP();
	}
}

void LwipEthernetInterface::Diagnostics(MessageType mtype)
{
	platform.Message(mtype, "- Ethernet -\n");
	platform.MessageF(mtype, "State: %d\n", (int)state);
	platform.Message(mtype, "Socket states:");
	for (LwipSocket *s : sockets)
	{
		platform.MessageF(mtype, " %d", s->GetState());
	}
	platform.Message(mtype, "\n");

#if LWIP_STATS
	// This prints LwIP diagnostics data to the USB port - blocking!
	stats_display();
#endif
}

// Enable or disable the network. For Ethernet the ssid parameter is not used.
GCodeResult LwipEthernetInterface::EnableInterface(int mode, const StringRef& ssid, const StringRef& reply)
{
	if (!activated)
	{
		state = (mode == 0) ? NetworkState::disabled : NetworkState::enabled;
	}
	else if (mode == 0)
	{
		if (state != NetworkState::disabled)
		{
			Stop();
			platform.Message(NetworkInfoMessage, "Network stopped\n");
		}

	}
	else if (state == NetworkState::disabled)
	{
		state = NetworkState::enabled;
		Start();
	}
	return GCodeResult::ok;
}

int LwipEthernetInterface::EnableState() const
{
	return (state == NetworkState::disabled) ? 0 : 1;
}

bool LwipEthernetInterface::InNetworkStack() const
{
	return lwipLocked;
}

bool LwipEthernetInterface::ConnectionEstablished(tcp_pcb *pcb)
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

const uint8_t *LwipEthernetInterface::GetIPAddress() const
{
	return ethernet_get_ipaddress();
}

void LwipEthernetInterface::SetIPAddress(const uint8_t ipAddress[], const uint8_t netmask[], const uint8_t gateway[])
{
	if (state == NetworkState::obtainingIP || state == NetworkState::active)
	{
		const bool wantDhcp = (ipAddress[0] == 0 && ipAddress[1] == 0 && ipAddress[2] == 0 && ipAddress[3] == 0);
		if (wantDhcp)
		{
			// Acquire dynamic IP address
			if (!usingDhcp)
			{
				state = NetworkState::obtainingIP;
				uint8_t nullAddress[4] = { 0, 0, 0, 0 };
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
				if (state == NetworkState::obtainingIP)
				{
					dhcp_stop(&gs_net_if);
				}
				state = NetworkState::active;
				usingDhcp = false;
			}

			ethernet_set_configuration(ipAddress, netmask, gateway);
			mdns_resp_netif_settings_changed(&gs_net_if);
		}
	}
}

void LwipEthernetInterface::UpdateHostname(const char *hostname)
{
	if (initialised)
	{
		netbiosns_set_name(hostname);
		RebuildMdnsServices();			// This updates the mDNS hostname too
	}
}

void LwipEthernetInterface::SetMacAddress(const uint8_t mac[])
{
	memcpy(macAddress, mac, sizeof(macAddress));
}

void LwipEthernetInterface::OpenDataPort(Port port)
{
	if (listeningPcbs[NumProtocols] != nullptr)
	{
		closeDataPort = true;
		TerminateDataPort();
	}

	tcp_pcb *pcb = tcp_new();
	tcp_bind(pcb, IP_ADDR_ANY, port);
	listeningPcbs[NumProtocols] = tcp_listen(pcb);
	tcp_accept(listeningPcbs[NumProtocols], conn_accept);

	sockets[FtpDataSocketNumber]->Init(FtpDataSocketNumber, port, FtpDataProtocol);
}

// Close FTP data port and purge associated resources
void LwipEthernetInterface::TerminateDataPort()
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

void LwipEthernetInterface::InitSockets()
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

void LwipEthernetInterface::TerminateSockets()
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

void LwipEthernetInterface::RebuildMdnsServices()
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

// End
